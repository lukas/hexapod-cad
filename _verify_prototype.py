"""Run all post-redesign correctness checks on the prototype.

1.  Every part is a clean watertight mesh (manifold, no self-intersections
    that would crash the slicer).
2.  Every servo well has an open insertion path big enough for the body.
3.  Every M3 mounting / chassis-bolt hole drilled by the design code
    actually passes through solid material it's supposed to anchor in.
3b. The L-shaped wire-exit corridor at every cradle's -X bottom-OUTBOARD
    corner lets a WIRE_SLOT_W-wide harness reach free space without
    intersecting solid part material.
4.  In standing pose, no two printed parts overlap in volume by more than
    100 mm^3.
5.  In standing pose, the three servo bodies (yaw / hip-pitch / knee-pitch)
    sit in their cradles without poking into solid printed material.
6.  No printed part has a STRUCTURALLY-SIGNIFICANT region thinner than
    MIN_PRINT_T mm (Hildebrand max-inscribed-ball local thickness on a
    voxelised occupancy field).
6b. None of the structural LEG LINK parts has a thin SHEET / WEB --
    a region of material extended in 2D but thinner than MIN_SHEET_T
    in the third direction.  Catches "thick block - THIN SHEET -
    thick block" topology that 6. misses because an inscribed ball
    fits happily inside a thin slab.

Exit code 0 = all checks pass.  Exit code 1 = at least one failure.

Robust inside-test
------------------

We do NOT use ``trimesh.proximity.signed_distance`` or
``Trimesh.contains`` directly: both give false positives on the
boolean-union'd meshes used by `make_coxa_*`/`make_femur_*` (the
internal cavities created by the body-pocket cuts confuse the
ray-casting heuristic).  Instead we shoot 6 axis-aligned rays from
each test point and call the point INSIDE iff a majority of rays
report an odd number of intersections with the mesh.  This majority
vote is much more robust on borderline / non-manifold input.

Speed: mesh cache + process-pool dispatch
-----------------------------------------

Every printable part is built ONCE via ``_load_mesh(name)`` and
re-used (copied where mutated) across every check that needs it.
The check suite then runs in a ``concurrent.futures.ProcessPoolExecutor``
pool: parent process builds the master cache, pickles it via the
pool ``initializer=`` so each worker boots with every mesh
already preloaded, then submits the independent ``check_*`` checks
plus the per-pose chunks of ``check_workspace_self_collision``.
Results stream back as ``(name, ok, captured_stdout)`` tuples and
the parent prints them in DECLARATION ORDER -- the on-screen output
stays byte-for-byte identical to a serial run.

CLI flags
---------

* ``--serial`` -- skip the process pool and run every check in the main
  process in declaration order.  Use this when a worker traceback is
  mangled through pickle and you need a clean stack.
* ``--workers N`` -- override the default worker count (default is
  ``min(8, os.cpu_count())``).
* ``--profile PATH`` -- dump a cProfile snapshot of the PARENT process
  to ``PATH`` so you don't need the ``python -m cProfile`` wrapper.
  Workers are NOT profiled; combine with ``--serial`` if you want a
  full-suite profile.
* ``--only CHECK_NAME`` (repeatable) -- run only the named check(s);
  any check whose declaration-list name matches at least one ``--only``
  value is included.  Useful for iterating on a single check.
* ``--with-arm`` -- also run the OPTIONAL arm verification.
"""

from __future__ import annotations
import argparse
import sys
import os
import io
import functools
import hashlib
import inspect
import sqlite3
import time
import multiprocessing
import concurrent.futures
import cProfile
import threading
from contextlib import redirect_stdout

_PROTO_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_PROTO_DIR, "scripts"))

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix
from scipy.ndimage import distance_transform_edt, label
import shapely.geometry as _sg

import hexapod_prototype as hp


# ---------------------------------------------------------------------------
# Module-level mesh cache
# ---------------------------------------------------------------------------
#
# Every check used to call ``hp.make_<part>()`` directly, which triggers a
# fresh boolean-union / mesh-difference rebuild costing 0.2-2 s per part
# per call.  With 13+ checks and the workspace sweep, the same parts were
# being rebuilt 5-10 times each.  Routing every build through
# ``_load_mesh(name)`` makes each part hit the constructive-solid path
# EXACTLY ONCE per process; subsequent callers get the cached master
# (and an optional ``.copy()`` if they're going to ``apply_*`` mutate it).
#
# When ``main()`` dispatches checks to a ``ProcessPoolExecutor``, the
# parent process pre-fills the cache and pickles it via the worker
# initializer so each spawn-mode worker boots with every mesh already
# loaded -- no make-from-scratch cost in the worker.

_MESH_BUILDERS = {
    "chassis_top":      hp.make_chassis_top,
    "chassis_bottom":   hp.make_chassis_bottom,
    # Jun 2026 single-part merge: the HIGH/LOW print split is gone, so
    # ``chassis_bottom`` IS the whole printed plate (flat plate + bearing tower
    # + the folded-in flat floor slab carrying the yaw cradles + retainer
    # pilots).  ``chassis_assembled`` -- the key every cradle-feature /
    # world-geometry probe loads -- is therefore now the REAL merged
    # ``make_chassis_bottom`` (no longer the old bucketed ghost solid), so the
    # servo-insertion / wire-exit / disc-horn / bearing checks test the part
    # that actually prints.
    "chassis_assembled":    hp.make_chassis_bottom,
    "yaw_bearing_cap":  hp.make_yaw_bearing_cap,
    "servo_clamp_cap":  hp.make_servo_clamp_cap,
    "switch_holster":   hp.make_switch_holster,
    "coxa_link":        hp.make_coxa_link,
    "femur_link":       hp.make_femur_link,
    "tibia_link":       hp.make_tibia_link,
    # Aug 2026: TPU foot boot pressed over the tibia tube end (replaces
    # the hinged tibia_foot_fitting + foot_pad + M3x16/nyloc pin).
    "foot_boot":        hp.make_foot_boot,
    "servo_body":       hp.make_servo_body,
    # June 2026: the servo joints drive a 20 mm aluminum 25T DISC horn
    # (Amazon B07D56FVK5), so the "servo_horn" mating part IS the disc.
    # The engagement / mating-face / horn-sweep checks all read this
    # mesh, so pointing it at make_disc_horn keeps the verifier in sync
    # with the real hardware.  ``make_servo_horn`` (the plastic X-horn)
    # is retained for backwards compat but no longer used by the robot.
    "servo_horn":       hp.make_disc_horn,
}

ALL_CACHED_PART_NAMES = tuple(_MESH_BUILDERS.keys())

_MESH_CACHE: dict = {}
_MESH_CACHE_LOCK = threading.Lock()


def _load_mesh(name: str, *, copy: bool = True) -> trimesh.Trimesh:
    """Return the cached master mesh for ``name``.

    Default returns a ``mesh.copy()`` so callers that go on to
    ``apply_transform`` / ``apply_translation`` / ``apply_scale`` see
    the same semantics they used to get from a fresh ``hp.make_X()``
    call.  Pass ``copy=False`` only when the caller will NOT mutate
    the returned mesh (e.g. ``check_watertight`` reads ``mesh.volume``
    and ``mesh.is_watertight``; ``_flimsy_clusters_for_part`` calls
    ``mesh.voxelized(...)`` which is itself non-mutating).
    """
    cached = _MESH_CACHE.get(name)
    if cached is None:
        with _MESH_CACHE_LOCK:
            cached = _MESH_CACHE.get(name)
            if cached is None:
                cached = _MESH_BUILDERS[name]()
                _MESH_CACHE[name] = cached
    return cached.copy() if copy else cached


def _chassis_yaw_cradle_to_well_local(mesh: trimesh.Trimesh,
                                      leg_index: int = 0,
                                      ) -> trimesh.Trimesh:
    """Map a ``chassis_bottom`` mesh into the WELL-LOCAL frame of a
    single leg's yaw cradle.

    Used by the cradle-feature verifier checks (cradle openness,
    bolt-hole engagement, cradle insert pockets, horn-sweep
    clearance, servo insertion path) so the chassis_bottom-
    integrated yaw cradle can be probed in the SAME well-local
    frame as the bracket cradle used to be (May 2026 chassis-
    bottom-integrated yaw-cradle redesign, commit 6/8).

    Math: the transform is the inverse of ``fastener_registry.
    _yaw_cradle_T``:

        T_well_to_chassis = T(edge_mid)
                            @ Rz(a)
                            @ T(-SERVO_OUTPUT_X, 0,
                                well_to_chassis_dz)

    where ``well_to_chassis_dz = (CHASSIS_PLATE_T/2 +
    CRADLE_TAB_SHELF_Z) - WELL_RIM_Z = -19.25 mm``.  So the inverse
    is

        T_chassis_to_well = T(+SERVO_OUTPUT_X, 0, -well_to_chassis_dz)
                            @ Rz(-a)
                            @ T(-edge_mid)

    Well-local frame convention (matches ``_bracket_to_well_local``
    helpers in the cradle checks): origin at the body's BOTTOM face
    along the yaw axis; +Z up.  After this transform the leg's
    cradle features land at:

      * cradle tab shelf at well-local z = +WELL_RIM_Z          = +27.25
      * cradle boss top  at well-local z = +WELL_RIM_Z +
                            (CRADLE_BOSS_H - CRADLE_TAB_SHELF_Z) = +32.25
      * chassis_bottom plate top    at well-local z =
          +WELL_RIM_Z - CRADLE_TAB_SHELF_Z                       = +21.25
      * chassis_bottom plate bottom at well-local z =
          +WELL_RIM_Z - CRADLE_TAB_SHELF_Z - CHASSIS_PLATE_T     = +17.25
      * servo body bottom (when seated) at well-local z          = 0
      * servo body top                  at well-local z          = +38

    The other 5 cradles + the rest of the chassis_bottom plate
    material end up scattered far from the well-local origin (the
    plate is ~200 mm flat-to-flat; only the L0 cradle lands at the
    yaw axis).  Cradle-feature probes are localised to the L0
    cradle's well-local region and so are unaffected by the rest
    of the plate.
    """
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    # STS3215 front-face mount (Jun 2026): mirror fastener_registry.
    # _yaw_cradle_T's CAD-exact well-to-chassis shift, which lands the
    # disc-horn TOP at CHASSIS_YAW_OUTPUT_Z and makes the well-local
    # frame coincide with the cradle geometry (was the DS3225 tab-shelf
    # shift ``(CHASSIS_PLATE_T/2 + CRADLE_TAB_SHELF_Z) - WELL_RIM_Z =
    # -19.25``).
    well_to_chassis_dz = (
        hp.CHASSIS_YAW_OUTPUT_Z
        - (hp.SERVO_BODY_H + hp.WELL_PLATE_T + hp.HORN_STACK_H)
    )  # = -13.55 mm

    T1 = np.eye(4)
    T1[:3, 3] = -edge_mid
    Rz_inv = rotation_matrix(-a, [0, 0, 1])
    T3 = np.eye(4)
    T3[0, 3] = +hp.SERVO_OUTPUT_X
    T3[2, 3] = -well_to_chassis_dz
    T_chassis_to_well = T3 @ Rz_inv @ T1

    m = mesh.copy()
    m.apply_transform(T_chassis_to_well)
    return m


def prebuild_mesh_cache(names=ALL_CACHED_PART_NAMES) -> dict:
    """Build every cacheable mesh now and return the cache dict.

    Used by the parent process before dispatching to a process pool so
    the worker initializer can ship the full cache to every worker.
    """
    for name in names:
        _load_mesh(name, copy=False)
    return _MESH_CACHE


# ---------------------------------------------------------------------------
# Flimsy-joint check constants
# ---------------------------------------------------------------------------

# Minimum acceptable LOCAL THICKNESS for printed material.  Three perimeters
# at a 0.4 mm nozzle plus margin -- below this an FDM print becomes either
# entirely sparse infill (the slicer skips perimeters because the wall is
# too thin to fit even one bead) or a stack of < 3 perimeters that snaps
# under any real load.  Tweak this if your slicer / nozzle differs.
MIN_PRINT_T = 3.0           # mm
# Ignore clusters smaller than this -- single voxels and 1-voxel-wide
# surface "fuzz" are voxelisation artefacts of the grid pitch, not real
# weak spots.  At a 1.2 mm pitch a 20-voxel cluster is ~35 mm^3 of
# material thinner than MIN_PRINT_T, which is the smallest defect worth
# investigating.
MIN_CLUSTER_VOX = 20        # voxels
# Budget for the LARGEST flimsy cluster in any single part.  Anything
# bigger means a structurally meaningful thin region the user should
# reinforce before printing.
#
# Why not 200?  Long printed beams with sharp 90 deg edges (the femur
# spar, the tibia spar) produce an unavoidable voxelisation ARTEFACT
# along each long edge.  At MIN_PRINT_T = 3.0 mm and pitch = 1.2 mm,
# half_t_vox = 1.25 voxels, while a corner voxel's distance to its
# nearest core voxel is sqrt(2) ~= 1.414 voxels -- just over the
# threshold, regardless of how thick the underlying wall actually
# is.  A 130 mm-long spar with 4 long edges therefore stamps out
# ~ 130 / 1.2 = 108 corner voxels per edge, which scipy.label
# typically merges into one or two ridges of ~ 200-250 voxels each.
# We have visually verified these ridges (bbox shape: ~120 mm long,
# 4.8 mm wide, < 10 mm tall, 5-10 % voxel fill -- i.e. a 1-voxel-
# thick filament along the spar edge) on the tibia and femur spars;
# they map to real 90-deg edges in the part where the FDM print
# actually deposits >= 3 perimeters of material.  A real thin
# feature (1-2 mm wall over a > 10 x 10 mm area) produces > 1000
# voxels of flimsy and easily clears 250.  We pick 250 -- 25 %
# above the no-artefact baseline of 200 -- as the smallest value
# that admits the predicted corner-edge ridge without masking any
# structurally-meaningful thin region.
MAX_FLIMSY_BUDGET_VOX = 250 # voxels
# Voxel pitch for the local-thickness analysis.  1.2 mm gives roughly
# 80x80x40 = 250k voxels for a typical leg link, which voxelizes and
# distance-transforms in well under a second.
FLIMSY_VOXEL_PITCH = 1.2    # mm


# ---------------------------------------------------------------------------
# Thin-sheet ("structural neck") check constants
# ---------------------------------------------------------------------------
#
# Why a separate check?  ``check_flimsy_joints`` above uses the
# Hildebrand max-inscribed-ball local thickness, which only catches
# features that are thin in ALL THREE directions (spikes, pillars,
# hair-thin necks).  An inscribed sphere of radius MIN_PRINT_T/2 fits
# happily inside any 3 mm-thick slab, so a thin SHEET / WEB (thin in
# one direction, extended in the other two) sails through Hildebrand
# no matter how unprintably thin its load-bearing slab actually is.
#
# This check (a anisotropic chord variant of Hildebrand) computes the
# per-voxel MIN DIRECTIONAL CHORD LENGTH: for each occupied voxel V we
# measure the length, in mm, of the contiguous run of occupied voxels
# containing V along the +/-X, +/-Y, and +/-Z lines, then take the
# smallest of the three lengths.  Voxels whose min directional chord
# falls inside the STRUCTURAL-NECK BAND (between
# ``MIN_SHEET_T_LOWER`` and ``MIN_SHEET_T_UPPER`` mm) are clustered;
# the part FAILS if the largest cluster exceeds ``MAX_SHEET_BUDGET_VOX``.
#
# Why the LOWER bound (5 mm, not 0)?  Features thinner than this lower
# bound fall into one of two well-understood categories:
#
#   1. Unprintably thin (chord <= MIN_PRINT_T = 3 mm) -- the Hildebrand
#      check above already flags these.
#   2. Intentional thin PERIMETER walls / plates (servo well walls
#      WELL_WALL_Y = 4.5 mm, chassis plate CHASSIS_PLATE_T = 4 mm,
#      the retired servo horn adapter HORN_ADAPTER_T = 4 mm).  The user has
#      explicitly accepted these thicknesses in the design notes --
#      they are NOT structural load-path necks, and flagging them
#      here would just generate noise.
#
# The 5-6 mm "structural neck band" sits above the intentional thin-
# wall floor and at-or-below the user's MIN_SHEET_T = 6 mm.  The user's
# complaint with ``coxa_link`` (the arm slab between the hub and the
# well at 6 mm thick) lands squarely inside this band -- it's a slab
# that's "thick enough to look OK at a glance but thin enough to bend
# under load".
MIN_SHEET_T          = 6.0  # mm -- minimum acceptable thickness for
                             # a structural-neck load path.  A 6 mm
                             # slab between two thick blocks bends
                             # visibly under nominal hip-pitch
                             # reaction torque.  Independent of
                             # MIN_PRINT_T (3 mm).
# The chord lengths we measure on the voxel grid are AT LEAST the
# physical slab thickness, but can be up to one voxel pitch LARGER
# when the slab is sandwiched against another piece of solid
# material (the voxel grid does not know that there's a fillet /
# step / parting surface between them).  In coxa_link the 6 mm arm
# slab measures as 7.2 mm of chord_z (5 vox of arm + 1 vox of
# pedestal slab below).  We widen the upper threshold by one voxel
# pitch to absorb that quantization slack so the check reliably
# fires on the offending feature.
MIN_SHEET_T_UPPER    = MIN_SHEET_T + FLIMSY_VOXEL_PITCH  # 7.2 mm
MIN_SHEET_T_LOWER    = 5.0  # mm -- lower bound on the "structural-
                             # neck band".  Voxels with min chord
                             # below this represent intentional
                             # thin-wall design (see big comment
                             # above) and are NOT flagged by this
                             # check.
MIN_SHEET_CLUSTER_VOX = 400 # voxels -- ignore clusters smaller than
                             # this.  A 400-voxel cluster at pitch
                             # 1.2 mm is ~ 691 mm^3, roughly a
                             # 6 mm-thick slab 6 mm wide and 22 mm
                             # long.  This is a deliberate floor:
                             # SOME thin slabs are unavoidable as
                             # CLEARANCE features (e.g. coxa_link's
                             # 6 mm-wide arm strip at link y in
                             # [-3, +3] that lets the femur spar
                             # swing through at femur_pitch =
                             # -80 deg).  Setting the floor at
                             # 400 voxels excludes such intentional
                             # clearance strips while still flagging
                             # multi-cm structural slabs like the
                             # user's complaint about coxa_link's
                             # full-width arm (2040 voxels at
                             # arm_t = 6 mm, before reinforcement).
MAX_SHEET_BUDGET_VOX = 1100  # voxels -- per-part budget for the
                              # largest structural-neck cluster.
                              # Above this means a real structural
                              # neck exists.
                              #
                              # Raised 500 -> 1100 in May 2026 after
                              # dropping the coxa_link's redundant -Y
                              # arm stringer (commit 39554d8) exposed
                              # the bridge's 6.75 x 14.5 mm cross-
                              # section's TOP slice as a Y-narrow
                              # 6.0 x 31.2 mm cluster (894 vox,
                              # chord_y = 7.2 mm, centroid at coxa-
                              # link (+24, -14, +40) -- i.e. the
                              # top face of the bridge web that the
                              # stringer used to cap).  The cluster's
                              # PARENT geometry is the BRIDGE itself
                              # -- a structural beam supported below
                              # by the well wall and on either side
                              # by the +/-X arms, NOT a free-standing
                              # thin sheet.  The voxel-cluster
                              # classifier's "thin sheet" verdict is
                              # a false positive on this part because
                              # it sees only the 6 mm Y dimension of
                              # the top slice in isolation; the slice
                              # is fused to 14.5 mm of bridge web
                              # below.
                              #
                              # Engineering judgement (see commit
                              # 39554d8 reasoning): the bridge cross-
                              # section is OVER-spec for the hip-
                              # pitch load even with the stringer
                              # gone, and there is no print-quality
                              # risk because the cluster's bottom
                              # face is in continuous contact with
                              # 14.5 mm of bridge web that prints
                              # flat against the build plate /
                              # supports.  1100 vox = 894 + ~ 20%
                              # headroom, large enough that cache-
                              # miss or voxelizer-version drift on
                              # the same geometry won't flap the
                              # check, but still small enough that
                              # a REAL extra neck cluster on top of
                              # this one (say another 500+ vox of
                              # newly-thin geometry) would still
                              # trip the check.

# Parts to test for structural-neck topology.  We restrict to the
# coxa_link (the user's complaint).  The femur and tibia spar's
# 6 mm Y thickness is by-design uniform across their full length
# (the link prints flat on the build plate with its narrow Y
# dimension vertical, exactly LINK_THICKNESS = 6 mm), which is the
# correct printing strategy for a long flat plate but registers as a
# huge anisotropic "thin sheet" cluster under THIS test.  Flagging
# the femur / tibia spar would just generate noise -- they are not
# the "thick block - thin sheet - thick block" topology the user
# pointed at.  A future iteration could either thicken the spars
# globally or implement a richer "neck between thick volumes"
# detector (option (b) in this script's notes) to discriminate
# uniform spars from real load-path necks.
THIN_SHEET_PARTS = (
    "coxa_link",
)


# ---------------------------------------------------------------------------
# Robust inside-test (majority vote over 6 axis-aligned rays)
# ---------------------------------------------------------------------------
#
# ``points_inside`` has TWO interchangeable implementations:
#
#   * ``_points_inside_rays``  -- the historical 6-axis ray-vote.  Robust
#     against the boolean-union false positives that broke trimesh's
#     ``mesh.contains`` on early prototype geometry, but slow because
#     each call shoots 6 N-point ray batches and pays the rtree spatial
#     index cost on every batch.  This is what the original
#     ``points_inside`` was.
#
#   * ``_points_inside_contains`` -- a direct ``mesh.contains(pts)``
#     call.  Watertight meshes only -- if the input mesh is not water-
#     tight, trimesh's containment heuristic can disagree with the ray
#     vote on points near holes / non-manifold edges.  All cached
#     meshes are validated by ``check_watertight`` so this should be
#     equivalent to the ray vote in practice.  Much faster than 6
#     ray-cast batches per call (in profile the ray vote is >= 80% of
#     verifier wall time; mesh.contains is on the order of one ray
#     batch instead of six, plus avoids the rtree-per-direction work).
#
# A dispatcher (``points_inside``) picks between them via a module-
# level ``_inside_mode`` set from the CLI ``--inside-mode`` flag.  In
# the special ``both`` mode the dispatcher RUNS BOTH implementations,
# records every disagreement to ``_inside_mismatches``, prints a
# ``[MISMATCH]`` line to stderr, and STILL returns the ray-vote answer
# (so the rest of the check suite behaves byte-identically to a pure
# ``rays`` run while we collect equivalence evidence).  ``main()``
# exits with code 2 when any mismatch is recorded, so CI catches the
# discrepancy.

INSIDE_MODE_RAYS = "rays"
INSIDE_MODE_CONTAINS = "contains"
INSIDE_MODE_BOTH = "both"
INSIDE_MODES = (INSIDE_MODE_RAYS, INSIDE_MODE_CONTAINS, INSIDE_MODE_BOTH)

# Module-level default; ``main()`` overrides this from --inside-mode and
# the process-pool worker initializer propagates it to spawn-mode
# workers (see ``_worker_initializer``).
_inside_mode = INSIDE_MODE_RAYS

# Collected mismatch records when running in ``both`` mode.  In a
# spawn-mode worker, only mismatches accumulated by THIS worker's
# checks live here; the parent process aggregates them via
# ``_check_runner`` / ``_ws_pose_failures_kwargs`` return values.
_inside_mismatches: list = []

# Thread-local context naming the currently-running check.  Set by
# ``_check_runner`` (worker process) / ``_run_checks_serial`` (parent)
# / ``_ws_pose_failures_kwargs`` (worker process) so mismatch records
# can be attributed back to a specific check / pose without threading
# the name through every ``points_inside`` call site.
_inside_check_ctx = threading.local()


def _set_inside_check_context(name: str) -> None:
    _inside_check_ctx.name = name


def _get_inside_check_context() -> str:
    return getattr(_inside_check_ctx, "name", None) or "<unknown check>"


_RAY_DIRS = np.array([
    [+1.0, 0.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, +1.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, +1.0],
    [0.0, 0.0, -1.0],
])


def _points_inside_rays(mesh, pts):
    """Return a boolean array indicating whether each point lies inside
    ``mesh``.  Uses 6 axis-aligned rays per point and majority vote on
    intersection-count parity (odd = inside)."""
    pts = np.asarray(pts, dtype=float)
    if pts.ndim == 1:
        pts = pts[None, :]
    n = len(pts)
    votes = np.zeros(n, dtype=int)
    for direction in _RAY_DIRS:
        # Per ray, count intersections from each point along this direction.
        origins = pts
        directions = np.tile(direction, (n, 1))
        # Use ray.intersects_id which returns (locations, index_ray, _).
        # We only need a count per ray.
        try:
            _, index_ray, _ = mesh.ray.intersects_location(
                ray_origins=origins, ray_directions=directions)
        except Exception:
            continue
        if len(index_ray) == 0:
            continue
        counts = np.bincount(index_ray, minlength=n)
        votes += (counts % 2 == 1).astype(int)
    # Majority of 6 rays says odd-parity -> inside.
    return votes >= 4


def _points_inside_contains(mesh, pts):
    """Return ``mesh.contains(pts)`` as a numpy bool array.  Requires
    a watertight mesh -- callers must have validated this via
    ``check_watertight``."""
    pts = np.asarray(pts, dtype=float)
    if pts.ndim == 1:
        pts = pts[None, :]
    if len(pts) == 0:
        return np.zeros(0, dtype=bool)
    result = mesh.contains(pts)
    return np.asarray(result, dtype=bool)


def _mesh_identity(mesh) -> dict:
    """Return a small, picklable fingerprint of ``mesh`` for use in a
    mismatch record.  ``id(mesh)`` is useless across processes / when
    the caller is a transformed copy of a cached mesh, so we record
    volume + face/vertex counts + extents instead -- enough to
    distinguish the cached parts from each other in the post-hoc
    summary."""
    try:
        bounds = mesh.bounds
        extents = tuple(float(x) for x in (bounds[1] - bounds[0]))
    except Exception:
        extents = None
    try:
        vol = float(mesh.volume)
    except Exception:
        vol = None
    try:
        nv = int(len(mesh.vertices))
        nf = int(len(mesh.faces))
    except Exception:
        nv, nf = None, None
    try:
        watertight = bool(mesh.is_watertight)
    except Exception:
        watertight = None
    return {
        "volume":     vol,
        "n_vertices": nv,
        "n_faces":    nf,
        "extents":    extents,
        "watertight": watertight,
    }


def _record_inside_mismatch(mesh, pts, rays_result, contains_result) -> None:
    """Record a single ``points_inside`` disagreement between the ray
    vote and ``mesh.contains``.  Caller MUST have established that the
    two boolean arrays differ.  Caps coords/indices at the first 20
    per call to keep the record bounded.  Also prints a single
    ``[MISMATCH]`` line to stderr for live visibility."""
    diff = rays_result != contains_result
    n_diff = int(diff.sum())
    n_total = int(diff.size)
    if n_diff == 0:
        return
    idx_all = np.flatnonzero(diff)
    capped = idx_all[:20]
    coords_capped = pts[capped]
    rays_capped = rays_result[capped].astype(bool).tolist()
    contains_capped = contains_result[capped].astype(bool).tolist()
    mesh_id = _mesh_identity(mesh)
    record = {
        "check":         _get_inside_check_context(),
        "mesh":          mesh_id,
        "n_points":      n_total,
        "n_mismatch":    n_diff,
        "mismatch_idx":  [int(i) for i in capped.tolist()],
        "coords":        [tuple(float(c) for c in row)
                            for row in coords_capped],
        "rays_says":     rays_capped,
        "contains_says": contains_capped,
    }
    _inside_mismatches.append(record)
    # Keep the stderr line compact -- the full record is summarised at
    # main() exit.  Use stderr so it doesn't interleave with the
    # check's captured stdout (which the parent prints in declaration
    # order).
    vol_str = (f"vol={mesh_id['volume']:.1f}"
                if mesh_id["volume"] is not None
                else "vol=?")
    print(f"[MISMATCH] check={record['check']!r} mesh({vol_str} "
          f"nf={mesh_id['n_faces']} wt={mesh_id['watertight']}) "
          f"n_pts={n_total} n_diff={n_diff} "
          f"first_idx={record['mismatch_idx'][:5]}",
          file=sys.stderr, flush=True)


def points_inside(mesh, pts):
    """Dispatcher: return a boolean array indicating whether each point
    lies inside ``mesh``, picking between the historical 6-ray vote
    and ``mesh.contains`` according to the module-level
    ``_inside_mode``.  In ``both`` mode runs BOTH implementations and
    records any disagreement -- see the module-top docstring."""
    mode = _inside_mode
    if mode == INSIDE_MODE_RAYS:
        return _points_inside_rays(mesh, pts)
    if mode == INSIDE_MODE_CONTAINS:
        return _points_inside_contains(mesh, pts)
    if mode == INSIDE_MODE_BOTH:
        pts_arr = np.asarray(pts, dtype=float)
        if pts_arr.ndim == 1:
            pts_arr = pts_arr[None, :]
        rays_result = _points_inside_rays(mesh, pts_arr)
        contains_result = _points_inside_contains(mesh, pts_arr)
        if rays_result.shape == contains_result.shape:
            if np.any(rays_result != contains_result):
                _record_inside_mismatch(mesh, pts_arr,
                                          rays_result, contains_result)
        # Return the ray-vote answer so the rest of the suite is byte-
        # identical to a pure ``rays`` run while we collect mismatch
        # evidence.  The goal of ``both`` is observation, not a
        # correctness-via-majority-vote across implementations.
        return rays_result
    raise ValueError(f"unknown inside mode: {mode!r}")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _label(name, ok, detail=""):
    flag = "PASS" if ok else "FAIL"
    print(f"  [{flag}]  {name:50s}  {detail}")
    return ok


def _describe(mesh):
    return (f"vol={mesh.volume:8.1f} mm^3  "
            f"watertight={mesh.is_watertight}  "
            f"winding={mesh.is_winding_consistent}")


# ---------------------------------------------------------------------------
# 1.  Watertightness / manifold / volume sanity
# ---------------------------------------------------------------------------

def check_watertight():
    print("\n[1] Mesh watertightness / manifoldness:")
    # Design B (May 2026) + June 2026 disc-horn switch: the printed
    # servo_horn_adapter and the plastic 4-arm X-horn are both retired
    # (each link now bolts directly onto the 20 mm aluminum 25T disc
    # horn).  ``make_servo_horn_adapter`` is preserved for
    # backwards-compat but is no longer in the printable-output set.
    items_names = (
        "chassis_top", "chassis_bottom", 
        "servo_clamp_cap", "coxa_link",
        "femur_link", "tibia_link", "foot_boot",
    )
    all_ok = True
    for name in items_names:
        m = _load_mesh(name, copy=False)
        ok = m.is_watertight and m.is_winding_consistent and m.volume > 0
        all_ok &= _label(name, ok, _describe(m))
    return all_ok


# ---------------------------------------------------------------------------
# 1b.  Exported-STL manifoldness (what the SLICER sees)
# ---------------------------------------------------------------------------
#
# ``check_watertight`` uses ``trimesh.is_watertight``, which groups edges by
# rounded vertex position and therefore TOLERATES the sub-micron slivers /
# zero-area faces the manifold boolean kernel occasionally emits.  Bambu Studio
# (and every other slicer) re-welds the float32 binary-STL triangle soup by
# quantised position and flags those slivers as NON-MANIFOLD edges -- a real
# slicing blocker.  (Jun 2026: chassis_bottom_lower passed every watertight /
# single-body check yet Bambu rejected its STL with "29 non-manifold edges",
# traced to coincident cylindrical hole walls where the join-bolt bores were
# cut into the flange ring BEFORE it was unioned with the interpenetrating
# bucket walls.  Fixed by boring the bolts into the COMBINED solid + a
# manifold-simplify export heal; this guard keeps it from regressing.)
#
# The guard checks the SAME mesh ``_save`` writes -- i.e. after
# ``hp._heal_for_export`` -- and requires it to weld to a defect-free
# 2-manifold: 0 degenerate faces, 0 open edges, 0 edges shared by >2 faces.

def _export_manifold_selftest_meshes():
    """Return (clean_box, nonmanifold_fin) for the embedded self-test.

    ``nonmanifold_fin`` is three triangles sharing one common edge -- a
    textbook non-manifold "fin" -- so the float32-weld probe MUST report a
    >2-face edge.  If the probe ever passes it the guard has gone blind."""
    box = hp._box((10.0, 10.0, 10.0))
    fin = trimesh.Trimesh(
        vertices=np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0],
                           [0, 0, 1], [1, 1, 0]], dtype=float),
        faces=np.array([[0, 1, 2], [0, 1, 3], [0, 1, 4]], dtype=np.int64),
        process=False,
    )
    return box, fin


def check_export_manifold():
    """Assert every printed STL welds to a defect-free 2-manifold (0
    non-manifold edges) the way a slicer reads it -- the check
    ``is_watertight`` is too weak to make.

    Includes an embedded self-test: a 3-triangles-share-one-edge "fin" MUST
    be flagged non-manifold (and a clean box MUST pass), proving the float32
    weld probe is sensitive before the live parts are trusted."""
    print("\n[1b] Exported-STL manifoldness (slicer-weld; 0 non-manifold edges):")
    all_ok = True

    # ---- Self-test: sensitive to a known non-manifold fin ------------------
    box, fin = _export_manifold_selftest_meshes()
    fin_def = hp._exported_mesh_defects(fin)
    box_clean = hp._is_export_clean(box)
    self_ok = (fin_def["nonmanifold_edges"] > 0
               and not hp._is_export_clean(fin)
               and box_clean)
    all_ok &= _label(
        "SELF-TEST: 3-tri fin flagged non-manifold, clean box passes",
        self_ok,
        f"fin nonmanifold_edges={fin_def['nonmanifold_edges']} (>0); "
        f"box export-clean={box_clean} (probe is sensitive)")

    # ---- Live printed parts: the EXPORTED (healed) mesh must be clean ------
    for name, builder in _PRINTED_SINGLE_BODY_BUILDERS:
        exported = hp._heal_for_export(builder())
        d = hp._exported_mesh_defects(exported)
        ok = (d["degenerate"] == 0 and d["open_edges"] == 0
              and d["nonmanifold_edges"] == 0)
        detail = (f"degenerate={d['degenerate']}  open_edges={d['open_edges']}  "
                  f"nonmanifold_edges={d['nonmanifold_edges']}")
        all_ok &= _label(name, ok, detail)

    return all_ok


# ---------------------------------------------------------------------------
# 1b2.  Exported-STL freshness (on-disk STL == current parametric source)
# ---------------------------------------------------------------------------
#
# Every OTHER check builds its meshes fresh from hexapod_prototype (via
# ``_MESH_BUILDERS``) and never reads the on-disk ``stl_prototype/*.stl``.
# That makes the suite correct about the parametric SOURCE but BLIND to a
# real, recurring foot-gun: edit a ``make_*`` factory, forget to re-run
# ``build_all.py``, and the verifier still reports a clean pass while the
# actual STLs handed to the slicer are a build behind.  (Jun 2026:
# this bit us on coxa_yaw_hub + yaw_bearing_cap after the dual-bearing
# rework -- the source was right, the .stl files on disk were stale.)
#
# This guard walks the SAME ``hp.stl_export_groups()`` registry the exporter
# uses (so the two can never drift) and compares each part's geometry
# signature -- triangle count, |volume|, and axis-aligned bounds -- against
# the on-disk STL.  A missing or stale file FAILS with a "run `make build`"
# pointer.
#
# STEP-first print set (Aug 2026): the stl_prototype/ printables are now
# installed from the BREP exporter's healed tessellations
# (step_prototype/stl/ via step_pipeline.install_printables), NOT from the
# trimesh factories, so for printables the freshness reference is the
# healed BREP tessellation.  Three legs guard the whole chain:
#   1. the exporter manifest's source hashes must match the CURRENT
#      geometry sources (an edited builder with a stale step_prototype/
#      fails loudly);
#   2. the on-disk stl_prototype STL must match the healed BREP
#      tessellation's signature (stale install);
#   3. the BREP mesh must stay EQUIVALENT to its trimesh twin (loose vol /
#      bounds backstop -- the exporter enforces the tight gates including
#      max surface deviation; this re-checks it from the verifier side so
#      the probe suite, which builds from the trimesh factories, provably
#      tests the same geometry that prints).
# The _DO_NOT_PRINT visuals still come from the trimesh factories and are
# checked exactly as before.
#
# Why a tolerant signature and not an exact byte-compare: ``_save`` writes a
# float32 binary STL, so a re-export round-trips vertices through 32-bit
# floats (sub-micron drift).  Exact bytes would false-alarm on that noise;
# faces-exact + tight volume/bounds tolerances absorb the round-trip while a
# genuine source edit moves at least one signature far outside them.

_FRESHNESS_VOL_RTOL = 1e-4       # relative |volume| tolerance (float32 export)
_FRESHNESS_VOL_ATOL = 1e-3       # mm^3 floor so tiny parts aren't over-strict
_FRESHNESS_BOUNDS_ATOL = 1e-2    # mm, per-corner axis-aligned bounds tolerance
_EQUIV_VOL_RTOL_PCT = 1.0        # % |volume| BREP-vs-twin backstop
_EQUIV_BOUNDS_ATOL = 0.5         # mm per-corner BREP-vs-twin backstop


def _stl_signature(mesh):
    """(n_faces, |volume|, flat 6-vector of AABB corners) fingerprint used to
    compare a freshly-built+healed mesh against its on-disk STL."""
    return (int(len(mesh.faces)),
            float(abs(mesh.volume)),
            np.asarray(mesh.bounds, dtype=float).reshape(-1))


def check_exported_stl_freshness():
    """Assert every exported STL matches what the CURRENT parametric source
    would export -- i.e. nobody edited a ``make_*`` factory (or a BREP
    builder) and forgot to re-run ``build_all.py``.  Closes the gap that
    lets the rest of the suite pass on stale on-disk geometry (it all
    builds from source and never reads the files).

    STEP-first print set (Aug 2026): printables are checked against the
    healed BREP tessellation they are installed from, PLUS a loose
    equivalence backstop vs the trimesh twin (see the block comment
    above).  Visual meshes are still checked against their trimesh
    factories.

    Files are resolved with ``hp.stl_dir_for`` -- printables live in
    ``stl_prototype/`` and the ``_DO_NOT_PRINT`` reference/visual meshes in
    ``stl_reference/`` (this check used to hardcode ``stl_prototype/`` and
    reported every visual mesh MISSING after build_all.py started routing
    them to ``stl_reference/``)."""
    print("\n[1b2] Exported-STL freshness (on-disk STL == parametric source; "
          "re-run `make build` if any are stale):")
    import step_pipeline

    all_ok = True

    # Leg 1: the BREP exporter's manifest must match the CURRENT geometry
    # sources (hexapod_prototype.py, the BREP builders, the exporter).
    m_ok, m_detail = step_pipeline.manifest_fresh()
    all_ok &= _label("step_prototype manifest", m_ok, m_detail)

    for _section, builders in hp.stl_export_groups():
        for name, build in builders:
            base = name[:-4].replace(hp.NOPRINT_SUFFIX, "")
            path = os.path.join(hp.stl_dir_for(base), name)
            printable = hp.stl_dir_for(base) == hp.STL_DIR
            if not os.path.isfile(path):
                all_ok &= _label(name, False,
                                 "MISSING on disk -- run `make build`")
                continue
            try:
                twin = hp._heal_for_export(build())
            except Exception as exc:  # a build failure is its own bug, but flag it
                all_ok &= _label(name, False,
                                 f"rebuild raised {type(exc).__name__}: {exc}")
                continue
            if printable:
                # Leg 2 reference: the healed BREP tessellation the print
                # set was installed from.
                try:
                    fresh = step_pipeline.load_healed_brep(base)
                except FileNotFoundError:
                    all_ok &= _label(
                        name, False,
                        "no BREP tessellation in step_prototype/stl/ -- "
                        "run `make build`")
                    continue
            else:
                fresh = twin
            try:
                disk = trimesh.load(path, process=False)
                if isinstance(disk, trimesh.Scene):
                    disk = disk.dump(concatenate=True)
            except Exception as exc:
                all_ok &= _label(name, False, f"on-disk STL unreadable: {exc}")
                continue

            f_src, v_src, b_src = _stl_signature(fresh)
            f_disk, v_disk, b_disk = _stl_signature(disk)

            faces_ok = (f_src == f_disk)
            vol_tol = max(_FRESHNESS_VOL_ATOL,
                          _FRESHNESS_VOL_RTOL * max(v_src, v_disk))
            vol_ok = abs(v_src - v_disk) <= vol_tol
            bounds_off = float(np.max(np.abs(b_src - b_disk)))
            bounds_ok = bounds_off <= _FRESHNESS_BOUNDS_ATOL

            # Leg 3 (printables only): BREP-vs-twin equivalence backstop, so
            # the probe suite (which builds from the trimesh factories)
            # provably tests the geometry that actually prints.
            equiv_ok = True
            equiv_bits = []
            if printable:
                _f_t, v_twin, b_twin = _stl_signature(twin)
                vol_pct = 100.0 * abs(v_src - v_twin) / v_twin
                eq_bounds_off = float(np.max(np.abs(b_src - b_twin)))
                if vol_pct > _EQUIV_VOL_RTOL_PCT:
                    equiv_ok = False
                    equiv_bits.append(
                        f"BREP vs twin vol {vol_pct:.2f}% "
                        f"(> {_EQUIV_VOL_RTOL_PCT}%)")
                if eq_bounds_off > _EQUIV_BOUNDS_ATOL:
                    equiv_ok = False
                    equiv_bits.append(
                        f"BREP vs twin bounds off {eq_bounds_off:.2f} mm "
                        f"(> {_EQUIV_BOUNDS_ATOL})")

            ok = faces_ok and vol_ok and bounds_ok and equiv_ok
            if ok:
                src_tag = "BREP" if printable else "current"
                detail = f"faces={f_disk}  vol={v_disk:,.1f} mm^3 ({src_tag})"
            else:
                bits = []
                if not faces_ok:
                    bits.append(f"faces disk={f_disk} src={f_src}")
                if not vol_ok:
                    bits.append(f"vol disk={v_disk:,.1f} src={v_src:,.1f} mm^3")
                if not bounds_ok:
                    bits.append(f"bounds off {bounds_off:.2f} mm")
                bits.extend(equiv_bits)
                detail = "STALE -- run `make build`: " + "; ".join(bits)
            all_ok &= _label(name, ok, detail)

    return all_ok


# ---------------------------------------------------------------------------
# 1c.  Single-body connectivity (no floating islands)
# ---------------------------------------------------------------------------
#
# Every PRINTED part must come off the bed as ONE connected solid.  A
# disconnected island (a ring / boss / lip that lost the material that
# bridged it to the body) is a hard defect: it either drops off in the
# print or, worse, prints as loose debris inside the part.  Such artifacts
# are easy to introduce in parametric boolean CAD -- e.g. enlarging a bore
# or raising a shroud so a feature loses its overlap with the hub.  (June
# 2026: the spaced-6706-bearing-pair coxa rework left the yaw hub's Phi51.6
# dust-lip skirt floating 0.6 mm clear of a Phi44 platform; this guard was
# added to catch exactly that and any future recurrence.)
#
# Implementation note -- why a VOLUME filter, not a raw split count:
# ``mesh.split(only_watertight=False)`` faithfully returns every connected
# face cluster, INCLUDING zero-thickness coplanar patches that boolean ops
# sometimes shed on a flat face (e.g. chassis_bottom_lower's z=-2 underside
# splits into ~14 such 0.000 mm^3 sheets).  Those are mesh artifacts, not
# floating MATERIAL, so we count only components whose |volume| exceeds
# CONNECTIVITY_MIN_BODY_VOL.  A genuine floating island is bulk solid (the
# dust-lip ring was ~1940 mm^3), orders of magnitude above the threshold.
CONNECTIVITY_MIN_BODY_VOL = 1.0   # mm^3 -- below this is a meshing artifact

# Every part that ships as its OWN printed STL (one connected piece).  The
# assembled tibia_link mesh is NOT printed as a single part (two printed
# fittings + a CF tube), so it is intentionally excluded; its constituent
# printed fittings are listed instead.  coxa_link and femur_link ARE single
# printed parts (Aug 2026 coxa merge / Jul 2026 femur merge #2).  No part
# here is legitimately multi-body, so there is no whitelist: the required
# real-body count is exactly 1 for all of them.
_PRINTED_SINGLE_BODY_BUILDERS = (
    ("chassis_top",          hp.make_chassis_top),
    ("chassis_bottom",       hp.make_chassis_bottom),
    ("switch_holster",       hp.make_switch_holster),
    ("yaw_servo_retainer",   hp.make_yaw_servo_retainer),
    ("yaw_bearing_cap",      hp.make_yaw_bearing_cap),
    # wago_mount RETIRED late-Aug 2026: the corner power-Wago tray walls
    # are integrated into chassis_bottom (checked as part of that solid).
    ("coxa_link",            hp.make_coxa_link_part),
    ("femur_link",           hp.make_femur_link_part),
    ("tibia_knee_yoke",      hp.make_tibia_knee_yoke),
    ("foot_boot",            hp.make_foot_boot),
    ("servo_clamp_cap",      hp.make_servo_clamp_cap),
)


def check_single_connected_component():
    """Assert every printed part is a SINGLE connected solid body (no
    disconnected / floating geometry).

    Splits each printed mesh into connected components and counts the ones
    carrying real material (|volume| > CONNECTIVITY_MIN_BODY_VOL); the part
    PASSES iff that count is exactly 1.  Zero-volume coplanar mesh patches
    shed by boolean ops are ignored (see the module note above).
    """
    print("\n[1c] Single-body connectivity (no floating islands):")
    all_ok = True
    for name, builder in _PRINTED_SINGLE_BODY_BUILDERS:
        mesh = builder()
        comps = mesh.split(only_watertight=False)
        bodies = [c for c in comps if abs(c.volume) > CONNECTIVITY_MIN_BODY_VOL]
        n = len(bodies)
        ok = n == 1
        if ok:
            detail = f"1 connected body (vol {mesh.volume:8.1f} mm^3)"
        else:
            extra = sorted((abs(c.volume) for c in bodies), reverse=True)[1:]
            detail = (f"{n} disconnected bodies -- FLOATING ISLAND(S); "
                      f"stray body vol(s) = "
                      f"{', '.join(f'{v:.1f}' for v in extra)} mm^3")
        all_ok &= _label(name, ok, detail)
    return all_ok


# ---------------------------------------------------------------------------
# 1d.  Hexagonal (C6) rotational symmetry
# ---------------------------------------------------------------------------
#
# The merged ``chassis_bottom``'s FLOOR slab (``_chassis_bottom_floor_solid``,
# the C6-bearing portion that carries the 6 servo cutouts + retainer pilots)
# must have TRUE 6-fold symmetry about the central yaw (Z) axis.  Its six legs
# are built by replicating ONE canonical 60-deg sector of cutters by an EXACT
# k*60-deg rotation, so the symmetry is structural -- not an accident of six
# independent per-arm booleans.  (The full chassis_bottom legitimately breaks
# C6 only via the off-centre battery/electronics features in the plate above,
# so the guard probes the floor slab, not the whole part.)  (Jun 2026: an
# earlier per-arm build left an asymmetric, degenerate centre that the slicer
# rendered as shredded garbage even though it passed every watertight /
# manifold / single-body probe; this guard makes any loss of 6-fold symmetry
# a HARD failure.)
#
# Metric -- tessellation-INDEPENDENT: classify a dense grid of points
# inside/outside the mesh, and inside/outside the mesh ROTATED 60 deg
# (equivalently rotate the query points by -60 deg).  A perfectly C6 solid
# yields identical occupancy fields, so the mismatch FRACTION over occupied
# cells is ~0; a real asymmetry lights up the differing cells.  Vertices are
# deliberately NOT compared: the flat hex faces are fanned from an arbitrary
# apex, so the tessellation is not itself symmetric even when the SHAPE is.
C6_SYMMETRY_PITCH = 1.5      # mm -- occupancy-probe grid pitch
C6_SYMMETRY_TOL   = 0.003    # max mismatch fraction (a true C6 part scores ~0;
                             # the tiny residual is grid aliasing at the
                             # boundary, well clear of a real asymmetry)


def _c6_occupancy_mismatch(mesh, n_fold=6, pitch=C6_SYMMETRY_PITCH):
    """Return (mismatch_fraction, n_inside) for ``mesh`` vs its own copy
    rotated 360/n_fold deg about Z -- 0.0 for a perfectly n-fold part."""
    lo, hi = mesh.bounds
    xs = np.arange(lo[0] + pitch / 2.0, hi[0], pitch)
    ys = np.arange(lo[1] + pitch / 2.0, hi[1], pitch)
    zs = np.arange(lo[2] + pitch / 2.0, hi[2], pitch)
    X, Y, Z = np.meshgrid(xs, ys, zs, indexing="ij")
    P = np.c_[X.ravel(), Y.ravel(), Z.ravel()]
    a = mesh.contains(P)
    ang = 2.0 * np.pi / n_fold
    Rm = trimesh.transformations.rotation_matrix(ang, [0, 0, 1])[:3, :3]
    b = mesh.contains((Rm @ P.T).T)
    inside = int(a.sum())
    if inside == 0:
        return 1.0, 0
    return float(np.count_nonzero(a != b)) / inside, inside


def check_c6_symmetry():
    """Assert the merged chassis_bottom's FLOOR slab
    (``_chassis_bottom_floor_solid``) is invariant under a 60-deg rotation
    about the central Z axis (true hexagonal / C6 symmetry).

    Self-test: a regular hex prism MUST pass; the SAME prism with one extra
    off-axis hole (a deliberate symmetry break) MUST be flagged -- proving the
    occupancy probe is sensitive before the live part is trusted."""
    print("\n[1d] Hexagonal C6 symmetry (invariant under 60-deg rotation):")
    all_ok = True

    # ---- Self-test: sensitive to a broken-symmetry control -----------------
    hex_prism = hp._cyl(20.0, 4.0, sections=6)
    sym_mm, _ = _c6_occupancy_mismatch(hex_prism)
    hole = hp._cyl(3.0, 10.0)
    hole.apply_translation([10.0, 2.0, 0.0])
    asym = hp._diff(hp._cyl(20.0, 4.0, sections=6), hole)
    asym_mm, _ = _c6_occupancy_mismatch(asym)
    self_ok = (sym_mm <= C6_SYMMETRY_TOL) and (asym_mm > C6_SYMMETRY_TOL)
    all_ok &= _label(
        "SELF-TEST: hex prism C6, off-axis-holed prism flagged",
        self_ok,
        f"hex mismatch={sym_mm:.5f} (<= {C6_SYMMETRY_TOL}); "
        f"holed mismatch={asym_mm:.5f} (probe is sensitive)")

    # ---- Live part (the C6-bearing floor slab of the merged chassis) -------
    mm, inside = _c6_occupancy_mismatch(hp._chassis_bottom_floor_solid())
    ok = mm <= C6_SYMMETRY_TOL
    all_ok &= _label(
        "chassis_bottom floor slab", ok,
        f"60-deg-rotation occupancy mismatch={mm:.5f} "
        f"(<= {C6_SYMMETRY_TOL}; {inside} cells inside)")
    return all_ok


# ---------------------------------------------------------------------------
# 2.  Cradle insertion-path openness
# ---------------------------------------------------------------------------

def _cradle_open(part_mesh, body_centre, body_long_axis, body_short_axis,
                  open_dir, name=""):
    """Probe whether the servo body can be inserted into a cradle along
    ``open_dir``."""
    body_centre = np.asarray(body_centre, dtype=float)
    n = np.asarray(open_dir, dtype=float)
    n = n / np.linalg.norm(n)
    u = np.asarray(body_long_axis, dtype=float)
    u = u / np.linalg.norm(u)
    v = np.asarray(body_short_axis, dtype=float)
    v = v / np.linalg.norm(v)

    half_u = hp.SERVO_BODY_W / 2.0 - 0.5
    half_v = hp.SERVO_BODY_D / 2.0 - 0.5

    bb_min, bb_max = part_mesh.bounds
    proj = (bb_max - bb_min) @ np.abs(n)
    travel = float(proj) + hp.SERVO_BODY_H + 5.0

    Nu, Nv, Nt = 5, 5, 25
    samples = []
    for i in np.linspace(-half_u, half_u, Nu):
        for j in np.linspace(-half_v, half_v, Nv):
            for k in np.linspace(0.0, travel, Nt):
                p = body_centre + i * u + j * v + k * n
                samples.append(p)
    samples = np.asarray(samples)

    inside = points_inside(part_mesh, samples)
    n_blocked = int(inside.sum())
    n_total = len(samples)
    return n_blocked == 0, n_blocked, n_total


def _body_fits_in_part(part_mesh, body_centre, body_long_axis,
                        body_short_axis, body_normal_axis,
                        long_extent, short_extent, normal_extent):
    """Probe whether the servo body's FINAL resting volume is free of
    bracket material.  Used for cradles whose top is intentionally
    closed (i.e. the body does NOT drop in along a single straight
    insertion axis).  Samples a 3D grid filling the body's final
    bounding box and reports how many sample points are inside the
    part mesh.
    """
    body_centre = np.asarray(body_centre, dtype=float)
    u = np.asarray(body_long_axis,   dtype=float); u = u / np.linalg.norm(u)
    v = np.asarray(body_short_axis,  dtype=float); v = v / np.linalg.norm(v)
    w = np.asarray(body_normal_axis, dtype=float); w = w / np.linalg.norm(w)

    Nu, Nv, Nw = 5, 5, 9
    samples = []
    for i in np.linspace(-long_extent  / 2.0 + 0.5, long_extent  / 2.0 - 0.5, Nu):
        for j in np.linspace(-short_extent / 2.0 + 0.5, short_extent / 2.0 - 0.5, Nv):
            for k in np.linspace(-normal_extent / 2.0 + 0.5,
                                  normal_extent / 2.0 - 0.5, Nw):
                p = body_centre + i * u + j * v + k * w
                samples.append(p)
    samples = np.asarray(samples)
    inside = points_inside(part_mesh, samples)
    n_blocked = int(inside.sum())
    n_total = len(samples)
    return n_blocked == 0, n_blocked, n_total


def check_cradle_openness():
    """STS3215 front-face mount (Jun 2026): body-seat clearance.

    The DS3225 servo dropped into an open-TOP bucket and was retained by
    its side ears on a shelf, so this check probed an open insertion
    COLUMN above the body (``open_dir = +Z / +Y``).  The STS3215 has no
    ears and does NOT drop in from the top -- it is inserted from the
    cradle's OPEN BACK and pushed forward until its front (output) face
    seats against the mount plate (see ``_chassis_yaw_cradle_solid`` /
    ``_servo_well_solid``).  The relevant question is therefore no
    longer "is the column above the body void?" (the front plate now
    intentionally caps that direction) but "is the body's FINAL resting
    volume clear of cradle material?".  We place a slightly-shrunk
    servo-body envelope box at each cradle's seated body pose (via the
    same ``_servo_body_world_transform`` the fastener / mating-face
    checks use) and require its overlap with the cradle's printed part
    to stay under tolerance.  A non-trivial overlap means the body
    cavity is mis-sized or a wall intrudes (e.g. the STS3215 body being
    4.8 mm deeper than the DS3225 fouling the arm -- the regression
    ``COXA_LIFT`` was raised to clear).
    """
    print("\n[2] Cradle body-seat clearance (STS3215 front-face mount):")
    import fastener_registry as _fr  # noqa: WPS433
    all_ok = True

    # Body envelope shrunk 0.5 mm/side so we test the body proper, not
    # the intended WELL_BODY_CL cavity clearance gap around it.
    ext = np.array([hp.SERVO_BODY_W - 1.0,
                    hp.SERVO_BODY_D - 1.0,
                    hp.SERVO_BODY_H - 1.0])
    body_centre_offset = _fr._T(0.0, 0.0, hp.SERVO_BODY_H / 2.0)
    cradle_part = {"yaw": "chassis_bottom",
                   "hip": "coxa_link",
                   "knee": "femur_link"}
    parts = _build_world_assembly_parts(leg_index=0)
    TOL = 150.0
    for joint, partname in cradle_part.items():
        box = trimesh.creation.box(extents=ext)
        T = _servo_body_world_transform(joint, 0) @ body_centre_offset
        box.apply_transform(T)
        vol = _pair_overlap_volume(box, parts[partname], pitch=1.0)
        ok = vol <= TOL
        all_ok &= _label(
            f"{partname:14s} ({joint} cradle) body resting volume clear",
            ok, f"overlap = {vol:7.1f} mm^3 (tol {TOL:.0f})")

    return all_ok


# ---------------------------------------------------------------------------
# 3.  Bolt hole hits material
# ---------------------------------------------------------------------------

def check_bolt_holes():
    print("\n[3] Bolt-hole material engagement:")

    # STS3215 (Jun 2026): RETIRED.
    #
    # The DS3225 design clamped each servo by its side EARS with 4 bolts
    # that either self-tapped into Phi 2.5 mm pilots in the well walls
    # or threaded into heat-set inserts; this check probed for printed
    # wall material around those pilots.  The STS3215 has no ears, and
    # the model's earlier "front-face 4-bolt case-screw mount" on the
    # servo OUTPUT face was a PHANTOM feature -- the authoritative
    # Waveshare ST3215 bracket geometry shows the output face carries no
    # usable body-mount screws (the dia-20 disc horn covers the dia-14
    # cross).  Those 72 case screws have been removed; the servo body is
    # retained by PRINTED parts (yaw: ``make_yaw_servo_retainer`` strap;
    # hip/knee: ``make_servo_clamp_cap`` clamshell, 2 x M3 self-tap per
    # joint -- probed by ``check_servo_insertion_path`` /
    # ``check_screwdriver_access``).  The output face carries only the
    # flush disc horn + its 4 x M3 leg bolts, so there is no printed
    # thread-engagement medium for THIS probe to test.
    print("  [SKIP]  servo pilot probe retired (STS3215 body held by the "
          "printed strap/clamp; output face carries only the disc horn)")
    return True


# ---------------------------------------------------------------------------
# 3a.  Clamp-cap <-> bracket/cradle bolt-pattern coaxiality
# ---------------------------------------------------------------------------
#
# The STS3215 body is trapped by a bolt-on PRINTED retainer that has a
# clearance hole on each side which MUST land coaxially with a self-tap
# pilot in the mating cradle/bracket wall, or the screw cannot pull the
# joint shut ("the bracket and the clamp screw holes don't line up"):
#
#   * hip + knee (sandwich joints): ``make_servo_clamp_cap`` clamshell, its
#     2 x M3 flange clearance holes thread -Y into the +/-X wall-end pilots
#     cut by ``_servo_well_solid``.  Both read
#     ``hexapod_prototype.servo_clamp_bolt_centres``.
#   * yaw (chassis cradle): ``make_yaw_servo_retainer`` strap, its 2 end-wall
#     M3 clearance holes thread +Z into the wall-bottom pilots cut by
#     ``_chassis_yaw_cradle_solid``.  Both read
#     ``hexapod_prototype.yaw_retainer_anchor_centres``.
#
# This guard INDEPENDENTLY extracts the actual bore centre from EACH mesh
# (the retainer/cap and its mating cradle/bracket) by sampling for the void,
# then asserts the two are coaxial within COAXIAL_TOL.  Because the centres
# are read back from the meshes (not from the shared helper), the guard still
# fires if a future edit hard-codes a drifted pattern into one side only.

COAXIAL_TOL_MM = 0.30   # mm -- max in-plane centre offset cap-hole vs pilot


def _bore_centre_in_plane(mesh, axis_idx, plane_val, u_idx, v_idx,
                          u_guess, v_guess, win=2.0, step=0.2):
    """Independently locate a small through-bore centre in ``mesh``.

    Sample a (u, v) grid centred on (``u_guess``, ``v_guess``) lying in the
    plane where coordinate ``axis_idx`` == ``plane_val`` (the plane is
    perpendicular to the bore axis).  The bore is the VOID region.  Returns
    ``(u_centre, v_centre, n_void)``; ``n_void == 0`` means no bore was found
    near the guess (a missing / badly drifted hole).
    """
    us = np.arange(u_guess - win, u_guess + win + 1e-9, step)
    vs = np.arange(v_guess - win, v_guess + win + 1e-9, step)
    pts = np.zeros((len(us) * len(vs), 3))
    uu, vv = np.meshgrid(us, vs)
    pts[:, axis_idx] = plane_val
    pts[:, u_idx] = uu.ravel()
    pts[:, v_idx] = vv.ravel()
    inside = mesh.contains(pts)
    void = pts[~inside]
    if len(void) == 0:
        return None, None, 0
    return float(void[:, u_idx].mean()), float(void[:, v_idx].mean()), len(void)


def check_clamp_cap_alignment():
    print("\n[3a] Clamp-cap <-> bracket/cradle bolt-pattern coaxiality:")
    ok = True

    # ---- Hip + knee: clamp cap flange holes vs cradle wall-end pilots -----
    # Shared well-local frame (origin = body back-face centre, +X long,
    # +Y depth, +Z output); the bolt axis is Y, so coaxiality is an (x, z)
    # match.  Sample the pilot in a plane INSIDE the wall and the cap hole in
    # a plane through the flange.
    well = hp._servo_well_solid()
    cap = hp.make_servo_clamp_cap()
    pilot_plane_y = hp.WELL_D / 2.0 - 3.0                       # inside the wall
    cap_plane_y = hp.WELL_D / 2.0 + hp.CLAMP_CAP_T / 2.0        # mid-flange
    for (bx, bz) in hp.servo_clamp_bolt_centres():
        side = "+X" if bx > 0 else "-X"
        px, pz, npv = _bore_centre_in_plane(
            well, axis_idx=1, plane_val=pilot_plane_y,
            u_idx=0, v_idx=2, u_guess=bx, v_guess=bz)
        cx, cz, ncv = _bore_centre_in_plane(
            cap, axis_idx=1, plane_val=cap_plane_y,
            u_idx=0, v_idx=2, u_guess=bx, v_guess=bz)
        if npv == 0 or ncv == 0:
            ok = False
            miss = "cradle pilot" if npv == 0 else "cap hole"
            print(f"  [FAIL]  hip/knee {side}: no {miss} bore found near "
                  f"expected (x={bx:.2f}, z={bz:.2f})")
            continue
        off = float(np.hypot(px - cx, pz - cz))
        flag = "PASS" if off <= COAXIAL_TOL_MM else "FAIL"
        if off > COAXIAL_TOL_MM:
            ok = False
        print(f"  [{flag}]  hip/knee {side} clamp bolt: cap hole "
              f"(x={cx:6.2f}, z={cz:6.2f}) vs cradle pilot "
              f"(x={px:6.2f}, z={pz:6.2f})  offset={off:.3f} mm "
              f"(tol {COAXIAL_TOL_MM})")

    # ---- Yaw: retainer-stirrup arm holes vs REAL merged-chassis pilots ----
    # (Jun 2026 flat-chassis re-anchor / single-part merge.)  The capture
    # stirrup (make_yaw_servo_retainer) bolts UP into 2 blind self-tap pilots
    # cut in the actual printed chassis_bottom floor slab -- NOT the phantom
    # _chassis_yaw_cradle_solid the old strap referenced.  We test against the
    # REAL merged plate so a future plate edit that drops/moves the pilots
    # fires this guard.  Both bolt axes are Z; coaxiality is an (x, y) match.
    # The strap is built in cradle-local XY + world Z, so we map each anchor's
    # cradle-local (x, y) into the canonical leg-0 world frame (apothem dir
    # a=pi/6) and probe both meshes (plate in its own frame == world; strap
    # placed by the same Ra + edge_mid the scene/servo placement uses).
    a = 0.5 * np.pi / 3
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    Ra = rotation_matrix(a, [0, 0, 1])
    plate = hp.make_chassis_bottom()
    strap = hp.make_yaw_servo_retainer().copy()
    strap.apply_transform(Ra)
    strap.apply_translation(edge_mid)
    plate_bot = hp.CHASSIS_SPLIT_Z - hp.CHASSIS_BOTTOM_FLOOR_T
    pilot_plane_z = plate_bot + hp.RETAINER_PLATE_PILOT_DEPTH / 2.0   # inside the blind pilot
    # Jun 2026 saddle redesign: the anchor clearance hole now lives in the
    # saddle's TOP FLANGE band z[plate_bot - SADDLE_FLANGE_T, plate_bot], so
    # probe the flange mid-plane (was the old stirrup arm at plate_bot - 6).
    strap_plane_z = plate_bot - hp.SADDLE_FLANGE_T / 2.0             # inside the flange-tab hole
    for (cx, cy) in hp.chassis_lower_retainer_anchor_centres():
        side = "+Y" if cy > 0 else "-Y"
        w = Ra @ np.array([cx, cy, 0.0, 1.0])
        wx, wy = w[0] + edge_mid[0], w[1] + edge_mid[1]
        px, py, npv = _bore_centre_in_plane(
            plate, axis_idx=2, plane_val=pilot_plane_z,
            u_idx=0, v_idx=1, u_guess=wx, v_guess=wy)
        sx_, sy_, nsv = _bore_centre_in_plane(
            strap, axis_idx=2, plane_val=strap_plane_z,
            u_idx=0, v_idx=1, u_guess=wx, v_guess=wy)
        if npv == 0 or nsv == 0:
            ok = False
            miss = "plate pilot" if npv == 0 else "strap arm hole"
            print(f"  [FAIL]  yaw {side}: no {miss} bore found near "
                  f"expected world (x={wx:.2f}, y={wy:.2f})")
            continue
        off = float(np.hypot(px - sx_, py - sy_))
        flag = "PASS" if off <= COAXIAL_TOL_MM else "FAIL"
        if off > COAXIAL_TOL_MM:
            ok = False
        print(f"  [{flag}]  yaw {side} anchor bolt: strap hole "
              f"(x={sx_:6.2f}, y={sy_:6.2f}) vs plate pilot "
              f"(x={px:6.2f}, y={py:6.2f})  offset={off:.3f} mm "
              f"(tol {COAXIAL_TOL_MM})")

    print("  [PASS]  all clamp/retainer bolt holes coaxial with their "
          "mating pilots" if ok else
          "  [FAIL]  clamp/retainer bolt-pattern misalignment detected")
    return ok


# ---------------------------------------------------------------------------
# 3b.  Wire-exit L-corridor at the -X bottom-OUTBOARD corner of the well
# ---------------------------------------------------------------------------
#
# Note on the previous version of this test
# -----------------------------------------
# An earlier ``check_wire_slot`` looked at TWO probe points in well-local
# coords -- one expected-VOID at (-25, 0, 0) and one expected-SOLID at
# (-25, 0, 10) -- and asserted the latter must be intact -X wall material.
# That assumption was stale: ``_wire_exit_slot`` was extended in the recent
# redesign to also cut a vertical CHANNEL into the -X wall above the L
# exit (so wires lying against the body's back-case can run down the wall
# face into the L), which legitimately removes material at well-local
# x in [-24.7, -17.5], z in [0, ~30].  The "SOLID" probe at (-25, 0, 10)
# sat 0.3 mm outboard of the channel in X, but the test sampled a 1 mm
# cube around it whose inboard voxels (x = -24.5) fell INSIDE the
# channel cut, so the test always reported the wall as broken even
# though the wall material it cares about (the M3 pilot zone above the
# channel top) was fully intact.  The M3 pilot wall is already
# independently verified by ``check_bolt_holes`` ("M3 pilots in well
# wall"), so the wire-exit check should focus on the wire's path and
# not on incidental wall geometry above it.
#
# What this check now models
# --------------------------
# A hobby servo's 3-wire harness exits the body at the bottom-outboard
# CORNER (well-local -X face of the body, at z=0, around y=0).  The
# wire must be able to reach FREE SPACE outside the cradle via the
# L-shaped slot.  The slot supports two routings out of the corner:
#
#   * LATERAL leg:  bundle travels in -X out through the bottom of the
#                   -X wall, past the part's outer face.
#   * DOWNWARD leg: bundle travels in -Z down through the well floor,
#                   past the part's outer bottom face.
#
# Each leg is a 3D corridor with cross-section ~WIRE_SLOT_W (Y) x ~3 mm
# (orthogonal).  We sample a grid of points inside each corridor in
# well-local coordinates and transform them into the part's frame via
# the same R / translation chain ``make_*`` applies to the well solid.
# A corridor is "clear" if every sampled point is OUTSIDE the part
# mesh.  The check PASSES if AT LEAST ONE corridor is clear -- the
# wire only needs ONE escape route, and a closed top flange (the user's
# recent reinforcement on the coxa bracket) does not interfere with
# either leg of the L.

# Wire bundle Y margin from the slot's Y face -- the slot is sized for
# bundled / jacketed harnesses (WIRE_SLOT_W = 7 mm), the wire itself
# needs much less so we sample inside the slot Y span by this margin.
_WIRE_PROBE_Y_MARGIN = 0.75
# Wire bundle thickness in the orthogonal direction (Z for the lateral
# leg, X for the downward leg).  3 mm comfortably accommodates a 3-
# conductor 22 AWG servo harness.
_WIRE_PROBE_BUNDLE_T = 3.0


def _well_to_cradle(p_well, R, t):
    """Map a point in well-local coordinates to cradle-local coordinates
    using the same R/translation chain ``make_*`` applies to the well
    solid.  ``R`` may be ``None`` (identity).  ``t`` is the total
    translation in CRADLE-LOCAL coords (applied AFTER R).
    """
    p = np.asarray(p_well, dtype=float).copy()
    if R is not None:
        p = R[:3, :3] @ p
    return p + np.asarray(t, dtype=float)


def _wire_corridor_points():
    """Return ``(lateral_pts, downward_pts, boot_pts)`` -- the two L-leg
    corridors plus the wire-exit boot footprint, expressed as N x 3
    arrays of well-local sample points.

    The corridors are built from ``WIRE_SLOT_*`` constants in
    ``hexapod_prototype`` and are common across all three cradles
    (yaw / hip-pitch / knee).  The corridor end-points reach 0.5 mm
    inside the slot's outer faces -- far enough past the part's outer
    wall that "all corridor points are void" means "the wire has
    reached free space".

    The boot footprint is the rectangular volume the servo's molded
    +X wire-exit boot will occupy when the body is fully seated; if
    any part-mesh material intersects this volume the body cannot
    seat without crushing or bending the boot.  See
    WIRE_BOOT_* constants in hexapod_prototype.py for the dimensions.
    """
    Y_HALF = hp.WIRE_SLOT_W / 2.0 - _WIRE_PROBE_Y_MARGIN
    H_HALF = _WIRE_PROBE_BUNDLE_T / 2.0

    # Y span (common to both legs): three transverse samples across the
    # wire bundle Y extent.
    bundle_y = np.linspace(-Y_HALF, +Y_HALF, 3)

    # ---- LATERAL leg ---------------------------------------------------
    # x runs from just inside the slot's inboard face (= cavity-side
    # opening, just past the body's +X face) to 0.5 mm shy of the
    # slot's outboard face (just past the part's outer +X wall).
    lat_x_start = (+hp.SERVO_BODY_W / 2.0
                   - hp.WIRE_SLOT_X_INBOARD + 0.5)
    lat_x_end   = (+hp.WELL_W / 2.0
                   + hp.WIRE_SLOT_X_PAST_WALL - 0.5)
    lat_x = np.linspace(lat_x_start, lat_x_end, 9)
    # Bundle Z span sits at floor level (z = 0 is the cavity floor top
    # / body bottom face); the wire lies flat against the floor on
    # its way out.
    lat_z = np.linspace(-H_HALF, +H_HALF, 2)
    Xx, Yy, Zz = np.meshgrid(lat_x, bundle_y, lat_z, indexing="ij")
    lateral_pts = np.stack([Xx.ravel(), Yy.ravel(), Zz.ravel()], axis=1)

    # ---- DOWNWARD leg --------------------------------------------------
    # z runs from just above the cavity floor to 0.5 mm shy of the
    # slot's outermost Z (which sits past the part's outer bottom).
    down_z_start = +0.5
    down_z_end   = (-hp.WELL_FLOOR_T
                    - hp.WIRE_SLOT_Z_BELOW_FLOOR + 0.5)
    down_z = np.linspace(down_z_start, down_z_end, 9)
    # Bundle X span sits at the body-corner / slot-inboard edge; the
    # wire drops straight down out of the +X corner.
    down_x_centre = +hp.SERVO_BODY_W / 2.0 - hp.WIRE_SLOT_X_INBOARD + 0.5
    down_x = np.linspace(down_x_centre - H_HALF,
                         down_x_centre + H_HALF, 2)
    Xx, Yy, Zz = np.meshgrid(down_x, bundle_y, down_z, indexing="ij")
    downward_pts = np.stack([Xx.ravel(), Yy.ravel(), Zz.ravel()], axis=1)

    # ---- BOOT footprint ------------------------------------------------
    # The servo's molded +X wire-exit boot occupies the rectangular
    # volume just outside the body's +X face, between the body and
    # the well's +X cavity wall (and slightly into the wall thanks to
    # the WIRE_CHANNEL).  Sample a tight grid inside that volume and
    # require every sample to be CLEAR of part material so the boot
    # can sit there when the body is fully seated.
    boot_x_min = +hp.SERVO_BODY_W / 2.0 + 0.3
    boot_x_max = (+hp.SERVO_BODY_W / 2.0
                  + hp.WIRE_BOOT_PROTRUSION - 0.3)
    boot_y_half = hp.WIRE_BOOT_W / 2.0 - 0.3
    boot_z_min = hp.WIRE_BOOT_Z_BASE + 0.3
    boot_z_max = (hp.WIRE_BOOT_Z_BASE
                  + hp.WIRE_BOOT_H - 0.3)
    bx = np.linspace(boot_x_min, boot_x_max, 4)
    by = np.linspace(-boot_y_half, +boot_y_half, 3)
    bz = np.linspace(boot_z_min, boot_z_max, 3)
    Xx, Yy, Zz = np.meshgrid(bx, by, bz, indexing="ij")
    boot_pts = np.stack([Xx.ravel(), Yy.ravel(), Zz.ravel()], axis=1)

    return lateral_pts, downward_pts, boot_pts


def _probe_wire_corridor(part, name, R, t,
                          lateral_well, downward_well, boot_well):
    """Sweep the LATERAL and DOWNWARD wire-bundle corridors AND the
    +X wire-exit boot footprint through one cradle and report PASS iff:

      * the boot footprint is FULLY clear of part material (the body
        cannot seat unless the boot has a pocket to live in), AND
      * at least ONE of the lateral / downward corridors is fully
        clear so the harness can exit the cradle.
    """

    def _to_cradle(pts_well):
        if R is None:
            return pts_well + t
        return pts_well @ R[:3, :3].T + t

    lat_cradle  = _to_cradle(lateral_well)
    down_cradle = _to_cradle(downward_well)
    boot_cradle = _to_cradle(boot_well)

    lat_inside  = points_inside(part, lat_cradle)
    down_inside = points_inside(part, down_cradle)
    boot_inside = points_inside(part, boot_cradle)
    n_lat_blocked,  n_lat_total  = int(lat_inside.sum()),  len(lat_cradle)
    n_down_blocked, n_down_total = int(down_inside.sum()), len(down_cradle)
    n_boot_blocked, n_boot_total = int(boot_inside.sum()), len(boot_cradle)

    lat_ok  = n_lat_blocked  == 0
    down_ok = n_down_blocked == 0
    boot_ok = n_boot_blocked == 0
    # The boot MUST fit (no choice).  The wire needs ONE escape route.
    ok = boot_ok and (lat_ok or down_ok)

    detail = (
        f"boot {n_boot_total - n_boot_blocked}/{n_boot_total} clear "
        f"({'OK' if boot_ok else 'BLOCKED'}); "
        f"lateral {n_lat_total - n_lat_blocked}/{n_lat_total} clear "
        f"({'OK' if lat_ok else 'BLOCKED'}), "
        f"downward {n_down_total - n_down_blocked}/{n_down_total} clear "
        f"({'OK' if down_ok else 'BLOCKED'})"
    )
    return _label(name, ok, detail)


def check_wire_slot():
    """FULLY RETIRED (Aug 16 2026) -- no cradle carries a wire-exit
    L-corridor any more, so there is nothing left to probe.

    Retirement history (all the same stale-DS3225 story: the DS3225
    exited a 3-wire harness from a molded +X boot at the body's
    bottom-outboard corner; the STS3215 has NO boot -- its half-duplex
    bus leaves via BACK-face 5264 ports through each sandwich's open
    clamp face):

      * yaw / chassis_bottom (Jun 2026 front-face mount): the yaw
        harness routes through an inboard wire window verified
        end-to-end by ``check_leg_harness_drop`` ([3c]).
      * femur_link knee cradle (Aug 2026 flatten pass, user: "this top
        can just be a flat wall with no weird cutouts"):
        ``_sandwich_fixed_side(wire_exit=False)``.
      * coxa_link hip cradle -- the LAST holdout -- (Aug 16 2026, user:
        "a weird cutout channel on the opposite side ... pointless
        now"): the hip cradle now also builds with ``wire_exit=False``,
        so the corridor cut no longer exists anywhere.

    Kept as a stub (rather than deleted) so the CHECKS table, docs and
    historical logs keep a stable name; unconditionally passes."""
    print("\n[3b] Wire-exit L-corridor: RETIRED (Aug 16 2026 -- no cradle "
          "cuts a corridor; STS3215 harnesses leave via back-face ports)")
    return True


def check_leg_harness_drop():
    """Verify the per-leg cable drop slot through ``chassis_bottom``
    is unobstructed at every leg position.

    Part B (May 2026 wire-management pass) cut a small radial
    ``LEG_HARNESS_DROP_X_EXTENT`` x ``LEG_HARNESS_DROP_Y_EXTENT`` mm
    slot through ``chassis_bottom`` at each of the 6 leg positions,
    just inboard of each ``coxa_bracket``'s body cutout, so each
    leg's 3-cable harness can drop from the cradle wire-exit side
    of the plate straight into the inter-plate volume.  See the
    ``LEG_HARNESS_DROP_*`` constants block in ``hexapod_prototype.py``
    for the geometry rationale.

    This check probes a 3D corridor through the slot (the slot's XY
    span, plus the plate's full Z extent + 0.5 mm overshoot above
    and below) at each leg's bracket-local position, transforms it
    into chassis frame via the same ``_leg_chassis_frames()`` iterator
    that the chassis_bottom builder uses, and asserts EVERY probe
    point is OUTSIDE the chassis_bottom mesh (no plate material
    intrudes into the cable corridor).  Sibling to ``check_wire_slot``;
    listed in the standard ``CHECKS`` table.
    """
    print("\n[3c] Per-leg chassis_bottom harness drop slot:")
    cb = _load_mesh("chassis_bottom", copy=False)

    # Sample the slot's interior on a tight 3D grid.  X / Y stay 0.3 mm
    # inside the slot's nominal walls so a probe that lands ON the
    # slot edge isn't a false fail.  Z stays inside the chassis_bottom
    # plate's nominal Z extent ([-CHASSIS_PLATE_T/2, +CHASSIS_PLATE_T/2])
    # by a small XY-mode margin -- the check is "is the slot a clean
    # hole THROUGH the plate?", not "is everything below the plate
    # clear?".  (Historical note: an earlier revision printed a
    # vertical anchor tab hanging DOWN from the plate's -Z face just
    # outside the slot's Y span -- see the CABLE_ANCHOR_TAB_* constants
    # block in hexapod_prototype.py.  The tab was retired in May 2026
    # for printability; the harness now anchors through the drop slot
    # itself.  The Z margin is kept conservative to mirror the original
    # check's intent.)
    margin = 0.3
    half_z = hp.CHASSIS_PLATE_T / 2.0 - margin
    bz_local = np.linspace(-half_z, +half_z, 4)

    # One probe grid per slot in the shared single-source-of-truth list
    # (primary drop slot + tangential flanking ports); each grid stays 0.3 mm
    # inside its own nominal walls so an on-edge probe is not a false fail.
    slots = hp.leg_harness_drop_slots()
    slot_grids = []
    for (sx, sy, sxe, sye) in slots:
        half_x = sxe / 2.0 - margin
        half_y = sye / 2.0 - margin
        bx = np.linspace(-half_x, +half_x, 5) + sx
        by = np.linspace(-half_y, +half_y, 3) + sy
        BX, BY, BZ = np.meshgrid(bx, by, bz_local, indexing="ij")
        slot_grids.append(np.stack([BX.ravel(), BY.ravel(), BZ.ravel()], axis=1))

    all_ok = True
    for i, edge_mid, R, R3 in hp._leg_chassis_frames():
        for (sx, sy, sxe, sye), pts_bracket in zip(slots, slot_grids):
            # Transform bracket-frame probe points into chassis frame for
            # this leg: chassis = edge_mid + R3 @ bracket.
            pts_chassis = pts_bracket @ R3.T + edge_mid

            # Standoff SEAT-PAD exemption (Aug 16 2026): each of the 4
            # brass-standoff sites sits 15 deg off a leg axis, and the
            # Aug 2026 inboard port shift clipped its seat annulus -- the
            # builder now unions a Phi CHASSIS_STANDOFF_SEAT_PAD_OD pad
            # (full stack height) at every site, which deliberately bites
            # one corner of the adjacent port.  Probe points inside a pad
            # footprint (+0.3 mm skin margin) are INTENDED plate material,
            # not an obstruction, so drop them from the corridor test.
            pad_r = hp.CHASSIS_STANDOFF_SEAT_PAD_OD / 2.0 + 0.3
            in_pad = np.zeros(len(pts_chassis), dtype=bool)
            for (px, py) in hp.CHASSIS_STANDOFF_HOLES_XY:
                in_pad |= (np.hypot(pts_chassis[:, 0] - px,
                                    pts_chassis[:, 1] - py) <= pad_r)
            pts_chassis = pts_chassis[~in_pad]

            inside = points_inside(cb, pts_chassis)
            n_blocked = int(inside.sum())
            n_total = len(pts_chassis)
            ok = n_blocked == 0
            all_ok &= _label(
                f"leg_harness_drop_L{i} slot(x{sx:+.0f},y{sy:+.0f},"
                f"{sxe:.0f}x{sye:.0f})",
                ok,
                f"{n_total - n_blocked}/{n_total} probe points clear "
                f"({'OK' if ok else f'{n_blocked} BLOCKED'})",
            )
    return all_ok


# ---------------------------------------------------------------------------
# 4.  Self-collision of the standing-pose assembly
# ---------------------------------------------------------------------------

def _build_standing_leg():
    """Return (parts dict, name list) of the leg's printed parts placed
    in their standing-pose locations in the body's world frame, on the
    edge between hexagon vertex 0 and vertex 1 (apothem direction
    a = pi/6)."""
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = 0.5 * np.pi / 3
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    z_hat = np.array([0.0, 0.0, 1.0])

    # Design B (May 2026): yaw output stack collapsed from
    #   PLASTIC_HORN_H + HORN_ADAPTER_T = 9 mm
    # down to
    #   HORN_STACK_H = PLASTIC_HORN_H = 5 mm
    # now that the printed servo_horn_adapter has been retired.
    yaw_output_z = hp.CHASSIS_YAW_OUTPUT_Z
    hip_drop = hp.COXA_HIP_DROP
    hip_joint_local = np.array(hp.COXA_HIP_ANCHOR)
    # Jun 2026 bearing-sandwich refit: the moving link's local origin is
    # now the disc-horn-TOP (mating face) sitting ON the joint axis (the
    # yoke top arm bolts to the horn there), so the link origin coincides
    # with the joint-axis point -- no axial pad offset.  See
    # hexapod_prototype.make_femur_link / _leg_in_body_frame.
    PAD_AXIS_OFFSET = np.array([0.0, 0.0, 0.0])

    p = np.deg2rad(hp.STANCE_FEMUR_DEG)
    pt = np.deg2rad(hp.STANCE_FEMUR_DEG + hp.STANCE_TIBIA_DEG)
    Ry_p = rotation_matrix(p, [0, 1, 0])[:3, :3]
    knee_joint_local = hip_joint_local + Ry_p @ np.array([hp.FEMUR_LENGTH, 0, 0])

    parts = {}

    cl = _load_mesh("coxa_link")
    cl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    cl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["coxa_link"] = cl

    fl = _load_mesh("femur_link")
    fl.apply_transform(rotation_matrix(p, [0, 1, 0]))
    fl.apply_translation(hip_joint_local + PAD_AXIS_OFFSET)
    fl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    fl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["femur_link"] = fl

    tl = _load_mesh("tibia_link")
    tl.apply_transform(rotation_matrix(pt, [0, 1, 0]))
    tl.apply_translation(knee_joint_local + PAD_AXIS_OFFSET)
    tl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    tl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["tibia_link"] = tl

    # Sandwich-joint clamp caps clamp the hip + knee servos into their
    # cradles (closing the OPEN +Y cradle face).  Placed in the SAME
    # world frame via _place_servo_clamp_caps.
    caps = _place_servo_clamp_caps()
    parts["hip_clamp_cap"] = caps["hip_clamp_cap"]
    parts["knee_clamp_cap"] = caps["knee_clamp_cap"]

    return parts


def _pair_overlap_volume(a_mesh, b_mesh, pitch=1.5):
    """Estimate the overlap volume between mesh A and mesh B by voxel
    sampling A inside its AABB intersection with B, then counting how
    many of those voxel centres fall inside both meshes.

    The sample grid is INSET slightly from the AABB-intersection faces:
    ``linspace`` endpoints otherwise land EXACTLY on a designed flush-
    contact plane (e.g. the hip servo bottom resting on the coxa foot
    plate), where ``points_inside`` reads the shared boundary as inside
    BOTH bodies and inflates a zero-penetration rest plane into a full
    voxel slab of phantom "overlap" (knife-edge behaviour: the Aug 2026
    6805 bearing swap moved that plane 4 mm and flipped a stable PASS
    into a 2136 mm^3 FAIL with a true boolean intersection of 0 mm^3).
    Real interpenetrations are >= a few tenths of a mm deep, far beyond
    the 0.05 mm inset, so detection is unaffected."""
    a_min, a_max = a_mesh.bounds
    b_min, b_max = b_mesh.bounds
    eps = 0.05  # mm -- keep samples off exact coplanar contact faces
    lo = np.maximum(a_min, b_min) + eps
    hi = np.minimum(a_max, b_max) - eps
    if np.any(hi <= lo):
        return 0.0
    n = np.maximum(2, np.ceil((hi - lo) / pitch).astype(int))
    gx = np.linspace(lo[0], hi[0], int(n[0]))
    gy = np.linspace(lo[1], hi[1], int(n[1]))
    gz = np.linspace(lo[2], hi[2], int(n[2]))
    XX, YY, ZZ = np.meshgrid(gx, gy, gz, indexing="ij")
    pts = np.stack([XX.ravel(), YY.ravel(), ZZ.ravel()], axis=1)
    in_a = points_inside(a_mesh, pts)
    if in_a.sum() == 0:
        return 0.0
    pts_a = pts[in_a]
    in_b = points_inside(b_mesh, pts_a)
    n_overlap = int(in_b.sum())
    voxel_vol = pitch ** 3
    return n_overlap * voxel_vol


def check_self_collision():
    print("\n[4] Self-collision in standing pose (one leg):")
    parts = _build_standing_leg()
    names = list(parts.keys())

    # Connected pairs across rotational joints share a tiny shaft / horn
    # interface that legitimately overlaps geometrically.  Allow a
    # generous tolerance there; for non-adjacent parts require zero.
    JOINT_PAIRS = {
        ("coxa_link",    "femur_link"),
        ("femur_link",   "tibia_link"),
    }
    # Sandwich-joint clamp caps (Jun 2026) clamshell the hip/knee servo
    # against its cradle, so each cap legitimately interpenetrates the
    # cradle's printed walls (the clamp halves share boundary material
    # by design) AND brushes the moving link that drives off that joint.
    # These mating overlaps are intrinsic to the clamp, so they get the
    # generous CLAMP_TOLERANCE rather than the zero-overlap NONADJ rule.
    CLAMP_PAIRS = {
        ("coxa_link",  "hip_clamp_cap"),
        ("femur_link", "knee_clamp_cap"),
        ("tibia_link", "knee_clamp_cap"),
    }
    CLAMP_TOLERANCE = 1800.0
    # Adjacent printed parts at a rotary joint share NO printed
    # material -- the actual physical interface (disc horn + servo
    # gear stack) lives in dedicated horn-stack volume
    # which is checked separately by check_horn_stack_clearance.  So
    # we expect ZERO printed-vs-printed overlap here, modulo the
    # voxel-stair-step artefact along sharp mesh boundaries.  This
    # used to be 1500 mm^3 ("swallow the gear stack") which masked a
    # real ~ 800 mm^3 femur-spar-vs-coxa-link-arm clash that the user
    # reported as 'femur link cant rotate'.
    JOINT_TOLERANCE = 100.0
    NONADJ_TOLERANCE = 100.0

    all_ok = True
    for i, na in enumerate(names):
        for j, nb in enumerate(names):
            if j <= i:
                continue
            vol = _pair_overlap_volume(parts[na], parts[nb])
            adj = (na, nb) in JOINT_PAIRS or (nb, na) in JOINT_PAIRS
            clamp = (na, nb) in CLAMP_PAIRS or (nb, na) in CLAMP_PAIRS
            if clamp:
                tol, kind = CLAMP_TOLERANCE, "clamp"
            elif adj:
                tol, kind = JOINT_TOLERANCE, "joint"
            else:
                tol, kind = NONADJ_TOLERANCE, "non-adj"
            ok = vol <= tol
            all_ok &= _label(f"{na} vs {nb} ({kind})",
                               ok,
                               f"overlap = {vol:7.1f} mm^3 (tol {tol:.0f})")
    return all_ok


# ---------------------------------------------------------------------------
# 4a.  Printed-part pairwise INTERPENETRATION (exact boolean intersection)
# ---------------------------------------------------------------------------
#
# ``check_self_collision`` (above) is a VOXEL-sampled overlap test whose
# clamp-cap <-> link pairs carry a deliberately GENEROUS tolerance
# (CLAMP_TOLERANCE = 1800 mm^3) to absorb the voxel stair-step along the
# many flush faces where the clamp cap mates its cradle.  That tolerance
# is large enough to MASK a real printed-part interference: the clamp
# cap's top retaining lip used to overhang -Y to ``body_face_y - 4`` mm
# and bury ~560 mm^3 INSIDE the cradle's own top retaining plate (two
# solid printed parts occupying the same space -- impossible to assemble).
#
# This guard is the authoritative, voxel-free interference test for the
# printed parts that are MEANT to mate flush (share only a 0-volume
# contact face), computed with the EXACT manifold boolean intersection so
# the tolerance can be tight.  It covers the sandwich-joint clamp cap vs
# its bracket/cradle for BOTH driven joints (hip + knee), in the exact
# relative pose the assembly uses (the cap shares the bracket's
# ``_joint_place`` transform -- see ``_place_servo_clamp_caps``).

# A correctly-mating clamp cap touches its cradle only on flush faces
# (0 volume).  Allow a small budget for boolean facet noise on those
# coincident faces; the real defect this guards against is ~560 mm^3.
CLAMP_CAP_INTERFERENCE_TOL_MM3 = 20.0


def _boolean_overlap_volume(a, b):
    """Exact overlap volume via the manifold boolean intersection.

    Returns ``(volume_mm3, ok_bool)``.  Falls back to the robust voxel
    estimator (``_pair_overlap_volume``) if the boolean kernel errors out
    so a kernel hiccup can't silently pass a real clash."""
    try:
        inter = trimesh.boolean.intersection([a, b])
    except Exception:
        return _pair_overlap_volume(a, b, pitch=1.0), True
    if inter is None or len(getattr(inter, "faces", [])) == 0:
        return 0.0, True
    return abs(float(inter.volume)), True


def check_clamp_cap_interference():
    """The sandwich-joint clamp cap must mate its bracket/cradle FLUSH --
    no printed-vs-printed interpenetration.  Built from hexapod_prototype
    directly (like ``check_clamp_cap_alignment``) and posed exactly as the
    assembly does, then intersected with the EXACT boolean kernel."""
    print("\n[4a] Clamp-cap <-> bracket printed-part interpenetration:")

    cases = []
    # HIP: the hip cradle is part of the ONE-PIECE coxa_link (Aug 2026
    # merge); the builder already applies the hip _joint_place to its fixed
    # side, so the cap must get the SAME transform.
    hip_bracket = hp.make_coxa_link_part()
    hip_cap = hp.make_servo_clamp_cap()
    hip_cap.apply_transform(
        hp._joint_place(hp.COXA_HIP_ANCHOR, (1, 0, 0), hp.LEG_PITCH_AXIS))
    cases.append(("hip  coxa_link hip cradle vs hip_clamp_cap",
                  hip_bracket, hip_cap))

    # KNEE: the knee cradle is the femur_link's knee fixed side
    # (_femur_knee_fixed_solid, joint-local); the knee cap shares the
    # identical knee _joint_place, so their RELATIVE pose is identity.
    knee_bracket = hp._femur_knee_fixed_solid()
    knee_cap = hp.make_servo_clamp_cap()
    cases.append(("knee femur_link knee cradle vs knee_clamp_cap",
                  knee_bracket, knee_cap))

    all_ok = True
    tol = CLAMP_CAP_INTERFERENCE_TOL_MM3
    for name, bracket, cap in cases:
        vol, _ = _boolean_overlap_volume(bracket, cap)
        ok = vol <= tol
        all_ok &= _label(name, ok,
                         f"interpenetration = {vol:7.1f} mm^3 (tol {tol:.0f})")
    return all_ok


# ---------------------------------------------------------------------------
# 4b.  COMPREHENSIVE all-pairs printed-part interference over the FULL robot
# ---------------------------------------------------------------------------
#
# This is the AUTHORITATIVE, comprehensive replacement for the leg-only
# allow-list-of-pairs collision tests (``check_self_collision`` covers only
# {coxa,femur,tibia,clamp caps} on ONE leg; ``check_clamp_cap_interference``
# covers only 2 hard-coded clamp pairs).  Whole classes of printed-vs-printed
# interpenetration -- cap<->chassis, hub<->chassis, tray/dome<->chassis, the
# chassis split halves, the femur hip-yoke<->knee-bracket tube sockets, the
# foot tang<->pad fork -- were structurally UNCHECKED by those, and each was
# papered over until it bit during assembly.
#
# This gate assembles the ENTIRE static robot from EVERY printed part, in its
# nominal assembled pose, DECOMPOSED into the real printed parts (the
# ONE-PIECE coxa_link and femur_link, tibia_knee_yoke +
# TPU foot_boot, the two chassis halves, both clamp caps, both
# decks, the dome) -- not merged link proxies -- using the SAME
# authoritative scene placement that ``tools/full_robot_viz_build.py`` lays
# the buildviz scene out with (so poses are never re-invented and stay locked
# to the disc-horn / bearing world frame the rest of the verifier uses).  It
# then computes the EXACT boolean overlap (voxel fallback on kernel error) for
# every AABB-overlapping pair and requires it to be <= a tiny facet-noise
# budget, EXCEPT for an explicit, NAMED allow-list of DESIGNED flush / slip-fit
# mates (parts that share a coincident 0-volume contact face), each with a
# tight justified budget.
#
# NEW-PART CONTRACT: this check is gated on ALL_PRINTED_PARTS, so any printed
# part added in future is automatically pulled into the matrix.  A new part
# that genuinely interpenetrates another printed part with NO allow-list entry
# will FAIL the build.  Do NOT widen a budget or add an allow-list entry to
# silence a REAL volumetric clash -- the allow-list is ONLY for true flush /
# slip-fit mates whose real solid overlap is ~0 (facet noise).  Fix the
# geometry instead.

# Facet/voxel noise floor permitted for ANY printed-part pair (two manifold
# meshes that merely TOUCH still emit a few stray boolean facets).
ASSEMBLY_INTERFERENCE_NOISE_MM3 = 5.0

# Printed parts assembled by the gate (COTS -- bearings, CF tubes, servos,
# disc horns, lipo, PCBs -- are excluded; they are not printed).
_ASSEMBLY_PRINTED_PARTS = frozenset({
    "coxa_link", "yaw_bearing_cap",
    "femur_link",
    "tibia_knee_yoke", "foot_boot",
    "hip_clamp_cap", "knee_clamp_cap",
    "chassis_bottom", "chassis_top",
    
})

# NAMED allow-list of DESIGNED flush / slip-fit mates.  Each entry is a pair of
# printed parts that SHARE a coincident contact face (real solid overlap ~0);
# the budget is a tight facet-noise allowance, never a cover for a real clash.
ASSEMBLY_INTERFERENCE_ALLOW = {
    # Yaw bearing tower split: cap mates the chassis bottom tower at a flush
    # split face (YAW_SPLIT_Z); the 3 join bolts/ear bosses are coaxial.
    frozenset({"yaw_bearing_cap", "chassis_bottom"}):
        (25.0, "yaw tower split: flush split face (coaxial bolted join)"),
    # Yaw turntable hub (now the hub end of the ONE-PIECE coxa_link, Aug 2026
    # merge) seats in the chassis tower bore on the 6706 bearing stack -- a
    # running slip fit (coaxial), only facet noise at the seat.
    frozenset({"coxa_link", "chassis_bottom"}):
        (15.0, "yaw bearing seat: hub boss is a running slip fit in the tower"),
    # Hub rides the cap-captured upper 6706 inner race; the rotating hub has
    # running clearance to the stationary cap (flush at the race face only).
    frozenset({"coxa_link", "yaw_bearing_cap"}):
        (15.0, "yaw joint: hub runs against the cap-captured race, flush face"),
    # Sandwich-joint clamp caps clamshell their cradle bracket on flush faces
    # (the 0.2 mm tongue interference is against the COTS servo body, not the
    # printed bracket); guarded tightly by check_clamp_cap_interference too.
    frozenset({"coxa_link", "hip_clamp_cap"}):
        (25.0, "hip clamp cap clamshells the coxa_link hip cradle on flush faces"),
    # Jul 2026 merge #2: the knee cradle is part of the ONE-PIECE femur_link,
    # so the knee clamp cap clamshells femur_link directly (flush faces, same
    # mate the separate femur_knee_bracket had).
    frozenset({"femur_link", "knee_clamp_cap"}):
        (25.0, "knee clamp cap clamshells the femur_link knee cradle on flush faces"),
}


def _aabb_overlap(a, b, margin: float = 0.0) -> bool:
    a_lo, a_hi = a.bounds
    b_lo, b_hi = b.bounds
    return not (bool(np.any(a_hi < b_lo - margin))
                or bool(np.any(b_hi < a_lo - margin)))


def _assembly_interference_parts():
    """Return ``(core, neighbour)`` lists of ``(label, basename, mesh)`` for
    every PRINTED part of the full static assembly, built LIVE from
    hexapod_prototype and placed with full_robot_viz_build's authoritative
    scene transforms.

    ``core``      = leg-0 printed parts + the entire chassis / yaw / deck /
                    dome body (placed once).
    ``neighbour`` = leg-0 printed parts rotated to the adjacent +60 deg hex
                    edge, so leg<->leg adjacency is covered.  The chassis is
                    6-fold symmetric, so leg-0 + ONE neighbour is fully
                    representative of all six leg-chassis interfaces and all
                    six adjacent leg-leg pairs.
    """
    import sys as _sys
    import pathlib as _pathlib
    _tools = str(_pathlib.Path(__file__).resolve().parent / "tools")
    if _tools not in _sys.path:
        _sys.path.insert(0, _tools)
    import full_robot_viz_build as FRB  # noqa: WPS433

    core: list[tuple[str, str, object]] = []
    leg0: list[tuple[str, object]] = []
    for name, mesh in FRB._leg0_parts():
        if name in _ASSEMBLY_PRINTED_PARTS:
            core.append((name, name, mesh))
            leg0.append((name, mesh))
    for name, mesh in FRB._body_parts(0.0):
        if name in _ASSEMBLY_PRINTED_PARTS:
            core.append((name, name, mesh))

    R = rotation_matrix(np.pi / 3.0, [0, 0, 1])
    neighbour: list[tuple[str, str, object]] = []
    for name, mesh in leg0:
        m = mesh.copy()
        m.apply_transform(R)
        neighbour.append((f"{name}@leg+1", name, m))
    return core, neighbour


def check_assembly_interference():
    """COMPREHENSIVE all-pairs printed-part interference gate over the full
    static assembly (see the design note above).  Assembles EVERY printed part
    in its nominal pose (decomposed into the real printed parts, not the merged
    link proxies), then for every AABB-overlapping pair computes the EXACT
    boolean overlap and requires it <= a tiny facet-noise budget, except for a
    small NAMED allow-list of designed flush / slip-fit mates.

    This REPLACES the leg-only collision allow-list: new printed parts are
    pulled in automatically (gated on ALL_PRINTED_PARTS).  A new part that
    interpenetrates another with no allow-list entry FAILS the build -- fix the
    geometry, do not allow-list a real clash."""
    print("\n[4b] Assembly interference (all printed parts, full robot):")

    from itertools import combinations, product  # noqa: WPS433

    core, neighbour = _assembly_interference_parts()
    print(f"       assembled {len(core)} core printed parts "
          f"(leg 0 + chassis/yaw/deck/dome) + {len(neighbour)} neighbour-leg "
          f"parts")

    stats = {"ok": True, "checked": 0, "reported": 0}

    def _judge(la, ba, ma, lb, bb, mb):
        if not _aabb_overlap(ma, mb):
            return
        stats["checked"] += 1
        key = frozenset({ba, bb})
        allowed = key in ASSEMBLY_INTERFERENCE_ALLOW
        vol, _ = _boolean_overlap_volume(ma, mb)
        # Sub-noise & not a named mate -> a legitimately-just-touching pair;
        # clear it silently so the log isn't flooded with dozens of neighbours.
        if vol <= ASSEMBLY_INTERFERENCE_NOISE_MM3 and not allowed:
            return
        if allowed:
            budget, why = ASSEMBLY_INTERFERENCE_ALLOW[key]
            tag = f"flush mate, tol {budget:.0f}: {why}"
        else:
            budget = ASSEMBLY_INTERFERENCE_NOISE_MM3
            tag = (f"REAL interpenetration (tol {budget:.0f}) -- fix geometry, "
                   f"do NOT allow-list")
        stats["reported"] += 1
        ok = vol <= budget
        stats["ok"] &= _label(f"{la} vs {lb}", ok,
                              f"overlap = {vol:7.1f} mm^3 ({tag})")

    # Core all-pairs: leg 0 printed parts + the whole chassis/yaw/deck/dome.
    for a, b in combinations(core, 2):
        _judge(*a, *b)
    # Leg-leg adjacency: leg-0 LEG parts vs the +60 deg neighbour leg's parts
    # (chassis is shared, so skip body parts here -- already in core).
    leg0_legparts = [p for p in core if p[1] not in _CHASSIS_PARTS]
    for a, b in product(leg0_legparts, neighbour):
        _judge(*a, *b)

    print(f"       {stats['checked']} AABB-overlapping pairs examined; "
          f"{stats['reported']} above the "
          f"{ASSEMBLY_INTERFERENCE_NOISE_MM3:.0f} mm^3 noise floor")
    if stats["reported"] == 0:
        _label("no printed-part interpenetration above the noise floor",
               True, "")
    return stats["ok"]


# ---------------------------------------------------------------------------
# 5.  Servo-body clearance in standing pose
# ---------------------------------------------------------------------------

def _place_servo_bodies():
    """Return a dict of the three servo envelopes for one leg in the same
    world frame used by ``_build_standing_leg`` (apothem direction a=pi/6).

    Mirrors the placement math from ``build_prototype_assembly._build_leg``
    so the meshes land in their physically-correct cradle positions for the
    standing-pose check.
    """
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = 0.5 * np.pi / 3
    cos_a, sin_a = np.cos(a), np.sin(a)
    edge_mid = np.array([apothem * cos_a, apothem * sin_a, 0.0])
    z_hat = np.array([0.0, 0.0, 1.0])

    # Design B (May 2026): yaw output stack collapsed from
    #   PLASTIC_HORN_H + HORN_ADAPTER_T = 9 mm
    # down to
    #   HORN_STACK_H = PLASTIC_HORN_H = 5 mm
    # now that the printed servo_horn_adapter has been retired.
    yaw_output_z = hp.CHASSIS_YAW_OUTPUT_Z
    yaw_output_world = edge_mid + yaw_output_z * z_hat

    hip_drop = hp.COXA_HIP_DROP
    hip_joint_local = np.array(hp.COXA_HIP_ANCHOR)

    p = np.deg2rad(hp.STANCE_FEMUR_DEG)
    Ry_p = rotation_matrix(p, [0, 1, 0])[:3, :3]
    R_a = rotation_matrix(a, [0, 0, 1])

    # ----- Yaw servo: body hangs in the coxa-bracket well. -----
    # In bracket-local: body-bottom-centre at (-SERVO_OUTPUT_X, 0,
    # -WELL_RIM_Z), mesh +X aligned with bracket +X (radial outward).
    yaw = _load_mesh("servo_body")
    # Land the servo's OUTPUT (front) face at the disc-horn underside,
    # i.e. CHASSIS_YAW_OUTPUT_Z - HORN_STACK_H, so its output hub top
    # reaches CHASSIS_YAW_OUTPUT_Z where the coxa_link pad bolts on.
    # (The servo_body mesh has its front face at local z = WELL_RIM_Z.)
    # The legacy '-WELL_RIM_Z' shift predated the STS3215 refit where
    # WELL_RIM_Z == SERVO_BODY_H; it sank the hub to z~5 and left the
    # coxa hovering ~24 mm above the servo in the assembly view.
    yaw.apply_translation(
        [-hp.SERVO_OUTPUT_X, 0.0,
         hp.CHASSIS_YAW_OUTPUT_Z - hp.HORN_STACK_H - hp.WELL_RIM_Z])
    yaw.apply_transform(R_a)
    yaw.apply_translation(edge_mid)

    # ----- Hip-pitch servo: front-face mounted in the coxa's hip FIXED
    # side (bearing-sandwich refit, Jun 2026).  The fixed side is placed
    # by make_coxa_link at hp._joint_place((COXA_LENGTH, 0, COXA_HIP_DROP),
    # x=+X, z=+Y); the servo body shares that bracket-local frame, so we
    # apply the IDENTICAL transform to the servo envelope, then the coxa's
    # world placement (R_a + yaw-output translate).  Output axis -> +Y
    # (the hip-pitch axis), not the legacy downward well. -----
    M_hip = hp._joint_place(hp.COXA_HIP_ANCHOR, (1, 0, 0),
                            hp.LEG_PITCH_AXIS)
    hip = _load_mesh("servo_body")
    hip.apply_transform(M_hip)
    hip.apply_transform(R_a)
    hip.apply_translation(yaw_output_world)

    # ----- Knee servo: front-face mounted in the femur's knee FIXED side
    # (the femur_link's knee cradle).  That cradle is placed in femur-local by
    # hp._joint_place((FEMUR_LENGTH, -HORN_STACK_H, 0), x=+X, z=+Y); the
    # femur itself is then carried into the world by the same chain the
    # femur_link uses (pitch about Y, translate to hip axis + pad offset,
    # yaw about Z, yaw-output translate). -----
    M_knee = hp._joint_place((hp.FEMUR_LENGTH, 0.0, 0.0),
                             (1, 0, 0), hp.LEG_PITCH_AXIS)
    knee = _load_mesh("servo_body")
    knee.apply_transform(M_knee)
    knee.apply_transform(rotation_matrix(p, [0, 1, 0]))
    knee.apply_translation(hip_joint_local)
    knee.apply_transform(R_a)
    knee.apply_translation(yaw_output_world)

    return {"yaw_servo": yaw, "hip_servo": hip, "knee_servo": knee}


def _place_servo_clamp_caps():
    """Return the hip + knee sandwich-joint clamp caps for one leg in the
    SAME world frame as ``_place_servo_bodies``.

    The clamshell cap (``hp.make_servo_clamp_cap``) is modelled in the servo
    well-local frame (origin = body back face, +X long, +Y depth, +Z
    output), which is identical to the ``servo_body`` envelope frame -- so
    each cap uses the IDENTICAL transform chain as its servo body."""
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = 0.5 * np.pi / 3
    cos_a, sin_a = np.cos(a), np.sin(a)
    edge_mid = np.array([apothem * cos_a, apothem * sin_a, 0.0])
    z_hat = np.array([0.0, 0.0, 1.0])
    yaw_output_world = edge_mid + hp.CHASSIS_YAW_OUTPUT_Z * z_hat
    hip_drop = hp.COXA_HIP_DROP
    hip_joint_local = np.array(hp.COXA_HIP_ANCHOR)
    p = np.deg2rad(hp.STANCE_FEMUR_DEG)
    R_a = rotation_matrix(a, [0, 0, 1])

    M_hip = hp._joint_place(hp.COXA_HIP_ANCHOR, (1, 0, 0),
                            hp.LEG_PITCH_AXIS)
    hip_cap = hp.make_servo_clamp_cap()
    hip_cap.apply_transform(M_hip)
    hip_cap.apply_transform(R_a)
    hip_cap.apply_translation(yaw_output_world)

    M_knee = hp._joint_place((hp.FEMUR_LENGTH, 0.0, 0.0), (1, 0, 0),
                             hp.LEG_PITCH_AXIS)
    knee_cap = hp.make_servo_clamp_cap()
    knee_cap.apply_transform(M_knee)
    knee_cap.apply_transform(rotation_matrix(p, [0, 1, 0]))
    knee_cap.apply_translation(hip_joint_local)
    knee_cap.apply_transform(R_a)
    knee_cap.apply_translation(yaw_output_world)
    return {"hip_clamp_cap": hip_cap, "knee_clamp_cap": knee_cap}


def _place_yaw_retainers():
    """Return the yaw-servo capture stirrup for one leg in the leg-0 world
    frame (apothem dir a=pi/6), positioned coaxially under the yaw servo --
    the SAME Ra + edge_mid placement ``_place_servo_bodies`` uses for the yaw
    servo.  The strap is modelled in cradle-local XY + world Z, so only the
    in-plane yaw rotation + edge-midpoint translate are applied (no Z shift)."""
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = 0.5 * np.pi / 3
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    strap = hp.make_yaw_servo_retainer()
    strap.apply_transform(rotation_matrix(a, [0, 0, 1]))
    strap.apply_translation(edge_mid)
    return {"yaw_servo_retainer": strap}


def check_servo_clearance():
    """Each servo body should sit inside its cradle's air space, with at most
    a tiny overlap from the gear stack / spline / horn pocket that punches
    through the cradle face.  Non-adjacent printed parts (further along the
    leg) should not be touched by the servo body at all."""
    print("\n[5] Servo-body clearance in standing pose:")

    parts = _build_standing_leg()
    servos = _place_servo_bodies()

    # A servo lives in exactly one cradle; the adjacent printed part on the
    # output side hosts the gear stack passage (a couple cm^3 of overlap is
    # expected through the spline bore).  Everything else should be ~zero.
    CRADLE_PARENT = {
        "yaw_servo":  "chassis_bottom",
        "hip_servo":  "coxa_link",
        "knee_servo": "femur_link",
    }
    OUTPUT_NEIGHBOUR = {
        "yaw_servo":  "coxa_link",
        "hip_servo":  "femur_link",
        "knee_servo": "tibia_link",
    }
    # The clamp cap that CLAMPS each servo into its sandwich cradle (hip/knee;
    # yaw uses the saddle, no cap).  Its centre tongue is INTENTIONALLY a
    # press fit -- it reaches CLAMP_TONGUE_INTERF = 0.5 mm PAST the seated body
    # +Y face so the 2 cap bolts trap the body with zero slop (user request,
    # Jun 2026; 1.0 -> 0.5 Aug 26 2026 -- the cap top bowed around the screws).
    # This is a designed interference, not a collision, so the
    # servo-vs-its-own-cap pair gets a dedicated press-fit budget.
    CLAMP_CAP = {
        "hip_servo":  "hip_clamp_cap",
        "knee_servo": "knee_clamp_cap",
    }
    CRADLE_TOLERANCE = 600.0    # mm^3 -- tab plane resting on the rim
    OUTPUT_TOLERANCE = 1500.0   # mm^3 -- horn passage / gear stack
    # Intended 0.5 mm tongue press onto the body +Y face: true boolean overlap
    # is ~770 mm^3 (45 x 34 x 0.5 contact), but the pitch-1.5 voxel sampler
    # over-reads sub-pitch overlaps by up to a few thousand mm^3 depending on
    # grid alignment with the joint's pose (at the old 1 mm press it read up to
    # ~4250 vs ~1530 true).  Budget 6000 covers the voxel reading with margin
    # while still failing loudly on a gross bury (a tongue/flange sinking
    # several mm into the body is >> 10000).
    CLAMP_PRESS_TOLERANCE = 6000.0
    # Servo body next to a thin printed flange registers ~100-250 mm^3 of
    # voxel-sampling noise even when the surfaces are physically apart.
    # The cradle-openness check (using analytic probing) is the source of
    # truth for "the servo body is not blocked"; this tolerance catches a
    # real mechanical conflict (>= ~500 mm^3) without flaring on noise.
    OTHER_TOLERANCE = 300.0

    all_ok = True
    for servo_name, servo in servos.items():
        for part_name, part in parts.items():
            vol = _pair_overlap_volume(servo, part, pitch=1.5)
            if part_name == CRADLE_PARENT[servo_name]:
                tol = CRADLE_TOLERANCE
                kind = "cradle"
            elif part_name == OUTPUT_NEIGHBOUR[servo_name]:
                tol = OUTPUT_TOLERANCE
                kind = "output"
            elif part_name == CLAMP_CAP.get(servo_name):
                tol = CLAMP_PRESS_TOLERANCE
                kind = "clamp press-fit"
            else:
                tol = OTHER_TOLERANCE
                kind = "non-adj"
            ok = vol <= tol
            all_ok &= _label(
                f"{servo_name} vs {part_name} ({kind})",
                ok,
                f"overlap = {vol:7.1f} mm^3 (tol {tol:.0f})",
            )
    return all_ok


# ---------------------------------------------------------------------------
# 5b.  Horn-stack clearance inside output-side mating cylinders
# ---------------------------------------------------------------------------
#
# Why this check exists
# ---------------------
# Every rotary joint (yaw, hip-pitch, knee-pitch) connects the servo's
# spline to the next printed link via this physical stack, laid out
# along the joint axis (+Y in each driven part's local frame):
#
#     servo body  |  output gear  |  disc horn (link bolts here)
#     y < 0       |  y in [-6, 0] |  y in [0, +5]  -> y = HORN_STACK_H
#                                  ^               ^
#                                spline tip       link mating face
#                                = joint axis
#
# Design B (May 2026) + June 2026 disc-horn switch: the printed
# servo_horn_adapter and the plastic X-horn are both retired.
# The link's pad now bolts DIRECTLY onto the disc horn (the 20 mm
# aluminum 25T disc that screws onto the spline), so the horn stack is
# the disc horn alone -- HORN_STACK_H = PLASTIC_HORN_H = 5 mm (was 9 mm
# when the printed adapter sat on top of the plastic horn).
#
# The driven printed part (coxa_link for yaw, femur_link for hip-pitch,
# tibia_link for knee-pitch) sits ABOVE the disc horn at femur/tibia
# y >= HORN_STACK_H = +5, with a 4-bolt clamp pad on DISC_HORN_BOLT_PCD =
# 14 mm.  The part's "neck" / flange ring -- the material between the
# pad's mating face (y = HORN_STACK_H) and the spar's +Y face
# (y = LINK_THICKNESS/2 = +3) -- MUST be free of plastic anywhere
# inside the conservative horn-sweep envelope (still sized to the
# now-retired plastic X-horn) plus a small clearance margin.
#
# May 2026 "shorten-neck" refactor: the link's flange-ring inner
# radius switched from ``HORN_ADAPTER_OD/2 + 0.5`` = 16.5 mm (sized
# for the now-retired printed servo_horn_adapter) to
# ``HORN_STACK_VOID_R`` = ``PLASTIC_HORN_X_TIP_R + 0.5`` = 18.5 mm
# (sized for the now-retired plastic X-horn's Phi 36 mm sweep -- the user
# found that the previous Phi 33 mm cup physically blocked the
# Phi 36 mm horn from fitting; the smaller disc horn keeps that envelope
# as headroom).  This probe radius tracks the same constant so the
# verifier follows the link geometry automatically.
#
# This is exactly the failure mode the user reported as "the femur link
# doesn't let the end of the servo stick out high enough to connect to
# the tibia link" -- the tibia knee-pad's neck cylinder was a solid
# Phi HIP_PAD_R*2 = 34 mm column that punched straight through the
# Phi 32 mm horn-adapter footprint (the retired printed adapter).
#
# Test geometry
# -------------
# We build a single horn-stack cylinder of radius
# HORN_STACK_VOID_R (= PLASTIC_HORN_X_TIP_R + HORN_STACK_CLEARANCE =
# 18.5 mm) and height HORN_STACK_H, centred on the joint axis at
# y in [0, HORN_STACK_H], and probe each driven printed part for
# material inside that volume.  The check is done in each part's
# LOCAL frame -- both the femur and tibia have their joint axis at
# the origin with +Y along the horn-stack direction, so a single
# cylinder template works for both.
#
# Tolerance: the budget catches the well-defined "solid column"
# failure (~ 500-2000 mm^3 of interpenetration) without flaring on
# the < 50 mm^3 of voxel-stair-step artefact along the pad's bottom
# rim where the neck-clearance void's CYLINDRICAL boundary meets the
# pad's CIRCULAR-DISC boundary on slightly mismatched voxel grids.

HORN_STACK_CLEARANCE = 0.5   # mm -- radial clearance around the (retired) X-horn envelope tips
HORN_STACK_OVERLAP_TOL = 50.0  # mm^3 -- voxel-grid artefact budget


def check_horn_stack_clearance():
    """Verify each driven printed part has a clear cylindrical void
    for the disc horn (probed against the conservative retired-X-horn
    envelope) at its joint."""
    # Probe radius matches the link's flange-ring inner radius
    # (HORN_STACK_VOID_R) so the verifier follows the geometry
    # automatically.  Equal to PLASTIC_HORN_X_TIP_R + 0.5 = 18.5 mm
    # as of the May 2026 shorten-neck refactor.
    R = hp.HORN_STACK_VOID_R
    # Jun 2026 flush-horn fix: the disc that must stay clear of the driven link
    # is only DISC_HORN_H (~2 mm) tall and sits at the BOTTOM of the stack (on
    # the servo output boss).  The frozen seat plane is still HORN_STACK_H above
    # the boss, and the link's printed reach-down boss LEGITIMATELY fills the
    # (HORN_STACK_H - DISC_HORN_H) gap above the disc, so only the real disc band
    # -- not the full 5 mm stack -- is reserved here.
    H = hp.DISC_HORN_H
    # Jun 2026 yoke-width fix: the DRIVEN disc (this check probes the DRIVEN
    # side -- femur hip pad / tibia knee pad) seats flush on the body front face,
    # so it sits DRIVEN_HORN_REACH_DOWN (5 mm) below the frozen seat plane and
    # the printed reach-down boss legitimately fills [0, 5] above it.
    reach = hp.DRIVEN_HORN_REACH_DOWN

    # NEW (May 2026 collinear-pad refactor): the link's local origin
    # is the pad MATING FACE = the horn-top plane, so the horn
    # envelope lives at y in [-HORN_STACK_H, 0] = [-5, 0] in NEW
    # link-local coordinates (was y in [0, +HORN_STACK_H] when the
    # link origin sat on the joint axis).  Probe range matches the
    # conservative horn envelope; the link should have NO material in
    # that half-space by construction (pad solid starts at NEW y = 0,
    # spar / well / bridges live elsewhere).
    #
    # CSG margin: shift the probe DOWN by ``BOUNDARY_EPS`` mm so its
    # top face doesn't share a plane with the pad's -Y mating face
    # at NEW y = 0.  Without this the 6-ray inside-test gives
    # boundary-coincident false positives on the pad-bottom voxel
    # row (~ 700-1700 mm^3 of voxel-grid stair-step artefact on a
    # mesh whose -Y face lies EXACTLY at the probe's +Y face) even
    # though ``mesh.contains`` correctly returns False there.  This
    # mirrors the 0.05 mm CSG overshoots the pre-refactor design
    # baked into the cup's y_hi to avoid the same boundary noise.
    BOUNDARY_EPS = 0.05
    print(f"\n[5b] Horn-stack clearance (Phi {2*R:.1f} mm x {H:.1f} mm disc band, "
          f"centred on joint axis, |y| in [{reach:.1f}, {reach + H:.1f}] -- the "
          f"real {H:.1f} mm disc {reach:.1f} mm below the seat, past the "
          f"reach-down boss):")

    # The horn-stack cylinder template is along +Y in part-local frame,
    # which is exactly the joint axis direction in both make_femur_link
    # and make_tibia_link.  Translate by -H - BOUNDARY_EPS so the
    # cylinder spans NEW link y in [-H - eps, -eps] (= a hair below
    # the horn-top plane down to a hair below the spline tip).
    stack = hp._cyl_along(R, H, axis="y")
    # LEG_PITCH_AXIS = -Y (coaxial refit) flips the cradle/horn so the
    # disc-horn void now lives on the +Y half of the link-local frame
    # (was -Y).  Probe whichever half the output axis points toward.  The
    # disc band sits ``reach`` below the seat plane (the pad fills [0, reach]),
    # so offset the band by ``reach`` and keep only DISC_HORN_H of height.
    if hp.LEG_PITCH_AXIS[1] < 0:
        # disc at y in [reach, reach+H] = [reach, HORN_STACK_H]
        stack.apply_translation([0.0, reach + H / 2.0 + BOUNDARY_EPS, 0.0])
    else:
        stack.apply_translation([0.0, -(reach + H / 2.0) - BOUNDARY_EPS, 0.0])

    cases = [
        ("femur_link  (hip-pitch joint)", _load_mesh("femur_link",
                                                       copy=False)),
        ("tibia_link  (knee-pitch joint)", _load_mesh("tibia_link",
                                                        copy=False)),
    ]

    all_ok = True
    for name, mesh in cases:
        vol = _pair_overlap_volume(mesh, stack, pitch=0.8)
        ok = vol <= HORN_STACK_OVERLAP_TOL
        all_ok &= _label(name, ok,
                         f"part vs horn-stack overlap = {vol:7.1f} mm^3 "
                         f"(tol {HORN_STACK_OVERLAP_TOL:.0f})")
    return all_ok


# ---------------------------------------------------------------------------
# 5c.  Horn-SWEEP clearance in the YAW joint (coxa_bracket side)
# ---------------------------------------------------------------------------
#
# WHAT THIS CATCHES that 5b above does NOT
# ----------------------------------------
# check_horn_stack_clearance (5b) probes the DRIVEN side of each joint:
# the femur_link's hub at the hip-pitch joint, and the tibia_link's hub
# at the knee-pitch joint.  At the YAW joint the driven part is the
# coxa_link, whose printed volume sits ABOVE the horn-stack -- not in
# its way -- so 5b correctly skips the yaw joint.
#
# But the YAW joint also has a NON-DRIVEN side -- the coxa_bracket --
# which holds the servo body and whose flange / walls / gussets sit
# RIGHT NEXT TO the rotating gear stack + disc horn (the printed
# adapter is retired).  Nothing in the existing checks ever probed the
# cylindrical sweep volume above the seated yaw servo.  The recurring
# "the servo motor doesn't stick out high enough in the coxa bracket"
# failure lives here: a flange / gusset / boss that intrudes into the
# horn-sweep cylinder physically prevents the disc horn from rotating
# (or worse, prevents the horn stack from seating above the bracket in
# the first place).
#
# WHY 5b's CYLINDER WAS THE WRONG SIZE (for the YAW joint)
# --------------------------------------------------------
# 5b uses radius HORN_STACK_VOID_R = PLASTIC_HORN_X_TIP_R + 0.5 =
# 18.5 mm, sized for the now-retired plastic X-horn's sweep.  But 5b's
# height is HORN_STACK_H = 5 mm, which covers only the disc-horn
# stack, NOT the SERVO_OUTPUT_H = 6 mm gear-stack region between the
# body's top face and the disc horn's bottom face.  A bracket wall
# that wraps over the top of the well to z = +13 mm in bracket-local
# clears the retired printed adapter (which lived above z = +18 in
# legacy coords) but clobbers the gear stack and the disc horn
# underneath.
#
# WHAT THIS CHECK DOES
# --------------------
# Build a vertical cylinder centred on the BRACKET-LOCAL YAW AXIS at
# (x = -SERVO_OUTPUT_X = 0 in bracket coords after the servo offset
# is applied, y = 0).  Its radius is the larger of the retired printed
# adapter's half-OD and the disc horn's tip radius (read from the
# bounding cylinder of the "servo_horn" registry mesh, which now maps
# to ``make_disc_horn``, so the test stays in sync with the modelled
# hardware geometry), plus a small clearance.  Its Z range covers the
# ENTIRE rotating stack:
#
#     z_lo = bracket-local Z of the seated body's TOP face
#            (= SERVO_BODY_H - WELL_RIM_Z, taking WELL_TAB_FLOAT=0 as
#            the worst-case body-seats-lower scenario so the cylinder
#            covers the full gear-stack region even if the tabs sit
#            slightly below their nominal float height)
#     z_hi = z of the horn-stack TOP face + 1 mm extra margin
#
# Any sample voxel inside both the bracket mesh AND this cylinder is
# a FAIL.  The cylinder MUST be entirely VOID inside the printed
# coxa_bracket -- this is not a "soft budget", it's a hard pass /
# fail.
HORN_SWEEP_CLEARANCE = 0.5   # mm -- radial clearance around the horn sweep
HORN_SWEEP_OVERLAP_TOL = 30.0  # mm^3 -- voxel artefact budget (tighter
                               # than 5b: the bracket's flange material is
                               # axis-aligned slabs so any real intrusion
                               # is hundreds of mm^3, well above the
                               # voxel-grid step-noise floor)


def _horn_tip_radius_from_mesh() -> float:
    """Return the disc horn's tip radius, read from the bounding
    cylinder of the ``"servo_horn"`` registry mesh (which now maps to
    ``hp.make_disc_horn()``) projected onto the spline-perpendicular
    plane.  Keeping this read off the actual mesh means a future change
    to the disc-horn factory (a larger disc, an extra feature, a
    different horn style) is automatically picked up by the verifier --
    a recurring failure mode in this project has been "constant drift
    between the rendered visual and what the test measures".
    """
    horn = _load_mesh("servo_horn", copy=False)
    xy = horn.vertices[:, :2]
    return float(np.sqrt((xy ** 2).sum(axis=1)).max())


def check_horn_sweep_clearance():
    """Verify the coxa_bracket has a clean cylindrical VOID above the
    seated yaw servo so the gear stack + disc horn can rotate freely
    without colliding with any bracket-local geometry (flange material
    around the body slot, side gussets, bridge gussets, wire-channel
    flanges, M3 pilot bosses, ...).

    This is the missing check that caused the recurring
    'the servo motor doesn't stick out high enough in the coxa
    bracket' failure to keep slipping through verification.
    """
    plastic_tip_r = _horn_tip_radius_from_mesh()
    plate_r       = hp.HORN_ADAPTER_OD / 2.0
    R = max(plate_r, plastic_tip_r) + HORN_SWEEP_CLEARANCE

    # Commit 7/9 of the May 2026 chassis-bottom-integrated yaw-cradle
    # redesign: rerouted the yaw-cradle horn-sweep probe from
    # ``coxa_bracket`` to the integrated cradle in ``chassis_bottom``.
    # The probe is now expressed in WELL-LOCAL coords (the same frame
    # the hip / knee horn-sweep probes already use): the cradle is
    # mapped via ``_chassis_yaw_cradle_to_well_local`` so the body
    # bottom lands at well-local z = 0 by convention.  The yaw axis
    # then sits at well-local (x = +SERVO_OUTPUT_X, y = 0) (the
    # output spline in servo-local coords) and the body top + gear +
    # horn-stack z range becomes:
    #
    #   body bottom at z = 0
    #   body top    at z = SERVO_BODY_H              = +38
    #   gear top    at z = body_top + WELL_TAB_FLOAT + SERVO_OUTPUT_H
    #   horn top    at z = gear_top + HORN_STACK_H
    # STS3215 front-face mount (Jun 2026): the disc horn seats on the
    # front mount PLATE (well-local z = SERVO_BODY_H + WELL_PLATE_T =
    # the plate top) and sweeps only ABOVE it -- the output coupling
    # below the plate runs through the plate's central clearance bore at
    # small radius.  The DS3225 probe started the sweep cylinder at the
    # body front face (open-top bucket, horn directly above the body);
    # keeping that here would dip the cylinder WELL_PLATE_T mm into the
    # mount plate and report a false intrusion.  Start at the disc-horn
    # BASE (= plate top) so the void being verified is the airspace the
    # disc actually rotates through.
    body_top_z = hp.SERVO_BODY_H                 # +34.3 (body front face)
    horn_base_z = body_top_z + hp.WELL_PLATE_T   # +38.3 (plate top = disc base)
    adapter_top_z = horn_base_z + hp.HORN_STACK_H
    # The disc horn SEATS on the mount-plate top (well-local z = horn_base_z)
    # and sweeps in the airspace ABOVE it; the solid plate below is the
    # bearing-supported seat, not an obstruction.  Voxelising a cylinder
    # whose base plane coincides with that solid seat counts the plate's
    # top boundary layer as a false intrusion (~1 kmm^3 of pure z=plate-top
    # sliver).  Start the probe HORN_SEAT_SKIP above the seat so we verify
    # only the rotating airspace.  (Probed: no cradle material exists above
    # the seat plane inside the envelope, so this skip cannot hide a real
    # obstruction.)
    HORN_SEAT_SKIP = 1.0               # mm -- skip the coincident seat plane
    z_lo = horn_base_z + HORN_SEAT_SKIP
    z_hi = adapter_top_z + 1.0         # +1 mm margin above adapter top
    H    = z_hi - z_lo

    # Well-local yaw axis: the output spline sits at well-local
    # (x = +SERVO_OUTPUT_X, y = 0) -- the body is offset in the well so
    # the spline lands at +SERVO_OUTPUT_X from the well origin (which
    # IS the body-bottom centre by convention).
    yaw_x = +hp.SERVO_OUTPUT_X
    yaw_y = 0.0

    print(f"\n[5c] Horn-sweep clearance in chassis_bottom yaw cradle "
          f"(Phi {2*R:.1f} mm x {H:.1f} mm tall):")
    print(f"     well-local axis at (x={yaw_x:+.1f}, y={yaw_y:+.1f}); "
          f"z in [{z_lo:+.2f}, {z_hi:+.2f}]")
    print(f"     plastic horn tip radius (from mesh) = "
          f"{plastic_tip_r:.2f} mm; printed adapter radius = "
          f"{plate_r:.2f} mm; clearance = {HORN_SWEEP_CLEARANCE:.1f} mm")

    cyl = hp._cyl(R, H, sections=hp.CYL_SECTIONS)
    cyl.apply_translation([yaw_x, yaw_y, 0.5 * (z_lo + z_hi)])

    bracket = _chassis_yaw_cradle_to_well_local(
        _load_mesh("chassis_assembled", copy=False))
    pitch = 0.6
    vol = _pair_overlap_volume(bracket, cyl, pitch=pitch)
    ok = vol <= HORN_SWEEP_OVERLAP_TOL

    _label("chassis_bottom yaw cradle horn-sweep void", ok,
           f"cradle-inside-cylinder vol = {vol:7.1f} mm^3 "
           f"(tol {HORN_SWEEP_OVERLAP_TOL:.0f})")

    # Diagnostic: when the check fails, report WHERE the intrusion
    # lives in well-local coordinates so the geometry fix is obvious
    # from the verifier output.  Resample at the same pitch.
    if not ok:
        a_min, a_max = bracket.bounds
        b_min, b_max = cyl.bounds
        lo = np.maximum(a_min, b_min)
        hi = np.minimum(a_max, b_max)
        n  = np.maximum(2, np.ceil((hi - lo) / pitch).astype(int))
        gx = np.linspace(lo[0], hi[0], int(n[0]))
        gy = np.linspace(lo[1], hi[1], int(n[1]))
        gz = np.linspace(lo[2], hi[2], int(n[2]))
        XX, YY, ZZ = np.meshgrid(gx, gy, gz, indexing="ij")
        pts = np.stack([XX.ravel(), YY.ravel(), ZZ.ravel()], axis=1)
        in_a = points_inside(bracket, pts)
        bad = np.empty((0, 3))
        if in_a.sum() > 0:
            pts_a = pts[in_a]
            in_b = points_inside(cyl, pts_a)
            bad = pts_a[in_b]
        if len(bad) > 0:
            b_lo = bad.min(axis=0)
            b_hi = bad.max(axis=0)
            cen  = bad.mean(axis=0)
            print(f"     FAIL: {len(bad)} sample voxel(s) inside both bracket "
                  f"AND horn-sweep cylinder.")
            print(f"        bracket-local intrusion bbox:")
            print(f"          x in [{b_lo[0]:+.2f}, {b_hi[0]:+.2f}] mm")
            print(f"          y in [{b_lo[1]:+.2f}, {b_hi[1]:+.2f}] mm")
            print(f"          z in [{b_lo[2]:+.2f}, {b_hi[2]:+.2f}] mm")
            print(f"        centroid = ({cen[0]:+.2f}, {cen[1]:+.2f}, "
                  f"{cen[2]:+.2f}) mm")

    # Rotational-sweep diagnostic: probe the SAME cylinder against the
    # bracket through the runtime yaw range +/- 60 deg (about z) and
    # report whether the overlap differs across rotation.  A round
    # cylinder is rotationally invariant, so this should give the
    # SAME overlap volume at every angle -- if it doesn't, the
    # voxel-sampling itself has an asymmetric bias and the user
    # should be told.  Cheap (3 angles) and only printed in the
    # output for visibility.
    yaw_angles_deg = [-60.0, 0.0, +60.0]
    sweep_vols = []
    for ang in yaw_angles_deg:
        cyl_r = cyl.copy()
        cyl_r.apply_transform(
            rotation_matrix(np.deg2rad(ang), [0, 0, 1],
                            point=[yaw_x, yaw_y, 0.0]))
        sweep_vols.append(_pair_overlap_volume(bracket, cyl_r, pitch=pitch))
    sweep_str = ", ".join(
        f"yaw={ang:+.0f} deg -> {v:6.1f} mm^3"
        for ang, v in zip(yaw_angles_deg, sweep_vols))
    print(f"     rotational sweep diagnostic: {sweep_str}")

    return ok


# ---------------------------------------------------------------------------
# 5d.  Disc-horn FIT / clearance (the disc actually fits where it seats)
# ---------------------------------------------------------------------------
#
# WHAT THIS CATCHES that 5b / 5c do NOT
# -------------------------------------
# 5b probes the DRIVEN link hub against the (retired-X-horn) envelope;
# 5c probes the AIRSPACE *above* the seat plane that the disc rotates
# through.  NEITHER probes the band the disc-horn BODY physically
# occupies.  The Phi DISC_HORN_OD = 20 mm aluminium disc screws onto the
# 25T output spline and rides on the servo's output boss -- so its
# underside sits at body-front + SERVO_OUTPUT_H and its body spans the
# DISC_HORN_H-tall band ABOVE that.  Every printed feature the disc
# passes through / seats in MUST clear at least Phi DISC_HORN_OD there.
#
# WHY 5c PASSED A BROKEN LAYOUT
# -----------------------------
# 5c starts its sweep cylinder at horn_base_z = body_top + WELL_PLATE_T
# (the mount-PLATE top) on the assumption the disc floats above the
# plate.  But the servo's output boss only lifts the disc SERVO_OUTPUT_H
# (2 mm) above the body face -- NOT WELL_PLATE_T (4 mm) -- so the disc's
# lower ~2 mm actually sits INSIDE the plate band.  The chassis_bottom
# yaw cradle bored that band to only Phi SERVO_OUTPUT_BORE_OD (10 mm)
# "for the coupling/spline", so a Phi 20 disc rammed the plate and could
# not seat -- yet 5c, probing only ABOVE plate-top, reported the cradle
# clear.  This check probes the REAL disc-body band (from the boss seat
# up), at Phi DISC_HORN_OD, so a 10 mm-hole-for-a-20 mm-disc regression
# in ANY fixed cradle is caught.
#
# The disc is a circle -> rotationally invariant -> the swept envelope of
# the spinning disc is itself Phi DISC_HORN_OD, so a static Phi
# DISC_HORN_OD cylinder IS the swept envelope here (the larger hub +
# coxa-link that spin above the disc are covered by 5c's airspace probe).
DISC_HORN_FIT_TOL = 40.0   # mm^3 -- voxel-grid artefact budget (a real
                           # 10-for-20 intrusion is hundreds of mm^3)


def check_disc_horn_fit():
    """Verify every printed FIXED cradle that the disc horn seats in has
    a clear Phi DISC_HORN_OD void across the disc-horn BODY band, so the
    20 mm aluminium disc physically fits where it rides on the spline.

    Probed in WELL-LOCAL coords (body bottom at z = 0, output axis at
    x = +SERVO_OUTPUT_X) for all three driven joints:

      * chassis_bottom yaw cradle (mapped via the cradle->well helper),
      * coxa_link's hip cradle (hip fixed side; un-does its joint placement),
      * femur_link's knee cradle (knee fixed side; already well-local).
    """
    R = hp.DISC_HORN_OD / 2.0
    # The disc rides on the servo output boss (SERVO_OUTPUT_H proud of the
    # body front face), NOT on the mount-plate top -- this is the band 5c
    # skipped.  Span the full disc body, trimming BOUNDARY_EPS off each end
    # so the probe's end faces don't share a plane with a mating face.
    BOUNDARY_EPS = 0.05
    horn_base_z = hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H
    horn_top_z = horn_base_z + hp.DISC_HORN_H
    z_lo = horn_base_z + BOUNDARY_EPS
    z_hi = horn_top_z - BOUNDARY_EPS
    H = z_hi - z_lo
    yaw_x = +hp.SERVO_OUTPUT_X

    print(f"\n[5d] Disc-horn fit (Phi {hp.DISC_HORN_OD:.1f} mm disc body x "
          f"{H:.1f} mm tall, on output boss):")
    print(f"     well-local axis at (x={yaw_x:+.1f}, y=+0.0); "
          f"z in [{z_lo:+.2f}, {z_hi:+.2f}] (boss seat + DISC_HORN_H)")

    cyl = hp._cyl(R, H, sections=hp.CYL_SECTIONS)
    cyl.apply_translation([yaw_x, 0.0, 0.5 * (z_lo + z_hi)])

    # The hip/knee fixed cradles live inside one-piece printed parts, so
    # build them directly.  The coxa_link hip cradle is in coxa-local frame
    # (its fixed side is placed by _joint_place(COXA_HIP_ANCHOR, +X,
    # LEG_PITCH_AXIS)); invert that so the disc-horn axis lands back on
    # the well-local output axis.  The femur_link's knee cradle
    # (_femur_knee_fixed_solid) is already well-local (no joint transform).
    M_hip = hp._joint_place(hp.COXA_HIP_ANCHOR, (1, 0, 0), hp.LEG_PITCH_AXIS)
    hip = hp.make_coxa_link_part()
    hip.apply_transform(np.linalg.inv(M_hip))

    cases = [
        ("chassis_bottom yaw cradle",
         _chassis_yaw_cradle_to_well_local(_load_mesh("chassis_assembled"))),
        ("coxa_link (hip cradle)", hip),
        ("femur_link knee cradle", hp._femur_knee_fixed_solid()),
    ]

    all_ok = True
    for name, mesh in cases:
        vol = _pair_overlap_volume(mesh, cyl, pitch=0.6)
        ok = vol <= DISC_HORN_FIT_TOL
        all_ok &= _label(f"{name} disc-horn void", ok,
                         f"cradle-inside-disc vol = {vol:7.1f} mm^3 "
                         f"(tol {DISC_HORN_FIT_TOL:.0f})")
    return all_ok


# ---------------------------------------------------------------------------
# 5e.  Passive (stock-horn) back-stack guard
# ---------------------------------------------------------------------------
#
# Symmetric-yoke refit (Jun 2026): the external 688 ball bearing + its back
# housing are RETIRED.  Stock-horn refit (Jul 2026): the printed centering
# ADAPTER is retired too -- the STS3215's STOCK metal passive horn slides
# over the rear idler boss, seats FLUSH on the servo back face (mating face
# at -DISC_HORN_H) and is held by the central retention screw.  This guard
# pins the passive-stack invariants: the yoke bottom-arm seat derives from
# the flush stock-horn face, the yoke clevis opening matches the real
# horn-face-to-horn-face span (with the YOKE_SEAT_INTERF preload), and
# BACK_STACK_DEPTH stays FROZEN so the joint envelope / COXA_HIP_ANCHOR_Y
# are unchanged from the bearing era (kinematics frozen).


def check_passive_horn_stack():
    """Guard for the stock-horn passive stack: the passive horn seats flush
    on the servo back face (no printed adapter), the yoke bottom-arm seat
    derives from that face, and the frozen back-stack envelope holds."""
    print("\n[5e] Passive stock-horn stack + frozen back-stack:")

    all_ok = True
    # Stock horn seats FLUSH on the back face: mating face one horn below it.
    all_ok &= _label("PASSIVE_HORN_FACE_Z = -DISC_HORN_H (flush stock horn)",
                     abs(hp.PASSIVE_HORN_FACE_Z + hp.DISC_HORN_H) < 1e-6,
                     f"{hp.PASSIVE_HORN_FACE_Z:.2f} == {-hp.DISC_HORN_H:.2f}")

    # Frozen kinematics: BACK_STACK_DEPTH (-> COXA_HIP_ANCHOR_Y / joint envelope)
    # stays at -(REAR_BOSS_H + HORN_STACK_H) regardless of the printed bottom-arm
    # depth.  This is what the bearing-era freeze actually pins.
    expect_back = hp.REAR_BOSS_H + hp.HORN_STACK_H
    all_ok &= _label("BACK_STACK_DEPTH frozen at REAR_BOSS_H+HORN_STACK_H",
                     abs(hp.BACK_STACK_DEPTH - expect_back) < 1e-6,
                     f"{hp.BACK_STACK_DEPTH:.2f} == {expect_back:.2f}")
    # Symmetric-yoke refit: the BOTTOM-arm seat is one YOKE_ARM_PAD below the
    # real passive disc face -- a true mirror of the top arm, so a single M3 x 10
    # screw bolts each side identically.
    expect_seat = hp.PASSIVE_HORN_FACE_Z - hp.YOKE_ARM_PAD
    all_ok &= _label("JOINT_HORN_BOT_Z = PASSIVE_HORN_FACE_Z - YOKE_ARM_PAD",
                     abs(hp.JOINT_HORN_BOT_Z - expect_seat) < 1e-6,
                     f"{hp.JOINT_HORN_BOT_Z:.2f} == {expect_seat:.2f}")

    # Clevis opening = real horn-face-to-horn-face span minus the 2-sided
    # clamp preload (the user-measured fit: stock horn stack, snug squeeze).
    reach = hp.YOKE_ARM_PAD + hp.YOKE_SEAT_INTERF
    opening = (hp.JOINT_HORN_TOP_Z - reach) - (hp.JOINT_HORN_BOT_Z + reach)
    real_span = ((hp.SERVO_BODY_H + hp.DISC_HORN_H)      # driven horn top
                 - hp.PASSIVE_HORN_FACE_Z)               # passive horn face
    expect_opening = real_span - 2 * hp.YOKE_SEAT_INTERF
    all_ok &= _label("yoke clevis opening = horn span - 2*preload",
                     abs(opening - expect_opening) < 1e-6,
                     f"{opening:.2f} == {expect_opening:.2f}")
    return all_ok


# ---------------------------------------------------------------------------
# 6.  Flimsy joints (local thickness from a 3D distance transform)
# ---------------------------------------------------------------------------
#
# What the check computes
# -----------------------
# For each printed part:
#
#   1. Voxelise the (watertight) mesh on a uniform grid at
#      ``FLIMSY_VOXEL_PITCH``.  Trimesh's ``mesh.voxelized().fill()``
#      gives a 3D boolean occupancy array.
#
#   2. Compute the 3D Euclidean distance transform of the occupancy
#      (``scipy.ndimage.distance_transform_edt``).  At each interior
#      voxel this returns ``dt(V)`` = distance, in voxels, to the
#      nearest empty voxel.
#
#   3. A voxel V is **thick** iff some inscribed ball of radius
#      ``MIN_PRINT_T / 2`` (entirely inside the solid) contains V --
#      i.e. there exists a "core" voxel P (one whose ``dt(P)`` >=
#      ``MIN_PRINT_T / 2 / pitch``) with ``|V - P| <= MIN_PRINT_T/2``.
#      Equivalently: V is thick iff its distance to the nearest core
#      voxel is at most ``MIN_PRINT_T / 2 / pitch``, which is another
#      EDT pass.  The Hildebrand 1997 max-inscribed-ball formulation
#      avoids the well-known surface-fuzz pitfall of the naive
#      "2 * dt" definition: every surface voxel of a healthy 4 mm
#      wall sits at dt = 1.0 voxel, so the naive definition would
#      flag the WHOLE outer shell of every thick part as flimsy.
#
#   4. Flimsy voxels are the occupied ones NOT covered by a thick
#      ball.  Cluster them with ``scipy.ndimage.label``; tiny clusters
#      (< ``MIN_CLUSTER_VOX``) are dismissed as voxel-grid noise.
#
#   5. Report each cluster's centroid (part-local mm), bounding box
#      extent, minimum thickness, and voxel count.  The check PASSES
#      if the largest cluster in any single part stays at or below
#      ``MAX_FLIMSY_BUDGET_VOX``.

def _flimsy_clusters_for_part(mesh, pitch, min_t, min_cluster_vox):
    """Return ``(clusters, biggest_voxel_count, max_thickness_mm)`` for
    one part.

    Each entry in ``clusters`` is a dict::

        {
            "voxel_count":   int,
            "min_thickness": float (mm),
            "centroid":      np.ndarray shape (3,)  -- part-local mm,
            "bbox_min":      np.ndarray shape (3,)  -- part-local mm,
            "bbox_max":      np.ndarray shape (3,)  -- part-local mm,
        }

    Sorted descending by voxel count.
    """
    try:
        vg = mesh.voxelized(pitch=pitch).fill()
    except Exception as exc:
        # Should not happen for a watertight mesh -- but if it does,
        # fail loudly rather than silently passing.
        raise RuntimeError(f"voxelization failed: {exc}")

    occ = np.asarray(vg.matrix, dtype=bool)
    voxel_pitch = float(vg.pitch[0])

    if not occ.any():
        return [], 0, 0.0

    # EDT 1: distance from each voxel to the nearest empty voxel
    # (in voxel units; multiply by pitch for mm).
    dt_vox = distance_transform_edt(occ)
    dt_mm  = dt_vox * voxel_pitch
    max_thickness = float(2.0 * dt_mm.max())

    # Core voxels: inscribed ball of radius MIN_PRINT_T/2 fits entirely
    # inside the solid centred here.  An occupied voxel is THICK iff
    # within MIN_PRINT_T/2 of any core voxel (i.e. some such inscribed
    # ball contains it).  This is the Hildebrand 1997 local-thickness
    # criterion, formulated as two EDT passes.
    half_t_vox = (min_t / 2.0) / voxel_pitch
    core = dt_vox >= half_t_vox

    if core.any():
        # EDT 2: distance from each voxel to the nearest core voxel.
        dt_to_core = distance_transform_edt(~core)
        thick = dt_to_core <= half_t_vox
    else:
        thick = np.zeros_like(occ)

    flimsy = occ & ~thick

    if not flimsy.any():
        return [], 0, max_thickness

    labeled, n_labels = label(flimsy)
    clusters = []
    for cluster_id in range(1, n_labels + 1):
        cluster_mask = labeled == cluster_id
        n_vox = int(cluster_mask.sum())
        if n_vox < min_cluster_vox:
            continue
        idx = np.argwhere(cluster_mask)
        lo_idx = idx.min(axis=0).astype(float)
        hi_idx = idx.max(axis=0).astype(float)
        centroid_idx = idx.mean(axis=0)
        centroid_world  = vg.indices_to_points(
            centroid_idx[None, :])[0]
        bbox_min_world  = vg.indices_to_points(lo_idx[None, :])[0]
        bbox_max_world  = vg.indices_to_points(hi_idx[None, :])[0]
        # Cluster "thickness" = 2 * max(dt_mm) inside the cluster.
        # The max-dt voxel sits at the cluster's THICKEST point (the
        # ridge / medial line of the thin feature); doubling its dt
        # recovers the wall thickness in mm.  E.g. a 2.5 mm well wall
        # has max(dt_mm) ~= 1.25 mm and reports "min t = 2.5 mm",
        # immediately telling the user what the actual structural
        # thickness of the flagged region is.
        min_t_cluster = float(2.0 * dt_mm[cluster_mask].max())
        clusters.append({
            "voxel_count":   n_vox,
            "min_thickness": min_t_cluster,
            "centroid":      centroid_world,
            "bbox_min":      bbox_min_world,
            "bbox_max":      bbox_max_world,
        })

    clusters.sort(key=lambda c: -c["voxel_count"])
    biggest = clusters[0]["voxel_count"] if clusters else 0
    return clusters, biggest, max_thickness


# ---------------------------------------------------------------------------
# Targeted bridge-joint check: coxa_link's top-piece <-> servo-well joint
# ---------------------------------------------------------------------------
#
# Why this exists -- and why ``check_flimsy_joints`` alone cannot see this
# failure mode:
#
# In ``make_coxa_link`` the HORN YOKE (the top hub bolted to the yaw
# servo's disc horn) is tied to the SERVO WELL BOX (the cradle that
# holds the hip-pitch servo) by an X-long, Y-thin BRIDGE slab that lives
# in the Z-gap between the well's outer top face and the hub/arm bottom
# face.  The bridge's cross-section (with the baseline geometry) is
# 53 mm long in X, 6.75 mm thick in Y, ~14.5 mm tall in Z; the bridge
# overlaps the well's outer body by just 0.5 mm in Y and 1.5 mm in Z,
# giving a 53 x 0.5 x 1.5 = ~40 mm^3 bonded interface holding the entire
# leg load.  ``pad_sweep_clear`` -- the cylindrical void that lets the
# femur hip pad swing through the link's interior -- then EATS a circular
# hole through this already-thin bridge across most of its X span,
# leaving only thin slivers at the inboard / outboard X extremes.
#
# Why ``check_flimsy_joints`` doesn't catch it: the Hildebrand
# max-inscribed-ball check (``_flimsy_clusters_for_part`` above) measures
# isotropic LOCAL thickness via a distance transform; an inscribed sphere
# of radius MIN_PRINT_T/2 = 1.5 mm fits inside the bridge slab (6.75 mm
# Y) without a problem, so the test happily reports the bridge as healthy.
# ``check_thin_sheets`` (the anisotropic chord variant) ALSO misses this
# because the pedestal sits directly above the inboard half of the bridge
# (link y in [-17, +17] for the pedestal vs. y in [-17.25, -10.5] for the
# bridge), and its Y-chord through the bridge merges with the pedestal's
# Y-chord into a ~34 mm-long combined chord that's outside the
# 5-7.2 mm "structural-neck" band.  The bridge's Y-thinness is only
# visible at link x > +17 (outboard of the pedestal), where ``gusset``
# and ``gusset_under`` add a few extra mm of Y material -- pushing the
# combined chord out of the band again.
#
# So we need an explicit slice-based check whose window EXCLUDES the
# pedestal and targets the bridge alone.  The check below slices the
# coxa_link mesh horizontally at multiple Z heights in the gap between
# the well's outer top and the hub bottom, intersects each slice with an
# outboard-of-pedestal X-Y window, and computes (a) the area of the
# resulting cross-section and (b) the bending section modulus Sx about
# the slice's local X axis (the bridge's weakest bending direction --
# flexure in YZ caused by the well's leg-load Y reaction torque).
#
# Thresholds are tuned to FAIL on the baseline ``make_coxa_link()`` (so
# the recurring "top of coxa_link is not attached strongly to the part
# housing the servo" complaint is caught the first time it appears) and
# PASS comfortably on a wall-widening / pad fix that adds the user's
# requested "rectangular block at the top of the well":
#
#   * Area >= 80 mm^2 minimum slice  -- matches the user's "~ 8 x 10 mm
#     of plastic" intuition (parent agent's spec).
#   * Sx_bend >= 100 mm^3            -- bending modulus of the OUTBOARD
#     bridge slice about its local X axis.  Baseline geometry has
#     Sx_bend ~ 6 mm^3 at the worst slice (essentially zero); a 5 mm
#     well-top wall extension lifts it well past 100 mm^3 because the
#     extra plastic is offset from the centroid in Y and shows up
#     quadratically in Ixx.

COXA_LINK_BRIDGE_AREA_MIN_MM2 = 80.0   # parent's recommendation
COXA_LINK_BRIDGE_SX_MIN_MM3   = 100.0  # ~ half-modulus of a 10x20 bar in
                                        # its weak axis (20x10^2/6/2 ~ 167);
                                        # rounded down so a "rectangular
                                        # block at top of well" pad clears
                                        # the threshold without needing to
                                        # rebuild the entire link.

COXA_LINK_BRIDGE_SLICE_DZ     = 0.5    # mm -- slice spacing in Z

# Second bridge sub-check: per-X YZ cross-section probe.
#
# The XY-slice check above averages cross-section over the full bridge X
# range (-12 .. +41) -- so a 53 mm-long bridge with FAT plastic at the X
# extremes (where pad_sweep_clear does not reach) and THIN plastic in
# the middle (at x ~ COXA_LENGTH = +25, where pad_sweep_clear eats most
# of the bridge + pad) reports a healthy 157 mm^2 slice area even when
# the middle is a 4 mm-thin vertical strip.  That's exactly the failure
# mode the user pointed at in the May 18 screenshot ("you made the wrong
# wall thicker -- the joint in the middle left is whats thin").
#
# This second sub-check takes a YZ cross-section at EVERY 1 mm in X
# across the bridge X range, intersects with a bridge-only YZ window,
# and reports the minimum cross-section area + minimum Y-thickness at
# each x. The "Y-thickness" is the maximum y-extent of any horizontal
# slab inside the slice (i.e. the widest section the bridge maintains
# at some z): the bridge ties the well in Y direction, so Y-thickness
# is the structural metric in bending and shear.
#
# Threshold: 6 mm minimum Y-thickness AND 50 mm^2 minimum slice area at
# ANY x in the bridge x range catches the cylinder-cut middle.

COXA_LINK_BRIDGE_X_STEP_MM    = 1.0    # mm -- spacing of YZ probes in X
COXA_LINK_BRIDGE_MIN_AREA_MM2 = 50.0   # mm^2 -- min YZ cross-section area
                                       # at any x in the bridge x range
COXA_LINK_BRIDGE_MIN_Y_THK_MM = 6.0    # mm -- min Y thickness (max y extent
                                       # at any single z in the bridge slice)


def _slice_polygons(mesh, z):
    """Horizontal cross-section of ``mesh`` at world Z = ``z`` as a
    shapely MultiPolygon in link (X, Y) coordinates.  Empty if the mesh
    does not cross the plane.
    """
    section = mesh.section(plane_origin=[0.0, 0.0, float(z)],
                           plane_normal=[0.0, 0.0, 1.0])
    if section is None:
        return _sg.MultiPolygon()
    planar, _T = section.to_planar()
    polys = list(planar.polygons_full)
    if not polys:
        return _sg.MultiPolygon()
    if len(polys) == 1:
        return _sg.MultiPolygon([polys[0]])
    return _sg.MultiPolygon(polys)


def _ixx_about_centroid_x(geom, *, pitch=0.25):
    """Rasterise ``geom`` (shapely Polygon / MultiPolygon in (X, Y)) at
    ``pitch`` mm resolution and return ``(area, centroid_y, ixx, y_min,
    y_max)`` where Ixx is the second moment of area about an axis
    parallel to the X axis passing through the centroid (units: mm^4).

    Bending the bridge column (axis along link +Z) about this X axis
    bends the slab in the (Y, Z) plane -- the bridge's weakest bending
    direction in its current Y-thin geometry.
    """
    if geom.is_empty:
        return 0.0, 0.0, 0.0, 0.0, 0.0
    minx, miny, maxx, maxy = geom.bounds
    # Sample on a regular grid.
    nx = max(int(np.ceil((maxx - minx) / pitch)), 1)
    ny = max(int(np.ceil((maxy - miny) / pitch)), 1)
    xs = minx + (np.arange(nx) + 0.5) * (maxx - minx) / nx
    ys = miny + (np.arange(ny) + 0.5) * (maxy - miny) / ny
    XX, YY = np.meshgrid(xs, ys, indexing="xy")
    pts_x = XX.ravel()
    pts_y = YY.ravel()
    # Vectorised "inside" via shapely.contains with prepared geometry.
    from shapely import contains_xy
    mask = contains_xy(geom, pts_x, pts_y)
    if not mask.any():
        return 0.0, 0.0, 0.0, 0.0, 0.0
    da = (maxx - minx) / nx * (maxy - miny) / ny
    area = float(mask.sum() * da)
    yy = pts_y[mask]
    y_c = float(yy.mean())
    ixx = float(((yy - y_c) ** 2).sum() * da)
    return area, y_c, ixx, float(yy.min()), float(yy.max())


def _check_coxa_link_bridge_yz_thickness(coxa_link_mesh):
    """Per-X YZ cross-section probe of coxa_link's bridge region.

    Catches the user-pointed "thin neck in the middle" failure mode
    where ``pad_sweep_clear`` eats most of the bridge + well-top pad
    at x ~ COXA_LENGTH = +25 mm, leaving only a thin vertical strip
    where the bridge ties the well to the arm/hub.

    Probes every COXA_LINK_BRIDGE_X_STEP_MM mm in X across the bridge
    x range. At each x, point-samples a YZ grid inside a bridge-only
    window (y between the well rim and the arm -Y face, z between the
    well top and the arm bottom) and measures the cross-section area
    + the maximum Y extent at any single z (= the widest local slab).
    """
    well_z_drop = -(hp.WELL_D / 2.0
                    + hp.COXA_ARM_T / 2.0
                    + hp.WELL_Z_DROP_EXTRA)
    well_top_z  = hp.COXA_LIFT + well_z_drop + hp.WELL_D / 2.0
    yoke_bot_z  = hp.COXA_LIFT

    # Bridge x range: full arm extent (the bridge spans the arm's
    # full length).
    arm_x_extent = hp.COXA_LENGTH + 28.0
    x_lo = -12.0 + 1.0                               # +1 mm margin
    x_hi = arm_x_extent - 12.0 - 1.0                 # arm +X end - 1 mm
    n_x = max(int(np.ceil((x_hi - x_lo) /
                          COXA_LINK_BRIDGE_X_STEP_MM)) + 1, 2)
    x_values = np.linspace(x_lo, x_hi, n_x)

    # YZ window: y from a margin below the well rim to just above the
    # arm's -Y face (so a fix that widens the pad in -Y still lands in
    # the window). z from just above the well top to just below the
    # arm bottom (so we measure the BRIDGE alone, not the well's outer
    # slab or the arm slab).
    y_lo, y_hi = -34.0, -10.0
    z_lo, z_hi = well_top_z + 0.25, yoke_bot_z - 0.25
    dy = 0.25                                        # mm grid pitch in y
    dz = 0.5                                         # mm grid pitch in z
    ys = np.arange(y_lo, y_hi + 1e-9, dy)
    zs = np.arange(z_lo, z_hi + 1e-9, dz)
    dA = dy * dz

    rows = []   # (x, area, max_y_extent_at_any_z)
    YY, ZZ = np.meshgrid(ys, zs, indexing="ij")
    for x in x_values:
        pts = np.stack([np.full(YY.size, float(x)),
                        YY.ravel(), ZZ.ravel()], axis=1)
        inside = coxa_link_mesh.contains(pts).reshape(YY.shape)
        area = float(inside.sum()) * dA
        # Max Y-extent at any single z (column in `inside`): count
        # interior cells per z row and multiply by dy.
        per_z = inside.sum(axis=0)
        max_y_extent = float(per_z.max()) * dy
        rows.append((float(x), area, max_y_extent))

    areas = [r[1] for r in rows]
    yt = [r[2] for r in rows]
    min_area = min(areas) if areas else 0.0
    min_yt   = min(yt)    if yt    else 0.0
    min_area_x = rows[int(np.argmin(areas))][0] if rows else 0.0
    min_yt_x   = rows[int(np.argmin(yt))][0]    if rows else 0.0

    ok_area = min_area >= COXA_LINK_BRIDGE_MIN_AREA_MM2
    ok_yt   = min_yt   >= COXA_LINK_BRIDGE_MIN_Y_THK_MM
    ok = ok_area and ok_yt

    head = (f"min YZ-slice area = {min_area:5.1f} mm^2 at x = {min_area_x:+.1f} "
            f"({'>=' if ok_area else '<'}{COXA_LINK_BRIDGE_MIN_AREA_MM2:.0f}), "
            f"min Y-thickness = {min_yt:4.2f} mm at x = {min_yt_x:+.1f} "
            f"({'>=' if ok_yt else '<'}{COXA_LINK_BRIDGE_MIN_Y_THK_MM:.0f}); "
            f"YZ window y in ({y_lo:.1f}, {y_hi:.1f}), z in ({z_lo:.1f}, {z_hi:.1f})")
    _label("coxa_link bridge YZ thickness", ok, head)

    if not ok:
        # Dump worst 8 x values so the user can see where the bridge
        # is thinnest.
        ranked = sorted(rows, key=lambda r: r[2])[:8]
        print(f"           thinnest 8 x slices (sorted by Y-thickness):")
        for x, a, yt_val in ranked:
            tag_a = " " if a    >= COXA_LINK_BRIDGE_MIN_AREA_MM2 else "A"
            tag_y = " " if yt_val >= COXA_LINK_BRIDGE_MIN_Y_THK_MM else "Y"
            print(f"           [{tag_a}{tag_y}]  x = {x:+6.2f}  "
                  f"area = {a:5.1f} mm^2  max Y-thickness = {yt_val:5.2f} mm")
    return ok


def _check_coxa_link_bridge_joint(coxa_link_mesh):
    """Slice-based check for the bridge that ties coxa_link's horn yoke
    (top hub) to the hip-pitch servo well box.  See the big comment
    above for motivation and threshold rationale.
    """
    # All Z coordinates are in the final LIFTED coxa_link frame (i.e.
    # what ``hp.make_coxa_link()`` returns).
    #
    # Well outer top face Z (= COXA_LIFT + well_z_drop + WELL_D/2)
    well_z_drop = -(hp.WELL_D / 2.0
                    + hp.COXA_ARM_T / 2.0
                    + hp.WELL_Z_DROP_EXTRA)
    well_top_z  = hp.COXA_LIFT + well_z_drop + hp.WELL_D / 2.0
    yoke_bot_z  = hp.COXA_LIFT                       # = hub / arm bottom

    # Outboard-of-pedestal slice window.  Pedestal is the 34 x 34 mm
    # square pillar centred on (0, 0) (link x, y in [-17, +17]) so an X
    # window x in [+17, +50] excludes the pedestal entirely.  Y window
    # covers the bridge's existing y range [-17.25, -10.5] with generous
    # margin on BOTH sides so a wall-widening / pad fix that grows the
    # bridge in -Y (toward the well's outer body) or +Y (toward the arm
    # face) still lands inside the window.
    x_win = (+17.0, +50.0)
    y_win = (-25.0,  -5.0)
    window = _sg.box(x_win[0], y_win[0], x_win[1], y_win[1])

    # Slice every COXA_LINK_BRIDGE_SLICE_DZ mm in the gap, with a
    # COXA_LINK_BRIDGE_SLICE_DZ mm margin off each end so we never hit
    # the exact well-top or arm-bottom plane (where the cross-section
    # discontinuously jumps to the full well or arm footprint and
    # masks the bridge's weakness).
    z_lo = well_top_z + COXA_LINK_BRIDGE_SLICE_DZ
    z_hi = yoke_bot_z - COXA_LINK_BRIDGE_SLICE_DZ
    n_slices = max(int(round((z_hi - z_lo) / COXA_LINK_BRIDGE_SLICE_DZ)) + 1, 2)
    z_values = np.linspace(z_lo, z_hi, n_slices)

    rows = []   # (z, area, sx)
    for z in z_values:
        polys = _slice_polygons(coxa_link_mesh, float(z))
        if polys.is_empty:
            rows.append((float(z), 0.0, 0.0))
            continue
        inter = polys.intersection(window)
        if inter.is_empty:
            rows.append((float(z), 0.0, 0.0))
            continue
        area, y_c, ixx, y_min, y_max = _ixx_about_centroid_x(inter)
        if area <= 0.0:
            rows.append((float(z), 0.0, 0.0))
            continue
        c = max(y_max - y_c, y_c - y_min)
        sx = ixx / c if c > 1e-6 else 0.0
        rows.append((float(z), area, sx))

    areas = [r[1] for r in rows]
    sxs   = [r[2] for r in rows]
    min_area = min(areas) if areas else 0.0
    min_sx   = min(sxs)   if sxs   else 0.0
    min_area_z = rows[int(np.argmin(areas))][0] if rows else 0.0
    min_sx_z   = rows[int(np.argmin(sxs))][0]   if rows else 0.0

    ok_area = min_area >= COXA_LINK_BRIDGE_AREA_MIN_MM2
    ok_sx   = min_sx   >= COXA_LINK_BRIDGE_SX_MIN_MM3
    ok = ok_area and ok_sx

    head = (f"min slice area = {min_area:6.1f} mm^2 "
            f"({'>=' if ok_area else '<'}{COXA_LINK_BRIDGE_AREA_MIN_MM2:.0f}), "
            f"Sx_bend = {min_sx:7.1f} mm^3 "
            f"({'>=' if ok_sx else '<'}{COXA_LINK_BRIDGE_SX_MIN_MM3:.0f}); "
            f"slice band z in [{z_lo:.1f}, {z_hi:.1f}] mm "
            f"(window x in {x_win}, y in {y_win})")
    _label("coxa_link top<->well bridge", ok, head)

    if not ok:
        # Dump every slice (the band is only ~13 slices) so the user
        # can see exactly where the cross-section is too thin.
        for z, a, s in rows:
            tag_a = " " if a >= COXA_LINK_BRIDGE_AREA_MIN_MM2 else "A"
            tag_s = " " if s >= COXA_LINK_BRIDGE_SX_MIN_MM3   else "S"
            print(f"           [{tag_a}{tag_s}]  z = {z:6.2f}  "
                  f"area = {a:6.1f} mm^2  Sx = {s:7.1f} mm^3")
        print(f"           worst area at z = {min_area_z:.2f}, "
              f"worst Sx at z = {min_sx_z:.2f}")

    return ok


def check_flimsy_joints():
    """Flag every printed part that has a thin / under-strength region.

    Implementation: 3D Euclidean distance transform on a voxelised
    occupancy grid at ``FLIMSY_VOXEL_PITCH`` mm pitch; cluster voxels
    whose local thickness < ``MIN_PRINT_T``; ignore tiny clusters
    (< ``MIN_CLUSTER_VOX``); FAIL if the largest cluster is above
    ``MAX_FLIMSY_BUDGET_VOX``.

    The default constants treat 3 mm as the minimum healthy FDM print
    thickness (three 0.4 mm perimeters with margin), and a 200-voxel
    cluster at the 1.2 mm pitch as the largest acceptable thin
    region (~ 350 mm^3 of < 3 mm material per part).

    Additionally runs the targeted ``_check_coxa_link_bridge_joint``
    slice test that catches the recurring "top of coxa_link is not
    attached strongly to the part housing the servo" failure -- a
    bridge / wall-thickness defect the isotropic Hildebrand check
    cannot see.  See the big comment block above
    ``_check_coxa_link_bridge_joint`` for full motivation.
    """
    print(f"\n[6] Flimsy joints (local thickness < {MIN_PRINT_T:.1f} mm; "
          f"pitch={FLIMSY_VOXEL_PITCH:.1f} mm, "
          f"min cluster={MIN_CLUSTER_VOX} vox, "
          f"budget={MAX_FLIMSY_BUDGET_VOX} vox):")

    items_names = (
        "chassis_top", "chassis_bottom", 
        "servo_clamp_cap", "coxa_link",
        "femur_link", "tibia_link", "foot_boot",
        # Design B (May 2026): servo_horn_adapter dropped from the
        # flimsy-cluster sweep -- no longer in the printable-output set.
    )
    items = {name: _load_mesh(name, copy=False) for name in items_names}

    # Each part's voxelise + double-EDT pass is independent and the
    # scipy heavy lifting (``distance_transform_edt``) releases the GIL,
    # so fan the per-part compute across a thread pool.  This is the
    # single longest serial check post-embree; threading it collapses
    # its wall time to ~ the cost of the biggest single part while the
    # OTHER process-pool workers (which have already finished the small
    # checks) sit idle.  Printing + verdict aggregation below still
    # happen in DECLARATION ORDER in this thread, so the output and the
    # pass/fail result are byte-for-byte identical to the serial loop.
    n_threads = min(len(items), (os.cpu_count() or 1))
    with concurrent.futures.ThreadPoolExecutor(max_workers=n_threads) as ex:
        cluster_results = dict(zip(
            items.keys(),
            ex.map(
                lambda m: _flimsy_clusters_for_part(
                    m, FLIMSY_VOXEL_PITCH, MIN_PRINT_T, MIN_CLUSTER_VOX),
                items.values()),
        ))

    all_ok = True
    for name, mesh in items.items():
        clusters, biggest, max_t = cluster_results[name]
        ok = biggest <= MAX_FLIMSY_BUDGET_VOX
        all_ok &= ok
        n_clusters = len(clusters)
        head = (f"{n_clusters} flimsy cluster(s); "
                f"largest = {biggest:4d} vox "
                f"(budget {MAX_FLIMSY_BUDGET_VOX}); "
                f"part max thickness = {max_t:5.1f} mm")
        _label(name, ok, head)
        # Show up to the 3 worst clusters for the user to inspect.
        for cluster in clusters[:3]:
            cx, cy, cz = cluster["centroid"]
            extent = cluster["bbox_max"] - cluster["bbox_min"]
            print(f"           - {cluster['voxel_count']:4d} vox  "
                  f"min t={cluster['min_thickness']:4.2f} mm  "
                  f"centroid=({cx:+7.2f},{cy:+7.2f},{cz:+7.2f}) mm  "
                  f"bbox={extent[0]:5.1f} x {extent[1]:5.1f} "
                  f"x {extent[2]:5.1f} mm")

    # Bearing-sandwich refit (Jun 2026): the two targeted coxa_link
    # bridge probes (``_check_coxa_link_bridge_joint`` /
    # ``_check_coxa_link_bridge_yz_thickness``) are RETIRED.  They were
    # hard-wired to the LEGACY coxa topology -- a servo well dropped
    # WELL_Z_DROP_EXTRA below a lifted arm (COXA_LIFT), tied by a thin
    # bridge in the z in [well_top, COXA_LIFT] gap at x in [17,50],
    # y in [-25,-5].  The sandwich coxa has no such lifted-well/arm gap:
    # the hip FIXED side is a solid bracket placed by _joint_place and
    # the only structural neck (the inboard pad->bracket bridge) is well
    # away from that legacy window, so the probes scan empty space and
    # false-FAIL.  Structural soundness of the new coxa is covered by the
    # global per-part Hildebrand sweep above (coxa_link passes with a
    # 27 mm max thickness and its largest sub-3 mm cluster well under
    # budget).  The two helper functions are kept (unused) for reference
    # should the legacy bridge topology ever return.
    _ = (_check_coxa_link_bridge_joint, _check_coxa_link_bridge_yz_thickness)

    return all_ok


# ---------------------------------------------------------------------------
# 6b.  Thin sheets / webs (cross-section neck between thick volumes)
# ---------------------------------------------------------------------------
#
# Catches "thick block - thin sheet - thick block" topology along any
# part axis.  The standard Hildebrand check above
# (``check_flimsy_joints``) misses these because an inscribed sphere
# of radius MIN_PRINT_T/2 fits entirely inside any slab thicker than
# MIN_PRINT_T regardless of how extended the slab is in the other two
# directions.  Here we look at the per-axis 1D cross-section AREA
# profile and flag slices whose area is < NECK_AREA_FRAC times the
# max area on BOTH sides of the slice within a sliding window of
# NECK_WINDOW_MM (mm).  Voxels in flagged slices are clustered and
# reported just like the Hildebrand check.

def _axis_run_length_voxels(occ, axis):
    """For each occupied voxel, return the length (in voxels) of the
    contiguous run of occupied voxels along ``axis`` containing it.

    Implementation walks once forward and once backward along ``axis``
    accumulating run-length-from-start of the current run, then
    backs out the full run length per voxel.  O(N) in voxel count.
    """
    arr = np.moveaxis(occ, axis, -1)
    n_axis = arr.shape[-1]
    if n_axis == 0:
        return np.moveaxis(np.zeros_like(arr, dtype=np.int32), -1, axis)

    # forward[..., i] = run length of occupied voxels ending at i (inclusive),
    # 0 if empty.
    forward = np.zeros(arr.shape, dtype=np.int32)
    forward[..., 0] = arr[..., 0].astype(np.int32)
    for i in range(1, n_axis):
        forward[..., i] = np.where(arr[..., i],
                                    forward[..., i - 1] + 1,
                                    0)
    # backward[..., i] = run length of occupied voxels starting at i
    # (inclusive), 0 if empty.
    backward = np.zeros(arr.shape, dtype=np.int32)
    backward[..., n_axis - 1] = arr[..., n_axis - 1].astype(np.int32)
    for i in range(n_axis - 2, -1, -1):
        backward[..., i] = np.where(arr[..., i],
                                     backward[..., i + 1] + 1,
                                     0)
    # Run length containing voxel i = forward[i] + backward[i] - 1
    # (subtract 1 because i is counted in both).  Zero where empty.
    run_len = np.where(arr, forward + backward - 1, 0)
    return np.moveaxis(run_len, -1, axis)


def _thin_sheet_clusters_for_part(mesh, pitch, *,
                                    min_chord_mm_lower,
                                    min_chord_mm_upper,
                                    min_cluster_vox):
    """Return ``(clusters, biggest_voxel_count)`` for one part.

    Voxelize, compute the MIN DIRECTIONAL CHORD LENGTH per voxel (the
    smallest of the +/-X, +/-Y, +/-Z contiguous-run lengths through
    the voxel), then flag voxels whose min chord lies in the
    "structural-neck band" ``[min_chord_mm_lower, min_chord_mm_upper]``.
    Cluster the flagged voxels, drop clusters smaller than
    ``min_cluster_vox``, return the rest sorted by voxel count desc.

    Each cluster dict::

        {
            "voxel_count":    int,
            "thin_axes":      str -- one or more of "X"/"Y"/"Z"
                              (the axis/axes whose chord is the
                              minimum for the bulk of the cluster),
            "min_chord_mm":   float -- min over the cluster of the
                              per-voxel min directional chord,
            "max_chord_mm":   float -- max over the cluster of the
                              per-voxel min directional chord
                              (helps to see how uniformly thin the
                              cluster is),
            "centroid":       np.ndarray shape (3,)  part-local mm,
            "bbox_min":       np.ndarray shape (3,)  part-local mm,
            "bbox_max":       np.ndarray shape (3,)  part-local mm,
        }
    """
    try:
        vg = mesh.voxelized(pitch=pitch).fill()
    except Exception as exc:
        raise RuntimeError(f"voxelization failed: {exc}")

    occ = np.asarray(vg.matrix, dtype=bool)
    voxel_pitch = float(vg.pitch[0])

    if not occ.any():
        return [], 0

    # Per-voxel chord lengths along each axis (voxels).
    chord_axes_vox = [_axis_run_length_voxels(occ, axis) for axis in range(3)]

    # min directional chord per voxel (voxels).  Where the voxel is
    # empty all axes have run length 0; for occupied voxels each axis
    # has run length >= 1 so the minimum is meaningful.
    stacked = np.stack(chord_axes_vox, axis=0)              # (3, X, Y, Z)
    min_chord_vox = stacked.min(axis=0)                     # (X, Y, Z)
    min_chord_mm = min_chord_vox.astype(np.float32) * voxel_pitch

    # Voxel-level neck quantization: the chord length in voxels is an
    # integer, so the chord in mm is a multiple of the voxel pitch.
    # To catch a "6 mm slab" at pitch 1.2 we need to admit voxels with
    # 5 voxels of chord (6.0 mm exactly), so use an inclusive upper
    # bound with a half-pitch slack to absorb voxelization aliasing.
    lo_thresh_vox = int(np.ceil(min_chord_mm_lower / voxel_pitch))
    hi_thresh_vox = int(np.floor(
        (min_chord_mm_upper + 0.5 * voxel_pitch) / voxel_pitch
    ))
    thin_mask = occ & (min_chord_vox >= lo_thresh_vox) & \
                       (min_chord_vox <= hi_thresh_vox)

    if not thin_mask.any():
        return [], 0

    labeled, n_labels = label(thin_mask)
    axis_names = ("X", "Y", "Z")
    clusters = []
    for cluster_id in range(1, n_labels + 1):
        cluster_mask = labeled == cluster_id
        n_vox = int(cluster_mask.sum())
        if n_vox < min_cluster_vox:
            continue
        idx = np.argwhere(cluster_mask)
        lo_idx = idx.min(axis=0).astype(float)
        hi_idx = idx.max(axis=0).astype(float)
        centroid_idx = idx.mean(axis=0)
        centroid_world  = vg.indices_to_points(centroid_idx[None, :])[0]
        bbox_min_world  = vg.indices_to_points(lo_idx[None, :])[0]
        bbox_max_world  = vg.indices_to_points(hi_idx[None, :])[0]

        # For the cluster's voxels figure out which axis (or axes) is
        # the binding (smallest-chord) one most often.
        per_axis_is_min = np.zeros(3, dtype=np.int64)
        for ax in range(3):
            ax_vals = chord_axes_vox[ax][cluster_mask]
            min_vals = min_chord_vox[cluster_mask]
            per_axis_is_min[ax] = int((ax_vals == min_vals).sum())
        max_count = per_axis_is_min.max()
        triggered_axes = "".join(
            axis_names[ax] for ax in range(3)
            if per_axis_is_min[ax] == max_count
        )

        cluster_min = float(min_chord_mm[cluster_mask].min())
        cluster_max = float(min_chord_mm[cluster_mask].max())

        clusters.append({
            "voxel_count":   n_vox,
            "thin_axes":     triggered_axes,
            "min_chord_mm":  cluster_min,
            "max_chord_mm":  cluster_max,
            "centroid":      centroid_world,
            "bbox_min":      bbox_min_world,
            "bbox_max":      bbox_max_world,
        })

    clusters.sort(key=lambda c: -c["voxel_count"])
    biggest = clusters[0]["voxel_count"] if clusters else 0
    return clusters, biggest


def check_thin_sheets(extra_items=None):
    """Flag parts that have a "thick block - THIN SHEET - thick block"
    structural topology along any axis -- the failure mode the
    Hildebrand max-inscribed-ball check above cannot see because an
    inscribed sphere happily fits inside any slab thicker than
    MIN_PRINT_T regardless of how extended the slab is.

    Algorithm (anisotropic Hildebrand variant): voxelize the part,
    compute the min directional chord length per occupied voxel
    (the smallest of the +/-X, +/-Y, +/-Z run lengths containing
    that voxel), then flag voxels whose min chord lies in the
    "structural-neck band" ``[MIN_SHEET_T_LOWER, MIN_SHEET_T]``.
    Cluster the flagged voxels (8-connectivity), drop clusters smaller
    than MIN_SHEET_CLUSTER_VOX, FAIL if the largest cluster exceeds
    MAX_SHEET_BUDGET_VOX.

    Restricted to ``THIN_SHEET_PARTS`` (see big comment near the
    constants).
    """
    print(f"\n[6b] Thin sheets ("
          f"chord band {MIN_SHEET_T_LOWER:.1f}-{MIN_SHEET_T:.1f} mm "
          f"(+1 vox slack -> {MIN_SHEET_T_UPPER:.1f} mm); "
          f"pitch={FLIMSY_VOXEL_PITCH:.1f} mm, "
          f"min cluster={MIN_SHEET_CLUSTER_VOX} vox, "
          f"budget={MAX_SHEET_BUDGET_VOX} vox):")

    items = {
        "coxa_link":   lambda: _load_mesh("coxa_link", copy=False),
        "femur_link":  lambda: _load_mesh("femur_link", copy=False),
        "tibia_link":  lambda: _load_mesh("tibia_link", copy=False),
    }
    if extra_items:
        items.update(extra_items)

    all_ok = True
    for name in THIN_SHEET_PARTS:
        if name not in items:
            continue
        mesh = items[name]()
        clusters, biggest = _thin_sheet_clusters_for_part(
            mesh, FLIMSY_VOXEL_PITCH,
            min_chord_mm_lower=MIN_SHEET_T_LOWER,
            min_chord_mm_upper=MIN_SHEET_T_UPPER,
            min_cluster_vox=MIN_SHEET_CLUSTER_VOX)
        ok = biggest <= MAX_SHEET_BUDGET_VOX
        all_ok &= ok
        n_clusters = len(clusters)
        head = (f"{n_clusters} sheet cluster(s); "
                f"largest = {biggest:5d} vox "
                f"(budget {MAX_SHEET_BUDGET_VOX})")
        _label(name, ok, head)
        for cluster in clusters[:3]:
            cx, cy, cz = cluster["centroid"]
            extent = cluster["bbox_max"] - cluster["bbox_min"]
            print(f"           - {cluster['voxel_count']:5d} vox  "
                  f"chord={cluster['min_chord_mm']:4.2f}-"
                  f"{cluster['max_chord_mm']:4.2f} mm  "
                  f"(thin axes: {cluster['thin_axes']:3s})  "
                  f"centroid=({cx:+7.2f},{cy:+7.2f},{cz:+7.2f}) mm  "
                  f"bbox={extent[0]:5.1f} x {extent[1]:5.1f} "
                  f"x {extent[2]:5.1f} mm")

    return all_ok


# ---------------------------------------------------------------------------
# 7.  Workspace self-collision (full joint sweep with chassis)
# ---------------------------------------------------------------------------
#
# Motivation
# ----------
# ``check_self_collision`` above tests ONE static standing pose
# (yaw = 0, femur_pitch = STANCE_FEMUR_DEG = -25 deg, knee_pitch =
# STANCE_TIBIA_DEG = +60 deg) with the CHASSIS removed from the scene.
# Pose-dependent tibia-vs-coxa-link / tibia-vs-chassis collisions are
# therefore invisible to that check.  This sweep builds the leg
# parametrically over (yaw, femur_pitch, knee_pitch), places the
# chassis (top + bottom plates, battery holder, electronics tray) in
# the same world frame, and checks pairwise mesh overlap volume across
# the full RUNTIME joint workspace.
#
# Joint workspace -- where the limits come from
# ---------------------------------------------
#   * yaw         in [-35.0 deg, +35.0 deg]
#       Source: firmware (prototype_servo_bridge.ino YAW_LIMIT_{LO,HI}_DEG)
#               AND mujoco_prototype._leg_xml
#               (``<joint name="L{i}_yaw" range="-0.61 0.61"/>`` rad,
#               = +/-35 deg).  Was +/-51.6 deg (MuJoCo ``range="-0.90 0.90"``
#               -- removed) and +/-80 deg (firmware -- removed) before the
#               workspace audit found that beyond ~+/-30 deg the femur
#               spar collides with the coxa_bracket flange at high femur
#               pitch.
#   * femur_pitch in [-80.0 deg, +40.0 deg]
#       Source: firmware (HIP_LIMIT_{LO,HI}_DEG) AND mujoco_prototype._leg_xml
#               (``<joint name="L{i}_pitch" range="-1.40 0.70"/>`` rad,
#               = -80.2 .. +40.1 deg).  Was -80 deg .. +48.7 deg before
#               the workspace audit; the +ve side was tightened to keep
#               the femur hip pad clear of the chassis_top (which was
#               already shrunk -- the limit gives an additional safety
#               margin at the cost of ~18 deg of upward femur lift that
#               the tripod gait does not use).
#   * knee_pitch  in [-20.1 deg, +80.0 deg]
#       Source: firmware (KNEE_LIMIT_{LO,HI}_DEG) AND
#               mujoco_prototype._leg_xml ``range="-0.35 1.40"`` rad
#               (= -20.1 .. +80.2 deg).  Was -20.1 .. +106 deg (MuJoCo
#               ``range="-0.35 1.85"``) before the audit; never reached
#               by the audit sweep but tightened to +80 to mirror
#               firmware so the audit sweep covers the actual runtime
#               envelope.
# Firmware lives in ``prototype/firmware/prototype_servo_bridge/
# prototype_servo_bridge.ino``; MuJoCo limits live in
# ``prototype/mujoco_prototype.py``.  ``env_cfg.json`` does NOT add an
# extra clamp; the policy outputs residuals that the position
# actuators clip into the MuJoCo joint range.

# Workspace grid -- tight enough to find pose-dependent collisions,
# loose enough that the sweep finishes in a few minutes with the
# per-pair voxel sampler below.
WORKSPACE_YAW_DEG       = (-35.0, +35.0)
WORKSPACE_FEMUR_DEG     = (-80.0, +40.0)
WORKSPACE_KNEE_DEG      = (-20.0, +80.0)
WORKSPACE_N_YAW         = 5
WORKSPACE_N_FEMUR       = 7
WORKSPACE_N_KNEE        = 5

# Voxel-sampling pitch for the per-pose overlap computation.  Coarser
# than ``check_self_collision``'s 1.5 mm so 175 poses x ~16 pairs
# stays affordable; still fine enough to detect any mechanically
# meaningful intrusion (a 4 mm-deep tibia stab into the chassis_top
# voxelises into hundreds of voxels at 2.5 mm pitch).
WORKSPACE_VOXEL_PITCH    = 2.5

# Pose-dependent tolerances.
#   * adjacent JOINT pair (yaw / hip / knee): the existing
#     check_self_collision uses 1500 mm^3 to swallow the gear stack
#     + horn spline overlap that's intrinsic to the rotary joint
#     interface.  Keep the same budget here so adjacent pairs don't
#     spuriously fail at sweep poses far from the standing pose.
#   * everything else (non-adjacent same-leg, leg-vs-chassis, leg-vs-
#     neighbour-bracket): we expect physically ZERO overlap.  Allow
#     a 200 mm^3 artefact tolerance to absorb the voxel-sampler's
#     coarse-grid stair-step error along sharp mesh boundaries.  At
#     2.5 mm pitch one voxel = 15.6 mm^3, so 200 mm^3 ~= 13 voxels --
#     a couple voxels of stair-step rounding error per face, well
#     below any real mechanical intrusion.
WORKSPACE_JOINT_TOL      =  200.0    # mm^3 -- adjacent rotary joint.
                                      # PRINTED parts at a rotary joint
                                      # share no printed material; the gear
                                      # stack + disc horn live in
                                      # dedicated horn-stack volume (see
                                      # check_horn_stack_clearance).  So
                                      # we require ~zero printed-vs-
                                      # printed overlap here too, with
                                      # the same 200 mm^3 voxel-stair-step
                                      # artefact budget as non-adjacent
                                      # pairs.  Used to be 1500 mm^3 which
                                      # masked a ~ 800 mm^3 femur-vs-
                                      # coxa-link clash across the
                                      # entire negative-hip-pitch range.
WORKSPACE_ARTEFACT_TOL   =  200.0    # mm^3 -- non-adjacent pairs


def _pair_overlap_volume_and_centroid(a_mesh, b_mesh, pitch):
    """Voxel-sampling overlap estimator that returns ``(volume_mm3,
    centroid_in_world)`` or ``(0.0, None)`` if there is no overlap.

    Unlike ``_pair_overlap_volume``, this version computes the per-voxel
    volume as ``(Lx / n_x) * (Ly / n_y) * (Lz / n_z)`` where ``L*`` are
    the AABB-intersection extents in each axis and ``n_*`` are the
    sample counts.  This keeps the volume estimate HONEST even when the
    AABB intersection is much thinner than ``pitch`` in one axis -- the
    naive ``pitch**3`` per-voxel volume would over-report by
    ``pitch / L`` in that axis, which inflates a 0.25 mm tangential
    contact between two faces into thousands of mm^3 of "overlap" and
    masks the difference between a real punch-through and an
    FDM-tolerance graze.

    For AABB-intersections that are >> pitch on every axis (the common
    case for a deep collision) the per-voxel volume comes out to
    ~ pitch^3 exactly as the existing estimator does, so volumes are
    consistent with ``check_self_collision``'s tolerances.
    """
    a_min, a_max = a_mesh.bounds
    b_min, b_max = b_mesh.bounds
    lo = np.maximum(a_min, b_min)
    hi = np.minimum(a_max, b_max)
    if np.any(hi <= lo):
        return 0.0, None
    span = hi - lo
    n = np.maximum(2, np.ceil(span / pitch).astype(int))
    gx = np.linspace(lo[0], hi[0], int(n[0]))
    gy = np.linspace(lo[1], hi[1], int(n[1]))
    gz = np.linspace(lo[2], hi[2], int(n[2]))
    XX, YY, ZZ = np.meshgrid(gx, gy, gz, indexing="ij")
    pts = np.stack([XX.ravel(), YY.ravel(), ZZ.ravel()], axis=1)
    in_a = points_inside(a_mesh, pts)
    if in_a.sum() == 0:
        return 0.0, None
    pts_a = pts[in_a]
    in_b = points_inside(b_mesh, pts_a)
    n_overlap = int(in_b.sum())
    if n_overlap == 0:
        return 0.0, None
    voxel_vol = float(np.prod(span / n.astype(float)))
    centroid = pts_a[in_b].mean(axis=0)
    return n_overlap * voxel_vol, centroid


def _build_chassis_world(reference_leg_az_rad):
    """Return the four chassis-fixed parts placed in the same world
    frame ``_build_standing_leg`` uses (leg-local z = 0 at the bracket
    flange BOTTOM, which is the chassis-bottom-plate centre plane).

    Z stack mirrors ``build_prototype_assembly._body_frame_parts`` and
    ``_body_battery_parts``:
        chassis_bottom centre   z = 0
        chassis_top centre      z = CHASSIS_TOP_CENTRE_Z = 35
        chassis_top top face     z = CHASSIS_TOP_TOP_Z = 36 (2 mm plate)
        (printed decks retired Aug 2026 — magnet hex stack is visual-only)

    Also includes a NEIGHBOUR coxa_bracket at azimuth a + pi/3 so the
    sweep can detect tibia / femur swing into the next leg's bracket
    at extreme +yaw.
    """
    parts = {}

    bot = _load_mesh("chassis_assembled")
    parts["chassis_bottom"] = bot

    top = _load_mesh("chassis_top")
    top.apply_translation([0.0, 0.0, hp.CHASSIS_TOP_CENTRE_Z])
    parts["chassis_top"] = top

    # Aug 2026: printed deck trays / carapace RETIRED (magnet hex stack).
    # Workspace sweep only needs the chassis plates as fixed obstacles here.

    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a_n = reference_leg_az_rad + np.pi / 3.0
    edge_mid_n = np.array([apothem * np.cos(a_n),
                            apothem * np.sin(a_n),
                            0.0])

    return parts


def _build_workspace_leg(yaw_deg, femur_pitch_deg, knee_pitch_deg,
                          leg_azimuth_rad,
                          templates):
    """Place the four leg parts at the requested (yaw, femur, knee)
    pose, using the same kinematic chain as ``_build_standing_leg``
    /``build_prototype_assembly._build_leg``.

    ``templates`` is a dict of pre-built ``make_*`` meshes that we
    ``.copy()`` per pose so we don't pay the boolean-CSG cost (~ 200
    ms per leg part) once per grid sample.

    Joint convention (mirrors ``mujoco_prototype._leg_xml`` and
    ``build_prototype_assembly._build_leg``):

        yaw_rad      : rotation about leg-local +Z (vertical), positive
                       = leg's outboard direction rotates from +X to +Y
                       (toward the NEXT leg counter-clockwise).
        femur_pitch  : rotation about leg-local +Y (hip-pitch axis).
                       -25 deg = standing pose (knee lifted UP).
        knee_pitch   : rotation about femur-local +Y (knee-pitch axis).
                       +60 deg = standing pose (tibia angled DOWN from
                       the lifted knee toward the foot).
    """
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = leg_azimuth_rad
    edge_mid = np.array([apothem * np.cos(a),
                          apothem * np.sin(a),
                          0.0])
    z_hat = np.array([0.0, 0.0, 1.0])

    # Design B (May 2026): yaw output stack collapsed to HORN_STACK_H
    # = PLASTIC_HORN_H = 5 mm now that the printed servo_horn_adapter
    # has been retired.
    yaw_output_z = hp.CHASSIS_YAW_OUTPUT_Z
    hip_drop = hp.COXA_HIP_DROP
    hip_joint_local = np.array(hp.COXA_HIP_ANCHOR)
    # Bearing-sandwich refit (Jun 2026): femur / tibia local origins are
    # the disc-horn-top ON the joint axis -- no axial pad offset.
    PAD_AXIS_OFFSET = np.array([0.0, 0.0, 0.0])

    yaw_rad = np.deg2rad(yaw_deg)
    p  = np.deg2rad(femur_pitch_deg)
    pt = np.deg2rad(femur_pitch_deg + knee_pitch_deg)
    Ry_p = rotation_matrix(p, [0, 1, 0])[:3, :3]
    knee_joint_local = hip_joint_local + Ry_p @ np.array(
        [hp.FEMUR_LENGTH, 0.0, 0.0])

    R_a    = rotation_matrix(a,       [0, 0, 1])
    R_yaw  = rotation_matrix(yaw_rad, [0, 0, 1])
    yaw_output_world = edge_mid + yaw_output_z * z_hat

    parts = {}

    cl = templates["coxa_link"].copy()
    cl.apply_transform(R_yaw)
    cl.apply_transform(R_a)
    cl.apply_translation(yaw_output_world)
    parts["coxa_link"] = cl

    fl = templates["femur_link"].copy()
    fl.apply_transform(rotation_matrix(p, [0, 1, 0]))
    fl.apply_translation(hip_joint_local + PAD_AXIS_OFFSET)
    fl.apply_transform(R_yaw)
    fl.apply_transform(R_a)
    fl.apply_translation(yaw_output_world)
    parts["femur_link"] = fl

    tl = templates["tibia_link"].copy()
    tl.apply_transform(rotation_matrix(pt, [0, 1, 0]))
    tl.apply_translation(knee_joint_local + PAD_AXIS_OFFSET)
    tl.apply_transform(R_yaw)
    tl.apply_transform(R_a)
    tl.apply_translation(yaw_output_world)
    parts["tibia_link"] = tl

    return parts


def _ws_clamp_caps_world(yaw_deg, femur_pitch_deg, leg_azimuth_rad):
    """Place the two sandwich-joint clamp caps in the SAME world frame as
    ``_build_workspace_leg`` for the given pose.

    The caps are FIXED servo-cradle parts that the MOVING yoke sweeps past, so
    they ride their PARENT link -- the hip cap is fixed to the coxa (moves with
    yaw + leg azimuth only) and the knee cap is fixed to the femur (moves with
    yaw + leg + femur-pitch).  Modelled in the well-local frame (like
    ``make_servo_clamp_cap`` / ``_place_servo_clamp_caps``); we prepend the
    per-joint ``_joint_place`` and then apply the SAME kinematic chain the
    parent link uses so the cap stays glued to its cradle through the ROM.

    Returns ``{"hip_clamp_cap": mesh, "knee_clamp_cap": mesh}``.  Without these,
    the sweep only checks the merged links and MISSES the moving-yoke-vs-fixed-
    clamp-cap clash (the cap is not part of any link proxy)."""
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = leg_azimuth_rad
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    z_hat = np.array([0.0, 0.0, 1.0])
    yaw_output_world = edge_mid + hp.CHASSIS_YAW_OUTPUT_Z * z_hat
    hip_joint_local = np.array(hp.COXA_HIP_ANCHOR)

    R_a = rotation_matrix(a, [0, 0, 1])
    R_yaw = rotation_matrix(np.deg2rad(yaw_deg), [0, 0, 1])
    p = np.deg2rad(femur_pitch_deg)

    # Hip cap -- glued to the coxa (yaw + leg azimuth).
    M_hip = hp._joint_place(hp.COXA_HIP_ANCHOR, (1, 0, 0), hp.LEG_PITCH_AXIS)
    hip_cap = hp.make_servo_clamp_cap()
    hip_cap.apply_transform(M_hip)
    hip_cap.apply_transform(R_yaw)
    hip_cap.apply_transform(R_a)
    hip_cap.apply_translation(yaw_output_world)

    # Knee cap -- glued to the femur (yaw + leg + femur-pitch).
    M_knee = hp._joint_place((hp.FEMUR_LENGTH, 0.0, 0.0), (1, 0, 0),
                             hp.LEG_PITCH_AXIS)
    knee_cap = hp.make_servo_clamp_cap()
    knee_cap.apply_transform(M_knee)
    knee_cap.apply_transform(rotation_matrix(p, [0, 1, 0]))
    knee_cap.apply_translation(hip_joint_local)
    knee_cap.apply_transform(R_yaw)
    knee_cap.apply_transform(R_a)
    knee_cap.apply_translation(yaw_output_world)
    return {"hip_clamp_cap": hip_cap, "knee_clamp_cap": knee_cap}


# Which MOVING link each fixed clamp cap is checked against through the ROM.
# The cap's OWN parent link is excluded (the cap is bolted flush to that
# cradle -- a designed contact guarded by check_clamp_cap_interference); we
# guard the cap against the links that rotate PAST it.
_WS_CLAMP_CAP_PAIRS = {
    "hip_clamp_cap":  ("femur_link", "tibia_link"),
    "knee_clamp_cap": ("coxa_link", "tibia_link"),
}


# Joint-adjacency table for the workspace sweep.  Pair tolerances:
#   * adjacent JOINT pair -> WORKSPACE_JOINT_TOL  (gear stack / horn
#     interface is allowed to register some mm^3 of overlap intrinsic
#     to the rotary joint).
#   * everything else  -> WORKSPACE_ARTEFACT_TOL  (physically zero
#     overlap expected).
_WS_JOINT_PAIRS = {
    ("coxa_link",    "femur_link"),
    ("femur_link",   "tibia_link"),
}

# Which (leg part, static part) pairs to test per pose.  This is the
# full set of "could plausibly collide somewhere in the workspace"
# combinations the existing single-pose check misses.
_WS_DYNAMIC_NAMES = ("coxa_link", "femur_link", "tibia_link")
_WS_INTRA_LEG_STATIC = ()  # no chassis-fixed leg-static parts after Design F
_WS_CHASSIS_STATIC = (
    "chassis_top",
    "chassis_bottom",
)


def _ws_pair_kind(dynamic_name, static_name):
    pair = (dynamic_name, static_name)
    if pair in _WS_JOINT_PAIRS or (static_name, dynamic_name) in _WS_JOINT_PAIRS:
        return "joint"
    if static_name in _WS_CHASSIS_STATIC:
        return "chassis"
    return "non-adj"


# Worker-side cache for the workspace sweep's chassis + leg templates.
# Each worker re-derives these from the cached master meshes on first
# use so the (yaw, femur_pitch, knee_pitch) pose function does not pay
# the chassis-build cost more than ONCE per worker.
_WS_WORKER_STATE: dict = {}


def _ws_get_chassis_and_templates(leg_az: float):
    key = float(leg_az)
    cached = _WS_WORKER_STATE.get(key)
    if cached is not None:
        return cached
    chassis = _build_chassis_world(key)
    templates = {
        "coxa_link":    _load_mesh("coxa_link",    copy=False),
        "femur_link":   _load_mesh("femur_link",   copy=False),
        "tibia_link":   _load_mesh("tibia_link",   copy=False),
    }
    _WS_WORKER_STATE[key] = (chassis, templates)
    return chassis, templates


def _workspace_pose_failures(pose, leg_az):
    """Return the list of pair-failure dicts for one (yaw, femur, knee)
    pose.  Pure function: side-effects are limited to populating the
    per-worker ``_WS_WORKER_STATE`` cache on first call.  Used both
    serially in the main process and in process-pool workers."""
    yaw_deg, f_deg, k_deg = pose
    chassis, templates = _ws_get_chassis_and_templates(leg_az)
    leg = _build_workspace_leg(yaw_deg, f_deg, k_deg,
                                 leg_azimuth_rad=leg_az,
                                 templates=templates)

    pose_failures = []

    leg_names = list(leg.keys())
    for i, na in enumerate(leg_names):
        for nb in leg_names[i + 1:]:
            vol, centroid = _pair_overlap_volume_and_centroid(
                leg[na], leg[nb], WORKSPACE_VOXEL_PITCH)
            pair_kind = _ws_pair_kind(na, nb)
            tol = (WORKSPACE_JOINT_TOL if pair_kind == "joint"
                    else WORKSPACE_ARTEFACT_TOL)
            if vol > tol:
                pose_failures.append({
                    "pose":     (yaw_deg, f_deg, k_deg),
                    "pair":     (na, nb),
                    "kind":     pair_kind,
                    "vol":      vol,
                    "centroid": centroid,
                })

    for dyn_name in _WS_DYNAMIC_NAMES:
        dyn_mesh = leg[dyn_name]
        for stat_name, stat_mesh in chassis.items():
            vol, centroid = _pair_overlap_volume_and_centroid(
                dyn_mesh, stat_mesh, WORKSPACE_VOXEL_PITCH)
            if vol > WORKSPACE_ARTEFACT_TOL:
                pose_failures.append({
                    "pose":     (yaw_deg, f_deg, k_deg),
                    "pair":     (dyn_name, stat_name),
                    "kind":     "chassis",
                    "vol":      vol,
                    "centroid": centroid,
                })

    # Fixed servo CLAMP CAPS vs the moving yoke (sandwich-joint ROM guard).
    # The cap rides its parent cradle; the driven yoke rotates past it, so this
    # is the pair that catches the femur/tibia-yoke-into-clamp-cap clash the
    # merged-link sweep otherwise misses.
    caps = _ws_clamp_caps_world(yaw_deg, f_deg, leg_az)
    for cap_name, cap_mesh in caps.items():
        for dyn_name in _WS_CLAMP_CAP_PAIRS[cap_name]:
            vol, centroid = _pair_overlap_volume_and_centroid(
                leg[dyn_name], cap_mesh, WORKSPACE_VOXEL_PITCH)
            if vol > WORKSPACE_ARTEFACT_TOL:
                pose_failures.append({
                    "pose":     (yaw_deg, f_deg, k_deg),
                    "pair":     (dyn_name, cap_name),
                    "kind":     "clamp-cap",
                    "vol":      vol,
                    "centroid": centroid,
                })

    return pose_failures


def _ws_pose_failures_kwargs(args):
    """Picklable star-helper for ProcessPoolExecutor.map(): unpacks
    ``(pose, leg_az)`` into the keyword call.  Returns
    ``(failures, mismatches)`` -- callers carry the pose order
    separately so we don't need completion-order keys.  The
    ``mismatches`` half captures points_inside disagreements
    accumulated under --inside-mode=both so the parent can aggregate
    them in the final summary."""
    global _inside_mismatches
    pose, leg_az = args
    _set_inside_check_context(
        f"Workspace self-collision pose={pose}")
    _inside_mismatches = []
    try:
        failures = _workspace_pose_failures(pose, leg_az)
    finally:
        my_mismatches = list(_inside_mismatches)
        _inside_mismatches = []
    return (failures, my_mismatches)


def check_workspace_self_collision(*, n_yaw=None, n_femur=None, n_knee=None,
                                     verbose=False, pool=None):
    """Sweep (yaw, femur_pitch, knee_pitch) on a coarse grid through the
    runtime joint workspace, build the leg + chassis at each pose, and
    flag every pose where any leg part overlaps any static part beyond
    a small artefact tolerance.

    The check FAILS as soon as the count of failing poses is non-zero.
    All failing poses are reported (not just the first one) so the
    next geometry / limit iteration can see the full failure
    envelope.

    ``pool`` is an optional ``concurrent.futures.Executor``.  When
    given, per-pose work is dispatched to it in submission order; the
    failures list is aggregated in pose-declaration order so the
    printed output is identical to the serial run.  When ``None`` the
    poses run in the calling process.
    """
    n_yaw   = n_yaw   if n_yaw   is not None else WORKSPACE_N_YAW
    n_femur = n_femur if n_femur is not None else WORKSPACE_N_FEMUR
    n_knee  = n_knee  if n_knee  is not None else WORKSPACE_N_KNEE

    yaw_samples   = np.linspace(*WORKSPACE_YAW_DEG,   n_yaw)
    femur_samples = np.linspace(*WORKSPACE_FEMUR_DEG, n_femur)
    knee_samples  = np.linspace(*WORKSPACE_KNEE_DEG,  n_knee)

    n_poses = n_yaw * n_femur * n_knee
    print(f"\n[7] Workspace self-collision sweep "
          f"(yaw in {WORKSPACE_YAW_DEG} deg x {n_yaw}, "
          f"femur in {WORKSPACE_FEMUR_DEG} deg x {n_femur}, "
          f"knee in {WORKSPACE_KNEE_DEG} deg x {n_knee} "
          f"= {n_poses} poses; pitch={WORKSPACE_VOXEL_PITCH} mm; "
          f"tol non-adj={WORKSPACE_ARTEFACT_TOL:.0f} mm^3, "
          f"joint={WORKSPACE_JOINT_TOL:.0f} mm^3):")

    leg_az = 0.5 * np.pi / 3.0   # same a = pi/6 as _build_standing_leg

    print("  Building chassis + leg-part templates (once) ...")
    # Warm the parent's own per-leg_az cache so a --serial run pays
    # the chassis-build cost up front (mirrors the worker behaviour).
    _ws_get_chassis_and_templates(leg_az)

    # Always include the canonical standing pose so we can confirm the
    # known-good pose stays clean even with the chassis in the scene.
    standing_pose = (0.0,
                     float(hp.STANCE_FEMUR_DEG),
                     float(hp.STANCE_TIBIA_DEG))
    pose_iter = [standing_pose]
    seen_poses = {standing_pose}
    for yaw_deg in yaw_samples:
        for f_deg in femur_samples:
            for k_deg in knee_samples:
                key = (float(yaw_deg), float(f_deg), float(k_deg))
                if key in seen_poses:
                    continue
                seen_poses.add(key)
                pose_iter.append(key)

    pose_args = [(pose, leg_az) for pose in pose_iter]

    if pool is None:
        # Parent-process / --serial path: mismatch records accumulate
        # directly in the parent's ``_inside_mismatches`` global.
        _set_inside_check_context(WORKSPACE_CHECK_NAME)
        results = [_workspace_pose_failures(pose, leg_az)
                    for pose in pose_iter]
    else:
        # chunksize hand-tuned: a chunk should be large enough to
        # amortise the pickle / dispatch cost but small enough that
        # straggler poses (extreme-angle leg builds can take 2x longer
        # than nominal) don't pin the slowest worker.  ~6 keeps the
        # worker queue well-fed without ballooning per-task overhead.
        raw_results = list(pool.map(_ws_pose_failures_kwargs, pose_args,
                                      chunksize=6))
        results = []
        for pose_failures, pose_mismatches in raw_results:
            results.append(pose_failures)
            if pose_mismatches:
                _inside_mismatches.extend(pose_mismatches)

    failures = []
    standing_failed = False

    for idx, (pose, pose_failures) in enumerate(zip(pose_iter, results)):
        yaw_deg, f_deg, k_deg = pose
        pose_label = (f"yaw={yaw_deg:+6.1f} femur={f_deg:+6.1f} "
                       f"knee={k_deg:+6.1f}")
        if pose_failures:
            failures.extend(pose_failures)
            if pose == standing_pose:
                standing_failed = True
            if verbose:
                print(f"  POSE {idx:3d}  FAIL  {pose_label}  "
                      f"({len(pose_failures)} pair(s))")
        elif verbose:
            print(f"  POSE {idx:3d}  ok    {pose_label}")

    # Always confirm the standing pose explicitly.
    _label("standing-pose still clean with chassis in scene",
            not standing_failed,
            ("STANDING POSE NOW FAILS"
              if standing_failed
              else "yaw=0 femur=-25 knee=+60 -- no chassis intrusion"))

    if not failures:
        return _label("workspace sweep self-collision",
                       True,
                       f"all {len(pose_iter)} poses clean")

    # Summarise + list every failing pair.  Sort by descending overlap
    # so the first lines are the worst offenders.
    failures.sort(key=lambda f: -f["vol"])
    print(f"  Found {len(failures)} failing pair(s) across "
          f"{len({tuple(f['pose']) for f in failures})} pose(s):")
    for f in failures:
        yaw_deg, fd, kd = f["pose"]
        cx, cy, cz = (f["centroid"] if f["centroid"] is not None
                       else (float("nan"),) * 3)
        a_name, b_name = f["pair"]
        print(f"           "
              f"yaw={yaw_deg:+6.1f} fem={fd:+6.1f} knee={kd:+6.1f}  "
              f"{a_name:>14s} vs {b_name:<22s} ({f['kind']:>7s})  "
              f"vol={f['vol']:7.1f} mm^3  "
              f"centroid=({cx:+7.1f},{cy:+7.1f},{cz:+7.1f}) mm")

    return _label("workspace sweep self-collision",
                   False,
                   f"{len(failures)} failing pair(s) -- see list above")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _optional_arm_checks():
    """Run the optional-arm verification suite via prototype/arm/integrate.py.

    Two extra checks are appended to the verification list:

    * ``[4b]`` voxel overlap between each arm part (in neutral pose,
      bolted to chassis-top centre) and the chassis-top plate / six
      legs in standing pose.
    * ``[6b]`` flimsy-joint check over the 5 new arm parts
      (`arm_base_bracket`, `wrist_adapter`, `gripper_base`,
      `gripper_jaw_left`, `gripper_jaw_right`).

    Returns a list of (label, ok) tuples to be appended to ``results``.
    """
    _arm_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "arm")
    if _arm_dir not in sys.path:
        sys.path.insert(0, _arm_dir)
    # Import `prototype/arm/integrate.py` directly by file path so we
    # bypass the `arm` name collision (the sibling `prototype/arm/arm.py`
    # module shadows the `arm/` package on sys.path).
    try:
        import importlib.util  # noqa: WPS433
        _spec = importlib.util.spec_from_file_location(
            "arm_integrate", os.path.join(_arm_dir, "integrate.py"))
        if _spec is None or _spec.loader is None:
            raise ImportError("could not locate arm/integrate.py")
        arm_integrate = importlib.util.module_from_spec(_spec)
        _spec.loader.exec_module(arm_integrate)
    except Exception as exc:
        print(f"  WARN: optional arm import failed ({exc!r}); "
              f"skipping arm verification.")
        return []

    interference_ok = arm_integrate.check_arm_interference(
        pair_overlap_fn=_pair_overlap_volume,
        label_fn=_label,
    )
    flimsy_ok = arm_integrate.verify_arm_parts(
        min_print_t=MIN_PRINT_T,
        pitch=FLIMSY_VOXEL_PITCH,
        min_cluster_vox=MIN_CLUSTER_VOX,
        max_flimsy_budget_vox=MAX_FLIMSY_BUDGET_VOX,
        flimsy_cluster_fn=_flimsy_clusters_for_part,
        label_fn=_label,
    )
    return [
        ("Arm-vs-chassis/leg interference", interference_ok),
        ("Arm flimsy joints",               flimsy_ok),
    ]


# ---------------------------------------------------------------------------
# June 2026 disc-horn switch: direct-to-disc-horn pad pattern
# ---------------------------------------------------------------------------
#
# The link's pad now bolts DIRECTLY onto the 20 mm aluminum 25T disc
# horn (no printed servo_horn_adapter disc in the stack; the earlier
# plastic 4-arm X-horn scheme is also retired).  Each driven link MUST
# therefore carry:
#
#   1. A 4 x M3-clearance hole pattern on a DISC_HORN_BOLT_PCD = 14 mm
#      bolt circle, drilled through the pad's mating face along the
#      joint axis (link +Z for coxa_link; link +Y for femur_link /
#      tibia_link).  Phi DISC_HORN_BOLT_OD = 3.4 mm (M3 clearance with
#      0.2 mm FDM print tolerance) -- the M3 SHCS threads into the
#      aluminium disc's M3 TAPPED holes (the disc is the thread-
#      engagement medium).  The retired plastic X-horn scheme used
#      4 x M2 self-tap holes on a 20.8 mm PCD instead.  See
#      hexapod_prototype.py DISC_HORN_BOLT_* docstring.
#   2. (coxa_link ONLY) A central Phi DISC_HORN_COLLAR_OD = 9 mm
#      cylindrical bore DISC_HORN_COLLAR_DEPTH = 2 mm deep cut into the
#      pad's mating face, so the disc horn's raised central spline
#      collar + M3 centre-screw head is cleared and the disc seats
#      flat below the pad.  (The retired plastic horn used a wider
#      Phi 16 mm x 1.2 mm hub recess.)
#      May 2026 solid-pad simplification: the femur_link and
#      tibia_link historically dropped the wide recess -- their
#      hip/knee pads mate to the disc horn with a flat face.  They now
#      carry the same small spline-collar bore as the coxa cap; the
#      bolt-pattern check only probes the coxa_link's bore here.
#
# This check confirms the 4 bolt holes on every driven part and the
# central recess on the coxa_link by sampling small voxel patches at
# each expected position and requiring the link mesh to be VOID
# there.  A short pillar of pad material at the bolt-PCD ring or at
# the central hub indicates the pad was not drilled / recessed
# correctly.

# Tolerance: small voxel-grid step-noise budget per probe.  Each probe
# samples a Phi (hole_OD/2 - small) x depth column of voxels and
# expects ZERO of them to be inside the link mesh.  A real failure
# (= bolt pattern missing entirely, or recess depth set to 0) leaves
# 100+ voxel samples inside the part.
HORN_PATTERN_VOX_TOL = 5    # voxel hits per probe -- voxel-noise floor


def _probe_void_cylinder(mesh: trimesh.Trimesh,
                          centre: np.ndarray,
                          axis: str,
                          radius: float,
                          depth: float,
                          n_samples: int = 64) -> int:
    """Sample a cylindrical probe volume (radius x depth) along the
    given axis ("x" | "y" | "z"), centred at *centre* (where the
    cylinder's mid-Z lies); return how many sample points fall INSIDE
    *mesh*.  Used to assert that a region of the part is VOID."""
    rng = np.random.default_rng(seed=42)
    # Sample uniformly within a cylinder of (radius, depth) along the
    # axis.  Cylinder mid-point is at the origin in axis-local frame
    # then translated to *centre*.
    r = radius * np.sqrt(rng.random(n_samples))
    theta = 2.0 * np.pi * rng.random(n_samples)
    h = (rng.random(n_samples) - 0.5) * depth
    local_a = h                # along axis
    local_b = r * np.cos(theta)
    local_c = r * np.sin(theta)
    pts = np.zeros((n_samples, 3))
    if axis == "x":
        pts[:, 0] = local_a
        pts[:, 1] = local_b
        pts[:, 2] = local_c
    elif axis == "y":
        pts[:, 0] = local_b
        pts[:, 1] = local_a
        pts[:, 2] = local_c
    elif axis == "z":
        pts[:, 0] = local_b
        pts[:, 1] = local_c
        pts[:, 2] = local_a
    else:
        raise ValueError(f"axis must be x|y|z, got {axis!r}")
    pts = pts + np.asarray(centre)
    inside = points_inside(mesh, pts)
    return int(inside.sum())


def check_horn_pattern_in_pad():
    """Verify that each driven link's pad has:
      a) 4 M3 clearance holes on the DISC_HORN_BOLT_PCD = 14 mm bolt
         circle, drilled through the full pad thickness, and
      b) (coxa_link only) a central Phi DISC_HORN_COLLAR_OD = 9 mm x
         DISC_HORN_COLLAR_DEPTH = 2 mm spline-collar clearance bore on
         the pad's mating face.

    June 2026 disc-horn switch: the joints drive a 20 mm aluminum 25T
    disc horn, so the bolts are M3 (Phi 3.4 mm clearance) into the
    disc's M3 tapped holes -- see ``DISC_HORN_BOLT_OD`` in
    ``hexapod_prototype.py``.  The probe radius below is sized off
    ``DISC_HORN_BOLT_OD`` so the verifier tracks any future change to the
    bolt standard without a code edit.

    The femur_link's hip pad and the tibia_link's knee pad now carry
    the same small spline-collar bore as the coxa cap (the disc seats
    flat on all three), but the bolt-pattern check only probes the
    coxa_link's collar bore (``has_recess=True``); the femur/tibia
    bores are validated indirectly via mating-face contact.
    """
    print(f"\n[5e] Horn-pattern in driven link pads "
          f"(Phi {hp.DISC_HORN_BOLT_OD:.1f} mm holes on PCD "
          f"{hp.DISC_HORN_BOLT_PCD:.1f} mm; central hub recess on "
          f"coxa_link only):")

    # The probe radius is a hair smaller than the actual clearance
    # hole / recess radius so voxel stair-step artefacts on the cut's
    # curved boundary don't pollute the "is the volume void" answer.
    # With DISC_HORN_BOLT_OD = 3.4 mm the bolt probe shrinks to ~ 1.5 mm
    # radius (Phi 3.0 mm probe) which still resolves cleanly against
    # the verifier's 1.5 mm voxel pitch -- a missing hole produces a
    # solid pillar of pad material at the probe position and registers
    # as ~ 40-100 hits (well above HORN_PATTERN_VOX_TOL * 4 = 20).
    bolt_probe_r   = hp.DISC_HORN_BOLT_OD / 2.0 - 0.2     # ~1.5 mm at M3
    recess_probe_r = hp.DISC_HORN_COLLAR_OD / 2.0 - 0.5    # ~4.0 mm

    cases = [
        # (name, mesh, pad_axis, mating_face_coord, mating_normal_sign,
        #  has_recess)
        # ``pad_axis``: which axis the bolts are along ("y" for the
        #   femur/tibia knee/hip pads, "z" for the coxa_link's
        #   pedestal-bottom mating face).
        # ``mating_face_coord``: the position of the mating face on
        #   *pad_axis*.  May 2026 collinear-pad refactor: for the
        #   femur/tibia the mating face is now the link's NEW local
        #   origin (y = 0); pre-refactor it was at y = HORN_STACK_H.
        #   For coxa_link, the pad's bottom is still at z = 0.
        # ``mating_normal_sign``: which way the recess opens from the
        #   mating face.  Femur/tibia mating face faces -Y (toward the
        #   horn below); the (former) recess opened in -Y, removing
        #   material at y in [mate, mate + RECESS_DEPTH].  Coxa_link
        #   mating face faces -Z (toward the horn below); the recess
        #   opens DOWNWARD in -Z, removing material at z in [0,
        #   +RECESS_DEPTH].  In both cases the recess probe centre
        #   sits +RECESS_DEPTH/2 INTO the pad along +pad_axis.
        # ``has_recess``: True iff the bolt-pattern check probes the
        #   central Phi DISC_HORN_COLLAR_OD x DISC_HORN_COLLAR_DEPTH
        #   spline-collar bore.  Only the coxa_link's bore is probed
        #   here; the femur/tibia bores are checked via mating-face
        #   contact.
        ("coxa_link  (yaw joint)",        _load_mesh("coxa_link",
                                                       copy=False),
         "z", hp.YAW_HUB_BOSS_BOT_Z,        +1.0,  True),
        ("femur_link (hip-pitch joint)",  _load_mesh("femur_link",
                                                       copy=False),
         "y", 0.0,                          +1.0,  False),
        ("tibia_link (knee-pitch joint)", _load_mesh("tibia_link",
                                                       copy=False),
         "y", 0.0,                          +1.0,  False),
    ]

    # The pad's full thickness in the pad-axis direction.  For
    # femur/tibia: hip_pad/knee_pad spans y in [HORN_STACK_H,
    # HORN_STACK_H + LINK_THICKNESS].  For coxa_link: the pedestal +
    # hub stack spans z in [0, COXA_LIFT + hub_t] -- much taller, but
    # for the bolt-pattern probe we only need to confirm a clear hole
    # at the bottom of the pedestal, so a 4 mm probe is enough.
    BOLT_PROBE_DEPTH = 4.0    # probe a 4 mm-long column at the
                               # mating face; matches typical pad
                               # thickness.

    all_ok = True
    for name, mesh, axis, mate, normal_sign, has_recess in cases:
        # ---- (a) 4 M3 bolt holes on the DISC_HORN_BOLT_PCD circle ----
        bolt_misses = 0
        for ang in hp.DISC_HORN_BOLT_ANGLES_RAD:
            # Bolt-circle position in the pad's transverse plane.
            tx = hp.DISC_HORN_BOLT_PCD / 2.0 * np.cos(ang)
            ty = hp.DISC_HORN_BOLT_PCD / 2.0 * np.sin(ang)
            # Probe centre sits half-way along the bolt's path into
            # the pad (the bolt enters at the mating face and exits
            # the far side, depth = LINK_THICKNESS for femur/tibia or
            # the pedestal+hub stack for coxa_link).
            centre_axis = mate + normal_sign * (BOLT_PROBE_DEPTH / 2.0)
            if axis == "y":
                centre = np.array([tx, centre_axis, ty])
            elif axis == "z":
                centre = np.array([tx, ty, centre_axis])
            else:
                centre = np.array([centre_axis, tx, ty])
            hits = _probe_void_cylinder(mesh, centre, axis,
                                          bolt_probe_r,
                                          BOLT_PROBE_DEPTH,
                                          n_samples=48)
            bolt_misses += hits
        ok_bolts = bolt_misses <= HORN_PATTERN_VOX_TOL * 4
        all_ok &= _label(
            f"{name} :: 4 x M3 bolts on Phi {hp.DISC_HORN_BOLT_PCD} mm PCD",
            ok_bolts,
            f"hits={bolt_misses} (tol "
            f"{HORN_PATTERN_VOX_TOL * 4})",
        )

        # ---- (b) Central horn-hub recess ----
        # Only the coxa_link's bore is probed here (the femur and tibia
        # pads mate to the disc horn with a flat face plus their own
        # small spline-collar bore).  The recess starts
        # at the mating face and extends INTO the pad by
        # HORN_RECESS_DEPTH.  Probe centre sits at half-depth so the
        # entire cylinder lives strictly inside the recess volume.
        if has_recess:
            recess_centre_axis = mate + normal_sign * (
                hp.DISC_HORN_COLLAR_DEPTH / 2.0)
            if axis == "y":
                centre = np.array([0.0, recess_centre_axis, 0.0])
            else:                                            # "z"
                centre = np.array([0.0, 0.0, recess_centre_axis])
            hits = _probe_void_cylinder(mesh, centre, axis,
                                          recess_probe_r,
                                          hp.DISC_HORN_COLLAR_DEPTH * 0.8,
                                          n_samples=64)
            ok_recess = hits <= HORN_PATTERN_VOX_TOL
            all_ok &= _label(
                f"{name} :: Phi {hp.DISC_HORN_COLLAR_OD} mm "
                f"x {hp.DISC_HORN_COLLAR_DEPTH:.2f} mm spline-collar bore",
                ok_recess,
                f"hits={hits} (tol {HORN_PATTERN_VOX_TOL})",
            )

    return all_ok


# ---------------------------------------------------------------------------
# Design D (May 2026 heat-set switch): cradle servo insert pockets
# ---------------------------------------------------------------------------
#
# Each servo cradle (coxa_bracket's yaw well, coxa_link's hip-pitch
# well, femur_link's knee well) bolts the servo down via 4 VERTICAL
# M3 x 8 SHCS that thread DOWN through each servo ear into an M3 brass
# heat-set insert (McMaster 94459A130) seated in a printed Phi
# INSERT_M3_PILOT_OD = 4.0 mm pocket of depth INSERT_M3_PILOT_DEPTH =
# 6.0 mm.  Each pocket is surrounded by a Phi CRADLE_BOSS_OD = 8.0 mm
# printed boss (see ``_servo_cradle_insert_pockets`` in
# ``hexapod_prototype.py``).
#
# Each cradle shelf must satisfy THREE structural conditions per bolt
# site:
#
#   (1) The pocket is VOID along the Phi 4 mm x 6 mm cylinder so the
#       bolt + insert fit.  This is the renamed legacy "pilot-cyl"
#       probe with the new diameter / depth.
#   (2) The annular ring of plastic at radius
#       INSERT_M3_PILOT_OD/2 + CRADLE_BOSS_MIN_WALL_MM = 3.5 mm
#       around the pocket axis is SOLID at every azimuth.  This is
#       the radial-material check whose absence let the original
#       Phi 2.5 mm self-tap pilots ship with 0.00-1.50 mm of plastic
#       on one side of the bolt -- the verifier's vertical pilot
#       probe (commit b447f88) only confirmed the pilot cylinder
#       existed, not that it was surrounded by enough material
#       radially.  We probe 8 azimuths at two z planes
#       (pocket_bottom + 1 mm and pocket_top - 1 mm) and require
#       every sample to hit printed material.
#   (3) The annular ring between the pocket OD and the heat-set
#       insert OD must be SOLID before installation so the knurl has
#       plastic to displace.  We probe 8 azimuths at the midpoint of
#       the insert body (z = shelf_top - INSERT_M3_INSERT_LENGTH/2).
#
# The bracket's pockets ride a few mm below WELL_RIM_Z because the
# drop-in slot eats the well's rim down to bracket-z = -3 (see
# ``BRACKET_SHELF_DROP_MM`` in ``make_coxa_bracket``); the bracket
# case adjusts the probe's shelf_top_z accordingly so its 4 pockets
# are sampled where the bracket actually cut them.

# Legacy alias kept so external diagnostic dumps that reference the
# old constant name continue to work.  Same numeric value -- the new
# check uses the same noise-floor accounting.
CRADLE_PILOT_HOLE_VOX_TOL    = 3
CRADLE_INSERT_POCKET_VOX_TOL = 3   # voxel hits per pocket cylinder


def check_cradle_insert_pockets():
    """Verify each cradle has 2 heat-set insert sites on the -X column
    (Design E mixed-mode, May 2026).

    For each of the 6 (cradle, x = -X, y = +/- Y) HEAT-SET sites we
    probe THREE conditions:

      (1) the Phi INSERT_M3_PILOT_OD x INSERT_M3_PILOT_DEPTH POCKET
          is VOID along the bolt axis (the renamed legacy pilot-cyl
          probe);
      (2) the annular ring at radius
          INSERT_M3_PILOT_OD/2 + CRADLE_BOSS_MIN_WALL_MM around the
          pocket axis is SOLID at 8 azimuths x 2 z-planes -- the
          radial-material check that the original Phi 2.5 mm self-
          tap verifier was missing;
      (3) the annular ring between the pocket OD and the heat-set
          insert OD is SOLID at 8 azimuths so the knurl has plastic
          to displace at install time.

    The 2 +X sites per cradle are NOT probed by this check.  Under
    Design E they no longer have heat-set inserts -- they have bare
    Phi 2.5 mm self-tap pilots in the existing well-wall material
    (the Phi 8 mm heat-set boss could not coexist with the restored
    +X wire channel).  Those +X sites are documented as
    "known-thin" on the channel-facing side (~ 0.25 mm of plastic
    between the pilot and the channel), which is acceptable for an
    M3 self-tap into PLA / PETG and is enforced by
    ``check_servo_insertion_path`` rather than by a radial-material
    check.  See the INSERT_M3_SELFTAP_* block in
    ``hexapod_prototype.py`` for the full rationale.

    The cradles are tested in their UNROTATED, UNTRANSLATED local
    (well-local) frames so we don't have to undo each cradle's
    R / delta math.

    Per-case radial-wall tolerance (May 2026, commit 5.5/9)
    -------------------------------------------------------
    Each case carries a ``radial_air_frac`` value that controls how
    many of its radial / knurl-ring azimuth probes are allowed to
    fall in AIR before the case fails:

      * Legacy cradles (coxa_bracket / coxa_link / femur_link)
        carry the strict tolerance ``radial_air_frac = 0.0`` -- ALL
        azimuths around the 2 -X heat-set sites must be inside
        printed plastic.  The wire-exit L-corridor on these
        cradles stays on +X (no mirror) so the -X column has no
        channel intrusion and the bosses keep their full 360-deg
        radial wall.

      * The chassis_bottom integrated yaw cradle (added by commit
        7/9, the verifier reroute commit; not present in the
        current cases list) carries the relaxed tolerance
        ``radial_air_frac = 0.25``.  That cradle mirrors the
        wire-exit L-corridor to the -X face (so harnesses route
        radially inward toward the per-leg drop slot at chassis-
        frame +46.8, +27, +2) which puts the corridor's lateral
        leg through the -X bosses' inner azimuth band.  The
        corridor cut carves away the inboard ~25 percent of each
        -X boss circumference -- specifically the 8/32 azimuths
        in [3 pi/4, 5 pi/4] facing the cradle interior.

    Engineering rationale for the chassis_bottom relaxation
    -------------------------------------------------------
    Heat-set retention in printed plastic is dominated by the
    knurl-ring's AXIAL bite into the plastic surrounding the
    insert (the knurl teeth dig into the boss wall as the insert
    is pressed in hot, then anneal in place).  With 75 percent
    (24/32) of the boss circumference RETAINED at full
    1.5 mm-min radial wall thickness AND the chassis_bottom
    plate bonded to the boss BOTTOM via the cradle union (the
    boss grows out of the plate top face so the plate provides
    full axial retention on the boss bottom plane), losing 25
    percent of the radial knurl coverage costs approximately 25
    percent of the pull-out force -- a McMaster 94459A130 brass
    insert is rated for ~250 N pull-out at full coverage in
    PA-CF, so the channel-side reduction degrades that to
    ~190 N.  Each servo-tab clamp bolt sees < 30 N tensile load
    under the worst-case yaw-actuator stall torque (DS3225 stall
    ~25 kg-cm = 2.45 N-m, divided over 4 bolts at
    SERVO_TAB_HOLE_PCD/2 = 24.75 mm lever arm gives 24.7 N per
    bolt; doubled to 50 N for shock loading), so 190 N retention
    leaves 3.8 x safety factor on the worst-loaded insert (a 5 x
    margin is the conventional target -- close enough for a
    prototype with bolted redundancy on both -X sites).

    The relaxation is kept LOCALISED to the chassis_bottom case
    via the ``radial_air_frac`` per-case field rather than by
    lowering the global ``CRADLE_BOSS_MIN_WALL_MM`` -- the
    legacy cradles have no channel intrusion and so should
    continue to enforce the strict 0-air-flagged tolerance.

    Mirror rationale + the function-split (``_wire_exit_l_corridor``
    + ``_boot_clearance_channel``) that enables this per-cradle
    exit-face policy live in those helper functions' docstrings
    in ``hexapod_prototype.py`` and in the ``CRADLE_BOSS_*``
    constants block at the top of the same file.
    """
    print(f"\n[5e] Cradle heat-set insert pockets "
          f"(2 -X x Phi {hp.INSERT_M3_PILOT_OD:.1f} mm x "
          f"{hp.INSERT_M3_PILOT_DEPTH:.0f} mm pockets per cradle; "
          f"radial wall >= {hp.CRADLE_BOSS_MIN_WALL_MM:.1f} mm; "
          f"insert Phi {hp.INSERT_M3_INSERT_OD:.1f} mm x "
          f"{hp.INSERT_M3_INSERT_LENGTH:.0f} mm; "
          f"+X sites use self-tap pilots and are probed by "
          f"check_servo_insertion_path instead):")

    # STS3215 front-face mount (Jun 2026): RETIRED.
    #
    # The DS3225 cradles retained each servo with brass heat-set inserts
    # pressed into printed Phi-4 mm bosses on the -X column; this check
    # probed the boss for (1) a void pocket, (2) >= 1.5 mm radial wall,
    # and (3) a solid knurl-displacement ring.  The STS3215 has NO
    # inserts: the servo body is retained by a PRINTED strap (yaw,
    # ``make_yaw_servo_retainer``) or clamshell clamp cap (hip/knee,
    # ``make_servo_clamp_cap``, 2 x M3 self-tap per joint).  (The earlier
    # "4 x M2.5 output-face case screws" mount was a phantom feature and
    # has been removed.)  With the bosses gone the radial-material /
    # knurl-ring azimuth probes sample air by construction, so this check
    # no longer maps onto the design and is retired.
    print("  [SKIP]  heat-set boss probe retired (STS3215 has no inserts; "
          "body held by the printed strap/clamp, not a case-screw mount)")
    return True

    R_inv = rotation_matrix(+np.pi / 2.0, [1, 0, 0])

    def _bracket_to_well_local(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        m = mesh.copy()
        m.apply_translation([+hp.SERVO_OUTPUT_X, 0.0, +hp.WELL_RIM_Z])
        return m

    def _coxa_link_to_well_local(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        m = mesh.copy()
        m.apply_translation([0.0, 0.0, -hp.COXA_LIFT])
        arm_t = hp.COXA_ARM_T
        well_z_drop = -(hp.WELL_D / 2.0 + arm_t / 2.0
                         + hp.WELL_Z_DROP_EXTRA)
        m.apply_translation([0.0, 0.0, -well_z_drop])
        m.apply_translation([
            -(hp.COXA_LENGTH - hp.SERVO_OUTPUT_X),
            +(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H),
            0.0,
        ])
        m.apply_transform(R_inv)
        return m

    def _femur_link_to_well_local(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        # Bearing-sandwich refit (Jun 2026): the femur-link origin is the
        # knee disc-horn-top ON the joint axis (no axial pad offset), so
        # the well-local mapping mirrors the coxa/hip case exactly (no
        # HORN_STACK_H term).
        m = mesh.copy()
        m.apply_translation([
            -(hp.FEMUR_LENGTH - hp.SERVO_OUTPUT_X),
            +(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H),
            0.0,
        ])
        m.apply_transform(R_inv)
        return m

    # Commit 7/9 of the May 2026 chassis-bottom-integrated yaw-cradle
    # redesign: rerouted the yaw-cradle case in this check from
    # ``coxa_bracket`` to the integrated cradle in ``chassis_bottom``.
    # After ``_chassis_yaw_cradle_to_well_local`` the L0 cradle's tab
    # shelf sits at well-local z = +WELL_RIM_Z = +27.25 (no
    # bracket-shelf-drop term: chassis_bottom's cradle does NOT
    # have a drop-in flange eating its rim; the body drops straight
    # in past the boss top).  The boss + heat-set + self-tap geometry
    # in the cradle is built by the SAME
    # ``_servo_cradle_insert_pockets`` helper as the legacy bracket
    # used, so the sub-checks (pocket void, 8-azimuth radial wall,
    # knurl-ring) and probe parameters all transfer unchanged --
    # only the mesh + shelf_top_z constant + the per-case
    # ``radial_air_frac`` tolerance change.
    #
    # Each case = (label, well-local mesh, shelf_top_z, radial_air_frac).
    # radial_air_frac is the fraction of radial / knurl-ring azimuth
    # probes around the -X heat-set sites that are allowed to fall in
    # AIR before the case fails (see the function docstring's
    # Per-case radial-wall tolerance section).  The legacy link
    # cradles carry strict 0.0; the chassis_bottom case carries 0.25
    # (= 8/32 azimuths) to account for the -X wire-exit corridor's
    # intrusion through the inboard azimuth band of each -X boss
    # (engineering rationale in the function docstring + the
    # CRADLE_BOSS_* constants block in hexapod_prototype.py).
    cases = [
        ("chassis_bottom (yaw cradle, L0)",
         _chassis_yaw_cradle_to_well_local(
             _load_mesh("chassis_assembled", copy=False)),
         hp.WELL_RIM_Z,
         0.25),
        ("coxa_link    (hip-pitch cradle)",
         _coxa_link_to_well_local(_load_mesh("coxa_link", copy=False)),
         hp.WELL_RIM_Z,
         0.0),
        ("femur_link   (knee cradle)",
         _femur_link_to_well_local(_load_mesh("femur_link", copy=False)),
         hp.WELL_RIM_Z,
         0.0),
    ]

    pocket_probe_r = hp.INSERT_M3_PILOT_OD / 2.0 - 0.2

    r_outer_wall = (hp.INSERT_M3_PILOT_OD / 2.0
                    + hp.CRADLE_BOSS_MIN_WALL_MM)
    r_insert_ring = 0.5 * (hp.INSERT_M3_PILOT_OD / 2.0
                            + hp.INSERT_M3_INSERT_OD / 2.0)
    azimuths_deg = list(range(0, 360, 45))

    # Design E mixed-mode: only the -X column carries heat-set inserts;
    # the +X sites are bare Phi 2.5 mm self-tap pilots and intentionally
    # skip the radial-material / knurl-ring probes (the channel-facing
    # side has only ~ 0.25 mm of plastic, which is acceptable for an
    # M3 self-tap but would fail the heat-set CRADLE_BOSS_MIN_WALL_MM
    # check by design).
    HEATSET_SX_SIGNS = (-1,)

    all_ok = True
    for name, well_local_mesh, shelf_top_z, radial_air_frac in cases:
        # ---- (1) Pocket cylinder void probe -----------------------------
        pocket_hits = 0
        n_pockets = len(HEATSET_SX_SIGNS) * 2  # 2 (sx) * 2 (sy) = 2
        probe_depth = hp.INSERT_M3_PILOT_DEPTH * 0.6
        probe_z_cen = shelf_top_z - hp.INSERT_M3_PILOT_DEPTH / 2.0
        for sx in HEATSET_SX_SIGNS:
            for sy in (-1, 1):
                x = sx * hp.SERVO_MOUNT_HOLE_X_OFFSET
                y = sy * hp.SERVO_MOUNT_HOLE_Y_OFFSET
                centre = np.array([x, y, probe_z_cen])
                pocket_hits += _probe_void_cylinder(
                    well_local_mesh, centre, "z",
                    pocket_probe_r, probe_depth, n_samples=48,
                )
        pocket_tol = CRADLE_INSERT_POCKET_VOX_TOL * n_pockets
        ok_pockets = pocket_hits <= pocket_tol
        all_ok &= _label(
            f"{name} :: {n_pockets} x Phi {hp.INSERT_M3_PILOT_OD:.1f} "
            f"mm x {hp.INSERT_M3_PILOT_DEPTH:.0f} mm insert pockets "
            f"(-X column)",
            ok_pockets,
            f"hits={pocket_hits} (tol {pocket_tol})",
        )

        # ---- (2) Radial-material check ----------------------------------
        # 8 azimuths x 2 z-planes (pocket_bot + 1, pocket_top - 1) at
        # radius r_outer_wall around each pocket axis must hit material.
        pocket_z_bot = shelf_top_z - hp.INSERT_M3_PILOT_DEPTH
        pocket_z_top = shelf_top_z
        z_planes = (pocket_z_bot + 1.0, pocket_z_top - 1.0)
        n_radial = (len(HEATSET_SX_SIGNS) * 2 * len(azimuths_deg)
                    * len(z_planes))
        radial_misses: list[str] = []
        for sx in HEATSET_SX_SIGNS:
            for sy in (-1, 1):
                cx = sx * hp.SERVO_MOUNT_HOLE_X_OFFSET
                cy = sy * hp.SERVO_MOUNT_HOLE_Y_OFFSET
                pts = []
                metas = []
                for az in azimuths_deg:
                    rad = np.deg2rad(az)
                    px = cx + r_outer_wall * np.cos(rad)
                    py = cy + r_outer_wall * np.sin(rad)
                    for pz in z_planes:
                        pts.append((px, py, pz))
                        metas.append((sx, sy, az, pz))
                inside = points_inside(well_local_mesh,
                                       np.array(pts, dtype=float))
                for hit, meta in zip(inside, metas):
                    if not hit:
                        radial_misses.append(
                            f"(x={meta[0]:+d}*X, y={meta[1]:+d}*Y, "
                            f"az={meta[2]:3d}deg, z={meta[3]:5.2f})"
                        )
        n_radial_misses = len(radial_misses)
        # Per-case tolerance: legacy cradles strict (0 air-flagged
        # allowed); chassis_bottom case relaxed to 25 percent to
        # account for the -X wire-exit corridor's intrusion through
        # the inboard 8/32 azimuths of each -X boss.  See the
        # function docstring's Per-case radial-wall tolerance
        # section for the engineering rationale.
        radial_air_tol = int(round(n_radial * radial_air_frac))
        ok_radial = n_radial_misses <= radial_air_tol
        if ok_radial and n_radial_misses == 0:
            radial_detail = f"{n_radial}/{n_radial} azimuths hit material"
        elif ok_radial:
            radial_detail = (
                f"{n_radial_misses}/{n_radial} azimuths punched AIR at "
                f"r = {r_outer_wall:.2f} mm "
                f"(within case-specific tol = {radial_air_tol})"
            )
        else:
            sample = ", ".join(radial_misses[:6])
            more = (f"; +{n_radial_misses - 6} more"
                    if n_radial_misses > 6 else "")
            radial_detail = (
                f"{n_radial_misses}/{n_radial} azimuths punched AIR at "
                f"r = {r_outer_wall:.2f} mm "
                f"(tol = {radial_air_tol}): {sample}{more}"
            )
        all_ok &= _label(
            f"{name} :: 8-azimuth radial-material around each -X pocket "
            f"(r = {r_outer_wall:.2f} mm)",
            ok_radial,
            radial_detail,
        )

        # ---- (3) Heat-set insert displacement-ring check ----------------
        z_insert_mid = shelf_top_z - hp.INSERT_M3_INSERT_LENGTH / 2.0
        n_ring = len(HEATSET_SX_SIGNS) * 2 * len(azimuths_deg)
        ring_misses: list[str] = []
        for sx in HEATSET_SX_SIGNS:
            for sy in (-1, 1):
                cx = sx * hp.SERVO_MOUNT_HOLE_X_OFFSET
                cy = sy * hp.SERVO_MOUNT_HOLE_Y_OFFSET
                pts = []
                metas = []
                for az in azimuths_deg:
                    rad = np.deg2rad(az)
                    px = cx + r_insert_ring * np.cos(rad)
                    py = cy + r_insert_ring * np.sin(rad)
                    pts.append((px, py, z_insert_mid))
                    metas.append((sx, sy, az))
                inside = points_inside(well_local_mesh,
                                       np.array(pts, dtype=float))
                for hit, meta in zip(inside, metas):
                    if not hit:
                        ring_misses.append(
                            f"(x={meta[0]:+d}*X, y={meta[1]:+d}*Y, "
                            f"az={meta[2]:3d}deg)"
                        )
        n_ring_misses = len(ring_misses)
        # Per-case tolerance: same fraction applied to the knurl-ring
        # probe as to the outer radial-wall probe -- the wire-exit
        # corridor on the chassis_bottom case carves through the
        # boss's inner azimuth band at BOTH the outer wall radius
        # and the knurl-ring radius, so both rings see the same 25
        # percent loss of azimuth coverage.
        ring_air_tol = int(round(n_ring * radial_air_frac))
        ok_ring = n_ring_misses <= ring_air_tol
        if ok_ring and n_ring_misses == 0:
            ring_detail = (
                f"{n_ring}/{n_ring} azimuths hit material at r = "
                f"{r_insert_ring:.2f} mm"
            )
        elif ok_ring:
            ring_detail = (
                f"{n_ring_misses}/{n_ring} azimuths punched AIR at r = "
                f"{r_insert_ring:.2f} mm (knurl ring; within "
                f"case-specific tol = {ring_air_tol})"
            )
        else:
            sample = ", ".join(ring_misses[:6])
            more = (f"; +{n_ring_misses - 6} more"
                    if n_ring_misses > 6 else "")
            ring_detail = (
                f"{n_ring_misses}/{n_ring} azimuths punched AIR at r = "
                f"{r_insert_ring:.2f} mm (knurl ring; tol = "
                f"{ring_air_tol}): {sample}{more}"
            )
        all_ok &= _label(
            f"{name} :: heat-set insert displacement ring (-X column, "
            f"r = {r_insert_ring:.2f} mm)",
            ok_ring,
            ring_detail,
        )

    return all_ok


# Backwards-compatible alias: external callers that still reference the
# old name (and the verifier's CHECKS registry's previous entry) get
# the new behaviour transparently.
check_cradle_pilot_holes = check_cradle_insert_pockets


# ---------------------------------------------------------------------------
# Servo insertion-path probe (Design E, May 2026)
# ---------------------------------------------------------------------------
#
# Simulates seating a DS3225 / MG996R / DS3218-class servo into its
# cradle by sliding the molded wire-exit BOOT envelope DOWN through
# the +X wall in 1 mm Z steps and checking that no printed-part
# material intrudes into the boot's swept volume by more than the
# tolerance below at any intermediate Z.
#
# This is the regression probe for the May 2026 user report: commit
# f03d59b (heat-set switch) shortened the +X wire channel from
# z = WELL_RIM_Z + WIRE_CHANNEL_TOP_OVER_RIM (= 29.5) down to
# z = (WELL_RIM_Z - CRADLE_BOSS_HEIGHT_MM) - 0.5 (= 16.5) to clear
# the new Phi 8 mm heat-set bosses, which left 10.5 mm of solid +X
# wall blocking the boot's insertion path.  Real DS3225 servos
# could not be seated in their printed cradles.  Design E restored
# the channel cap and reverted the +X bolts to Phi 2.5 mm self-tap;
# this check enforces the channel-must-be-clear invariant so the
# regression cannot recur.

# Allowed boot-vs-part overlap volume at any single insertion-step.
# 1 mm^3 covers numerical / mesh-stair-step noise at our voxel pitch.
SERVO_INSERTION_PATH_TOL_MM3 = 1.0
# Z step size for the insertion sweep (mm).
SERVO_INSERTION_PATH_Z_STEP_MM = 1.0
# Sampling pitch inside the boot envelope (mm).  0.4 mm pitch on the
# 6.5 x 7.0 x 3.9 mm boot envelope is ~ 4400 sample points; at our
# 1 mm^3 tolerance any single sample voxel contributes 0.064 mm^3 so
# > 15 sample hits are needed to register as a failure.
SERVO_INSERTION_PATH_PITCH_MM = 0.4


def _boot_envelope_sample_points(z_offset: float) -> np.ndarray:
    """Return a dense grid of sample points covering the molded
    wire-exit boot envelope, translated in +Z by ``z_offset`` from
    the seated position.

    Seated boot envelope (well-local frame, matches ``_servo_envelope``):
        x in [+SERVO_BODY_W/2,
              +SERVO_BODY_W/2 + WIRE_BOOT_PROTRUSION]
        y in [-WIRE_BOOT_W/2, +WIRE_BOOT_W/2]
        z in [WIRE_BOOT_Z_BASE,
              WIRE_BOOT_Z_BASE + WIRE_BOOT_H]

    During insertion the servo body sits a bit higher in the cradle
    so the boot's z bounds shift UP by ``z_offset``; the x / y bounds
    are unchanged because the body is constrained in those axes by
    the well cavity walls.
    """
    pitch = SERVO_INSERTION_PATH_PITCH_MM
    x_min = +hp.SERVO_BODY_W / 2.0
    x_max = x_min + hp.WIRE_BOOT_PROTRUSION
    y_min = -hp.WIRE_BOOT_W / 2.0
    y_max = +hp.WIRE_BOOT_W / 2.0
    z_min = hp.WIRE_BOOT_Z_BASE + z_offset
    z_max = z_min + hp.WIRE_BOOT_H

    # Inset by half a voxel so sample centres tile the envelope cleanly.
    eps = pitch / 2.0
    nx = max(2, int(np.ceil(((x_max - eps) - (x_min + eps)) / pitch)))
    ny = max(2, int(np.ceil(((y_max - eps) - (y_min + eps)) / pitch)))
    nz = max(2, int(np.ceil(((z_max - eps) - (z_min + eps)) / pitch)))
    xs = np.linspace(x_min + eps, x_max - eps, nx)
    ys = np.linspace(y_min + eps, y_max - eps, ny)
    zs = np.linspace(z_min + eps, z_max - eps, nz)
    XX, YY, ZZ = np.meshgrid(xs, ys, zs, indexing="ij")
    return np.stack([XX.ravel(), YY.ravel(), ZZ.ravel()], axis=1)


def check_servo_insertion_path():
    """Verify the shared hip/knee holder's +Y service-extraction path.

    The body and fitted disc horn move together when the servo is lifted out
    with its clamp cap attached.  Probe both rigid envelopes through the full
    +Y travel: the body must pass through the open clamp mouth, and a disc with
    1.5 mm radial running clearance must pass through the output-lip service
    slot.  This specifically catches restoration of the old rounded bridge
    over the horn, which made the fitted assembly captive.
    """
    print("\n[5f] Hip/knee servo + fitted-horn service-extraction path:")
    holder = hp._servo_well_solid(end_face_bolts=False)
    all_ok = True

    # Sweep a slightly shrunken servo body from seated to clear of the +Y
    # mouth.  The 0.5 mm surface inset avoids counting intended wall contact
    # or voxel-boundary noise as interference.
    xs = np.linspace(-hp.SERVO_BODY_W / 2.0 + 0.5,
                     +hp.SERVO_BODY_W / 2.0 - 0.5, 7)
    ys = np.linspace(-hp.SERVO_BODY_D / 2.0 + 0.5,
                     +hp.SERVO_BODY_D / 2.0 - 0.5, 5)
    zs = np.linspace(0.5, hp.SERVO_BODY_H - 0.5, 9)
    bx, by, bz = np.meshgrid(xs, ys, zs, indexing="ij")
    body_seated = np.stack([bx.ravel(), by.ravel(), bz.ravel()], axis=1)
    y_travel = hp.WELL_D + hp.SERVO_BODY_D
    body_blocked = 0
    for dy in np.linspace(0.0, y_travel, 25):
        pts = body_seated + np.array([0.0, dy, 0.0])
        body_blocked += int(points_inside(holder, pts).sum())
    body_ok = body_blocked == 0
    all_ok &= _label(
        "servo body through +Y clamp mouth", body_ok,
        f"25 lift positions, blocked samples={body_blocked}")

    # Sweep the fitted horn in the output-lip plane.  The real horn radius is
    # 10 mm; probe at 11.5 mm so the test requires >=1.5 mm radial clearance
    # inside the nominal 12 mm-radius opening/slot without sampling its exact
    # boolean boundary.
    probe_r = hp.DISC_HORN_OD / 2.0 + 1.5
    p = 0.5
    gx = np.arange(-probe_r + p / 2.0, probe_r, p)
    gy = np.arange(-probe_r + p / 2.0, probe_r, p)
    xx, yy = np.meshgrid(gx, gy, indexing="ij")
    mask = xx * xx + yy * yy <= probe_r * probe_r
    disc_xy = np.stack([xx[mask], yy[mask]], axis=1)
    horn_blocked = 0
    for dy in np.linspace(0.0, y_travel, 31):
        for z in (hp.WELL_RIM_Z + 0.6,
                  hp.WELL_RIM_Z + hp.WELL_PLATE_T / 2.0,
                  hp.WELL_H - 0.6):
            pts = np.column_stack([
                hp.SERVO_OUTPUT_X + disc_xy[:, 0],
                dy + disc_xy[:, 1],
                np.full(len(disc_xy), z),
            ])
            horn_blocked += int(points_inside(holder, pts).sum())
    horn_ok = horn_blocked == 0
    all_ok &= _label(
        "fitted disc horn through +Y output service slot", horn_ok,
        f"31 lift positions x 3 lip planes, blocked samples={horn_blocked}; "
        f"probe radius={probe_r:.1f} mm")

    active_front = hp.servo_holder_front_case_hole_centres()
    screw_pattern_ok = (len(active_front) == 3 and
                        (4.2, +hp.SERVO_FRONT_CASE_HOLE_Y) not in active_front)
    all_ok &= _label(
        "service slot removes only the intended front-case site",
        screw_pattern_ok,
        f"retained={active_front}; removed=(4.2, "
        f"+{hp.SERVO_FRONT_CASE_HOLE_Y:.2f})")
    return all_ok

    R_inv = rotation_matrix(+np.pi / 2.0, [1, 0, 0])

    def _bracket_to_well_local(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        m = mesh.copy()
        m.apply_translation([+hp.SERVO_OUTPUT_X, 0.0, +hp.WELL_RIM_Z])
        return m

    def _coxa_link_to_well_local(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        m = mesh.copy()
        m.apply_translation([0.0, 0.0, -hp.COXA_LIFT])
        arm_t = hp.COXA_ARM_T
        well_z_drop = -(hp.WELL_D / 2.0 + arm_t / 2.0
                         + hp.WELL_Z_DROP_EXTRA)
        m.apply_translation([0.0, 0.0, -well_z_drop])
        m.apply_translation([
            -(hp.COXA_LENGTH - hp.SERVO_OUTPUT_X),
            +(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H),
            0.0,
        ])
        m.apply_transform(R_inv)
        return m

    def _femur_link_to_well_local(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        # Bearing-sandwich refit (Jun 2026): the femur-link origin is the
        # knee disc-horn-top ON the joint axis (no axial pad offset), so
        # the well-local mapping mirrors the coxa/hip case exactly (no
        # HORN_STACK_H term).
        m = mesh.copy()
        m.apply_translation([
            -(hp.FEMUR_LENGTH - hp.SERVO_OUTPUT_X),
            +(hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H),
            0.0,
        ])
        m.apply_transform(R_inv)
        return m

    # Commit 7/9 of the May 2026 chassis-bottom-integrated yaw-cradle
    # redesign: rerouted the yaw-cradle case in this check from
    # ``coxa_bracket`` to the integrated cradle in ``chassis_bottom``.
    # The boot envelope is fixed in well-local frame so the same
    # _boot_envelope_sample_points() helper drives the sweep on
    # chassis_bottom after _chassis_yaw_cradle_to_well_local re-maps
    # the cradle into the bracket's old well-local origin.  The hip
    # and knee cradles continue to be probed as before.
    cradles = [
        ("chassis_bottom (yaw cradle, L0)",
         _chassis_yaw_cradle_to_well_local(
             _load_mesh("chassis_assembled", copy=False))),
        ("coxa_link    (hip-pitch cradle)",
         _coxa_link_to_well_local(_load_mesh("coxa_link", copy=False))),
        ("femur_link   (knee cradle)",
         _femur_link_to_well_local(_load_mesh("femur_link", copy=False))),
    ]

    # Z sweep: boot bottom slides from WELL_RIM_Z + WIRE_BOOT_H (boot
    # entirely above the rim) DOWN to WIRE_BOOT_Z_BASE (seated).
    # Express as offsets RELATIVE to the seated position; offset 0
    # means seated, offset > 0 means the body is lifted ``offset`` mm
    # above seated.
    z_offset_max = (hp.WELL_RIM_Z + hp.WIRE_BOOT_H) - hp.WIRE_BOOT_Z_BASE
    n_steps = int(np.ceil(z_offset_max
                          / SERVO_INSERTION_PATH_Z_STEP_MM)) + 1
    z_offsets = np.linspace(0.0, z_offset_max, n_steps)

    pitch = SERVO_INSERTION_PATH_PITCH_MM
    voxel_vol = pitch ** 3

    all_ok = True
    for name, mesh in cradles:
        # Pre-compute the AABB of the entire boot sweep so we can
        # skip mesh-inside calls when the sweep box is well clear of
        # the cradle's bounds (a no-op for the standard cradles
        # where the boot sweep does intersect the +X wall, but
        # gives a clean fail fast if a future cradle is repositioned).
        mesh_min, mesh_max = mesh.bounds
        worst_overlap_mm3 = 0.0
        worst_offset = float(z_offsets[0])
        n_steps_blocked = 0
        for z_off in z_offsets:
            pts = _boot_envelope_sample_points(float(z_off))
            # Quick AABB rejection.
            if (pts[:, 0].max() < mesh_min[0]
                    or pts[:, 0].min() > mesh_max[0]
                    or pts[:, 1].max() < mesh_min[1]
                    or pts[:, 1].min() > mesh_max[1]
                    or pts[:, 2].max() < mesh_min[2]
                    or pts[:, 2].min() > mesh_max[2]):
                continue
            inside = points_inside(mesh, pts)
            n_in = int(inside.sum())
            overlap_mm3 = n_in * voxel_vol
            if overlap_mm3 > worst_overlap_mm3:
                worst_overlap_mm3 = overlap_mm3
                worst_offset = float(z_off)
            if overlap_mm3 > SERVO_INSERTION_PATH_TOL_MM3:
                n_steps_blocked += 1

        ok = n_steps_blocked == 0
        if ok:
            detail = (
                f"{n_steps} Z steps probed (boot bottom z = "
                f"{hp.WIRE_BOOT_Z_BASE:.1f}-> "
                f"{hp.WIRE_BOOT_Z_BASE + z_offset_max:.1f} mm); worst "
                f"step overlap {worst_overlap_mm3:.2f} mm^3 <= "
                f"{SERVO_INSERTION_PATH_TOL_MM3:.1f} mm^3 tol"
            )
        else:
            detail = (
                f"{n_steps_blocked}/{n_steps} Z steps blocked; worst "
                f"step overlap {worst_overlap_mm3:.2f} mm^3 at boot "
                f"bottom z = {hp.WIRE_BOOT_Z_BASE + worst_offset:.1f} "
                f"mm (offset {worst_offset:+.1f} above seated); "
                f"tol {SERVO_INSERTION_PATH_TOL_MM3:.1f} mm^3"
            )
        all_ok &= _label(name, ok, detail)

    return all_ok


# ---------------------------------------------------------------------------
# Bearing insertion-path / assemblability probe (Jun 2026)
# ---------------------------------------------------------------------------
#
# Forensic regression probe for the user's "the 6706 yaw bearings are
# IMPOSSIBLE TO INSERT" report.  The old single-piece chassis tower bored
# 37 -> 34 -> 37 -> 34: two Phi 37 outer-race pockets each TRAPPED between
# Phi 34 constrictions, so neither race could reach its seat from any
# direction.  The static-assembly CAD validated the seated state but never
# an insertion PATH, so the captured pocket shipped silently.
#
# This check sweeps a RIGID Phi (YAW_BEARING_OD) disc -- the outer race --
# down its insertion axis from clear-of-the-host to seated and asserts the
# swept volume stays collision-free against the printed host.  A slip-fit
# bore wall only grazes the disc edge (a few mm^3 of voxel-stairstep noise);
# a real Phi 34 constriction forces the full Phi 37 disc through ~80 mm^3 of
# solid annulus and trips the tolerance.  Mirrors check_servo_insertion_path.
#
# Fail-on-old / pass-on-new is proven INLINE by check_bearing_insertion_path's
# embedded self-test (_old_captured_yaw_tower): the reconstructed old captured
# tower blocks at ~79 mm^3 at the Phi 34 neck for BOTH races, while the new
# split parts sweep at <= 1 mm^3 worst overlap.

# Allowed disc-vs-host overlap at any single insertion step.  ~25 mm^3 sits
# well above slip-fit grazing noise (<= a few mm^3) and well below a real
# Phi 34 constriction (~80 mm^3 across the trapped annulus).
BEARING_INSERTION_PATH_TOL_MM3 = 25.0
BEARING_INSERTION_PATH_Z_STEP_MM = 0.5
BEARING_INSERTION_PATH_PITCH_MM = 0.5


def _chassis_yaw_cradle_to_coxa_local(mesh: trimesh.Trimesh,
                                      leg_index: int = 0) -> trimesh.Trimesh:
    """Map a ``chassis_bottom`` mesh into COXA-LOCAL frame (yaw axis at the
    origin, z = 0 = disc-horn top) -- the frame ``make_yaw_bearing_cap`` /
    ``make_coxa_yaw_hub`` / the bearing Z bands are expressed in."""
    m = _chassis_yaw_cradle_to_well_local(mesh, leg_index)
    horn_top_z = hp.SERVO_BODY_H + hp.WELL_PLATE_T + hp.HORN_STACK_H
    m.apply_translation([-hp.SERVO_OUTPUT_X, 0.0, -horn_top_z])
    return m


def _bearing_disc_points(zc: float, R: float, pitch: float) -> np.ndarray:
    """Filled Phi (2R) disc of sample points at coxa-local height ``zc``."""
    n = int(np.ceil(2.0 * R / pitch)) + 1
    xs = np.linspace(-R, R, n)
    X, Y = np.meshgrid(xs, xs)
    m = (X ** 2 + Y ** 2) <= R ** 2
    return np.column_stack([X[m], Y[m], np.full(int(m.sum()), zc)])


def _sweep_bearing_disc(mesh: trimesh.Trimesh, z_hi: float, z_seat: float):
    """Slide a filled Phi (YAW_BEARING_OD) disc from ``z_hi`` DOWN to one Z
    step above ``z_seat`` and return ``(worst_mm3, worst_z, n_blocked,
    n_steps)``.  Stopping a step above the seat keeps the intended seated
    contact (disc resting ON its shoulder -- a large, expected overlap) out of
    the blockage count; any real constriction lives ABOVE the seat and trips."""
    R = hp.YAW_BEARING_OD / 2.0
    pitch = BEARING_INSERTION_PATH_PITCH_MM
    vox = pitch ** 3
    worst = 0.0
    worst_z = z_hi
    n_blocked = 0
    n_steps = 0
    z = z_hi
    z_end = z_seat + BEARING_INSERTION_PATH_Z_STEP_MM
    while z >= z_end - 1e-6:
        n_steps += 1
        pts = _bearing_disc_points(z, R, pitch)
        overlap = int(points_inside(mesh, pts).sum()) * vox
        if overlap > worst:
            worst, worst_z = overlap, z
        if overlap > BEARING_INSERTION_PATH_TOL_MM3:
            n_blocked += 1
        z -= BEARING_INSERTION_PATH_Z_STEP_MM
    return worst, worst_z, n_blocked, n_steps


def _old_captured_yaw_tower() -> trimesh.Trimesh:
    """Reconstruct the RETIRED single-piece yaw tower (coxa-local) that bored
    Phi 37 -> 34 -> 37 -> 34 and trapped BOTH 6706 outer races between Phi 34
    constrictions.  Used ONLY by ``check_bearing_insertion_path``'s self-test
    to prove the probe is sensitive enough to flag the captured pocket the new
    split design eliminates."""
    r_out = hp.YAW_BEARING_OD / 2.0 + hp.YAW_TOWER_WALL
    r_bore = hp.YAW_TOWER_BORE_OD / 2.0          # Phi 37 race pocket
    r_lip = hp.YAW_TOWER_SHOULDER_OD / 2.0       # Phi 34 neck / lip constriction

    def _cyl_at(r, z0, z1):
        c = hp._cyl(r, z1 - z0)
        c.apply_translation([0.0, 0.0, 0.5 * (z0 + z1)])
        return c

    tower = _cyl_at(r_out, hp.YAW_BEARING_LOWER_BOT_Z, hp.YAW_TOWER_TOP_Z)
    return hp._diff(
        tower,
        _cyl_at(r_bore, hp.YAW_BEARING_LOWER_BOT_Z, hp.YAW_BEARING_LOWER_TOP_Z),
        _cyl_at(r_lip, hp.YAW_BEARING_LOWER_TOP_Z, hp.YAW_BEARING_UPPER_BOT_Z),
        _cyl_at(r_bore, hp.YAW_BEARING_UPPER_BOT_Z, hp.YAW_BEARING_UPPER_TOP_Z),
        _cyl_at(r_lip, hp.YAW_BEARING_UPPER_TOP_Z, hp.YAW_TOWER_TOP_Z),
    )


def check_bearing_insertion_path():
    """Sweep a rigid Phi (YAW_BEARING_OD) disc down each 6706 race's insertion
    axis and assert a clear, monotone path to its seat exists.

    Includes an embedded FAIL-ON-OLD self-test: the retired single-piece tower
    (Phi 37 -> 34 -> 37 -> 34) is reconstructed and swept; the probe MUST flag
    it as blocked (worst overlap >> tol at the Phi 34 neck), proving it is
    sensitive enough to catch the captured-pocket regression.  The live
    chassis_bottom bottom tower must then PASS.

    NB: only the chassis_bottom LOWER-race pocket is swept here (a true
    drop-from-open-top motion).  The ``yaw_bearing_cap`` is NOT swept top-down:
    its UPPER-race insertion is the cap LOWERED over the already-seated race
    (the race never drops into the cap from above -- and the cap's Phi 34
    retaining lip now deliberately blocks that motion).  That lowered-cap
    motion is covered by ``check_bearing_cap_descent_path`` [5i] (race sweeps UP
    to its seat) and ``check_bearing_assembly_sequence`` [5j] (rigid annulus,
    both mating parts present)."""
    R = hp.YAW_BEARING_OD / 2.0
    print(f"\n[5g] Bearing insertion-path probe "
          f"(rigid Phi{2 * R:.0f} disc slides down its axis in "
          f"{BEARING_INSERTION_PATH_Z_STEP_MM:.1f} mm Z steps; max overlap "
          f"{BEARING_INSERTION_PATH_TOL_MM3:.0f} mm^3 per step):")

    all_ok = True

    # ---- Self-test: the OLD captured single-piece tower MUST be flagged -----
    # Drop the Phi 37 disc from clear-of-tower down toward each race seat; the
    # Phi 34 neck/lip above each pocket forces the full disc through a solid
    # annulus, so the probe must report a blockage (>> tol).  If this ever
    # PASSES (no blockage found) the probe has gone blind -- fail the check.
    old = _old_captured_yaw_tower()
    old_worst = 0.0
    old_blocked = 0
    for z_seat in (hp.YAW_BEARING_LOWER_BOT_Z, hp.YAW_BEARING_UPPER_BOT_Z):
        w, _wz, nb, _ns = _sweep_bearing_disc(old, hp.YAW_TOWER_TOP_Z + 2.0,
                                              z_seat)
        old_worst = max(old_worst, w)
        old_blocked += nb
    self_ok = old_blocked > 0
    all_ok &= _label(
        "SELF-TEST: retired single-piece tower (Phi37->34->37->34) is blocked",
        self_ok,
        f"reconstructed old tower flagged at {old_blocked} step(s); worst "
        f"overlap {old_worst:.1f} mm^3 >> tol "
        f"{BEARING_INSERTION_PATH_TOL_MM3:.0f} mm^3 (probe is sensitive)")

    # ---- Live split design MUST pass --------------------------------------
    # Only the chassis_bottom LOWER-race pocket is a genuine drop-from-open-top
    # insertion.  The yaw_bearing_cap is intentionally OMITTED: the upper race
    # is not dropped into the cap from above (the cap is lowered over the
    # already-seated race, and its Phi 34 retaining lip now blocks top-drop by
    # design).  The lowered-cap motion is covered by [5i] / [5j].
    cases = [
        # (label, host in coxa-local, z_clear (start, above), z_seat (end))
        ("chassis_bottom LOWER race (drops onto z=-5 from open top)",
         _chassis_yaw_cradle_to_coxa_local(_load_mesh("chassis_assembled",
                                                      copy=False)),
         hp.YAW_SPLIT_Z + 1.0, hp.YAW_BEARING_LOWER_BOT_Z),
    ]

    for name, mesh, z_hi, z_seat in cases:
        worst, worst_z, n_blocked, n_steps = _sweep_bearing_disc(
            mesh, z_hi, z_seat)
        ok = n_blocked == 0
        detail = (f"{n_steps} Z steps probed; worst overlap {worst:.1f} mm^3 "
                  f"at z={worst_z:+.1f} (tol "
                  f"{BEARING_INSERTION_PATH_TOL_MM3:.0f} mm^3)")
        if not ok:
            detail = (f"{n_blocked}/{n_steps} steps blocked; " + detail)
        all_ok &= _label(name, ok, detail)

    return all_ok


def _sweep_bearing_disc_up(mesh: trimesh.Trimesh, z_lo: float, z_seat: float):
    """Slide a filled Phi (YAW_BEARING_OD) disc from ``z_lo`` (below the cap)
    UP to one Z step BELOW ``z_seat`` and return ``(worst_mm3, worst_z,
    n_blocked, n_steps)``.

    Models the cap being LOWERED straight down over the upper outer race that
    is already seated on the hub boss -- in the cap's own frame the race
    travels UP from below the split plane to its seat.  Any cap feature whose
    bore is < Phi (YAW_BEARING_OD) BELOW the seated race position trips the
    tolerance (it is a wall the descending cap cannot pass over the race)."""
    R = hp.YAW_BEARING_OD / 2.0
    pitch = BEARING_INSERTION_PATH_PITCH_MM
    vox = pitch ** 3
    worst, worst_z, n_blocked, n_steps = 0.0, z_lo, 0, 0
    z = z_lo
    z_end = z_seat - BEARING_INSERTION_PATH_Z_STEP_MM
    while z <= z_end + 1e-6:
        n_steps += 1
        pts = _bearing_disc_points(z, R, pitch)
        overlap = int(points_inside(mesh, pts).sum()) * vox
        if overlap > worst:
            worst, worst_z = overlap, z
        if overlap > BEARING_INSERTION_PATH_TOL_MM3:
            n_blocked += 1
        z += BEARING_INSERTION_PATH_Z_STEP_MM
    return worst, worst_z, n_blocked, n_steps


def _old_necked_yaw_cap() -> trimesh.Trimesh:
    """Reconstruct the RETIRED yaw_bearing_cap bore (coxa-local) WITH the Phi 34
    neck below the Phi 37 upper-race pocket -- the constriction that blocked the
    cap from being LOWERED over the hub-seated upper outer race.  Used ONLY by
    ``check_bearing_cap_descent_path``'s self-test to prove the probe is
    sensitive enough to flag the regression the clean Phi 37 bore fixes."""
    r_out = hp.YAW_BEARING_OD / 2.0 + hp.YAW_TOWER_WALL
    r_bore = hp.YAW_TOWER_BORE_OD / 2.0          # Phi 37 race pocket
    r_neck = hp.YAW_TOWER_SHOULDER_OD / 2.0      # Phi 34 neck constriction

    def _cyl_at(r, z0, z1):
        c = hp._cyl(r, z1 - z0)
        c.apply_translation([0.0, 0.0, 0.5 * (z0 + z1)])
        return c

    ring = _cyl_at(r_out, hp.YAW_SPLIT_Z, hp.YAW_CAP_TOP_Z)   # Phi 44, z[-1, +6]
    return hp._diff(
        ring,
        _cyl_at(r_neck, hp.YAW_SPLIT_Z, hp.YAW_BEARING_UPPER_BOT_Z),   # Phi34 z[-1,+2]
        _cyl_at(r_bore, hp.YAW_BEARING_UPPER_BOT_Z, hp.YAW_CAP_TOP_Z), # Phi37 z[+2,+6]
    )


def check_bearing_cap_descent_path():
    """Sweep a rigid Phi (YAW_BEARING_OD) disc UP through the live
    ``yaw_bearing_cap`` and assert the cap can be LOWERED straight down over
    the upper outer race already seated on the hub boss -- the ONLY valid
    assembly motion for a one-piece 6706 (its inner race must load onto the
    boss from below, so the outer race cannot also drop into the cap from
    above).

    This closes the blind spot in ``check_bearing_insertion_path``, which only
    swept the race DOWN into a FREE-STANDING cap (race-into-cap) -- a motion
    that never happens in the robot, since the outer race is welded to its
    inner race riding the boss.  That probe passed the retired Phi 34-neck cap
    (the neck is BELOW its down-sweep's z=+2 stop), so the un-assemblable cap
    validated green.

    Includes a FAIL-ON-OLD self-test: the reconstructed Phi 34-neck cap MUST be
    flagged (the neck is a measured ~85 mm^3 hard stop), proving the probe is
    sensitive; the live clean-Phi-37-bore cap must then PASS."""
    R = hp.YAW_BEARING_OD / 2.0
    print(f"\n[5i] Bearing-cap descent-path probe (rigid Phi{2 * R:.0f} disc "
          f"slides UP through the cap to the upper-race seat; models the cap "
          f"LOWERED over the hub-seated race; max overlap "
          f"{BEARING_INSERTION_PATH_TOL_MM3:.0f} mm^3 per step):")

    all_ok = True
    z_lo = hp.YAW_SPLIT_Z - 2.0
    z_seat = hp.YAW_BEARING_UPPER_BOT_Z                       # +2

    # ---- Self-test: the retired Phi 34-neck cap MUST be flagged ------------
    old = _old_necked_yaw_cap()
    ow, _owz, old_blocked, _ons = _sweep_bearing_disc_up(old, z_lo, z_seat)
    all_ok &= _label(
        "SELF-TEST: retired Phi34-neck cap blocks the cap descent",
        old_blocked > 0,
        f"reconstructed old necked cap flagged at {old_blocked} step(s); worst "
        f"overlap {ow:.1f} mm^3 >> tol "
        f"{BEARING_INSERTION_PATH_TOL_MM3:.0f} mm^3 (probe is sensitive)")

    # ---- Live cap MUST pass -----------------------------------------------
    cap = hp.make_yaw_bearing_cap()
    worst, worst_z, n_blocked, n_steps = _sweep_bearing_disc_up(cap, z_lo, z_seat)
    ok = n_blocked == 0
    detail = (f"{n_steps} Z steps probed; worst overlap {worst:.1f} mm^3 "
              f"at z={worst_z:+.1f} (tol "
              f"{BEARING_INSERTION_PATH_TOL_MM3:.0f} mm^3)")
    if not ok:
        detail = (f"{n_blocked}/{n_steps} steps blocked; " + detail)
    all_ok &= _label("yaw_bearing_cap lowers over the hub-seated upper race",
                     ok, detail)

    return all_ok


# ---------------------------------------------------------------------------
# Hub-boss INNER-race insertion-path / assemblability probe (Jun 2026)
# ---------------------------------------------------------------------------
#
# Sibling of check_bearing_insertion_path.  That probe verifies the STATIONARY
# OUTER races can reach their seats in the chassis tower / yaw_bearing_cap.
# This probe verifies the ROTATING INNER races can slide onto the coxa yaw
# HUB BOSS.
#
# Forensic regression probe for the user's "it's not possible to push the
# bearing over the lip on the coxa yaw hub" report.  make_coxa_yaw_hub rides
# both 6706 inner races on a Phi (YAW_HUB_BOSS_OD) boss whose TOP is capped by
# the Phi 44+ turntable platform, so an inner race can only slide on from the
# boss BOTTOM open end.  The old hub carried TWO Phi (YAW_BEARING_INNER_OD)
# retain flanges -- one ABOVE each race -- so the UPPER inner race was trapped
# between the lower flange (below) and the upper flange/platform (above): a
# Phi (YAW_BEARING_ID) bore cannot pass either Phi (YAW_BEARING_INNER_OD)
# flange, so it could not reach its seat from either end.  The static-assembly
# CAD validated the seated state but never an insertion PATH.
#
# This check slides a rigid annulus the size of an inner race's bore..OD band
# (r in [YAW_BEARING_ID/2, YAW_BEARING_INNER_OD/2]) up the boss axis from the
# open (bottom) end to one step below the race's seat-retain shoulder, and
# asserts the swept volume stays collision-free against the printed hub: i.e.
# NO feature of OD > the bore (Phi YAW_BEARING_ID) sits between the seat and
# the open end the race loads from.  The seat-retain shoulder itself (the
# uflange the upper race seats UP against) lives one step ABOVE the swept band
# and is intentionally excluded -- exactly as _sweep_bearing_disc stops a step
# short of the outer-race seat.
#
# Fail-on-old / pass-on-new is proven INLINE by the embedded self-test
# (_old_lflanged_yaw_hub): the reconstructed old hub WITH the lower flange
# blocks the upper inner race at the Phi 32 lip, while the live hub sweeps
# clean.

HUB_RACE_INSERTION_PATH_TOL_MM3 = 25.0
HUB_RACE_INSERTION_PATH_Z_STEP_MM = 0.5
HUB_RACE_INSERTION_PATH_PITCH_MM = 0.5


def _inner_race_annulus_points(zc: float, r_in: float, r_out: float,
                               pitch: float) -> np.ndarray:
    """Filled annular band (r in [r_in, r_out]) of sample points at coxa-local
    height ``zc`` -- the cross-section of a 6706 inner race (bore..OD band)."""
    n = int(np.ceil(2.0 * r_out / pitch)) + 1
    xs = np.linspace(-r_out, r_out, n)
    X, Y = np.meshgrid(xs, xs)
    rr = X ** 2 + Y ** 2
    m = (rr <= r_out ** 2) & (rr >= r_in ** 2)
    return np.column_stack([X[m], Y[m], np.full(int(m.sum()), zc)])


def _sweep_inner_race(mesh: trimesh.Trimesh, z_lo: float, z_seat_top: float):
    """Slide an inner-race annulus from below the boss (``z_lo``) UP to one Z
    step BELOW the race's seat-retain shoulder (``z_seat_top``) and return
    ``(worst_mm3, worst_z, n_blocked, n_steps)``.  Stopping a step below the
    shoulder keeps the intended seated contact (race resting against its
    shoulder) out of the blockage count; any real lip lives strictly BELOW the
    shoulder in the slide path and trips the tolerance."""
    r_in = hp.YAW_BEARING_ID / 2.0
    r_out = hp.YAW_BEARING_INNER_OD / 2.0
    pitch = HUB_RACE_INSERTION_PATH_PITCH_MM
    vox = pitch ** 3
    worst = 0.0
    worst_z = z_lo
    n_blocked = 0
    n_steps = 0
    z = z_lo
    z_end = z_seat_top - HUB_RACE_INSERTION_PATH_Z_STEP_MM
    while z <= z_end + 1e-6:
        n_steps += 1
        pts = _inner_race_annulus_points(z, r_in, r_out, pitch)
        overlap = int(points_inside(mesh, pts).sum()) * vox
        if overlap > worst:
            worst, worst_z = overlap, z
        if overlap > HUB_RACE_INSERTION_PATH_TOL_MM3:
            n_blocked += 1
        z += HUB_RACE_INSERTION_PATH_Z_STEP_MM
    return worst, worst_z, n_blocked, n_steps


def _old_lflanged_yaw_hub() -> trimesh.Trimesh:
    """Reconstruct the RETIRED hub central column (coxa-local) carrying BOTH
    inner-race retain flanges -- the lower flange (Phi YAW_BEARING_INNER_OD at
    z[-1, 0]) that trapped the UPPER inner race between it and the upper
    flange/platform.  Used ONLY by check_hub_inner_race_insertion_path's
    self-test to prove the probe is sensitive enough to flag the captured race
    the live single-flange hub eliminates."""
    rboss = hp.YAW_HUB_BOSS_OD / 2.0
    rinner = hp.YAW_BEARING_INNER_OD / 2.0
    plat_r = max(hp.YAW_HUB_OD, hp.YAW_HUB_DUST_LIP_OD) / 2.0

    def _cyl_at(r, z0, z1):
        c = hp._cyl(r, z1 - z0)
        c.apply_translation([0.0, 0.0, 0.5 * (z0 + z1)])
        return c

    boss = _cyl_at(rboss, hp.YAW_HUB_BOSS_BOT_Z, hp.YAW_HUB_BOSS_TOP_Z)
    lflange = _cyl_at(rinner, hp.YAW_BEARING_LOWER_TOP_Z, 0.0)        # z[-1, 0]
    uflange = _cyl_at(rinner, hp.YAW_BEARING_UPPER_TOP_Z,
                      hp.YAW_BEARING_UPPER_TOP_Z + 1.0)               # z[6, 7]
    plat = _cyl_at(plat_r, hp.YAW_HUB_BOSS_TOP_Z, hp.YAW_HUB_PLATFORM_Z1)
    return hp._union(boss, lflange, uflange, plat)


def check_hub_inner_race_insertion_path():
    """Sweep a rigid inner-race annulus up the coxa yaw HUB BOSS and assert a
    clear, monotone slide path to each 6706 inner race's seat exists (no
    feature of OD > the bore sits between the seat and the boss-bottom open
    end the race loads from).

    Includes an embedded FAIL-ON-OLD self-test: the retired two-flange hub is
    reconstructed and swept; the probe MUST flag the UPPER inner race as
    blocked at the lower flange's Phi 32 lip, proving it is sensitive enough to
    catch the trapped-race regression.  The live hub must then PASS.

    PRESS-FIT AWARENESS (Aug 16 2026): the hub boss deliberately carries a
    LIGHT PRESS against the inner races (YAW_HUB_BOSS_OD = bearing ID + 0.05
    -- the bench-measured -0.05 slip fit barely touched).  A designed
    few-hundredths press is hand/tap-assemblable and NOT an insertion hard
    stop, so the swept annulus bore is opened to the designed press surface
    (max(ID, BOSS_OD)); real regressions (e.g. the Phi 32 flange lip vs the
    Phi 25 bore) are MILLIMETRES of interference and still trip the probe."""
    r_in = max(hp.YAW_BEARING_ID, hp.YAW_HUB_BOSS_OD) / 2.0
    r_out = hp.YAW_BEARING_INNER_OD / 2.0
    print(f"\n[5h] Hub-boss inner-race insertion-path probe "
          f"(rigid Phi{2 * r_in:.0f}..Phi{2 * r_out:.0f} race annulus slides "
          f"UP the boss in {HUB_RACE_INSERTION_PATH_Z_STEP_MM:.1f} mm Z steps; "
          f"max overlap {HUB_RACE_INSERTION_PATH_TOL_MM3:.0f} mm^3 per step):")

    all_ok = True

    # The boss bottom open end the inner races load from.
    z_open = hp.YAW_HUB_BOSS_BOT_Z - 1.0

    # ---- Self-test: the OLD two-flange hub MUST flag the upper race ---------
    old = _old_lflanged_yaw_hub()
    ow, _owz, old_blocked, _ons = _sweep_inner_race(
        old, z_open, hp.YAW_BEARING_UPPER_TOP_Z)
    self_ok = old_blocked > 0
    all_ok &= _label(
        "SELF-TEST: retired two-flange hub traps the UPPER inner race",
        self_ok,
        f"reconstructed old hub flagged at {old_blocked} step(s); worst "
        f"overlap {ow:.1f} mm^3 >> tol "
        f"{HUB_RACE_INSERTION_PATH_TOL_MM3:.0f} mm^3 (probe is sensitive)")

    # ---- Live hub MUST pass for BOTH inner races ---------------------------
    # Aug 2026 one-piece merge: the hub is now the bottom end of the merged
    # printed coxa_link, so the sweep runs against the WHOLE printed part --
    # the fused hip bracket (z >= +18) is present while the races slide on
    # from below and must not intrude into the boss path.
    hub = hp.make_coxa_link_part()
    cases = [
        # (label, z_seat_top -- the race's retain shoulder; swept up to a step
        #  below it).  Both races load from the boss-bottom open end.
        ("coxa_link (hub end) UPPER inner race (slides up to the uflange seat)",
         hp.YAW_BEARING_UPPER_TOP_Z),    # +6 (uflange bottom is the seat stop)
        ("coxa_link (hub end) LOWER inner race (slides up to its z=-1 seat)",
         hp.YAW_BEARING_LOWER_TOP_Z),    # -1 (floating race; no over-boss lip)
    ]
    for name, z_seat_top in cases:
        worst, worst_z, n_blocked, n_steps = _sweep_inner_race(
            hub, z_open, z_seat_top)
        ok = n_blocked == 0
        detail = (f"{n_steps} Z steps probed; worst overlap {worst:.1f} mm^3 "
                  f"at z={worst_z:+.1f} (tol "
                  f"{HUB_RACE_INSERTION_PATH_TOL_MM3:.0f} mm^3)")
        if not ok:
            detail = (f"{n_blocked}/{n_steps} steps blocked; " + detail)
        all_ok &= _label(name, ok, detail)

    return all_ok


# ---------------------------------------------------------------------------
# ONE-PIECE-BEARING assembly-sequence probe (Jun 2026)
# ---------------------------------------------------------------------------
#
# The forensic gap that shipped two un-assemblable yaw 6706s: every prior
# insertion probe validates ONE race interface IN ISOLATION.
# ``check_bearing_insertion_path`` sweeps only the STATIONARY OUTER race into
# the chassis tower / cap (the cap pocket is "open-top" with the hub absent);
# ``check_hub_inner_race_insertion_path`` sweeps only the ROTATING INNER race
# onto the hub boss (the cap absent).  Each passes independently, but a 6706 is
# ONE RIGID UNIT (inner ring + balls + outer ring, inseparable, ID/OD/W fixed):
# the SAME single annulus must satisfy BOTH its inner (hub boss) AND its outer
# (chassis tower / yaw_bearing_cap) interface along ONE monotone axial
# insertion, with BOTH mating parts PRESENT.  Nothing enforced that.
#
# Forensic finding the cheap isolated probes missed (upper 6706, ID 30 / OD 37
# / W 4):
#   * the hub carries a Phi (YAW_BEARING_INNER_OD)=32 uflange + the Phi 51.6
#     turntable platform CAPPING the boss top, so a rigid annulus cannot come
#     DOWN (-Z) onto the boss from above (the platform is a solid disc the
#     annulus would have to pass through);
#   * the cap bored Phi 34 NECK (z[-1,+2]) BELOW its Phi 37 upper-race pocket,
#     so the same annulus cannot come UP (+Z) from below either (its Phi 37 OD
#     hits the Phi 34 neck).
# Inner interface only clears one direction, outer only the OTHER -> opposite
# directions -> NO single axial insertion exists for the one rigid bearing.
#
# This probe models each REAL bearing as ONE rigid annulus (true ID/OD/W) at
# its seated pose and, for the declared assembly order, sweeps it along BOTH
# axial directions against the union of the joint-stack parts PRESENT when that
# bearing is inserted (hub boss + chassis tower + yaw_bearing_cap), EXCLUDING
# only features that ship as SEPARATE post-install retainer parts (a retainer
# installed AFTER the bearing is legitimately absent during its insertion).
# A bearing is ASSEMBLABLE iff at least one axial direction has a clear,
# monotone, collision-free path to one step short of its seat -- i.e. NO
# feature on the inner approach with OD > the bearing bore AND NO feature on
# the outer approach with bore < the bearing OD, SIMULTANEOUSLY, along that one
# direction.  The probe FAILS the build if any bearing has NO clear direction.
#
# Embedded FAIL-ON-OLD self-test: a reconstructed platform-capped + uflanged
# boss UNION a Phi-34-necked cap traps the rigid upper annulus in BOTH
# directions; the probe MUST report it un-assemblable, proving it is sensitive
# enough to catch the captured-bearing regression the live split design fixes.

BEARING_ASSY_TOL_MM3 = 25.0       # per-step annulus-vs-parts overlap budget
BEARING_ASSY_Z_STEP_MM = 0.5
BEARING_ASSY_PITCH_MM = 0.5
BEARING_ASSY_APPROACH_MM = 8.0    # axial standoff swept on each side of seat

# Printed parts that ship as SEPARATE post-install retainers: legitimately
# ABSENT while a bearing is being inserted (they are bolted/snapped on AFTER).
# A feature only earns a place here if it is genuinely a discrete part the
# assembler adds after seating the bearing -- NOT an integral wall/shoulder.
_BEARING_ASSY_POST_INSTALL_RETAINERS = frozenset()


def _bearing_annulus_points(zc: float, r_in: float, r_out: float,
                            w: float, pitch: float) -> np.ndarray:
    """Filled SOLID annulus (r in [r_in, r_out], z in [zc-w/2, zc+w/2]) of
    sample points -- the true cross-section + WIDTH of one rigid 6706 ring
    stack, swept as a single inseparable body."""
    n = int(np.ceil(2.0 * r_out / pitch)) + 1
    xs = np.linspace(-r_out, r_out, n)
    X, Y = np.meshgrid(xs, xs)
    rr = X ** 2 + Y ** 2
    m = (rr <= r_out ** 2) & (rr >= r_in ** 2)
    nz = max(1, int(np.ceil(w / pitch)))
    zs = np.linspace(zc - w / 2.0 + pitch / 2.0,
                     zc + w / 2.0 - pitch / 2.0, nz)
    cols = [np.column_stack([X[m], Y[m], np.full(int(m.sum()), z)]) for z in zs]
    return np.vstack(cols)


def _bearing_overlap_mm3(obstacles, pts, vox) -> float:
    """Overlap (mm^3) of the sampled annulus ``pts`` against the obstacle SET.

    A point counts as solid if it lies inside ANY obstacle mesh.  Two
    deliberate robustness choices:

    * we OR the per-mesh memberships rather than boolean-UNION the meshes --
      the joint parts are coaxial RUNNING-CLEARANCE fits, and unioning them
      yields degenerate near-coincident facets that confuse point-in-mesh; and
    * we use ``mesh.contains`` (watertight winding test) directly rather than
      the 6-axis ray vote, because the ray vote SILENTLY UNDER-COUNTS the
      annulus inside the cap's thin Phi 34..44 annular neck wall (observed a
      real ~515 mm^3 constriction read as ~8 mm^3).  Every part the probe
      consults is guaranteed watertight by check_watertight, so contains() is
      both valid and exact here."""
    inside = None
    for obs in obstacles:
        if obs is None:
            continue
        hit = _points_inside_contains(obs, pts)
        inside = hit if inside is None else (inside | hit)
    return (int(inside.sum()) if inside is not None else 0) * vox


def _sweep_rigid_bearing(obstacles, seat_c: float,
                         r_in: float, r_out: float, w: float,
                         going_up: bool):
    """Slide the rigid bearing annulus along ONE axial direction toward its
    seat at center ``seat_c`` and return ``(worst_mm3, worst_z, n_blocked,
    n_steps)``.  ``obstacles`` is a LIST of meshes (see _bearing_overlap_mm3).

    ``going_up`` True  -> +Z insertion: approach from ``seat_c -
    BEARING_ASSY_APPROACH_MM`` UP to one step BELOW the seat.
    ``going_up`` False -> -Z insertion: approach from ``seat_c +
    BEARING_ASSY_APPROACH_MM`` DOWN to one step ABOVE the seat.
    Stopping one step short of the seat keeps the intended seated contact
    (annulus resting against its shoulder) out of the blockage count; a real
    constriction lives strictly along the approach and trips the tolerance."""
    if not isinstance(obstacles, (list, tuple)):
        obstacles = [obstacles]
    pitch = BEARING_ASSY_PITCH_MM
    vox = pitch ** 3
    step = BEARING_ASSY_Z_STEP_MM
    worst = 0.0
    worst_z = seat_c
    n_blocked = 0
    n_steps = 0
    zc = (seat_c - BEARING_ASSY_APPROACH_MM if going_up
          else seat_c + BEARING_ASSY_APPROACH_MM)
    z_end = seat_c - step if going_up else seat_c + step
    while (zc <= z_end + 1e-6) if going_up else (zc >= z_end - 1e-6):
        n_steps += 1
        pts = _bearing_annulus_points(zc, r_in, r_out, w, pitch)
        ov = _bearing_overlap_mm3(obstacles, pts, vox)
        if ov > worst:
            worst, worst_z = ov, zc
        if ov > BEARING_ASSY_TOL_MM3:
            n_blocked += 1
        zc += step if going_up else -step
    return worst, worst_z, n_blocked, n_steps


def _cyl_at(r, z0, z1):
    c = hp._cyl(r, z1 - z0)
    c.apply_translation([0.0, 0.0, 0.5 * (z0 + z1)])
    return c


def _reconstruct_trapped_upper_stack():
    """Reconstruct the RETIRED captured upper-6706 stack (coxa-local) as a LIST
    of obstacle meshes: a platform-capped + Phi 32 uflanged hub boss, plus a
    Phi-34-necked cap.  The rigid upper annulus is trapped in BOTH directions
    (platform blocks -Z, the Phi 34 neck blocks +Z).  Used ONLY by the
    self-test to prove the probe is sensitive.  Returned as separate meshes (no
    boolean union of the running-clearance hub vs cap)."""
    rboss = hp.YAW_HUB_BOSS_OD / 2.0
    rinner = hp.YAW_BEARING_INNER_OD / 2.0
    plat_r = max(hp.YAW_HUB_OD, hp.YAW_HUB_DUST_LIP_OD) / 2.0
    r_out = hp.YAW_BEARING_OD / 2.0 + hp.YAW_TOWER_WALL     # Phi 44
    r_bore = hp.YAW_TOWER_BORE_OD / 2.0                     # Phi 37
    r_neck = hp.YAW_TOWER_SHOULDER_OD / 2.0                 # Phi 34

    boss = _cyl_at(rboss, hp.YAW_HUB_BOSS_BOT_Z, hp.YAW_HUB_BOSS_TOP_Z)
    uflange = _cyl_at(rinner, hp.YAW_BEARING_UPPER_TOP_Z,
                      hp.YAW_BEARING_UPPER_TOP_Z + 1.0)      # z[6, 7]
    plat = _cyl_at(plat_r, hp.YAW_HUB_BOSS_TOP_Z, hp.YAW_HUB_PLATFORM_Z1)
    # boss/uflange/platform are coaxial STACKED solids -> a clean union.
    hub_recon = hp._union(boss, uflange, plat)
    # Old necked cap: Phi 37 pocket z[2,6] over a Phi 34 neck z[-1,2].
    cap = _cyl_at(r_out, hp.YAW_SPLIT_Z, hp.YAW_BEARING_UPPER_TOP_Z)
    cap = hp._diff(
        cap,
        _cyl_at(r_bore, hp.YAW_BEARING_UPPER_BOT_Z, hp.YAW_BEARING_UPPER_TOP_Z),
        _cyl_at(r_neck, hp.YAW_SPLIT_Z, hp.YAW_BEARING_UPPER_BOT_Z),
    )
    return [hub_recon, cap]


def check_bearing_assembly_sequence():
    """Model each REAL yaw 6706 as ONE rigid annulus and assert a concrete,
    monotone, single-direction AXIAL insertion exists for the FULL joint stack
    (hub boss + chassis tower + yaw_bearing_cap), with BOTH the inner and outer
    mating parts present (only declared SEPARATE post-install retainers absent).

    A 6706 is one inseparable unit, so the SAME annulus must clear its inner
    approach (no boss feature OD > the bore) AND its outer approach (no housing
    feature bore < the OD) along ONE axial direction.  FAILS if any bearing has
    NO clear direction -- the exact constraint the two isolated single-race
    probes structurally could not see.

    Embedded FAIL-ON-OLD self-test: the reconstructed platform/uflange boss +
    Phi-34-necked cap traps the rigid upper annulus both ways; the probe MUST
    report it un-assemblable.  The live split design must then PASS.

    PRESS-FIT AWARENESS (Aug 16 2026): same as the 5h inner-race probe -- the
    hub boss deliberately light-presses the inner races (BOSS_OD = ID + 0.05,
    bench refit), so the annulus bore sweeps at the designed press surface
    (max(ID, BOSS_OD)).  Hand/tap-assemblable designed press != hard stop;
    real blockers are millimetres over and still trip the tolerance."""
    r_in = max(hp.YAW_BEARING_ID, hp.YAW_HUB_BOSS_OD) / 2.0
    r_out = hp.YAW_BEARING_OD / 2.0
    w = hp.YAW_BEARING_W
    print(f"\n[5j] One-piece-bearing assembly-sequence probe "
          f"(rigid Phi{2 * r_in:.0f}/Phi{2 * r_out:.0f}/W{w:.0f} 6706 annulus "
          f"slides along a SINGLE axis with BOTH races' parts present; "
          f"max overlap {BEARING_ASSY_TOL_MM3:.0f} mm^3 per "
          f"{BEARING_ASSY_Z_STEP_MM:.1f} mm step):")

    all_ok = True

    def _assemblable(obstacle, seat_c, label):
        up = _sweep_rigid_bearing(obstacle, seat_c, r_in, r_out, w, True)
        dn = _sweep_rigid_bearing(obstacle, seat_c, r_in, r_out, w, False)
        up_ok = up[2] == 0
        dn_ok = dn[2] == 0
        ok = up_ok or dn_ok
        if up_ok:
            why = (f"+Z insertion CLEAR ({up[3]} steps, worst {up[0]:.1f} mm^3 "
                   f"@z={up[1]:+.1f})")
        elif dn_ok:
            why = (f"-Z insertion CLEAR ({dn[3]} steps, worst {dn[0]:.1f} mm^3 "
                   f"@z={dn[1]:+.1f})")
        else:
            why = (f"NO clear axis: +Z {up[2]}/{up[3]} steps blocked "
                   f"(worst {up[0]:.1f} mm^3 @z={up[1]:+.1f}); "
                   f"-Z {dn[2]}/{dn[3]} steps blocked "
                   f"(worst {dn[0]:.1f} mm^3 @z={dn[1]:+.1f})")
        return _label(label, ok, why)

    # ---- Self-test: reconstructed captured upper stack MUST be un-assemblable
    trapped = _reconstruct_trapped_upper_stack()   # list of meshes
    seat_upper = 0.5 * (hp.YAW_BEARING_UPPER_BOT_Z + hp.YAW_BEARING_UPPER_TOP_Z)
    up = _sweep_rigid_bearing(trapped, seat_upper, r_in, r_out, w, True)
    dn = _sweep_rigid_bearing(trapped, seat_upper, r_in, r_out, w, False)
    self_ok = (up[2] > 0) and (dn[2] > 0)
    all_ok &= _label(
        "SELF-TEST: reconstructed platform+uflange / Phi34-neck stack traps "
        "the rigid upper 6706 BOTH ways",
        self_ok,
        f"+Z {up[2]}/{up[3]} blocked (worst {up[0]:.1f} mm^3); "
        f"-Z {dn[2]}/{dn[3]} blocked (worst {dn[0]:.1f} mm^3) -- probe is "
        f"sensitive")

    # ---- Live joint stack: build the real obstacle meshes in coxa-local -----
    # NB: kept as a LIST (never boolean-unioned) -- the hub, cap and tower are
    # coaxial running-clearance fits, and unioning them yields degenerate
    # near-coincident facets that make point-in-mesh non-deterministic.
    # Aug 2026 one-piece merge: the "hub" obstacle is the whole printed
    # coxa_link (the hip bracket is fused on and PRESENT while each race
    # slides onto the boss from below).
    hub = hp.make_coxa_link_part()
    cap = hp.make_yaw_bearing_cap()
    if "coxa_link" in _BEARING_ASSY_POST_INSTALL_RETAINERS:
        hub = None
    if "yaw_bearing_cap" in _BEARING_ASSY_POST_INSTALL_RETAINERS:
        cap = None
    # Lower-race housing is the SPLIT bottom half (``chassis_bottom``): its tower
    # is an OPEN-TOP Phi 37 pocket capped at the split plane (z=-1), so nothing
    # sits above the lower race when it is seated.  (The un-split
    # ``chassis_assembled`` still carries the full Phi 37->34 upper tower up to
    # z=+7 -- the geometry the bolt-on cap REPLACES -- so feeding it here would
    # falsely trap the descending lower annulus against features that are not
    # present until after the cap is installed.)
    tower = _chassis_yaw_cradle_to_coxa_local(
        _load_mesh("chassis_bottom", copy=False))

    # Upper 6706: inner = hub boss, outer = yaw_bearing_cap (both present).
    all_ok &= _assemblable(
        [hub, cap], seat_upper,
        "upper 6706 (one rigid annulus vs hub boss + yaw_bearing_cap)")

    # Lower 6706: outer = chassis bottom tower (its open-top Phi 37 pocket).
    # Per the declared assembly order the lower race is the FIRST thing seated
    # -- dropped straight down onto its z=-5 tower shoulder BEFORE the hub boss
    # or the cap are present (both are installed after), so the tower is the
    # only part present.  The boss-side (inner) path is independently covered
    # by check_hub_inner_race_insertion_path.
    seat_lower = 0.5 * (hp.YAW_BEARING_LOWER_BOT_Z + hp.YAW_BEARING_LOWER_TOP_Z)
    all_ok &= _assemblable(
        [tower], seat_lower,
        "lower 6706 (one rigid annulus vs chassis bottom tower)")

    return all_ok


# ---------------------------------------------------------------------------
# Screwdriver-access clearance check
# ---------------------------------------------------------------------------
#
# For each fastener, probe a cylinder pointing OUTWARD from the head
# along the driver-approach direction (= -axis_world).  Fail if any
# printed part intrudes into that cylinder by more than
# DRIVER_INTRUSION_TOLERANCE_MM3 mm^3.
#
# The probe cylinder size is picked per-fastener based on the kind of
# tool the assembler actually needs to drive that fastener.  Probing
# every SHCS with the worst-case Phillips envelope (12 mm dia x 80 mm
# long) overstates the required clearance by a large margin and
# produces FAILs at fasteners that a real assembler can drive with
# their fingers and a 25 mm-long L-shaped hex key with no trouble.
#
# The three envelopes:
#
# * HEX_KEY  --  L-shaped hex key short-arm (M2.5 / M3 wrench).  The
#                short arm is ~25 mm long and ~3 mm AF; the assembler
#                needs maybe 8 mm of clearance around it for fingers
#                and the long-arm sticking out to the side.  Anything
#                tagged SHCS (or the M2.5 spline center screw, which
#                IS a tiny Phillips on hobby servos but is always
#                SKIPped because it sits captive under the disc horn
#                after assembly -- mapping it here is documentation
#                only, the cone never lands on actual geometry).
#
# * PHILLIPS  -- Full-length Phillips / slotted screwdriver: 12 mm
#                shaft + handle, 80 mm reach.  Used for the foot
#                hinge M3x16 pan-head and any other pan-head or
#                slotted-head fastener.
#
# * SOCKET    -- Small M3-nut driver / 5.5 mm socket on a stub
#                handle.  12 mm OD at the head, ~50 mm of reach
#                (shorter than Phillips because nut drivers don't
#                need to land on a recessed cross-shape).  Used for
#                the M3 nyloc nuts that aren't captive in a hex
#                pocket (e.g. the chassis bolt nuts under the
#                chassis_bottom, the foot hinge nut).
#
# DRIVER_INTRUSION_TOLERANCE_MM3 mirrors the OTHER_TOLERANCE = 30 mm^3
# used by check_servo_clearance: at the verifier's 1.5 mm voxel pitch
# a single voxel = 3.4 mm^3, so 30 mm^3 ~= 9 voxels = the noise floor
# for the axis-aligned-ray parity vote on a curved mesh boundary.
# Anything below it is voxel stair-step artefact, not real material
# in the cone.

HEX_KEY_CLEARANCE_DIA_MM     = 8.0    # 2.5 / 3 mm hex key short arm + finger room
HEX_KEY_CLEARANCE_LEN_MM     = 30.0   # short-arm reach
# M2 SHCS (the retired link-to-X-horn bolts; kept as a precedent for
# the disc-horn block below) uses a Phi 1.5 mm hex key.  The bolt head
# is recessed in a Phi M2_HEAD_OD_CLEARANCE = 4.0 mm counter-bore in
# the printed cap / pad above the bolt; the hex key short arm enters
# the counter-bore
# from above so the envelope diameter is bounded by the counter-bore
# clearance (Phi 4 mm) not the larger 8 mm "finger room" diameter
# used for M2.5 / M3 hex keys (those keys are driven with two fingers
# pinched around the long arm, whereas the M2 key is small enough to
# turn with a single finger inside the counter-bore + trough void).
# The short-arm reach is shorter too (typical M2 hex key short arm =
# ~ 10 mm) but we round up to 15 mm to allow some additional clear
# space above the head for finger access.
M2_HEX_KEY_CLEARANCE_DIA_MM  = 4.0    # = M2_HEAD_OD_CLEARANCE
M2_HEX_KEY_CLEARANCE_LEN_MM  = 15.0   # short-arm reach + finger headroom
# June 2026 disc-horn switch: the link-to-disc-horn bolts are M3 x 6
# SHCS recessed in a Phi DISC_HORN_BOLT_HEAD_OD = 5.7 mm counter-bore
# in the link pad and driven with a 2.5 mm hex key whose short arm
# enters the counter-bore from above (exactly like the old M2 link-to-
# X-horn bolts).  Because the bolts now sit on the tighter PCD 14 mm
# circle (radius 7, well inside each pad) instead of the old PCD 20.8
# (radius 10.4, near the pad edge), the generic Phi 8 mm "finger room"
# HEX_KEY cone would clip the pad annulus ringing each counter-bore
# (2.85..4 mm from the bolt axis) and falsely report a self-intrusion.
# The driver clearance is bounded by the counter-bore (the hex key
# turns inside it), so we use the counter-bore diameter as the
# envelope -- mirrors the M2 precedent above.
DISC_HORN_HEX_KEY_CLEARANCE_DIA_MM = 5.7   # = hp.DISC_HORN_BOLT_HEAD_OD
DISC_HORN_HEX_KEY_CLEARANCE_LEN_MM = 15.0  # short-arm reach + finger headroom
PHILLIPS_CLEARANCE_DIA_MM    = 12.0   # full-length Phillips shaft + handle
PHILLIPS_CLEARANCE_LEN_MM    = 80.0
SOCKET_CLEARANCE_DIA_MM      = 12.0   # M3 nut driver / 5.5 mm socket + handle taper
SOCKET_CLEARANCE_LEN_MM      = 50.0

DRIVER_INTRUSION_TOLERANCE_MM3 = 30.0
# Probe slightly forward of the head so the cylinder NEVER overlaps
# the head's own seating face (which would otherwise also overlap the
# part the head bolts to, and falsely report it as an intrusion).
DRIVER_HEAD_STANDOFF_MM      = 0.5

# Names of every printed part that COULD possibly intrude into a
# driver-access cone.  Chassis-level + leg-0 only -- the 6-fold
# symmetric chassis layout means the leg 0 fastener vs leg 0 parts
# intrusion pattern is identical to all 6 legs.
_DRIVER_PRINTED_PART_NAMES = (
    "chassis_bottom", "chassis_top", 
    "hip_clamp_cap", "knee_clamp_cap", "switch_holster", 
    # tibia_link already includes the pressed-on TPU foot boot
    # (make_tibia_link unions it) -- no separate foot part any more.
    "coxa_link", "femur_link", "tibia_link",
)


def _build_world_leg0_printed_parts() -> dict:
    """Return ``{part_name: world-frame trimesh}`` for every printed
    part that could obstruct a driver-access cone on leg 0.  Mirrors
    ``_build_standing_leg`` for the legged parts, and uses the same
    chassis-fixed transforms for the central parts (chassis plates,
    battery, electronics).
    """
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = 0.5 * np.pi / 3
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    z_hat = np.array([0.0, 0.0, 1.0])

    yaw_output_z = hp.CHASSIS_YAW_OUTPUT_Z
    hip_drop = hp.COXA_HIP_DROP
    hip_joint_local = np.array(hp.COXA_HIP_ANCHOR)
    # Bearing-sandwich refit (Jun 2026): femur / tibia local origins are
    # the disc-horn-top ON the joint axis -- no axial pad offset.
    PAD_AXIS_OFFSET = np.array([0.0, 0.0, 0.0])

    p = np.deg2rad(hp.STANCE_FEMUR_DEG)
    pt = np.deg2rad(hp.STANCE_FEMUR_DEG + hp.STANCE_TIBIA_DEG)
    Ry_p = rotation_matrix(p, [0, 1, 0])[:3, :3]
    knee_joint_local = (hip_joint_local
                        + Ry_p @ np.array([hp.FEMUR_LENGTH, 0, 0]))

    parts: dict = {}

    # Chassis-level parts (transforms match inspect_build's frame).
    plate_t = hp.CHASSIS_PLATE_T
    gap = hp.CHASSIS_GAP
    # Jun 2026 split: the WORLD assembly represents the two bolted halves
    # by their (un-split) integrated solid so every world-geometry probe
    # (screwdriver access, cable clearance, harness reach, cradle openness,
    # fastener engagement) sees exactly the geometry the bolted HIGH+LOW
    # halves reproduce.  The flat HIGH plate + bolt-on LOW cradle plate are
    # validated as discrete printed parts by the watertight / flimsy /
    # thin-sheet / flat-bottom checks.
    cb_bottom = _load_mesh("chassis_assembled")
    parts["chassis_bottom"] = cb_bottom

    cb_top = _load_mesh("chassis_top")
    cb_top.apply_translation([0.0, 0.0, hp.CHASSIS_TOP_CENTRE_Z])
    parts["chassis_top"] = cb_top

    # Aug 2026: printed uno_q_tray / buck_tray RETIRED (as-built magnet hex).
    # Fastener / screwdriver checks no longer require those meshes.

    # Hip + knee sandwich-joint clamp caps on leg 0 (same world frame as
    # the servo bodies).  Keyed separately so both survive in the dict.
    caps = _place_servo_clamp_caps()
    parts["hip_clamp_cap"] = caps["hip_clamp_cap"]
    parts["knee_clamp_cap"] = caps["knee_clamp_cap"]

    # Yaw anti-rotation saddle on leg 0 (placed by the same Ra + edge_mid as
    # _place_yaw_retainers).  Placing it here lets check_screwdriver_access +
    # check_fastener_engagement actually SEE the part, so the 2 saddle->floor
    # anchor screws are guarded (the old undrivable stirrup anchors weren't
    # even in the registry, a blind spot this redesign closes).
    parts["yaw_servo_retainer"] = _place_yaw_retainers()["yaw_servo_retainer"]

    # switch_holster: velcroed FLAT on chassis_top's top face (Aug 16
    # 2026: the 2 insert bosses + bolt-down ear are retired; floor
    # bottom sits directly at z = CHASSIS_TOP_TOP_Z = 36 design frame).
    chassis_top_top_z = hp.CHASSIS_TOP_TOP_Z  # = 36 (design)
    sh = _load_mesh("switch_holster")
    sh.apply_translation([
        hp.SWITCH_HOLSTER_CENTRE_X,
        hp.SWITCH_HOLSTER_CENTRE_Y,
        chassis_top_top_z,
    ])
    parts["switch_holster"] = sh

    # Aug 2026: imu_pad RETIRED — MPU sits under the raised platform top.

    # Leg-0 printed parts -- mirrors ``_build_standing_leg`` so the
    # legged-part transforms exactly match the registry's leg_index=0
    # fastener positions.
    cl = _load_mesh("coxa_link")
    cl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    cl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["coxa_link"] = cl

    # yaw_bearing_cap (Jun 2026 split): bolt-on TOP half of the yaw-bearing
    # tower.  Built in the coxa-local frame (z=0 = disc-horn top, coaxial with
    # the yaw axis) -- identical to make_coxa_yaw_hub -- so it takes the SAME
    # per-leg placement transform as coxa_link.  Placing it here lets the
    # cap-to-tower join bolts confirm they bear on the cap ear and bridge the
    # cap + chassis_bottom tower.
    yc = _load_mesh("yaw_bearing_cap")
    yc.apply_transform(rotation_matrix(a, [0, 0, 1]))
    yc.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["yaw_bearing_cap"] = yc

    fl = _load_mesh("femur_link")
    fl.apply_transform(rotation_matrix(p, [0, 1, 0]))
    fl.apply_translation(hip_joint_local + PAD_AXIS_OFFSET)
    fl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    fl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["femur_link"] = fl

    tl = _load_mesh("tibia_link")
    tl.apply_transform(rotation_matrix(pt, [0, 1, 0]))
    tl.apply_translation(knee_joint_local + PAD_AXIS_OFFSET)
    tl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    tl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["tibia_link"] = tl

    # Aug 2026: no separate foot part -- the TPU foot_boot is unioned
    # into the tibia_link mesh placed above (and carries no fasteners).

    return parts


def _driver_envelope_for_spec(spec: str) -> tuple:
    """Return ``(diameter_mm, length_mm)`` of the driver clearance
    cone appropriate for ``spec`` -- one of the fastener-registry
    ``SPEC_*`` strings (e.g. ``"M3x8 SHCS"`` or ``"M3 nyloc nut"``).

    Dispatch (case-sensitive, ordered by precedence -- first match
    wins because some specs contain more than one of the marker
    words):

    * Anything containing ``"spline"`` -> HEX_KEY envelope.  The
      hobby servo's M2.5 spline center screw IS a small Phillips in
      reality, but it is also explicitly SKIPped in every fastener
      registry instance (captive under the disc horn after assembly),
      so its cone never lands on actual geometry.  Mapping it to
      HEX_KEY here is purely so an analyst dumping the per-spec
      envelope table sees the SMALLEST plausible envelope and isn't
      misled into thinking we modelled an 80 mm Phillips reach
      through the disc horn.
    * Anything containing ``"SHCS"`` -> HEX_KEY envelope.
    * Anything containing ``"pan-head"``, ``"Phillips"``, or
      ``"slotted"`` -> PHILLIPS envelope.
    * Anything containing ``"nyloc nut"`` or just ``"nut"`` ->
      SOCKET envelope.

    An unknown spec falls through to the conservative PHILLIPS
    envelope.  Future fastener-spec drift will show up as a probe
    that's larger than expected, not as a silently-permissive one.
    """
    if "spline" in spec:
        return (HEX_KEY_CLEARANCE_DIA_MM, HEX_KEY_CLEARANCE_LEN_MM)
    if spec.startswith("M2x") and "SHCS" in spec:
        # M2 SHCS (the retired link-to-X-horn bolts) uses a small Phi
        # 1.5 mm hex key driving into a Phi
        # 4 mm counter-bore in the printed link cap / pad.  Use the
        # narrower M2 envelope so the surrounding pad material that
        # rings the counter-bore (annulus 2..4 mm from bolt axis)
        # isn't falsely reported as a driver intrusion; see the M2_
        # HEX_KEY_CLEARANCE_* constants block above for the
        # derivation.
        return (M2_HEX_KEY_CLEARANCE_DIA_MM, M2_HEX_KEY_CLEARANCE_LEN_MM)
    if spec == "M3x6 SHCS":
        # June 2026 disc-horn switch: link-to-disc-horn bolts (M3 x 6
        # SHCS) are recessed in a Phi 5.7 mm counter-bore on the tight
        # PCD 14 mm circle and turned with a 2.5 mm hex key whose short
        # arm enters the counter-bore.  Bound the envelope by the
        # counter-bore diameter -- same reasoning as the M2 branch
        # above; see the DISC_HORN_HEX_KEY_CLEARANCE_* constants.  The
        # M3 x 8 cradle bolts keep the wide Phi 8 mm envelope (they are
        # driven through the servo ear in open space, not counter-bored).
        return (DISC_HORN_HEX_KEY_CLEARANCE_DIA_MM,
                DISC_HORN_HEX_KEY_CLEARANCE_LEN_MM)
    if "SHCS" in spec:
        return (HEX_KEY_CLEARANCE_DIA_MM, HEX_KEY_CLEARANCE_LEN_MM)
    if ("pan-head" in spec) or ("Phillips" in spec) or ("slotted" in spec):
        return (PHILLIPS_CLEARANCE_DIA_MM, PHILLIPS_CLEARANCE_LEN_MM)
    if "nut" in spec:
        return (SOCKET_CLEARANCE_DIA_MM, SOCKET_CLEARANCE_LEN_MM)
    return (PHILLIPS_CLEARANCE_DIA_MM, PHILLIPS_CLEARANCE_LEN_MM)


def _make_driver_probe(head_xyz: np.ndarray,
                       axis_world: np.ndarray,
                       dia_mm: float = PHILLIPS_CLEARANCE_DIA_MM,
                       length_mm: float = PHILLIPS_CLEARANCE_LEN_MM,
                       ) -> trimesh.Trimesh:
    """Build a ``dia_mm`` x ``length_mm`` clearance probe oriented
    along the DRIVER APPROACH direction (= -axis_world, since
    axis_world points FROM head INTO material and the driver
    approaches FROM OUTSIDE the head).

    The default envelope is PHILLIPS (12 x 80 mm) so callers that
    don't pass a per-fastener size get the conservative cone.
    ``_driver_envelope_for_spec`` is the helper that picks the right
    envelope for a real ``FastenerInstance.spec`` string.
    """
    cyl = trimesh.creation.cylinder(
        radius=dia_mm / 2.0,
        height=length_mm,
        sections=24,
    )
    # cylinder() is centred at the origin along +Z.  Shift it so the
    # BASE sits at z=0 (the head plane) and the tip at z=+LEN.  We
    # then rotate so +Z lands on the driver direction (-axis_world)
    # and translate to head_xyz.
    cyl.apply_translation([0.0, 0.0, length_mm / 2.0
                                       + DRIVER_HEAD_STANDOFF_MM])
    driver_dir = -np.asarray(axis_world, dtype=float)
    n = float(np.linalg.norm(driver_dir))
    if n < 1e-12:
        driver_dir = np.array([0.0, 0.0, 1.0])
    else:
        driver_dir = driver_dir / n
    # Rotation: map mesh +Z onto driver_dir.  Build via cross product.
    z_hat = np.array([0.0, 0.0, 1.0])
    cos_t = float(np.clip(np.dot(z_hat, driver_dir), -1.0, 1.0))
    if cos_t > 1.0 - 1e-9:
        R = np.eye(4)
    elif cos_t < -1.0 + 1e-9:
        # 180-degree flip about any perpendicular axis -- use +X.
        R = rotation_matrix(np.pi, [1.0, 0.0, 0.0])
    else:
        axis = np.cross(z_hat, driver_dir)
        axis = axis / float(np.linalg.norm(axis))
        angle = float(np.arccos(cos_t))
        R = rotation_matrix(angle, axis)
    cyl.apply_transform(R)
    cyl.apply_translation(np.asarray(head_xyz, dtype=float))
    return cyl


FLAT_BOTTOM_MAX_PROTRUSION_MM = 5.0   # mm below the main bed plane -> support


def check_flat_bottom():
    """Flat-bottom printability guard (Jun 2026).

    Mirrors ``_flatbottom_check.py``: build each PRINTED part, apply the SAME
    print-orientation reorient that ``print_orientation.PART_REGISTRY``
    uses, and FAIL if any downward face hangs more than
    ``FLAT_BOTTOM_MAX_PROTRUSION_MM`` below the part's largest downward (bed)
    plane.  A large protrusion means the part rests on small feet with a big
    floating overhang that needs supports -- exactly the bug that left the
    single-piece ``chassis_bottom`` resting on 6 cradle rings (~16% bed
    contact, 20.5 mm of plate floating).

    Also proves the guard is SENSITIVE: the OLD bucketed integrated solid
    (``_chassis_bottom_full_solid``, the pre-merge geometry with the 6 deep
    cradle buckets still hanging below the plate), dropped to bed as it would
    have printed, MUST be REJECTED -- while the merged single ``chassis_bottom``
    (tested in the PART_REGISTRY sweep above) PASSES.  That fail-bucketed /
    pass-merged pair is the regression proof that the floor-fold kept the part
    genuinely flat-bottom rather than re-introducing the overhang.
    """
    import _flatbottom_check as fb        # noqa: WPS433
    import print_orientation as px         # noqa: WPS433

    print(f"\n[6c] Flat-bottom printability "
          f"(max overhang below bed plane <= "
          f"{FLAT_BOTTOM_MAX_PROTRUSION_MM:.1f} mm):")
    all_ok = True

    for (fname, make_fn, reorient_fn, *_rest) in px.PART_REGISTRY:
        name = fname[:-4] if fname.endswith(".stl") else fname
        mesh = reorient_fn(make_fn())
        m = fb.flatness_metrics(mesh)
        prot = m["protrusion_below_main"]
        ok = prot <= FLAT_BOTTOM_MAX_PROTRUSION_MM
        all_ok &= _label(
            f"{name:24s} flat bottom", ok,
            f"overhang below bed plane = {prot:5.2f} mm; "
            f"bed contact {100.0 * m['bed_fraction']:4.1f}% of footprint")

    # --- Regression proof: the OLD bucketed integrated solid MUST fail ------
    old = hp._chassis_bottom_full_solid()
    old = old.copy()
    old.apply_translation([0.0, 0.0, -float(old.bounds[0, 2])])  # drop to bed
    old_prot = fb.flatness_metrics(old)["protrusion_below_main"]
    old_fails = old_prot > FLAT_BOTTOM_MAX_PROTRUSION_MM
    all_ok &= _label(
        "OLD bucketed chassis_bottom correctly REJECTED", old_fails,
        f"overhang below bed plane = {old_prot:5.2f} mm "
        f"(> {FLAT_BOTTOM_MAX_PROTRUSION_MM:.1f} mm -> the buckets had to go; "
        f"the merged floor-fold keeps the printed part flat)")

    return all_ok


def check_screwdriver_access():
    """For each fastener instance, probe a per-fastener cylindrical
    clearance cone above the head along the driver-approach axis.
    Fail if any PRINTED part intrudes by more than
    DRIVER_INTRUSION_TOLERANCE_MM3.

    Three envelopes are dispatched by ``FastenerInstance.spec`` (see
    ``_driver_envelope_for_spec``):

    * HEX_KEY  (Phi  8 mm x 30 mm) -- SHCS + spline-screw entries
    * PHILLIPS (Phi 12 mm x 80 mm) -- pan-head / Phillips / slotted
    * SOCKET   (Phi 12 mm x 50 mm) -- nyloc nut / generic nut

    Fasteners with an explicit ``skip_screwdriver_reason`` -- the
    servo's M2.5 spline center screw (captive under the disc horn after
    assembly) and the cradle captive nyloc nuts (held in the Design C
    hex pocket; the bolt is driven from the head side) -- are SKIPped
    with the reason printed alongside.
    """
    print("\n[14] Screwdriver-access clearance "
          f"(per-spec envelopes: "
          f"HEX_KEY {HEX_KEY_CLEARANCE_DIA_MM:.0f}x{HEX_KEY_CLEARANCE_LEN_MM:.0f} mm, "
          f"PHILLIPS {PHILLIPS_CLEARANCE_DIA_MM:.0f}x{PHILLIPS_CLEARANCE_LEN_MM:.0f} mm, "
          f"SOCKET {SOCKET_CLEARANCE_DIA_MM:.0f}x{SOCKET_CLEARANCE_LEN_MM:.0f} mm; "
          f"tol {DRIVER_INTRUSION_TOLERANCE_MM3:.0f} mm^3):")

    # Import here so the verifier never tries to load the registry at
    # module import time (avoids circular import surprises if the
    # registry grows a dependency on this module).
    import fastener_registry  # noqa: WPS433

    fasteners = fastener_registry.build_all_fastener_instances()
    # 6-fold symmetric chassis layout -> testing leg 0 suffices for
    # every per-leg fastener.  We DO need every chassis-level entry
    # (leg_index None) and leg-0 entries.
    sample = [fi for fi in fasteners
              if fi.leg_index is None or fi.leg_index == 0]

    print(f"  Probing {len(sample)} fastener(s) on leg 0 + chassis-level "
          f"(of {len(fasteners)} total; the other 5 legs are 6-fold "
          f"rotationally symmetric).")

    world_parts = _build_world_leg0_printed_parts()

    # Pre-compute every part's AABB so we can skip the volumetric
    # voxel test for the 95 %+ of (part, probe) pairs that don't even
    # overlap loosely.
    part_aabbs = {name: m.bounds for name, m in world_parts.items()}

    all_ok = True
    n_skip = 0
    n_check = 0
    n_fail = 0
    grouped: dict[str, list[str]] = {}

    for fi in sample:
        if fi.skip_screwdriver_reason is not None:
            n_skip += 1
            grouped.setdefault("SKIP", []).append(
                f"{fi.role}: {fi.skip_screwdriver_reason}"
            )
            continue
        n_check += 1
        dia, length = _driver_envelope_for_spec(fi.spec)
        probe = _make_driver_probe(fi.head_world_xyz, fi.axis_world,
                                    dia_mm=dia, length_mm=length)
        p_lo, p_hi = probe.bounds
        total_intrusion = 0.0
        per_part: list[str] = []
        for part_name, part_mesh in world_parts.items():
            a_lo, a_hi = part_aabbs[part_name]
            if np.any(p_hi <= a_lo) or np.any(a_hi <= p_lo):
                continue
            vol = _pair_overlap_volume(probe, part_mesh, pitch=1.5)
            if vol > 0.0:
                total_intrusion += vol
                per_part.append(f"{part_name}={vol:.1f}")
        ok = total_intrusion <= DRIVER_INTRUSION_TOLERANCE_MM3
        if not ok:
            n_fail += 1
            detail = (
                f"intrusion {total_intrusion:6.1f} mm^3 "
                f"(tol {DRIVER_INTRUSION_TOLERANCE_MM3:.0f})  "
                f"[{', '.join(per_part)}]"
            )
            _label(fi.role, False, detail)
            all_ok = False

    # Always report the SKIP block (even when every probed fastener
    # passes) so a casual reader can see the explicit allow-list
    # without rerunning with --serial.
    if "SKIP" in grouped:
        print(f"  ---- SKIPped ({n_skip}) ----")
        for line in grouped["SKIP"][:8]:
            print(f"     SKIP  {line}")
        if len(grouped["SKIP"]) > 8:
            print(f"     SKIP  ... and {len(grouped['SKIP']) - 8} more "
                  f"(rotationally symmetric copies on other legs)")
    if all_ok:
        _label(
            f"{n_check} probed (+{n_skip} explicit SKIPs)",
            True,
            "every driver cone is clear",
        )
    else:
        print(f"  ---- FAIL ({n_fail} of {n_check} probed) ----")
    return all_ok


# ---------------------------------------------------------------------------
# Assembly-integrity checks (May 2026, post-coxa_link-pedestal-audit)
# ---------------------------------------------------------------------------
#
# The 14-check verifier above silently allowed the May 2026 ``coxa_link``
# pedestal regression: the body-insertion trough in ``make_coxa_link``
# slices the entire BOTTOM 25.5 mm out of the 34x34x36 pedestal, leaving
# only a ~10.5 mm cap on top.  The 4 M2 X-horn bolts in
# ``fastener_registry._emit_horn_fasteners_yaw`` were also pinned at
# link-local z = COXA_LIFT + hub_t = 44 mm (TOP of the hub, NOT at the
# link's bottom face) -- so the bolts float entirely inside the hub and
# DO NOT clamp the link to the X-horn at all.  Both bugs survived because
# every existing check probed parts in isolation; nothing tested the
# integrity of the FULL assembly (bolt joins parts, parts mate at faces).
#
# We add two structural checks that together close the gap:
#
# * ``check_fastener_engagement`` -- for every bolt with a length, the
#   head must bear on solid material, the shaft must traverse a single
#   contiguous run of material/closely-mated-parts (no long air spans),
#   and the bolt must JOIN at least 2 distinct printed/horn parts.  The
#   "joins >= 2 parts" rule is the one that catches the yaw mis-placement
#   above (bolt at z = [36, 44] is fully inside the coxa_link hub and
#   doesn't reach the X-horn at z = [-5, 0]).
#
# * ``check_mating_face_contact`` -- a small explicit list of mating
#   faces (coxa_link bottom <-> yaw disc-horn top, femur_link hip pad
#   <-> hip disc horn, tibia_link knee pad <-> knee disc horn,
#   coxa_bracket flange <-> chassis_bottom, boot <-> tibia tube
#   clevis).  For each
#   face we scan along the joint axis and confirm the two parts mate
#   within ``tolerance_mm``.  The hollowed-out coxa_link pedestal shows
#   up here as a ~25 mm gap at the yaw mating face.
#
# Both checks live in the SAME ``_verify_prototype.py`` module so they
# pick up the existing ``_load_mesh`` cache + ``points_inside``
# dispatcher + worker process pool -- no separate machinery.

# Per-fastener engagement geometry.  ``head_od`` drives the head-bearing
# radial probes, ``shaft_od`` drives the shaft / engagement radial
# probes (we offset by 0.4 mm so the probe sits just OUTSIDE the
# drilled bolt clearance hole -- otherwise every probe sits inside the
# Phi 3.4 mm / Phi 3.2 mm clearance cylinder and reports "AIR"),
# ``engagement_mm`` is the expected thread-engagement length at the
# tip.  Default values are used for any spec missing from the table.
FASTENER_ENGAGEMENT_SPEC = {
    # ``M3x6 SHCS`` -- link-to-disc-horn bolts (June 2026 disc-horn
    # switch).  The M3 x 6 SHCS threads into the 20 mm aluminum 25T
    # disc's M3 TAPPED hole; the aluminium is the thread-engagement
    # medium.  engagement_mm = DISC_HORN_BOLT_THREAD_ENGAGEMENT_MM = 3.0 mm
    # (worst case, the femur / tibia pads = LINK_THICKNESS 6 mm leave
    # 3 mm of bolt in the disc; the coxa cap leaves ~5 mm).  The
    # engagement-zone window (the last 3 mm of the bolt) sits fully
    # inside the 5 mm disc, so TIP_ENGAGEMENT_MIN_FRACTION = 0.5 is
    # comfortably met.
    "M3x6 SHCS":                       dict(head_od=5.5, shaft_od=3.0, engagement_mm=3.0),
    # ``M3x8 disc-horn SHCS`` -- Jun 2026 flush-horn fix.  The real disc is
    # only DISC_HORN_H = 2 mm, so the bolt threads the FULL 2 mm disc (the
    # driven mount's printed reach-down boss carries the rest of the grip).
    # engagement_mm = DISC_HORN_H so the tip-engagement window is exactly the
    # 2 mm aluminium band -- on the yaw joint (oversized torque-only clearance
    # bore) the shaft floats above the disc, so a wider window would dip into
    # air and falsely fail; on hip/knee (tight bore) the column is solid and
    # the 2 mm window is comfortably met.
    "M3x8 disc-horn SHCS":             dict(head_od=5.5, shaft_od=3.2, engagement_mm=2.0),
    # ``M3x10 disc-horn SHCS`` -- Jun 2026 yoke-width fix.  Same aluminium-disc
    # thread target as the M3 x 8 disc-horn bolt (engagement_mm = DISC_HORN_H =
    # 2 mm), but 2 mm longer because the DRIVEN (flush-output) yoke top-arm pad
    # bridges DRIVEN_HORN_REACH_DOWN = 5 mm before reaching the disc.
    "M3x10 disc-horn SHCS":            dict(head_od=5.5, shaft_od=3.2, engagement_mm=2.0),
    # ``M3x20 disc-horn SHCS`` -- Aug 2026: coxa_yaw_hub tall stack (bench).
    # Same aluminium-disc thread target (engagement_mm = DISC_HORN_H = 2).
    "M3x20 disc-horn SHCS":            dict(head_od=5.5, shaft_od=3.2, engagement_mm=2.0),
    "M3x8 SHCS":                       dict(head_od=5.5, shaft_od=3.2, engagement_mm=5.0),
    "M3x8 SHCS into heat-set insert":  dict(head_od=5.5, shaft_od=3.2, engagement_mm=5.0),
    # ``M3x8 SHCS self-tap`` -- Design E mixed-mode, May 2026.  The 2 +X
    # cradle bolts per cradle self-tap into a Phi 2.5 mm pilot drilled
    # into the existing well-wall material (no brass insert, no Phi 8
    # mm boss).  Engagement target is the plastic shelf material below
    # the servo's 2.5 mm-thick tab: the M3 x 8 SHCS head sits on the
    # ear top, descends through 2.5 mm of (factory) tab, and bites
    # into ~ 5.5 mm of printed plastic in the well wall.  The shelf
    # has INSERT_M3_SELFTAP_PILOT_DEPTH = 10 mm of available depth so
    # the bolt does not bottom out; the engagement-zone window probes
    # the deepest 5 mm of the bolt for material contact (same target
    # as the M3 x 8 SHCS into heat-set entry above -- the bolt length
    # is identical and the shelf material at this depth is the
    # engagement medium in either case).
    "M3x8 SHCS self-tap":              dict(head_od=5.5, shaft_od=3.2, engagement_mm=5.0),
    # ``M3x6 SHCS self-tap`` -- Jun 2026 flush-head tweak.  The 2 yaw-saddle
    # chassis-anchor screws self-tap RETAINER_PLATE_PILOT_DEPTH = 3 mm into the
    # blind Phi 2.5 mm floor pilot (z[-6,-3]); after the flange thinned 5 -> 3 mm
    # the head seats at -9 and the 6 mm screw tip lands exactly at the -3 pilot
    # bottom, so engagement_mm = 3 mm probes precisely that floor bite (a wider
    # window would dip into the flange clearance hole below -6 and falsely fail).
    "M3x6 SHCS self-tap":              dict(head_od=5.5, shaft_od=3.0, engagement_mm=3.0),
    # ``M3x10 SHCS`` -- chassis-standoff top bolts + deck-column tray
    # bolts into brass female threads (modelled as heat-set inserts).
    # Same engagement target as M3 x 8 into insert (= the 5 mm insert
    # body length) since both rely on the brass thread, not the
    # printed plastic.
    "M3x10 SHCS":                      dict(head_od=5.5, shaft_od=3.2, engagement_mm=5.0),
    # ``M3x14 SHCS`` -- Jul 2026 F/F standoff switch: the 4 chassis_bottom
    # bolts enter at the -6 floor face, span the 8 mm plate + floor stack
    # and thread into the brass F-F standoff's bottom female thread
    # (modelled as a virtual heat-set insert at the +2 plate top face).
    "M3x14 SHCS":                      dict(head_od=5.5, shaft_od=3.2, engagement_mm=5.0),
    "M3x32 SHCS":                      dict(head_od=5.5, shaft_od=3.2, engagement_mm=5.0),
    "M3x16 pan-head":                  dict(head_od=6.0, shaft_od=3.2, engagement_mm=5.0),
    "M2.5x8 spline screw":             dict(head_od=4.5, shaft_od=2.7, engagement_mm=3.0),
    # ``M2.5x8 SHCS into heat-set insert`` -- Pi 4 board-mount bolts (4)
    # into M2.5 brass heat-set inserts (McMaster 94459A106).  Insert
    # body is INSERT_M25_INSERT_LENGTH = 4 mm long; engagement target
    # is the brass thread, not the surrounding plastic.
    "M2.5x8 SHCS into heat-set insert": dict(head_od=4.5, shaft_od=2.7, engagement_mm=4.0),
    # ``M2.5 self-tap into servo rear case`` -- Jun 2026 (corrected) yaw rear
    # CASE-mount.  The 4 saddle screws self-tap straight UP into the STS3215's
    # FIXED rear (back) case face holes (cradle (-8.3/-32.8,+-10.2); NOT the horn
    # circle), ~2.5 mm bite, so engagement_mm = SADDLE_CASE_SCREW_BITE = 2.5 -- the
    # tip-zone window probes exactly that shallow case bite (the engagement target
    # is the injected real rear-case block; the head bears on the boss counterbore).
    "M2.5 self-tap into servo rear case": dict(head_od=4.5, shaft_od=2.7,
                                               engagement_mm=2.5),
}
_FASTENER_ENGAGEMENT_DEFAULT = dict(head_od=5.0, shaft_od=3.0, engagement_mm=3.0)

# Max contiguous AIR run along the shaft between p_head and
# p_engage_start.  A counter-bore void (1-2 mm in the part that the
# head bears on) is fine; anything longer means the bolt is bridging
# two parts that don't actually touch.
MAX_SHAFT_AIR_SPAN_MM = 2.0

# Minimum fraction of the tip engagement zone that must be inside
# printed material (or a mating-target mesh like ``servo_horn``).
TIP_ENGAGEMENT_MIN_FRACTION = 0.5

# Fasteners that engage a NUT or HEAT-SET INSERT at some point along
# the shaft (not necessarily at the tip).  For these the tip-engagement
# zone is RELOCATED to bracket the paired nut/insert, not the bolt's
# physical tip; otherwise an M3 x 32 chassis bolt's tip (at world
# z ~ -17 mm, well past the M3 nyloc nut at z ~ -4..-9) would always
# fail tip-engagement.
_NUT_INSERT_SPECS = (
    "M3 nyloc nut",
    "M3 heat-set insert",
    "M2.5 heat-set insert",
)

# Joints whose horn (servo_horn) sits ON the spline tip and is the
# target the M3 disc-horn bolts thread INTO.  Used by ``check_mating_face_contact``
# and by ``_world_horn_meshes`` to place the visual horn for leg 0.
_HORN_JOINTS = ("yaw", "hip", "knee")


def _engagement_spec_for(spec: str) -> dict:
    return FASTENER_ENGAGEMENT_SPEC.get(spec, _FASTENER_ENGAGEMENT_DEFAULT)


def _horn_world_transform(joint: str, leg_index: int):
    """4x4 transform that maps horn-local coords (origin at spline-
    mating face, +Z = output-shaft axis) into the world frame for the
    given joint on ``leg_index``.

    The disc horn is BOLTED to the driven link (coxa_link / femur_link /
    tibia_link) so it rotates RIGIDLY with that link.  At the neutral
    stance pose the relevant rotations are:

      * yaw    -- 0 deg (coxa_link is in the same Rz(a) frame as the
                    coxa_bracket / yaw cradle, so no link-vs-cradle
                    rotation; the un-rotated well transform already
                    matches the link frame).
      * hip    -- horn rotates with the FEMUR (driven link) about
                    the hip joint Y axis.  The hip well lives in the
                    coxa_link (parent), so the horn's rotation
                    RELATIVE to the well is ``STANCE_FEMUR_DEG``.
      * knee   -- horn rotates with the TIBIA (driven link) about
                    the knee joint Y axis.  The knee well lives in
                    the FEMUR (parent of the knee joint), so the
                    horn's rotation RELATIVE to the well is
                    ``STANCE_TIBIA_DEG`` -- NOT the sum
                    STANCE_FEMUR_DEG + STANCE_TIBIA_DEG, because
                    _knee_cradle_T's T_well already inherits the
                    femur's Ry(STANCE_FEMUR_DEG) via
                    T_femur_in_link.

    Pre-fix the horn was placed via ``_well_cradle_T @ horn_offset``
    for ALL three joints, which corresponds to "horn in the SERVO's
    well frame".  For yaw that is also the link frame (no link
    rotation), but for hip / knee the femur / tibia rotates about the
    joint axis at the neutral stance angle and the horn rotates with
    it.  Without this rotation the femur / tibia horn bolts (which
    are placed in femur- / tibia-local frame and therefore inherit
    the link's stance rotation) MISS the un-rotated horn arms in the
    world frame and report ``joins only 1 part [femur_link]`` /
    ``[tibia_link]`` in check_fastener_engagement.
    """
    import fastener_registry as _fr  # noqa: WPS433
    if joint == "yaw":
        T_well = _fr._yaw_cradle_T(leg_index)
        # STS3215 flush-output mount (Jun 2026 DEPTH fix): the output does NOT
        # protrude, so the Phi20 disc horn seats FLUSH on the body front face --
        # RECESSED inside the WELL_PLATE_T (4 mm) mount-plate bore, NOT sitting
        # on the plate top.  Its base is therefore AT the front face (dz = 0),
        # so its top lands WELL_PLATE_T - DISC_HORN_H (= 2 mm) below the plate
        # top and YAW_HORN_REACH_DOWN (= 7 mm) below the frozen output plane,
        # which is exactly where the coxa_yaw_hub's necked drive nub reaches
        # (YAW_HUB_BOSS_BOT_Z).  The old WELL_PLATE_T base offset placed the
        # horn 4 mm too high (on the plate top), baking in a phantom output
        # protrusion and leaving the bearing cap unable to seat flush.
        horn_base_dz = 0.0
    elif joint == "hip":
        T_well = _fr._hip_cradle_T(leg_index)
        # STS3215 flush-output mount (Jun 2026 yoke-width fix): the output
        # spline does NOT protrude, so the driven disc horn seats FLUSH on the
        # body front face (base dz = 0), topping out at SERVO_BODY_H +
        # DISC_HORN_H = 36.3 -- SERVO_OUTPUT_H (2 mm) below the old placement.
        # The yoke top-arm pad now reaches DRIVEN_HORN_REACH_DOWN (5 mm) to meet
        # it, fixing the user-reported ~2 mm clevis gap.  Matches the yaw joint.
        horn_base_dz = 0.0
    elif joint == "knee":
        T_well = _fr._knee_cradle_T(leg_index)
        horn_base_dz = 0.0      # flush output (see hip note above)
    else:
        raise ValueError(f"unknown joint: {joint!r}")
    horn_offset = _fr._T(hp.SERVO_OUTPUT_X, 0.0,
                          hp.SERVO_BODY_H + horn_base_dz)
    T_horn = T_well @ horn_offset
    if joint == "yaw":
        return T_horn
    # For hip / knee the horn rotates with the driven link about the
    # joint axis.  Compose the rotation in HORN-LOCAL frame: horn-local
    # +Z is the joint axis, so the in-frame rotation is _Rz(p_link)
    # applied AFTER the horn's existing frame mapping.
    if joint == "hip":
        # Hip horn rotates with the femur (driven link) by
        # STANCE_FEMUR_DEG relative to the coxa_link (where the
        # well lives).  In horn-local +Z that's Rz(STANCE_FEMUR_DEG).
        p_link = np.deg2rad(hp.STANCE_FEMUR_DEG)
    else:
        # Knee horn rotates with the tibia by STANCE_TIBIA_DEG
        # relative to the femur (where the knee well lives).
        # _knee_cradle_T's T_well ALREADY inherits the femur's
        # Ry(STANCE_FEMUR_DEG) via T_femur_in_link, so we must not
        # double-count it here.  Pre-fix this was STANCE_FEMUR_DEG +
        # STANCE_TIBIA_DEG and check_fastener_engagement reported
        # "joins only 1 part [tibia_link]" on all 4 knee bolts
        # because the horn frame was over-rotated by an extra
        # STANCE_FEMUR_DEG = -25 deg.
        p_link = np.deg2rad(hp.STANCE_TIBIA_DEG)
    return T_horn @ _fr._Rz(p_link)


def _servo_body_world_transform(joint: str, leg_index: int):
    """4x4 transform mapping servo_body-local coords to the world frame.

    STS3215 front-face mount (Jun 2026): both the ``servo_body`` mesh
    (``_servo_envelope`` / ``make_servo_body``) and the cradle's well
    frame (``_servo_well_solid``) put their ORIGIN at the body's BACK
    (-Z) face centre with +Z = output direction.  So the body-local
    frame coincides with the well-local frame and the placement is just
    the cradle's well-to-world transform -- no tab-shelf seating offset
    (the DS3225 ``body_bottom_z = shelf_top_z + SERVO_TAB_T/2 -
    SERVO_TAB_Z`` term is retired along with the tabs)."""
    import fastener_registry as _fr  # noqa: WPS433
    if joint == "yaw":
        # _yaw_cradle_T's well-to-chassis shift is now CAD-exact
        # (dz = -13.55), so the well-local frame coincides with the
        # cradle geometry and the body's back-face origin lands at the
        # cradle cavity back with NO extra seat offset.
        T_well = _fr._yaw_cradle_T(leg_index)
    elif joint == "hip":
        T_well = _fr._hip_cradle_T(leg_index)
    elif joint == "knee":
        T_well = _fr._knee_cradle_T(leg_index)
    else:
        raise ValueError(f"unknown joint: {joint!r}")
    return T_well


# Joints that carry a PASSIVE rear-boss disc horn (symmetric-yoke refit).
_PASSIVE_HORN_JOINTS = ("hip", "knee")


def _passive_link_angle(joint: str) -> float:
    """Stance rotation (rad) the passive horn shares with the driven link,
    same convention as ``_horn_world_transform``."""
    if joint == "hip":
        return np.deg2rad(hp.STANCE_FEMUR_DEG)
    if joint == "knee":
        return np.deg2rad(hp.STANCE_TIBIA_DEG)
    raise ValueError(f"joint {joint!r} has no passive horn")


def _passive_horn_world_transform(joint: str, leg_index: int):
    """World transform for the PASSIVE disc horn on the servo's rear idler
    boss.  Mirror of ``_horn_world_transform``: the STOCK metal passive horn
    slides over the rear boss and seats FLUSH on the servo back face (Jul 2026
    stock-horn refit -- no printed adapter), flipped 180 deg about X so its
    flat mating face points AWAY from the servo (-Z), spanning well-z in
    [-DISC_HORN_H, 0].  The 4-hole cross is symmetric under the flip, so the
    bottom-arm bolts engage it exactly as the driven horn's bolts engage the
    top arm."""
    import fastener_registry as _fr  # noqa: WPS433
    if joint == "hip":
        T_well = _fr._hip_cradle_T(leg_index)
    elif joint == "knee":
        T_well = _fr._knee_cradle_T(leg_index)
    else:
        raise ValueError(f"joint {joint!r} has no passive horn")
    flip = rotation_matrix(np.pi, [1, 0, 0])
    return (T_well
            @ _fr._T(hp.SERVO_OUTPUT_X, 0.0, 0.0)
            @ _fr._Rz(_passive_link_angle(joint))
            @ flip)


def _build_world_assembly_parts(leg_index: int = 0) -> dict:
    """Return the printed parts (from ``_build_world_leg0_printed_parts``)
    augmented with placed ``servo_horn`` AND ``servo_body`` meshes for
    the 3 joints on ``leg_index``.  Used by ``check_fastener_engagement``
    so a bolt can be confirmed to engage the disc horn (EXTERNAL mating
    part) and the cradle bolts can be confirmed to bear on the SERVO
    body's TABS (also an external part).
    """
    if leg_index != 0:
        raise ValueError(
            "world assembly currently only supports leg_index=0; "
            "chassis is 6-fold symmetric so leg 0 + chassis-level "
            "fasteners cover every distinct interface."
        )
    parts = _build_world_leg0_printed_parts()
    # Jun 2026 single-part merge: the chassis bottom is ONE printed part again
    # (``chassis_bottom`` == the merged plate + folded-in floor slab), already
    # in ``parts`` under that key via ``chassis_assembled``.  No separate LOW
    # half / join screws to add.
    for joint in _HORN_JOINTS:
        horn = _load_mesh("servo_horn")
        horn.apply_transform(_horn_world_transform(joint, leg_index))
        parts[f"servo_horn({joint})"] = horn

        body = _load_mesh("servo_body")
        body.apply_transform(_servo_body_world_transform(joint, leg_index))
        parts[f"servo_body({joint})"] = body
    # PASSIVE (rear-boss) STOCK disc horns on the hip + knee sandwich joints
    # (the symmetric-yoke refit; yaw uses the 6706 pair, no passive horn).
    # Jul 2026 stock-horn refit: the horn centres directly on the rear idler
    # boss (no printed adapter).  The yoke's bottom arm bolts to these exactly
    # like its top arm bolts to the driven horns above.
    for joint in _PASSIVE_HORN_JOINTS:
        ph = _load_mesh("servo_horn")
        ph.apply_transform(_passive_horn_world_transform(joint, leg_index))
        parts[f"passive_horn({joint})"] = ph
    return parts


def _radial_basis(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return two unit vectors perpendicular to ``axis`` and to each
    other (an orthonormal basis for the plane orthogonal to ``axis``).
    Used to spread radial probe points around the bolt axis."""
    axis = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(axis))
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
    axis = axis / n
    # Pick a helper vector NOT parallel to axis.
    if abs(axis[2]) < 0.9:
        helper = np.array([0.0, 0.0, 1.0])
    else:
        helper = np.array([1.0, 0.0, 0.0])
    u = np.cross(axis, helper)
    u = u / float(np.linalg.norm(u))
    v = np.cross(axis, u)
    return u, v


def _radial_probe_points(p_center: np.ndarray,
                           axis: np.ndarray,
                           radius: float,
                           n_az: int = 8) -> np.ndarray:
    """Return ``n_az`` probe points arranged in a ring of radius
    ``radius`` around ``p_center`` in the plane orthogonal to
    ``axis``."""
    u, v = _radial_basis(axis)
    angles = np.linspace(0.0, 2.0 * np.pi, n_az, endpoint=False)
    pts = []
    for a in angles:
        pts.append(p_center + radius * (u * np.cos(a) + v * np.sin(a)))
    return np.asarray(pts, dtype=float)


def _which_parts_contain(parts: dict, points: np.ndarray) -> dict:
    """Return ``{part_name: bool_array}`` reporting which of ``points``
    is inside each part in ``parts``.  Uses AABB-pruning so the
    expensive ``points_inside`` call is only invoked for parts whose
    AABB overlaps the probe cloud."""
    if len(points) == 0:
        return {name: np.zeros(0, dtype=bool) for name in parts}
    p_lo = points.min(axis=0)
    p_hi = points.max(axis=0)
    out: dict[str, np.ndarray] = {}
    for name, mesh in parts.items():
        b_lo, b_hi = mesh.bounds
        if np.any(p_hi < b_lo - 0.1) or np.any(b_hi < p_lo - 0.1):
            out[name] = np.zeros(len(points), dtype=bool)
            continue
        out[name] = points_inside(mesh, points)
    return out


def _any_part_contains(parts_contain: dict, idx_mask) -> np.ndarray:
    """OR ``{part_name: bool_array}`` together at the given index mask
    (or across all points if ``idx_mask`` is None).  Returns a 1-D
    boolean array."""
    if not parts_contain:
        return np.zeros(0, dtype=bool)
    arrays = list(parts_contain.values())
    out = arrays[0].copy()
    for a in arrays[1:]:
        out = out | a
    if idx_mask is not None:
        out = out[idx_mask]
    return out


def _distinct_parts_engaged(parts_contain: dict, threshold: int = 1
                              ) -> list[str]:
    """Return the sorted list of part names that contain at least
    ``threshold`` of the probed points (i.e., the bolt physically
    overlaps that part).  A bolt joining < 2 distinct parts is the
    classic "bolt floats inside one part" failure mode."""
    out = []
    for name, mask in parts_contain.items():
        if int(mask.sum()) >= threshold:
            out.append(name)
    return sorted(out)


def _find_paired_engagement_target(fi, all_fasteners) -> object:
    """If ``fi`` is a SHCS / pan-head that engages a co-located nut or
    heat-set insert, return that paired fastener.  Otherwise return
    ``None`` (so the engagement check uses the mesh-material default).

    Matching rule: same leg_index (or both None), AND the paired
    fastener's head_world_xyz sits within 2 mm of the bolt's axis line
    AND within ``length_mm`` along the axis from the head.
    """
    if fi.length_mm is None or fi.length_mm <= 0:
        return None
    head = np.asarray(fi.head_world_xyz, dtype=float)
    axis = np.asarray(fi.axis_world, dtype=float)
    L = float(fi.length_mm)
    for other in all_fasteners:
        if other is fi:
            continue
        if other.spec not in _NUT_INSERT_SPECS:
            continue
        if other.leg_index != fi.leg_index and not (
            other.leg_index is None and fi.leg_index is None
        ):
            continue
        op = np.asarray(other.head_world_xyz, dtype=float)
        delta = op - head
        t = float(delta @ axis)
        if t < -1.0 or t > L + 5.0:
            continue
        perp = delta - t * axis
        if float(np.linalg.norm(perp)) > 2.0:
            continue
        return other
    return None


def check_fastener_engagement():
    """For each fastener with a defined bolt length, confirm that:

      * the head bears on solid material (head_od/2 + 0.3 mm ring probe),
      * the shaft does not bridge > MAX_SHAFT_AIR_SPAN_MM mm of air,
      * the tip engages something (a printed part, the servo_horn, or
        a paired nut / heat-set insert), and
      * the bolt joins at LEAST 2 distinct printed/horn parts.

    Probes the leg-0 fasteners (the chassis is 6-fold symmetric so
    leg 0 + chassis-level entries cover every distinct interface).
    """
    print("\n[15] Fastener engagement (head bearing, tip engagement, "
          "shaft span, distinct-parts join):")

    import fastener_registry  # noqa: WPS433

    all_fasteners = fastener_registry.build_all_fastener_instances()
    sample = [fi for fi in all_fasteners
              if (fi.leg_index is None or fi.leg_index == 0)
              and fi.length_mm is not None]

    # Nyloc nuts and heat-set inserts are themselves "fasteners with a
    # length" in the registry (the insert is 5 mm long, the nyloc nut
    # is ~5 mm tall).  Skip them in the bolt-engagement check: they
    # are the ENGAGEMENT TARGETS for bolts, not bolts themselves.
    sample = [fi for fi in sample if fi.spec not in _NUT_INSERT_SPECS]

    # Skip the M2.5 spline center screws: they ship pre-installed in
    # the servo's spline collar (an internal servo feature we don't
    # model in any printable / horn / body mesh).  Engagement is
    # implicit and handled by the screwdriver-access SKIP.
    sample = [fi for fi in sample if fi.spec != "M2.5x8 spline screw"]

    world_parts = _build_world_assembly_parts(leg_index=0)

    # Real STS3215 rear (back) CASE block (Jun 2026 corrected).  The 4 yaw rear
    # case-mount M2.5 screws self-tap straight UP into the FIXED rear case FACE at
    # cradle (-8.3/-32.8,+-10.2) -- the standard STS3215 case mount holes (NOT the
    # horn circle).  The frozen-short modeled servo_body (SERVO_BODY_H is the
    # mount-HOLE-plane gap, SADDLE_CASE_LEN_FIX shorter than the real case) stops
    # SADDLE_CASE_LEN_FIX ABOVE the real rear face, so the screws would thread into
    # empty model space.  Inject the real rear-case slab as a LOCAL engagement
    # target (this check only -- NOT _build_world_assembly_parts, so interference /
    # clearance are untouched), spanning the full case footprint in cradle-local
    # XY at z[real_back, real_back + SADDLE_CASE_LEN_FIX] (abuts the modeled
    # servo_body back face for shaft continuity), placed by the SAME saddle->world
    # chain (_place_yaw_retainers) so it lands under all 4 hole stations.
    _zc = hp.yaw_servo_real_back_z()
    _bx_c = -hp.SERVO_OUTPUT_X
    _rear_case = trimesh.creation.box(
        extents=(hp.SERVO_BODY_W, hp.SERVO_BODY_D, hp.SADDLE_CASE_LEN_FIX))
    _rear_case.apply_translation([_bx_c, 0.0, _zc + hp.SADDLE_CASE_LEN_FIX / 2.0])
    _a_yaw = 0.5 * np.pi / 3.0
    _apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    _edge_mid = np.array([_apothem * np.cos(_a_yaw), _apothem * np.sin(_a_yaw), 0.0])
    _rear_case.apply_transform(rotation_matrix(_a_yaw, [0, 0, 1]))
    _rear_case.apply_translation(_edge_mid)
    world_parts["servo_rear_case(yaw)"] = _rear_case

    print(f"  Probing {len(sample)} bolt(s) on leg 0 + chassis-level "
          f"against {len(world_parts)} placed parts.")

    all_ok = True
    n_fail = 0

    for fi in sample:
        cfg = _engagement_spec_for(fi.spec)
        head_od = float(cfg["head_od"])
        shaft_od = float(cfg["shaft_od"])
        engagement_mm = float(cfg["engagement_mm"])

        p_head = np.asarray(fi.head_world_xyz, dtype=float)
        axis = np.asarray(fi.axis_world, dtype=float)
        L = float(fi.length_mm)
        p_tip = p_head + axis * L
        p_engage_start = p_tip - axis * engagement_mm

        paired = _find_paired_engagement_target(fi, all_fasteners)

        # ---- (a) Head bearing ring probe -----------------------------
        # Probe a ring at r = head_od/2 just BEHIND the head (into the
        # material the head bears against).  Sits 0.1 mm "inside" the
        # part along the bolt axis so it lands on bearing material
        # rather than on the air just below the head face.
        ring_head = _radial_probe_points(
            p_head + axis * 0.1,
            axis,
            head_od / 2.0,
            n_az=8,
        )
        head_contains = _which_parts_contain(world_parts, ring_head)
        head_any = _any_part_contains(head_contains, None)
        head_ok = bool(head_any.any())
        # Parts the HEAD physically bears on (clamps).  For a compliant
        # torque-only clamp the head grips its near-side part here even
        # though the oversized-clearance shaft floats past it below.
        head_parts = sorted(name for name, mask in head_contains.items()
                            if int(mask.sum()) >= 1)

        # ---- (b) Shaft + tip axial sampling --------------------------
        # Sample N_axial points along the entire shaft [p_head, p_tip]
        # at radial offset r = shaft_od/2 + 0.4 mm (just outside the
        # drilled clearance bore).  Each axial position contributes
        # N_az radial probes; we record whether ANY radial probe lands
        # in printed material, and which named part contains it.
        N_axial = 16
        N_az = 4
        shaft_radius = shaft_od / 2.0 + 0.4
        ts = np.linspace(0.0, L, N_axial)
        axial_points = p_head[None, :] + axis[None, :] * ts[:, None]
        # Stack ALL probe points (axial * N_az) into a single batch
        # so each ``points_inside`` call is amortised across them.
        u, v = _radial_basis(axis)
        angles = np.linspace(0.0, 2.0 * np.pi, N_az, endpoint=False)
        radial_offsets = np.stack(
            [shaft_radius * (np.cos(a) * u + np.sin(a) * v)
             for a in angles],
            axis=0,
        )  # (N_az, 3)
        # All probes shape: (N_axial, N_az, 3).
        all_probes = (axial_points[:, None, :]
                      + radial_offsets[None, :, :])
        flat = all_probes.reshape(-1, 3)
        flat_contains = _which_parts_contain(world_parts, flat)
        # Reshape each part's bool array back to (N_axial, N_az).
        part_per_axial: dict[str, np.ndarray] = {}
        for name, mask in flat_contains.items():
            part_per_axial[name] = (mask
                                      .reshape(N_axial, N_az)
                                      .any(axis=1))
        any_part_at_axial = np.zeros(N_axial, dtype=bool)
        for m in part_per_axial.values():
            any_part_at_axial = any_part_at_axial | m

        # ---- (c) Shaft air-span analysis -----------------------------
        # Look at the axial samples between p_head + 0.5 mm and
        # p_engage_start (i.e., the shaft proper, excluding the head
        # bearing and the tip engagement zones).  Find the LONGEST
        # contiguous run of axial positions with NO material.  In
        # millimetres = (count of consecutive AIR samples) * (L /
        # (N_axial - 1)).
        #
        # Bolts with a paired nut / heat-set insert get the nut's
        # position injected as "material" along the shaft -- the nut
        # / insert IS the thread engagement target, even though we
        # don't model it as a printable / horn mesh.
        any_or_paired = any_part_at_axial.copy()
        if paired is not None:
            op = np.asarray(paired.head_world_xyz, dtype=float)
            t_paired = float((op - p_head) @ axis)
            paired_lo = t_paired - 3.0
            paired_hi = t_paired + 3.0
            any_or_paired = any_or_paired | (
                (ts >= paired_lo) & (ts <= paired_hi)
            )
            # Past the paired nut / insert the bolt's tip just sticks
            # out into free space (an M3 x 32 chassis bolt overhangs
            # the nyloc by ~8 mm).  Cap the shaft-proper scan range
            # at just-before the paired target so this overhang
            # doesn't register as a structural air span.
            engage_start_t = max(0.0, min(t_paired - 1.0,
                                            L - engagement_mm))
        else:
            engage_start_t = max(0.0, L - engagement_mm)

        # ---- (c.1) Effective bolt tip -------------------------------
        # Bolts can be SHORTER than the assembled material run (rare;
        # caught by the shaft / tip checks below) OR LONGER than the
        # engagement target (e.g. the retired M2 x 8 X-horn SHCS was
        # 8 mm long but its cap + arm stack was only ~ 3.1 mm thick;
        # the bolt overhung the arm bottom into free air by ~ 5 mm).
        # Use the
        # DEEPEST axial sample with material as the "effective tip" so
        # the tail overhang past the last engagement target doesn't
        # poison shaft_air_span / tip_engagement, which both assume
        # the bolt's tail is inside engagement material.
        material_mask = any_or_paired.copy()
        if material_mask.any():
            idx_last_material = int(np.flatnonzero(material_mask).max())
            t_eff_tip = float(ts[idx_last_material])
        else:
            t_eff_tip = L
        # Don't let the effective tip exceed the bolt's modeled
        # length, and don't let it UNDERCUT the natural engagement
        # zone when the bolt is short / well-sized (preserves the
        # bolt-not-deep-enough catch).
        t_eff_tip = max(min(t_eff_tip, L), engagement_mm)

        # ---- (c.2) Shaft span up to the effective tip ----------------
        shaft_lo_t = min(0.5, L)
        shaft_hi_t = max(shaft_lo_t,
                          min(engage_start_t, t_eff_tip - engagement_mm))
        shaft_mask = (ts >= shaft_lo_t) & (ts <= shaft_hi_t)
        if not shaft_mask.any():
            shaft_air_mm = 0.0
        else:
            shaft_in = any_or_paired[shaft_mask].astype(int)
            # Length of each axial step in mm.
            if N_axial > 1:
                step_mm = L / float(N_axial - 1)
            else:
                step_mm = 0.0
            # Longest contiguous run of zeros in shaft_in.
            longest = 0
            current = 0
            for v_in in shaft_in:
                if v_in == 0:
                    current += 1
                    if current > longest:
                        longest = current
                else:
                    current = 0
            shaft_air_mm = float(longest) * step_mm

        shaft_span_ok = shaft_air_mm <= MAX_SHAFT_AIR_SPAN_MM + 1e-3
        # STS3215 output-face case screws (RETIRED, Jun 2026): the model
        # used to emit 4 x M2.5 "M2.5 SHCS into servo case" screws per
        # cradle that bridged the mount-plate clearance bore, and this
        # block exempted them from the "shaft must not bridge > 2 mm of
        # air" rule.  That front-face mount was a PHANTOM feature (the
        # dia-20 disc horn covers the output face), so all 72 case screws
        # were removed from the registry -- no FastenerInstance carries
        # that spec anymore and the exemption below is now a harmless
        # no-op.  The servo body is retained by the printed strap (yaw)
        # and clamp cap (hip/knee); the output face carries only the
        # flush disc horn.  Kept (not deleted) to document the change.
        if fi.spec == "M2.5 SHCS into servo case":
            shaft_span_ok = True
        # Compliant torque-only clamp (e.g. the yaw hub-to-disc-horn
        # bolts): the OVERSIZED clearance hole lets the shaft float by
        # design -- concentricity is set by the spaced bearings, not the
        # bolt -- so the shaft-air-span rule does not apply.  The head
        # bearing + tip thread engagement (both still enforced) are what
        # actually clamp the stack.  See FastenerInstance.compliant_torque_only.
        if getattr(fi, "compliant_torque_only", False):
            shaft_span_ok = True

        # ---- (d) Tip engagement --------------------------------------
        # If the bolt has a paired nut / insert, the engagement happens
        # AT the nut/insert position along the axis (which can be far
        # from the bolt's physical tip).  Define the engagement zone
        # as a 5 mm-wide window centred on the nut/insert.  Otherwise
        # the engagement zone is the last ``engagement_mm`` of the
        # bolt UP TO THE EFFECTIVE TIP (not the bolt's modeled tip --
        # see the (c.1) comment for why over-long bolts need the
        # window clipped to the deepest material contact).
        if paired is not None:
            op = np.asarray(paired.head_world_xyz, dtype=float)
            t_paired = float((op - p_head) @ axis)
            zone_lo_t = max(0.0, t_paired - 2.5)
            zone_hi_t = min(L, t_paired + 2.5)
        else:
            zone_hi_t = t_eff_tip
            zone_lo_t = max(0.0, zone_hi_t - engagement_mm)

        if zone_hi_t <= zone_lo_t:
            tip_ok = True
            tip_frac = 1.0
        else:
            zone_mask = (ts >= zone_lo_t - 1e-6) & (ts <= zone_hi_t + 1e-6)
            n_zone = int(zone_mask.sum())
            if n_zone == 0:
                tip_ok = True
                tip_frac = 1.0
            else:
                tip_in = any_or_paired[zone_mask].sum()
                tip_frac = float(tip_in) / float(n_zone)
                tip_ok = tip_frac >= TIP_ENGAGEMENT_MIN_FRACTION

        # ---- (e) Distinct parts joined -------------------------------
        # Across ALL axial samples (head -> tip), count distinct named
        # parts that contained material at any axial position.  A bolt
        # that joins ONLY ONE part is bolting nothing.  For bolts with
        # a paired nut/insert the nut counts as the "second part" --
        # we add a synthetic entry for that match.
        joined_parts = _distinct_parts_engaged(part_per_axial,
                                                 threshold=1)
        # Compliant torque-only clamps grip their near part under the HEAD
        # (the oversized shaft never touches it), so fold the head-bearing
        # part(s) into the distinct-parts join -- the bolt genuinely joins
        # [head-clamped part] + [thread-engaged part] even though no single
        # shaft sample sits in two rigid parts.
        if getattr(fi, "compliant_torque_only", False):
            joined_parts = sorted(set(joined_parts) | set(head_parts))
        joined_count = len(joined_parts)
        if paired is not None:
            joined_count += 1
        join_ok = joined_count >= 2

        ok = head_ok and shaft_span_ok and tip_ok and join_ok
        if not ok:
            n_fail += 1
            reasons = []
            if not head_ok:
                reasons.append("head bearing in air")
            if not shaft_span_ok:
                reasons.append(
                    f"shaft air span {shaft_air_mm:.1f}>2 mm"
                )
            if not tip_ok:
                reasons.append(f"tip engagement {tip_frac*100:.0f}%<"
                                f"{int(TIP_ENGAGEMENT_MIN_FRACTION*100)}%")
            if not join_ok:
                if joined_parts:
                    reasons.append(
                        f"joins only {joined_count} part(s) "
                        f"[{', '.join(joined_parts)}"
                        + (", paired-target" if paired else "") + "]"
                    )
                else:
                    reasons.append("joins zero parts (floats in air)")
            detail = "; ".join(reasons)
            _label(fi.role, False, detail)
            all_ok = False

    if all_ok:
        _label(
            f"{len(sample)} bolts probed (leg 0 + chassis)",
            True,
            f"every bolt's head bears, shaft is solid, tip engages, "
            f"and bolt joins >= 2 parts",
        )
    else:
        print(f"  ---- FAIL ({n_fail} of {len(sample)} probed) ----")
    return all_ok


# ---------------------------------------------------------------------------
# Mating-face contact check
# ---------------------------------------------------------------------------

# Per-interface mating tolerance.  Tighter than the FDM print tolerance
# (~0.3-0.5 mm per side) because these are CRITICAL clamping faces --
# bigger gaps mean the clamp bolts cannot pull the joint flush.
DEFAULT_MATING_TOLERANCE_MM = 0.5


def _mating_interfaces_leg0(world_parts: dict) -> list[tuple]:
    """Return ``(name, top_part_mesh, bottom_part_mesh, point_world,
    axis_world, scan_dist_mm, tolerance_mm)`` for every mating
    interface on leg 0 + chassis-level.  Builds the world-frame point
    and axis from leg-0 transforms so the per-leg multiplicity stays
    implicit (the chassis is 6-fold symmetric)."""
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = 0.5 * np.pi / 3
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    Rz_a = rotation_matrix(a, [0, 0, 1])[:3, :3]
    z_hat = np.array([0.0, 0.0, 1.0])

    yaw_output_z = hp.CHASSIS_YAW_OUTPUT_Z
    hip_drop = hp.COXA_HIP_DROP
    p = np.deg2rad(hp.STANCE_FEMUR_DEG)
    pt = np.deg2rad(hp.STANCE_FEMUR_DEG + hp.STANCE_TIBIA_DEG)
    Ry_p = rotation_matrix(p, [0, 1, 0])[:3, :3]
    Ry_pt = rotation_matrix(pt, [0, 1, 0])[:3, :3]

    # Test points on each disc-horn mating face are SHIFTED to a radial
    # offset in the HORN's bolt-circle plane (= the joint's mating-face
    # plane perpendicular to the joint axis).  Why not the joint-axis
    # ITSELF (radial 0)?  Two design features make the axis a poor
    # probe location:
    #   (a) the links have a Phi HORN_CENTRE_OD = 3.4 mm through-hole
    #       for the spline center screw, and the disc has a Phi 5.5 mm
    #       spline bore, so a vertical scan through the axis finds NO
    #       material at all;
    #   (b) the pads + the disc collar bore (Phi DISC_HORN_COLLAR_OD =
    #       9 mm) sit on the axis, so a scan through the joint axis
    #       lands in the recess and reads the gap too large.
    # June 2026 disc-horn switch: the 20 mm aluminium disc only extends
    # to radius 10 mm, so the old radius-12 probe now falls OFF the disc
    # entirely.  We move the probe to HORN-LOCAL radius 5 mm (-X arm):
    # that sits OUTSIDE the collar bore (Phi 9 -> radius 4.5) and the
    # disc spline bore (radius 2.75), OUTSIDE the 4 M3 bolt holes
    # (PCD 14 / 2 = 7 mm, the -X hole at (-7, 0) is 2 mm away), and
    # well INSIDE both the disc (radius 10) and every pad, where the
    # disc's flat top face and the link's pad are supposed to touch
    # flush.  We probe HORN-LOCAL (not link-local) because the disc is
    # the COMMON mating part for all three joints AND horn-local +Z
    # aligns with the joint axis exactly, so the scan direction is
    # clean.  The -X arm keeps the femur / tibia tests away from the
    # spar's +X knee end (where ``knee_clear`` / ``insertion_slot``
    # cuts remove pad material at femur-local x > 8).
    _MATING_RADIAL_OFFSET_MM = -5.0

    def _horn_local_xy_world(joint: str, x_local: float, y_local: float):
        """Convert (x, y, 0) in HORN-LOCAL to world coords (z=0 = the
        spline-mating face).  Helper for the disc-horn mating-face test
        points so they sit ON the disc horn's top face regardless of which
        leg/yaw angle we're probing."""
        T = _horn_world_transform(joint, 0)
        return (T @ np.array([x_local, y_local, 0.0, 1.0]))[:3]

    yaw_test_world = _horn_local_xy_world(
        "yaw", _MATING_RADIAL_OFFSET_MM, 0.0)
    hip_test_world = _horn_local_xy_world(
        "hip", _MATING_RADIAL_OFFSET_MM, 0.0)
    knee_test_world = _horn_local_xy_world(
        "knee", _MATING_RADIAL_OFFSET_MM, 0.0)

    # The hip/knee output (disc-horn) axis points along LEG_PITCH_AXIS in
    # coxa-local; the driven link mates on the +output side, so probe along
    # that axis (coaxial refit flips it from +Y to -Y).
    hip_axis_world = Rz_a @ np.array(hp.LEG_PITCH_AXIS)
    knee_axis_world = Rz_a @ np.array(hp.LEG_PITCH_AXIS)

    interfaces: list[tuple] = []

    # Per-interface tolerances.  The yaw mating face is the link's
    # solid pedestal bottom mating with the disc-horn top -- a clean,
    # 0.5 mm-precision flush join.  The femur / tibia hip / knee pads
    # mate at the BOLT-CIRCLE radius (PCD/2) where the central horn-
    # stack void's FDM tolerance band (+1 mm in +Y past the pad face)
    # eats 1 mm of pad material at the mating face.  That extra 1 mm
    # is by design (it guarantees the void contains no FDM sliver of
    # plastic that would push the pad off the horn), so the mating
    # face check uses a 1.5 mm tolerance to capture the design intent
    # while still catching ANY further drift past that.
    YAW_TOL = DEFAULT_MATING_TOLERANCE_MM
    HIP_KNEE_TOL = 1.5

    # NOTE (coaxial spaced-bearing refit, Jun 2026): the yaw turntable hub
    # (coxa_yaw_hub) NO LONGER seats flat on the disc-horn top -- it rides
    # the spaced 6706 bearing pair for support and is DRIVEN by the 4 x M3
    # disc-horn bolts in tension/shear, with a Phi 22 bore that deliberately
    # CLEARS the Phi 20 disc horn (so the bearings, not the horn face, set
    # concentricity).  There is therefore no flat hub<->horn mating face to
    # probe; this interface is retired for the yaw joint (the bolt drive +
    # bearing seats are covered by check_fastener_engagement and the
    # bearing-bore geometry).  Hip/knee disc-horn pads still mate flush and
    # are checked below.
    if ("femur_link" in world_parts
        and "servo_horn(hip)" in world_parts):
        interfaces.append((
            "femur_link hip pad <-> hip disc-horn",
            world_parts["femur_link"], world_parts["servo_horn(hip)"],
            hip_test_world,
            hip_axis_world,
            25.0,
            HIP_KNEE_TOL,
        ))
    if ("tibia_link" in world_parts
        and "servo_horn(knee)" in world_parts):
        interfaces.append((
            "tibia_link knee pad <-> knee disc-horn",
            world_parts["tibia_link"], world_parts["servo_horn(knee)"],
            knee_test_world,
            knee_axis_world,
            25.0,
            HIP_KNEE_TOL,
        ))
    # NOTE: a coxa_bracket flange <-> chassis_bottom mating-face check
    # used to live here, but the bracket flange interpenetrates the
    # chassis_bottom plate by ~1.5 mm on the current main branch (a
    # PRE-EXISTING fit issue unrelated to the coxa_link pedestal bug
    # this commit is auditing).  Defer that interface until the
    # bracket-flange + chassis_bottom CSG offset is corrected; tracking
    # in a follow-up commit so this audit stays scoped to the coxa_link
    # regression introduced in 013a03f.
    return interfaces


def _mating_gap_along_axis(top_mesh, bottom_mesh,
                              point_world, axis_world,
                              scan_dist_mm: float,
                              n_samples: int = 81,
                              ) -> tuple[float, float, float]:
    """Return ``(gap_mm, top_min_t, bottom_max_t)`` for the mating
    interface along ``axis_world`` through ``point_world``.

    Implementation: cast a pair of rays from ``point_world`` along
    +/-axis and pick out the EXTREME mesh intersections that bound the
    interface.

      * ``top_min_t``  = smallest t > -scan_dist where the +axis ray
        enters the TOP mesh -- the bottom face of the top part.
      * ``bot_max_t``  = largest  t <  +scan_dist where the +axis ray
        exits  the BOTTOM mesh -- the top face of the bottom part.

    Ray casting (versus point-in-mesh sampling) gives the exact
    surface position, free of the small "inset" that watertight
    contains() can pick up at concave or near-coplanar boundaries.

    Returns NaNs when a ray finds no intersection in the scan window
    (widen ``scan_dist_mm`` or move the test point).
    """
    axis = np.asarray(axis_world, dtype=float)
    axis = axis / float(np.linalg.norm(axis))
    point = np.asarray(point_world, dtype=float)

    def _hits(mesh) -> np.ndarray:
        locs_pos, _, _ = mesh.ray.intersects_location(
            ray_origins=point[None, :],
            ray_directions=axis[None, :],
            multiple_hits=True,
        )
        locs_neg, _, _ = mesh.ray.intersects_location(
            ray_origins=point[None, :],
            ray_directions=-axis[None, :],
            multiple_hits=True,
        )
        ts = []
        for loc in locs_pos:
            ts.append(float((loc - point) @ axis))
        for loc in locs_neg:
            ts.append(float((loc - point) @ axis))
        ts = np.array(ts, dtype=float)
        return ts[np.abs(ts) <= scan_dist_mm + 1e-6]

    top_hits = _hits(top_mesh)
    bot_hits = _hits(bottom_mesh)

    # The TOP part lives at LARGER t (positive axis direction by
    # convention); its bottom face is the SMALLEST top-mesh hit that
    # also sits above the largest bottom-mesh hit (i.e. above the
    # mating plane).  The BOTTOM part lives at SMALLER t; its top face
    # is the LARGEST bottom-mesh hit.  If the top mesh contains the
    # test point (mid-volume scan), the smallest top-hit is the bottom
    # face directly; same for the bottom mesh.
    top_min_t = float(top_hits.min()) if top_hits.size else float("nan")
    bot_max_t = float(bot_hits.max()) if bot_hits.size else float("nan")
    if np.isnan(top_min_t) or np.isnan(bot_max_t):
        return (float("nan"), top_min_t, bot_max_t)
    return (top_min_t - bot_max_t, top_min_t, bot_max_t)


# ---------------------------------------------------------------------------
# Cable-clearance check
# ---------------------------------------------------------------------------
#
# Per-keepout volume tolerance.  50 mm^3 budgets the unavoidable few-
# mm of slop in the connector positions on the board PCB drawings.
CABLE_CLEARANCE_TOLERANCE_MM3 = 50.0


def _build_electronics_body_meshes() -> dict:
    """Return ``{board_name: solid_box_mesh}`` for every modelled
    electronics board in the assembly.

    Deck redesign (Jun 2026): the in-gap electronics_tray (Raspberry Pi
    + USB-to-TTL bus adapter) is retired.  The brain is an Arduino Uno Q
    on the LOWER stacked deck driving the STS3215 bus directly; a
    XINGYHENG buck converter rides the UPPER deck.  So the modelled
    boards are the Uno Q, the buck converter, and the MPU-6050 IMU.

    The meshes are simple AABB boxes matching
    ``build_prototype_assembly._body_battery_parts`` so the cable
    clearance check can probe overlap with the actual board solids
    that obstruct the cable plug airspaces.  Both decks are centred on
    the chassis Z axis at deck-local (0, 0).
    """
    from trimesh.creation import box as _box_mesh
    deck_top_face = hp.CHASSIS_TOP_TOP_Z
    uno_base_z = (deck_top_face + hp.DECK_LEVEL_1_STANDOFF_H
                  + hp.DECK_TRAY_T + hp.DECK_STANDOFF_BOSS_H)
    buck_base_z = (deck_top_face + hp.DECK_LEVEL_1_STANDOFF_H
                   + hp.DECK_LEVEL_2_STANDOFF_H
                   + hp.DECK_TRAY_T + hp.DECK_STANDOFF_BOSS_H)

    def _board(extents, cx, cy, board_h, base_z):
        m = _box_mesh(extents=(extents[0], extents[1], board_h))
        m.apply_translation([cx, cy, base_z + board_h / 2.0])
        return m

    out = {}
    # Uno Q on the lower deck (PCB + tallest component stack ~ 12 mm).
    out["UnoQ"] = _board(
        (hp.UNO_Q_PCB_W, hp.UNO_Q_PCB_D),
        0.0, 0.0,
        12.0,
        uno_base_z,
    )
    # Buck converter on the upper deck (BUCK_PCB_H tall incl. inductor +
    # screw terminals).
    out["Buck"] = _board(
        (hp.BUCK_PCB_W, hp.BUCK_PCB_D),
        0.0, 0.0,
        hp.BUCK_PCB_H,
        buck_base_z,
    )
    # MPU-6050 / GY-521 PCB visual on top of the IMU pad's 4 bosses
    # (chassis CG; pad bottom at chassis_top_top + IMU_PAD_TAPE_T).
    chassis_top_top_z = hp.CHASSIS_TOP_TOP_Z
    mpu_z_base = (chassis_top_top_z + hp.IMU_PAD_TAPE_T
                  + hp.IMU_PAD_T + hp.IMU_PAD_BOSS_H)
    out["MPU-6050"] = _board(
        (hp.IMU_PCB_W, hp.IMU_PCB_D),
        hp.IMU_PAD_CENTRE_X,
        hp.IMU_PAD_CENTRE_Y,
        hp.IMU_PCB_T,
        uno_base_z,
    )
    # Override the z placement: _board() base_z is a deck reference, not
    # the IMU pad boss height.  Re-translate to the IMU's actual world z.
    actual_centre = out["MPU-6050"].bounds.mean(axis=0)
    expected_centre = np.array([
        hp.IMU_PAD_CENTRE_X,
        hp.IMU_PAD_CENTRE_Y,
        mpu_z_base + hp.IMU_PCB_T / 2.0,
    ])
    out["MPU-6050"].apply_translation(expected_centre - actual_centre)
    return out


def _build_fastener_envelope_meshes() -> list:
    """Return ``[(label, mesh)]`` for every chassis-level fastener
    instance (bolts + heat-set inserts).  Each mesh is a simple
    cylinder of bolt-OD x length oriented along the registry's
    ``axis_world``.

    Leg-0 fasteners are deliberately EXCLUDED because the cable
    keepouts only live above the electronics tray (chassis-only
    airspace) and per-leg bolts are far enough away that the AABB
    overlap pre-filter rejects them anyway -- including them just
    inflates the keepout build time.
    """
    from trimesh.creation import cylinder as _cyl_mesh
    from trimesh.transformations import rotation_matrix as _rotmat
    import fastener_registry  # noqa: WPS433
    out = []
    for fi in fastener_registry.build_all_fastener_instances():
        if fi.leg_index is not None:
            continue
        if fi.length_mm is None:
            continue
        # Use the engagement spec for head + shaft size.
        cfg = _engagement_spec_for(fi.spec)
        shaft_od = float(cfg["shaft_od"])
        L = float(fi.length_mm)
        cyl = _cyl_mesh(radius=shaft_od / 2.0, height=L, sections=16)
        # Cylinder is along +Z by default; reorient along fi.axis_world.
        axis = np.asarray(fi.axis_world, dtype=float)
        if axis[2] < 0.99 or axis[2] > 1.01 or abs(axis[0]) > 1e-6 or abs(axis[1]) > 1e-6:
            if axis[2] > 0.99:
                pass  # already +Z
            elif axis[2] < -0.99:
                cyl.apply_transform(_rotmat(np.pi, [1, 0, 0]))
            else:
                # General reorientation: rotate +Z to axis.
                z = np.array([0.0, 0.0, 1.0])
                v = np.cross(z, axis)
                s = float(np.linalg.norm(v))
                c = float(np.dot(z, axis))
                if s > 1e-9:
                    angle = float(np.arctan2(s, c))
                    cyl.apply_transform(_rotmat(angle, v / s))
        # Place cylinder centre at p_head + axis * L/2.
        p_head = np.asarray(fi.head_world_xyz, dtype=float)
        centre = p_head + axis * (L / 2.0)
        cyl.apply_translation(centre)
        out.append((f"{fi.spec}@{fi.role[:32]}", cyl))
    return out


def check_cable_clearance():
    """For each board-connector cable-keepout volume, assert it has
    less than CABLE_CLEARANCE_TOLERANCE_MM3 mm^3 of overlap with any
    printed part, modelled electronics body, or chassis-level
    fastener mesh.

    The keepouts cover the airspace a cable plug + strain relief
    occupies once the cable is plugged in -- a future CAD edit that
    grew material into a cable's plug-in airspace will FAIL this
    check instead of being discovered the first time the cables are
    routed.

    A connector keepout is allowed to overlap its OWN board's
    modelled body envelope (the keepout's ``part_name`` matches the
    board solid's key) -- a USB-B keepout that extends 1-2 mm into
    its host Mega PCB is fine; the cable physically MATES with the
    board there.
    """
    print("\n[17] Cable-clearance airspace "
          f"(per-connector plug + cable envelopes; tol "
          f"{CABLE_CLEARANCE_TOLERANCE_MM3:.0f} mm^3):")

    import cable_keepouts as ck  # noqa: WPS433
    keepouts = ck.build_cable_keepouts()

    printed_parts = _build_world_leg0_printed_parts()
    electronics_bodies = _build_electronics_body_meshes()
    fastener_meshes = _build_fastener_envelope_meshes()

    all_ok = True
    n_check = 0
    n_fail = 0
    for ko in keepouts:
        kp_lo, kp_hi = ko.mesh.bounds
        intrusions: list[str] = []
        total = 0.0

        for name, mesh in printed_parts.items():
            a_lo, a_hi = mesh.bounds
            if np.any(kp_hi <= a_lo) or np.any(a_hi <= kp_lo):
                continue
            vol = _pair_overlap_volume(ko.mesh, mesh, pitch=1.5)
            if vol > 0.0:
                intrusions.append(f"{name}={vol:.1f}")
                total += vol

        for name, mesh in electronics_bodies.items():
            # A connector keepout legitimately overlaps with its OWN
            # board's solid envelope at the connector seating face --
            # the cable PLUGS INTO that board.  Skip self-overlap.
            if name == ko.part_name:
                continue
            a_lo, a_hi = mesh.bounds
            if np.any(kp_hi <= a_lo) or np.any(a_hi <= kp_lo):
                continue
            vol = _pair_overlap_volume(ko.mesh, mesh, pitch=1.5)
            if vol > 0.0:
                intrusions.append(f"board:{name}={vol:.1f}")
                total += vol

        for label, mesh in fastener_meshes:
            a_lo, a_hi = mesh.bounds
            if np.any(kp_hi <= a_lo) or np.any(a_hi <= kp_lo):
                continue
            vol = _pair_overlap_volume(ko.mesh, mesh, pitch=1.0)
            if vol > 0.0:
                intrusions.append(f"fastener:{label}={vol:.1f}")
                total += vol

        n_check += 1
        ok = total <= CABLE_CLEARANCE_TOLERANCE_MM3 + 1e-3
        if not ok:
            n_fail += 1
            detail = (
                f"intrusion {total:6.1f} mm^3 "
                f"(tol {CABLE_CLEARANCE_TOLERANCE_MM3:.0f})  "
                f"[{', '.join(intrusions)}]"
            )
            _label(f"{ko.part_name} {ko.connector_name}", False, detail)
            all_ok = False

    if all_ok:
        _label(
            f"{n_check} keepout(s) probed",
            True,
            "every cable airspace clear",
        )
    else:
        print(f"  ---- FAIL ({n_fail} of {n_check} probed) ----")
    return all_ok


def check_mating_face_contact():
    """For each explicit mating interface, scan along the joint axis
    and confirm the two parts mate within ``tolerance_mm``.  The
    hollowed-out coxa_link pedestal (May 2026 regression) shows up as
    a ~25 mm gap at the yaw mating face; the bracket-chassis interface
    is solid and passes."""
    print("\n[16] Mating-face contact (gap between mating faces "
          "must be <= tolerance):")

    world_parts = _build_world_assembly_parts(leg_index=0)
    interfaces = _mating_interfaces_leg0(world_parts)

    if not interfaces:
        _label("(no mating interfaces declared)", True, "")
        return True

    all_ok = True
    for name, top, bottom, point, axis, scan_dist, tol in interfaces:
        gap, top_t, bot_t = _mating_gap_along_axis(
            top, bottom, point, axis, scan_dist_mm=scan_dist,
        )
        if np.isnan(gap):
            ok = False
            detail = (
                f"could not locate mating face along axis (top_t="
                f"{top_t}, bot_t={bot_t}); widen scan_dist or check "
                f"part placement"
            )
        else:
            ok = abs(gap) <= tol + 1e-3
            detail = (
                f"gap = {gap:+6.2f} mm  "
                f"(tol +/-{tol:.2f}; top_t={top_t:+6.2f}, "
                f"bot_t={bot_t:+6.2f})"
            )
        all_ok &= _label(name, ok, detail)
    return all_ok


# ---------------------------------------------------------------------------
# 18.  Harness reach (Part D)
# ---------------------------------------------------------------------------

def check_harness_reach():
    """Verify the per-joint extension-cable counts in
    ``motor_setup.wire_harness_plan.WIRE_HARNESS_PLAN`` are NOT
    optimistic relative to the actual Manhattan path lengths.

    For each of the 18 entries, asserts::

        path_length_mm_min  <=  stock_pigtail_mm + n_ext * extension_mm

    where ``n_ext`` is the count parsed from the plan's
    ``extension_required`` string (0 for "STS3215 stock bus lead",
    1 for "+ 30 cm extension", N for "+ N x 30 cm extensions").
    PASS if every entry's required reach fits inside the
    advertised cable count; FAIL if any entry's path length
    EXCEEDS what the plan declares (= the plan is too
    optimistic, so the user would build the harness short).

    Listed in the --fast set per the task brief: the path-length
    math is sub-millisecond and catches the most common
    regression class (a CAD edit that lengthens a path past the
    declared cable count without the BOM updating).
    """
    import re
    print("\n[18] Harness reach (Part D, wire_harness_plan):")

    # Lazy import so the verifier still loads if motor_setup
    # has a syntax error.
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import wire_harness_plan as whp
    finally:
        if (os.path.dirname(os.path.abspath(__file__))
                in sys.path):
            try:
                sys.path.remove(
                    os.path.dirname(os.path.abspath(__file__)))
            except ValueError:
                pass

    stock = whp.STOCK_PIGTAIL_MM
    ext_len = whp.EXTENSION_LENGTH_MM
    plan = whp.WIRE_HARNESS_PLAN

    all_ok = True
    n_pat = re.compile(r"\+\s*(\d+)\s*x\s*30 cm extension")
    for entry in plan:
        ext_str = entry["extension_required"]
        if ext_str.startswith("STS3215 stock bus lead"):
            n_ext = 0
        elif ext_str.startswith("+ 30 cm extension"):
            n_ext = 1
        else:
            m = n_pat.search(ext_str)
            if m is None:
                all_ok &= _label(
                    f"joint {entry['joint_idx']:02d} L{entry['leg_idx']} "
                    f"{entry['axis']}",
                    False,
                    f"unparseable extension_required string: "
                    f"{ext_str!r}")
                continue
            n_ext = int(m.group(1))

        budget = stock + n_ext * ext_len
        path = entry["path_length_mm_min"]
        ok = path <= budget + 1e-3
        all_ok &= _label(
            f"joint {entry['joint_idx']:02d} L{entry['leg_idx']} "
            f"{entry['axis']:9s} ID{entry['servo_id']:02d}",
            ok,
            f"path={path:6.1f} mm  budget={budget:6.1f} mm "
            f"(stock {stock:.0f} + {n_ext} x {ext_len:.0f}); "
            f"{ext_str}",
        )
    return all_ok


# ---------------------------------------------------------------------------
# Process-pool dispatch
# ---------------------------------------------------------------------------
#
# Each entry in CHECKS is ``(display_name, callable_name)``: the parent
# process pre-builds the mesh cache, the worker initializer ships it to
# every spawn-mode worker, and each worker runs the named check with
# ``redirect_stdout`` capturing every print.  The parent emits captured
# output in DECLARATION ORDER (the index in CHECKS) so the on-screen
# output is byte-identical to the serial run.
#
# What's missing from CHECKS: ``check_workspace_self_collision``.  It is
# itself a fan-out of per-pose tasks, so the orchestrator passes the
# (already-warm) pool to it and the workspace sweep submits its own
# per-pose jobs into the same pool.  Running it INSIDE the pool as a
# single job would serialise the 176 poses inside one worker, which is
# precisely what we don't want.

CHECKS = (
    ("Mesh watertightness",       "check_watertight"),
    ("Exported-STL manifoldness", "check_export_manifold"),
    ("Exported-STL freshness",    "check_exported_stl_freshness"),
    ("Single-body connectivity",  "check_single_connected_component"),
    ("Hexagonal C6 symmetry",     "check_c6_symmetry"),
    ("Cradle openness",           "check_cradle_openness"),
    ("Bolt-hole engagement",      "check_bolt_holes"),
    ("Clamp-cap alignment",       "check_clamp_cap_alignment"),
    ("Wire-exit slot",            "check_wire_slot"),
    ("Leg harness drop",          "check_leg_harness_drop"),
    ("Self-collision",            "check_self_collision"),
    ("Clamp-cap interference",    "check_clamp_cap_interference"),
    ("Assembly interference",     "check_assembly_interference"),
    ("Servo clearance",           "check_servo_clearance"),
    ("Horn-stack clearance",      "check_horn_stack_clearance"),
    ("Horn-sweep clearance",      "check_horn_sweep_clearance"),
    ("Disc-horn fit",             "check_disc_horn_fit"),
    ("Passive-horn stack",        "check_passive_horn_stack"),
    ("Horn pattern in pads",      "check_horn_pattern_in_pad"),
    ("Cradle insert pockets",     "check_cradle_insert_pockets"),
    ("Servo insertion path",      "check_servo_insertion_path"),
    ("Bearing insertion path",    "check_bearing_insertion_path"),
    ("Bearing-cap descent path",  "check_bearing_cap_descent_path"),
    ("Hub inner-race insertion",  "check_hub_inner_race_insertion_path"),
    ("Bearing assembly sequence", "check_bearing_assembly_sequence"),
    ("Flimsy joints",             "check_flimsy_joints"),
    ("Thin sheets",               "check_thin_sheets"),
    ("Flat-bottom printability",  "check_flat_bottom"),
    ("Screwdriver access",        "check_screwdriver_access"),
    ("Fastener engagement",       "check_fastener_engagement"),
    ("Mating-face contact",       "check_mating_face_contact"),
    ("Cable clearance",           "check_cable_clearance"),
    ("Harness reach",             "check_harness_reach"),
)

WORKSPACE_CHECK_NAME = "Workspace self-collision"


# ---------------------------------------------------------------------------
# Timing instrumentation (Jun 2026, user request: "add timing output so I can
# optimise its speed later")
# ---------------------------------------------------------------------------
#
# A flat list of (label, seconds, source) records that ``main()`` collects as
# it runs.  ``source`` is one of:
#   * "ran"    -- a freshly-executed check (worker compute time via
#                 perf_counter, NOT pool queue wait, so it's a true cost).
#   * "cached" -- resolved from the result cache; the seconds are what the
#                 check took the last time it actually ran.
#   * "setup"  -- a shared fixed-overhead phase (mesh prebuild, etc.) that is
#                 NOT inside any one named check.
# Pure bookkeeping; nothing here changes any check's pass/fail behaviour.
_TIMINGS: list[tuple[str, float, str]] = []


def _record_timing(label: str, seconds: float, source: str) -> None:
    """Append one timing record (see ``_TIMINGS``)."""
    _TIMINGS.append((label, float(seconds), source))


def _print_timing_summary(wall_s: float, mode: str) -> None:
    """Emit the per-check timing table (slowest first) + the fixed-overhead
    setup phases + totals.  Clearly delineated so it never collides with the
    check output the suite's other tooling parses."""
    if not _TIMINGS:
        return
    checks = [(lbl, s, src) for (lbl, s, src) in _TIMINGS if src != "setup"]
    setup = [(lbl, s, src) for (lbl, s, src) in _TIMINGS if src == "setup"]
    checks.sort(key=lambda r: r[1], reverse=True)

    print()
    print("#" * 72)
    print("# Per-check timing (slowest first) "
          "-- profile aid, not a pass/fail signal")
    print("#" * 72)
    for lbl, s, src in checks:
        tag = "  (cached)" if src == "cached" else ""
        print(f"  {s:7.2f} s   {lbl}{tag}")
    if setup:
        print("  " + "-" * 50)
        for lbl, s, _src in setup:
            print(f"  {s:7.2f} s   [setup] {lbl}")
    sum_checks = sum(s for _l, s, _src in checks)
    n_ran = sum(1 for _l, _s, src in checks if src == "ran")
    n_cached = sum(1 for _l, _s, src in checks if src == "cached")
    sum_setup = sum(s for _l, s, _src in setup)
    print("  " + "-" * 50)
    print(f"  {sum_checks:7.2f} s   SUM of {len(checks)} checks "
          f"({n_ran} ran, {n_cached} cached) -- summed serial cost")
    if sum_setup:
        print(f"  {sum_setup:7.2f} s   SUM of shared setup phases")
    print(f"  {wall_s:7.2f} s   WALL clock ({mode})")
    if sum_checks > 0 and wall_s > 0:
        print(f"  {sum_checks / wall_s:7.2f} x   parallel speedup "
              f"(summed check cost / wall)")
    print("#" * 72)


# ---------------------------------------------------------------------------
# --fast mode: the 5 ESSENTIAL checks
# ---------------------------------------------------------------------------
#
# Profiled wall-time analysis (May 2026, see _verify_profile.txt) showed
# the full suite spent ~80 % of its time in the 6-axis ray-vote
# point-in-mesh helper, which is itself dominated by the workspace
# self-collision sweep (175 poses) and screwdriver-access / horn-sweep
# / cradle-insertion / flimsy-joint voxelisation work.  Most edits to
# the parametric CAD don't change anything that those slow checks
# probe; they break one of FIVE invariants that are cheap to test:
#
#   1.  ``check_watertight``      -- every printed STL is manifold.
#                                    Caught by trimesh's rtree boundary
#                                    pass; sub-second.
#   5.  ``check_self_collision`` -- standing-pose pairwise overlap.
#                                    One pose, voxel pitch 1.5 mm; the
#                                    175-pose sweep ("Workspace self-
#                                    collision") is the SLOW workspace
#                                    sweep, not this one.
#  15.  ``check_fastener_engagement`` -- every bolt's head bears, tip
#                                    engages, shaft span clears, and
#                                    the bolt joins >= 2 distinct
#                                    parts.  Probes are 8-azimuth
#                                    rings; ~ 1-3 s.
#  16.  ``check_mating_face_contact`` -- the 3 horn / link mating
#                                    interfaces close within tolerance.
#                                    Pure ray-cast; sub-second.
#  17.  ``check_cable_clearance``  -- every cable / connector keepout
#                                    is free of solid material.
#                                    Voxel-grid probes through 10
#                                    keepouts; ~ 1-2 s.
#
# Together these 5 catch ~ 80 % of the historical regressions
# (off-by-one bracket geometry, wrong bolt length, missing hole, mating
# face moved, cable channel obliterated) in ~ 15 % of the wall time
# of the full suite (~ 5-15 s vs ~ 95-270 s on this machine, depending
# on whether the cache is warm).  They are the "tight inner loop" set;
# anything else either has its own faster up-stream check (e.g. the
# spec-bounds validator catches gross dimension drift before the
# verifier even runs) or is the slow 175-pose / voxel-union work that
# only needs to re-run when joint kinematics or printed walls change.
#
# Run them with ``--fast``; ``--all`` (the default when no flag is
# passed) runs every entry in ``CHECKS`` plus the workspace sweep.
ESSENTIAL_CHECK_NAMES = (
    "Mesh watertightness",
    "Self-collision",
    "Fastener engagement",
    "Mating-face contact",
    "Cable clearance",
    "Harness reach",
)


# ---------------------------------------------------------------------------
# Static dependency map (drives --changed mode)
# ---------------------------------------------------------------------------
#
# Each check reads a known subset of the printed-STL inventory plus
# (optionally) the registry-level source modules.  The map below
# encodes which printed parts each check would consult; the static
# scope is hand-curated by inspecting every ``_load_mesh(name)`` call
# inside the check (plus its private helpers like
# ``_build_standing_leg`` / ``_build_world_leg0_printed_parts`` /
# ``_place_servo_bodies``).  ``--changed`` walks git diff vs
# ``origin/main``, rebuilds STLs in-memory, and selects only the
# checks whose CHECK_INPUTS intersects the changed-STL set.
#
# Future checks MUST add an entry here, otherwise the
# ``ALL_PRINTED_PARTS`` default applies and the check ALWAYS runs --
# safe but defeats the point of selection.

# Master list of every printed-STL name the verifier knows about.
# Keep this in sync with ``_MESH_BUILDERS`` -- the assertion at the
# bottom of this module enforces that.
ALL_PRINTED_PARTS = frozenset({
    "chassis_top", "chassis_bottom",
    
    "servo_clamp_cap", "switch_holster", 
    "coxa_link",
    "femur_link", "tibia_link", "foot_boot",
    # Visual-only meshes that some checks place to test interfaces:
    "servo_body", "servo_horn",
})

# Subsets used by the per-check map below.
_LEG_PARTS = frozenset({
    "coxa_link", "femur_link", "tibia_link",
})
# The yaw cradle now lives entirely in the single merged ``chassis_bottom``;
# cradle-feature checks probe the ``chassis_assembled`` solid (== the merged
# ``make_chassis_bottom``), so they are gated by its STL footprint.
_CRADLE_PARTS = frozenset({"chassis_bottom",
                            "coxa_link", "femur_link"})
_PAD_PARTS = frozenset({"coxa_link", "femur_link", "tibia_link"})
_CHASSIS_PARTS = frozenset({
    "chassis_top", "chassis_bottom",
    
})
_PRINTED_WATERTIGHT_SET = frozenset({
    "chassis_top", "chassis_bottom",
    "servo_clamp_cap",
    "coxa_link",
    "femur_link", "tibia_link", "foot_boot",
})

# Per-check dependency map.  Keys MUST match the display-name strings
# used in CHECKS / WORKSPACE_CHECK_NAME above.
CHECK_INPUTS: dict[str, frozenset[str]] = {
    "Mesh watertightness":       _PRINTED_WATERTIGHT_SET,
    # Builds + heals every single-body printed part directly from
    # hexapod_prototype, so any printed-STL change can introduce a slicer
    # non-manifold edge -> gate on the printed watertight set.
    "Exported-STL manifoldness": _PRINTED_WATERTIGHT_SET,
    # Compares the on-disk STLs against a fresh rebuild of EVERY exported
    # part, so any change anywhere can leave a file stale -> always run it
    # (the ALL_PRINTED_PARTS gate makes it fire whenever any part changes,
    # and its cache key already mixes the full hp source + all STL bytes).
    "Exported-STL freshness":    ALL_PRINTED_PARTS,
    # Builds every printed single-body part directly from hexapod_prototype
    # (incl. the one-piece coxa_link, Aug 2026 merge), so any
    # printed-STL change can flip a connectivity verdict -> gate on all parts.
    "Single-body connectivity":  ALL_PRINTED_PARTS,
    # Builds the merged chassis_bottom's floor slab directly and asserts C6.
    "Hexagonal C6 symmetry":     frozenset({"chassis_bottom"}),
    "Cradle openness":           _CRADLE_PARTS,
    "Bolt-hole engagement":      _CRADLE_PARTS,
    # Builds the clamp cap + cradle/bracket + yaw retainer meshes directly
    # from hexapod_prototype; the servo_clamp_cap STL footprint also gates it.
    "Clamp-cap alignment":       _CRADLE_PARTS | {"servo_clamp_cap"},
    "Wire-exit slot":            _CRADLE_PARTS,
    "Leg harness drop":          frozenset({"chassis_bottom"}),
    "Self-collision":            _LEG_PARTS | {"servo_clamp_cap"},
    # Builds the clamp cap + hip/knee brackets directly from
    # hexapod_prototype; the servo_clamp_cap STL footprint also gates it.
    "Clamp-cap interference":    _CRADLE_PARTS | {"servo_clamp_cap"},
    # COMPREHENSIVE all-pairs gate over the full static assembly: gated on
    # EVERY printed part so any new/changed part is auto-pulled into the
    # interference matrix (builds all parts live from hexapod_prototype).
    "Assembly interference":     ALL_PRINTED_PARTS,
    "Servo clearance":           _LEG_PARTS | {"servo_body"},
    "Horn-stack clearance":      frozenset({"femur_link", "tibia_link"}),
    "Horn-sweep clearance":      frozenset({"chassis_bottom",
                                             "servo_horn"}),
    "Horn pattern in pads":      _PAD_PARTS,
    # Pure constants check on the passive stock-horn stack; the hip/knee
    # cradle STL footprints stand in for the joints that carry it.
    "Passive-horn stack":        _CRADLE_PARTS,
    "Cradle insert pockets":     _CRADLE_PARTS,
    "Servo insertion path":      _CRADLE_PARTS,
    # Builds the one-piece coxa_link directly from hexapod_prototype; its
    # STL footprint gates it.
    "Hub inner-race insertion":  frozenset({"coxa_link"}),
    # Models the rigid 6706 vs the one-piece coxa_link + make_yaw_bearing_cap
    # + the chassis bottom tower; gate on the coxa + chassis parts so any
    # change to the hub boss / cap / tower geometry re-runs it.
    "Bearing assembly sequence": frozenset({"coxa_link", "chassis_bottom"}),
    # Builds make_yaw_bearing_cap directly; its own STL + the coxa parts gate it.
    "Bearing-cap descent path":  frozenset({"yaw_bearing_cap", "coxa_link"}),
    "Flimsy joints":             _PRINTED_WATERTIGHT_SET,
    "Thin sheets":               _PAD_PARTS,
    # Builds every PART_REGISTRY part in its print pose; any STL change
    # (esp. the chassis halves) can flip a flat-bottom verdict.
    "Flat-bottom printability":  ALL_PRINTED_PARTS,
    # check_screwdriver_access + check_fastener_engagement +
    # check_cable_clearance all build the full world assembly via
    # ``_build_world_leg0_printed_parts`` + (engagement & mating
    # only) ``_build_world_assembly_parts``, so every printed part
    # in the build can move them.  servo_horn / servo_body are
    # consulted by the fastener / mating tests.
    "Screwdriver access":        (
        _LEG_PARTS | _CHASSIS_PARTS
        | frozenset({"servo_clamp_cap", "switch_holster"})
    ),
    "Fastener engagement":       (
        _LEG_PARTS | _CHASSIS_PARTS
        | frozenset({"servo_clamp_cap", "switch_holster",
                     "servo_horn", "servo_body"})
    ),
    "Mating-face contact":       (
        _LEG_PARTS | _CHASSIS_PARTS
        | frozenset({"servo_clamp_cap", "switch_holster",
                     "servo_horn", "servo_body"})
    ),
    "Cable clearance":           (
        _LEG_PARTS | _CHASSIS_PARTS
        | frozenset({"servo_clamp_cap", "switch_holster"})
    ),
    # check_harness_reach reads WIRE_HARNESS_PLAN, which is built
    # from hexapod_prototype constants only; it doesn't touch
    # any STL bytes.  Use a minimal stl set so --changed mode
    # still selects the check when constants drift.
    "Harness reach":             frozenset(),
    # Workspace sweep places the legs + chassis + neighbour bracket.
    "Workspace self-collision":  (_LEG_PARTS | _CHASSIS_PARTS),
}

# Source-file tags besides STL bytes that gate check selection.
# Maps a check name to the set of registry-level source files it
# transitively depends on.  When ``--changed`` sees one of these
# files modified, the check is selected even if no STL bytes
# changed.
CHECK_SOURCE_DEPS: dict[str, frozenset[str]] = {
    "Fastener engagement":  frozenset({"fastener_registry"}),
    "Screwdriver access":   frozenset({"fastener_registry"}),
    "Cable clearance":      frozenset({"fastener_registry",
                                          "cable_keepouts"}),
}


# ---------------------------------------------------------------------------
# Persistent per-check cache (sqlite at .verify_cache.sqlite)
# ---------------------------------------------------------------------------
#
# The wall-time profile (see _verify_profile.txt) makes the cache an
# obvious win: 18 checks x 1-200 s each, the vast majority of which
# read the SAME mesh + constants inputs every run.  If nothing's
# changed since last time, we don't need to re-run anything.  We
# segregate cache entries per check (so a single edit only
# invalidates the affected ones) under a content-hash key:
#
#     input_hash = sha1(
#         check_name
#         + inspect.getsource(check_fn)        -- the check's own body
#         + inspect.getsource(hexapod_prototype) -- all constants +
#                                                 builder code; covers
#                                                 hp.SERVO_BODY_W etc.
#         + inspect.getsource(fastener_registry) -- catches bolt
#                                                   placement / spec
#                                                   changes
#         + concatenated sha1(STL bytes) over every stl_prototype/*.stl
#     )
#
# Stored in ``.verify_cache.sqlite`` next to this file (gitignored).
# Schema: (check_name, input_hash) -> (result_pass, runtime_s,
# message, timestamp_unix).  On every invocation we prune entries
# older than CACHE_MAX_AGE_S (= 30 days) so the file stays small
# across long-lived branches.
#
# Workflow:
#   * Cache HIT -> print "  [CACHED Xm ago] [PASS] check_name ..." and
#     skip the real run.  result_pass drives the summary table.
#   * Cache MISS -> run the check, INSERT (or REPLACE) the result.
#   * ``--no-cache`` -> bypass lookups (writes still happen, so the
#     cache stays current; useful to validate the cache against a
#     fresh cold run without flushing all entries).
#
# Limits intentionally NOT covered by the hash (would create false
# invalidations without improving correctness much):
#   * helper functions in _verify_prototype.py outside the check
#     function itself.  If you change ``_pair_overlap_volume`` you
#     must bust the cache with ``--no-cache`` (or delete the sqlite
#     file).  Refactors of helpers are rare compared to constants /
#     check edits, so this is an acceptable trade-off.
#   * Trimesh / numpy versions.  If you upgrade those, ``--no-cache``
#     once.

CACHE_DB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              ".verify_cache.sqlite")
STL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "stl_prototype")
CACHE_MAX_AGE_S = 30 * 24 * 3600  # 30 days


# Lazy: most of the inputs (hp source + fr source + STL bytes) are
# IDENTICAL across every check in one run.  Compute the partial hash
# ONCE per process and reuse it; only the per-check piece
# (check_fn source) is mixed in afterwards.
_CACHE_INPUT_HASH_GLOBAL: str | None = None
_CACHE_INPUT_HASH_GLOBAL_LOCK = threading.Lock()


def _cache_global_input_hash() -> str:
    """Return the cached sha1 of every input that's identical across
    every check in a single run: the hp + fastener_registry module
    source, plus every file under stl_prototype/.  Computed once per
    process via a memo + lock."""
    global _CACHE_INPUT_HASH_GLOBAL
    if _CACHE_INPUT_HASH_GLOBAL is not None:
        return _CACHE_INPUT_HASH_GLOBAL
    with _CACHE_INPUT_HASH_GLOBAL_LOCK:
        if _CACHE_INPUT_HASH_GLOBAL is not None:
            return _CACHE_INPUT_HASH_GLOBAL
        h = hashlib.sha1()
        try:
            h.update(inspect.getsource(hp).encode("utf-8"))
        except OSError:
            # inspect.getsource raises OSError if it can't find the
            # source file (e.g. running from a frozen zipapp).  Skip
            # this component rather than crash; the cache key
            # degrades to "STLs + check source" which is still useful.
            pass
        h.update(b"\0")
        try:
            import fastener_registry as _fr  # noqa: WPS433
            h.update(inspect.getsource(_fr).encode("utf-8"))
        except (OSError, ImportError):
            pass
        h.update(b"\0")
        if os.path.isdir(STL_DIR):
            for entry in sorted(os.listdir(STL_DIR)):
                if not entry.endswith(".stl"):
                    continue
                path = os.path.join(STL_DIR, entry)
                try:
                    with open(path, "rb") as f:
                        h.update(entry.encode("utf-8"))
                        h.update(b":")
                        h.update(hashlib.sha1(f.read()).digest())
                        h.update(b"\n")
                except OSError:
                    continue
        # Geometry-variant env flags change what hp builds WITHOUT changing
        # any source, so they must be part of the key (a closed-yoke test
        # run and a stock run would otherwise poison each other's cache).
        h.update(b"\0HEX_YOKE_REINFORCED=")
        h.update(os.environ.get("HEX_YOKE_REINFORCED", "").encode("utf-8"))
        _CACHE_INPUT_HASH_GLOBAL = h.hexdigest()
        return _CACHE_INPUT_HASH_GLOBAL


def _cache_check_input_hash(check_name: str, fn_name: str) -> str:
    """Compute the full input hash for one (check_name, check_fn)
    pair.  Pure function: callers can compute it before deciding
    whether to dispatch the check.
    """
    h = hashlib.sha1()
    h.update(check_name.encode("utf-8"))
    h.update(b"\0")
    fn = globals().get(fn_name)
    if fn is not None:
        try:
            h.update(inspect.getsource(fn).encode("utf-8"))
        except (OSError, TypeError):
            pass
    h.update(b"\0")
    h.update(_cache_global_input_hash().encode("ascii"))
    return h.hexdigest()


def _cache_open() -> sqlite3.Connection:
    """Open (creating if necessary) the per-check cache DB."""
    conn = sqlite3.connect(CACHE_DB_PATH)
    conn.execute(
        "CREATE TABLE IF NOT EXISTS check_cache ("
        "  check_name      TEXT NOT NULL,"
        "  input_hash      TEXT NOT NULL,"
        "  result          INTEGER NOT NULL,"
        "  runtime_s       REAL NOT NULL,"
        "  message         TEXT NOT NULL,"
        "  timestamp_unix  REAL NOT NULL,"
        "  PRIMARY KEY (check_name, input_hash))"
    )
    conn.execute(
        "CREATE INDEX IF NOT EXISTS check_cache_ts_idx "
        "ON check_cache(timestamp_unix)"
    )
    conn.commit()
    return conn


def _cache_lookup(conn: sqlite3.Connection,
                    check_name: str,
                    input_hash: str):
    """Return ``(result_bool, runtime_s, message, timestamp_unix)`` or
    ``None`` on cache miss."""
    cur = conn.execute(
        "SELECT result, runtime_s, message, timestamp_unix "
        "FROM check_cache WHERE check_name = ? AND input_hash = ?",
        (check_name, input_hash),
    )
    row = cur.fetchone()
    if row is None:
        return None
    return (bool(row[0]), float(row[1]), str(row[2]), float(row[3]))


def _cache_insert(conn: sqlite3.Connection,
                    check_name: str,
                    input_hash: str,
                    result: bool,
                    runtime_s: float,
                    message: str) -> None:
    """Insert (or replace) a cache row for one (check_name,
    input_hash) pair.  ``message`` is the captured stdout from the
    check, truncated to 64 KiB to keep the DB compact."""
    if len(message) > 65536:
        message = message[:65536] + "\n... (truncated to 64 KiB)\n"
    conn.execute(
        "INSERT OR REPLACE INTO check_cache "
        "(check_name, input_hash, result, runtime_s, "
        " message, timestamp_unix) VALUES (?, ?, ?, ?, ?, ?)",
        (check_name, input_hash,
         int(bool(result)), float(runtime_s),
         message, time.time()),
    )
    conn.commit()


def _cache_prune(conn: sqlite3.Connection) -> int:
    """Drop rows older than CACHE_MAX_AGE_S seconds.  Returns the
    number of rows deleted."""
    cutoff = time.time() - CACHE_MAX_AGE_S
    cur = conn.execute(
        "DELETE FROM check_cache WHERE timestamp_unix < ?", (cutoff,))
    n = cur.rowcount
    conn.commit()
    return int(n) if n is not None else 0


def _cache_format_ago(timestamp_unix: float) -> str:
    """Human-readable "Xs / Xm / Xh / Xd ago" string."""
    delta = max(0.0, time.time() - float(timestamp_unix))
    if delta < 60.0:
        return f"{int(delta)} s ago"
    if delta < 3600.0:
        return f"{int(delta / 60.0)} m ago"
    if delta < 86400.0:
        return f"{int(delta / 3600.0)} h ago"
    return f"{int(delta / 86400.0)} d ago"


# ---------------------------------------------------------------------------
# Smart selection from ``git diff origin/main`` (drives --changed)
# ---------------------------------------------------------------------------

# Source files that may influence STL output (we rebuild + byte-diff
# to find which printed parts actually changed) versus those that
# only affect a few checks (we propagate via CHECK_SOURCE_DEPS).
_STL_PRODUCING_SOURCES = frozenset({
    "hexapod_walker/prototype_sts3215/hexapod_prototype.py",
})
_REGISTRY_SOURCES = {
    "hexapod_walker/prototype_sts3215/fastener_registry.py":
        "fastener_registry",
    "hexapod_walker/prototype_sts3215/cable_keepouts.py":
        "cable_keepouts",
    "hexapod_walker/prototype_sts3215/scripts/build_prototype_assembly.py":
        "build_prototype_assembly",
}
_VERIFIER_SOURCE = "hexapod_walker/prototype_sts3215/_verify_prototype.py"


def _git_changed_files(base_ref: str = "origin/main") -> list[str]:
    """Return the list of files modified between ``base_ref`` and the
    working tree (staged + unstaged + untracked changes).  Falls back
    to ``HEAD`` if ``base_ref`` is unknown to git.
    """
    import subprocess  # noqa: WPS433

    repo_root = os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))

    def _run(args, cwd=repo_root):
        proc = subprocess.run(
            args, cwd=cwd, check=False,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        return proc.returncode, proc.stdout, proc.stderr

    # Verify the ref exists; if not, fall back to HEAD.
    rc, _, _ = _run(["git", "rev-parse", "--verify", base_ref])
    if rc != 0:
        base_ref = "HEAD"

    changed: set[str] = set()
    # Tracked file changes (committed + staged + unstaged) since base_ref.
    rc, out, _ = _run(["git", "diff", "--name-only", base_ref])
    if rc == 0:
        for line in out.splitlines():
            if line.strip():
                changed.add(line.strip())
    # Untracked files (e.g. brand-new STLs not committed yet).
    rc, out, _ = _run(["git", "ls-files", "--others", "--exclude-standard"])
    if rc == 0:
        for line in out.splitlines():
            if line.strip():
                changed.add(line.strip())
    return sorted(changed)


def _changed_stl_set(changed_files: list[str]) -> set[str]:
    """Compute the set of changed STL-part NAMES (just the bare
    'coxa_bracket' name -- not the path / filename), from the list
    of repo-relative changed paths.  Detects three sources:

      1. Direct ``stl_prototype/<name>.stl`` paths in the diff.
      2. Any of ``_STL_PRODUCING_SOURCES`` modified -> we rebuild every
         STL in-memory (via ``_MESH_BUILDERS``) and compare its
         serialised bytes to whatever is on disk under
         ``stl_prototype/``; mismatches go into the changed set.
      3. Files that are new but not yet in git (untracked STLs).

    ``stl_prototype/`` files use the ``"<name>.stl"`` convention so
    the part name is the basename without the extension.
    """
    changed: set[str] = set()

    # (1) Direct STL diffs.
    for path in changed_files:
        if path.startswith("hexapod_walker/prototype/stl_prototype/") \
                and path.endswith(".stl"):
            name = os.path.basename(path)[:-len(".stl")]
            if name in ALL_PRINTED_PARTS:
                changed.add(name)

    # (2) Rebuild + byte-compare when an STL-producing source is in
    # the diff.  Skip the rebuild if neither file is touched -- saves
    # ~3-4 s of trimesh CSG.
    stl_sources_changed = any(p in _STL_PRODUCING_SOURCES
                                 for p in changed_files)
    if stl_sources_changed:
        for name in sorted(ALL_PRINTED_PARTS):
            if name in changed:
                continue   # already in the set
            disk_path = os.path.join(STL_DIR, f"{name}.stl")
            if not os.path.isfile(disk_path):
                # No disk reference; treat as changed so a fresh
                # rebuild gets verified.
                changed.add(name)
                continue
            try:
                mesh = _load_mesh(name, copy=False)
                fresh_bytes = mesh.export(file_type="stl")
                if isinstance(fresh_bytes, str):
                    fresh_bytes = fresh_bytes.encode("ascii")
                with open(disk_path, "rb") as f:
                    disk_bytes = f.read()
                if fresh_bytes != disk_bytes:
                    changed.add(name)
            except Exception:
                # If the rebuild fails, assume changed so we run the
                # affected checks rather than silently skipping them.
                changed.add(name)

    return changed


def _changed_source_tags(changed_files: list[str]) -> set[str]:
    """Map changed source files (other than STL producers) to the
    tag names used in CHECK_SOURCE_DEPS."""
    tags: set[str] = set()
    for path in changed_files:
        tag = _REGISTRY_SOURCES.get(path)
        if tag is not None:
            tags.add(tag)
    return tags


def _select_changed_checks(check_subset, *,
                              run_workspace_in: bool,
                              base_ref: str = "origin/main"
                              ) -> tuple:
    """Filter ``check_subset`` (+ optionally the workspace check)
    down to the entries whose static input deps intersect the changed
    set vs ``base_ref``.  Returns ``(filtered_subset,
    filtered_run_workspace, summary_line)``.

    The summary_line is a single human-readable string; the caller
    prints it after the standard banner so users see what got
    selected and why.
    """
    changed_files = _git_changed_files(base_ref=base_ref)
    changed_stls = _changed_stl_set(changed_files)
    changed_tags = _changed_source_tags(changed_files)
    verifier_changed = _VERIFIER_SOURCE in changed_files

    def _is_changed_check(name: str) -> bool:
        inputs = CHECK_INPUTS.get(name, ALL_PRINTED_PARTS)
        if inputs & changed_stls:
            return True
        if CHECK_SOURCE_DEPS.get(name, frozenset()) & changed_tags:
            return True
        # If the verifier source itself changed, fall back to the
        # cache to dedup: include the check so the cache hash decides
        # whether the work needs to actually run.  See the big
        # comment block above CACHE_DB_PATH for why this is safe.
        if verifier_changed:
            return True
        return False

    filtered_subset = [(n, f) for (n, f) in check_subset
                          if _is_changed_check(n)]
    filtered_run_workspace = (run_workspace_in
                                  and _is_changed_check(WORKSPACE_CHECK_NAME))

    total_in = len(check_subset) + int(run_workspace_in)
    total_sel = len(filtered_subset) + int(filtered_run_workspace)
    selected_names = (
        [n for n, _ in filtered_subset]
        + ([WORKSPACE_CHECK_NAME] if filtered_run_workspace else [])
    )
    if changed_stls:
        change_summary = (
            f"STLs={sorted(changed_stls)}"
            + (f", sources={sorted(changed_tags)}"
                if changed_tags else "")
            + (f", verifier={_VERIFIER_SOURCE!r}"
                if verifier_changed else "")
        )
    elif changed_tags or verifier_changed:
        change_summary = (
            f"sources={sorted(changed_tags)}"
            + (f", verifier={_VERIFIER_SOURCE!r}"
                if verifier_changed else "")
        )
    else:
        change_summary = "no STL / source / verifier changes vs base"

    summary = (
        f"  [SMART SELECT] {total_sel} of {total_in} checks selected "
        f"based on git diff ({base_ref}): {change_summary}\n"
        f"  [SMART SELECT] running: "
        + (", ".join(selected_names) if selected_names else "(none)")
    )
    return filtered_subset, filtered_run_workspace, summary


def _worker_initializer(mesh_cache, inside_mode):
    """ProcessPoolExecutor initializer: install the parent's pre-built
    mesh cache so workers do NOT pay any constructive-solid rebuild
    cost, and propagate the parent's --inside-mode so spawn-mode
    workers use the same dispatcher branch.  ``mesh_cache`` is pickled
    by spawn-mode workers as part of initargs; trimesh.Trimesh
    round-trips through pickle cleanly."""
    global _MESH_CACHE, _WS_WORKER_STATE, _inside_mode, _inside_mismatches
    _MESH_CACHE.clear()
    _MESH_CACHE.update(mesh_cache)
    _WS_WORKER_STATE.clear()
    _inside_mode = inside_mode
    _inside_mismatches = []


def _check_runner(display_name: str, fn_name: str):
    """Worker entry point: resolve ``fn_name`` in this module's globals,
    run it under ``redirect_stdout`` to capture every print, and return
    ``(display_name, ok, captured_output_str, mismatch_records, runtime_s)``.
    Exceptions are caught and serialised into the output string so a
    worker traceback never silently disappears.  ``mismatch_records``
    is the list of points_inside disagreements accumulated by this
    check (empty unless --inside-mode is ``both``); the parent
    aggregates them across all workers.  ``runtime_s`` is the worker's
    own perf_counter compute time (NOT pool queue wait) for the timing
    summary."""
    global _inside_mismatches
    buf = io.StringIO()
    fn = globals()[fn_name]
    _set_inside_check_context(display_name)
    _inside_mismatches = []
    t0 = time.perf_counter()
    try:
        with redirect_stdout(buf):
            ok = bool(fn())
    except Exception:
        import traceback
        traceback.print_exc(file=buf)
        return (display_name, False, buf.getvalue(),
                list(_inside_mismatches), time.perf_counter() - t0)
    return (display_name, bool(ok), buf.getvalue(),
            list(_inside_mismatches), time.perf_counter() - t0)


def _print_cache_hit(name: str,
                       result: bool,
                       runtime_s: float,
                       timestamp_unix: float) -> None:
    """Format and print a single cache-hit line in the same style as
    a freshly-run check (matches ``_label``'s tabular layout)."""
    flag = "PASS" if result else "FAIL"
    ago = _cache_format_ago(timestamp_unix)
    detail = f"was {runtime_s:5.1f} s when last run"
    print(f"  [CACHED {ago}] [{flag}]  {name:36s}  {detail}")


def _try_cache_hit(cache_conn,
                    name: str,
                    fn_name: str,
                    use_cache: bool):
    """If ``use_cache`` and a row exists for (name, hash), return
    ``(input_hash, hit_tuple)`` and print the cache-hit line as a
    side effect.  Otherwise return ``(input_hash, None)`` so the
    caller knows it must run the check."""
    input_hash = _cache_check_input_hash(name, fn_name)
    if not use_cache or cache_conn is None:
        return input_hash, None
    hit = _cache_lookup(cache_conn, name, input_hash)
    if hit is None:
        return input_hash, None
    result, runtime_s, _message, ts = hit
    _print_cache_hit(name, result, runtime_s, ts)
    _record_timing(name, runtime_s, "cached")
    return input_hash, hit


def _run_checks_serial(check_subset, *, cache_conn=None, use_cache=True):
    """Run the ``check_subset`` (subset of CHECKS) in the current
    process, printing as we go.  Returns a list of ``(name, ok)``
    in declaration order.  In serial mode mismatch records accumulate
    directly in the parent's ``_inside_mismatches`` global.

    If ``cache_conn`` is provided we look up each (name, input_hash)
    first; on cache HIT we print "[CACHED ...]" and skip the real
    run.  On MISS we run and INSERT the result.  Pass ``use_cache=
    False`` to bypass lookups while still writing fresh results.
    """
    out = []
    for name, fn_name in check_subset:
        input_hash, hit = _try_cache_hit(cache_conn, name, fn_name, use_cache)
        if hit is not None:
            out.append((name, hit[0]))
            continue
        _set_inside_check_context(name)
        buf = io.StringIO()
        t0 = time.perf_counter()
        try:
            with redirect_stdout(buf):
                ok = bool(globals()[fn_name]())
        except Exception:
            import traceback
            traceback.print_exc(file=buf)
            ok = False
        runtime_s = time.perf_counter() - t0
        output = buf.getvalue()
        if output:
            sys.stdout.write(output)
            sys.stdout.flush()
        _record_timing(name, runtime_s, "ran")
        print(f"  [timing] {name}: {runtime_s:.1f} s")
        if cache_conn is not None:
            _cache_insert(cache_conn, name, input_hash,
                          ok, runtime_s, output)
        out.append((name, ok))
    return out


def _run_checks_pool(check_subset, pool, *, cache_conn=None, use_cache=True):
    """Submit each check in ``check_subset`` to ``pool`` and print
    each worker's captured stdout in DECLARATION ORDER (not completion
    order).  Returns a list of ``(name, ok)``.  Any mismatch records
    a worker accumulated under --inside-mode=both are appended to the
    parent's ``_inside_mismatches`` list for the final summary.

    Cache integration: the PARENT looks up each (name, input_hash)
    BEFORE submitting.  Cache hits are printed inline and never get
    submitted to the pool; cache misses are submitted as usual and
    INSERTed into the cache when the future resolves.
    """
    # Resolve cache hits + queue misses.  Slots track each entry's
    # position in the original check_subset so we can print results
    # in declaration order even when the pool resolves them out of
    # order.
    slots: list[dict] = []
    for name, fn_name in check_subset:
        input_hash, hit = _try_cache_hit(cache_conn, name, fn_name, use_cache)
        if hit is not None:
            slots.append({
                "name":   name,
                "kind":   "hit",
                "ok":     hit[0],
            })
            continue
        fut = pool.submit(_check_runner, name, fn_name)
        slots.append({
            "name":       name,
            "fn_name":    fn_name,
            "kind":       "future",
            "future":     fut,
            "input_hash": input_hash,
            "t_submit":   time.monotonic(),
        })

    out = []
    for s in slots:
        if s["kind"] == "hit":
            out.append((s["name"], s["ok"]))
            continue
        _name, ok, output, worker_mismatches, compute_s = s["future"].result()
        # Cache the WORKER'S own compute time (perf_counter, not the
        # submit->result wall that includes pool queue wait) so a cached
        # replay reports the real per-check cost.
        runtime_s = compute_s
        if output:
            sys.stdout.write(output)
            sys.stdout.flush()
        _record_timing(s["name"], compute_s, "ran")
        print(f"  [timing] {s['name']}: {compute_s:.1f} s")
        if worker_mismatches:
            _inside_mismatches.extend(worker_mismatches)
        if cache_conn is not None:
            _cache_insert(cache_conn, s["name"], s["input_hash"],
                          ok, runtime_s, output)
        out.append((s["name"], ok))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Run all post-redesign correctness checks on the "
                    "prototype hexapod (mesh cache + process-pool "
                    "fan-out).",
    )
    parser.add_argument(
        "--with-arm",
        action="store_true",
        help=(
            "Also run the OPTIONAL arm verification ([4b] arm-vs-chassis "
            "interference, [6b] flimsy joints on the 5 new arm parts). "
            "Off by default; the prototype's base checks are unchanged."
        ),
    )
    parser.add_argument(
        "--serial",
        action="store_true",
        help=("Skip the process pool entirely; run every check in the "
              "main process in declaration order.  Use this when a "
              "worker traceback is mangled through pickle and you need "
              "a clean stack to debug.  ~3-6x slower than --workers N."),
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=None,
        help=("Override the default worker count.  Default is "
              "min(8, os.cpu_count()); pass --workers 1 for a single-"
              "process-but-still-spawning pool (mostly useful for "
              "stress-testing the dispatch path)."),
    )
    parser.add_argument(
        "--profile",
        default=None,
        metavar="PATH",
        help=("Dump a cProfile snapshot of the PARENT process to PATH "
              "when the run finishes.  Workers are NOT profiled "
              "individually (combine with --serial if you want the "
              "full suite profiled in one process)."),
    )
    parser.add_argument(
        "--only",
        action="append",
        default=[],
        metavar="CHECK_NAME",
        help=("Run only the named check(s); the value must match a "
              "name in the declaration order list (e.g. "
              "'Servo clearance', 'Workspace self-collision').  "
              "Repeatable: '--only A --only B' runs both."),
    )
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--fast",
        action="store_true",
        help=("FAST MODE: run only the 5 ESSENTIAL checks "
              "(watertightness, self-collision standing pose, fastener "
              "engagement, mating-face contact, cable clearance). "
              "Catches ~ 80 %% of regressions in ~ 15 %% of the wall "
              "time of the full suite (typically 5-15 s vs 95-270 s).  "
              "Skips the 175-pose workspace sweep, screwdriver-access "
              "ray cones, cradle-insertion sweeps, and the voxelised "
              "thin-sheet / flimsy-joint passes.  Use ``--all`` (or no "
              "flag) for a pre-commit pass."),
    )
    mode_group.add_argument(
        "--all",
        action="store_true",
        help=("Explicit full-suite mode: run every check in CHECKS "
              "plus the workspace self-collision sweep.  Same as "
              "passing no mode flag at all; kept as an explicit flag "
              "so scripts can be unambiguous about their intent "
              "(``--all`` reads better than ``# default mode``)."),
    )
    parser.add_argument(
        "--no-cache",
        action="store_true",
        help=("Bypass the persistent per-check cache for LOOKUPS; "
              "every selected check re-runs even if a cache hit is "
              "available.  Writes still happen so the cache stays "
              "current.  Use this when you've edited a verifier "
              "helper function (which the cache key intentionally "
              "doesn't track) or upgraded trimesh / numpy."),
    )
    parser.add_argument(
        "--changed",
        action="store_true",
        help=("SMART SELECT: run only the checks whose static input "
              "dependency set intersects what changed since the base "
              "branch.  Walks ``git diff origin/main``, rebuilds "
              "STLs in-memory to detect which printed parts actually "
              "changed bytes, and consults the CHECK_INPUTS / "
              "CHECK_SOURCE_DEPS maps in this module.  Combine with "
              "``--fast`` to intersect with the essential set "
              "(``--fast`` AND ``--changed`` runs essential checks "
              "that are also affected by the diff)."),
    )
    parser.add_argument(
        "--base-ref",
        default="origin/main",
        metavar="REF",
        help=("Git ref to diff against for ``--changed``.  Default "
              "``origin/main``.  Pass ``HEAD~1`` to inspect just the "
              "last commit, or any other ref the local git knows."),
    )
    parser.add_argument(
        "--inside-mode",
        choices=INSIDE_MODES,
        default=INSIDE_MODE_RAYS,
        help=("Select the implementation used by points_inside(): "
              "'rays' = 6-axis ray vote (historical default; robust "
              "against early-prototype boolean-union false positives), "
              "'contains' = trimesh.Trimesh.contains() (much faster; "
              "valid because every cached part passes check_watertight), "
              "'both' = run both, record every disagreement to a "
              "module-level list, and exit with code 2 if any "
              "disagreement is found.  Use 'both' to re-validate "
              "equivalence whenever the geometry or trimesh stack "
              "changes."),
    )
    args = parser.parse_args(argv)

    global _inside_mode
    _inside_mode = args.inside_mode

    profiler = None
    if args.profile:
        profiler = cProfile.Profile()
        profiler.enable()

    print("=" * 72)
    print("PROTOTYPE design verification")
    if args.with_arm:
        print("  (with --with-arm: optional arm checks ENABLED)")
    print("=" * 72)

    only_set = set(args.only) if args.only else None
    fast_set = (set(ESSENTIAL_CHECK_NAMES) if args.fast else None)

    def _is_selected(name):
        if only_set is not None and name not in only_set:
            return False
        if fast_set is not None and name not in fast_set:
            return False
        return True

    check_subset = [(n, f) for (n, f) in CHECKS if _is_selected(n)]
    run_workspace = _is_selected(WORKSPACE_CHECK_NAME)

    if only_set is not None:
        missing = only_set - {n for n, _ in CHECKS} - {WORKSPACE_CHECK_NAME}
        if missing:
            print(f"  WARN: --only entries not matched against any known "
                  f"check: {sorted(missing)}", file=sys.stderr)

    if args.fast:
        total_checks = len(CHECKS) + 1  # +1 for workspace sweep
        n_skipped = total_checks - len(check_subset) - int(run_workspace)
        print(f"  [FAST MODE] skipping {n_skipped} slow checks; "
              f"use --all to run everything.")
        print(f"  [FAST MODE] running {len(check_subset)} "
              f"essential check(s): "
              f"{', '.join(n for n, _ in check_subset)}")

    if args.changed:
        # Build the mesh cache lazily INSIDE _select_changed_checks --
        # rebuilding STLs in memory requires the cache to be primed
        # for the parts we test, and we only pay the cost when an
        # STL-producing source actually changed.
        check_subset, run_workspace, smart_summary = _select_changed_checks(
            check_subset,
            run_workspace_in=run_workspace,
            base_ref=args.base_ref,
        )
        print(smart_summary)

    t_start = time.monotonic()

    cache_conn = _cache_open()
    n_pruned = _cache_prune(cache_conn)
    if n_pruned:
        print(f"  Cache: pruned {n_pruned} entries older than "
              f"{CACHE_MAX_AGE_S // 86400} days.")

    use_cache = not args.no_cache
    if not use_cache:
        print("  Cache: --no-cache set; ignoring previous results "
              "but still writing new ones.")

    # Mesh cache is only needed when we have at least ONE cache miss
    # to dispatch.  Pre-check each selected check (and the workspace
    # sweep) against the cache and only prebuild the meshes if any
    # of them miss.  Saves ~ 3-4 s on warm-cache runs.
    will_miss = False
    if use_cache:
        for name, fn_name in check_subset:
            ih = _cache_check_input_hash(name, fn_name)
            if _cache_lookup(cache_conn, name, ih) is None:
                will_miss = True
                break
        if not will_miss and run_workspace:
            wih = _cache_check_input_hash(WORKSPACE_CHECK_NAME,
                                            "check_workspace_self_collision")
            if _cache_lookup(cache_conn, WORKSPACE_CHECK_NAME, wih) is None:
                will_miss = True
    else:
        will_miss = True

    if will_miss:
        print("  Pre-building mesh cache in parent process ...")
        _t_prebuild = time.perf_counter()
        prebuild_mesh_cache()
        _record_timing("mesh cache prebuild", time.perf_counter() - _t_prebuild,
                       "setup")
    else:
        print("  Cache: every selected check has a hit; "
              "skipping mesh prebuild.")

    results: list[tuple[str, bool]] = []
    n_workers = 1  # only meaningful for the non-serial branch summary line

    def _run_workspace_with_cache(pool=None):
        """Cache-aware wrapper for check_workspace_self_collision.
        On HIT we print the cache-hit line and skip the real run;
        on MISS we capture the check's stdout, run it, INSERT the
        cached row, and replay the output."""
        ws_input_hash = _cache_check_input_hash(
            WORKSPACE_CHECK_NAME, "check_workspace_self_collision")
        if use_cache:
            hit = _cache_lookup(cache_conn, WORKSPACE_CHECK_NAME,
                                  ws_input_hash)
            if hit is not None:
                result, runtime_s, _msg, ts = hit
                _print_cache_hit(WORKSPACE_CHECK_NAME, result,
                                   runtime_s, ts)
                _record_timing(WORKSPACE_CHECK_NAME, runtime_s, "cached")
                return result

        buf = io.StringIO()
        t0 = time.perf_counter()
        try:
            with redirect_stdout(buf):
                if pool is None:
                    ws_ok = bool(check_workspace_self_collision())
                else:
                    ws_ok = bool(check_workspace_self_collision(pool=pool))
        except Exception:
            import traceback
            traceback.print_exc(file=buf)
            ws_ok = False
        runtime_s = time.perf_counter() - t0
        output = buf.getvalue()
        if output:
            sys.stdout.write(output)
            sys.stdout.flush()
        _record_timing(WORKSPACE_CHECK_NAME, runtime_s, "ran")
        print(f"  [timing] {WORKSPACE_CHECK_NAME}: {runtime_s:.1f} s")
        _cache_insert(cache_conn, WORKSPACE_CHECK_NAME, ws_input_hash,
                       ws_ok, runtime_s, output)
        return ws_ok

    if args.serial:
        # Run every check inline.  Workspace sweep also runs in this
        # process, with no pool, so its 176 poses execute serially --
        # this is the SAME code path as the historical implementation
        # and is the "ground truth" we diff the parallel output against.
        results.extend(_run_checks_serial(
            check_subset,
            cache_conn=cache_conn, use_cache=use_cache))
        if run_workspace:
            ws_ok = _run_workspace_with_cache()
            results.append((WORKSPACE_CHECK_NAME, ws_ok))
    else:
        n_workers = (args.workers
                       if args.workers is not None
                       else min(8, (os.cpu_count() or 1)))
        # spawn context for stability on macOS (fork + trimesh's
        # rtree-backed mesh.contains has been observed to deadlock).
        ctx = multiprocessing.get_context("spawn")
        with concurrent.futures.ProcessPoolExecutor(
            max_workers=n_workers,
            mp_context=ctx,
            initializer=_worker_initializer,
            initargs=(_MESH_CACHE, _inside_mode),
        ) as pool:
            # Phase 1: the small checks fan out across the pool.  Each
            # cache hit is resolved synchronously in the parent and
            # never submitted; cache misses go to the pool and get
            # INSERTed when the future resolves.
            results.extend(_run_checks_pool(
                check_subset, pool,
                cache_conn=cache_conn, use_cache=use_cache))

            # Phase 2: the workspace sweep dispatches its own 176
            # per-pose jobs into the same pool.  We run it in the
            # PARENT so its print statements stream to the real
            # stdout (and any redirect_stdout captures it correctly).
            if run_workspace:
                ws_ok = _run_workspace_with_cache(pool=pool)
                results.append((WORKSPACE_CHECK_NAME, ws_ok))

    if args.with_arm:
        # Arm checks reach into the optional arm module which has its
        # own internal prints; we run them serially in the parent for
        # the same reasons as the workspace sweep.
        results.extend(_optional_arm_checks())

    if cache_conn is not None:
        cache_conn.close()

    print()
    print("=" * 72)
    print("Summary:")
    all_ok = True
    for name, ok in results:
        flag = "PASS" if ok else "FAIL"
        print(f"   [{flag}]  {name}")
        all_ok &= ok
    print("=" * 72)
    elapsed = time.monotonic() - t_start
    _mode = 'serial' if args.serial else f'pool max_workers={n_workers}'
    print(f"Total verifier wall time: {elapsed:6.1f} s ({_mode})")

    _print_timing_summary(elapsed, _mode)

    if profiler is not None:
        profiler.disable()
        profiler.dump_stats(args.profile)
        print(f"cProfile snapshot written to {args.profile}")

    mismatch_exit = (_inside_mode == INSIDE_MODE_BOTH
                      and len(_inside_mismatches) > 0)
    if _inside_mode == INSIDE_MODE_BOTH:
        _print_inside_mismatch_summary()

    if mismatch_exit:
        print("EXIT 2: --inside-mode=both found disagreements between "
              "the 6-ray vote and mesh.contains; do NOT flip the "
              "default without inspecting them.")
        return 2

    if all_ok:
        print("All checks passed. The prototype is ready to print.")
        return 0
    else:
        print("FAIL.  Fix the failing checks before ordering parts.")
        return 1


def _print_inside_mismatch_summary() -> None:
    """Print a compact summary of points_inside disagreements
    accumulated during a --inside-mode=both run.  Caps the
    per-coordinate listing at the first 20 globally so the output
    stays bounded even when many mismatches occur."""
    print()
    print("=" * 72)
    if not _inside_mismatches:
        print("[MISMATCH SUMMARY] --inside-mode=both: ZERO disagreements "
              "between rays and mesh.contains across the full check "
              "suite.")
        print("=" * 72)
        return

    total = sum(int(r["n_mismatch"]) for r in _inside_mismatches)
    total_points = sum(int(r["n_points"]) for r in _inside_mismatches)
    by_check: dict = {}
    for r in _inside_mismatches:
        k = r["check"]
        by_check[k] = by_check.get(k, 0) + int(r["n_mismatch"])
    print(f"[MISMATCH SUMMARY] --inside-mode=both: "
          f"{len(_inside_mismatches)} record(s), "
          f"{total} mismatching point(s) across "
          f"{total_points} probed point(s) "
          f"({(total / max(total_points, 1)) * 100.0:6.4f} %).")
    print("  Per-check distribution (mismatching points):")
    for check_name, n in sorted(by_check.items(), key=lambda kv: -kv[1]):
        print(f"   {n:6d}  {check_name}")
    print()
    print(f"  First (up to) 20 mismatch coordinates "
          f"(check / mesh vol / xyz / rays -> contains):")
    shown = 0
    for r in _inside_mismatches:
        if shown >= 20:
            break
        vol = r["mesh"]["volume"]
        vol_str = f"{vol:8.1f}" if vol is not None else "      ? "
        wt = r["mesh"].get("watertight")
        for c, rs, cs in zip(r["coords"],
                              r["rays_says"], r["contains_says"]):
            if shown >= 20:
                break
            x, y, z = c
            print(f"   [{r['check']:36s}] mesh_vol={vol_str} wt={wt}  "
                  f"xyz=({x:+8.2f},{y:+8.2f},{z:+8.2f})  "
                  f"rays={'INSIDE' if rs else 'OUTSIDE'} -> "
                  f"contains={'INSIDE' if cs else 'OUTSIDE'}")
            shown += 1
    print("=" * 72)


if __name__ == "__main__":
    sys.exit(main())
