#!/usr/bin/env python3
"""THE geometry source for the rigid-hip variant's six printables.

STEP-FIRST (user, Aug 2026: "the official way is to make step files"):
every variant printable is authored HERE as a build123d/OpenCascade BREP
solid.  This script exports each part as .step (the editable CAD truth)
plus a tessellated .stl into ``step/stl/``; the assembly/check driver
``make_rigid_hip_variant.py`` then loads those STLs, runs the full
geometric check suite on the assembled robot, copies them into the
print set (``stl/``) and builds the BuildViz scene.  There is no trimesh
twin of these builders anymore -- edit geometry here, nowhere else.

Constants are imported from ``make_rigid_hip_variant.py`` (which in turn
derives them from hexapod_prototype), so there are no dimension forks.
Production parts the variant edits (coxa link, chassis bottom, servo
clamp cap) start from the cad_step_test base sidecar's BREP ports and
receive the variant's boolean edits.

Run (build123d needs a 3.12 interpreter; make_rigid_hip_variant.py runs
this for you by default):

  uv run --no-project --python 3.12 \
    --with build123d --with trimesh --with numpy --with manifold3d \
    python concepts/rigid_hip/build_rigid_hip_step.py
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

from build123d import Pos, Rotation

HERE = Path(__file__).resolve().parent            # concepts/rigid_hip
PROTO_DIR = HERE.parent.parent                    # prototype_sts3215
CAD_STEP_DIR = PROTO_DIR / "cad_step_test"        # base BREP ports + export tail
sys.path.insert(0, str(CAD_STEP_DIR))
sys.path.insert(0, str(HERE))

from step_common import StepPart, export_all, write_bundle  # noqa: E402

# Outputs live IN the concept directory (next to the print set in
# concepts/rigid_hip/stl/): .step files directly in step/, the
# BREP-derived STLs in step/stl/, manifest + bundle alongside the .step files.
STEP_OUT_DIR = HERE / "step"

import build_step_first_test as step  # noqa: E402
import hexapod_prototype as hp  # noqa: E402
import make_rigid_hip_variant as rv  # noqa: E402


# --- span-style primitive adapters -----------------------------------------
# The variant's mesh builders specify cylinders by (r, z0, z1) spans; these
# wrap the base sidecar's (radius, length, center) helpers with the same
# signatures so each BREP builder below stays line-for-line comparable with
# its mesh twin in make_rigid_hip_variant.py.

_box = step._box  # same (extents, center) signature as the variant helper


def _cyl_z(r: float, z0: float, z1: float,
           x: float = 0.0, y: float = 0.0) -> object:
    return step._cyl_z(r, z1 - z0, (x, y, (z0 + z1) / 2.0))


def _cyl_y(r: float, y0: float, y1: float,
           x: float = 0.0, z: float = 0.0) -> object:
    return step._cyl_y(r, y1 - y0, (x, (y0 + y1) / 2.0, z))


def _hex_prism(apothem: float, z0: float, z1: float,
               flats_at_rings: bool = False) -> object:
    """Matches the variant's _hex_prism: default puts hex VERTICES at the
    ring azimuths (30/90/... deg); flats_at_rings rotates 30 deg."""
    circ = apothem / math.cos(math.pi / 6.0)
    a0 = 0.0 if flats_at_rings else math.pi / 6.0
    pts = [
        (circ * math.cos(a0 + i * math.pi / 3.0),
         circ * math.sin(a0 + i * math.pi / 3.0))
        for i in range(6)
    ]
    return Pos(0.0, 0.0, (z0 + z1) / 2.0) * step._xy_polygon_prism(pts, z1 - z0)


# --- variant printables -----------------------------------------------------

def make_hip_clamp_cap_rigid() -> object:
    """Stock clamp cap + yaw-axis pedestal + inner-race press boss + two
    puller notches."""
    cap = step.make_servo_clamp_cap()
    ped = _cyl_y(rv.PED_OD / 2.0, rv.PED_Y0, rv.PED_Y1, z=rv.AXIS_Z)
    boss = _cyl_y(rv.BOSS_OD / 2.0, rv.PED_Y1 - 1.0, rv.BOSS_Y1, z=rv.AXIS_Z)
    tip = _cyl_y(rv.BOSS_OD / 2.0 - rv.BOSS_TIP_STEP, rv.BOSS_Y1 - 0.1,
                 rv.TIP_Y1, z=rv.AXIS_Z)
    body = step._union(cap, ped, boss, tip)
    notches = []
    for sx in (+1.0, -1.0):
        ext_x = rv.PED_OD / 2.0 - rv.PULLER_NOTCH_R0 + 3.0
        notches.append(_box(
            (ext_x, rv.PULLER_NOTCH_DEPTH + 0.05, rv.PULLER_NOTCH_W),
            (sx * (rv.PULLER_NOTCH_R0 + ext_x / 2.0),
             rv.PED_Y1 - rv.PULLER_NOTCH_DEPTH / 2.0 + 0.025,
             rv.AXIS_Z),
        ))
    return step._diff(body, *notches)


def make_chassis_top_rigid() -> object:
    """Hex frame sheet + six bearing-pocket bosses + hatch opening +
    six under-sheet insert bosses for the hatch screws."""
    solids = [_hex_prism(rv.APOTHEM, rv.SHEET_Z0, rv.SHEET_Z1)]
    cuts = []
    for _i, edge, _R, _R3 in hp._leg_chassis_frames():
        x, y = float(edge[0]), float(edge[1])
        solids.append(_cyl_z(rv.RING_OD / 2.0, rv.RING_BOT_W, rv.SHEET_Z1,
                             x, y))
        cuts.append(_cyl_z(rv.POCKET_BORE / 2.0, rv.RING_BOT_W - 1.0,
                           rv.SHEET_Z0, x, y))
        cuts.append(_cyl_z(rv.POCKET_BORE / 2.0 + rv.POCKET_LEADIN,
                           rv.RING_BOT_W - 1.0,
                           rv.RING_BOT_W + rv.POCKET_LEADIN, x, y))
        # Shoulder bore stops BR_ROOF_T short of the deck: the remaining
        # disc is the integral dust roof over the bearing (user, Aug 26).
        cuts.append(_cyl_z(rv.SHOULDER_OD / 2.0, rv.SHEET_Z0 - 0.01,
                           rv.SHEET_Z1 - rv.BR_ROOF_T, x, y))
    for x, y in rv._access_hole_xy():
        cuts.append(_cyl_z(rv.ACCESS_HOLE_D / 2.0, rv.SHEET_Z0 - 1.0,
                           rv.SHEET_Z1 + 1.0, x, y))
    # With the pillars deleted (user, Aug 26) the hatch screws thread
    # into the FRAME itself: a boss hangs under the sheet at each screw,
    # the M3 heat-set insert installs from below (screw tension then
    # pulls the insert AGAINST the sheet, compression -- the strong
    # direction), with the melt-relief counterbore on the open bottom
    # face.  The hatch-opening cutter runs down past the bosses so any
    # sliver poking inside the opening is trimmed flush at the wall,
    # preserving the lid lip's drop-in clearance.
    for x, y in rv._hatch_screw_xy():
        solids.append(_cyl_z(rv.FRAME_BOSS_OD / 2.0,
                             rv.SHEET_Z0 - rv.FRAME_BOSS_H,
                             rv.SHEET_Z0 + 1.0, x, y))
        cuts.append(_cyl_z(rv.INSERT_BORE_OD / 2.0,
                           rv.SHEET_Z0 - rv.FRAME_BOSS_H - 0.01,
                           rv.SHEET_Z0, x, y))
        cuts.append(_cyl_z(rv.INSERT_RELIEF_OD / 2.0,
                           rv.SHEET_Z0 - rv.FRAME_BOSS_H - 0.01,
                           rv.SHEET_Z0 - rv.FRAME_BOSS_H
                           + rv.INSERT_RELIEF_DEPTH, x, y))
        cuts.append(_cyl_z(rv.HOLE_D / 2.0, rv.SHEET_Z0 - 1.0,
                           rv.SHEET_Z1 + 1.0, x, y))
    cuts.append(_hex_prism(rv.HATCH_OPEN_APO,
                           rv.SHEET_Z0 - rv.FRAME_BOSS_H - 1.0,
                           rv.SHEET_Z1 + 1.0, flats_at_rings=True))
    return step._diff(step._union(*solids), *cuts)


def make_top_hatch_rigid() -> object:
    """Removable hex service lid with registration lip and screw ears."""
    lid = _hex_prism(rv.HATCH_APO, rv.SHEET_Z1, rv.SHEET_Z1 + rv.PLATE_T,
                     flats_at_rings=True)
    lip = step._diff(
        _hex_prism(rv.HATCH_OPEN_APO - rv.HATCH_LIP_CL,
                   rv.SHEET_Z1 - rv.HATCH_LIP_H, rv.SHEET_Z1 + 0.1,
                   flats_at_rings=True),
        _hex_prism(rv.HATCH_OPEN_APO - rv.HATCH_LIP_CL - rv.HATCH_LIP_W,
                   rv.SHEET_Z1 - rv.HATCH_LIP_H - 1.0, rv.SHEET_Z1 + 1.0,
                   flats_at_rings=True),
    )
    ears = [_cyl_z(rv.HATCH_EAR_OD / 2.0, rv.SHEET_Z1,
                   rv.SHEET_Z1 + rv.PLATE_T, x, y)
            for (x, y) in rv._hatch_screw_xy()]
    body = step._union(lid, lip, *ears)
    cuts = []
    for (x, y) in list(rv._hatch_screw_xy()) \
            + [(float(x), float(y)) for (x, y) in hp.CHASSIS_STANDOFF_HOLES_XY] \
            + [(float(x), float(y)) for (x, y) in hp.ELEC_CHASSIS_MOUNT_HOLES_XY]:
        cuts.append(_cyl_z(rv.HOLE_D / 2.0, rv.SHEET_Z1 - 1.0,
                           rv.SHEET_Z1 + rv.PLATE_T + 1.0, x, y))
    cuts.append(_cyl_z(rv.CENTRE_HOLE_D / 2.0, rv.SHEET_Z1 - 1.0,
                       rv.SHEET_Z1 + rv.PLATE_T + 1.0))
    return step._diff(body, *cuts)


def make_centre_wago_block() -> object:
    """Central 4-bay Wago 221-415 splice block."""
    z0 = rv.BOT_SHEET_TOP_Z
    zf = z0 + rv.WBLK_FLOOR_T
    top = zf + rv.WBLK_WALL_H
    body = _box((2.0 * rv.WBLK_HALF_X, 2.0 * rv.WBLK_HALF_Y, top - z0),
                (0.0, 0.0, (z0 + top) / 2.0))
    x_c = rv.WBLK_WALL_T / 2.0 + rv.WBLK_BAY_W / 2.0
    cuts = []
    for sy in (+1.0, -1.0):
        for sx in (-1.0, +1.0):
            y0 = rv.WBLK_WALL_T / 2.0
            y1 = rv.WBLK_HALF_Y + 2.0
            cuts.append(_box(
                (rv.WBLK_BAY_W, y1 - y0, rv.WBLK_WALL_H + 2.0),
                (sx * x_c, sy * (y0 + y1) / 2.0,
                 zf + (rv.WBLK_WALL_H + 2.0) / 2.0),
            ))
    return step._diff(body, *cuts)


def make_coxa_link_rigid() -> object:
    """Production coxa re-assembled with the SHORTENED hub column: the hub sub-solid is truncated at
    rv.HUB_TRIM_Z with the dust-lip skirt / platform disc deleted, the
    slab + cradle sub-solid drops rv.COL_DROP as one rigid body, the
    seat ring + Phi 38 brim are added, the horn-screw shafts are re-cut
    from the VARIANT seat planes (rv.HORN_HEAD_SEAT_Z /
    rv.HORN_CENTRE_SEAT_Z -- 10 mm deeper, tracking the M3x30 -> M3x20
    swap so the tip planes / horn engagement never move; shank
    clearances re-opened through the dropped slab), and the result is
    trimmed to the rotation envelope."""
    hub = step.make_coxa_yaw_hub(one_piece=True)
    skirt_cut = step._diff(
        _cyl_z(hp.YAW_HUB_DUST_LIP_OD / 2.0 + 2.0, rv.SLAB_BOT_Z - 1.5,
               hp.YAW_HUB_BOSS_TOP_Z + 1.0),
        _cyl_z(20.0, rv.SLAB_BOT_Z - 2.5, hp.YAW_HUB_BOSS_TOP_Z + 2.0),
    )
    hub = step._diff(hub, skirt_cut)
    bb = hub.bounding_box()
    hub = step._intersect(hub, _cyl_z(60.0, bb.min.Z - 1.0, rv.HUB_TRIM_Z))
    bracket = (Pos(0.0, 0.0, -rv.COL_DROP)
               * step.make_coxa_hip_bracket(one_piece=True))
    ring = step._diff(
        _cyl_z(rv.HUB_RING_OD / 2.0, rv.HUB_RING_Z0, rv.HUB_RING_Z1),
        _cyl_z(rv.HUB_RING_ID / 2.0, rv.HUB_RING_Z0 - 1.0,
               rv.HUB_RING_Z1 + 1.0),
    )
    brim = step._diff(
        _cyl_z(rv.BRIM_OD / 2.0, rv.BRIM_BOT_Z, rv.BRIM_TOP_Z),
        _cyl_z(rv.HUB_RING_ID / 2.0, rv.BRIM_BOT_Z - 1.0,
               rv.BRIM_TOP_Z + 1.0),
    )
    body = step._union(hub, bracket, ring, brim)
    hip_ax_x, _, hip_ax_z = rv.COXA_HIP_ANCHOR_V
    cuts = [_cyl_y(16.75, ylo, yhi, x=hip_ax_x, z=hip_ax_z)
            for (ylo, yhi) in ((21.75, 30.0), (-31.0, -24.75))]
    shaft_top_z = 80.0
    drive_clear = hp.DISC_HORN_BOLT_OD + 0.3
    stations = [(0.0, 0.0, rv.HORN_CENTRE_SEAT_Z,
                 hp.HORN_CENTRE_OD)]
    r = hp.DISC_HORN_BOLT_PCD / 2.0
    stations.extend(
        (r * math.cos(t), r * math.sin(t), rv.HORN_HEAD_SEAT_Z,
         drive_clear)
        for t in hp.DISC_HORN_BOLT_ANGLES_RAD)
    for sx, sy, seat_z, shank_d in stations:
        cuts.append(_cyl_z(hp.YAW_HUB_HORN_HEAD_CB_OD / 2.0, seat_z,
                           shaft_top_z, sx, sy))
        cuts.append(_cyl_z(shank_d / 2.0, rv.SLAB_BOT_Z - 1.0,
                           seat_z + 0.5, sx, sy))
    body = step._diff(body, *cuts)
    bb = body.bounding_box()
    keep = _cyl_z(rv.ROT_ENVELOPE_R, bb.min.Z - 1.0, bb.max.Z + 1.0)
    return step._intersect(body, keep)


def make_chassis_bottom_rigid() -> object:
    """Production chassis bottom + corner trim to the tower cylinder, all
    three dead-ear shaves (az 210 flush to the deck top),
    wago-tray deletes, the per-leg wire-corridor + cradle-shell
    flatten (rv.CHB_FLAT_* box minus the tower keep cylinder -- Aug 24
    rev 5 + Aug 25 rev 6), the Aug 25 LOWERED bearing
    pocket: the old tower band above the new deck-level seat plane is
    cut away (leaving the 0.5 mm-proud Phi 34/Phi 37.15 seat ledge)
    and a fresh full-wrap Phi 44/Phi 37.15 ring is unioned from the
    deck band to the new race top, plus the Aug 25 rev-7 tower-flank
    bump shave (the production swing-relief protect ring, r 23.5, is
    cut back to the trim cylinder so the tower outer profile is one
    vertical cylinder sheet-top -> rim), plus the Aug 25 rev-8
    ABOVE-SHEET WHITELIST cut: everything above the bare sheet top
    outside the six tower cylinders goes, at every azimuth."""
    cb = step.make_chassis_bottom()
    ear_r = hp.YAW_CAP_BOLT_PCD / 2.0
    cutters = []
    for i in range(6):
        deg = math.degrees((i + 0.5) * math.pi / 3.0)
        box = _box((23.5, 45.0, rv.CHB_PLATE_TOP + 6.5),
                   (rv.APOTHEM + 23.5 / 2.0, 0.0,
                    (rv.CHB_PLATE_TOP - 6.5) / 2.0))
        corner = step._diff(box, _cyl_z(rv.CHB_TRIM_R, -8.0, 7.5,
                                        x=rv.APOTHEM))
        keep = _cyl_z(rv.CHB_KEEP_R, rv.CHB_PLATE_TOP - 1.0, 21.6,
                      x=rv.APOTHEM)
        ear330 = step._diff(
            _cyl_z(rv.CHB_EAR_R, rv.CHB_PLATE_TOP, 20.6,
                   x=rv.APOTHEM + ear_r * math.cos(-math.pi / 6.0),
                   y=ear_r * math.sin(-math.pi / 6.0)),
            keep,
        )
        box90 = step._diff(
            _box((12.0, 10.0, 20.6 - rv.CHB_PLATE_TOP),
                 (rv.APOTHEM, rv.CHB_WALL_FACE_Y + 5.0,
                  (rv.CHB_PLATE_TOP + 20.6) / 2.0)),
            keep,
        )
        ear210 = step._diff(          # inboard ear, flush to the deck top
            _cyl_z(rv.CHB_EAR_R, rv.CHB_DECK_TOP, 20.6,
                   x=rv.APOTHEM + ear_r * math.cos(math.pi * 7.0 / 6.0),
                   y=ear_r * math.sin(math.pi * 7.0 / 6.0)),
            keep,
        )
        flat = step._diff(            # wire-corridor + cradle-shell flatten
            _box(                     # (Aug 24 rev 5 + Aug 25 rev 6): box to
                (rv.CHB_FLAT_X1 - rv.CHB_FLAT_X0,     # the yaw axis MINUS the
                 2.0 * rv.CHB_FLAT_HALF_Y,            # tower keep cylinder
                 rv.CHB_FLAT_Z1 - rv.CHB_FLAT_Z0),    # (in-keep shell carries
                ((rv.CHB_FLAT_X0 + rv.CHB_FLAT_X1) / 2.0, 0.0,  # the 6805
                 (rv.CHB_FLAT_Z0 + rv.CHB_FLAT_Z1) / 2.0)),     # seat's
            _cyl_z(rv.CHB_KEEP_R,                     # inboard arc)
                   rv.CHB_FLAT_Z0 - 0.5, rv.CHB_FLAT_Z1 + 0.5,
                   x=rv.APOTHEM),
        )
        bump = step._diff(            # tower-flank bump shave (Aug 25 rev 7):
            _box((23.5, 45.0,         # production's swing-relief protect ring
                  (rv.CHB_DECK_TOP + 0.25) - (rv.CHB_PLATE_TOP - 0.25)),
                 (rv.APOTHEM + 23.5 / 2.0, 0.0,   # (r 23.5) bulged the outboard
                  ((rv.CHB_PLATE_TOP - 0.25)      # flank in the mount-plate
                   + (rv.CHB_DECK_TOP + 0.25)) / 2.0)),  # band -- shave flush
            _cyl_z(rv.CHB_TRIM_R, rv.CHB_PLATE_TOP - 1.0,
                   rv.CHB_DECK_TOP + 1.0, x=rv.APOTHEM),
        )
        band = _cyl_z(rv.CHB_TOWER_R + 0.1,   # tower band rebuild (Aug 25):
                      rv.CHB_SEAT_W,          # everything above the NEW seat
                      rv.CHB_RIM_OLD_W + 1.5, # plane goes; the 0.5 mm Phi 34
                      x=rv.APOTHEM)           # band left below is the ledge
        cutters.extend(Rotation(0, 0, deg) * c
                       for c in (corner, ear330, box90, ear210, flat, bump,
                                 band))
    for M in hp.wago_tray_corner_transforms():   # wago tray delete (Aug 24)
        deg = math.degrees(math.atan2(M[1, 0], M[0, 0]))
        cutters.append(
            Pos(M[0, 3], M[1, 3], M[2, 3])
            * Rotation(0, 0, deg)
            * _box((2.0 * rv.TRAY_HALF_X + 0.4, 2.0 * rv.TRAY_HALF_Y + 0.4,
                    hp.WAGO_MOUNT_WALL_H + 1.0),
                   (0.0, 0.0, (hp.WAGO_MOUNT_WALL_H + 1.0) / 2.0)))
    wl_keeps = [                  # rev-8 ABOVE-SHEET WHITELIST cut (Aug 25):
        _cyl_z(rv.CHB_WL_KEEP_R,  # one global box from the sheet top up,
               rv.CHB_FLAT_Z0 - 1.0, rv.CHB_WL_Z1 + 1.0,   # minus the six
               x=rv.APOTHEM * math.cos((i + 0.5) * math.pi / 3.0),
               y=rv.APOTHEM * math.sin((i + 0.5) * math.pi / 3.0))
        for i in range(6)]        # tower cylinders -- EVERYTHING else above
    cutters.append(step._diff(    # z 2 goes to the sheet.  Historically
        _box((400.0, 400.0,       # this is what killed the L-stubs the
              rv.CHB_WL_Z1 - rv.CHB_FLAT_Z0),  # blacklist passes missed:
             (0.0, 0.0, (rv.CHB_FLAT_Z0 + rv.CHB_WL_Z1) / 2.0)),  # the base
        *wl_keeps))               # STEP chassis used to model the RETIRED
    # two-bay WAGO3 corner trays, 4.8 mm wider than the WAGO5 tray the
    # tray-delete cutter above is sized for, so their side walls survived
    # at all six corners (the circled diagonal slashes / L-brackets).
    # Fixed Aug 26 (the base port now builds production's WAGO5 tray, so
    # the tray-delete alone clears them) -- the whitelist stays as the
    # catch-all guarantee.  The rims are unioned after, so the towers are
    # untouched.
    body = step._diff(cb, *cutters)
    rims = []                    # rebuilt full-wrap tower rings: Phi 44 /
    for i in range(6):           # Phi 37.15, deck band to the race top
        deg = math.degrees((i + 0.5) * math.pi / 3.0)
        rim = step._diff(
            _cyl_z(rv.CHB_TOWER_R, rv.CHB_SEAT_W - 1.0, rv.CHB_RIM_W,
                   x=rv.APOTHEM),
            _cyl_z(rv.POCKET_BORE / 2.0, rv.CHB_SEAT_W - 2.0,
                   rv.CHB_RIM_W + 1.0, x=rv.APOTHEM))
        rims.append(Rotation(0, 0, deg) * rim)
    return step._union(body, *rims)


# --- specs, checks, main -----------------------------------------------------

def rigid_hip_part_specs() -> list[StepPart]:
    # legacy_stl=None everywhere: the mesh-pipeline twins are RETIRED --
    # this BREP is the only geometry source, there is nothing to diff
    # against.  Part-level invariants live in _sanity_problems below;
    # assembly-level checks live in make_rigid_hip_variant.py.
    return [
        StepPart(
            "hip_clamp_cap_rigid",
            make_hip_clamp_cap_rigid,
            None,
            "Stock clamp cap grown a yaw-axis pedestal and 6805 press boss.",
        ),
        StepPart(
            "chassis_top_rigid",
            make_chassis_top_rigid,
            None,
            "Top frame: hex sheet, six bearing pockets, service-hatch opening.",
        ),
        StepPart(
            "top_hatch_rigid",
            make_top_hatch_rigid,
            None,
            "Removable service lid with registration lip and screw ears.",
        ),
        StepPart(
            "centre_wago_block",
            make_centre_wago_block,
            None,
            "Central 4-bay Wago splice block under the hatch.",
        ),
        StepPart(
            "coxa_link_rigid",
            make_coxa_link_rigid,
            None,
            "Production coxa + hub seat ring + dust brim, envelope-rounded (6x).",
        ),
        StepPart(
            "chassis_bottom_rigid",
            make_chassis_bottom_rigid,
            None,
            "Production chassis bottom with tower-cylinder corners and the "
            "lowered deck-level bearing pocket.",
        ),
    ]


def _sanity_problems(rows: list[dict]) -> list[str]:
    """Part-level gates that don't need the assembled robot.

    Every derived STL must HEAL into a closed volume with the exact
    hp._heal_for_export pass the assembly driver applies before anything
    reaches the print set (OpenCascade tessellation occasionally emits
    float32-STL-grid slivers -- the coxa welds 2 non-manifold edges --
    which that pass dissolves via manifold3d.simplify; hence manifold3d
    in this exporter's uv --with set).  A mesh that STILL is not a
    volume after healing has a genuine crack and would poison every
    downstream boolean/probe.  chassis_bottom_rigid additionally gets
    the rev-8 ABOVE-SHEET WHITELIST census
    (rv.chassis_whitelist_violations): nothing may stand above the bare
    sheet outside the six tower cylinders.  The full geometric suite
    (fits, clearances, sweeps) runs in make_rigid_hip_variant.py on the
    assembled meshes."""
    import trimesh

    problems = []
    for row in rows:
        raw = trimesh.load(STEP_OUT_DIR / row["stl"], process=True)
        mesh = hp._heal_for_export(raw)
        if not mesh.is_volume:
            problems.append(
                f"{row['name']}: derived STL does not heal into a closed "
                "volume (genuine tessellation crack)")
        if row["name"] == "chassis_bottom_rigid":
            n_bad, worst_r = rv.chassis_whitelist_violations(mesh)
            if n_bad:
                problems.append(
                    f"{row['name']}: {n_bad} vertices above the sheet "
                    f"outside the tower whitelist (worst r {worst_r:.2f})"
                )
    return problems


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()

    exported = export_all(rigid_hip_part_specs(), out_dir=STEP_OUT_DIR,
                          step_dir=STEP_OUT_DIR,
                          stl_dir=STEP_OUT_DIR / "stl")
    problems = _sanity_problems(exported)
    manifest = {
        "units": "mm",
        "source": (
            "build123d/OpenCascade BREP (CANONICAL geometry source), "
            "constants imported from "
            "concepts/rigid_hip/make_rigid_hip_variant.py"
        ),
        "exported_parts": exported,
        "checks": {
            "passed": not problems,
            "problems": problems,
        },
        "files": [rel for row in exported for rel in (row["step"], row["stl"])],
    }
    manifest_path = STEP_OUT_DIR / "rigid_hip_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    bundle = write_bundle(manifest, "rigid_hip_step_first_bundle.zip",
                          "rigid_hip_manifest.json", out_dir=STEP_OUT_DIR)
    print(f"wrote {manifest_path.relative_to(PROTO_DIR)}")
    print(f"wrote {bundle.relative_to(PROTO_DIR)}")
    if problems:
        print("rigid-hip STEP part checks failed:")
        for problem in problems:
            print(f"  - {problem}")
        raise SystemExit(1)
    print("rigid-hip BREP export complete; run make_rigid_hip_variant.py "
          "for the assembly checks + scene.")


if __name__ == "__main__":
    main()
