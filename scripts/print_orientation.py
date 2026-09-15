"""Canonical print poses for every printed part.

PART_REGISTRY maps each printable STL to its builder in
`hexapod_prototype.py` and the re-orientation that puts it in its print
pose (broad flat face on the build plate, hollow pockets opening toward
+Z, z_min = 0 so the part rests on the bed).

This is the single source of truth for print orientation:

  * the verifier's flat-bottom printability guard (`check_flat_bottom`
    in `_verify_prototype.py`) sweeps the registry and fails any part
    whose print pose leaves a support-needing overhang;
  * `scripts/_flatbottom_check.py` uses it for the standalone audit;
  * `design_spec.yaml`'s `print_orientations` section mirrors the
    `_reorient_*` rotations for documentation.

(History: this module started as `prepare_xometry_upload.py`, which also
wrote an upload bundle for ordering prints from Xometry.  The bundle
generation was removed in Aug 2026 — parts are printed at home from
`stl_prototype/` — but the print-pose registry stayed load-bearing.)
"""

from __future__ import annotations

import os
import sys
from typing import Callable

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.dirname(SCRIPT_DIR)
if PROTO_DIR not in sys.path:
    sys.path.insert(0, PROTO_DIR)

from hexapod_prototype import (  # noqa: E402
    make_chassis_bottom,
    make_chassis_top,
    make_coxa_link_part,
    make_femur_link_part,
    make_foot_boot,
    make_servo_clamp_cap,
    make_tibia_knee_yoke,
    make_yaw_bearing_cap,
    make_yaw_servo_retainer,
)


# ---------------------------------------------------------------------------
# Re-orientation helpers
# ---------------------------------------------------------------------------

def _rotate(mesh: trimesh.Trimesh, angle_rad: float,
            axis: tuple[float, float, float]) -> trimesh.Trimesh:
    out = mesh.copy()
    out.apply_transform(rotation_matrix(angle_rad, axis))
    return out


def _drop_to_bed(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Translate so z_min = 0 and the part is centred over (x=0, y=0)."""
    out = mesh.copy()
    cx, cy = (out.bounds[0, :2] + out.bounds[1, :2]) / 2.0
    out.apply_translation([-cx, -cy, -float(out.bounds[0, 2])])
    return out


# ---------------------------------------------------------------------------
# Per-part orientation rules
# ---------------------------------------------------------------------------
# Rule of thumb for FDM: hollow pockets must open toward +Z, broadest flat
# face goes on the bed.

def _reorient_chassis_plate(mesh):
    """Hex plate — already flat. Just drop to z=0.

    Jun 2026 single-part merge: ``chassis_bottom`` is the whole bottom plate
    (flat plate + bearing tower + the folded-in flat floor slab).  Its broad
    flat -6 mm underside is the bed face, so it prints face-DOWN exactly as
    built (bearing tower up) — drop to bed, no flip."""
    return _drop_to_bed(mesh)


def _reorient_coxa_link(mesh):
    """One-piece coxa (Aug 2026 merge of the yaw turntable hub + hip
    bracket).  Lay it on its SIDE (rotate -90 deg about Y: the yaw axis
    goes horizontal, the cradle's outboard end wall becomes the bed face)
    -- the flattest stable pose; both the boss-down and cradle-down
    uprights leave 20+ mm of overhang hanging below the main plane."""
    out = _rotate(mesh, -np.pi / 2, [0, 1, 0])
    return _drop_to_bed(out)


def _lay_flat(mesh):
    """Generic FDM reorient for the bearing-sandwich leg socket fittings
    (hip yoke, knee bracket, knee yoke, foot fitting).  Picks whichever
    of the three axis-aligned orientations leaves the SHORTEST Z extent
    so the part lies on its broadest face (largest bed contact, lowest
    centre of gravity, least support).  The CF-tube socket bore ends up
    roughly horizontal -- the user can spin individual parts in Bambu
    Studio if a particular bore would print better vertically."""
    best = None
    for axis, angle in ((None, 0.0), ((1, 0, 0), np.pi / 2), ((0, 1, 0), np.pi / 2)):
        out = mesh.copy()
        if axis is not None:
            out.apply_transform(rotation_matrix(angle, axis))
        z_extent = float(out.extents[2])
        if best is None or z_extent < best[0]:
            best = (z_extent, out)
    return _drop_to_bed(best[1])


def _reorient_servo_clamp_cap(mesh):
    """servo_clamp_cap: FLANGE-DOWN (Aug 19 2026).  ``_lay_flat`` used to
    pick +90 about X (shortest Z), but that lands the TONGUE side on the
    bed -- fine when the tongue tip was the lowest feature, broken once the
    back-face hook grew: its shelf reaches 6.4 mm below the tongue tip,
    leaving ~1% bed contact (flat-bottom guard rightly rejects it).
    Rotate -90 about X instead so the flange's broad OUTER (+Y) face is
    the bed; the whole hook then prints as a straight vertical column at
    the top, no supports, and the M3 head counterbores open at the bed."""
    out = _rotate(mesh, -np.pi / 2, [1, 0, 0])
    return _drop_to_bed(out)


def _reorient_foot_boot(mesh):
    """Foot boot: rotate -90 deg about Y so the MOUTH face (local -X)
    becomes the bed face and the dome tip points straight up (Aug 19
    2026 dome tip -- a dome cannot be the bed face; the bore's blind
    end is a 45-deg internal cone so nothing bridges).  No supports."""
    out = _rotate(mesh, -np.pi / 2, [0, 1, 0])
    return _drop_to_bed(out)


def _reorient_yaw_servo_retainer(mesh):
    """yaw_servo_retainer: built hanging under the chassis (flange on top,
    four corner ground poles below).  Flip 180 deg about X so the broad
    FLANGE plane is the bed and the poles print as clean vertical columns
    ending in their small pads.  Feet-down would rest the part on four
    Ø12 pads with the backstop floor floating ~10 mm above the bed (the
    flat-bottom guard rightly rejects that); flange-down nothing hangs
    below the bed plane.  MJF needs no supports for the floor bridging
    over the wall cavity; on FDM it is a ~21 mm bridge, same as every
    prior foot revision."""
    out = mesh.copy()
    out.apply_transform(rotation_matrix(np.pi, (1, 0, 0)))
    return _drop_to_bed(out)


def _reorient_yaw_bearing_cap(mesh):
    """yaw_bearing_cap: built open-pocket-UP in coxa-local (z[-1,+6]).  Flip
    180 deg about X so the broad Phi 44 top-rim annulus lies FLAT on the bed
    (the clean Phi 37 through-bore opens straight up); the 3 join-bolt ear
    lugs point UP.  No internal overhang -- the bore is a clean through-hole,
    so no supports are needed."""
    out = mesh.copy()
    out.apply_transform(rotation_matrix(np.pi, (1, 0, 0)))
    return _drop_to_bed(out)


# ---------------------------------------------------------------------------
# Part registry
# ---------------------------------------------------------------------------
# Each entry: (filename, make-fn, reorient-fn, qty).  Filenames match
# stl_prototype/; qty is per full robot.
PART_REGISTRY: list[tuple[str,
                          Callable[[], trimesh.Trimesh],
                          Callable[[trimesh.Trimesh], trimesh.Trimesh],
                          int]] = [
    ("chassis_top.stl",        make_chassis_top,        _reorient_chassis_plate, 1),
    ("chassis_bottom.stl",     make_chassis_bottom,     _reorient_chassis_plate, 1),
    ("coxa_link.stl",          make_coxa_link_part,     _reorient_coxa_link,     6),
    ("femur_link.stl",         make_femur_link_part,    _lay_flat,               6),
    ("tibia_knee_yoke.stl",    make_tibia_knee_yoke,    _lay_flat,               6),
    ("foot_boot.stl",          make_foot_boot,          _reorient_foot_boot,     6),
    ("yaw_servo_retainer.stl", make_yaw_servo_retainer, _reorient_yaw_servo_retainer, 6),
    ("yaw_bearing_cap.stl",    make_yaw_bearing_cap,    _reorient_yaw_bearing_cap, 6),
    # wago_mount.stl RETIRED late-Aug 2026 (tray walls integrated into
    # chassis_bottom's top face -- prints with the chassis, no supports).
    ("servo_clamp_cap.stl",    make_servo_clamp_cap,    _reorient_servo_clamp_cap, 12),
]
