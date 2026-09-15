#!/usr/bin/env python3
"""Per-joint wire-harness reach model for the 18-servo hexapod prototype.

Jun 2026 STS3215 serial-bus refit.  The robot is 18x FEETECH STS3215
**serial-bus** servos on a single half-duplex TTL bus driven directly
by an **Arduino Uno Q** (no PCA9685, no Raspberry Pi, no FE-URT-1 /
Waveshare bus adapter).  Each leg's three servos are **daisy-chained
with stock FEETECH 3-pin bus cables**; the leg's bundle drops through
that leg's ``chassis_bottom`` drop slot into the inter-plate volume,
where it meets the leg's **power branch from the distributed bus bar**
(V+/GND) and the **leg-to-leg data jumper** (signal + GND only).  The
full, buildable wiring plan — the DATA daisy-chain plus the
DISTRIBUTED-POWER bus-bar / per-leg branch harness that keeps any one
Molex 5264 pin under its ~3 A rating — lives in
``../firmware/WIRING.md`` (§6).  See also ``../docs/BOM.md``.

This module is the SOURCE OF TRUTH for the per-joint **cable reach
budget**: "starting from each servo's cradle wire-exit, how long a
bus lead does it take to reach the electronics-tray bus landing,
routing through that leg's drop slot?"  It reads the parametric CAD
geometry (``hexapod_prototype``) and the chassis-frame leg layout
(``hexapod_prototype._leg_chassis_frames``) to generate one
``WIRE_HARNESS_PLAN`` entry per servo:

  * ``joint_idx`` (0 .. 17): canonical joint index.
    ``joint = leg * 3 + axis`` with ``axis in {0=yaw, 1=hip_pitch,
    2=knee}``.
  * ``leg_idx`` (0 .. 5): hex-leg index (0 = +X half north-east,
    going CCW; matches ``_leg_chassis_frames()``).
  * ``axis`` (string): "yaw" / "hip_pitch" / "knee".
  * ``servo_id`` (int 2 .. 19): the bus address assigned to this
    joint (``hexapod_core.joint_frame.servo_id(joint_idx)``; set once with
    ``feetech_bus.py setid``).  ID 1 is left free (factory default)
    so a fresh servo can join a live daisy-chain.  Replaces the
    retired PWM ``pca_board`` / ``pca_channel`` pair — on the serial
    bus a joint is addressed by ID, not by a driver-board channel.
  * ``source_xyz_chassis``: chassis-frame XYZ of the cradle
    wire-exit's mouth (where the harness physically exits the
    servo well).  As of the May 2026 chassis_bottom-integrated
    yaw cradle redesign the YAW cradle's exit is on the cradle's
    -X (radially-inward) face; the legacy ``coxa_bracket`` had
    it on +X.  HIP-PITCH and KNEE cradles still exit on the
    link's +X face, but for path-length estimation we still
    model all three cradles' sources as the YAW exit mouth (see
    ``_cradle_source_bracket_xyz``).
  * ``destination_xyz_chassis``: chassis-frame XYZ of this leg's
    landing post on the electronics-tray **power bus bar + Uno Q
    UART tap** (see ``_bus_landing_chassis_xyz``).  All three
    servos in a leg share that leg's branch terminal, so the
    destination depends only on ``leg_idx``; the per-axis
    difference in reach comes from the slack budget below.
  * ``via_chassis_drop_xyz``: chassis-frame XYZ of the leg's
    drop slot through ``chassis_bottom``
    (= bracket_frame(LEG_HARNESS_DROP_X_CENTRE, 0, 0)).
  * ``path_length_mm_min``: minimum bus-lead length needed to
    span source -> drop -> bus landing, computed as a Manhattan
    distance plus a +30 mm slack-loop budget per joint axis
    the harness physically crosses (see SLACK_BUDGET below).
    Manhattan rather than Euclidean because the harness has
    to follow the drop slot's radial corridor and the tray's
    surface rather than fly diagonally through the chassis.
    NB: the BuildViz scene's ``routes[]`` (emitted by
    ``tools/full_robot_viz_build.py``) draw the AS-BUILT
    daisy-chain topology of WIRING.md section 6.2 — **as-built**:
    DATA enters each leg at the **yaw** then runs yaw→hip→knee;
    POWER injects at each **hip** and tees to yaw + knee (Jul/Aug
    2026), while this star model -- each joint's own lead spanning
    source -> drop -> landing -- deliberately stays the
    conservative Manhattan reach budget: a chain-segment
    (Euclidean) length would make the reach check optimistic
    about the cable the builder actually needs.
  * ``extension_required``: human-readable string describing
    the cable build for this joint, computed from
    ``path_length_mm_min`` minus the stock bus-lead length and
    rounded up to the next 30 cm extension cable.

Stock bus lead + extension assumption:

    STS3215 stock bus lead = STOCK_PIGTAIL_MM (declared below).
    Each STS3215 ships with one 3-pin serial-bus cable, and
    docs/BOM.md calls for a spare pack of "FEETECH 3-pin serial-bus
    servo cables".  We treat the stock lead as
    300 mm = STOCK_PIGTAIL_MM (the longest assorted length), the
    reach available before a leg needs an extra/longer cable.

    Extension cables are 30 cm, 3-pin serial-bus extensions.
    Each adds EXTENSION_LENGTH_MM = 300 mm of reach when wired
    end-to-end with the stock lead.

    extension_required:
      n_ext = max(0, ceil((path_length_mm_min - STOCK_PIGTAIL_MM)
                          / EXTENSION_LENGTH_MM))
      0 -> "STS3215 stock bus lead"
      1 -> "+ 30 cm extension"
      n -> "+ N x 30 cm extensions"

The leg azimuth + chassis-frame transforms come from
``hexapod_prototype._leg_chassis_frames()`` -- DO NOT re-derive
the leg layout here; a future change to leg count / azimuth
must happen in one place.

CLI
---

    uv run python -m motor_setup.wire_harness_plan
        -> prints the 18-row markdown table to stdout (the BOM)

    from motor_setup.wire_harness_plan \\
        import WIRE_HARNESS_PLAN, print_harness_plan
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path
from typing import TypedDict

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
_PROTOTYPE_DIR = _THIS_DIR.parent
if str(_PROTOTYPE_DIR) not in sys.path:
    sys.path.insert(0, str(_PROTOTYPE_DIR))

import hexapod_prototype as hp  # noqa: E402
from hexapod_core.joint_frame import servo_id  # noqa: E402


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

STOCK_PIGTAIL_MM = 300.0
"""mm.  STS3215 stock 3-pin serial-bus lead.  See the module
docstring for the docs/BOM.md citation; treated as 300 mm,
the reach available before a leg needs an extra/longer cable."""

EXTENSION_LENGTH_MM = 300.0
"""mm.  Standard 30 cm 3-pin serial-bus extension; matches the
"Bus servo cables" line in docs/BOM.md."""

SLACK_PER_JOINT_CROSSING_MM = 30.0
"""mm of slack loop the harness needs at every joint axis it
physically crosses (so the wire bend radius stays > the servo
wire's minimum bend radius across the full joint sweep)."""

# Electronics-tray serial-bus landing (Jun 2026 STS3215 refit).
# The retired PWM star fanned each servo's own extension cable
# back to one of two PCA9685 boards in the tray's +X strip.  On
# the serial bus there is no driver board: each leg's bundle lands
# at that leg's branch terminal on the POWER BUS BAR + the Uno Q
# UART tap, which sit in the SAME +X strip the two PCA9685s used
# to occupy (tray-local x ~= +64 mm, ~3.5 mm inside the tray +X
# edge).  We model the bus bar with one landing post per leg,
# fanned along tray-Y at BUS_BAR_POST_PITCH_MM so each leg's
# branch is a distinct, geometrically-grounded reach target.
BUS_BAR_TRAY_X = 64.0
"""mm, tray-local.  +X strip the retired PCA9685 boards occupied;
where the power bus bar + Uno Q UART tap now live."""
BUS_BAR_TRAY_Y_CENTRE = 2.5
"""mm, tray-local.  Bus-bar centre, midway between the two retired
PCA centres (PCA1 +22.5 / PCA2 -17.5)."""
BUS_BAR_POST_PITCH_MM = 8.0
"""mm.  Per-leg branch-post spacing along the bus bar (tray-Y).
The six legs' posts fan symmetrically about BUS_BAR_TRAY_Y_CENTRE."""
BUS_LANDING_Z_ABOVE_TRAY_TOP_MM = (hp.ELEC_STANDOFF_H
                                   + 1.6   # PCB / bus-bar base thickness
                                   + 1.5)  # nominal terminal-post tip
"""mm.  Bus-bar branch-post tips sit this far above the tray top
face (tray top face at chassis-z = +CHASSIS_PLATE_T/2 + 3 = +5 mm;
post tips at +5 + 5 + 1.6 + 1.5 = +13.1 mm)."""

CHASSIS_PLATE_TOP_Z = hp.CHASSIS_PLATE_T / 2.0
"""mm.  chassis_bottom top face in chassis-frame Z."""
ELEC_TRAY_TOP_Z = CHASSIS_PLATE_TOP_Z + 3.0
"""mm.  Electronics tray's top face in chassis-frame Z.  3 mm
brass standoff between chassis_bottom and tray bottom."""


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _cradle_source_bracket_xyz(axis: str) -> tuple[float, float, float]:
    """Return the wire-exit 'mouth' position in BRACKET-LOCAL frame
    for one of the three cradle axes.

    As of the May 2026 chassis_bottom-integrated yaw-cradle
    redesign the YAW cradle's wire-exit corridor was mirrored
    to the cradle's -X (radially INWARD) face.  The HIP-PITCH
    and KNEE cradles still exit on the link's +X face -- their
    cradles weren't redesigned -- but for path-length estimation
    we still model all three cradles' sources as the YAW exit
    mouth (= the harness's effective "I'm now in the inter-plate
    volume on the inboard side of the leg" Manhattan node).

    YAW cradle, post-redesign:
      Well-local wire-exit mouth = (-WELL_W/2 - WIRE_SLOT_X_PAST_WALL,
      0, +small) -- i.e. just past the well's -X (inboard) outer
      face, on the centreline, near the cavity floor.
      Transforming that into bracket frame for the YAW cradle:
        bracket-x = body_centre_x + wire_exit_x_well
                  = -SERVO_OUTPUT_X - WELL_W/2 - WIRE_SLOT_X_PAST_WALL
                  ~= -41 mm  (was +21 mm with the legacy +X exit;
                  the new value is INBOARD of the bracket origin)
        bracket-y = 0
        bracket-z = chassis-plate-top + small.  In the new cradle
                    the wire-exit z range is the same well-local
                    band as the legacy bracket, but the cradle
                    floor IS the chassis_bottom top, so the mouth
                    sits roughly at chassis-plate-top + 3 mm of
                    bundle thickness.

    The drop-slot centre is at bracket-x = LEG_HARNESS_DROP_X_CENTRE
    = -47.5 mm, only ~6 mm INBOARD of the new wire-exit mouth -- the
    redesigned cradle shortens the per-leg yaw-cable run by
    ~ |old - new| = |+21 - (-41)| = 62 mm of horizontal Manhattan
    distance compared to the legacy +X bracket exit.  The path-length
    model below picks up this shortening automatically since both
    the source and the destination are computed in chassis frame.

    For the HIP-PITCH and KNEE cradles the well sits in coxa_link
    / femur_link frame at the end of the arm.  We continue to
    model their harness "I'm now at the leg drop slot" position
    as the YAW source for path-length purposes (their physical
    routing follows the parent link inboard to the yaw axis and
    then onto the chassis_bottom top face; the YAW source's chassis
    coords already encode "leg's inboard wire-exit on the chassis"),
    plus a +SLACK_PER_JOINT_CROSSING_MM joint-axis-crossing budget
    each via ``n_crossings()``.  This sidesteps having to derive the
    link-local routing path (which would just add a constant offset
    smaller than the +/-30 mm slack budget anyway) and matches the
    user's "+30 mm slack-loop budget per joint crossing" rule.
    """
    assert axis in ("yaw", "hip_pitch", "knee")
    # Bracket-frame wire-exit mouth, common to all 3 cradles
    # for path-length estimation.  As of May 2026 the YAW cradle
    # exits on the well's -X face (radially inboard); the well-
    # local mouth coords are (-WELL_W/2 - WIRE_SLOT_X_PAST_WALL,
    # 0, +small).
    bx = -hp.SERVO_OUTPUT_X - hp.WELL_W / 2.0 - hp.WIRE_SLOT_X_PAST_WALL
    by = 0.0
    # Bracket-z = chassis-plate-top + ~3 mm of bundle thickness.
    # In the chassis_bottom cradle the cavity floor IS the
    # chassis_bottom top face, so bracket-z = 0 + 3 = +3 mm.
    bz = 3.0
    return (bx, by, bz)


def _drop_slot_bracket_xyz() -> tuple[float, float, float]:
    """Return the drop-slot centre in BRACKET-LOCAL frame.

    Drop slot is centred at bracket
    (LEG_HARNESS_DROP_X_CENTRE, 0, 0).  Bracket-z = 0 puts the
    slot mouth at the chassis plate mid-plane (which is the
    natural "I'm now in the inter-plate volume" Manhattan node
    for the harness)."""
    return (hp.LEG_HARNESS_DROP_X_CENTRE, 0.0, 0.0)


def _bus_landing_chassis_xyz(leg_idx: int
                             ) -> tuple[float, float, float]:
    """Return the chassis-frame XYZ of one leg's landing post on the
    electronics-tray power bus bar + Uno Q UART tap.

    The bus bar lives in tray-local frame; tray-frame -> chassis-
    frame translation is (ELEC_TRAY_CENTRE_X, ELEC_TRAY_CENTRE_Y,
    ELEC_TRAY_TOP_Z + tray-z) where tray-z = 0 at the tray's mid-
    plane.  Post tips sit BUS_LANDING_Z_ABOVE_TRAY_TOP_MM above the
    tray top face.

    Post layout: the six per-leg branch posts fan along the bus
    bar's tray-Y axis at BUS_BAR_POST_PITCH_MM pitch, symmetric
    about BUS_BAR_TRAY_Y_CENTRE, on the bus bar's +X strip
    (tray-local x = BUS_BAR_TRAY_X).  Leg 0's post sits at the -Y
    end, leg 5's at the +Y end -- so each leg's bundle lands at the
    branch terminal nearest the direction it routes inboard from.
    """
    assert 0 <= leg_idx <= 5
    n_legs = 6
    span_y = (n_legs - 1) * BUS_BAR_POST_PITCH_MM
    post_y_tray = (BUS_BAR_TRAY_Y_CENTRE - span_y / 2.0
                   + leg_idx * BUS_BAR_POST_PITCH_MM)
    post_x_tray = BUS_BAR_TRAY_X
    # Tray -> chassis
    px = hp.ELEC_TRAY_CENTRE_X + post_x_tray
    py = hp.ELEC_TRAY_CENTRE_Y + post_y_tray
    pz = ELEC_TRAY_TOP_Z + BUS_LANDING_Z_ABOVE_TRAY_TOP_MM
    return (px, py, pz)


def _bracket_to_chassis(edge_mid: np.ndarray, R3: np.ndarray,
                        bracket_xyz: tuple[float, float, float]
                        ) -> tuple[float, float, float]:
    """Transform bracket-local XYZ to chassis-frame XYZ for one
    leg.  Bracket-z = 0 corresponds to chassis plate-top.
    """
    bx, by, bz = bracket_xyz
    xy = edge_mid + R3 @ np.array([bx, by, 0.0])
    cz = CHASSIS_PLATE_TOP_Z + bz
    return (float(xy[0]), float(xy[1]), cz)


# ---------------------------------------------------------------------------
# Path-length model
# ---------------------------------------------------------------------------

_AXIS_ORDER = ("yaw", "hip_pitch", "knee")
_AXIS_SLACK_CROSSINGS: dict[str, int] = {
    # Yaw cradle is chassis-fixed (integrated into chassis_bottom);
    # no downstream joint between the cradle and the chassis-side
    # bus landing, so 0 slack loops.
    "yaw": 0,
    # Hip-pitch cradle is on coxa_link (rotates with yaw); harness
    # crosses the yaw joint once on its way back to the chassis.
    "hip_pitch": 1,
    # Knee cradle is on femur_link (rotates with hip-pitch, which
    # itself rotates with yaw); harness crosses BOTH the yaw and
    # hip-pitch joints on its way back to the chassis.
    "knee": 2,
}
_AXIS_SLACK_MM: dict[str, float] = {
    axis: _AXIS_SLACK_CROSSINGS[axis] * SLACK_PER_JOINT_CROSSING_MM
    for axis in _AXIS_ORDER
}


def _manhattan(a: tuple[float, float, float],
               b: tuple[float, float, float]) -> float:
    return (abs(a[0] - b[0])
            + abs(a[1] - b[1])
            + abs(a[2] - b[2]))


# ---------------------------------------------------------------------------
# Joint mapping (serial-bus: servo ID = joint + 2; ID 1 left free)
# ---------------------------------------------------------------------------


def joint_idx(leg_idx: int, axis: str) -> int:
    return leg_idx * 3 + _AXIS_ORDER.index(axis)


def joint_to_servo_id(joint: int) -> int:
    """Return the serial-bus address for one joint index.

    ``hexapod_core.joint_frame.servo_id`` (IDs 2..19), matching
    ``motor_setup/feetech_bus.py`` and ``firmware/WIRING.md``.  ID 1 is
    never assigned -- it is the STS3215 factory default, kept free so a
    fresh servo can join a live daisy-chain without colliding.
    """
    return servo_id(joint)


# ---------------------------------------------------------------------------
# WIRE_HARNESS_PLAN construction
# ---------------------------------------------------------------------------


class HarnessEntry(TypedDict, total=False):
    joint_idx: int
    leg_idx: int
    axis: str
    servo_id: int
    source_xyz_chassis: tuple[float, float, float]
    destination_xyz_chassis: tuple[float, float, float]
    via_chassis_drop_xyz: tuple[float, float, float]
    path_length_mm_min: float
    extension_required: str


def _extension_required_str(path_mm: float) -> str:
    """Return the BOM string for one harness path.

    See module docstring for the assumption + rounding rule."""
    deficit_mm = path_mm - STOCK_PIGTAIL_MM
    n_ext = max(0, math.ceil(deficit_mm / EXTENSION_LENGTH_MM))
    if n_ext == 0:
        return "STS3215 stock bus lead"
    if n_ext == 1:
        return "+ 30 cm extension"
    return f"+ {n_ext} x 30 cm extensions"


def _build_plan() -> list[HarnessEntry]:
    plan: list[HarnessEntry] = []
    for leg_idx, edge_mid, _R, R3 in hp._leg_chassis_frames():
        for axis in _AXIS_ORDER:
            j_idx = joint_idx(leg_idx, axis)
            servo_id = joint_to_servo_id(j_idx)

            source_bracket = _cradle_source_bracket_xyz(axis)
            drop_bracket = _drop_slot_bracket_xyz()

            source_chassis = _bracket_to_chassis(edge_mid, R3,
                                                  source_bracket)
            drop_chassis = _bracket_to_chassis(edge_mid, R3,
                                                drop_bracket)
            dest_chassis = _bus_landing_chassis_xyz(leg_idx)

            man_path = (_manhattan(source_chassis, drop_chassis)
                        + _manhattan(drop_chassis, dest_chassis))
            slack = _AXIS_SLACK_MM[axis]
            path_length_mm_min = man_path + slack
            ext = _extension_required_str(path_length_mm_min)

            plan.append(HarnessEntry(
                joint_idx=j_idx,
                leg_idx=leg_idx,
                axis=axis,
                servo_id=servo_id,
                source_xyz_chassis=tuple(round(v, 2) for v in source_chassis),
                destination_xyz_chassis=tuple(round(v, 2) for v in dest_chassis),
                via_chassis_drop_xyz=tuple(round(v, 2) for v in drop_chassis),
                path_length_mm_min=round(path_length_mm_min, 1),
                extension_required=ext,
            ))
    return plan


WIRE_HARNESS_PLAN: list[HarnessEntry] = _build_plan()


# ---------------------------------------------------------------------------
# Markdown table printer
# ---------------------------------------------------------------------------


def _fmt_xyz(v: tuple[float, float, float]) -> str:
    return f"({v[0]:+6.1f}, {v[1]:+6.1f}, {v[2]:+6.1f})"


def print_harness_plan() -> None:
    """Print ``WIRE_HARNESS_PLAN`` as a markdown table to stdout.

    18 rows, one per joint, in joint-index order (0..17).  The
    table is the BOM the user prints to build the leg harnesses;
    feed it into a Markdown viewer or just read in plain text.
    """
    cols = [
        ("joint", 5),
        ("leg", 4),
        ("axis", 10),
        ("ID",    3),
        ("src (mm)", 24),
        ("via drop (mm)", 24),
        ("bus landing (mm)", 24),
        ("path mm", 8),
        ("extension", 40),
    ]
    fmt_cells = lambda cells: "| " + " | ".join(
        f"{c:<{w}}" for (c, (_n, w)) in zip(cells, cols)) + " |"

    headers = [name for name, _ in cols]
    print(fmt_cells(headers))
    print(fmt_cells(["-" * w for _, w in cols]))
    for entry in WIRE_HARNESS_PLAN:
        row = [
            str(entry["joint_idx"]),
            f"L{entry['leg_idx']}",
            entry["axis"],
            str(entry["servo_id"]),
            _fmt_xyz(entry["source_xyz_chassis"]),
            _fmt_xyz(entry["via_chassis_drop_xyz"]),
            _fmt_xyz(entry["destination_xyz_chassis"]),
            f"{entry['path_length_mm_min']:.1f}",
            entry["extension_required"],
        ]
        print(fmt_cells(row))


if __name__ == "__main__":
    print_harness_plan()
