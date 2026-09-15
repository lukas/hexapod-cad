"""Generate a kid-assemblable 4WD pickup truck for the DIANN TT-motor kit.

The purchased kit is Amazon ASIN B0GY8L28XJ: four 1:48 yellow TT motors,
four nominal 65 mm wheels, and two red HW-095/L298N driver modules.  This
generator creates printable STEP/STL parts, visual COTS envelopes, a complete
assembly STEP, a BuildViz scene, and a machine-readable dimensional report.

The design deliberately clamps each motor by its outside envelope instead of
depending on the small molded motor mounting holes.  Those holes vary between
TT-motor clones; the 70 x 22 x 18 mm advertised envelope and 22.5 mm gearbox
width are much more consistent.  All chassis screws go in from above so a kid
can assemble the truck with one Phillips screwdriver.

Run from this directory (or use ``make build``)::

    uv run --no-project --python 3.12 \
      --with build123d --with trimesh --with numpy \
      python build_tt_truck.py
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path

import trimesh
from build123d import Box, Compound, Cylinder, Pos, Rot, export_step, export_stl


HERE = Path(__file__).resolve().parent
OUT_DIR = HERE / "out"
STEP_DIR = OUT_DIR / "step"
STL_DIR = OUT_DIR / "stl"

# ---------------------------------------------------------------------------
# Purchased-part geometry (millimetres)
# ---------------------------------------------------------------------------

# Amazon/Adafruit both list the body as 70 x 22 x 18 mm.  The dimensioned
# drawing in the listing resolves the useful mounting envelope more precisely.
MOTOR_TOTAL_L = 64.2
MOTOR_GEARBOX_L = 31.8
MOTOR_GEARBOX_W = 22.5
MOTOR_GEARBOX_H = 22.5
MOTOR_CAN_L = MOTOR_TOTAL_L - MOTOR_GEARBOX_L
MOTOR_CAN_H = 18.6
MOTOR_SHAFT_FROM_NOSE = 11.2
MOTOR_AXLE_SPAN = 36.8
MOTOR_AXLE_D = 5.4

WHEEL_OD = 66.03
WHEEL_TIRE_W = 30.0
WHEEL_HUB_SPAN = 42.0  # visual hub; the rubber tread itself is 30 mm wide

L298_BOARD_L = 43.0
L298_BOARD_W = 43.0
L298_BOARD_H = 28.6
L298_HOLE_PITCH = 36.6
L298_HOLE_D = 3.0

# ---------------------------------------------------------------------------
# Truck layout and print-fit allowances
# ---------------------------------------------------------------------------

BASE_L = 150.0
BASE_W = 110.0
BASE_T = 4.0
BASE_CORNER_R = 8.0

AXLE_X = 58.0
MOTOR_Y = 40.0
MOTOR_POCKET_CLEARANCE = 0.8
MOTOR_POCKET_DEPTH = 0.8
MOTOR_SEAT_Z = BASE_T - MOTOR_POCKET_DEPTH
MOTOR_SHAFT_Z = MOTOR_SEAT_Z + MOTOR_GEARBOX_H / 2.0

WHEEL_CENTER_Y = 76.5
WHEEL_R = WHEEL_OD / 2.0

STRAP_X = 48.0
STRAP_L = 12.0
STRAP_W = 40.0
STRAP_T = 3.2
STRAP_PAD_H = 0.7
STRAP_HOLE_PITCH = 32.0
M3_CLEAR_D = 3.6
M3_PILOT_D = 2.7
STRAP_BOSS_D = 7.5
STRAP_ASSEMBLY_Z = MOTOR_SEAT_Z + MOTOR_GEARBOX_H
STRAP_BOSS_TOP_Z = STRAP_ASSEMBLY_Z + STRAP_PAD_H

DECK_Z = 34.0
DECK_L = 144.0
DECK_W = 102.0
DECK_T = 3.0
DECK_POST_X = 62.0
DECK_POST_Y = 20.0
DECK_POST_D = 10.0

BOARD_CENTER_X = 35.0
BOARD_CENTER_Y = 23.5
BOARD_STANDOFF_H = 6.0
BOARD_STANDOFF_D = 6.4
M2P5_PILOT_D = 2.2

BATTERY_ZONE_X = -49.0
BATTERY_ZONE_L = 44.0
BATTERY_ZONE_W = 68.0

# Removable classic-pickup body.  +X is the front of the truck.  The lower
# body rests on four raised bosses on the electronics deck; the hood and roof
# are separate flat-printing panels so the whole body remains support-free.
BODY_MOUNT_H = 5.0
BODY_ASSEMBLY_Z = DECK_Z + DECK_T + BODY_MOUNT_H
BODY_MOUNT_X = 67.0
BODY_MOUNT_Y = 43.0
BODY_MOUNT_BOSS_D = 9.0

BODY_SIDE_Y = 54.0
BODY_SIDE_T = 4.0
BODY_FRONT_X = 76.0
BODY_REAR_X = -76.0
BODY_WHEEL_ARCH_R = 38.0
BODY_WHEEL_CENTER_Z = MOTOR_SHAFT_Z - BODY_ASSEMBLY_Z

HOOD_CENTER_X = 42.0
HOOD_L = 66.0
HOOD_W = 108.0
HOOD_SIDE_H = 34.0
HOOD_T = 4.0
HOOD_SCREW_X = (18.0, 68.0)
HOOD_SCREW_Y = 50.0

CAB_CENTER_X = -6.0
CAB_FRONT_X = 10.0
CAB_REAR_X = -22.0
CAB_BELT_H = 31.0
CAB_FRAME_TOP = 62.0
ROOF_L = 42.0
ROOF_W = 112.0
ROOF_T = 4.0
ROOF_SCREW_X = (-19.0, 7.0)
ROOF_SCREW_Y = 48.0


def _box(length: float, width: float, height: float, *,
         x: float = 0.0, y: float = 0.0, z0: float = 0.0):
    """Axis-aligned box centred in X/Y with its bottom at ``z0``."""
    return Pos(x, y, z0 + height / 2.0) * Box(length, width, height)


def _cylinder(radius: float, height: float, *,
              x: float = 0.0, y: float = 0.0, z0: float = 0.0):
    """Z-axis cylinder centred in X/Y with its bottom at ``z0``."""
    return Pos(x, y, z0 + height / 2.0) * Cylinder(radius, height)


def _y_cylinder(radius: float, length: float, *,
                x: float = 0.0, y: float = 0.0, z: float = 0.0):
    """Y-axis cylinder centred at ``x, y, z`` (useful for wheel arches)."""
    return Pos(x, y, z) * (
        Rot(90.0, 0.0, 0.0) * Cylinder(radius, length)
    )


def _rounded_prism(length: float, width: float, height: float,
                   radius: float, *, z0: float = 0.0):
    """Support-free rounded rectangular prism using boxes + corner cylinders."""
    shape = _box(length - 2.0 * radius, width, height, z0=z0)
    shape += _box(length, width - 2.0 * radius, height, z0=z0)
    for x in (-length / 2.0 + radius, length / 2.0 - radius):
        for y in (-width / 2.0 + radius, width / 2.0 - radius):
            shape += _cylinder(radius, height, x=x, y=y, z0=z0)
    return shape


def _obround_cut(length: float, width: float, depth: float, *,
                 x: float = 0.0, y: float = 0.0, z0: float = 0.0):
    """Obround through/recess cutter whose long axis is X."""
    radius = width / 2.0
    center_span = max(0.0, length - 2.0 * radius)
    cut = _box(center_span, width, depth, x=x, y=y, z0=z0)
    for dx in (-center_span / 2.0, center_span / 2.0):
        cut += _cylinder(radius, depth, x=x + dx, y=y, z0=z0)
    return cut


def _motor_body_center_x(axle_x: float) -> float:
    # The gearbox nose is 11.2 mm outboard of the axle and the motor tail is
    # 53.0 mm inboard.  Front/rear motors point toward the chassis centre.
    local_mid_from_axle = (MOTOR_TOTAL_L - 2.0 * MOTOR_SHAFT_FROM_NOSE) / 2.0
    return math.copysign(abs(axle_x) - local_mid_from_axle, axle_x)


def make_chassis():
    """One-piece lower frame with motor seats, clamp bosses, and deck posts."""
    chassis = _rounded_prism(BASE_L, BASE_W, BASE_T, BASE_CORNER_R)

    # Truck-like raised bumpers.  They stay inside the wheel sweep in Y.
    for x in (-BASE_L / 2.0 + 2.5, BASE_L / 2.0 - 2.5):
        # Width stops inside the rounded base's tangent points, so even the
        # corner ends sit on printed material and need no support.
        chassis += _box(5.0, BASE_W - 18.0, 8.0, x=x, z0=BASE_T)

    # Two large central hand/wire openings leave a strong perimeter and spine.
    for x in (-29.0, 29.0):
        chassis -= _obround_cut(38.0, 20.0, BASE_T + 2.0,
                                x=x, z0=-1.0)

    # Shallow motor pockets tolerate clone variation and positively locate the
    # bodies without relying on their inconsistent molded screw holes.
    pocket_l = MOTOR_TOTAL_L + MOTOR_POCKET_CLEARANCE
    pocket_w = MOTOR_GEARBOX_W + MOTOR_POCKET_CLEARANCE
    for axle_x in (-AXLE_X, AXLE_X):
        cx = _motor_body_center_x(axle_x)
        for motor_y in (-MOTOR_Y, MOTOR_Y):
            chassis -= _box(pocket_l, pocket_w, MOTOR_POCKET_DEPTH + 0.2,
                            x=cx, y=motor_y,
                            z0=BASE_T - MOTOR_POCKET_DEPTH)

            # Short guide rails are clear of the clamp and wire exit.
            for side in (-1.0, 1.0):
                rail_y = motor_y + side * (pocket_w / 2.0 + 0.9)
                chassis += _box(18.0, 1.8, 3.0,
                                x=math.copysign(26.0, axle_x),
                                y=rail_y, z0=BASE_T)

            # Two tall bosses per motor receive M3 thread-forming screws.
            clamp_x = math.copysign(STRAP_X, axle_x)
            for dy in (-STRAP_HOLE_PITCH / 2.0,
                       STRAP_HOLE_PITCH / 2.0):
                boss_y = motor_y + dy
                chassis += _cylinder(
                    STRAP_BOSS_D / 2.0,
                    STRAP_BOSS_TOP_Z,
                    x=clamp_x, y=boss_y, z0=0.0,
                )

    # Four central posts carry the removable electronics deck above the motors.
    for x in (-DECK_POST_X, DECK_POST_X):
        for y in (-DECK_POST_Y, DECK_POST_Y):
            chassis += _cylinder(DECK_POST_D / 2.0, DECK_Z - BASE_T,
                                 x=x, y=y, z0=BASE_T)

    # Blind pilot holes: all assembly screws enter from above.
    for axle_x in (-AXLE_X, AXLE_X):
        clamp_x = math.copysign(STRAP_X, axle_x)
        for motor_y in (-MOTOR_Y, MOTOR_Y):
            for dy in (-STRAP_HOLE_PITCH / 2.0,
                       STRAP_HOLE_PITCH / 2.0):
                chassis -= _cylinder(
                    M3_PILOT_D / 2.0, 12.0,
                    x=clamp_x, y=motor_y + dy,
                    z0=STRAP_BOSS_TOP_Z - 11.0,
                )
    for x in (-DECK_POST_X, DECK_POST_X):
        for y in (-DECK_POST_Y, DECK_POST_Y):
            chassis -= _cylinder(M3_PILOT_D / 2.0, 14.0,
                                 x=x, y=y, z0=DECK_Z - 13.0)
    return chassis


def make_motor_clamp():
    """One of four identical screw-down bars with a compliant contact pad."""
    clamp = _box(STRAP_L, STRAP_W, STRAP_T,
                 z0=STRAP_PAD_H)
    clamp += _box(10.0, 14.0, STRAP_PAD_H, z0=0.0)
    for y in (-STRAP_HOLE_PITCH / 2.0, STRAP_HOLE_PITCH / 2.0):
        clamp -= _cylinder(M3_CLEAR_D / 2.0,
                           STRAP_T + STRAP_PAD_H + 2.0,
                           y=y, z0=-1.0)
    return clamp


def make_electronics_deck():
    """Upper deck for both L298N boards, battery, and removable body."""
    deck = _rounded_prism(DECK_L, DECK_W, DECK_T, 6.0)

    # Four chassis attachment holes.
    for x in (-DECK_POST_X, DECK_POST_X):
        for y in (-DECK_POST_Y, DECK_POST_Y):
            deck -= _cylinder(M3_CLEAR_D / 2.0, DECK_T + 2.0,
                              x=x, y=y, z0=-1.0)

    # Four body bosses sit outside the electronics envelopes.  Short M3
    # screws pass through tabs on the pickup shell into blind pilot holes.
    for x in (-BODY_MOUNT_X, BODY_MOUNT_X):
        for y in (-BODY_MOUNT_Y, BODY_MOUNT_Y):
            deck += _cylinder(BODY_MOUNT_BOSS_D / 2.0, BODY_MOUNT_H,
                              x=x, y=y, z0=DECK_T)
            deck -= _cylinder(M3_PILOT_D / 2.0, BODY_MOUNT_H + 1.0,
                              x=x, y=y, z0=DECK_T)

    # Two boards, each on the measured 36.6 mm square hole pattern.
    for board_y in (-BOARD_CENTER_Y, BOARD_CENTER_Y):
        for dx in (-L298_HOLE_PITCH / 2.0, L298_HOLE_PITCH / 2.0):
            for dy in (-L298_HOLE_PITCH / 2.0, L298_HOLE_PITCH / 2.0):
                sx = BOARD_CENTER_X + dx
                sy = board_y + dy
                deck += _cylinder(BOARD_STANDOFF_D / 2.0,
                                  BOARD_STANDOFF_H,
                                  x=sx, y=sy, z0=DECK_T)
                deck -= _cylinder(M2P5_PILOT_D / 2.0,
                                  BOARD_STANDOFF_H + 0.2,
                                  x=sx, y=sy, z0=DECK_T)

    # Rear universal 64 x 64 mm battery zone.  Four low corner fences locate
    # anything from a compact 4xAA holder to a small protected battery case.
    for x in (BATTERY_ZONE_X - BATTERY_ZONE_L / 2.0 + 3.0,
              BATTERY_ZONE_X + BATTERY_ZONE_L / 2.0 - 3.0):
        for y in (-BATTERY_ZONE_W / 2.0 + 3.0,
                  BATTERY_ZONE_W / 2.0 - 3.0):
            deck += _box(6.0, 6.0, 5.0, x=x, y=y, z0=DECK_T)

    # Two hook-and-loop strap stations, plus centre cable-routing slots.
    for x in (-55.0, -21.0):
        for y in (-33.0, 33.0):
            deck -= _box(10.0, 3.5, DECK_T + 2.0,
                         x=x, y=y, z0=-1.0)
    for y in (-25.0, 25.0):
        deck -= _box(8.0, 3.5, DECK_T + 2.0,
                     x=1.0, y=y, z0=-1.0)
    return deck


def _windowed_cross_wall(*, x: float, window_w: float):
    """Cab front/rear wall with a large rectangular window opening."""
    wall = _box(4.0, 108.0, CAB_FRAME_TOP - CAB_BELT_H,
                x=x, z0=CAB_BELT_H)
    wall -= _box(6.0, window_w, 18.0, x=x, z0=38.0)
    return wall


def make_truck_body_lower():
    """Support-free lower pickup shell: bed, cab frame, hood sides, grille."""
    body = None

    # Three height zones create the unmistakable hood/cab/open-bed profile.
    side_zones = (
        (42.0, 68.0, 34.0),    # long hood and front fender
        (-6.0, 32.0, CAB_BELT_H),  # cab lower body
        (-49.0, 54.0, 26.0),   # open pickup bed sides
    )
    for y in (-BODY_SIDE_Y, BODY_SIDE_Y):
        for x, length, height in side_zones:
            zone = _box(length, BODY_SIDE_T, height, x=x, y=y)
            body = zone if body is None else body + zone

    assert body is not None

    # Cut generous wheel arches through both side skins.  The outer body is
    # deliberately inboard of the tyre sidewalls for static running clearance.
    for axle_x in (-AXLE_X, AXLE_X):
        body -= _y_cylinder(BODY_WHEEL_ARCH_R, 130.0,
                            x=axle_x, z=BODY_WHEEL_CENTER_Z)

    # Tailgate, front grille surround, and low protective bumpers.
    body += _box(4.0, 108.0, 26.0, x=BODY_REAR_X + 2.0)
    body += _box(4.0, 108.0, HOOD_SIDE_H,
                 x=BODY_FRONT_X - 2.0)
    body += _box(5.0, 114.0, 6.0,
                 x=BODY_FRONT_X + 2.5, z0=2.0)
    body += _box(5.0, 114.0, 6.0,
                 x=BODY_REAR_X - 2.5, z0=2.0)

    # Square classic-truck cab with open windows.  Full lower bulkheads carry
    # the window frames; the roof is a separate flat-printing panel.
    body += _box(4.0, 108.0, CAB_BELT_H + 1.0, x=CAB_FRONT_X)
    body += _box(4.0, 108.0, CAB_BELT_H + 1.0, x=CAB_REAR_X)
    body += _windowed_cross_wall(x=CAB_FRONT_X, window_w=78.0)
    body += _windowed_cross_wall(x=CAB_REAR_X, window_w=70.0)
    for y in (-BODY_SIDE_Y, BODY_SIDE_Y):
        side_frame = _box(36.0, BODY_SIDE_T,
                          CAB_FRAME_TOP - CAB_BELT_H,
                          x=CAB_CENTER_X, y=y, z0=CAB_BELT_H)
        side_frame -= _box(24.0, BODY_SIDE_T + 2.0, 18.0,
                           x=CAB_CENTER_X, y=y, z0=38.0)
        body += side_frame

    # Four deck tabs make the entire body removable with short M3 screws.
    for x in (-BODY_MOUNT_X, BODY_MOUNT_X):
        for y in (-BODY_MOUNT_Y, BODY_MOUNT_Y):
            body += _box(14.0, 18.0, 3.0, x=x, y=y)
            body -= _cylinder(M3_CLEAR_D / 2.0, 5.0,
                              x=x, y=y, z0=-1.0)

    # Local pilot bosses receive the separately printed hood and cab roof.
    for x in HOOD_SCREW_X:
        for y in (-HOOD_SCREW_Y, HOOD_SCREW_Y):
            body += _cylinder(4.5, 12.0, x=x, y=y,
                              z0=HOOD_SIDE_H - 12.0)
            body -= _cylinder(M3_PILOT_D / 2.0, 10.0,
                              x=x, y=y, z0=HOOD_SIDE_H - 9.0)
    for x in ROOF_SCREW_X:
        for y in (-ROOF_SCREW_Y, ROOF_SCREW_Y):
            body += _cylinder(4.5, 10.0, x=x, y=y,
                              z0=CAB_FRAME_TOP - 10.0)
            body -= _cylinder(M3_PILOT_D / 2.0, 8.0,
                              x=x, y=y, z0=CAB_FRAME_TOP - 7.0)
    return body


def make_truck_hood():
    """Flat-printing removable hood panel with a shallow centre power bulge."""
    hood = _rounded_prism(HOOD_L, HOOD_W, HOOD_T, 4.0)
    hood += _rounded_prism(34.0, 34.0, 2.0, 3.0, z0=HOOD_T)
    for x in HOOD_SCREW_X:
        for y in (-HOOD_SCREW_Y, HOOD_SCREW_Y):
            hood -= _cylinder(M3_CLEAR_D / 2.0, HOOD_T + 4.0,
                              x=x - HOOD_CENTER_X, y=y, z0=-1.0)
    return hood


def make_truck_roof():
    """Flat-printing cab roof with a small front sun visor."""
    roof = _rounded_prism(ROOF_L, ROOF_W, ROOF_T, 5.0)
    roof += _box(5.0, ROOF_W + 4.0, 2.0,
                 x=ROOF_L / 2.0 - 2.5, z0=1.0)
    for x in ROOF_SCREW_X:
        for y in (-ROOF_SCREW_Y, ROOF_SCREW_Y):
            roof -= _cylinder(M3_CLEAR_D / 2.0, ROOF_T + 2.0,
                              x=x - CAB_CENTER_X, y=y, z0=-1.0)
    return roof


def make_windshield_visual():
    """One dark front windshield appearance insert."""
    return _box(1.6, 78.0, 18.0)


def make_rear_window_visual():
    """One dark rear cab-window appearance insert."""
    return _box(1.6, 70.0, 18.0)


def make_side_window_visual():
    """One dark side-window appearance insert, instanced left and right."""
    return _box(24.0, 1.6, 18.0)


def make_truck_grille_visual():
    """Inset dark grille panel for the BuildViz appearance model."""
    return _box(1.6, 62.0, 14.0)


def make_headlight_visual():
    """One simple rectangular headlamp lens, placed twice in the scene."""
    return _box(1.6, 14.0, 9.0)


def make_taillight_visual():
    """One simple rectangular tail lamp lens, placed twice in the scene."""
    return _box(1.6, 10.0, 12.0)


def make_motor_fit_coupon():
    """Quick print to verify the purchased motor and one production clamp."""
    coupon_l = 24.0
    coupon_w = 44.0
    coupon = _rounded_prism(coupon_l, coupon_w, BASE_T, 4.0)
    coupon -= _box(
        20.0,
        MOTOR_GEARBOX_W + MOTOR_POCKET_CLEARANCE,
        MOTOR_POCKET_DEPTH + 0.2,
        z0=BASE_T - MOTOR_POCKET_DEPTH,
    )
    for side in (-1.0, 1.0):
        coupon += _box(18.0, 1.8, 3.0,
                       y=side * ((MOTOR_GEARBOX_W + MOTOR_POCKET_CLEARANCE)
                                 / 2.0 + 0.9),
                       z0=BASE_T)
    for y in (-STRAP_HOLE_PITCH / 2.0, STRAP_HOLE_PITCH / 2.0):
        coupon += _cylinder(STRAP_BOSS_D / 2.0, STRAP_BOSS_TOP_Z,
                            y=y, z0=0.0)
        coupon -= _cylinder(M3_PILOT_D / 2.0, 12.0,
                            y=y, z0=STRAP_BOSS_TOP_Z - 11.0)
    return coupon


def make_l298n_fit_gauge():
    """Small pin gauge for the nominal 36.6 mm-square controller pattern."""
    gauge = _rounded_prism(44.0, 44.0, 2.0, 4.0)
    gauge -= _obround_cut(22.0, 10.0, 4.0, z0=-1.0)
    for x in (-L298_HOLE_PITCH / 2.0, L298_HOLE_PITCH / 2.0):
        for y in (-L298_HOLE_PITCH / 2.0, L298_HOLE_PITCH / 2.0):
            gauge += _cylinder(1.25, 4.0, x=x, y=y, z0=2.0)
    return gauge


# ---------------------------------------------------------------------------
# COTS visualization envelopes (not printable replacement parts)
# ---------------------------------------------------------------------------

def make_motor_visual():
    """TT motor envelope; local +X points from axle toward motor tail."""
    gear_x0 = -MOTOR_SHAFT_FROM_NOSE
    gear = _box(MOTOR_GEARBOX_L, MOTOR_GEARBOX_W, MOTOR_GEARBOX_H,
                x=gear_x0 + MOTOR_GEARBOX_L / 2.0)
    can_x0 = gear_x0 + MOTOR_GEARBOX_L
    can = _box(MOTOR_CAN_L, MOTOR_GEARBOX_W, MOTOR_CAN_H,
               x=can_x0 + MOTOR_CAN_L / 2.0,
               z0=(MOTOR_GEARBOX_H - MOTOR_CAN_H) / 2.0)
    axle = Pos(0.0, 0.0, MOTOR_GEARBOX_H / 2.0) * (
        Rot(90.0, 0.0, 0.0) * Cylinder(MOTOR_AXLE_D / 2.0,
                                        MOTOR_AXLE_SPAN)
    )
    return gear + can + axle


def make_wheel_visual():
    """Nominal tyre plus its longer press-fit hub, already oriented on Y."""
    tyre = Rot(90.0, 0.0, 0.0) * Cylinder(WHEEL_R, WHEEL_TIRE_W)
    hub = Rot(90.0, 0.0, 0.0) * Cylinder(6.0, WHEEL_HUB_SPAN)
    return tyre + hub


def make_l298n_visual():
    """Conservative 43 x 43 x 28.6 mm controller-board envelope."""
    pcb = _box(L298_BOARD_L, L298_BOARD_W, 1.6)
    heatsink = _box(24.0, 17.0, L298_BOARD_H - 1.6,
                    x=-3.0, z0=1.6)
    terminals_a = _box(10.0, 29.0, 10.0, x=16.5, z0=1.6)
    terminals_b = _box(19.0, 9.0, 10.0, x=-9.0, y=-17.0, z0=1.6)
    return pcb + heatsink + terminals_a + terminals_b


def _transform(tx: float = 0.0, ty: float = 0.0, tz: float = 0.0,
               rz_deg: float = 0.0) -> list[float]:
    """Three.js-compatible column-major rigid transform."""
    a = math.radians(rz_deg)
    c = math.cos(a)
    s = math.sin(a)
    return [
        c, s, 0.0, 0.0,
        -s, c, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        tx, ty, tz, 1.0,
    ]


def _instance(instance_id: str, mesh: str, name: str, part_type: str,
              color: str, transform: list[float], focus: str,
              *, cots: bool = False, role: str | None = None) -> dict:
    return {
        "id": instance_id,
        "meshId": f"stl:{mesh}",
        "name": name,
        "partType": part_type,
        "role": role or ("purchased component" if cots else
                         "3D printed part"),
        "cots": cots,
        "color": color,
        "transform": transform,
        "centroid": [transform[12], transform[13], transform[14]],
        "focusGroup": focus,
    }


def make_scene() -> dict:
    mesh_names = {
        "chassis": "chassis.stl",
        "motor_clamp": "motor_clamp.stl",
        "electronics_deck": "electronics_deck.stl",
        "truck_body_lower": "truck_body_lower.stl",
        "truck_hood": "truck_hood.stl",
        "truck_roof": "truck_roof.stl",
        "windshield": "windshield_DO_NOT_PRINT.stl",
        "rear_window": "rear_window_DO_NOT_PRINT.stl",
        "side_window": "side_window_DO_NOT_PRINT.stl",
        "truck_grille": "truck_grille_DO_NOT_PRINT.stl",
        "headlight": "headlight_DO_NOT_PRINT.stl",
        "taillight": "taillight_DO_NOT_PRINT.stl",
        "tt_motor": "tt_motor_DO_NOT_PRINT.stl",
        "wheel": "wheel_DO_NOT_PRINT.stl",
        "l298n_board": "l298n_board_DO_NOT_PRINT.stl",
    }
    instances = [
        _instance("chassis", "chassis", "Lower truck chassis", "chassis",
                  "#4f86c6", _transform(), "printable"),
        _instance("electronics-deck", "electronics_deck",
                  "Electronics + battery deck", "electronics_deck",
                  "#7aa6d8", _transform(tz=DECK_Z), "printable"),
        _instance("truck-body-lower", "truck_body_lower",
                  "Classic pickup lower body", "truck_body_lower",
                  "#d64b3c", _transform(tz=BODY_ASSEMBLY_Z),
                  "printable"),
        _instance("truck-hood", "truck_hood", "Removable pickup hood",
                  "truck_hood", "#e05243",
                  _transform(HOOD_CENTER_X, 0.0,
                             BODY_ASSEMBLY_Z + HOOD_SIDE_H),
                  "printable"),
        _instance("truck-roof", "truck_roof", "Removable cab roof",
                  "truck_roof", "#c83f34",
                  _transform(CAB_CENTER_X, 0.0,
                             BODY_ASSEMBLY_Z + CAB_FRAME_TOP),
                  "printable"),
        _instance("truck-windshield", "windshield", "Dark windshield insert",
                  "truck_glazing", "#182b3d",
                  _transform(CAB_FRONT_X - 2.8, 0.0,
                             BODY_ASSEMBLY_Z + 38.0),
                  "pickup-body", role="visual appearance only"),
        _instance("truck-rear-window", "rear_window",
                  "Dark rear-window insert", "truck_glazing", "#182b3d",
                  _transform(CAB_REAR_X + 2.8, 0.0,
                             BODY_ASSEMBLY_Z + 38.0),
                  "pickup-body", role="visual appearance only"),
        _instance("truck-window-left", "side_window",
                  "Dark side-window insert left", "truck_glazing", "#182b3d",
                  _transform(CAB_CENTER_X, BODY_SIDE_Y - 2.8,
                             BODY_ASSEMBLY_Z + 38.0),
                  "pickup-body", role="visual appearance only"),
        _instance("truck-window-right", "side_window",
                  "Dark side-window insert right", "truck_glazing", "#182b3d",
                  _transform(CAB_CENTER_X, -BODY_SIDE_Y + 2.8,
                             BODY_ASSEMBLY_Z + 38.0),
                  "pickup-body", role="visual appearance only"),
        _instance("truck-grille", "truck_grille", "Front grille insert",
                  "truck_grille", "#26313a",
                  _transform(BODY_FRONT_X + 0.8, 0.0,
                             BODY_ASSEMBLY_Z + 9.0), "pickup-body",
                  role="visual appearance only"),
    ]

    for side_name, y in (("left", BODY_SIDE_Y - 15.0),
                         ("right", -BODY_SIDE_Y + 15.0)):
        instances.append(_instance(
            f"headlight-{side_name}", "headlight",
            f"Headlight {side_name}", "headlight", "#ffd166",
            _transform(BODY_FRONT_X + 0.8, y,
                       BODY_ASSEMBLY_Z + 21.0),
            "pickup-body", role="visual appearance only",
        ))
        instances.append(_instance(
            f"taillight-{side_name}", "taillight",
            f"Tail light {side_name}", "taillight", "#9f1d2d",
            _transform(BODY_REAR_X - 0.8,
                       math.copysign(BODY_SIDE_Y - 12.0, y),
                       BODY_ASSEMBLY_Z + 8.0),
            "pickup-body", role="visual appearance only",
        ))

    for axle_name, axle_x in (("rear", -AXLE_X), ("front", AXLE_X)):
        motor_rz = 0.0 if axle_x < 0.0 else 180.0
        clamp_x = math.copysign(STRAP_X, axle_x)
        for side_name, motor_y in (("left", MOTOR_Y),
                                   ("right", -MOTOR_Y)):
            tag = f"{axle_name}-{side_name}"
            instances.append(_instance(
                f"clamp-{tag}", "motor_clamp", f"Motor clamp {tag}",
                "motor_clamp", "#efc65a",
                _transform(clamp_x, motor_y, STRAP_ASSEMBLY_Z),
                "printable",
            ))
            instances.append(_instance(
                f"motor-{tag}", "tt_motor", f"TT motor {tag}",
                "tt_motor", "#f2c400",
                _transform(axle_x, motor_y, MOTOR_SEAT_Z, motor_rz),
                "hardware", cots=True,
            ))
            wheel_y = math.copysign(WHEEL_CENTER_Y, motor_y)
            instances.append(_instance(
                f"wheel-{tag}", "wheel", f"65 mm wheel {tag}",
                "wheel", "#20242b",
                _transform(axle_x, wheel_y, MOTOR_SHAFT_Z),
                "hardware", cots=True,
            ))

    board_z = DECK_Z + DECK_T + BOARD_STANDOFF_H
    for name, y in (("A", -BOARD_CENTER_Y), ("B", BOARD_CENTER_Y)):
        instances.append(_instance(
            f"l298n-{name.lower()}", "l298n_board",
            f"L298N motor board {name}", "l298n_board", "#c53b3b",
            _transform(BOARD_CENTER_X, y, board_z),
            "hardware", cots=True,
        ))

    return {
        "schemaVersion": 1,
        "name": "TT-motor kid 4x4 classic pickup truck",
        "source": "build_tt_truck.py",
        "designSpecUrl": "design_spec.yaml",
        "assetsBaseUrl": "./",
        "units": "mm",
        "center": [0.0, 0.0, 48.0],
        "checksConfig": {
            "toleranceMm": 0.35,
            "clearanceMm": 0.8,
            "minWallMm": 1.5,
            "matingToleranceMm": 1.1,
        },
        "meshes": [
            {"id": f"stl:{key}", "name": filename,
             "url": f"stl/{filename}"}
            for key, filename in mesh_names.items()
        ],
        "instances": instances,
    }


def _placed(shape, *, x: float = 0.0, y: float = 0.0, z: float = 0.0,
            rz: float = 0.0):
    return Pos(x, y, z) * (Rot(0.0, 0.0, rz) * shape)


def make_assembly(parts: dict[str, object]):
    children = [parts["chassis"],
                _placed(parts["electronics_deck"], z=DECK_Z),
                _placed(parts["truck_body_lower"], z=BODY_ASSEMBLY_Z),
                _placed(parts["truck_hood"], x=HOOD_CENTER_X,
                        z=BODY_ASSEMBLY_Z + HOOD_SIDE_H),
                _placed(parts["truck_roof"], x=CAB_CENTER_X,
                        z=BODY_ASSEMBLY_Z + CAB_FRAME_TOP),
                _placed(parts["windshield"], x=CAB_FRONT_X - 2.8,
                        z=BODY_ASSEMBLY_Z + 38.0),
                _placed(parts["rear_window"], x=CAB_REAR_X + 2.8,
                        z=BODY_ASSEMBLY_Z + 38.0),
                _placed(parts["side_window"], x=CAB_CENTER_X,
                        y=BODY_SIDE_Y - 2.8,
                        z=BODY_ASSEMBLY_Z + 38.0),
                _placed(parts["side_window"], x=CAB_CENTER_X,
                        y=-BODY_SIDE_Y + 2.8,
                        z=BODY_ASSEMBLY_Z + 38.0),
                _placed(parts["truck_grille"],
                        x=BODY_FRONT_X + 0.8,
                        z=BODY_ASSEMBLY_Z + 9.0)]
    for y in (BODY_SIDE_Y - 15.0, -BODY_SIDE_Y + 15.0):
        children.append(_placed(
            parts["headlight"], x=BODY_FRONT_X + 0.8, y=y,
            z=BODY_ASSEMBLY_Z + 21.0,
        ))
        children.append(_placed(
            parts["taillight"], x=BODY_REAR_X - 0.8,
            y=math.copysign(BODY_SIDE_Y - 12.0, y),
            z=BODY_ASSEMBLY_Z + 8.0,
        ))
    for axle_x in (-AXLE_X, AXLE_X):
        motor_rz = 0.0 if axle_x < 0.0 else 180.0
        clamp_x = math.copysign(STRAP_X, axle_x)
        for motor_y in (-MOTOR_Y, MOTOR_Y):
            children.append(_placed(parts["motor_clamp"], x=clamp_x,
                                    y=motor_y, z=STRAP_ASSEMBLY_Z))
            children.append(_placed(parts["tt_motor"], x=axle_x,
                                    y=motor_y, z=MOTOR_SEAT_Z,
                                    rz=motor_rz))
            children.append(_placed(
                parts["wheel"], x=axle_x,
                y=math.copysign(WHEEL_CENTER_Y, motor_y),
                z=MOTOR_SHAFT_Z,
            ))
    board_z = DECK_Z + DECK_T + BOARD_STANDOFF_H
    for y in (-BOARD_CENTER_Y, BOARD_CENTER_Y):
        children.append(_placed(parts["l298n_board"], x=BOARD_CENTER_X,
                                y=y, z=board_z))
    return Compound(children=children)


def _mesh_stats(path: Path) -> dict:
    mesh = trimesh.load(path, process=True)
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(
            [g for g in mesh.geometry.values() if len(g.faces) > 0]
        )
    return {
        "watertight": bool(mesh.is_watertight),
        "triangles": int(len(mesh.faces)),
        "volume_mm3": round(float(abs(mesh.volume)), 2),
        "bbox_mm": [round(float(v), 3) for v in mesh.extents],
    }


def _validate(parts: dict[str, object], stats: dict[str, dict]) -> list[dict]:
    checks: list[dict] = []

    def add(check_id: str, passed: bool, value: object, requirement: str):
        checks.append({
            "id": check_id,
            "status": "pass" if passed else "fail",
            "value": value,
            "requirement": requirement,
        })

    for name, row in stats.items():
        add(f"watertight-{name}", bool(row["watertight"]),
            row["watertight"], "all exported meshes are watertight")

    ground_clearance = -(MOTOR_SHAFT_Z - WHEEL_R)
    add("ground-clearance", ground_clearance >= 15.0,
        round(ground_clearance, 2), ">= 15 mm under lower chassis")

    motor_top = MOTOR_SEAT_Z + MOTOR_GEARBOX_H
    deck_clearance = DECK_Z - motor_top
    add("motor-to-deck-clearance", deck_clearance >= 7.0,
        round(deck_clearance, 2), ">= 7 mm for motor wires")

    outer_boss_y = MOTOR_Y + STRAP_HOLE_PITCH / 2.0 + STRAP_BOSS_D / 2.0
    wheel_inner_y = WHEEL_CENTER_Y - WHEEL_TIRE_W / 2.0
    tyre_boss_clearance = wheel_inner_y - outer_boss_y
    add("wheel-to-clamp-boss-clearance", tyre_boss_clearance >= 1.5,
        round(tyre_boss_clearance, 2), ">= 1.5 mm static tyre clearance")

    inner_motor_tail = AXLE_X - (MOTOR_TOTAL_L - MOTOR_SHAFT_FROM_NOSE)
    tail_gap = 2.0 * inner_motor_tail
    add("front-rear-motor-tail-gap", tail_gap >= 8.0,
        round(tail_gap, 2), ">= 8 mm between opposed motor tails")

    board_outer_y = BOARD_CENTER_Y + L298_BOARD_W / 2.0
    board_edge_clearance = DECK_W / 2.0 - board_outer_y
    add("controller-deck-edge-clearance", board_edge_clearance >= 5.0,
        round(board_edge_clearance, 2), ">= 5 mm around controller PCB")

    body_outer_y = BODY_SIDE_Y + BODY_SIDE_T / 2.0
    body_tyre_clearance = wheel_inner_y - body_outer_y
    add("pickup-body-to-tyre-clearance", body_tyre_clearance >= 3.0,
        round(body_tyre_clearance, 2),
        ">= 3 mm between removable body and tyre sidewall")

    board_top = DECK_Z + DECK_T + BOARD_STANDOFF_H + L298_BOARD_H
    hood_underside = BODY_ASSEMBLY_Z + HOOD_SIDE_H
    hood_clearance = hood_underside - board_top
    add("electronics-to-hood-clearance", hood_clearance >= 3.0,
        round(hood_clearance, 2),
        ">= 3 mm above the L298N visual envelope")

    board_rear_x = BOARD_CENTER_X - L298_BOARD_L / 2.0
    cab_front_inner_x = CAB_FRONT_X + 2.0
    board_cab_clearance = board_rear_x - cab_front_inner_x
    add("controller-to-cab-clearance", board_cab_clearance >= 1.0,
        round(board_cab_clearance, 2),
        ">= 1 mm between controller boards and cab front wall")

    print_bbox = parts["chassis"].bounding_box().size
    fits_x1c = print_bbox.X <= 256.0 and print_bbox.Y <= 256.0
    add("chassis-fits-x1c-bed", fits_x1c,
        [round(print_bbox.X, 2), round(print_bbox.Y, 2)],
        "fits 256 x 256 mm print bed")

    body_bbox = parts["truck_body_lower"].bounding_box().size
    body_fits_x1c = body_bbox.X <= 256.0 and body_bbox.Y <= 256.0
    add("pickup-body-fits-x1c-bed", body_fits_x1c,
        [round(body_bbox.X, 2), round(body_bbox.Y, 2)],
        "fits 256 x 256 mm print bed")

    return checks


def generate() -> dict:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    STEP_DIR.mkdir(parents=True, exist_ok=True)
    STL_DIR.mkdir(parents=True, exist_ok=True)

    parts = {
        "chassis": make_chassis(),
        "motor_clamp": make_motor_clamp(),
        "electronics_deck": make_electronics_deck(),
        "truck_body_lower": make_truck_body_lower(),
        "truck_hood": make_truck_hood(),
        "truck_roof": make_truck_roof(),
        "windshield": make_windshield_visual(),
        "rear_window": make_rear_window_visual(),
        "side_window": make_side_window_visual(),
        "truck_grille": make_truck_grille_visual(),
        "headlight": make_headlight_visual(),
        "taillight": make_taillight_visual(),
        "motor_fit_coupon": make_motor_fit_coupon(),
        "l298n_fit_gauge": make_l298n_fit_gauge(),
        "tt_motor": make_motor_visual(),
        "wheel": make_wheel_visual(),
        "l298n_board": make_l298n_visual(),
    }
    filenames = {
        "chassis": "chassis",
        "motor_clamp": "motor_clamp",
        "electronics_deck": "electronics_deck",
        "truck_body_lower": "truck_body_lower",
        "truck_hood": "truck_hood",
        "truck_roof": "truck_roof",
        "windshield": "windshield_DO_NOT_PRINT",
        "rear_window": "rear_window_DO_NOT_PRINT",
        "side_window": "side_window_DO_NOT_PRINT",
        "truck_grille": "truck_grille_DO_NOT_PRINT",
        "headlight": "headlight_DO_NOT_PRINT",
        "taillight": "taillight_DO_NOT_PRINT",
        "motor_fit_coupon": "motor_fit_coupon",
        "l298n_fit_gauge": "l298n_fit_gauge",
        "tt_motor": "tt_motor_DO_NOT_PRINT",
        "wheel": "wheel_DO_NOT_PRINT",
        "l298n_board": "l298n_board_DO_NOT_PRINT",
    }

    stats: dict[str, dict] = {}
    for key, part in parts.items():
        stem = filenames[key]
        step_path = STEP_DIR / f"{stem}.step"
        stl_path = STL_DIR / f"{stem}.stl"
        export_step(part, step_path)
        export_stl(part, stl_path)
        stats[key] = _mesh_stats(stl_path)
        bbox = stats[key]["bbox_mm"]
        print(f"wrote {stl_path.relative_to(HERE)}  "
              f"bbox={bbox[0]:.2f} x {bbox[1]:.2f} x {bbox[2]:.2f} mm")

    assembly = make_assembly(parts)
    assembly_path = STEP_DIR / "tt_kid_truck_assembly.step"
    export_step(assembly, assembly_path)
    print(f"wrote {assembly_path.relative_to(HERE)}")

    scene = make_scene()
    scene_path = OUT_DIR / "scene.json"
    scene_path.write_text(json.dumps(scene, indent=2) + "\n")

    checks = _validate(parts, stats)
    report = {
        "name": "TT-motor kid 4x4 classic pickup truck",
        "source_asin": "B0GY8L28XJ",
        "units": "mm",
        "source_geometry": {
            "motor_advertised_body": [70.0, 22.0, 18.0],
            "motor_dimensioned_body_length": MOTOR_TOTAL_L,
            "motor_gearbox_width": MOTOR_GEARBOX_W,
            "motor_axle_span": MOTOR_AXLE_SPAN,
            "wheel_nominal_diameter": 65.0,
            "wheel_drawing_diameter": WHEEL_OD,
            "wheel_tread_width": WHEEL_TIRE_W,
            "l298n_board": [L298_BOARD_L, L298_BOARD_W, L298_BOARD_H],
            "l298n_hole_pitch_square": L298_HOLE_PITCH,
            "l298n_hole_diameter": L298_HOLE_D,
        },
        "vehicle_envelope": {
            "overall_length": round(2.0 * (AXLE_X + WHEEL_R), 2),
            "overall_tire_width": round(2.0 *
                                         (WHEEL_CENTER_Y + WHEEL_TIRE_W / 2.0), 2),
            "overall_height": round(
                BODY_ASSEMBLY_Z + CAB_FRAME_TOP + ROOF_T -
                (MOTOR_SHAFT_Z - WHEEL_R), 2),
            "ground_clearance": round(-(MOTOR_SHAFT_Z - WHEEL_R), 2),
            "wheelbase": 2.0 * AXLE_X,
            "track_width": 2.0 * WHEEL_CENTER_Y,
        },
        "print_quantities": {
            "chassis": 1,
            "electronics_deck": 1,
            "truck_body_lower": 1,
            "truck_hood": 1,
            "truck_roof": 1,
            "motor_clamp": 4,
            "motor_fit_coupon_optional": 1,
            "l298n_fit_gauge_optional": 1,
        },
        "mesh_stats": stats,
        "checks": checks,
    }
    report_path = OUT_DIR / "design_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    for source_name in ("design_spec.yaml", "ASSEMBLY.md", "BOM.md"):
        shutil.copyfile(HERE / source_name, OUT_DIR / source_name)
    print(f"wrote {scene_path.relative_to(HERE)}")
    print(f"wrote {report_path.relative_to(HERE)}")

    failures = [row for row in checks if row["status"] != "pass"]
    for row in checks:
        print(f"  [{row['status'].upper():4s}] {row['id']}: {row['value']}")
    if failures:
        raise SystemExit(f"{len(failures)} design check(s) failed")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check-only", action="store_true",
                        help="validate existing output without regenerating")
    args = parser.parse_args()
    if args.check_only:
        report_path = OUT_DIR / "design_report.json"
        report = json.loads(report_path.read_text())
        failures = [row for row in report["checks"]
                    if row["status"] != "pass"]
        if failures:
            raise SystemExit(f"{len(failures)} recorded check(s) failed")
        for key, row in report["mesh_stats"].items():
            path_name = {
                "tt_motor": "tt_motor_DO_NOT_PRINT.stl",
                "wheel": "wheel_DO_NOT_PRINT.stl",
                "l298n_board": "l298n_board_DO_NOT_PRINT.stl",
                "windshield": "windshield_DO_NOT_PRINT.stl",
                "rear_window": "rear_window_DO_NOT_PRINT.stl",
                "side_window": "side_window_DO_NOT_PRINT.stl",
                "truck_grille": "truck_grille_DO_NOT_PRINT.stl",
                "headlight": "headlight_DO_NOT_PRINT.stl",
                "taillight": "taillight_DO_NOT_PRINT.stl",
            }.get(key, f"{key}.stl")
            current = _mesh_stats(STL_DIR / path_name)
            if not current["watertight"]:
                raise SystemExit(f"{path_name} is not watertight")
        print("all recorded dimensional and mesh checks pass")
        return
    generate()


if __name__ == "__main__":
    main()
