#!/usr/bin/env python3
"""Export a STEP-first sidecar bundle for the STS3215 hexapod.

This is intentionally additive: it writes only under cad_step_test/out and
imports dimensions from the existing hexapod_prototype module so the experiment
does not fork the design constants.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys

import numpy as np
from build123d import (
    Align,
    Axis,
    Box,
    BuildLine,
    BuildPart,
    BuildSketch,
    Cone,
    Cylinder,
    Plane,
    Polyline,
    Pos,
    Rotation,
    extrude,
    make_face,
    revolve,
)

from step_common import (
    OUT_DIR,
    PROTO_DIR,
    THIS_DIR,
    StepPart,
    export_all,
    write_bundle,
)

sys.path.insert(0, str(PROTO_DIR))
import hexapod_prototype as hp  # noqa: E402


def _box(extents: tuple[float, float, float],
         center: tuple[float, float, float]) -> object:
    return Pos(*center) * Box(*extents)


def _cyl_z(radius: float, height: float,
           center: tuple[float, float, float]) -> object:
    return Pos(*center) * Cylinder(radius, height)


def _cyl_x(radius: float, length: float,
           center: tuple[float, float, float]) -> object:
    return Pos(*center) * Rotation(0, 90, 0) * Cylinder(radius, length)


def _cyl_y(radius: float, length: float,
           center: tuple[float, float, float]) -> object:
    return Pos(*center) * Rotation(90, 0, 0) * Cylinder(radius, length)


def _cone_x_from_base(radius: float,
                      length: float,
                      *,
                      base_x: float,
                      y: float,
                      z: float,
                      direction: int = 1) -> object:
    """Cone with its circular base on ``base_x`` and apex along +/-X."""
    rot = Rotation(0, 90 if direction > 0 else -90, 0)
    return (
        Pos(base_x, y, z)
        * rot
        * Cone(radius, 0.0, length, align=(Align.CENTER, Align.CENTER, Align.MIN))
    )


def _union(*parts: object) -> object:
    live = [p for p in parts if p is not None]
    if not live:
        raise ValueError("cannot union an empty part list")
    out = live[0]
    for part in live[1:]:
        out = out + part
    return out


def _diff(base: object, *cuts: object) -> object:
    out = base
    for cut in cuts:
        if cut is not None:
            out = out - cut
    return out


def _intersect(base: object, tool: object) -> object:
    return base & tool


def _xy_polygon_prism(points: list[tuple[float, float]], thickness: float) -> object:
    with BuildPart() as prism:
        with BuildSketch(Plane.XY):
            with BuildLine():
                Polyline(*points, close=True)
            make_face()
        extrude(amount=thickness)
    return Pos(0.0, 0.0, -thickness / 2.0) * prism.part


def _hex_plate(flat_to_flat: float,
               thickness: float,
               *,
               with_centre_holes: bool = False,
               with_chassis_standoffs: bool = False,
               with_leg_features: bool = False,
               with_leg_harness_drops: bool = False) -> object:
    apothem = flat_to_flat / 2.0
    circum = apothem / math.cos(math.pi / 6.0)
    points = [
        (
            circum * math.cos(math.pi / 6.0 + i * math.pi / 3.0),
            circum * math.sin(math.pi / 6.0 + i * math.pi / 3.0),
        )
        for i in range(6)
    ]
    plate = _xy_polygon_prism(points, thickness)
    cuts = []
    if with_leg_features:
        body_centre_x = -hp.SERVO_OUTPUT_X
        body_cutout_w = hp.WELL_W + 2.0
        body_cutout_d = hp.WELL_D + 2.0
        for _i, edge_mid, _R, _R3 in hp._leg_chassis_frames():
            az = math.atan2(float(edge_mid[1]), float(edge_mid[0]))
            cuts.append(
                Pos(float(edge_mid[0]), float(edge_mid[1]), 0.0)
                * Rotation(0, 0, math.degrees(az))
                * _box((body_cutout_w, body_cutout_d, thickness * 4.0),
                       (body_centre_x, 0.0, 0.0))
            )
            if with_leg_harness_drops:
                for sx, sy, sxe, sye in hp.leg_harness_drop_slots():
                    cuts.append(
                        Pos(float(edge_mid[0]), float(edge_mid[1]), 0.0)
                        * Rotation(0, 0, math.degrees(az))
                        * _box((sxe, sye, thickness * 4.0), (sx, sy, 0.0))
                    )

    if with_centre_holes:
        for cx, cy in hp.ELEC_CHASSIS_MOUNT_HOLES_XY:
            cuts.append(
                _cyl_z(hp.BRACKET_BOLT_HOLE / 2.0, thickness * 4.0,
                       (cx, cy, 0.0))
            )
    if with_chassis_standoffs:
        for cx, cy in hp.CHASSIS_STANDOFF_HOLES_XY:
            cuts.append(
                _cyl_z(hp.BRACKET_BOLT_HOLE / 2.0, thickness * 4.0,
                       (cx, cy, 0.0))
            )
    return _diff(plate, *cuts)


def make_disc_horn() -> object:
    """20 mm aluminum disc horn as a clean BREP."""
    h = hp.DISC_HORN_H
    disc = _cyl_z(hp.DISC_HORN_OD / 2.0, h, (0.0, 0.0, h / 2.0))
    cuts = [
        _cyl_z(hp.DISC_HORN_SPLINE_OD / 2.0, h + 2.0, (0.0, 0.0, h / 2.0))
    ]
    r = hp.DISC_HORN_BOLT_PCD / 2.0
    for angle in hp.DISC_HORN_BOLT_ANGLES_RAD:
        cuts.append(
            _cyl_z(
                hp.DISC_HORN_TAP_OD / 2.0,
                h + 2.0,
                (r * math.cos(angle), r * math.sin(angle), h / 2.0),
            )
        )
    return _diff(disc, *cuts)


def make_tibia_tube_printable(length: float | None = None,
                              *,
                              solid: bool = False) -> object:
    """Printable substitute for the 8 mm OD tibia tube."""
    length = hp.TIBIA_LENGTH - hp.FOOT_BOOT_TIP_L if length is None else length
    r_out = hp.LEG_TUBE_OD / 2.0
    r_in = r_out - hp.LEG_TUBE_WALL
    body = _cyl_z(r_out, length, (0.0, 0.0, length / 2.0))
    cuts = []
    if not solid:
        cuts.append(_cyl_z(r_in, length + 2.0, (0.0, 0.0, length / 2.0)))
    cuts.append(
        _cyl_y(
            hp.LEG_TUBE_PIN_OD / 2.0,
            4.0 * hp.LEG_TUBE_OD,
            (0.0, 0.0, hp.LEG_TUBE_PIN_INSET),
        )
    )
    return _diff(body, *cuts)


def make_chassis_top() -> object:
    """Top deck plate as BREP, ported from hp.make_chassis_top."""
    plate = _hex_plate(
        hp.CHASSIS_TOP_FLAT_TO_FLAT,
        hp.CHASSIS_TOP_T,
        with_centre_holes=True,
        with_chassis_standoffs=True,
    )

    yaw_clearance = 1.0
    yaw_hole_r = hp.DISC_HORN_OD / 2.0 + yaw_clearance
    yaw_passthroughs = [
        _cyl_z(yaw_hole_r, hp.CHASSIS_PLATE_T * 4.0,
               (edge_mid[0], edge_mid[1], 0.0))
        for _i, edge_mid, _R, _R3 in hp._leg_chassis_frames()
    ]
    plate = _diff(plate, *yaw_passthroughs)

    # Aug 2026 mainline: switch-holster insert bosses are retired. The
    # holster velcros to the flat deck, so the STEP port keeps this face flat.
    column_holes = [
        _cyl_z(hp.BRACKET_BOLT_HOLE / 2.0, hp.CHASSIS_PLATE_T * 4.0,
               (cx, cy, 0.0))
        for cx, cy in hp.DECK_COLUMN_XY
    ]
    top = _diff(plate, *column_holes)
    clip = _cyl_z(
        hp.CHASSIS_TOP_RADIUS,
        hp.CHASSIS_PLATE_T * 6.0 + 40.0,
        (0.0, 0.0, 0.0),
    )
    return _intersect(top, clip)


def make_switch_holster() -> object:
    """Printed anti-spark-switch holster as BREP."""
    outer_l = hp.SWITCH_HOLSTER_OUTER_L
    outer_w = hp.SWITCH_HOLSTER_OUTER_W
    socket_l = hp.SWITCH_SOCKET_OUTER_L
    socket_centre_x = outer_l / 2.0 - socket_l / 2.0
    socket_outer_h = hp.SWITCH_BODY_H + hp.SWITCH_BODY_CL + hp.SWITCH_HOLSTER_FLOOR

    block = _box(
        (socket_l, outer_w, socket_outer_h),
        (socket_centre_x, 0.0, socket_outer_h / 2.0),
    )

    cavity_l = hp.SWITCH_BODY_L + 2.0 * hp.SWITCH_BODY_CL
    cavity_w = hp.SWITCH_BODY_W + 2.0 * hp.SWITCH_BODY_CL
    cavity_h = hp.SWITCH_BODY_H + hp.SWITCH_BODY_CL
    cavity = _box(
        (cavity_l, cavity_w, cavity_h + 0.5),
        (
            socket_centre_x,
            0.0,
            hp.SWITCH_HOLSTER_FLOOR + cavity_h / 2.0 + 0.25,
        ),
    )
    toggle = _box(
        (hp.SWITCH_HOLSTER_WALL + 0.4, hp.SWITCH_TOGGLE_W, hp.SWITCH_TOGGLE_H),
        (
            outer_l / 2.0 - (hp.SWITCH_HOLSTER_WALL + 0.4) / 2.0 + 0.2,
            0.0,
            hp.SWITCH_HOLSTER_FLOOR + hp.SWITCH_BODY_H / 2.0,
        ),
    )

    cuts = [cavity, toggle]
    socket_minus_x_face = socket_centre_x - socket_l / 2.0
    pigtail_len = hp.SWITCH_HOLSTER_WALL + 0.4
    for sy in (-1.0, +1.0):
        cuts.append(
            _cyl_x(
                hp.SWITCH_PIGTAIL_OD / 2.0,
                pigtail_len,
                (
                    socket_minus_x_face - 0.2 + pigtail_len / 2.0,
                    sy * hp.SWITCH_PIGTAIL_DY,
                    hp.SWITCH_HOLSTER_FLOOR + hp.SWITCH_BODY_H / 2.0,
                ),
            )
        )
    return _diff(block, *cuts)


def make_foot_boot(*, extra_tip: float = 0.0) -> object:
    """TPU boot as a revolved BREP dome profile."""
    tip_l = hp.FOOT_BOOT_TIP_L + float(extra_tip)
    total_l = hp.FOOT_BOOT_SOCKET_DEPTH + tip_l
    r_out = hp.FOOT_BOOT_OD / 2.0
    r_bore = hp.FOOT_BOOT_BORE_D / 2.0
    lead = hp.FOOT_BOOT_MOUTH_LEAD
    theta = np.linspace(0.0, math.pi / 2.0, 25)
    dome = [
        (r_out * math.sin(float(t)), r_out * (1.0 - math.cos(float(t))))
        for t in theta
    ]
    profile = dome + [
        (r_out, total_l),
        (r_bore + lead, total_l),
        (r_bore, total_l - lead),
        (r_bore, tip_l),
        (0.0, tip_l - r_bore),
    ]
    with BuildPart() as boot:
        with BuildSketch(Plane.XZ):
            with BuildLine():
                Polyline(*profile, close=True)
            make_face()
        revolve(axis=Axis.Z)
    return Pos(tip_l, 0.0, 0.0) * Rotation(0, -90, 0) * boot.part


def make_servo_body() -> object:
    """FEETECH STS3215 visual envelope as BREP primitives."""
    body = _box(
        (hp.SERVO_BODY_W, hp.SERVO_BODY_D, hp.SERVO_BODY_H),
        (0.0, 0.0, hp.SERVO_BODY_H / 2.0),
    )
    out_boss_h = 2.0
    coupling = _cyl_z(
        (hp.SERVO_OUTPUT_BORE_OD - 1.0) / 2.0,
        out_boss_h,
        (hp.SERVO_OUTPUT_X, 0.0, hp.SERVO_BODY_H - out_boss_h / 2.0),
    )
    spline = _cyl_z(
        hp.SERVO_SPLINE_OD / 2.0,
        out_boss_h,
        (hp.SERVO_OUTPUT_X, 0.0, hp.SERVO_BODY_H - out_boss_h / 2.0),
    )
    idler = _cyl_z(
        (hp.SERVO_OUTPUT_BORE_OD - 1.0) / 2.0,
        1.5,
        (hp.SERVO_OUTPUT_X, 0.0, -0.75),
    )
    boot = _box(
        (hp.WIRE_BOOT_PROTRUSION, hp.WIRE_BOOT_W, hp.WIRE_BOOT_H),
        (
            hp.SERVO_BODY_W / 2.0 + hp.WIRE_BOOT_PROTRUSION / 2.0,
            0.0,
            hp.WIRE_BOOT_Z_BASE + hp.WIRE_BOOT_H / 2.0,
        ),
    )
    port_a = _box(
        (6.0, 4.0, 2.0),
        (hp.STS3215_PORT_X_MM, 3.5, hp.STS3215_PORT_Z_MM),
    )
    port_b = _box(
        (6.0, 4.0, 2.0),
        (hp.STS3215_PORT_X_MM, -3.5, hp.STS3215_PORT_Z_MM),
    )
    return _union(body, coupling, spline, idler, boot, port_a, port_b)


def make_servo_clamp_cap() -> object:
    """Bolt-on servo clamp cap, ported from the mesh helper."""
    body_face_y = hp.SERVO_BODY_D / 2.0
    wall_end_y = hp.WELL_D / 2.0
    flange_y0 = wall_end_y
    flange_y1 = wall_end_y + hp.CLAMP_CAP_T
    cap_z1 = hp.WELL_RIM_Z + hp.WELL_LIP_SLIDE_CL
    cav_w = hp.SERVO_BODY_W + 2 * hp.WELL_BODY_CL - hp.WELL_INSIDE_X_TIGHTEN

    flange = _box(
        (hp.WELL_W, flange_y1 - flange_y0, cap_z1),
        (0.0, 0.5 * (flange_y0 + flange_y1), cap_z1 / 2.0),
    )
    tongue_y0 = body_face_y - hp.CLAMP_TONGUE_INTERF
    tongue_z0 = -hp.CLAMP_SEAT_DROP
    tongue = _box(
        (cav_w, wall_end_y - tongue_y0, cap_z1 - tongue_z0),
        (
            0.0,
            0.5 * (tongue_y0 + wall_end_y),
            0.5 * (tongue_z0 + cap_z1),
        ),
    )
    cav_d = hp.SERVO_BODY_D + 2 * hp.WELL_BODY_CL
    lip_y0 = cav_d / 2.0 - 0.01
    lip_y1 = wall_end_y
    lip = _box(
        (cav_w, lip_y1 - lip_y0, hp.WELL_H - hp.WELL_RIM_Z),
        (
            0.0,
            0.5 * (lip_y0 + lip_y1),
            0.5 * (hp.WELL_RIM_Z + hp.WELL_H),
        ),
    )
    # Back-face hook + horn-side mini hook + yoke-sweep edge chamfers
    # (Aug 18-19 2026 production features; same solids as the mesh builder).
    hook_wall = _box(
        (hp.CLAMP_BACK_HOOK_X1 - hp.CLAMP_BACK_HOOK_X0,
         flange_y1 - tongue_y0, hp.CLAMP_BACK_HOOK_T + 1.0),
        (0.5 * (hp.CLAMP_BACK_HOOK_X0 + hp.CLAMP_BACK_HOOK_X1),
         0.5 * (tongue_y0 + flange_y1),
         0.5 * (1.0 - hp.CLAMP_BACK_HOOK_T)),
    )
    shelf_y1 = tongue_y0 + 0.5
    hook_shelf = _box(
        (hp.CLAMP_BACK_HOOK_X1 - hp.CLAMP_BACK_HOOK_X0,
         shelf_y1 - hp.CLAMP_BACK_HOOK_Y0,
         hp.CLAMP_BACK_HOOK_T - hp.CLAMP_BACK_HOOK_SHELF_Z),
        (0.5 * (hp.CLAMP_BACK_HOOK_X0 + hp.CLAMP_BACK_HOOK_X1),
         0.5 * (hp.CLAMP_BACK_HOOK_Y0 + shelf_y1),
         -0.5 * (hp.CLAMP_BACK_HOOK_SHELF_Z + hp.CLAMP_BACK_HOOK_T)),
    )
    horn_hook = _box(
        (cav_w / 2.0 - hp.CLAMP_HORN_HOOK_X0,
         flange_y1 - hp.CLAMP_HORN_HOOK_Y0,
         hp.CLAMP_SEAT_DROP + 1.0),
        (0.5 * (hp.CLAMP_HORN_HOOK_X0 + cav_w / 2.0),
         0.5 * (hp.CLAMP_HORN_HOOK_Y0 + flange_y1),
         0.5 * (1.0 - hp.CLAMP_SEAT_DROP)),
    )
    body = _union(flange, tongue, lip, hook_wall, hook_shelf, horn_hook)
    c = hp.CLAMP_YOKE_EDGE_CHAMFER
    chamfers = [
        Pos(ex, flange_y1, 0.0)
        * Rotation(0, 0, 45)
        * _box((c * math.sqrt(2.0), c * math.sqrt(2.0), ez1 - ez0),
               (0.0, 0.0, 0.5 * (ez0 + ez1)))
        for (ex, ez0, ez1) in (
            (-hp.WELL_W / 2.0, -1.0, cap_z1 + 1.0),
            (hp.WELL_W / 2.0, -1.0, cap_z1 + 1.0),
            (hp.CLAMP_BACK_HOOK_X0, -hp.CLAMP_BACK_HOOK_T - 1.0, 0.0),
            (hp.CLAMP_BACK_HOOK_X1, -hp.CLAMP_BACK_HOOK_T - 1.0, 0.0),
            (hp.CLAMP_HORN_HOOK_X0, -hp.CLAMP_SEAT_DROP - 1.0, 0.0),
            (cav_w / 2.0, -hp.CLAMP_SEAT_DROP - 1.0, 0.0),
        )
    ]
    cuts = [
        _cyl_z(
            hp.HORN_CLEAR_OPENING_OD / 2.0,
            (hp.WELL_H - hp.WELL_RIM_Z) * 4.0,
            (hp.SERVO_OUTPUT_X, 0.0, hp.WELL_RIM_Z),
        ),
        *chamfers,
    ]
    for bx, bz in hp.servo_clamp_bolt_centres():
        cuts.append(
            _cyl_y(
                hp.CLAMP_BOLT_CLEAR_OD / 2.0,
                (flange_y1 - flange_y0) + 4.0,
                (bx, 0.5 * (flange_y0 + flange_y1), bz),
            )
        )
        cb_h = hp.CLAMP_HEAD_CB_DEPTH + 1.0
        cuts.append(
            _cyl_y(
                hp.CLAMP_HEAD_CB_OD / 2.0,
                cb_h,
                (bx, flange_y1 - hp.CLAMP_HEAD_CB_DEPTH + cb_h / 2.0, bz),
            )
        )
    return _diff(body, *cuts)


def _wire_exit_l_corridor() -> object:
    slot_x_min = hp.SERVO_BODY_W / 2.0 - hp.WIRE_SLOT_X_INBOARD
    slot_x_max = hp.WELL_W / 2.0 + hp.WIRE_SLOT_X_PAST_WALL
    slot_z_bottom = -hp.WELL_FLOOR_T - hp.WIRE_SLOT_Z_BELOW_FLOOR
    slot_z_top = hp.WIRE_SLOT_DEPTH
    return _box(
        (slot_x_max - slot_x_min, hp.WIRE_SLOT_W, slot_z_top - slot_z_bottom),
        (
            0.5 * (slot_x_min + slot_x_max),
            0.0,
            0.5 * (slot_z_bottom + slot_z_top),
        ),
    )


def _boot_clearance_channel() -> object:
    ch_x_min = hp.SERVO_BODY_W / 2.0 - hp.WIRE_SLOT_X_INBOARD
    ch_x_max = hp.SERVO_BODY_W / 2.0 + hp.WELL_BODY_CL + hp.WIRE_CHANNEL_DEPTH
    ch_z_bottom = 0.0
    ch_z_top = hp.WELL_RIM_Z + hp.WIRE_CHANNEL_TOP_OVER_RIM
    return _box(
        (ch_x_max - ch_x_min, hp.WIRE_SLOT_W, ch_z_top - ch_z_bottom),
        (
            0.5 * (ch_x_min + ch_x_max),
            0.0,
            0.5 * (ch_z_bottom + ch_z_top),
        ),
    )


def _wire_exit_slot() -> object:
    return _union(_wire_exit_l_corridor(), _boot_clearance_channel())


def _servo_well_solid(*, end_face_bolts: bool = True) -> object:
    """Shared STS3215 sandwich cradle as BREP."""
    outer = _box((hp.WELL_W, hp.WELL_D, hp.WELL_H),
                 (0.0, 0.0, hp.WELL_H / 2.0))
    cuts = []

    cav_z_bot = -1.0
    cav_z_top = hp.WELL_RIM_Z + hp.WELL_LIP_SLIDE_CL
    cav_w = hp.SERVO_BODY_W + 2.0 * hp.WELL_BODY_CL - hp.WELL_INSIDE_X_TIGHTEN
    cav_d = hp.SERVO_BODY_D + 2.0 * hp.WELL_BODY_CL
    cuts.append(
        _box(
            (cav_w, cav_d, cav_z_top - cav_z_bot),
            (0.0, 0.0, 0.5 * (cav_z_top + cav_z_bot)),
        )
    )

    open_y0 = cav_d / 2.0 - 0.01
    open_y1 = hp.WELL_D / 2.0 + 1.0
    cuts.append(
        _box(
            (cav_w, open_y1 - open_y0, hp.WELL_H + 2.0),
            (0.0, 0.5 * (open_y0 + open_y1), hp.WELL_H / 2.0),
        )
    )
    cuts.append(
        _cyl_z(
            hp.HORN_CLEAR_OPENING_OD / 2.0,
            hp.WELL_PLATE_T * 4.0,
            (hp.SERVO_OUTPUT_X, 0.0, hp.WELL_RIM_Z),
        )
    )

    # Sep 2026 service extraction: continue the circular horn opening into
    # the +Y clamp mouth at the same width.  This is the BREP twin of
    # ``hexapod_prototype._servo_well_solid`` and lets the servo, fitted horn
    # and attached clamp cap lift out together.
    service_y0 = 0.0
    service_y1 = hp.WELL_D / 2.0 + 1.0
    cuts.append(
        _box(
            (hp.HORN_SERVICE_SLOT_W, service_y1 - service_y0,
             hp.WELL_PLATE_T * 4.0),
            (hp.SERVO_OUTPUT_X, 0.5 * (service_y0 + service_y1),
             hp.WELL_RIM_Z),
        )
    )

    for fx, fy in hp.servo_holder_front_case_hole_centres():
        cuts.append(
            _cyl_z(
                hp.FRONT_CASE_SCREW_OD / 2.0,
                hp.WELL_PLATE_T + 4.0,
                (fx, fy, hp.WELL_RIM_Z + hp.WELL_PLATE_T / 2.0),
            )
        )
        cuts.append(
            _cyl_z(
                hp.FRONT_CASE_CBORE_OD / 2.0,
                hp.FRONT_CASE_CBORE_DEPTH + 1.0,
                (fx, fy, hp.WELL_H - (hp.FRONT_CASE_CBORE_DEPTH - 1.0) / 2.0),
            )
        )

    pilot_y = hp.WELL_D / 2.0 - hp.CLAMP_BOLT_PILOT_DEPTH / 2.0 + 1.0
    for bx, bz in hp.servo_clamp_bolt_centres():
        cuts.append(
            _cyl_y(
                hp.CLAMP_BOLT_PILOT_OD / 2.0,
                hp.CLAMP_BOLT_PILOT_DEPTH + 2.0,
                (bx, pilot_y, bz),
            )
        )

    if end_face_bolts:
        body_face_x = -hp.SERVO_BODY_W / 2.0
        wall_outer_x = -hp.WELL_W / 2.0
        head_plane_x = body_face_x - hp.SERVO_BODY_BOLT_STANDOFF
        cl_outer = wall_outer_x - 1.0
        cl_inner = body_face_x + hp.SERVO_MOUNT_THREAD_DEPTH
        for by, bz in hp.servo_end_face_bolt_centres():
            cuts.append(
                _cyl_x(
                    hp.SERVO_BODY_BOLT_OD / 2.0,
                    cl_inner - cl_outer,
                    (0.5 * (cl_outer + cl_inner), by, bz),
                )
            )
            cuts.append(
                _cyl_x(
                    hp.SERVO_BODY_BOLT_HEAD_OD / 2.0,
                    head_plane_x - cl_outer,
                    (0.5 * (cl_outer + head_plane_x), by, bz),
                )
            )

    return _diff(outer, *cuts)


def _sandwich_fixed_side(*,
                         end_face_bolts: bool = True,
                         farwall_pad: bool = False,
                         rear_tab: bool = False,
                         wire_exit: bool = True) -> object:
    body = _servo_well_solid(end_face_bolts=end_face_bolts)
    if farwall_pad:
        pad_x0 = hp.WELL_W / 2.0 - 1.0
        pad_x1 = hp.WELL_W / 2.0 + hp.FEMUR_KNEE_FARWALL_PAD_T
        pad_z1 = hp.WELL_RIM_Z
        body = _union(
            body,
            _box(
                (pad_x1 - pad_x0, 2.0 * hp.FEMUR_KNEE_FARWALL_PAD_HALF_Y, pad_z1),
                (0.5 * (pad_x0 + pad_x1), 0.0, pad_z1 / 2.0),
            ),
        )
    cuts = [_wire_exit_slot()] if wire_exit else []
    if rear_tab:
        # Aug 2026 rear retention tab (production `_sandwich_fixed_side`):
        # plate + fusing riser under the -X wall, tongue-relief shelf, and
        # 2x M2.5 self-tap holes with head-recess counterbores.
        tab_x0 = -hp.WELL_W / 2.0
        tab_y0 = -hp.WELL_D / 2.0
        plate = _box(
            (hp.FEMUR_REAR_TAB_X1 - tab_x0,
             hp.FEMUR_REAR_TAB_Y1 - tab_y0, hp.FEMUR_REAR_TAB_T),
            (0.5 * (tab_x0 + hp.FEMUR_REAR_TAB_X1),
             0.5 * (tab_y0 + hp.FEMUR_REAR_TAB_Y1),
             -hp.FEMUR_REAR_TAB_T / 2.0),
        )
        wall_in_x = -(hp.SERVO_BODY_W / 2.0 + hp.WELL_BODY_CL)
        riser = _box(
            (wall_in_x - tab_x0, hp.FEMUR_REAR_TAB_Y1 - tab_y0,
             hp.FEMUR_REAR_TAB_FUSE_Z),
            (0.5 * (tab_x0 + wall_in_x),
             0.5 * (tab_y0 + hp.FEMUR_REAR_TAB_Y1),
             hp.FEMUR_REAR_TAB_FUSE_Z / 2.0),
        )
        body = _union(body, plate, riser)
        tongue_y0 = hp.SERVO_BODY_D / 2.0 - hp.CLAMP_TONGUE_INTERF
        cuts.append(_box(
            (abs(wall_in_x - (hp.FEMUR_REAR_TAB_X1 + 0.5)) + 1.0,
             (hp.FEMUR_REAR_TAB_Y1 + 2.0) - (tongue_y0 - 0.25),
             hp.CLAMP_SEAT_DROP + 0.25),
            (0.5 * (wall_in_x + hp.FEMUR_REAR_TAB_X1 + 0.5),
             0.5 * ((tongue_y0 - 0.25) + hp.FEMUR_REAR_TAB_Y1 + 2.0),
             -(hp.CLAMP_SEAT_DROP + 0.25) / 2.0),
        ))
        hx = hp.SADDLE_CASE_HOLE_X2 + hp.SERVO_OUTPUT_X
        for sy in (+1.0, -1.0):
            cuts.append(_cyl_z(
                hp.SADDLE_CASE_SCREW_OD / 2.0, hp.FEMUR_REAR_TAB_T + 4.0,
                (hx, sy * hp.SADDLE_CASE_HOLE_Y, -hp.FEMUR_REAR_TAB_T / 2.0),
            ))
            cb_h = hp.FEMUR_REAR_TAB_HEAD_CB + 1.0
            cuts.append(_cyl_z(
                hp.FEMUR_REAR_TAB_HEAD_CB_OD / 2.0, cb_h,
                (hx, sy * hp.SADDLE_CASE_HOLE_Y,
                 -hp.FEMUR_REAR_TAB_SHANK_T - cb_h / 2.0),
            ))
    return _diff(body, *cuts)


def _disc_horn_bolt_centres() -> list[tuple[float, float]]:
    r = hp.DISC_HORN_BOLT_PCD / 2.0
    return [
        (hp.SERVO_OUTPUT_X + r * math.cos(t), r * math.sin(t))
        for t in hp.DISC_HORN_BOLT_ANGLES_RAD
    ]


def _leg_tube_socket_x(x_mouth: float,
                       sock_z: float,
                       *,
                       direction: int = 1,
                       length: float | None = None,
                       pin_inset: float | None = None) -> tuple[object, object, object]:
    sock_len = length if length is not None else (hp.LEG_TUBE_SOCKET_DEPTH + 6.0)
    inset = pin_inset if pin_inset is not None else hp.LEG_TUBE_PIN_INSET
    s = float(direction)
    boss = _cyl_x(
        hp.LEG_TUBE_OD / 2.0 + hp.LEG_TUBE_SOCKET_WALL,
        sock_len,
        (x_mouth + s * sock_len / 2.0, 0.0, sock_z),
    )
    bore = _cyl_x(
        hp.LEG_TUBE_OD / 2.0 + hp.LEG_TUBE_SOCKET_CLEAR,
        sock_len * 2.0,
        (x_mouth + s * sock_len, 0.0, sock_z),
    )
    pin = _cyl_y(
        hp.LEG_TUBE_PIN_OD / 2.0,
        (hp.LEG_TUBE_OD / 2.0 + hp.LEG_TUBE_SOCKET_WALL) * 4.0,
        (x_mouth + s * inset, 0.0, sock_z),
    )
    return boss, bore, pin


def _sandwich_moving_yoke(*,
                          tube_socket: bool = True,
                          socket_length: float | None = None,
                          socket_pin_inset: float | None = None,
                          socket_pin: bool = True,
                          spine_extra_t: float = 0.0,
                          pad_extra_reach: float = 0.0) -> object:
    arm_t = hp._YOKE_ARM_T
    reach = hp.YOKE_ARM_PAD + hp.YOKE_SEAT_INTERF + pad_extra_reach

    def _disc_arm(seat_z: float, arm_dir: int) -> object:
        d = float(arm_dir)
        arm = _box(
            (hp._YOKE_ARM_X1 - hp._YOKE_ARM_X0, 2.0 * hp._YOKE_ARM_Y, arm_t),
            (
                0.5 * (hp._YOKE_ARM_X0 + hp._YOKE_ARM_X1),
                0.0,
                seat_z + d * arm_t / 2.0,
            ),
        )
        pad = _cyl_z(
            hp.DISC_HORN_OD / 2.0 - 0.5 + hp.YOKE_PAD_RADIAL_GROW,
            reach,
            (hp.SERVO_OUTPUT_X, 0.0, seat_z - d * reach / 2.0),
        )
        body = _union(arm, pad)
        cuts = []
        for hx, hy in _disc_horn_bolt_centres():
            cuts.append(
                _cyl_z(
                    hp.DISC_HORN_BOLT_OD / 2.0,
                    arm_t * 4.0 + 2.0 * reach,
                    (hx, hy, seat_z + d * arm_t / 2.0),
                )
            )
        cuts.append(
            _cyl_z(
                hp.DISC_HORN_COLLAR_OD / 2.0 + 0.25,
                hp.DISC_HORN_COLLAR_DEPTH + 1.0,
                (
                    hp.SERVO_OUTPUT_X,
                    0.0,
                    (seat_z - d * reach)
                    + d * (hp.DISC_HORN_COLLAR_DEPTH + 1.0) / 2.0,
                ),
            )
        )
        cuts.append(
            _cyl_z(
                hp.HORN_CENTRE_OD / 2.0,
                arm_t * 4.0 + 2.0 * reach + 4.0,
                (hp.SERVO_OUTPUT_X, 0.0, seat_z + d * arm_t / 2.0),
            )
        )
        return _diff(body, *cuts)

    top = _disc_arm(hp.JOINT_HORN_TOP_Z, +1)
    bot = _disc_arm(hp.JOINT_HORN_BOT_Z, -1)
    spine_x1 = hp._YOKE_SPINE_X1 + spine_extra_t
    spine = _box(
        (
            spine_x1 - hp._YOKE_SPINE_X0,
            2.0 * hp._YOKE_ARM_Y,
            (hp.JOINT_HORN_TOP_Z + arm_t) - hp._YOKE_BOT_Z0,
        ),
        (
            0.5 * (hp._YOKE_SPINE_X0 + spine_x1),
            0.0,
            0.5 * (hp._YOKE_BOT_Z0 + hp.JOINT_HORN_TOP_Z + arm_t),
        ),
    )
    parts = [top, bot, spine]
    if tube_socket:
        boss, bore, pin = _leg_tube_socket_x(
            hp._YOKE_SOCKET_X,
            hp.JOINT_SOCKET_Z,
            length=socket_length,
            pin_inset=socket_pin_inset,
        )
        parts.append(boss)
        cuts = [bore] + ([pin] if socket_pin else [])
        return _diff(_union(*parts), *cuts)
    return _union(*parts)


def make_tibia_knee_yoke() -> object:
    """Knee-end moving yoke with the CF-tube socket, as BREP."""
    return _sandwich_moving_yoke(
        tube_socket=True,
        socket_pin=False,
        spine_extra_t=hp.TIBIA_YOKE_SPINE_PAD_T,
        pad_extra_reach=hp.YOKE_PAD_EXTRA_REACH,
    )


def _femur_fused_spar() -> object:
    length = hp.FEMUR_SPAR_LEN + hp._FEMUR_SPAR_WALL_BITE
    spar = _cyl_x(
        hp.FEMUR_SPAR_OD / 2.0,
        length,
        (hp._YOKE_SOCKET_X + length / 2.0, 0.0, hp.JOINT_SOCKET_Z),
    )
    spine_face = hp._YOKE_SPINE_X1 + hp.FEMUR_YOKE_SPINE_PAD_T
    wall_face = hp._YOKE_SOCKET_X + hp.FEMUR_SPAR_LEN
    cone_h = (wall_face - spine_face) + hp._FEMUR_SPAR_WALL_BITE
    gussets = []
    for base_x, direction, base_r in (
        (
            spine_face - hp._FEMUR_SPAR_WALL_BITE,
            +1,
            hp.FEMUR_GUSSET_R_HIP,
        ),
        (
            wall_face + hp._FEMUR_SPAR_WALL_BITE,
            -1,
            hp.FEMUR_GUSSET_R_KNEE,
        ),
    ):
        if base_r > 0.0:
            gussets.append(
                _cone_x_from_base(
                    base_r,
                    cone_h,
                    base_x=base_x,
                    y=0.0,
                    z=hp.JOINT_SOCKET_Z,
                    direction=direction,
                )
            )
    return _union(spar, *gussets)


def _femur_knee_fixed_solid() -> object:
    return _sandwich_fixed_side(end_face_bolts=False, farwall_pad=True,
                                rear_tab=True, wire_exit=False)


def make_femur_link_part() -> object:
    yoke = _sandwich_moving_yoke(
        tube_socket=False,
        spine_extra_t=hp.FEMUR_YOKE_SPINE_PAD_T,
        pad_extra_reach=hp.YOKE_PAD_EXTRA_REACH,
    )
    spar = _femur_fused_spar()
    knee = Pos(hp.FEMUR_LENGTH, 0.0, 0.0) * _femur_knee_fixed_solid()
    return _union(yoke, spar, knee)


def _joint_pitch_place(part: object,
                       mount: tuple[float, float, float]) -> object:
    """Production _joint_place for x_dir=(1,0,0), z_dir=LEG_PITCH_AXIS."""
    tx = mount[0] - hp.SERVO_OUTPUT_X
    ty = mount[1] + hp.JOINT_HORN_TOP_Z
    tz = mount[2]
    return Pos(tx, ty, tz) * Rotation(90, 0, 0) * part


def make_femur_link() -> object:
    return _joint_pitch_place(make_femur_link_part(), (0.0, 0.0, 0.0))


def make_yaw_servo_retainer() -> object:
    """Yaw anti-rotation saddle with four corner feet, as BREP."""
    plate_bot = hp.CHASSIS_SPLIT_Z - hp.CHASSIS_BOTTOM_FLOOR_T
    servo_back_z = hp.yaw_servo_real_back_z()

    bx_c = -hp.SERVO_OUTPUT_X
    x_in = bx_c - hp.SERVO_BODY_W / 2.0
    x_out = bx_c + hp.SERVO_BODY_W / 2.0
    yf = hp.SERVO_BODY_D / 2.0
    yi = yf + hp.SADDLE_BODY_CL
    yo = yi + hp.SADDLE_WALL_T
    xi = x_in - hp.SADDLE_BODY_CL
    xo = xi - hp.SADDLE_WALL_T

    wall_z0 = servo_back_z
    wall_z1 = plate_bot
    parts = []
    for sgn in (-1.0, +1.0):
        parts.append(
            _box(
                (x_out - xi, hp.SADDLE_WALL_T, wall_z1 - wall_z0),
                (
                    0.5 * (xi + x_out),
                    sgn * 0.5 * (yi + yo),
                    0.5 * (wall_z0 + wall_z1),
                ),
            )
        )
    parts.append(
        _box(
            (hp.SADDLE_WALL_T, 2.0 * yo, wall_z1 - wall_z0),
            (0.5 * (xo + xi), 0.0, 0.5 * (wall_z0 + wall_z1)),
        )
    )

    fl_z1 = servo_back_z
    fl_z0 = fl_z1 - hp.SADDLE_FLOOR_T
    parts.append(
        _box(
            (hp.SADDLE_FLOOR_X_OUT - xo, 2.0 * yo, hp.SADDLE_FLOOR_T),
            (
                0.5 * (xo + hp.SADDLE_FLOOR_X_OUT),
                0.0,
                0.5 * (fl_z0 + fl_z1),
            ),
        )
    )

    flange_z0 = plate_bot - hp.SADDLE_FLANGE_T
    flange_z1 = plate_bot
    for ax, ay in hp.chassis_lower_retainer_anchor_centres():
        parts.append(
            _cyl_z(
                hp.SADDLE_ANCHOR_PAD_R,
                hp.SADDLE_FLANGE_T,
                (ax, ay, 0.5 * (flange_z0 + flange_z1)),
            )
        )

    tip_z = plate_bot - hp.RETAINER_FOOT_H
    pad_top_z = tip_z + hp.RETAINER_PAD_H

    def _corner_pad(px: float, py: float) -> object:
        pad = _cyl_z(
            hp.RETAINER_PAD_OD / 2.0,
            hp.RETAINER_PAD_H,
            (px, py, tip_z + hp.RETAINER_PAD_H / 2.0),
        )
        ch = hp.RETAINER_PAD_CHAMFER
        ring = _cyl_z(
            hp.RETAINER_PAD_OD / 2.0 + 0.2,
            ch,
            (px, py, tip_z + ch / 2.0),
        )
        keep = _cyl_z(
            hp.RETAINER_PAD_OD / 2.0 - ch,
            ch + 0.2,
            (px, py, tip_z + ch / 2.0),
        )
        return _diff(pad, _diff(ring, keep))

    pw = hp.RETAINER_POLE_W
    fx, fy = hp.RETAINER_POLE_FRONT
    for sgn in (-1.0, +1.0):
        parts.append(
            _box(
                (pw, pw, plate_bot - pad_top_z),
                (fx, sgn * fy, 0.5 * (plate_bot + pad_top_z)),
            )
        )
        parts.append(_corner_pad(fx, sgn * fy))

    rx, ry = hp.RETAINER_POLE_REAR
    gz1 = plate_bot - 18.0
    gx0, gx1 = rx, -34.0
    gy0, gy1 = 14.0, ry
    for sgn in (-1.0, +1.0):
        parts.append(
            _box(
                (gx1 - gx0, gy1 - gy0, gz1 - fl_z0),
                (
                    0.5 * (gx0 + gx1),
                    sgn * 0.5 * (gy0 + gy1),
                    0.5 * (gz1 + fl_z0),
                ),
            )
        )
        parts.append(
            _box(
                (pw, pw, plate_bot - pad_top_z),
                (rx, sgn * ry, 0.5 * (plate_bot + pad_top_z)),
            )
        )
        parts.append(_corner_pad(rx, sgn * ry))

    saddle = _union(*parts)

    win_x0 = xo + hp.SADDLE_FLOOR_RIM
    win_x1 = hp.SADDLE_FLOOR_X_OUT + 1.0
    win_y = yo - hp.SADDLE_FLOOR_RIM
    cuts = [
        _box(
            (win_x1 - win_x0, 2.0 * win_y, hp.SADDLE_FLOOR_T + 2.0),
            (0.5 * (win_x0 + win_x1), 0.0, 0.5 * (fl_z0 + fl_z1)),
        )
    ]
    for ax, ay in hp.chassis_lower_retainer_anchor_centres():
        cuts.append(
            _cyl_z(
                hp.RETAINER_BOLT_CLEAR_OD / 2.0,
                hp.SADDLE_FLANGE_T + 2.0,
                (ax, ay, 0.5 * (flange_z0 + flange_z1)),
            )
        )
        cb_h = hp.SADDLE_HEAD_CB_DEPTH + 1.0
        cuts.append(
            _cyl_z(
                hp.SADDLE_HEAD_CB_OD / 2.0,
                cb_h,
                (ax, ay, flange_z0 + hp.SADDLE_HEAD_CB_DEPTH - cb_h / 2.0),
            )
        )
    body = _diff(saddle, *cuts)

    zc = servo_back_z
    boss_z0 = zc - hp.SADDLE_CASE_SHANK
    bosses = []
    boss_cuts = []
    for rx, ry in hp.yaw_rear_screw_centres():
        bosses.append(
            _cyl_z(
                hp.SADDLE_CASE_BOSS_R,
                hp.SADDLE_CASE_SHANK,
                (rx, ry, 0.5 * (boss_z0 + zc)),
            )
        )
        boss_cuts.append(
            _cyl_z(
                hp.SADDLE_CASE_SCREW_OD / 2.0,
                hp.SADDLE_CASE_SHANK + 2.0,
                (rx, ry, 0.5 * (boss_z0 + zc)),
            )
        )
    return _diff(_union(body, *bosses), *boss_cuts)


def make_yaw_bearing_cap() -> object:
    """Split yaw-bearing cap as a clean BREP."""
    r_out = hp.YAW_BEARING_OD / 2.0 + hp.YAW_TOWER_WALL
    r_bore = hp.YAW_TOWER_BORE_OD / 2.0
    split_z = hp.YAW_SPLIT_Z
    top_z = hp.YAW_CAP_TOP_Z
    rim_z = hp.YAW_CAP_RIM_Z
    ear_top = hp.YAW_CAP_EAR_TOP_Z

    cap = _cyl_z(r_out, rim_z - split_z, (0.0, 0.0, 0.5 * (split_z + rim_z)))
    for ex, ey in hp._yaw_cap_bolt_centres():
        cap = cap + _cyl_z(
            hp.YAW_CAP_BOLT_BOSS_OD / 2.0,
            ear_top - split_z,
            (ex, ey, 0.5 * (split_z + ear_top)),
        )

    cuts = [
        _cyl_z(r_bore, (top_z - split_z) + 0.02,
               (0.0, 0.0, 0.5 * (split_z + top_z))),
        _cyl_z(hp.YAW_CAP_LIP_ID / 2.0, (rim_z - top_z) + 0.04,
               (0.0, 0.0, 0.5 * (top_z + rim_z))),
    ]
    cb_floor = ear_top - (hp.INSERT_M3_BOLT_HEAD_H + 0.3)
    for ex, ey in hp._yaw_cap_bolt_centres():
        cuts.append(
            _cyl_z(
                hp.YAW_CAP_BOLT_OD / 2.0,
                (ear_top - split_z) + 2.0,
                (ex, ey, 0.5 * (split_z + ear_top)),
            )
        )
        cb_top = rim_z + 0.02
        cuts.append(
            _cyl_z(
                hp.YAW_CAP_BOLT_HEAD_OD / 2.0,
                cb_top - cb_floor,
                (ex, ey, 0.5 * (cb_floor + cb_top)),
            )
        )
    return _diff(cap, *cuts)


def make_yaw_bearing(which: str) -> object:
    """Simple two-ring visual bearing BREP."""
    bot = hp.YAW_BEARING_UPPER_BOT_Z if which == "upper" else hp.YAW_BEARING_LOWER_BOT_Z
    mid_z = bot + hp.YAW_BEARING_W / 2.0
    inner = _diff(
        _cyl_z(hp.YAW_BEARING_INNER_OD / 2.0, hp.YAW_BEARING_W,
               (0.0, 0.0, mid_z)),
        _cyl_z(hp.YAW_BEARING_ID / 2.0, hp.YAW_BEARING_W * 2.0,
               (0.0, 0.0, mid_z)),
    )
    outer = _diff(
        _cyl_z(hp.YAW_BEARING_OD / 2.0, hp.YAW_BEARING_W,
               (0.0, 0.0, mid_z)),
        _cyl_z(hp.YAW_BEARING_OUTER_ID / 2.0, hp.YAW_BEARING_W * 2.0,
               (0.0, 0.0, mid_z)),
    )
    return _union(inner, outer)


def _coxa_join_bolt_centres() -> list[tuple[float, float]]:
    r = hp.COXA_JOIN_BOLT_PCD / 2.0
    return [
        (r * math.cos(t), r * math.sin(t))
        for t in hp.COXA_JOIN_BOLT_ANGLES_RAD
    ]


def make_coxa_yaw_hub(*, one_piece: bool = False) -> object:
    rboss = hp.YAW_HUB_BOSS_OD / 2.0
    rinner = hp.YAW_BEARING_INNER_OD / 2.0

    uboss = _cyl_z(
        rboss,
        hp.YAW_HUB_BOSS_TOP_Z - hp.YAW_HUB_BOSS_WIDE_BOT_Z,
        (0.0, 0.0, 0.5 * (hp.YAW_HUB_BOSS_WIDE_BOT_Z + hp.YAW_HUB_BOSS_TOP_Z)),
    )
    drive_nub = _cyl_z(
        hp.YAW_HUB_DRIVE_NUB_OD / 2.0,
        hp.YAW_HUB_BOSS_WIDE_BOT_Z - hp.YAW_HUB_BOSS_BOT_Z,
        (0.0, 0.0, 0.5 * (hp.YAW_HUB_BOSS_BOT_Z + hp.YAW_HUB_BOSS_WIDE_BOT_Z)),
    )
    uflange = _cyl_z(
        rinner,
        2.0,
        (0.0, 0.0, hp.YAW_BEARING_UPPER_TOP_Z + 1.0),
    )
    plat_od = max(hp.YAW_HUB_OD, hp.YAW_HUB_DUST_LIP_OD)
    plat = _cyl_z(
        plat_od / 2.0,
        hp.YAW_HUB_PLATFORM_Z1 - hp.YAW_HUB_BOSS_TOP_Z,
        (0.0, 0.0, 0.5 * (hp.YAW_HUB_BOSS_TOP_Z + hp.YAW_HUB_PLATFORM_Z1)),
    )
    lip_inner_r = hp.YAW_BEARING_OD / 2.0 + hp.YAW_TOWER_WALL + hp.YAW_HUB_DUST_LIP_CL
    lip_z1 = hp.YAW_HUB_BOSS_TOP_Z
    lip_z0 = lip_z1 - 4.0
    lip = _diff(
        _cyl_z(hp.YAW_HUB_DUST_LIP_OD / 2.0, lip_z1 - lip_z0,
               (0.0, 0.0, 0.5 * (lip_z0 + lip_z1))),
        _cyl_z(lip_inner_r, (lip_z1 - lip_z0) * 3.0,
               (0.0, 0.0, 0.5 * (lip_z0 + lip_z1))),
    )
    hub = _union(uboss, drive_nub, uflange, plat, lip)

    cuts = []
    collar_h = hp.DISC_HORN_COLLAR_DEPTH + 1.0
    cuts.append(
        _cyl_z(
            hp.DISC_HORN_COLLAR_OD / 2.0 + 0.25,
            collar_h,
            (0.0, 0.0, hp.YAW_HUB_BOSS_BOT_Z + collar_h / 2.0),
        )
    )
    hub_h = (hp.YAW_HUB_PLATFORM_Z1 - hp.YAW_HUB_BOSS_BOT_Z) + 4.0
    cuts.append(
        _cyl_z(
            hp.HORN_CENTRE_OD / 2.0,
            hub_h,
            (0.0, 0.0, 0.5 * (hp.YAW_HUB_BOSS_BOT_Z + hp.YAW_HUB_PLATFORM_Z1)),
        )
    )
    cb_h = (hp.YAW_HUB_PLATFORM_Z1 - hp.YAW_HUB_HORN_HEAD_SEAT_Z) + 1.0
    cb_z = hp.YAW_HUB_HORN_HEAD_SEAT_Z + cb_h / 2.0
    cuts.append(_cyl_z(hp.YAW_HUB_HORN_HEAD_CB_OD / 2.0, cb_h, (0.0, 0.0, cb_z)))
    drive_clear = hp.DISC_HORN_BOLT_OD + 0.3
    r = hp.DISC_HORN_BOLT_PCD / 2.0
    for t in hp.DISC_HORN_BOLT_ANGLES_RAD:
        cx, cy = r * math.cos(t), r * math.sin(t)
        cuts.append(
            _cyl_z(
                drive_clear / 2.0,
                hp.YAW_HUB_PLATFORM_Z1 * 2.0 + 4.0,
                (cx, cy, hp.YAW_HUB_PLATFORM_Z1 / 2.0),
            )
        )
        cuts.append(_cyl_z(hp.YAW_HUB_HORN_HEAD_CB_OD / 2.0, cb_h, (cx, cy, cb_z)))

    if not one_piece:
        for jx, jy in _coxa_join_bolt_centres():
            cuts.append(
                _cyl_z(
                    hp.COXA_JOIN_PILOT_OD / 2.0,
                    hp.YAW_HUB_PAD_T,
                    (jx, jy, hp.YAW_HUB_PLATFORM_Z1 - hp.YAW_HUB_PAD_T / 2.0),
                )
            )
    return _diff(hub, *cuts)


def _coxa_part_a_envelope() -> object:
    env_h = hp.YAW_HUB_PLATFORM_Z1 + 6.0
    env = _cyl_z(
        max(hp.YAW_HUB_OD, hp.YAW_HUB_DUST_LIP_OD) / 2.0 + 0.6,
        env_h,
        (0.0, 0.0, env_h / 2.0 - 6.0),
    )
    lip_h = hp.YAW_HUB_BOSS_TOP_Z + 6.0
    lip = _cyl_z(
        hp.YAW_HUB_DUST_LIP_OD / 2.0 + 0.6,
        lip_h,
        (0.0, 0.0, lip_h / 2.0 - 6.0),
    )
    return _union(env, lip)


def make_coxa_hip_bracket(*, one_piece: bool = False) -> object:
    if one_piece:
        foot_z0 = hp.YAW_HUB_BOSS_TOP_Z
        foot_z1 = hp.YAW_HUB_PLATFORM_Z1 + hp.COXA_WELL_FLOOR_LIFT
    else:
        foot_z0 = hp.YAW_HUB_PLATFORM_Z1
        foot_z1 = hp.YAW_HUB_PLATFORM_Z1 + hp.YAW_HUB_PAD_T

    # Production hip cradle (Aug 17 2026): no end-face bolts, no wire-exit
    # corridor, rear retention tab.  The foot slab is sized from the NO-TAB
    # cradle so it does not grow under the tab into the yoke-arm sweep band.
    fixed = _joint_pitch_place(
        _sandwich_fixed_side(end_face_bolts=False, wire_exit=False,
                             rear_tab=True),
        hp.COXA_HIP_ANCHOR)
    fixed_notab = _joint_pitch_place(
        _sandwich_fixed_side(end_face_bolts=False, wire_exit=False),
        hp.COXA_HIP_ANCHOR)
    fb = fixed_notab.bounding_box()
    foot_x0 = float(fb.min.X) - 1.0
    foot_x1 = float(fb.max.X) + 1.0
    foot_y0 = float(fb.min.Y) - 1.0
    foot_y1 = float(fb.max.Y) + 1.0
    foot = _box(
        (foot_x1 - foot_x0, foot_y1 - foot_y0, foot_z1 - foot_z0),
        (
            0.5 * (foot_x0 + foot_x1),
            0.5 * (foot_y0 + foot_y1),
            0.5 * (foot_z0 + foot_z1),
        ),
    )
    back_z0 = float(fb.min.Z)
    if back_z0 > foot_z1:
        ped = _box(
            (foot_x1 - foot_x0, foot_y1 - foot_y0, back_z0 - foot_z1 + 0.5),
            (
                0.5 * (foot_x0 + foot_x1),
                0.5 * (foot_y0 + foot_y1),
                0.5 * (foot_z1 + back_z0 + 0.5),
            ),
        )
        body = _union(foot, ped, fixed)
    else:
        body = _union(foot, fixed)

    if not one_piece:
        body = _diff(body, _coxa_part_a_envelope())

    # Femur-swing clearance (production FEMUR_CLEAR_X/Z locals): the Z
    # threshold rides with the hip axis as COXA_HIP_DROP - 24.9 (it was
    # bench-derived as 18.5 back when the drop was 43.4 -- hardcoding
    # 18.5 here cut a phantom 0.5 mm step into the slab underside).
    femur_clear_x = 27.5
    femur_clear_z = hp.COXA_HIP_DROP - 24.9
    body = _diff(
        body,
        _box(
            (40.0, foot_y1 - foot_y0 + 4.0, femur_clear_z - 4.0),
            (
                femur_clear_x + 20.0,
                0.5 * (foot_y0 + foot_y1),
                0.5 * (4.0 + femur_clear_z),
            ),
        ),
    )

    chamfer = hp.LIP_RELIEF_CHAMFER
    normal = np.array([0.0, -1.0, -1.0]) / math.sqrt(2.0)
    mid = np.array([0.0, foot_y0 + chamfer / 2.0, foot_z0 + chamfer / 2.0])
    ctr = mid + normal * 25.0
    wedge = (
        Pos(0.0, float(ctr[1]), float(ctr[2]))
        * Rotation(-45.0, 0.0, 0.0)
        * _box((foot_x1 - foot_x0 + 2.0, 50.0, 50.0),
               (0.5 * (foot_x0 + foot_x1), 0.0, 0.0))
    )
    body = _diff(body, wedge)

    if one_piece:
        return body

    cuts = []
    for jx, jy in _coxa_join_bolt_centres():
        cuts.append(
            _cyl_z(
                hp.COXA_JOIN_BOLT_OD / 2.0,
                (foot_z1 - foot_z0) * 3.0,
                (jx, jy, foot_z0),
            )
        )
        cuts.append(
            _cyl_z(
                hp.INSERT_M3_BOLT_HEAD_OD / 2.0,
                hp.INSERT_M3_BOLT_HEAD_H + 0.5,
                (jx, jy, foot_z1 - (hp.INSERT_M3_BOLT_HEAD_H + 0.5) / 2.0),
            )
        )
    return _diff(body, *cuts)


def make_coxa_link() -> object:
    body = _union(
        make_coxa_yaw_hub(one_piece=True),
        make_coxa_hip_bracket(one_piece=True),
    )
    # Yoke-end sweep clearance (Aug 17 2026 production): Y-axis cylinders of
    # r 16.75 through the two femur yoke-arm bands about the hip axis.
    hip_ax_x, _, hip_ax_z = hp.COXA_HIP_ANCHOR
    sweeps = [
        _cyl_y(16.75, yhi - ylo, (hip_ax_x, 0.5 * (ylo + yhi), hip_ax_z))
        for (ylo, yhi) in ((21.75, 30.0), (-31.0, -24.75))
    ]
    # Head-access shafts: centre station has its own 1 mm-deeper seat.
    shaft_top_z = 80.0
    stations = [(0.0, 0.0, hp.YAW_HUB_HORN_CENTRE_SEAT_Z)]
    r = hp.DISC_HORN_BOLT_PCD / 2.0
    stations.extend(
        (r * math.cos(t), r * math.sin(t), hp.YAW_HUB_HORN_HEAD_SEAT_Z)
        for t in hp.DISC_HORN_BOLT_ANGLES_RAD
    )
    cuts = [
        _cyl_z(
            hp.YAW_HUB_HORN_HEAD_CB_OD / 2.0,
            shaft_top_z - seat_z,
            (sx, sy, 0.5 * (seat_z + shaft_top_z)),
        )
        for sx, sy, seat_z in stations
    ]
    return _diff(body, *sweeps, *cuts)


def _chassis_yaw_cradle_solid() -> object:
    body_centre_x = -hp.SERVO_OUTPUT_X
    out_z = hp.CHASSIS_YAW_OUTPUT_Z - hp.CHASSIS_PLATE_T / 2.0
    plate_top_z = out_z - hp.HORN_STACK_H
    front_face_z = plate_top_z - hp.WELL_PLATE_T
    body_back_z = front_face_z - hp.SERVO_BODY_H

    outer_w = hp.WELL_W + 2.0 + 2.0 * hp.CRADLE_BOND_STRIP_MM
    outer_d = hp.WELL_D + 2.0 + 2.0 * hp.CRADLE_BOND_STRIP_MM
    outer = _box(
        (outer_w, outer_d, plate_top_z - body_back_z),
        (body_centre_x, 0.0, 0.5 * (body_back_z + plate_top_z)),
    )

    cav_z_min = body_back_z - 5.0
    cav_z_max = front_face_z
    cavity = _box(
        (
            hp.SERVO_BODY_W + 2.0 * hp.WELL_BODY_CL,
            hp.SERVO_BODY_D + 2.0 * hp.WELL_BODY_CL,
            cav_z_max - cav_z_min,
        ),
        (body_centre_x, 0.0, 0.5 * (cav_z_min + cav_z_max)),
    )

    wire_z_min = -hp.CHASSIS_PLATE_T
    wire_z_max = 6.0
    wire_x_inner = body_centre_x + 5.0
    wire_x_outer = body_centre_x - outer_w / 2.0 - 2.0
    wire_slot = _box(
        (wire_x_inner - wire_x_outer, hp.WIRE_SLOT_W, wire_z_max - wire_z_min),
        (
            0.5 * (wire_x_inner + wire_x_outer),
            0.0,
            0.5 * (wire_z_min + wire_z_max),
        ),
    )

    anchor_pilots = [
        _cyl_z(
            hp.RETAINER_ANCHOR_PILOT_OD / 2.0,
            hp.RETAINER_ANCHOR_PILOT_DEPTH + 1.0,
            (
                ax,
                ay,
                body_back_z - 0.5 + (hp.RETAINER_ANCHOR_PILOT_DEPTH + 1.0) / 2.0,
            ),
        )
        for ax, ay in hp.yaw_retainer_anchor_centres()
    ]

    out_bore = _cyl_z(
        hp.HORN_CLEAR_OPENING_OD / 2.0,
        (plate_top_z + 1.0) - (front_face_z - 1.0),
        (0.0, 0.0, 0.5 * ((front_face_z - 1.0) + (plate_top_z + 1.0))),
    )

    boot_x_face = body_centre_x + hp.SERVO_BODY_W / 2.0
    boot_ch_x0 = boot_x_face - 1.0
    boot_ch_x1 = body_centre_x + outer_w / 2.0 + 2.0
    boot_seat_z0 = body_back_z + hp.WIRE_BOOT_Z_BASE
    boot_seat_z1 = boot_seat_z0 + hp.WIRE_BOOT_H
    boot_ch_z0 = body_back_z - 1.0
    boot_ch_z1 = boot_seat_z1 + 4.0
    boot_channel = _box(
        (boot_ch_x1 - boot_ch_x0, hp.WIRE_BOOT_W + 1.0, boot_ch_z1 - boot_ch_z0),
        (
            0.5 * (boot_ch_x0 + boot_ch_x1),
            0.0,
            0.5 * (boot_ch_z0 + boot_ch_z1),
        ),
    )

    body = _diff(
        outer,
        cavity,
        wire_slot,
        out_bore,
        boot_channel,
        *anchor_pilots,
    )

    clear_floor_z = front_face_z
    cut_h = (plate_top_z + 30.0) - clear_floor_z
    cut_z_cen = clear_floor_z + cut_h / 2.0
    relief = _diff(
        _box(
            (outer_w + 6.0, outer_d + 6.0, cut_h),
            (body_centre_x + (outer_w + 6.0) / 2.0, 0.0, cut_z_cen),
        ),
        _cyl_z(
            hp.YAW_BEARING_OD / 2.0 + hp.YAW_TOWER_WALL + 1.5,
            cut_h * 2.0,
            (0.0, 0.0, cut_z_cen),
        ),
    )
    body = _diff(body, relief)

    def _cz(coxa_z: float) -> float:
        return out_z + coxa_z

    r_out = hp.YAW_BEARING_OD / 2.0 + hp.YAW_TOWER_WALL
    r_bore = hp.YAW_TOWER_BORE_OD / 2.0
    r_shoulder = hp.YAW_TOWER_SHOULDER_OD / 2.0
    tower_bot = -(hp.CHASSIS_PLATE_T + hp.CHASSIS_BOTTOM_FLOOR_T)
    race_seat_z = _cz(hp.YAW_BEARING_LOWER_BOT_Z)
    tower_top = _cz(hp.YAW_SPLIT_Z)
    tower = _cyl_z(r_out, tower_top - tower_bot, (0.0, 0.0, 0.5 * (tower_bot + tower_top)))
    tower = _diff(
        tower,
        _cyl_z(
            r_bore,
            (tower_top + 0.02) - race_seat_z,
            (0.0, 0.0, 0.5 * (race_seat_z + tower_top + 0.02)),
        ),
    )

    relief_step_z = plate_top_z
    tower = _diff(
        tower,
        _cyl_z(
            r_shoulder,
            race_seat_z - (relief_step_z - 0.02),
            (0.0, 0.0, 0.5 * ((relief_step_z - 0.02) + race_seat_z)),
        ),
        _cyl_z(
            hp.HORN_CLEAR_OPENING_OD / 2.0,
            (relief_step_z + 0.02) - (tower_bot - 0.02),
            (0.0, 0.0, 0.5 * ((tower_bot - 0.02) + (relief_step_z + 0.02))),
        ),
    )

    ear_bot_z = front_face_z
    ear_top_z = tower_top
    for ang in hp.YAW_CAP_BOLT_ANGLES_RAD:
        ex = hp.YAW_CAP_BOLT_PCD / 2.0 * math.cos(ang)
        ey = hp.YAW_CAP_BOLT_PCD / 2.0 * math.sin(ang)
        boss = _cyl_z(
            hp.YAW_CAP_BOLT_BOSS_OD / 2.0,
            ear_top_z - ear_bot_z,
            (ex, ey, 0.5 * (ear_bot_z + ear_top_z)),
        )
        pilot = _cyl_z(
            hp.YAW_CAP_BOLT_PILOT_OD / 2.0,
            (ear_top_z - ear_bot_z) + 0.5,
            (ex, ey, ear_top_z - ((ear_top_z - ear_bot_z) + 0.5) / 2.0 + 0.25),
        )
        tower = _diff(_union(tower, boss), pilot)

    body = _union(body, tower)
    return _diff(body, cavity, out_bore, boot_channel, wire_slot)


def _chassis_bottom_full_solid() -> object:
    plate = _hex_plate(
        hp.CHASSIS_FLAT_TO_FLAT,
        hp.CHASSIS_PLATE_T,
        with_chassis_standoffs=True,
        with_leg_features=True,
        with_leg_harness_drops=True,
    )

    cradles = []
    for _i, edge_mid, _R, _R3 in hp._leg_chassis_frames():
        az = math.atan2(float(edge_mid[1]), float(edge_mid[0]))
        cradles.append(
            Pos(float(edge_mid[0]), float(edge_mid[1]), hp.CHASSIS_PLATE_T / 2.0)
            * Rotation(0, 0, math.degrees(az))
            * _chassis_yaw_cradle_solid()
        )
    plate = _union(plate, *cradles)

    drops = []
    for _i, edge_mid, _R, _R3 in hp._leg_chassis_frames():
        az = math.atan2(float(edge_mid[1]), float(edge_mid[0]))
        for sx, sy, sxe, sye in hp.leg_harness_drop_slots():
            drops.append(
                Pos(float(edge_mid[0]), float(edge_mid[1]), 0.0)
                * Rotation(0, 0, math.degrees(az))
                * _box((sxe, sye, hp.CHASSIS_PLATE_T * 4.0), (sx, sy, 0.0))
            )
    plate = _diff(plate, *drops)

    try:
        import cable_keepouts as ck  # noqa: WPS433
    except Exception:
        ck = None
    if ck is not None:
        corridor_cuts = []
        for keepout in ck.build_cable_keepouts():
            lo, hi = keepout.mesh.bounds
            pad = 3.5
            ext = (hi - lo) + 2.0 * pad
            corridor_cuts.append(
                _box(tuple(float(v) for v in ext),
                     tuple(float(v) for v in (lo + hi) / 2.0))
            )
        plate = _diff(plate, *corridor_cuts)

    through = []
    for strap_dx in hp.BATTERY_STRAP_DX:
        sx_centre = hp.BATTERY_HOLDER_CENTRE_X + strap_dx
        for sy in (-1.0, 1.0):
            through.append(
                _box(
                    (hp.BATTERY_STRAP_W, 4.0, hp.CHASSIS_PLATE_T * 4.0),
                    (sx_centre, sy * hp.BATTERY_STRAP_SLOT_Y, 0.0),
                )
            )
    through.append(
        _box(
            (hp.BATTERY_TRUNK_HOLE_X, hp.BATTERY_TRUNK_HOLE_Y,
             hp.CHASSIS_PLATE_T * 4.0),
            (hp.BATTERY_TRUNK_HOLE_CENTRE[0], hp.BATTERY_TRUNK_HOLE_CENTRE[1], 0.0),
        )
    )
    return _diff(plate, *through)


def _chassis_bottom_floor_solid() -> object:
    floor_bot = hp.CHASSIS_SPLIT_Z - hp.CHASSIS_BOTTOM_FLOOR_T
    floor_top = hp.CHASSIS_SPLIT_Z + 1.0
    z_c = 0.5 * (floor_bot + floor_top)
    t = floor_top - floor_bot
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0

    plate = Pos(0.0, 0.0, z_c) * _hex_plate(hp.CHASSIS_FLAT_TO_FLAT, t)

    outer_w = hp.WELL_W + 2.0 + 2.0 * hp.CRADLE_BOND_STRIP_MM
    outer_d = hp.WELL_D + 2.0 + 2.0 * hp.CRADLE_BOND_STRIP_MM
    pad_r = apothem - hp.SERVO_OUTPUT_X
    pads = []
    for k in range(6):
        az = math.radians(30.0 + 60.0 * k)
        pads.append(
            Pos(pad_r * math.cos(az), pad_r * math.sin(az), z_c)
            * Rotation(0, 0, math.degrees(az))
            * _box((outer_w, outer_d, t), (0.0, 0.0, 0.0))
        )
    plate = _union(plate, *pads)

    cutters = []
    body_r = apothem - hp.SERVO_OUTPUT_X
    body_cut_w = hp.SERVO_BODY_W + 2.0 * hp.WELL_BODY_CL
    body_cut_d = hp.SERVO_BODY_D + 2.0 * hp.WELL_BODY_CL
    for k in range(6):
        az = math.radians(30.0 + 60.0 * k)
        rad = np.array([math.cos(az), math.sin(az)])
        tan = np.array([-math.sin(az), math.cos(az)])
        edge = apothem * rad
        cutters.append(
            Pos(body_r * math.cos(az), body_r * math.sin(az), z_c)
            * Rotation(0, 0, math.degrees(az))
            * _box((body_cut_w, body_cut_d, t * 4.0), (0.0, 0.0, 0.0))
        )
        for sx, sy, sxe, sye in hp.leg_harness_drop_slots():
            pxy = edge + sx * rad + sy * tan
            cutters.append(
                Pos(float(pxy[0]), float(pxy[1]), z_c)
                * Rotation(0, 0, math.degrees(az))
                * _box((sxe, sye, t * 4.0), (0.0, 0.0, 0.0))
            )
        pil_h = hp.RETAINER_PLATE_PILOT_DEPTH + 0.2
        for cxr, cyt in hp.chassis_lower_retainer_anchor_centres():
            pxy = edge + cxr * rad + cyt * tan
            cutters.append(
                _cyl_z(
                    hp.RETAINER_PLATE_PILOT_OD / 2.0,
                    pil_h,
                    (float(pxy[0]), float(pxy[1]), floor_bot - 0.2 + pil_h / 2.0),
                )
            )
    return _diff(plate, *cutters)


def _chassis_wago_tray_solid() -> object:
    """ONE single-bay WAGO5 corner tray, mirroring production's
    ``_chassis_wago_tray_solid`` (Aug 16 2026: one 5-port 221-415 bay per
    corner, press-fit; the two-bay WAGO3 pair this port used to model is
    RETIRED)."""
    bay_w = hp.WAGO5_W + hp.WAGO_MOUNT_BAY_CLEAR
    bay_d = hp.WAGO5_D + hp.WAGO_MOUNT_BAY_CLEAR
    t = hp.WAGO_MOUNT_WALL_T
    half_x = bay_d / 2.0 + t
    half_y = bay_w / 2.0 + t
    emb = 1.0
    h = hp.WAGO_MOUNT_WALL_H + emb
    z_c = hp.WAGO_MOUNT_WALL_H / 2.0 - emb / 2.0
    outer = _box((t, 2.0 * half_y, h), (half_x - t / 2.0, 0.0, z_c))
    walls = [
        _box((2.0 * half_x, t, h), (0.0, s * (half_y - t / 2.0), z_c))
        for s in (-1.0, 1.0)
    ]
    return _union(outer, *walls)


def make_chassis_bottom() -> object:
    full = _chassis_bottom_full_solid()
    big = 800.0
    high = _diff(full, _box((big, big, big), (0.0, 0.0, hp.CHASSIS_SPLIT_Z - big / 2.0)))
    merged = _union(high, _chassis_bottom_floor_solid())

    # Standoff SEAT PADS (production, Aug 16 2026): full-stack Phi 9 pad at
    # each standoff site, unioned BEFORE the through-cuts re-drill the
    # Phi 3.4 bore -- restores the solid seat annulus the inboard-shifted
    # harness ports clip.
    pad_top_z = hp.CHASSIS_PLATE_T / 2.0
    pad_bot_z = hp.CHASSIS_SPLIT_Z - hp.CHASSIS_BOTTOM_FLOOR_T
    merged = _union(merged, *[
        _cyl_z(hp.CHASSIS_STANDOFF_SEAT_PAD_OD / 2.0, pad_top_z - pad_bot_z,
               (cx, cy, 0.5 * (pad_top_z + pad_bot_z)))
        for (cx, cy) in hp.CHASSIS_STANDOFF_HOLES_XY
    ])

    through_cuts = []
    for strap_dx in hp.BATTERY_STRAP_DX:
        sx_centre = hp.BATTERY_HOLDER_CENTRE_X + strap_dx
        for sy in (-1.0, 1.0):
            through_cuts.append(
                _box(
                    (hp.BATTERY_STRAP_W, 4.0, hp.CHASSIS_PLATE_T * 8.0),
                    (sx_centre, sy * hp.BATTERY_STRAP_SLOT_Y, 0.0),
                )
            )
    for cx, cy in hp.CHASSIS_STANDOFF_HOLES_XY:
        through_cuts.append(
            _cyl_z(hp.BRACKET_BOLT_HOLE / 2.0, hp.CHASSIS_PLATE_T * 8.0, (cx, cy, 0.0))
        )
    through_cuts.append(
        _box(
            (hp.BATTERY_TRUNK_HOLE_X, hp.BATTERY_TRUNK_HOLE_Y,
             hp.CHASSIS_PLATE_T * 8.0),
            (hp.BATTERY_TRUNK_HOLE_CENTRE[0], hp.BATTERY_TRUNK_HOLE_CENTRE[1], 0.0),
        )
    )
    merged = _diff(merged, *through_cuts)

    trays = []
    for M in hp.wago_tray_corner_transforms():
        deg = math.degrees(math.atan2(M[1, 0], M[0, 0]))
        trays.append(
            Pos(float(M[0, 3]), float(M[1, 3]), float(M[2, 3]))
            * Rotation(0, 0, deg)
            * _chassis_wago_tray_solid()
        )
    return _union(merged, *trays)


def part_specs() -> list[StepPart]:
    proto = PROTO_DIR
    return [
        StepPart(
            "chassis_top",
            make_chassis_top,
            proto / "stl_prototype" / "chassis_top.stl",
            "First larger printable port: clipped top deck, holes, and switch bosses.",
        ),
        StepPart(
            "chassis_bottom",
            make_chassis_bottom,
            proto / "stl_prototype" / "chassis_bottom.stl",
            "Merged bottom plate with six integrated yaw cradles, folded floor, and Wago tray walls.",
        ),
        StepPart(
            "switch_holster",
            make_switch_holster,
            proto / "stl_prototype" / "switch_holster.stl",
            "Printable anti-spark switch holster; box CSG plus pigtail/bolt cutouts.",
        ),
        StepPart(
            "disc_horn",
            make_disc_horn,
            proto / "stl_reference" / "disc_horn_DO_NOT_PRINT.stl",
            "COTS visual, but useful as a clean bolt-pattern BREP test.",
        ),
        StepPart(
            "foot_boot",
            make_foot_boot,
            proto / "stl_prototype" / "foot_boot.stl",
            "Printable TPU boot; revolved BREP profile.",
        ),
        StepPart(
            "servo_body",
            make_servo_body,
            proto / "stl_reference" / "servo_body_DO_NOT_PRINT.stl",
            "COTS servo envelope visual; validates boxes plus bosses.",
        ),
        StepPart(
            "servo_clamp_cap",
            make_servo_clamp_cap,
            proto / "stl_prototype" / "servo_clamp_cap.stl",
            "Printable clamp cap; validates rectangular CSG plus through holes.",
        ),
        StepPart(
            "tibia_tube_printable",
            make_tibia_tube_printable,
            proto / "extra_stl" / "tibia_tube_printable.stl",
            "Existing STEP-first idea folded into the sidecar output.",
        ),
        StepPart(
            "tibia_knee_yoke",
            make_tibia_knee_yoke,
            proto / "stl_prototype" / "tibia_knee_yoke.stl",
            "Printable sandwich moving yoke with CF-tube socket.",
        ),
        StepPart(
            "coxa_link",
            make_coxa_link,
            proto / "stl_prototype" / "coxa_link.stl",
            "One-piece yaw hub plus hip-pitch fixed cradle, with horn-bolt access shafts.",
        ),
        StepPart(
            "femur_link",
            make_femur_link,
            proto / "stl_prototype" / "femur_link.stl",
            "One-piece femur: hip moving yoke, solid spar with gussets, and knee fixed cradle.",
        ),
        StepPart(
            "yaw_bearing_cap",
            make_yaw_bearing_cap,
            proto / "stl_prototype" / "yaw_bearing_cap.stl",
            "Printable bearing cap; validates ring, ears, bores, and counterbores.",
        ),
        StepPart(
            "yaw_servo_retainer",
            make_yaw_servo_retainer,
            proto / "stl_prototype" / "yaw_servo_retainer.stl",
            "Printable yaw anti-rotation saddle with four corner feet.",
        ),
        StepPart(
            "yaw_bearing_lower",
            lambda: make_yaw_bearing("lower"),
            None,
            "COTS bearing visual for assembly STEP tests.",
        ),
        StepPart(
            "yaw_bearing_upper",
            lambda: make_yaw_bearing("upper"),
            None,
            "COTS bearing visual for assembly STEP tests.",
        ),
    ]


PENDING_PARTS = [
]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Remove cad_step_test/out before exporting.",
    )
    args = parser.parse_args()

    if args.clean and OUT_DIR.exists():
        shutil.rmtree(OUT_DIR)

    exported = export_all(part_specs())
    manifest = {
        "units": "mm",
        "source": "build123d/OpenCascade BREP, constants imported from hexapod_prototype.py",
        "exported_parts": exported,
        "pending_native_brep_parts": PENDING_PARTS,
        "files": [rel for row in exported for rel in (row["step"], row["stl"])],
    }
    manifest_path = OUT_DIR / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    bundle = write_bundle(manifest, "hexapod_step_first_test_bundle.zip",
                          "manifest.json")
    print(f"wrote {manifest_path.relative_to(THIS_DIR)}")
    print(f"wrote {bundle.relative_to(THIS_DIR)}")
    print()
    print("STEP-first sidecar complete; existing STL generator was not modified.")


if __name__ == "__main__":
    main()
