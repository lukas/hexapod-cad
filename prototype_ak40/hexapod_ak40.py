#!/usr/bin/env python3
"""Parametric CAD source of truth for the AK40 hexapod prototype.

Same contract as prototype_sts3215/hexapod_prototype.py: the constants in
THIS file are the single source of truth; design_spec.yaml mirrors a curated
subset and docs/BOM.md + PROTOTYPE.md quote the derived numbers.

Maturity (Design B, Aug 2026): PRINTABLE-PENDING-VERIFICATION.  The AK40
interface dimensions below come from the official CubeMars 2D drawing
"AK40-10 V3.0" dated 2026/6/12 (data/cms/202607/ak40-10-v3-0-kv170-2d-
drawing.pdf) -- NOT yet from calipers.  Before printing six leg sets,
verify on one real actuator: output bolt PCD 27 / M2.5x3, pilot Ø15+0.05
(assumed to be a BORE -- flip AK40_PILOT_IS_BORE if it is a boss), front
case 3x M2.5x5 on Ø47.5, rear case 4x M2.5x5 on Ø47, rear boss Ø37x1.

Run:  python hexapod_ak40.py                # STLs + budget report
      python hexapod_ak40.py --report-only  # budget report only
      python hexapod_ak40.py --check        # + watertight/mass verification
"""

from __future__ import annotations

import argparse
import math
import os

import numpy as np
import trimesh

# ---------------------------------------------------------------------------
# Actuator: CubeMars AK40-10 (KV170).  Electrical/catalog values cross-
# checked 2026-08-11 against cubemars.com (V3.0 page) and OpenELAB.
# ---------------------------------------------------------------------------
AK40_BODY_DIA        = 53.0    # mm  -- Phi53 pancake
AK40_BODY_LEN        = 40.2    # mm  -- V3.0 length (original: 37.0)
AK40_MASS            = 0.190   # kg  -- V3.0 (original: 0.185)
AK40_RATED_TORQUE    = 1.3     # N*m -- continuous (thermal) limit
AK40_PEAK_TORQUE     = 4.1     # N*m
AK40_RATED_SPEED_RPM = 370.0   # rpm -- at rated torque
AK40_RATED_VOLTAGE   = 24.0    # V   -- run from 6S LiPo (22.2 nom / 25.2 full)
AK40_RATED_CURRENT   = 2.7     # A
AK40_PEAK_CURRENT    = 7.3     # A
AK40_GEAR_RATIO      = 10.0    # 10:1 planetary
AK40_KT_MOTOR        = 0.056   # N*m/A at the rotor (x10 at the output)
AK40_BACKLASH_ARCMIN = 18.0
AK40_BACKDRIVE_TORQUE = 0.06   # N*m -- quasi-direct-drive, torque-transparent

# Interface dims from the official 2D drawing (VERIFY WITH CALIPERS on one
# unit before committing six leg sets -- see module docstring):
AK40_OUT_BOLT_PCD   = 27.0     # mm -- output flange, 3x M2.5 tapped x3 deep
AK40_OUT_BOLT_N     = 3
AK40_OUT_THREAD_DEPTH = 3.0    # mm
AK40_PILOT_DIA      = 15.0     # mm -- Ø15 +0.05 centre pilot in the flange
AK40_PILOT_IS_BORE  = True     # printed parts grow a 14.85 boss into it
AK40_FRONT_BOLT_PCD = 47.5     # mm -- front (output-side) case, 3x M2.5 x5
AK40_FRONT_BOLT_N   = 3
AK40_REAR_BOLT_PCD  = 47.0     # mm -- rear case, 4x M2.5 x5
AK40_REAR_BOLT_N    = 4
AK40_REAR_BOSS_DIA  = 37.0     # mm -- rear centering boss, 1 mm proud
AK40_REAR_BOSS_H    = 1.0
# Connectors (product page): power+CAN on one side-mounted XT30PW(2+2)-M,
# UART config port A1257WR-S-3P (1.25 mm pitch).

M25_CLEAR   = 2.8      # mm -- M2.5 clearance hole
M25_CB_DIA  = 5.5      # mm -- SHCS head counterbore (head Ø4.5 + slop)
M3_CLEAR    = 3.4
PILOT_BOSS_DIA = 14.85  # mm -- into the Ø15+0.05 bore
PILOT_BOSS_H   = 1.8

# Flange-rub relief: if the rotating output flange sits FLUSH with the
# fixed front ring (side view is ambiguous; assume worst case), any hub
# face wider than the flange rubs the ring.  Every hub contact face gets
# a 0.6 mm relief annulus outside the flange radius so contact happens
# only on the rotating flange.
FLANGE_RELIEF_R0 = 16.0   # mm -- inside = contact land (flange assumed >=Ø32)
FLANGE_RELIEF_R1 = 22.0   # mm -- relief out to past any hub edge
FLANGE_RELIEF_D  = 0.6    # mm

# Side-mounted XT30PW(2+2) power/CAN plug keep-out (drawing shows it on
# the cylinder near the front face).  Modeled as a stub on the mock so
# overlap checks enforce plug clocking; assembly clocks every plug to
# local -x (inboard / up-leg), which clears everything.
PLUG_R_EXTENT = 33.5   # mm -- radial tip (body 26.5 + ~7 connector)
PLUG_W        = 12.0   # mm -- tangential width
PLUG_Z0, PLUG_Z1 = -14.0, -4.0   # mm -- axial span behind the output face

BOLT3_DEG = [90.0, 210.0, 330.0]
BOLT4_DEG = [45.0, 135.0, 225.0, 315.0]

# ---------------------------------------------------------------------------
# Chassis
# ---------------------------------------------------------------------------
CHASSIS_FLAT_TO_FLAT = 260.0   # mm -- six Phi53 yaw pancakes need the floor
CHASSIS_BOT_T        = 5.0     # mm
CHASSIS_TOP_T        = 4.0     # mm
CHASSIS_GAP          = 50.0    # mm -- yaw bodies (40.2) stand on the bottom
                               #       plate inside the gap + wiring room
CHASSIS_CIRCUMRADIUS = CHASSIS_FLAT_TO_FLAT / 2.0 / math.cos(math.pi / 6.0)
CHASSIS_STANDOFF_XY  = (40.0, 40.0)   # +-x,+-y M3 standoff square (x4)

LEG_COUNT    = 6
LEG_MOUNT_R  = 150.0           # mm -- yaw axes on the hex vertex radius
LEG_AZIMUTHS = [i * 60.0 for i in range(LEG_COUNT)]   # deg

# z datums (body frame, z=0 mid-gap)
PLATE_B_TOP = -CHASSIS_GAP / 2.0                    # -25  bottom plate top
PLATE_B_BOT = PLATE_B_TOP - CHASSIS_BOT_T           # -30
PLATE_T_BOT = CHASSIS_GAP / 2.0                     # +25
PLATE_T_TOP = PLATE_T_BOT + CHASSIS_TOP_T           # +29
YAW_WELL_DIA = 40.0    # mm -- through-hole; the coxa hub (Ø36) turns inside.
                       #       Was 44/Ø40: the front-case counterbores (Ø5.5
                       #       on Ø47.5 PCD, inner edge r=21.0) broke through
                       #       a Ø44 well wall; Ø40 leaves a 1.0 mm land.
COXA_HUB_DIA = 36.0    # mm -- 2 mm radial clearance inside the well

# ---------------------------------------------------------------------------
# Leg geometry (axis-to-axis)
# ---------------------------------------------------------------------------
COXA_LENGTH  = 65.0    # mm -- yaw axis -> hip pitch axis
FEMUR_LENGTH = 100.0   # mm -- hip axis -> knee axis (flat printed plate)
TIBIA_LENGTH = 150.0   # mm -- knee axis -> foot tip

HIP_AXIS_DROP = 66.0   # mm -- hip pitch axis below body-centre z=0.  Sized
                       #       so the Ø53 hip body clears the coxa arm plate
                       #       above it by ~1.5 mm (see coxa builder)
HIP_AXIS_Z = -HIP_AXIS_DROP

# Lateral (tangential) stack in the leg frame -- x radial, y tangential:
# coxa wall y[-37,-32.2] | hip body y[-32.2,+8], output faces +y
# femur plate y[+8,+14]  | knee body y[-32.2,+8], output faces -y
# tibia yoke y[-38.2,-32.2] ; tibia tube axis at y ~= -35.2
HIP_OUT_Y   = 8.0      # mm -- hip output face plane
KNEE_OUT_Y  = HIP_OUT_Y - AK40_BODY_LEN            # -32.2 knee output face
COXA_WALL_T = 4.8      # mm -- wall behind the hip rear face
FEMUR_PLATE_T = 6.0
TIBIA_TUBE_Y  = -3.0   # mm -- tube axis offset in the yoke PART frame
                       #       (leg frame: KNEE_OUT_Y + TIBIA_TUBE_Y)

# Tibia structure
TIBIA_TUBE_OD  = 12.0
TIBIA_TUBE_ID  = 10.0
TIBIA_SOCKET_OD = 19.0
TIBIA_SOCKET_TOP = -28.0   # part frame z -- clears the Ø53 knee body (r26.5)
TIBIA_SOCKET_BOT = -60.0
TIBIA_BORE_DEPTH = 30.0
FOOT_SOLID_TIP = 18.0      # mm of TPU below the tube end
TIBIA_TUBE_TOP = TIBIA_SOCKET_BOT + TIBIA_BORE_DEPTH   # -30, seats at bore top
TIBIA_TUBE_END = -(TIBIA_LENGTH - FOOT_SOLID_TIP)      # -132
TIBIA_TUBE_CUT = TIBIA_TUBE_TOP - TIBIA_TUBE_END       # 102 mm cut length

# ---------------------------------------------------------------------------
# Named stances: (femur angle below horizontal, tibia angle from vertical)
# ---------------------------------------------------------------------------
STANCES = {
    "tall":     (60.0, 12.0),   # travel stance: minimum joint torque
    "nominal":  (55.0, 15.0),   # default stand + walk
    "crouch":   (20.0, 45.0),   # TRANSITIONAL only (see report)
}
NOMINAL_STANCE = "nominal"

# ---------------------------------------------------------------------------
# Mass budget (kg)
# ---------------------------------------------------------------------------
MASS_BUDGET = {
    "actuators (18x AK40-10 V3.0)":            18 * AK40_MASS,
    "printed links + CF tibias + feet (6 legs)": 0.72,
    "chassis plates + standoffs + fasteners":    0.55,
    "battery (6S 5000 mAh LiPo)":                0.72,
    "compute (Pi 5 + 3x USB-CAN + buck + IMU)":  0.22,
    "wiring + e-stop loopkey + fuse + misc":     0.17,
}
GRAVITY = 9.81
PETG_DENSITY = 1.27e-3   # g/mm^3


def total_mass() -> float:
    return sum(MASS_BUDGET.values())


# ---------------------------------------------------------------------------
# Kinematics + static torque model
# ---------------------------------------------------------------------------
def foot_offsets(femur_deg: float, tibia_deg: float) -> tuple[float, float]:
    """(horizontal reach hip->foot, ride height hip->ground), mm."""
    f, t = math.radians(femur_deg), math.radians(tibia_deg)
    horiz = FEMUR_LENGTH * math.cos(f) + TIBIA_LENGTH * math.sin(t)
    drop = FEMUR_LENGTH * math.sin(f) + TIBIA_LENGTH * math.cos(t)
    return horiz, drop


def stance_torques(femur_deg: float, tibia_deg: float,
                   legs_down: int) -> tuple[float, float]:
    """Static (hip pitch, knee) torque in N*m for a vertical ground force
    of total_weight/legs_down at the foot.  Yaw sees ~0 static load."""
    f, t = math.radians(femur_deg), math.radians(tibia_deg)
    force = total_mass() * GRAVITY / legs_down
    knee_arm = TIBIA_LENGTH * math.sin(t) / 1000.0
    hip_arm = (FEMUR_LENGTH * math.cos(f)) / 1000.0 + knee_arm
    return force * hip_arm, force * knee_arm


def budget_report(printed_masses: dict[str, float] | None = None) -> str:
    lines = []
    m = total_mass()
    lines.append("AK40 hexapod -- derived design budget")
    lines.append("=" * 60)
    lines.append("Mass budget:")
    for k, v in MASS_BUDGET.items():
        lines.append(f"  {k:<45s} {v:6.2f} kg")
    lines.append(f"  {'TOTAL':<45s} {m:6.2f} kg   ({m * GRAVITY:.1f} N)")
    lines.append("")
    lines.append("Static joint torque vs stance (per-foot load = W/legs):")
    lines.append(f"  {'stance':<9s} {'ride':>6s} {'reach':>6s} "
                 f"{'hip3':>6s} {'knee3':>6s} {'hip6':>6s} {'knee6':>6s}"
                 "   (N*m; 3=tripod, 6=all legs)")
    for name, (fd, td) in STANCES.items():
        horiz, drop = foot_offsets(fd, td)
        h3, k3 = stance_torques(fd, td, 3)
        h6, k6 = stance_torques(fd, td, 6)
        lines.append(f"  {name:<9s} {drop:6.0f} {horiz:6.0f} "
                     f"{h3:6.2f} {k3:6.2f} {h6:6.2f} {k6:6.2f}")
    lines.append("")
    lines.append(f"  AK40-10 rated {AK40_RATED_TORQUE} N*m / "
                 f"peak {AK40_PEAK_TORQUE} N*m.")
    lines.append("  Rules: PARK poses (tall/nominal, 6 legs) must sit under")
    lines.append("  RATED -- both do (<=0.91).  Tripod walking may exceed")
    lines.append("  rated transiently (~50% stance duty keeps the thermal")
    lines.append("  average under rated: nominal hip 1.82 -> avg 0.91).")
    lines.append("  'crouch' exceeds rated even on 6 legs (1.90) -- it is a")
    lines.append("  TRANSITIONAL pose (sit-down/stand-up path only, seconds,")
    lines.append("  46% of peak); never park or walk there.")
    if printed_masses:
        lines.append("")
        lines.append("Printed part mass (solid PETG @1.27 g/cm^3; real prints")
        lines.append("with walls+infill land ~55-70% of solid):")
        for k, v in printed_masses.items():
            lines.append(f"  {k:<30s} {v:7.1f} g")
        leg_solid = sum(v for k, v in printed_masses.items()
                        if k in ("coxa_link", "femur_link", "tibia_yoke"))
        lines.append(f"  -> per-leg link set solid {leg_solid:.0f} g; x6 at "
                     f"~0.65 solid factor ~= "
                     f"{6 * leg_solid * 0.65 / 1000:.2f} kg "
                     "(budget line: 0.72 kg incl. tubes+feet)")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Mesh helpers.  Booleans REQUIRE the manifold backend; a silent fallback
# would emit parts without bolt holes, so we fail loudly instead.
# ---------------------------------------------------------------------------
def _require_manifold() -> None:
    import importlib.util
    if importlib.util.find_spec("manifold3d") is None:
        raise RuntimeError(
            "manifold3d missing -- `uv pip install manifold3d` (holes would "
            "silently vanish without it)")


def _union(*meshes) -> trimesh.Trimesh:
    return trimesh.boolean.union([m for m in meshes if m is not None])


def _diff(a: trimesh.Trimesh, *cuts) -> trimesh.Trimesh:
    parts = [m for m in cuts if m is not None]
    if not parts:
        return a
    return trimesh.boolean.difference([a, *parts])


def _rot(axis, deg, point=None):
    return trimesh.transformations.rotation_matrix(
        math.radians(deg), axis, point=point)


def _trans(x, y, z):
    return trimesh.transformations.translation_matrix([x, y, z])


def _cyl_z(dia, z0, z1, cx=0.0, cy=0.0, sections=48) -> trimesh.Trimesh:
    c = trimesh.creation.cylinder(radius=dia / 2, height=abs(z1 - z0),
                                  sections=sections)
    c.apply_translation([cx, cy, (z0 + z1) / 2])
    return c


def _cyl_y(dia, y0, y1, cx=0.0, cz=0.0, sections=48) -> trimesh.Trimesh:
    c = trimesh.creation.cylinder(radius=dia / 2, height=abs(y1 - y0),
                                  sections=sections)
    c.apply_transform(_rot([1, 0, 0], 90))
    c.apply_translation([cx, (y0 + y1) / 2, cz])
    return c


def _box(x0, x1, y0, y1, z0, z1) -> trimesh.Trimesh:
    b = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0])
    b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    return b


def _bolt_ring_z(pcd, angles_deg, dia, z0, z1, cx=0.0, cy=0.0):
    """Vertical clearance holes on a bolt circle (returns list of cutters)."""
    out = []
    for a in angles_deg:
        r = math.radians(a)
        out.append(_cyl_z(dia, z0, z1,
                          cx + pcd / 2 * math.cos(r),
                          cy + pcd / 2 * math.sin(r)))
    return out


def _bolt_ring_y(pcd, angles_deg, dia, y0, y1, cx=0.0, cz=0.0):
    """Tangential (y-axis) clearance holes on a bolt circle in the x-z plane."""
    out = []
    for a in angles_deg:
        r = math.radians(a)
        out.append(_cyl_y(dia, y0, y1,
                          cx + pcd / 2 * math.cos(r),
                          cz + pcd / 2 * math.sin(r)))
    return out


# ---------------------------------------------------------------------------
# Printable parts.  Leg parts are built in the LEG frame (x radial out,
# y tangential, z up, origin on the yaw axis) or their own part frame as
# noted; chassis parts in the body frame.
# ---------------------------------------------------------------------------
def make_coxa_link() -> trimesh.Trimesh:
    """Yaw output hub + arm + hip rear-mount wall, one print (leg frame).

    Hub Ø40 turns inside the chassis Ø44 well; bolts UP into the yaw output
    flange (3x M2.5x6, heads counterbored from below, boss into the Ø15
    pilot).  The wall takes the hip actuator's REAR face (4x M2.5x8 into
    the case, Ø37 boss recess).  Hip body hangs below the arm plate with
    ~1.5 mm clearance -- that clearance is what set HIP_AXIS_DROP."""
    hub = _cyl_z(COXA_HUB_DIA, -32.0, PLATE_B_TOP)            # z -32..-25
    boss = _cyl_z(PILOT_BOSS_DIA, PLATE_B_TOP, PLATE_B_TOP + PILOT_BOSS_H)
    arm = _box(-15.0, 92.0, -37.0, 15.0, -38.0, -31.5)
    wall = _box(38.0, 92.0, -37.0, -37.0 + COXA_WALL_T, -94.0, -37.0)
    gus1 = _box(38.0, 43.0, -32.2, -12.0, -46.0, -38.0)
    gus2 = _box(87.0, 92.0, -32.2, -12.0, -46.0, -38.0)
    body = _union(hub, boss, arm, wall, gus1, gus2)

    cuts = []
    # flange-rub relief on the hub top face (see FLANGE_RELIEF_*)
    cuts.append(_diff(
        _cyl_z(2 * FLANGE_RELIEF_R1, PLATE_B_TOP - FLANGE_RELIEF_D,
               PLATE_B_TOP + 0.5),
        _cyl_z(2 * FLANGE_RELIEF_R0, PLATE_B_TOP - FLANGE_RELIEF_D - 1,
               PLATE_B_TOP + 1)))
    # yaw output bolts: M2.5x6 up into the flange, cb depth 4 from below.
    # The arm plate (z -38..-31.5) runs under all three positions, so each
    # gets a Ø7 insertion/driver tunnel through the arm -- without it the
    # screw's rest position is legal but it can never be inserted.
    cuts += _bolt_ring_z(AK40_OUT_BOLT_PCD, BOLT3_DEG, M25_CLEAR, -33, -24)
    cuts += _bolt_ring_z(AK40_OUT_BOLT_PCD, BOLT3_DEG, M25_CB_DIA, -33, -28)
    cuts += _bolt_ring_z(AK40_OUT_BOLT_PCD, BOLT3_DEG, 7.0, -39, -32.5)
    # ...and the chassis front-case screw at 330 deg sits over the arm too:
    # Ø7 access tunnel, aligned at yaw = 0 (home pose = service pose)
    cuts += _bolt_ring_z(AK40_FRONT_BOLT_PCD, [330.0], 7.0, -39, -31)
    # hip rear mount: 4x M2.5x8 through the wall (cb 1.5 from -y face),
    # Ø37 boss recess on the +y face, Ø14 centre hole
    y_out = -37.0 + COXA_WALL_T                                # -32.2
    cuts += _bolt_ring_y(AK40_REAR_BOLT_PCD, BOLT4_DEG, M25_CLEAR,
                         -38, -31, cx=COXA_LENGTH, cz=HIP_AXIS_Z)
    cuts += _bolt_ring_y(AK40_REAR_BOLT_PCD, BOLT4_DEG, M25_CB_DIA,
                         -38, -35.5, cx=COXA_LENGTH, cz=HIP_AXIS_Z)
    cuts.append(_cyl_y(AK40_REAR_BOSS_DIA + 0.4, y_out - 1.2, y_out + 1,
                       cx=COXA_LENGTH, cz=HIP_AXIS_Z))
    cuts.append(_cyl_y(14.0, -38, -31, cx=COXA_LENGTH, cz=HIP_AXIS_Z))
    return _diff(body, *cuts)


def make_femur_link() -> trimesh.Trimesh:
    """Flat plate femur, PART frame: hip axis = origin along y, knee at
    (+FEMUR_LENGTH,0,0), plate y[0,6] with the y=0 face on the hip output
    flange.  Boss into the hip pilot bore; knee actuator rear face bolts
    to the -y side (Ø37 recess + 4x M2.5x8, cb from +y)."""
    t = FEMUR_PLATE_T
    hub = _cyl_y(40.0, 0, t)
    strip = _box(0.0, FEMUR_LENGTH, 0.0, t, -15.0, 15.0)
    knee_disc = _cyl_y(58.0, 0, t, cx=FEMUR_LENGTH)
    boss = _cyl_y(PILOT_BOSS_DIA, -PILOT_BOSS_H, 0)
    body = _union(hub, strip, knee_disc, boss)

    cuts = []
    # flange-rub relief on the hip contact face (y=0)
    cuts.append(_diff(
        _cyl_y(2 * FLANGE_RELIEF_R1, -0.5, FLANGE_RELIEF_D),
        _cyl_y(2 * FLANGE_RELIEF_R0, -1, FLANGE_RELIEF_D + 1)))
    # hip output bolts: M2.5x6 from +y, cb depth 3
    cuts += _bolt_ring_y(AK40_OUT_BOLT_PCD, BOLT3_DEG, M25_CLEAR, -3, t + 1)
    cuts += _bolt_ring_y(AK40_OUT_BOLT_PCD, BOLT3_DEG, M25_CB_DIA,
                         t - 3, t + 1)
    # knee rear mount: recess on -y face + 4x M2.5x8 (cb 2.5 from +y)
    cuts.append(_cyl_y(AK40_REAR_BOSS_DIA + 0.4, -1, 1.2, cx=FEMUR_LENGTH))
    cuts += _bolt_ring_y(AK40_REAR_BOLT_PCD, BOLT4_DEG, M25_CLEAR,
                         -1, t + 1, cx=FEMUR_LENGTH)
    cuts += _bolt_ring_y(AK40_REAR_BOLT_PCD, BOLT4_DEG, M25_CB_DIA,
                         t - 2.5, t + 1, cx=FEMUR_LENGTH)
    cuts.append(_cyl_y(14.0, -1, t + 1, cx=FEMUR_LENGTH))
    return _diff(body, *cuts)


def make_tibia_yoke() -> trimesh.Trimesh:
    """Knee output hub + web + CF tube socket, PART frame: knee axis =
    origin along y, contact face y=0 (against the knee output flange,
    which faces -y in the leg frame), tube along -z at y=TIBIA_TUBE_Y.
    Socket top starts below z=-28 so it clears the Ø53 knee body."""
    hub = _cyl_y(40.0, -6.0, 0.0)
    boss = _cyl_y(PILOT_BOSS_DIA, 0.0, PILOT_BOSS_H)
    web = _box(-9.0, 9.0, -6.0, 0.0, -30.0, -14.0)
    sock = _cyl_z(TIBIA_SOCKET_OD, TIBIA_SOCKET_BOT, TIBIA_SOCKET_TOP,
                  cy=TIBIA_TUBE_Y)
    # stays behind the y=0 contact plane -- everything past it is knee body
    neck = _box(-8.0, 8.0, -6.0, 0.0, -32.0, -24.0)
    body = _union(hub, boss, web, sock, neck)

    cuts = []
    # flange-rub relief on the knee contact face (y=0)
    cuts.append(_diff(
        _cyl_y(2 * FLANGE_RELIEF_R1, -FLANGE_RELIEF_D, 0.5),
        _cyl_y(2 * FLANGE_RELIEF_R0, -FLANGE_RELIEF_D - 1, 1)))
    # knee output bolts: M2.5x6 from -y, cb depth 3
    cuts += _bolt_ring_y(AK40_OUT_BOLT_PCD, BOLT3_DEG, M25_CLEAR, -7, 3)
    cuts += _bolt_ring_y(AK40_OUT_BOLT_PCD, BOLT3_DEG, M25_CB_DIA, -7, -3)
    # tube bore + 2 cross roll-pin holes (Ø2.5 pins, no drilling)
    cuts.append(_cyl_z(TIBIA_TUBE_OD + 0.15, TIBIA_SOCKET_BOT - 1,
                       TIBIA_SOCKET_BOT + TIBIA_BORE_DEPTH, cy=TIBIA_TUBE_Y))
    for z in (-38.0, -50.0):
        pin = trimesh.creation.cylinder(radius=1.3, height=TIBIA_SOCKET_OD + 4)
        pin.apply_transform(_rot([0, 1, 0], 90))
        pin.apply_translation([0, TIBIA_TUBE_Y, z])
        cuts.append(pin)
    return _diff(body, *cuts)


def make_foot_boot() -> trimesh.Trimesh:
    """TPU boot, own frame: tube end enters bore at z=0 going up; solid
    rounded tip below.  Ø11.7 bore = 0.3 mm interference on the Ø12 tube."""
    shell = _cyl_z(20.0, -FOOT_SOLID_TIP - 6.0, 24.0)
    tip = trimesh.creation.icosphere(radius=10.0, subdivisions=3)
    tip.apply_translation([0, 0, -FOOT_SOLID_TIP - 6.0])
    body = _union(shell, tip)
    bore = _cyl_z(11.7, 0.0, 25.0)
    return _diff(body, bore)


def _hex_plate(thick: float) -> trimesh.Trimesh:
    p = trimesh.creation.cylinder(radius=CHASSIS_CIRCUMRADIUS, height=thick,
                                  sections=6)
    return p


def make_chassis_bottom() -> trimesh.Trimesh:
    """260 f2f hex, 5 mm.  Per leg: Ø44 yaw well + 3x M2.5x8 up into the
    actuator FRONT case ring (cb from below).  Plus M3 standoff square,
    battery strap slots, XT90 pass-through, per-leg harness slots."""
    plate = _hex_plate(CHASSIS_BOT_T)
    plate.apply_translation([0, 0, (PLATE_B_TOP + PLATE_B_BOT) / 2])
    cuts = []
    for az in LEG_AZIMUTHS:
        r = math.radians(az)
        cx, cy = LEG_MOUNT_R * math.cos(r), LEG_MOUNT_R * math.sin(r)
        cuts.append(_cyl_z(YAW_WELL_DIA, PLATE_B_BOT - 1, PLATE_B_TOP + 1,
                           cx, cy))
        # front-case bolt ring, rotated so no hole lands on the hex edge
        for a in BOLT3_DEG:
            aa = math.radians(a + az)
            hx = cx + AK40_FRONT_BOLT_PCD / 2 * math.cos(aa)
            hy = cy + AK40_FRONT_BOLT_PCD / 2 * math.sin(aa)
            cuts.append(_cyl_z(M25_CLEAR, PLATE_B_BOT - 1, PLATE_B_TOP + 1,
                               hx, hy))
            cuts.append(_cyl_z(M25_CB_DIA, PLATE_B_BOT - 1,
                               PLATE_B_BOT + 1.5, hx, hy))
        # harness slot inboard of the well
        sx, sy = 112.0 * math.cos(r), 112.0 * math.sin(r)
        slot = _box(-9, 9, -4, 4, PLATE_B_BOT - 1, PLATE_B_TOP + 1)
        slot.apply_transform(_rot([0, 0, 1], az))
        slot.apply_translation([sx, sy, 0])
        cuts.append(slot)
    sx, sy = CHASSIS_STANDOFF_XY
    for gx in (-sx, sx):
        for gy in (-sy, sy):
            cuts.append(_cyl_z(M3_CLEAR, PLATE_B_BOT - 1, PLATE_B_TOP + 1,
                               gx, gy))
    for bx in (-35.0, 35.0):
        for by in (-28.0, 28.0):
            cuts.append(_box(bx - 15, bx + 15, by - 2.5, by + 2.5,
                             PLATE_B_BOT - 1, PLATE_B_TOP + 1))
    cuts.append(_box(82.0, 108.0, -8.0, 8.0, PLATE_B_BOT - 1,
                     PLATE_B_TOP + 1))       # XT90 / trunk pass-through
    return _diff(plate, *cuts)


def make_chassis_top() -> trimesh.Trimesh:
    """260 f2f hex, 4 mm: Ø80 centre access, M3 standoff square, Pi 5
    mount holes (58 x 49 pattern) east of centre."""
    plate = _hex_plate(CHASSIS_TOP_T)
    plate.apply_translation([0, 0, (PLATE_T_BOT + PLATE_T_TOP) / 2])
    cuts = [_cyl_z(80.0, PLATE_T_BOT - 1, PLATE_T_TOP + 1)]
    sx, sy = CHASSIS_STANDOFF_XY
    for gx in (-sx, sx):
        for gy in (-sy, sy):
            cuts.append(_cyl_z(M3_CLEAR, PLATE_T_BOT - 1, PLATE_T_TOP + 1,
                               gx, gy))
    for px in (-29.0, 29.0):            # Pi 5 58 x 49 hole pattern
        for py in (-24.5, 24.5):
            cuts.append(_cyl_z(2.7, PLATE_T_BOT - 1, PLATE_T_TOP + 1,
                               75.0 + px, py))
    return _diff(plate, *cuts)


PRINTABLE_PARTS = {
    "chassis_bottom": make_chassis_bottom,
    "chassis_top": make_chassis_top,
    "coxa_link": make_coxa_link,
    "femur_link": make_femur_link,
    "tibia_yoke": make_tibia_yoke,
    "foot_boot": make_foot_boot,        # TPU
}


# ---------------------------------------------------------------------------
# Assembly (real printed parts + actuator mocks)
# ---------------------------------------------------------------------------
def make_actuator_mock() -> trimesh.Trimesh:
    """Phi53 x 40.2 mock; output face at z=0, body extends -z.  Carries the
    Ø15 output pilot bore (so hub pilot bosses check clean) and the side
    XT30PW plug stub along local +x (so plug clocking is overlap-checked;
    the assembly clocks every plug to leg-frame -x)."""
    body = _cyl_z(AK40_BODY_DIA, -AK40_BODY_LEN, 0.0, sections=40)
    plug = _box(AK40_BODY_DIA / 2 - 1, PLUG_R_EXTENT,
                -PLUG_W / 2, PLUG_W / 2, PLUG_Z0, PLUG_Z1)
    bore = _cyl_z(AK40_PILOT_DIA + 0.1, -2.2, 0.5)
    return _diff(_union(body, plug), bore)


def assemble_leg(azimuth_deg: float, femur_deg: float, tibia_deg: float,
                 parts_cache: dict) -> trimesh.Trimesh:
    f = math.radians(femur_deg)
    knee_x = COXA_LENGTH + FEMUR_LENGTH * math.cos(f)
    knee_z = HIP_AXIS_Z - FEMUR_LENGTH * math.sin(f)
    meshes = []

    yaw = make_actuator_mock()
    yaw.apply_transform(_rot([0, 0, 1], 180))    # clock plug to -x (inboard)
    yaw.apply_transform(_rot([1, 0, 0], 180))    # output faces down
    yaw.apply_translation([0, 0, PLATE_B_TOP])
    meshes.append(yaw)

    meshes.append(parts_cache["coxa_link"].copy())

    hip = make_actuator_mock()
    hip.apply_transform(_rot([0, 0, 1], 180))    # clock plug to -x (up-leg)
    hip.apply_transform(_rot([1, 0, 0], -90))    # output faces +y
    hip.apply_translation([COXA_LENGTH, HIP_OUT_Y, HIP_AXIS_Z])
    meshes.append(hip)

    fem = parts_cache["femur_link"].copy()
    fem.apply_transform(_rot([0, 1, 0], femur_deg))
    fem.apply_translation([COXA_LENGTH, HIP_OUT_Y, HIP_AXIS_Z])
    meshes.append(fem)

    knee = make_actuator_mock()
    knee.apply_transform(_rot([0, 0, 1], 180))   # clock plug to -x (up-leg)
    knee.apply_transform(_rot([1, 0, 0], 90))    # output faces -y
    knee.apply_translation([knee_x, KNEE_OUT_Y, knee_z])
    meshes.append(knee)

    yoke = parts_cache["tibia_yoke"].copy()
    yoke.apply_transform(_rot([0, 1, 0], -tibia_deg))
    yoke.apply_translation([knee_x, KNEE_OUT_Y, knee_z])
    meshes.append(yoke)

    t = math.radians(tibia_deg)
    tib_dir = np.array([math.sin(t), 0.0, -math.cos(t)])
    tube_c = (np.array([knee_x, KNEE_OUT_Y + TIBIA_TUBE_Y, knee_z])
              + tib_dir * (-(TIBIA_TUBE_TOP + TIBIA_TUBE_END) / 2))
    tube = trimesh.creation.cylinder(radius=TIBIA_TUBE_OD / 2,
                                     height=TIBIA_TUBE_CUT, sections=32)
    tube.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], tib_dir))
    tube.apply_translation(tube_c)
    meshes.append(tube)

    boot = parts_cache["foot_boot"].copy()
    boot.apply_transform(_rot([0, 1, 0], -tibia_deg))
    boot.apply_translation(np.array([knee_x, KNEE_OUT_Y + TIBIA_TUBE_Y,
                                     knee_z]) + tib_dir * (-TIBIA_TUBE_END))
    meshes.append(boot)

    leg = trimesh.util.concatenate(meshes)
    leg.apply_transform(_rot([0, 0, 1], azimuth_deg))
    leg.apply_translation([LEG_MOUNT_R * math.cos(math.radians(azimuth_deg)),
                           LEG_MOUNT_R * math.sin(math.radians(azimuth_deg)),
                           0])
    return leg


def assemble_robot(stance: str, parts_cache: dict) -> trimesh.Trimesh:
    fd, td = STANCES[stance]
    parts = [parts_cache["chassis_top"].copy(),
             parts_cache["chassis_bottom"].copy()]
    for az in LEG_AZIMUTHS:
        parts.append(assemble_leg(az, fd, td, parts_cache))
    return trimesh.util.concatenate(parts)


# ---------------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--report-only", action="store_true")
    ap.add_argument("--check", action="store_true",
                    help="verify watertightness + print part masses")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    art = os.path.join(here, "artifacts")
    os.makedirs(art, exist_ok=True)

    if args.report_only:
        report = budget_report()
        print(report)
        with open(os.path.join(art, "design_budget.md"), "w") as fh:
            fh.write("```\n" + report + "\n```\n")
        return

    _require_manifold()
    proto = os.path.join(here, "stl_prototype")
    ref = os.path.join(here, "stl_reference")
    os.makedirs(proto, exist_ok=True)
    os.makedirs(ref, exist_ok=True)

    cache, masses = {}, {}
    failures = []
    for name, builder in PRINTABLE_PARTS.items():
        mesh = builder()
        cache[name] = mesh
        masses[name] = mesh.volume * PETG_DENSITY
        if not mesh.is_watertight:
            failures.append(f"{name}: not watertight")
        if mesh.volume <= 0:
            failures.append(f"{name}: non-positive volume")
        mesh.export(os.path.join(proto, f"{name}.stl"))
        print(f"wrote stl_prototype/{name}.stl  "
              f"({masses[name]:.0f} g solid, "
              f"{'watertight' if mesh.is_watertight else 'LEAKY'})")

    report = budget_report(masses)
    print()
    print(report)
    with open(os.path.join(art, "design_budget.md"), "w") as fh:
        fh.write("```\n" + report + "\n```\n")

    for stance in STANCES:
        robot = assemble_robot(stance, cache)
        path = os.path.join(ref, f"full_robot_ak40_{stance}_DO_NOT_PRINT.stl")
        robot.export(path)
        ext = robot.bounds[1] - robot.bounds[0]
        print(f"wrote stl_reference/{os.path.basename(path)}  "
              f"envelope {ext[0]:.0f} x {ext[1]:.0f} x {ext[2]:.0f} mm")
    leg = assemble_leg(0.0, *STANCES[NOMINAL_STANCE], cache)
    leg.export(os.path.join(ref, "single_leg_ak40_DO_NOT_PRINT.stl"))
    print("wrote stl_reference/single_leg_ak40_DO_NOT_PRINT.stl")

    if args.check:
        if failures:
            raise SystemExit("CHECK FAILED: " + "; ".join(failures))
        print("CHECK OK: all printable parts watertight, positive volume")


if __name__ == "__main__":
    main()
