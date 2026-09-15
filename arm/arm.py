"""Arm — OPTIONAL 5-DOF anthropomorphic arm + gripper add-on for the
hexapod walker prototype.

This module now lives at ``prototype/arm/`` (was ``hexapod_walker/arm/``)
so the optional arm is bundled with the prototype it bolts onto.  It
still re-uses the leg's parametric servo well, disc horn, and link
helpers (in ``prototype/hexapod_prototype.py``) as-is, so the arm
shares the same servo specs (DS3225-class 25 kg-cm, 14 mm disc-horn
bolt circle, 49.5 x 10 mm tab pattern) without modifying any prototype
geometry.

The arm is OPT-IN -- see the ``--with-arm`` flag on
``prototype/build_all.py`` and ``prototype/_verify_prototype.py``.
By default the prototype's build + verification flow does not touch
any of this directory.

Kinematic chain
---------------

    chassis-top (4 centre M3 holes at radius 35 mm)
        |
      [J1 base yaw]               DS3225, +Z axis
        |
    arm_shoulder_link             (= coxa_link  re-export)
        |
      [J2 shoulder pitch]         DS3225, +Y axis
        |
    arm_upper                     (= femur_link re-export)
        |
      [J3 elbow pitch]            DS3225, +Y axis (parallel J2)
        |
    arm_forearm                   (= tibia_link re-export)
        |
    wrist_adapter                 (NEW: foot-socket plug + 4-bolt face)
        |
      [J4 wrist pitch]            DS3225, +Y axis (parallel J3)
        |
    gripper_base                  (NEW: J4 cradle + J5 cradle + jaw pivots)
        |
      [J5 gripper]                MG90S 9 g micro servo
        |
    gripper_jaw_left/right        (NEW: parallel-jaw end-effector)

Reach
-----
Neutral straight-out pose: gripper tip at
    R = COXA_LENGTH + FEMUR_LENGTH + TIBIA_LENGTH + WRIST_OFFSET + GRIPPER_OFFSET
      ~ 25 + 90 + 130 + 20 + 60 = ~ 325 mm
from the J1 axis.  Standing height (chassis top face) is about 117 mm
above the floor with the standard prototype legs in stance.  At full
"reach down" pose (J2 = -90 deg, J3 = J4 = 0) the gripper tip sits
below the chassis edge, low enough to pick objects off the floor.

Outputs:

    stl_arm/           -- slicer-ready printables only
        arm_base_bracket.stl
        arm_shoulder_link.stl     (re-export of leg's coxa_link)
        arm_upper.stl             (re-export of leg's femur_link)
        arm_forearm.stl           (re-export of leg's tibia_link)
        wrist_adapter.stl
        gripper_base.stl
        gripper_jaw_left.stl
        gripper_jaw_right.stl
    stl_reference/     -- not for printing
        arm_assembly_preview_DO_NOT_PRINT.stl

The arm subdirectory is fully self-contained: nothing in the prototype
directory imports anything from here, so the user can simply choose
not to print these STLs to ship without the arm.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

# The prototype dir is our PARENT; add it to sys.path so we can
# re-use its parametric helpers WITHOUT modifying anything in there.
# (`run.sh` already exposes the repo root on PYTHONPATH, but a direct
# `import hexapod_prototype as HP` only resolves once the prototype
# folder itself is on sys.path.)
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROTO_DIR = os.path.normpath(os.path.join(_THIS_DIR, os.pardir))
if _PROTO_DIR not in sys.path:
    sys.path.insert(0, _PROTO_DIR)

import hexapod_prototype as HP   # noqa: E402  (sys.path tweak above)


# ---------------------------------------------------------------------------
# Output dirs
# ---------------------------------------------------------------------------

STL_DIR = os.path.join(_THIS_DIR, "stl_arm")
REF_STL_DIR = os.path.join(_THIS_DIR, "stl_reference")
RENDER_DIR = os.path.join(_THIS_DIR, "renders")
os.makedirs(STL_DIR, exist_ok=True)
os.makedirs(REF_STL_DIR, exist_ok=True)
os.makedirs(RENDER_DIR, exist_ok=True)


# ---------------------------------------------------------------------------
# Local helpers (re-exports of HP primitives so we don't poke its privates
# everywhere)
# ---------------------------------------------------------------------------

_box = HP._box
_cyl = HP._cyl
_union = HP._union
_diff = HP._diff


# ---------------------------------------------------------------------------
# Arm-only constants
# ---------------------------------------------------------------------------

# Chassis-top centre-hole bolt pattern (same one the electronics tray
# uses; see `_hex_plate(with_centre_holes=True)` in hexapod_prototype.py).
# 4 holes at angles {45, 135, 225, 315} deg on a 35 mm circle => XY =
# (+/- 24.75, +/- 24.75).
CHASSIS_CENTRE_BOLT_R = 35.0

# ---- MG90S micro servo (J5 gripper) ----
# Nominal Tower Pro MG90S body: 22.5 x 12 x 22.5 mm (excluding tabs +
# output gear).  Tabs span ~32.5 mm tip-to-tip.  M2 self-tappers (or
# M2.5) into ~ 1.7 mm pilots.
MG90S_BODY_W      = 22.5
MG90S_BODY_D      = 12.0
MG90S_BODY_H      = 22.5
MG90S_OUTPUT_H    = 4.5
MG90S_OUTPUT_X    = 5.5
MG90S_TAB_W       = 32.5
MG90S_TAB_T       = 1.6
MG90S_TAB_Z       = 16.0
MG90S_TAB_HOLE_PCD = 28.0
MG90S_TAB_HOLE     = 2.2

# ---- Wrist adapter geometry ----
# A short plate that plugs into the tibia's foot socket on one side and
# carries the 4-bolt J4 horn-adapter pattern on the other.  The wrist
# adapter itself is "passive" (no servo body); J4 is housed inside the
# gripper_base and its horn drives the wrist_adapter face that we bolt
# onto here.  Thus J4 rotates the gripper_base about its output axis
# relative to the (forearm-fixed) wrist_adapter.
WRIST_PLATE_W = 34.0   # mm (was 30 in the original sketch; 34 leaves
                       #     ~ 4 mm of material outside the 24 mm horn
                       #     PCD on every side)
WRIST_PLATE_D = 30.0
WRIST_PLATE_T = 6.0
WRIST_PLUG_OD = HP.FOOT_HUB_OD       # plugs into the tibia's foot socket
WRIST_PLUG_H  = HP.FOOT_HUB_HEIGHT   # height of the plug above the plate

# ---- Gripper-base body ----
GBASE_W = 50.0    # along the jaw-open direction
GBASE_D = 30.0    # depth (along J5 servo output axis)
GBASE_H = 46.0    # along the wrist-to-jaw direction
GBASE_WALL = 3.0

# Jaw pivots: hinged on a single M2.5 pin across the front of the
# gripper_base.  Both jaws share the same pivot pin position so they
# track each other through a small 4-bar linkage to J5's horn.
JAW_LENGTH       = 45.0
JAW_W            = 14.0
JAW_T            = 4.0
JAW_PIVOT_OD     =  2.7   # M2.5 clearance
JAW_PIVOT_PCD    = 26.0   # distance between the two jaws' pivot axes
JAW_LINKAGE_OD   =  2.2   # M2 clearance for the linkage pins

# Output reach used by the assembly preview (the J4 horn-driven
# gripper_base hangs straight down from the wrist_adapter at neutral
# pose).
WRIST_TO_GRIPPER_TIP = WRIST_PLATE_T + GBASE_H + JAW_LENGTH  # mm
J4_HORN_STACK_Z      = (HP.SERVO_BODY_H - HP.WELL_RIM_Z      # ~10.75
                        + HP.SERVO_OUTPUT_H                  # +6
                        + HP.HORN_STACK_H)                   # +5 disc = 21.75
# (= the same "yaw_output_z" stack used in the leg: the 20 mm aluminum
#  25T disc horn seats on the spline and its 5 mm-thick top face is the
#  link mating plane.  The old printed servo_horn_adapter is gone, so
#  the stack dropped by HORN_ADAPTER_T relative to the pre-disc design.)


# ---------------------------------------------------------------------------
# Common save helper
# ---------------------------------------------------------------------------

def _save(mesh: trimesh.Trimesh, name: str, *, reference: bool = False) -> str:
    out_dir = REF_STL_DIR if reference else STL_DIR
    path = os.path.join(out_dir, name)
    mesh.export(path)
    n_faces = len(mesh.faces)
    extents = mesh.extents
    rel = "stl_reference" if reference else "stl_arm"
    print(f"  wrote {rel}/{name:32s}"
          f"  {n_faces:>6d} faces"
          f"  envelope {extents[0]:5.1f} x {extents[1]:5.1f} x {extents[2]:5.1f} mm")
    return path


# ---------------------------------------------------------------------------
# NEW PART 1: J1 base bracket
# ---------------------------------------------------------------------------

def make_arm_base_bracket() -> trimesh.Trimesh:
    """J1 base-yaw servo bracket.

    Bolts to the chassis-top centre via 4 M3 holes on a 35 mm-radius
    square pattern (= same pattern as the electronics tray uses).  The
    DS3225 J1 servo seats with its output spline pointing UP; the body
    sits ABOVE the chassis top plate (entirely contained in the
    bracket) so we don't need to cut into the existing chassis plates.

    Local frame:
        Origin = J1 yaw axis, sitting on the chassis-top TOP face.
        +Z = J1 yaw axis (output up).
        +X = arm neutral-reach direction.
        +Y = orthogonal in the chassis plane.

    Why the well's long axis runs along +Y (not +X):
        With the well rotated 90 deg about Z, its X-extent shrinks to
        +/- 12.5 mm (= WELL_D/2) and its Y-extent grows to +/- 29 mm
        (= WELL_W/2, plus a 10 mm offset from re-centering on the J1
        axis).  The 4 chassis bolts at (+/-24.7, +/-24.7) then all
        miss the well wall material in 2D, so their heads are
        accessible from straight above and they don't punch through
        the cradle ceiling.
    """
    flange_t = HP.BRACKET_FLANGE_T   # 4 mm
    flange_size = 76.0               # square; > 2 * 24.7 + 26 of edge
                                     # margin so the bolt holes have
                                     # plenty of material around them
    flange = _box((flange_size, flange_size, flange_t),
                  center=(0, 0, flange_t / 2.0))

    # The well sits ON TOP of the chassis plate face: its OUTER floor
    # is at bracket z = 0, so the body cavity opens upward and the
    # bottom 4 mm of well wall fuses with the flange.
    well = HP._servo_well_solid()
    R_well = rotation_matrix(np.pi / 2.0, [0, 0, 1])    # well +X -> bracket +Y
    well.apply_transform(R_well)
    # After R_well the body's output gear axis (well-local
    # (SERVO_OUTPUT_X, 0, *)) maps to bracket (0, +SERVO_OUTPUT_X, *).
    # We want it on the J1 axis (0, 0, *), so shift in -Y by
    # SERVO_OUTPUT_X.  Add WELL_FLOOR_T in +Z so the well's outer
    # floor face (well-local z = -WELL_FLOOR_T) lands on bracket z = 0.
    well.apply_translation([0, -HP.SERVO_OUTPUT_X, HP.WELL_FLOOR_T])

    wire_slot = HP._wire_exit_slot()
    wire_slot.apply_transform(R_well)
    wire_slot.apply_translation([0, -HP.SERVO_OUTPUT_X, HP.WELL_FLOOR_T])

    # 4 chassis-mount bolt holes through the flange.
    bolt_holes = []
    for i in range(4):
        a = np.pi / 4 + i * np.pi / 2
        h = _cyl(HP.BRACKET_BOLT_HOLE / 2.0, flange_t * 4)
        h.apply_translation([CHASSIS_CENTRE_BOLT_R * np.cos(a),
                             CHASSIS_CENTRE_BOLT_R * np.sin(a),
                             flange_t / 2.0])
        bolt_holes.append(h)

    body = _union(well, flange)
    return _diff(body, wire_slot, *bolt_holes)


# ---------------------------------------------------------------------------
# REUSED PARTS — thin wrappers over the leg's part generators
# ---------------------------------------------------------------------------

def make_arm_shoulder_link() -> trimesh.Trimesh:
    """Arm shoulder link: J2 servo holder.

    Geometrically IDENTICAL to the leg's coxa_link — same horn-driven
    hub on one end, same hip-pitch servo cradle on the other.  Here the
    hub bolts to the J1 disc horn on top of arm_base_bracket and the
    cradle holds the J2 (shoulder pitch) servo.
    """
    return HP.make_coxa_link()


def make_arm_upper() -> trimesh.Trimesh:
    """Upper arm: J2 horn-driven flat link with a J3 servo cradle at
    the far end.  IDENTICAL to the leg's femur_link."""
    return HP.make_femur_link()


def make_arm_forearm() -> trimesh.Trimesh:
    """Forearm: J3 horn-driven link terminating in the same FOOT_HUB
    socket the leg uses for its foot pad.  IDENTICAL to the leg's
    tibia_link; the wrist_adapter plugs into the foot socket where the
    foot pad would normally sit."""
    return HP.make_tibia_link()


# ---------------------------------------------------------------------------
# NEW PART 3: wrist adapter
# ---------------------------------------------------------------------------

def make_wrist_adapter() -> trimesh.Trimesh:
    """Wrist-adapter plate.

    Bridges the forearm's foot socket to the J4 disc horn:

        forearm.foot_socket  <- FOOT_HUB plug ON TOP face
        gripper_base side    <- 4 x M3 horn-bolt pattern ON BOTTOM face
                                (matching DISC_HORN_BOLT_PCD = 14 mm so the
                                 SAME 20 mm aluminum 25T disc horn that the
                                 legs use mates here too: 4 x M3 x 6 SHCS on
                                 the cross pattern thread into the disc's
                                 tapped holes -- see hexapod_prototype.py
                                 DISC_HORN_BOLT_*)

    Local frame:
        Origin = wrist-adapter centre, on the plate's TOP face.
        +Z = TOWARD the gripper (away from the forearm).
        +X = perpendicular to J4 pitch axis (jaw-open direction at neutral).
        +Y = J4 pitch axis (= parallel to J3's pitch axis).

    The wrist_adapter itself is PASSIVE.  J4 is housed inside the
    gripper_base; its output horn drives the wrist_adapter face that
    you bolt onto from the gripper side, so when J4 rotates the
    gripper_base swings about J4's output axis relative to the
    forearm-fixed wrist_adapter.
    """
    # Plate body, centred on the plate's bottom face (z = 0 is the BOTTOM
    # face that mates to the J4 disc horn; +Z reaches up to the foot-
    # socket plug).
    plate = _box((WRIST_PLATE_W, WRIST_PLATE_D, WRIST_PLATE_T),
                 center=(0, 0, WRIST_PLATE_T / 2.0))

    # Foot-socket plug on +Z face (mates into the tibia's foot socket).
    # Slightly under-sized so it slides in with ~ 0.4 mm radial clearance
    # (the tibia's foot bore is FOOT_HUB_OD + 0.4 on the bore side).
    plug = _cyl(WRIST_PLUG_OD / 2.0 - 0.2, WRIST_PLUG_H + 1.0)
    plug.apply_translation([0, 0, WRIST_PLATE_T + (WRIST_PLUG_H + 1.0) / 2.0])

    # 4 horn-bolt-pattern holes through the plate so the 20 mm aluminum
    # 25T disc horn (same part the legs use) bolts directly to the bottom
    # face.  Phi DISC_HORN_BOLT_OD = 3.4 mm M3 clearance; the M3 SHCS
    # thread into the disc's tapped holes on the DISC_HORN_BOLT_PCD =
    # 14 mm cross pattern (DISC_HORN_BOLT_ANGLES_RAD = 0/90/180/270).
    bolt_holes = []
    for a in HP.DISC_HORN_BOLT_ANGLES_RAD:
        h = _cyl(HP.DISC_HORN_BOLT_OD / 2.0, WRIST_PLATE_T * 4)
        h.apply_translation([HP.DISC_HORN_BOLT_PCD / 2.0 * np.cos(a),
                             HP.DISC_HORN_BOLT_PCD / 2.0 * np.sin(a),
                             WRIST_PLATE_T / 2.0])
        bolt_holes.append(h)

    # Centre clearance hole (so the M3 horn screw on top of the J4 horn
    # can poke up into the plate without bottoming out).
    centre = _cyl(HP.HORN_CENTRE_OD / 2.0, WRIST_PLATE_T * 4)
    centre.apply_translation([0, 0, WRIST_PLATE_T / 2.0])

    body = _union(plate, plug)
    return _diff(body, centre, *bolt_holes)


# ---------------------------------------------------------------------------
# NEW PART 4: gripper base
# ---------------------------------------------------------------------------

def make_gripper_base() -> trimesh.Trimesh:
    """Gripper base body.

    Houses TWO servos:
      * J4 wrist-pitch (DS3225-class) — its output horn drives the
        wrist_adapter on the TOP face.  When J4 actuates the entire
        gripper_base swings about J4's output axis relative to the
        forearm-fixed wrist_adapter.
      * J5 gripper (MG90S) — its output horn drives a small 4-bar
        linkage on the FRONT face that opens / closes the two jaws.

    Also carries the M2.5 pivot pin shared by the two jaws (hinging
    them about a single axis on the front face) and four small
    pin-anchor pockets for the M2 linkage pins between J5's horn and
    each jaw.

    Local frame:
        Origin = J4 horn-adapter axis on the gripper_base's TOP face
                 (= matches wrist_adapter's bottom-face origin in
                  assembled pose).
        +Z = AWAY from the wrist (= toward the jaws).
        +X = jaw-open direction.
        +Y = J5 output-shaft direction (= parallel to J4 pitch axis).
    """
    # ---- Outer body ----
    body = _box((GBASE_W, GBASE_D, GBASE_H),
                center=(0, 0, GBASE_H / 2.0))

    # ---- J4 servo cradle: cavity for the DS3225 body inside the upper
    # half of the gripper_base.  Body's tabs sit on the gripper_base's
    # TOP face (z = 0 here; the wrist_adapter bolts down on top).
    # Output spline pokes UP through the top face and into the
    # wrist_adapter's bolt circle.
    # We use the same _servo_well_solid as the leg cradles but rotated
    # so the cavity opens UP (matching insertion from above when the
    # gripper_base is upside-down on the build plate).
    j4_well = HP._servo_well_solid()
    # well-local +Z (output direction) -> gripper-local +Z (up toward
    # the wrist) — no rotation needed except translating the output
    # column to (0, 0):
    j4_well.apply_translation([-HP.SERVO_OUTPUT_X, 0, -HP.WELL_RIM_Z])
    # After the translation:
    #   well rim at gripper z = 0 (= top face)
    #   well floor at gripper z = -(WELL_RIM_Z + WELL_FLOOR_T) ~ -29.75
    #   body bottom at gripper z = -WELL_RIM_Z ~ -27.25
    # That's all inside the GBASE_H = 46 mm tall body.  Output gear
    # poking UP through the top face at z in [0, +SERVO_OUTPUT_H].
    j4_wire = HP._wire_exit_slot()
    j4_wire.apply_translation([-HP.SERVO_OUTPUT_X, 0, -HP.WELL_RIM_Z])

    # ---- J5 (MG90S) cavity: a simple rectangular pocket in the FRONT
    # (+X side) of the gripper_base, with the output shaft pointing
    # toward +Y (= along the jaw pivot axis).
    j5_cavity_d_clear = MG90S_BODY_D + 1.2
    j5_cavity_h_clear = MG90S_BODY_H + 1.2
    j5_cavity_w_clear = MG90S_BODY_W + 1.2
    j5_pocket_centre_x = (GBASE_W / 2.0) - (j5_cavity_w_clear / 2.0) - 2.0
    j5_pocket_centre_z = GBASE_H - j5_cavity_h_clear / 2.0 - 4.0
    j5_pocket = _box((j5_cavity_w_clear, j5_cavity_d_clear, j5_cavity_h_clear),
                     center=(j5_pocket_centre_x, 0, j5_pocket_centre_z))
    # Pop the MG90S output shaft out the +Y face so the horn can drive
    # the linkage on the gripper's exterior.
    j5_output_clear = _cyl(MG90S_OUTPUT_X / 2.0 + 3.0, GBASE_D + 4.0)
    j5_output_clear.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    j5_output_clear.apply_translation([j5_pocket_centre_x + MG90S_OUTPUT_X,
                                       0,
                                       j5_pocket_centre_z + MG90S_BODY_H / 2.0
                                           - 1.5])
    # M2.5 / M2 pilot holes for J5 tab mounting (two per tab, 4 total).
    j5_pilots = []
    for sx in (-1, 1):
        h = _cyl(MG90S_TAB_HOLE / 2.0, GBASE_D + 4.0)
        h.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))
        h.apply_translation([j5_pocket_centre_x + sx * MG90S_TAB_HOLE_PCD / 2.0,
                             0,
                             j5_pocket_centre_z + MG90S_BODY_H / 2.0
                                 - MG90S_TAB_T - 1.0])
        j5_pilots.append(h)

    # ---- Jaw pivot pin bore: single M2.5 pin running along +Y through
    # the FRONT-BOTTOM of the gripper_base.  Both jaws hinge on this
    # axis (one on either +Y side).
    pivot_pin = _cyl(JAW_PIVOT_OD / 2.0, GBASE_D + 4.0)
    pivot_pin.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    pivot_pin.apply_translation([GBASE_W / 2.0 - 6.0, 0, GBASE_H - 4.0])

    # ---- 4-bar linkage anchor pockets: two slots in the FRONT face (+X)
    # so the linkage pins from J5's horn out to each jaw's lever arm
    # have clearance to swing.
    linkage_slot = _box((10.0, GBASE_D - 4.0, 6.0),
                        center=(GBASE_W / 2.0,
                                0,
                                j5_pocket_centre_z - 9.0))

    # ---- Wrist-side bolt pattern: 4 horn-pattern bolt holes through
    # the TOP face into the J4 disc horn that drives the wrist_adapter.
    # These are not strictly necessary to BUILD the gripper (J4's horn
    # already drives the adapter from the cradle side), but having them
    # gives us 4 access slots for the assembler's screwdriver and acts
    # as a visual key for "this is the wrist-mating face".
    wrist_screw_slots = []
    for a in HP.DISC_HORN_BOLT_ANGLES_RAD:
        h = _cyl(HP.DISC_HORN_BOLT_OD / 2.0 + 0.3, 12.0)
        h.apply_translation([HP.DISC_HORN_BOLT_PCD / 2.0 * np.cos(a),
                             HP.DISC_HORN_BOLT_PCD / 2.0 * np.sin(a),
                             GBASE_H - 6.0])
        wrist_screw_slots.append(h)

    return _diff(body, j4_well, j4_wire, j5_pocket, j5_output_clear,
                 *j5_pilots, pivot_pin, linkage_slot, *wrist_screw_slots)


# ---------------------------------------------------------------------------
# NEW PART 5/6: gripper jaws
# ---------------------------------------------------------------------------

def _gripper_jaw(mirror: bool) -> trimesh.Trimesh:
    """One half of the parallel-jaw gripper.

    Local frame:
        Origin = the pivot pin axis (= shared with the gripper_base's
                 pivot pin hole).
        +Z = nominal jaw-tip direction at neutral pose.
        +X = +1 for the +X jaw, mirrored to -1 for the -X jaw.
        +Y = along the pivot pin.
    """
    s = -1.0 if mirror else 1.0

    # Pivot hub: a small puck around the pin axis.
    hub = _cyl(5.0, JAW_T)
    hub.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))

    # Jaw blade: a flat rectangular fang extending in (+s X, +Z).
    blade_x_centre = s * (JAW_PIVOT_PCD / 2.0)
    blade = _box((JAW_LENGTH - 8.0, JAW_T, JAW_W),
                 center=(blade_x_centre, 0, JAW_LENGTH / 2.0))

    # Inner gripping pad: a small bump on the blade's -X face (+X face
    # for the mirrored jaw) so the two pads close on a small object.
    pad = _box((4.0, JAW_T - 1.0, JAW_W - 4.0),
               center=(blade_x_centre - s * 4.0, 0, JAW_LENGTH * 0.55))

    # Pivot pin clearance hole.
    pin = _cyl(JAW_PIVOT_OD / 2.0 + 0.2, JAW_T * 3)
    pin.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))

    # 4-bar linkage pin near the pivot (so a small lever to J5's horn
    # can open / close the jaw).
    link_pin = _cyl(JAW_LINKAGE_OD / 2.0 + 0.1, JAW_T * 3)
    link_pin.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))
    link_pin.apply_translation([s * 6.0, 0, -5.0])

    body = _union(hub, blade, pad)
    return _diff(body, pin, link_pin)


def make_gripper_jaw_left() -> trimesh.Trimesh:
    return _gripper_jaw(mirror=False)


def make_gripper_jaw_right() -> trimesh.Trimesh:
    return _gripper_jaw(mirror=True)


# ---------------------------------------------------------------------------
# Assembly preview
# ---------------------------------------------------------------------------

# Neutral pose used for the preview.  Chosen so the gripper hangs over
# the chassis edge in a "ready to pick" stance: shoulder pitched down a
# bit, elbow bent forward, wrist straight, jaws ajar.
NEUTRAL_J1_DEG = 0.0      # arm faces the chassis +X edge
NEUTRAL_J2_DEG = -25.0    # shoulder pitch (= HP.STANCE_FEMUR_DEG -- same drop)
NEUTRAL_J3_DEG = 60.0     # elbow pitch  (= HP.STANCE_TIBIA_DEG)
NEUTRAL_J4_DEG = 25.0     # wrist pitch -- nudge the gripper slightly forward
NEUTRAL_JAW_DEG = 18.0    # jaws open at this half-angle


def _yaw_output_world_z():
    """Vertical offset from chassis-top TOP face up to the J1 disc
    horn's TOP face (= where arm_shoulder_link.z=0 lands)."""
    return ((HP.SERVO_BODY_H - HP.WELL_RIM_Z)
            + HP.SERVO_OUTPUT_H
            + HP.HORN_STACK_H)          # 5 mm aluminum 25T disc horn
                                        # (no printed adapter; disc top
                                        # face = link mating plane)


def _arm_in_chassis_frame(j1_deg: float = NEUTRAL_J1_DEG,
                          j2_deg: float = NEUTRAL_J2_DEG,
                          j3_deg: float = NEUTRAL_J3_DEG,
                          j4_deg: float = NEUTRAL_J4_DEG,
                          jaw_deg: float = NEUTRAL_JAW_DEG,
                          chassis_top_z: float = 0.0):
    """Return a list of meshes representing the full arm assembly,
    placed in the chassis frame.

    The chassis-top TOP face is at world z = chassis_top_z (default 0).
    The J1 yaw axis is at world (0, 0, chassis_top_z).
    """
    parts: list[trimesh.Trimesh] = []

    j1 = np.deg2rad(j1_deg)
    j2 = np.deg2rad(j2_deg)
    j3 = np.deg2rad(j3_deg)
    j4 = np.deg2rad(j4_deg)

    # ---- arm_base_bracket (chassis-fixed) ----
    bracket = make_arm_base_bracket()
    bracket.apply_translation([0, 0, chassis_top_z])
    parts.append(bracket)

    # ---- J1 disc horn (rotates with J1) ----
    # The disc-horn top is exactly J1_HORN_STACK_Z above the bracket
    # origin, which equals chassis_top_z + _yaw_output_world_z().
    j1_stack_top_z = chassis_top_z + _yaw_output_world_z()
    j1_horn = HP.make_disc_horn()
    j1_horn.apply_transform(rotation_matrix(j1, [0, 0, 1]))
    # make_disc_horn's +Z = 0 is its BOTTOM (spline) face and its TOP
    # mating face sits HORN_STACK_H above, so drop it by HORN_STACK_H to
    # land the top face at j1_stack_top_z.
    j1_horn.apply_translation([0, 0, j1_stack_top_z - HP.HORN_STACK_H])
    parts.append(j1_horn)

    # ---- arm_shoulder_link (= coxa_link) — rotates with J1 ----
    shoulder = make_arm_shoulder_link()
    shoulder.apply_transform(rotation_matrix(j1, [0, 0, 1]))
    shoulder.apply_translation([0, 0, j1_stack_top_z])
    parts.append(shoulder)

    # ---- J2 horn output in shoulder-local coords ----
    # (mirrors hexapod_prototype._leg_in_body_frame: hip joint at
    # (COXA_LENGTH, 0, COXA_HIP_DROP) in coxa-link frame.)
    j2_local = np.array([HP.COXA_LENGTH, 0.0, HP.COXA_HIP_DROP])

    def _shoulder_to_world(local_xyz: np.ndarray) -> np.ndarray:
        R = rotation_matrix(j1, [0, 0, 1])[:3, :3]
        return R @ local_xyz + np.array([0, 0, j1_stack_top_z])

    # ---- arm_upper (= femur) — rotates about J2 ----
    upper = make_arm_upper()
    upper.apply_transform(rotation_matrix(j2, [0, 1, 0]))
    upper.apply_translation(j2_local)
    upper.apply_transform(rotation_matrix(j1, [0, 0, 1]))
    upper.apply_translation([0, 0, j1_stack_top_z])
    parts.append(upper)

    # ---- J3 horn output in shoulder-local coords ----
    Ry_j2 = rotation_matrix(j2, [0, 1, 0])[:3, :3]
    j3_local = j2_local + Ry_j2 @ np.array([HP.FEMUR_LENGTH, 0.0, 0.0])

    # ---- arm_forearm (= tibia) — rotates about J3 (cumulative J2+J3) ----
    forearm_pitch = j2 + j3
    forearm = make_arm_forearm()
    forearm.apply_transform(rotation_matrix(forearm_pitch, [0, 1, 0]))
    forearm.apply_translation(j3_local)
    forearm.apply_transform(rotation_matrix(j1, [0, 0, 1]))
    forearm.apply_translation([0, 0, j1_stack_top_z])
    parts.append(forearm)

    # ---- Foot socket tip in shoulder-local coords ----
    Ry_forearm = rotation_matrix(forearm_pitch, [0, 1, 0])[:3, :3]
    tibia_tip_local = j3_local + Ry_forearm @ np.array([HP.TIBIA_LENGTH,
                                                        0.0, 0.0])
    # The foot socket's mouth points in -Z of the tibia frame (= the
    # tibia's "down").  We want the wrist_adapter's plug to insert
    # INTO that mouth, so the wrist_adapter sits at tibia-tip-local
    # with its +Z aligned to the tibia's -Z direction.

    # ---- wrist_adapter — plugged into the foot socket ----
    wrist = make_wrist_adapter()
    # Wrist's +Z is "toward the foot socket" = tibia's -Z.  Flip wrist
    # so its +Z plug points in tibia -Z direction.
    R_wrist_flip = rotation_matrix(np.pi, [1, 0, 0])
    wrist.apply_transform(R_wrist_flip)
    # After flip, the plug now hangs in -Z of the wrist's new local
    # frame.  Push the plug top up into the tibia's foot bore (the
    # foot bore is FOOT_HUB_HEIGHT deep).  We translate so the wrist's
    # top of plug (originally at +WRIST_PLATE_T + WRIST_PLUG_H + 1.0,
    # now at -(WRIST_PLATE_T + WRIST_PLUG_H + 1.0) after flipping) sits
    # at z = 0 in the tibia tip frame (the foot socket bore mouth).
    # i.e. translate by (0, 0, +(WRIST_PLATE_T + WRIST_PLUG_H + 1.0))
    # WAIT — after the flip the plate top is now at z = -WRIST_PLATE_T,
    # the plug is at z in [-WRIST_PLATE_T - WRIST_PLUG_H - 1, -WRIST_PLATE_T].
    # We want the plug TOP (-WRIST_PLATE_T) to land at the tibia
    # foot-socket mouth, which is z = -1.5 in tibia coords (the
    # _cyl(...) for foot_socket starts at z = -(FOOT_HUB_HEIGHT + 4)/2
    # ... let's just say the foot socket mouth is at tibia z = 0 with
    # the bore going down to z = -FOOT_HUB_HEIGHT).
    wrist.apply_translation([0, 0, -WRIST_PLATE_T])

    # The wrist_adapter's local origin was its BOTTOM face (z=0); after
    # the X-axis flip the origin is now at the TOP of the flipped plate
    # (in tibia frame).  Apply J4 rotation about local +Y, with the
    # rotation pivot at the wrist's +Z plug top (which is the actual
    # J4 axis through the foot socket).  But for visualization we
    # simply apply J4 about the wrist's centre origin -- close enough.
    # (The forearm-fixed wrist_adapter doesn't rotate with J4; the
    # gripper_base does.  So skip the J4 rotation on the wrist plate
    # itself.)

    # Place into tibia tip frame: the wrist's origin lands at
    # tibia_tip_local + (0,0,0) = tibia_tip_local.
    wrist.apply_translation(tibia_tip_local)
    # And then through the femur + shoulder + J1 stack:
    wrist.apply_transform(rotation_matrix(j1, [0, 0, 1]))
    wrist.apply_translation([0, 0, j1_stack_top_z])
    parts.append(wrist)

    # ---- gripper_base — rotates with J4 about its top-face axis ----
    # In the wrist-adapter frame (BEFORE the flip), the gripper_base
    # sits with its TOP face at wrist z = 0 (= wrist's bottom face)
    # and hangs in -Z.  After the wrist flip (rotation by pi about
    # +X), the gripper's frame is upside-down in tibia coords: its
    # +Z (toward jaws) points in tibia +Z, and the gripper hangs in
    # tibia +Z above the foot socket... that's the wrong direction.
    #
    # Easier: build the gripper_base in its OWN frame with +Z = toward
    # jaws (its natural frame), then apply the same R_wrist_flip used
    # for the wrist_adapter so the gripper hangs in tibia -Z, then
    # additionally apply J4 about local +Y.
    gripper = make_gripper_base()
    # J4 rotates the gripper about its TOP-face axis (= local origin).
    # Equivalent to a pre-rotation about local +Y.
    gripper.apply_transform(rotation_matrix(j4, [0, 1, 0]))
    # Same flip as wrist so the gripper hangs below the wrist.
    gripper.apply_transform(R_wrist_flip)
    # Now gripper's TOP face is at the wrist_adapter's BOTTOM face level.
    # Wrist bottom face in tibia frame = -WRIST_PLATE_T (post-flip,
    # post-translate).  Place gripper TOP at that level so they kiss.
    # After the flip, the gripper's TOP face (was z = 0) is at z = 0
    # in the flipped frame too (it's the centre of rotation).  The
    # gripper extends in +Z (now -Z of original) so a translation in
    # -Z by 0 leaves the top kissing the wrist bottom (-WRIST_PLATE_T).
    gripper.apply_translation([0, 0, -WRIST_PLATE_T])
    gripper.apply_translation(tibia_tip_local)
    gripper.apply_transform(rotation_matrix(j1, [0, 0, 1]))
    gripper.apply_translation([0, 0, j1_stack_top_z])
    parts.append(gripper)

    # ---- Jaws on the gripper_base's bottom (jaw-tip side) ----
    # The gripper's pivot pin axis is at gripper-local (GBASE_W/2 - 6,
    # 0, GBASE_H - 4) -- in OUR frame +Z = toward jaws -- so the pin
    # sits near the jaws-end of the body, on the +X side.
    pivot_local = np.array([GBASE_W / 2.0 - 6.0, 0.0, GBASE_H - 4.0])
    for mirror, sign in ((False, +1.0), (True, -1.0)):
        jaw = _gripper_jaw(mirror=mirror)
        # Open by NEUTRAL_JAW_DEG (rotate about +Y; mirrored jaw goes
        # the other way).  +Y rotation lifts the +Z end toward +X.
        # Mirrored jaw opens to -X, so we rotate by -jaw_deg there.
        jaw.apply_transform(rotation_matrix(sign * np.deg2rad(jaw_deg),
                                            [0, 1, 0]))
        jaw.apply_translation(pivot_local)
        # Same gripper-base transform chain.
        jaw.apply_transform(rotation_matrix(j4, [0, 1, 0]))
        jaw.apply_transform(R_wrist_flip)
        jaw.apply_translation([0, 0, -WRIST_PLATE_T])
        jaw.apply_translation(tibia_tip_local)
        jaw.apply_transform(rotation_matrix(j1, [0, 0, 1]))
        jaw.apply_translation([0, 0, j1_stack_top_z])
        parts.append(jaw)

    return parts


def make_arm_assembly_preview() -> trimesh.Trimesh:
    """Full arm in neutral pose, attached to a stand-in chassis-top
    plate so the bolt pattern is visible.  Returned in Z-up.

    The mesh is a simple concatenation of part meshes (no boolean
    union) so each printed STL keeps its own surface in the preview —
    this is just a visualization artefact, not a manifold model.
    """
    parts: list[trimesh.Trimesh] = []

    # Chassis top plate (read-only re-use; we don't modify _hex_plate).
    chassis_top = HP.make_chassis_top()
    parts.append(chassis_top)
    # The chassis_top is centred on its own z = 0 (plate centre).  Its
    # TOP face is at z = +CHASSIS_PLATE_T/2.  Arm origin sits on that.
    chassis_top_z = HP.CHASSIS_PLATE_T / 2.0

    parts.extend(_arm_in_chassis_frame(chassis_top_z=chassis_top_z))

    return trimesh.util.concatenate(parts)


# ---------------------------------------------------------------------------
# Reach / workspace metrics
# ---------------------------------------------------------------------------

def _gripper_tip_local(j2_deg: float, j3_deg: float, j4_deg: float):
    """Return the (radial, axial) position of the gripper tip in the
    arm's local plane (J1 = 0 frame), starting from the J1 axis at the
    chassis top.
    """
    j2 = np.deg2rad(j2_deg)
    j3 = np.deg2rad(j3_deg)
    j4 = np.deg2rad(j4_deg)

    shoulder = np.array([HP.COXA_LENGTH, 0.0,
                         _yaw_output_world_z() + HP.COXA_HIP_DROP])

    upper_dir = np.array([np.cos(j2), 0.0, np.sin(j2)])
    elbow = shoulder + HP.FEMUR_LENGTH * upper_dir

    forearm_dir = np.array([np.cos(j2 + j3), 0.0, np.sin(j2 + j3)])
    wrist_root = elbow + HP.TIBIA_LENGTH * forearm_dir

    # Wrist + gripper offset along the wrist's "down" axis (= tibia
    # local -Z, rotated into world by the cumulative pitch).  This is
    # approximate because we ignore the J4 rotation pivot offset.
    wrist_down = np.array([-np.sin(j2 + j3 + j4), 0.0,
                           -np.cos(j2 + j3 + j4)])
    gripper_tip = wrist_root + WRIST_TO_GRIPPER_TIP * wrist_down

    return gripper_tip


def reach_summary():
    """Quick reach / workspace metrics for the README + final print."""
    neutral = _gripper_tip_local(NEUTRAL_J2_DEG, NEUTRAL_J3_DEG,
                                 NEUTRAL_J4_DEG)
    full_out = _gripper_tip_local(0.0, 0.0, 0.0)
    fully_down = _gripper_tip_local(-90.0, 0.0, 0.0)

    # Workspace radius (straight-out 2D distance from J1 axis to gripper tip).
    r_max = float(np.hypot(full_out[0], full_out[2] - _yaw_output_world_z()))

    return {
        "neutral_tip": neutral,
        "neutral_reach_xy": float(np.hypot(neutral[0], neutral[1])),
        "full_out_tip": full_out,
        "fully_down_tip": fully_down,
        "workspace_radius_mm": r_max,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    print("Hexapod walker ARM ADD-ON — generating STLs in stl_arm/ ...")

    parts: list[tuple[str, trimesh.Trimesh]] = []

    print("New printable parts:")
    parts.append(("arm_base_bracket.stl",  make_arm_base_bracket()))
    parts.append(("wrist_adapter.stl",     make_wrist_adapter()))
    parts.append(("gripper_base.stl",      make_gripper_base()))
    parts.append(("gripper_jaw_left.stl",  make_gripper_jaw_left()))
    parts.append(("gripper_jaw_right.stl", make_gripper_jaw_right()))

    print("Re-exports of leg parts (printed for the arm shoulder / "
          "upper / forearm):")
    parts.append(("arm_shoulder_link.stl", make_arm_shoulder_link()))
    parts.append(("arm_upper.stl",         make_arm_upper()))
    parts.append(("arm_forearm.stl",       make_arm_forearm()))

    for name, mesh in parts:
        _save(mesh, name)

    print("Assembly preview (arm in neutral pose on top of chassis):")
    preview = make_arm_assembly_preview()
    _save(preview, "arm_assembly_preview_DO_NOT_PRINT.stl", reference=True)

    metrics = reach_summary()
    new_count = 5
    reused_count = 3
    tip = metrics["neutral_tip"]
    print()
    print(f"OK -- wrote {len(parts)} printables under stl_arm/ "
          f"+ 1 preview under stl_reference/.")
    print(f"  New printable parts:        {new_count}")
    print(f"  Re-exported leg parts:      {reused_count}")
    print(f"  Neutral-pose gripper tip:   "
          f"({tip[0]:+.1f}, {tip[1]:+.1f}, {tip[2]:+.1f}) mm from chassis centre")
    print(f"  Neutral-pose XY reach:      "
          f"{metrics['neutral_reach_xy']:.1f} mm")
    print(f"  Maximum workspace radius:   "
          f"{metrics['workspace_radius_mm']:.1f} mm")


if __name__ == "__main__":
    main()
