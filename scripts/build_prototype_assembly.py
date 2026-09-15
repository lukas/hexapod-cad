"""Build the full assembled PROTOTYPE hexapod walker (hobby-servo
edition) and export it as four category STLs the Blender render script
treats as separate materials.

Imports the part builders from `hexapod_prototype.py`, then places:
    - 1 chassis (top + bottom plate)
    - 1 battery holder + a 3S LiPo pack
    - 1 electronics tray + Arduino Nano + PCA9685 PWM driver
    - 6 leg sub-assemblies, each with:
        coxa bracket, coxa link, femur, tibia, foot
        + 3 hobby servo bodies (yaw, hip-pitch, knee-pitch)
        + 3 aluminum 25T disc servo horns (Amazon B07D56FVK5 -- the
          links now bolt DIRECTLY onto these with 4 x M3 x 6 SHCS; the
          printed servo_horn_adapter disc and the stock plastic 4-arm
          X-horn are both retired, see hexapod_prototype.py)

Outputs (in ./artifacts/assembly/):
    frame.stl       printed PLA / PETG structural parts
    motors.stl      18 black plastic hobby servo bodies
    battery.stl     LiPo pack + Arduino + PCA9685
    soft.stl        (empty since Aug 2026 -- TPU boots live in frame.stl)
    full.stl        single-mesh dump for non-Blender viewers

All output is rotated to Y-up so it matches the convention used by
build_full_assembly.py (and Blender's STL importer with up=Y).
"""

from __future__ import annotations

import argparse
import os
import sys
import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.dirname(SCRIPT_DIR)

import hexapod_prototype as HP  # noqa: E402


OUT_DIR = os.path.join(PROTO_DIR, "artifacts", "assembly")
os.makedirs(OUT_DIR, exist_ok=True)


# ---------------------------------------------------------------------------
# Local helpers (re-using HP where possible)
# ---------------------------------------------------------------------------

def _cyl(radius: float, height: float, sections: int = HP.CYL_SECTIONS):
    return trimesh.creation.cylinder(radius=radius, height=height,
                                      sections=sections)


def _box(extents, center=(0.0, 0.0, 0.0)):
    m = trimesh.creation.box(extents=extents)
    m.apply_translation(center)
    return m


# ---------------------------------------------------------------------------
# Visually rich hobby-servo body (used 18 x in the assembly)
# ---------------------------------------------------------------------------

def _hobby_servo_visual() -> trimesh.Trimesh:
    """A nicer-looking hobby servo than HP._servo_envelope.

    Local frame matches HP._servo_envelope:
        +Z = output shaft
        +X = body long axis (output offset toward +X)
        +Y = body short axis (depth)
        Origin = centre of body bottom face

    Adds details:
        - Chamfered body (subtle) with a top-cap groove
        - Output gear stack
        - Mounting tabs with a couple of tab-screws modelled
        - 3-wire cable bundle exiting on the -X side
    """
    # STS3215 visual = the model's own servo envelope (body + output hub
    # + spline + idler + bus-cable boot), so the inspector renders exactly
    # match the geometry the cradles cut their pocket from.
    parts = [HP._servo_envelope()]

    # 3-wire bus harness exiting the +X short face.
    bundle = _cyl(2.2, 18.0)
    bundle.apply_transform(rotation_matrix(np.pi / 2, [0, 1, 0]))
    bundle.apply_translation([+HP.SERVO_BODY_W / 2.0 + 9.0,
                               0, HP.SERVO_BODY_H * 0.55])
    parts.append(bundle)

    return trimesh.util.concatenate(parts)


def _horn_visual() -> trimesh.Trimesh:
    """The 20 mm aluminum 25T DISC servo horn (Amazon B07D56FVK5) that
    every servo joint now drives.  Used 18 x to dress the output of
    every joint; the driven link bolts onto the disc's flat top face
    with 4 x M3 SHCS into the disc's tapped holes.

    Delegates to ``HP.make_disc_horn`` so the assembly, the static
    ``servo_horn.stl`` and the verifier all share ONE disc geometry.

    Local frame: bottom face (spline side) at z=0, top mating face at
    z=HP.HORN_STACK_H = HP.DISC_HORN_H = 5 mm, so callers that
    translate by ``horn_base_z`` get a disc whose mating face lands at
    ``horn_base_z + HP.HORN_STACK_H`` -- the same reference the plastic
    X-horn used, so no placement math changes.
    """
    return HP.make_disc_horn()


# ---------------------------------------------------------------------------
# Per-leg category meshes
# ---------------------------------------------------------------------------

def _build_leg(leg_index: int):
    """Return (frame_parts, motor_parts, soft_parts) for one leg in
    chassis frame.  Uses the same kinematic chain as
    HP._leg_in_body_frame but splits output by category and inserts a
    hobby-servo body + horn at every joint.
    """
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3
    edge_mid = np.array([apothem * np.cos(a),
                         apothem * np.sin(a),
                         0.0])
    z_hat = np.array([0.0, 0.0, 1.0])

    frame_parts = []
    motor_parts = []
    soft_parts  = []

    # Standing-pose pitch angles
    p  = np.deg2rad(HP.STANCE_FEMUR_DEG)
    pt = np.deg2rad(HP.STANCE_FEMUR_DEG + HP.STANCE_TIBIA_DEG)

    R_a = rotation_matrix(a, [0, 0, 1])
    R_a_3 = R_a[:3, :3]

    # Yaw output reference height (above the chassis edge plane).
    # Must match HP._leg_in_body_frame's vertical stack.  May 2026:
    # the yaw servo migrated from the standalone ``coxa_bracket`` to
    # the integrated cradle inside ``chassis_bottom``.  Commit 4 of
    # the transition (this commit) flips the assembly's yaw-servo
    # placement so the tabs land on the chassis_bottom cradle's tab
    # shelf at chassis-z = CHASSIS_PLATE_T/2 + CRADLE_TAB_SHELF_Z =
    # +2 + +6 = +8 mm (= +8 mm UP from the bracket shelf which was
    # at chassis-z = 0).  The disc-horn / coxa_link mating plane
    # follows the servo: +8 mm UP from the legacy yaw_output_z =
    # +21.75 to the new +29.75.  See the constant block near the
    # CRADLE_BOSS_H_MM definition in hexapod_prototype.py for the
    # Path-A history that placed the cradle shelf at +6.
    #
    # Z-stack (chassis-z = 0 = chassis_bottom CENTRE; world-z post-
    # lift is chassis-z + chassis_lift):
    #   chassis_bottom top     z = +CHASSIS_PLATE_T/2 = +2
    #   cradle tab shelf       z = +CHASSIS_PLATE_T/2
    #                                + CRADLE_TAB_SHELF_Z = +8
    #   servo body bottom      z = shelf - WELL_RIM_Z = -19.25
    #   servo body top         z = body_bottom + SERVO_BODY_H = +18.75
    #   output-gear top        z = body_top + SERVO_OUTPUT_H = +24.75
    #   disc-horn top (yaw_output_z): z = gear_top + HORN_STACK_H = +29.75
    #   chassis_top bottom     z = +CHASSIS_TOP_BOT_Z = +34
    #                                (2 mm plate, Aug 2026; deck face +36)
    #     (= 32 - 29.75 = 2.25 mm clearance from disc-horn top to top
    #      plate; only relevant at the yaw axis which sits outside
    #      chassis_top's 70-mm apothem hex anyway, so no XY overlap)
    #
    # The legacy bracket is still PLACED in the assembly during this
    # transition commit; the bracket's well well-z = -WELL_RIM_Z body
    # bottom would be at chassis-z = -27.25 (8 mm BELOW the new servo
    # position).  We place the servo at the NEW cradle position; the
    # bracket overlaps the cradle's outer shell in chassis-z [+2, +11]
    # and the servo body hovers above the bracket's well floor at
    # chassis-z = -19.25 (8 mm above the bracket's nominal well-
    # floor seating plane).  This visual overlap is intentional for
    # the transition; commit 8 retires the bracket entirely.
    # (chassis_bottom cradle's tab shelf sits at chassis-z =
    #  CHASSIS_PLATE_T/2 + CRADLE_TAB_SHELF_Z = +8 mm; the yaw servo +
    #  horn placement below is keyed off CHASSIS_YAW_OUTPUT_Z directly.)

    yaw_output_z = HP.CHASSIS_YAW_OUTPUT_Z   # = +29.75 mm; disc-horn top
    yaw_output_world = edge_mid + yaw_output_z * z_hat

    arm_t = 6.0       # MUST match make_coxa_link()'s arm_t
    hip_drop = HP.COXA_HIP_DROP
    hip_joint_local = np.array(HP.COXA_HIP_ANCHOR)
    Ry_p_3 = rotation_matrix(p, [0, 1, 0])[:3, :3]
    # Femur's knee-end joint axis is on the spar centreline at z=0
    # in femur local, so we don't add a 'drop' offset for the knee.
    knee_joint_local = hip_joint_local + Ry_p_3 @ np.array([HP.FEMUR_LENGTH,
                                                             0.0, 0.0])
    # NEW (May 2026 collinear-pad refactor): the femur / tibia NEW
    # local origins are their PAD MATING FACES, which sit HORN_STACK_H
    # above their respective joint axes along link +Y.  Femur +Y is
    # parallel to coxa +Y here (the hip-pitch axis), so the entire
    # leg shifts +HORN_STACK_H in coxa-Y as a rigid body relative to
    # the joint axes.  Pre-refactor the links' local origins were on
    # the joint axes themselves (no +Y offset).
    PAD_AXIS_OFFSET = np.array([0.0, HP.HORN_STACK_H, 0.0])

    def to_world(mesh):
        mesh.apply_transform(R_a)
        mesh.apply_translation(yaw_output_world)
        return mesh

    # ------------- Yaw servo (output up) ------------------------------
    # Servo body sits below the chassis with output gear poking up.
    # In bracket-local: body bottom at z = -SERVO_BODY_H, output at z=0.
    # The bracket itself is in leg-local coords with edge_mid at z=0.
    yaw_servo = _hobby_servo_visual()
    # The yaw servo body sits in the chassis_bottom-integrated yaw
    # cradle.  In the chassis frame (yaw axis at x=0, chassis_bottom
    # centre at z=0, chassis_bottom top at z = +CHASSIS_PLATE_T/2),
    # the body's bottom face lands at chassis-z =
    # _CRADLE_SHELF_CHASSIS_Z - WELL_RIM_Z (the cradle's tab shelf
    # rests under the servo's tab bottoms) and the body's long axis
    # is along +X with output offset toward +X.
    # ``_hobby_servo_visual`` has body bottom at z=0 and output (front)
    # face at z = SERVO_BODY_H.  The front face seats against the mount-plate
    # UNDERSIDE, which sits WELL_PLATE_T below the plate top and HORN_STACK_H +
    # WELL_PLATE_T below the frozen output plane (CHASSIS_YAW_OUTPUT_Z).  Land
    # the front face there so the body matches the printed cradle stack-up.
    # (Jun 2026 flush-output DEPTH fix: the old placement omitted WELL_PLATE_T
    # and sat the servo 4 mm too high, baking in a phantom output protrusion.)
    yaw_face_z = (HP.CHASSIS_YAW_OUTPUT_Z - HP.HORN_STACK_H - HP.WELL_PLATE_T)
    yaw_servo.apply_translation(
        [-HP.SERVO_OUTPUT_X, 0, yaw_face_z - HP.SERVO_BODY_H],
    )
    yaw_servo.apply_transform(R_a)
    yaw_servo.apply_translation(edge_mid)
    motor_parts.append(yaw_servo)

    # Yaw disc horn -- bottom (spline side) seats FLUSH on the servo front face
    # (the output does NOT protrude), RECESSED inside the mount-plate bore.  Its
    # base lands at yaw_face_z so its top sits DISC_HORN_H above the face, where
    # the coxa_link's necked drive nub reaches it.  Drives the coxa link via
    # the 4 link-to-disc-horn M3 bolts.
    yaw_horn = _horn_visual()
    yaw_horn.apply_translation(
        [0, 0, yaw_face_z],
    )
    yaw_horn.apply_transform(R_a)
    yaw_horn.apply_translation(edge_mid)
    motor_parts.append(yaw_horn)

    # ------------- Coxa link (rotates with yaw, frame) -----------------
    cl = HP.make_coxa_link()
    cl.apply_transform(R_a)
    cl.apply_translation(yaw_output_world)
    frame_parts.append(cl)

    # ------------- Hip-pitch servo (output along +Y_local) -------------
    # The cradle in the coxa link expects the servo with:
    #   servo +Z (output) = leg +Y_local
    #   servo body extends in -Y_local from the output
    # Output point lives at (COXA_LENGTH, 0, hip_drop) in coxa-link
    # local.  In leg-local the servo's local frame is rotated by -90
    # about +X (servo +Z -> leg +Y, servo +Y -> leg -Z).
    hip_servo = _hobby_servo_visual()
    R_hip = rotation_matrix(-np.pi / 2.0, [1, 0, 0])
    hip_servo.apply_transform(R_hip)
    # After R_hip the servo's body bottom (z=0) maps to post-y=0
    # and the spline tip (z=SERVO_BODY_H+SERVO_OUTPUT_H) maps to
    # post-y=SERVO_BODY_H+SERVO_OUTPUT_H.  We want the spline tip to
    # land on the joint axis at (COXA_LENGTH, 0, hip_drop):
    delta = np.array([HP.COXA_LENGTH - HP.SERVO_OUTPUT_X,
                       -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H) + HP.COXA_HIP_ANCHOR_Y,
                       hip_drop])
    hip_servo.apply_translation(delta)
    to_world(hip_servo)
    motor_parts.append(hip_servo)

    # Hip sandwich-joint clamp cap: closes the coxa_link cradle's OPEN
    # +Y face around the STS3215 body.  Modelled in the servo well-local
    # frame, so it reuses the IDENTICAL transform chain as hip_servo.
    hip_cap = HP.make_servo_clamp_cap()
    hip_cap.apply_transform(R_hip)
    hip_cap.apply_translation(delta)
    to_world(hip_cap)
    frame_parts.append(hip_cap)

    # Hip horn -- bolted to the servo output, sticking out into +Y
    # from the cradle face.  The femur's hip-pad clamps onto this.
    hip_horn = _horn_visual()
    R_hip_horn = rotation_matrix(-np.pi / 2.0, [1, 0, 0])
    hip_horn.apply_transform(R_hip_horn)
    hip_horn.apply_translation([HP.COXA_LENGTH, HP.COXA_HIP_ANCHOR_Y, hip_drop])
    to_world(hip_horn)
    motor_parts.append(hip_horn)

    # ------------- Femur (rotates with hip-pitch) ----------------------
    fl = HP.make_femur_link()
    fl.apply_transform(rotation_matrix(p, [0, 1, 0]))
    fl.apply_translation(hip_joint_local + PAD_AXIS_OFFSET)
    to_world(fl)
    frame_parts.append(fl)

    # ------------- Knee-pitch servo (output along +Y_local) ------------
    # Lives at the femur's far end.  In NEW femur-local coords the
    # knee servo's spline tip is at (FEMUR_LENGTH, -HORN_STACK_H, 0)
    # (the joint axis is HORN_STACK_H below the link's NEW origin =
    # pad mating face); the body sits HORN_STACK_H deeper in -Y than
    # the pre-refactor placement so its world position is unchanged.
    knee_servo = _hobby_servo_visual()
    knee_servo.apply_transform(R_hip)
    delta = np.array([HP.FEMUR_LENGTH - HP.SERVO_OUTPUT_X,
                       -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H) - HP.HORN_STACK_H,
                       0])
    knee_servo.apply_translation(delta)
    knee_servo.apply_transform(rotation_matrix(p, [0, 1, 0]))
    knee_servo.apply_translation(hip_joint_local + PAD_AXIS_OFFSET)
    to_world(knee_servo)
    motor_parts.append(knee_servo)

    # Knee sandwich-joint clamp cap (same well-local frame as knee_servo).
    knee_cap = HP.make_servo_clamp_cap()
    knee_cap.apply_transform(R_hip)
    knee_cap.apply_translation(delta)
    knee_cap.apply_transform(rotation_matrix(p, [0, 1, 0]))
    knee_cap.apply_translation(hip_joint_local + PAD_AXIS_OFFSET)
    to_world(knee_cap)
    frame_parts.append(knee_cap)

    knee_horn = _horn_visual()
    knee_horn.apply_transform(R_hip_horn)
    knee_horn.apply_translation([HP.FEMUR_LENGTH, -HP.HORN_STACK_H, 0])
    knee_horn.apply_transform(rotation_matrix(p, [0, 1, 0]))
    knee_horn.apply_translation(hip_joint_local + PAD_AXIS_OFFSET)
    to_world(knee_horn)
    motor_parts.append(knee_horn)

    # ------------- Tibia (rotates with knee-pitch) ---------------------
    tl = HP.make_tibia_link()
    tl.apply_transform(rotation_matrix(pt, [0, 1, 0]))
    tl.apply_translation(knee_joint_local + PAD_AXIS_OFFSET)
    to_world(tl)
    frame_parts.append(tl)

    # ------------- Foot at tibia tip ----------------------------------
    # Aug 2026: the TPU foot_boot is unioned into make_tibia_link above
    # (pressed over the tube end), so no separate foot part is placed.

    return frame_parts, motor_parts, soft_parts


# ---------------------------------------------------------------------------
# Body parts builders (categorised)
# ---------------------------------------------------------------------------

def _body_frame_parts(chassis_lift):
    parts = []
    bot = HP.make_chassis_bottom()
    bot.apply_translation([0, 0, chassis_lift])
    parts.append(bot)

    # Jun 2026: the bottom plate is a SINGLE merged part again (the old HIGH/LOW
    # print split is folded back into make_chassis_bottom).

    top = HP.make_chassis_top()
    top.apply_translation([0, 0, chassis_lift + HP.CHASSIS_TOP_CENTRE_Z])
    parts.append(top)

    # 4 stand-off posts between the plates on the rotated-45-deg 35-mm-
    # radius pattern (= HP.CHASSIS_STANDOFF_HOLES_XY = (+/-35, 0) and
    # (0, +/-35)).  May 2026 tray-mount fix: the standoffs MOVED off
    # HP.ELEC_CHASSIS_MOUNT_HOLES_XY (the +/-24.75 square pattern) so
    # the chassis_bottom plate could carry an M3 heat-set insert at
    # each of those 4 XY positions for the tray-mount bolts without
    # the brass standoff body conflicting with the insert pocket.
    # Each standoff is M3 F-F, length HP.CHASSIS_GAP = 20 mm.
    for (sx, sy) in HP.CHASSIS_STANDOFF_HOLES_XY:
        post = _cyl(2.5, HP.CHASSIS_GAP)
        post.apply_translation([sx, sy,
                                  chassis_lift + HP.CHASSIS_PLATE_T
                                      + HP.CHASSIS_GAP / 2.0])
        parts.append(post)

    return parts


def _body_battery_parts(chassis_lift):
    """Two under-belly shorty LiPo packs + as-built electronics
    stack (Aug 2026): trunk Wagos + motor ctrl on chassis_top, 4 magnet posts,
    hex mount plate + Uno Q / breakout, raised platform with screen / MPU,
    power Wagos on top periphery + data Wagos under chassis.

    RETIRED: clip-in battery_holder, in-gap electronics_tray, stacked
    uno_q_tray / buck_tray / spider_carapace / chassis-top imu_pad.
    """
    parts = []

    # As-built electronics (posts, hex board, raised platform, Wagos,
    # and the two under-belly shorty LiPo packs -- Aug 2026: no bay
    # battery; HP.asbuilt_electronics_local_parts places the packs).
    Tlift = np.eye(4)
    Tlift[2, 3] = chassis_lift
    for _name, mesh, M0 in HP.asbuilt_electronics_local_parts():
        m = mesh.copy()
        m.apply_transform(Tlift @ M0)
        parts.append(m)

    # Switch holster: anti-spark on/off switch.  Sits on 2 printed
    # bosses on chassis_top's top face.
    chassis_top_top_z = chassis_lift + HP.CHASSIS_TOP_TOP_Z
    holster_z = chassis_top_top_z + HP.SWITCH_HOLSTER_BOSS_H
    holster = HP.make_switch_holster()
    holster.apply_translation([HP.SWITCH_HOLSTER_CENTRE_X,
                                HP.SWITCH_HOLSTER_CENTRE_Y,
                                holster_z])
    parts.append(holster)

    return parts


def _body_soft_parts(chassis_lift):
    """No saddle/grips on the prototype.  Aug 2026: the TPU foot boots
    are unioned into each tibia_link (frame.stl), so soft.stl is now
    legitimately empty (cat() skips writing empty groups)."""
    return []


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _yup(mesh):
    mesh.apply_transform(rotation_matrix(-np.pi / 2.0, [1, 0, 0]))
    return mesh


def _optional_arm_meshes(chassis_lift: float) -> list[trimesh.Trimesh]:
    """Build the optional arm in neutral pose, placed on top of the
    standing hexapod's chassis top plate.  Returns an empty list if
    the arm module can't be imported (e.g. ``prototype/arm/`` is
    missing).  No frame coordinate flip is applied here -- the caller
    must run them through the same ``_yup()`` rotation as the rest of
    the assembly meshes.
    """
    _arm_dir = os.path.join(PROTO_DIR, "arm")
    if _arm_dir not in sys.path:
        sys.path.insert(0, _arm_dir)
    try:
        import arm as ARM_MOD  # noqa: WPS433
    except Exception as exc:
        print(f"  WARN: optional arm import failed ({exc!r}); skipping arm.")
        return []

    # Chassis-top TOP face in assembly frame (where the arm bracket
    # lands); CHASSIS_TOP_TOP_Z is the authoritative deck-face height.
    chassis_top_face_z = chassis_lift + HP.CHASSIS_TOP_TOP_Z
    arm_parts = ARM_MOD._arm_in_chassis_frame(  # noqa: SLF001
        chassis_top_z=chassis_top_face_z,
    )
    return arm_parts


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--with-arm",
        action="store_true",
        default=os.environ.get("HEXAPOD_PROTOTYPE_WITH_ARM") == "1",
        help=(
            "Place the OPTIONAL 5-DOF arm on top of the chassis in the "
            "assembly preview (in addition to the standard 6-leg body). "
            "Off by default; respects HEXAPOD_PROTOTYPE_WITH_ARM=1 in "
            "the environment so build_all.py can propagate the flag."
        ),
    )
    # Tolerate being invoked without argv (e.g. via importlib.import_module
    # in build_all.py, which then calls .main()) by defaulting argv to []
    # so argparse doesn't read sys.argv twice.
    if argv is None:
        argv = []
    args, _ = parser.parse_known_args(argv)

    print("Building full hexapod walker PROTOTYPE assembly ...")
    if args.with_arm:
        print("  --with-arm: including optional arm in neutral pose.")

    probe_frame, probe_motor, probe_soft = _build_leg(0)
    probe_meshes = probe_frame + probe_motor + probe_soft
    z_min = min(float(m.bounds[0][2]) for m in probe_meshes)
    chassis_lift = -z_min
    print(f"  chassis_lift computed as {chassis_lift:.1f} mm "
          f"(foot pad bottom at z = {z_min:.1f})")

    frame_meshes  = []
    motor_meshes  = []
    soft_meshes   = []
    battery_meshes = []

    frame_meshes.extend(_body_frame_parts(chassis_lift))
    battery_meshes.extend(_body_battery_parts(chassis_lift))
    soft_meshes.extend(_body_soft_parts(chassis_lift))

    for i in range(6):
        f, m, s = _build_leg(i)
        for part in f:
            part.apply_translation([0, 0, chassis_lift])
            frame_meshes.append(part)
        for part in m:
            part.apply_translation([0, 0, chassis_lift])
            motor_meshes.append(part)
        for part in s:
            part.apply_translation([0, 0, chassis_lift])
            soft_meshes.append(part)

    if args.with_arm:
        arm_meshes = _optional_arm_meshes(chassis_lift)
        if arm_meshes:
            frame_meshes.extend(arm_meshes)
            print(f"  arm: added {len(arm_meshes)} arm part(s) to the "
                  f"assembly preview.")

    def cat(name, meshes_list):
        if not meshes_list:
            print(f"  skip artifacts/assembly/{name:18s} (empty group)")
            return None
        mesh = trimesh.util.concatenate(meshes_list)
        _yup(mesh)
        path = os.path.join(OUT_DIR, name)
        mesh.export(path)
        print(f"  wrote artifacts/assembly/{name:18s}"
              f" {len(mesh.faces):>6d} faces"
              f"  envelope {mesh.extents[0]:5.1f} x "
              f"{mesh.extents[1]:5.1f} x {mesh.extents[2]:5.1f} mm")
        return mesh

    frame   = cat("frame.stl",   frame_meshes)
    motors  = cat("motors.stl",  motor_meshes)
    battery = cat("battery.stl", battery_meshes)
    soft    = cat("soft.stl",    soft_meshes)

    full = trimesh.util.concatenate(
        [m for m in (frame, motors, battery, soft) if m is not None])
    full.export(os.path.join(OUT_DIR, "full.stl"))
    print(f"  wrote artifacts/assembly/full.stl              "
          f"{len(full.faces):>6d} faces")

    print()
    print(f"OK -- wrote 5 STL files under artifacts/assembly/")
    print(f"   Walker envelope (foot to foot):  {full.extents[0]/10:.1f} cm")
    print(f"   Walker standing height:          {full.extents[1]/10:.1f} cm")
    print(f"   Total triangles:                 {len(full.faces):,}")
    print()


if __name__ == "__main__":
    main(sys.argv[1:])
