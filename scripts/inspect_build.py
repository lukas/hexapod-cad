"""Interactive PyVista build inspector for the hexapod walker prototype.

This is the third "view" target alongside the two MuJoCo viewers.  It
loads every per-part STL emitted by ``hexapod_prototype.main`` into a
PyVista scene, places each instance at the assembled-pose transform
used by ``build_prototype_assembly._build_leg`` / ``make_assembly_preview``,
colors it by part type (from ``part_palette.PART_COLORS``), labels it
with its role, and gives you an exploded-view slider so you can pull
the legs out and inspect the joints.

Design
------
* Transforms are recomputed in this file (mirroring the math in
  ``build_prototype_assembly._build_leg`` and ``make_assembly_preview``)
  so the inspector uses the STLs as the source of truth and applies
  numpy 4x4 matrices, instead of carrying around already-baked
  trimesh objects.
* The explode slider does NOT mutate the cached world transforms; it
  only sets each actor's ``user_matrix`` to a translation offset
  proportional to ``(instance_centroid - chassis_centroid)``.  Going
  back to 0.0 returns to the exact assembled pose.
* The per-part-type hide/show checkboxes flip ``actor.visibility``
  on every actor of a given type.  Labels respect visibility too.
* The status overlay at the bottom-left is a real PyVista legend
  built from ``PART_COLORS`` so the user never has to memorize what
  "the teal one" is.

Keyboard shortcuts (also printed at launch)
-------------------------------------------
* ``L`` -- toggle the floating per-instance labels
* ``E`` -- toggle exploded view between 0.0 and 1.5
* ``F`` -- focus the hovered part's sub-assembly (toggle)
* ``I`` / ``Esc`` -- clear focus / isolation
* ``R`` -- reset the camera view
* ``S`` -- save a screenshot to ``artifacts/views/build_inspect.png``
* ``Q`` -- quit (also bound by PyVista's default)

Mouse
-----
* Hover -> show the part role under the cursor in the top banner.
* Left-click a part -> isolate it (dim every other part to 15%).
* Double-click a part -> focus its **sub-assembly** (the servo in its
  cradle if any, the disc horn it rotates with if any, and every
  fastener through it).  Everything else is hidden and the camera
  auto-fits the focused group.  Same effect as ``F`` while hovering.

CLI flags
---------
* ``--screenshot PATH`` -- render once headless and save a PNG, then exit.
* ``--explode FLOAT``   -- initial explode amount in ``[0, 2]``
                          (default 0.0 = exact assembled pose).
* ``--focus PART/Lx``   -- start in focus mode on the named printable
                          part, e.g. ``coxa_link/L0`` or just
                          ``chassis_top`` for chassis-level parts.

Extending
---------
To add another part type:
    1. Add a color to ``part_palette.PART_COLORS``.
    2. Add a label rule in ``part_palette.instance_role``.
    3. Append an instance with the right 4x4 transform in
       :func:`_build_assembly_instances` below.
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass

import numpy as np

# pyvista + trimesh are heavy; defer import where convenient so the
# argparse --help path stays snappy.
import pyvista as pv
import trimesh

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.dirname(SCRIPT_DIR)
for _p in (PROTO_DIR, SCRIPT_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import hexapod_prototype as HP  # noqa: E402
import part_palette as palette  # noqa: E402
import fastener_registry  # noqa: E402
import inspect_build_relations as relations  # noqa: E402


STL_DIR = HP.STL_DIR
FASTENERS_DIR = os.path.join(PROTO_DIR, "fasteners")
ARTIFACTS_DIR = os.path.join(PROTO_DIR, "artifacts", "views")


# Disc-horn height stack (matches build_prototype_assembly.py and
# mujoco_prototype.py via HP.HORN_STACK_H).  June 2026 disc-horn
# switch: with the printed servo_horn_adapter retired, the link's
# mating face sits directly on top of the aluminum disc horn, so the
# stack collapses to HP.HORN_STACK_H = PLASTIC_HORN_H = 5 mm above the
# spline tip (was PLASTIC_HORN_H + HORN_ADAPTER_T = 9 mm).
PLASTIC_HORN_H = HP.HORN_STACK_H   # 5 mm


# ---------------------------------------------------------------------------
# Transform helpers
# ---------------------------------------------------------------------------


def _trans(x: float, y: float, z: float) -> np.ndarray:
    """4x4 pure-translation matrix."""
    m = np.eye(4)
    m[0, 3] = x
    m[1, 3] = y
    m[2, 3] = z
    return m


def _rot_x(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s,  c, 0],
        [0, 0,  0, 1],
    ])


def _rot_y(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [ c, 0, s, 0],
        [ 0, 1, 0, 0],
        [-s, 0, c, 0],
        [ 0, 0, 0, 1],
    ])


def _rot_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ])


# ---------------------------------------------------------------------------
# Per-instance world transforms
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Instance:
    """One placed copy of one STL inside the assembled hexapod."""
    part_type: str
    stl_name: str       # filename inside stl_prototype/ (or fasteners/ if
                        # ``stl_dir`` is set).
    leg_index: int | None
    joint: str | None   # 'yaw' / 'hip' / 'knee' for joint hardware, else None
    transform: np.ndarray  # 4x4 world transform (pre-chassis-lift)
    # Fasteners use ``fasteners/`` instead of ``stl_prototype/``, and
    # carry their own role string built by the registry (e.g.
    # ``"coxa_link L0 hip cradle -X top SHCS"``).  Empty / None for
    # printed + servo parts so nothing changes for them.
    stl_dir: str | None = None
    fastener_role: str | None = None


def _build_assembly_instances() -> list[Instance]:
    """Return every part instance in the assembled robot.

    The transforms returned here do NOT yet include ``chassis_lift``;
    the caller adds a single +Z translation to every instance after
    computing chassis_lift from the bounding boxes of the loaded STLs.
    Matches the structure of ``build_prototype_assembly._build_leg``
    and ``make_assembly_preview`` so a part type appears in the same
    spatial location as in a Cycles render.
    """
    instances: list[Instance] = []

    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    plate_t = HP.CHASSIS_PLATE_T
    gap = HP.CHASSIS_GAP

    # ---------- chassis-level (one each) ----------
    instances.append(Instance(
        "chassis_bottom", "chassis_bottom.stl", None, None, _trans(0, 0, 0),
    ))
    # Jun 2026: the bottom plate is a single merged part (HIGH/LOW split folded
    # back into chassis_bottom).
    instances.append(Instance(
        "chassis_top", "chassis_top.stl", None, None,
        _trans(0, 0, HP.CHASSIS_TOP_CENTRE_Z),
    ))
    # Jun 2026 deck redesign: the clip-in battery_holder + the in-gap
    # electronics_tray (Pi + bus adapter) are retired.  The two stacked
    # electronics decks (Uno Q lower + buck upper) bolt onto 4 standoff
    # columns rising ABOVE chassis_top's top face.
    deck_z0 = HP.CHASSIS_TOP_TOP_Z                     # chassis_top top face
    uno_tray_z = deck_z0 + HP.DECK_LEVEL_1_STANDOFF_H
    buck_tray_z = deck_z0 + HP.DECK_LEVEL_1_STANDOFF_H + HP.DECK_LEVEL_2_STANDOFF_H
    instances.append(Instance(
        "uno_q_tray", "uno_q_tray.stl", None, None,
        _trans(0.0, 0.0, uno_tray_z),
    ))
    instances.append(Instance(
        "buck_tray", "buck_tray.stl", None, None,
        _trans(0.0, 0.0, buck_tray_z),
    ))
    # Spider carapace dome bolts on as a 3rd deck level above the buck tray
    # (4 M3 standoffs at DECK_COLUMN_XY).  Its local rim/seat plane sits at
    # deck_z0 + L1 + L2 + L3 so it clears the whole electronics stack.
    carapace_z = (deck_z0 + HP.DECK_LEVEL_1_STANDOFF_H
                  + HP.DECK_LEVEL_2_STANDOFF_H
                  + HP.DECK_LEVEL_3_STANDOFF_H)
    instances.append(Instance(
        "spider_carapace", "spider_carapace.stl", None, None,
        _trans(0.0, 0.0, carapace_z),
    ))
    # Switch holster sits on TWO printed BOSSES on chassis_top's TOP
    # face (one boss under each SWITCH_HOLSTER_BOLT_CHASSIS_XY
    # position).  Ear bottom rests on the boss tops at
    # z = chassis_top_top + SWITCH_HOLSTER_BOSS_H = gap + 1.5 *
    # plate_t + BOSS_H.  Bolts thread DOWN from above the ear into
    # the brass heat-set insert in each boss.
    sw_z = HP.CHASSIS_TOP_TOP_Z + HP.SWITCH_HOLSTER_BOSS_H
    instances.append(Instance(
        "switch_holster", "switch_holster.stl", None, None,
        _trans(HP.SWITCH_HOLSTER_CENTRE_X, HP.SWITCH_HOLSTER_CENTRE_Y,
                sw_z),
    ))
    # IMU pad sits on TOP of chassis_top at the chassis centre of mass
    # (chassis (0, 0)).  Pad's BOTTOM face is HP.IMU_PAD_TAPE_T above
    # chassis_top's TOP face (the gap holds the double-sided foam tape
    # that doubles as the mount AND the vibration damper).  No
    # fasteners between the pad and chassis_top.
    chassis_top_top_z = HP.CHASSIS_TOP_TOP_Z
    imu_z = chassis_top_top_z + HP.IMU_PAD_TAPE_T
    instances.append(Instance(
        "imu_pad", "imu_pad.stl", None, None,
        _trans(HP.IMU_PAD_CENTRE_X, HP.IMU_PAD_CENTRE_Y, imu_z),
    ))
    # MPU-6050 / GY-521 visual mesh sits on the 4 boss tops of the
    # IMU pad.  Synthesised as a flat IMU_PCB slab written to
    # stl_prototype/mpu6050.stl alongside the other visual-only
    # meshes (servo_body.stl, servo_horn.stl).
    instances.append(Instance(
        "mpu6050", "mpu6050_DO_NOT_PRINT.stl", None, None,
        _trans(
            HP.IMU_PAD_CENTRE_X,
            HP.IMU_PAD_CENTRE_Y,
            imu_z + HP.IMU_PAD_T + HP.IMU_PAD_BOSS_H + HP.IMU_PCB_T / 2.0,
        ),
    ))

    # ----------------------------------------------------------------
    # Non-printed electronics visuals (May 2026 follow-up: BuildViz
    # pass).  Same placement convention as the MPU-6050 above: each
    # visual STL has its origin at the body's geometric centre (or
    # PCB midplane for the boards), and the inspector translates by
    # the chassis-frame coordinates of that centre.  See the
    # ``make_*_visual()`` docstrings in hexapod_prototype.py for the
    # per-mesh axis conventions.
    #
    # The boards sit on the electronics_tray's 5 mm-tall printed
    # standoff bosses; PCB midplane = tray_top + ELEC_STANDOFF_H +
    # COMMODITY_PCB_T/2.  Connector lumps stick UP above the PCB top
    # face -- the verifier's cable-clearance check already covers
    # those volumes via the keepouts in cable_keepouts.py, so the
    # visuals participate in NO design verification.
    # Board visuals sit on their deck's standoff bosses (board PCB
    # bottom face = tray top + DECK_STANDOFF_BOSS_H).  Both
    # make_uno_q_visual / make_buck_converter_visual put their mesh
    # origin at the PCB BOTTOM face (z = 0), so the inspector translates
    # by the board-base z directly (no PCB-midplane offset).
    uno_board_base_z = uno_tray_z + HP.DECK_TRAY_T + HP.DECK_STANDOFF_BOSS_H
    buck_board_base_z = buck_tray_z + HP.DECK_TRAY_T + HP.DECK_STANDOFF_BOSS_H

    instances.append(Instance(
        "uno_q", "uno_q_DO_NOT_PRINT.stl", None, None,
        _trans(0.0, 0.0, uno_board_base_z),
    ))
    # XINGYHENG 12V->5V buck converter on the upper deck (replaces the
    # retired Raspberry Pi + USB-to-TTL bus adapter; the Uno Q drives
    # the STS3215 serial bus directly).
    instances.append(Instance(
        "buck_converter", "buck_converter_DO_NOT_PRINT.stl", None, None,
        _trans(0.0, 0.0, buck_board_base_z),
    ))

    # Anti-spark switch BODY sits in the switch_holster's socket
    # cavity.  Holster local frame: +X = toggle face; socket cavity
    # centred at (socket_centre_x, 0, FLOOR + BODY_H/2) where
    # socket_centre_x = outer_l/2 - socket_l/2.  In chassis frame:
    # (HOLSTER_CENTRE_X + socket_centre_x, HOLSTER_CENTRE_Y,
    #  sw_z + FLOOR + BODY_H/2).
    socket_centre_x_local = (HP.SWITCH_HOLSTER_OUTER_L / 2.0
                              - HP.SWITCH_SOCKET_OUTER_L / 2.0)
    switch_body_z = (sw_z + HP.SWITCH_HOLSTER_FLOOR
                      + HP.SWITCH_BODY_H / 2.0)
    switch_body_x = HP.SWITCH_HOLSTER_CENTRE_X + socket_centre_x_local
    instances.append(Instance(
        "antispark_switch", "antispark_switch_body_DO_NOT_PRINT.stl", None, None,
        _trans(switch_body_x, HP.SWITCH_HOLSTER_CENTRE_Y, switch_body_z),
    ))
    # Toggle: centred 4 mm past the switch body's +X face (Phi 4 x 8
    # mm cylinder, half outside the body so it protrudes through the
    # toggle cutout in the holster's +X end wall).
    toggle_x = switch_body_x + HP.SWITCH_BODY_L / 2.0 + 8.0 / 2.0
    instances.append(Instance(
        "antispark_switch_toggle", "antispark_switch_toggle_DO_NOT_PRINT.stl",
        None, None,
        _trans(toggle_x, HP.SWITCH_HOLSTER_CENTRE_Y, switch_body_z),
    ))

    # Two shorty LiPo pack BODIES velcro'd UNDER chassis_bottom's flat
    # belly (Aug 2026; the single bay pack is retired).  Placement comes
    # from HP.battery_pack_transforms() (block yawed 30 deg, packs offset
    # +/- half the width+gap).
    for k, M_pack in enumerate(HP.battery_pack_transforms()):
        instances.append(Instance(
            f"lipo_battery_{k}", "lipo_battery_body_DO_NOT_PRINT.stl",
            None, None, M_pack,
        ))
    # XT60s + Y-harness node at the pack block's +axis end (az 30,
    # ~2 mm past the 75 mm pack ends), below the belly.
    yaw = np.deg2rad(HP.BATTERY_UNDER_YAW_DEG)
    xt60_r = HP.BATTERY_W / 2.0 + 14.0 / 2.0
    lipo_cz = -(plate_t / 2.0 + 4.0) - HP.BATTERY_H / 2.0
    instances.append(Instance(
        "lipo_xt60", "lipo_xt60_DO_NOT_PRINT.stl", None, None,
        _trans(xt60_r * np.cos(yaw), xt60_r * np.sin(yaw), lipo_cz),
    ))

    yaw_output_z = HP.CHASSIS_YAW_OUTPUT_Z   # = +29.75 mm; disc-horn top
    p_femur = np.deg2rad(HP.STANCE_FEMUR_DEG)
    pt = np.deg2rad(HP.STANCE_FEMUR_DEG + HP.STANCE_TIBIA_DEG)
    hip_drop = HP.COXA_HIP_DROP

    R_hip = _rot_x(-np.pi / 2.0)

    for i in range(6):
        a = (i + 0.5) * np.pi / 3.0
        edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
        R_a = _rot_z(a)

        yaw_output_world = edge_mid + np.array([0.0, 0.0, yaw_output_z])

        # Frame helpers: T_edge places a mesh in chassis-edge coords
        # (bracket origin); T_yaw_out places a mesh in coxa-link / leg
        # local coords (yaw output at the origin).
        T_edge = _trans(*edge_mid) @ R_a
        T_yaw_out = _trans(*yaw_output_world) @ R_a

        # ----- yaw servo body (hangs below chassis_bottom in its
        #       integrated cradle; May 2026 chassis_bottom-integrated
        #       yaw cradle redesign retired the standalone coxa_bracket).
        #
        # Anchor: the cradle's tab shelf is +8 mm UP from the legacy
        # bracket shelf (was at chassis-z = 0).  Build it from the same
        # constants ``build_prototype_assembly._build_leg`` uses so the
        # two views can never drift apart again.
        cradle_shelf_z = HP.CHASSIS_PLATE_T / 2.0 + HP.CRADLE_TAB_SHELF_Z
        T_yaw_body = T_edge @ _trans(
            -HP.SERVO_OUTPUT_X, 0,
            cradle_shelf_z - HP.WELL_RIM_Z,
        )
        instances.append(Instance(
            "servo_body", "servo_body_DO_NOT_PRINT.stl", i, "yaw", T_yaw_body,
        ))

        # ----- yaw disc horn (above the cradle shelf).  June 2026
        # disc-horn switch: the link's pad now bolts DIRECTLY onto this
        # 20 mm aluminum 25T disc horn -- no printed servo_horn_adapter
        # disc in the stack any more.  The coxa_link's pedestal bottom
        # mating face therefore lands at z = yaw_horn_z + PLASTIC_HORN_H
        # (= the disc horn's top face) which must equal
        # ``HP.CHASSIS_YAW_OUTPUT_Z`` -- i.e. the same +29.75 mm the
        # coxa_link is anchored to via ``yaw_output_world``.  If you
        # ever see the link floating above the horn, this is where the
        # bug will be.
        yaw_horn_z = (
            cradle_shelf_z
            + (HP.SERVO_BODY_H - HP.WELL_RIM_Z)
            + HP.SERVO_OUTPUT_H
        )
        T_yaw_horn = T_edge @ _trans(0, 0, yaw_horn_z)
        instances.append(Instance(
            "servo_horn", "servo_horn_DO_NOT_PRINT.stl", i, "yaw", T_yaw_horn,
        ))

        # ----- coxa link (rotates with yaw output -- in standing pose, yaw=0)
        instances.append(Instance(
            "coxa_link", "coxa_link.stl", i, None, T_yaw_out,
        ))

        # ----- hip-pitch servo body (in the coxa-link cradle)
        delta_hip = np.array([
            HP.COXA_LENGTH - HP.SERVO_OUTPUT_X,
            -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H) + HP.COXA_HIP_ANCHOR_Y,
            hip_drop,
        ])
        T_hip_body = T_yaw_out @ _trans(*delta_hip) @ R_hip
        instances.append(Instance(
            "servo_body", "servo_body_DO_NOT_PRINT.stl", i, "hip", T_hip_body,
        ))

        # Hip-pitch sandwich-joint clamp cap: closes the OPEN +Y face of
        # the coxa_link hip cradle around the STS3215 body.  The cap is
        # modelled in the servo well-local frame (same origin/axes as
        # servo_body), so it reuses the IDENTICAL transform as the hip
        # servo body (Jun 2026 sandwich-joint clamp cap).
        instances.append(Instance(
            "servo_clamp_cap", "servo_clamp_cap.stl", i, "hip", T_hip_body,
        ))

        # ----- hip disc horn (on the hip-pitch output axis).
        # June 2026 disc-horn switch: femur's hip pad bolts DIRECTLY
        # onto this aluminum disc horn; the printed servo_horn_adapter
        # is gone.
        T_hip_horn = T_yaw_out @ _trans(HP.COXA_LENGTH, HP.COXA_HIP_ANCHOR_Y, hip_drop) @ R_hip
        instances.append(Instance(
            "servo_horn", "servo_horn_DO_NOT_PRINT.stl", i, "hip", T_hip_horn,
        ))

        # ----- femur link (rotated by hip-pitch stance angle)
        # NEW (May 2026 collinear-pad refactor): the femur's NEW local
        # origin is its hip pad mating face = HORN_STACK_H above the
        # joint axis along leg +Y, so the femur translation shifts
        # +HORN_STACK_H in coxa-Y compared with the pre-refactor
        # placement on the joint axis.
        hip_joint_local = np.array([HP.COXA_LENGTH, HP.COXA_HIP_ANCHOR_Y, hip_drop])
        pad_axis_offset = np.array([0.0, HP.HORN_STACK_H, 0.0])
        T_femur = (T_yaw_out
                    @ _trans(*(hip_joint_local + pad_axis_offset))
                    @ _rot_y(p_femur))
        instances.append(Instance(
            "femur_link", "femur_link_DO_NOT_PRINT.stl", i, None, T_femur,
        ))

        # ----- knee-pitch servo body (in the femur cradle)
        # NEW (May 2026 collinear-pad refactor): in NEW femur-local
        # the well sits HORN_STACK_H deeper in -Y so the servo body's
        # world position is unchanged (the femur's transform shifted
        # +HORN_STACK_H above, the well shifts -HORN_STACK_H here,
        # net zero in world).
        delta_knee = np.array([
            HP.FEMUR_LENGTH - HP.SERVO_OUTPUT_X,
            -(HP.SERVO_BODY_H + HP.SERVO_OUTPUT_H) - HP.HORN_STACK_H,
            0.0,
        ])
        T_knee_body = T_femur @ _trans(*delta_knee) @ R_hip
        instances.append(Instance(
            "servo_body", "servo_body_DO_NOT_PRINT.stl", i, "knee", T_knee_body,
        ))

        # Knee sandwich-joint clamp cap: closes the OPEN +Y face of the
        # femur_link knee cradle around the STS3215 body.  Same
        # well-local frame as the knee servo body.
        instances.append(Instance(
            "servo_clamp_cap", "servo_clamp_cap.stl", i, "knee", T_knee_body,
        ))

        # ----- knee disc horn.  June 2026 disc-horn switch: tibia's
        # knee pad bolts DIRECTLY onto this aluminum disc horn; the
        # printed servo_horn_adapter is gone.  May 2026 collinear-pad
        # refactor: the knee disc horn lives on the knee axis at NEW femur-y =
        # -HORN_STACK_H (the joint axis is HORN_STACK_H below the
        # NEW link origin); world position unchanged.
        T_knee_horn = T_femur @ _trans(HP.FEMUR_LENGTH,
                                        -HP.HORN_STACK_H, 0) @ R_hip
        instances.append(Instance(
            "servo_horn", "servo_horn_DO_NOT_PRINT.stl", i, "knee", T_knee_horn,
        ))

        # ----- tibia link
        Ry_p_3 = _rot_y(p_femur)[:3, :3]
        knee_joint_local = (
            hip_joint_local + Ry_p_3 @ np.array([HP.FEMUR_LENGTH, 0, 0])
        )
        T_tibia = (T_yaw_out
                    @ _trans(*(knee_joint_local + pad_axis_offset))
                    @ _rot_y(pt))
        instances.append(Instance(
            "tibia_link", "tibia_link_DO_NOT_PRINT.stl", i, None, T_tibia,
        ))

        # ----- foot pad (passive hinge -- only the leg azimuth is applied)
        # NEW (May 2026 collinear-pad refactor): hinge axis in NEW
        # tibia-local is at (TIBIA_LENGTH, +LINK_THICKNESS/2,
        # FOOT_HINGE_TIBIA_Z); tang moved with the spar so the hinge
        # axis is now at the spar centreline (was 0 pre-refactor).
        # Aug 2026: the TPU foot_boot is unioned into tibia_link.stl
        # (make_tibia_link), so no separate foot instance is placed.

    instances.extend(_build_fastener_instances())
    return instances


def _axis_to_transform(axis: np.ndarray, origin: np.ndarray) -> np.ndarray:
    """Build a 4x4 that maps mesh-local +Z to ``axis`` and puts the
    mesh origin at ``origin``.  Used to place parametric fastener
    meshes (whose +Z is along the screw shaft) into the world.
    """
    z_new = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(z_new))
    if n < 1e-12:
        z_new = np.array([0.0, 0.0, 1.0])
    else:
        z_new = z_new / n
    # Pick the world axis least parallel to z_new as a seed for the new X.
    seed = (
        np.array([1.0, 0.0, 0.0])
        if abs(z_new[0]) < 0.9
        else np.array([0.0, 1.0, 0.0])
    )
    x_new = seed - z_new * float(np.dot(seed, z_new))
    x_new /= float(np.linalg.norm(x_new))
    y_new = np.cross(z_new, x_new)
    T = np.eye(4)
    T[:3, 0] = x_new
    T[:3, 1] = y_new
    T[:3, 2] = z_new
    T[:3, 3] = np.asarray(origin, dtype=float)
    return T


def _build_fastener_instances() -> list[Instance]:
    """Convert every entry in ``fastener_registry.build_all_fastener_instances()``
    into an inspector ``Instance``.

    The parametric fastener meshes in ``fasteners/`` use the convention:
        origin = head mating face, +Z = body axis (into material).
    So the world transform that places mesh-local +Z along
    ``fi.axis_world`` and the mesh origin at ``fi.head_world_xyz``
    correctly drops the head onto the printed part.
    """
    out: list[Instance] = []
    for fi in fastener_registry.build_all_fastener_instances():
        if fi.is_virtual:
            continue
        T = _axis_to_transform(fi.axis_world, fi.head_world_xyz)
        out.append(Instance(
            part_type=fi.spec,
            stl_name=fi.cache_stl,
            leg_index=fi.leg_index,
            joint=fi.joint,
            transform=T,
            stl_dir=FASTENERS_DIR,
            fastener_role=fi.role,
        ))
    return out


# ---------------------------------------------------------------------------
# STL loading
# ---------------------------------------------------------------------------


def _missing_stl_message(missing: list[str]) -> str:
    listing = "\n  ".join(missing)
    return (
        f"inspect_build: missing {len(missing)} STL(s) under {STL_DIR}:\n"
        f"  {listing}\n"
        "Run `make build` (or `./run.sh hexapod_walker/prototype/build_all.py`) "
        "first to regenerate the per-part STLs."
    )


def _load_stl_cache(stl_keys: set[tuple[str, str]]) -> dict[tuple[str, str], pv.PolyData]:
    """Load each unique STL once.  Returns a (stl_dir, stl_name) -> PolyData
    cache, so fasteners (loaded out of ``fasteners/``) and printed parts
    (loaded out of ``stl_prototype/``) share the same lookup path.
    """
    cache: dict[tuple[str, str], pv.PolyData] = {}
    missing: list[str] = []
    for stl_dir, name in sorted(stl_keys):
        path = os.path.join(stl_dir, name)
        if not os.path.isfile(path):
            missing.append(os.path.join(os.path.basename(stl_dir.rstrip("/")), name))
            continue
        tm = trimesh.load_mesh(path, process=False)
        if isinstance(tm, trimesh.Scene):
            tm = trimesh.util.concatenate(list(tm.geometry.values()))
        cache[(stl_dir, name)] = _trimesh_to_pv(tm)
    if missing:
        print(_missing_stl_message(missing), file=sys.stderr)
        sys.exit(2)
    return cache


def _instance_stl_key(inst: Instance) -> tuple[str, str]:
    """Resolve ``(directory, filename)`` for an instance's mesh on disk."""
    if inst.stl_dir is not None:
        return (inst.stl_dir, inst.stl_name)
    base = inst.stl_name.replace(HP.NOPRINT_SUFFIX, "")
    if base.endswith(".stl"):
        base = base[:-4]
    if base in HP.NOT_PRINTED_MESHES or HP.NOPRINT_SUFFIX in inst.stl_name:
        return (HP.REF_STL_DIR, inst.stl_name)
    return (STL_DIR, inst.stl_name)


def _trimesh_to_pv(tm: trimesh.Trimesh) -> pv.PolyData:
    """Cheap conversion: PolyData with triangle connectivity."""
    faces = np.hstack([
        np.full((len(tm.faces), 1), 3, dtype=np.int64),
        tm.faces.astype(np.int64),
    ]).ravel()
    return pv.PolyData(tm.vertices.astype(np.float64), faces)


def _apply_transform(mesh: pv.PolyData, T: np.ndarray) -> pv.PolyData:
    """Return a transformed copy (does not modify the cached source)."""
    return mesh.copy().transform(T, inplace=False)


# ---------------------------------------------------------------------------
# Scene assembly
# ---------------------------------------------------------------------------


@dataclass
class PlacedInstance:
    """An Instance after STL loading + chassis_lift + add_mesh."""
    instance: Instance
    actor: pv.Actor
    centroid: np.ndarray   # world-space centroid of the placed mesh
    label_pos: np.ndarray  # where to anchor the floating label
    # Companion actor that draws ONLY the part's feature edges (dihedral
    # angle >= 30 deg) on top of the smooth surface, so the part reads
    # crisp without the "fan-triangulation cracks" that ``show_edges=True``
    # would otherwise paint across every flat face cut by a boolean
    # hole.  None for fasteners (we already turn show_edges off on them).
    edges_actor: pv.Actor | None = None


def _compute_chassis_lift(
    instances: list[Instance],
    stl_cache: dict[str, pv.PolyData],
) -> float:
    """Mirror build_prototype_assembly._build_leg's chassis_lift logic.

    The lowest z point across leg 0's parts (foot pad, tibia, knee
    horn, etc.) gives ``z_min``; the chassis lift is ``-z_min`` so
    the foot pads land on z = 0.
    """
    z_min = np.inf
    for inst in instances:
        if inst.leg_index != 0:
            continue
        # Skip fasteners when computing chassis lift -- their cache
        # mesh has the head at (0,0,0) and the shank at (0,0,+L), so
        # they never extend the floor; counting them would lift the
        # foot from the ground.
        if palette.is_fastener(inst.part_type):
            continue
        mesh = _apply_transform(stl_cache[_instance_stl_key(inst)], inst.transform)
        zlow = float(mesh.bounds[4])  # bounds = (xmin,xmax,ymin,ymax,zmin,zmax)
        if zlow < z_min:
            z_min = zlow
    if not np.isfinite(z_min):
        return 0.0
    return -z_min


# ---------------------------------------------------------------------------
# Plotter app
# ---------------------------------------------------------------------------


SHORTCUTS_TEXT = (
    "Hover -> label.  Left-click -> isolate part.  "
    "Double-click -> focus sub-assembly.\n"
    "Trackpad / mouse:\n"
    "  drag                 orbit (rotate the view)\n"
    "  shift + drag         pan\n"
    "  cmd  + drag          roll (rotate around view axis)\n"
    "  cmd  + shift + drag  zoom\n"
    "  two-finger swipe     zoom in / out\n"
    "Keyboard:\n"
    "  arrows        pan camera (l/r/u/d in screen plane)\n"
    "  , / .         roll left / right\n"
    "  + / -         zoom in / out\n"
    "  L     toggle ALL labels on at once\n"
    "  E     toggle exploded view (0 / 1.5)\n"
    "  F     focus the hovered part's sub-assembly (toggle)\n"
    "  I/Esc clear focus / isolation\n"
    "  R     reset camera\n"
    "  S     save screenshot\n"
    "  Q     quit"
)

# Opacity applied to every non-isolated actor while one part is
# isolated.  The isolated part stays at full opacity in its original
# assembled-pose position -- we do NOT translate it on click, because
# the whole point of isolation is seeing how the part sits relative to
# its neighbours.  Pulling it outward defeats that.
ISOLATE_DIM_OPACITY = 0.15


def _print_shortcuts() -> None:
    print(SHORTCUTS_TEXT)


def _add_instances_to_plotter(
    plotter_factory_args: dict,
    instances: list[Instance],
    stl_cache: dict[str, pv.PolyData],
    chassis_lift: float,
) -> tuple[pv.Plotter, list[PlacedInstance], np.ndarray]:
    """Create the plotter, add every instance as a colored actor.

    Returns (plotter, placed_instances, chassis_centroid).
    """
    plotter = pv.Plotter(**plotter_factory_args)
    plotter.set_background("white", top="lightgray")

    placed: list[PlacedInstance] = []
    chassis_centroids: list[np.ndarray] = []

    lift = _trans(0, 0, chassis_lift)

    fastener_counter = 0
    for inst in instances:
        world_T = lift @ inst.transform
        mesh = _apply_transform(stl_cache[_instance_stl_key(inst)], world_T)
        r, g, b = palette.PART_COLORS.get(
            inst.part_type, (0.7, 0.7, 0.7),
        )
        is_servo = inst.part_type in ("servo_body", "servo_horn")
        is_fastener = palette.is_fastener(inst.part_type)
        # Fasteners are small + numerous; turn off edge rendering so
        # the inspector stays responsive at full count (~300 actors
        # added by fastener_registry).
        if is_fastener:
            fastener_counter += 1
            actor_name = f"fastener_{inst.part_type}_{fastener_counter}"
        else:
            actor_name = f"{inst.part_type}_L{inst.leg_index}_{inst.joint}"
        actor = plotter.add_mesh(
            mesh,
            color=(r, g, b),
            show_edges=False,
            line_width=0.5 if is_servo else 0.7,
            opacity=0.85 if is_servo else 1.0,
            ambient=0.22 if is_fastener else 0.18,
            diffuse=0.85,
            specular=0.35 if is_fastener else 0.15,
            specular_power=24.0 if is_fastener else 18.0,
            smooth_shading=True,
            name=actor_name,
        )

        # Feature-edge overlay on printed + servo parts.  Trimesh
        # booleans triangulate flat faces around drilled holes as a
        # fan of long thin triangles whose interior edges aren't
        # actual geometry features -- ``show_edges=True`` would paint
        # those triangulation edges as black "cracks" radiating
        # across the surface (see Lukas' bug report 2026-05).
        # extract_feature_edges with the default 30 deg dihedral
        # threshold keeps only the edges that bound a real crease
        # (hole rims, part silhouettes, pocket walls) and drops the
        # coplanar triangulation noise.  Fasteners are tiny + many
        # so skip the overlay for them; they already render without
        # edges via show_edges=False above.
        #
        # IMPORTANT: the STLs are loaded via trimesh with
        # ``process=False`` (see _load_stl_cache), so duplicate
        # vertices are NOT merged -- every triangle ships its own
        # three independent points.  vtkFeatureEdges decides "is this
        # edge shared with a neighbour?" by point-id equality, so on
        # an unmerged mesh it sees NO shared edges and emits every
        # triangle edge as a "boundary edge".  We feed the edge
        # filter a cleaned (point-merged) copy so the dihedral test
        # has actual neighbour relationships to evaluate.  The
        # surface actor still consumes the unmerged mesh -- nothing
        # else cares about adjacency and avoiding the clean keeps the
        # surface render bit-exact w.r.t. the on-disk STL.
        edges_actor: pv.Actor | None = None
        if not is_fastener:
            try:
                # Absolute 1 um merge tolerance: STL coords are in mm,
                # so 1e-6 mm welds duplicates from the STL "every
                # triangle has its own 3 verts" representation
                # without bridging genuinely-distinct hole-rim
                # vertices that sit ~10s of microns apart at worst.
                cleaned = mesh.clean(
                    point_merging=True,
                    tolerance=1e-6,
                    absolute=True,
                )
                feat = cleaned.extract_feature_edges(
                    feature_angle=30.0,
                    boundary_edges=True,
                    non_manifold_edges=False,
                    feature_edges=True,
                    manifold_edges=False,
                )
            except Exception:
                feat = None
            if feat is not None and feat.n_points > 0:
                edges_actor = plotter.add_mesh(
                    feat,
                    color=(0.08, 0.08, 0.08),
                    line_width=0.5 if is_servo else 0.7,
                    opacity=0.85 if is_servo else 1.0,
                    lighting=False,
                    pickable=False,
                    render_lines_as_tubes=False,
                    name=f"{actor_name}__edges",
                )

        cen = np.asarray(mesh.center, dtype=np.float64)
        # For long, narrow parts like the femur and tibia, anchoring
        # the label at the centroid drops it into the middle of the
        # mesh.  Push the label up by a few mm so it sits above the
        # geometry instead of inside it.
        zoff = float(mesh.bounds[5] - cen[2])
        label_anchor = cen + np.array([0.0, 0.0, 0.6 * max(zoff, 8.0)])
        placed.append(PlacedInstance(
            instance=inst,
            actor=actor,
            centroid=cen,
            label_pos=label_anchor,
            edges_actor=edges_actor,
        ))
        if inst.part_type in ("chassis_top", "chassis_bottom",
                              "chassis_plate_a", "chassis_plate_b"):
            chassis_centroids.append(cen)

    if chassis_centroids:
        chassis_centroid = np.mean(np.vstack(chassis_centroids), axis=0)
    else:
        # Fallback: average all instance centroids
        chassis_centroid = np.mean(
            np.vstack([p.centroid for p in placed]), axis=0,
        )

    return plotter, placed, chassis_centroid


def _resolve_focus_arg(
    focus_spec: str | None,
    placed: list[PlacedInstance],
) -> PlacedInstance | None:
    """Parse a ``--focus PART/Lx`` CLI string into a ``PlacedInstance``.

    Format:
        * ``"coxa_link/L0"``  -> printable part on leg 0
        * ``"chassis_top"``   -> chassis-level part (no leg index)

    Raises ``SystemExit`` with a helpful diagnostic if the spec does
    not name a real placed instance.
    """
    if focus_spec is None:
        return None
    if "/" in focus_spec:
        pt, leg = focus_spec.split("/", 1)
        if not (leg.startswith("L") and leg[1:].isdigit()):
            print(
                f"inspect_build: --focus leg part must be 'L<n>', "
                f"got {leg!r}",
                file=sys.stderr,
            )
            sys.exit(2)
        leg_index: int | None = int(leg[1:])
    else:
        pt = focus_spec
        leg_index = None
    candidates = [
        p for p in placed
        if p.instance.part_type == pt and p.instance.leg_index == leg_index
        and not palette.is_fastener(p.instance.part_type)
        and p.instance.part_type not in ("servo_body", "servo_horn")
    ]
    if not candidates:
        available = sorted({
            f"{p.instance.part_type}"
            + (f"/L{p.instance.leg_index}" if p.instance.leg_index is not None else "")
            for p in placed
            if not palette.is_fastener(p.instance.part_type)
            and p.instance.part_type not in ("servo_body", "servo_horn")
        })
        print(
            f"inspect_build: --focus {focus_spec!r} did not match any printable "
            f"part.  Try one of:\n  " + "\n  ".join(available),
            file=sys.stderr,
        )
        sys.exit(2)
    # Printable parts are unique on (part_type, leg_index); the filter
    # above excludes the servo bodies/horns where multiple joints share
    # the same leg.  Pick the first deterministically.
    return candidates[0]


def run(
    *,
    screenshot: str | None,
    explode: float,
    focus: str | None = None,
    window_size: tuple[int, int] = (1600, 1000),
) -> None:
    instances = _build_assembly_instances()
    all_fasteners = fastener_registry.build_all_fastener_instances()
    stl_keys = {_instance_stl_key(inst) for inst in instances}
    stl_cache = _load_stl_cache(stl_keys)
    chassis_lift = _compute_chassis_lift(instances, stl_cache)

    headless = screenshot is not None
    # We initially build a placeholder plotter for actor creation, then
    # discard it and rebuild a "full" plotter with all the widgets and
    # callbacks attached.  Simpler approach: just construct one plotter
    # up-front with the right off_screen mode.
    plotter_kwargs = {"off_screen": headless, "window_size": window_size}
    plotter, placed, chassis_centroid = _add_instances_to_plotter(
        plotter_kwargs, instances, stl_cache, chassis_lift,
    )

    initial_focus = _resolve_focus_arg(focus, placed)

    # Re-decorate this plotter with the legend, labels, slider, and
    # widgets.  We pass a constructor function so _build_plotter can
    # use a single plotter (we already have one populated with actors).
    # Re-implement the "decoration" inline here to avoid double-creation:
    _decorate_plotter(
        plotter, placed, chassis_centroid, explode,
        instances=instances,
        all_fasteners=all_fasteners,
        initial_focus=initial_focus,
    )

    plotter.camera_position = "iso"
    plotter.reset_camera()

    # Activate --focus AFTER the default camera reset so the focused
    # sub-assembly's bounds-based reset_camera is the final word on
    # framing.  _decorate_plotter stashed the helper + the resolved
    # PlacedInstance on the plotter for this purpose.
    init_focus = getattr(plotter, "_inspect_build_initial_focus", None)
    if init_focus is not None:
        plotter._inspect_build_set_focus(init_focus)  # type: ignore[attr-defined]

    if headless:
        _print_shortcuts()
        os.makedirs(os.path.dirname(os.path.abspath(screenshot)), exist_ok=True)
        plotter.screenshot(screenshot)
        size = os.path.getsize(screenshot)
        print(
            f"inspect_build: headless screenshot -> {screenshot} "
            f"({size / 1024:.1f} KB)"
        )
        plotter.close()
        return

    _print_shortcuts()
    plotter.show(title="hexapod build inspector")


def _decorate_plotter(
    plotter: pv.Plotter,
    placed: list[PlacedInstance],
    chassis_centroid: np.ndarray,
    initial_explode: float,
    *,
    instances: list[Instance] | None = None,
    all_fasteners: list[fastener_registry.FastenerInstance] | None = None,
    initial_focus: PlacedInstance | None = None,
) -> None:
    """Attach legend, labels, slider, checkboxes, key events to plotter.

    The optional ``instances`` / ``all_fasteners`` / ``initial_focus``
    arguments power the focus-on-sub-assembly mode (``F`` keybinding
    and double-click).  When omitted the focus mode is still wired up
    on the keyboard / mouse but resolves to a no-op because the
    sub-assembly query needs both lists.
    """
    if instances is None:
        instances = [p.instance for p in placed]
    if all_fasteners is None:
        all_fasteners = []
    present_types: list[str] = []
    for p in placed:
        if p.instance.part_type not in present_types:
            present_types.append(p.instance.part_type)

    legend_entries = [
        (pt, palette.PART_COLORS[pt]) for pt in present_types
        if pt in palette.PART_COLORS and not palette.is_fastener(pt)
    ]
    if any(palette.is_fastener(pt) for pt in present_types):
        legend_entries.append(
            ("fasteners (M3/M2.5)", palette.PART_COLORS["M3x8 SHCS"]),
        )
    if legend_entries:
        plotter.add_legend(
            labels=legend_entries,
            bcolor="white",
            face="rectangle",
            size=(0.16, max(0.02 * len(legend_entries), 0.05)),
            loc="lower right",
            border=True,
        )

    plotter.add_text(
        SHORTCUTS_TEXT,
        position="upper_left",
        font_size=9,
        color="black",
        shadow=False,
    )

    label_positions = np.array(
        [p.label_pos for p in placed], dtype=np.float64,
    )
    label_texts = [
        palette.instance_label(
            p.instance.part_type,
            p.instance.leg_index,
            p.instance.joint,
            fastener_role=p.instance.fastener_role,
        )
        for p in placed
    ]

    # Build the label anchors as a real pv.PolyData with a named
    # ``labels`` string array on point_data, then hand it to
    # add_point_labels via the ``labels="labels"`` string path.  That
    # path makes add_point_labels feed the polydata DIRECTLY into the
    # vtkPointSetToLabelHierarchy (instead of cloning it via the
    # list-of-strings code path), so we can mutate
    # ``label_polydata.points`` in place on explode and just call
    # Modified() -- no more remove_actor / add_point_labels rebuild
    # cycle.  The remove+re-add cycle was segfaulting VTK on the
    # first explode toggle: it tore down the label-hierarchy actor
    # while VTK still held internal references through the pipeline,
    # taking the whole window down with it.
    import vtk as _vtk_for_labels  # local: keep top-level import set lean
    label_polydata = pv.PolyData(label_positions)
    _labels_vtk_arr = _vtk_for_labels.vtkStringArray()
    _labels_vtk_arr.SetName("labels")
    for _s in label_texts:
        _labels_vtk_arr.InsertNextValue(_s)
    label_polydata.GetPointData().AddArray(_labels_vtk_arr)

    label_actor_holder = {"actor": plotter.add_point_labels(
        label_polydata,
        "labels",
        point_size=0,
        show_points=False,
        font_size=14,
        bold=True,
        always_visible=True,
        shape_color="white",
        shape_opacity=0.7,
        text_color="black",
        name="instance_labels",
    )}
    # Persistent all-labels-at-once is OFF by default now -- hover gives
    # you the same info one part at a time without the clutter.  Press
    # `L` to bring the full cloud back when you want overview context.
    labels_state = {"visible": False}
    label_actor_holder["actor"].SetVisibility(False)

    def _toggle_labels() -> None:
        labels_state["visible"] = not labels_state["visible"]
        label_actor_holder["actor"].SetVisibility(labels_state["visible"])
        plotter.render()

    state = {"explode": float(initial_explode)}

    # --- isolation state ----------------------------------------------
    # When the user left-clicks on a part, every OTHER actor's opacity
    # drops to ISOLATE_DIM_OPACITY so the picked part stands out.  We
    # deliberately do not move the picked part -- it stays where it
    # belongs in the assembly so the user can see how it fits.
    iso_state: dict[str, object | None] = {"placed": None}
    # Save opacities for both the main mesh and (if any) its feature-edge
    # overlay, so the isolation/focus paths can restore the exact paired
    # look on exit.
    saved_opacity: dict[int, float] = {}
    for p in placed:
        saved_opacity[id(p.actor)] = float(p.actor.prop.opacity)
        if p.edges_actor is not None:
            saved_opacity[id(p.edges_actor)] = float(p.edges_actor.prop.opacity)

    # --- focus state --------------------------------------------------
    # Focus-on-sub-assembly is orthogonal to isolation: when active,
    # every actor that does NOT belong to the focused sub-assembly is
    # hidden (visibility=False).  Members keep their saved opacity and
    # full visibility so the focused group reads as a clean exploded
    # detail view of the picked part.
    #
    # ``placed``      -- the PlacedInstance currently focused (or None)
    # ``members``     -- ids of the actors that should remain visible
    # ``saved_vis``   -- the actor visibility map at the moment focus
    #                    was entered, so we can restore the per-part-
    #                    type checkbox effect on exit
    focus_state: dict[str, object] = {
        "placed": None,
        "members": set(),
        "saved_vis": {},
    }

    def _apply_explode(factor: float) -> None:
        for p in placed:
            offset = p.centroid - chassis_centroid
            shift = factor * offset
            m = np.eye(4)
            m[0, 3] = shift[0]
            m[1, 3] = shift[1]
            m[2, 3] = shift[2]
            p.actor.user_matrix = m
            if p.edges_actor is not None:
                p.edges_actor.user_matrix = m
        # Update label anchor positions in place on the cached
        # polydata.  ``add_point_labels`` was given this polydata as
        # its hierarchy input at setup time, so calling Modified() is
        # enough to make VTK re-read the new positions on the next
        # render -- no actor rebuild, no risk of tearing down a live
        # vtkPointSetToLabelHierarchy mid-frame.
        new_pos = np.asarray(
            [
                p.label_pos + factor * (p.centroid - chassis_centroid)
                for p in placed
            ],
            dtype=np.float64,
        )
        label_polydata.points = new_pos
        label_polydata.Modified()

    def _slider_callback(value: float) -> None:
        state["explode"] = float(value)
        _apply_explode(state["explode"])

    plotter.add_slider_widget(
        _slider_callback,
        rng=(0.0, 2.0),
        value=state["explode"],
        title="explode",
        pointa=(0.62, 0.06),
        pointb=(0.96, 0.06),
        title_height=0.020,
        slider_width=0.02,
        tube_width=0.005,
        style="modern",
        fmt="%.2f",
    )

    if state["explode"] != 0.0:
        _apply_explode(state["explode"])

    # checkbox column.  Bundle the feature-edge overlay actor (when
    # present) into the same list as its main mesh actor so a single
    # checkbox toggle hides/shows both, and the focus / isolation /
    # explode paths below don't have to special-case the overlay.
    actors_by_type: dict[str, list[pv.Actor]] = {}
    fastener_actors: list[pv.Actor] = []
    for p in placed:
        bucket = actors_by_type.setdefault(p.instance.part_type, [])
        bucket.append(p.actor)
        if p.edges_actor is not None:
            bucket.append(p.edges_actor)
        if palette.is_fastener(p.instance.part_type):
            fastener_actors.append(p.actor)
            if p.edges_actor is not None:
                fastener_actors.append(p.edges_actor)

    # The per-part-type checkboxes drive a "desired visibility" map
    # rather than mutating actor visibility directly.  When NOT in
    # focus mode, the desired-visibility is mirrored straight onto
    # actor.SetVisibility() on every click.  When IN focus mode, the
    # checkboxes only update the desired-visibility map; the focused
    # sub-assembly's hidden actors stay hidden until focus is cleared,
    # at which point we apply the (possibly-updated) desired-vis map
    # back onto the actors.  The fasteners master toggle has a
    # special-case while focused: it hides ONLY the sub-assembly's
    # fastener members, leaving the rest unchanged.
    checkbox_desired: dict[str, bool] = {
        pt: True for pt in actors_by_type.keys()
    }
    checkbox_desired["__fasteners__"] = True

    def _make_toggle(part_type: str):
        def _cb(value: bool) -> None:
            checkbox_desired[part_type] = bool(value)
            if focus_state["placed"] is None:
                for actor in actors_by_type[part_type]:
                    actor.SetVisibility(bool(value))
                plotter.render()
        return _cb

    cb_size = 18
    cb_x = 12
    cb_y0 = 12

    # Filter the per-part-type column to NON-fastener types -- the
    # fasteners share a single master toggle (below) so we don't end
    # up with six near-identical SHCS rows competing for column space.
    non_fastener_types = [pt for pt in present_types if not palette.is_fastener(pt)]
    has_fasteners = bool(fastener_actors)

    column_rows = list(non_fastener_types)
    if has_fasteners:
        # Master "fasteners" toggle sits at the TOP of the column (idx
        # 0) so the user always finds it in the same spot.  Render it
        # black to make it visually distinct from the per-printed-part
        # color swatches below.
        column_rows = ["__fasteners__"] + column_rows

    for idx, row_key in enumerate(column_rows):
        pos_y = cb_y0 + idx * (cb_size + 6)
        if row_key == "__fasteners__":
            def _toggle_fasteners(value: bool) -> None:
                checkbox_desired["__fasteners__"] = bool(value)
                if focus_state["placed"] is None:
                    for actor in fastener_actors:
                        actor.SetVisibility(bool(value))
                else:
                    # Special-case while focused: the master toggle
                    # only flips the sub-assembly's fastener members.
                    members = focus_state["members"]
                    for actor in fastener_actors:
                        if id(actor) in members:
                            actor.SetVisibility(bool(value))
                plotter.render()
            plotter.add_checkbox_button_widget(
                _toggle_fasteners,
                value=True,
                position=(cb_x, pos_y),
                size=cb_size,
                border_size=1,
                color_on=(0.10, 0.10, 0.10),       # black, per spec
                color_off=(0.85, 0.85, 0.85),
                background_color="white",
            )
            plotter.add_text(
                "fasteners",
                position=(cb_x + cb_size + 4, pos_y + 2),
                font_size=8,
                color="black",
            )
            continue
        part_type = row_key
        r, g, b = palette.PART_COLORS.get(part_type, (0.5, 0.5, 0.5))
        plotter.add_checkbox_button_widget(
            _make_toggle(part_type),
            value=True,
            position=(cb_x, pos_y),
            size=cb_size,
            border_size=1,
            color_on=(r, g, b),
            color_off=(0.85, 0.85, 0.85),
            background_color="white",
        )
        plotter.add_text(
            part_type,
            position=(cb_x + cb_size + 4, pos_y + 2),
            font_size=8,
            color="black",
        )

    explode_toggle_state = {"on": initial_explode > 0.0}

    def _toggle_explode() -> None:
        explode_toggle_state["on"] = not explode_toggle_state["on"]
        state["explode"] = 1.5 if explode_toggle_state["on"] else 0.0
        # PyVista 0.45+ moved slider_widgets onto Plotter.widgets; access
        # the new path first and fall back to the deprecated top-level
        # attribute on older PyVista versions.  Touching the old path
        # under PyVista >= 0.48 emits a PyVistaDeprecationWarning that, if
        # the user has warnings-as-error configured, escapes the explode
        # toggle entirely.
        try:
            widget_group = getattr(plotter, "widgets", None)
            sliders = (
                getattr(widget_group, "slider_widgets", None)
                if widget_group is not None
                else None
            )
            if sliders is None:
                sliders = plotter.slider_widgets  # legacy fallback
            for sw in sliders:
                sw.GetSliderRepresentation().SetValue(state["explode"])
        except Exception:
            pass
        try:
            _apply_explode(state["explode"])
        except Exception as exc:
            # Don't let a stale label actor or an add_point_labels API
            # change take down the whole viewer -- log it and continue.
            print(f"inspect_build: explode apply failed: {exc!r}")
        plotter.render()

    def _reset_camera() -> None:
        plotter.reset_camera()
        plotter.render()

    def _save_screenshot() -> None:
        os.makedirs(ARTIFACTS_DIR, exist_ok=True)
        out = os.path.join(ARTIFACTS_DIR, "build_inspect.png")
        plotter.screenshot(out)
        print(f"inspect_build: saved screenshot -> {out}")

    # ------------------------------------------------------------------
    # Hover label + click-to-isolate (mouse interaction)
    # ------------------------------------------------------------------
    # 2D overlay text that shows whichever part is under the cursor (or
    # the currently isolated part if any).  Sits near the top center so
    # it's visible regardless of camera angle without crowding the
    # bottom slider or the upper-left shortcuts.
    ww, wh = plotter.window_size
    hover_text_actor = plotter.add_text(
        "(hover a part)",
        position=(int(ww * 0.30), wh - 36),
        font_size=14,
        color="black",
        shadow=False,
        name="hover_label",
    )

    def _set_hover_text(s: str) -> None:
        try:
            hover_text_actor.SetInput(s)
        except Exception:
            # vtkTextActor in some PyVista builds uses SetText(0, s).
            try:
                hover_text_actor.SetText(0, s)
            except Exception:
                pass

    # Map actor identity -> placed instance for picker lookups.  Using
    # id() lets us key without paying for repeated isinstance checks.
    # The feature-edge overlay actor is registered as ``pickable=False``
    # so the picker shouldn't return it, but include it here too as a
    # safety net so a hit on the line actor still resolves to the right
    # PlacedInstance.
    actor_to_placed: dict[int, PlacedInstance] = {}
    for p in placed:
        actor_to_placed[id(p.actor)] = p
        if p.edges_actor is not None:
            actor_to_placed[id(p.edges_actor)] = p

    # vtk cell picker: cheap, ray-casts against renderer actors and
    # returns the closest hit.  We reuse a single picker instance.
    try:
        import vtk  # type: ignore
        picker = vtk.vtkCellPicker()
        picker.SetTolerance(0.001)
    except Exception:
        picker = None  # type: ignore

    def _placed_under_cursor() -> "PlacedInstance | None":
        if picker is None:
            return None
        try:
            x, y = plotter.iren.interactor.GetEventPosition()
        except Exception:
            return None
        try:
            picker.Pick(x, y, 0, plotter.renderer)
        except Exception:
            return None
        actor = picker.GetActor()
        if actor is None:
            return None
        return actor_to_placed.get(id(actor))

    def _label_for(p: PlacedInstance) -> str:
        return palette.instance_label(
            p.instance.part_type,
            p.instance.leg_index,
            p.instance.joint,
            fastener_role=p.instance.fastener_role,
        )

    def _on_mouse_move(_obj, _evt) -> None:
        # Don't overwrite the isolation banner with hover noise.
        if iso_state["placed"] is not None:
            return
        p = _placed_under_cursor()
        _set_hover_text(_label_for(p) if p is not None else "(hover a part)")
        plotter.render()

    def _apply_isolation_visuals() -> None:
        """Reflect the current iso_state on every actor's opacity.

        Drives both the surface actor and (when present) its feature-
        edge overlay so the dim/highlight effect reads as a single part.
        """
        iso = iso_state["placed"]
        for p in placed:
            companions = [p.actor]
            if p.edges_actor is not None:
                companions.append(p.edges_actor)
            for a in companions:
                if iso is None:
                    a.prop.opacity = saved_opacity[id(a)]
                elif p is iso:
                    # Keep isolated part at full opacity so it pops.
                    a.prop.opacity = max(saved_opacity[id(a)], 0.95)
                else:
                    a.prop.opacity = ISOLATE_DIM_OPACITY

    def _set_isolation(p: "PlacedInstance | None") -> None:
        iso_state["placed"] = p
        _apply_isolation_visuals()
        if p is None:
            _set_hover_text("(hover a part)")
        else:
            _set_hover_text(f"Isolated: {_label_for(p)}  [press I / Esc to clear]")
        plotter.render()

    # ------------------------------------------------------------------
    # Focus-on-sub-assembly
    # ------------------------------------------------------------------
    # Look-ups for resolving the SubAssembly's Instance / FastenerInstance
    # members back to their PlacedInstance + actor.  These are stable
    # for the lifetime of the plotter; nothing rebuilds them.
    placed_by_instance_id: dict[int, PlacedInstance] = {
        id(p.instance): p for p in placed
    }
    placed_by_fastener_role: dict[str, PlacedInstance] = {
        p.instance.fastener_role: p
        for p in placed
        if p.instance.fastener_role
    }

    def _sub_member_actors(
        sub: relations.SubAssembly,
    ) -> list[pv.Actor]:
        """Resolve a SubAssembly's members to their pyvista actors.

        Instances (focus, servos_inside, horns_below) are looked up by
        ``id()``; fasteners are looked up by their role string (the
        stable key that bridges FastenerInstance <-> Instance wrapper).
        Members that fail to resolve are silently dropped -- the
        consequence is just that one extra hardware item stays hidden
        in focus mode, which is the safe direction.
        """
        out: list[pv.Actor] = []
        for inst in (sub.focus, *sub.servos_inside, *sub.horns_below):
            p = placed_by_instance_id.get(id(inst))
            if p is not None:
                out.append(p.actor)
        for fi in sub.fasteners:
            p = placed_by_fastener_role.get(fi.role)
            if p is not None:
                out.append(p.actor)
        return out

    def _union_bounds(actors: list[pv.Actor]) -> tuple[float, ...] | None:
        """Union the world-space bounding boxes of every actor.

        ``vtkActor.GetBounds`` returns ``(xmin, xmax, ymin, ymax,
        zmin, zmax)`` AFTER applying any ``user_matrix`` (i.e. the
        explode shift is already baked in), so this works correctly
        regardless of explode-slider state.
        """
        xs: list[float] = []
        ys: list[float] = []
        zs: list[float] = []
        for a in actors:
            try:
                b = a.GetBounds()
            except Exception:
                continue
            if any((v is None or not np.isfinite(v)) for v in b):
                continue
            xs.extend((b[0], b[1]))
            ys.extend((b[2], b[3]))
            zs.extend((b[4], b[5]))
        if not xs:
            return None
        return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))

    def _apply_desired_visibility() -> None:
        """Drive every actor's visibility from the checkbox map.

        Called on focus exit so the user's per-part-type checkbox
        choices snap back into effect.  The fasteners master toggle
        wins over the per-fastener-type loop (the per-fastener-type
        row was removed from the column on construction, so the only
        toggle that flips fastener actors is ``__fasteners__``).
        """
        for pt, vis in checkbox_desired.items():
            if pt == "__fasteners__":
                continue
            for actor in actors_by_type.get(pt, []):
                actor.SetVisibility(bool(vis))
        for actor in fastener_actors:
            actor.SetVisibility(bool(checkbox_desired["__fasteners__"]))

    def _format_focus_banner(
        sub: relations.SubAssembly, p: PlacedInstance,
    ) -> str:
        bits: list[str] = []
        if sub.servos_inside:
            bits.append(f"{len(sub.servos_inside)} servo(s)")
        if sub.horns_below:
            bits.append(f"{len(sub.horns_below)} horn(s)")
        if sub.fasteners:
            bits.append(f"{len(sub.fasteners)} fastener(s)")
        n_total = 1 + len(sub.servos_inside) + len(sub.horns_below) + len(sub.fasteners)
        members_str = ", ".join(bits) if bits else "just the part itself"
        return (
            f"Focused: {_label_for(p)}  "
            f"({n_total} members: {members_str})  "
            f"[press F again or I / Esc to clear]"
        )

    def _set_focus(p: PlacedInstance | None) -> None:
        if p is None:
            if focus_state["placed"] is None:
                return
            focus_state["placed"] = None
            focus_state["members"] = set()
            focus_state["saved_vis"] = {}
            # Restore visibility from the per-part-type checkbox map,
            # so any changes the user made via the checkbox column
            # WHILE focused take effect on exit.
            _apply_desired_visibility()
            _set_hover_text("(hover a part)")
            plotter.render()
            return

        sub = relations.build_subassembly_for(
            p.instance, instances, all_fasteners,
        )
        member_actors = _sub_member_actors(sub)
        member_ids = {id(a) for a in member_actors}
        # Snapshot current visibility so we can restore on exit even
        # if the user enters focus mode via --focus at startup.
        saved_vis = {id(pp.actor): bool(pp.actor.GetVisibility()) for pp in placed}
        focus_state["placed"] = p
        focus_state["members"] = member_ids
        focus_state["saved_vis"] = saved_vis

        # Hide every non-member; show every member.  Members keep
        # their saved opacity (we explicitly don't dim them like the
        # isolation path does -- the whole point of focus is that the
        # surviving members read full-brightness).  The feature-edge
        # overlay actor tracks the surface actor's visibility 1:1 so
        # the hidden parts disappear cleanly (otherwise you'd see a
        # floating wire-frame skeleton of every hidden hole rim).
        for pp in placed:
            is_member = id(pp.actor) in member_ids
            pp.actor.SetVisibility(bool(is_member))
            if pp.edges_actor is not None:
                pp.edges_actor.SetVisibility(bool(is_member))
            if is_member:
                pp.actor.prop.opacity = saved_opacity[id(pp.actor)]
                if pp.edges_actor is not None:
                    pp.edges_actor.prop.opacity = saved_opacity[
                        id(pp.edges_actor)
                    ]

        # Drop any pending isolation -- focus supersedes it.
        iso_state["placed"] = None
        _apply_isolation_visuals()

        _set_hover_text(_format_focus_banner(sub, p))

        # Auto-fit camera to the sub-assembly bounds.
        bounds = _union_bounds(member_actors)
        if bounds is not None:
            plotter.reset_camera(bounds=bounds)
        else:
            plotter.reset_camera()
        plotter.render()

    def _clear_focus() -> None:
        _set_focus(None)

    # Double-click detection (timing heuristic).  PyVista's
    # ``track_click_position`` fires once per LeftButtonReleaseEvent,
    # which means the second click of a real double-click would
    # normally just bounce through ``_set_isolation`` again.  We
    # instead remember the time + screen position of the previous
    # left-click; if the current click lands within DOUBLE_CLICK_MS
    # of it AND within DOUBLE_CLICK_PX, we treat it as a double-click
    # and route to ``_set_focus`` instead.  This is the documented
    # fallback path in the focus-mode spec; observing
    # ``LeftButtonDoubleClickEvent`` directly would race the
    # trackball's own double-click binding, and the timing path is
    # robust against that.
    DOUBLE_CLICK_MS = 350.0
    DOUBLE_CLICK_PX = 5
    last_click_state: dict[str, float] = {"t_ms": -1.0e9, "x": -9999.0, "y": -9999.0}

    def _now_ms() -> float:
        import time
        return time.monotonic() * 1000.0

    def _on_left_click(point) -> None:
        # ``point`` is (x_pixel, y_pixel).
        if picker is None:
            return
        x, y = float(point[0]), float(point[1])
        now = _now_ms()
        dt = now - last_click_state["t_ms"]
        dx = abs(x - last_click_state["x"])
        dy = abs(y - last_click_state["y"])
        is_double = (
            dt < DOUBLE_CLICK_MS and dx < DOUBLE_CLICK_PX and dy < DOUBLE_CLICK_PX
        )
        last_click_state["t_ms"] = now
        last_click_state["x"] = x
        last_click_state["y"] = y

        try:
            picker.Pick(x, y, 0, plotter.renderer)
        except Exception:
            return
        actor = picker.GetActor()
        placed_hit = actor_to_placed.get(id(actor)) if actor is not None else None
        # Reject hits on fastener / servo actors as focus targets --
        # focusing on a fastener gives the user a sub-assembly of just
        # the fastener itself, which is rarely what they want.  The
        # ``--focus`` CLI flag is already gated on printable parts;
        # mirror that for the mouse here.
        if placed_hit is not None and (
            palette.is_fastener(placed_hit.instance.part_type)
            or placed_hit.instance.part_type in ("servo_body", "servo_horn")
        ):
            focus_target = None
        else:
            focus_target = placed_hit

        if is_double:
            if focus_target is None:
                # Double-click empty space (or on hardware) -> clear focus.
                _clear_focus()
                return
            # Same part double-clicked again -> toggle focus off.
            if focus_state["placed"] is focus_target:
                _clear_focus()
                return
            _set_focus(focus_target)
            return

        # Single click: route to isolation, but only when NOT focused.
        # While focused the single-click is a documented no-op (focus
        # already hides everything that would have been dimmed).
        if focus_state["placed"] is not None:
            return
        if placed_hit is None:
            if iso_state["placed"] is not None:
                _set_isolation(None)
        else:
            _set_isolation(placed_hit)

    def _on_f_key() -> None:
        """Toggle focus on whatever printable part is under the cursor."""
        p = _placed_under_cursor()
        if p is not None and (
            palette.is_fastener(p.instance.part_type)
            or p.instance.part_type in ("servo_body", "servo_horn")
        ):
            p = None
        if p is None:
            # Hovering empty space (or hardware) + F -> toggle off.
            if focus_state["placed"] is not None:
                _clear_focus()
            return
        if focus_state["placed"] is p:
            _clear_focus()
            return
        if iso_state["placed"] is not None:
            _set_isolation(None)
        _set_focus(p)

    def _clear_focus_or_isolation() -> None:
        """``I`` / ``Esc`` handler.  Focus takes priority on press."""
        cleared = False
        if focus_state["placed"] is not None:
            _clear_focus()
            cleared = True
        if iso_state["placed"] is not None:
            _set_isolation(None)
            cleared = True
        if not cleared:
            # Nothing to clear -- still rerender to refresh hover text.
            plotter.render()

    try:
        plotter.iren.add_observer("MouseMoveEvent", _on_mouse_move)
    except Exception:
        # Older PyVistas expose this via plotter.iren.interactor.
        try:
            plotter.iren.interactor.AddObserver(
                "MouseMoveEvent", _on_mouse_move,
            )
        except Exception:
            pass

    try:
        plotter.track_click_position(
            callback=_on_left_click, side="left", viewport=True,
        )
    except Exception:
        pass

    # ------------------------------------------------------------------
    # Camera pan / roll / zoom: trackpad gestures + keyboard fallback.
    # ------------------------------------------------------------------
    # Mouse-wheel / two-finger-swipe handling is left to VTK's default
    # trackball-style dolly, which matches the user's expectation that
    # "two fingers should zoom" (and matches the rest of the macOS CAD
    # ecosystem).  We DO still:
    #   1. Register Pan/Pinch/Rotate gesture observers, even though
    #      they currently never fire on macOS Cocoa.  On Cocoa the
    #      pinch gesture isn't forwarded to VTK today; on Linux/Qt
    #      builds (and on a hypothetical future VTK with Cocoa
    #      magnifyWithEvent: support) the pinch handler below maps
    #      pinch -> dolly, which keeps "two-finger pinch = zoom"
    #      consistent with the wheel default.
    #   2. Add a keyboard fallback (arrow keys to pan, ``,``/``.`` to
    #      roll, ``+``/``-`` to zoom) sized as a fraction of the
    #      current window so the user can frame the scene precisely
    #      even without modifier-drag gymnastics.
    #
    # The default vtkInteractorStyleTrackballCamera already supports
    # shift+drag = pan and cmd+drag = roll (on macOS Cmd folds into
    # the ControlKey state); those bindings are documented in
    # SHORTCUTS_TEXT but not redefined here.
    import math as _gesture_math

    def _world_per_pixel() -> float:
        """Length in world units of one screen pixel at the focal plane."""
        try:
            _, wh_px = plotter.window_size
            wh_px = max(1, int(wh_px))
        except Exception:
            wh_px = 1000
        cam = plotter.camera
        if cam.GetParallelProjection():
            world_h = 2.0 * cam.GetParallelScale()
        else:
            fp = np.asarray(cam.GetFocalPoint(), dtype=np.float64)
            pos = np.asarray(cam.GetPosition(), dtype=np.float64)
            distance = float(np.linalg.norm(fp - pos))
            va = _gesture_math.radians(cam.GetViewAngle())
            world_h = 2.0 * distance * _gesture_math.tan(va / 2.0)
        return float(world_h) / float(wh_px)

    def _pan_camera_pixels(dx_px: float, dy_px: float) -> None:
        """Pan the camera by ``(dx_px, dy_px)`` screen-space pixels.

        Sign convention: positive ``dx_px`` shifts the SCENE right
        (camera moves left), positive ``dy_px`` shifts the SCENE up
        (camera moves down).  That matches the macOS "natural
        scrolling" feel where the content moves with the fingers.
        """
        try:
            cam = plotter.camera
            fp = np.asarray(cam.GetFocalPoint(), dtype=np.float64)
            pos = np.asarray(cam.GetPosition(), dtype=np.float64)
            view_up = np.asarray(cam.GetViewUp(), dtype=np.float64)
            view_dir = fp - pos
            n = float(np.linalg.norm(view_dir))
            if n < 1e-9:
                return
            view_dir /= n
            right = np.cross(view_dir, view_up)
            rn = float(np.linalg.norm(right))
            if rn < 1e-9:
                return
            right /= rn
            up_ortho = np.cross(right, view_dir)
            wpp = _world_per_pixel()
            shift = (float(dx_px) * right + float(dy_px) * up_ortho) * wpp
            cam.SetFocalPoint(*(fp - shift))
            cam.SetPosition(*(pos - shift))
            try:
                plotter.renderer.ResetCameraClippingRange()
            except Exception:
                pass
            plotter.render()
        except Exception as exc:
            print(f"inspect_build: pan failed: {exc!r}")

    def _roll_camera(degrees: float) -> None:
        try:
            plotter.camera.Roll(float(degrees))
            plotter.render()
        except Exception as exc:
            print(f"inspect_build: roll failed: {exc!r}")

    def _dolly_camera(factor: float) -> None:
        """Multiplicative zoom: ``factor > 1`` zooms in, ``< 1`` zooms out."""
        try:
            cam = plotter.camera
            f = float(factor)
            if not np.isfinite(f) or f <= 0.0:
                return
            if cam.GetParallelProjection():
                cam.SetParallelScale(cam.GetParallelScale() / f)
            else:
                cam.Dolly(f)
            try:
                plotter.renderer.ResetCameraClippingRange()
            except Exception:
                pass
            plotter.render()
        except Exception as exc:
            print(f"inspect_build: zoom failed: {exc!r}")

    def _keyboard_pan_step_px() -> float:
        """Keyboard nudge size in screen pixels.

        ~6% of the smaller window dim feels right empirically: large
        enough to be useful, small enough for fine framing.  Since
        ``_pan_camera_pixels`` converts pixels -> world units via the
        current focal-plane scale on every call, the same pixel step
        produces the same on-screen motion at any zoom level.
        """
        try:
            ww, wh = plotter.window_size
            return 0.06 * float(min(ww, wh))
        except Exception:
            return 40.0

    # ---- Forward-compatible VTK gesture observers ----
    # vtkRenderWindowInteractor emits {Start,,End}{Pinch,Rotate,Pan}Event
    # when the platform view forwards multitouch gestures.  Today this
    # only happens on Qt + Win32; macOS Cocoa doesn't (see comment
    # block above).  Wiring the observers anyway costs nothing and
    # means the right thing happens on Linux/Qt builds and on a
    # future VTK that adds Cocoa gesture support.
    def _on_pinch_gesture(caller, _event) -> None:
        try:
            scale = float(caller.GetScale())
        except Exception:
            return
        if not np.isfinite(scale) or scale <= 0.0:
            return
        _dolly_camera(scale)

    def _on_pan_gesture(caller, _event) -> None:
        try:
            tx, ty = caller.GetTranslation()
        except Exception:
            return
        _pan_camera_pixels(float(tx), float(ty))

    def _on_rotate_gesture(caller, _event) -> None:
        try:
            rot = float(caller.GetRotation())
        except Exception:
            return
        if not np.isfinite(rot):
            return
        _roll_camera(rot)

    try:
        iren_raw = plotter.iren.interactor
        # RecognizeGestures defaults to True on VTK 9.x; set it
        # explicitly for robustness against older builds.
        try:
            iren_raw.SetRecognizeGestures(True)
        except Exception:
            pass
        iren_raw.AddObserver("PinchEvent", _on_pinch_gesture)
        iren_raw.AddObserver("PanEvent", _on_pan_gesture)
        iren_raw.AddObserver("RotateEvent", _on_rotate_gesture)
        # Start/End placeholders kept here for symmetry -- we don't
        # need them today (no per-gesture accumulator state), but
        # observing them ensures VTK's gesture-recognizer machinery
        # actually fires the middle events.
        for _ev_name in (
            "StartPinchEvent", "EndPinchEvent",
            "StartPanEvent", "EndPanEvent",
            "StartRotateEvent", "EndRotateEvent",
        ):
            iren_raw.AddObserver(_ev_name, lambda *_a: None)
    except Exception as exc:
        print(f"inspect_build: gesture observer setup failed: {exc!r}")

    # ---- Keyboard fallback ----
    def _kbd_pan_left() -> None:
        _pan_camera_pixels(-_keyboard_pan_step_px(), 0.0)

    def _kbd_pan_right() -> None:
        _pan_camera_pixels(+_keyboard_pan_step_px(), 0.0)

    def _kbd_pan_up() -> None:
        _pan_camera_pixels(0.0, +_keyboard_pan_step_px())

    def _kbd_pan_down() -> None:
        _pan_camera_pixels(0.0, -_keyboard_pan_step_px())

    def _kbd_roll_ccw() -> None:
        _roll_camera(-5.0)

    def _kbd_roll_cw() -> None:
        _roll_camera(+5.0)

    def _kbd_zoom_in() -> None:
        _dolly_camera(1.15)

    def _kbd_zoom_out() -> None:
        _dolly_camera(1.0 / 1.15)

    for k in ("l", "L"):
        plotter.add_key_event(k, _toggle_labels)
    for k in ("e", "E"):
        plotter.add_key_event(k, _toggle_explode)
    for k in ("f", "F"):
        plotter.add_key_event(k, _on_f_key)
    for k in ("r", "R"):
        plotter.add_key_event(k, _reset_camera)
    for k in ("s", "S"):
        plotter.add_key_event(k, _save_screenshot)
    for k in ("i", "I", "Escape"):
        plotter.add_key_event(k, _clear_focus_or_isolation)
    for k in ("q", "Q"):
        plotter.add_key_event(k, plotter.close)

    # Pan / roll / zoom keys.  We bind both X11 keysym names (the
    # canonical thing GetKeySym() returns for arrow / punctuation
    # keys on most VTK builds) AND the literal character forms, so
    # the bindings survive any keysym-table inconsistency between
    # Cocoa / X11 / Qt.  PyVista's add_key_event stores callbacks
    # in a defaultdict keyed by the exact GetKeySym() string, so
    # double-registering on synonyms is cheap and side-effect-free.
    for k in ("Left",):
        plotter.add_key_event(k, _kbd_pan_left)
    for k in ("Right",):
        plotter.add_key_event(k, _kbd_pan_right)
    for k in ("Up",):
        plotter.add_key_event(k, _kbd_pan_up)
    for k in ("Down",):
        plotter.add_key_event(k, _kbd_pan_down)
    for k in ("comma", ","):
        plotter.add_key_event(k, _kbd_roll_ccw)
    for k in ("period", "."):
        plotter.add_key_event(k, _kbd_roll_cw)
    for k in ("plus", "equal", "+", "="):
        plotter.add_key_event(k, _kbd_zoom_in)
    for k in ("minus", "underscore", "-", "_"):
        plotter.add_key_event(k, _kbd_zoom_out)

    try:
        plotter.show_axes()
    except Exception:
        pass
    try:
        plotter.enable_anti_aliasing("msaa")
    except Exception:
        pass

    # Expose _set_focus on the plotter so run() can activate --focus
    # AFTER its own ``camera_position = "iso"`` / ``reset_camera()``
    # (which would otherwise clobber the sub-assembly bounds set by
    # _set_focus).  Stashing on the plotter object avoids changing
    # the function's return type.
    plotter._inspect_build_set_focus = _set_focus  # type: ignore[attr-defined]
    plotter._inspect_build_initial_focus = initial_focus  # type: ignore[attr-defined]


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Interactive PyVista build inspector for the hexapod "
            "prototype.  Colored, labeled parts placed at the assembled "
            "pose; exploded-view slider + per-part-type hide toggles."
        ),
    )
    parser.add_argument(
        "--screenshot",
        type=str,
        default=None,
        help=(
            "Render once headless to PATH and exit.  When set, no "
            "interactive window is opened."
        ),
    )
    parser.add_argument(
        "--explode",
        type=float,
        default=0.0,
        help=(
            "Initial explode amount in [0, 2].  0 = exact assembled "
            "pose; 1 = pulled out enough to inspect joints; 2 = fully "
            "exploded."
        ),
    )
    parser.add_argument(
        "--focus",
        type=str,
        default=None,
        help=(
            "Open the inspector with focus-on-sub-assembly mode "
            "already active on the named printable part.  Format: "
            "'PART_TYPE/L<n>' for per-leg parts (e.g. 'coxa_link/L0', "
            "'femur_link/L3') or just 'PART_TYPE' for chassis-level "
            "parts (e.g. 'chassis_top').  Composes with --screenshot "
            "for headless captures of one sub-assembly."
        ),
    )
    parser.add_argument(
        "--window-w", type=int, default=1600, help="Window width (pixels).",
    )
    parser.add_argument(
        "--window-h", type=int, default=1000, help="Window height (pixels).",
    )
    args = parser.parse_args(argv)

    if not 0.0 <= args.explode <= 2.0:
        print(
            f"inspect_build: --explode must be in [0, 2], got {args.explode}",
            file=sys.stderr,
        )
        sys.exit(2)

    run(
        screenshot=args.screenshot,
        explode=args.explode,
        focus=args.focus,
        window_size=(args.window_w, args.window_h),
    )


if __name__ == "__main__":
    main()
