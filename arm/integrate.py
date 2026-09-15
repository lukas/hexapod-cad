"""Optional-arm integration helpers for the prototype build / verify flow.

This module is loaded ONLY when the user opts in via the ``--with-arm``
flag on ``build_all.py`` or ``_verify_prototype.py``.  None of the
prototype's default codepaths touch it, so a developer who doesn't
print the arm never pays for its imports or its STLs.

Two surfaces:

    * :func:`build_with_arm` -- regenerates ``stl_arm/*.stl`` via
      :func:`arm.main` (and optionally an assembly-preview PNG).
      Called from ``build_all.py --with-arm``.
    * :func:`verify_arm_parts` and :func:`check_arm_interference` -- run
      the flimsy-joints check on the 5 new arm parts and a voxel-overlap
      collision check between the arm in its neutral pose and the
      prototype chassis-top + standing-pose legs.  Called from
      ``_verify_prototype.py --with-arm``.
"""

from __future__ import annotations

import importlib
import os
import sys
from typing import Any


# ---------------------------------------------------------------------------
# Path bootstrapping (so `import arm` / `import hexapod_prototype` work
# regardless of where the parent script was launched from).
# ---------------------------------------------------------------------------

_HERE = os.path.dirname(os.path.abspath(__file__))
_PROTO_DIR = os.path.normpath(os.path.join(_HERE, os.pardir))
for _p in (_HERE, _PROTO_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _import_arm():
    """Lazy-import the arm part generator (so this module is cheap to
    import when ``--with-arm`` is not set)."""
    return importlib.import_module("arm")


# ---------------------------------------------------------------------------
# Build pipeline
# ---------------------------------------------------------------------------

def build_with_arm(*, render_preview: bool = True,
                   build_bambu: bool = False) -> None:
    """Regenerate everything the optional arm needs.

    1. Run :func:`arm.main` to rebuild printable STLs into ``stl_arm/``
       and the assembly preview into ``stl_reference/``.
    2. If ``render_preview`` is True (and PyVista imports succeed),
       render ``renders/arm_assembly_preview.png``.  The render is
       OFF-SCREEN and may fail in headless CI without libGL; we treat
       that as a soft warning, not a hard failure.
    3. ``build_bambu`` is retained only for CLI compatibility; Bambu
       plate packing was removed — print individuals from ``stl_arm/``.
    """
    arm_mod = _import_arm()

    print()
    print("=" * 72)
    print("Optional arm: regenerating prototype/arm/stl_arm/*.stl")
    print("=" * 72)
    arm_mod.main()

    if render_preview:
        print()
        print("=" * 72)
        print("Optional arm: rendering assembly preview PNG")
        print("=" * 72)
        try:
            render_mod = importlib.import_module("render_arm_preview")
            render_mod.main()
        except Exception as exc:
            print(f"  WARN: skipping arm preview render: {exc!r}")

    if build_bambu:
        print()
        print("Optional arm: Bambu plate packing was removed — "
              "print individuals from arm/stl_arm/.")

# ---------------------------------------------------------------------------
# Verification: flimsy joints + chassis / leg interference
# ---------------------------------------------------------------------------

ARM_NEW_PART_BUILDERS: dict[str, str] = {
    "arm_base_bracket":  "make_arm_base_bracket",
    "wrist_adapter":     "make_wrist_adapter",
    "gripper_base":      "make_gripper_base",
    "gripper_jaw_left":  "make_gripper_jaw_left",
    "gripper_jaw_right": "make_gripper_jaw_right",
}


def _build_arm_new_parts() -> dict[str, Any]:
    """Return a dict mapping arm-part name -> trimesh.Trimesh for each
    of the 5 genuinely new arm parts (not the leg re-exports)."""
    arm_mod = _import_arm()
    return {
        name: getattr(arm_mod, builder)()
        for name, builder in ARM_NEW_PART_BUILDERS.items()
    }


def verify_arm_parts(*,
                     min_print_t: float,
                     pitch: float,
                     min_cluster_vox: int,
                     max_flimsy_budget_vox: int,
                     flimsy_cluster_fn,
                     label_fn) -> bool:
    """Run the prototype's flimsy-joints check on the 5 NEW arm parts.

    The caller (``_verify_prototype.check_flimsy_joints`` or its
    arm-only wrapper) provides the cluster-extraction function
    (``_flimsy_clusters_for_part``) and the result-label printer
    (``_label``) so this module doesn't depend on them at import
    time.
    """
    print(f"\n[6b] Optional arm: flimsy joints (local thickness < "
          f"{min_print_t:.1f} mm; pitch={pitch:.1f} mm, "
          f"min cluster={min_cluster_vox} vox, "
          f"budget={max_flimsy_budget_vox} vox):")
    parts = _build_arm_new_parts()
    all_ok = True
    for name, mesh in parts.items():
        clusters, biggest, max_t = flimsy_cluster_fn(
            mesh, pitch, min_print_t, min_cluster_vox)
        ok = biggest <= max_flimsy_budget_vox
        all_ok &= ok
        n_clusters = len(clusters)
        head = (f"{n_clusters} flimsy cluster(s); "
                f"largest = {biggest:4d} vox "
                f"(budget {max_flimsy_budget_vox}); "
                f"part max thickness = {max_t:5.1f} mm")
        label_fn(name, ok, head)
        for cluster in clusters[:3]:
            cx, cy, cz = cluster["centroid"]
            extent = cluster["bbox_max"] - cluster["bbox_min"]
            print(f"           - {cluster['voxel_count']:4d} vox  "
                  f"min t={cluster['min_thickness']:4.2f} mm  "
                  f"centroid=({cx:+7.2f},{cy:+7.2f},{cz:+7.2f}) mm  "
                  f"bbox={extent[0]:5.1f} x {extent[1]:5.1f} "
                  f"x {extent[2]:5.1f} mm")
    return all_ok


# ---------------------------------------------------------------------------
# Arm-vs-chassis / arm-vs-leg interference check
# ---------------------------------------------------------------------------

# Tolerance for the per-pair voxel overlap (mm^3).  The arm's base
# bracket sits ON TOP of the chassis top plate via its flange -- the
# flange-to-plate interface itself is a planar contact, not a volume
# overlap, so any overlap > a few cubic mm of voxel noise indicates
# a real clash (centre bolts poking through, well wall biting into
# the plate, etc.).
ARM_CHASSIS_TOL_MM3 = 200.0
ARM_LEG_TOL_MM3 = 200.0


def _build_one_leg_in_body_frame(leg_index: int):
    """Mirror of ``_verify_prototype._build_standing_leg`` but
    parameterised by ``leg_index`` (the upstream version hard-codes
    leg 0 for self-collision; the arm check needs all 6 legs).

    Returns a dict ``{"coxa_link": mesh, "femur_link": mesh,
    "tibia_link": mesh}`` placed in the body's world frame for the
    chosen edge (``a = (leg_index + 0.5) * np.pi / 3``).  No servo
    bodies or horns -- just the 3 printed leg parts (May 2026 Design
    F: the legacy ``coxa_bracket`` mesh has been folded into
    ``chassis_bottom``).  Kept here so the arm check stays additive and
    doesn't depend on private helpers in ``_verify_prototype``
    being parameterised.
    """
    import numpy as np  # noqa: WPS433
    from trimesh.transformations import rotation_matrix  # noqa: WPS433
    hp = importlib.import_module("hexapod_prototype")

    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3
    edge_mid = np.array([apothem * np.cos(a),
                         apothem * np.sin(a),
                         0.0])
    z_hat = np.array([0.0, 0.0, 1.0])

    plastic_horn_h = 5.0
    yaw_output_z = ((hp.SERVO_BODY_H - hp.WELL_RIM_Z)
                    + hp.SERVO_OUTPUT_H
                    + plastic_horn_h
                    + hp.HORN_ADAPTER_T)
    arm_t = 6.0
    hip_drop = hp.COXA_HIP_DROP
    hip_joint_local = np.array([hp.COXA_LENGTH, 0.0, hip_drop])
    # NEW (May 2026 collinear-pad refactor): femur / tibia local
    # origins are pad mating faces, +HORN_STACK_H above the joint
    # axis along link +Y.
    pad_axis_offset = np.array([0.0, hp.HORN_STACK_H, 0.0])

    p = np.deg2rad(hp.STANCE_FEMUR_DEG)
    pt = np.deg2rad(hp.STANCE_FEMUR_DEG + hp.STANCE_TIBIA_DEG)
    ry_p = rotation_matrix(p, [0, 1, 0])[:3, :3]
    knee_joint_local = hip_joint_local + ry_p @ np.array(
        [hp.FEMUR_LENGTH, 0, 0])

    parts: dict[str, Any] = {}

    cl = hp.make_coxa_link()
    cl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    cl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["coxa_link"] = cl

    fl = hp.make_femur_link()
    fl.apply_transform(rotation_matrix(p, [0, 1, 0]))
    fl.apply_translation(hip_joint_local + pad_axis_offset)
    fl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    fl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["femur_link"] = fl

    tl = hp.make_tibia_link()
    tl.apply_transform(rotation_matrix(pt, [0, 1, 0]))
    tl.apply_translation(knee_joint_local + pad_axis_offset)
    tl.apply_transform(rotation_matrix(a, [0, 0, 1]))
    tl.apply_translation(edge_mid + yaw_output_z * z_hat)
    parts["tibia_link"] = tl

    return parts


def check_arm_interference(*,
                           pair_overlap_fn,
                           label_fn) -> bool:
    """Voxel-volume overlap between the optional arm (in neutral pose,
    bolted to the chassis-top centre) and the prototype's chassis-top
    plate / standing-pose leg parts.

    PASSES iff every pairwise overlap stays below
    ``ARM_CHASSIS_TOL_MM3`` (chassis_top) or ``ARM_LEG_TOL_MM3``
    (six legs).

    Mirrors the existing :func:`_verify_prototype.check_self_collision`
    pattern -- voxel-sampling each candidate AABB intersection and
    summing the intersected volume.  The caller passes in the overlap
    function (``_pair_overlap_volume``) and the label printer
    (``_label``) so we don't import private names from
    ``_verify_prototype`` at module-load time.
    """
    print("\n[4b] Optional arm: arm-vs-chassis / arm-vs-leg interference "
          "(arm in neutral pose at J2=-25, J3=+60, J4=+25):")
    all_ok = True

    arm_mod = _import_arm()
    hp = importlib.import_module("hexapod_prototype")
    import numpy as np  # noqa: WPS433

    chassis_top_z = hp.CHASSIS_PLATE_T / 2.0
    arm_parts_list = arm_mod._arm_in_chassis_frame(  # noqa: SLF001
        chassis_top_z=chassis_top_z,
    )
    # Mirror the names used by render_arm_preview.py so the printed
    # results are self-explanatory.
    arm_part_names = [
        "arm_base_bracket", "j1_horn", "arm_shoulder_link",
        "arm_upper", "arm_forearm", "wrist_adapter",
        "gripper_base", "gripper_jaw_left", "gripper_jaw_right",
    ]
    if len(arm_parts_list) != len(arm_part_names):
        arm_part_names = [
            f"arm_part_{i:02d}" for i in range(len(arm_parts_list))
        ]
    arm_parts = dict(zip(arm_part_names, arm_parts_list))

    chassis_top = hp.make_chassis_top()  # already centred on chassis frame

    standing_legs: dict[str, Any] = {}
    for leg_index in range(6):
        leg_parts = _build_one_leg_in_body_frame(leg_index)
        for part_name, mesh in leg_parts.items():
            standing_legs[f"leg{leg_index}_{part_name}"] = mesh

    print("  vs chassis_top:")
    for arm_name, arm_mesh in arm_parts.items():
        vol = pair_overlap_fn(arm_mesh, chassis_top, pitch=1.5)
        ok = vol <= ARM_CHASSIS_TOL_MM3
        all_ok &= ok
        label_fn(f"{arm_name} vs chassis_top", ok,
                 f"overlap = {vol:7.1f} mm^3 (tol "
                 f"{ARM_CHASSIS_TOL_MM3:.0f})")

    print("  vs standing-pose legs (6 legs x 4 parts each, "
          "aggregated worst per arm part):")
    for arm_name, arm_mesh in arm_parts.items():
        total_per_leg = []
        for leg_index in range(6):
            vol_leg = 0.0
            for part_name in ("coxa_link",
                              "femur_link", "tibia_link"):
                leg_mesh = standing_legs.get(f"leg{leg_index}_{part_name}")
                if leg_mesh is None:
                    continue
                vol_leg += pair_overlap_fn(arm_mesh, leg_mesh, pitch=1.5)
            total_per_leg.append(vol_leg)
        worst_leg = int(np.argmax(total_per_leg))
        worst_vol = float(total_per_leg[worst_leg])
        ok = worst_vol <= ARM_LEG_TOL_MM3
        all_ok &= ok
        label_fn(f"{arm_name} vs legs (worst=leg{worst_leg})", ok,
                 f"overlap = {worst_vol:7.1f} mm^3 (tol "
                 f"{ARM_LEG_TOL_MM3:.0f})")

    return all_ok
