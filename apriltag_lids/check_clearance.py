#!/usr/bin/env python3
"""Audit the AprilTag lid and flush screw heads through the joint ROM."""

from __future__ import annotations

import itertools
import json
import sys
import warnings
from pathlib import Path

import numpy as np
import trimesh


HERE = Path(__file__).resolve().parent
PROTO_DIR = HERE.parent
for path in (PROTO_DIR, HERE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import _verify_prototype as verify  # noqa: E402
import hexapod_prototype as hp  # noqa: E402
import make_apriltag_lids as lids  # noqa: E402


MIN_CRITICAL_CLEARANCE_MM = 0.40


def print_to_well_transform() -> np.ndarray:
    """Map printable XYZ to the clamp's well-local XYZ.

    The part prints in its camera-facing plane.  Installed, print +Z points
    along well +Y, while print +Y points along well +Z.
    """
    outer_face_y = hp.WELL_D / 2.0 + hp.CLAMP_CAP_T
    return np.asarray([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, outer_face_y],
        [0.0, 1.0, 0.0, hp.CLAMP_BOLT_Z],
        [0.0, 0.0, 0.0, 1.0],
    ])


def placed_accessories(template: trimesh.Trimesh, yaw_deg: float,
                       femur_deg: float, leg_az: float):
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    yaw_output_world = np.asarray([
        apothem * np.cos(leg_az),
        apothem * np.sin(leg_az),
        hp.CHASSIS_YAW_OUTPUT_Z,
    ])
    rotate_leg = trimesh.transformations.rotation_matrix(leg_az, [0, 0, 1])
    rotate_yaw = trimesh.transformations.rotation_matrix(
        np.deg2rad(yaw_deg), [0, 0, 1]
    )

    hip = template.copy()
    hip.apply_transform(hp._joint_place(
        hp.COXA_HIP_ANCHOR, (1, 0, 0), hp.LEG_PITCH_AXIS
    ))
    hip.apply_transform(rotate_yaw)
    hip.apply_transform(rotate_leg)
    hip.apply_translation(yaw_output_world)

    knee = template.copy()
    knee.apply_transform(hp._joint_place(
        (hp.FEMUR_LENGTH, 0, 0), (1, 0, 0), hp.LEG_PITCH_AXIS
    ))
    knee.apply_transform(trimesh.transformations.rotation_matrix(
        np.deg2rad(femur_deg), [0, 1, 0]
    ))
    knee.apply_translation(np.asarray(hp.COXA_HIP_ANCHOR))
    knee.apply_transform(rotate_yaw)
    knee.apply_transform(rotate_leg)
    knee.apply_translation(yaw_output_world)
    return {"hip_clamp_cap": hip, "knee_clamp_cap": knee}


def exact_intersection_volume(a: trimesh.Trimesh,
                              b: trimesh.Trimesh) -> float:
    lo = np.maximum(a.bounds[0], b.bounds[0])
    hi = np.minimum(a.bounds[1], b.bounds[1])
    if np.any(hi <= lo):
        return 0.0
    overlap = trimesh.boolean.intersection(
        [a, b], engine="manifold", check_volume=False
    )
    if overlap is None or len(overlap.faces) == 0:
        return 0.0
    if np.any(np.asarray(overlap.extents) <= 1e-7):
        return 0.0
    return abs(float(overlap.volume))


def main() -> None:
    white, black = lids.build_tag_meshes(1)
    accessory = lids.clearance_envelope(white, black)
    accessory.apply_transform(print_to_well_transform())

    leg_az = np.pi / 6.0
    _, templates = verify._ws_get_chassis_and_templates(leg_az)
    standing = (
        0.0,
        float(hp.STANCE_FEMUR_DEG),
        float(hp.STANCE_TIBIA_DEG),
    )
    samples = itertools.product(
        np.linspace(*verify.WORKSPACE_YAW_DEG, verify.WORKSPACE_N_YAW),
        np.linspace(*verify.WORKSPACE_FEMUR_DEG, verify.WORKSPACE_N_FEMUR),
        np.linspace(*verify.WORKSPACE_KNEE_DEG, verify.WORKSPACE_N_KNEE),
    )
    poses = [standing]
    poses.extend(
        tuple(float(value) for value in pose)
        for pose in samples
        if tuple(float(value) for value in pose) != standing
    )

    hits = []
    pair_tests = 0
    for yaw_deg, femur_deg, knee_deg in poses:
        leg = verify._build_workspace_leg(
            yaw_deg,
            femur_deg,
            knee_deg,
            leg_azimuth_rad=leg_az,
            templates=templates,
        )
        accessories = placed_accessories(accessory, yaw_deg, femur_deg, leg_az)
        for cap_name, cap_mesh in accessories.items():
            for moving_name in verify._WS_CLAMP_CAP_PAIRS[cap_name]:
                pair_tests += 1
                volume = exact_intersection_volume(cap_mesh, leg[moving_name])
                if volume > 1e-5:
                    hits.append({
                        "pose_deg": [yaw_deg, femur_deg, knee_deg],
                        "accessory": cap_name,
                        "moving_part": moving_name,
                        "intersection_mm3": volume,
                    })

    # The closest known approach occurs at hip pitch -60 degrees.  A
    # bidirectional vertex-to-surface query includes the installed flat-head
    # envelopes and is deterministic (no stochastic surface sampling).
    critical_pose = (0.0, -60.0, -20.0)
    critical_leg = verify._build_workspace_leg(
        *critical_pose,
        leg_azimuth_rad=leg_az,
        templates=templates,
    )
    critical_cap = placed_accessories(
        accessory, critical_pose[0], critical_pose[1], leg_az
    )["hip_clamp_cap"]
    critical_link = critical_leg["femur_link"]
    # Trimesh can emit a harmless divide warning for degenerate candidate
    # triangles that it subsequently rejects; the returned distances remain
    # finite and are explicitly consumed below.
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore", category=RuntimeWarning, module="trimesh.triangles"
        )
        cap_to_link = trimesh.proximity.closest_point(
            critical_link, critical_cap.vertices
        )[1]
        link_to_cap = trimesh.proximity.closest_point(
            critical_cap, critical_link.vertices
        )[1]
    minimum_clearance = float(min(cap_to_link.min(), link_to_cap.min()))

    report = {
        "schema_version": 1,
        "poses": len(poses),
        "pair_tests": pair_tests,
        "ranges_deg": {
            "yaw": list(verify.WORKSPACE_YAW_DEG),
            "hip": list(verify.WORKSPACE_FEMUR_DEG),
            "knee": list(verify.WORKSPACE_KNEE_DEG),
        },
        "exact_intersections": hits,
        "critical_pose_deg": list(critical_pose),
        "critical_vertex_surface_clearance_mm": round(minimum_clearance, 4),
        "required_critical_clearance_mm": MIN_CRITICAL_CLEARANCE_MM,
        "includes_flush_m3_head_envelopes": True,
        "pass": not hits and minimum_clearance >= MIN_CRITICAL_CLEARANCE_MM,
    }
    out = HERE / "out" / "clearance_report.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if not report["pass"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
