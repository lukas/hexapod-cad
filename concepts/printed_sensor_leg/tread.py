"""Thick TPU snap rim for the instrumented foot's moving shoe.

Retires the predecessor's nearly vanishing rim by moving retention below the
rigid overload flange. Dimensions are millimetres; manufacturing fit remains
a prototype assumption until a TPU/rigid-material coupon is printed.
"""
from __future__ import annotations

import math

import numpy as np
import trimesh


def make_reinforced_tread(
    meshes: dict[str, trimesh.Trimesh],
) -> tuple[trimesh.Trimesh, trimesh.Trimesh, dict]:
    """Return tread, revised carriage and diagnostics without mutating inputs.

    Expected predecessor keys: hall_tpu_tread, hall_guided_carriage and
    hall_foot_housing. The original upper carriage, guide, magnet seat and
    9.8-radius overload flange are retained. A new smaller snap bead beneath
    that flange gives the TPU sufficient thickness inside the housing bore.
    """
    required = {"hall_tpu_tread", "hall_guided_carriage", "hall_foot_housing"}
    if missing := required - meshes.keys():
        raise ValueError(f"Missing predecessor foot meshes {sorted(missing)}; load the instrumented-foot v1 assets.")

    sphere_center_z = 13.4
    tread_outer_r = 15.4  # Contact datum remains z=-2, as in sensor-foot v1.
    tread_inner_r = 12.4
    shoe_r = 12.25
    rim_outer_r = 9.4
    pocket_r = 8.35
    throat_r = 7.8
    bead_r = 8.15
    neck_r = 7.6
    pocket_top_z = 4.4
    throat_bottom_z = 4.55
    rim_top_z = 5.35
    flange_bottom_z = 5.55

    def sphere_profile(radius: float, edge_radius: float) -> np.ndarray:
        angles = np.linspace(0, math.asin(edge_radius / radius), 80)
        return np.column_stack((radius * np.sin(angles),
                                sphere_center_z - radius * np.cos(angles)))

    edge_r = math.sqrt(tread_outer_r**2 - (sphere_center_z - 3.6)**2)
    outer = sphere_profile(tread_outer_r, edge_r)
    # A broad seat backs the bead; carrying the sphere to the bead's edge
    # would leave a knife edge on the rigid snap feature.
    inner = sphere_profile(tread_inner_r, 7.15)
    tread_profile = np.vstack([
        outer,
        [[rim_outer_r, 3.6], [rim_outer_r, rim_top_z],
         [throat_r, rim_top_z], [throat_r, throat_bottom_z],
         [pocket_r, pocket_top_z], [pocket_r, 3.35]],
        inner[::-1],
    ])
    tread = trimesh.creation.revolve(tread_profile, sections=128)

    shoe_profile = np.vstack([
        sphere_profile(shoe_r, 7.15),
        [[bead_r, 3.55], [bead_r, 4.35], [neck_r, 4.45],
         [neck_r, flange_bottom_z + 0.02], [0, flange_bottom_z + 0.02]],
    ])
    shoe = trimesh.creation.revolve(shoe_profile, sections=128)
    upper_keep = trimesh.creation.box([40, 40, 100])
    upper_keep.apply_translation([0, 0, flange_bottom_z + 50])
    upper = trimesh.boolean.intersection(
        [meshes["hall_guided_carriage"], upper_keep], engine="manifold")
    carriage = trimesh.boolean.union([upper, shoe], engine="manifold")

    def overlap_volume(a, b) -> float:
        overlap = trimesh.boolean.intersection([a, b], engine="manifold")
        return abs(float(overlap.volume)) if len(overlap.faces) else 0.0

    parts = {"hall_tpu_tread": tread, "hall_guided_carriage": carriage}
    invalid = [name for name, mesh in parts.items()
               if not mesh.is_watertight or len(mesh.split()) != 1 or mesh.volume <= 0]
    checks = []
    for stroke in (0, 0.35, 1.5):
        for name, mesh in parts.items():
            moving = mesh.copy()
            moving.apply_translation([0, 0, stroke])
            checks.append({"pair": f"{name}/housing", "strokeMm": stroke,
                           "overlapMm3": overlap_volume(moving, meshes["hall_foot_housing"])})
    checks.append({"pair": "tread/carriage", "strokeMm": 0,
                   "overlapMm3": overlap_volume(tread, carriage)})
    # The upper carriage is a functional interface: preserve its geometry,
    # with no added or removed volume above the flange.
    revised_upper = trimesh.boolean.intersection([carriage, upper_keep], engine="manifold")
    upper_difference = trimesh.boolean.difference([revised_upper, upper], engine="manifold")
    checks.append({"pair": "upper-carriage-added-volume", "overlapMm3":
                   abs(float(upper_difference.volume)) if len(upper_difference.faces) else 0.0})
    missing_upper = trimesh.boolean.difference([upper, revised_upper], engine="manifold")
    checks.append({"pair": "upper-carriage-removed-volume", "overlapMm3":
                   abs(float(missing_upper.volume)) if len(missing_upper.faces) else 0.0})
    report = {
        "passed": not invalid and all(c["overlapMm3"] < 0.001 for c in checks),
        "invalidMeshes": invalid,
        "checks": checks,
        "tipDatumZMm": -2.0,
        "rimOuterRadiusMm": rim_outer_r,
        "snapPocketRadiusMm": pocket_r,
        "snapThroatRadiusMm": throat_r,
        "shoeSnapBeadRadiusMm": bead_r,
        "shoeNeckRadiusMm": neck_r,
        "snapRadialUndercutMm": bead_r - throat_r,
        "nominalThroatStretchPercent": 100 * (bead_r / throat_r - 1),
        "designedPocketSidewallRadialMm": rim_outer_r - pocket_r,
        "designedReturnLipAxialMm": rim_top_z - throat_bottom_z,
        "designedShoeBeadAxialMm": 0.8,
        "rimToOverloadFlangeAxialGapMm": flange_bottom_z - rim_top_z,
        "preservedOverloadFlange": {"radiusMm": 9.8, "zMm": [5.55, 6.0]},
        "volumesMm3": {name: float(mesh.volume) for name, mesh in parts.items()},
        "scope": "Closed solids and seated geometry through three stroke states; TPU stretch, print fit, retention, fatigue and loaded deformation require physical tests.",
    }
    if not report["passed"]:
        raise RuntimeError(f"Reinforced tread geometry failed; fix the snap or housing clearance before publishing: {report}")
    return tread, carriage, report
