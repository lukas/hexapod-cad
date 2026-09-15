#!/usr/bin/env python3
"""SEED probe for the OUTBOARD hip-pivot offset (COXA_EXT): how far
must the hip servo move out along the coxa +x so a PLAIN C-clamp (no
swan-neck notch, no web bevel, no cap crown shave) clears the plate +
hatch on the way past vertical?

Two tests per candidate offset:
  A. raster of the PLAIN clamp plan (full rectangle + disc round end +
     full-height web, both blade slabs) about the shifted hip axis;
  B. the knee-side stack as PRODUCTION shapes (femur_link block region,
     production knee_clamp_cap) + this concept's tibia socket/tube,
     sampled by vertices + surface points.

FINDINGS (kept for provenance -- the FINAL value was set by sweeping
the real BREP solids in make_cnc_chorn_variant.py, see its COXA_EXT
block): the blade raster clears from ext 36, but the real clamp's low
corners + pad bosses (not rastered here) meet the plate's yaw-tower
collar until ext 42; the knee-end parts saturate at first contact
-117.5 at ANY offset (the phi44 dust collar always sits under the
knee-end's inboard dip window -- the physics ceiling), which is why
the printed knee block + cap keep the single WEDGE chamfer.

Run:  uv run python concepts/cnc_chorn_overhead/_probe_outboard.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "rigid_hip"))

import hexapod_prototype as hp  # noqa: E402
import make_rigid_hip_variant as rv  # noqa: E402
import make_cnc_chorn_variant as cv  # noqa: E402


def _trans(v):
    M = np.eye(4)
    M[:3, 3] = v[:3]
    return M


def _rotz(t):
    return rotation_matrix(t, [0, 0, 1])


def leg_T(ext: float, yaw_deg: float, femur_deg: float,
          tibia_deg: float | None = None):
    """rv.leg_transforms with the hip anchor pushed +ext along coxa x."""
    tibia_deg = hp.STANCE_TIBIA_DEG if tibia_deg is None else tibia_deg
    a = 0.5 * np.pi / 3.0
    edge = np.array([rv.APOTHEM * np.cos(a), rv.APOTHEM * np.sin(a),
                     hp.CHASSIS_YAW_OUTPUT_Z])
    p = np.deg2rad(femur_deg)
    pt = np.deg2rad(femur_deg + tibia_deg)
    hip_local = np.array(rv.COXA_HIP_ANCHOR_V) + np.array([ext, 0.0, 0.0])
    knee_local = hip_local + rotation_matrix(p, [0, 1, 0])[:3, :3] \
        @ np.array([hp.FEMUR_LENGTH, 0.0, 0.0])
    T_coxa = _trans(edge) @ _rotz(a) @ _rotz(np.deg2rad(yaw_deg))
    T_femur = T_coxa @ _trans(hip_local) @ rotation_matrix(p, [0, 1, 0])
    T_tibia = T_coxa @ _trans(knee_local) @ rotation_matrix(pt, [0, 1, 0])
    return {"femur": T_femur, "tibia": T_tibia,
            "knee_cap": T_femur @ rv.M_KNEE_JP}


def main() -> None:
    meshes = cv.build_meshes()
    plate, hatch = meshes["chassis_top_rigid"], meshes["top_hatch_rigid"]

    # -- A: plain clamp plan raster (clamp local frame = femur @ MH) ------
    xs = np.arange(-1.0, 58.6, 1.0)
    ys = np.arange(-20.0, 13.1, 1.0)
    zs = [cv.ARM_BOT_Z0 + 0.2, cv.ARM_BOT_Z1 - 0.2,
          cv.ARM_TOP_Z0 + 0.2, cv.ARM_TOP_Z1 - 0.2]
    grid = np.array([(x, y, z) for x in xs for y in ys for z in zs
                     if np.hypot(x - cv.AXIS_X, y) > 6.0])

    # -- B: knee-side production/variant meshes ---------------------------
    tube = meshes["tibia_tube_ovh"]
    kside = [("femur_link", meshes["femur_link"], "femur_MH"),
             ("knee_clamp_cap", meshes["knee_clamp_cap"], "knee_cap"),
             ("tibia_ovh_socket", meshes["tibia_ovh_socket"], "tibia_MH"),
             ("tibia_tube_ovh", tube, "tibia"),
             ("foot_boot", meshes["foot_boot"], None)]
    samples = {}
    for name, m, _fr in kside:
        pts = m.vertices
        extra, _ = trimesh.sample.sample_surface(m, 1500)
        samples[name] = np.vstack([pts, extra])

    pitches = np.arange(-95.0, -117.6, -2.5)
    yaws = (-35.0, -20.0, 0.0, 20.0, 35.0)
    for ext in (0.0, 5.0, 8.0, 10.0, 12.0, 15.0, 18.0):
        worstA = None   # shallowest contacting pitch, plan raster
        worstB = {}     # per part
        for pitch in pitches:
            for yaw in yaws:
                T = leg_T(ext, yaw, float(pitch))
                P = trimesh.transform_points(grid, T["femur"] @ cv.MH)
                if (plate.contains(P) | hatch.contains(P)).any():
                    if worstA is None or pitch > worstA:
                        worstA = pitch
                for name, m, fr in kside:
                    if fr == "femur_MH":
                        M = T["femur"] @ cv.MH
                    elif fr == "tibia_MH":
                        M = T["tibia"] @ cv.MH
                    elif fr == "knee_cap":
                        M = T["knee_cap"]
                    elif fr == "tibia":
                        M = T["tibia"]
                    else:
                        continue
                    Q = trimesh.transform_points(samples[name], M)
                    if (plate.contains(Q) | hatch.contains(Q)).any():
                        prev = worstB.get(name)
                        if prev is None or pitch > prev:
                            worstB[name] = pitch
        parts = ", ".join(f"{k} {v:+.1f}" for k, v in sorted(worstB.items()))
        print(f"ext {ext:5.1f}: plain-clamp first contact "
              f"{worstA if worstA is not None else 'CLEAR thru -117.5'}"
              f"  |  knee-side: {parts if parts else 'CLEAR'}")


if __name__ == "__main__":
    main()
