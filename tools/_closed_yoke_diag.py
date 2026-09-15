#!/usr/bin/env python3
"""Scratch diagnostic for the CLOSED-YOKE test variant (Aug 2026).

Question: where can material be added to _sandwich_moving_yoke to close the
open C-channel (tie the two clevis arms together) WITHOUT colliding with the
fixed side (cradle + servo + clamp cap) or the incoming limb (femur spar /
coxa arm) anywhere in the joint's ROM?

Method: express the FIXED side in the MOVING yoke's joint-local frame as a
function of joint angle (the yoke frame is the natural place to design yoke
material), sample the fixed meshes' surfaces, sweep the ROM, and report the
swept occupancy in cylindrical coords about the pivot axis -- plus a direct
point-in-slab test for candidate side-wall boxes.
"""
from __future__ import annotations


import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

import hexapod_prototype as HP


def _trans(v):
    M = np.eye(4)
    M[:3, 3] = np.asarray(v, dtype=float)
    return M


def rel_fixed_in_yoke(theta_deg: float) -> np.ndarray:
    """Transform placing the KNEE fixed-side (well/joint-local frame) into
    the tibia yoke's joint-local frame at knee angle theta (deg).

    Mirrors the viz chain:  fixed_world = T(knee_local) @ rotY(p) @ R0,
    yoke_world = T(knee_local) @ rotY(p + theta) @ R0
    => rel = R0^-1 @ rotY(-theta) @ R0.
    """
    xz = (1, 0, 0), HP.LEG_PITCH_AXIS
    R0 = HP._joint_place((0.0, 0.0, 0.0), *xz)
    return np.linalg.inv(R0) @ rotation_matrix(
        np.deg2rad(-theta_deg), [0, 1, 0]) @ R0


def _sample(meshes_counts):
    pts = []
    for m, n in meshes_counts:
        pts.append(np.vstack([m.vertices,
                              trimesh.sample.sample_surface(m, n)[0]]))
    pts = np.vstack(pts)
    piv = np.array([HP.SERVO_OUTPUT_X, 0.0])
    return pts[np.linalg.norm(pts[:, :2] - piv, axis=1) < 90.0]


def _sweep(pts0, rom_deg):
    rom = np.arange(rom_deg[0] - 5.0, rom_deg[1] + 5.0 + 1e-9, 2.0)
    out = []
    for th in rom:
        M = rel_fixed_in_yoke(th)
        out.append((M[:3, :3] @ pts0.T).T + M[:3, 3])
    return np.vstack(out)


def _report(name, swept):
    piv = np.array([HP.SERVO_OUTPUT_X, 0.0])
    x, y, z = swept.T
    r = np.hypot(x - piv[0], y - piv[1])
    phi = np.rad2deg(np.arctan2(y - piv[1], x - piv[0]))  # 0 = +X outboard
    zb, zt = HP._YOKE_BOT_Z0, HP._YOKE_TOP_Z1
    inband = (z > zb + 0.5) & (z < zt - 0.5)
    print(f"\n[{name}] swept occupancy, z in ({zb:.1f},{zt:.1f}):")
    for lo in range(-180, 180, 15):
        m = inband & (phi >= lo) & (phi < lo + 15)
        if m.any():
            print(f"  phi [{lo:+4d},{lo+15:+4d}) deg: r max {r[m].max():6.1f}"
                  f"  n={m.sum()}")
        else:
            print(f"  phi [{lo:+4d},{lo+15:+4d}) deg: EMPTY")

    # Drum test: partial cylindrical shell about the pivot, phi in
    # [phi0, phi1], r in [r_in, r_out], z full yoke height.  Min gap =
    # distance from every swept point to the drum solid (0 = inside).
    print(f"[{name}] drum candidates (min gap to swept pts, mm):")
    for phi0, phi1 in ((-50.0, 50.0), (-55.0, 55.0), (-60.0, 60.0),
                       (-45.0, 55.0), (-40.0, 60.0), (-35.0, 65.0),
                       (-45.0, 45.0)):
        for r_in in (31.0, 32.0, 33.0, 34.0, 35.0, 36.0):
            r_out = r_in + 3.5
            inphi = (phi >= phi0 - 6.0) & (phi <= phi1 + 6.0)
            # conservative: inside the padded phi window only radial/z
            # escape counts.
            dr = np.maximum(np.maximum(r_in - r, r - r_out), 0.0)
            dz = np.maximum(np.maximum(zb - z, z - zt), 0.0)
            d = np.sqrt(dr**2 + dz**2)
            d = np.where(inphi, d, np.inf)
            i = int(np.argmin(d))
            culprit = (f"  worst pt x{x[i]:7.1f} y{y[i]:7.1f} z{z[i]:6.1f} "
                       f"(r{r[i]:5.1f} phi{phi[i]:+7.1f})") \
                if d[i] < 2.5 else ""
            print(f"    phi [{phi0:+5.1f},{phi1:+5.1f}] "
                  f"r [{r_in:.1f},{r_out:.1f}]: min gap {d[i]:6.2f}{culprit}")


def _mesh_gap(name, closure_pts, swept):
    """Min distance between the closure solids' surface samples and the
    swept fixed-side point cloud (KD-tree; both clouds ~1 mm dense, so the
    reported gap is accurate to ~+/-0.5 mm; 0 ~= contact/penetration)."""
    from scipy.spatial import cKDTree
    d, idx = cKDTree(swept).query(closure_pts, k=1)
    i = int(np.argmin(d))
    piv = np.array([HP.SERVO_OUTPUT_X, 0.0])
    cp, op = closure_pts[i], swept[idx[i]]
    print(f"[{name}] closure-mesh min gap to swept fixed pts: "
          f"{d.min():.2f} mm")
    for tag, p in (("closure", cp), ("fixed  ", op)):
        r = np.hypot(p[0] - piv[0], p[1] - piv[1])
        phi = np.rad2deg(np.arctan2(p[1] - piv[1], p[0] - piv[0]))
        print(f"    {tag} pt x{p[0]:7.1f} y{p[1]:7.1f} z{p[2]:6.1f} "
              f"(r{r:5.1f} phi{phi:+7.1f})")


def main():
    ang, axis, point = trimesh.transformations.rotation_from_matrix(
        rel_fixed_in_yoke(30.0))
    print(f"rel(30 deg): angle {np.rad2deg(ang):+.2f} deg  "
          f"axis {np.round(axis, 4)}  point {np.round(point, 3)}")

    print("building meshes ...")
    # "Closure" = the ADDED material of the current reinforcement variant
    # (boolean delta reinforced - stock), so pre-existing tight spots of the
    # stock yoke (e.g. the spine's 0.39 mm cap graze) don't mask new ones.
    reinf = HP._sandwich_moving_yoke(tube_socket=True, reinforced=True)
    stock = HP._sandwich_moving_yoke(tube_socket=True, reinforced=False)
    closure = HP._diff(reinf, stock)
    closure_pts = np.vstack([
        closure.vertices, trimesh.sample.sample_surface(closure, 60000)[0]])
    servo = HP._servo_envelope()
    cap = HP.make_servo_clamp_cap()

    # ---- KNEE joint: fixed side = femur_link (cradle fused) + servo + cap.
    femur = HP.make_femur_link_part()
    femur.apply_translation([-HP.FEMUR_LENGTH, 0.0, 0.0])  # knee-local
    pts = _sample([(femur, 12000), (servo, 4000), (cap, 3000)])
    print(f"knee: {len(pts)} obstacle samples")
    swept = _sweep(pts, (-20.0, 80.0))
    _report("KNEE rom -20..80", swept)
    _mesh_gap("KNEE", closure_pts, swept)

    # ---- HIP joint: fixed side = coxa_link + servo + cap, in hip-local.
    coxa = HP.make_coxa_link_part()
    M = HP._joint_place(HP.COXA_HIP_ANCHOR, (1, 0, 0), HP.LEG_PITCH_AXIS)
    coxa.apply_transform(np.linalg.inv(M))
    print("coxa-in-hip-local bounds:", np.round(coxa.bounds, 1).tolist())
    pts = _sample([(coxa, 12000), (servo, 4000), (cap, 3000)])
    print(f"hip: {len(pts)} obstacle samples")
    swept = _sweep(pts, (-80.0, 30.0))
    _report("HIP rom -80..30", swept)
    _mesh_gap("HIP", closure_pts, swept)


if __name__ == "__main__":
    main()
