"""Throwaway diagnostic: sample the leg-0 intra-leg wire polylines
(_servo_attach_paths) in WORLD frame and measure penetration into every
leg-0 solid, so wire waypoint fixes are evidence-based.  Prints, per
route chain and per part, the deepest penetration and where it happens.

Run:  .venv python tools/_wire_probe_diag.py [conductor_offset_mm]
"""
from __future__ import annotations

import os
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import full_robot_viz_build as B  # noqa: E402


def world_chains() -> dict[str, np.ndarray]:
    """Reconstruct the world-frame rest polyline of every leg-0 conductor."""
    M0 = B._servo_M0s()
    Mf = next(M for n, _m, M in B._leg0_local_link_parts()
              if n == "femur_link")
    frames = {"yaw": M0["yaw"], "hip": M0["hip"], "knee": M0["knee"],
              "femur": Mf}
    out = {}
    for name, conductors in B._servo_attach_paths().items():
        for suffix, chain in conductors.items():
            pts = []
            for anchor, loc in chain:
                w = frames[anchor] @ np.array([loc[0], loc[1], loc[2], 1.0])
                pts.append(w[:3])
            out[f"{name}.{suffix}"] = np.array(pts)
    return out


def sample(poly: np.ndarray, step: float = 1.0,
           trim_ends_mm: float = 4.0) -> np.ndarray:
    """Sample the polyline, skipping the first/last ``trim_ends_mm`` of arc
    length: chain ends seat INTO recessed connectors (the 5264 port cluster
    sits below the case surface), so endpoint 'penetration' is real,
    intended geometry -- the same reason BuildViz exempts endpoint-anchored
    instances from its obstruction ray test."""
    segs = []
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(2, int(np.linalg.norm(b - a) / step))
        t = np.linspace(0.0, 1.0, n, endpoint=False)[:, None]
        segs.append(a + t * (b - a))
    segs.append(poly[-1:])
    pts = np.vstack(segs)
    d = np.cumsum(np.r_[0.0, np.linalg.norm(np.diff(pts, axis=0), axis=1)])
    keep = (d > trim_ends_mm) & (d < d[-1] - trim_ends_mm)
    return pts[keep] if keep.any() else pts


def main() -> None:
    wire_r = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0

    parts = []
    for name, mesh, M in B._leg0_local_parts():
        m = mesh.copy()
        m.apply_transform(M)
        parts.append((name, m))

    M0 = B._servo_M0s()
    Mf = next(M for n, _m, M in B._leg0_local_link_parts()
              if n == "femur_link")
    invs = {"yaw": np.linalg.inv(M0["yaw"]), "hip": np.linalg.inv(M0["hip"]),
            "knee": np.linalg.inv(M0["knee"]), "femur": np.linalg.inv(Mf),
            "cradle": np.linalg.inv(M0["retainer"])}

    any_hit = False
    for route, poly in world_chains().items():
        pts = sample(poly)
        hits = []
        for pname, m in parts:
            # cheap AABB prefilter
            lo, hi = m.bounds
            near = np.all((pts > lo - 2) & (pts < hi + 2), axis=1)
            if not near.any():
                continue
            d = trimesh.proximity.signed_distance(m, pts[near])
            pen = d + wire_r          # >0 means inside (or within wire radius)
            worst = pen.max()
            if worst > 0.05:
                any_hit = True
                at = pts[near][int(pen.argmax())]
                locs = "  ".join(
                    f"{fr}=({p[0]:6.1f},{p[1]:6.1f},{p[2]:6.1f})"
                    for fr, p in (
                        (fr, (invs[fr] @ np.array([*at, 1.0]))[:3])
                        for fr in ("cradle", "hip", "knee", "femur")))
                hits.append(f"  PIERCES {pname:18s} depth {worst:6.2f} mm\n"
                            f"     {locs}")
        if hits:
            print(f"\n{route}: {len(poly)} waypoints, {len(pts)} samples")
            print("\n".join(hits))
    if not any_hit:
        print("\nAll chains clear of every leg-0 solid.")


if __name__ == "__main__":
    main()
