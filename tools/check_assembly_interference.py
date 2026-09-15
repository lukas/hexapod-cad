"""Interference check for the full_robot_viz scene (real printed parts).

Loads full_robot_viz/scene.json, picks one leg (the six are rotationally
symmetric), and estimates the OVERLAP VOLUME of every AABB-overlapping
pair of that leg's parts by voxel sampling (robust for non-watertight
meshes).  Designed-contact pairs (link<->link across a joint, servo in its
cradle, horn on its servo) are flagged EXPECTED; anything else above the
threshold is a real pass-through.

Run:
    ./run.sh hexapod_walker/prototype_sts3215/tools/check_assembly_interference.py [--leg 4]
"""

from __future__ import annotations

import argparse
import json
from itertools import combinations
from pathlib import Path

import numpy as np
import trimesh

_HERE = Path(__file__).resolve().parent
VIZ_DIR = _HERE.parent / "full_robot_viz"


def _norm(part: str) -> str:
    return part


# Pairs that legitimately share volume in a correct assembly.
EXPECTED = {
    frozenset({"coxa_link", "femur_link"}),
    frozenset({"femur_link", "tibia_link"}),
    frozenset({"coxa_link", "yaw_servo"}),
    frozenset({"coxa_link", "yaw_horn"}),
    frozenset({"coxa_link", "hip_servo"}),
    frozenset({"coxa_link", "hip_horn"}),
    frozenset({"yaw_servo", "yaw_horn"}),
    frozenset({"hip_servo", "hip_horn"}),
    frozenset({"hip_horn", "femur_link"}),
    frozenset({"femur_link", "knee_servo"}),
    frozenset({"femur_link", "knee_horn"}),
    frozenset({"knee_servo", "knee_horn"}),
    frozenset({"knee_horn", "tibia_link"}),
    frozenset({"tibia_link", "foot"}),
}


def _aabb_overlap(a: trimesh.Trimesh, b: trimesh.Trimesh) -> bool:
    amin, amax = a.bounds
    bmin, bmax = b.bounds
    return bool(np.all(amax >= bmin) and np.all(bmax >= amin))


def _overlap_volume(a: trimesh.Trimesh, b: trimesh.Trimesh, pitch: float = 1.0) -> float:
    """Voxel-sample the AABB intersection; count centres inside both."""
    lo = np.maximum(a.bounds[0], b.bounds[0])
    hi = np.minimum(a.bounds[1], b.bounds[1])
    if np.any(hi <= lo):
        return 0.0
    n = np.maximum(2, np.ceil((hi - lo) / pitch).astype(int))
    gx = np.linspace(lo[0], hi[0], int(n[0]))
    gy = np.linspace(lo[1], hi[1], int(n[1]))
    gz = np.linspace(lo[2], hi[2], int(n[2]))
    XX, YY, ZZ = np.meshgrid(gx, gy, gz, indexing="ij")
    pts = np.stack([XX.ravel(), YY.ravel(), ZZ.ravel()], axis=1)
    try:
        in_a = a.contains(pts)
    except Exception:
        return float("nan")
    if not in_a.any():
        return 0.0
    pts_a = pts[in_a]
    try:
        in_b = b.contains(pts_a)
    except Exception:
        return float("nan")
    return float(int(in_b.sum()) * pitch ** 3)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--leg", type=int, default=4)
    parser.add_argument("--min-vol", type=float, default=20.0)
    parser.add_argument("--pitch", type=float, default=2.0,
                        help="Voxel sampling pitch (mm); coarser = faster.")
    parser.add_argument("--only", type=str, default=None,
                        help="Only check pairs involving this partType.")
    args = parser.parse_args(argv)

    scene = json.loads((VIZ_DIR / "scene.json").read_text())
    mesh_url = {m["id"]: m["url"] for m in scene["meshes"]}

    parts: list[tuple[str, trimesh.Trimesh]] = []
    for inst in scene["instances"]:
        if inst.get("leg") != args.leg:
            continue
        mesh = trimesh.load(VIZ_DIR / mesh_url[inst["meshId"]], process=False)
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(list(mesh.geometry.values()))
        parts.append((inst["partType"], mesh))

    print(f"Leg L{args.leg}: {len(parts)} parts. Pairwise overlap "
          f"(>= {args.min_vol:.0f} mm^3):\n")

    findings = []
    for (la, ma), (lb, mb) in combinations(parts, 2):
        if args.only and args.only not in (la, lb):
            continue
        if not _aabb_overlap(ma, mb):
            continue
        vol = _overlap_volume(ma, mb, pitch=args.pitch)
        if vol != vol or vol < args.min_vol:
            continue
        findings.append((vol, la, lb, frozenset({la, lb}) in EXPECTED))

    findings.sort(reverse=True)
    print(f"{'overlap mm^3':>12}  {'status':<14}  pair")
    print("-" * 60)
    for vol, la, lb, expected in findings:
        print(f"{vol:12.1f}  {'EXPECTED' if expected else '>> UNEXPECTED':<14}  "
              f"{la}  <->  {lb}")

    unexpected = [f for f in findings if not f[3]]
    print()
    if unexpected:
        print(f"{len(unexpected)} UNEXPECTED interference(s):")
        for vol, la, lb, _ in unexpected:
            print(f"  - {la} <-> {lb}: {vol:.1f} mm^3")
    else:
        print("No unexpected interferences above threshold. Assembly is clean.")


if __name__ == "__main__":
    main()
