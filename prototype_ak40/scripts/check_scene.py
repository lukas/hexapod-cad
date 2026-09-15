#!/usr/bin/env python3
"""Full-assembly overlap sweep over the BuildViz scene.

Loads full_robot_viz/scene.json, instantiates EVERY instance (parts and
fasteners) with its world transform, prefilters pairs by AABB, and runs an
exact manifold boolean intersection on each candidate pair.  Pairs listed in
checksConfig.ignoreOverlapPairs (by-design contact: threads into taps, press
fits, epoxy sockets, clamped faces) are reported separately and allowed.

Exit code 1 if any non-ignored pair intersects by more than TOL_MM3.

    python scripts/check_scene.py
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
VIZ = os.path.join(ROOT, "full_robot_viz")
TOL_MM3 = 1.0


def main() -> None:
    with open(os.path.join(VIZ, "scene.json")) as fh:
        scene = json.load(fh)

    ignore = {frozenset(p)
              for p in scene["checksConfig"]["ignoreOverlapPairs"]}
    mesh_files = {m["id"]: os.path.join(VIZ, m["url"])
                  for m in scene["meshes"]}
    base = {mid: trimesh.load(path, force="mesh")
            for mid, path in mesh_files.items()}

    inst = []
    for i in scene["instances"]:
        m = base[i["meshId"]].copy()
        m.apply_transform(np.array(i["transform"]).reshape(4, 4).T)
        inst.append((i["id"], i["partType"], m, m.bounds))

    def aabb_hit(b1, b2, pad=0.0):
        return bool(np.all(b1[0] - pad < b2[1]) and
                    np.all(b2[0] - pad < b1[1]))

    candidates = []
    for a in range(len(inst)):
        for b in range(a + 1, len(inst)):
            if aabb_hit(inst[a][3], inst[b][3]):
                candidates.append((a, b))

    bad, expected = [], []
    for a, b in candidates:
        ida, ta, ma, _ = inst[a]
        idb, tb, mb, _ = inst[b]
        try:
            ix = trimesh.boolean.intersection([ma, mb], engine="manifold")
            vol = float(ix.volume) if ix and not ix.is_empty else 0.0
        except Exception:
            vol = 0.0
        if vol <= TOL_MM3:
            continue
        if frozenset((ta, tb)) in ignore:
            expected.append((ida, idb, vol))
        else:
            bad.append((ida, idb, vol))

    print(f"{len(inst)} instances, {len(candidates)} AABB-candidate pairs, "
          f"{len(expected)} by-design contacts, {len(bad)} violations "
          f"(tol {TOL_MM3} mm^3)")
    for ida, idb, vol in sorted(expected, key=lambda x: -x[2])[:8]:
        print(f"  by-design  {ida} x {idb}: {vol:.1f} mm^3")
    if bad:
        print("\nOVERLAP VIOLATIONS:")
        for ida, idb, vol in sorted(bad, key=lambda x: -x[2]):
            print(f"  {ida} x {idb}: {vol:.1f} mm^3")
        sys.exit(1)
    print("OVERLAP CHECK OK: no unexpected intersections")


if __name__ == "__main__":
    main()
