"""Census of EVERYTHING standing above the bare sheet top (z 2) on a
chassis_bottom mesh, judged against the rev-8 ABOVE-SHEET WHITELIST
(see the CHB_WL_* block in make_rigid_hip_variant.py): the six yaw
towers are the whole whitelist -- the deck-plateau category measured
degenerate (the functional deck lives inside the tower keep cylinder)
and the mates category measured empty (pillars bolt through the sheet,
retainer bolts from below, wago block is taped).

Usage:  uv run python probe_above_sheet.py [path/to/chassis.stl]

Two reports:
  * the shared vertex census (chassis_whitelist_violations -- exactly
    what check_chassis_variant asserts on every build);
  * plane cross-sections at several heights, one line per solid loop
    with its leg-frame bbox + radial span from the nearest yaw axis,
    so any offender can be matched to its code origin.

Exits 1 if the whitelist is violated.  Historical note: run against
the pre-rev-8 STEP sidecar STL this reported 120 offending vertices
(worst r 35.13) in 12 wall clusters, two per corner flat -- the stale
two-bay WAGO3 tray side walls; the mesh pipeline was already clean.
"""
from __future__ import annotations

import sys

import numpy as np
import trimesh

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import make_rigid_hip_variant as mk  # noqa: E402

AX = mk.APOTHEM  # 100 -- yaw axis radial position


def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else \
        __file__.rsplit("/", 1)[0] + "/stl/chassis_bottom_rigid.stl"
    m = trimesh.load(path)
    print(f"mesh: {path}\n  {len(m.vertices)} verts, "
          f"watertight={m.is_watertight}")

    n_bad, worst_r = mk.chassis_whitelist_violations(m)
    print(f"whitelist census: {n_bad} vertices above z 2.05 outside the "
          f"tower cylinders" + (f" (worst r {worst_r:.2f})" if n_bad else ""))

    for z in (2.5, 4.0, 6.5, 9.5, 12.5, 16.0):
        sec = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        if sec is None:
            print(f"z={z}: no material")
            continue
        loops = []
        for ent in sec.discrete:      # world-frame polylines, no to_2D skew
            pts = np.asarray(ent)[:, :2]
            d = np.full(len(pts), np.inf)
            best_i = 0
            for i in range(6):
                a = (i + 0.5) * np.pi / 3.0
                di = np.hypot(pts[:, 0] - AX * np.cos(a),
                              pts[:, 1] - AX * np.sin(a))
                if di.mean() < d.mean():
                    d, best_i = di, i
            lp = trimesh.transform_points(
                np.column_stack([pts, np.zeros(len(pts))]),
                mk._rotz(-(best_i + 0.5) * np.pi / 3.0))
            tag = ("TOWER" if d.max() <= mk.CHB_TRIM_R + mk.CHB_WL_TOL
                   else "OUTSIDE")
            loops.append((tag, best_i, lp, d))
        print(f"z={z}: {len(loops)} loops")
        for tag, i, lp, d in loops:
            print(f"  {tag} leg{i} x[{lp[:,0].min():7.2f},{lp[:,0].max():7.2f}]"
                  f" y[{lp[:,1].min():7.2f},{lp[:,1].max():7.2f}]"
                  f" r[{d.min():6.2f},{d.max():6.2f}]")
    sys.exit(1 if n_bad else 0)


if __name__ == "__main__":
    main()
