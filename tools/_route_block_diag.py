"""Throwaway diagnostic: for the published scene.json, sample every route
polyline (world frame, rest pose) and report WHICH instances it passes
through (excluding the route's declared terminal/pass-through instances
and any instance a waypoint is anchored to).  Complements the BuildViz
routing_reach gate, which reports only a blocked count.

Run:  .venv python tools/_route_block_diag.py [route-id-substring]
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
VIZ = os.path.join(HERE, "..", "full_robot_viz")

scene = json.load(open(os.path.join(VIZ, "scene.json")))
want = sys.argv[1] if len(sys.argv) > 1 else ""

meshes = {}


def mesh_for(inst):
    key = inst["mesh"] if "mesh" in inst else inst["partType"]
    if key not in meshes:
        path = os.path.join(VIZ, "stl", f"{key}.stl")
        meshes[key] = trimesh.load(path) if os.path.exists(path) else None
    return meshes[key]


inst_by_id = {i["id"]: i for i in scene["instances"]}
placed = []
for inst in scene["instances"]:
    m = mesh_for(inst)
    if m is None:
        continue
    mm = m.copy()
    mm.apply_transform(np.array(inst["transform"]).reshape(4, 4).T)
    placed.append((inst["id"], inst["partType"], mm))


def route_points(route):
    if "points" in route:
        return [np.array(p, float) for p in route["points"]]
    pts = []
    for wp in route["waypoints"]:
        M = np.array(inst_by_id[wp["instanceId"]]["transform"]).reshape(4, 4).T
        pts.append((M @ np.array([*wp["local"], 1.0]))[:3])
    return pts


def sample(poly, step=1.5, trim=4.0):
    segs = []
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(2, int(np.linalg.norm(b - a) / step))
        t = np.linspace(0.0, 1.0, n, endpoint=False)[:, None]
        segs.append(a + t * (b - a))
    segs.append(poly[-1][None, :])
    pts = np.vstack(segs)
    d = np.cumsum(np.r_[0.0, np.linalg.norm(np.diff(pts, axis=0), axis=1)])
    keep = (d > trim) & (d < d[-1] - trim)
    return pts[keep] if keep.any() else pts


for route in scene.get("routes", []):
    if want and want not in route["id"]:
        continue
    exempt = set(route.get("instances", []))
    for wp in route.get("waypoints", []):
        exempt.add(wp["instanceId"])
    poly = route_points(route)
    pts = sample(poly)
    r = float(route.get("radiusMm", 0.5))
    hits = []
    for iid, ptype, m in placed:
        if iid in exempt:
            continue
        lo, hi = m.bounds
        near = np.all((pts > lo - 2) & (pts < hi + 2), axis=1)
        if not near.any():
            continue
        d = trimesh.proximity.signed_distance(m, pts[near])
        pen = d + r
        if pen.max() > 0.05:
            at = pts[near][int(pen.argmax())]
            hits.append(f"    {iid} ({ptype}): {pen.max():5.2f} mm at "
                        f"({at[0]:6.1f},{at[1]:6.1f},{at[2]:6.1f})")
    if hits:
        print(f"{route['id']}:")
        print("\n".join(hits))
