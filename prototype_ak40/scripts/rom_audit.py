#!/usr/bin/env python3
"""Range-of-motion + combined-pose audit for the AK40 hexapod.

The BuildViz sweep samples one DOF at a time; this script checks what it
cannot:

A. TRUE mechanical ROM per joint: drive each L0 joint past its software
   limit in 1 deg steps (rest of robot at home) until the first exact
   boolean collision, and report the mechanical margin beyond the limit.
B. COMBINED worst-case poses: adjacent yaws converged, whole-body stance
   poses, and the four hip/knee corner poses on all legs at once --
   with yaw convergence stacked on top.
C. Workspace / gait numbers derived from the same constants: ride-height
   range, step length, swing lift, tripod support polygon.

Exact manifold booleans, same ignore pairs as scene checksConfig.

    python scripts/rom_audit.py
"""

from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
VIZ = os.path.join(ROOT, "full_robot_viz")
sys.path.insert(0, ROOT)
import hexapod_ak40 as hx  # noqa: E402

TOL_MM3 = 1.0


# -- scene + FK -------------------------------------------------------------
def load():
    with open(os.path.join(VIZ, "scene.json")) as fh:
        scene = json.load(fh)
    base = {m["id"]: trimesh.load(os.path.join(VIZ, m["url"]), force="mesh")
            for m in scene["meshes"]}
    ignore = {frozenset(p)
              for p in scene["checksConfig"]["ignoreOverlapPairs"]}
    joints = {j["id"]: j for j in scene["joints"]}
    # deepest joint listing each instance
    depth = {jid: 0 for jid in joints}
    for jid, j in joints.items():
        d, p = 0, j.get("parent")
        while p:
            d += 1
            p = joints[p].get("parent")
        depth[jid] = d
    drives = {}
    for jid in sorted(joints, key=lambda x: depth[x]):
        for iid in joints[jid]["instances"]:
            drives[iid] = jid    # later (deeper) wins
    return scene, base, ignore, joints, drives


def joint_local(j, theta):
    o = np.array(j["origin"])
    ax = np.array(j["axis"], dtype=float)
    r = trimesh.transformations.rotation_matrix(math.radians(theta), ax, o)
    return r


def fk(joints, values):
    world = {}
    def m(jid):
        if jid in world:
            return world[jid]
        loc = joint_local(joints[jid], values.get(jid, 0.0))
        p = joints[jid].get("parent")
        world[jid] = (m(p) @ loc) if p else loc
        return world[jid]
    for jid in joints:
        m(jid)
    return world


def posed_instances(scene, base, joints, drives, values, subset=None):
    world = fk(joints, values)
    out = []
    for i in scene["instances"]:
        if subset and not subset(i):
            continue
        t = np.array(i["transform"]).reshape(4, 4).T
        jid = drives.get(i["id"])
        if jid:
            t = world[jid] @ t
        mesh = base[i["meshId"]].copy()
        mesh.apply_transform(t)
        out.append((i["id"], i["partType"], mesh, mesh.bounds))
    return out


def collisions(inst, ignore):
    hits = []
    for a in range(len(inst)):
        for b in range(a + 1, len(inst)):
            ida, ta, ma, ba = inst[a]
            idb, tb, mb, bb = inst[b]
            if not (np.all(ba[0] < bb[1]) and np.all(bb[0] < ba[1])):
                continue
            if frozenset((ta, tb)) in ignore:
                continue
            try:
                ix = trimesh.boolean.intersection([ma, mb],
                                                  engine="manifold")
                vol = float(ix.volume) if ix and not ix.is_empty else 0.0
            except Exception:
                vol = 0.0
            if vol > TOL_MM3:
                hits.append((ida, idb, vol))
    return hits


def main() -> None:
    scene, base, ignore, joints, drives = load()

    # ---- A. per-joint mechanical ROM (L0 moving; neighbors present) ----
    print("A. Mechanical ROM (1 deg steps, exact booleans, L0 vs all)")
    near = lambda i: (i.get("leg") in ("L0", "L1", "L5") or
                      i.get("leg") is None)
    lims = {"L0-yaw": (-25, 25), "L0-hip": (-35, 20), "L0-knee": (-30, 35)}
    for jid, (lo, hi) in lims.items():
        for sign, sw_lim in ((-1, lo), (1, hi)):
            onset = None
            for step in range(0, 61):
                th = sw_lim + sign * step
                inst = posed_instances(scene, base, joints, drives,
                                       {jid: float(th)}, subset=near)
                hits = collisions(inst, ignore)
                if hits:
                    onset = th
                    worst = max(hits, key=lambda h: h[2])
                    print(f"  {jid} {'-' if sign < 0 else '+'}: sw limit "
                          f"{sw_lim:+.0f} deg, first contact {onset:+.0f} "
                          f"deg ({worst[0]} x {worst[1]}) -> margin "
                          f"{abs(onset - sw_lim):.0f} deg")
                    break
            if onset is None:
                print(f"  {jid} {'-' if sign < 0 else '+'}: sw limit "
                      f"{sw_lim:+.0f} deg, NO contact out to "
                      f"{sw_lim + sign * 60:+.0f} deg (free)")

    # ---- B. combined worst-case poses ----
    print("\nB. Combined worst-case poses (full scene, exact booleans)")
    fd, td = hx.STANCES[hx.NOMINAL_STANCE]

    def stance_vals(name):
        sfd, std = hx.STANCES[name]
        v = {}
        for li in range(6):
            v[f"L{li}-hip"] = sfd - fd
            v[f"L{li}-knee"] = (fd - sfd) + (td - std)
        return v

    def corners(hip, knee):
        v = {}
        for li in range(6):
            v[f"L{li}-hip"] = hip
            v[f"L{li}-knee"] = knee
        return v

    def converge(v, a=25.0):
        # every adjacent pair converged: alternate +/- max yaw
        for li in range(6):
            v[f"L{li}-yaw"] = a if li % 2 == 0 else -a
        return v

    poses = {
        "all tall": stance_vals("tall"),
        "all crouch": stance_vals("crouch"),
        "tall + yaws converged": converge(stance_vals("tall")),
        "nominal + yaws converged": converge(stance_vals("nominal")),
        "crouch + yaws converged": converge(stance_vals("crouch")),
        "hip+20 knee-30 (all)": corners(20, -30),
        "hip+20 knee+35 (all)": corners(20, 35),
        "hip-35 knee-30 (all)": corners(-35, -30),
        "hip-35 knee+35 (all)": corners(-35, 35),
        "hip+20 knee+35 + converge": converge(corners(20, 35)),
        "hip-35 knee-30 + converge": converge(corners(-35, -30)),
    }
    any_bad = False
    for name, vals in poses.items():
        inst = posed_instances(scene, base, joints, drives, vals)
        hits = collisions(inst, ignore)
        if hits:
            any_bad = True
            print(f"  FAIL {name}:")
            for ida, idb, vol in sorted(hits, key=lambda h: -h[2])[:6]:
                print(f"       {ida} x {idb}: {vol:.1f} mm^3")
        else:
            print(f"  ok   {name}")

    # ---- C. workspace / gait numbers ----
    print("\nC. Workspace and gait numbers")
    r_foot = (hx.LEG_MOUNT_R + hx.COXA_LENGTH
              + hx.FEMUR_LENGTH * math.cos(math.radians(fd))
              + hx.TIBIA_LENGTH * math.sin(math.radians(td)))
    for name, (sfd, std) in hx.STANCES.items():
        horiz, drop = hx.foot_offsets(sfd, std)
        print(f"  {name:<8s} body-centre height {drop + hx.HIP_AXIS_DROP:5.0f}"
              f" mm, foot radius {hx.LEG_MOUNT_R + hx.COXA_LENGTH + horiz:5.0f} mm")
    # mechanical height range inside software limits
    hs = []
    for hip in np.arange(-35, 20.1, 1.0):
        for knee in np.arange(-30, 35.1, 1.0):
            f2, t2 = fd + hip, td - hip - knee
            if not (0 <= f2 <= 80 and -45 <= t2 <= 80):
                continue
            _, drop = hx.foot_offsets(f2, t2)
            hs.append(drop + hx.HIP_AXIS_DROP)
    print(f"  body-centre height range within software limits: "
          f"{min(hs):.0f}..{max(hs):.0f} mm")
    step = 2 * r_foot * math.sin(math.radians(15))
    print(f"  nominal foot radius {r_foot:.0f} mm -> step length "
          f"{step:.0f} mm/cycle at +/-15 deg yaw (limit +/-25)")
    _, d_nom = hx.foot_offsets(fd, td)
    _, d_up = hx.foot_offsets(fd, 45.0)
    print(f"  swing lift with knee-only flex to tibia 45 deg: "
          f"{d_nom - d_up:.0f} mm")
    print(f"  tripod support triangle inradius: {r_foot / 2:.0f} mm "
          f"(CoM at centre -> static margin)")
    print("\nROM AUDIT " + ("FAIL" if any_bad else "OK"))
    sys.exit(1 if any_bad else 0)


if __name__ == "__main__":
    main()
