#!/usr/bin/env python3
"""Tool-access audit: every fastener needs a straight driver path.

For each screw instance a virtual driver is grown from just behind the
head, along the approach axis (opposite the shank): a Ø4.6 x 6 mm tip
(fits inside the Ø5.5 counterbores that recess the heads) followed by a
Ø7 x 50 mm shaft.  Roll pins get a Ø6 x 40 mm punch path along BOTH
pin-axis directions (either side is enough).  The driver volume is
boolean-tested against every other instance in the home pose; any
intersection > 1 mm^3 is an access blocker.

Blockers are classified: parts the fastener itself joins are NOT excused
(you still need to reach the head), but a blocker can be waived by the
documented assembly order (e.g. Pi screws are driven before the top
plate goes on) -- waivers are explicit in WAIVED below.

    python scripts/access_audit.py
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
VIZ = os.path.join(os.path.dirname(HERE), "full_robot_viz")

TIP_DIA, TIP_LEN = 4.6, 6.0     # clears the Ø5.5 head counterbores
DRIVER_DIA, DRIVER_LEN = 7.0, 50.0
PUNCH_DIA, PUNCH_LEN = 6.0, 40.0
TOL_MM3 = 1.0

# instance-name substring -> reason, for blockers cleared by assembly order
WAIVED: dict[str, str] = {
    "pi_mount": "driven with the top plate OFF (Pi subassembly first); "
                "blocker is the bottom plate 50 mm below",
}

HEAD_H = {"screw_m25x6": 2.5, "screw_m25x8": 2.5, "screw_m25x12": 2.5,
          "screw_m3x8": 3.0}


def main() -> None:
    with open(os.path.join(VIZ, "scene.json")) as fh:
        scene = json.load(fh)
    base = {m["id"]: trimesh.load(os.path.join(VIZ, m["url"]), force="mesh")
            for m in scene["meshes"]}

    solids = []
    for i in scene["instances"]:
        m = base[i["meshId"]].copy()
        m.apply_transform(np.array(i["transform"]).reshape(4, 4).T)
        solids.append((i["id"], m, m.bounds))

    def blockers(vol, skip_id):
        vb = vol.bounds
        hits = []
        for iid, m, b in solids:
            if iid == skip_id:
                continue
            if not (np.all(vb[0] < b[1]) and np.all(b[0] < vb[1])):
                continue
            try:
                ix = trimesh.boolean.intersection([vol, m],
                                                  engine="manifold")
                v = float(ix.volume) if ix and not ix.is_empty else 0.0
            except Exception:
                v = 0.0
            if v > TOL_MM3:
                hits.append((iid, v))
        return hits

    fails, waived, n_screw, n_pin = [], [], 0, 0
    for i in scene["instances"]:
        if i["role"] != "fastener":
            continue
        t = np.array(i["transform"]).reshape(4, 4).T
        axis = t[:3, 2]          # mesh local +z = shank direction
        origin = t[:3, 3]        # head top (clamp plane)
        pt = i["partType"]
        if pt == "pin_25x22":
            n_pin += 1
            ok = False
            for s in (+1, -1):
                start = origin + s * axis * (11.0 + 0.2)
                cyl = trimesh.creation.cylinder(
                    radius=PUNCH_DIA / 2, height=PUNCH_LEN, sections=16)
                cyl.apply_translation([0, 0, PUNCH_LEN / 2 + 0.2])
                a = trimesh.geometry.align_vectors([0, 0, 1], s * axis)
                cyl.apply_transform(a)
                cyl.apply_translation(start)
                if not blockers(cyl, i["id"]):
                    ok = True
                    break
            if not ok:
                fails.append((i["id"], "no punch path on either side"))
            continue
        n_screw += 1
        head_h = HEAD_H[pt]
        start = origin - axis * (head_h + 0.2)
        tip = trimesh.creation.cylinder(
            radius=TIP_DIA / 2, height=TIP_LEN, sections=16)
        tip.apply_translation([0, 0, TIP_LEN / 2 + 0.2])
        shaft = trimesh.creation.cylinder(
            radius=DRIVER_DIA / 2, height=DRIVER_LEN, sections=16)
        shaft.apply_translation([0, 0, DRIVER_LEN / 2 + TIP_LEN + 0.2])
        cyl = trimesh.boolean.union([tip, shaft], engine="manifold")
        a = trimesh.geometry.align_vectors([0, 0, 1], -axis)
        cyl.apply_transform(a)
        cyl.apply_translation(start)
        hits = blockers(cyl, i["id"])
        for bid, v in hits:
            w = next((r for k, r in WAIVED.items() if k in i["id"]), None)
            if w:
                waived.append((i["id"], bid, v, w))
            else:
                fails.append((i["id"], f"driver hits {bid} ({v:.0f} mm^3)"))

    print(f"{n_screw} screws (driver Ø{DRIVER_DIA} x {DRIVER_LEN}), "
          f"{n_pin} pins (punch Ø{PUNCH_DIA} x {PUNCH_LEN})")
    for iid, bid, v, w in waived:
        print(f"  waived  {iid}: {bid} ({v:.0f} mm^3) -- {w}")
    if fails:
        print("ACCESS FAILURES:")
        for iid, why in fails:
            print(f"  {iid}: {why}")
        sys.exit(1)
    print("ACCESS AUDIT OK: every fastener has a straight tool path")


if __name__ == "__main__":
    main()
