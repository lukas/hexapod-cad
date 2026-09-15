#!/usr/bin/env python3
"""Generate the BuildViz scene for the AK40 hexapod.

Exports per-part STLs into full_robot_viz/stl/ and writes scene.json with
EVERY physical part AND EVERY fastener as an instance (6 legs x 8 parts +
22 fasteners/leg + chassis + electronics + chassis fasteners = 200
instances), 18 revolute joints (fasteners ride their joints), and the three
named stances as poses.

The generator also builds the ATTACHMENT GRAPH (which fastener/epoxy/press
joint holds which pair of parts) and refuses to write the scene if any part
is not reachable from chassis_bottom.  The graph is written to
artifacts/attachment_report.md.

Geometry comes straight from hexapod_ak40.py -- run after any CAD change:

    python full_robot_viz/make_scene.py
    npx buildviz register full_robot_viz --build-id prototype_ak40
    # (FLAT build id -- the ?build= viewer param can't resolve project/build)
"""

from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import hexapod_ak40 as hx  # noqa: E402

_, NOM_DROP = hx.foot_offsets(*hx.STANCES[hx.NOMINAL_STANCE])
Z0 = hx.HIP_AXIS_DROP + NOM_DROP        # ground at z=0

COLORS = {
    "chassis_bottom": "#79b0e1", "chassis_top": "#a7c7ec",
    "coxa_link": "#4ade80", "femur_link": "#fbbf24", "tibia_yoke": "#f472b6",
    "tibia_tube": "#111827", "foot_boot": "#92400e",
    "actuator": "#374151", "battery": "#ef4444", "pi5": "#22c55e",
    "standoff": "#d1d5db",
    "screw_m25x6": "#52525b", "screw_m25x8": "#3f3f46",
    "screw_m3x8": "#71717a", "screw_m25x12": "#a1a1aa",
    "pin_25x22": "#9ca3af",
}

BOLT3 = hx.BOLT3_DEG
BOLT4 = hx.BOLT4_DEG


def T(mat) -> list[float]:
    return [float(v) for v in np.asarray(mat).T.flatten()]


def rot_z(deg):
    return trimesh.transformations.rotation_matrix(math.radians(deg), [0, 0, 1])


def rot_y(deg):
    return trimesh.transformations.rotation_matrix(math.radians(deg), [0, 1, 0])


def rot_x(deg):
    return trimesh.transformations.rotation_matrix(math.radians(deg), [1, 0, 0])


def trans(x, y, z):
    return trimesh.transformations.translation_matrix([x, y, z])


def screw_mesh(shank_d, length, head_d, head_h) -> trimesh.Trimesh:
    """SHCS mock along +z: head top (clamp plane) at z=0, head below,
    shank z 0..length."""
    head = trimesh.creation.cylinder(radius=head_d / 2, height=head_h,
                                     sections=16)
    head.apply_translation([0, 0, -head_h / 2])
    shank = trimesh.creation.cylinder(radius=shank_d / 2, height=length,
                                      sections=16)
    shank.apply_translation([0, 0, length / 2])
    return trimesh.util.concatenate([head, shank])


def export_meshes(stl_dir: str) -> dict[str, str]:
    os.makedirs(stl_dir, exist_ok=True)
    meshes = {
        "chassis_bottom": hx.make_chassis_bottom(),
        "chassis_top": hx.make_chassis_top(),
        "coxa_link": hx.make_coxa_link(),
        "femur_link": hx.make_femur_link(),
        "tibia_yoke": hx.make_tibia_yoke(),
        "foot_boot": hx.make_foot_boot(),
        "ak40_actuator": hx.make_actuator_mock(),
        "tibia_tube": trimesh.creation.cylinder(
            radius=hx.TIBIA_TUBE_OD / 2, height=hx.TIBIA_TUBE_CUT, sections=32),
        "battery_6s": trimesh.creation.box(extents=[155, 48, 55]),
        "pi5": trimesh.creation.box(extents=[85, 56, 18]),
        "standoff_m3x50": trimesh.creation.cylinder(radius=2.5, height=50.0,
                                                    sections=16),
        "screw_m25x6": screw_mesh(2.5, 6, 4.5, 2.5),
        "screw_m25x8": screw_mesh(2.5, 8, 4.5, 2.5),
        "screw_m3x8": screw_mesh(3.0, 8, 5.5, 3.0),
        "screw_m25x12": screw_mesh(2.5, 12, 4.5, 2.5),
        "pin_25x22": trimesh.creation.cylinder(radius=1.25, height=22,
                                               sections=16),
    }
    for name, m in meshes.items():
        m.export(os.path.join(stl_dir, f"{name}.stl"))
    return {name: f"stl/{name}.stl" for name in meshes}


class Scene:
    def __init__(self):
        self.instances = []
        self.edges = []          # (instance_id_a, instance_id_b, how)
        self._n = 0

    def add(self, mesh, name, part_type, role, mat, *, leg=None, joint=None,
            cots=False):
        iid = f"{self._n:03d}-{name}"
        self._n += 1
        self.instances.append({
            "id": iid, "meshId": f"stl:{mesh}", "name": name,
            "partType": part_type, "role": role, "leg": leg, "joint": joint,
            "cots": cots, "color": COLORS.get(part_type, "#9ca3af"),
            "transform": T(mat),
        })
        return iid

    def screw(self, mesh_type, name, mat, joins, *, leg=None, joint=None):
        """Fastener instance + the attachment edge it creates."""
        iid = self.add(mesh_type, name, mesh_type, "fastener", mat,
                       leg=leg, joint=joint, cots=True)
        self.edges.append((joins[0], joins[1], iid))
        return iid

    def bond(self, a, b, how):
        self.edges.append((a, b, how))


def axis_frame(axis: str):
    """Rotation taking the screw mesh +z to the given world/leg axis."""
    return {"+z": np.eye(4), "-z": rot_x(180), "+y": rot_x(-90),
            "-y": rot_x(90), "+x": rot_y(90), "-x": rot_y(-90)}[axis]


def build_scene() -> dict:
    mesh_urls = export_meshes(os.path.join(HERE, "stl"))
    fd, td = hx.STANCES[hx.NOMINAL_STANCE]
    f = math.radians(fd)
    sc = Scene()

    lift = trans(0, 0, Z0)
    chassis_b = sc.add("chassis_bottom", "chassis_bottom", "chassis_bottom",
                       "chassis", lift)
    chassis_t = sc.add("chassis_top", "chassis_top", "chassis_top",
                       "chassis", lift)
    for k, (gx, gy) in enumerate([(sx, sy)
                                  for sx in (-hx.CHASSIS_STANDOFF_XY[0],
                                             hx.CHASSIS_STANDOFF_XY[0])
                                  for sy in (-hx.CHASSIS_STANDOFF_XY[1],
                                             hx.CHASSIS_STANDOFF_XY[1])]):
        so = sc.add("standoff_m3x50", f"standoff_{k}", "standoff", "frame",
                    lift @ trans(gx, gy, 0.0), cots=True)
        sc.screw("screw_m3x8", f"standoff_{k}_top",
                 lift @ trans(gx, gy, hx.PLATE_T_TOP) @ axis_frame("-z"),
                 (chassis_t, so))
        sc.screw("screw_m3x8", f"standoff_{k}_bot",
                 lift @ trans(gx, gy, hx.PLATE_B_BOT) @ axis_frame("+z"),
                 (chassis_b, so))
    battery = sc.add("battery_6s", "battery_6s", "battery", "electronics",
                     lift @ trans(0, 0, hx.PLATE_B_BOT - 27.5), cots=True)
    sc.bond(chassis_b, battery, "velcro + Dual Lock + cinch strap")
    pi = sc.add("pi5", "raspberry_pi_5", "pi5", "electronics",
                lift @ trans(75, 0, hx.PLATE_T_TOP + 9), cots=True)
    for k, (px, py) in enumerate([(-29.0, -24.5), (-29.0, 24.5),
                                  (29.0, -24.5), (29.0, 24.5)]):
        sc.screw("screw_m25x12", f"pi_mount_{k}",
                 lift @ trans(75 + px, py, hx.PLATE_T_BOT) @ axis_frame("+z"),
                 (chassis_t, pi))

    joints, contact_pairs = [], set()
    for li, az in enumerate(hx.LEG_AZIMUTHS):
        leg = f"L{li}"
        azr = math.radians(az)
        place = lift @ trans(hx.LEG_MOUNT_R * math.cos(azr),
                             hx.LEG_MOUNT_R * math.sin(azr), 0) @ rot_z(az)
        hip_pt = place @ trans(hx.COXA_LENGTH, hx.HIP_OUT_Y, hx.HIP_AXIS_Z)
        knee_x = hx.COXA_LENGTH + hx.FEMUR_LENGTH * math.cos(f)
        knee_z = hx.HIP_AXIS_Z - hx.FEMUR_LENGTH * math.sin(f)
        knee_pt = place @ trans(knee_x, hx.KNEE_OUT_Y, knee_z)
        fem_fr = hip_pt @ rot_y(fd)
        yoke_fr = knee_pt @ rot_y(-td)
        t = math.radians(td)
        tib_dir = np.array([math.sin(t), 0.0, -math.cos(t)])

        yaw_act = sc.add("ak40_actuator", f"{leg}_yaw_act", "actuator",
                         "motor", place @ trans(0, 0, hx.PLATE_B_TOP)
                         @ rot_x(180) @ rot_z(180),
                         leg=leg, joint="yaw", cots=True)
        coxa = sc.add("coxa_link", f"{leg}_coxa", "coxa_link", "leg",
                      place, leg=leg, joint="yaw")
        hip_act = sc.add("ak40_actuator", f"{leg}_hip_act", "actuator",
                         "motor", hip_pt @ rot_x(-90) @ rot_z(180),
                         leg=leg, joint="hip", cots=True)
        femur = sc.add("femur_link", f"{leg}_femur", "femur_link", "leg",
                       fem_fr, leg=leg, joint="hip")
        knee_act = sc.add("ak40_actuator", f"{leg}_knee_act", "actuator",
                          "motor", knee_pt @ rot_x(90) @ rot_z(180),
                          leg=leg, joint="knee", cots=True)
        yoke = sc.add("tibia_yoke", f"{leg}_tibia_yoke", "tibia_yoke",
                      "leg", yoke_fr, leg=leg, joint="knee")
        tube = sc.add("tibia_tube", f"{leg}_tibia_tube", "tibia_tube",
                      "spar", knee_pt @ trans(0, hx.TIBIA_TUBE_Y, 0)
                      @ rot_y(-td) @ trans(0, 0, (hx.TIBIA_TUBE_TOP
                                                  + hx.TIBIA_TUBE_END) / 2),
                      leg=leg, joint="knee", cots=True)
        boot = sc.add("foot_boot", f"{leg}_foot_boot", "foot_boot", "leg",
                      knee_pt @ trans(0, hx.TIBIA_TUBE_Y, 0) @ rot_y(-td)
                      @ trans(0, 0, hx.TIBIA_TUBE_END),
                      leg=leg, joint="knee")
        sc.bond(yoke, tube, "epoxy socket (30 mm)")
        sc.bond(tube, boot, "TPU press fit (0.3 mm interference)")

        # -- fasteners (positions mirror the counterbores in the CAD) --
        pcd_o, pcd_f, pcd_r = (hx.AK40_OUT_BOLT_PCD / 2,
                               hx.AK40_FRONT_BOLT_PCD / 2,
                               hx.AK40_REAR_BOLT_PCD / 2)
        yaw_flange, hip_rear = [], []
        for a in BOLT3:
            r = math.radians(a)
            yaw_flange.append(sc.screw(
                "screw_m25x6", f"{leg}_yawflange_{int(a)}",
                place @ trans(pcd_o * math.cos(r), pcd_o * math.sin(r),
                              -28.0) @ axis_frame("+z"),
                (coxa, yaw_act), leg=leg, joint="yaw"))
            sc.screw(
                "screw_m25x8", f"{leg}_yawcase_{int(a)}",
                place @ trans(pcd_f * math.cos(r), pcd_f * math.sin(r),
                              hx.PLATE_B_BOT + 1.5) @ axis_frame("+z"),
                (chassis_b, yaw_act), leg=leg, joint=None)
        for a in BOLT4:
            r = math.radians(a)
            hip_rear.append(sc.screw(
                "screw_m25x8", f"{leg}_hipcase_{int(a)}",
                place @ trans(hx.COXA_LENGTH + pcd_r * math.cos(r), -35.5,
                              hx.HIP_AXIS_Z + pcd_r * math.sin(r))
                @ axis_frame("+y"),
                (coxa, hip_act), leg=leg, joint="yaw"))
        hip_flange, knee_rear = [], []
        for a in BOLT3:
            r = math.radians(a)
            hip_flange.append(sc.screw(
                "screw_m25x6", f"{leg}_hipflange_{int(a)}",
                fem_fr @ trans(pcd_o * math.cos(r), 3.0,
                               pcd_o * math.sin(r)) @ axis_frame("-y"),
                (femur, hip_act), leg=leg, joint="hip"))
        for a in BOLT4:
            r = math.radians(a)
            knee_rear.append(sc.screw(
                "screw_m25x8", f"{leg}_kneecase_{int(a)}",
                fem_fr @ trans(hx.FEMUR_LENGTH + pcd_r * math.cos(r), 3.5,
                               pcd_r * math.sin(r)) @ axis_frame("-y"),
                (femur, knee_act), leg=leg, joint="hip"))
        knee_flange, pins = [], []
        for a in BOLT3:
            r = math.radians(a)
            knee_flange.append(sc.screw(
                "screw_m25x6", f"{leg}_kneeflange_{int(a)}",
                yoke_fr @ trans(pcd_o * math.cos(r), -3.0,
                                pcd_o * math.sin(r)) @ axis_frame("+y"),
                (yoke, knee_act), leg=leg, joint="knee"))
        for zz in (-38.0, -50.0):
            pins.append(sc.screw(
                "pin_25x22", f"{leg}_pin_{int(-zz)}",
                yoke_fr @ trans(0, hx.TIBIA_TUBE_Y, zz) @ rot_y(90),
                (yoke, tube), leg=leg, joint="knee"))

        # -- joints (fasteners ride the group they are torqued into) --
        distal_knee = [yoke, tube, boot] + knee_flange + pins
        distal_hip = [femur, knee_act] + distal_knee + hip_flange + knee_rear
        distal_yaw = [coxa, hip_act] + distal_hip + yaw_flange + hip_rear
        yaw_origin = (lift @ trans(hx.LEG_MOUNT_R * math.cos(azr),
                                   hx.LEG_MOUNT_R * math.sin(azr),
                                   hx.PLATE_B_TOP))[:3, 3]
        tang = [-math.sin(azr), math.cos(azr), 0.0]
        joints.append({"id": f"{leg}-yaw", "type": "revolute",
                       "axis": [0.0, 0.0, 1.0],
                       "origin": [float(v) for v in yaw_origin],
                       "instances": distal_yaw,
                       "limits": {"min": -25.0, "max": 25.0},
                       "home": 0, "label": f"{leg} yaw"})
        joints.append({"id": f"{leg}-hip", "type": "revolute", "axis": tang,
                       "origin": [float(v) for v in hip_pt[:3, 3]],
                       "parent": f"{leg}-yaw", "instances": distal_hip,
                       "limits": {"min": -35.0, "max": 20.0},
                       "home": 0, "label": f"{leg} hip"})
        joints.append({"id": f"{leg}-knee", "type": "revolute", "axis": tang,
                       "origin": [float(v) for v in knee_pt[:3, 3]],
                       "parent": f"{leg}-hip", "instances": distal_knee,
                       "limits": {"min": -30.0, "max": 35.0},
                       "home": 0, "label": f"{leg} knee"})

    poses = [{"id": "nominal", "name": "Nominal stance (built pose)",
              "jointValues": {}}]
    for stance in ("tall", "crouch"):
        sfd, std = hx.STANCES[stance]
        jv = {}
        for li in range(hx.LEG_COUNT):
            # hip is an absolute femur-pitch change; the knee value must
            # counter the hip rotation to hold the tibia's absolute angle
            jv[f"L{li}-hip"] = round(sfd - fd, 1)
            jv[f"L{li}-knee"] = round((fd - sfd) + (td - std), 1)
        note = " (TRANSITIONAL only)" if stance == "crouch" else ""
        poses.append({"id": stance, "name": f"{stance.capitalize()} stance"
                      + note, "jointValues": jv})

    # by-design contact/thread-engagement pairs the overlap check ignores
    ignore = [["coxa_link", "actuator"], ["femur_link", "actuator"],
              ["tibia_yoke", "actuator"], ["tibia_yoke", "tibia_tube"],
              ["foot_boot", "tibia_tube"], ["chassis_bottom", "actuator"],
              ["chassis_bottom", "standoff"], ["chassis_top", "standoff"],
              ["chassis_bottom", "battery"], ["chassis_top", "pi5"],
              ["screw_m25x6", "actuator"], ["screw_m25x8", "actuator"],
              ["screw_m3x8", "standoff"], ["screw_m25x12", "pi5"],
              ["pin_25x22", "tibia_tube"]]

    meshes = [{"id": f"stl:{k}", "name": f"{k}.stl", "url": v}
              for k, v in mesh_urls.items()]
    return {
        "name": "Hexapod AK40 — full robot (Design C, every fastener)",
        "source": HERE,
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": [0.0, 0.0, Z0],
        "checksConfig": {"overlapMm3": 40.0, "pitchMm": 2.0,
                         "ignoreOverlapPairs": ignore},
        "meshes": meshes,
        "instances": sc.instances,
        "joints": joints,
        "poses": poses,
        "animations": [],
        "routes": [],
    }, sc


def verify_attachment(scene: dict, sc: Scene) -> str:
    """Every non-fastener instance must be reachable from chassis_bottom
    through fastener/epoxy/press/velcro edges."""
    adj: dict[str, list[tuple[str, str]]] = {}
    for a, b, how in sc.edges:
        adj.setdefault(a, []).append((b, how))
        adj.setdefault(b, []).append((a, how))
    root = next(i["id"] for i in scene["instances"]
                if i["partType"] == "chassis_bottom")
    seen, stack = {root}, [root]
    while stack:
        for nxt, _ in adj.get(stack.pop(), []):
            if nxt not in seen:
                seen.add(nxt)
                stack.append(nxt)
    parts = [i for i in scene["instances"] if i["role"] != "fastener"]
    loose = [i["id"] for i in parts if i["id"] not in seen]
    lines = ["# Attachment report (auto-generated by make_scene.py)", ""]
    lines.append(f"{len(parts)} parts, "
                 f"{sum(1 for i in scene['instances'] if i['role'] == 'fastener')}"
                 " fasteners, "
                 f"{len(sc.edges)} attachment edges.")
    lines.append("")
    by_how: dict[str, int] = {}
    for a, b, how in sc.edges:
        key = how if not how.startswith("0") else "screw"
        by_how[key] = by_how.get(key, 0) + 1
    named = {k: v for k, v in sorted(by_how.items()) if "-" not in k}
    fastened = len(sc.edges) - sum(named.values())
    lines.append(f"- fastener edges: {fastened}")
    for k, v in named.items():
        lines.append(f"- {k}: {v}")
    lines.append("")
    if loose:
        lines.append("## LOOSE PARTS (not reachable from chassis_bottom)")
        lines += [f"- {x}" for x in loose]
    else:
        lines.append("All parts reachable from chassis_bottom. No loose parts.")
    report = "\n".join(lines)
    if loose:
        raise SystemExit("ATTACHMENT CHECK FAILED:\n" + report)
    return report


def main() -> None:
    scene, sc = build_scene()
    report = verify_attachment(scene, sc)
    art = os.path.join(os.path.dirname(HERE), "artifacts")
    os.makedirs(art, exist_ok=True)
    with open(os.path.join(art, "attachment_report.md"), "w") as fh:
        fh.write(report + "\n")
    with open(os.path.join(HERE, "scene.json"), "w") as fh:
        json.dump(scene, fh, indent=1)
    nfast = sum(1 for i in scene["instances"] if i["role"] == "fastener")
    print(f"wrote scene.json: {len(scene['instances'])} instances "
          f"({nfast} fasteners), {len(scene['joints'])} joints, "
          f"{len(scene['poses'])} poses")
    print(report.splitlines()[-1])


if __name__ == "__main__":
    main()
