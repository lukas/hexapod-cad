"""Integrated printed tibia study; outputs stay in artifacts/printed-leg.

Run with uv run python concepts/printed_sensor_leg/make_printed_leg.py.
Pinned source assets require BUILDVIZ_API_KEY on first download.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import trimesh
import yaml
import manifold3d as mf

import hexapod_prototype as hp
from concepts.sensor_fusion_foot import make_sensor_foot as foot

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "artifacts/printed-leg"
SOURCE = "prototype_sts3215/imu-tof-contact-foot"
BUILD = "prototype_sts3215/printed-sensor-leg"
CATALOG = "sts-printed-sensor-leg"
box, cylinder, cut, union, ring = foot.box, foot.cylinder, foot.cut, foot.union, foot.ring
IDENTITY = np.eye(4).T.flatten().tolist()
BASE_POD_ORIGIN = foot.POD_ORIGIN.copy()


def rod(a, b, radius):
    a, b = np.asarray(a), np.asarray(b)
    mesh = trimesh.creation.cylinder(radius, np.linalg.norm(b-a), sections=48)
    mesh.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], b-a))
    return mesh.apply_translation((a+b)/2)


def loft(stations, chamfer=2):
    vertices = []
    for z, w, d in stations:
        x, y, c = w/2, d/2, chamfer
        vertices += [[a, b, z] for a, b in [(x-c, -y), (x, -y+c), (x, y-c),
            (x-c, y), (-x+c, y), (-x, y-c), (-x, -y+c), (-x+c, -y)]]
    faces = []
    for j in range(len(stations)-1):
        for i in range(8):
            a, b, c, d = j*8+i, j*8+(i+1)%8, (j+1)*8+(i+1)%8, (j+1)*8+i
            faces += [[a, b, c], [a, c, d]]
    for i in range(1, 7):
        faces += [[0, i+1, i], [(len(stations)-1)*8, (len(stations)-1)*8+i, (len(stations)-1)*8+i+1]]
    return trimesh.Trimesh(vertices=vertices, faces=faces, process=True)


def source():
    base = f"{foot.HUB}/builds/{SOURCE}/versions/v1"
    path = foot.fetch(f"{base}/scene.json", OUT / "source/scene.json")
    scene = json.loads(path.read_text())
    spec = yaml.safe_load(foot.fetch(f"{base}/design_spec.yaml", OUT / "source/design_spec.yaml").read_text())
    meshes = {}
    for m in scene["meshes"]:
        url = m["url"] if m["url"].startswith("/") else f"/builds/{SOURCE}/versions/v1/{m['url']}"
        dest = foot.fetch(f"{foot.HUB}{url}", OUT / "source/stl" / m["name"])
        meshes[m["id"].split(":", 1)[1]] = trimesh.load_mesh(dest)
    return scene, spec, meshes


def knee():
    # Import the actual horn/pad interface; do not duplicate its dimensions.
    mesh = hp._sandwich_moving_yoke(tube_socket=False, socket_pin=False, reinforced=False,
        spine_extra_t=hp.TIBIA_YOKE_SPINE_PAD_T, pad_extra_reach=hp.YOKE_PAD_EXTRA_REACH)
    proximal = hp.FOOT_BOOT_TIP_L + (hp.TIBIA_LENGTH-hp.FOOT_BOOT_TIP_L)
    offset = proximal + hp._YOKE_SOCKET_X
    transform = np.array([[0, -1, 0, 0], [0, 0, 1, -hp.JOINT_SOCKET_Z],
                          [-1, 0, 0, offset], [0, 0, 0, 1]])
    mesh.apply_transform(transform)
    return mesh, offset-hp.SERVO_OUTPUT_X


def main():
    foot.POD_ORIGIN = BASE_POD_ORIGIN.copy()
    (OUT / "stl").mkdir(parents=True, exist_ok=True)
    old_scene, old_spec, old = source()
    retained = ["hall_guided_carriage", "hall_tpu_tread", "hall_tube_bushing", "hall_sensor_cover",
        "drv5055_to92", "radial_magnet", "compression_spring_envelope", "sensor_pod_lid",
        "sensor_custom_pcb", "vl53l4cd", "bmi270", "controller_allocation", "harness_connector"]
    retained += [f"pod_screw_{i}" for i in range(4)]
    meshes = {n: old[n].copy() for n in retained}
    pod_shift = np.array([10.0, 0.0, 6.0])
    pod_parts = foot.make_parts()
    for n in ["sensor_pod_lid", "sensor_custom_pcb", "vl53l4cd", "bmi270", "controller_allocation", "harness_connector"] + [f"pod_screw_{i}" for i in range(4)]:
        meshes[n].apply_translation(pod_shift)
    foot.POD_ORIGIN = foot.POD_ORIGIN + pod_shift
    yoke, pivot_z = knee()
    root_z = float(yoke.bounds[0, 2])
    outer = loft([(34, 24, 34), (40, 24, 34), (46, 22, 20), (62, 18, 20), (root_z, 24, 20), (root_z+2, 24, 20)])
    cavity = loft([(43, 20, 16), (62, 14, 16), (root_z-5, 20, 16)], chamfer=1)
    beam = cut(outer, cavity)
    # Retire the carbon collar, grooves and ties. Reuse only the rigid PCB shell.
    shell = cut(pod_parts["sensor_pod_carrier"][0], box([18, 30, 36], [-3, 0, 45])).apply_translation(pod_shift)
    # The old arm reaches the beam; clear the entire service/lid volume first.
    web = loft([(39, 20, 16), (49, 24, 16), (60, 16, 16)]).apply_translation([12, 0, 0])
    body = union(beam, web, yoke)
    body = cut(body, foot.pod_mesh(box([18.1, 14.1, 5.5], [0, 0, 2.2])),
        foot.pod_mesh(box([24, 20, 6.8], [0, 0, 7.25])))
    body = union(body, shell)
    # Hollow beam service hatch: face is planar y=-10; root spine remains solid.
    hatch_holes = [(0, 110), (0, 138)]
    body = union(body, *(rod([x, -10, z], [x, -4, z], 2.6) for x, z in hatch_holes))
    body = cut(body, box([10, 10, 22], [0, -10, 124]),
        *(rod([x, -13, z], [x, -3, z], .85) for x, z in hatch_holes))
    lid = cut(box([14, 1.5, 34], [0, -10.75, 124]),
        *(rod([x, -13, z], [x, -9, z], 1.1) for x, z in hatch_holes))
    # A dedicated cable passage joins the electronics bay to the central hollow.
    cable = [[*foot.pod_point([0, 6, 1.5])], [24, 6, 51], [16, 4, 55], [7, 0, 61], [0, 0, 78], [0, 0, 113], [0, 2, 123], [0, 7, 127], [0, 12, 128]]
    body = cut(body, *(rod(a, b, 2.1) for a, b in zip(cable, cable[1:])))
    # Upper harness exits through its own transverse 4.2 mm bore below the root.
    body = cut(body, rod([0, 0, 128], [0, 14, 128], 2.1))
    # An open-sided connector notch lets the cover lift clear of the fixed wire.
    meshes["sensor_pod_lid"] = cut(meshes["sensor_pod_lid"], foot.pod_mesh(box([5, 6, 5], [0, 7, 4.6])))
    # Cartridge sleeve takes the former guide function of the carbon tube.
    flange = box([24, 34, 5.7], [0, 0, 31.15])
    cartridge = cut(union(old["hall_foot_housing"], ring(4, 3, 7.8, 34), flange), cylinder(3, 7, 35))
    # Outboard bolt heads are accessible without dismantling the sensing shoe.
    bolt_xy = [(0, -14), (0, 14)]
    cartridge = cut(cartridge, *(cylinder(1.4, 26, 35, xy) for xy in bolt_xy),
        *(cylinder(2.45, 25, 28.3, xy) for xy in bolt_xy), cylinder(3.3, 21.25, 25.25))
    body = cut(body, *(cylinder(1.05, 33, 41, xy) for xy in bolt_xy), cylinder(3.2, 32, 43.5))
    end = foot.pod_point([0, 6, 1.5])
    p0, p1, p2 = np.array([7, 0, 24]), np.array([10, 5, 40]), np.array([end[0], 20, end[2]])
    hall_path = [((1-t)**3*p0 + 3*(1-t)**2*t*p1 + 3*(1-t)*t*t*p2 + t**3*end).tolist()
                 for t in np.linspace(0, 1, 10)]
    lead_tools = [rod(a, b, 1.45) for a, b in zip(hall_path, hall_path[1:])]
    cartridge = cut(cartridge, *lead_tools, box([6, 6, 7], [12, 2, 31.5]))
    body = cut(body, *lead_tools)
    meshes.update(printed_shank=body, service_hatch=lid, hall_foot_housing=cartridge)
    for i, xy in enumerate(bolt_xy):
        meshes[f"cartridge_screw_{i}"] = union(cylinder(1.25, 28.3, 38.3, xy), cylinder(2.25, 26.8, 28.3, xy))
    for i, (x, z) in enumerate(hatch_holes):
        meshes[f"hatch_screw_{i}"] = union(rod([x, -11.5, z], [x, -5.5, z], 1), rod([x, -12.7, z], [x, -11.5, z], 1.8))
    from concepts.printed_sensor_leg.tread import make_reinforced_tread
    meshes["hall_tpu_tread"], meshes["hall_guided_carriage"], tread_report = make_reinforced_tread(old)
    # Boolean intersections can leave micron-scale facets; bound cleanup to 1 um.
    for n in ("printed_shank", "hall_foot_housing", "sensor_pod_lid"):
        m = meshes[n]
        solid = mf.Manifold(mf.Mesh(vert_properties=np.asarray(m.vertices, dtype=np.float32),
                                    tri_verts=np.asarray(m.faces, dtype=np.uint32)))
        if solid.status() != mf.Error.NoError:
            raise RuntimeError(f"{n}: invalid Boolean solid; fix geometry before exporting")
        clean = solid.simplify(.001).to_mesh()
        meshes[n] = trimesh.Trimesh(vertices=clean.vert_properties[:, :3], faces=clean.tri_verts, process=True)
    body, cartridge = meshes["printed_shank"], meshes["hall_foot_housing"]
    descriptions = {
        "printed_shank": "One-piece hollow printed tibia, canonical knee yoke and rigid sensor bay; no carbon spar or pod collar",
        "service_hatch": "Removable cable access cover; beam stays hollow with a solid knee root",
        "hall_foot_housing": "Bolted replaceable Hall cartridge, integrated guide sleeve and positive overload stop",
    }
    lookup = {i["partType"]: i for i in old_scene["instances"]}
    scene = {k: old_scene[k] for k in ["units", "designSpecUrl", "animations"]}
    scene.update(name="Printed sensor leg · hollow shank + replaceable foot", buildId=BUILD,
        source="concepts/printed_sensor_leg/make_printed_leg.py", center=[4, 0, 94], meshes=[], instances=[])
    for name, mesh in meshes.items():
        mesh.export(OUT / "stl" / f"{name}.stl")
        desc = descriptions.get(name, lookup.get(name, {}).get("name", name.replace("_", " ")))
        scene["meshes"].append({"id": f"mesh:{name}", "name": f"{name}.stl", "url": f"stl/{name}.stl"})
        screw = "screw" in name
        scene["instances"].append({"id": name, "partType": name, "meshId": f"mesh:{name}", "name": desc,
            "transform": IDENTITY, "color": {"printed_shank": "#e6934e", "service_hatch": "#536981", "hall_foot_housing": "#4f78b8"}.get(name, lookup.get(name, {}).get("color", "#d1d5db")),
            "role": "fastener" if screw else "structure" if name == "printed_shank" else lookup.get(name, {}).get("role", "part"),
            "cots": screw or lookup.get(name, {}).get("cots", False), "focusGroup": "printed_leg"})
    moving = ["hall_guided_carriage", "hall_tpu_tread", "radial_magnet"]
    cartridge_ids = ["hall_foot_housing", "hall_sensor_cover", "hall_tube_bushing", "drv5055_to92", "compression_spring_envelope"] + [f"cartridge_screw_{i}" for i in range(2)]
    pod_lid = ["sensor_pod_lid"] + [f"pod_screw_{i}" for i in range(4)]
    hatch_ids = ["service_hatch"] + [f"hatch_screw_{i}" for i in range(2)]
    free_ids = moving+cartridge_ids+pod_lid+hatch_ids
    def slide(id, axis, ids, parent="leg-pitch", maximum=25, minimum=0):
        return {"id": id, "type": "prismatic", "axis": axis, "origin": [0, 0, 0], "parent": parent,
                "home": 0, "limits": {"min": minimum, "max": maximum}, "instances": ids}
    scene["joints"] = [{"id": "leg-pitch", "type": "revolute", "axis": [0, 1, 0], "origin": [0, 0, 13.4], "home": 0,
        "limits": {"min": 0, "max": 50}, "instances": [n for n in meshes if n not in free_ids],
        "label": "Inspection pitch about the foot, not robot actuation"},
        slide("remove-cartridge", [0, 0, -1], cartridge_ids),
        slide("contact-stroke", [0, 0, 1], moving, "remove-cartridge", 1.5),
        slide("open-pod", (foot.POD_R @ [0, 0, 1]).tolist(), [], maximum=4),
        slide("open-pod-side", [0, 1, 0], pod_lid, "open-pod"),
        slide("open-hatch", [0, -1, 0], hatch_ids)]
    scene["poses"] = [{"id": "assembled", "name": "Complete printed leg", "jointValues": {}},
        {"id": "service", "name": "Open covers and remove foot cartridge", "jointValues": {"remove-cartridge": 22, "open-pod": 4, "open-pod-side": 25, "open-hatch": 24}},
        {"id": "sensors", "name": "Inspect rigid sensor board", "jointValues": {"open-pod": 4, "open-pod-side": 25}},
        {"id": "touchdown", "name": "Touchdown / 0.35 mm", "jointValues": {"contact-stroke": .35}},
        {"id": "stop", "name": "Positive overload stop / 1.50 mm", "jointValues": {"contact-stroke": 1.5}},
        {"id": "stance", "name": "40 degree stance / touchdown", "jointValues": {"leg-pitch": 40, "contact-stroke": .35}}]
    allowed = []
    for declaration in old_scene["checksConfig"]["allowedInterferences"]:
        entry = dict(declaration)
        entry["instances"] = [next((x["partType"] for x in old_scene["instances"] if x["id"] == i), i) for i in entry["instances"]]
        if all(i in meshes for i in entry["instances"]):
            allowed.append(entry)
    for n in meshes:
        if "screw" in n:
            allowed.append({"kind": "thread_engagement", "instances": [n, "printed_shank"], "maxPenetrationMm": .3,
                "reason": "Nominal thread envelope cuts into printed pilot; coupon fit and torque remain unvalidated."})
    scene["checksConfig"] = {"toleranceMm": .25, "minPenetrationMm": .25, "minWallMm": .7, "allowedInterferences": allowed}
    scene["routes"] = [{"id": "internal-harness", "label": "Protected power/data through hollow beam", "kind": "data", "diameterMm": 1.2,
        "minBendRadiusMm": 3, "maxLengthMm": 160, "instances": ["printed_shank", "harness_connector", "sensor_custom_pcb"],
        "waypoints": [{"instanceId": "printed_shank", "local": p} for p in cable]},
        {"id": "hall-leads", "label": "Fixed Hall leads / unplug to replace cartridge", "kind": "signal", "diameterMm": .8,
         "minBendRadiusMm": 2.4, "maxLengthMm": 65, "instances": ["drv5055_to92", "hall_foot_housing", "hall_sensor_cover", "printed_shank", "harness_connector", "sensor_custom_pcb"],
         "waypoints": [{"instanceId": "printed_shank", "local": p} for p in hall_path]}]
    invalid = [n for n, m in meshes.items() if not m.is_watertight or len(m.split()) != 1]
    overlaps = []
    for travel in (0, .2, .35, .75, 1, 1.5):
        for n in moving:
            moved = meshes[n].copy().apply_translation([0, 0, travel])
            for receiver in ("printed_shank", "hall_foot_housing"):
                v = trimesh.boolean.intersection([moved, meshes[receiver]], engine="manifold").volume
                overlaps.append({"moving": n, "fixed": receiver, "travelMm": travel, "overlapMm3": round(abs(float(v)), 5)})
    yoke_missing = abs(float(cut(yoke, body).volume))
    removal = []
    for separation in (0, .5, 2, 6, 12, 22):
        moved = cartridge.copy().apply_translation([0, 0, -separation])
        overlap = trimesh.boolean.intersection([moved, body], engine="manifold")
        removal.append({"withdrawalMm": separation, "overlapMm3": round(abs(float(overlap.volume)), 5)})
    optics = foot.optical_report(meshes, "printed_shank")
    service_checks = []
    lid_offsets = [(foot.POD_R @ [0, 0, v]) for v in np.linspace(0, 4, 9)]
    lid_offsets += [(foot.POD_R @ [0, 0, 4])+[0, v, 0] for v in np.linspace(0, 25, 11)]
    for offset in lid_offsets:
        cover = meshes["sensor_pod_lid"].copy().apply_translation(offset)
        v = trimesh.boolean.intersection([body, cover], engine="manifold").volume
        service_checks.append(abs(float(v)))
    drivers = []
    for xy in bolt_xy:
        tool = rod([*xy, 0], [*xy, 26.8], 1.6)
        for stroke in (0, .35, 1.5):
            for n in ("hall_foot_housing", "hall_guided_carriage", "hall_tpu_tread"):
                target = meshes[n].copy()
                if n in moving:
                    target.apply_translation([0, 0, stroke])
                drivers.append(abs(float(trimesh.boolean.intersection([tool, target], engine="manifold").volume)))
    pcb_overlap = abs(float(trimesh.boolean.intersection([body, meshes["sensor_custom_pcb"]], engine="manifold").volume))
    report = {"invalidMeshes": invalid, "movingClearances": overlaps, "optics": optics, "tread": tread_report,
        "yokeMaterialRemovedMm3": yoke_missing, "cartridgeRemoval": removal,
        "serviceAccess": {"lidSamples": len(service_checks), "lidMaxOverlapMm3": max(service_checks),
            "driverDiameterMm": 3.2, "driverMaxOverlapMm3": max(drivers), "pcbOverlapMm3": pcb_overlap},
        "kneeAxisZMm": pivot_z, "axisToTipAxialMm": pivot_z-float(meshes["hall_tpu_tread"].bounds[0, 2]),
        "printedShankVolumeCm3": round(body.volume/1000, 3),
        "status": "Geometry concept; untested mechanical strength, print orientation, electronics and fit"}
    (OUT / "report.json").write_text(json.dumps(report, indent=2)+"\n")
    scene["analysis"] = {"sourceRevision": {"buildId": SOURCE, "branch": "main", "version": "v1"},
        "sensorContract": old_scene["analysis"]["sensorContract"], "status": report["status"], "kneeAxis": [0, 0, pivot_z],
        "podOrigin": foot.POD_ORIGIN.tolist(), "geometryCleanupToleranceMm": .001}
    spec = {"build": {"id": BUILD, "name": scene["name"], "units": "mm", "generated_by": scene["source"], "status": report["status"]},
        "parts": {n: {"purpose": descriptions.get(n, old_spec["parts"].get(n, {}).get("purpose", n.replace("_", " "))), "file": f"stl/{n}.stl"} for n in meshes},
        "robot_interface": {"source": "canonical hexapod_prototype.py knee horn/pad interface, socket removed",
            "knee_axis_z_mm": pivot_z, "unloaded_axis_to_tip_axial_mm": report["axisToTipAxialMm"],
            "correction": "The predecessor's 152 mm knee-to-tip label was socket-to-tip. Actual axial knee distance includes the yoke offset."},
        "structure": {"nominal_wall_mm": 2, "beam_section_lower_mm": [18, 20], "beam_section_root_mm": [24, 20],
            "root": "Continuous solid knee spine; cavity terminates 5 mm below root face",
            "retired": "Carbon tube, epoxy socket, separate pod carrier collar and cable ties; sleeve built into removable cartridge",
            "printing": "Candidate tough PA/PETG prototype. Orient the long beam in the bed plane; support knee and pocket as needed. Material, process and load capacity require coupons and physical tests."},
        "sensor_packaging": old_spec["sensor_packaging"], "contact_logic": old_spec["contact_logic"], "spring_target": old_spec["spring_target"],
        "wiring": {"internal-harness": {"purpose": "Protected power/data via beam hollow, service hatch and upper exit; strain relief required at both ends"},
            "hall-leads": {"purpose": "Fixed cartridge Hall leads unplug at electronics bay before cartridge removal; no wire crosses moving shoe"}},
        "validation_required": ["Verify whole-leg clearance at the wider printed beam; this is an isolated leg study.",
            "Select print material, orientation and wall count; compare assembled weight, bending and torsion to carbon leg.",
            "Confirm cartridge screw retention and guide clearance, free return at tilted touchdown, fatigue and dirt tolerance.",
            "Design actual PCB and connector; calibrate IMU alignment, ToF transform/offset and Hall threshold.",
            "Choose and validate real spring. Hall displacement saturates at stop and is not full load sensing."]}
    spec["sensor_packaging"]["manufacturing"] = "Integrated printed bay; no tube clamp. PCB and package allocations retained from source; electronics unvalidated."
    (OUT / "scene.json").write_text(json.dumps(scene, indent=2)+"\n")
    (OUT / "design_spec.yaml").write_text(yaml.safe_dump(spec, sort_keys=False, allow_unicode=True))
    from concepts.printed_sensor_leg.presentation import write_presentation
    write_presentation(OUT, scene, report, spec)
    print(json.dumps({k: v for k, v in report.items() if k not in ("optics", "movingClearances")}, indent=2))
    if invalid or yoke_missing > .001 or any(x["overlapMm3"] > .001 for x in overlaps+removal) or not optics["passed"] or max(service_checks+drivers+[pcb_overlap]) > .001:
        raise RuntimeError("Geometry gate failed: inspect artifacts/printed-leg/report.json before publishing")


if __name__ == "__main__":
    main()
