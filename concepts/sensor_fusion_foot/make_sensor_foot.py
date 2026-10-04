"""One-foot BuildViz packaging study, built on the published Hall-foot v4.

Run: uv run python concepts/sensor_fusion_foot/make_sensor_foot.py
Outputs are generated under artifacts/sensor-foot, never committed.
This is packaging geometry, not released electronics or a calibrated force sensor.
"""
from __future__ import annotations

import json
import math
import os
import shutil
import urllib.request
import urllib.error
from pathlib import Path

import numpy as np
import trimesh
import yaml

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "artifacts/sensor-foot"
HUB = "https://buildviz.cwd1f0-new-cluster.coreweave.app"
SOURCE = "prototype_sts3215/hall-contact-foot"
SOURCE_VERSION = "v4"
BUILD = "prototype_sts3215/imu-tof-contact-foot"
POD_ORIGIN = np.array([17.0, 0.0, 41.0])
OPTICAL_TILT_DEG = 40.0
OPTICAL_HALF_CONE_DEG = 12.5  # ST collector exclusion, not advertised 18-degree FoV.
PCB_SIZE = (16.0, 12.0, 1.0)  # Custom-board allocation, not an existing product.
TOF_SIZE = (4.4, 2.4, 1.0)  # ST VL53L4CD package.
IMU_SIZE = (2.5, 3.0, 0.8)  # Bosch BMI270 package.


def box(size, center):
    mesh = trimesh.creation.box(size)
    mesh.apply_translation(center)
    return mesh


def cylinder(radius, z0, z1, xy=(0, 0)):
    mesh = trimesh.creation.cylinder(radius, z1-z0, sections=64)
    mesh.apply_translation([*xy, (z0+z1)/2])
    return mesh


def union(*meshes):
    return trimesh.boolean.union(list(meshes), engine="manifold")


def cut(body, *tools):
    return trimesh.boolean.difference([body, *tools], engine="manifold")


def ring(outer, inner, z0, z1):
    return cut(cylinder(outer, z0, z1), cylinder(inner, z0-1, z1+1))


def ry(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0],
                     [-math.sin(a), 0, math.cos(a)]])


POD_R = ry(-OPTICAL_TILT_DEG)


def pod_point(p):
    return POD_R @ np.asarray(p) + POD_ORIGIN


def pod_mesh(mesh):
    mat = np.eye(4)
    mat[:3, :3] = POD_R
    mat[:3, 3] = POD_ORIGIN
    return mesh.apply_transform(mat)


def fetch(url, dest):
    dest.parent.mkdir(parents=True, exist_ok=True)
    if not dest.exists():
        request = urllib.request.Request(url)
        if os.environ.get("BUILDVIZ_API_KEY"):
            request.add_header("X-API-Key", os.environ["BUILDVIZ_API_KEY"])
        try:
            with urllib.request.urlopen(request, timeout=45) as response:
                dest.write_bytes(response.read())
        except urllib.error.HTTPError as error:
            if error.code == 401:
                raise RuntimeError("BuildViz authentication required: set BUILDVIZ_API_KEY for the configured cloud hub, then rerun. No source geometry was substituted.") from error
            raise
    return dest


def source():
    path = fetch(f"{HUB}/builds/{SOURCE}/versions/{SOURCE_VERSION}/scene.json",
                 OUT / "source_scene.json")
    scene = json.loads(path.read_text())
    spec_path = fetch(f"{HUB}/builds/{SOURCE}/versions/{SOURCE_VERSION}/design_spec.yaml",
                      OUT / "source_design_spec.yaml")
    spec = yaml.safe_load(spec_path.read_text())
    meshes = {}
    for mesh in scene["meshes"]:
        dest = OUT / "source_stl" / mesh["name"]
        fetch(f"{HUB}{mesh['url']}", dest)
        (OUT / "stl").mkdir(exist_ok=True)
        shutil.copyfile(dest, OUT / "stl" / mesh["name"])
        mesh["url"] = f"stl/{mesh['name']}"
        meshes[mesh["id"].split(":", 1)[1]] = trimesh.load_mesh(dest)
    return scene, spec, meshes


def make_parts():
    # Split collar: 8.30 mm bore around existing 8 mm carbon tube; no drilling.
    collar = cut(ring(6, 4.15, 31, 56), box([8, 1, 28], [-7, 0, 43.5]))
    for z in (34, 53):
        collar = cut(collar, ring(7, 5.5, z, z+2))
    # Four bosses support a removable lid; PCB corners clear the bosses.
    bosses = [(x, y) for x in (-8, 8) for y in (-6, 6)]
    shell = cut(box([20, 16, 7], [0, 0, 0.5]), box([18, 14, 8], [0, 0, 2]))
    shell = union(shell, *(cylinder(2.2, -2, 4, xy) for xy in bosses),
                  *(cylinder(1, -2, -0.5, xy) for xy in [(-5, -3), (-5, 3), (5, -3), (5, 3)]))
    # Opening is 8 x 6 mm through 1.5 mm recessed nose. No optical cover window.
    shell = cut(shell, box([8, 6, 4], [0, 0, -3]),
                *(cylinder(0.85, -0.2, 8, xy) for xy in bosses),
                box([5, 6, 3], [0, 8, 2]))
    # Trim only the arm in pod coordinates, before joining the finished shell.
    # Expanded keepouts avoid coplanar boolean slivers and protect the PCB/lid.
    arm = box([9, 10, 8], [7, 0, 38])
    arm.vertices = (arm.vertices-POD_ORIGIN) @ POD_R
    arm = cut(arm, box([18.1, 14.1, 20], [0, 0, 8]),
              box([24, 20, 26], [0, 0, 16.95]))
    carrier = cut(union(collar, pod_mesh(arm), pod_mesh(shell)), cylinder(4.15, 29, 58))
    lid = cut(box([20, 16, 1.2], [0, 0, 4.6]),
              *(cylinder(1.1, 3, 7, xy) for xy in bosses))
    pcb = cut(box(PCB_SIZE, [0, 0, 0]),
              *(cylinder(2.5, -1, 1, xy) for xy in bosses))
    parts = {
        "sensor_pod_carrier": (carrier, "#f39c55", False, "Split tube collar, protected optical nose and rigid PCB supports"),
        "sensor_pod_lid": (pod_mesh(lid), "#b1bbc9", False, "Removable four-screw electronics lid"),
        "sensor_custom_pcb": (pod_mesh(pcb), "#177b63", True, "16 x 12 x 1 mm custom PCB allocation; layout unvalidated"),
        "vl53l4cd": (pod_mesh(box(TOF_SIZE, [0, 0, -1])), "#d7e5ed", True, "VL53L4CD 4.4 x 2.4 x 1 mm package; optical face points out of nose"),
        "bmi270": (pod_mesh(box(IMU_SIZE, [-4, 0, 0.9])), "#8c6ff0", True, "BMI270 2.5 x 3 x 0.8 mm; rigid tibia orientation, not terrain normal"),
        "controller_allocation": (pod_mesh(box([4, 4, 1], [4, 0, 1])), "#344659", True, "Reserved controller/regulator area; device and PCB routing unselected"),
        "harness_connector": (pod_mesh(box([4, 2, 2], [0, 5, 1.5])), "#f0d17c", True, "Connector envelope; pinout and actual part unselected"),
    }
    for i, xy in enumerate(bosses):
        screw = union(cylinder(1, 0.2, 5.2, xy), cylinder(1.8, 5.2, 6.4, xy))
        parts[f"pod_screw_{i}"] = (pod_mesh(screw), "#d1d5db", True, "M2 x 5 nominal lid screw envelope; pilot fit to bench check")
    for i, z in enumerate((34, 53)):
        strap = union(ring(6.3, 5.5, z, z+2), box([2.5, 3.5, 2], [-6.4, 0, z+1]))
        parts[f"pod_strap_{i}"] = (strap, "#414a59", True, "2 mm nylon tie envelope in collar groove; clamp fit unvalidated")
    return parts


def optical_report(meshes):
    """Check sampled aperture rays and the actual pod/tread meshes, not boxes."""
    tip = meshes["hall_tpu_tread"]
    body = meshes["sensor_pod_carrier"]
    forward = POD_R @ [0, 0, -1]
    # Full package footprint is conservative for the unspecified Rx/Tx centers.
    origins, directions = [], []
    for x in (-TOF_SIZE[0]/2, 0, TOF_SIZE[0]/2):
        for y in (-TOF_SIZE[1]/2, 0, TOF_SIZE[1]/2):
            for half in (0, OPTICAL_HALF_CONE_DEG):
                for azimuth in np.linspace(0, 2*math.pi, 32, endpoint=False):
                    a = math.radians(half)
                    d = np.array([math.sin(a)*math.cos(azimuth), math.sin(a)*math.sin(azimuth), -math.cos(a)])
                    origins.append(pod_point([x, y, -1.501]))
                    directions.append(POD_R @ d)
    origins, directions = np.asarray(origins), np.asarray(directions)
    obstructions = []
    states = []
    movable = {"hall_guided_carriage", "hall_tpu_tread", "radial_magnet"}
    excluded = {"vl53l4cd", "bmi270", "controller_allocation", "sensor_custom_pcb", "harness_connector"}
    for stroke in (0, 0.35, 1.5):
        for name, mesh in meshes.items():
            if name in excluded:
                continue
            test = mesh.copy()
            if name in movable:
                test.apply_translation([0, 0, stroke])
            loc, rays, _ = test.ray.intersects_location(origins, directions, multiple_hits=False)
            if len(rays):
                distances = np.linalg.norm(loc-origins[rays], axis=1)
                hits = distances[distances < 150]
                if len(hits):
                    obstructions.append({"part": name, "strokeMm": stroke, "rays": len(hits), "nearestMm": float(hits.min())})
        # Nominal leg-pitch sector only, not arbitrary tilt azimuth or terrain.
        for angle in (0, 40, 50):
            rot = ry(angle)
            tread_world = (tip.vertices+[0, 0, stroke]) @ rot.T
            lift = -float(tread_world[:, 2].min())
            pod_world = body.vertices @ rot.T
            clearance = float(pod_world[:, 2].min()+lift)
            fixed_clearances = {name: round(float((mesh.vertices @ rot.T)[:, 2].min()+lift), 3)
                for name, mesh in meshes.items() if name not in movable}
            origin = rot @ pod_point([0, 0, -1.5]) + [0, 0, lift]
            direction = rot @ forward
            ground_range = -origin[2]/direction[2]
            states.append({"tibiaPitchDeg": angle, "strokeMm": stroke,
                           "podGroundClearanceMm": round(clearance, 3),
                           "fixedPartClearancesMm": fixed_clearances,
                           "centralBeamRangeToFlatGroundMm": round(float(ground_range), 3),
                           "beamFromVerticalDeg": round(math.degrees(math.acos(-direction[2])), 3)})
    return {"passed": not obstructions and min(min(s["fixedPartClearancesMm"].values()) for s in states)>0.5,
            "sampledRaysPerStroke": len(origins), "collectorConeFullDeg": 25,
            "opticalOcclusionsWithin150Mm": obstructions, "states": states,
            "scope": "Sampled 25-degree cone, full package origin footprint, three contact strokes. Ground plane in nominal pitch direction. Not a continuous motion or arbitrary-terrain proof."}


def mechanical_report(meshes):
    """Guard the actual removed collisions, including moving TPU/housing fit."""
    invalid = [name for name, mesh in meshes.items()
               if not mesh.is_watertight or len(mesh.split()) != 1]
    checks = []
    for stroke in (0, 0.35, 1.5):
        tread = meshes["hall_tpu_tread"].copy()
        tread.apply_translation([0, 0, stroke])
        overlap = trimesh.boolean.intersection([tread, meshes["hall_foot_housing"]], engine="manifold")
        checks.append({"pair": "tread/housing", "strokeMm": stroke,
                       "overlapMm3": abs(float(overlap.volume)) if len(overlap.faces) else 0})
    for name in ("sensor_custom_pcb", "sensor_pod_lid"):
        overlap = trimesh.boolean.intersection([meshes["sensor_pod_carrier"], meshes[name]], engine="manifold")
        checks.append({"pair": f"carrier/{name}", "overlapMm3": abs(float(overlap.volume)) if len(overlap.faces) else 0})
    result = {"passed": not invalid and all(c["overlapMm3"] < 0.001 for c in checks),
              "invalidMeshes": invalid, "checks": checks}
    if not result["passed"]:
        raise RuntimeError(f"Mechanical regression: fix before publishing: {result}")
    return result


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    scene, spec, meshes = source()
    # Source v4's housing fouls the ground before full travel at 40/50 degrees.
    # Add a 2 mm outer tread cap BELOW the housing sweep, retaining the lock.
    # This deliberately changes knee-to-tip length from 150 to 152 mm unloaded.
    outer_angles = np.linspace(0, math.acos((13.4-3.6)/15.4), 64)
    inner_angles = np.linspace(math.acos((13.4-3.6)/13.3), 0, 64)
    profile = [[15.4*math.sin(a), 13.4-15.4*math.cos(a)] for a in outer_angles]
    profile += [[13.3*math.sin(a), 13.4-13.3*math.cos(a)] for a in inner_angles]
    profile += [profile[0]]
    cap = trimesh.creation.revolve(profile, sections=96)
    meshes["hall_tpu_tread"] = union(meshes["hall_tpu_tread"], cap)
    # Retire the inherited upper TPU rim that rubs the stationary housing.
    # Preserve the inner snap interface, relieve only the outside above 4.05 mm.
    meshes["hall_tpu_tread"] = cut(meshes["hall_tpu_tread"], ring(20, 9.4, 4.05, 10))
    meshes["hall_tpu_tread"].export(OUT / "stl/hall_tpu_tread.stl")
    # Bored lead exit through the existing sensor bay, away from the moving stem.
    lead_port = box([6, 2.8, 8], [9, 0, 27])
    for name in ("hall_foot_housing", "hall_sensor_cover"):
        meshes[name] = cut(meshes[name], lead_port)
        meshes[name].export(OUT / "stl" / f"{name}.stl")
        spec["parts"][name]["purpose"] += " Added open 2.8 mm fixed-lead notch from the Hall sensor bay; seal after calibration."
    spec["parts"]["hall_tpu_tread"]["purpose"] = "TPU tread with 2 mm deeper spherical cap, clearing fixed housing through full angled contact stroke; lock interface retained."
    spec["robot_interface"]["preserved_knee_to_tip_mm"] = 152.0
    spec["robot_interface"]["tip_change"] = "2 mm longer unloaded than source v4; update kinematics and calibrate actual tread deformation."
    original_ids = [i["id"] for i in scene["instances"]]
    descriptions = {}
    for name, (mesh, color, cots, description) in make_parts().items():
        if not mesh.is_watertight or len(mesh.split()) != 1:
            raise RuntimeError(f"{name}: expected one closed solid; fix geometry before publishing")
        mesh.export(OUT / "stl" / f"{name}.stl")
        meshes[name] = mesh
        descriptions[name] = description
        scene["meshes"].append({"id": f"mesh:{name}", "name": f"{name}.stl", "url": f"stl/{name}.stl"})
        scene["instances"].append({"id": name, "meshId": f"mesh:{name}", "partType": name,
                                  "name": description, "color": color, "cots": cots,
                                  "role": "fastener" if "screw" in name else "sensor_packaging",
                                  "focusGroup": "sensor_pod", "transform": np.eye(4).T.flatten().tolist()})
        spec["parts"][name] = {"purpose": description, "file": f"stl/{name}.stl"}
    # Parent all original inspection joints to a real, labeled pitch study.
    assigned = {i for joint in scene["joints"] for i in joint["instances"]}
    for joint in scene["joints"]:
        joint["parent"] = "leg-pitch"
    new_ids = list(descriptions)
    lid_ids = ["sensor_pod_lid"] + [i for i in new_ids if "screw" in i]
    scene["joints"].insert(0, {"id": "leg-pitch", "type": "revolute", "axis": [0, 1, 0],
        "origin": [0, 0, 13.4], "home": 0, "limits": {"min": 0, "max": 50},
        "instances": [i for i in original_ids if i not in assigned]+[i for i in new_ids if i not in lid_ids],
        "label": "Leg pitch study (ground shown in companion diagram)"})
    scene["joints"].append({"id": "open-pod", "type": "prismatic", "axis": (POD_R @ [0, 0, 1]).tolist(),
        "origin": POD_ORIGIN.tolist(), "parent": "leg-pitch", "home": 0, "limits": {"min": 0, "max": 18},
        "instances": lid_ids, "label": "Remove lid to inspect IMU and custom PCB"})
    scene["poses"] += [{"id": "stance-40", "name": "40 degree stance / touchdown", "jointValues": {"leg-pitch": 40, "contact-stroke": 0.35}},
        {"id": "stance-50", "name": "50 degree stance / hard stop", "jointValues": {"leg-pitch": 50, "contact-stroke": 1.5}},
        {"id": "inspect-pod", "name": "Open electronics pod", "jointValues": {"open-pod": 18}}]
    next(p for p in scene["poses"] if p["id"] == "exploded")["jointValues"]["open-pod"] = 18
    scene["name"] = "Foot sensing prototype · IMU + ToF + Hall contact"
    scene["buildId"] = BUILD
    scene["source"] = "concepts/sensor_fusion_foot/make_sensor_foot.py"
    scene["center"] = [8, 0, 29]
    scene["checksConfig"]["allowedInterferences"] += [
        {"kind": "press_fit", "instances": ["sensor_pod_carrier", f"pod_strap_{i}"],
         "reason": "Nylon tie sits in a molded collar groove; elastic clamp envelope.", "maxPenetrationMm": 0.6} for i in range(2)]
    # Sensor wiring leaves the fixed pod; nothing crosses the moving shoe.
    scene["routes"] = [{"id": "foot-sensor-harness", "label": "Fixed sensor harness / strain relief to tibia", "kind": "data",
        "diameterMm": 1.2, "maxLengthMm": 65, "minBendRadiusMm": 3, "maxUnsupportedMm": 45,
        "instances": ["harness_connector", "sensor_custom_pcb", "sensor_pod_carrier", "pod_strap_1"],
        "waypoints": [{"instanceId": "harness_connector", "local": pod_point([0, 6, 1.5]).tolist()},
                      {"instanceId": "sensor_pod_carrier", "local": [16, 12, 43]},
                      {"instanceId": "sensor_pod_carrier", "local": [16, 17, 49]},
                      {"instanceId": "sensor_pod_carrier", "local": [10, 15, 54]},
                      {"instanceId": "pod_strap_1", "local": [0, 6.5, 54], "label": "Tie strain relief"}]}]
    scene["routes"].append({"id": "hall-to-pod", "label": "Hall sensor leads / fixed outside tube", "kind": "signal",
        "diameterMm": 0.8, "maxLengthMm": 55, "minBendRadiusMm": 2.4,
        "instances": ["hall-sensor-cover", "hall-foot-housing", "sensor_pod_carrier", "sensor_custom_pcb", "harness_connector"],
        "waypoints": [{"instanceId": "drv5055-to92", "local": [7, 0, 24]},
                      {"instanceId": "sensor_pod_carrier", "local": [10, 0, 27]},
                      {"instanceId": "sensor_pod_carrier", "local": [12, 2, 32]},
                      {"instanceId": "harness_connector", "local": pod_point([0, 6, 1.5]).tolist()}]})
    mechanics = mechanical_report(meshes)
    (OUT / "mechanical_report.json").write_text(json.dumps(mechanics, indent=2)+"\n")
    report = optical_report(meshes)
    (OUT / "optical_report.json").write_text(json.dumps(report, indent=2)+"\n")
    scene["analysis"] = {"sourceRevision": {"buildId": SOURCE, "branch": "main", "version": SOURCE_VERSION},
        "opticalPackaging": report, "status": "Geometric packaging study; no physical/electronic validation",
        "sensorContract": {"imu": "Rigid lower-leg orientation; fuse with body IMU/encoders, reject impact-corrupted tilt",
         "tof": "Beam distance + quality/status + timestamp; calibrated transform and terrain assumption required for tip clearance",
         "hall": "Touchdown/release/displacement; saturation at 1.5 mm, not full support force"}}
    spec["build"] = {"id": BUILD, "name": scene["name"], "units": "mm", "status": scene["analysis"]["status"],
                     "generated_by": scene["source"]}
    spec["sensor_packaging"] = {"base_revision": f"{SOURCE}@main@{SOURCE_VERSION}",
        "imu_package_mm": IMU_SIZE, "tof_package_mm": TOF_SIZE, "custom_pcb_allocation_mm": PCB_SIZE,
        "optical_mount_angle_deg": OPTICAL_TILT_DEG, "collector_exclusion_full_deg": 25,
        "aperture_mm": [8, 6], "recess_mm": 1.5,
        "manufacturing": "Carrier/lid prototype PETG or PA. Custom PCB, connector, screw fit and tie fit remain unselected/unvalidated.",
        "sources": ["https://www.st.com/resource/en/datasheet/vl53l4cd.pdf", "https://www.bosch-sensortec.com/en/products/motion-sensors/imus/bmi270"],
        "limits": ["No optical cover glass; if added, redesign window and calibrate crosstalk.",
                   "Single beam does not measure terrain normal. Mount angle serves 0–50 degree nominal pitch sector.",
                   "IMU yaw is not absolute; impact acceleration corrupts gravity tilt.",
                   "Spring bottoms at approximately 2 N axial target load; Hall is contact, not weight sensing.",
                   "Prototype adds distal mass and width; weigh final assembly and check whole-leg sweep."]}
    spec["wiring"] = {"foot-sensor-harness": {"purpose": "Power/data through fixed pod and strain relief, external to occupied Hall tube bore",
        "notes": "Short local SPI/I2C on custom board; local ADC for Hall. Select controller and robust upstream transport before PCB layout; do not assume six sensors share one unbuffered long I2C bus. Timestamp all channels; retain ranging validity and stale/dropout flags."}}
    spec["wiring"]["hall-to-pod"] = {"purpose": "Three thin fixed Hall leads through sensor bay and outside carbon tube to local ADC", "notes": "No lead crosses moving shoe; seal bay after calibration. Route geometry is an allocation; wire gauge and connector remain unselected."}
    spec["validation_required"] += ["Build one foot first; check ToF on actual dark/shiny flooring and with ambient IR, dust and contamination.",
        "Calibrate ToF offset and IMU-to-tibia/sensor-to-tip transforms; synchronize samples with body IMU and encoders.",
        "Check full-leg interference, cable fatigue and housing-ground clearance outside the illustrated pitch plane.",
        "If used for RL, model sensor latency/noise/dropouts and saturating Hall displacement; this file does not train a policy."]
    (OUT / "scene.json").write_text(json.dumps(scene, indent=2)+"\n")
    (OUT / "design_spec.yaml").write_text(yaml.safe_dump(spec, sort_keys=False, allow_unicode=True))
    print(json.dumps({"out": str(OUT), "meshCount": len(meshes), "optics": report}, indent=2))
    if not report["passed"]:
        raise RuntimeError("Optical/ground packaging check failed; inspect optical_report.json and fix before publishing")


if __name__ == "__main__":
    main()
