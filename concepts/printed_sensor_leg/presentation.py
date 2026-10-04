"""Write BuildViz presentation metadata from the actual generated leg scene.

This adds no geometry and does not publish. Service offsets, wire paths, part
references and validation statements come from the generator's own outputs.
"""
from __future__ import annotations

import json
from pathlib import Path
import numpy as np

from concepts.sensor_fusion_foot import make_sensor_foot as foot


DIAGRAM_NAME = "hexapod-printed-sensor-leg"
VERSION = "v1"
LEFT = [-55.0, 0.0, 0.0]
RIGHT = [60.0, 0.0, 0.0]
INK = "#e2e8f0"
AMBER = "#f5ad65"
TEAL = "#65d6b7"
BLUE = "#60c7f5"
YELLOW = "#f8d878"


def _add(a, b):
    return [round(float(x + y), 5) for x, y in zip(a, b)]


def _service_offsets(scene):
    """Use the same exploded pose as the mechanical model, including parents."""
    pose = next(p for p in scene["poses"] if p["id"] == "service")
    joints = {j["id"]: j for j in scene["joints"]}
    values = pose["jointValues"]

    def offset(joint_id):
        joint = joints[joint_id]
        value = float(values.get(joint_id, joint.get("home", 0)))
        vector = [0.0, 0.0, 0.0]
        if joint["type"] == "prismatic":
            vector = [value * float(v) for v in joint["axis"]]
        elif value:
            raise ValueError("Presentation service pose requires zero revolute angles; author its rotations before publishing.")
        if joint.get("parent"):
            vector = _add(vector, offset(joint["parent"]))
        return vector

    return {instance: offset(joint_id) for joint_id, joint in joints.items()
            for instance in joint.get("instances", [])}


def _validation_text(report):
    moving = report["movingClearances"]
    maximum = max((item["overlapMm3"] for item in moving), default=0)
    optics = report["optics"]
    text = (f"Generated geometry: {len(report['invalidMeshes'])} invalid meshes; "
            f"maximum sampled moving/fixed overlap {maximum:.4f} mm³. "
            f"Optical/ground gate {'passed' if optics['passed'] else 'did not pass'} "
            f"({optics['sampledRaysPerStroke']} rays per tested contact stroke).")
    margins = [float(v) for state in optics.get("states", [])
               for v in state.get("fixedPartClearancesMm", {}).values()]
    if margins:
        text += f" Minimum sampled fixed-part ground margin {min(margins):.2f} mm."
    if "yokeMaterialRemovedMm3" in report:
        text += f" Removed canonical knee material: {report['yokeMaterialRemovedMm3']:.4f} mm³."
    if "cartridgeRemoval" in report:
        removal = max(item["overlapMm3"] for item in report["cartridgeRemoval"])
        text += f" Maximum sampled cartridge-removal overlap: {removal:.4f} mm³."
    text += (" These are the generator's sampled geometric checks. Full robot sweep, "
             "material strength, actual mass, fit, sensing and durability remain unvalidated.")
    if "buildviz" in report:
        checks = report["buildviz"]
        failures = [c for c in checks["checks"] if c["status"] == "fail"]
        warnings = [c["label"] for c in checks["checks"] if c["status"] == "warn" and c["kind"] != "clearance"]
        text += f" BuildViz reports {len(failures)} failed checks."
        if warnings:
            text += " Remaining warnings: " + "; ".join(warnings) + ". Refine and verify these before fabrication."
    return text


def write_presentation(out, scene, report, spec):
    """Write diagram.json and workflow.json for this exact generated revision."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    build_id = scene["buildId"]
    instances = scene["instances"]
    by_part = {i["partType"]: i for i in instances}
    required = {"printed_shank", "service_hatch", "hall_foot_housing", "hall_tpu_tread",
                "sensor_pod_lid", "sensor_custom_pcb", "bmi270", "vl53l4cd", "hall_tube_bushing"}
    missing = required - by_part.keys()
    if missing:
        raise ValueError(f"Printed-leg presentation is missing parts {sorted(missing)}; regenerate the full scene first.")
    identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    if any(i["transform"] != identity for i in instances):
        raise ValueError("Part meshes must use the common foot frame; bake instance transforms before writing this presentation.")
    offsets = _service_offsets(scene)
    pod_origin = np.asarray(scene["analysis"]["podOrigin"])
    elements = []
    for side, base in (("assembled", LEFT), ("service", RIGHT)):
        for instance in instances:
            part = instance["partType"]
            shift = offsets.get(instance["id"], [0, 0, 0]) if side == "service" else [0, 0, 0]
            opacity = 0.28 if side == "service" and part == "printed_shank" else 1
            if side == "service" and part == "hall_foot_housing":
                opacity = 0.5
            elements.append({"id": f"{side}-{instance['id']}", "kind": "part",
                "buildId": build_id, "branch": "main", "version": VERSION,
                "part": part, "at": _add(base, shift), "opacity": opacity,
                "color": instance["color"]})

    # Draw the authored routes, including the otherwise hidden beam interior.
    for route in scene["routes"]:
        points = [w["local"] for w in route["waypoints"]]
        for side, base in (("assembled", LEFT), ("service", RIGHT)):
            if side == "service" and route["id"] == "hall-leads":
                continue  # The detached cartridge is unplugged in the service view.
            for index, (a, b) in enumerate(zip(points, points[1:])):
                elements.append({"id": f"{side}-{route['id']}-{index}", "kind": "cylinder",
                    "from": _add(base, a), "to": _add(base, b),
                    "radiusMm": float(route["diameterMm"]) / 2,
                    "color": YELLOW if route["id"] == "internal-harness" else TEAL})

    pivot_z = float(report["kneeAxisZMm"])
    tip_z = pivot_z - float(report["axisToTipAxialMm"])
    elements.extend([
        {"id": "assembled-title", "kind": "text", "at": _add(LEFT, [0, 0, 207]),
         "text": "ASSEMBLED", "sizePx": 19, "color": INK},
        {"id": "service-title", "kind": "text", "at": _add(RIGHT, [0, 0, 207]),
         "text": "OPEN FOR SERVICE", "sizePx": 19, "color": INK},
        {"id": "ground-reference", "kind": "box", "at": _add(LEFT, [10, 0, tip_z - 1]),
         "size": [66, 42, 2], "color": "#24374c", "opacity": 0.65},
        {"id": "knee-axis", "kind": "line",
         "points": [_add(LEFT, [0, -34, pivot_z]), _add(LEFT, [0, 34, pivot_z])],
         "color": INK, "dashed": True},
    ])

    # Annotation anchors describe geometry, never substitute for mechanical datums.
    def callout(identifier, base, anchor, text, offset, color):
        elements.append({"id": identifier, "kind": "callout", "at": _add(base, anchor),
                         "text": text, "offsetPx": offset, "color": color})

    callout("knee-note", LEFT, [0, -24, pivot_z],
            "Existing knee mounting\nHorn holes and pads retained", [-180, -28], INK)
    callout("beam-note", LEFT, [0, -10, 107],
            "One printed shank\nHollow beam + solid knee root", [-180, -12], AMBER)
    callout("sensor-note", LEFT, pod_origin.tolist(),
            "Rigid IMU + laser bay\nPCB layout still to design", [-178, 5], BLUE)
    callout("contact-note", LEFT, [0, 0, 8],
            "Replaceable contact foot\nHall stroke + overload stop", [-180, 28], TEAL)
    callout("wire-note", RIGHT, [0, 0, 91],
            "Protected internal wiring\nVisible through the hollow beam", [145, -28], YELLOW)
    hatch_shift = offsets[by_part["service_hatch"]["id"]]
    callout("hatch-note", RIGHT, _add([0, -10.75, 124], hatch_shift),
            "Removable service hatch\nCable access below the solid root", [150, -75], INK)
    cartridge_shift = offsets[by_part["hall_foot_housing"]["id"]]
    callout("cartridge-note", RIGHT, _add([0, 0, 27], cartridge_shift),
            "Two-bolt foot cartridge\nUnplug Hall leads before removal", [145, 48], TEAL)

    for identifier, part, anchor in (("cartridge", "hall_foot_housing", [0, 0, 34]),
                                     ("hatch", "service_hatch", [0, -11, 124]),
                                     ("pod-lid", "sensor_pod_lid", (pod_origin + foot.POD_R @ [0, 0, 4.6]).tolist())):
        shift = offsets[by_part[part]["id"]]
        elements.append({"id": f"{identifier}-service-guide", "kind": "line",
            "points": [_add(RIGHT, anchor), _add(RIGHT, _add(anchor, shift))],
            "color": "#94a8c2", "dashed": True})

    optical_origin = pod_origin + foot.POD_R @ [0, 0, -foot.PCB_SIZE[2] / 2 - foot.TOF_SIZE[2]]
    optical_direction = foot.POD_R @ [0, 0, -1]
    ray_length = (tip_z + 0.2 - optical_origin[2]) / optical_direction[2]
    optical_target = optical_origin + ray_length * optical_direction
    elements.append({"id": "laser-ray", "kind": "arrow", "from": _add(LEFT, optical_origin),
                     "to": _add(LEFT, optical_target), "shaftMm": 0.35, "color": BLUE})

    validation = _validation_text(report)
    volume = float(report["printedShankVolumeCm3"])
    notes = (
        "PRINTED SENSOR LEG — mechanical and electronics packaging prototype.\n\n"
        "Left: assembled leg. Right: the same real parts in the model's service pose, "
        "with the printed shank transparent, covers removed and complete foot cartridge lowered. "
        "Yellow is the internal power/data route. The detached Hall leads are unplugged.\n\n"
        f"The integrated shank replaces the carbon tube, epoxy socket, separate sensor collar and ties. "
        f"Its modeled volume is {volume:.2f} cm³, including the knee yoke and sensor bay; "
        "this is not an assembled mass or strength result.\n\n"
        "The IMU measures local leg orientation; laser ranging measures distance along one beam; "
        "Hall displacement detects contact/release and saturates at the overload stop. "
        "One beam does not determine terrain slope, and the Hall mechanism does not measure full support force.\n\n"
        f"Actual axial knee-axis-to-tip geometry: {report['axisToTipAxialMm']:.2f} mm unloaded. "
        "The predecessor's 152 mm label described socket-to-tip span and omitted the yoke offset. "
        "Use measured geometry when calibrating the robot.\n\n"
        + validation + "\n\n"
        "Print material, process and fastener fit still need selection and coupons. "
        "The custom PCB, connector, controller and return spring remain design allocations. "
        "Compare assembled bending, torsion, weight and durability with the carbon leg before loaded use."
    )
    diagram = {"name": DIAGRAM_NAME, "title": "Printed sensor leg · integrated structure, serviceable foot",
        "background": "#0c1526", "camera": {"position": [100, -400, 215], "target": [5, 0, 91]},
        "elements": elements, "notes": notes}

    classifications = {}
    printed = {"printed_shank", "service_hatch", "hall_foot_housing", "hall_guided_carriage",
               "hall_sensor_cover", "sensor_pod_lid", "hall_tpu_tread"}
    purchased = {"bmi270", "vl53l4cd", "drv5055_to92", "radial_magnet", "compression_spring_envelope"}
    for name in by_part:
        purpose = spec["parts"][name]["purpose"]
        if name in printed:
            classifications[name] = {"kind": "printed", "material": "TPU 95A candidate" if name == "hall_tpu_tread" else "Tough PA or PETG candidate",
                "notes": purpose + ". Prototype only; validate print process, fit and strength before loaded use.",
                "evidence": "Authored prototype geometry; physical fabrication is not validated."}
        elif name in purchased or "screw" in name:
            classifications[name] = {"kind": "purchased", "notes": purpose + ". Reference envelope; select and verify the exact part before assembly."}
        else:
            classifications[name] = {"kind": "other", "notes": purpose + ". Design allocation; detailed part and fabrication method are not released."}
    classifications["hall_tube_bushing"] = {"kind": "other", "material": "POM preferred; PA fit coupon",
        "notes": "Fit the keyed guide to the printed cartridge sleeve using measured coupons. Verify free return under side load and tilted contact."}
    classifications["printed_shank"]["notes"] += " " + spec["structure"]["printing"]
    workflow = {"schema": 1, "parts": classifications,
        "bom": {"notes": "One-leg packaging study. Modeled envelopes and counts are not a completed procurement or fabrication release.", "items": []},
        "instructions": [
            {"id": "overview", "title": "Annotated assembled and service views",
             "url": f"https://buildviz.cwd1f0-new-cluster.coreweave.app/?diagram={DIAGRAM_NAME}",
             "text": "Inspect the integrated knee and hollow beam, protected wire route, rigid sensor bay and replaceable foot cartridge."},
            {"id": "prototype-assembly", "title": "Prototype assembly and service",
             "text": "Establish material, print orientation and fit coupons first. Preserve the canonical knee horn holes and pads. Fit the keyed bushing, carriage, chosen spring, magnet and TPU tread; verify free return and hard-stop engagement. Bolt the complete cartridge onto the shank. Fit the sensor PCB on the rigid supports. Route the power/data harness through the beam, using the hatch and upper exit for access; provide strain relief. Plug in the fixed Hall leads at the electronics bay, then fit both covers. Unplug those Hall leads before removing the cartridge. No wire crosses the moving shoe. Screw lengths, pilot fits and torque require verification."},
            {"id": "calibration", "title": "Calibrate the three distinct signals",
             "text": f"Measure the built sensor and foot transforms; the generated axial knee-to-tip span is {report['axisToTipAxialMm']:.2f} mm unloaded. Calibrate IMU alignment, laser offset and Hall airborne/contact/release thresholds. Preserve timestamps and validity flags. Check ranging on relevant floors and lighting. Hall displacement saturates at the stop and does not establish total support force."},
            {"id": "validation", "title": "Evidence and remaining work", "text": validation},
        ], "runs": []}
    element_ids = [e["id"] for e in elements]
    if len(element_ids) != len(set(element_ids)):
        raise ValueError("Diagram element IDs must be unique; fix duplicate scene IDs before publishing.")
    for filename, payload in (("diagram.json", diagram), ("workflow.json", workflow)):
        (out / filename).write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    return diagram, workflow
