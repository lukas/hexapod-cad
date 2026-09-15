#!/usr/bin/env python3
"""Build STEP assembly-view compounds from the BuildViz robot scene.

The individual STEP exports are printable-part frames. BuildViz scene
transforms are assembly-local frames. For parts where those differ, this file
uses an assembly-frame BREP builder so the composed robot view matches the
verified scene instead of double-applying a print-pose transform.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections import Counter
from pathlib import Path
from typing import Callable

import numpy as np
from build123d import Color, Compound, Plane, export_step, export_stl

THIS_DIR = Path(__file__).resolve().parent
PROTO_DIR = THIS_DIR.parent
OUT_DIR = THIS_DIR / "out"
STEP_DIR = OUT_DIR / "step"
STL_DIR = OUT_DIR / "stl"

sys.path.insert(0, str(THIS_DIR))

import build_step_first_test as step  # noqa: E402
import hexapod_prototype as hp  # noqa: E402


PartBuilder = Callable[[], object]

FASTENER_PRIM_RE = re.compile(
    r"^stl:fastener_prim_(?P<diameter>[0-9.]+)_(?P<length>[0-9.]+)$"
)


def _hex_color(value: str | None) -> Color | None:
    if not value or not value.startswith("#") or len(value) != 7:
        return None
    r = int(value[1:3], 16) / 255.0
    g = int(value[3:5], 16) / 255.0
    b = int(value[5:7], 16) / 255.0
    return Color(r, g, b)


def _buildviz_matrix(flat: list[float] | None) -> np.ndarray:
    """BuildViz stores Three.js Matrix4 elements: column-major 4x4."""
    if not flat or len(flat) != 16:
        return np.eye(4)
    return np.asarray(flat, dtype=float).reshape(4, 4).T


def _location_from_matrix(matrix: np.ndarray):
    return Plane(
        origin=tuple(float(v) for v in matrix[:3, 3]),
        x_dir=tuple(float(v) for v in matrix[:3, 0]),
        z_dir=tuple(float(v) for v in matrix[:3, 2]),
    ).location


def make_tibia_tube_assembly() -> object:
    """Solid CF tube visual in the same local frame as BuildViz tibia_tube."""
    length = hp.TIBIA_LENGTH - 8.0
    start_x = -hp.SERVO_OUTPUT_X + hp._YOKE_SOCKET_X
    center = (
        start_x + length / 2.0,
        hp.JOINT_HORN_TOP_Z - hp.JOINT_SOCKET_Z,
        0.0,
    )
    return step._cyl_x(hp.LEG_TUBE_OD / 2.0, length, center)


def make_primitive_fastener(diameter_mm: float, length_mm: float) -> object:
    """Simple screw proxy matching the BuildViz fastener local frame.

    BuildViz primitive fasteners use local +Z along the screw shaft, with the
    origin on the outer head face. This BREP is intentionally simple; it is
    only for simulation/connectivity exports, not manufacturing drawings.
    """
    head_t = max(0.6 * diameter_mm, 1.0)
    head = step._cyl_z(
        diameter_mm * 0.9,
        head_t,
        (0.0, 0.0, head_t / 2.0),
    )
    shank = step._cyl_z(
        diameter_mm / 2.0,
        length_mm,
        (0.0, 0.0, head_t + length_mm / 2.0),
    )
    return step._union(head, shank)


ASSEMBLY_BUILDERS: dict[str, PartBuilder] = {
    "stl:chassis_bottom": step.make_chassis_bottom,
    "stl:chassis_top": step.make_chassis_top,
    "stl:switch_holster": step.make_switch_holster,
    "stl:disc_horn": step.make_disc_horn,
    "stl:coxa_link": step.make_coxa_link,
    # BuildViz places femur_link in the make_femur_link_part local frame and
    # carries the joint pitch transform in the instance matrix.
    "stl:femur_link": step.make_femur_link_part,
    "stl:tibia_knee_yoke": step.make_tibia_knee_yoke,
    "stl:tibia_tube": make_tibia_tube_assembly,
    "stl:foot_boot": step.make_foot_boot,
    "stl:servo_clamp_cap": step.make_servo_clamp_cap,
    "stl:yaw_bearing_cap": step.make_yaw_bearing_cap,
    "stl:yaw_bearing_lower": lambda: step.make_yaw_bearing("lower"),
    "stl:yaw_bearing_upper": lambda: step.make_yaw_bearing("upper"),
    "stl:yaw_servo_retainer": step.make_yaw_servo_retainer,
    "stl:servo_body": step.make_servo_body,
}


BASE_INCLUDED = {
    "stl:chassis_bottom",
    "stl:chassis_top",
    "stl:switch_holster",
    "stl:coxa_link",
    "stl:femur_link",
    "stl:tibia_knee_yoke",
    "stl:tibia_tube",
    "stl:foot_boot",
    "stl:servo_clamp_cap",
    "stl:yaw_bearing_cap",
    "stl:yaw_bearing_lower",
    "stl:yaw_bearing_upper",
    "stl:yaw_servo_retainer",
}


def _bbox(compound: object) -> dict:
    bb = compound.bounding_box()
    lo = np.array([bb.min.X, bb.min.Y, bb.min.Z], dtype=float)
    hi = np.array([bb.max.X, bb.max.Y, bb.max.Z], dtype=float)
    return {
        "min": [round(float(v), 4) for v in lo],
        "max": [round(float(v), 4) for v in hi],
        "size": [round(float(v), 4) for v in hi - lo],
    }


def _place_instance(inst: dict, cache: dict[str, object]) -> object | None:
    mesh_id = inst.get("meshId")
    builder = ASSEMBLY_BUILDERS.get(mesh_id)
    if mesh_id not in cache:
        if builder is not None:
            cache[mesh_id] = builder()
        else:
            match = FASTENER_PRIM_RE.match(mesh_id or "")
            if match is None:
                return None
            cache[mesh_id] = make_primitive_fastener(
                float(match.group("diameter")),
                float(match.group("length")),
            )
    shape = _location_from_matrix(_buildviz_matrix(inst.get("transform"))) * cache[mesh_id]
    shape.label = inst.get("name") or mesh_id.replace("stl:", "")
    color = _hex_color(inst.get("color"))
    if color is not None:
        shape.color = color
    return shape


def build_assembly(*, include_servo_bodies: bool) -> dict:
    included = set(BASE_INCLUDED)
    if include_servo_bodies:
        included.add("stl:servo_body")

    scene_path = PROTO_DIR / "full_robot_viz" / "scene.json"
    scene = json.loads(scene_path.read_text())
    cache: dict[str, object] = {}
    placed = []
    counts: Counter[str] = Counter()
    skipped: Counter[str] = Counter()

    for inst in scene.get("instances", []):
        mesh_id = inst.get("meshId")
        if mesh_id not in included:
            skipped[mesh_id] += 1
            continue
        shape = _place_instance(inst, cache)
        if shape is None:
            skipped[mesh_id] += 1
            continue
        placed.append(shape)
        counts[mesh_id] += 1

    variant = "assembled_robot_with_servos" if include_servo_bodies else "assembled_robot_corrected"
    compound = Compound(placed, label=variant)
    step_path = STEP_DIR / f"{variant}.step"
    stl_path = STL_DIR / f"{variant}.stl"
    STEP_DIR.mkdir(parents=True, exist_ok=True)
    STL_DIR.mkdir(parents=True, exist_ok=True)
    export_step(compound, step_path)
    export_stl(compound, stl_path)

    manifest = {
        "variant": variant,
        "sourceScene": str(scene_path.relative_to(PROTO_DIR)),
        "step": str(step_path.relative_to(THIS_DIR)),
        "stl": str(stl_path.relative_to(THIS_DIR)),
        "includedInstances": len(placed),
        "countsByMeshId": dict(counts),
        "skippedMeshIds": {k: v for k, v in skipped.items() if k},
        "bboxMm": _bbox(compound),
    }
    manifest_path = OUT_DIR / f"{variant}_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


YAW_FOCUS_INCLUDED = {
    "stl:chassis_bottom",
    "stl:coxa_link",
    "stl:yaw_bearing_cap",
    "stl:yaw_bearing_lower",
    "stl:yaw_bearing_upper",
    "stl:disc_horn",
}


ONE_LEG_LOAD_INCLUDED = {
    "stl:chassis_bottom",
    "stl:chassis_top",
    "stl:coxa_link",
    "stl:disc_horn",
    "stl:femur_link",
    "stl:foot_boot",
    "stl:servo_clamp_cap",
    "stl:tibia_knee_yoke",
    "stl:tibia_tube",
    "stl:yaw_bearing_cap",
    "stl:yaw_bearing_lower",
    "stl:yaw_bearing_upper",
    "stl:yaw_servo_retainer",
}


def build_connected_one_leg_load_path(*, leg_index: int) -> dict:
    """Export one leg with servo bodies and fastener proxies for simulation.

    This is less tidy visually than ``build_one_leg_load_path`` but it keeps
    the physical bridges that Onshape needs for a believable load path.
    """
    scene_path = PROTO_DIR / "full_robot_viz" / "scene.json"
    scene = json.loads(scene_path.read_text())
    cache: dict[str, object] = {}
    placed = []
    counts: Counter[str] = Counter()
    source_instances = []
    skipped: Counter[str] = Counter()

    for inst in scene.get("instances", []):
        mesh_id = inst.get("meshId")
        keep = mesh_id in {"stl:chassis_bottom", "stl:chassis_top"}
        if inst.get("leg") == leg_index:
            keep = True
        if not keep:
            continue

        shape = _place_instance(inst, cache)
        if shape is None:
            skipped[mesh_id] += 1
            continue
        placed.append(shape)
        counts[mesh_id] += 1
        source_instances.append(
            {
                "id": inst.get("id"),
                "name": inst.get("name"),
                "meshId": mesh_id,
                "role": inst.get("role"),
                "leg": inst.get("leg"),
                "joint": inst.get("joint"),
            }
        )

    variant = f"connected_one_leg_load_path_L{leg_index}"
    compound = Compound(placed, label=variant)
    step_path = STEP_DIR / f"{variant}.step"
    stl_path = STL_DIR / f"{variant}.stl"
    STEP_DIR.mkdir(parents=True, exist_ok=True)
    STL_DIR.mkdir(parents=True, exist_ok=True)
    export_step(compound, step_path)
    export_stl(compound, stl_path)

    manifest = {
        "variant": variant,
        "sourceScene": str(scene_path.relative_to(PROTO_DIR)),
        "step": str(step_path.relative_to(THIS_DIR)),
        "stl": str(stl_path.relative_to(THIS_DIR)),
        "includedInstances": len(placed),
        "countsByMeshId": dict(counts),
        "sourceInstances": source_instances,
        "skippedMeshIds": {k: v for k, v in skipped.items() if k},
        "bboxMm": _bbox(compound),
    }
    manifest_path = OUT_DIR / f"{variant}_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def build_connected_full_load_path() -> dict:
    """Export the full chassis plus all six connected leg load paths.

    This is the Onshape-simulation handoff model: every leg-local instance
    that has a supported BREP builder, plus primitive fastener proxies, is
    placed from the verified BuildViz scene. Non-structural electronics are
    intentionally skipped unless they are already part of the chassis plate
    geometry.
    """
    scene_path = PROTO_DIR / "full_robot_viz" / "scene.json"
    scene = json.loads(scene_path.read_text())
    cache: dict[str, object] = {}
    placed = []
    counts: Counter[str] = Counter()
    source_instances = []
    skipped: Counter[str] = Counter()

    for inst in scene.get("instances", []):
        mesh_id = inst.get("meshId")
        keep = mesh_id in {"stl:chassis_bottom", "stl:chassis_top"}
        if inst.get("leg") is not None:
            keep = True
        if not keep:
            continue

        shape = _place_instance(inst, cache)
        if shape is None:
            skipped[mesh_id] += 1
            continue
        placed.append(shape)
        counts[mesh_id] += 1
        source_instances.append(
            {
                "id": inst.get("id"),
                "name": inst.get("name"),
                "meshId": mesh_id,
                "role": inst.get("role"),
                "leg": inst.get("leg"),
                "joint": inst.get("joint"),
            }
        )

    variant = "connected_full_robot_load_path"
    compound = Compound(placed, label=variant)
    step_path = STEP_DIR / f"{variant}.step"
    stl_path = STL_DIR / f"{variant}.stl"
    STEP_DIR.mkdir(parents=True, exist_ok=True)
    STL_DIR.mkdir(parents=True, exist_ok=True)
    export_step(compound, step_path)
    export_stl(compound, stl_path)

    manifest = {
        "variant": variant,
        "sourceScene": str(scene_path.relative_to(PROTO_DIR)),
        "step": str(step_path.relative_to(THIS_DIR)),
        "stl": str(stl_path.relative_to(THIS_DIR)),
        "includedInstances": len(placed),
        "countsByMeshId": dict(counts),
        "sourceInstances": source_instances,
        "skippedMeshIds": {k: v for k, v in skipped.items() if k},
        "bboxMm": _bbox(compound),
    }
    manifest_path = OUT_DIR / f"{variant}_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def build_one_leg_load_path(*, leg_index: int) -> dict:
    """Export a compact one-leg force-path assembly for simulation setup."""
    scene_path = PROTO_DIR / "full_robot_viz" / "scene.json"
    scene = json.loads(scene_path.read_text())
    cache: dict[str, object] = {}
    placed = []
    counts: Counter[str] = Counter()
    source_instances = []
    skipped: Counter[str] = Counter()

    for inst in scene.get("instances", []):
        mesh_id = inst.get("meshId")
        keep = mesh_id in {"stl:chassis_bottom", "stl:chassis_top"}
        if mesh_id in ONE_LEG_LOAD_INCLUDED and inst.get("leg") == leg_index:
            keep = True
        if not keep:
            if mesh_id in ONE_LEG_LOAD_INCLUDED:
                skipped[mesh_id] += 1
            continue

        shape = _place_instance(inst, cache)
        if shape is None:
            skipped[mesh_id] += 1
            continue
        placed.append(shape)
        counts[mesh_id] += 1
        source_instances.append(
            {
                "id": inst.get("id"),
                "name": inst.get("name"),
                "meshId": mesh_id,
                "leg": inst.get("leg"),
                "joint": inst.get("joint"),
            }
        )

    variant = f"one_leg_load_path_L{leg_index}"
    compound = Compound(placed, label=variant)
    step_path = STEP_DIR / f"{variant}.step"
    stl_path = STL_DIR / f"{variant}.stl"
    STEP_DIR.mkdir(parents=True, exist_ok=True)
    STL_DIR.mkdir(parents=True, exist_ok=True)
    export_step(compound, step_path)
    export_stl(compound, stl_path)

    manifest = {
        "variant": variant,
        "sourceScene": str(scene_path.relative_to(PROTO_DIR)),
        "step": str(step_path.relative_to(THIS_DIR)),
        "stl": str(stl_path.relative_to(THIS_DIR)),
        "includedInstances": len(placed),
        "countsByMeshId": dict(counts),
        "sourceInstances": source_instances,
        "skippedCandidateMeshIds": {k: v for k, v in skipped.items() if k},
        "bboxMm": _bbox(compound),
    }
    manifest_path = OUT_DIR / f"{variant}_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def build_yaw_bearing_focus(*, leg_index: int) -> dict:
    """Export a compact yaw-stack assembly for one leg plus chassis_bottom."""
    scene_path = PROTO_DIR / "full_robot_viz" / "scene.json"
    scene = json.loads(scene_path.read_text())
    cache: dict[str, object] = {}
    placed = []
    counts: Counter[str] = Counter()
    source_instances = []
    skipped: Counter[str] = Counter()

    for inst in scene.get("instances", []):
        mesh_id = inst.get("meshId")
        keep = mesh_id == "stl:chassis_bottom"
        if mesh_id in YAW_FOCUS_INCLUDED and inst.get("leg") == leg_index:
            keep = True
            if mesh_id == "stl:disc_horn" and inst.get("joint") != "yaw":
                keep = False
        if not keep:
            if mesh_id in YAW_FOCUS_INCLUDED:
                skipped[mesh_id] += 1
            continue

        shape = _place_instance(inst, cache)
        if shape is None:
            skipped[mesh_id] += 1
            continue
        placed.append(shape)
        counts[mesh_id] += 1
        source_instances.append(
            {
                "id": inst.get("id"),
                "name": inst.get("name"),
                "meshId": mesh_id,
                "leg": inst.get("leg"),
                "joint": inst.get("joint"),
            }
        )

    variant = f"yaw_bearing_focus_L{leg_index}"
    compound = Compound(placed, label=variant)
    step_path = STEP_DIR / f"{variant}.step"
    stl_path = STL_DIR / f"{variant}.stl"
    STEP_DIR.mkdir(parents=True, exist_ok=True)
    STL_DIR.mkdir(parents=True, exist_ok=True)
    export_step(compound, step_path)
    export_stl(compound, stl_path)

    manifest = {
        "variant": variant,
        "sourceScene": str(scene_path.relative_to(PROTO_DIR)),
        "step": str(step_path.relative_to(THIS_DIR)),
        "stl": str(stl_path.relative_to(THIS_DIR)),
        "includedInstances": len(placed),
        "countsByMeshId": dict(counts),
        "sourceInstances": source_instances,
        "skippedCandidateMeshIds": {k: v for k, v in skipped.items() if k},
        "bboxMm": _bbox(compound),
    }
    manifest_path = OUT_DIR / f"{variant}_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--with-servos",
        action="store_true",
        help="Include STS3215 servo-body envelopes for clamp-cap diagnosis.",
    )
    parser.add_argument(
        "--yaw-bearing-focus",
        action="store_true",
        help="Export one yaw-bearing stack plus chassis_bottom instead of the full robot.",
    )
    parser.add_argument(
        "--one-leg-load-path",
        action="store_true",
        help="Export one full leg force path plus chassis plates for simulation.",
    )
    parser.add_argument(
        "--connected-one-leg-load-path",
        action="store_true",
        help=(
            "Export one leg plus chassis plates, servo bodies, and primitive "
            "fastener proxies for a connected simulation load path."
        ),
    )
    parser.add_argument(
        "--connected-full-load-path",
        action="store_true",
        help="Export full chassis plus all six connected leg load paths.",
    )
    parser.add_argument(
        "--leg-index",
        type=int,
        default=0,
        help="Leg index to use with --yaw-bearing-focus or --one-leg-load-path.",
    )
    args = parser.parse_args()

    if args.connected_full_load_path:
        manifest = build_connected_full_load_path()
    elif args.connected_one_leg_load_path:
        manifest = build_connected_one_leg_load_path(leg_index=args.leg_index)
    elif args.one_leg_load_path:
        manifest = build_one_leg_load_path(leg_index=args.leg_index)
    elif args.yaw_bearing_focus:
        manifest = build_yaw_bearing_focus(leg_index=args.leg_index)
    else:
        manifest = build_assembly(include_servo_bodies=args.with_servos)
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
