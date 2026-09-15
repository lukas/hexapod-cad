#!/usr/bin/env python3
"""Prepare repeatable MuJoCo -> Onshape structural study inputs.

This runner keeps the handoff explicit:

1. replay stand/walk/rise/lower in MuJoCo and write peak load frames;
2. export a full connected STEP load path with chassis + all six legs;
3. bind those together into an Onshape setup manifest and Markdown sheet;
4. optionally upload the STEP into the configured Onshape document.

Onshape Simulation still needs a final UI solve/setup pass. The public REST API
is reliable for import/assembly/metadata, but the Simulation force/fixture rows
we have seen are UI-stateful enough that this script records named selections
and load vectors instead of pretending the solve is fully headless.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any


THIS_DIR = Path(__file__).resolve().parent
PROTO_DIR = THIS_DIR.parent
ROOT = PROTO_DIR.parents[1]
CAD_TEST = PROTO_DIR / "cad_step_test"
ARTIFACT_DIR = PROTO_DIR / "artifacts" / "strength" / "onshape"
DEFAULT_LOADS = PROTO_DIR / "artifacts" / "strength" / "mujoco_loads.json"
DEFAULT_STEP_MANIFEST = (
    CAD_TEST / "out" / "connected_full_robot_load_path_manifest.json"
)
DEFAULT_OUT_JSON = ARTIFACT_DIR / "mujoco_onshape_study_manifest.json"
DEFAULT_OUT_MD = ARTIFACT_DIR / "mujoco_onshape_study.md"
N_TO_LBF = 0.2248089431


def _rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path.resolve())


def _run(cmd: list[str]) -> None:
    print("[onshape-study] " + " ".join(str(c) for c in cmd))
    subprocess.run(cmd, cwd=ROOT, check=True)


def _read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def _round_vec(values: list[float] | tuple[float, ...], digits: int = 4) -> list[float]:
    return [round(float(v), digits) for v in values]


def _mag(values: list[float] | tuple[float, ...]) -> float:
    return math.sqrt(sum(float(v) * float(v) for v in values))


def _scene_targets(scene: dict[str, Any]) -> dict[str, Any]:
    by_leg: dict[int, dict[str, dict[str, Any]]] = {i: {} for i in range(6)}
    standoffs = []
    for inst in scene.get("instances", []):
        leg = inst.get("leg")
        mesh_id = inst.get("meshId")
        if leg is not None and mesh_id in {
            "stl:foot_boot",
            "stl:tibia_tube",
            "stl:tibia_knee_yoke",
            "stl:yaw_bearing_lower",
            "stl:coxa_link",
        }:
            by_leg[int(leg)][mesh_id] = {
                "id": inst.get("id"),
                "name": inst.get("name"),
                "meshId": mesh_id,
                "centroid_mm": _round_vec(inst.get("centroid", [0, 0, 0]), 3),
            }
        if mesh_id == "stl:chassis_standoff":
            standoffs.append(
                {
                    "id": inst.get("id"),
                    "name": inst.get("name"),
                    "centroid_mm": _round_vec(inst.get("centroid", [0, 0, 0]), 3),
                }
            )
    return {"legs": by_leg, "chassisStandoffs": standoffs}


def _fallback_frame_from_top_foot(run: dict[str, Any]) -> list[dict[str, Any]]:
    frames = []
    for event in run.get("top_foot_events", [])[:3]:
        feet = []
        for leg in range(6):
            if leg == int(event["leg"]):
                force = event.get("force_world_N", [0.0, 0.0, event["normal_N"]])
                point = event.get("point_world_m", [0.0, 0.0, 0.0])
                normal = event.get("normal_N", 0.0)
            else:
                force = [0.0, 0.0, 0.0]
                point = [0.0, 0.0, 0.0]
                normal = 0.0
            feet.append(
                {
                    "leg": leg,
                    "normal_N": normal,
                    "vertical_N": force[2],
                    "horizontal_N": _mag(force[:2]),
                    "force_world_N": force,
                    "point_world_m": point,
                    "contact_count": 1 if normal else 0,
                    "other_geoms": [],
                }
            )
        frames.append(
            {
                "selector": "legacy_top_foot_event",
                "t_s": event.get("t_s", 0.0),
                "max_single_foot_normal_N": event.get("normal_N", 0.0),
                "total_foot_normal_N": event.get("normal_N", 0.0),
                "max_joint_abs_torque_Nm": 0.0,
                "feet": feet,
                "top_joints": [],
            }
        )
    return frames


def _frame_events(run: dict[str, Any]) -> list[dict[str, Any]]:
    frames = run.get("top_frame_events") or []
    if frames:
        return frames
    return _fallback_frame_from_top_foot(run)


def _load_rows_for_frame(
    frame: dict[str, Any],
    targets: dict[str, Any],
    *,
    load_factor: float,
    target_mesh_id: str,
    min_normal_N: float,
) -> list[dict[str, Any]]:
    rows = []
    for foot in frame.get("feet", []):
        if float(foot.get("normal_N", 0.0)) < min_normal_N:
            continue
        leg = int(foot["leg"])
        target = targets["legs"].get(leg, {}).get(target_mesh_id)
        if not target:
            continue
        force = [load_factor * float(v) for v in foot.get("force_world_N", [0, 0, 0])]
        rows.append(
            {
                "leg": leg,
                "target": target,
                "force_N": _round_vec(force, 4),
                "magnitude_N": round(_mag(force), 4),
                "magnitude_lbf": round(_mag(force) * N_TO_LBF, 3),
                "sourceContactPointWorld_m": _round_vec(
                    foot.get("point_world_m", [0, 0, 0]), 6
                ),
                "sourceNormal_N": round(float(foot.get("normal_N", 0.0)), 4),
            }
        )
    return rows


def build_manifest(
    *,
    loads: dict[str, Any],
    step_manifest: dict[str, Any],
    scene: dict[str, Any],
    load_factor: float,
    min_normal_N: float,
) -> dict[str, Any]:
    targets = _scene_targets(scene)
    scenarios = []
    for run in loads.get("runs", []):
        for frame in _frame_events(run):
            selector = frame.get("selector", "frame")
            scenario_id = f"{run['name']}__{selector}"
            foot_loads = _load_rows_for_frame(
                frame,
                targets,
                load_factor=load_factor,
                target_mesh_id="stl:foot_boot",
                min_normal_N=min_normal_N,
            )
            yaw_loads = _load_rows_for_frame(
                frame,
                targets,
                load_factor=load_factor,
                target_mesh_id="stl:yaw_bearing_lower",
                min_normal_N=min_normal_N,
            )
            scenarios.append(
                {
                    "id": scenario_id,
                    "sourceRun": run["name"],
                    "sourceSelector": selector,
                    "time_s": frame.get("t_s"),
                    "peak": {
                        "maxSingleFootNormal_N": frame.get("max_single_foot_normal_N"),
                        "totalFootNormal_N": frame.get("total_foot_normal_N"),
                        "maxJointAbsTorque_Nm": frame.get("max_joint_abs_torque_Nm"),
                    },
                    "studies": [
                        {
                            "id": f"{scenario_id}__whole_robot_load_path",
                            "purpose": (
                                "Full connected assembly triage: load all active "
                                "feet from the MuJoCo frame and fixture the chassis "
                                "reference/standoff region to reveal leg-to-chassis "
                                "stress concentrations."
                            ),
                            "fixture": {
                                "type": "fixed_support",
                                "target": "chassis standoff hole rings or fixed_frame_chassis mate connector",
                                "selectionHints": targets["chassisStandoffs"],
                            },
                            "loads": foot_loads,
                        },
                        {
                            "id": f"{scenario_id}__chassis_plate_equivalent",
                            "purpose": (
                                "Chassis-focused triage: apply the same active foot "
                                "reactions at the six yaw-bearing seats so plate, "
                                "standoff, and yaw-pocket hotspots are visible without "
                                "noisy leg contact intersections."
                            ),
                            "fixture": {
                                "type": "fixed_support",
                                "target": "chassis standoff hole rings",
                                "selectionHints": targets["chassisStandoffs"],
                            },
                            "loads": yaw_loads,
                        },
                    ],
                    "topJoints": frame.get("top_joints", []),
                }
            )

    return {
        "schema": "hexapod_mujoco_onshape_static_study_v1",
        "units": {
            "force": "N",
            "moment": "N*m",
            "length": "mm for CAD targets, m for MuJoCo source points",
        },
        "source": {
            "mujocoLoads": _rel(Path(loads.get("_path", DEFAULT_LOADS))),
            "mujocoSchema": loads.get("schema"),
            "loadFactor": load_factor,
            "minNormal_N": min_normal_N,
        },
        "cad": {
            "step": step_manifest.get("step"),
            "stl": step_manifest.get("stl"),
            "variant": step_manifest.get("variant"),
            "includedInstances": step_manifest.get("includedInstances"),
            "countsByMeshId": step_manifest.get("countsByMeshId"),
            "bboxMm": step_manifest.get("bboxMm"),
        },
        "onshape": {
            "automationLevel": "import_and_setup_manifest",
            "note": (
                "Use the generated STEP and these named force/fixture rows for "
                "Onshape Simulation. Do not trust stale UI force rows; recreate "
                "or verify them from this manifest."
            ),
        },
        "selectionTargets": targets,
        "scenarios": scenarios,
    }


def render_markdown(manifest: dict[str, Any]) -> str:
    lines = [
        "# MuJoCo -> Onshape structural study setup",
        "",
        "## CAD model",
        "",
        f"- Variant: `{manifest['cad'].get('variant')}`",
        f"- STEP: `{manifest['cad'].get('step')}`",
        f"- Included instances: {manifest['cad'].get('includedInstances')}",
        "",
        "## How to use in Onshape",
        "",
        "1. Import the STEP or run this script with `--upload`.",
        "2. Create one Simulation study per scenario row below.",
        "3. For the whole-robot study, fixture the chassis standoff rings or a named chassis mate connector and apply the listed loads at each active `foot_boot`.",
        "4. For the chassis-only study, fixture the chassis standoff rings and apply the listed equivalent loads at each active yaw-bearing seat.",
        "5. Recreate the force rows from this file if Onshape shows a stale force with zero direction selections.",
        "",
        "## Scenario summary",
        "",
        "| Scenario | t (s) | Max foot (N) | Total feet (N) | Max joint (N*m) | Active loads |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for scenario in manifest["scenarios"]:
        active = len(scenario["studies"][0]["loads"])
        peak = scenario["peak"]
        lines.append(
            f"| `{scenario['id']}` | {float(scenario.get('time_s') or 0):.2f} "
            f"| {float(peak.get('maxSingleFootNormal_N') or 0):.1f} "
            f"| {float(peak.get('totalFootNormal_N') or 0):.1f} "
            f"| {float(peak.get('maxJointAbsTorque_Nm') or 0):.3f} "
            f"| {active} |"
        )
    lines.extend(["", "## Load rows", ""])
    for scenario in manifest["scenarios"]:
        lines.append(f"### `{scenario['id']}`")
        lines.append("")
        for study in scenario["studies"]:
            lines.append(f"#### `{study['id']}`")
            lines.append("")
            lines.append(f"- Fixture: {study['fixture']['target']}")
            if not study["loads"]:
                lines.append("- No active contact loads above the threshold.")
                lines.append("")
                continue
            lines.append("")
            lines.append("| Leg | Target | Fx | Fy | Fz | Magnitude lbf |")
            lines.append("|---:|---|---:|---:|---:|---:|")
            for row in study["loads"]:
                fx, fy, fz = row["force_N"]
                target = row["target"].get("name") or row["target"].get("id")
                lines.append(
                    f"| L{row['leg']} | `{target}` | {fx:.2f} | {fy:.2f} "
                    f"| {fz:.2f} | {row['magnitude_lbf']:.2f} |"
                )
            lines.append("")
    lines.extend(
        [
            "## Caveats",
            "",
            "- These are static triage studies from dynamic MuJoCo frames, not a validated fatigue or impact certification.",
            "- The chassis-equivalent case is intentionally cleaner than the full assembly; use it to find plate/yaw-pocket/standoff hotspots after contact intersections make the full assembly noisy.",
            "- Treat Onshape stress spikes at imported part intersections as suspect until the assembly contact set is cleaned.",
            "",
        ]
    )
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("stand", "walk", "rise", "lower", "both", "all"),
        default="all",
        help="MuJoCo load replay mode used unless --skip-mujoco is set.",
    )
    parser.add_argument("--loads", type=Path, default=DEFAULT_LOADS)
    parser.add_argument("--step-manifest", type=Path, default=DEFAULT_STEP_MANIFEST)
    parser.add_argument("--out-json", type=Path, default=DEFAULT_OUT_JSON)
    parser.add_argument("--out-md", type=Path, default=DEFAULT_OUT_MD)
    parser.add_argument("--load-factor", type=float, default=1.0)
    parser.add_argument("--min-normal-N", type=float, default=0.25)
    parser.add_argument("--skip-mujoco", action="store_true")
    parser.add_argument("--skip-step", action="store_true")
    parser.add_argument(
        "--upload",
        action="store_true",
        help="Upload the connected full-load-path STEP to the configured Onshape document.",
    )
    parser.add_argument(
        "--upload-out",
        type=Path,
        default=ARTIFACT_DIR / "mujoco_onshape_upload_result.json",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.skip_mujoco:
        _run(
            [
                sys.executable,
                str(THIS_DIR / "extract_mujoco_loads.py"),
                "--mode",
                args.mode,
                "--out",
                str(args.loads),
            ]
        )
    if not args.skip_step:
        _run(
            [
                sys.executable,
                str(CAD_TEST / "build_step_assembly_views.py"),
                "--connected-full-load-path",
            ]
        )

    if not args.loads.is_file():
        raise SystemExit(f"MuJoCo load artifact missing: {args.loads}")
    if not args.step_manifest.is_file():
        raise SystemExit(f"STEP manifest missing: {args.step_manifest}")

    loads = _read_json(args.loads)
    loads["_path"] = str(args.loads)
    step_manifest = _read_json(args.step_manifest)
    scene = _read_json(PROTO_DIR / "full_robot_viz" / "scene.json")

    manifest = build_manifest(
        loads=loads,
        step_manifest=step_manifest,
        scene=scene,
        load_factor=args.load_factor,
        min_normal_N=args.min_normal_N,
    )
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(manifest, indent=2) + "\n")
    args.out_md.write_text(render_markdown(manifest))
    print(f"[onshape-study] wrote {_rel(args.out_json)}")
    print(f"[onshape-study] wrote {_rel(args.out_md)}")

    if args.upload:
        step_path = CAD_TEST / step_manifest["step"]
        upload_script = CAD_TEST / "upload_connected_load_path_to_onshape.py"
        _run(
            [
                sys.executable,
                str(upload_script),
                "--step",
                str(step_path),
                "--manifest",
                str(args.out_json),
                "--import-name",
                "mujoco_full_chassis_load_path",
                "--assembly-name",
                "MuJoCo load simulation - full chassis repeatable",
                "--out",
                str(args.upload_out),
            ]
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
