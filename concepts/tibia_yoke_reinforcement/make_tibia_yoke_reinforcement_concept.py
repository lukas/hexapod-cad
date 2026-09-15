"""CONCEPT: tibia/yoke load-path reinforcement test for STS3215.

This is a BuildViz-only sidecar. It does not change production CAD.

The model is deliberately focused on the parts with evidence behind them:
the printed tibia knee yoke/socket, the carbon tube, the foot boot, and the
optional bought aluminum C-horn path from ``docs/CHORN_VARIANT.md``.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Iterable

import numpy as np
import trimesh
from trimesh.boolean import union as boolean_union
from trimesh.transformations import rotation_matrix


HERE = Path(__file__).resolve().parent
PROTO_DIR = HERE.parents[1]
STL_DIR = HERE / "stl"
LOAD_MANIFEST = (
    PROTO_DIR
    / "artifacts"
    / "strength"
    / "onshape"
    / "mujoco_onshape_study_manifest.json"
)

sys.path.insert(0, str(PROTO_DIR / "tools"))
import hexapod_prototype as hp  # noqa: E402
import make_chorn_variant as ch  # noqa: E402


BUILD_ID = "prototype_sts3215/tibia-yoke-reinforcement-test"

BASE_TOP_Z = 3.0
LANE_Y_CURRENT = 38.0
LANE_Y_CHORN = -38.0
CURRENT_Z_SHIFT = BASE_TOP_Z - (-11.0)
CHORN_Z_SHIFT = BASE_TOP_Z - ch.Z_BOT_PLATE0
CURRENT_TUBE_Z = hp.JOINT_SOCKET_Z + CURRENT_Z_SHIFT
CHORN_TUBE_Z = hp.JOINT_SOCKET_Z + CHORN_Z_SHIFT

TUBE_OD_R = hp.LEG_TUBE_OD / 2.0
TUBE_ID_R = 3.0
SOCKET_OUTER_R = TUBE_OD_R + hp.LEG_TUBE_SOCKET_WALL
FOOT_BOOT_R = 7.0

TUBE_X0 = 62.0
TUBE_X1 = 158.0
BOOT_SOCKET_X0 = 158.0
BOOT_TIP_X1 = 186.0


def _mat(
    tx: float = 0.0,
    ty: float = 0.0,
    tz: float = 0.0,
) -> list[float]:
    return [
        1,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        1,
        0,
        float(tx),
        float(ty),
        float(tz),
        1,
    ]


def _box(extents: tuple[float, float, float], center: tuple[float, float, float]) -> trimesh.Trimesh:
    mesh = trimesh.creation.box(extents=extents)
    mesh.apply_translation(center)
    return mesh


def _cyl_x(
    radius: float,
    length: float,
    center: tuple[float, float, float],
    *,
    sections: int = 96,
) -> trimesh.Trimesh:
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=sections)
    mesh.apply_transform(rotation_matrix(math.pi / 2.0, [0, 1, 0]))
    mesh.apply_translation(center)
    return mesh


def _annular_cylinder_x(
    *,
    outer_r: float,
    inner_r: float,
    x0: float,
    x1: float,
    center_y: float,
    center_z: float,
    sections: int = 128,
) -> trimesh.Trimesh:
    verts: list[tuple[float, float, float]] = []
    for x in (x0, x1):
        for r in (outer_r, inner_r):
            for i in range(sections):
                a = 2.0 * math.pi * i / sections
                verts.append((x, center_y + r * math.cos(a), center_z + r * math.sin(a)))

    def idx(layer: int, ring: int, i: int) -> int:
        return layer * 2 * sections + ring * sections + (i % sections)

    faces: list[tuple[int, int, int]] = []
    for i in range(sections):
        j = i + 1
        # Outer wall.
        faces.append((idx(0, 0, i), idx(1, 0, j), idx(0, 0, j)))
        faces.append((idx(0, 0, i), idx(1, 0, i), idx(1, 0, j)))
        # Inner wall.
        faces.append((idx(0, 1, i), idx(0, 1, j), idx(1, 1, j)))
        faces.append((idx(0, 1, i), idx(1, 1, j), idx(1, 1, i)))
        # Start annulus.
        faces.append((idx(0, 0, i), idx(0, 0, j), idx(0, 1, j)))
        faces.append((idx(0, 0, i), idx(0, 1, j), idx(0, 1, i)))
        # End annulus.
        faces.append((idx(1, 0, i), idx(1, 1, j), idx(1, 0, j)))
        faces.append((idx(1, 0, i), idx(1, 1, i), idx(1, 1, j)))

    return trimesh.Trimesh(vertices=np.asarray(verts, dtype=float), faces=np.asarray(faces), process=True)


def _solid_union(parts: Iterable[trimesh.Trimesh]) -> trimesh.Trimesh:
    try:
        mesh = boolean_union(list(parts), engine="manifold")
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(
            "manifold3d is required for clean BuildViz envelope booleans"
        ) from exc
    mesh.remove_unreferenced_vertices()
    return mesh

def _current_printed_yoke() -> trimesh.Trimesh:
    """Simplified current tibia yoke showing the printed crack-prone path."""
    y = LANE_Y_CURRENT
    return _solid_union(
        [
            _box((40.0, 24.0, 4.0), (22.0, y, BASE_TOP_Z + 2.0)),
            _box((20.0, 24.0, 56.3), (52.0, y, BASE_TOP_Z + 28.15)),
            _box((40.0, 24.0, 4.0), (22.0, y, BASE_TOP_Z + 54.3)),
        ]
    )


def _current_crack_marker() -> trimesh.Trimesh:
    """Red face markers for the documented vertical spine split path."""
    y = LANE_Y_CURRENT
    return _box((1.0, 26.0, 42.0), (41.5, y, BASE_TOP_Z + 28.15))


def _carbon_tube(y: float, z: float) -> trimesh.Trimesh:
    return _annular_cylinder_x(
        outer_r=TUBE_OD_R,
        inner_r=TUBE_ID_R,
        x0=TUBE_X0,
        x1=TUBE_X1,
        center_y=y,
        center_z=z,
    )


def _foot_boot(y: float, z: float) -> trimesh.Trimesh:
    return _cyl_x(FOOT_BOOT_R, BOOT_TIP_X1 - BOOT_SOCKET_X0, ((BOOT_SOCKET_X0 + BOOT_TIP_X1) / 2.0, y, z))


def _load_fixture(y: float, z: float) -> trimesh.Trimesh:
    height = (z - FOOT_BOOT_R) - BASE_TOP_Z
    return _box(
        (36.0, 18.0, height),
        ((BOOT_SOCKET_X0 + BOOT_TIP_X1) / 2.0, y, BASE_TOP_Z + height / 2.0),
    )


def _chorn_reference() -> trimesh.Trimesh:
    """Simplified bought C-horn as one clean U-channel solid."""
    y = LANE_Y_CHORN
    dz = CHORN_Z_SHIFT
    top_z0 = ch.Z_TOP_PLATE0 + dz
    top_z1 = ch.Z_TOP_PLATE1 + dz
    bot_z0 = ch.Z_BOT_PLATE0 + dz
    bot_z1 = ch.Z_BOT_PLATE1 + dz
    return _solid_union(
        [
            _box(
                (ch.CHORN_WEB_X0 + ch.CHORN_WEB_T - 0.5, ch.CHORN_WIDTH, bot_z1 - bot_z0),
                ((0.5 + ch.CHORN_WEB_X0 + ch.CHORN_WEB_T) / 2.0, y, (bot_z0 + bot_z1) / 2.0),
            ),
            _box(
                (ch.CHORN_WEB_T, ch.CHORN_WIDTH, top_z1 - bot_z0),
                (ch.CHORN_WEB_X0 + ch.CHORN_WEB_T / 2.0, y, (bot_z0 + top_z1) / 2.0),
            ),
            _box(
                (ch.CHORN_WEB_X0 + ch.CHORN_WEB_T - 0.5, ch.CHORN_WIDTH, top_z1 - top_z0),
                ((0.5 + ch.CHORN_WEB_X0 + ch.CHORN_WEB_T) / 2.0, y, (top_z0 + top_z1) / 2.0),
            ),
        ]
    )


def _chorn_printed_socket() -> trimesh.Trimesh:
    y = LANE_Y_CHORN
    zc = CHORN_TUBE_Z
    dz = CHORN_Z_SHIFT
    flange_z0 = ch.Z_BOT_PLATE0 + dz
    flange_z1 = ch.Z_TOP_PLATE1 + dz
    return _solid_union(
        [
            _box(
                (ch.FLANGE_T, ch.CHORN_WIDTH, flange_z1 - flange_z0),
                ((ch.FLANGE_X0 + ch.FLANGE_X1) / 2.0, y, (flange_z0 + flange_z1) / 2.0),
            ),
            _box(
                (62.0 - (ch.FLANGE_X1 - 0.1), ch.CHORN_WIDTH, 2.0 * SOCKET_OUTER_R),
                (((ch.FLANGE_X1 - 0.1) + 62.0) / 2.0, y, zc),
            ),
        ]
    )


def _scene_instance(
    iid: str,
    mesh_id: str,
    name: str,
    part_type: str,
    color: str,
    *,
    role: str,
    focus_group: str,
) -> dict:
    return {
        "id": iid,
        "meshId": mesh_id,
        "name": name,
        "partType": part_type,
        "role": role,
        "focusGroup": focus_group,
        "joint": "knee",
        "leg": 0,
        "cots": part_type in {"aluminum_c_horn_reference", "carbon_tube"},
        "color": color,
        "transform": _mat(),
    }


def _load_summary() -> dict:
    fallback = {
        "source": "artifacts/strength/onshape/onshape_mujoco_weak_spot_note_2026-08-25.md",
        "available": False,
        "standing_peak_single_foot_normal_N": 39.481,
        "walking_peak_single_foot_normal_N": 44.6966,
        "walking_peak_force_vector_N": [-5.1222, -1.5464, 44.6966],
        "servo_torque_envelope_Nm": 2.2,
    }
    if not LOAD_MANIFEST.exists():
        return fallback

    manifest = json.loads(LOAD_MANIFEST.read_text())
    top = []
    for scenario in manifest.get("scenarios", []):
        peak = scenario.get("peak", {})
        top.append(
            {
                "id": scenario.get("id"),
                "time_s": scenario.get("time_s"),
                "max_single_foot_normal_N": peak.get("maxSingleFootNormal_N"),
                "total_foot_normal_N": peak.get("totalFootNormal_N"),
                "max_joint_abs_torque_Nm": peak.get("maxJointAbsTorque_Nm"),
            }
        )
    top.sort(key=lambda row: row.get("max_single_foot_normal_N") or 0.0, reverse=True)
    return {
        **fallback,
        "source": str(LOAD_MANIFEST.relative_to(PROTO_DIR)),
        "available": True,
        "top_scenarios": top[:5],
    }


def _export_meshes() -> list[dict]:
    STL_DIR.mkdir(parents=True, exist_ok=True)
    meshes = {
        "inspection_base.stl": _box((240.0, 116.0, BASE_TOP_Z), (86.0, 0.0, BASE_TOP_Z / 2.0)),
        "current_printed_tibia_yoke.stl": _current_printed_yoke(),
        "current_spine_crack_marker.stl": _current_crack_marker(),
        "chorn_aluminum_path_reference.stl": _chorn_reference(),
        "chorn_printed_tibia_socket.stl": _chorn_printed_socket(),
        "carbon_tube_current.stl": _carbon_tube(LANE_Y_CURRENT, CURRENT_TUBE_Z),
        "carbon_tube_chorn.stl": _carbon_tube(LANE_Y_CHORN, CHORN_TUBE_Z),
        "foot_boot_current.stl": _foot_boot(LANE_Y_CURRENT, CURRENT_TUBE_Z),
        "foot_boot_chorn.stl": _foot_boot(LANE_Y_CHORN, CHORN_TUBE_Z),
        "load_fixture_current.stl": _load_fixture(LANE_Y_CURRENT, CURRENT_TUBE_Z),
        "load_fixture_chorn.stl": _load_fixture(LANE_Y_CHORN, CHORN_TUBE_Z),
    }

    mesh_defs = []
    for file_name, mesh in meshes.items():
        mesh.remove_unreferenced_vertices()
        path = STL_DIR / file_name
        mesh.export(path)
        mesh_defs.append({"id": f"stl:{path.stem}", "name": file_name, "url": f"stl/{file_name}"})
    return mesh_defs


def build_scene() -> dict:
    mesh_defs = _export_meshes()
    instances = [
        _scene_instance(
            "fixture-inspection-base",
            "stl:inspection_base",
            "inspection base tying the two comparison load paths together",
            "inspection_fixture",
            "#59636e",
            role="fixture",
            focus_group="shared_test_fixture",
        ),
        _scene_instance(
            "current-printed-yoke",
            "stl:current_printed_tibia_yoke",
            "current printed tibia knee yoke with doubled spine/socket path",
            "current_printed_tibia_yoke",
            "#4d78a8",
            role="current_design",
            focus_group="current_printed_path",
        ),
        _scene_instance(
            "current-spine-crack-marker",
            "stl:current_spine_crack_marker",
            "documented vertical spine crack path markers",
            "stress_marker",
            "#e0332f",
            role="analysis_marker",
            focus_group="current_printed_path",
        ),
        _scene_instance(
            "current-carbon-tube",
            "stl:carbon_tube_current",
            "current tibia carbon tube",
            "carbon_tube",
            "#2e3a46",
            role="load_path",
            focus_group="current_printed_path",
        ),
        _scene_instance(
            "current-foot-boot",
            "stl:foot_boot_current",
            "current TPU foot boot where MuJoCo ground reaction enters",
            "foot_boot",
            "#6cb7cb",
            role="load_input",
            focus_group="current_printed_path",
        ),
        _scene_instance(
            "current-load-fixture",
            "stl:load_fixture_current",
            "walking peak load contact fixture: 44.6966 N single-foot normal",
            "load_fixture",
            "#f0a33a",
            role="fixture",
            focus_group="current_printed_path",
        ),
        _scene_instance(
            "chorn-aluminum-path",
            "stl:chorn_aluminum_path_reference",
            "bought aluminum C-horn load-path envelope",
            "aluminum_c_horn_reference",
            "#d9dde2",
            role="reinforcement",
            focus_group="c_horn_path",
        ),
        _scene_instance(
            "chorn-printed-socket",
            "stl:chorn_printed_tibia_socket",
            "printed C-horn tibia socket envelope, same tube mouth position",
            "chorn_printed_tibia_socket",
            "#2f65b7",
            role="reinforcement",
            focus_group="c_horn_path",
        ),
        _scene_instance(
            "chorn-carbon-tube",
            "stl:carbon_tube_chorn",
            "C-horn variant tibia carbon tube",
            "carbon_tube",
            "#2e3a46",
            role="load_path",
            focus_group="c_horn_path",
        ),
        _scene_instance(
            "chorn-foot-boot",
            "stl:foot_boot_chorn",
            "C-horn variant TPU foot boot, unchanged contact geometry",
            "foot_boot",
            "#6cb7cb",
            role="load_input",
            focus_group="c_horn_path",
        ),
        _scene_instance(
            "chorn-load-fixture",
            "stl:load_fixture_chorn",
            "walking peak load contact fixture: 44.6966 N single-foot normal",
            "load_fixture",
            "#f0a33a",
            role="fixture",
            focus_group="c_horn_path",
        ),
    ]

    return {
        "name": "STS3215 tibia/yoke reinforcement load-path test",
        "source": "make_tibia_yoke_reinforcement_concept.py",
        "buildId": BUILD_ID,
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": [86.0, 0.0, 28.0],
        "meshes": mesh_defs,
        "instances": instances,
        "checksConfig": {
            "overlapMm3": 50.0,
            "pitchMm": 0.25,
            "partDensitiesGCm3": {
                "inspection_fixture": 0.05,
                "current_printed_tibia_yoke": 0.62,
                "stress_marker": 0.05,
                "carbon_tube": 1.55,
                "foot_boot": 1.20,
                "load_fixture": 0.05,
                "aluminum_c_horn_reference": 2.70,
                "chorn_printed_tibia_socket": 0.62,
            },
        },
        "analysis": {
            "purpose": "Focus reinforcement work on the evidence-backed tibia/yoke/foot load path instead of the not-yet-proven chassis path.",
            "loadSummary": _load_summary(),
            "priority": [
                "Highest risk: tibia load path at the knee-side printed socket/yoke transition.",
                "Next: foot boot to carbon-tube interface where the MuJoCo ground reaction enters.",
                "Then: hip/knee horn interfaces and clamp ears under the 2.2 N*m servo torque envelope.",
                "Chassis/yaw pocket remains an FEA cleanup target, but current evidence does not make it the first reinforcement job.",
            ],
            "conceptComparison": {
                "current_printed_path": "Shows the current printed yoke/socket and marks the documented vertical spine split path around the socket tee.",
                "c_horn_path": "Moves the clevis plates into bought aluminum and leaves a smaller printed socket/flange adapter at the same tube-mouth position.",
            },
        },
    }


def main() -> None:
    scene = build_scene()
    (HERE / "scene.json").write_text(json.dumps(scene, indent=2) + "\n")
    (HERE / "load_case_summary.json").write_text(
        json.dumps(scene["analysis"]["loadSummary"], indent=2) + "\n"
    )
    print(f"Wrote {HERE / 'scene.json'}")
    print(f"Wrote {HERE / 'load_case_summary.json'}")
    print(f"Wrote {STL_DIR}")


if __name__ == "__main__":
    main()
