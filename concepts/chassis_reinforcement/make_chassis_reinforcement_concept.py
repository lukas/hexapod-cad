"""CONCEPT: removable chassis reinforcement kit for the STS3215 hexapod.

This is a BuildViz-only test project.  It does not change production CAD.

The geometry is deliberately conservative: a simplified two-plate chassis
context plus add-on aluminum pieces that can be visualized, checked, and
iterated before any production part is touched.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np
import trimesh


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

BUILD_ID = "prototype_sts3215/chassis-reinforcement-test"

PLATE_R = 118.0
PLATE_T = 2.4
BOTTOM_PLATE_Z = 47.45535707092013
TOP_PLATE_Z = 70.45535707092013
YAW_R = 100.0
STANDOFF_R = 31.11269837220809

LEG_ANGLES_DEG = (30, 90, 150, 210, 270, 330)
STANDOFF_POINTS = (
    (STANDOFF_R, STANDOFF_R),
    (-STANDOFF_R, STANDOFF_R),
    (-STANDOFF_R, -STANDOFF_R),
    (STANDOFF_R, -STANDOFF_R),
)


def _hex_vertices(radius: float) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(math.radians(30 + 60 * i)),
            radius * math.sin(math.radians(30 + 60 * i)),
        )
        for i in range(6)
    ]


def _prism(vertices: Iterable[tuple[float, float]], z0: float, z1: float) -> trimesh.Trimesh:
    pts_2d = list(vertices)
    verts = [(x, y, z0) for x, y in pts_2d] + [(x, y, z1) for x, y in pts_2d]
    n = len(pts_2d)
    faces: list[tuple[int, int, int]] = []

    for i in range(1, n - 1):
        faces.append((0, i + 1, i))
        faces.append((n, n + i, n + i + 1))

    for i in range(n):
        j = (i + 1) % n
        faces.append((i, j, n + j))
        faces.append((i, n + j, n + i))

    return trimesh.Trimesh(vertices=np.asarray(verts, dtype=float), faces=np.asarray(faces), process=True)


def _annular_cylinder(
    *,
    outer_r: float,
    inner_r: float,
    height: float,
    sections: int = 144,
) -> trimesh.Trimesh:
    verts: list[tuple[float, float, float]] = []
    for z in (-height / 2.0, height / 2.0):
        for r in (outer_r, inner_r):
            for i in range(sections):
                a = 2.0 * math.pi * i / sections
                verts.append((r * math.cos(a), r * math.sin(a), z))

    def idx(layer: int, ring: int, i: int) -> int:
        return layer * 2 * sections + ring * sections + (i % sections)

    faces: list[tuple[int, int, int]] = []
    for i in range(sections):
        j = i + 1
        # Outer wall.
        faces.append((idx(0, 0, i), idx(0, 0, j), idx(1, 0, j)))
        faces.append((idx(0, 0, i), idx(1, 0, j), idx(1, 0, i)))
        # Inner wall, reversed.
        faces.append((idx(0, 1, i), idx(1, 1, j), idx(0, 1, j)))
        faces.append((idx(0, 1, i), idx(1, 1, i), idx(1, 1, j)))
        # Top annulus.
        faces.append((idx(1, 0, i), idx(1, 0, j), idx(1, 1, j)))
        faces.append((idx(1, 0, i), idx(1, 1, j), idx(1, 1, i)))
        # Bottom annulus, reversed.
        faces.append((idx(0, 0, i), idx(0, 1, j), idx(0, 0, j)))
        faces.append((idx(0, 0, i), idx(0, 1, i), idx(0, 1, j)))

    return trimesh.Trimesh(vertices=np.asarray(verts, dtype=float), faces=np.asarray(faces), process=True)


def _box(extents: tuple[float, float, float], center: tuple[float, float, float]) -> trimesh.Trimesh:
    mesh = trimesh.creation.box(extents=extents)
    mesh.apply_translation(center)
    return mesh


def _mat(tx: float = 0.0, ty: float = 0.0, tz: float = 0.0, angle_deg: float = 0.0) -> list[float]:
    a = math.radians(angle_deg)
    c = math.cos(a)
    s = math.sin(a)
    return [
        c,
        s,
        0,
        0,
        -s,
        c,
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


def _instance(
    iid: str,
    mesh_id: str,
    name: str,
    part_type: str,
    color: str,
    *,
    role: str,
    tx: float = 0.0,
    ty: float = 0.0,
    tz: float = 0.0,
    angle_deg: float = 0.0,
    leg: int | None = None,
) -> dict:
    return {
        "id": iid,
        "meshId": mesh_id,
        "name": name,
        "partType": part_type,
        "role": role,
        "leg": leg,
        "joint": None,
        "cots": False,
        "color": color,
        "transform": _mat(tx, ty, tz, angle_deg),
    }


def _load_summary() -> dict:
    if not LOAD_MANIFEST.exists():
        return {"source": str(LOAD_MANIFEST.relative_to(PROTO_DIR)), "available": False}

    manifest = json.loads(LOAD_MANIFEST.read_text())
    scenarios = []
    for scenario in manifest.get("scenarios", []):
        peak = scenario.get("peak", {})
        scenarios.append(
            {
                "id": scenario.get("id"),
                "time_s": scenario.get("time_s"),
                "max_single_foot_normal_N": peak.get("maxSingleFootNormal_N"),
                "total_foot_normal_N": peak.get("totalFootNormal_N"),
                "max_joint_abs_torque_Nm": peak.get("maxJointAbsTorque_Nm"),
            }
        )
    scenarios.sort(
        key=lambda s: (
            s.get("max_single_foot_normal_N") or 0.0,
            s.get("total_foot_normal_N") or 0.0,
        ),
        reverse=True,
    )
    return {
        "source": str(LOAD_MANIFEST.relative_to(PROTO_DIR)),
        "available": True,
        "top_scenarios": scenarios[:5],
    }


def _export_meshes() -> list[dict]:
    STL_DIR.mkdir(parents=True, exist_ok=True)

    meshes = {
        "chassis_reference_plate.stl": _prism(_hex_vertices(PLATE_R), -PLATE_T / 2.0, PLATE_T / 2.0),
        "aluminum_top_doubler_ring.stl": _annular_cylinder(outer_r=111.0, inner_r=46.0, height=2.0),
        "aluminum_bottom_spreader_ring.stl": _annular_cylinder(outer_r=107.0, inner_r=39.0, height=2.0),
        "aluminum_radial_rib.stl": _box((62.0, 12.0, 2.5), (73.0, 0.0, 0.0)),
        "aluminum_yaw_saddle_pad.stl": _box((36.0, 34.0, 2.0), (YAW_R, 0.0, 0.0)),
        "aluminum_standoff_sleeve.stl": _annular_cylinder(outer_r=6.2, inner_r=3.2, height=20.6, sections=72),
    }

    mesh_defs = []
    for file_name, mesh in meshes.items():
        path = STL_DIR / file_name
        mesh.export(path)
        mesh_defs.append(
            {
                "id": f"stl:{path.stem}",
                "name": file_name,
                "url": f"stl/{file_name}",
            }
        )
    return mesh_defs


def build_scene() -> dict:
    mesh_defs = _export_meshes()

    bottom_top = BOTTOM_PLATE_Z + PLATE_T / 2.0
    top_bottom = TOP_PLATE_Z - PLATE_T / 2.0
    top_face = TOP_PLATE_Z + PLATE_T / 2.0
    bottom_face = BOTTOM_PLATE_Z - PLATE_T / 2.0
    top_ring_center_z = top_face + 1.0
    top_ring_top_z = top_ring_center_z + 1.0
    rib_center_z = top_ring_top_z + 2.5 / 2.0
    rib_top_z = rib_center_z + 2.5 / 2.0
    saddle_center_z = rib_top_z + 2.0 / 2.0

    instances = [
        _instance(
            "context-chassis-bottom",
            "stl:chassis_reference_plate",
            "simplified lower chassis plate context",
            "chassis_bottom_reference",
            "#7aa6c2",
            role="context",
            tz=BOTTOM_PLATE_Z,
        ),
        _instance(
            "context-chassis-top",
            "stl:chassis_reference_plate",
            "simplified upper chassis plate context",
            "chassis_top_reference",
            "#4d76a1",
            role="context",
            tz=TOP_PLATE_Z,
        ),
        _instance(
            "reinforcement-top-doubler-ring",
            "stl:aluminum_top_doubler_ring",
            "2 mm 6061 top doubler ring tying six yaw pockets together",
            "aluminum_top_doubler_ring",
            "#d7dde2",
            role="reinforcement",
            tz=top_ring_center_z,
        ),
        _instance(
            "reinforcement-bottom-spreader-ring",
            "stl:aluminum_bottom_spreader_ring",
            "2 mm 6061 lower load-spreader ring under chassis",
            "aluminum_bottom_spreader_ring",
            "#b8c0c8",
            role="reinforcement",
            tz=bottom_face - 1.0,
        ),
    ]

    for leg, angle in enumerate(LEG_ANGLES_DEG):
        instances.append(
            _instance(
                f"reinforcement-radial-rib-L{leg}",
                "stl:aluminum_radial_rib",
                f"L{leg} radial rib strap from center ring to yaw saddle",
                "aluminum_radial_rib",
                "#f0c23e",
                role="reinforcement",
                tz=rib_center_z,
                angle_deg=angle,
                leg=leg,
            )
        )
        instances.append(
            _instance(
                f"reinforcement-yaw-saddle-L{leg}",
                "stl:aluminum_yaw_saddle_pad",
                f"L{leg} yaw-bearing saddle pad / local doubler",
                "aluminum_yaw_saddle_pad",
                "#e08c2f",
                role="reinforcement",
                tz=saddle_center_z,
                angle_deg=angle,
                leg=leg,
            )
        )

    for idx, (x, y) in enumerate(STANDOFF_POINTS):
        instances.append(
            _instance(
                f"reinforcement-standoff-sleeve-{idx}",
                "stl:aluminum_standoff_sleeve",
                f"standoff {idx} compression sleeve tying upper/lower plates",
                "aluminum_standoff_sleeve",
                "#8f9aa3",
                role="reinforcement",
                tx=x,
                ty=y,
                tz=(bottom_top + top_bottom) / 2.0,
            )
        )

    return {
        "name": "STS3215 chassis reinforcement test kit",
        "source": "make_chassis_reinforcement_concept.py",
        "buildId": BUILD_ID,
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": [0, 0, (BOTTOM_PLATE_Z + TOP_PLATE_Z) / 2.0],
        "meshes": mesh_defs,
        "instances": instances,
        "checksConfig": {
            "overlapMm3": 50.0,
            "pitchMm": 1.5,
            "partDensitiesGCm3": {
                "chassis_bottom_reference": 0.62,
                "chassis_top_reference": 0.62,
                "aluminum_top_doubler_ring": 2.70,
                "aluminum_bottom_spreader_ring": 2.70,
                "aluminum_radial_rib": 2.70,
                "aluminum_yaw_saddle_pad": 2.70,
                "aluminum_standoff_sleeve": 2.70,
            },
        },
        "analysis": {
            "purpose": "First-pass visual reinforcement concept for the chassis/yaw-pocket load path before production CAD changes.",
            "loadSummary": _load_summary(),
            "reinforcementIntent": [
                "Spread six yaw bearing reactions into a continuous top doubler instead of isolated plastic pockets.",
                "Give the lower plate a matching load-spreader so the standoff screws are not point fixtures.",
                "Use standoff sleeves as compression columns between plates to reduce plate dish/bending under rise/lower foot peaks.",
                "Keep the design add-on/removable so Onshape/BuildViz iteration does not add production tech debt.",
            ],
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
