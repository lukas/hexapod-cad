#!/usr/bin/env python3
"""Build a focused BuildViz scene for the knee-yoke AprilTag flag."""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import numpy as np
import trimesh


HERE = Path(__file__).resolve().parent
PROTO_DIR = HERE.parent
BUILD_DIR = HERE / "buildviz"
STL_DIR = BUILD_DIR / "stl"
for import_dir in (PROTO_DIR, HERE):
    if str(import_dir) not in sys.path:
        sys.path.insert(0, str(import_dir))

import hexapod_prototype as hp  # noqa: E402
import make_knee_yoke_apriltag_flag as flag  # noqa: E402


BUILD_ID = "prototype_sts3215/knee-yoke-apriltag-flag"
EXPLODE_Z = 24.0


def mat(tx: float = 0.0, ty: float = 0.0, tz: float = 0.0) -> list[float]:
    return [
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        float(tx), float(ty), float(tz), 1,
    ]


def tube_reference() -> trimesh.Trimesh:
    x0 = hp._YOKE_SOCKET_X
    x1 = hp.TIBIA_LENGTH - 18.0
    return hp._tube_between(
        np.array([x0, 0.0, hp.JOINT_SOCKET_Z]),
        np.array([x1, 0.0, hp.JOINT_SOCKET_Z]),
        hp.LEG_TUBE_OD / 2.0,
    )


def m3x12_shcs_reference() -> trimesh.Trimesh:
    """Display reference: mating face at Z=0, screw points toward -Z."""
    shank = trimesh.creation.cylinder(radius=1.5, height=12.0, sections=64)
    shank.apply_translation((0.0, 0.0, -6.0))
    head = trimesh.creation.cylinder(
        radius=flag.DEFAULT_HEAD_D / 2.0,
        height=flag.DEFAULT_HEAD_H,
        sections=64,
    )
    head.apply_translation((0.0, 0.0, flag.DEFAULT_HEAD_H / 2.0))
    return flag.boolean_union([shank, head])


def mesh_entry(mesh_id: str, filename: str) -> dict:
    return {"id": mesh_id, "name": filename, "url": f"stl/{filename}"}


def instance(
    instance_id: str,
    mesh_id: str,
    name: str,
    part_type: str,
    role: str,
    focus_group: str,
    color: str,
    transform: list[float],
    *,
    cots: bool = False,
) -> dict:
    return {
        "id": instance_id,
        "meshId": mesh_id,
        "name": name,
        "partType": part_type,
        "role": role,
        "focusGroup": focus_group,
        "joint": "knee",
        "leg": 0,
        "cots": cots,
        "color": color,
        "transform": transform,
    }


def main() -> None:
    STL_DIR.mkdir(parents=True, exist_ok=True)
    holder, dimensions = flag.build_holder(
        tag_id=flag.DEFAULT_TAG_ID,
        head_diameter=flag.DEFAULT_HEAD_D,
        head_height=flag.DEFAULT_HEAD_H,
        throat_clearance=flag.DEFAULT_THROAT_CLEARANCE,
    )
    white, black = flag.build_colour_parts(holder, flag.DEFAULT_TAG_ID)
    installed_white = flag.installed_mesh(white, dimensions["total_height_mm"])
    installed_black = flag.installed_mesh(black, dimensions["total_height_mm"])

    assets = {
        "stl:tibia_knee_yoke": ("tibia_knee_yoke.stl", hp.make_tibia_knee_yoke()),
        "stl:tibia_tube_reference": ("tibia_tube_reference.stl", tube_reference()),
        "stl:m3x12_shcs_reference": ("m3x12_shcs_reference.stl", m3x12_shcs_reference()),
        "stl:tag_holder_white": ("tag_holder_white.stl", installed_white),
        "stl:tag_features_black": ("tag_features_black.stl", installed_black),
    }
    for _mesh_id, (filename, mesh) in assets.items():
        mesh.export(STL_DIR / filename)

    instances: list[dict] = [
        instance(
            "knee-yoke",
            "stl:tibia_knee_yoke",
            "production tibia knee yoke",
            "tibia_knee_yoke",
            "production_reference",
            "knee-tag",
            "#4385c6",
            mat(),
        ),
        instance(
            "tibia-tube",
            "stl:tibia_tube_reference",
            "carbon tibia tube reference",
            "tibia_tube_reference",
            "context",
            "knee-tag",
            "#29333d",
            mat(),
            cots=True,
        ),
        instance(
            "tag-holder",
            "stl:tag_holder_white",
            "white holder and four split grip cups",
            "knee_yoke_apriltag_holder_white",
            "measurement_accessory",
            "knee-tag",
            "#f5f5f0",
            mat(),
        ),
        instance(
            "tag-black",
            "stl:tag_features_black",
            "black AprilTag cells and raised rear ID",
            "knee_yoke_apriltag_features_black",
            "measurement_target",
            "knee-tag",
            "#111111",
            mat(),
        ),
    ]

    for index, (x, y) in enumerate(flag.bolt_centres_about_output()):
        instances.append(
            instance(
                f"screw-{index}",
                "stl:m3x12_shcs_reference",
                f"M3x12 SHCS {index + 1}",
                "m3x12_shcs_reference",
                "existing_fastener",
                "knee-tag",
                "#b9c1c7",
                mat(hp.SERVO_OUTPUT_X + x, y, hp._YOKE_TOP_Z1),
                cots=True,
            )
        )

    tag_instance_ids: list[str] = ["tag-holder", "tag-black"]

    scene = {
        "name": "STS3215 knee-yoke AprilTag push-on measurement flag",
        "source": "build_buildviz.py",
        "buildId": BUILD_ID,
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": [60.0, 0.0, 27.0],
        "meshes": [
            mesh_entry(mesh_id, filename)
            for mesh_id, (filename, _mesh) in assets.items()
        ],
        "instances": instances,
        "joints": [
            {
                "id": "remove-tag-flag",
                "type": "prismatic",
                "axis": [0.0, 0.0, 1.0],
                "origin": [hp.SERVO_OUTPUT_X, 0.0, hp._YOKE_TOP_Z1],
                "instances": tag_instance_ids,
                "limits": {"min": 0.0, "max": EXPLODE_Z},
                "home": 0.0,
                "label": "Pull flag off screw heads",
            }
        ],
        "poses": [
            {"id": "installed", "name": "Installed", "jointValues": {}},
            {
                "id": "exploded",
                "name": "Exploded cup/head view",
                "jointValues": {"remove-tag-flag": EXPLODE_Z},
            },
        ],
        "metadata": {
            "tagFamily": "tag36h11",
            "tagId": flag.DEFAULT_TAG_ID,
            "tagSizeMm": flag.TAG_SIZE,
            "plateSizeMm": flag.PLATE_SIZE,
            "humanReadableId": "raised black material on rear/cup side only",
            "cameraFace": "unobstructed tag and full white quiet zone",
            "attachment": "four split cups over existing M3 SHCS heads",
            "cupThroatDiameterMm": dimensions["cup_throat_diameter_mm"],
            "explodedPoseTravelMm": EXPLODE_Z,
        },
    }
    (BUILD_DIR / "scene.json").write_text(json.dumps(scene, indent=2) + "\n")
    shutil.copy2(HERE / "design_spec.yaml", BUILD_DIR / "design_spec.yaml")
    print(
        f"wrote {BUILD_DIR / 'scene.json'} with "
        f"{len(scene['meshes'])} meshes and {len(instances)} instances"
    )


if __name__ == "__main__":
    main()
