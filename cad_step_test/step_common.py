"""Shared machinery for the STEP-first sidecar exporters.

Every exporter in this directory (base parts, C-horn variant, rigid-hip
variant) builds parts as build123d/OpenCascade BREP solids and then goes
through the exact same tail: export STEP + STL, stat the results against a
legacy mesh-pipeline STL when one exists, and zip a manifest bundle.  That
tail lives here once instead of being copied into each exporter.
"""

from __future__ import annotations

import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np
import trimesh
from build123d import export_step, export_stl

THIS_DIR = Path(__file__).resolve().parent
PROTO_DIR = THIS_DIR.parent
OUT_DIR = THIS_DIR / "out"
STEP_DIR = OUT_DIR / "step"
STL_DIR = OUT_DIR / "stl"


@dataclass(frozen=True)
class StepPart:
    name: str
    builder: Callable[[], object]
    legacy_stl: Path | None
    note: str
    printable: bool = True


def _round_list(values: np.ndarray | list[float],
                digits: int = 4) -> list[float]:
    return [round(float(v), digits) for v in values]


def _part_bbox(part: object) -> dict:
    bb = part.bounding_box()
    return {
        "min": _round_list([bb.min.X, bb.min.Y, bb.min.Z]),
        "max": _round_list([bb.max.X, bb.max.Y, bb.max.Z]),
        "size": _round_list([bb.size.X, bb.size.Y, bb.size.Z]),
    }


def _load_mesh(path: Path, process: bool) -> trimesh.Trimesh:
    mesh = trimesh.load(path, process=process)
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(
            [g for g in mesh.geometry.values() if len(g.faces) > 0]
        )
    return mesh


def _legacy_bbox(path: Path | None) -> dict | None:
    if path is None or not path.exists():
        return None
    mesh = _load_mesh(path, process=False)
    return {
        "path": str(path.relative_to(PROTO_DIR)),
        "min": _round_list(mesh.bounds[0]),
        "max": _round_list(mesh.bounds[1]),
        "size": _round_list(mesh.extents),
        "triangles": int(len(mesh.faces)),
        "volume_mm3": round(float(mesh.volume), 4),
    }


def _mesh_stats(path: Path) -> dict:
    # Binary STL is triangle soup: vertices are duplicated per facet. Process
    # the load so trimesh welds coincident vertices before judging watertightness.
    mesh = _load_mesh(path, process=True)
    return {
        "triangles": int(len(mesh.faces)),
        "watertight": bool(mesh.is_watertight),
        "volume_mm3": round(float(mesh.volume), 4),
        "bbox": {
            "min": _round_list(mesh.bounds[0]),
            "max": _round_list(mesh.bounds[1]),
            "size": _round_list(mesh.extents),
        },
    }


def export_one(spec: StepPart, out_dir: Path = OUT_DIR,
               step_dir: Path | None = None,
               stl_dir: Path | None = None) -> dict:
    """Export one part. Manifest paths are relative to ``out_dir``; the
    STEP/STL files land in ``step_dir``/``stl_dir`` (default: ``out_dir``'s
    ``step/`` and ``stl/`` subdirectories, the shared-pool layout)."""
    part = spec.builder()
    step_dir = step_dir if step_dir is not None else out_dir / "step"
    stl_dir = stl_dir if stl_dir is not None else out_dir / "stl"
    step_path = step_dir / f"{spec.name}.step"
    stl_path = stl_dir / f"{spec.name}.stl"
    export_step(part, step_path)
    export_stl(part, stl_path)

    legacy = _legacy_bbox(spec.legacy_stl)
    bbox = _part_bbox(part)
    size_delta = None
    if legacy is not None:
        size_delta = _round_list(
            np.asarray(bbox["size"]) - np.asarray(legacy["size"])
        )

    return {
        "name": spec.name,
        "note": spec.note,
        "printable": spec.printable,
        "step": str(step_path.relative_to(out_dir)),
        "stl": str(stl_path.relative_to(out_dir)),
        "brep_faces": int(len(part.faces())),
        "brep_volume_mm3": round(float(part.volume), 4),
        "brep_bbox": bbox,
        "derived_stl": _mesh_stats(stl_path),
        "legacy_stl": legacy,
        "bbox_size_delta_vs_legacy_mm": size_delta,
    }


def export_all(specs: list[StepPart], out_dir: Path = OUT_DIR,
               step_dir: Path | None = None,
               stl_dir: Path | None = None) -> list[dict]:
    step_dir = step_dir if step_dir is not None else out_dir / "step"
    stl_dir = stl_dir if stl_dir is not None else out_dir / "stl"
    step_dir.mkdir(parents=True, exist_ok=True)
    stl_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for spec in specs:
        row = export_one(spec, out_dir, step_dir, stl_dir)
        rows.append(row)
        size = row["brep_bbox"]["size"]
        print(
            f"wrote {row['step']:<42s} "
            f"faces={row['brep_faces']:>3d} "
            f"bbox={size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} mm"
        )
    return rows


def write_bundle(manifest: dict, bundle_name: str, manifest_name: str,
                 out_dir: Path = OUT_DIR) -> Path:
    bundle = out_dir / bundle_name
    with zipfile.ZipFile(bundle, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.write(out_dir / manifest_name, manifest_name)
        for rel in manifest["files"]:
            zf.write(out_dir / rel, rel)
    return bundle
