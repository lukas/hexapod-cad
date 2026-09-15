#!/usr/bin/env python3
"""THE canonical exporter for the production print set (STEP-first).

Every ``stl_prototype/`` printable is authored as a build123d/OpenCascade
BREP solid (the ported builders in ``cad_step_test/build_step_first_test.py``,
which import every dimension from ``hexapod_prototype.py`` -- no constant
forks).  This script exports each printable as ``.step`` (the editable CAD
truth) plus a tessellated ``.stl`` into ``step_prototype/``, then PROVES
each part equivalent to its trimesh twin before anything may reach the
print set:

  * the derived STL must heal (``hp._heal_for_export``) into a closed
    volume;
  * |volume delta| vs the healed twin <= VOL_RTOL_PCT;
  * bbox size delta per axis <= BBOX_ATOL_MM;
  * two-sided sampled max surface deviation <= DEV_MAX_MM (tessellation
    error is sagitta-bounded ~0.06 mm at CYL_SECTIONS=48 for the part
    radii here; a real missing feature shows >= 0.3 mm).

Any gate failure exits nonzero, so ``build_all.py`` (which runs this as a
subprocess and then installs the healed tessellations into
``stl_prototype/``) can never ship a print set that drifted from the
parametric source.  The manifest records per-part metrics plus sha256
hashes of the geometry sources for the verifier's staleness check.

Run (build123d needs a 3.12 interpreter; build_all.py does this for you):

  uv run --no-project --python 3.12 \
    --with build123d --with trimesh --with numpy --with manifold3d \
    --with rtree \
    python hexapod_walker/prototype_sts3215/build_step_prototype.py
"""

from __future__ import annotations

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "cad_step_test"))

import numpy as np  # noqa: E402
import trimesh  # noqa: E402
from build123d import export_step, export_stl  # noqa: E402

import hexapod_prototype as hp  # noqa: E402
import step_pipeline  # noqa: E402
import build_step_first_test as step  # noqa: E402

# --- equivalence gates ------------------------------------------------------
DEV_MAX_MM = 0.15      # two-sided max surface deviation vs the healed twin
VOL_RTOL_PCT = 1.0     # |volume delta| %
BBOX_ATOL_MM = 0.2     # per-axis bbox size delta
DEV_SAMPLES = 8000     # surface samples per direction (seeded, deterministic)

# Parts whose deviation is expected ABOVE pure cylinder-sagitta noise, with
# the reason recorded in the manifest.  (Gates still apply unchanged.)
JUSTIFICATIONS = {
    "chassis_top": (
        "0.12 mm ring at the deck edge: the mesh twin clips the deck with "
        "a CYL_SECTIONS=48 polygon circle whose sagitta sits 0.123 mm "
        "inside the true BREP clip circle (r 57.5) -- a legacy tessellation "
        "artifact; the BREP outline is the more accurate one."
    ),
}


def _max_deviation(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    """Two-sided sampled max surface deviation between two meshes (mm)."""
    worst = 0.0
    for src, dst in ((a, b), (b, a)):
        pts, _ = trimesh.sample.sample_surface(src, DEV_SAMPLES, seed=0)
        d = np.abs(trimesh.proximity.ProximityQuery(dst).signed_distance(pts))
        worst = max(worst, float(d.max()))
    return worst


def _load_healed(path: str) -> trimesh.Trimesh:
    mesh = trimesh.load(path, process=True)
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(
            [g for g in mesh.geometry.values() if len(g.faces) > 0])
    return hp._heal_for_export(mesh)


def export_print_set() -> tuple[list[dict], list[str]]:
    bases = step_pipeline.printable_bases()
    twins = {}
    for _section, builders in hp.stl_export_groups():
        for name, build in builders:
            twins[name[:-4].replace(hp.NOPRINT_SUFFIX, "")] = build

    os.makedirs(step_pipeline.STEP_STL_DIR, exist_ok=True)
    rows, problems = [], []
    for base in bases:
        builder = getattr(step, f"make_{base}", None)
        if builder is None:
            problems.append(
                f"{base}: printable has NO BREP builder "
                f"(cad_step_test.build_step_first_test.make_{base} missing) "
                "-- port it before it can ship")
            continue
        part = builder()
        step_path = os.path.join(step_pipeline.STEP_PROTO_DIR, f"{base}.step")
        stl_path = os.path.join(step_pipeline.STEP_STL_DIR, f"{base}.stl")
        export_step(part, step_path)
        export_stl(part, stl_path)

        derived = _load_healed(stl_path)
        twin = hp._heal_for_export(twins[base]())

        vol_pct = 100.0 * (derived.volume - twin.volume) / twin.volume
        bbox_delta = np.abs(derived.extents - twin.extents)
        dev = _max_deviation(derived, twin)

        checks = {
            "healed_is_volume": bool(derived.is_volume),
            "volume_delta_pct": round(vol_pct, 4),
            "bbox_size_delta_mm": [round(float(v), 4) for v in bbox_delta],
            "max_surface_deviation_mm": round(dev, 4),
        }
        ok = (derived.is_volume
              and abs(vol_pct) <= VOL_RTOL_PCT
              and float(bbox_delta.max()) <= BBOX_ATOL_MM
              and dev <= DEV_MAX_MM)
        if not ok:
            problems.append(f"{base}: equivalence gate FAILED: {checks}")

        rows.append({
            "name": base,
            "step": os.path.relpath(step_path, step_pipeline.STEP_PROTO_DIR),
            "stl": os.path.relpath(stl_path, step_pipeline.STEP_PROTO_DIR),
            "brep_faces": int(len(part.faces())),
            "brep_volume_mm3": round(float(part.volume), 4),
            "derived_stl_triangles": int(len(derived.faces)),
            "twin_volume_mm3": round(float(twin.volume), 4),
            "equivalence": checks,
            "justification": JUSTIFICATIONS.get(base),
        })
        print(f"  {base:<20s} dev={dev:6.4f} mm  vol {vol_pct:+.3f}%  "
              f"bbox d {float(bbox_delta.max()):.4f} mm  "
              f"{'OK' if ok else 'FAIL'}")
    return rows, problems


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()

    print("STEP-first print set: exporting BREP printables to "
          "step_prototype/ ...")
    rows, problems = export_print_set()
    manifest = {
        "units": "mm",
        "source": (
            "build123d/OpenCascade BREP (CANONICAL print-set geometry), "
            "builders in cad_step_test/build_step_first_test.py, constants "
            "imported from hexapod_prototype.py"
        ),
        "gates": {
            "max_surface_deviation_mm": DEV_MAX_MM,
            "volume_rtol_pct": VOL_RTOL_PCT,
            "bbox_atol_mm": BBOX_ATOL_MM,
            "deviation_samples": DEV_SAMPLES,
        },
        "exported_parts": rows,
        "source_sha256": step_pipeline.source_hashes(),
        "checks": {"passed": not problems, "problems": problems},
    }
    with open(step_pipeline.MANIFEST_PATH, "w") as fh:
        json.dump(manifest, fh, indent=2)
        fh.write("\n")
    print(f"wrote {os.path.relpath(step_pipeline.MANIFEST_PATH, HERE)}")
    if problems:
        print("STEP-first print-set export FAILED:")
        for problem in problems:
            print(f"  - {problem}")
        raise SystemExit(1)
    print(f"OK -- {len(rows)} printables exported + proven equivalent.")


if __name__ == "__main__":
    main()
