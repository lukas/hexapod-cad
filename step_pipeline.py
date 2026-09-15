"""STEP-first print-set plumbing shared by build_all.py and the verifier.

Since Aug 2026 the ``stl_prototype/`` printables are STEP-FIRST: their
geometry is authored as build123d/OpenCascade BREP solids (the ported
builders in ``cad_step_test/build_step_first_test.py``), exported by
``build_step_prototype.py`` as ``.step`` (the editable CAD truth) plus a
tessellated ``.stl`` per part into ``step_prototype/``, and installed into
the print set after the same ``hp._heal_for_export`` pass the mesh
pipeline always applied.  The trimesh ``make_*`` twins in
``hexapod_prototype.py`` REMAIN the parametric source for constants, the
verifier's probes, MuJoCo and BuildViz -- the exporter proves every
printable equivalent to its twin (volume / bbox / max surface deviation
gates) on every run, so the two sources cannot silently drift.

build123d has no Python-3.14 wheels yet, so the exporter runs in its own
uv-provisioned 3.12 interpreter (same incantation as the rigid-hip
concept) and talks to this side purely through the ``step_prototype/``
files.  This module is the ONE place that knows the layout, the
subprocess incantation, and the source-hash freshness contract; it must
stay importable under both interpreters.
"""

from __future__ import annotations

import hashlib
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
STEP_PROTO_DIR = os.path.join(HERE, "step_prototype")
STEP_STL_DIR = os.path.join(STEP_PROTO_DIR, "stl")
MANIFEST_PATH = os.path.join(STEP_PROTO_DIR, "manifest.json")
EXPORTER = os.path.join(HERE, "build_step_prototype.py")

# Geometry-defining inputs of the BREP print set.  The exporter records
# their sha256 in the manifest; the verifier recomputes them so an edited
# builder with a stale step_prototype/ fails loudly ("run make build").
SOURCE_FILES = (
    "hexapod_prototype.py",
    "build_step_prototype.py",
    "step_pipeline.py",
    os.path.join("cad_step_test", "build_step_first_test.py"),
    os.path.join("cad_step_test", "step_common.py"),
)


def brep_export_cmd() -> list[str]:
    return ["uv", "run", "--no-project", "--python", "3.12",
            "--with", "build123d", "--with", "trimesh",
            "--with", "numpy", "--with", "manifold3d",
            "--with", "rtree",
            "python", EXPORTER]


def printable_bases() -> list[str]:
    """Print-set part names (stl_prototype/ filenames minus .stl), derived
    from the SAME ``hp.stl_export_groups()`` registry the mesh exporter and
    the verifier walk -- so a new printable that lacks a BREP port can
    never slip into the print set unnoticed."""
    import hexapod_prototype as hp

    bases = []
    for _section, builders in hp.stl_export_groups():
        for name, _build in builders:
            base = name[:-4].replace(hp.NOPRINT_SUFFIX, "")
            if hp.stl_dir_for(base) == hp.STL_DIR:
                bases.append(base)
    return bases


def source_hashes() -> dict[str, str]:
    out = {}
    for rel in SOURCE_FILES:
        path = os.path.join(HERE, rel)
        with open(path, "rb") as fh:
            out[rel.replace(os.sep, "/")] = hashlib.sha256(
                fh.read()).hexdigest()
    return out


def manifest_fresh() -> tuple[bool, str]:
    """Does step_prototype/manifest.json exist and match the CURRENT
    geometry sources?  A mismatch means someone edited a builder and did
    not re-run the exporter."""
    if not os.path.isfile(MANIFEST_PATH):
        return False, "step_prototype/manifest.json MISSING -- run `make build`"
    with open(MANIFEST_PATH) as fh:
        manifest = json.load(fh)
    recorded = manifest.get("source_sha256", {})
    stale = [rel for rel, sha in source_hashes().items()
             if recorded.get(rel) != sha]
    if stale:
        return False, ("BREP export STALE vs " + ", ".join(stale)
                       + " -- run `make build`")
    n = len(manifest.get("exported_parts", []))
    return True, f"sources match ({n} parts exported)"


def load_healed_brep(base: str):
    """The healed BREP tessellation for one printable -- EXACTLY the mesh
    the print set holds for it (heal is deterministic for a given file)."""
    import trimesh

    import hexapod_prototype as hp

    path = os.path.join(STEP_STL_DIR, f"{base}.stl")
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"{base}: no BREP tessellation at {path} -- run `make build` "
            "(build_all.py runs build_step_prototype.py for you)")
    mesh = trimesh.load(path, process=True)
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(
            [g for g in mesh.geometry.values() if len(g.faces) > 0])
    return hp._heal_for_export(mesh)


def install_printables() -> list[str]:
    """Install every printable's healed BREP tessellation into
    stl_prototype/ (the slicer-facing print set).  Refuses any mesh that
    does not heal into a closed volume."""
    import hexapod_prototype as hp

    written = []
    for base in printable_bases():
        mesh = load_healed_brep(base)
        assert mesh.is_volume, (
            f"{base}: BREP tessellation does not heal into a closed volume")
        path = hp.stl_path(base)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        mesh.export(path)
        rel = os.path.relpath(path, HERE)
        print(f"  installed {rel:42s} {len(mesh.faces):>6d} faces "
              f"(BREP tessellation, healed)")
        written.append(path)
    return written
