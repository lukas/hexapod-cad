"""Tetrahedralize a printed-part STL via Gmsh.

Tries the Python ``gmsh`` API first (most reliable; ships with the
``gmsh`` brew package).  Falls back to ``pygmsh`` if available; finally
shells out to the ``gmsh`` CLI.  If none of those is on PATH, raises
``MesherUnavailable`` so the caller can downgrade to beam-bending only.

Target element size is configurable; the brief asks for ~ 2 mm tet
edges with 1 mm refinement at sharp internal corners.  Gmsh's
"distance" + "threshold" field combo does the refinement automatically
based on the input STL's curvature surface tag.

Output is a ``.msh`` file (Gmsh native) that
``run_calculix._convert_msh_to_inp`` will translate into a CalculiX
``.inp`` deck.  We also return the loaded ``meshio.Mesh`` object so
the caller can avoid a redundant read.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from dataclasses import dataclass

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.dirname(THIS_DIR)
ARTIFACT_DIR = os.path.join(PROTO_DIR, "artifacts", "strength")
os.makedirs(ARTIFACT_DIR, exist_ok=True)


class MesherUnavailable(RuntimeError):
    """Neither the gmsh Python API nor the gmsh CLI is available."""


@dataclass
class TetMesh:
    """A tetrahedral mesh ready for CalculiX consumption."""
    msh_path: str
    num_nodes: int
    num_tets: int
    num_tris: int   # surface triangles (boundary)
    backend: str    # "gmsh-python" / "pygmsh" / "gmsh-cli"


def _have_gmsh_python() -> bool:
    try:
        import gmsh  # noqa: F401
        return True
    except Exception:
        return False


def _have_gmsh_cli() -> bool:
    return shutil.which("gmsh") is not None


# Per-part default sizes.  The hexapod's chassis plates are large
# (200 mm flat-to-flat hex with 6 integrated yaw cradles) and would
# blow up to ~ 100k tets at the default 2 mm target; we coarsen them
# to keep ``make check-strength`` runnable on a laptop.  Tweak via
# the ``target_size_mm`` / ``refine_size_mm`` kwargs when running
# from a Python REPL.
PART_DEFAULT_SIZES_MM: dict[str, tuple[float, float]] = {
    "chassis_bottom": (6.0, 4.0),
    "chassis_top":    (5.0, 3.0),
    "electronics_tray": (5.0, 3.0),
    "battery_holder": (4.0, 2.5),
    "femur_link":     (3.0, 1.5),
    "tibia_link":     (2.5, 1.2),
    "coxa_link":      (2.5, 1.2),
}


def tetrahedralize(stl_path: str,
                   *,
                   out_msh_path: str | None = None,
                   target_size_mm: float | None = None,
                   refine_size_mm: float | None = None,
                   verbose: bool = False) -> TetMesh:
    """Tetrahedralize ``stl_path`` and write a Gmsh .msh.

    The STL is assumed to be in mm (matches the rest of the prototype's
    geometry).  Tets are first-order (CCX is happiest with first-order
    C3D4 elements for a linear-static deck).
    """
    if not os.path.exists(stl_path):
        raise FileNotFoundError(stl_path)
    base = os.path.splitext(os.path.basename(stl_path))[0]
    if out_msh_path is None:
        out_msh_path = os.path.join(ARTIFACT_DIR, f"{base}.msh")
    os.makedirs(os.path.dirname(out_msh_path), exist_ok=True)

    # Pick per-part defaults if the caller didn't specify; fall back
    # to the brief's 2 mm / 1 mm pair for unknown parts.
    if target_size_mm is None or refine_size_mm is None:
        defaults = PART_DEFAULT_SIZES_MM.get(base, (2.0, 1.0))
        if target_size_mm is None:
            target_size_mm = defaults[0]
        if refine_size_mm is None:
            refine_size_mm = defaults[1]

    if _have_gmsh_python():
        return _tet_via_gmsh_python(
            stl_path, out_msh_path,
            target_size_mm, refine_size_mm, verbose,
        )
    if _have_gmsh_cli():
        return _tet_via_gmsh_cli(
            stl_path, out_msh_path,
            target_size_mm, refine_size_mm, verbose,
        )
    raise MesherUnavailable(
        "Neither gmsh Python module nor gmsh CLI found.  Install via "
        "``brew install gmsh`` (mac) and re-run; or stick with the "
        "beam-bending-only solver path."
    )


def _heal_stl_via_trimesh(stl_path: str,
                          max_edge_mm: float | None = None) -> str:
    """Run trimesh's standard cleanup pipeline on the STL and write a
    healed copy.  Returns the path to the healed STL.

    Boolean unions in ``hexapod_prototype`` occasionally leave
    duplicate vertices / coplanar overlapping facets / inverted
    normals along the seam.  Gmsh's ``classifySurfaces`` /
    ``createGeometry`` step refuses to accept those (raises "Invalid
    boundary mesh (overlapping facets)"), so we run trimesh's
    standard cleanup pipeline before handing the STL to Gmsh.

    Aug 2026: two more healing passes, found meshing femur_link (its
    CSG seams made gmsh's Delaunay throw "PLC Error: a segment and a
    facet intersect" and HXT segfault in facet recovery):
    * a manifold3d self-union, which rebuilds the mesh as a guaranteed
      self-intersection-free solid, and
    * ``subdivide_to_size(max_edge=max_edge_mm)`` so the boundary has
      no long skinny CSG facets -- gmsh's classify+reparametrize pass
      then generates its own conforming surface mesh without
      "overlapping facets" failures.  Pass ``max_edge_mm`` ~ the tet
      target size.
    """
    import trimesh
    m = trimesh.load(stl_path, force="mesh")
    try:
        m = trimesh.boolean.union([m], engine="manifold")
    except BaseException:
        pass  # manifold3d missing/failed: fall through to basic cleanup
    # Standard cleanup pipeline:
    m.merge_vertices()
    m.remove_duplicate_faces() if hasattr(m, "remove_duplicate_faces") else None
    m.remove_degenerate_faces() if hasattr(m, "remove_degenerate_faces") else None
    m.remove_unreferenced_vertices()
    m.fix_normals()
    if max_edge_mm is not None and max_edge_mm > 0:
        v, f = trimesh.remesh.subdivide_to_size(
            m.vertices, m.faces, max_edge=max_edge_mm)
        m = trimesh.Trimesh(v, f)
        m.merge_vertices()
    base = os.path.splitext(os.path.basename(stl_path))[0]
    out = os.path.join(ARTIFACT_DIR, f"{base}_healed.stl")
    m.export(out)
    return out


def _tet_via_gmsh_python(stl_path: str,
                        out_msh_path: str,
                        target_size_mm: float,
                        refine_size_mm: float,
                        verbose: bool) -> TetMesh:
    """Tetrahedralize via the Gmsh Python API.

    Strategy: clean the STL with trimesh first, then ask Gmsh to
    classify + reparametrize the surface, build a single closed
    volume, and tet-mesh it with a curvature-aware size field.  If
    classifySurfaces+createGeometry rejects the STL (overlapping
    facets, slivers, ...) we fall back to the "use STL as boundary
    triangulation directly" path that skips the reparametrization.
    """
    import gmsh
    import meshio

    healed_path = _heal_stl_via_trimesh(stl_path, max_edge_mm=target_size_mm)

    def _try_classify_geometry() -> bool:
        gmsh.merge(healed_path)
        ang = 40.0 * 3.14159265358979 / 180.0
        try:
            gmsh.model.mesh.classifySurfaces(ang, True, True, ang)
            gmsh.model.mesh.createGeometry()
        except Exception:
            return False
        s = gmsh.model.getEntities(2)
        s_tags = [e[1] for e in s]
        if not s_tags:
            return False
        try:
            sl = gmsh.model.geo.addSurfaceLoop(s_tags)
            gmsh.model.geo.addVolume([sl])
            gmsh.model.geo.synchronize()
        except Exception:
            return False
        return True

    def _try_stl_boundary_only() -> bool:
        """Fallback: import the STL as a single discrete surface and
        ask Gmsh to mesh the volume it bounds.  No curvature-aware
        refinement, but tolerates STLs with minor overlapping facets."""
        gmsh.clear()
        gmsh.merge(healed_path)
        # Reclassify as a single surface (no sharp-edge detection).
        try:
            gmsh.model.mesh.classifySurfaces(3.1415, True, False, 3.1415)
            gmsh.model.mesh.createTopology()
        except Exception:
            pass
        s = gmsh.model.getEntities(2)
        s_tags = [e[1] for e in s]
        if not s_tags:
            return False
        try:
            sl = gmsh.model.geo.addSurfaceLoop(s_tags)
            gmsh.model.geo.addVolume([sl])
            gmsh.model.geo.synchronize()
        except Exception:
            return False
        return True

    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 1 if verbose else 0)
        gmsh.option.setNumber("Mesh.MshFileVersion", 2.2)
        # Pass 1: full classify + recreate-geometry
        gmsh.model.add("part")
        ok = _try_classify_geometry()
        if not ok:
            # Pass 2: STL as boundary
            gmsh.model.add("part_fallback")
            ok = _try_stl_boundary_only()
        if not ok:
            raise RuntimeError(
                f"Gmsh could not build a closed volume from {stl_path!r}; "
                "the STL may have self-intersections beyond trimesh's "
                "automatic cleanup."
            )
        # Constant background size; if curves exist refine at sharp edges.
        bg = gmsh.model.mesh.field.add("Constant")
        gmsh.model.mesh.field.setNumber(bg, "VIn", target_size_mm)
        gmsh.model.mesh.field.setNumber(bg, "VOut", target_size_mm)
        all_curves = [e[1] for e in gmsh.model.getEntities(1)]
        # Aug 2026: the Distance field costs O(curves x sampling) per mesh
        # point -- on a healed/subdivided CSG part classify can emit 1000s
        # of tiny curves and the "refinement" then pegs a core for 30+ min
        # (observed on femur_link).  Cap it: past ~200 curves skip the
        # sharp-edge refinement and mesh at the constant target size.
        if len(all_curves) > 200:
            print(
                f"[mesh_part] {len(all_curves)} classified curves -- "
                "skipping Distance-field edge refinement (would hang); "
                "meshing at constant size."
            )
            all_curves = []
        if all_curves:
            df = gmsh.model.mesh.field.add("Distance")
            gmsh.model.mesh.field.setNumbers(df, "CurvesList", all_curves)
            gmsh.model.mesh.field.setNumber(df, "Sampling", 100)
            tf = gmsh.model.mesh.field.add("Threshold")
            gmsh.model.mesh.field.setNumber(tf, "InField", df)
            gmsh.model.mesh.field.setNumber(tf, "SizeMin", refine_size_mm)
            gmsh.model.mesh.field.setNumber(tf, "SizeMax", target_size_mm)
            gmsh.model.mesh.field.setNumber(tf, "DistMin", refine_size_mm)
            gmsh.model.mesh.field.setNumber(tf, "DistMax", target_size_mm * 3)
            mn = gmsh.model.mesh.field.add("Min")
            gmsh.model.mesh.field.setNumbers(mn, "FieldsList", [bg, tf])
            gmsh.model.mesh.field.setAsBackgroundMesh(mn)
        else:
            gmsh.model.mesh.field.setAsBackgroundMesh(bg)
        gmsh.option.setNumber("Mesh.CharacteristicLengthFromCurvature", 0)
        gmsh.option.setNumber("Mesh.CharacteristicLengthExtendFromBoundary", 0)
        gmsh.option.setNumber("Mesh.CharacteristicLengthFromPoints", 0)
        gmsh.option.setNumber("Mesh.Algorithm", 6)
        gmsh.option.setNumber("Mesh.Algorithm3D", 1)
        gmsh.option.setNumber("Mesh.OptimizeNetgen", 1)
        gmsh.model.mesh.generate(3)
        gmsh.write(out_msh_path)
    finally:
        gmsh.finalize()

    m = meshio.read(out_msh_path)
    num_nodes = len(m.points)
    num_tets = 0
    num_tris = 0
    for cb in m.cells:
        if cb.type == "tetra":
            num_tets += len(cb.data)
        elif cb.type == "triangle":
            num_tris += len(cb.data)
    return TetMesh(
        msh_path=out_msh_path,
        num_nodes=num_nodes,
        num_tets=num_tets,
        num_tris=num_tris,
        backend="gmsh-python",
    )


def _tet_via_gmsh_cli(stl_path: str,
                     out_msh_path: str,
                     target_size_mm: float,
                     refine_size_mm: float,
                     verbose: bool) -> TetMesh:
    """Shell out to ``gmsh`` CLI with a tiny .geo wrapper."""
    import meshio
    geo_path = out_msh_path + ".geo"
    with open(geo_path, "w") as f:
        f.write(
            f'Merge "{stl_path}";\n'
            f'Mesh.CharacteristicLengthMin = {refine_size_mm};\n'
            f'Mesh.CharacteristicLengthMax = {target_size_mm};\n'
            f'Mesh.Algorithm3D = 1;\n'
            f'Mesh.MshFileVersion = 2.2;\n'
            f'ClassifySurfaces{{Pi/3, 1, 1, Pi/3}};\n'
            f'CreateGeometry;\n'
            f'Surface Loop(1) = Surface{{:}};\n'
            f'Volume(1) = {{1}};\n'
        )
    cmd = ["gmsh", geo_path, "-3", "-format", "msh22", "-o", out_msh_path]
    if not verbose:
        cmd.append("-v")
        cmd.append("1")
    subprocess.run(cmd, check=True)
    m = meshio.read(out_msh_path)
    num_nodes = len(m.points)
    num_tets = 0
    num_tris = 0
    for cb in m.cells:
        if cb.type == "tetra":
            num_tets += len(cb.data)
        elif cb.type == "triangle":
            num_tris += len(cb.data)
    return TetMesh(
        msh_path=out_msh_path,
        num_nodes=num_nodes,
        num_tets=num_tets,
        num_tris=num_tris,
        backend="gmsh-cli",
    )


__all__ = ["TetMesh", "MesherUnavailable", "tetrahedralize"]
