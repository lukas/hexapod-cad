"""Write a CalculiX ``.inp`` deck for one load case, run ``ccx``, and parse the ``.frd``.

The deck we emit is LINEAR static, isotropic, single-step, with:

* one solid-element block (C3D4) of the part's tets
* one material card from ``materials.Material``
* one set of NSET constraints (BOUNDARY) for each clamp region
* one set of CLOAD point loads + DLOAD pressure tractions for each load
* a single ``*STEP, NLGEOM=NO`` block with ``*STATIC`` and a
  request for nodal stress + displacement output to the ``.frd``

The parser pulls nodal von Mises and (u, v, w) displacement directly
from the FRD ascii format (CalculiX's default text format).  We do NOT
parse the binary ``.dat`` -- nothing in this pipeline needs the
per-element results.

If ``ccx`` is unavailable we raise ``CCXUnavailable`` so the caller can
fall back to beam-bending only.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from dataclasses import dataclass

import numpy as np

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.dirname(THIS_DIR)
ARTIFACT_DIR = os.path.join(PROTO_DIR, "artifacts", "strength")
os.makedirs(ARTIFACT_DIR, exist_ok=True)


class CCXUnavailable(RuntimeError):
    """The CalculiX ``ccx`` binary is not on PATH."""


@dataclass
class FEAResult:
    """Nodal stress + displacement field from one ccx run."""
    case_name: str
    part: str
    inp_path: str
    frd_path: str
    num_nodes: int
    num_elements: int
    peak_von_mises_Pa: float
    peak_displacement_m: float
    peak_node_id: int
    peak_node_xyz_m: tuple[float, float, float]
    material_short_name: str


def ccx_available() -> bool:
    return shutil.which("ccx") is not None or shutil.which("ccx_2.23") is not None


def _ccx_executable() -> str:
    for cand in ("ccx", "ccx_2.23", "ccx_2.20", "ccx_2.22"):
        path = shutil.which(cand)
        if path is not None:
            return path
    raise CCXUnavailable(
        "ccx binary not found on PATH.  Install via "
        "``brew install costerwi/homebrew-calculix/calculix-ccx`` "
        "(macOS) or ``apt install calculix-ccx`` (Ubuntu)."
    )


# ---------------------------------------------------------------------------
# Mesh helpers: write the .inp from the gmsh .msh
# ---------------------------------------------------------------------------


def _read_msh_nodes_and_tets(msh_path: str):
    """Return (nodes, tets) as float / int numpy arrays.

    nodes is (N, 3) in mm (Gmsh native).  tets is (M, 4) with 1-based
    Gmsh node ids -- CalculiX accepts arbitrary integer ids so we
    just renumber 1..N to keep the deck small.
    """
    import meshio
    m = meshio.read(msh_path)
    pts = np.asarray(m.points, dtype=float)
    tet_cells = None
    for cb in m.cells:
        if cb.type == "tetra":
            tet_cells = np.asarray(cb.data, dtype=int)
            break
    if tet_cells is None:
        raise ValueError(f"No tetra elements in {msh_path!r}")
    return pts, tet_cells


def _clamp_nodeset(nodes_mm: np.ndarray, clamp) -> np.ndarray:
    """Return the 1-based node ids hit by ``clamp``.

    ``clamp`` is a ``load_cases.ClampedRegion``; ``nodes_mm`` is the
    (N, 3) node array in mm (Gmsh native).  Distances are computed in
    metres after a 1e-3 scale.
    """
    pts_m = nodes_mm * 1.0e-3
    if clamp.kind == "bolt_circle":
        cx, cy, cz = clamp.centre_m
        if clamp.axis == "z":
            # Hits within radius in XY, near plane z = cz +/- tol.
            dxy = np.sqrt((pts_m[:, 0] - cx) ** 2 + (pts_m[:, 1] - cy) ** 2)
            dz = np.abs(pts_m[:, 2] - cz)
            mask = (dxy <= clamp.radius_m + 1.5e-3) & (dz <= 5.0e-3)
        elif clamp.axis == "y":
            dxz = np.sqrt((pts_m[:, 0] - cx) ** 2 + (pts_m[:, 2] - cz) ** 2)
            dy = np.abs(pts_m[:, 1] - cy)
            mask = (dxz <= clamp.radius_m + 1.5e-3) & (dy <= 5.0e-3)
        elif clamp.axis == "x":
            dyz = np.sqrt((pts_m[:, 1] - cy) ** 2 + (pts_m[:, 2] - cz) ** 2)
            dx = np.abs(pts_m[:, 0] - cx)
            mask = (dyz <= clamp.radius_m + 1.5e-3) & (dx <= 5.0e-3)
        else:
            raise ValueError(f"Unknown bolt-circle axis {clamp.axis!r}")
    elif clamp.kind == "face":
        if clamp.axis == "z":
            coord = pts_m[:, 2]
        elif clamp.axis == "y":
            coord = pts_m[:, 1]
        else:
            coord = pts_m[:, 0]
        extreme = coord.max() if (clamp.sign or 0) > 0 else coord.min()
        mask = np.abs(coord - extreme) <= 0.5e-3
    elif clamp.kind == "edge_ring":
        cx, cy, cz = clamp.centre_m
        d = np.sqrt(
            (pts_m[:, 0] - cx) ** 2
            + (pts_m[:, 1] - cy) ** 2
            + (pts_m[:, 2] - cz) ** 2
        )
        mask = np.abs(d - clamp.radius_m) <= 1.0e-3
    else:
        raise ValueError(f"Unknown clamp kind {clamp.kind!r}")
    return np.where(mask)[0] + 1


def _point_load_nodeset(nodes_mm: np.ndarray, load) -> np.ndarray:
    """Return the single closest node id (1-based) to ``load.location_m``."""
    pts_m = nodes_mm * 1.0e-3
    target = np.array(load.location_m)
    d = np.linalg.norm(pts_m - target[None, :], axis=1)
    return np.array([int(np.argmin(d)) + 1])


def _face_nodeset(nodes_mm: np.ndarray, traction) -> np.ndarray:
    """All nodes on the +/-axis face named by ``traction``."""
    pts_m = nodes_mm * 1.0e-3
    if traction.axis == "z":
        coord = pts_m[:, 2]
    elif traction.axis == "y":
        coord = pts_m[:, 1]
    else:
        coord = pts_m[:, 0]
    extreme = coord.max() if traction.sign > 0 else coord.min()
    mask = np.abs(coord - extreme) <= 0.5e-3
    return np.where(mask)[0] + 1


# ---------------------------------------------------------------------------
# .inp deck writer
# ---------------------------------------------------------------------------


def write_inp(msh_path: str,
              case,             # load_cases.LoadCase
              material,         # materials.Material
              *,
              out_inp_path: str | None = None,
              infill_factor: float = 0.65) -> str:
    """Write a CalculiX .inp deck and return its path."""
    if out_inp_path is None:
        base = os.path.splitext(os.path.basename(msh_path))[0]
        out_inp_path = os.path.join(
            ARTIFACT_DIR,
            f"{base}_{material.short_name}_{case.name}.inp",
        )
    nodes_mm, tets = _read_msh_nodes_and_tets(msh_path)
    # nodes_mm is mm; CCX is unit-agnostic so we pick SI (m).  Convert.
    nodes_m = nodes_mm * 1.0e-3
    E_eff = material.E_iso * infill_factor
    sigma_y_eff = material.sigma_y_iso * infill_factor / material.brittleness_factor

    # Build clamp / load nodesets.
    clamp_sets = []
    for i, clamp in enumerate(case.clamps):
        ids = _clamp_nodeset(nodes_mm, clamp)
        clamp_sets.append((f"CLAMP{i}", ids, clamp))
    load_sets = []
    for i, load in enumerate(case.loads):
        ids = _point_load_nodeset(nodes_mm, load)
        load_sets.append((f"LOAD{i}", ids, load))
    traction_sets = []
    for i, trac in enumerate(case.tractions):
        ids = _face_nodeset(nodes_mm, trac)
        traction_sets.append((f"TRAC{i}", ids, trac))

    with open(out_inp_path, "w") as f:
        f.write(f"** CalculiX deck for {case.part!r} / {case.name!r}\n")
        f.write(f"** Material: {material.name}\n")
        f.write(f"** E_eff = {E_eff:.3e} Pa, sigma_y_eff = {sigma_y_eff:.3e} Pa\n")
        f.write("**\n")
        f.write("*NODE, NSET=NALL\n")
        for nid, (x, y, z) in enumerate(nodes_m, start=1):
            f.write(f"{nid}, {x:.6e}, {y:.6e}, {z:.6e}\n")
        f.write("*ELEMENT, TYPE=C3D4, ELSET=EALL\n")
        for eid, tet in enumerate(tets, start=1):
            n1, n2, n3, n4 = (int(x) + 1 for x in tet)
            f.write(f"{eid}, {n1}, {n2}, {n3}, {n4}\n")
        # Section + material card
        f.write("*MATERIAL, NAME=MAT_PRINT\n")
        f.write("*ELASTIC\n")
        f.write(f"{E_eff:.6e}, {material.nu:.4f}\n")
        f.write(f"*DENSITY\n{material.density:.6e}\n")
        f.write("*SOLID SECTION, ELSET=EALL, MATERIAL=MAT_PRINT\n")
        # Node sets
        for name, ids, _clamp in clamp_sets:
            f.write(f"*NSET, NSET={name}\n")
            _write_nset(f, ids)
        for name, ids, _load in load_sets:
            f.write(f"*NSET, NSET={name}\n")
            _write_nset(f, ids)
        for name, ids, _trac in traction_sets:
            f.write(f"*NSET, NSET={name}\n")
            _write_nset(f, ids)
        # Boundary conditions (full 6-dof clamp on every node in each
        # clamp set; degrees of freedom 1,2,3 in CalculiX).
        f.write("*BOUNDARY\n")
        for name, _ids, _clamp in clamp_sets:
            f.write(f"{name}, 1, 3, 0.0\n")
        # Step
        f.write("*STEP\n")
        f.write("*STATIC\n")
        # Concentrated loads
        for name, ids, load in load_sets:
            fx, fy, fz = load.force_N
            # Distribute equally across the (1-node) set.
            n = max(len(ids), 1)
            f.write("*CLOAD\n")
            f.write(f"{name}, 1, {fx/n:.6e}\n")
            f.write(f"{name}, 2, {fy/n:.6e}\n")
            f.write(f"{name}, 3, {fz/n:.6e}\n")
        # Pressure tractions: emit a *CLOAD per node in the face set
        # equal to (pressure * face_area / N_nodes) along the inward
        # normal.  Approximation: face area ~ disk area / N_nodes per
        # node.  For the foot pad face this is exact (uniform).
        for name, ids, trac in traction_sets:
            # Disk area in m^2 -- estimate from nodes on the face.
            face_nodes = nodes_m[ids - 1]
            if trac.axis == "z":
                xs = face_nodes[:, 0]
                ys = face_nodes[:, 1]
                # Use circumscribed bounding circle radius for area
                # estimate.
                r2 = np.max(xs * xs + ys * ys)
                area = np.pi * r2
            elif trac.axis == "y":
                xs = face_nodes[:, 0]
                zs = face_nodes[:, 2]
                r2 = np.max(xs * xs + zs * zs)
                area = np.pi * r2
            else:
                ys = face_nodes[:, 1]
                zs = face_nodes[:, 2]
                r2 = np.max(ys * ys + zs * zs)
                area = np.pi * r2
            total_force = trac.pressure_Pa * area * trac.sign * -1.0
            # sign < 0 + pressure_Pa > 0 -> compressive into the -axis
            # face -> force is in the +axis direction at the nodes
            # we flip with sign * -1 above.
            n = max(len(ids), 1)
            f.write("*CLOAD\n")
            axis_dof = {"x": 1, "y": 2, "z": 3}[trac.axis]
            f.write(f"{name}, {axis_dof}, {total_force/n:.6e}\n")
        # Output requests
        f.write("*NODE FILE\nU,S\n")
        f.write("*EL FILE\nS\n")
        f.write("*END STEP\n")
    return out_inp_path


def _write_nset(f, ids: np.ndarray):
    # CalculiX accepts up to 16 ids per line.
    ids = list(int(x) for x in ids)
    for i in range(0, len(ids), 16):
        chunk = ids[i:i + 16]
        f.write(",".join(str(x) for x in chunk) + "\n")


# ---------------------------------------------------------------------------
# ccx invocation
# ---------------------------------------------------------------------------


def run(inp_path: str, *, verbose: bool = False) -> str:
    """Run ``ccx`` on ``inp_path`` and return the resulting ``.frd`` path.

    CalculiX runs in the directory of the ``.inp`` and writes
    ``<basename>.frd`` next to it.  We let it do that and return the
    path.
    """
    ccx = _ccx_executable()
    work_dir = os.path.dirname(inp_path)
    base = os.path.splitext(os.path.basename(inp_path))[0]
    cmd = [ccx, base]
    proc = subprocess.run(
        cmd, cwd=work_dir,
        capture_output=not verbose,
        text=True,
    )
    if proc.returncode != 0:
        out = (proc.stdout or "") + "\n" + (proc.stderr or "")
        raise RuntimeError(f"ccx failed (rc {proc.returncode}):\n{out}")
    frd_path = os.path.join(work_dir, base + ".frd")
    if not os.path.exists(frd_path):
        raise RuntimeError(
            f"ccx ran but produced no .frd at {frd_path!r} -- "
            "check ccx stderr for hints."
        )
    return frd_path


# ---------------------------------------------------------------------------
# .frd parser  --  pull nodal von Mises + displacement from the ASCII format
# ---------------------------------------------------------------------------


def _parse_frd_record(line: str, n_values: int) -> tuple[int, list[float]]:
    """Parse a CalculiX ``-1`` fixed-width result record.

    FRD floats are 12-character fields and may be adjacent with no
    separating whitespace, e.g. ``1-2.61052E-03`` after the node id.
    """
    nid = int(line[3:13])
    values = [
        float(line[13 + 12 * i:25 + 12 * i])
        for i in range(n_values)
    ]
    return nid, values


def parse_frd(frd_path: str):
    """Return (nodes (N,3) in m, displacement (N,3) in m, von_mises (N,) in Pa).

    CalculiX writes a fixed-column ASCII ``.frd``.  Format reference:
    Wittig's CCX manual section "Result file format", or the
    cgx-supplied ``frd.h``.  We only need the NODES block, the DISP
    field, and the STRESS field (from which we compute von Mises).
    """
    with open(frd_path, "r") as f:
        lines = f.readlines()

    nodes: dict[int, np.ndarray] = {}
    disp: dict[int, np.ndarray] = {}
    stress: dict[int, np.ndarray] = {}  # 6 components per node: xx, yy, zz, xy, yz, zx

    i = 0
    while i < len(lines):
        line = lines[i]
        # Nodes block starts with "    2C" header.  The actual node
        # records are "-1   <id> <x> <y> <z>".
        if line.startswith("    2C"):
            i += 1
            while i < len(lines) and lines[i].startswith(" -1"):
                nid, (x, y, z) = _parse_frd_record(lines[i], 3)
                nodes[nid] = np.array([x, y, z])
                i += 1
            continue
        # Displacement field block starts with "    1PSTEP" / "    100C"
        # but the field block itself opens with "  -4  DISP" and the
        # records are "-1   <id> <u> <v> <w>".
        if line.startswith(" -4  DISP"):
            # Skip the -5 sub-headers.
            while i < len(lines) and not lines[i].startswith(" -1"):
                i += 1
            while i < len(lines) and lines[i].startswith(" -1"):
                nid, (u, v, w) = _parse_frd_record(lines[i], 3)
                disp[nid] = np.array([u, v, w])
                i += 1
            continue
        # Stress field block.  Six components per node: SXX SYY SZZ SXY SYZ SZX.
        if line.startswith(" -4  STRESS"):
            while i < len(lines) and not lines[i].startswith(" -1"):
                i += 1
            while i < len(lines) and lines[i].startswith(" -1"):
                nid, vals = _parse_frd_record(lines[i], 6)
                stress[nid] = np.array(vals)
                i += 1
            continue
        i += 1

    if not nodes:
        raise RuntimeError(f"No nodes parsed from {frd_path!r}")

    n_ids = sorted(nodes.keys())
    n_xyz = np.array([nodes[i] for i in n_ids])
    u = np.array([disp.get(i, np.zeros(3)) for i in n_ids])
    s = np.array([stress.get(i, np.zeros(6)) for i in n_ids])
    # von Mises from the 6 stress components.
    sxx, syy, szz, sxy, syz, szx = s.T
    vm = np.sqrt(0.5 * (
        (sxx - syy) ** 2 + (syy - szz) ** 2 + (szz - sxx) ** 2
        + 6.0 * (sxy * sxy + syz * syz + szx * szx)
    ))
    return n_ids, n_xyz, u, vm


# ---------------------------------------------------------------------------
# Top-level: mesh -> write inp -> run ccx -> parse frd
# ---------------------------------------------------------------------------


def run_case(msh_path: str,
             case,
             material,
             *,
             infill_factor: float = 0.65,
             verbose: bool = False) -> FEAResult:
    """Full ccx pipeline for one ``(msh, case, material)`` triple."""
    if not ccx_available():
        raise CCXUnavailable(
            "ccx not found on PATH; FEA pass cannot run.  "
            "Install via ``brew install costerwi/homebrew-calculix/calculix-ccx`` "
            "or apt, or set --solver beam to skip."
        )
    inp_path = write_inp(
        msh_path, case, material,
        infill_factor=infill_factor,
    )
    frd_path = run(inp_path, verbose=verbose)
    n_ids, n_xyz, u, vm = parse_frd(frd_path)
    disp_norm = np.linalg.norm(u, axis=1)
    peak_idx = int(np.argmax(vm))
    peak_node_xyz = tuple(float(c) for c in n_xyz[peak_idx])
    return FEAResult(
        case_name=case.name,
        part=case.part,
        inp_path=inp_path,
        frd_path=frd_path,
        num_nodes=len(n_ids),
        num_elements=0,
        peak_von_mises_Pa=float(np.max(vm)),
        peak_displacement_m=float(np.max(disp_norm)),
        peak_node_id=int(n_ids[peak_idx]),
        peak_node_xyz_m=peak_node_xyz,
        material_short_name=material.short_name,
    )


__all__ = [
    "CCXUnavailable", "FEAResult", "ccx_available",
    "write_inp", "run", "parse_frd", "run_case",
]
