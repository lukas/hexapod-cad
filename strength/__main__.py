"""CLI entry point for the opt-in strength pipeline.

Run via the Makefile (``make check-strength``) or directly::

    cd <repo>
    .venv/bin/python -m hexapod_walker.prototype.strength \
        --material petg --solver both

Flags
-----
* ``--material {pla,petg,onyx,abs}``  -- material card (default petg).
* ``--parts <names>``  -- comma-separated list, default = every
  load-bearing printed part (see ``load_cases.PRINTED_PARTS_FOR_STRENGTH``).
* ``--solver {beam,calculix,both}``  -- which path to run (default both).
* ``--report-only``  -- skip every solver, just rebuild the markdown
  from cached ``.frd`` artifacts.
* ``--no-png``  -- skip the per-part stress-field PNG rendering.
* ``--verbose``  -- pipe ccx / gmsh stdout through.
"""

from __future__ import annotations

import argparse
import os
import sys
import traceback

import trimesh

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.dirname(THIS_DIR)

import hexapod_prototype as hp

from strength import materials
from strength import load_cases
from strength import beam_check
from strength import mesh_part
from strength import run_calculix
from strength import report as report_mod

ARTIFACT_DIR = os.path.join(PROTO_DIR, "artifacts", "strength")
os.makedirs(ARTIFACT_DIR, exist_ok=True)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Opt-in strength / failure-point check for the hexapod's "
            "3D-printed parts.  Not wired into make check-cad."
        ),
    )
    p.add_argument("--material", "-m",
                   default="petg",
                   choices=sorted(materials.MATERIALS.keys()),
                   help="Filament material card (default: petg).")
    p.add_argument("--parts", "-p",
                   default=None,
                   help=(
                       "Comma-separated part names.  Default: every "
                       "load-bearing printed part."
                   ))
    p.add_argument("--solver", "-s",
                   default="both",
                   choices=["beam", "calculix", "both"],
                   help="Which solver path(s) to run (default: both).")
    p.add_argument("--report-only",
                   action="store_true",
                   help="Skip solvers; just rebuild the markdown from cached .frd files.")
    p.add_argument("--no-png",
                   action="store_true",
                   help="Skip stress-field PNGs.")
    p.add_argument("--verbose", "-v",
                   action="store_true",
                   help="Pipe gmsh / ccx stdout through to this terminal.")
    return p.parse_args()


def _select_parts(req: str | None) -> list[str]:
    if req is None:
        return list(load_cases.PRINTED_PARTS_FOR_STRENGTH)
    asked = [p.strip() for p in req.split(",") if p.strip()]
    unknown = [p for p in asked if p not in load_cases.PRINTED_PARTS_FOR_STRENGTH]
    if unknown:
        raise SystemExit(
            f"Unknown part(s): {unknown}.  "
            f"Known: {sorted(load_cases.PRINTED_PARTS_FOR_STRENGTH)}"
        )
    return asked


def main() -> int:
    args = _parse_args()
    material = materials.get(args.material)
    parts = _select_parts(args.parts)
    infill_factor = materials.DEFAULT_INFILL_FACTOR
    notes: list[str] = []

    print(f"[strength] material = {material.short_name} ({material.name})")
    print(f"[strength] parts    = {parts}")
    print(f"[strength] solver   = {args.solver}")
    print(f"[strength] artifact dir = {ARTIFACT_DIR}")

    # 1. Build the mass report from STL volumes + the fixed non-printed
    #    budget.  This drives every load case.
    mass = load_cases.build_mass_report(material)
    print(f"[strength] inferred assembled mass = {mass.total*1000:.1f} g "
          f"(printed {mass.printed_total*1000:.1f} g + "
          f"non-printed {mass.non_printed_total*1000:.1f} g)")
    cases = load_cases.build_all_cases(mass)

    mesher_available = (
        mesh_part._have_gmsh_python() or mesh_part._have_gmsh_cli()
    )
    fea_available = run_calculix.ccx_available()
    print(f"[strength] gmsh available    = {mesher_available}")
    print(f"[strength] ccx  available    = {fea_available}")
    if args.solver in ("calculix", "both") and not fea_available:
        notes.append(
            "CalculiX `ccx` not found on PATH.  Falling back to beam-bending "
            "check only.  Install via `brew install "
            "costerwi/homebrew-calculix/calculix-ccx` (macOS Intel / Linux); "
            "on Apple Silicon the brew formula currently fails to link "
            "ARPACK -- track upstream issue or use a Docker CCX image."
        )
    if args.solver in ("calculix", "both") and not mesher_available:
        notes.append(
            "Gmsh not available -- cannot tetrahedralize the STLs for the "
            "CalculiX deck.  Install via `brew install gmsh` and "
            "re-run.  The beam-bending pre-check still ran."
        )

    rows: list[report_mod.PartRow] = []
    fea_rows_with_frd: list[tuple[report_mod.PartRow, str]] = []

    for part in parts:
        case = cases[part]
        # hp.stl_path resolves reference-mesh parts (e.g. the fused
        # tibia_link, which lives in stl_reference/ with a _DO_NOT_PRINT
        # suffix) as well as the plain stl_prototype/ printables.
        stl_path = hp.stl_path(part)
        if not os.path.exists(stl_path):
            print(f"[strength] WARN: {stl_path} missing; skipping {part}")
            continue
        mesh = trimesh.load(stl_path, force="mesh")
        beam_res = None
        if args.solver in ("beam", "both"):
            print(f"[strength] beam check {part!r} ... ", end="", flush=True)
            beam_res = beam_check.run_beam_check(
                case, mesh, material, infill_factor=infill_factor,
            )
            if isinstance(beam_res, beam_check.BeamResult):
                print(
                    f"sigma_peak = {beam_res.peak_bending_stress_Pa/1e6:.2f} MPa, "
                    f"SF = {beam_res.safety_factor:.2f}"
                )
            else:
                print(f"skipped ({beam_res.reason})")

        fea_res = None
        fea_error: str | None = None
        if args.solver in ("calculix", "both") and not args.report_only:
            if fea_available and mesher_available:
                try:
                    print(f"[strength] meshing {part!r} ...", flush=True)
                    tet = mesh_part.tetrahedralize(
                        stl_path,
                        target_size_mm=2.0,
                        refine_size_mm=1.0,
                        verbose=args.verbose,
                    )
                    print(
                        f"[strength]   tet mesh: "
                        f"{tet.num_nodes} nodes, {tet.num_tets} tets "
                        f"({tet.backend})"
                    )
                    print(f"[strength] running ccx for {part!r} ...", flush=True)
                    fea_res = run_calculix.run_case(
                        tet.msh_path, case, material,
                        infill_factor=infill_factor,
                        verbose=args.verbose,
                    )
                    print(
                        f"[strength]   peak von Mises = "
                        f"{fea_res.peak_von_mises_Pa/1e6:.2f} MPa, "
                        f"peak displacement = "
                        f"{fea_res.peak_displacement_m*1e3:.3f} mm"
                    )
                except (run_calculix.CCXUnavailable,
                        mesh_part.MesherUnavailable) as e:
                    fea_error = f"{type(e).__name__}: {e}"
                    print(f"[strength] FEA unavailable: {fea_error}")
                except Exception as e:
                    fea_error = (
                        f"{type(e).__name__}: {e}\n"
                        + traceback.format_exc(limit=2)
                    )
                    print(f"[strength] FEA failed: {fea_error}")
            else:
                missing = []
                if not run_calculix.ccx_available():
                    missing.append("ccx")
                if not (mesh_part._have_gmsh_python()
                        or mesh_part._have_gmsh_cli()):
                    missing.append("gmsh")
                fea_error = f"{'/'.join(missing)} not on PATH"

        row = report_mod.PartRow(
            part=part,
            case_name=case.name,
            case_description=case.description,
            beam=beam_res,
            fea=fea_res,
            fea_error=fea_error,
            material_short_name=material.short_name,
        )
        rows.append(row)
        if isinstance(fea_res, run_calculix.FEAResult):
            fea_rows_with_frd.append((row, fea_res.frd_path))

    # 3. Render PNGs for the top 3 highest-stress parts (FEA only).
    if not args.no_png and fea_rows_with_frd:
        fea_rows_with_frd.sort(
            key=lambda kv: kv[0].fea.peak_von_mises_Pa, reverse=True,
        )
        top3 = fea_rows_with_frd[:3]
        for row, frd_path in top3:
            stl_path = hp.stl_path(row.part)
            png_path = os.path.join(
                ARTIFACT_DIR,
                f"{row.part}_{material.short_name}_{row.case_name}.png",
            )
            try:
                print(f"[strength] rendering stress PNG for {row.part} -> {png_path}")
                report_mod.render_stress_png(stl_path, frd_path, png_path)
            except Exception as e:
                print(f"[strength] PNG render failed for {row.part}: {e}")

    # 4. Write the markdown.
    paths = report_mod.write_report(
        rows, mass, material,
        infill_factor=infill_factor,
        fea_available=fea_available,
        mesher_available=mesher_available,
        notes=notes,
    )
    for p in paths:
        print(f"[strength] wrote {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
