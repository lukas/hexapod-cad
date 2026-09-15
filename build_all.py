"""Generate every printable tabletop-prototype STL bundle.

Run from the repository root:

    ./run.sh hexapod_walker/prototype_sts3215/build_all.py

This writes:
    - step_prototype/  STEP-first print-set: per-printable .step (the
                       editable CAD truth) + BREP tessellations + manifest
    - stl_prototype/   slicer-ready printables only (installed from the
                       healed BREP tessellations -- STEP-first since Aug 2026)
    - stl_reference/   bought-part / fused-link visuals (MuJoCo, BuildViz)
    - artifacts/assembly/  visual category STLs (optional)

The printables' geometry is authored as build123d BREP solids
(``cad_step_test/build_step_first_test.py``; constants imported from
``hexapod_prototype.py``).  ``build_step_prototype.py`` exports them and
PROVES each part equivalent to its trimesh twin (volume / bbox / max
surface deviation gates) before the print set is touched; the trimesh
builders remain the source for the visuals, the verifier's probes,
MuJoCo and BuildViz.  See ``step_pipeline.py`` for the layout.

Bambu tray / test-plate packing was removed — print individuals from
``stl_prototype/``.  Print poses live in ``scripts/print_orientation.py``.
"""

from __future__ import annotations

import argparse
import importlib
import os
import subprocess
import sys
import time
from collections.abc import Callable


HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, "scripts")
if SCRIPTS not in sys.path:
    # build_prototype_assembly / print_orientation live under scripts/
    sys.path.insert(0, SCRIPTS)


def _run(label: str, module_name: str) -> None:
    print()
    print("=" * 72)
    print(label)
    print("=" * 72)
    started = time.monotonic()
    module = importlib.import_module(module_name)
    main: Callable[..., None] = module.main
    try:
        main(argv=[])
    except TypeError:
        main()
    elapsed = time.monotonic() - started
    print(f"{label} finished in {elapsed:.1f}s")


def _run_step_first_print_set() -> None:
    """Regenerate the STEP-first print set: run the BREP exporter (its own
    uv 3.12 interpreter -- build123d has no 3.14 wheels) which exports
    step_prototype/ and enforces the per-part equivalence gates, then
    install the healed tessellations into stl_prototype/, REPLACING the
    trimesh twins hexapod_prototype.main() just wrote (the twins stay on
    disk in stl_reference/ + in-memory as the verifier's probe source)."""
    print()
    print("=" * 72)
    print("STEP-first print set (BREP -> STEP -> STL)")
    print("=" * 72)
    started = time.monotonic()
    import step_pipeline
    subprocess.run(step_pipeline.brep_export_cmd(), check=True, cwd=HERE)
    step_pipeline.install_printables()
    elapsed = time.monotonic() - started
    print(f"STEP-first print set finished in {elapsed:.1f}s")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--skip-assembly",
        action="store_true",
        help="Skip artifacts/assembly/*.stl visual exports.",
    )
    # Deprecated: the Xometry order-bundle pipeline was removed entirely
    # (Aug 2026); flag kept so old Makefile / muscle-memory commands
    # don't crash.
    parser.add_argument("--skip-xometry", action="store_true",
                        help=argparse.SUPPRESS)
    parser.add_argument(
        "--with-arm",
        action="store_true",
        help=(
            "Also build the OPTIONAL 5-DOF arm add-on (arm/). "
            "Regenerates stl_arm/*.stl and runs the arm-inclusive "
            "assembly preview."
        ),
    )
    # Deprecated flags kept so old Makefile / muscle-memory commands
    # don't crash (trays were removed).
    parser.add_argument("--skip-bambu", action="store_true",
                        help=argparse.SUPPRESS)
    parser.add_argument("--skip-test-plate", action="store_true",
                        help=argparse.SUPPRESS)
    args = parser.parse_args()

    os.chdir(HERE)
    print(f"Generating tabletop hexapod prototype STL bundles in {HERE}")

    _run("Individual prototype STLs", "hexapod_prototype")
    _run_step_first_print_set()
    if not args.skip_assembly:
        if args.with_arm:
            os.environ["HEXAPOD_PROTOTYPE_WITH_ARM"] = "1"
        try:
            _run("Prototype assembly STLs", "build_prototype_assembly")
        finally:
            os.environ.pop("HEXAPOD_PROTOTYPE_WITH_ARM", None)
    if args.with_arm:
        import importlib.util  # noqa: WPS433
        _arm_dir = os.path.join(HERE, "arm")
        if _arm_dir not in __import__("sys").path:
            __import__("sys").path.insert(0, _arm_dir)
        _spec = importlib.util.spec_from_file_location(
            "arm_integrate", os.path.join(_arm_dir, "integrate.py"))
        _arm_integrate = importlib.util.module_from_spec(_spec)
        _spec.loader.exec_module(_arm_integrate)
        _arm_integrate.build_with_arm(
            render_preview=True,
            build_bambu=False,
        )

    print()
    print("Done. Printables: stl_prototype/ (from step_prototype/ BREP)  ·  "
          "sim visuals: stl_reference/")
    if args.with_arm:
        print("Optional arm: arm/stl_arm/ (see arm/ARM.md).")


if __name__ == "__main__":
    main()
