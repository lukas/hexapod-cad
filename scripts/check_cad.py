"""Top-level orchestrator for the hexapod prototype CAD check.

Run order:

    1. ``build`` -- ensure ``stl_prototype/*.stl`` exist (re-runs
       ``build_all.py`` if anything is missing).
    2. ``validate`` -- run ``scripts/validate_geometry.py`` against
       ``design_spec.yaml`` and (by default) the full
       ``_verify_prototype.main()`` suite.
    3. ``render`` -- run ``scripts/render_views.py`` to dump
       4-view PNGs to ``artifacts/views/<part>/``.
    4. ``report`` -- write ``artifacts/cad_report.md`` summarising
       all of the above.

Exit code is non-zero on any sub-step failure.

Run from the repo root:

    ./run.sh hexapod_walker/prototype/scripts/check_cad.py

Or:

    make -C hexapod_walker/prototype check-cad
"""

from __future__ import annotations

import argparse
import datetime
import os
import sys
import textwrap
import time

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.dirname(THIS_DIR)

# Add the prototype dir (for `hexapod_prototype`, `_verify_prototype`,
# `keepout_volumes`) AND this `scripts/` dir (for sibling-script
# imports below).
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

ARTIFACTS_DIR = os.path.join(PROTO_DIR, "artifacts")
REPORT_PATH = os.path.join(ARTIFACTS_DIR, "cad_report.md")
VIEWS_DIR = os.path.join(ARTIFACTS_DIR, "views")


# ---------------------------------------------------------------------------
# Sub-steps
# ---------------------------------------------------------------------------

def _heading(title: str) -> None:
    bar = "=" * 72
    print()
    print(bar)
    print(title)
    print(bar)


def _build(verbose: bool) -> bool:
    """Ensure the per-part STLs exist (don't re-run if they're already
    on disk -- saves several seconds on subsequent invocations)."""
    from validate_geometry import _ensure_stls_built  # noqa: WPS433

    needed = [
        "chassis_top", "chassis_bottom",
        "coxa_bracket", "coxa_link",
        "femur_link", "tibia_link",
        "foot_boot",
    ]
    _heading("[1/4] Ensure per-part STLs are present")
    _ensure_stls_built(needed)
    print("  STLs present.")
    return True


def _validate(fast: bool, run_verify_prototype: bool):
    from validate_geometry import run_validation, _print_report
    _heading("[2/4] Validate geometry vs design_spec.yaml")
    report = run_validation(
        fast=fast,
        run_verify_prototype=run_verify_prototype,
        skip_build=True,
    )
    _print_report(report)
    return report


def _render(skip_render: bool):
    _heading("[3/4] Render 4-view PNGs per part")
    if skip_render:
        print("  --skip-render set; not generating new images.")
        return []
    from render_views import render_all
    return render_all(out_root=VIEWS_DIR)


def _write_report(validation_report, render_results,
                  fast: bool, skip_render: bool) -> str:
    _heading("[4/4] Write cad_report.md")
    os.makedirs(ARTIFACTS_DIR, exist_ok=True)
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    lines: list[str] = []
    lines.append(f"# Hexapod prototype CAD check report")
    lines.append("")
    lines.append(f"_Generated: {now}._")
    lines.append("")
    lines.append(f"- Validator: `hexapod_walker/prototype/scripts/"
                 f"validate_geometry.py`")
    lines.append(f"- Design spec: `hexapod_walker/prototype/"
                 f"design_spec.yaml`")
    lines.append(f"- Keep-out registry: `hexapod_walker/prototype/"
                 f"keepout_volumes.py`")
    lines.append(f"- Fast mode: **{fast}**")
    lines.append(f"- Rendering: "
                 f"**{'skipped' if skip_render else 'on'}**")
    lines.append("")

    # ----- Summary checklist
    lines.append("## Pass / fail checklist")
    lines.append("")
    all_ok = validation_report.passed
    lines.append(f"- **Overall:** "
                 f"{'PASS' if all_ok else 'FAIL'}")
    for part in validation_report.parts:
        part_ok = all(c.passed for c in part.checks)
        lines.append(f"  - `{part.name}`: "
                     f"{'PASS' if part_ok else 'FAIL'}")
    for grp in validation_report.group_results:
        lines.append(f"  - `{grp.name}`: "
                     f"{'PASS' if grp.passed else 'FAIL'}")
    lines.append("")

    # ----- Discovered dimensions
    lines.append("## Discovered dimensions")
    lines.append("")
    lines.append("| Part | Bounds (mm) | Volume (cm³) | Watertight | "
                 "Holes ✓ | Channels ✓ | Keep-outs ✓ |")
    lines.append("|---|---|---|---|---|---|---|")
    for p in validation_report.parts:
        bb = p.bounds_observed_mm
        lines.append(
            f"| `{p.name}` | "
            f"{bb[0]:.1f} × {bb[1]:.1f} × {bb[2]:.1f} | "
            f"{p.volume_mm3 / 1000.0:.1f} | "
            f"{'yes' if p.watertight else '**no**'} | "
            f"{p.holes_passed}/{p.holes_checked} | "
            f"{p.wire_channels_passed}/{p.wire_channels_checked} | "
            f"{p.keep_outs_passed}/{p.keep_outs_checked} |"
        )
    lines.append("")

    # ----- Missing assumptions
    if validation_report.todo_fields:
        lines.append("## Missing assumptions")
        lines.append("")
        lines.append("These fields in `design_spec.yaml` are currently "
                     "`TODO`.  Fill them in so the spec is a complete "
                     "contract.")
        lines.append("")
        for field in validation_report.todo_fields:
            lines.append(f"- `{field}`")
        lines.append("")

    # ----- Failure details (copy-pastable into an LLM)
    failing_details = []
    for p in validation_report.parts:
        for chk in p.checks:
            if not chk.passed:
                failing_details.append(
                    f"- `{p.name}::{chk.name}`: {chk.detail}"
                )
    if failing_details:
        lines.append("## Failure details")
        lines.append("")
        lines.append("Each line below is copy-pastable as context for "
                     "an LLM coding agent that should fix the issue.")
        lines.append("")
        lines.extend(failing_details)
        lines.append("")

    # ----- Shape: block warnings (RULE 7 -- WARN only, never flips
    # PASS/FAIL).  Surface them here so a part-by-part rollout of the
    # ``shape:`` anatomy block is easy to track.
    shape_warnings = []
    for p in validation_report.parts:
        for warn in getattr(p, "warnings", []) or []:
            shape_warnings.append(
                f"- `{p.name}::{warn.name}`: {warn.detail}"
            )
    if shape_warnings:
        lines.append("## Shape: warnings (RULE 7)")
        lines.append("")
        lines.append(
            "These are part-by-part rollout WARNINGS for the "
            "`shape:` anatomy block added in May 2026 (see RULE 7 "
            "at the top of `design_spec.yaml`).  They do NOT flip "
            "the part's PASS/FAIL bit -- they're surfaced here so a "
            "rebuild-from-spec engineer can see which parts still "
            "need their feature list filled in."
        )
        lines.append("")
        lines.extend(shape_warnings)
        lines.append("")

    for grp in validation_report.group_results:
        if not grp.passed and grp.detail:
            lines.append(f"### `{grp.name}` output")
            lines.append("")
            lines.append("```")
            lines.extend(textwrap.dedent(grp.detail).splitlines())
            lines.append("```")
            lines.append("")

    # ----- Generated images (relative paths under hexapod_walker/prototype/)
    lines.append("## Generated images")
    lines.append("")
    if not render_results:
        lines.append("_No images this run "
                     "(use `--skip-render` to disable rendering)._")
    else:
        for r in render_results:
            lines.append(f"### `{r.part}`  (rendered via {r.backend})")
            lines.append("")
            for p in r.output_paths:
                rel = os.path.relpath(p, PROTO_DIR)
                lines.append(f"- `{rel}`")
            lines.append("")
    lines.append("")

    # ----- Auto-derived fastener table (mirrors docs/BOM.md
    # "Fasteners" section; kept in sync via scripts/render_fastener_bom.py).
    try:
        import render_fastener_bom  # type: ignore  # noqa: WPS433
        lines.append("## Fasteners (auto-derived)")
        lines.append("")
        lines.append(render_fastener_bom.render_table())
        lines.append("")
    except Exception as exc:  # noqa: BLE001
        # Reporting the fastener BOM is a nice-to-have; never let it
        # break the report write.
        lines.append("## Fasteners")
        lines.append("")
        lines.append(f"_render_fastener_bom unavailable: {exc!r}_")
        lines.append("")

    # ----- Helpful tail
    lines.append("## What to do next")
    lines.append("")
    lines.append("- If a PART check failed, the most useful diagnostic "
                 "is the matching `_verify_prototype` line under "
                 "`_verify_prototype.main()` output below.")
    lines.append("- Re-run with `make -C hexapod_walker/prototype "
                 "check-cad` after each geometry edit.")
    lines.append("- See `docs/CAD_AGENT_INSTRUCTIONS.md` and "
                 "`docs/CAD_WORKFLOW.md` for the design-spec contract.")
    lines.append("")

    text = "\n".join(lines)
    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"  wrote {REPORT_PATH}")
    return REPORT_PATH


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.
                                      RawDescriptionHelpFormatter)
    parser.add_argument(
        "--fast",
        action="store_true",
        help=("Skip the slow `_verify_prototype` workspace sweep "
              "(reduces it to a single pose).  Default off."),
    )
    parser.add_argument(
        "--skip-verify-prototype",
        action="store_true",
        help="Skip the legacy _verify_prototype.main() sub-step.",
    )
    parser.add_argument(
        "--skip-render",
        action="store_true",
        help="Don't render PNGs (skip step 3).",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help=("Don't try to (re-)build STLs (assume they're already "
              "in stl_prototype/)."),
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Verbose sub-step logging.",
    )
    args = parser.parse_args(argv)

    t0 = time.monotonic()

    if not args.skip_build:
        try:
            _build(verbose=args.verbose)
        except Exception as exc:
            print(f"FAIL during build step: {exc}")
            return 1

    validation_report = _validate(
        fast=args.fast,
        run_verify_prototype=not args.skip_verify_prototype,
    )

    render_results = _render(skip_render=args.skip_render)

    _write_report(validation_report, render_results,
                  fast=args.fast, skip_render=args.skip_render)

    elapsed = time.monotonic() - t0
    _heading("Summary")
    if validation_report.passed:
        print(f"  PASS  ({elapsed:.1f}s total).  See "
              f"hexapod_walker/prototype/artifacts/cad_report.md.")
        return 0
    else:
        print(f"  FAIL  ({elapsed:.1f}s total).  See "
              f"hexapod_walker/prototype/artifacts/cad_report.md "
              f"for details.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
