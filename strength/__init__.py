"""Opt-in strength / failure-point pipeline for the hexapod walker prototype.

This package is *deliberately* not wired into ``make check-cad`` or any of
the geometry verifiers.  It exists so the user can run a one-off
finite-element + closed-form beam-bending pass on the current 3D-printed
parts and see which parts are marginal at the named load cases.

Run via the Makefile::

    make check-strength               # PETG, every part, beam + FEA
    make check-strength MATERIAL=pla  # PLA derated yield, same suite
    make check-strength PARTS=tibia_link,femur_link SOLVER=beam

or directly::

    python -m hexapod_walker.prototype.strength --solver both

Outputs land under ``artifacts/strength/`` and a roll-up Markdown report
goes to ``artifacts/strength_report.md``.

The package imports geometry constants from ``hexapod_prototype`` so the
load cases track the real design; the *other* direction (build/inspect
pipelines importing this package) is forbidden so a missing CalculiX
binary can never break the geometry workflow.
"""

from __future__ import annotations

__all__ = [
    "materials",
    "load_cases",
    "mesh_part",
    "run_calculix",
    "beam_check",
    "report",
]
