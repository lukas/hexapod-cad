"""Aggregate beam-bending + CalculiX results into ``artifacts/strength_report.md``.

Also writes one nodal-von-Mises PNG per part (top 3 highest-stress only,
to keep ``artifacts/strength/`` small) using PyVista.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from typing import Iterable

import numpy as np

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.dirname(THIS_DIR)
ARTIFACT_DIR = os.path.join(PROTO_DIR, "artifacts", "strength")
REPORT_PATH = os.path.join(PROTO_DIR, "artifacts", "strength_report.md")
REPORT_PATH_BY_MATERIAL = os.path.join(
    PROTO_DIR, "artifacts", "strength", "strength_report_{material}.md",
)
os.makedirs(ARTIFACT_DIR, exist_ok=True)

if PROTO_DIR not in sys.path:
    sys.path.insert(0, PROTO_DIR)

from strength.beam_check import BeamResult, BeamSkipped, BeamReport  # noqa: E402
from strength.run_calculix import FEAResult, parse_frd  # noqa: E402
from strength.materials import Material  # noqa: E402
from strength.load_cases import MassReport  # noqa: E402


# ---------------------------------------------------------------------------
# Report row & rendering
# ---------------------------------------------------------------------------


@dataclass
class PartRow:
    part: str
    case_name: str
    case_description: str
    beam: BeamReport | None
    fea: FEAResult | None
    fea_error: str | None
    material_short_name: str

    @property
    def peak_stress_Pa(self) -> float | None:
        """Best available peak stress (FEA preferred over beam)."""
        if isinstance(self.fea, FEAResult):
            return self.fea.peak_von_mises_Pa
        if isinstance(self.beam, BeamResult):
            return self.beam.peak_bending_stress_Pa
        return None

    @property
    def peak_displacement_m(self) -> float | None:
        if isinstance(self.fea, FEAResult):
            return self.fea.peak_displacement_m
        if isinstance(self.beam, BeamResult):
            return self.beam.tip_deflection_m
        return None

    @property
    def effective_yield_Pa(self) -> float | None:
        if isinstance(self.beam, BeamResult):
            return self.beam.effective_yield_Pa
        return None

    def safety_factor(self, material: Material,
                      infill_factor: float) -> float | None:
        """SF = effective_yield / peak_stress (None if peak_stress missing)."""
        peak = self.peak_stress_Pa
        if peak is None or peak <= 0.0:
            return None
        sigma_y_eff = (
            material.sigma_y_iso * infill_factor / material.brittleness_factor
        )
        return sigma_y_eff / peak

    def interpretation(self, sf: float | None) -> str:
        if sf is None:
            return "no quantitative result -- solver unavailable"
        if sf < 1.0:
            return "**FAILS at this load case** (predicted yield)"
        if sf < 2.0:
            return "marginal -- consider thicker walls / higher infill"
        if sf < 4.0:
            return "safe with margin"
        return "comfortable margin"


# ---------------------------------------------------------------------------
# Markdown renderer
# ---------------------------------------------------------------------------


# Known design-time stress-concentration concerns -- pulled from the
# in-source design comments in ``hexapod_prototype.py`` (e.g. the
# "WELL_TOP_PAD" docstring around line 1610) and the user-flagged
# audit history.  The beam-bending check cannot quantify these
# (they're local features, not cantilever bending), so the report
# names them explicitly so the user can prioritize what to look at
# in the FEA pass on a Linux/Intel host.
_STRUCTURAL_CONCERNS: dict[str, str] = {
    "tibia_link": (
        "spar-to-knee-pad transition (x ~ 20-25 mm from the knee axis): "
        "the 6 x 18 mm spar cross-section drops sharply from the 40 mm "
        "knee pad, creating a re-entrant corner where bending fibre "
        "stress concentrates.  This IS the bending peak the beam check "
        "found (4-7 MPa under impact)."
    ),
    "femur_link": (
        "knee-end servo-body insertion slot (FEMUR_SPAR_H = 34 mm spar "
        "with a 22 mm-tall slot cut through it for the knee servo body): "
        "the 6 mm-tall flange-pair above + below the slot is the thinnest "
        "structural neck on the link.  Comment block at "
        "hexapod_prototype.py:~1476 specifically calls this out; the "
        "post-2026 bump from FEMUR_SPAR_H = 30 -> 34 widened the flanges "
        "to 6 mm each, but they remain the most-likely-to-yield region."
    ),
    "coxa_link": (
        "bridge slab tying the horn yoke to the hip-pitch servo well "
        "(link x in [0, 25] mm).  Comment block at "
        "hexapod_prototype.py:~1610 documents repeated user complaints "
        "about this exact failure mode: the bridge-to-well bond is the "
        "narrow 'fuse' between the yaw output and the hip-pitch motor "
        "housing.  WELL_TOP_PAD_Y_EXT = 13.25 mm was added in May 2026 to "
        "improve this; FEA can quantify whether it's now adequate."
    ),
    "chassis_bottom": (
        "edges of the 4 standoff holes on the 35-mm-radius square "
        "pattern.  The 4 mm-thick plate plus 6 integrated yaw cradle "
        "pockets is a multi-load-path structure that the FEA pass should "
        "tell us whether it bends visibly under the 18 N dead load."
    ),
    "chassis_top": (
        "the 4 standoff holes (same as bottom) plus the printed bosses "
        "for the switch_holster heat-set inserts.  Top plate is 140 mm "
        "flat-to-flat (smaller than bottom) so the cantilever overhang "
        "to the IMU pad is shorter; we expect very low stress."
    ),
    # foot_pad concern RETIRED (Aug 2026): the hinged pad is replaced by
    # the pressed-on TPU foot_boot, which is not FEA'd (elastomer).
}


def _structural_concern_for(part: str) -> str:
    return _STRUCTURAL_CONCERNS.get(
        part, "no recorded concern (no comment-block flag in source)."
    )


def _fmt_mpa(p: float | None) -> str:
    if p is None:
        return "n/a"
    return f"{p / 1.0e6:6.2f} MPa"


def _fmt_mm(d: float | None) -> str:
    if d is None:
        return "n/a"
    return f"{d * 1.0e3:6.3f} mm"


def _fmt_sf(sf: float | None) -> str:
    if sf is None:
        return "n/a"
    if not np.isfinite(sf):
        return "inf"
    return f"{sf:5.2f}"


def render_markdown(rows: list[PartRow],
                    mass: MassReport,
                    material: Material,
                    *,
                    infill_factor: float,
                    fea_available: bool,
                    mesher_available: bool,
                    notes: list[str] | None = None,
                    ) -> str:
    notes = list(notes or [])
    lines: list[str] = []
    lines.append("# Hexapod prototype: opt-in strength / failure-point report")
    lines.append("")
    lines.append(
        "_This report is generated by `make check-strength` and is "
        "**deliberately not** part of the default `make check-cad` "
        "flow.  Re-run it explicitly whenever you change a printed-part "
        "STL or the loaded mass budget._"
    )
    lines.append("")
    lines.append("## Inputs")
    lines.append("")
    lines.append(f"- **Filament**: {material.name} (`{material.short_name}`)")
    lines.append(
        f"- **As-printed properties** (datasheet, before infill / "
        f"brittleness derate):"
    )
    lines.append(f"  - E_iso = {material.E_iso / 1e9:.2f} GPa "
                 f"(from E_xy = {material.E_xy / 1e9:.2f} GPa, "
                 f"E_z = {material.E_z / 1e9:.2f} GPa)")
    lines.append(f"  - sigma_y_iso = {material.sigma_y_iso / 1e6:.1f} MPa "
                 f"(from {material.sigma_y_xy / 1e6:.1f} / "
                 f"{material.sigma_y_z / 1e6:.1f} MPa)")
    lines.append(f"  - density = {material.density:.0f} kg/m^3")
    lines.append(f"  - brittleness derate = {material.brittleness_factor:.1f}x")
    lines.append(f"  - source: {material.source}")
    lines.append(f"- **Print profile**: 25 % gyroid infill, 4 walls -> "
                 f"effective E and yield * {infill_factor:.2f}")
    eff_E = material.E_iso * infill_factor
    eff_y = material.sigma_y_iso * infill_factor / material.brittleness_factor
    lines.append(f"  - **effective E**     = {eff_E / 1e9:.2f} GPa")
    lines.append(f"  - **effective yield** = {eff_y / 1e6:.2f} MPa")
    lines.append("")
    lines.append("### Mass budget")
    lines.append("")
    lines.append("| Item | Mass (g) | Count | Total (g) |")
    lines.append("|---|---:|---:|---:|")
    for part, m in sorted(mass.printed_mass_per_part.items()):
        n = mass.printed_count_per_part.get(part, 1)
        lines.append(
            f"| `{part}` (printed) | {m*1000:7.1f} | {n} | {m*n*1000:7.1f} |"
        )
    for item, m in mass.non_printed_detail.items():
        lines.append(f"| {item} | {m*1000:7.1f} | 1 | {m*1000:7.1f} |")
    lines.append(f"| **printed total** |  |  | **{mass.printed_total*1000:7.1f}** |")
    lines.append(f"| **non-printed total** |  |  | **{mass.non_printed_total*1000:7.1f}** |")
    lines.append(f"| **assembled mass** |  |  | **{mass.total*1000:7.1f}** |")
    lines.append("")
    lines.append("### Solver status")
    lines.append("")
    lines.append(f"- Gmsh mesher: {'available' if mesher_available else '**unavailable**'}")
    lines.append(f"- CalculiX FEA: {'available' if fea_available else '**unavailable** -- beam-bending check used as primary number'}")
    if not fea_available:
        lines.append(
            "  - On macOS, install with `brew install "
            "costerwi/homebrew-calculix/calculix-ccx`.  As of May 2026 "
            "this formula does not build on Apple Silicon (linker "
            "fails finding `_dseupd_` from the bundled ARPACK); on "
            "Intel macOS and Linux it works."
        )
    lines.append("")

    # ----------------- summary table -----------------
    lines.append("## Summary table")
    lines.append("")
    lines.append("| Part | Case | Peak σ (FEA) | Peak σ (beam) | Tip δ | SF (FEA) | SF (beam) | Interpretation |")
    lines.append("|---|---|---:|---:|---:|---:|---:|---|")
    summary_for_top3: list[tuple[float, PartRow]] = []
    for r in rows:
        sigma_fea = r.fea.peak_von_mises_Pa if isinstance(r.fea, FEAResult) else None
        sigma_beam = (
            r.beam.peak_bending_stress_Pa if isinstance(r.beam, BeamResult) else None
        )
        delta = r.peak_displacement_m
        sf = r.safety_factor(material, infill_factor)
        sf_fea = None
        sf_beam = None
        if isinstance(r.fea, FEAResult) and r.fea.peak_von_mises_Pa > 0:
            sf_fea = eff_y / r.fea.peak_von_mises_Pa
        if isinstance(r.beam, BeamResult) and r.beam.peak_bending_stress_Pa > 0:
            sf_beam = eff_y / r.beam.peak_bending_stress_Pa
        interp = r.interpretation(sf)
        lines.append(
            f"| `{r.part}` | {r.case_name} "
            f"| {_fmt_mpa(sigma_fea)} | {_fmt_mpa(sigma_beam)} "
            f"| {_fmt_mm(delta)} | {_fmt_sf(sf_fea)} | {_fmt_sf(sf_beam)} "
            f"| {interp} |"
        )
        peak = r.peak_stress_Pa or 0.0
        summary_for_top3.append((peak, r))
    lines.append("")

    # ----------------- weakest-parts callout -----------------
    summary_for_top3.sort(key=lambda kv: kv[0], reverse=True)
    weakest = summary_for_top3[:3]
    lines.append("## Weakest 3 parts (highest peak stress)")
    lines.append("")
    if not weakest or weakest[0][0] == 0.0:
        lines.append(
            "_No quantitative stress data available -- the FEA solver is "
            "unavailable and the beam-bending check applies only to the "
            "slender links._  See per-part sections below for the qualitative "
            "answer."
        )
    else:
        for rank, (peak, r) in enumerate(weakest, start=1):
            if peak <= 0.0:
                continue
            sf = r.safety_factor(material, infill_factor)
            why = _structural_concern_for(r.part)
            lines.append(
                f"{rank}. **`{r.part}`** "
                f"(case: `{r.case_name}`): peak stress = "
                f"{peak / 1e6:.2f} MPa, "
                f"SF = {_fmt_sf(sf)}.  "
                f"{r.interpretation(sf)}.  "
                f"_Likely critical region:_ {why}"
            )
        lines.append("")

    # ----------------- FEA-required modes ---------------------
    if not fea_available:
        lines.append("## Failure modes the beam check CANNOT see")
        lines.append("")
        lines.append(
            "The beam-bending pre-check above models each link as a "
            "constant-cross-section cantilever and reports the peak "
            "fibre stress at the worst-section sample.  This catches "
            "primary bending failure on the slender links but misses "
            "*local stress concentrations* -- which is exactly where the "
            "user-flagged 'thin web' / 'fragile fuse' failure modes live. "
            "Re-run on a Linux box or Intel macOS host with `ccx` "
            "installed to get a real von Mises field over the geometry; "
            "the following spots are the priorities the FEA pass should "
            "reveal:"
        )
        lines.append("")
        for part, concern in _STRUCTURAL_CONCERNS.items():
            lines.append(f"- **`{part}`**: {concern}")
        lines.append("")

    # ----------------- per-part detail -----------------
    lines.append("## Per-part detail")
    lines.append("")
    for r in rows:
        lines.append(f"### `{r.part}`")
        lines.append("")
        lines.append(f"- **Load case** (`{r.case_name}`): {r.case_description}")
        if isinstance(r.beam, BeamResult):
            b = r.beam
            lines.append(
                f"- **Beam-bending cross-check** (Euler-Bernoulli, "
                f"cantilever, 14-station worst-section scan):"
            )
            lines.append(
                f"  - L = {b.length_m*1000:.1f} mm, "
                f"F_tip = {b.tip_force_N:.1f} N, "
                f"M_at_worst = {b.bending_moment_N_m:.3f} N-m"
            )
            lines.append(
                f"  - A = {b.area_m2*1e6:.1f} mm^2, "
                f"I_strong = {b.I_max_m4*1e12:.0f} mm^4, "
                f"I_weak = {b.I_min_m4*1e12:.0f} mm^4, "
                f"c_max = {b.c_max_m*1000:.2f} mm"
            )
            lines.append(
                f"  - peak bending stress = {b.peak_bending_stress_Pa/1e6:.2f} MPa "
                f"({b.bending_axis})"
            )
            lines.append(
                f"  - tip deflection = {b.tip_deflection_m*1000:.3f} mm"
            )
            lines.append(
                f"  - SF (beam) = effective_yield ({b.effective_yield_Pa/1e6:.2f} MPa) "
                f"/ peak = {_fmt_sf(b.safety_factor)}"
            )
        elif isinstance(r.beam, BeamSkipped):
            lines.append(f"- **Beam-bending cross-check**: skipped ({r.beam.reason})")
        if isinstance(r.fea, FEAResult):
            fr = r.fea
            sf_fea = (eff_y / fr.peak_von_mises_Pa) if fr.peak_von_mises_Pa > 0 else None
            lines.append(
                f"- **CalculiX FEA** (linear-static, isotropic-equiv, C3D4 tets):"
            )
            lines.append(f"  - mesh: {fr.num_nodes} nodes")
            lines.append(
                f"  - peak von Mises = {fr.peak_von_mises_Pa/1e6:.2f} MPa "
                f"at node {fr.peak_node_id} "
                f"({fr.peak_node_xyz_m[0]*1000:+.1f}, "
                f"{fr.peak_node_xyz_m[1]*1000:+.1f}, "
                f"{fr.peak_node_xyz_m[2]*1000:+.1f}) mm"
            )
            lines.append(
                f"  - peak displacement = {fr.peak_displacement_m*1000:.3f} mm"
            )
            lines.append(f"  - SF (FEA) = {_fmt_sf(sf_fea)}")
            png = os.path.join("strength", f"{r.part}_{material.short_name}_{r.case_name}.png")
            if os.path.exists(os.path.join(os.path.dirname(REPORT_PATH), png)):
                lines.append(f"- **Stress field**: ![{r.part} stress]({png})")
        elif r.fea_error is not None:
            lines.append(f"- **CalculiX FEA**: failed -- `{r.fea_error}`")
        else:
            lines.append("- **CalculiX FEA**: not run (solver unavailable or skipped via `--solver beam`)")
        lines.append("")

    if notes:
        lines.append("## Notes")
        lines.append("")
        for n in notes:
            lines.append(f"- {n}")
        lines.append("")

    return "\n".join(lines)


def write_report(rows: list[PartRow],
                 mass: MassReport,
                 material: Material,
                 *,
                 infill_factor: float,
                 fea_available: bool,
                 mesher_available: bool,
                 notes: list[str] | None = None,
                 path: str | None = None) -> list[str]:
    """Write both the canonical strength_report.md and a per-material
    sibling under artifacts/strength/strength_report_<material>.md.

    Returns the list of written paths.
    """
    md = render_markdown(
        rows, mass, material,
        infill_factor=infill_factor,
        fea_available=fea_available,
        mesher_available=mesher_available,
        notes=notes,
    )
    paths_to_write = []
    if path is not None:
        paths_to_write.append(path)
    else:
        paths_to_write.append(REPORT_PATH)
        paths_to_write.append(
            REPORT_PATH_BY_MATERIAL.format(material=material.short_name)
        )
    for p in paths_to_write:
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w") as f:
            f.write(md)
    return paths_to_write


# ---------------------------------------------------------------------------
# PNG renderer (PyVista) for the top-N stress fields
# ---------------------------------------------------------------------------


def render_stress_png(stl_path: str,
                      frd_path: str,
                      out_png_path: str) -> str:
    """Color the part's surface by nodal von Mises and save a PNG."""
    import pyvista as pv
    import trimesh

    # Load the original STL surface (this is what we'll color); load
    # the nodal stress from the FRD and resample onto the STL vertices
    # by nearest neighbour.
    tm = trimesh.load(stl_path, force="mesh")
    verts_mm = np.asarray(tm.vertices, dtype=float)
    faces = np.asarray(tm.faces, dtype=int)

    n_ids, n_xyz, _u, vm = parse_frd(frd_path)
    nodes_mm = np.asarray(n_xyz, dtype=float) * 1.0e3   # FRD nodes are m -> mm
    vm_mpa = vm / 1.0e6

    # Nearest-neighbour from STL vert -> FEA node.
    from scipy.spatial import cKDTree
    tree = cKDTree(nodes_mm)
    _, idx = tree.query(verts_mm)
    vert_vm = vm_mpa[idx]

    # PyVista PolyData expects faces as flat (n, v0, v1, ..., vn) per row.
    pv_faces = np.hstack([
        np.full((len(faces), 1), 3, dtype=int),
        faces,
    ]).ravel()
    surf = pv.PolyData(verts_mm, pv_faces)
    surf["von Mises (MPa)"] = vert_vm

    plotter = pv.Plotter(off_screen=True, window_size=(960, 720))
    plotter.add_mesh(
        surf, scalars="von Mises (MPa)",
        cmap="inferno",
        clim=(0.0, float(vert_vm.max())),
        smooth_shading=True,
    )
    plotter.add_scalar_bar(title="von Mises [MPa]", n_labels=5)
    plotter.view_isometric()
    plotter.add_axes()
    os.makedirs(os.path.dirname(out_png_path), exist_ok=True)
    plotter.screenshot(out_png_path)
    plotter.close()
    return out_png_path


__all__ = [
    "PartRow", "render_markdown", "write_report",
    "render_stress_png",
]
