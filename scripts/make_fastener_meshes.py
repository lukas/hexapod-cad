"""Regenerate the fastener cache STLs in ``fasteners/``.

For each unique part number in ``fastener_registry.build_all_fastener_instances()``
this script picks a source mesh by priority:

    1. ``fasteners/<part_number>.step`` -- a real McMaster-Carr STEP
       download.  Loaded only when ``trimesh`` can read it (requires
       ``python-occ`` for accurate boundary representation).
    2. ``fasteners/<part_number>.stl``  -- a user-supplied STL.
    3. ``fasteners/scad/<spec_id>.scad`` -- NopSCADlib-backed ISO-spec
       geometry rendered through the OpenSCAD CLI.  This is the
       default in 2026 onward; see ``fasteners/README.md``'s
       "Default: NopSCADlib via OpenSCAD" section.
    4. ``fasteners/_parametric.build_for_spec(<spec>)`` -- the
       handwritten parametric fallback (visualization only).  Used
       only when ``openscad`` is missing OR the NopSCADlib render
       fails.

The chosen source is converted to a single Trimesh, re-oriented to
match the convention in ``fasteners/_parametric.py`` (origin = head
mating face / nut outboard face; +Z = body axis pointing INTO the
material), and written out as ``fasteners/<part_number>.cache.stl``.
The build inspector and the verifier read ONLY the cache STL at
runtime, so a parametric fallback today silently becomes a real
STEP later when the user drops one in and re-runs
``make regen-fasteners``.

For each part number we also write
``fasteners/<part_number>.cache.source.txt`` -- a one-line
breadcrumb of the form ``openscad NopSCADlib <git-sha>`` /
``parametric`` / ``step:<filename>`` / ``user STL:<filename>`` so a
human or another tool can tell at a glance which source produced the
cached mesh.

Run with ``python make_fastener_meshes.py`` from the prototype
directory (or via ``make regen-fasteners``).
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import traceback
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent.parent  # prototype_sts3215/
FASTENERS = HERE / "fasteners"
SCAD_DIR = FASTENERS / "scad"


from fastener_registry import build_all_fastener_instances
from fasteners import _parametric


# ---------------------------------------------------------------------------
# Spec-string -> .scad filename mapping
# ---------------------------------------------------------------------------
#
# Each registry spec string ("M3x8 SHCS", "M3 nyloc nut", ...) maps
# to a single .scad file under fasteners/scad/.  Keep this dict in
# sync with the entries in fastener_registry.SPEC_* and with the
# files actually present under fasteners/scad/.

SPEC_TO_SCAD: dict[str, str] = {
    # June 2026 disc-horn switch: link-to-disc-horn bolts are "M3x6
    # SHCS".  Intentionally NOT mapped here -- there is no m3x6_shcs.scad
    # and the parametric fallback (``_parametric.build_for_spec`` ->
    # ``make_m3_shcs(6.0)``) renders it fine, same as the heat-set
    # inserts fall through to parametric.
    "M3x8 SHCS":                      "m3x8_shcs",
    # May 2026 heat-set switch: cradle M3 x 8 SHCS now thread into
    # a brass insert instead of a plastic pilot.  Same physical
    # bolt (same P/N 91290A113), distinct spec string -- both
    # share the ``m3x8_shcs`` scad rendering so the inspector
    # mesh is identical.
    "M3x8 SHCS into heat-set insert": "m3x8_shcs",
    # M3 x 10 SHCS -- battery_holder foot bolts (4) into heat-set
    # inserts.  Same M3 SHCS family as M3 x 8 / M3 x 32, just at the
    # 10 mm length stock.
    "M3x10 SHCS":                     "m3x10_shcs",
    "M3x32 SHCS":                     "m3x32_shcs",  # rendered at 30 mm (BOM stock)
    "M3x16 pan-head":                 "m3x16_pan",
    "M2.5x8 spline screw":            "m2p5x8_shcs",
    # May 2026 (electronics-tray expansion): the Raspberry Pi 4 / Pi 5
    # board-mount bolts thread into M2.5 brass heat-set inserts in the
    # electronics_tray.  Same physical bolt shape as the servo spline
    # screw (M2.5 x 8 SHCS, P/N 91290A102 vs the spline's 91290A104),
    # so re-use the existing m2p5x8_shcs scad render for the cache
    # mesh.  Distinct spec string lets the BOM call out the Pi-mount
    # role separately.
    "M2.5x8 SHCS into heat-set insert": "m2p5x8_shcs",
    "M3 nyloc nut":                   "m3_nyloc_nut",
    # The heat-set insert intentionally has NO scad mapping:
    # NopSCADlib doesn't ship insert vitamins, and the parametric
    # fallback in ``fasteners/_parametric.make_m3_heatset_insert``
    # is sufficient for the inspector (Phi 5.7 mm brass cylinder
    # with 16 axial flutes).  Falling through to parametric is
    # the *expected* path for the heat-set P/N -- see the
    # WARN-when-missing print in regenerate().
}


# ---------------------------------------------------------------------------
# OpenSCAD CLI discovery
# ---------------------------------------------------------------------------


_OPENSCAD_CANDIDATES = [
    "openscad",
    "/opt/homebrew/bin/openscad",
    "/usr/local/bin/openscad",
    "/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD",
    "/Applications/OpenSCAD-2021.01.app/Contents/MacOS/OpenSCAD",
]


def _find_openscad() -> str | None:
    """Return the path to the OpenSCAD CLI, or None if not installed."""
    p = shutil.which("openscad")
    if p:
        return p
    for cand in _OPENSCAD_CANDIDATES:
        if os.path.isfile(cand) and os.access(cand, os.X_OK):
            return cand
        # On macOS shutil.which already covers $PATH; still try the
        # absolute app-bundle path.
        if cand.startswith("/Applications/") and os.path.isfile(cand):
            return cand
    return None


def _nopscadlib_sha() -> str:
    """Best-effort lookup of the NopSCADlib git SHA, for the provenance
    breadcrumb.  Returns the short sha, or ``"unknown"`` if NopSCADlib
    is somewhere we can't find or git is unavailable.
    """
    candidates = [
        Path.home() / "Documents" / "OpenSCAD" / "libraries" / "NopSCADlib",
        Path.home() / ".local" / "share" / "OpenSCAD" / "libraries" / "NopSCADlib",
    ]
    for path in candidates:
        if (path / ".git").exists():
            try:
                sha = subprocess.check_output(
                    ["git", "-C", str(path), "rev-parse", "--short", "HEAD"],
                    text=True,
                    stderr=subprocess.DEVNULL,
                    timeout=5,
                ).strip()
                if sha:
                    return sha
            except Exception:  # noqa: BLE001
                pass
    return "unknown"


# ---------------------------------------------------------------------------
# Loader helpers (each returns a Trimesh or None)
# ---------------------------------------------------------------------------


def _try_load_step(path: Path) -> trimesh.Trimesh | None:
    try:
        loaded = trimesh.load(path, force="mesh")
    except Exception as exc:  # noqa: BLE001
        print(f"  STEP load failed for {path.name}: {exc}", file=sys.stderr)
        return None
    if loaded is None or not hasattr(loaded, "vertices"):
        return None
    if isinstance(loaded, trimesh.Scene):
        if not loaded.geometry:
            return None
        loaded = trimesh.util.concatenate(list(loaded.geometry.values()))
    return loaded


def _try_load_stl(path: Path) -> trimesh.Trimesh | None:
    try:
        return trimesh.load(path, force="mesh")
    except Exception as exc:  # noqa: BLE001
        print(f"  STL load failed for {path.name}: {exc}", file=sys.stderr)
        return None


def _render_via_openscad(
    scad_path: Path,
    out_stl_path: Path,
    *,
    openscad_bin: str,
    timeout_s: float = 60.0,
) -> bool:
    """Render ``scad_path`` to ``out_stl_path`` via the OpenSCAD CLI.

    Returns True on success; on any failure (binary missing, timeout,
    non-zero exit, OpenSCAD prints an error) returns False so the
    priority chain falls through to the parametric fallback.
    """
    try:
        result = subprocess.run(
            [openscad_bin, "-o", str(out_stl_path), str(scad_path)],
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
    except subprocess.TimeoutExpired:
        print(
            f"  openscad render timed out (> {timeout_s:.0f} s): {scad_path.name}",
            file=sys.stderr,
        )
        return False
    except Exception as exc:  # noqa: BLE001
        print(f"  openscad invocation failed for {scad_path.name}: {exc}", file=sys.stderr)
        return False
    if result.returncode != 0:
        print(
            f"  openscad exited {result.returncode} for {scad_path.name}:",
            file=sys.stderr,
        )
        if result.stderr:
            tail = result.stderr.strip().splitlines()[-6:]
            for line in tail:
                print(f"    {line}", file=sys.stderr)
        return False
    if not out_stl_path.exists() or out_stl_path.stat().st_size == 0:
        print(
            f"  openscad wrote no output for {scad_path.name}",
            file=sys.stderr,
        )
        return False
    return True


def _orient_for_registry(spec: str, mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Re-orient an OpenSCAD-rendered mesh into the fastener_registry
    convention used by ``_parametric.py``.

    NopSCADlib's ``screw(type, length)`` puts the head at +Z and the
    shank at -Z; ``nut(type, nyloc=true)`` puts the steel body at
    z=[0, t] and the nyloc collar above it.  The registry expects:

        * For SHCS / pan-head screws:
            head at z in [-head_h, 0], shank at z in [0, +length];
            mesh +Z = the direction the shank goes INTO the material
            (= the registry's ``axis_world``).
        * For the nyloc nut:
            visible outer face at z = 0, body at z in [-nut_h, 0]
            (extending INTO the printed hex pocket, i.e. INTO the
            material).  Mesh +Z = direction the bolt's shank would
            pass through (the inspector still maps mesh +Z onto
            ``axis_world``).

    The simplest transform that satisfies both: flip the NopSCADlib
    output across the XY plane (negate Z).  This matches the
    parametric convention exactly.
    """
    flip = np.eye(4)
    flip[2, 2] = -1.0
    mesh = mesh.copy()
    mesh.apply_transform(flip)
    # Mirroring inverts face winding -- fix the normals.
    mesh.invert()
    return mesh


# ---------------------------------------------------------------------------
# Top-level pipeline
# ---------------------------------------------------------------------------


def regenerate() -> dict:
    """Return a report dict ``{part_number: source}`` recording which
    source produced each cache."""
    FASTENERS.mkdir(parents=True, exist_ok=True)
    seen: dict[str, str] = {}  # part_number -> spec
    for fi in build_all_fastener_instances():
        if fi.part_number in seen:
            continue
        seen[fi.part_number] = fi.spec

    openscad_bin = _find_openscad()
    nopsha = _nopscadlib_sha() if openscad_bin else "unknown"
    if openscad_bin:
        print(f"  OpenSCAD: {openscad_bin}")
        print(f"  NopSCADlib SHA: {nopsha}")
    else:
        print(
            "  OpenSCAD CLI not found in PATH or /Applications -- "
            "the NopSCADlib stage will be SKIPPED and every spec will "
            "fall through to the parametric fallback.",
            file=sys.stderr,
        )

    report: dict[str, str] = {}
    for pn, spec in sorted(seen.items()):
        step_path = FASTENERS / f"{pn}.step"
        stl_path  = FASTENERS / f"{pn}.stl"
        cache_path = FASTENERS / f"{pn}.cache.stl"
        source_breadcrumb_path = FASTENERS / f"{pn}.cache.source.txt"
        mesh = None
        source = "parametric"
        source_breadcrumb = "parametric"

        # 1. real McMaster-Carr STEP file (highest fidelity)
        if step_path.exists():
            mesh = _try_load_step(step_path)
            if mesh is not None:
                source = "STEP"
                source_breadcrumb = f"step:{step_path.name}"

        # 2. user-supplied STL
        if mesh is None and stl_path.exists():
            mesh = _try_load_stl(stl_path)
            if mesh is not None:
                source = "user STL"
                source_breadcrumb = f"user STL:{stl_path.name}"

        # 3. NopSCADlib via OpenSCAD
        if mesh is None and openscad_bin is not None:
            scad_name = SPEC_TO_SCAD.get(spec)
            if scad_name is None:
                print(
                    f"  WARN: no scad mapping for spec {spec!r} (pn={pn}); "
                    "falling through to parametric",
                    file=sys.stderr,
                )
            else:
                scad_path = SCAD_DIR / f"{scad_name}.scad"
                if not scad_path.exists():
                    print(
                        f"  WARN: {scad_path.relative_to(HERE)} missing; "
                        "falling through to parametric",
                        file=sys.stderr,
                    )
                else:
                    tmp_stl = FASTENERS / f"{pn}.openscad.raw.stl"
                    ok = _render_via_openscad(
                        scad_path, tmp_stl, openscad_bin=openscad_bin
                    )
                    if ok:
                        raw = _try_load_stl(tmp_stl)
                        try:
                            tmp_stl.unlink()
                        except OSError:
                            pass
                        if raw is not None and len(raw.vertices) > 0:
                            mesh = _orient_for_registry(spec, raw)
                            source = "openscad"
                            source_breadcrumb = f"openscad NopSCADlib {nopsha}"

        # 4. parametric fallback (last resort -- visualization only)
        if mesh is None:
            try:
                mesh = _parametric.build_for_spec(spec)
            except Exception:  # noqa: BLE001
                traceback.print_exc()
                continue
        if mesh is None:
            print(f"  ERROR: no source for {pn} ({spec})", file=sys.stderr)
            continue
        mesh.export(cache_path)
        source_breadcrumb_path.write_text(source_breadcrumb + "\n")
        report[pn] = source
        size_kb = cache_path.stat().st_size / 1024.0
        print(
            f"  cache {pn:12s} <- {source:10s}  ({spec:24s}) "
            f"[{size_kb:6.1f} KB]"
        )
    return report


if __name__ == "__main__":
    print(f"Regenerating fastener cache in {FASTENERS}/ ...")
    rep = regenerate()
    print()
    print("Summary:")
    for pn, source in sorted(rep.items()):
        print(f"  {pn:12s}  {source}")
