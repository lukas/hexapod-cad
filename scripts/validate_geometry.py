"""Validate the hexapod prototype STLs against ``design_spec.yaml``.

Run from the repo root:

    ./run.sh hexapod_walker/prototype/scripts/validate_geometry.py

Or:

    make -C hexapod_walker/prototype validate

What it does
------------

1.  Ensures ``stl_prototype/*.stl`` exist; if missing, runs ``build_all.py``.
2.  Loads ``design_spec.yaml`` and every required STL.
3.  Performs SPEC-vs-STL checks:
       - STL loads & is watertight.
       - Bounding box matches spec.bounds_mm +/- spec.bounds_tol_mm.
       - Each declared hole punches a tunnel of the right diameter
         through the STL along the declared axis (raycast tunnel probe).
       - Keep-out-volume samples are NOT inside the STL (the part does
         not penetrate the keep-out).
     Wire-channel corridors are NOT re-probed here -- they are named
     references whose open-corridor sweep is owned by
     ``_verify_prototype.check_wire_slot`` (run in step 4); this script
     only records the declared channel names.
4.  Optionally runs the full ``_verify_prototype.main()`` validator
    (skipped under ``--fast``: the workspace sweep is the slow group).
5.  Writes a structured report dict (returned by ``run_validation()``)
    and prints a PASS/FAIL summary.

Exit code is 0 on PASS, 1 on any FAIL.

The whole script tries to be friendly when called from an LLM coding
agent: error messages are designed to be COPY-PASTABLE into the
parametric source as a fix prompt.
"""

from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys
import time
from contextlib import redirect_stdout
from dataclasses import dataclass, field

import numpy as np
import trimesh

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.dirname(THIS_DIR)

# Make hexapod_walker/prototype importable so we can re-use
# _verify_prototype's helpers.

STL_DIR = os.path.join(PROTO_DIR, "stl_prototype")
SPEC_PATH = os.path.join(PROTO_DIR, "design_spec.yaml")


# ---------------------------------------------------------------------------
# YAML loader -- soft-fail with a helpful message if PyYAML is missing
# ---------------------------------------------------------------------------

def _load_yaml(path: str) -> dict:
    try:
        import yaml  # noqa: WPS433 (deferred import for graceful fail)
    except ImportError:
        print(
            "ERROR: PyYAML is not installed.\n"
            "  pip install -r hexapod_walker/prototype/scripts/"
            "requirements_cad_check.txt\n"
            "(or just  pip install pyyaml  -- the only extra dep this "
            "validator needs).",
            file=sys.stderr,
        )
        raise
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# Report dataclass
# ---------------------------------------------------------------------------

@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str = ""


@dataclass
class PartReport:
    name: str
    stl_path: str
    bounds_observed_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    bounds_expected_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    watertight: bool = False
    volume_mm3: float = 0.0
    holes_checked: int = 0
    holes_passed: int = 0
    wire_channels_checked: int = 0
    wire_channels_passed: int = 0
    keep_outs_checked: int = 0
    keep_outs_passed: int = 0
    shape_features_checked: int = 0
    shape_features_passed: int = 0
    checks: list[CheckResult] = field(default_factory=list)
    warnings: list[CheckResult] = field(default_factory=list)


@dataclass
class ValidationReport:
    started_unix: float
    finished_unix: float = 0.0
    spec_path: str = SPEC_PATH
    stl_dir: str = STL_DIR
    fast: bool = False
    parts: list[PartReport] = field(default_factory=list)
    group_results: list[CheckResult] = field(default_factory=list)
    todo_fields: list[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        for part in self.parts:
            for chk in part.checks:
                if not chk.passed:
                    return False
        for grp in self.group_results:
            if not grp.passed:
                return False
        return True


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _stl_path_for(part_name: str) -> str:
    return os.path.join(STL_DIR, f"{part_name}.stl")


def _ensure_stls_built(needed: list[str]) -> None:
    """If any required STL is missing, regenerate the bundle via
    ``build_all.py --skip-assembly`` (only the per-part STLs are needed
    for validation, so we skip the heavier sub-steps).
    """
    missing = [n for n in needed if not os.path.isfile(_stl_path_for(n))]
    if not missing:
        return
    print(f"  Missing STLs: {missing}.  Running build_all.py to "
          f"regenerate stl_prototype/ ...")
    cmd = [
        sys.executable,
        os.path.join(PROTO_DIR, "build_all.py"),
        "--skip-assembly",
    ]
    subprocess.check_call(cmd, cwd=PROTO_DIR)


def _axis_vec(axis: str) -> np.ndarray:
    return {
        "x": np.array([1.0, 0.0, 0.0]),
        "y": np.array([0.0, 1.0, 0.0]),
        "z": np.array([0.0, 0.0, 1.0]),
    }[axis.lower()]


def _hole_probe(mesh: trimesh.Trimesh,
                centre: np.ndarray,
                axis: np.ndarray,
                diameter_mm: float) -> tuple[bool, str]:
    """Probe whether ``mesh`` has a hole of the declared diameter,
    centred on ``centre`` and aligned along ``axis``.

    Strategy
    --------
    A hole is a region of VOID embedded in SOLID material.  We require
    both halves to be true:

      * the hole's centreline (3 samples along ``axis`` through
        ``centre``) is OUTSIDE the mesh -- the bore is open, AND
      * at least ONE probe in a ring at radius ~ ``1.5 * diameter``
        around the centre line is INSIDE the mesh -- material exists
        somewhere around the bore.

    The second check is intentionally lax: many holes legitimately
    have material on only ONE side (e.g. a hub's bolt-circle hole that
    sits at the edge of the pedestal).  This is a regression detector
    against "the parametric source forgot to drill the hole", not a
    metrology pass.

    Returns ``(ok, detail)``.
    """
    axis = axis / np.linalg.norm(axis)
    pick = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 \
        else np.array([0.0, 1.0, 0.0])
    ortho1 = np.cross(axis, pick)
    ortho1 = ortho1 / np.linalg.norm(ortho1)
    ortho2 = np.cross(axis, ortho1)
    ortho2 = ortho2 / np.linalg.norm(ortho2)

    centre_samples = np.array([
        centre,
        centre + 1.0 * axis,
        centre - 1.0 * axis,
    ])
    centre_inside = _is_inside_majority(mesh, centre_samples)
    n_centre_blocked = int(np.sum(centre_inside))
    if n_centre_blocked > 0:
        return False, (f"hole centreline NOT open: "
                       f"{n_centre_blocked}/3 axial samples inside mesh "
                       f"-- the bore is blocked")

    ring_radius = max(diameter_mm * 1.5, diameter_mm + 1.5)
    ring_pts = []
    for theta in np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False):
        p = centre + ring_radius * (np.cos(theta) * ortho1
                                     + np.sin(theta) * ortho2)
        ring_pts.append(p)
    ring_pts = np.asarray(ring_pts)
    ring_inside = _is_inside_majority(mesh, ring_pts)
    n_ring_solid = int(np.sum(ring_inside))
    ok = n_ring_solid >= 1
    detail = (f"centreline: 0/3 blocked (open).  "
              f"surrounding ring: {n_ring_solid}/8 inside mesh "
              f"(>=1 = bore has SOME material around it)")
    return ok, detail


def _voxel_sample_interior(mesh: trimesh.Trimesh,
                           pitch: float = 3.0,
                           max_samples: int = 200) -> np.ndarray:
    """Return a set of points sampled INSIDE ``mesh`` (= interior of a
    named keep-out volume).  Uses a regular grid stepping at ``pitch``
    mm over the AABB, kept only if the point is inside the volume.

    Capped to ``max_samples`` points to keep the inside-test cheap on
    larger volumes.
    """
    bounds = mesh.bounds
    xs = np.arange(bounds[0, 0] + pitch / 2.0,
                   bounds[1, 0] - pitch / 2.0 + 1e-6, pitch)
    ys = np.arange(bounds[0, 1] + pitch / 2.0,
                   bounds[1, 1] - pitch / 2.0 + 1e-6, pitch)
    zs = np.arange(bounds[0, 2] + pitch / 2.0,
                   bounds[1, 2] - pitch / 2.0 + 1e-6, pitch)
    if len(xs) == 0 or len(ys) == 0 or len(zs) == 0:
        return mesh.bounds.mean(axis=0)[None, :]
    X, Y, Z = np.meshgrid(xs, ys, zs, indexing="ij")
    pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)
    try:
        inside = mesh.contains(pts)
    except Exception:
        inside = np.ones(len(pts), dtype=bool)
    interior = pts[inside]
    if len(interior) > max_samples:
        idx = np.linspace(0, len(interior) - 1, max_samples,
                          dtype=int)
        interior = interior[idx]
    if len(interior) == 0:
        interior = mesh.bounds.mean(axis=0)[None, :]
    return interior


def _is_inside_majority(mesh: trimesh.Trimesh,
                        pts: np.ndarray) -> np.ndarray:
    """Robust inside-test via the same 6-ray majority vote that
    ``_verify_prototype.points_inside`` uses.  Imported on-demand to
    avoid a hard dependency at module import time.
    """
    try:
        from _verify_prototype import points_inside as _pi
    except Exception:
        return mesh.contains(pts)
    return _pi(mesh, pts)


# ---------------------------------------------------------------------------
# Shape-block sanity checks (warnings, not errors)
# ---------------------------------------------------------------------------

# Whitelist of feature ``type`` tokens recognised by RULE 7 (see the
# header of design_spec.yaml).  Unknown types only produce a warning
# so a user adding a new primitive doesn't break the validator.
_SHAPE_PRIMITIVE_KINDS = frozenset({
    "cylindrical_disc",
    "extruded_rectangle",
    "hex_plate",
    "rectangular_well",
    "cylindrical_boss",
    "rectangular_tang",
    "cylindrical_tang",
    "fork",
    "gusset_rib",
    "slot",
    "recess",
    # A deliberately EMPTY volume the part must keep open (e.g. the open
    # back over a passive rear-boss disc horn).  Documented in RULE 7.
    "open_clearance",
})

_SHAPE_AXIS_NORMALS = frozenset({
    "+x", "-x", "+y", "-y", "+z", "-z",
})


def _check_shape_block(part_name: str,
                       spec_entry: dict,
                       report: PartReport) -> None:
    """Validate the ``shape:`` block (RULE 7) at WARNING severity.

    Each warning is recorded on ``report.warnings`` (a parallel list to
    ``report.checks``) so a part-by-part rollout doesn't break the
    existing pipeline -- a missing or malformed ``shape:`` field does
    NOT flip the part's PASS/FAIL bit.

    Checks performed (per feature):

    * ``name``, ``type``, ``description``, ``centre_mm`` are present.
    * At least one of ``length_mm``, ``diameter_mm``, ``thickness_mm``
      is present (so a feature isn't dimensionless).
    * ``type`` is in the recognised primitive set (whitelist).
    * If ``holes_on_feature`` is present, every referenced hole name
      exists in the part's ``holes:`` block.
    * If ``mating_face`` is present, ``normal`` is one of the 6 axis
      tokens and ``mates_to`` is a non-empty string.
    """
    shape = spec_entry.get("shape")
    if shape is None:
        report.warnings.append(CheckResult(
            name="shape:missing",
            passed=True,    # warning, not error
            detail=(f"part '{part_name}' has no `shape:` block -- "
                    f"add per RULE 7 in design_spec.yaml so a "
                    f"rebuild-from-spec engineer can reconstruct "
                    f"the part without reading the CAD source"),
        ))
        return

    features = shape.get("features") or []
    if not isinstance(features, list) or not features:
        report.warnings.append(CheckResult(
            name="shape:no_features",
            passed=True,
            detail=(f"part '{part_name}' has a `shape:` block with no "
                    f"`features:` list -- shape is the anatomical "
                    f"reference, every part needs at least 1 feature"),
        ))
        return

    declared_hole_names = {
        h.get("name") for h in (spec_entry.get("holes") or []) if h.get("name")
    }

    required_fields = ("name", "type", "description", "centre_mm")
    dimension_fields = ("length_mm", "diameter_mm", "thickness_mm")

    for idx, feat in enumerate(features):
        report.shape_features_checked += 1
        if not isinstance(feat, dict):
            report.warnings.append(CheckResult(
                name=f"shape:feature[{idx}]",
                passed=True,
                detail=f"feature index {idx} is not a mapping; skipped",
            ))
            continue

        fname = feat.get("name", f"<feature {idx}>")
        fkey = f"shape:{part_name}.{fname}"

        missing = [k for k in required_fields if k not in feat]
        if missing:
            report.warnings.append(CheckResult(
                name=fkey,
                passed=True,
                detail=(f"missing required field(s): {missing} -- "
                        f"every shape feature needs name + type + "
                        f"description + centre_mm"),
            ))
            continue

        if not any(k in feat for k in dimension_fields):
            report.warnings.append(CheckResult(
                name=fkey,
                passed=True,
                detail=(f"feature has no length_mm / diameter_mm / "
                        f"thickness_mm -- need at least one to "
                        f"reconstruct the primitive"),
            ))
            continue

        ftype = feat.get("type")
        if ftype not in _SHAPE_PRIMITIVE_KINDS:
            report.warnings.append(CheckResult(
                name=fkey,
                passed=True,
                detail=(f"type='{ftype}' is not in the recognised "
                        f"primitive set {sorted(_SHAPE_PRIMITIVE_KINDS)} "
                        f"-- add the new kind to RULE 7 in "
                        f"design_spec.yaml + this validator if needed"),
            ))
            continue

        holes_on_feat = feat.get("holes_on_feature")
        if holes_on_feat:
            if not isinstance(holes_on_feat, list):
                report.warnings.append(CheckResult(
                    name=fkey,
                    passed=True,
                    detail=(f"holes_on_feature must be a list of hole "
                            f"names; got {type(holes_on_feat).__name__}"),
                ))
                continue
            missing_holes = [h for h in holes_on_feat
                             if h not in declared_hole_names]
            if missing_holes:
                report.warnings.append(CheckResult(
                    name=fkey,
                    passed=True,
                    detail=(f"holes_on_feature references undeclared "
                            f"hole name(s) {missing_holes}; each name "
                            f"must appear in the part's `holes:` block"),
                ))
                continue

        mating = feat.get("mating_face")
        if mating is not None:
            if not isinstance(mating, dict):
                report.warnings.append(CheckResult(
                    name=fkey,
                    passed=True,
                    detail=(f"mating_face must be a mapping; got "
                            f"{type(mating).__name__}"),
                ))
                continue
            normal = (mating.get("normal") or "").strip().lower()
            if normal not in _SHAPE_AXIS_NORMALS:
                report.warnings.append(CheckResult(
                    name=fkey,
                    passed=True,
                    detail=(f"mating_face.normal='{normal}' must be one "
                            f"of +x / -x / +y / -y / +z / -z"),
                ))
                continue
            mates_to = mating.get("mates_to")
            if not (isinstance(mates_to, str) and mates_to.strip()):
                report.warnings.append(CheckResult(
                    name=fkey,
                    passed=True,
                    detail=("mating_face.mates_to must be a non-empty "
                            "string naming the mating part + feature"),
                ))
                continue

        report.shape_features_passed += 1


# ---------------------------------------------------------------------------
# Per-part checks
# ---------------------------------------------------------------------------

def _check_part(part_name: str, spec_entry: dict,
                bounds_tol_mm: float) -> PartReport:
    path = _stl_path_for(part_name)
    report = PartReport(name=part_name, stl_path=path)
    if not os.path.isfile(path):
        report.checks.append(CheckResult(
            name="stl-present",
            passed=False,
            detail=f"missing file: {path}",
        ))
        return report

    mesh = trimesh.load(path)
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(
            [g for g in mesh.geometry.values() if len(g.faces) > 0]
        )

    report.watertight = bool(mesh.is_watertight)
    report.volume_mm3 = float(mesh.volume)
    report.bounds_observed_mm = tuple(float(v) for v in mesh.extents)
    report.bounds_expected_mm = tuple(float(v) for v in
                                       spec_entry.get("bounds_mm",
                                                       (0.0, 0.0, 0.0)))
    report.checks.append(CheckResult(
        name="watertight",
        passed=report.watertight,
        detail=("OK" if report.watertight
                else "STL is NOT watertight — slicer will misbehave"),
    ))

    # Bounding-box check
    obs = np.array(report.bounds_observed_mm)
    exp = np.array(report.bounds_expected_mm)
    if exp.any():
        delta = np.abs(obs - exp)
        ok = bool((delta <= bounds_tol_mm).all())
        report.checks.append(CheckResult(
            name="bounds_within_tol",
            passed=ok,
            detail=(f"observed=({obs[0]:.1f},{obs[1]:.1f},{obs[2]:.1f}) "
                    f"expected=({exp[0]:.1f},{exp[1]:.1f},{exp[2]:.1f}) "
                    f"tol={bounds_tol_mm:.1f} mm"
                    f"{'' if ok else f'   delta={delta.tolist()}'}"),
        ))
    else:
        report.checks.append(CheckResult(
            name="bounds_within_tol",
            passed=True,
            detail="bounds_mm: TODO -- skipped",
        ))

    # Holes
    holes = spec_entry.get("holes", []) or []
    n_pass = 0
    for hole in holes:
        report.holes_checked += 1
        centre = np.asarray(hole["position_mm"], dtype=float)
        axis = _axis_vec(hole.get("axis", "z"))
        diameter = float(hole["diameter_mm"])
        ok, detail = _hole_probe(mesh, centre, axis, diameter)
        if ok:
            n_pass += 1
        report.checks.append(CheckResult(
            name=f"hole:{hole['name']}",
            passed=ok,
            detail=detail,
        ))
    report.holes_passed = n_pass

    # Wire channels: each named channel is delegated to
    # _verify_prototype.check_wire_slot, which actually has a tight
    # corridor sweep already.  Here we just RECORD the names; the
    # validator group_results section reports the verify_prototype
    # PASS/FAIL.
    channels = spec_entry.get("wire_channels", []) or []
    for ch in channels:
        report.wire_channels_checked += 1
        report.wire_channels_passed += 1
        report.checks.append(CheckResult(
            name=f"wire_channel:{ch['name']}",
            passed=True,
            detail=(ch.get("note", "")
                    + "  [delegated to _verify_prototype.check_wire_slot]"),
        ))

    # Keep-out volumes: NAME-resolved against the keepout_volumes
    # registry.  We sample the keep-out volume's INTERIOR and require
    # every sample to be OUTSIDE the printed part (= no part material
    # intrudes into the keep-out).
    try:
        from keepout_volumes import KEEP_OUT_VOLUMES
    except Exception as exc:
        KEEP_OUT_VOLUMES = None
        ko_import_error = str(exc)
    else:
        ko_import_error = None

    keep_outs = spec_entry.get("keep_out_volumes", []) or []
    for ko in keep_outs:
        report.keep_outs_checked += 1
        ko_name = ko["name"]
        if KEEP_OUT_VOLUMES is None:
            report.checks.append(CheckResult(
                name=f"keep_out:{ko_name}",
                passed=False,
                detail=(f"could not import keepout_volumes: "
                        f"{ko_import_error}"),
            ))
            continue
        if ko_name not in KEEP_OUT_VOLUMES:
            report.checks.append(CheckResult(
                name=f"keep_out:{ko_name}",
                passed=False,
                detail=("keep-out name is not in "
                        "keepout_volumes.KEEP_OUT_VOLUMES; add a "
                        "factory there or fix the YAML reference"),
            ))
            continue

        ko_mesh = KEEP_OUT_VOLUMES[ko_name]()
        pts = _voxel_sample_interior(ko_mesh)
        # Use the standard ``mesh.contains`` test (single ray) here:
        # the 6-ray majority vote in ``points_inside`` is intentionally
        # liberal and produces false positives near the part's outer
        # surface, which inflates the intrusion count on keep-out
        # volumes that sit FLUSH with the part (e.g. the horn-stack
        # void inside the femur's hip-end neck torus -- the void
        # surface IS the printed wall's inner surface).  ``contains``
        # is more conservative and gives a more useful signal here.
        try:
            inside = np.asarray(mesh.contains(pts), dtype=bool)
        except Exception:
            inside = _is_inside_majority(mesh, pts)
        n_inside = int(np.sum(inside))
        # Allow a small tolerance: voxelisation / CSG slop can put a
        # handful of samples right on a boundary.  <= 5% intrusion
        # passes -- this is a regression detector, not metrology.
        tol = max(2, int(0.05 * len(pts)))
        ok = n_inside <= tol
        if ok:
            report.keep_outs_passed += 1
        report.checks.append(CheckResult(
            name=f"keep_out:{ko_name}",
            passed=ok,
            detail=(f"{n_inside}/{len(pts)} interior samples inside "
                    f"part (tol={tol})"),
        ))

    # Shape: block (RULE 7) -- WARNINGS only, never flips PASS/FAIL.
    _check_shape_block(part_name, spec_entry, report)

    return report


# ---------------------------------------------------------------------------
# Group check: run _verify_prototype.main() and forward result
# ---------------------------------------------------------------------------

def _run_verify_prototype(fast: bool) -> tuple[bool, str]:
    """Run the existing prototype verification suite.  ``fast=True``
    monkey-patches the workspace sweep to a single pose so the slowest
    check completes in a few seconds.
    """
    try:
        import _verify_prototype as vp
    except Exception as exc:
        return False, f"could not import _verify_prototype: {exc}"

    if fast:
        # Reduce workspace sweep to ~1 pose to stay fast.
        vp.WORKSPACE_N_YAW = 1
        vp.WORKSPACE_N_FEMUR = 1
        vp.WORKSPACE_N_KNEE = 1

    # vp.main() builds its OWN argparse and reads sys.argv -- if we
    # leave OUR CLI args (--fast, --skip-build, …) in sys.argv they'd
    # cause argparse to abort with SystemExit(2).  Mask sys.argv to the
    # bare script name for the duration of the call.
    buf = io.StringIO()
    saved_argv = sys.argv
    try:
        sys.argv = [saved_argv[0] if saved_argv else "_verify_prototype"]
        with redirect_stdout(buf):
            rc = vp.main()
    except SystemExit as exc:
        rc = int(exc.code) if exc.code is not None else 0
    except Exception as exc:
        return False, f"_verify_prototype crashed: {exc}\n{buf.getvalue()}"
    finally:
        sys.argv = saved_argv

    output = buf.getvalue()
    return rc == 0, output


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def run_validation(*, fast: bool = False,
                   run_verify_prototype: bool = True,
                   skip_build: bool = False) -> ValidationReport:
    report = ValidationReport(
        started_unix=time.time(),
        fast=fast,
    )

    if not os.path.isfile(SPEC_PATH):
        report.group_results.append(CheckResult(
            name="design_spec_present",
            passed=False,
            detail=f"missing {SPEC_PATH}",
        ))
        report.finished_unix = time.time()
        return report

    spec = _load_yaml(SPEC_PATH)
    bounds_tol_mm = float(spec.get("bounds_tol_mm", 2.0))
    parts_spec = spec.get("parts", {}) or {}

    if not skip_build:
        try:
            _ensure_stls_built(list(parts_spec.keys()))
        except Exception as exc:
            report.group_results.append(CheckResult(
                name="ensure_stls_built",
                passed=False,
                detail=str(exc),
            ))
            report.finished_unix = time.time()
            return report

    # Collect TODO fields (look for the literal string 'TODO' in any
    # printability / part value).
    def _walk(node, trail):
        if isinstance(node, dict):
            for k, v in node.items():
                _walk(v, trail + (str(k),))
        elif isinstance(node, list):
            for i, item in enumerate(node):
                _walk(item, trail + (f"[{i}]",))
        else:
            if isinstance(node, str) and node.strip().upper() == "TODO":
                report.todo_fields.append(".".join(trail))

    _walk(spec, ())

    # Per-part checks
    for name, entry in parts_spec.items():
        part_report = _check_part(name, entry, bounds_tol_mm)
        report.parts.append(part_report)

    if run_verify_prototype:
        ok, output = _run_verify_prototype(fast)
        report.group_results.append(CheckResult(
            name="_verify_prototype.main()",
            passed=ok,
            detail=output,
        ))

    report.finished_unix = time.time()
    return report


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _print_report(report: ValidationReport) -> None:
    print()
    print("=" * 72)
    print("design-spec validation summary:")
    print("=" * 72)
    total_shape_warnings = 0
    for part in report.parts:
        ok = all(c.passed for c in part.checks)
        flag = "PASS" if ok else "FAIL"
        print(f"  [{flag}] {part.name:22s} "
              f"bounds={part.bounds_observed_mm[0]:5.1f} x "
              f"{part.bounds_observed_mm[1]:5.1f} x "
              f"{part.bounds_observed_mm[2]:5.1f} mm  "
              f"holes={part.holes_passed}/{part.holes_checked}  "
              f"channels={part.wire_channels_passed}/"
              f"{part.wire_channels_checked}  "
              f"keep-outs={part.keep_outs_passed}/"
              f"{part.keep_outs_checked}  "
              f"shape={part.shape_features_passed}/"
              f"{part.shape_features_checked}")
        for chk in part.checks:
            if not chk.passed:
                print(f"            FAIL  {chk.name}: {chk.detail}")
        for warn in part.warnings:
            total_shape_warnings += 1
            print(f"            WARN  {warn.name}: {warn.detail}")

    for grp in report.group_results:
        flag = "PASS" if grp.passed else "FAIL"
        print(f"  [{flag}] {grp.name}")
        if not grp.passed and grp.detail:
            for line in grp.detail.splitlines()[-30:]:
                print(f"            {line}")

    if report.todo_fields:
        print()
        print(f"  TODO fields in design_spec.yaml: {len(report.todo_fields)}")
        for trail in report.todo_fields:
            print(f"      - {trail}")

    if total_shape_warnings:
        print()
        print(f"  shape: WARN count: {total_shape_warnings} "
              f"(see RULE 7 in design_spec.yaml; warnings do NOT "
              f"flip PASS/FAIL).")

    print("=" * 72)
    if report.passed:
        print("All design-spec validations PASSED.")
    else:
        print("FAIL.  Fix the issues above before printing / committing.")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.
                                      RawDescriptionHelpFormatter)
    parser.add_argument(
        "--fast",
        action="store_true",
        help=("Skip the slow ``_verify_prototype`` workspace sweep "
              "(reduces it to a single pose).  Default: full sweep."),
    )
    parser.add_argument(
        "--skip-verify-prototype",
        action="store_true",
        help="Skip the legacy _verify_prototype.main() sub-step entirely.",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Don't auto-rebuild the STLs even if they are missing.",
    )
    args = parser.parse_args(argv)

    report = run_validation(
        fast=args.fast,
        run_verify_prototype=not args.skip_verify_prototype,
        skip_build=args.skip_build,
    )
    _print_report(report)
    return 0 if report.passed else 1


if __name__ == "__main__":
    sys.exit(main())
