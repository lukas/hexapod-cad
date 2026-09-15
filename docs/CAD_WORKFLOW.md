# Hexapod prototype — CAD workflow

This folder builds a tabletop hexapod from 3D-printed parts.  The
pipeline is parametric end-to-end: there is **no GUI CAD file** — the
STLs in `stl_prototype/` are derived artifacts.  This document is the
short version of how the pieces fit together and how to validate a
change before printing.

## Source of truth

| Layer                                  | File                          | What it is                                                              |
|----------------------------------------|-------------------------------|-------------------------------------------------------------------------|
| Parametric constants + probe/visual geometry | `hexapod_prototype.py`  | Python + `trimesh`.  All `make_*` functions; constants block at the top.|
| Printable BREP geometry (STEP-first)   | `cad_step_test/build_step_first_test.py` | build123d/OpenCascade solids for every printable; imports every dimension from `hexapod_prototype.py` (no constant forks). |
| Print-set exporter + equivalence gates | `build_step_prototype.py`     | Exports `step_prototype/*.step` + tessellations; PROVES each printable equivalent to its trimesh twin (volume / bbox / max surface deviation) before install.  Layout + subprocess plumbing in `step_pipeline.py`. |
| STEP CAD exchange files                | `step_prototype/*.step`       | The editable BREP truth per printable (Onshape/CAD import ready).      |
| Derived printable STLs                 | `stl_prototype/*.stl`         | Healed BREP tessellations installed by `build_all.py` (STEP-first since Aug 2026). |
| Reference / sim visuals (not print)    | `stl_reference/*_DO_NOT_PRINT.stl` | Servo / board / fused-link meshes for MuJoCo & BuildViz (still trimesh). |
| Human-readable design contract         | `design_spec.yaml`            | Local frames, bounding boxes, holes, channels, keep-out volumes.        |
| Code-driven keep-out / clearance shapes| `keepout_volumes.py`          | Named `trimesh` factories tracking the parametric constants.            |
| Deep validation                        | `_verify_prototype.py`        | The 34-check correctness suite (manifoldness, cradle openness, …).      |
| Light validation + reporting           | `scripts/validate_geometry.py`| Spec-vs-STL regression check + report generator.                        |

The contract is: **constants in `hexapod_prototype.py` are the truth**.
`design_spec.yaml` mirrors a curated subset of those constants and adds
human-readable documentation.  The validator catches drift between the
two.

STEP-first (Aug 2026): the printables' shipping geometry is authored as
build123d BREP solids and flows `BREP -> .step -> tessellated .stl`;
`hexapod_prototype.py`'s trimesh `make_*` twins remain for constants,
the verifier's probes, MuJoCo and BuildViz.  The exporter's equivalence
gates plus the verifier's freshness check (manifest source hashes +
BREP-vs-twin backstop) keep the two representations locked together —
edit BOTH the trimesh twin and the BREP builder when changing a
printable, then `make build`.

## One-liner: validate everything before you commit

```
make -C hexapod_walker/prototype_sts3215 check-cad
```

This runs (from the repo root):

1. `build_all.py` to ensure `stl_prototype/*.stl` exists.
2. `scripts/validate_geometry.py` — bounding boxes, holes, keep-out
   volumes (resolved against `keepout_volumes.KEEP_OUT_VOLUMES`), and
   the full `_verify_prototype.main()` suite.
3. `scripts/render_views.py` — 4 standard views (front / side / top /
   iso) per part, with keep-out volumes overlaid translucently, under
   `artifacts/views/<part>/`.
4. `artifacts/cad_report.md` — copy-pastable Markdown summary of pass/
   fail status, discovered dimensions, missing `TODO`s in the spec,
   exact error messages from the validator, and links to all generated
   images.

`make check-cad-fast` runs the same pipeline but reduces the
`_verify_prototype` workspace-sweep collision check to a single pose
(useful for the inner-loop edit cycle; **don't trust a fast-mode pass
as the final go-ahead** — run the full sweep at least once before
committing).

## Individual steps

| Goal                         | Make target            | Equivalent direct invocation                                                                          |
|------------------------------|------------------------|-------------------------------------------------------------------------------------------------------|
| Regenerate STLs              | `make build`           | `./run.sh hexapod_walker/prototype_sts3215/build_all.py --skip-assembly` |
| Run spec validator only      | `make validate`        | `./run.sh hexapod_walker/prototype_sts3215/scripts/validate_geometry.py`                                      |
| Render PNG views only        | `make render`          | `./run.sh hexapod_walker/prototype_sts3215/scripts/render_views.py`                                            |
| Legacy verify only           | `make verify-prototype`| `./run.sh hexapod_walker/prototype_sts3215/_verify_prototype.py`                                              |

The render backend tries `pyrender` → `pyvista` → `matplotlib` in
order; the first one that works is used.  `pyvista` is a declared
dependency in the repo-root `pyproject.toml`, so the pipeline degrades
gracefully on a fresh checkout.

## Recommended edit loop

1. Read the relevant `make_*` function's docstring and the constants
   it references.
2. Make the smallest possible parametric change.
3. `make -C hexapod_walker/prototype check-cad-fast` for a quick check
   (~30 s).
4. If FAIL: open `artifacts/cad_report.md`, find the failing check,
   paste its line into your LLM along with the relevant `make_*` source.
5. If PASS: run `make -C hexapod_walker/prototype check-cad` (the full
   workspace sweep, ~ 2 min) before committing.

See `CAD_AGENT_INSTRUCTIONS.md` for the rules an LLM coding agent
should follow when editing CAD.
