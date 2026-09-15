# STEP-first CAD sidecar test

This directory is an additive migration experiment. It does not overwrite the
existing `hexapod_prototype.py` / `stl_prototype/` mesh pipeline.

It is not intended to become a second permanent geometry system. The migration
rule is:

1. Keep the current mesh generator as the trusted printer/reference path while
   STEP-first parts are being proven.
2. Import dimensions from `hexapod_prototype.py` during the experiment so
   constants do not fork.
3. Once the STEP-first builders cover the real printables, move them inward as
   the source of truth and make STL export a derived final step.
4. Delete or retire duplicated mesh builders after equivalence checks pass.

The goal is to prove the more typical CAD flow:

```text
build123d/OpenCascade BREP solid -> STEP for Onshape/CAD -> STL as final slicer mesh
```

Run from `dynrep-launch-clean/`:

```bash
uv run --no-project --python 3.12 \
  --with build123d --with trimesh --with numpy \
  python hexapod_walker/prototype_sts3215/cad_step_test/build_step_first_test.py
```

Assembly-view STEP compounds are generated separately because BuildViz scene
transforms expect assembly-local frames, while several printable STEP parts are
stored in print pose:

```bash
uv run --no-project --python 3.12 \
  --with build123d --with trimesh --with numpy \
  python hexapod_walker/prototype_sts3215/cad_step_test/build_step_assembly_views.py --with-servos
```

A focused yaw-bearing stack for one leg plus `chassis_bottom` can be generated
from the same BuildViz scene transforms:

```bash
uv run --no-project --python 3.12 \
  --with build123d --with trimesh --with numpy \
  python hexapod_walker/prototype_sts3215/cad_step_test/build_step_assembly_views.py \
    --yaw-bearing-focus --leg-index 0
```

A full connected load-path assembly for Onshape structural triage includes both
chassis plates and all six leg load paths:

```bash
uv run --no-project --python 3.12 \
  --with build123d --with trimesh --with numpy \
  python hexapod_walker/prototype_sts3215/cad_step_test/build_step_assembly_views.py \
    --connected-full-load-path
```

The repeatable MuJoCo-to-Onshape prep target replays standing, walking, rising,
and lowering; writes a full-chassis STEP; then generates a load-case manifest
and Onshape setup sheet:

```bash
make -C hexapod_walker/prototype_sts3215 prepare-onshape-strength
make -C hexapod_walker/prototype_sts3215 prepare-onshape-strength \
  ONSHAPE_STUDY_ARGS=--upload
```

The optional stock C-horn variant has its own additive STEP-first exporter. It
imports the measured/assumed constants from `tools/make_chorn_variant.py`, but
does not write to `extra_stl/chorn/`:

```bash
uv run --no-project --python 3.12 \
  --with build123d --with trimesh --with numpy \
  python hexapod_walker/prototype_sts3215/cad_step_test/build_chorn_step.py
```

The rigid-hip variant's exporter GRADUATED out of this directory (Aug 26
2026): for that concept, STEP-first is the OFFICIAL pipeline, not a
sidecar.  Its builder lives at `concepts/rigid_hip/build_rigid_hip_step.py`
— it is the canonical geometry source for the seven variant printables
(the trimesh twins are retired), imports the variant constants from
`concepts/rigid_hip/make_rigid_hip_variant.py`, and still reuses this
directory's BREP builders for the production parts the variant edits
(coxa link, chassis bottom, servo clamp cap).  Its outputs land in
`concepts/rigid_hip/step/`, and `make_rigid_hip_variant.py` (the
assembly/check driver) runs it automatically.  See
`concepts/rigid_hip/README.md` for the flow.

Shared export machinery (part spec dataclass, STEP+STL export, manifest rows,
bundle zipping) lives in `step_common.py`; all three exporters use it.  The
output directories are parameterizable (`export_all`/`export_one`/
`write_bundle` take an `out_dir`, defaulting to the shared `out/` pool);
manifest paths and bundle members are relative to that `out_dir`.

Base and C-horn outputs are written to `cad_step_test/out/`:

- `step/*.step`: clean CAD/BREP exchange files.
- `stl/*.stl`: final tessellated slicer meshes generated from the BREP solids.
- `manifest.json`: dimensions, BREP face counts, STL triangle counts, and
  legacy-STL comparison when a baseline STL exists.
- `assembled_robot*_manifest.json`: composed robot-view STEP/STL diagnostics
  using BuildViz's assembly frames.
- `yaw_bearing_focus_L*_manifest.json`: compact one-leg yaw stack diagnostics.
- `connected_full_robot_load_path_manifest.json`: full chassis plus all six
  connected leg load paths for Onshape structural triage.
- `chorn_manifest.json`: STEP-first stock C-horn variant diagnostics.
- `hexapod_step_first_test_bundle.zip`: a small bundle suitable for uploading
  to Onshape as a test.
- `chorn_step_first_bundle.zip`: C-horn variant STEP/STL bundle.

Rigid-hip outputs live in the concept directory instead
(`concepts/rigid_hip/step/`: the seven printables' `.step` files, their
tessellations in `step/stl/`, manifest + bundle) — documented in
`concepts/rigid_hip/README.md` now that the exporter graduated there.

## Current migration scope

Migrated in this sidecar:

- `chassis_top`
- `chassis_bottom`
- `coxa_link`
- `disc_horn`
- `femur_link`
- `foot_boot`
- `servo_body`
- `servo_clamp_cap`
- `switch_holster`
- `tibia_knee_yoke`
- `tibia_tube_printable`
- `yaw_bearing_cap`
- `yaw_bearing_lower`
- `yaw_servo_retainer`
- `yaw_bearing_upper`

Additional additive STEP-first diagnostics:

- `yaw_bearing_focus_L0` assembly: full `chassis_bottom` plus leg-0
  `coxa_link`, `yaw_bearing_cap`, lower/upper 6805 bearings, and yaw
  `disc_horn`.
- `connected_full_robot_load_path` assembly: full chassis plus all six
  connected leg load paths, including servo bodies, disc horns, and primitive
  fastener proxies where the BuildViz scene carries them.
- Stock C-horn variant parts:
  - `chorn_reference_DO_NOT_PRINT`
  - `spacers`
  - `femur_chorn_body`
  - `tibia_chorn_socket`
- Rigid-hip variant parts (`concepts/rigid_hip/build_rigid_hip_step.py`,
  graduated out of this directory -- STEP-first is that concept's
  official pipeline):
  - `hip_clamp_cap_rigid`
  - `chassis_top_rigid`
  - `top_hatch_rigid`
  - `corner_pillar`
  - `centre_wago_block`
  - `coxa_link_rigid`
  - `chassis_bottom_rigid`

Fidelity notes (Aug 24 2026 catch-up pass, found by the rigid-hip
equivalence check): the base builders were re-synced with production mesh
features they predated -- the clamp cap's back-face hook, horn-side mini
hook, and yoke-sweep edge chamfers (Aug 18-19); the cradle's rear retention
tab and retired wire-exit corridor / end-face bolts on the hip cradle
(Aug 17); and the coxa's yoke-sweep reliefs and deeper centre-screw seat.

Aug 26 2026 equivalence audit (two-sided surface deviation + boolean XOR
against the production meshes, gating the build_all.py flip) found and
fixed three more real port gaps:

- the moving-yoke port lacked production's ``pad_extra_reach`` --
  ``YOKE_PAD_EXTRA_REACH`` (0.5 mm deeper horn pads on the femur hip yoke
  and tibia knee yoke; femur volume gap 0.5% -> 0.01%);
- ``make_coxa_hip_bracket`` hardcoded the old femur-swing clearance plane
  ``18.5`` instead of ``hp.COXA_HIP_DROP - 24.9`` (a phantom 0.5 mm step
  cut into the slab underside);
- ``make_chassis_bottom`` still modelled the RETIRED two-bay WAGO3 corner
  trays and lacked the Phi 9 standoff seat pads (Aug 16 production
  features; max deviation 6.5 mm -> 0.05 mm).

All ten ``stl_prototype/`` printables now match their mesh twins within
0.13 mm max surface deviation (nine within 0.06 mm).  The one 0.12 mm
outlier is ``chassis_top``: the mesh pipeline clips the deck with a
48-gon circle (``CYL_SECTIONS``) whose sagitta sits 0.123 mm inside the
true BREP circle -- a tessellation artifact of the LEGACY pipeline, not a
port defect (the BREP is more accurate).  ``coxa_link`` / ``femur_link``
/ ``foot_boot`` still tessellate non-watertight out of OCC;
``hp._heal_for_export`` closes all three (verified ``is_volume``), which
is exactly the pass the print-set installer applies before anything
reaches ``stl_prototype/``.

Still pending:

- None. Every production printable has a native BREP builder here, proven
  equivalent; the print set is generated from these builders via
  ``build_step_prototype.py`` (see the prototype README).

A mesh-to-STEP wrapper would preserve the triangles and would not solve the
Onshape artifact issue; this sidecar uses native BREP solids before exporting
STEP.
