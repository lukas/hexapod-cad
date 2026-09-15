# hexapod-cad

Mechanical design of the 18-servo Feetech STS3215 hexapod: parametric CAD,
STEP-first printables, BOM, fastener registry, strength checks, BuildViz
scenes, concept variants, and the generator for the as-built MuJoCo mesh model.

Split out of [lukas/hexapod](https://github.com/lukas/hexapod) on 2026-09-15
(source commit `c3113d88b`, path `hexapod_walker/prototype_sts3215/`). That
repo keeps the robot software, simulation and RL; its history still has every
CAD commit up to the split. This repo starts as a snapshot.

Start at [`PROTOTYPE.md`](PROTOTYPE.md) for the build story, printing and
assembly; [`docs/`](docs/) for BOM, CAD workflow, BuildViz and agent rules;
[`concepts/README.md`](concepts/README.md) for the variant catalog.

## Layout

| Path | What |
|------|------|
| `hexapod_prototype.py` | Parametric constants + trimesh twins (the source of truth for every dimension) |
| `_verify_prototype.py` | Design-rule verifier (`make verify-fast` / `verify-prototype`) |
| `cad_step_test/build_step_first_test.py` | Printable BREP builders (STEP-first geometry source) |
| `build_step_prototype.py` / `step_pipeline.py` / `build_all.py` | Print-set exporter + STL/STEP regeneration (`make build`) |
| `design_spec.yaml` | Human-readable geometry contract |
| `fastener_registry.py`, `fasteners/` | Fastener BOM + cache meshes |
| `concepts/` | Isolated mechanical and sensing experiments |
| `strength/` | Opt-in FEA failure-point check (Gmsh + CalculiX) |
| `full_robot_viz/`, `tools/` | BuildViz scene builder, checks, cloud mirror, diagnostics |
| `scripts/` | check_cad, validate, render, inspect, fastener BOM, harness diagram |
| `mesh_mujoco/build_mesh_model.py` | Builds the as-built MuJoCo mesh model consumed by the robot repo |
| `xtool/`, `extra_stl/`, `apriltag_lids/`, `knee_yoke_apriltag_flag/`, `apriltags/`, `arm/`, `stl_reference/` | Fixtures, jigs, tag mounts, reference meshes |
| `prototype_ak40/`, `vehicles/` | Other designs (AK40 concept variant, vehicles) |

## Setup

```sh
uv sync                 # CAD toolchain (build123d, trimesh, pyvista, mujoco, ...)
uv sync --extra sim     # + the robot repo checked out at ../hexapod, for the
                        #   bridges that need its joint contract / sim env
make help
```

`make verify-fast` includes the harness-reach check, which needs the `sim`
extra (it reads servo ids from the robot repo's joint contract). Point the
path dependency at an up-to-date checkout; a stale `../hexapod` fails with
`cannot import name 'servo_id'`.

## Contract with the robot repo

Two things cross the boundary, both committed on the robot side:

1. `hexapod_core/geometry.py` in lukas/hexapod is a frozen copy of the
   dimensions the MuJoCo primitive model needs. Change a dimension here, update
   that file in the same change.
2. `mesh_mujoco/assets/*.stl` + `hexapod_mesh.xml` there are the output of
   `make mesh` here. `make mesh-sync HEXAPOD=../hexapod` regenerates and copies
   them; commit the result in the robot repo.

Joint order, joint names and servo ids are defined once, in the robot repo's
`hexapod_core/joint_frame.py`; the bridges here import it via the `sim` extra.
