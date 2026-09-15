# Hexapod AK40 prototype

Quasi-direct-drive tabletop hexapod: 18 × CubeMars AK40-10 (KV170)
actuators on CAN, Raspberry Pi 5, 6S. Third-generation sibling of
[`prototype_sts3215`](../prototype_sts3215/README.md) — same hex + 6×3-DOF
architecture, but real torque control (MIT mode) at every joint.

**Status: printable-pending-verification (Design D, Aug 2026).** Actuators
in hand; `stl_prototype/` holds the printable set generated from the
official CubeMars drawing, and the assembly is build-audited: all 144
fasteners are instanced in BuildViz with machine-checked attachment,
overlap, tool-access, and combined-pose ROM audits (`make audit`), and
an audited assembly order in [`PROTOTYPE.md`](PROTOTYPE.md) §10. Caliper
checks on one unit gate the six-set print run. Start at
[`PROTOTYPE.md`](PROTOTYPE.md) §9 for the build path.

| You are here to… | Start at |
|------------------|----------|
| Understand the design (rationale, budgets, architecture) | [`PROTOTYPE.md`](PROTOTYPE.md) |
| Buy parts / plan the bench bring-up | [`docs/BOM.md`](docs/BOM.md) |
| Change geometry / regenerate budgets + reference meshes | `hexapod_ak40.py` (source of truth; `design_spec.yaml` mirrors it) |

## Layout

| Path | What |
|------|------|
| `hexapod_ak40.py` | Parametric constants + budget report + reference massing meshes |
| `design_spec.yaml` | Human-readable geometry contract (mirror of the .py) |
| `PROTOTYPE.md` | Design rationale, torque budget, electrical + control architecture |
| `docs/BOM.md` | Everything to buy; bench test order |
| `stl_prototype/` | Slicer-ready printables (pending 4 caliper checks — see BOM) |
| `stl_reference/` | `*_DO_NOT_PRINT.stl` assembly meshes (stance/envelope checks only) |
| `full_robot_viz/` | BuildViz scene, all 200 instances incl. every fastener (`make view-buildviz` regenerates + registers into the :5183 hub) |
| `scripts/check_scene.py` | Full-assembly boolean overlap sweep (`make check-scene`) |
| `scripts/rom_audit.py` | ROM onset search + combined worst-case poses (`make audit`) |
| `scripts/access_audit.py` | Straight-driver tool path for all 144 fasteners (`make audit`) |
| `artifacts/` | Generated `design_budget.md` + `attachment_report.md` |
| `Makefile` | `build` / `check` / `check-scene` / `report` / `help` |

## Quick commands

```sh
make -C hexapod_walker/prototype_ak40 report      # mass + torque budget table
make -C hexapod_walker/prototype_ak40 build       # regenerate all STLs
make -C hexapod_walker/prototype_ak40 check       # + watertightness verification
make -C hexapod_walker/prototype_ak40 check-scene # full-assembly overlap sweep
```

Hardware-safety rules in the repo root `AGENTS.md` apply to this robot
unchanged once it is powered.
