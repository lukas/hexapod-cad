# STS3215 concept catalog

This directory contains isolated experiments, not the production robot CAD.
Each concept keeps its generator, semantic design spec, BuildViz scene, and
evidence together so experimental assumptions do not leak into
`hexapod_prototype.py` or the STEP-first production pipeline.

## Joint drive and printable mechanisms

- [`rigid_hip/`](rigid_hip/) — rigid-hip bearing and yoke variant.
- [`cnc_chorn_overhead/`](cnc_chorn_overhead/) — CNC C-horn overhead layout.
- [`cnc_chorn_two_piece/`](cnc_chorn_two_piece/) — two-piece version of the CNC
  C-horn concept.
- [`premade_chorn_56/`](premade_chorn_56/) — off-the-shelf 56 mm C-horn study.
- [`horn_compression_limiters/`](horn_compression_limiters/) — metal
  compression-limiter and spacer experiments.
- `dovetail_coxa/` and `qr_femur/` — printable link-joining experiments.

## Sensing and contact

- [`fsr_sensor_foot/`](fsr_sensor_foot/) — force-sensitive-resistor foot and
  load-transfer stack.

The AprilTag perception package is not a mechanical concept. It lives in the
separate [`../hexapod-tracker/`](../hexapod-tracker/) submodule.

## Structural reinforcement

- [`chassis_reinforcement/`](chassis_reinforcement/) — chassis and yaw-pocket
  load-path test.
- [`tibia_yoke_reinforcement/`](tibia_yoke_reinforcement/) — tibia/yoke
  reinforcement study.

Concept paths and BuildViz build IDs are intentionally stable because design
specs, generated scenes, and build history refer to them. Promote a concept
into the production CAD only after its own checks and physical fit evidence
are complete.
