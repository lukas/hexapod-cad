# mesh_mujoco — the as-built robot in MuJoCo (EXPERIMENTAL)

The real STS3215 hexapod in MuJoCo: every printed part plus the bought
hardware (servos, disc horns, 6805 bearings, CF tubes, TPU boots, LiPo
packs, the electronics stack) placed at the **verified CAD assembly
transforms**, used for rendering **and** collision — not the capsule/box
approximation `mujoco_prototype.py` simulates.

Self-contained experiment: nothing outside this directory is touched, and
nothing in `rl_move/` / `sim_viewer/` consumes it.

## Quickstart

```sh
make build          # CAD factories -> assets/*.stl + hexapod_mesh.xml (~3 s)
make view           # interactive viewer at the standing plant (mjpython)
make view-stance    # ... at the CAD display stance (belly-rest, see below)
```

`build` also runs the self-checks (mass, penetration scan, 3 s settle at
both keyframes) and writes `previews/*.png`.

Direct builder invocation, for agents or scripts that must avoid Makefile
indirection:

```sh
uv run --with numpy --with scipy --with shapely --with trimesh \
  python build_mesh_model.py --no-render
```

## Where the geometry comes from

- Link/servo placements: `tools/full_robot_viz_build.py`'s leg-0 local-frame
  part lists (the BuildViz scene math), guarded at build time by
  `_assert_servo_placement()` against `_verify_prototype`'s authoritative
  poses. Disc horns come straight from the verifier's
  `_horn_world_transform` / `_passive_horn_world_transform`.
- Meshes are built by the `hexapod_prototype.make_*()` factories (no
  dependence on `stl_prototype/` exports). The electronics stack needs the
  ignored helper STLs from `tools/make_xtool_hex_mount_plate.py` and
  `tools/make_xtool_hex_raised_platform.py`; if they are missing, the builder
  refuses by default so RL assets cannot silently become a lighter bare-plates
  robot.
- Fasteners (~240 screws) are the one part family intentionally omitted.

## What the mesh model fixes vs the legacy sim

Discovered by diffing against `mujoco_prototype.py` while building this:

1. **Hip axis height.** Real `COXA_HIP_ANCHOR` puts the hip-pitch axis
   +38.4 mm above the yaw output plane (and the yaw plane itself at
   +15.25 mm over the chassis, not +7). The legacy sim put the hip at the
   yaw plane, so its whole leg chain rode ~46 mm too high — its "stance
   height" numbers are off by about that much.
2. **Foot line.** The tibia/foot runs at the sandwich mid-plane (+24.15 mm
   along the knee axis), which cancels the hip anchor's −25.65 mm
   tangential offset. Legacy left the foot ~24 mm off the leg radial.
3. **Tibia span.** The CAD merged-tibia formula measures the tube run from
   the yoke socket and lands the boot apex at ~179.5 mm; the bench-measured
   knee→tip span is `TIBIA_LENGTH` = 150 mm. Printed parts stay exact here;
   the CF tube (a cut-to-length bought part) is cut so the apex lands at
   exactly 150 mm. All six legs use the same tube length and standard
   `foot_boot`.
4. **The CAD display stance (−25°/+75°) is not a standing pose.** With the
   real hip height the feet reach 22 mm above the under-belly hardware:
   holding it, the robot rests on the six yaw-servo retainer stirrups
   (base ≈ 40 mm). The robot genuinely stands at the RL plant
   (+20°/+80° rel, keyframe `plant`, base ≈ 125 mm) — verified by a 3 s
   servo-hold settle: all six boots loaded, 1 mm sag, upright.

## Compatibility with the RL stack conventions

Joint names (`L{i}_yaw/pitch/knee`), zero pose (legs straight out), joint
ranges, actuator layout (position+velocity pairs, 2.2 N·m clamp), body/geom/
site names (`chassis`, `L{i}_pad`, `L{i}_foot`, `L{i}_foot_site`,
`chassis_imu`) and the sensor list all match `mujoco_prototype.py`, so qpos
and ctrl semantics carry over. Actuator gains are the legacy defaults —
the fitted STS3215 gains from `rl_move/sim/sim_model.json` can still be
pushed with `servo_model.apply_params_to_model`.

## Physics notes

- Collision is per-part **convex hulls** (MuJoCo's native mesh contact) of
  the real part meshes. Concave parts are hull-filled per part, which is far
  tighter than one hull per link; the build reports any non-floor
  penetrations at both keyframes (currently: none).
- Horns, bearings and the electronics stack are render-only (group 2);
  LiPo packs and servo retainers DO collide — they are what the robot
  rests on when it sits.
- Masses are the AS-BUILT estimate (total **3.50 kg** — the legacy sim's
  2.104 kg budgets were never weighed and are ~40 % light): bought parts at
  spec/typical grams (`full_robot_viz_build.KNOWN_PART_MASSES_G`: STS3215
  61 g, 6805-2RS 10 g, LiPo ~140 g est, metal horns, ...), prints at CAD
  volume × infill-corrected density (25 % gyroid plates ×0.5, 30–40 %
  brackets ×0.6 of solid PLA), plus per-body screw + wiring allowances
  (`EXTRA_MASS_G`). One table drives BuildViz `get_mass_properties`
  (scene `checksConfig.partMassesGrams`) and both MuJoCo models. Refine
  by weighing the robot / one LiPo pack. Inertias follow the true shapes.
- Boot contact: soft solref + high friction on the actual TPU dome mesh
  (whose apex IS the printed hemisphere).

## Files

| file | what |
|---|---|
| `build_mesh_model.py` | generator: CAD factories → `assets/` + `hexapod_mesh.xml` + checks + previews |
| `view_mesh.sh` / `view_mesh.py` | interactive viewer (`plant` / `stance` keyframes) |
| `assets/`, `hexapod_mesh.xml` | generated, gitignored — rebuild any time |
| `hexapod_mesh_mjx.xml` | CHECKED-IN primitive-collision twin: same kinematics, baked full-mesh inertials, fitted box/capsule/sphere contacts with legacy geom names (so `servo_model`'s collision rewrites apply). MJX-compatible; what pods and asset-less checkouts load for `env.model_source=mesh` |
| `previews/*.png` | rendered stills from the last build |
