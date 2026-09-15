# BuildViz

This project is set up for BuildViz. Agents should use BuildViz when inspecting
or debugging CAD/build outputs, STL assets, `scene.json`, or
`design_spec.yaml`.

## Files

- `scene.json`: BuildViz scene manifest. It lists meshes, instances, transforms,
  colors, roles, and focus groups — **and now the baked-in motion blocks**
  (`joints[]` + `poses[]` + `animations[]`). It lives in `full_robot_viz/`.
- `design_spec.yaml`: Semantic design source for part descriptions, features,
  holes, dimensions, and LLM context.
- STL files: Mesh assets referenced by `scene.json`.

## One build, motion baked in

There is exactly ONE build for the full robot: **`prototype_sts3215`**, served
from `full_robot_viz/`. Motion is part of that single `scene.json` — it carries
BuildViz's `joints[]` (18 leg DOFs: 6 legs × yaw/hip/knee), `poses[]` (stance
variants + a tripod march) and a looping `animations[]` walk clip, on the exact
same meshes and home-pose transforms as the static geometry. This mirrors the
prototype_v1 `hexapod-prototype` build.

There is **no** separate `scene_motion.json` file and **no** separate
`prototype_sts3215_motion` build id any more — both were retired when motion was
folded into the one build.

### Why motion is always on (but the sweep is opt-in)

Measured Jul 2026: baking the motion blocks costs ~0.01 s on top of the ~4 s STL
rebuild — free — so motion is **always** generated and published. Every push also
runs the cheap `buildviz validate` (~1 s) to confirm the joints/poses are
well-formed. The one expensive motion test is the swept self-overlap check
(`buildviz sweep`, ~13 s ≈ 3× the whole build), so it is **opt-in**:

```sh
make -C hexapod_walker/prototype_sts3215 verify-buildviz          # fast: build + validate + push (motion baked in)
make -C hexapod_walker/prototype_sts3215 verify-buildviz SWEEP=1  # + swept self-overlap motion test (~13 s)
```

## Commands

Validate the build:

```sh
npx buildviz validate . --json
```

View the build — use the ONE machine-wide hub (default port `5183`), never a
new per-project dev server:

```sh
npx buildviz hub --detach                        # idempotent; single hub on :5183
# publishing is normally done by `make verify-buildviz`; to register the dir directly:
npx buildviz register full_robot_viz --build-id prototype_sts3215
# then open: http://127.0.0.1:5183/?build=prototype_sts3215
```

Do NOT run `npm run dev` or `npx buildviz --port <n>` (e.g. the old
`--port 5174` motion-preview pattern) to view builds — the hub already serves
every project's builds at once. See `~/buildviz/README.md` ("How to run
BuildViz") for the full convention.

Regenerate a starter scene only when `scene.json` is missing:

```sh
npx buildviz init --dry-run
npx buildviz init
```

## Agent Notes

- Do not guess part semantics from STL geometry alone. Use `design_spec.yaml`
  as the source of truth.
- If asking a human a visual question, use `window.buildviz.setHighlights(...)`
  with `annotation` text after opening the viewer.
- This initial scene may place STL files in a simple grid. For a true assembly,
  update `scene.json` with real transforms from the CAD/export pipeline.

Build name: prototype

## Catalog discipline (moved here from the root AGENTS.md, 2026-09-14)

Start with MCP `list_catalog({collection:"hexapods"})`, then
`get_catalog_item({id:"..."})`, before choosing or creating a build. CLI
fallback: `buildviz catalog list --collection hexapods --json` and
`buildviz catalog show <id>`. Read the returned source and children rather
than inferring identity from an old project name or version count.

Robot entries on the cloud hub (`https://buildviz.cwd1f0-new-cluster.coreweave.app/?catalog=<id>`):

- `hexapod-1`: original RobotLab STS assembly; as-built pinned to
  `prototype_sts3215/hexapod-v1`, branch `main`, version `2026-08-07-f9c91cf`.
- `hexapod-2`: three-bearing STS build (two lower, one upper bearing, spacer
  retrofit `sts-horn-compression-study` in progress); per-leg state in
  `robots/hexapod-2.yaml`. The bundled `buildviz/hexapod-2` scene does not
  identify this robot.
- `hexapod-metal`: planned purchased-56 mm-bracket build from
  `prototype_sts3215/premade-chorn-56`; CNC and split-clamp alternatives are
  studies beneath it.

Rules:

- `create_view` for an inspection selection of existing geometry, pinned to
  an exact source branch and revision. A view does not create a robot.
- `publish_revision` for geometry changes under the existing identity;
  alternatives, coupons, loading setups and comparisons are studies with an
  explicit parent. Every revision carries a one or two sentence message.
- Keep CAD `source` separate from `asBuilt`; record installed hardware only
  from evidence. Version numbers have been reused; sequence by timestamp.
- Preserve old source IDs, branches and URLs; archive obsolete entries
  instead of deleting histories.
- Mirror to the cloud hub after a local publish (`make verify-buildviz` does
  this; standalone `make push-cloud`). Auth is `X-API-Key` = `BUILDVIZ_API_KEY`
  from the CoreWeave secret `buildviz-api-key`. A dead network must never
  fail the local publish.

Hub build ids: `prototype_sts3215` (full robot, motion baked into its
scene.json) with sibling builds `prototype_sts3215/rigid-hip`,
`cnc-chorn-overhead`, `chassis-reinforcement-test`,
`tibia-yoke-reinforcement-test`, `cnc-chorn-two-piece`, `fsr-sensor-foot`,
`horn-compression-limiters`, `premade-chorn-56` (see `concepts/README.md`),
plus `hexapod-prototype` (prototype_v1). Stale copies of retired ids
(`cnc_chorn_overhead`, `sts3215-rigid-hip`, `sts3215-rigid-hip-step`) linger
on the cloud hub, which has no delete endpoint; ignore them. List live builds
with `npx buildviz hub status`. Reference: `/Users/Shared/buildviz/README.md`
and `BUILDVIZ_LLM_INTERFACE.md`; if the package is not on npm's path, run
`/Users/Shared/buildviz/bin/buildviz.mjs` directly.
