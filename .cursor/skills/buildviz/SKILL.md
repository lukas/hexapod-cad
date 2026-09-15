---
name: buildviz
description: Use BuildViz for CAD/build visualization workflows. Use when working with scene.json, design_spec.yaml, STL files, physical build assets, feature labels, dimensions, or when the user asks to inspect, visualize, validate, or annotate a mechanical design.
---

# BuildViz

## When To Use

Use this skill when the task involves BuildViz, `scene.json`,
`design_spec.yaml`, STL assets, CAD build visualization, feature labels,
dimensions, or human-in-the-loop visual debugging.

## Two-port convention: 5183 central, 5173 dev

BuildViz uses exactly **two** fixed ports:

- **`5183` = the shared central hub** — the ONE instance everyone uses. It
  serves *every* project's builds at once. View builds at
  `http://127.0.0.1:5183/?build=<id>`; projects expose builds by REGISTERING (or
  pushing) into it.
- **`5173` = reserved for BuildViz's own dev/testing** (`npm run dev`). Never
  the hub. **Leave it alone** — don't view project builds on it, don't register
  into it, don't kill it.

Never start a server on any other (random) port to view a build. Full details:
`~/buildviz/README.md` → "How to run BuildViz" and
`~/buildviz/BUILDVIZ_LLM_INTERFACE.md`.

## Workflow

1. Check for `BUILDVIZ.md`, `scene.json`, and `design_spec.yaml` in the
   project root.
2. If `scene.json` is missing, run:

   ```sh
   npx buildviz init --dry-run
   ```

   Only run `npx buildviz init` after confirming it will not overwrite useful
   project files.

3. Validate before relying on the viewer:

   ```sh
   npx buildviz validate . --json
   ```

4. Ensure the central hub is running (idempotent — a no-op if already up):

   ```sh
   npx buildviz hub --detach        # single machine-wide hub on :5183
   ```

5. REGISTER this project's build into the hub, then open it in the hub:

   ```sh
   npx buildviz register . --project <project> --build <build>
   # then view: http://127.0.0.1:5183/?build=<project>/<build>
   ```

## Asking Human Questions

Use the runtime highlight API in the browser:

```js
window.buildviz.setHighlights({
  parts: [{ partType: 'coxa_link', annotation: 'Should this part be wider?' }],
  points: [{ partType: 'coxa_link', point: [0, 0, 0], annotation: 'Is this origin right?' }],
})
```

Clear highlights:

```js
window.buildviz.clearHighlights()
```

## Rules

- Always use the single central BuildViz hub on port `5183`
  (`npx buildviz hub --detach`, `http://127.0.0.1:5183`); register builds into
  it. Port `5173` is reserved for BuildViz's own dev/testing — leave it alone.
  Never start a server on any other (random) port.
- Treat `design_spec.yaml` as the source of truth for design intent.
- Treat `scene.json` as the source of truth for what BuildViz renders.
- Do not infer design intent from STL geometry alone.
- Do not overwrite `scene.json`, `BUILDVIZ.md`, or Cursor skill files unless
  the user explicitly asks.
