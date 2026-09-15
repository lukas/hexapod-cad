# Tibia/yoke reinforcement load-path test

This BuildViz sidecar is the focused reinforcement sandbox for the weak path
the repo evidence actually supports: foot boot -> carbon tube -> tibia knee
yoke/socket -> knee horn stack.

It compares:

- current printed tibia knee yoke/socket path, with red markers on the
  documented spine split path;
- optional bought aluminum C-horn path from `docs/CHORN_VARIANT.md`, keeping
  the same tube mouth position and foot geometry.

The meshes here are clean BuildViz envelopes, not replacement production
geometry. The production-shape parts remain in the STEP-first C-horn workflow.

Run from repo root:

```bash
uv run --no-project --python 3.12 --with trimesh --with numpy --with manifold3d \
  python hexapod_walker/prototype_sts3215/concepts/tibia_yoke_reinforcement/make_tibia_yoke_reinforcement_concept.py

python hexapod_walker/prototype_sts3215/tools/buildviz_checks.py \
  hexapod_walker/prototype_sts3215/concepts/tibia_yoke_reinforcement/scene.json \
  --pitch 0.25 --emit --json

buildviz push \
  --build-id prototype_sts3215/tibia-yoke-reinforcement-test \
  --scene hexapod_walker/prototype_sts3215/concepts/tibia_yoke_reinforcement/scene.json \
  --design-spec hexapod_walker/prototype_sts3215/concepts/tibia_yoke_reinforcement/design_spec.yaml \
  --name "STS3215 tibia/yoke reinforcement load-path test" \
  --bump --upload-assets \
  --message "focused tibia/yoke load-path reinforcement concept"
```

The chassis reinforcement concept remains a lower-priority sandbox until the
full robot Onshape/contact setup is clean enough to make chassis stress claims.
