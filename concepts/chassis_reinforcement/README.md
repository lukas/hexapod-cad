# STS3215 chassis reinforcement test

BuildViz-only concept for removable chassis reinforcement. This is not
production CAD.

Generate the scene:

```sh
uv run --no-project --python 3.12 --with trimesh --with numpy \
  python hexapod_walker/prototype_sts3215/concepts/chassis_reinforcement/make_chassis_reinforcement_concept.py
```
Push to BuildViz:

```sh
buildviz push \
  --build-id prototype_sts3215/chassis-reinforcement-test \
  --scene hexapod_walker/prototype_sts3215/concepts/chassis_reinforcement/scene.json \
  --design-spec hexapod_walker/prototype_sts3215/concepts/chassis_reinforcement/design_spec.yaml \
  --name "STS3215 chassis reinforcement test kit" \
  --bump --upload-assets \
  --message "removable chassis/yaw-pocket reinforcement concept from MuJoCo rise/walk loads"
```
