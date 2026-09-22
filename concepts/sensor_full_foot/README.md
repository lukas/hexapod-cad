# Rounded sensor foot — sockets in bottom (v5)

A 19 mm diameter rounded bottom supports the full 10 mm sensor on a flat
seat. Three 2.4 mm blind sockets surround the sensor with 0.8 mm clearance.
The separate cylinder has a 16 mm bore, 19 mm OD, 20 mm straight height,
wire exit, and three downward 2 mm pins. Pins engage 4.8 mm into the bottom,
with 0.2 mm radial glue clearance and 0.3 mm tip clearance. Minimum outer
wall around the blind sockets is approximately 0.78 mm.

Print both v5 parts: they replace the opposite pin/socket arrangement in v4.
The bottom prints flat sensor face down, dome up; its 2.4 mm socket ends
need short bridges. The cylinder prints unnotched rim down, pins up; sloped
internal supports grow inward from its wall. No external supports expected,
but inspect your slicer and dry-fit before gluing. Physical fit and strength
remain untested. Overall assembled height is 31.7 mm.

Use sensor_full_foot.3mf and mating_cylinder.3mf, or the *_print.stl files.
BuildViz uses these print orientations with assembly transforms. The plain
STL files retain assembly coordinates. sensor_reference.stl is not printable.
Film thickness 0.2 mm and tab length are illustrative; tab width is 6.5 mm.
Reported sensor inner diameter 8.5 mm is not assumed to be a physical hole.
No force applicator or calibrated force-transfer mechanism is included.

Generator checks: watertight connected solids, zero sensor/part interference,
zero assembled interference, and clear intermediate axial insertion positions.

Regenerate from repository root:

```sh
uv run --no-project --with trimesh --with manifold3d python \
  concepts/sensor_full_foot/make_foot.py --layout surround
```

[Cloud BuildViz v5](https://buildviz.cwd1f0-new-cluster.coreweave.app/?build=prototype_sts3215/sensor-full-foot&version=v5)
