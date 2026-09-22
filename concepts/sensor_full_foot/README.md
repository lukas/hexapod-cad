# Ground-contact foot — v8 bench prototype

This replaces the bonded pin/socket joint with guided relative motion.
The goal is ground-contact detection, not calibrated weight measurement.
The nominal 8 mm carbon rod seats on a closed structural web in an 8.2 mm
bore. Rod load goes through that web, a central 6.5 mm TPU pad, the film
sensor, and the rounded foot. The unglued 2 mm pins guide axial motion in
2.4 mm bores. A TPU collar snaps into grooves in both rigid parts to retain
and return them; do not glue the moving joint.

## Parts and sizes

- Rigid bottom: 19 mm diameter, 9.5 mm radius hemispherical underside.
- Rigid cylinder: 16 mm OD, 20 mm height, 8.2 mm rod bore, 17.2 mm socket depth.
- TPU return collar: 22.8 mm maximum OD at its bowed wall; has a wire window.
- TPU contact pad: 6.5 mm diameter, 1 mm thick; attach to the cylinder web.
- Sensor: 10 mm OD, 6.5 mm assumed tab width. The reported 8.5 mm inner
  diameter is assumed to mean active sensing area, not an actual hole.

TPU capability was not confirmed at design time; TPU collar and pad are the
chosen prototype assumption. A soft TPU is the intended starting material;
spring rate, fatigue, creep, snap retention and contact threshold are untested.
The collar is shown in its unloaded shape; no compliant deformation simulation
or load-capacity validation has been performed.

## Contact and overload stop

At rest the outer stop gap is 0.6 mm. Assuming film plus adhesive totals
0.2 mm, the pad starts 0.2 mm above the sensor. It then compresses nominally
0.4 mm before the cylinder rim contacts the foot and limits further motion.
The FSR itself is not intended to compress by 0.4 mm; that deformation is in
the TPU pad. This is a displacement stop, not a validated force limiter.
Changing pad stiffness, film thickness or adhesive changes threshold and load.
The collar also carries a parallel return force; its spring rate must be low
enough for the desired contact threshold. Tilted loading can add guide friction.

Measure the actual sensor and adhesive, then adjust pad thickness with thin
shims. Bench-test raw signal, touch/release, hard-stop contact and oblique loads.
Use separate on/off thresholds in the eventual controller to avoid flicker;
no robot software or hardware was changed here.

## Print and assemble

Use all v8 parts; previous glue-joint prints are incompatible.
Print bottom flat seat down and cylinder unnotched rim down, pins up. Both
rigid print-oriented STLs and 3MFs already have these orientations. The
cylinder requires a bridge over its 8.2 mm rod bore. Print collar in TPU on
its lower cuff, and pad flat in TPU. Inspect collar overhangs in the slicer;
all print settings and support needs require a first-piece trial.

Attach sensor flat without covering active area/vent. Attach pad to cylinder
web. Ensure guides slide without binding. Stretch lower collar cuff into the
bottom groove, fit pins, and stretch upper cuff into cylinder groove. Align
its window with the sensor tab. Rod may be bonded into the cylinder socket;
that bond does not bridge the moving foot joint. Do not glue guides or collar
across the two rigid parts.

The rigid meshes are watertight single solids; seven stroke positions, sensor
and wire keep-outs, pin tip clearance, positive stop, rod fit and rod end-stop
were checked. Print-to-assembly transforms reproduce original vertices.
These geometric checks do not establish a working physical trigger.

Regenerate from repository root:

```sh
uv run --no-project --with trimesh --with manifold3d python \
  concepts/sensor_full_foot/make_foot.py
```

[Cloud BuildViz v8](https://buildviz.cwd1f0-new-cluster.coreweave.app/?build=prototype_sts3215/sensor-full-foot&version=v8)
