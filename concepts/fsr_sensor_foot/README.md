# Angle-tolerant RP-C10 sensor foot

This sidecar replaces the STS3215 prototype's press-on boot with a guided
contact-sensing foot for the MakerHawk RP-C10-ST thin-film FSR (Amazon ASIN
`B0CZ6L5NMM`). It preserves the existing Ø8 × 142 mm carbon tibia and the
150 mm knee-to-ground-tip station.

The original shallow flexible flange was not adequate for the robot's normal
stance: the tibia is about 40° from the ground normal, so the housing could
edge-load or jam before the center puck loaded the FSR. This version separates
the jobs:

- a 1.0 mm TPU spherical skin provides grip and snaps onto the carriage with
  a captive 0.50 mm radial undercut;
- a rigid spherical carriage supports the contact patch;
- three 6.7 mm-long guides prevent the shoe from rocking sideways;
- a 7.5 mm disk loads the flat FSR after 0.10 mm nominal approach;
- a three-spoke TPU spring absorbs the next 0.15 mm and returns the spreader;
- a broad rigid shoulder stops the carriage at 0.25 mm and bypasses impact.

The supported tread is designed through 50°, giving 10° of margin beyond the
normal 40° planted pose. The visible surface continues to 54° as an edge lip.
The flat sensor tail still has a preferred direction: install its lobe toward
the chassis/uphill side of each tibia.

## Generate

```sh
cd /Users/lukas/hexapod
uv run --no-project --python 3.12 \
  --with trimesh --with numpy --with manifold3d \
  python hexapod_walker/prototype_sts3215/concepts/fsr_sensor_foot/make_fsr_sensor_foot.py
```

Printable outputs are in `stl/`; bought, section, load-arrow, and ground meshes
are in `viz/`.

## BuildViz

```sh
cd /Users/lukas/hexapod
npx buildviz push \
  --build-id prototype_sts3215/fsr-sensor-foot \
  --scene hexapod_walker/prototype_sts3215/concepts/fsr_sensor_foot/scene.json \
  --design-spec hexapod_walker/prototype_sts3215/concepts/fsr_sensor_foot/design_spec.yaml \
  --name "STS3215 angle-tolerant guided RP-C10 sensor foot" \
  --bump --upload-assets \
  --message "add captive TPU tread snap and guided sensing carriage"
```

View locally at:

`http://127.0.0.1:5183/?project=prototype_sts3215&build=fsr-sensor-foot`

## Prototype status

The geometry is printable, but the 0.10 mm sensing gap and 0.25 mm stop travel
are deliberately calibration dimensions, not claims about an FDM printer.
Polish/open the guide bores until the empty carriage falls freely, then set the
FSR gap with smooth PET or Kapton shim while observing raw ADC. Test one foot at
0°, 40°, and 50° before installing the rest.
