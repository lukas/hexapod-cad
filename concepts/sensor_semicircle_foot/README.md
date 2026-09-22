# Flat semicircle sensor foot (superseded draft)

Use [the full rounded foot](../sensor_full_foot/README.md) for the final design. This draft is retained only as design history.

User measurements, September 21, 2026: sensor outside **diameter 10 mm**
(corrected from radius), inner diameter 8.5 mm, top wire tab 6.5 mm,
paper-thin film. Cylinder bore is 10 mm. The user selected a flat half-circle.
The inner diameter is shown as a reference, without assuming it is a hole.
The preview assumes the 6.5 mm tab measurement is its width; drawn tab length
is illustrative only. No tab pocket or film recess depends on its thickness.

The backing is a solid 12 mm diameter semicircle, 2 mm thick. Stick the
lower half of the sensor onto the flat underside, with its center at the
middle of the straight edge and its wire tab pointing out over the open half.
The upper half of the sensor is intentionally unsupported. This is a mounting
prototype, not a validated force-transfer or whole-foot mechanism.

Three 1.9 mm diameter posts extend 5 mm from the rear at 210°, 270°, and
330°. Their outer envelope is 9.6 mm diameter, leaving 0.2 mm radial adhesive
space inside the separately printed 10 mm cylinder bore. Glue the posts to
the cylinder's inner wall. Keep adhesive off the sensing face and wires.
The cylinder is not included; its outer diameter and length remain unspecified.

Print the flat sensor face on the bed, posts pointing upward; no supports.
A 0.2 mm layer height and solid infill are reasonable starting settings.
Trial-fit before gluing; printed bore accuracy has not been physically tested.

Outputs: `sensor_semicircle_foot.stl` and millimeter-unit
`sensor_semicircle_foot.3mf`. The generated mesh is one connected watertight
solid with consistent winding and positive volume. Dimensions and validation
are in `dimensions.json`; `preview.png` shows the rear and sensor placement.

Regenerate from the repository root:

```sh
uv run --no-project --with trimesh --with manifold3d --with matplotlib python \
  concepts/sensor_semicircle_foot/make_foot.py \
  --sensor-od 10 --cylinder-id 10
```
