# TT-motor kid 4x4 classic pickup truck

This is a small, 3D-printable 4×4 pickup truck for the exact
[DIANN Amazon kit (ASIN B0GY8L28XJ)](https://www.amazon.com/dp/B0GY8L28XJ):
four yellow 1:48 TT motors, four nominal 65 mm wheels, and two red L298N
driver boards.  A removable bright pickup body adds a square cab, open cargo
bed, hood, grille, headlights, bumpers, and wheel arches while leaving the
proven drivetrain geometry unchanged.

The assembled tire envelope is about **182 × 183 mm**, with a **116 mm
wheelbase**, **153 mm track width**, and about **18.6 mm ground clearance**.
The lower chassis, removable electronics deck, three-piece pickup body, and
four identical yellow motor clamps all fit on a 256 × 256 mm printer bed.

## Why this mounting scheme

TT motors are standard in broad outline but clone makers vary the little
molded screw bosses.  The chassis therefore uses a shallow locating pocket
and a screw-down external clamp over each 22.5 mm-wide gearbox.  It does not
depend on the motor's small holes.  Eight M3 screws install all four motors,
from above, with no loose nuts and no need to flip a half-assembled vehicle.

The two L298N boards use the common **43 × 43 mm** PCB and **36.6 mm square**
mounting pattern.  Print `l298n_fit_gauge.stl` first; its four 2.5 mm pins
verify the board pattern in a few minutes.  `motor_fit_coupon.stl` similarly
checks one actual motor with the production `motor_clamp.stl`.

## Source geometry

| Part | Geometry used | Source |
|---|---:|---|
| Wheel | 66.03 mm drawing OD; 30.0 mm tread width (sold as 65 × 26 mm nominal) | Amazon listing image and description |
| TT motor | 64.2 mm dimensioned body, 22.5 mm gearbox width/height, 36.8 mm axle span; 70 × 22 × 18 mm advertised envelope | [Adafruit TT motor #3777](https://www.adafruit.com/product/3777) and Amazon listing drawing |
| L298N module | 43 × 43 × 28.6 mm; 4 × Ø3.0 holes on a 36.6 mm square | [Make Electronics L298N Type 2](https://make.net.za/product/9me3001/) |

The visual models in `out/stl/*_DO_NOT_PRINT.stl` are conservative envelopes,
not replacement motors, wheels, or electronics.

## Generate and inspect

All Python commands use `uv` per the repository convention.

```sh
make -C vehicles/tt_kid_truck build          # STEP, STL, assembly STEP, scene, checks
make -C vehicles/tt_kid_truck check          # re-check generated meshes and recorded dimensions
make -C vehicles/tt_kid_truck view-buildviz  # publish to :5183; optionally mirror to cloud
make -C vehicles/tt_kid_truck pack           # export a printer-ready 3MF plate set
```

Primary outputs after `make build`:

- `out/stl/chassis.stl` — print one, flat side down
- `out/stl/electronics_deck.stl` — print one, flat side down
- `out/stl/truck_body_lower.stl` — print one, deck tabs down
- `out/stl/truck_hood.stl` — print one, broad flat face down
- `out/stl/truck_roof.stl` — print one, broad flat face down
- `out/stl/motor_clamp.stl` — print four, flat side down
- `out/stl/motor_fit_coupon.stl` — optional quick fit test
- `out/stl/l298n_fit_gauge.stl` — optional quick board-pattern test
- `out/step/tt_kid_truck_assembly.step` — complete editable assembly
- `out/scene.json` — full assembled BuildViz model
- `out/design_report.json` — dimensions, quantities, mesh stats, and checks

Suggested FDM settings: 0.20 mm layers, 4 perimeters, 5 top/bottom layers,
25–35% infill, and PLA or PETG.  The drivetrain parts and hood print without
supports; use build-plate-only supports beneath the lower body's wheel arches
and window headers.  Print the roof broad face down and keep the clamps in the
supplied orientation so their layers run across the clamp width.

## Important electrical boundary

The red L298N boards are **motor drivers**, not a radio receiver or complete
controller.  A working car still needs a switched battery holder and a small
controller/receiver.  The motors are rated for 3–6 V; do not apply the L298N
module's advertised 35 V maximum to them.  Disconnect the battery while a
child changes wiring, cover exposed battery contacts, and have an adult make
the first powered test with the wheels raised off the table.

See [BOM.md](BOM.md) and [ASSEMBLY.md](ASSEMBLY.md) for the exact hardware and
the child-friendly assembly order.
