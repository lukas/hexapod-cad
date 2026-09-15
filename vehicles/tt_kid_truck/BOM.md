# Bill of materials

## From the purchased DIANN kit

| Qty | Item | Used here |
|---:|---|---|
| 4 | Yellow 3–6 V, 1:48 TT gearmotor with leads | One motor per wheel |
| 4 | Nominal 65 mm press-fit wheel | Four-wheel drive |
| 2 | Red L298N/HW-095 dual H-bridge board | One bridge channel per motor |
| as needed | Dupont jumper wires | Low-current logic wiring only |

## 3D-printed parts

| Qty | File | Notes |
|---:|---|---|
| 1 | `out/stl/chassis.stl` | Lower frame, motor seats, bumpers, deck posts |
| 1 | `out/stl/electronics_deck.stl` | Two L298N mounts plus 44 × 68 mm battery zone and body bosses |
| 1 | `out/stl/truck_body_lower.stl` | Pickup bed, cab/window frame, wheel arches, grille and bumpers |
| 1 | `out/stl/truck_hood.stl` | Removable flat-printing hood |
| 1 | `out/stl/truck_roof.stl` | Removable flat-printing cab roof |
| 4 | `out/stl/motor_clamp.stl` | Identical clamps |
| 1 optional | `out/stl/motor_fit_coupon.stl` | Print before the chassis to verify one motor |
| 1 optional | `out/stl/l298n_fit_gauge.stl` | Print before the deck to verify board holes |

## Fasteners and supplies to add

| Qty | Item | Purpose |
|---:|---|---|
| 12 | M3 × 12 mm Phillips pan-head thread-forming screws for plastic | 8 motor-clamp + 4 deck screws |
| 12 | M3 × 10 mm Phillips pan-head thread-forming screws for plastic | 4 lower-body + 4 hood + 4 roof screws |
| 8 | M2.5 × 8 mm Phillips pan-head screws | L298N boards into printed pilot holes |
| 2 | 10–12 mm-wide hook-and-loop straps, about 200 mm long | Battery holder retention |
| 1 | Switched, enclosed battery holder appropriate to the chosen controller | Not included in the Amazon kit |
| 1 | Small microcontroller, RC receiver, or other control board | L298N is only the power driver |
| 1 | Small Phillips screwdriver | All listed screws use the same tool style |

M3 machine screws can also self-thread into the 2.7 mm printed pilot holes in
PLA/PETG.  Tighten only until snug.  Dedicated thread-forming screws survive
more repeated assembly cycles.

## Electrical notes

- The motors are 3–6 V parts.  The L298N board's much higher maximum input
  rating does not increase the motor rating.
- Each L298N board has two H-bridges.  With two boards, give each of the four
  motors its own bridge channel rather than paralleling two motors on one
  channel.
- The common on-board 5 V regulator generally needs more input headroom than a
  4-cell AA pack provides.  Choose the battery and logic supply with an adult
  who understands the selected controller; this chassis intentionally does
  not prescribe a potentially unsafe battery chemistry.
