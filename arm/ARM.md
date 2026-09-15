# Optional 5-DOF Arm Add-On

This is an **optional** anthropomorphic arm + parallel-jaw gripper that
bolts on top of the hexapod's chassis using the 4 centre M3 holes that
otherwise hold the electronics tray.

The arm lives at `prototype_sts3215/arm/`.  It is **off by default**:

* `./run.sh hexapod_walker/prototype_sts3215/build_all.py` -- builds the
  hexapod prototype as before, ignores this directory.
* `./run.sh hexapod_walker/prototype_sts3215/build_all.py --with-arm` --
  additionally regenerates `arm/stl_arm/` printables and an arm
  assembly preview under `arm/stl_reference/`.
* `./run.sh hexapod_walker/prototype_sts3215/_verify_prototype.py --with-arm`
  -- additionally runs the arm-vs-chassis / arm-vs-leg interference
  check (`[4b]`) and the flimsy-joint check on the 5 new arm parts
  (`[6b]`).

The arm is otherwise fully self-contained: nothing in `prototype_sts3215/`
imports anything from this directory unless the `--with-arm` flag is
set, so a hexapod build with no arm pays nothing for these files.

The arm re-uses the leg's parametric geometry (DS3225 servo well,
24 mm horn PCD, 49.5 × 10 mm tab pattern) so a single bag of M3
hardware works for the entire robot.  Only **5 genuinely new printable
parts** are introduced; the J2 / J3 / J4 kinematic chain is built out
of three re-exported leg parts.

## Joint table

| Joint | Role            | Servo            | Axis     | Travel    | Cradle      |
|-------|-----------------|------------------|----------|-----------|-------------|
| J1    | base yaw        | DS3225 25 kg·cm  | +Z       | ±90°      | `arm_base_bracket`   |
| J2    | shoulder pitch  | DS3225 25 kg·cm  | +Y       | -90°..+90°| `arm_shoulder_link` (= leg coxa_link) |
| J3    | elbow pitch     | DS3225 25 kg·cm  | +Y       | -10°..+120° | `arm_upper` (= leg femur_link) |
| J4    | wrist pitch     | DS3225 25 kg·cm  | +Y       | -90°..+90°| `gripper_base` (J4 cradle inside) |
| J5    | gripper         | Tower Pro MG90S  | +Y       | -45°..+45°| `gripper_base` (J5 cradle inside) |

All four pitch axes (J2, J3, J4) stay parallel — the arm is a planar
chain rotated about a single base yaw (J1).  Reach is straight-out
**220 mm** from the J1 axis to the wrist root (= `FEMUR_LENGTH` +
`TIBIA_LENGTH` = 90 + 130 mm), and another ~ 90 mm from the wrist
root to the gripper tip in a fully-extended wrist pose.  In the neutral
"ready to pick" pose (J2 = -25°, J3 = +60°, J4 = +25°) the gripper tip
sits at roughly **(+129, 0, +5) mm** from the J1 axis — just past the
chassis edge, at the chassis-top elevation.

## Bolt-on instructions

1. **Remove the electronics tray** (the flat PLA plate that lives in
   the centre of the chassis top, bolted to the 4 centre M3 holes on
   the 35 mm radius).  Re-mount it elsewhere (e.g. underneath the
   chassis bottom plate, or to one of the empty side-edge regions) or
   leave it for later if you're feeling brave with wire-routing.
2. **Bolt `arm_base_bracket.stl` onto the chassis top.**  Use 4 M3 x
   16 mm cap screws + nylock nuts from underneath, through the same
   35-mm radius 4-hole pattern that the electronics tray used.  No
   chassis modification or extra cutouts are required: the J1 servo
   well sits entirely **above** the chassis top plate.
3. **Drop the J1 DS3225 servo into the base bracket's well from
   above** until its mounting tabs land on the well rim, then drive
   4 servo self-tappers down through the tabs into the bracket pilot
   holes.  Route the servo's 3-wire harness out of the L-shaped wire
   exit slot at the -Y / -Z corner of the well.
4. **Seat the J1 disc horn and bolt `arm_shoulder_link.stl` onto it.**
   Press the 20 mm aluminum 25T disc horn (Amazon B07D56FVK5) onto the
   J1 spline and secure it with the servo's M2.5 centre screw, then bolt
   the shoulder link's hub onto the disc's flat top with 4 M3 x 6 SHCS
   on the 14 mm bolt circle (4 holes at 0/90/180/270 deg) threading into
   the disc's M3 tapped holes.  The shoulder link is a literal re-export
   of the leg's `coxa_link` — its hub mates to the disc horn and its
   outboard cradle takes the J2 DS3225 servo.  (No printed adapter — the
   aluminum disc replaces the retired `servo_horn_adapter.stl`.)
5. **Repeat the cradle / disc-horn pattern** for J2 → `arm_upper`,
   J3 → `arm_forearm`, J4 → `gripper_base`.  At each joint a 20 mm
   aluminum disc horn seats on the servo spline and the next link's
   4-bolt hub bolts directly onto it (4 M3 x 6 SHCS, 14 mm circle).
6. **Press `wrist_adapter.stl`** into the forearm's foot socket bore
   (FOOT_HUB_OD = 12 mm friction fit; you can secure it with a tiny
   drop of CA glue or by drilling a 1 mm cross-pin through the
   socket).
7. **Bolt `gripper_base.stl`** to the underside of the wrist adapter
   using its 4 disc-horn-PCD M3 holes (engaged with the J4 disc horn
   on the gripper side).  When J4 actuates, the entire gripper base
   pivots about the wrist axis relative to the (forearm-fixed) wrist
   adapter.
8. **Install J5 (MG90S)** in the gripper-base cavity from the +X side
   (output shaft exits the +Y face).  Two M2 / M2.5 self-tappers per
   side fix it to the gripper-base pilots.
9. **Pivot in the two jaws** on a single M2.5 cross-pin through the
   front of the gripper base.  Two small 1.5 mm steel link rods + M2
   pins connect each jaw's pivot lever to the J5 horn, forming the
   parallel-jaw 4-bar.

## Parts list

| File                       | Type                 | Qty | Notes |
|----------------------------|----------------------|-----|-------|
| `arm_base_bracket.stl`     | NEW                  | 1   | bolts to chassis top centre |
| `arm_shoulder_link.stl`    | re-export coxa_link  | 1   | = leg coxa_link |
| `arm_upper.stl`            | re-export femur_link | 1   | = leg femur_link |
| `arm_forearm.stl`          | re-export tibia_link | 1   | = leg tibia_link |
| `wrist_adapter.stl`        | NEW                  | 1   | foot-socket plug → 4-bolt horn pattern |
| `gripper_base.stl`         | NEW                  | 1   | J4 + J5 cradles + jaw pivots |
| `gripper_jaw_left.stl`     | NEW                  | 1   |       |
| `gripper_jaw_right.stl`    | NEW                  | 1   | mirror of jaw_left |

Plus **4 × 20 mm aluminum 25T disc horns** (Amazon B07D56FVK5, one per
arm joint J1-J4) — the same disc the legs use, bolted with 4 M3 × 6
SHCS each. No printed `servo_horn_adapter` is needed any more.

## Print order + filament estimate

Recommended print order (small to large; lets you debug the
smaller parts before committing to longer prints):

1. `gripper_jaw_left.stl`, `gripper_jaw_right.stl`  (~ 5 g each, 20 min)
2. `wrist_adapter.stl`                              (~ 7 g, 30 min)
3. `gripper_base.stl`                               (~ 28 g, 1 h 45 min)
4. `arm_base_bracket.stl`                           (~ 32 g, 2 h 00 min)
5. `arm_shoulder_link.stl`                          (~ 38 g, 2 h 15 min)
6. `arm_upper.stl`                                  (~ 48 g, 2 h 30 min)
7. `arm_forearm.stl`                                (~ 36 g, 2 h 10 min)

Total: ~ **200 g of PLA / PETG** + ~ 11 hours of print time on an
Ender 3 / Bambu P1S-class FDM machine at 0.2 mm layer height.

Estimated mass of the assembled arm including all servos + hardware:
**~ 560 g** (4 × DS3225 ~ 60 g each = 240 g, 1 × MG90S = 9 g, ~ 200 g
of printed plastic, ~ 100 g of M3 hardware + cabling).

## Wiring / electronics notes

The arm adds **5 servos** (4 × DS3225 + 1 × MG90S) to the 18 already
on the legs, for a total of **23 PWM channels**.  Two PCA9685 boards
(16 channels each) cover this with room to spare — the arm's 5
channels typically go on the SECOND PCA9685 (the first is fully
booked by the legs).

Power:

* J1 - J4 (DS3225 ×4):  ~ 2.5 A average / ~ 8 A stall — same 5-7.4 V
  rail as the leg servos.
* J5 (MG90S):           ~ 0.2 A average — can share the rail or run
  off the Arduino's 5 V if you don't draw too much elsewhere.
* Add ~ 2 A to your existing power budget; an extra 3S 2200 mAh
  pack in parallel covers it.

I²C: the second PCA9685 lives at a different I²C address (default
0x40 vs 0x41 — change one board's address jumper) and daisy-chains
off the first via the I²C pass-through pads.

A starter Arduino sketch that exposes J1-J5 over the same serial
protocol as the legs (`A1+45 J2-10 ... \n`) lives in
`prototype/firmware/prototype_servo_bridge/` — extend the channel
map to include the arm's PCA9685 outputs.

## Workspace

* Maximum 2D reach from J1 axis to gripper tip: **~ 267 mm**
* Reach DOWN below chassis level: with J2 = -90° (shoulder hanging
  straight down) and J3 = J4 = 0° the gripper tip drops ~ 240 mm
  below the chassis top, well past the standing height (~ 117 mm)
  so the gripper can grasp objects on the floor when the hexapod
  is in stance.
* Neutral pose tip: (+129, 0, +5) mm from chassis centre — gripper
  hovers just past the chassis edge at chassis-top elevation.

## Files

```
prototype_sts3215/arm/
├── ARM.md                          this file
├── arm.py                          part-generator (run via ./run.sh)
├── render_arm_preview.py           PyVista preview render
├── integrate.py                    opt-in glue for build_all.py /
│                                   _verify_prototype.py --with-arm
├── stl_arm/                        printable STLs
│   ├── arm_base_bracket.stl
│   ├── arm_shoulder_link.stl
│   ├── arm_upper.stl
│   ├── arm_forearm.stl
│   ├── wrist_adapter.stl
│   ├── gripper_base.stl
│   ├── gripper_jaw_left.stl
│   └── gripper_jaw_right.stl
├── stl_reference/                  not for printing
│   └── arm_assembly_preview_DO_NOT_PRINT.stl
└── renders/
    └── arm_assembly_preview.png
```

## Regenerate

```
# Just the arm STLs / preview:
./run.sh hexapod_walker/prototype_sts3215/arm/arm.py                # rebuild STLs
./run.sh hexapod_walker/prototype_sts3215/arm/render_arm_preview.py # rebuild PNG

# Or, opt in via build_all to rebuild prototype + arm:
./run.sh hexapod_walker/prototype_sts3215/build_all.py --with-arm

# And run the optional-arm verification suite:
./run.sh hexapod_walker/prototype_sts3215/_verify_prototype.py --with-arm
```
