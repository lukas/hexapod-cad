# AK40 Hexapod BOM

This is the controlled bill of materials for **one complete AK40
quasi-direct-drive hexapod**. The design is built around **CubeMars
AK40-10 (KV170) actuators** — 24 V, 10:1 planetary, integrated FOC
driver + absolute encoder, CAN bus, MIT torque mode — with every link
bolted **directly to the actuator's output flange** (the actuator's own
bearings carry the joint; there are no horns, no external bearings, no
clamp caps anywhere in this build). Do not substitute other actuator
models unless you are ready to measure them and regenerate every
printed bracket.

> **Design C (Aug 2026): build-audited.** The 18 actuators are in hand
> and the printable STL set exists in `stl_prototype/`, generated from
> the official CubeMars 2D drawing (AK40-10 V3.0, 2026/6/12): output
> flange 3× M2.5×3 on Ø27 + Ø15 pilot, front case 3× M2.5×5 on Ø47.5,
> rear case 4× M2.5×5 on Ø47 + Ø37×1 boss, power+CAN on one side
> XT30PW(2+2), UART A1257WR-S-3P. All **144 fasteners are instanced in
> the BuildViz scene**, the attachment graph is machine-checked (every
> part reachable from the chassis), and a 214-pose swept overlap check
> across all joint limits is clean.
> **Verify with calipers on one real actuator before printing six leg
> sets** — especially (a) whether the Ø15 pilot is a bore (assumed; the
> printed parts grow a 14.85 boss) or a boss, (b) whether the rotating
> output flange sits proud of or flush with the fixed front ring (hubs
> carry a 0.6 mm relief annulus outside r16 in case it is flush), and
> (c) the XT30PW plug's exact azimuth — all 18 plugs must be clocked
> inboard/up-leg per the scene or they foul the printed parts.

Links are stable Amazon search links rather than one-off ASINs, because
Amazon listings churn. Pick a well-reviewed Prime listing that matches
the spec exactly.

## Already Owned

| Qty | Item | Notes |
|---:|---|---|
| 18 | CubeMars AK40-10 KV170 | Purchased Aug 2026. 3 per leg × 6 legs. No spares — treat the bring-up torque clamps seriously; a replacement is ~$230 and weeks of lead time. Confirm on arrival: original (Ø53×37, single encoder) vs V3.0 (Ø53×40.2, dual encoder, position retained across power loss). CAD assumes the V3.0 envelope so either fits; avoid mixing revisions on one robot. Each ships with one power cable and one CAN daisy cable (XT30PW 2+2). |

## Required Purchases

### Electronics

| Qty | Item | Spec to buy | Link |
|---:|---|---|---|
| 1 | Raspberry Pi 5, 8 GB | Runs the 50 Hz policy loop + 3 socketcan buses. Add the official **active cooler** (the FOC telemetry logger pins a core). | [Amazon: Raspberry Pi 5 8GB](https://www.amazon.com/s?k=raspberry+pi+5+8gb) |
| 1 | Raspberry Pi 5 active cooler | Official blower + heatsink. | [Amazon: Raspberry Pi 5 active cooler](https://www.amazon.com/s?k=raspberry+pi+5+active+cooler) |
| 1 | microSD card | 64 GB, A2 rated. | [Amazon: 64GB A2 microSD card](https://www.amazon.com/s?k=64GB+A2+microSD+card) |
| 4 | CANable 2.0 USB-CAN adapter | **CANable 2.0 (OpenLight Labs or MKS clone) flashed with candleLight firmware** → Linux socketcan `gs_usb`, zero driver drama. 3 used (one 1 Mbps bus per leg pair, 6 actuators each) + 1 bench spare — the single most annoying part to be short of. | [Amazon: CANable 2.0 USB CAN adapter](https://www.amazon.com/s?k=CANable+2.0+USB+CAN+adapter) |
| 1 | 24→5 V buck, 5 A | **Pololu D24V50F5-class** (or potted automotive equivalent): 6S rail → 5 V/5 A into the Pi 5 header. Taps **upstream of the loop key** so compute + telemetry survive an e-stop. Not a bare LM2596 board. | [Amazon: 24v to 5v 5a buck converter](https://www.amazon.com/s?k=24v+to+5v+5a+buck+converter) |
| 1 | MPU-6050 IMU (GY-521) | 6-DOF gyro + accel, I²C to the Pi. Same part + mounting habit as the STS3215 robot. | [Amazon: MPU-6050 GY-521 module](https://www.amazon.com/s?k=MPU-6050+GY-521+module) |
| 1 | 1.25 mm-pitch 3-pin pigtails | Mates the actuators' A1257WR-S-3P UART config port (firmware/ID setup). A 10-pack of pre-crimped 1.25 mm (PicoBlade-class) pigtails. | [Amazon: 1.25mm 3 pin connector pigtail](https://www.amazon.com/s?k=1.25mm+3+pin+picoblade+pigtail) |

### Power

| Qty | Item | Spec to buy | Link |
|---:|---|---|---|
| 1 | LiPo battery | **6S 5000 mAh, 50C+, XT90** (~155 × 48 × 55 mm class, ~720 g). 22.2 V nominal feeds the motor rail raw; 25.2 V full is standard on 24 V-rated CubeMars. Velcro'd under the belly — the CAD's strap slots assume this footprint; **keep the pack inside r ≈ 95 mm of centre** or the swinging hip actuators will find it. A second pack later doubles run time — wire the trunk for it, don't buy it yet. | [Amazon: 6S 5000mAh lipo 50C XT90](https://www.amazon.com/s?k=6S+5000mAh+lipo+50C+XT90) |
| 1 | LiPo charger, 6S capable | The house 3S charger doesn't cover this. SkyRC B6neo-class or better, ideally 200 W so a 5000 mAh pack charges in ~1 h. | [Amazon: 6S lipo balance charger 200W](https://www.amazon.com/s?k=6S+lipo+balance+charger+200W) |
| 1 | LiPo safety bag | Sized for the 6S pack. | [Amazon: lipo safety bag fireproof large](https://www.amazon.com/s?k=lipo+safety+bag+fireproof+large) |
| 2 | XT90-S anti-spark pair | **Wired as a LOOP KEY on the motor rail — this is the e-stop.** Pull = all 18 actuators dead, Pi stays up. One spare pair; the key gets pulled a lot. | [Amazon: XT90-S anti spark connector](https://www.amazon.com/s?k=XT90S+anti+spark+connector) |
| 4 | XT90 pigtails | Male/female, 10–12 AWG silicone: battery trunk, fuse leg, loop-key leg. | [Amazon: XT90 pigtail 12awg](https://www.amazon.com/s?k=XT90+pigtail+12awg) |
| 1 | Main fuse + holder | **40 A MAXI blade** + inline 10–12 AWG holder, first thing after the battery. ~2× realistic worst-case concurrent draw; it exists for a hard chassis short. | [Amazon: MAXI blade fuse holder 10 awg 40A](https://www.amazon.com/s?k=maxi+blade+fuse+holder+10+awg) |
| 1 | XT30 connector kit | ~20 pairs + heat shrink: per-leg power branches into the actuators' XT30PW daisy chains. | [Amazon: XT30 connector kit](https://www.amazon.com/s?k=XT30+connector+male+female+kit) |
| 1 | 12 AWG silicone wire | Red + black, ~3 m each: trunk and bus-bar runs. | [Amazon: 12 awg silicone wire red black](https://www.amazon.com/s?k=12+awg+silicone+wire+red+black) |
| 1 | 20 AWG twisted pair | ~10 m for CAN-H/L runs (one bus per leg pair, daisy-chained). | [Amazon: 20 awg twisted pair wire](https://www.amazon.com/s?k=20+awg+twisted+pair+wire) |
| 6 | 120 Ω resistors | CAN termination, both ends of each of the 3 buses. **Check first** whether the AK40 harness / CANable already terminate — never more than 2 per bus. | [Amazon: 120 ohm resistor 1/4w](https://www.amazon.com/s?k=120+ohm+resistor+quarter+watt) |
| 1 | Wago 221 lever-nuts | Assortment (~20): motor-rail trunk → 6 leg-branch splices + the 5 V distribution, per house power-distribution tradition. | [Amazon: Wago 221 lever nuts assortment](https://www.amazon.com/s?k=Wago+221+lever+nut) |

### Structure + fasteners

| Qty | Item | Spec to buy | Link |
|---:|---|---|---|
| 2 | Carbon-fibre tube, Ø12 mm | **Ø12 OD × Ø10 ID roll-wrapped**, 2 × 1 m: six 102 mm tibia cuts + spares. | [Amazon: 12mm carbon fiber tube 10mm ID](https://www.amazon.com/s?k=12mm+carbon+fiber+tube+10mm+id) |
| 1 | M2.5 × 6 SHCS, 100-pack | **54 used**: output-flange bolts, 3 per joint hub (coxa/femur/tibia hubs × 6 legs), threading into the actuators' M2.5×3 flange taps through counterbored printed hubs. | [Amazon: M2.5 6mm socket head screw 100](https://www.amazon.com/s?k=M2.5+6mm+socket+head+cap+screw) |
| 1 | M2.5 × 8 SHCS, 100-pack | **66 used**: 18 chassis→yaw front-case bolts (3×6, from below the bottom plate) + 24 coxa-wall→hip rear-case + 24 femur→knee rear-case. All thread into the actuators' M2.5×5 case taps. | [Amazon: M2.5 8mm socket head screw 100](https://www.amazon.com/s?k=M2.5+8mm+socket+head+cap+screw) |
| 1 | Threadlocker, blue (243) | Every M2.5 goes into aluminum case threads on a vibrating robot. Blue, never red. | [Amazon: loctite 243 blue threadlocker](https://www.amazon.com/s?k=loctite+243+blue+threadlocker) |
| 4 | M3 × 50 F-F brass standoffs | Chassis plate columns across the 50 mm gap at (±40, ±40); M3 × 8 SHCS each end. | [Amazon: M3 50mm standoff female brass](https://www.amazon.com/s?k=M3+50mm+standoffs+female+female+brass) |
| 1 | M3 × 8 SHCS + assortment | 8 used on the standoffs + electronics mounting; buy the usual M3 kit. | [Amazon: M3 stainless screw kit assortment](https://www.amazon.com/s?k=M3+stainless+screw+kit+assortment) |
| 4 | M2.5 × 12 SHCS + nyloc | Pi 5 mount: up through the top plate (Ø2.7 holes on the 58 × 49 pattern) into nylon spacers under the Pi. Comes in the M2.5 kit. | [Amazon: M2.5 12mm socket head screw](https://www.amazon.com/s?k=M2.5+12mm+socket+head+cap+screw) |
| 12 | Ø2.5 mm roll pins | Tibia tube retention, 2 per leg through the printed cross-holes (no drilling). Assortment box. | [Amazon: 2.5mm spring roll pin assortment](https://www.amazon.com/s?k=2.5mm+spring+roll+pin+assortment) |
| 1 | Two-part epoxy | Slow-cure (30 min) structural, tibia tube sockets. | [Amazon: 30 minute structural epoxy](https://www.amazon.com/s?k=30+minute+two+part+epoxy) |
| 2 kg | PETG (or PETG-CF) filament | 1.75 mm, all structural parts. **Not PLA** — park loads on a 5.8 kg robot creep PLA. | [Amazon: PETG filament 1.75mm 1kg](https://www.amazon.com/s?k=PETG+filament+1.75mm+1kg) |
| 250 g | TPU 95A filament | Foot boots (Ø11.7 bore, 0.3 mm interference on the tube). | [Amazon: TPU 95A filament 1.75mm](https://www.amazon.com/s?k=TPU+95A+filament+1.75mm) |
| 1 | Industrial hook-and-loop | 3M Dual Lock for the under-belly pack + cinch strap through the printed slot pairs. | [Amazon: 3M dual lock heavy duty](https://www.amazon.com/s?k=3M+dual+lock+heavy+duty) |
| 1 | Heat-shrink kit + zip ties | Assorted shrink; 100-pack 3 mm ties for the per-leg harness drops. | [Amazon: heat shrink tubing assorted](https://www.amazon.com/s?k=heat+shrink+tubing+assorted) |

## Strongly Recommended Tools

| Qty | Item | Why | Link |
|---:|---|---|---|
| 1 | Digital calipers | Verify the drawing-derived AK40 interface dims on a real unit **before** printing six leg sets. | [Amazon: digital calipers](https://www.amazon.com/s?k=digital+calipers) |
| 1 | Bench PSU, 24 V / 10 A+ | Single-actuator bring-up without a charged LiPo in the loop. **Caveat:** a bare PSU can't absorb regen — keep decelerations gentle or parallel a partly-charged pack. | [Amazon: bench power supply 30V 10A](https://www.amazon.com/s?k=bench+power+supply+30v+10a) |
| 1 | Crimper for XT30/XT90 | 18 actuators of harness; crimp, don't solder-and-pray. | [Amazon: xt60 xt30 connector crimper](https://www.amazon.com/s?k=xt60+xt30+connector+crimper) |
| 1 | Metric hex key set | 2 mm (M2.5) and 2.5 mm (M3) do the whole robot. | [Amazon: metric ball end hex key set](https://www.amazon.com/s?k=metric+ball+end+hex+key+set) |

## Printed Parts

Files in `hexapod_walker/prototype_ak40/stl_prototype/`, regenerated by
`make build` (`--check` verifies watertightness). Solid masses from the
CAD; real prints land ~55–70% of solid. Settings for a Bambu A1 /
Ender-class FDM printer.

| Qty | STL | Material | Layer | Infill | Walls | Solid mass | Notes |
|---:|---|---|---|---:|---:|---:|---|
| 1 | `chassis_bottom.stl` | PETG | 0.2 mm | 30% gyroid | 5 | 343 g | 260 f2f hex, 5 mm. Ø40 yaw wells (was Ø44 — the Ø47.5-PCD counterbores broke through) + counterbored M2.5 ring per leg (screws enter from BELOW), harness slots, battery strap slots, XT90 pass-through. Prints flat, no supports. |
| 1 | `chassis_top.stl` | PETG | 0.2 mm | 25% gyroid | 4 | 272 g | 260 f2f hex, 4 mm. Ø80 centre access, standoff square, Pi 5 hole pattern east of centre. |
| 6 | `coxa_link.stl` | PETG | 0.2 mm | 40% gyroid | 5 | 71 g | Yaw hub (Ø36, turns inside the Ø40 well, 0.6 mm flange-relief face) + arm + hip rear-mount wall with Ø37 boss recess + side gussets. 4× Ø7 screw tunnels through the arm (3 yaw-flange + the 330° case screw) — required to insert those screws at all. Print wall-face down; supports under the hub. |
| 6 | `femur_link.stl` | PETG | 0.2 mm | 40% gyroid | 5 | 39 g | Flat plate: hip-flange hub (pilot boss + 3 cb holes) → knee rear mount (Ø37 recess + 4 cb holes). Prints flat on the +y face, no supports. |
| 6 | `tibia_yoke.stl` | PETG | 0.2 mm | 40% gyroid | 5 | 17 g | Knee-flange hub + web + Ø12 tube socket (30 mm bore, 2 roll-pin cross-holes). Print socket-down. |
| 6 | `foot_boot.stl` | TPU 95A (required) | 0.25 mm | 100% | 3 | 18 g | Pressed over the tube end, Ø11.7 bore = 0.3 mm interference. Tip-down, no supports. |

For a **one-leg test** (do this before committing six sets): 1×
`coxa_link` + `femur_link` + `tibia_yoke` + `foot_boot`, plus one 102 mm
tube cut.

## Bench Test Order

1. Wire battery → 40 A fuse → loop key → **one** actuator (or bench PSU
   with the regen caveat). Verify the loop key kills it.
2. One CANable: `ip link set can0 up type can bitrate 1000000`,
   `candump can0` — confirm telemetry, set the actuator ID over the
   UART/config tool.
3. MIT-mode wiggle with a **0.3 N·m torque clamp**, unloaded. Verify
   direction convention, zeroing, temperature telemetry.
4. **Calipers on the drawing dims** (flange PCD 27 / pilot Ø15
   bore-or-boss / front Ø47.5 / rear Ø47 + Ø37 boss). Fix any deltas in
   `hexapod_ak40.py`, `make build`, then print the one-leg set.
5. Assemble one leg; air moves, then a touchdown-detection test
   (current spike on contact).
6. Repeat × 6, chassis, first supervised stand — operator present, hand
   on the loop key, house safety rules in force.

## Rough Cost

(Actuators excluded — already owned; ~$4,100 of motor is on the shelf,
which is the argument for the torque clamps above.)

| Bucket | Estimate |
|---|---:|
| Pi 5 + cooler + microSD + 4× CANable + buck + IMU | $200–$270 |
| Battery + 6S charger + safety bag | $150–$220 |
| Power distribution (fuse, XT90/XT30, loop key, wire, Wagos) | $60–$100 |
| CF tube + epoxy + filament (PETG + TPU) | $80–$120 |
| Fasteners + threadlocker + standoffs + velcro + consumables | $50–$80 |
| Bench PSU + crimper (if not owned) | $80–$150 |
| **Total** | **~$620–$940** |
