# Hexapod Prototype BOM

This is the controlled bill of materials for **one complete tabletop
hexapod prototype**. The design is built around **FEETECH STS3215
(ST-3215-C018, 12 V / 30 kg-cm) serial-bus servos** in a **symmetric
disc-horn sandwich joint** (one STS3215 driving a disc horn on the front
spline, a SECOND reused disc horn on the servo's own rear idler boss for
passive support, the yoke bolting identically to both — no external ball
bearing — with an Ø8 mm carbon-fibre tube as the TIBIA segment and the
WHOLE FEMUR printed as one part, `femur_link` — hip yoke + solid Ø14
spar + knee bracket, Jul 2026). Do not
substitute other servo models unless you are ready to measure them and
regenerate the printed brackets/yokes.

> **Jun 2026 refit / Aug 2026 as-built stack.** This supersedes the earlier
> DS3225 + Arduino Mega + 2×PCA9685 + drop-in-cradle design. The STS3215
> are smart serial-bus servos daisy-chained on a single half-duplex TTL
> bus driven from an **Arduino Uno Q** (on-board Linux SoC + MCU) — there
> is **no Arduino Mega, no PCA9685, no Raspberry Pi**.  **No external
> buck, no PDB:** the battery trunk lands on a **central trunk Wago
> splice pair** — two 5-port Wago 221-415 at the chassis_top centre —
> (→ servos via the corner power Wago pairs seated between tray walls
> printed INTO the chassis_bottom top face at the hex corners — the
> whole power distribution is Wago 221 lever nuts)
> and the Uno Q on a separate tap (share ground only).  Electronics deck:
> four posts at `CHASSIS_STANDOFF_HOLES_XY` (±31.1) — 20 mm standoff +
> thumb nut + Ø8×8 mm magnet — hold a round Ø115
> **`round_mount_plate_115`** (Uno Q + breakout on top, 3.3 V Wago
> underneath); **`hex_raised_platform_110`** sits above (screen on
> top; MPU glued on chassis_top beside the central trunk Wagos).  Printed
> `uno_q_tray` / `buck_tray` /
> `spider_carapace` / chassis-top `imu_pad` are retired.  Battery =
> **two 3S 2200 mAh shorty LiPos in parallel, velcro'd UNDER
> `chassis_bottom`'s flat belly** (Aug 2026; the single bay pack is
> retired and `CHASSIS_GAP` shrank 32 → 20 mm); data Wagos hang under
> the belly at the hex vertices.

Links are stable Amazon search links rather than one-off ASINs, because
Amazon listings churn. Pick a well-reviewed Prime listing that matches the
spec exactly.

> This is the single everything-you-need-to-buy-and-print sheet. The old
> separate `SHOPPING_LIST.md` was folded into this file (Aug 2026) — its
> print-settings and print-queue content now lives in
> [Printed Parts](#printed-parts) below.

## Required Purchases

| Qty | Item | Spec to buy | Link |
|---:|---|---|---|
| 20 | FEETECH STS3215 serial-bus servo | **STS3215 (ST-3215-C018), 12 V, 30 kg-cm metal-gear smart serial-bus servo**, standard ~40 x 20 x 40 mm case with a 25T output spline carrying the flush 20 mm disc horn (4×M3 on a Ø14 cross).  The body is bolted to the cradle by 4× M2.5 into the servo's real END-face 10×10 mm hole square (plus the printed strap/clamp), not by output-face screws. Daisy-chained on a 1 Mbps half-duplex TTL bus. Buy all 20 from the same batch. 18 used + 2 spares. | [Amazon: FEETECH STS3215 30kg serial bus servo](https://www.amazon.com/s?k=FEETECH+STS3215+30kg+serial+bus+servo) |
| 1 | Arduino Uno Q | **Arduino Uno Q** (on-board Linux SoC + MCU). Runs Python gait/RL/teleop AND drives the half-duplex STS3215 TTL bus — replaces both the Raspberry Pi and the Mega/PCA stack. Mounts on the round Ø115 `round_mount_plate_115` (with breakout), held by the four magnet posts. Powered directly from the 3S battery (no external buck). | [Arduino Store: Uno Q](https://store.arduino.cc/products/uno-q) |
| 1 | microSD card | 32 GB or 64 GB, A1/A2 rated (Uno Q OS/storage if not using on-board eMMC). | [Amazon: 32GB A1 microSD card](https://www.amazon.com/s?k=32GB+A1+microSD+card) |
| 2 | LiPo battery | **3S 2200 mAh "shorty" pack, 25C+, XT60, ≤ 75 × 34 × 26.5 mm** (Zeee 3S 2200 shorty class — the envelope the CAD reserves; Aug 2026 under-belly rework, replaces the single 138 × 46 × 24 mm bay pack). Two packs wired in **parallel** via an XT60 Y-harness for 4400 mAh; 11.1 V nominal feeds the Wago→servo domain and a separate Uno Q tap. Velcro'd side by side UNDER `chassis_bottom`'s flat belly, block yawed 30°. | [Amazon: Zeee 3S 2200mAh shorty lipo XT60](https://www.amazon.com/s?k=Zeee+3S+2200mAh+lipo+XT60+shorty) |
| 1 | XT60 parallel Y-harness | 1-female-to-2-male XT60 Y (12 AWG) joining the two shorty packs into one trunk feed. | [Amazon: XT60 parallel Y harness 12awg](https://www.amazon.com/s?k=XT60+parallel+Y+harness+12awg) |
| 1 | Industrial hook-and-loop | Heavy-duty adhesive velcro / 3M Dual Lock strips between pack tops and the flat belly (primary retention). | [Amazon: 3M dual lock heavy duty velcro strips](https://www.amazon.com/s?k=3M+dual+lock+heavy+duty) |
| 1 | Velcro straps | Hook-and-loop cinch strap (~15–20 mm wide) through the CENTRE chassis_bottom slot pair (x = −8, y = ±26) wrapping both under-belly packs as the safety strap (the outer 2 slot pairs are spare tie points). | [Amazon: velcro cinch straps 15mm](https://www.amazon.com/s?k=velcro+cinch+straps+15mm) |
| 1 | Bus servo cables | FEETECH 3-pin serial-bus servo cables (servos ship with one each; buy a small spare pack for the chassis-to-first-servo runs). | [Amazon: FEETECH serial bus servo cable](https://www.amazon.com/s?k=FEETECH+serial+bus+servo+cable) |
| 1 | LiPo charger | 3S balance charger, e.g. SkyRC B6 style or better.  Optional: an XT60 parallel-charging board charges both shorty packs in one session (only connect packs within ~0.1 V/cell of each other). | [Amazon: 3S lipo balance charger](https://www.amazon.com/s?k=3S+lipo+balance+charger) |
| 1 | LiPo safety bag | Fire-resistant charging/storage bag. | [Amazon: lipo safety bag fireproof](https://www.amazon.com/s?k=lipo+safety+bag+fireproof) |
| 1 | Anti-spark switch | XT60 RC LiPo anti-spark/on-off switch for servo rail power (also the e-stop). | [Amazon: rc lipo anti-spark switch xt60](https://www.amazon.com/s?k=rc+lipo+anti-spark+switch+xt60) |
| 2 | XT60 pigtails | Male/female XT60 silicone-wire pigtails, 12-14 AWG. | [Amazon: XT60 pigtail 12awg](https://www.amazon.com/s?k=XT60+pigtail+12awg) |
| 1 | Wago 221 lever-nuts | Compact splices — the WHOLE power distribution (as-built Aug 2026: **no PDB**): a central **trunk** V+/GND pair of **5-port 221-415** near the chassis_top centre where the fused battery feed lands and the 6 branches fan out (buy the 2 five-port nuts if your assortment lacks them), **power** injection nuts of **5-port 221-415**, ONE press-fit per hex corner flat between the single-bay tray walls integrated into the chassis_bottom top face (wire entries facing inward; Aug 16 2026 — replaced the V+/GND pairs of 3-port 221-413, and the bay is now 0.75 mm tighter for a press fit; late-Aug 2026 — the separate taped `wago_mount` trays are retired), **data** nuts hung under the belly at the hex vertices (r = 70; moved off the leg azimuths, which the under-belly battery packs occupy), and one more **5-port 221-415** as the **3.3 V rail splice** VHB'd flat under the round mount plate's south rim (feed = Uno Q 3V3 pin through the plate's east Ø8 wire port; loads = MPU VCC + screen VCC, 2 spares).  Distributed per-leg injection is **required** because the 18 servos cannot be power-daisy-chained through the ~3 A Molex 5264 pins (see `firmware/WIRING.md` §6).  Pack of ~20 covers all three groups. | [Amazon: Wago 221 lever nuts assortment](https://www.amazon.com/s?k=Wago+221+lever+nut) |
| 4 | M3 × 20 mm brass standoffs | Magnet-post bases at `CHASSIS_STANDOFF_HOLES_XY` (±31.1) above chassis_top. | [Amazon: M3 20mm standoffs brass](https://www.amazon.com/s?k=M3+20mm+standoffs+brass) |
| 7 | M3 knurled thumb nuts (~2.5 mm) | 4 sit on the 20 mm posts under the magnets; 3 hold the Uno Q's three-point mount under the round plate (finger-tight — late-Aug 2026 review round 2 folded the retired io board's UNO hole pattern into the plate). | [Amazon: M3 knurled thumb nut](https://www.amazon.com/s?k=M3+knurled+thumb+nut) |
| 4 | Ø8 × 8 mm disc magnets | Top of each post; hold the round Ø115 mount plate. | [Amazon: 8x8mm neodymium disc magnets](https://www.amazon.com/s?k=8x8mm+neodymium+disc+magnet) |
| 1 | MPU-6050 IMU (GY-521) | 6-DOF gyro + accel, I²C @ 3V3. Glued on `chassis_top` beside the central trunk Wagos, near the robot centre. | [Amazon: MPU-6050 GY-521 module](https://www.amazon.com/s?k=MPU-6050+GY-521+module) |
| 1 | Main fuse + holder | **20 A** blade fuse + inline holder, between the anti-spark switch and the trunk Wagos; protects the whole servo rail. (Sized by the Jul 2026 MuJoCo stand-up sim — `standup_current_sim.py` / `firmware/WIRING.md` §6.5: ~10 A standing hold, 16.7 A ~50 ms stand-up spike — 20 A never nuisance-blows; 15 A also survives but with less margin.) | [Amazon: inline blade fuse holder 12awg](https://www.amazon.com/s?k=inline+blade+fuse+holder+12awg) |
| 6 | Per-branch fuse + holder (optional) | 5-7 A mini-blade fuse + holder, one per leg branch; catches a sustained single-leg multi-servo stall before it cooks the leg's 16-18 AWG harness. | [Amazon: 5a 7a mini blade fuse holder](https://www.amazon.com/s?k=5a+7a+mini+blade+fuse+holder) |
| 1 | 16-18 AWG silicone wire | Red + black, ~5 m each. The six per-leg V+/GND power branches from the trunk Wagos to each leg's 5264 injection pigtail. | [Amazon: 16 awg silicone wire red black](https://www.amazon.com/s?k=16+awg+silicone+wire+red+black) |
| 1 | VHB / thick foam tape | ~1 mm double-sided mounting tape for the under-belly data Wagos and general electronics mounting (late-Aug 2026: no longer needed for the corner Wago trays — their walls are printed into chassis_bottom). | [Amazon: 3M VHB double sided tape](https://www.amazon.com/s?k=3M+VHB+double+sided+mounting+tape) |
| 1 | Molex 5264 connector + crimp kit | 3-pin Molex 5264 / Mini-SPOX 2.5 mm connector kit (~20 sets) to crimp the per-leg V+/GND injection pigtails and the leg-to-leg **signal+GND-only** data jumpers (V+ pin omitted so power never bridges legs). | [Amazon: molex 5264 2.5mm connector kit crimp](https://www.amazon.com/s?k=molex+5264+2.5mm+connector+kit+crimp) |
| 18 | 20 mm aluminium 25T disc horn | **Ø20 mm aluminium 25T servo disc horn (4× M3 tapped on a Ø14 cross).** Drives every joint (3 per leg × 6 legs = 18). Ships in 10-packs — buy **2 packs**. The PASSIVE rear support on the hip + knee uses the STS3215's own **stock metal passive horn** (ships with the servo; same 4-hole pattern): its centre bore slides over the rear idler boss so it seats flush on the back face, held by one M2.5 screw — no extra purchased horns and no printed adapter (Jul 2026 stock-horn refit). | [Amazon: 10Pcs 25T aluminum servo disc horn (B07D56FVK5)](https://www.amazon.com/s?k=25T+aluminum+servo+disc+horn) |
| 12 | 6805-2RS ball bearing (YAW) | **Ø25 mm bore × Ø37 mm OD × 7 mm wide sealed bearing (also sold as 61805; common bike bottom-bracket stock).** Aug 2026 thick-section swap — was 6706-2RS 30×37×4: same Ø37 OD keeps `chassis_bottom` print-compatible, the smaller bore roughly doubles the race cross-section so every printed retaining lip/seat got thicker. The yaw joint of every leg uses a **TOUCHING PAIR** (2 per leg × 6 legs) stacked face-to-face on the `coxa_link`'s hub boss — 7 mm-wide races touching give the same 7 mm centre-to-centre moment spacing the old spaced 4 mm pair had (tilt stiffness ∝ spacing²), reacted into the `chassis_bottom` tower instead of the servo spline. **The tower is SPLIT:** the LOWER race drops onto the open-top Ø37 pocket in `chassis_bottom` (its top 3 mm stands proud of the frozen split plane), then the `yaw_bearing_cap` lowers over both races and 3 M3 screws pull it down, sandwiching the outer races between the chassis seat and the cap's Ø32 lip. Inner races slip onto the hub boss from below. Buy a 12+ pack for spares. | [Amazon: 6805-2RS bearing 25x37x7](https://www.amazon.com/s?k=6805-2RS+bearing+25x37x7) |
| 1 | Carbon-fibre tube, Ø8 mm | **Ø8 mm OD × Ø6 mm ID roll-wrapped CF tube.** TIBIA leg segments only (1 per leg — the femur is one printed part with no tube, Jul 2026). One ~1 m length yields all 6 × ~110 mm tibia segments with spare. Epoxy-bonded into the printed sockets. | [Amazon: 8mm carbon fiber tube 6mm ID](https://www.amazon.com/s?k=8mm+carbon+fiber+tube+6mm+id) |
| 1 | Two-part epoxy | Slow-cure (30 min) structural epoxy for bonding the tibia CF tubes into the printed yoke/fitting sockets (the femur needs no epoxy — it is one printed part). | [Amazon: 30 minute structural epoxy](https://www.amazon.com/s?k=30+minute+two+part+epoxy) |
| 1 | Nylon zip-ties, 100-pack | 24 used for cable strain relief: 3 per leg at the cradle `cable_zip_post`s (yaw / hip / knee) + 1 per leg looped through the chassis_bottom leg-harness drop slot (each leg's cable drop slot doubles as the zip-tie anchor -- pass a zip-tie through the slot to bundle the per-leg harness).  2-3 mm wide, ~ 100 mm long. | [Amazon: nylon zip ties 4 inch 100 pack](https://www.amazon.com/s?k=nylon+zip+ties+4+inch+100+pack) |
| 1 | Dupont jumper kit | Mixed M-F / F-F / M-M jumper wires for I2C and logic wiring. | [Amazon: dupont jumper wires 120 pcs](https://www.amazon.com/s?k=dupont+jumper+wires+120+pcs) |
| 1 | USB-C cable | USB-C cable for flashing / powering / console access to the Arduino Uno Q. | [Amazon: USB C cable data](https://www.amazon.com/s?k=USB+C+cable+data) |
| 1 | M3 screw assortment | M3 socket-head screws, nuts, washers, lengths 6/8/10/12/16/20 mm. | [Amazon: M3 stainless screw kit assortment](https://www.amazon.com/s?k=M3+stainless+screw+kit+assortment) |
| 1 | TPU 95A filament (250 g) | Foot boots: 6 (+2 spare) `foot_boot.stl` pressed over the tibia CF-tube ends (Aug 2026 — replaces the hinged foot pads; the M3 × 16 pan-head hinge pins + nyloc nuts are RETIRED, so no nut driver or Phillips remains anywhere in the build). | [Amazon: TPU 95A filament](https://www.amazon.com/s?k=TPU+95A+filament+1.75mm) |
| 1 | M3 x 10 SHCS (`91290A114`) | M3 x 10 mm socket-head cap screw, black-oxide steel.  **96 used:** the link-to-disc-horn bolts (see the census below).  Aug 16 2026: the 2 switch_holster mount bolts are RETIRED (velcro mount); the 4 chassis_top → brass-standoff bolts moved to M3 × 8 when the top plate was halved to 2 mm.  Buy a 100-pack. | [McMaster 91290A114](https://www.mcmaster.com/91290A114/) |
| 1 | M3 x 14 SHCS (`91290A115`) | M3 x 14 mm socket-head cap screw, black-oxide steel.  **4 used (Jul 2026 F/F standoff switch):** one per chassis standoff, driven UP from below chassis_bottom's −6 mm floor face, through the 8 mm plate + floor stack, into the F-F brass standoff's bottom female thread.  Replaces the old M-F male stud + nyloc, which could not span the merged 8 mm plate. | [McMaster 91290A115](https://www.mcmaster.com/91290A115/) |
| 1 | M3 x 8 SHCS (`91290A113`) | M3 x 8 mm socket-head cap screw, black-oxide steel.  **~52 used (Aug 2026):** **24 sandwich-joint clamp-cap bolts** + **18 yaw_bearing_cap join screws** + **4 chassis_top → brass-standoff top bolts** (moved from M3 × 10 when the top plate was halved to 2 mm) + **3 screen-stand feet** (up from under the round plate into the feet's Φ2.5 self-tap pilots) + **3 Uno Q mount** (down into thumb nuts).  Buy a 100-pack so you have spares. | [McMaster 91290A113](https://www.mcmaster.com/91290A113/) |
| 1 | M2.5 x 8 SHCS (`91290A104`) | M2.5 x 8 mm socket-head cap screw.  **30 used:** 18 servo spline center screws + 12 hip/knee passive-horn retention screws.  The END-face **body-retention screws are fully RETIRED** (Aug 16 2026 — the hip cradle's last 24 went with their holes; every servo is held by its clamp cap + retaining lip + output-face seat.  Yaw had already dropped its set in the Jun 2026 flush-horn refit, knee in the Jul 2026 one-piece femur).  Buy a 100-pack. | [McMaster 91290A104](https://www.mcmaster.com/91290A104/) |
| 1 | M3 standoffs (chassis) | M3 x 20 mm FEMALE-FEMALE brass standoffs, pack of 10-20 (**same stock as the magnet-post bases** — one bag covers both).  **4 used** on the 44-mm-radius diagonal pattern (`CHASSIS_STANDOFF_HOLES_XY` = (±31.1, ±31.1)) clamping chassis_top to chassis_bottom across the CHASSIS_GAP = 20 mm inter-plate gap (Aug 2026: was 32 mm; the gap shrank when the battery moved under the belly — the bay now only clears the 11.3 mm corner Wago trays + wiring).  Jul 2026 F/F switch: an M3 × 14 SHCS enters UP from below chassis_bottom's −6 mm floor face into the standoff's bottom female thread; an M3 × 8 SHCS drops DOWN from above chassis_top into the female top (was × 10 through the 4 mm plate; the 2 mm plate needs × 8).  Re-verify lengths whenever CHASSIS_GAP or CHASSIS_TOP_T changes. | [Amazon: M3 20mm standoffs female female brass](https://www.amazon.com/s?k=M3+20mm+standoffs+female+female+brass) |
| 1 | M2.5 screw pack | M2.5 x 8 mm screws, useful for horn/adapter work if the servo-included screws are bad. | [Amazon: M2.5 8mm screw 50 pack](https://www.amazon.com/s?k=M2.5+8mm+screw+50+pack) |
| 1 kg | PLA or PETG filament | 1.75 mm. Structural printed parts. PLA is easiest; PETG is tougher. | [Amazon: PLA filament 1.75mm 1kg](https://www.amazon.com/s?k=PLA+filament+1.75mm+1kg) |
| 250 g | TPU 95A filament | Foot pads. If you skip TPU, print feet in PLA and glue rubber tread into the cups. | [Amazon: TPU 95A filament 1.75mm](https://www.amazon.com/s?k=TPU+95A+filament+1.75mm) |
| 1 | Heat-shrink kit | Assorted small heat-shrink tubing for power wiring cleanup. | [Amazon: heat shrink tubing assorted](https://www.amazon.com/s?k=heat+shrink+tubing+assorted) |

<!-- BEGIN FASTENERS (auto-generated by scripts/render_fastener_bom.py) -->

## Fasteners

Auto-derived from `fastener_registry.build_all_fastener_instances()`.
Edit the registry (not this table) and re-run `make bom-fasteners`.

| Spec | McMaster P/N | Qty | Used in |
|------|--------------|-----|---------|
| M3x6 SHCS self-tap | 91290A111 | 24 | yaw_servo_retainer L0 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L0 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L0 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L0 saddle outboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L1 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L1 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L1 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L1 saddle outboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L2 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L2 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L2 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L2 saddle outboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L3 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L3 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L3 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L3 saddle outboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L4 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L4 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L4 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L4 saddle outboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L5 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L5 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L5 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L5 saddle outboard -Y chassis anchor M3 self-tap |
| M2.5x8 spline screw | 91290A104 | 30 | hip passive-horn retention screw L0, hip passive-horn retention screw L1, hip passive-horn retention screw L2, hip passive-horn retention screw L3, hip passive-horn retention screw L4, hip passive-horn retention screw L5, knee passive-horn retention screw L0, knee passive-horn retention screw L1, knee passive-horn retention screw L2, knee passive-horn retention screw L3, knee passive-horn retention screw L4, knee passive-horn retention screw L5, servo spline center screws |
| M3x8 SHCS | 91290A113 | 4 | brass standoff female threads (virtual heat-set insert engagement targets) |
| M3x8 SHCS self-tap | 91290A113 | 42 | sandwich-joint clamp-cap bolts (M3 SHCS self-tap), yaw_bearing_cap join screws (cap -> chassis_bottom tower, M3 x 8 SHCS self-tap) |
| M3x10 disc-horn SHCS | 91290A114 | 96 | link-to-disc-horn bolts |
| M3x14 SHCS | 91290A115 | 4 | chassis_bottom brass standoff bolts (M3 x 14 SHCS up into standoff female thread) |
| M3x30 disc-horn SHCS | 91290A123 | 24 | link-to-disc-horn bolts |
| M2.5 self-tap into servo rear case | 96877A150 | 24 | yaw_servo_retainer L0 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L0 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L0 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L0 saddle x2-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L1 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L1 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L1 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L1 saddle x2-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L2 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L2 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L2 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L2 saddle x2-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L3 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L3 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L3 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L3 saddle x2-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L4 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L4 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L4 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L4 saddle x2-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L5 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L5 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L5 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L5 saddle x2-Y rear case-mount M2.5 self-tap |
|  |  | **248** | **total fasteners** |

Notes:
- The servo OUTPUT face is reserved for the flush 20 mm disc
  horn (no front-face mounting).  END-face body-retention screws
  are FULLY RETIRED (Aug 16 2026): the hip cradle's last 4-per-leg
  set (24 x M2.5 x 8 through the -X wall) went with its holes
  (user: "four meaningless holes ... pointless now" -- never
  installed on the bench).  The yaw cradle had dropped its set in
  the Jun 2026 flush-horn refit (servo hangs below the chassis
  floor; held by the `yaw_servo_retainer` strap + output-face
  seat) and the knee cradle in the Jul 2026 one-piece femur (the
  fused spar covers that wall).  Every servo is held by its
  clamp cap + retaining lip + output-face seat.
- Link-to-disc-horn bolts (120 total) thread into the 20 mm
  aluminum 25T disc horn's M3 TAPPED holes on a 14 mm bolt
  circle (cross pattern at 0/90/180/270 deg); the aluminum is
  the thread-engagement medium -- no self-tap, no heat-set.
  Two lengths (Aug 2026 coxa-merge era): the hip + knee horns
  (driven front + passive rear-boss) all take **96 x M3x10 SHCS
  (`91290A114`)**, while the 6 yaw horns take **24 x M3x30 SHCS
  (`91290A123`)** through the taller coxa yaw-hub stack (Aug 16
  2026, user's own M3x30 stock -- was M3x20/91290A120; the head
  seat rose 10 mm so a normal driver reaches the heads through
  the empty hip servo well).  (June
  2026 disc-horn switch, retiring the plastic 4-arm X-horn's
  M2x8 self-tap scheme.)  See `fasteners/README.md` for the
  full rationale.
- NO nuts and NO Phillips anywhere (late-Aug 2026): the last
  nyloc + pan-head pair (the foot hinge pin) left with the
  pressed-on TPU `foot_boot`.  Every screw above is hex-socket
  except the M2.5 rear-case self-taps; add the 4 off-registry
  M2 self-tappers that hold the screen to its stand.
- OFF-REGISTRY M2.5 x 6 self-taps (`96877A150`): the femur and
  coxa rear retention tabs each take 2 per leg (Aug 2026 femur
  knee tab; Aug 17 2026 matching coxa hip tab) = **24 more**
  than the 24 saddle screws in the table -- order **48 total**.
- The M2.5 spline center screw ships with each STS3215 servo --
  it's listed here so the screwdriver-access verifier check knows the
  fastener exists, but you do NOT order it separately.
- See `fasteners/README.md` for the McMaster STEP swap-in flow if you
  want to replace the parametric fallback geometry with real CAD.

<!-- END FASTENERS -->

## Strongly Recommended Tools

| Qty | Item | Why | Link |
|---:|---|---|---|
| 1 | Digital calipers | Check STS3215 body/tab dimensions before printing all six legs. | [Amazon: digital calipers](https://www.amazon.com/s?k=digital+calipers) |
| 1 | Soldering iron kit | Header / harness soldering.  (Aug 16 2026: the build has ZERO heat-set inserts left -- the switch_holster's were the last, retired with its velcro mount.) | [Amazon: soldering iron kit beginner](https://www.amazon.com/s?k=soldering+iron+kit+beginner) |
| 1 | Metric hex key set | M3 socket-head screws usually need a 2.5 mm hex key. | [Amazon: metric ball end hex key set](https://www.amazon.com/s?k=metric+ball+end+hex+key+set) |
| 1 | Super glue gel | Foot tread / small print fixes. | [Amazon: super glue gel](https://www.amazon.com/s?k=super+glue+gel) |
| 1 | Needle file set | Useful if a servo well or femur slot is slightly tight from FDM over-extrusion. | [Amazon: needle file set](https://www.amazon.com/s?k=needle+file+set) |

## Printed Parts

Files are in `hexapod_walker/prototype_sts3215/stl_prototype/`. Print
settings below are for an Ender 3 / Bambu A1-class FDM printer; total
machine time is roughly **16 hours** across 6–7 sessions.

| Qty | STL | Material | Layer | Infill | Walls | Print time | Notes |
|---:|---|---|---|---:|---:|---:|---|
| 1 | `chassis_top.stl` | PLA / PETG | 0.2 mm | 25% gyroid | 4 | ~2 h | Top hex; 4× `CHASSIS_STANDOFF_HOLES_XY` (±31.1) take sandwich standoffs below + magnet posts above.  Fully flat both faces (Aug 16 2026: switch-holster bosses retired). |
| 1 | `chassis_bottom.stl` | PLA / PETG | 0.2 mm | 25% gyroid | 4 | ~2 h | Single merged Jun 2026 part: genuinely FLAT-bottom 200 mm hex plate with a folded-in ~8 mm-thick flat floor carrying the 6 yaw-servo cradles + upward yaw-bearing tower. Prints face-DOWN, no supports. Aug 2026 rev: one open 18 × 28 mm harness port per leg (ribs removed) + a 14 × 22 mm battery XT60 pass-through at (48, 0); late-Aug 2026: the 6 corner power-Wago tray wall sets are printed into the top face (the separate `wago_mount` trays are retired). Aug 16 2026: Ø9 standoff seat pads restore the 4 spacer seats the shifted harness ports had clipped. Two shorty LiPos velcro'd under the belly (centre slot pair = safety strap); data Wagos under the belly at the hex vertices. |
| 1 | `switch_holster.stl` | PLA / PETG | 0.2 mm | 25% gyroid | 3 | ~0.3 h | Snap-in holster for the anti-spark on/off switch. Velcroed flat to chassis_top's +X edge (Aug 16 2026: bolt-down ear/bosses/inserts retired -- solid flat floor for the velcro patch). |
| 1 | `extra_stl/round_mount_plate_115_with_leg_holes.stl` | PLA / PETG (print-only — the SVG cut file is retired, a laser-cut plate loses the registration bosses) | 0.2 mm | 20% gyroid | 3 | ~0.5 h | Round Ø115 magnet-held board (matches the chassis_top disc) for Uno Q + breakout on top, 3.3 V Wago underneath. Late-Aug 2026: 4 underside bosses socket the Ø8 magnet tops (shear registration — magnets only carry pull); legacy 49.5 mm bolt square dropped. Review round 2: 3 stand-foot holes (M3×8 up into the stand's pilots), 3 Uno Q mount holes on the standard UNO pattern (M3×8 down into thumb nuts — SE hole omitted, too close to the SE boss), 2 Ø8 wire ports + E/W zip-tie slots (now cut in the STL). Generate with `tools/make_xtool_hex_raised_platform.py`. |
| 1 | `extra_stl/hex_raised_platform_110_h28_screen.stl` | PLA / PETG | 0.2 mm | 15% gyroid | 3 | ~0.8 h | Screen stand ("hex" is historical) — round Ø115 top disc matching the plate below, on 3 blade legs at az 90/210/330 (28 mm, shortened from 72 in the late-Aug 2026 design review — less lever on the magnet-held plate, ~44 mm CoG drop). Mounts with 3× M3×8 SHCS driven up from under the plate into blind Φ2.5 self-tap pilots in the feet. Screen on the top face, held by 4× M2 self-tappers in Ø1.8 corner pilot holes; its 8-wire Uno pigtail enters through the 24×5 slot behind the panel's +X edge (MPU is on chassis_top near the centre, not under this plate). |
| 12 | `servo_clamp_cap.stl` | MJF PA12 (PLA OK) | 0.2 mm | solid | 3 | ~0.2 h ea | Sandwich-joint clamp cap — 1 per hip + knee joint (2 per leg × 6 legs). Bolts to the cradle ±X wall ends with 2× M3 × 8 SHCS self-tapping into the cradle pilots. Aug 18–19 2026: a 10 mm-wide L-shaped back-face hook near the wire end — 5.5 mm wall + a shelf that laps 7.4 mm over the back of the motor toward its middle, flush with the back surface (the wire-end band steps up, so no bite there), plus a tiny corner pad at the horn end with a 1 mm press-fit bite (gap to the Ø20 passive horn) that preloads the body against the front retaining lip (the corridor beside the main hook stays open for the 5264 plugs and cables, clear of the swinging yoke); the outer face's vertical edges carry 1 mm 45° chamfers so the sweeping yoke spine can't catch a corner. |
| 6 | `coxa_link.stl` | PLA / PETG | 0.2 mm | 30% gyroid | 4 | ~4 h total | ONE piece (Aug 2026 merge): yaw turntable hub (rides the touching 6805 pair) + hip fixed side (Aug 16 2026: no end-face bolt holes / wire channel); 5 head-access shafts reach the M3x30 yaw horn screws (Aug 17 2026 sink pass: well floor dropped 5 mm to just over the platform top, heads ~5.3 mm below the shaft mouths; corner seats 1.25 mm deeper for full horn bite, centre 1 mm deeper still; cap labyrinth gaps widened to 1.5 axial / 1.0 radial after a bench scrape). Aug 17 2026: same rear retention tab as the femur — 2× M2.5×6 self-tap (PN 96877A150) into the hip servo's rear molded pair nearer the inboard/wire end, heads flush in Ø5.2×2 mm recesses; connector-end pair stays open. Print on its SIDE (yaw axis horizontal, cradle end wall on the bed). |
| 6 | `yaw_bearing_cap.stl` | PLA / PETG | 0.2 mm | 40% gyroid | 4 | ~0.3 h ea | TOP half of the split yaw-bearing tower; bolts onto each `chassis_bottom` tower with 3× M3 × 8 SHCS to capture the touching 6805 pair (12 mm tall ring, Ø32 × 2 mm retaining lip). Print flat, ring face down (no supports). |
| 6 | `femur_link.stl` | PLA / PETG | 0.2 mm | 35% gyroid | 4 | ~8 h total | The WHOLE femur, one printed part (Jul 2026 merge) — hip moving yoke (8 mm spine plate since Aug 2026) + solid Ø18 spar with small cone flares at both ends + knee fixed side. No CF tube, no socket, no roll pin. Aug 2026: rear retention tab under the knee servo's open back face — 2× M2.5×6 self-tap (PN 96877A150; +24 total over the saddle count with the coxa's matching tab) into the rear molded hole pair nearer the spar, fully-enclosed holes since the Aug 17 2026 1 mm-longer tab (bench: the flat back-face ledge is ~5 mm, not 4 — the old keyhole slots are retired), heads flush in Ø5.2×2 mm recesses (5.5 mm tab = 3.5 shank + 2 head pocket); the connector-end pair stays open for the bus harness. Print yoke spine-down / spar horizontal; support the knee servo well through its open back. |
| 6 | `tibia_knee_yoke.stl` | PLA / PETG | 0.2 mm | 35% gyroid | 4 | ~4 h total | Knee moving yoke (symmetric clevis — both arms bolt to a disc horn) + Ø8 CF-tube socket (epoxy-only since Aug 2026 — the retention-pin cross-hole is gone, no drilling, no pin). |
| 6 | `foot_boot.stl` | TPU 95A (required — PLA has no grip or stretch) | 0.25 mm | 100% (TPU) | 3 | ~0.8 h total | Standard TPU boot pressed over every tibia Ø8 CF-tube end (Aug 2026 — replaces `tibia_foot_fitting` + `foot_pad` + the hinge pin). Ø8.1 bore = same nominal slip fit as the yoke socket; printed TPU lands snug, CA/epoxy dab for keeps. Nominal Ø9 rounded/spherical ground-contact end with an 8 mm solid tip section; the furthest tip is the shared 150 mm knee-centre-to-tip station on all six legs. Print mouth-face on bed, dome up — no supports. Regenerate the slicer copy with `uv run --with numpy --with trimesh python hexapod_walker/prototype_sts3215/tools/make_extra_foot_boot.py`. |
| spare | `extra_stl/tibia_tube_printable.step` (+ `.stl`) | PETG preferred, 100% infill | 0.2 mm | 100% | — | ~0.5 h | **Emergency printed stand-in for one tibia Ø8 CF tube** (field replacement when no tube stock is at hand). Exact tube geometry: Ø8 OD × Ø6 ID × 142 mm, matching `TIBIA_LENGTH - FOOT_BOOT_TIP_L` for the 150 mm knee-centre-to-tip leg. Print LYING DOWN so layers run along the tube; regenerate (or get a bore-less stronger variant with `--solid`) via `tools/make_tibia_tube_step.py` — see its docstring for the uv run line (needs build123d). Expect less stiffness than CF; treat as a get-home part. |
| 6 | `yaw_servo_retainer.stl` | PLA / PETG | 0.2 mm | 20% gyroid | 3 | ~0.4 h ea | Anti-rotation saddle under each hanging yaw servo, with FOUR corner-pole FEET (Aug 2026 v4: the flat-belly rework deleted the 38 mm stand, which left the yaw harness crushed against the floor — 34 mm feet returned; the v3 central tripod disk blocked the case-bottom servo plug, so v4 uses four slender corner poles with Ø12 pads and an open underside). Central wire drop window. Prints flange-down (180° X flip). |

Every part fits a 220 × 220 mm bed individually; the chassis plates
(200 × 230 mm in default orientation) need a 30° rotation on a true
220 × 220 bed, but fit a 235 × 235 mm bed without rotation.

### Suggested print order

Print **legs first, body last** so you can dry-fit each leg on a servo
before committing the chassis plates:

1. **1 full leg set first** — `coxa_link` + `femur_link` +
   `tibia_knee_yoke` + `foot_boot` — to validate
   the symmetric bracket + dual-disc-horn fit before committing six sets
   (the passive side uses the servo's stock metal rear horn; nothing
   extra to print).
2. 6 × `coxa_link` + 6 × `yaw_bearing_cap` — ~4 h.
3. 6 × `femur_link` (one-piece femurs) — ~8 h.
4. 6 × `tibia_knee_yoke` — ~4 h.
5. 12 × `servo_clamp_cap` (MJF PA12 or PLA) — ~3 h.
6. `chassis_top` + `chassis_bottom` + `switch_holster` + `extra_stl`
   hex mount plate / raised platform — ~6 h.
7. 6 × `foot_boot` in TPU — ~0.8 h.

After step 2 you can start mounting the yaw servos into the
`chassis_bottom` cradles + split bearing towers and verifying the bolt
patterns line up — a parallel track between printing and final assembly.

All six tibias now use the same CF tube length and the same standard
`foot_boot`; the kinematic knee-centre-to-furthest-tip length is 150 mm.

Femur = `femur_link`, one printed body — no pins, no sockets, no CF tube
(Jul 2026 merge of the old hip yoke + knee bracket; the connecting spar
is solid Ø14, the old socket-boss outer diameter).
Tibia = `tibia_knee_yoke` + Ø8 CF tube + pressed-on `foot_boot`.

Design B (May 2026) retired the printed `servo_horn_adapter` -- each
link now bolts DIRECTLY onto a purchased servo horn, so it no longer
appears in the printed-parts set.  June 2026 disc-horn switch: the
purchased horn is now a 20 mm aluminum 25T disc (Amazon B07D56FVK5),
bolted with 4 x M3 x 6 SHCS on a 14 mm bolt circle into the disc's
M3 tapped holes (replacing the old plastic 4-arm X-horn + M2 scheme).

For a one-leg test, print only:

| Qty | STL |
|---:|---|
| 1 | `coxa_link.stl` |
| 1 | `femur_link.stl` |
| 1 | `tibia_knee_yoke.stl` |
| 1 | `foot_boot.stl` |

## Bench Test Order

1. Buy **one STS3215** + **one Arduino Uno Q** + **one 25T disc horn** (driven side; the passive side uses the servo's stock metal rear horn) first to de-risk the joint.
2. Print one full leg set (above) to validate the symmetric bracket + dual-disc-horn fit before printing six sets.
3. Confirm the STS3215 body seats snug in each cradle: at the HIP it is bolted by 4× M2.5 into its END-face 10×10 hole square (driven through the cradle's −X wall) plus the printed clamp cap (which seats FLUSH against the body's +Y face so the bolts trap it with no slop); at the KNEE there are NO end-face bolts (Jul 2026 one-piece femur — the fused spar covers that wall), so verify the clamp cap + retaining lip alone hold the body with no slop.  The front disc horn drives the yoke top arm via its 4× M3 leg bolts on the Ø14 cross, and the rear STOCK passive horn — its centre bore over the servo's idler boss, flush on the back face, retained by one M2.5 screw — takes the bottom arm's 4× M3 the same way.
4. Wire one STS3215 to the Uno Q's TTL bus pins and run:

```bash
python hexapod_walker/prototype_sts3215/motor_setup/feetech_bus.py --port /dev/ttyUSB0 wiggle --joint 0
```

5. Once fit and motion are good, buy/print the rest.

## Rough Cost

| Bucket | Estimate |
|---|---:|
| STS3215 serial-bus servos, 20 total | $360-$500 |
| Arduino Uno Q + microSD + USB-C cable | $60-$100 |
| Battery + charger + safety bag + switch + velcro | $70-$120 |
| Power distribution (fuses + Wagos + 16-18 AWG silicone + 5264 crimp kit; no PDB) | $30-$50 |
| Disc horns (30) + CF tube + epoxy | $30-$55 |
| Fasteners / standoffs / magnet posts / wiring consumables | $30-$55 |
| Filament + MJF clamp caps + hex plate / raised platform | $30-$55 |
| **Total** | **~$615-$1000** |

If you already own a charger, tools, or filament, the actual
cash outlay is much lower.
