# Hexapod Walker — AK40 Quasi-Direct-Drive Prototype

> The third-generation tabletop hexapod: same regular-hex chassis, six
> identical 3-DOF legs, and alternating-tripod architecture as
> [`prototype_sts3215`](../prototype_sts3215/PROTOTYPE.md), but every joint
> is a **CubeMars AK40-10 (KV170) quasi-direct-drive actuator** — a 24 V
> brushless pancake with an integrated 10:1 planetary, FOC driver, and
> absolute encoder — instead of a position-only hobby bus servo. This is
> the jump from "commanded positions and hoped" to **real torque control**.
>
> Status (Design C, Aug 2026): **printable-pending-verification**. The 18
> actuators are in hand and `stl_prototype/` holds a full printable set
> generated from the official CubeMars 2D drawing. The full assembly is
> build-audited: all 144 fasteners are instanced in BuildViz, the
> attachment graph is machine-checked, and static + 214-pose swept
> overlap checks are clean. Verify the interface dims with calipers on
> one unit before printing six leg sets
> ([§9 Build path](#9-build-path--open-items)).

---

## 1. Why a new prototype?

The STS3215 robot proved the geometry, the gait code, and the RL pipeline.
What it could never give us:

1. **Torque control.** The AK40's MIT mode takes a
   (position, velocity, kp, kd, feed-forward torque) tuple per cycle and
   closes an FOC current loop onboard. RL policies can output torques or
   impedances, not just position targets — the sim-to-real gap that hurt
   most on the STS3215 largely disappears.
2. **Proprioceptive contact sensing.** Back-drive torque is 0.06 N·m
   (10:1 planetary, no worm, no clutch). Foot touchdown is visible in the
   current estimate — no foot switches needed.
3. **Speed.** 370 rpm rated at the output vs ~45 rpm-class servo speed.
   Swing-phase speed stops being the gait bottleneck; dynamic gaits are on
   the table.
4. **Real joints.** The actuator has proper output bearings, so links bolt
   straight to the output flange. The entire STS3215 sandwich-joint
   apparatus — passive rear horns, disc-horn pairs, printed clamp caps,
   the 6805 yaw bearing towers — is deleted. Expected fastener count drops
   by roughly half.
5. **A real bus.** CAN at 1 Mbps with onboard drivers, vs a half-duplex
   TTL daisy-chain. Per-joint telemetry (position, velocity, current,
   temperature) at control rate instead of polled.

The price: ~4.5× the mass (190 g vs ~60 g per joint) and a 24 V system.
The whole design below is the consequence of those two facts.

## 2. Specification at a glance

| Property | Value |
|---|---|
| Configuration | 6 legs × 3 DOF, alternating-tripod gait |
| Actuator (all 18 joints) | CubeMars AK40-10 KV170, 10:1 QDD, CAN, MIT + servo modes |
| Rated / peak joint torque | 1.3 N·m / 4.1 N·m |
| Rated joint speed | 370 rpm (38.7 rad/s) |
| Chassis | 260 mm flat-to-flat hex, two 5 mm printed plates, 50 mm gap |
| Leg links (axis-to-axis) | coxa 65 / femur 100 / tibia 150 mm |
| Overall envelope (nominal stance) | ~645 × 597 × 338 mm |
| Ride height (hip axis to ground) | 227 mm nominal, 140 mm crouch (transitional) |
| Vehicle mass (budgeted) | ~5.8 kg (56.9 N) |
| Per-foot static load | 9.5 N six-leg / 19.0 N tripod |
| Battery | 6S 5000 mAh LiPo (22.2 V nom / 25.2 V full) |
| Bus voltage | Raw 6S to all actuators (24 V-rated) |
| Compute | Raspberry Pi 5 + 3 × USB-CAN (one bus per leg pair) |
| Control | MIT mode ≤500 Hz per bus; policy loop 50 Hz |
| E-stop | XT90-S anti-spark **loop key** on the motor rail (compute stays up) |
| Estimated cruise draw | 90–180 W walking; ~25 W standing |
| Estimated run time | 40–60 min walking on 111 Wh |

All derived numbers regenerate with
`python hexapod_ak40.py --report-only` (written to
`artifacts/design_budget.md`); constants in `hexapod_ak40.py` are the
source of truth, mirrored in `design_spec.yaml`.

## 3. Torque budget — the hip pitch sizes the robot

Static joint torque for a vertical ground reaction of (weight ÷ legs
down) at the foot, from `hexapod_ak40.py`:

| Stance | Ride height | Foot reach | Hip (tripod) | Knee (tripod) | Hip (6-leg) | Knee (6-leg) |
|---|---:|---:|---:|---:|---:|---:|
| tall (femur −60°, tibia 12°) | 233 mm | 81 mm | 1.54 | 0.59 | 0.77 | 0.30 |
| **nominal (−55°, 15°)** | 227 mm | 96 mm | 1.82 | 0.74 | 0.91 | 0.37 |
| crouch (−20°, 45°) | 140 mm | 200 mm | 3.79 | 2.01 | 1.90 | 1.01 |

(N·m per joint. Yaw joints see ~zero static load; they size on swing
acceleration and are trivially fine.)

Sizing rules this design commits to:

- **Park poses live under rated.** Six-leg stand at tall/nominal loads
  the hip at ≤0.91 N·m — 70% of the 1.3 N·m continuous rating. The
  robot can stand indefinitely without thermal drama.
- **Tripod walking exceeds rated transiently but not thermally.** At
  nominal stance the hip sees 1.82 N·m during its ~50% stance phase, so
  the thermal average is ~0.91 N·m — at 44% of the 4.1 N·m peak there
  is 2.2× dynamic headroom for accelerations and disturbance rejection.
- **Crouch is transitional only.** At 140 mm belly height the hip needs
  1.90 N·m even on six legs (above rated, 46% of peak). It exists as the
  sit-down/stand-up path — seconds at a time — never as a parking or
  walking stance. This is the AK40 analogue of the STS3215 "stilt pose"
  lesson: the software torque limits and any RL reward must enforce it.

Unlike the STS3215 (stall torque quoted, real thermal limit unknown),
the AK40 has a specified continuous rating and onboard temperature
telemetry, so these limits are enforceable in software per-cycle.

## 4. Geometry rationale

- **Coxa 65 mm** (vs 12.5 on the STS3215). Not a choice — a Ø53 pancake
  hip actuator physically needs the standoff from the yaw axis. The
  torque cost lands on the yaw joint (unloaded) and slightly on body-roll
  moments, so it is nearly free.
- **Femur 100 / tibia 150 mm.** The tibia is deliberately long relative
  to the femur: static hip torque grows with the *horizontal* projection
  of the femur, while a long, near-vertical tibia buys ride height and
  stride length at low knee torque (see the stance table — knee runs at
  ~40% of hip everywhere). 4.5× the mass of the STS3215 robot but ~4.4×
  the joint torque means similar proportions scale honestly.
- **Chassis 260 mm flat-to-flat, 50 mm plate gap.** Six Ø53 yaw
  actuators stand between the plates (output face down through the
  bottom plate), and the gap clears their 40.2 mm V3.0 bodies plus
  wiring. The 6S pack rides under the belly, house tradition.
- **Stance is tall.** QDD torque economics reward standing tall with
  feet close to the hip vertical — the opposite of the sprawled hobby-
  servo stance. Support polygon is still ~590 mm across at nominal
  (foot circle radius ~296 mm) against a ~230 mm CoM height; tip margin
  is comfortable.

## 5. Structure

**AK40 interfaces** (official CubeMars 2D drawing, AK40-10 V3.0 dated
2026/6/12 — verify with calipers before committing six leg sets):
output flange = 3× M2.5 tapped ×3 deep on Ø27 PCD + Ø15 (+0.05) centre
pilot (assumed a **bore**; printed hubs grow a Ø14.85 boss —
`AK40_PILOT_IS_BORE` flips it if calipers disagree); front case = 3×
M2.5 ×5 on Ø47.5; rear case = 4× M2.5 ×5 on Ø47 with a Ø37 × 1 mm
centering boss. Power+CAN is one side-mounted XT30PW(2+2); UART config
is a 3-pin 1.25 mm A1257WR.

The printed set (all in `stl_prototype/`, built + interference-checked
by `hexapod_ak40.py`):

- **Joints bolt to the output flange.** The actuator's internal bearings
  take all joint loads. No passive horns, no external bearings, no clamp
  caps. Every hub is the same detail: Ø40 disc, pilot boss, 3× M2.5×6
  counterbored into the flange taps (blue threadlocker — aluminum
  threads on a vibrating robot).
- **Yaw mounting**: the actuator stands on the bottom plate inside the
  50 mm plate gap, output down through a Ø44 well; 3× M2.5×8 enter from
  below the plate into the front-case taps. The `coxa_link` hub (Ø40)
  turns inside the well with 2 mm radial clearance.
- **`coxa_link`** (74 g solid): yaw hub + arm plate + hip rear-mount
  wall (Ø37 boss recess, 4× M2.5×8, side gussets). The hip body hangs
  1.5 mm below the arm plate — that clearance is what set
  `HIP_AXIS_DROP` = 66 mm.
- **`femur_link`** (39 g): one flat 6 mm plate, hip-flange hub on one
  end, knee rear mount (Ø37 recess + 4× M2.5×8) on the other. Actuator
  bodies alternate sides of the plate, Mini-Cheetah style; the tibia
  plane ends up ~35 mm tangentially outboard of the hip plane (constant
  offset, folded into the leg kinematics).
- **`tibia_yoke`** (18 g) + Ø12 × 10 CF tube (102 mm cut, 30 mm epoxied
  socket, 2× Ø2.5 roll pins through printed cross-holes) + TPU
  `foot_boot` pressed on the tip (Ø8 tube was fine at 1.3 kg; not at
  5.8 kg).
- **Material**: PETG / PETG-CF everywhere structural (PLA creeps under a
  5.8 kg robot's park loads).
- **Software joint limits from the geometry** (enforce in the safety
  layer): femur pitch ≤ ~75° below horizontal (tibia yoke approaches the
  coxa wall past that), yaw ± ~25° (neighbour-leg spacing), plus the
  crouch torque rule from §3.

## 6. Electrical

Power tree (see BOM for parts):

```
6S 5000 mAh (XT90)
 └─ 40 A main fuse
     ├─ 24→5 V 5 A buck ── Raspberry Pi 5 + IMU        (always on)
     └─ XT90-S anti-spark LOOP KEY ── motor bus bar
         └─ 6 × leg branch (XT30 pigtails, 3 actuators daisy-chained per leg)
```

- **The loop key is the e-stop.** Pulling it kills all 18 actuators
  instantly while the Pi, CAN adapters, and telemetry stay alive — you
  never lose the log of whatever just went wrong, and there is no
  software in the kill path. Given this house's history
  (2026-08-06 process lessons), this is non-negotiable and comes first
  on the bench, before any motion.
- **Currents.** Standing burns ~25 W (copper loss at ≤0.91 N·m per
  loaded joint). Walking estimate 90–180 W. Worst-case transient: a
  stand-up from crouch with all hips near 1.9 N·m ≈ 3.4 A motor-side
  each — bus draw stays well under the 40 A fuse. Each AK40 peaks at
  7.3 A, but never all 18 simultaneously at peak; the per-leg branches
  and 12 AWG trunk are sized for realistic concurrency, not 18 × peak.
- **6S and the 24 V rating.** 25.2 V at full charge is standard practice
  for 24 V-rated CubeMars actuators; regen from backdriven joints is
  absorbed by the battery (do not run the motor rail from a bench PSU
  without a regen clamp or partially charged battery in parallel).

## 7. Control architecture

- **Raspberry Pi 5** running the policy/gait loop at 50 Hz.
- **Three CAN buses, one per leg pair** (6 actuators each), via three
  USB CANable 2.0 adapters (candleLight firmware → Linux socketcan).
  At 1 Mbps, a 6-motor MIT-mode command+reply round is ~1.5 ms, so each
  bus supports ≥500 Hz — 10× the policy rate is spent on the onboard
  impedance loop instead.
- **Control stack**: policy (50 Hz) → per-joint MIT-mode
  (q, qd, kp, kd, τ_ff) → onboard FOC torque loop. Position-only
  "servo mode" exists as a fallback and for bring-up.
- **Safety layer** (software, below the policy, above the bus):
  per-joint torque clamp ramping from a low bring-up limit, watchdog
  that zeros torque commands if the policy loop stalls, temperature
  cutback from onboard telemetry, and the crouch-pose torque rule from
  §3. All the repo hardware-safety rules apply unchanged: no motion
  without an explicit operator ask, single-joint air moves before
  multi-joint, predict → move → read.

## 8. RL path

The `rl_move` stack transfers with the obs/action layout intact, but the
action space changes meaningfully: 18 joints, MIT mode. Phase 0 trains
position-target policies (drop-in for the existing pipeline, kp/kd fixed);
phase 1 moves to torque-offset actions (τ_ff on top of an impedance
stand), which is what this actuator was bought for. Contact estimation
from joint currents replaces the foot-switch observations that were
always missing. Sim actuator model: torque-limited motor + 10:1 ratio +
18 arcmin backlash is small enough to ignore initially.

## 9. Build path — open items

Deliberately staged, basics-first (the STS3215 bring-up order, which is
now house law):

1. **Bench one actuator**: 6S battery + fuse + loop key on the bench,
   one CANable, `candump` telemetry, MIT-mode wiggle at a 0.3 N·m clamp.
   Verify ID assignment, direction conventions, zeroing behaviour.
2. **Calipers on the drawing dims** (flange PCD 27, pilot Ø15
   bore-or-boss, front Ø47.5, rear Ø47 + Ø37 boss, **flange proud vs
   flush** with the fixed front ring, **XT30PW plug azimuth** relative
   to the bolt pattern). Fix any deltas in `hexapod_ak40.py`,
   `make build`.
3. Print the **one-leg set** (`coxa_link`, `femur_link`, `tibia_yoke`,
   `foot_boot`) and dry-fit on real actuators.
4. One-leg air moves + touchdown-detection test on the bench.
5. Only then: six legs, chassis, first supervised stand on the loop key.
   When bolting actuators, **clock every XT30PW plug inboard/up-leg**
   exactly as shown in the BuildViz scene (the bolt patterns allow
   120°/90° indexing; a hip plug clocked "up" hits the coxa arm plate).

Open questions tracked in `design_spec.yaml` `open_items`: pilot
bore-vs-boss confirmation, CAN termination already present or not,
whether the yaw actuator wants a thermal pad to the bottom plate, and
final battery strap geometry.

## 10. Assembly order (audited — every screw has a verified tool path)

The order below is not a suggestion: two fastener groups are only
reachable in this sequence (`scripts/access_audit.py` proves the rest
have straight Ø7 × 50 driver paths in the fully-assembled home pose).

1. **Bench-configure all 18 actuators first** (UART): CAN IDs 1–18,
   direction convention, zero. Mark each case at the plug so the
   clocking (§5, plug inboard/up-leg) is visible during bolt-up.
2. **Six leg subassemblies**, each on the bench:
   a. Knee actuator rear → femur plate (4× M2.5×8), plug toward the hip.
   b. Tibia yoke → knee output flange (3× M2.5×6).
   c. Epoxy the CF tube into the yoke socket; after cure, drive the two
      Ø2.5 roll pins and press the TPU boot.
   d. Femur hub → hip output flange (3× M2.5×6).
   e. Hip actuator rear → coxa wall (4× M2.5×8), plug up-leg.
3. **Yaw actuators onto the bottom plate** (3× M2.5×8 each, from
   below). Must precede the legs: the 330° front-case screw is reached
   through a Ø7 tunnel in the coxa arm that only lines up at yaw = 0 —
   with no leg mounted it is unobstructed.
4. **Legs onto the yaw flanges**: offer the coxa hub up into the Ø40
   well, hand-rotate the flange (0.06 N·m backdrive) to line up, and
   drive the 3× M2.5×6 up through the Ø7 tunnels in the coxa arm.
5. **Electronics deck**: bolt the Pi to the top plate FIRST (4×
   M2.5×12 from below — impossible once the plate is on the
   standoffs), then standoffs to the bottom plate (4× M3×8 from
   below), route the XT30 daisy chains and 3 CAN buses up through the
   Ø80 centre access, then top plate onto the standoffs (4× M3×8 from
   above).
6. **Battery last** (velcro + strap on the belly), after first
   power-on happens on the bench supply through the fuse + loop key.

Threadlocker (blue) on every M2.5 into aluminum; torque to snug +
1/8 turn — these are 2.5 mm screws, not lug nuts.

---

## Design log

> **Design A (Aug 2026) — initial concept.** 18 × AK40-10 KV170 in hand.
> Geometry, mass/torque/power budgets, electrical and control
> architecture. Reference massing STLs only.
>
> **Design B (Aug 2026) — drawing-derived printable set.** Pulled the
> official CubeMars 2D drawing (all-M2.5 interfaces, Ø27/Ø47.5/Ø47
> PCDs, Ø15 pilot, Ø37 rear boss) and grew the six printable parts:
> `chassis_bottom/top`, `coxa_link`, `femur_link`, `tibia_yoke`,
> `foot_boot`. `HIP_AXIS_DROP` went 40 → 66 mm so the Ø53 hip body
> clears the coxa arm plate by 1.5 mm. All parts watertight; pairwise
> part-vs-actuator interference < 1 mm³ across all three stances (the
> actuator mock carries the pilot bore so hub bosses check honestly).
> Fastening: 54× M2.5×6 (flange hubs) + 66× M2.5×8 (case mounts) +
> blue threadlocker; four M3×50 standoffs carry the plate stack.
> Electronics locked: Pi 5 8 GB + 3× CANable 2.0 (candleLight) + Pololu
> D24V50F5-class buck + MPU-6050. Status: printable-pending-
> verification — four caliper checks gate the six-set print run.
>
> **Design C (Aug 2026) — build audit.** Walked the assembly screw by
> screw and fixed three problems the part-level checks had missed:
> **(1) Counterbore breach** — the chassis front-case counterbores
> (Ø5.5 on Ø47.5 PCD, inner edge r = 21.0) broke ~1 mm through the Ø44
> yaw well wall; the well is now Ø40 and the coxa hub Ø36 (still 2 mm
> radial clearance, 1.0 mm land at the cb).
> **(2) Flange-rub risk** — if the rotating output flange sits *flush*
> with the fixed front ring (the side view is ambiguous), any hub face
> wider than the flange rubs the ring under load. All three hub contact
> faces now carry a 0.6 mm relief annulus r16–r22 so contact lands only
> on the flange; caliper item (b).
> **(3) Plug clocking** — the side XT30PW(2+2) plug is now modeled on
> the actuator mock; a hip actuator clocked plug-up would hit the coxa
> arm plate. All 18 plugs clock to leg-frame −x (inboard/up-leg), and
> the overlap check enforces it.
> The BuildViz scene now carries **every fastener as an instance**
> (200 instances: 56 parts + 144 fasteners — 54× M2.5×6, 66× M2.5×8,
> 8× M3×8, 4× M2.5×12, 12× Ø2.5 roll pins), fasteners ride their
> joints, and `make_scene.py` machine-checks the **attachment graph**
> (every part reachable from `chassis_bottom` through a fastener,
> epoxy, press-fit, or velcro edge — report in
> `artifacts/attachment_report.md`). `scripts/check_scene.py` booleans
> every AABB-touching pair: 0 violations; the 150 by-design contacts
> reconcile exactly (120 screws in taps, 12 pins in tubes, 6 boot
> press-fits, 8 standoff screws, 4 Pi screws). `buildviz check` passes
> and a 214-pose `buildviz sweep` across all joint limits + stances
> shows worst penetration 1.23 mm, all in allowed thread-engagement
> pairs. Status: printable-pending-verification (calipers still gate).
>
> **Design D (Aug 2026) — ROM, tool-access + assembly audit.** Two new
> machine checks. `scripts/rom_audit.py`: exact-boolean collision-onset
> search past every software limit (yaw free to ±85°, hip− first
> contact at −79° = 44° of mechanical margin, hip+/knee free past 60°
> of margin) plus 11 combined worst-case poses the one-DOF sweep never
> visits (adjacent yaws converged at ±25° in every stance, all-leg
> hip/knee corner poses) — all clean. `scripts/access_audit.py`: grows
> a two-stage virtual driver (Ø4.6 tip + Ø7 × 50 shaft) behind all 132
> screw heads and a punch path for all 12 pins, booleaned against the
> whole robot. It caught a real build blocker the overlap checks
> could not: the coxa arm plate ran **solid under all three yaw-flange
> screws and the 330° yaw-case screw** — legal rest positions, but the
> screws could never be inserted. Fixed with four Ø7 insertion/driver
> tunnels through the arm (the case-screw tunnel lines up at yaw 0 =
> home = service pose). Only waived access: the 4 Pi screws, driven
> with the top plate off (§10 step 5). Audited assembly order is now
> §10. Workspace numbers: body height 126–313 mm, 161 mm step/cycle at
> ±15° yaw, 39 mm swing lift, 156 mm static stability inradius.
