# Mechanical lessons for the next hexapod — 2026-09-05

Build the next robot around compact, serviceable legs with a well-supported yaw
axis and durable knee/yoke connections. Keep the current robot walking while
developing that leg. The evidence supports improving load transfer and access;
it does not establish that a complete metal robot, larger servos, or an overhead
leg workspace will produce smoother walking. The build choices below address
documented damage, assembly access, and load-path preferences; they are not
prerequisites for restoring the current robot's walking performance.

The existing motors and single bus already demonstrated combined command and
snapshot transactions at 162 Hz on August 19, and a later read-only 100 Hz test
had 6.1 ms p99 latency across 3,000 successful requests. That is transport
evidence, not a loaded motor speed/torque test. Today's timing failures do not
justify replacing servos or redesigning legs before restoring and measuring the
known fast control path. [Timing evidence and wiring implications](NEXT_ROBOT_WIRING_LESSONS_2026-09-05.md)

## What the hardware actually established

- **Measured:** the matched September 4 air screen completed 4,680/4,680 ticks,
  with no trips or overruns. L5 was not an unloaded outlier:

  | Leg | Hip loop | Knee loop |
  |---|---:|---:|
  | L0 | 0.674° | 0.474° |
  | L1 | 0.761° | 0.181° |
  | L2 | 0.264° | 0.382° |
  | L3 | 0.674° | 0.177° |
  | L4 | 0.147° | 0.088° |
  | L5 | 0.615° | 0.350° |

  L4 is unusually tight in this comparison. The earlier two-leg conclusion
  that L5 was uniquely abnormal is superseded for unloaded behavior.
  [Published all-leg result](https://robot-lab.cwd1f0-new-cluster.coreweave.app/experiments/0ab3781b4f3b42daa0d8d71de5d1575c),
  [local experiment state](../robots/experiments/hexapod-1-joint-flex/status.yaml)

- **Measured:** September 3 supported planted tests still gave L5 hip loops
  of 0.776–1.271° and knee loops of 0.688–1.085°, against earlier L4 planted
  values of roughly 0.09–0.35°. There is no completed matched six-leg planted
  comparison in the evidence reviewed. No force channel was calibrated, so
  these are command/response hysteresis measurements, not stiffness in N·m/°.
  The operator accepted this residual; it was not demonstrated repaired.
  [Acceptance report](../robots/experiments/hexapod-1-joint-flex/ACCEPTANCE_2026-09-03.md)

- **Measured:** L4 changed only one encoder count (0.088°) during a 90-second
  loaded hold, with no change through 118 seconds of recovery; L5 had no
  encoder drift in its corresponding test. Moving hip loops of about 5–7°
  include actuator lag and are not static material flex. These tests do not
  support progressive creep at their particular load and duration.
  [September 2 results](../robots/experiments/hexapod-1-joint-flex/RESULTS_2026-09-02.md)

- **Reported, not localized:** screws rock with the femur and the hidden horn
  is difficult to inspect. Visible link movement accompanied by encoder
  movement implicates the servo/output system, but does not identify a loose
  horn, gearbox, yoke, bearing, or fastener uniquely. The acceptance cameras
  did not track the L5 knee or its intended component tags. A calibrated load
  plus proximal/distal markers would distinguish these mechanisms; another
  whole-body walking video cannot do that alone.
  [Localization definitions](../robots/experiments/hexapod-1-joint-flex/README.md),
  [camera limitations](../robots/experiments/hexapod-1-joint-flex/ACCEPTANCE_2026-09-03.md)

For scale, 1° of rotation at a 150 mm lever is about 2.6 mm of endpoint motion
(`r × angle`). This is a geometric illustration, not a measured foot error.

## Prioritized build decisions

| Priority | Decision for the next build | Expected benefit and tradeoff |
|---|---|---|
| 1 | Make one repeatable leg assembly with accessible horn retention, perimeter screws, bearing seats, and removable servo holders. Mark part revision and leg identity on the parts. | Makes preload, wear, and repairs observable; reduces accidental assembly differences. Small access features cost material and require clearance checks, but avoid disassembling a leg just to distinguish screw motion from link motion. |
| 2 | Reinforce the knee/yoke/tube connection first. Use a continuous load path around the socket, generous transitions, and a replaceable moving bracket; retain the carbon tube and foot position initially. | Historical field cracks directly support this target. A suitable metal bracket removes the printed layer-seam failure mode; it does not remove servo backlash and can transfer stress into its printed adapter. |
| 3 | Put bolt preload through correctly sized metal compression stops where the horn joint clamps plastic. Use a washer that also overlaps the plastic, and retain deliberate light plastic compression. | Reduces sensitivity to plastic relaxation and tightening history. It cannot fix worn gears, splines, stripped horn threads, or bearing play. It adds hardware and makes sleeve length/thread engagement important. |
| 4 | For a new chassis, favor the compact rigid-hip support concept: separated upper/lower yaw bearings with a structural upper frame and a service hatch. Keep the hip near its existing radial station. | A larger bearing separation reduces the bearing reaction needed to resist a given moment. Actual stiffness improvement is unmeasured; the upper frame must carry that moment, and six-axis alignment must avoid binding. |
| 5 | Keep feet replaceable and consistent. Reserve a protected sensor-wire route and an interchangeable contact-sensing foot, without making the first walking build depend on force sensing. | Consistent contact geometry helps separate gait problems from grip/compliance changes. A contact signal can identify stance and unloading, but a complicated foot adds mass, friction, tolerances, and wiring. |

**Why the knee/yoke comes before a heavier chassis:** the repository records
two field cracks in the moving printed clevis/socket path. Its reinforcement
study cites a 44.7 N peak single-foot normal load, but that number is a MuJoCo
load case, not a measured ground reaction. Use it for comparative design work;
do not publish a physical safety factor from it. The chassis reinforcement
scene is explicitly a concept sandbox with unresolved full-robot contact/FEA
assumptions. [C-horn history](CHORN_VARIANT.md),
[tibia load-path study](../concepts/tibia_yoke_reinforcement/design_spec.yaml)

**The compression-stop design already has useful fit evidence:** existing
stock is 10.00 mm long, Ø5.5 mm OD/Ø3.2 mm ID; the user-confirmed horizontal
coupon bore is Ø5.8 mm in CAD. Its revised stack uses M3×12 and a thin washer,
with approximately 1.5 mm horn-thread engagement. These are dimensions for
that concept, not a drop-in prescription for the legacy robot. Keep the thin
aluminum horn thread and screw-tip clearance as the limiting hardware details.
[Repository concept](../concepts/horn_compression_limiters/README.md).
The general preload principle is also supported by
[SPIROL's compression-limiter design guidance](https://www.spirol.com/product/compression-limiters/);
its generic bolt-torque guidance must not be substituted for this thin horn's
thread capacity.

**The rigid-hip idea is useful without its obsolete “third bearing” label:**
the current concept uses two bearings per leg, one below and one above, with
about 56 mm center separation and 12 bearings total. Its 6 mm upper frame
provides the load path. This is a structural design rationale, not a measured
56/7-fold stiffness improvement. [Current rigid-hip design](../concepts/rigid_hip/README.md)

## Choose the metal concept for its purpose

- **Compact replacement yoke:** the best first metal intervention if replacing
  the cracked moving load path. Preserve the link lengths and normal walking
  workspace. The older generic C-horn document contains assumed dimensions
  and historical sourcing; use the actually measured bracket geometry instead
  of treating its buying list as current specifications.
- **CNC overhead version:** useful if legs-over-head motion is a real product
  requirement. Its CAD moves the hip 42 mm outward and gains a −110° up-limit,
  but predicts +322 g/robot relative to rigid-hip, 2.15× leg first moment,
  approximately 2.88× point-mass yaw inertia, and 21% more foot-load bending
  leverage. These are model estimates, not hardware measurements. That is a
  costly trade for smooth ordinary walking; do not select it merely because
  aluminum looks stiffer. [Overhead comparison](../concepts/cnc_chorn_overhead/README.md)
- **Two-piece CNC clamp:** improves assembly by separating the two horn-disc
  hole patterns. The modeled penalty is only 1.3 g/clamp (49.3 versus 48.0 g),
  but the locating tongue introduces a new joint and an unresolved root
  strength detail. Keep this serviceability idea; the current geometry is
  still an experiment. [Split-clamp design](../concepts/cnc_chorn_two_piece/README.md)

## Feet, mass, wiring, and access

The guided FSR-foot concept preserves the 150 mm knee-to-tip station and
addresses a real geometric issue: approximately 40° tibia inclination can
edge-load a flat sensor. Its 50° supported tread, 0.10 mm sensing gap, and
0.25 mm hard-stop travel are design dimensions, not demonstrated FDM accuracy
or calibrated force performance. Use it first as a contact/on-off sensor.
[Foot concept](../concepts/fsr_sensor_foot/README.md).
Interlink's [FSR integration guide](https://www.interlinkelectronics.com/downloads/integration-guides/fsr-400-series-integration-guide.pdf)
explains why actuator placement and load distribution strongly affect FSR
readings; it is general guidance, not calibration data for this RP-C10 sensor.

As design inferences, keep battery and power distribution close to the body
center, minimize added mass beyond the hip, and give each leg the same harness
route with strain relief at the fixed end and a service loop clear of the
joint sweep. Make the electronics hatch removable without pulling servo
connections, and make horn access possible without cutting cable ties. The
rigid-hip concept already relocates inaccessible corner power connectors under
its central hatch. These are worthwhile service and load-distribution choices;
the reviewed records do not establish cable drag or center-of-mass offset as
the cause of the present walking asymmetry.

**Size the leg around rated torque and the actual stance lever.**
[Feetech's ST-3215-C018 specification](https://www.feetechrc.com/525603.html)
lists rated torque of 10 kgf·cm (0.98 N·m), versus peak stall torque of
30 kgf·cm (2.94 N·m), at 12 V. Do not use the stall number as a sustained
walking design target. With measured total mass `m` and the minimum supporting
leg count `n`, a first static estimate under equal vertical load sharing is
`τ ≈ (m × g / n) × horizontal joint-to-contact lever`. For a hip or knee, that
lever is the perpendicular distance to the vertical force line; it is not
automatically the full tibia length. Unequal load sharing, limb weight,
horizontal forces, and acceleration require further allowance.
**Illustration only:** 3.5 kg, three supporting legs, and a 100 mm horizontal
lever give about 1.14 N·m before dynamics, already above the published rated
torque. Those numbers are not measured robot mass or a measured load case.
They explain why a lighter, more compact leg can help before changing motors;
the final sizing must use the actual actuator variant, mass, stance geometry,
and duty cycle.
This sizing illustration does not establish that the present robot is
underpowered or that different motors would fix its command timing.

Do not bulk-replace servos based on the old L4/L5 comparison, thicken the whole
chassis from an uncalibrated sim current trace, or replace the carbon tubes
without evidence that they dominate deflection. Also do not mix current CAD
into the physical robot by part name: `hexapod-1` has a confirmed two-piece
coxa and thinner legacy yaw bearings; the inferred old 6706 stack and current
6805 stack have different bores and widths. Compression sleeves and the newer
service-slot revision are not recorded as installed.
[Physical assembly record](../robots/hexapod-1.yaml)

The next tangible mechanical deliverable should be one compact, accessible
leg with the improved knee connection and controlled horn clamp stack. Compare
its loaded movement with the existing leg, then duplicate the winning assembly.
That gives the next robot a demonstrated improvement while walking work on
the present machine continues.
