# Premade 56 mm C-horn experimental hexapod

This is a separate, non-production full-robot build. It combines the
`rigid_hip` top/bottom 6805 bearing structure and the outboard hip layout with
the user's sixteen premade long aluminum C brackets. A symmetric hexapod needs
**12 brackets** (one hip and one knee bracket per leg); the remaining **4 are
spares**.

The build is `prototype_sts3215/premade-chorn-56` in BuildViz.

## Chassis-bottom cleanup

The obsolete rectangular **14 x 22 mm battery-trunk pass-through** at chassis
XY (48, 0) is closed through the complete lower plate in this concept. The six
Velcro strap slots remain unchanged; the center pair remains available for the
battery leads as well as the under-belly safety strap. This is a concept-only
chassis change and does not alter the production or shared `rigid_hip` chassis.

## Horn-on servo removal

Both motor holders now inherit the current production service opening. The
former rounded bridge over the horn has been removed: the Ø24 horn bore
continues at full width through the open +Y clamp face. After removing the two
clamp-cap bolts, the servo can slide out together with its fitted Ø20 disc horn
and clamp cap; the horn no longer has to be removed inside the holder. Three of
the four small front-case capture sites remain, with only the site swept by the
service slot omitted.

The concept generator refreshes the `cnc_chorn_overhead` BREP ancestors before
building, so the coxa and femur cannot silently retain an older closed-bridge
STL. It also sweeps a fitted-horn envelope through both final derived holders
with 1.5 mm radial running clearance. `--skip-ancestor-brep` exists only for
deliberate fast local iteration; the normal Make target never uses it.

## Captive nuts for the removable servo clamps

The remaining M3×8 clamp-cap screws no longer self-tap into PETG. Each hip
cradle has two face-loaded 5.8 mm-AF × 2.7 mm captive-nut pockets, and each knee
cradle has one. The overhead knee design had already removed its inboard side
screw where the bought bracket replaces that wall; its separate vertical cap
retention remains unchanged. That makes **18 captive M3 nuts and 18 M3×8 SHCS
robot-wide**.

Load each ordinary M3 nut from the open clamp face, then seat the cap to close
the pocket. The hex flats are oriented across the 9 mm wall, leaving 1.6 mm of
PETG on both sides. From the recessed head seat, the 8 mm screw reaches 3.3 mm
beyond the conservative back of the nut pocket, and a Ø3.4 clearance tunnel
prevents the tip from bottoming in the former pilot. The M2.5 screws entering
the servo's own molded case holes are deliberately unchanged.

One upper-bearing-carrier screw moved from 0° to 35° on the same bolt circle;
its countersink had been only 1.66 mm from the fixed inboard hip-clamp screw.
The revised head pockets have 4.35 mm edge-to-edge clearance without moving the
servo walls or either clamp screw. A fourth carrier fastener at 180° supports
the formerly unbolted rear side of the bearing cartridge at the walking-load
hotspot.

## Rear support for the outboard coxa

The support is part of the coxa itself. Its footprint is the **rear circular
segment of the lower yaw hub**, using the same 19 mm outer radius and stopping
at X=−10.5 mm. That contour is extruded straight upward from Z=4 to Z=46.25
mm, so the stand never projects beyond the lower coxa envelope.

A matching flange is part of the removable upper-bearing carrier above it.
Two M3×8 screws enter from the top at X=−14 mm and Y=±6 mm, pass through the
carrier flange, and engage captive M3 nuts in 5.8 mm-AF hex pockets opened from
the top of the coxa stand. Drop in the nuts before seating the carrier; the
carrier then closes the pockets. The assembly can be rotated for direct
installation access. There is no separate support print, lower stand
interface, or lower stand screw set.

The marked lower and upper yaw-bearing protrusions are separate cartridges.
The lower carrier is no longer a thin plate on two little screw tongues: an
R13.0–R16.6 annular collar rises 8 mm into a continuous R16.9–R19.0 socket in
the coxa. Three M3×6 low-profile screws enter radially through edge-access
tunnels at 120°, 180°, and 240° and engage captive nuts in the carrier edge.
The collar/socket has 0.30 mm radial assembly clearance and leaves all five
vertical horn-driver paths open. The upper cartridge uses four M3×8
countersunk screws from the cap underside into four captive M3 nuts. Those nuts drop into
5.8 mm-AF hex recesses in the carrier's flat mounting face and are captured
when the cap is seated. Both cartridges preserve their original 6805 bearing
seats and horn interfaces.

The old perforated lower rail remains deleted. The negative-Y side uses one
lower and one upper contour arm. On positive Y, the former two stepped arms
are now one solid rectangular block from Z=4 to 36.7 mm. Its root follows the
same R19 circle, and it stops at X=23.8 mm—the front edge of the existing servo
rear-retention tab—instead of continuing uselessly across the servo. Two Ø6.8
driver openings at X=21.7 mm preserve access to both tab screws. The inherited
front-case screw remains open through the Ø6.5 negative-side driver tunnel;
the positive side ends before that screw axis and is open air. The circular
bridge with the nine-hole grid is still absent, all five yaw-horn driver paths
remain open, and the horn-on servo extraction path is unchanged.

The coxa servo-holder foot is now centered in the top view. Its former
Y=−23.65..+16.65 footprint is completed to Y=+24.25, putting its centre at
Y=+0.30 versus the seated servo at Y=+0.50. Exact centering is shortened by
0.40 mm to retain 0.15 mm model clearance to the moving aluminum bracket. The
positive-Y tab block sits inside this footprint and stops at the retention tab,
so the odd ledge that projected above the servo in the drawing is gone. The
foot material is
only in the low Z=4..12 foot: the servo, horn, cap, hip axis, and open +Y
horn-on extraction path have not moved.

The existing under-plate hip-cap driver path is now completely outside the
R19 stand. The nearest yaw-horn screwdriver path gets a local scallop with 2
mm of extra radial clearance. The generator checks those service paths, the
matched outer envelope, and all six tower-to-chassis clearances.

The two coxa-tower nut traps are 5.8 mm across flats × 2.7 mm deep, sized for
ordinary M3 hex nuts with printing clearance. Verify the nut fit in one coxa
before committing to all six.

## What is new

Per hip or knee joint:

- one purchased C bracket, modeled from the measured 56.0 mm outside span,
  approximately 2.1 mm plate, and 51.0 mm horn-axis-to-front reach;
- one `driven_spacer_7mm`: a one-piece Ø21 × 7 mm four-hole puck with a
  **blind Ø8.8 × 2.8 mm relief** for the driven horn's center screw head and a
  Ø4.2 tool-access hole through the roof;
- one `passive_spacer_7mm`: the same four-hole puck with a **complete Ø8.6
  center hole** for the passive retaining screw;
- a printed front receiver fused into either the femur body or tibia socket.
  It uses all six small holes on the bracket front: the usual 4× M3 on a Ø14
  pattern plus the two additional M3 holes. The extra pair measures **34.0 mm
  between its inner edges** and **40.0 mm across its outer edges**. That means
  the physical holes are Ø3.0 mm and their centers are **37.0 mm apart**, or
  **9.5 mm from each outside edge** of the 56 mm bracket. The bracket's central
  Ø8 hole is clearance, not a primary fastener.

At the femur, six M3×5 screws pass through the 2.1 mm metal web into ordinary
M3 hex nuts loaded into 5.8 mm-AF × 2.7 mm-deep pockets from the bracket face.
The seated bracket captures the nuts. Each screw reaches 0.5 mm beyond its nut
but stops inside the remaining printed skin before the servo cavity.

At the tibia, the front M3 screws pass through the metal web and 8 mm printed
receiver into accessible captive M3 nyloc pockets. This gives four screws
around the center plus a much larger top/bottom anti-twist couple. The tibia's
Ø8 carbon tube is coaxial with the front center hole and seats against a
0.30 mm printed stop behind the metal web.

The tibia tube boss now has four 6.8 mm-wide rounded radial windows around its
root. They expose the four captive nylocs that would otherwise be hidden by
the ring. The measured upper and lower screw pair already sits outside the
ring, so all six fasteners are reachable. The windows stop after the receiver;
the final 21.5 mm at the tube mouth remains a complete circular support collar.
Install the six bracket screws and nylocs before inserting the carbon tube.

The Ø8 tube hole is now **30 mm deep**, up from 14 mm. The longer socket grows
16 mm outward along the existing carbon tube, reducing its unsupported bending
span by 16 mm while leaving the foot position and blind tube stop unchanged.
It therefore does not require an additional 16 mm cut from the tube itself.

## Spacer fit

The current STS3215 horn-face span is 38.04 mm. Two 7.00 mm pucks produce a
52.04 mm assembled inside span. A nominal 56.0 mm bracket made from two 2.1 mm
plates has a 51.80 mm inside span, so the model predicts only **0.24 mm total
spread**, or 0.12 mm per bracket arm.

That is a sensible light spring preload, but the 56.0 and 2.1 measurements
were approximate. Measure all three before machining a batch. If the true
inside span differs, change `hardware_config.toml`; do not pull the arms into
place with the horn screws. A useful target is zero to 0.25 mm total spread.

Use M3×12 disc screws with a washer stack chosen from an actual projection
measurement. With 2.1 mm plate + 7.0 mm spacer, a 12 mm screw has 2.9 mm left
before washers; target roughly 1.5–2.0 mm engagement in the thin aluminum horn
and make sure the tip cannot touch the servo case. Do not assume the nominal
screw length is safe without this dry measurement.

The spacer STL files are suitable for a printed fit test. For the final robot,
6061 aluminum or another rigid, non-creeping material is preferred because the
pucks carry bolt preload in compression. Matching solids are under `step/`:
`driven_spacer_7mm.step`, `passive_spacer_7mm.step`,
and `front_6hole_fit_coupon.step`.

## Coupon result and remaining measurement assumption

The note supplied the outside span, plate thickness, reach, and center hole,
but not the bracket's blade/front width. The model uses the matching commodity
long-U nominal of **25.0 mm**. The 34.0 mm inner-edge and 40.0 mm outer-edge
measurements establish a 37.0 mm center span. The production coupon and both
receivers use those centers with Ø3.4 mm printed clearance holes; the bought
bracket reference shows the inferred Ø3.0 mm physical holes.

Reprint `stl/front_6hole_fit_coupon.stl` to confirm the measured pair before
printing the femur and tibia parts. A straight-on caliper photo would remove
the remaining 25 mm front-width assumption.

## Printable/COTS files

| file | quantity | use |
|---|---:|---|
| `front_6hole_fit_coupon.stl` | 1 first | verify purchased front pattern |
| `driven_spacer_7mm.stl` | 12 | prototype or machine in aluminum |
| `passive_spacer_7mm.stl` | 12 | prototype or machine in aluminum |
| `femur_body_premade_chorn.stl` | 6 | front receiver + femur/knee body |
| `tibia_socket_premade_chorn.stl` | 6 | front receiver + CF-tube socket |
| `coxa_link_ovh.stl` | 6 | flat coxa with stand + solid servo-tab block |
| `coxa_yaw_hub_carrier_ovh.stl` | 6 | deep socketed lower 6805/horn carrier |
| `hip_clamp_cap_ovh.stl` | 6 | flat yaw pedestal with stand flange |
| `hip_bearing_carrier_ovh.stl` | 6 | screw-on upper 6805 cartridge |
| `premade_chorn_56_DO_NOT_PRINT.stl` | COTS reference | the bought bracket |

The rest of `stl/` is the self-contained inherited robot print/reference set.
Files ending in `DO_NOT_PRINT` are viewer references.

## Geometry results

- 12 brackets, 12 driven pucks, and 12 passive pucks are instantiated.
- Each coxa contains its own contour-matched rear stand, one solid positive-Y
  servo-tab block, and two negative-Y contour arms. The tab block has two Ø6.8
  driver holes; the remaining upper arm retains its Ø6.5 case-screw access.
  Two rear annular buttresses now carry the upper-bearing wall down into the
  coxa from Z=44.0 to 34.5 mm without entering the servo enclosure.
- The old full-width lower rail, nine-hole grid, and two small vertical screw
  pads are gone. The lower carrier now keys 8 mm into the coxa and is retained
  by three radial edge screws.
- Both yaw-bearing protrusions are split into closed, single-body screw-on
  cartridges. The lower carrier has a 0.30 mm-clearance collar/socket joint;
  its three captive nuts are closed by the coxa after the collar is inserted.
- The upper-bearing connection uses six ordinary captive M3 nuts per leg:
  four between the hip cap and carrier, plus two between the carrier and coxa
  tower. No heat-set inserts are used at this connection.
- The removable servo clamp caps use 18 additional ordinary captive M3 nuts:
  two per hip and one per knee. Every M3×8 screw passes fully through its nut;
  the servo-case self-tappers and the knee's separate vertical screw are
  unchanged.
- Both the coxa/hip and femur/knee motor holders have a full-width Ø24 service
  slot and pass the horn-on servo extraction sweep.
- The tibia socket exposes all six bracket fasteners: four central captive
  nylocs through rounded radial windows and two outer pockets directly, while
  retaining a 21.95 mm complete tube collar at the mouth. A Ø24-to-Ø18 tapered
  root flare and four diagonal ribs bridge the receiver into that collar while
  keeping every nut window open.
- The normal knee range −30°…+20° is clear.
- The longer premade bracket first meets the rigid top structure at −117.5°;
  the build uses a conservative −110° hip up-limit, the same safe limit as the
  custom-CNC overhead concept.
- The top and bottom 6805 bearing checks, plate seating, hatch removal path,
  and yaw-axis interfaces are inherited and re-run by the generator.
- The blind tube stop is 5.2 mm farther out than the CNC-C concept and 21.8 mm
  farther out than production, so the CF tube cut remains 21.8 mm shorter than
  production. The socket mouth now extends another 16 mm along that same tube;
  this adds support without changing the cut length or foot position.

## MuJoCo load run and FEA screen

The rigid-top/bottom-bearing MuJoCo twin is the dynamic proxy for this concept:
the joint centers and foot endpoint are unchanged, while the concept CAD is
used for the structural meshes. A deterministic 10 s rollout of
`ppo_goal_cw_walk_longdist_r2.zip` completed without termination and travelled
0.39 m. Its recorded maxima were 28.64 N foot force, 3.757 N·m link bending,
2.2 N·m servo torque, and 30.774 N transmitted joint force. The FEA also uses
the existing rise/lower envelope, increasing the applied single-foot and
transmitted load to 50.715 N.

The current BuildViz full-leg screen uses material-specific models: isotropic
PETG at 25% infill (E=900 MPa, 22.66 MPa effective yield), the real hollow
8 mm OD / 6 mm ID carbon tube (55 GPa isotropic bending proxy, 300 MPa
conservative allowable), the bought aluminum C-horn (69 GPa, 110 MPa
conservative allowable because its alloy/temper is unverified), and a TPU 95A
linear secant proxy (E=25 MPa, 7 MPa screening allowable). The C-horn and tube
use fine quadratic meshes; the printed parts use repaired C3D4 meshes because
their otherwise watertight exports contain degenerate surface triangles.

The six-part foot-to-hip load-chain envelope is:

| part | peak stress | safety factor | screen |
|---|---:|---:|---|
| aluminum C-horn, worst knee context | 127.57 MPa | 0.86 vs unverified-alloy allowable | fails pending alloy verification |
| femur body | 23.93 MPa | 0.95 | fails |
| TPU 95A foot boot, linear proxy | 5.01 MPa | 1.40 | marginal |
| tibia tube socket | 15.48 MPa | 1.46 | marginal |
| coxa link | 14.60 MPa | 1.55 | marginal |
| hollow carbon tube | 188.63 MPa | 1.59 | marginal |

The aluminum result is governed by sideways bending of the 2.1 mm arms. A
simple two-arm beam check gives about 102 MPa nominal under the 3.757 N·m knee
moment; mesh refinement puts the solid-model peak higher. If the bracket is
verified 6061-T6, 127.57 MPa is below its yield strength, but the plot remains
red against 110 MPa until the commodity bracket's actual alloy and temper are
known. The hollow-tube result converged to 189-190 MPa at 0.8/0.6 mm mesh size;
its nominal beam stress is 109 MPa and the higher FEA peak is the physically
expected socket-edge restraint concentration. The foot result is only a
small-strain TPU proxy: its approximately 0.63 mm predicted displacement and
stress utilization need a hyperelastic material test before certification.

### v28 reinforced-target check

The tibia root flare/ribs and the upper-bearing coxa buttresses were checked
against v26 with the same 80-voxel repaired C3D4 mesh, the same recorded
MuJoCo + rise/lower envelope, and deterministic single-threaded CalculiX
solves. The comparison is:

| part | v26 peak / SF | v28 peak / SF | result |
|---|---:|---:|---|
| tibia socket | 15.48 MPa / 1.46 | 4.87 MPa / 4.65 | root transition fixed |
| coxa link | 14.60 MPa / 1.55 | 3.68 MPa / 6.16 | bearing-wall load path fixed |
| upper bearing carrier | 5.52 MPa / 4.10 | 6.02 MPa / 3.76 | still clear; extra nut pocket shifts the bonded-solid peak |

The carrier correction matters: it is an upper **yaw**-bearing cartridge, not
a hip-pitch link. Its radial walking reaction is resolved from the measured
2.856 N·m yaw moment across the 56.3 mm upper/lower bearing-center span
(approximately 51 N). The former automatic model divided the complete hip
moment by the cartridge's roughly 15 mm local contact spacing and created an
artificial 200+ N point load. `fea_materials.json` records the axis and
reaction span, and the result JSON records the applied load model. The fourth
rear fastener improves the physical clamp pattern, but this bonded single-part
screen does not model bolt preload or contact slip and therefore cannot credit
that benefit.

Treat all values as design screening, not certification. The assembled view
does not yet model bolt preload, hole bearing/slip, carbon laminate directions,
TPU hyperelasticity, print-layer anisotropy, creep, or nonlinear multi-part
contact. The remaining v26 concerns are the unverified-alloy C-horn, femur
knee-end transition, carbon tube socket-edge concentration, and TPU foot
model. Target a load-path safety factor above 2 before a full 25%-infill print.

Recorded artifacts:

- `mujoco_walk_loads.json`
- `fea_materials.json`
- `mujoco_walk_fea_full_leg_envelope.json`
- `mujoco_walk_fea_reinforced_targets_v28.json`
- `mujoco_walk_fea_petg25_loadpath.json`
- `mujoco_walk_fea_petg25_chassis.json`

The material-specific full-leg BuildViz page is `mujoco-walk-fea-full-leg` on
v26. The corrected changed-part page is
`mujoco-walk-fea-reinforced-targets` on v28. The older PETG-only pages remain
as historical printed-part/chassis screens.

## Regenerate and publish

From `hexapod_walker/prototype_sts3215`:

```sh
make buildviz-premade-chorn-56
```

This regenerates the geometry, runs the full femur/plate sweep and BuildViz
checks, publishes the local build on port 5183, and mirrors it to the cloud hub
without making a cloud/network failure invalidate the local result.

To regenerate only the four STEP hardware files:

```sh
uv run --no-project --python 3.12 --with build123d \
  python concepts/premade_chorn_56/build_premade_chorn_hardware_step.py
```
