# Horn compression-limiter experiment

This is a print-and-bench-test sidecar for the smooth, springy looseness seen
at the robot's horn joints. It does **not** change the production print set.

The variant widens only the four perimeter M3 horn-screw passages in:

- `femur_link`: both hip-yoke arms;
- `tibia_knee_yoke`: both knee-yoke arms;
- `coxa_link`: the four long yaw horn-bolt columns.

The redesign uses the user's existing standard M3 spacers, fixed at **10.00 mm
long**, currently **Ø5.5 mm OD × Ø3.2 mm ID**. There is one sleeve around each
perimeter horn screw; no sleeves are cut or stacked. The screw head/washer
bears on one sleeve end and the aluminum disc horn bears on the other. Plastic
still locates and surrounds the tube, but it no longer has to preserve screw
preload indefinitely.

## Changing to a different spacer diameter

Edit only `spacer_config.toml`. The important current settings are:

```toml
[spacer]
enabled = true
outside_diameter_mm = 5.50

[printed_fit]
bore_diametral_allowance_mm = 0.30
```

Set `enabled = false` for the normal no-spacer version. In that mode the
generator uses the original production M3 passages and omits the enlarged
bores, local spacer bosses, metal-spacer models, and fit coupon. The saved
dimensions are ignored until the switch is turned back on.

The generator sets the printed CAD bore to `spacer OD + diametral allowance`.
Thus the current Ø5.50 mm spacer produces the physically verified Ø5.80 mm
second-hole bore. The 0.30 mm value is total diameter allowance—nominally
0.15 mm per side—not the actual looseness expected in the finished print.
The sideways FDM hole prints smaller and rougher than its CAD diameter.

Changing `outside_diameter_mm` also shifts all five coupon holes while keeping
the selected bore as the second hole. Leave the 0.30 mm allowance alone unless
a new physical coupon test shows that a different value fits better.

## Why the bore is larger than the Ø5.5 mm sleeve

The first fit coupon was wrong: its holes printed vertically, while all three
real test parts print their sleeve bores horizontally. Horizontal FDM holes
lose substantially more clearance at the unsupported ceiling. The old
Ø5.05/5.15/5.25 coupon therefore predicted a fit that the parts could not
deliver.

The corrected coupon reproduces the real 10.1 mm-long horizontal tunnel and,
with the current configuration, spans Ø5.60–6.40 mm. The current experimental
parts use the user-confirmed second coupon size: a Ø5.80 mm CAD bore.
This is intentionally a clearance fit. The M3 screw passing through the
nominal Ø3.2 mm sleeve ID centers the metal sleeve on the horn thread; the
printed hole does not. Once tightened, the sleeve carries the clamp load.

The standard horn's raised center spline boss is on the servo-facing side, not
between the printed link and horn. On the printed-link side, the four sleeves
fit the standard Ø14 mm bolt pattern and remain 0.25 mm inside the nominal
Ø20 mm disc edge. The yaw variant grows only its printed horn-facing neck from
Ø20 to Ø22 mm to restore wall around the wider bores; it still has 1 mm radial
clearance through the production Ø24 mm chassis opening.

## How the fixed 10 mm length is accommodated

- **Hip and knee yokes:** the current assembled head-to-horn stack is 9 mm.
  Each of the four screw sites gets a local 1.1 mm-tall, Ø8 mm head-bearing
  boss. This leaves a 10.1 mm free plastic stack around the fixed 10 mm sleeve:
  the first 0.1 mm of tightening lightly compresses the print, then the sleeve
  becomes the hard metal stop. The rest of each arm stays its current
  thickness, minimizing sweep-envelope changes.
- **Yaw:** only the four perimeter bolt seats move down. Their new free
  head-to-horn distance is also 10.1 mm. Each existing screwdriver shaft is
  continued down to the new seat, so the normal hex driver still reaches it.
  The center spline screw is unchanged and does not receive a sleeve.

The perimeter hardware becomes **M3×12 SHCS + a 7 mm OD × approximately
0.5 mm standard M3 washer**. The washer must be wider than the Ø5.8 mm printed
bore so it overlaps the plastic as well as the sleeve. With a 10 mm sleeve this
gives about 1.5 mm engagement in the 2 mm aluminum horn and keeps the screw tip
about 0.5 mm short of the servo-side horn face. M3×10 cannot be reused: it
would have zero thread engagement through a 10 mm sleeve. The yaw access
shafts are widened to Ø7.2 mm for the washer.

The replacement fit coupon has five horizontal bores. A notch marks the small
end; read them from that end toward the other end:

- Ø5.60 mm;
- Ø5.80 mm;
- Ø6.00 mm;
- Ø6.20 mm;
- Ø6.40 mm.

Choose the smallest bore that accepts the actual tube with a light push or
slip fit after normal string cleanup. Do not drill, hammer, or hard-press the
sleeve into the coupon. Print a complete part only if its Ø5.80 coupon hole
fits; if the first fit is Ø6.00 or larger, update the generated part bore first.

## One-joint validation

1. Print the fit coupon and choose the bore for the actual tube stock.
2. Print **one** tibia yoke first; it is the cheapest complete test part.
3. Deburr eight existing 10.00 mm sleeves and confirm an M3 screw passes
   freely through every sleeve. Do not shorten them. Test all four sleeves on
   the actual horn bolt pattern before installing screws.
4. Assemble one driven and one passive horn using M3×12 screws and the thin
   washers. Confirm at least 1.3 mm and no more than 1.8 mm of actual screw
   projection beyond the sleeve/washer stack before installing against a horn.
   Verify by hand that the printed pad contacts the horn face before final
   torque and that no tube holds the pad visibly off the horn.
5. Use very modest torque because the horn has only about 2 mm of aluminum
   thread. Add a tiny amount of removable blue threadlocker to the metal
   threads only, and add witness marks across each screw head and the print.
6. Hand-rock the joint, then repeat after several heat/load cycles. A witness
   mark that stays aligned while play returns points away from screw rotation
   and toward horn spline, servo gearbox, or bearing/pocket motion.
7. Only after the yoke result is good, print one yaw coxa variant. Use one
   unchanged 10 mm sleeve, one **7 mm OD washer**, and one M3×12 screw
   at each of its four perimeter stations. Dry-assemble and recheck screw-tip
   projection so no tolerance stack can bottom a screw against the servo case
   below the thin horn.

Do not resume gait testing until the repaired test leg passes the hand-rock
check. This experiment addresses plastic clamp creep; it does not repair a
stripped horn thread, worn spline, gearbox backlash, or loose bearing fit.

## Regenerate and inspect

From the repository root:

```bash
make -C hexapod_walker/prototype_sts3215 buildviz-horn-compression-limiters
```

The generated printable candidates are under `stl/`, and the BuildViz scene
is published as `prototype_sts3215/horn-compression-limiters`. The target also
runs `verify_workspace_variant.py`, which substitutes the three experimental
parts into the production collision checker and gates a 76-pose coarse
yaw/hip/knee workspace sweep.
