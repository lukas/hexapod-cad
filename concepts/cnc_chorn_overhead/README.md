# cnc_chorn_overhead — outboard hip pivot + CNC aluminum C-clamps, legs-over-head workspace

CONCEPT BUILD (not production).  User, Aug 2026: "ok can you make a
version of this with metal CNCed C clamps?  I'd also like to make them
a little bigger so this hexapod can put its legs over its head."
"This" = `concepts/rigid_hip` (single tower-seated bottom 6805, 6 mm
top plate with dust roofs, hip axis at world z 39.65, femur up-limit
−47.5° because the femur hits the plate).

**Architecture (user course-correction, Aug 2026): the variant coxa
link positions the hip servo 42 mm further OUTBOARD radially, so the
up-swinging femur clears the top plate's rim because the pivot sits
beyond it.  The CNC C-clamps stay, but as a PLAIN C — no plate-dodging
features.**  (A previous rev of this concept instead carved a swan-neck
blade profile + notch + cap shave out of a measured plate-contact map
at the production hip station — superseded: overcomplicated machining,
thin blade corridor in the load path.)

**Result (measured by this build's sweep): femur up-limit −47.5° →
−110.0° (20° past vertical), first contact −117.5° at every tested yaw
(0, ±20, ±35) — the physics ceiling, see below — all rigid-hip checks
still green.**

## What "C-clamp" means here (repo evidence)

`cad_step_test/build_chorn_step.py` + `docs/CHORN_VARIANT.md` define the
"C-horn variant": a **C-shaped bracket that clamps the servo output
disc AND the opposite idler disc at the hip/knee joints**, replacing
the printed clevis/yoke arms (the original experiment assumed a
*bought* aluminum C-horn plus printed adapters and spacers).  This
concept keeps that meaning but **CNCs a custom C-clamp** instead of
buying one: reach grows from the experiment's 29.5 mm (axis → web
inner face) to 42.0 mm, and the loose spacers become integral pads.
One design serves both hip and knee — 12 identical clamps per robot.

## Why outboard-pivot beats the swan-neck

The production/rigid coxa put the hip axis only 12.5 mm outboard of
the yaw axis — right at the plate rim — so anything on the femur with
swing radius over ~40 mm crossed plate structure on the way up.  The
swan-neck rev survived there by carving the clamp around a contact
map: a notched blade, a ~4.5 × 3 mm load-path corridor per blade, a
beveled web.  Moving the pivot out instead:

* **Simpler machining** — the plain C is rectangles + fillets; no
  contact-map notch pocket, no corridor, no bevel.
* **No thin blade in the load path** — the swan-neck corridor pinched
  the blade→web section to ~4.5 × 3 mm; the plain C keeps the full
  33 mm blade depth end to end.
* **The printed coxa does the reaching** — PETG arm in bending, cheap
  to print and iterate, instead of aluminum carved to a contact map.

Downside: the whole leg beyond the coxa moves 42 mm outboard, and the
yaw joint pays for it (quantified below).

## Sweep-derived sizing (why 42 mm)

The plain clamp's low corners meet the plate's yaw-tower collar (a
yaw-INVARIANT contact — the collar is centred on the leg's own yaw
axis) at −110° for ext 36, −117.5° for ext 41, and recede past −117.5°
from ext 42 up.  The KNEE-end parts saturate at first contact
**−117.5° at ANY offset**: past ~−102° the knee end dips below the
plate-top plane 31–51 mm inboard of the hip axis, and the plate's Φ44
dust-roof collar over the yaw bearing sits in that window for any
offset up to ~+60 (it recedes at the same rate the dip window grows).
**That is the physics ceiling.**  COXA_EXT = 42 is the minimum at
which the saturating knee end — not the clamp — is the limiter; more
offset buys nothing but yaw-joint moment.  The one overhead feature
kept from the swan-neck rev is the single planar **WEDGE chamfer** on
the printed knee block + cap, which holds the knee-end swing radius
under the collar; no offset removes the need for it.

## Parts

| part | material | mass | replaces |
|---|---|---|---|
| `chorn_clamp_cnc` (×12) | CNC 6061-T6 | 48.0 g | printed yoke arms of `femur_link` (hip end) / `tibia_knee_yoke` (knee) |
| `coxa_link_ovh` (×6) | PETG | 64.2 g | `coxa_link_rigid` (cradle arm moved 42 mm outboard; yaw-axis interfaces untouched) |
| `hip_clamp_cap_ovh` (×6) | PETG | 42.3 g | `hip_clamp_cap_rigid` (jaw moves out with the servo; yaw-axis pedestal + 6805 boss stay on the yaw axis, tied by a two-step arm) |
| `femur_ovh_body` (×6) | PETG | 43.6 g | `femur_link` (the knee block half; the hip half IS the clamp now) |
| `tibia_ovh_socket` (×6) | PETG | 14.0 g | `tibia_knee_yoke` tube socket half |
| `knee_clamp_cap_ovh` (×6) | PETG | 25.7 g | `knee_clamp_cap` (wedge tail + new up-screw retention) |

## Quantified consequences of the outboard pivot

All measured by `report_outboard_geometry` / the check suite
(`sweep_report.json` has the raw numbers), rigid_hip → this concept:

* **Hip axis radial station** (from body centre): 115.4 → 156.6 mm.
* **Stance footprint**: each foot moves 42 mm out along its leg ray —
  stance foot lever about the yaw axis 204.3 → 246.3 mm; across-flats
  stance width grows ~84 mm.  (Wider stance = more roll/pitch
  stability margin, at the cost of floor space.)
* **Yaw joint moment (the price)**: rotating mass per leg 386.3 →
  438.6 g, its cg lever about the yaw axis 45.4 → 86.1 mm, first
  moment 17 543 → 37 743 g·mm (**2.15×**) — that is what lateral
  accelerations and stance shear put through the yaw servo, tower
  bearing and coxa hub.  Point-mass yaw inertia 1.56 → 4.50 kg·cm²
  (**2.88×**): yaw swing accelerations cost ~3× the servo torque, so
  expect slower yaw ramps in gait tuning.  Stance loads on the tower
  bearing scale with the foot lever: **+21 %** bending.
* **Ground reach / down-swing**: femur down-limit +30° and knee range
  unchanged (checked); femur/tibia lengths unchanged, so leg-local
  reach is identical — the whole workspace annulus just shifts 42 mm
  outboard.  Robot height unchanged (z-stack untouched).
* **Adjacent-leg clearance**: worst-case convergence (+35°/−35° facing
  yaws) measured at stance/+30/−60/−110 femur: min gap **17.9 mm**
  (at stance).  Free yaw range of the full leg vs everything static:
  **±60°** (software limit ±35°).
* **The rigid_hip 38.2 mm rotating-envelope rule is superseded**: the
  variant coxa's plan reach is 78.4 mm.  Replacement guarantees, all
  measured in `check_yaw_envelope_ovh`: ±60° free range, adjacent-leg
  gaps above, and ≥ 41.8 mm to the centre wago block across the range.
* **Mass**: +53.6 g/leg → **+321.7 g/robot** vs rigid_hip (576 g of
  aluminum total in 12 clamps).  Static hip torque from the knee-side
  clamp (48 g at ~80 mm): ~0.04 N·m, ~2 % of an STS3215 stall —
  negligible; overhead poses put the leg CG above the hip, so gravity
  torque reverses there.
* **New PETG cantilever**: the hip cap's pedestal arm carries the top
  plate's 6805 load path (plate → 6805 → boss → arm → cap crown → hip
  cradle) across the 42 mm offset — full pedestal-chord section, but
  it is printed PETG; watch it on the bench.

## The clamp (plain C)

Two 3 mm rectangular blades (y −20…+13, full length) with integral Φ19
ring pads land ON the nominal disc faces (span 38.04), R13 rounded end
over the disc, joined by a full-height 3.8 mm web whose **outer face
is DATUM A** — it mates the frozen production femur wall plane
(x = 58.3 in link coords), so every downstream printed part keeps its
production interface.  R3 plan fillets, R2.5 web/blade gussets —
nothing smaller than R2 anywhere.  Same production disc-bolt pattern:
4× M3 on Ø14 PCD per disc face (8/clamp), threading into the metal
discs' factory-tapped holes.

## Web joint (clamp → printed part), per joint

* 2× **M3×10 SHCS** through the printed wall into **M3-6H tapped**
  holes in the web (upper row — reachable through the open servo well).
* 2× **M3×12 CSK** from the metal side (90° csk sunk in the web's inner
  face) into **T-slot nylocs** in the printed part (lower row — lives
  under the cradle floor where SHCS heads can't be reached).

## Knee cap retention (the wedge ate the old boss)

The overhead wedge chamfer removes the old inboard side-screw boss, so
the variant cap hangs a Φ4×6 heat-set insert boss under the tongue
line, bridged by a riser + a strap riding 0.3 over the servo top; an
**M3×25 SHCS up-screw** comes from the cradle underside into the
insert.  (The swan-neck rev also shaved the cap's crown; the outboard
pivot made that shave unnecessary — the wedge-only cap first contacts
at −117.5°, same as the block.)

## Workspace (measured by `sweep_femur_envelope`)

| | production | rigid_hip | cnc_chorn_overhead |
|---|---|---|---|
| femur up-limit (safe, 7.5° grid margin) | −80° | −47.5° | **−110°** |
| femur down-limit | +30° | +30° | +30° (unchanged) |
| tibia spin checked | −30…+20 baked | same | −50…+40 clear |

First contact −117.5° at every yaw (knee block vs the plate's Φ44
dust-roof collar — the saturating physics ceiling); the clamp's own
first contact (fixed-side 6805/pedestal stack) is at −125°.  −110° is
the deepest safe limit this architecture can reach at any offset.

## Z-stack (unchanged from rigid_hip)

Deck/case top 10.25 → race 10.75…17.75 → hip axis **39.65** → plate
74.05…80.05 (hatch above).  Nothing in this concept moves the stack;
the growth is all in-plane.

## CNC quoting notes (chorn_clamp_cnc.step)

* Material 6061-T6, qty 12 + spares, ~48.0 g finished, stock
  ≈ 65 × 40 × 60 mm.
* 3-axis: blades, pads, disc bores and plan profile all machine along
  the disc axis (±z, one flip); the 4 web holes drill/tap from the
  side (x) in one extra fixture.  Internal corners: plan fillets R3,
  web/blade gussets R2.5 — nothing smaller than R2 anywhere, no
  contact-map features.
* Datums: **A** = web outer face (mates the printed wall — flatness /
  perpendicularity to the disc axis matter most), **B** = pad faces
  (disc span 38.04), **C** = Φ8 horn centre bore.
* Critical: pad span 38.04 is the NOMINAL disc-face span.  The PETG
  yoke ran `YOKE_SEAT_INTERF` squeeze per side; **metal must not
  preload the servo horn** — quote ±0.05 and expect a bench-tune shim
  pass (see needs-bench-tuning in `design_spec.yaml`).
* Tapped M3-6H ×2 per clamp; Φ3.4 + 90° CSK ×2 sunk in the web inner
  face.
* **No press fits in the metal** by design: pads land on the disc
  faces, collar recesses run 0.5 clearance.  Every bench-tuned PETG
  interference value stays in PETG parts and is flagged
  needs-bench-tuning in the spec rather than translated to aluminum.

## BOM delta vs rigid_hip (per leg)

* − prints: `femur_link`, `tibia_knee_yoke`, `knee_clamp_cap`,
  `coxa_link_rigid`, `hip_clamp_cap_rigid`
* + CNC: 2× `chorn_clamp_cnc`
* + prints: `coxa_link_ovh`, `hip_clamp_cap_ovh`, `femur_ovh_body`,
  `tibia_ovh_socket`, `knee_clamp_cap_ovh`
* + hardware: 4× M3×10 SHCS, 4× M3×12 CSK, 4× M3 nyloc, 1× M3×25 SHCS,
  1× Φ4×6 heat-set insert
* tibia CF tube: **16.6 mm shorter** than production (new socket mouth
  sits outboard so the tube tip lands 0.3 off the metal web)
* disc bolts: same 4× M3 pattern per face — verify grip length at
  assembly (blade+pad stack is 7.0/7.5 mm vs the yoke's arm stack)

## Assembly order (one joint)

1. Bolt the clamp to the printed part first (2× SHCS into the taps,
   2× CSK into the T-slot nylocs) — DATUM A face-to-face.
2. Offer the assembly to the joint: pads onto both disc faces.
3. 4+4 disc bolts into the factory-tapped discs.
4. Knee only: drop the cap in, M3×25 up-screw from the cradle
   underside into the insert.

Hip station: the coxa horn screws drive first (through the plate's
access holes, checked line-of-sight), then `hip_clamp_cap_ovh` drops
over the trimmed hub top — same removable-cap service logic as
rigid_hip.

## Print set (`stl/`) — the complete part set for this robot version

`stl/` presents the FULL part set to build this variant robot,
rigid_hip-style: slice the PETG parts, quote the CNC part from
`step/`, and treat `*_DO_NOT_PRINT.stl` as COTS visuals.

* **Variant printables (PETG, this concept)**: `coxa_link_ovh` ×6,
  `hip_clamp_cap_ovh` ×6, `femur_ovh_body` ×6, `tibia_ovh_socket` ×6,
  `knee_clamp_cap_ovh` ×6.  Print each like the rigid_hip /
  production part it replaces (see rigid_hip's Print notes and the
  production print list); the parts keep their ancestors' wall
  planes and seat geometry, only reach/wedge features moved.
* **CNC — do not print**: `chorn_clamp_cnc_CNC_6061.stl` is the
  tessellated viewer stand-in for the aluminum clamp.  The quoting
  deliverable is `step/chorn_clamp_cnc.step` (×12 + spares, with
  `step/cnc_chorn_manifest.json` and
  `step/cnc_chorn_step_first_bundle.zip` — send the zip).
* **Inherited rigid_hip printables** (byte-identical geometry to
  `concepts/rigid_hip`; print per its Print notes):
  `chassis_bottom_rigid`, `chassis_top_rigid`, `top_hatch_rigid`,
  `centre_wago_block`.
* **Unchanged production printables**: `foot_boot` ×6,
  `yaw_servo_retainer` ×6 — print from the main `stl_prototype/` set;
  the copies here are for the viewer.
* **COTS / cut stock (do not print)**: `bearing_6805`, `servo_body`,
  `wago5`, `yaw_bearing_upper` visuals, plus `tibia_tube_ovh` — the
  CF tibia tube cut 16.6 mm SHORTER than production (see BOM delta).
  Fastener deltas are in the BOM section above.

## Pipeline

```sh
# full run: BREP export (uv 3.12 subprocess) + checks + sweep + scene
uv run python concepts/cnc_chorn_overhead/make_cnc_chorn_variant.py
# fast re-check without re-export / without the sweep
uv run python concepts/cnc_chorn_overhead/make_cnc_chorn_variant.py \
    --skip-brep --skip-sweep
```

Outputs: `step/*.step` (CNC quoting deliverable + manifest + zip
bundle), `step/stl/` tessellations, `stl/` baked scene set,
`scene.json` + `preview.png` (BuildViz), `sweep_report.json`.
Geometry truth lives ONLY in `build_cnc_chorn_step.py` (constants
imported from `make_cnc_chorn_variant.py`); `_probe_outboard.py` is
the plan-raster probe that seeded the COXA_EXT search (the final value
was set by sweeping the real solids — see the COXA_EXT block in the
driver).

## Build & view (BuildViz)

This concept is a sibling BUILD inside the `prototype_sts3215` hub
PROJECT — grouped with the production robot and the reinforcement test
kits, not a standalone project.  Build id:
**`prototype_sts3215/cnc-chorn-overhead`**.

```sh
# publish to the local hub (run from hexapod_walker/prototype_sts3215):
npx buildviz push --project prototype_sts3215 --build cnc-chorn-overhead \
    --upload-assets --scene concepts/cnc_chorn_overhead/scene.json \
    -m "<what changed>"
# view: http://127.0.0.1:5183/?project=prototype_sts3215&build=cnc-chorn-overhead

# cloud mirror (BUILDVIZ_API_KEY from the k8s secret -- see AGENTS.md):
uv run ../../tools/push_cloud_buildviz.py \
    --build-id prototype_sts3215/cnc-chorn-overhead
# https://buildviz.cwd1f0-new-cluster.coreweave.app/?project=prototype_sts3215&build=cnc-chorn-overhead
```

(`design_spec.yaml` next to the scene rides along automatically.  The
original standalone `cnc_chorn_overhead` project id was retired
2026-08-27 when the build moved into the STS project; the local entry
was deleted, but the cloud hub has no delete endpoint, so a stale
`cnc_chorn_overhead` entry lingers there — ignore it.)
