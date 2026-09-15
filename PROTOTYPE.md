# Hexapod Walker — Tabletop Prototype Build Guide

Physical robots can differ from the current CAD. Their installed configurations
are tracked separately in [`robots/`](robots/README.md), beginning with
[`hexapod-1`](robots/hexapod-1.yaml).

> A scaled-down sibling of the [full-size walker](../archive/fullsize_v1/ASSEMBLY.md) intended
> for proving out the geometry, kinematics, and gait controller before
> you commit to industrial servomotors. Same architecture: regular hex
> chassis, six identical 3-DOF legs, alternating-tripod gait — but
> everything is shrunk roughly 6× and every joint is driven by a
> **FEETECH STS3215 (ST-3215-C018, 12 V / 30 kg·cm) serial-bus smart
> servo** instead of a $5000 harmonic-drive servomotor.
>
> Total parts cost: **~$615 – $1000** in 2026 USD (see
> [`docs/BOM.md`](docs/BOM.md)). A motivated builder can have
> it walking on a tabletop in a weekend.

![Cycles render of the prototype hexapod](renders/prototype.png)

---

## 1. Why a separate prototype?

The full-size walker is a serious build: a 280 kg vehicle, $40 k+ BOM,
custom CNC and casting work. Even if your end goal is a rideable
walker, you almost certainly want to:

1. **Validate the kinematics and gait controller end-to-end** on cheap
   hardware before machining anything in aluminum.
2. **Iterate on the leg link ratios** without a 6-week lead time on
   castings — you can re-print every link in 12 hours.
3. **Train the gait scheduler in simulation, then prove it on real
   hardware** at 1/200 the mass and 1/250 the cost.
4. **Show the design works** to collaborators, funders, or your spouse
   before committing to the bigger build.

The prototype is **mechanically and kinematically identical** to the
full-size walker. The same six-legged tripod gait, the same per-joint
PID + feed-forward controller architecture, even the same coxa : femur
: tibia ~ 1 : 4 : 5 ratio. You can re-use the gait code unchanged
when you graduate to the big version — just re-tune the gains.

---

## 2. Specification at a glance

| Property | Value |
|---|---|
| Configuration | 6 legs × 3 DOF, alternating-tripod gait |
| Overall envelope (foot-to-foot, fully extended) | ~ 580 × 670 × 110 mm |
| Standing height (chassis underside) | ~ 70 mm |
| Stride length | ~ 120 mm |
| Cruise speed (tripod gait) | ~ 80 mm/s |
| Vehicle dry mass | ~ 1.3 kg |
| Per-leg static load (tripod stance) | ~ 4.3 N (~ 0.43 kg) |
| Peak knee torque | ~ 0.6 N·m (~ 6 kg·cm) |
| Battery | 2 × 3S 2200 mAh shorty LiPo (11.1 V nom / 12.6 V full, ≤ 75 × 34 × 26.5 mm each) in parallel via an XT60 Y-harness, velcro'd under the belly |
| Servo rail | Raw 3S (12 V) via the central trunk pair (two 5-port Wago 221-415 at the chassis_top centre) → corner power Wago pairs (seated between tray walls printed into the chassis_bottom top face at the hex corners) → per-leg branches (as-built: no PDB, all lever nuts); see [`firmware/WIRING.md`](firmware/WIRING.md) §6 |
| Logic rail | Raw 3S straight to Uno Q (on-board regulator; **no external buck**) |
| Continuous draw (cruise) | ~ 9 A @ 12 V (all 18 servos walking) + Uno Q on its own battery feed |
| Run time, level ground | ~ 30 min |

**Knee torque margin:** the actuator is a FEETECH STS3215
(ST-3215-C018, 12 V / 30 kg·cm) serial-bus smart servo.  At 12 V it
gives a comfortable safety factor over the worst-case knee torque, and
its built-in 12-bit encoder closes the position loop and reports
load/voltage/current/temperature back over the bus.  The printed
cradles, disc-horn sandwich joints and RL servo torque limits are all
tuned around this STS3215-class case.

> **Design B (May 2026):** the printed `servo_horn_adapter` disc has
> been retired.  Each link now bolts directly onto a purchased servo
> horn.  Drops the printed-leg-bolt-up Z stack by ``HORN_ADAPTER_T``
> (4 mm) per joint.
>
> **June 2026 disc-horn switch:** the servo horn is now a **20 mm
> aluminum 25T disc** (Amazon B07D56FVK5, MG995 / MG996R type), NOT
> the plastic 4-arm X-horn.  Each link bolts onto the disc's flat top
> face via 4 x Phi 3.4 mm M3 clearance holes on a **14 mm bolt circle**
> (``XHORN_BOLT_PCD`` = 14.0, cross pattern at 0/90/180/270 deg) plus a
> Phi 9 x 2 mm central spline-collar clearance bore.  The 4 x M3 x 6
> SHCS thread DIRECTLY into the disc's M3 tapped holes -- the aluminium
> is the thread-engagement medium.  (The 14 mm value is interpreted as
> a bolt-circle DIAMETER; if calipers say it is a 14 mm square instead,
> it is a one-line change to ``XHORN_BOLT_PCD`` + the bolt angles.)
>
> **Design E (May 2026) mixed-mode cradle bolts:** each cradle has
> 4 vertical M3 x 8 SHCS driven DOWN through the servo's mounting
> tabs.  Sites split by X sign:
>   * The 2 -X bolts thread into M3 brass heat-set inserts (McMaster
>     `94459A130`) pressed into Phi 4 mm pockets inside Phi 8 mm
>     printed bosses -- real metal threads, full pull-out durability.
>   * The 2 +X bolts self-tap into bare Phi 2.5 mm pilots in the
>     existing well-wall material (no boss enlargement, no insert).
>     The Phi 8 mm heat-set boss footprint physically cannot coexist
>     with the +X wire-exit channel that the servo's molded wire
>     boot must pass through during insertion; see
>     `INSERT_M3_SELFTAP_*` in `hexapod_prototype.py` for the
>     design-decision rationale and `check_servo_insertion_path`
>     in `_verify_prototype.py` for the regression probe.
> Counts per robot: **72 x M3 x 8 SHCS** (36 -X into-insert + 36 +X
> self-tap, same physical stock for both engagement modes) plus
> **36 x M3 brass heat-set inserts** (-X column only).  Replaces
> the brief Design C captive-nyloc and Design D all-heat-set
> iterations.
>
> **Design F (May 2026) chassis_bottom-integrated yaw cradle:**
> the standalone `coxa_bracket` (orange flange + dangling servo
> well) has been folded into the `chassis_bottom` plate as a
> per-leg printed-in cradle that grows up 11 mm from the plate's
> top face.  The yaw servo drops INTO chassis_bottom rather than
> hanging from a screwed-on flange; the servo's output shaft pokes
> up between the two chassis plates via the Phi 12 mm pass-through
> cutouts in `chassis_top` (currently a no-op because the cradle
> sites sit just outside chassis_top's 70 mm apothem, but the
> cutout intent is documented in `make_chassis_top()`).  The wire-
> exit corridor and zip-tie post mirror to the cradle's -X
> (radially-INBOARD) face, putting each leg's harness mouth right
> next to the existing leg drop slot in chassis_bottom and
> shortening each yaw cable run by ~ 62 mm of Manhattan distance.
> Fastener counts per cradle are unchanged from Design E (4 x
> M3 x 8 SHCS, 2 of which thread into M3 heat-set inserts); the
> previous 24 x M3 x 14 coxa-bracket-to-chassis bolts (4 per leg)
> and the matching 4-hole-per-leg pattern in `chassis_top` /
> `chassis_bottom` are dropped.  The legacy `coxa_bracket` STL is
> retired in the final cleanup commit of this redesign.

---

## Visual inspection

Three viewers share the same `part_palette.PART_COLORS` table, so a
given part type (e.g. `coxa_link` = green, `foot_boot` = brown,
`servo_body` = dark gray) is rendered with the *same* color in each:

- `make view-static`  — MuJoCo viewer, prototype frozen in stance pose
  (now colored by part type, body labels on by default).
- `make view`         — MuJoCo viewer, full physics + tripod gait.
- `make view-build`   — PyVista build inspector, colored + labeled
  parts, exploded-view slider, per-part-type hide toggle.

Forward extra flags through `ARGS=`, e.g.
`make view-build ARGS="--explode 1.0"` opens the inspector half-way
exploded, or
`make view-build ARGS="--screenshot artifacts/views/build_inspect.png"`
renders a single headless PNG and exits.

### PyVista build inspector controls

The `view-build` window prints these to stdout on launch:

**Mouse**

- **Hover** a part → its `part_type Lx  role` is shown in the top-of-window banner. No clutter from other parts.
- **Left-click** a part → every other part dims to 15% opacity while the picked part stays in place at full opacity. The clicked part does not move, so you can see exactly how it sits in the assembly without its neighbours obscuring it.
- **Left-click in empty space** (or press `I` / `Esc`) → restores every part's opacity to its full value.
- Left-drag still rotates the camera. Right-drag pans. Scroll zooms.

**Keyboard**

| Key | Action |
|---|---|
| `L` | Toggle the persistent all-labels-at-once cloud (off by default — hover is usually less noisy) |
| `E` | Toggle exploded view between 0.0 and 1.5 |
| `I` / `Esc` | Clear isolation |
| `R` | Reset the camera view |
| `S` | Save a screenshot to `artifacts/views/build_inspect.png` |
| `Q` | Quit |

The bottom slider (0.0 – 2.0) scales each part's distance from the
chassis centroid for finer exploded-view control than the `E` toggle,
and the row of color-coded checkboxes along the left edge hides or
shows all instances of a given part type so you can isolate, say,
just the coxa links or just the servo bodies. The MuJoCo
viewers' body-name labels default to ON (toggle with `B` once inside
the viewer).

If `view-build` errors out saying STLs are missing, run
`make build` first to regenerate `stl_prototype/`.

---

## How to validate / regenerate

The STLs under `stl_prototype/` are derived artifacts.  STEP-first
(Aug 2026): each printable's shipping geometry is a build123d BREP solid
(`cad_step_test/build_step_first_test.py`, constants imported from
`hexapod_prototype.py`), exported by `build_step_prototype.py` as
`step_prototype/*.step` + tessellations and installed into
`stl_prototype/` after equivalence gates against the trimesh twin
(volume / bbox / max surface deviation — see `docs/CAD_WORKFLOW.md`).
`hexapod_prototype.py` (constants + twins) plus `design_spec.yaml`
(human-readable contract) remain the parametric truth; when editing a
printable, change BOTH the trimesh twin and the BREP builder. After any
parametric edit, run the validate → render → report pipeline from the
repo root:

```bash
make -C hexapod_walker/prototype check-cad        # full validate + render + report
make -C hexapod_walker/prototype check-cad-fast   # inner-loop variant
```

Both targets rebuild `stl_prototype/`, run the validators, render
4-view PNGs per part under `artifacts/views/`, and write
`artifacts/cad_report.md`. See [`docs/CAD_WORKFLOW.md`](docs/CAD_WORKFLOW.md) for
the full pipeline and [`docs/CAD_AGENT_INSTRUCTIONS.md`](docs/CAD_AGENT_INSTRUCTIONS.md)
for the rules LLM coding agents should follow when editing CAD.

The legacy verifier `_verify_prototype.py` is parallelised via a
process pool and exposes several CLI flags for faster inner-loop
iteration:

| Flag | What it does |
|---|---|
| `--fast` | Run only the 5 ESSENTIAL checks (watertightness, self-collision standing, fastener engagement, mating-face contact, cable clearance). ~5-15 s vs ~95-270 s for the full suite. Use between every edit. |
| `--all` | Explicit full-suite mode (same as no mode flag). |
| `--changed` | SMART SELECT: only run checks whose static input deps intersect what `git diff origin/main` changed. Rebuilds STLs in-memory to detect which printed parts actually changed bytes; consults the `CHECK_INPUTS` map. Compose with `--fast` to intersect with the essential set. Pass `--base-ref REF` to diff against a different ref. |
| `--no-cache` | Bypass the persistent per-check cache (`hexapod_walker/prototype_sts3215/.verify_cache.sqlite`). Lookups are skipped; writes still happen. Use when you've edited a verifier helper that the cache key intentionally doesn't track. |
| `--serial` | Skip the process pool entirely; run every check in the main process in declaration order. Use this when a worker traceback is mangled through pickle and you need a clean stack. |
| `--workers N` | Override the default worker count (default `min(8, os.cpu_count())`). |
| `--profile PATH` | Dump a cProfile snapshot of the parent process to `PATH` when the run finishes (combine with `--serial` to profile the entire suite in one process). |
| `--only CHECK_NAME` | Run only the named check(s). Repeatable: `--only "Servo clearance" --only "Workspace self-collision"`. Names match the declaration list (see top of `_verify_prototype.py` `CHECKS`). |

Make targets wrap the common flag combinations:

| Target | Underlying command | Typical use |
|---|---|---|
| `make verify-prototype` | `_verify_prototype.py` (full suite, cache-aware) | Pre-commit / pre-print gate. |
| `make verify-fast` | `_verify_prototype.py --fast` | Inner-loop between every edit. |
| `make verify-changed` | `_verify_prototype.py --changed` | After a small CAD edit; selects only checks affected by the diff. |

All flags compose with the existing `--with-arm` flag. Parallel output
is printed in declaration order so a `diff` against the serial baseline
is byte-for-byte clean for every `[PASS]/[FAIL]` line.

The persistent per-check cache stores `(check_name, input_hash) ->
(result, runtime_s, message, timestamp)` rows in
`.verify_cache.sqlite` (gitignored). The hash mixes the check
function's own source, every STL file in `stl_prototype/`, and the
`hexapod_prototype` + `fastener_registry` module sources. A second
run with no inputs changed hits the cache for every check and lands
in ~1-2 s end-to-end. Rows older than 30 days are pruned on every
invocation. Bust with `--no-cache` or by deleting the sqlite file.

`--changed` walks `git diff origin/main` and rebuilds every STL
in-memory to detect which printed parts ACTUALLY changed bytes (so a
cosmetic comment-only edit to `hexapod_prototype.py` selects zero
checks). It consults two maps near the top of `_verify_prototype.py`:

* `CHECK_INPUTS` -- per-check set of printed-part STL names the check
  consults. Future checks MUST add an entry; missing entries default
  to `ALL_PRINTED_PARTS` (safe but defeats selection).
* `CHECK_SOURCE_DEPS` -- per-check set of registry-source tags
  (`fastener_registry`, `cable_keepouts`, ...) that should also
  trigger the check.

If `_verify_prototype.py` itself is in the diff, `--changed` falls
back to "select all"; the cache then dedups unchanged check sources
at exec time.

### Opt-in strength / failure-point check

A SEPARATE pipeline lives under `hexapod_walker/prototype_sts3215/strength/`
that runs a closed-form Euler-Bernoulli beam-bending sanity check on
the slender links plus -- when the toolchain is available -- a
CalculiX linear-static FEA pass on every load-bearing printed part.
Output is `artifacts/strength_report.md` (with per-part stress-field
PNGs under `artifacts/strength/`).

This pipeline is **deliberately not** wired into `make check-cad`.
It's observation-only and you opt in explicitly:

```bash
make -C hexapod_walker/prototype check-strength            # PETG (default)
make -C hexapod_walker/prototype check-strength MATERIAL=pla
make -C hexapod_walker/prototype check-strength PARTS=tibia_link,femur_link SOLVER=beam
make -C hexapod_walker/prototype check-strength SOLVER=beam   # closed-form only
```

**Installing the toolchain (macOS):**

```bash
# Tetrahedral mesher (works on Apple Silicon).
brew install gmsh

# CalculiX solver (Intel macOS / Linux):
brew install costerwi/homebrew-calculix/calculix-ccx

# Python deps (gmsh, pygmsh, meshio) are the opt-in `fea` group of the
# repo-root pyproject.toml:
uv sync --group fea
```

As of May 2026 the `calculix-ccx` Homebrew formula does **not** build
on Apple Silicon (the bundled ARPACK fails to link `_dseupd_`);
`gmsh` works fine.  If `ccx` is missing the pipeline falls back to
the beam-bending check alone and notes the missing solver in the
report.  On Linux (Ubuntu) `apt install calculix-ccx gmsh` covers
both.

**What the load cases look like** (derived from the design constants
in `hexapod_prototype.py` and an inferred 2.15 kg assembled mass):

| Part | Load case | Source |
|---|---|---|
| `tibia_link`    | foot tip -Z impact = `robot_weight / 3 * 2g` | `FEMUR_LENGTH`, `TIBIA_LENGTH`, `XHORN_BOLT_PCD` |
| `femur_link`    | knee-end -Z impact = same `2g` foot load           | same + `FEMUR_LENGTH` |
| `coxa_link`     | DS3225 stall = 2.5 N-m about the yaw axis          | `COXA_LENGTH` + `DS3225_STALL_TORQUE` |
| `chassis_bottom`| dead load (battery + 6 cradle/leg masses)          | `BATTERY_UNDER_CENTRE`, yaw cradle XY |
| `chassis_top`   | dead load minus battery                            | same supports |

> **Legacy symbol names.** The opt-in `strength/` module has **not**
> been renamed to the STS3215 sandwich-joint part set: it still keys its
> load cases on the old single-spar names `tibia_link` / `femur_link` /
> `coxa_link` and on the constant `DS3225_STALL_TORQUE` (~2.5 N·m yaw
> stall, comparable to the STS3215's ~30 kg·cm ≈ 2.9 N·m). These are
> internal strength-tooling symbols only — the actual robot is driven by
> 18× FEETECH STS3215 serial-bus servos (see §2/§4). The CLI examples
> above use those legacy `PARTS=` names because that is what the module
> currently expects.

The strength module imports from `hexapod_prototype` but the reverse
is forbidden -- a missing FEA toolchain cannot break the geometry
build.

### `check_screwdriver_access` envelopes

The screwdriver-access check probes a per-fastener cylindrical
clearance cone above each head along the driver-approach axis and
fails if any printed part intrudes by more than 30 mm³. Three
envelopes are dispatched off `FastenerInstance.spec`:

| Envelope | Dia × length | Used for |
|---|---|---|
| `HEX_KEY`  | 8 mm × 30 mm  | SHCS (M3x8 / M3x14 / M3x32) + the M2.5 spline center screw — anything driven with an L-shaped hex key short arm |
| `HEX_KEY` (counter-bored) | 5.7 mm × 15 mm | `M3x6 SHCS` disc-horn bolts — recessed in a Phi 5.7 mm counter-bore on the tight 14 mm bolt circle, so the driver turns inside the bore (narrower envelope, mirrors the old M2 link-bolt special case) |
| `PHILLIPS` | 12 mm × 80 mm | `pan-head` / `Phillips` / `slotted` — none since the Aug 2026 TPU boot retired the M3x16 foot hinge bolt |
| `SOCKET`   | 12 mm × 50 mm | `nyloc nut` / generic `nut` — none since the same change retired the hinge nyloc |

Spline center screws are technically a small Phillips on hobby
servos, but every such fastener is also explicitly SKIPped (captive
under the disc horn after assembly), so they're mapped to `HEX_KEY`
purely so the per-spec envelope table reads consistently. See the
top of `check_screwdriver_access` in `_verify_prototype.py` for the
exact dispatch logic.

---

## 3. STL files in `stl_prototype/`

Run the one-command prototype builder to generate every print/export bundle:

```bash
./run.sh hexapod_walker/prototype_sts3215/build_all.py
```

The printables land in `stl_prototype/` as healed BREP tessellations
(STEP-first, Aug 2026): `build_all.py` runs the BREP exporter
`build_step_prototype.py` (which also writes the per-part `.step` CAD
files into `step_prototype/` and enforces the twin-equivalence gates)
and installs its output.  Running
`./run.sh hexapod_walker/prototype_sts3215/hexapod_prototype.py` alone only
regenerates the trimesh twins + `stl_reference/` visuals — the verifier
will flag the print set as stale until `make build` re-installs the BREP
tessellations.  All dimensions are in millimetres. All individual STLs
are sized to fit a 220 × 220 mm 3D-printer bed (Ender 3 / Bambu A1 mini
class).

### 3.1 Body parts (one of each)

| File | Function | Suggested print settings |
|---|---|---|
| `chassis_top.stl` | Top deck (2 mm PLA — halved Aug 2026 so M3 + nut / short self-tap screw mounts clamp cleanly through it; a 140 mm flat-to-flat hex CLIPPED to a Ø115 mm disk so the yaw-centred hip servo bodies sweep clear).  Completely flat both faces (Aug 16 2026: the 2 switch-holster insert bosses retired — the holster velcros down), plus the 4 `CHASSIS_STANDOFF_HOLES_XY` (±31.1) sites that take the sandwich standoffs below (M3×8 top bolts — were M3×10 with the 4 mm plate) and the magnet-post stack above (20 mm standoff + thumb nut + Ø8×8 mm magnet). | 0.2 mm layer, 25% gyroid infill, 4 walls |
| `chassis_bottom.stl` | Single merged bottom plate (Jun 2026): a flat 200 mm hex with a ~8 mm floor that carries the 6 yaw-servo cradles, the split yaw-bearing tower bases, and the velcro-strap slots (centre pair = under-belly battery safety strap).  Aug 2026 rev: each leg's 3-port ribbed harness band is now ONE open 18 × 28 mm port (Molex connectors push straight through), plus a 14 × 22 mm battery XT60 pass-through at (48, 0) on the +X vertex axis.  Late-Aug 2026: the 6 corner power-Wago tray wall sets are printed INTO the top face (the separate taped `wago_mount.stl` trays are retired — wire pull is reacted by solid plastic, and the trays print as clean vertical walls on the face-down plate).  Aug 16 2026: each corner tray is now a SINGLE press-fit bay for one 5-port Wago 221-415 (was a two-bay 3-port V+/GND pair; the bay is 0.75 mm tighter — nominal 0.15 mm interference — so the nut wedges in), and Ø9 full-stack seat pads restore the 4 brass-standoff seats that the inboard-shifted harness ports had clipped (each pad nibbles one port corner; connectors still pass).  Prints face-DOWN, no supports. | same as top |
| `switch_holster.stl` | Snap-in holster for one ~ 32 x 17 x 17 mm anti-spark on/off switch (the e-stop).  VELCROED flat to chassis_top's +X edge (Aug 16 2026: the bolt-down ear, chassis_top bosses, inserts and M3x10 bolts are retired — solid flat floor for the velcro patch).  Toggle protrudes +X past the chassis edge for user access; XT60 pigtails exit the -X end wall. | 0.2 mm, 25% infill, 3 walls |

> **Aug 2026 as-built electronics deck** (from `extra_stl/`, not `stl_prototype/`):
> four posts at `CHASSIS_STANDOFF_HOLES_XY` hold a round Ø115 **`round_mount_plate_115`**
> on magnets; the Uno Q bolts to it three-point (3× M3×8 down into M3
> thumb nuts — review round 2 folded the retired `hex_uno_q_io_board`'s
> UNO hole pattern into the plate) with the breakout beside it.  Above
> sits the **screen stand** (`hex_raised_platform_110_h28_screen` —
> "hex" is historical: round Ø115 top disc on 3 legs, 28 mm, shortened
> from 72 in the late-Aug 2026 design review to cut the lever arm on
> the magnet-held plate; mounts by 3× M3×8 driven UP from under the
> plate into blind self-tap pilots in the feet; 24×5 pigtail slot + 4×
> M2 screen pilot holes in the top disc) with the display on the top
> face and the MPU-6050 glued on chassis_top beside the central trunk
> Wagos.  Generate / refresh with
> `tools/make_xtool_hex_raised_platform.py`.  Printed trays
> (`uno_q_tray`, `buck_tray`), `spider_carapace`, chassis-top
> `imu_pad`, and the separate `hex_uno_q_io_board` are retired.

### 3.2 Per-leg parts (print 6 sets)

The leg is a **disc-horn sandwich joint** design (Jun 2026): each hip
and knee joint drives a 20 mm aluminium 25T disc horn on the servo's
output spline and reuses a SECOND disc horn on the servo's rear idler
boss for passive support, with the yoke bolting identically to both.
The tibia segment is a Ø8 mm carbon-fibre tube epoxy-bonded into printed
end-fittings.  The femur is ONE printed part, `femur_link` (Jul 2026 —
the separate printed `femur_strut`, the femur CF tube, and the two-part
yoke + knee bracket are all retired): hip moving yoke, a SOLID Ø18 spar
(the old socket-boss outer diameter) and the knee servo bracket printed
as a single body — no socket, no slip fit, no retention pin.

| File | Function | Print orientation |
|---|---|---|
| `coxa_link.stl` | ONE piece (Aug 2026 merge of the old yaw hub + hip bracket): yaw turntable hub (bolts the driven disc horn, rides the touching 6805 pair on a Ø25.15 press-fit boss) fused to the hip fixed side (symmetric servo cradle; the passive disc horn rides the servo's own rear idler boss; Aug 16 2026 — no end-face bolt holes, no wire-exit channel: a clean 4-wall box).  5 vertical head-access shafts reach the yaw horn screws through the (empty) hip servo well — M3 × 30 since Aug 16 2026; Aug 17 2026 sink pass: the hip cradle rides 5 mm lower (well floor 2 mm over the platform top), so the heads sit only ~5.3 mm below the shaft mouths; the cap-to-link labyrinth gaps widened to 1.5 mm axial / 1.0 mm radial after a bench scrape, and the horn-screw seats are deeper (corners +1.25 mm for full disc bite, centre +1 mm more into the spline tap); plus the same rear retention tab as the femur (2 × M2.5 × 6 into the hip servo's rear molded pair, inboard end, heads flush in Ø5.2 × 2 mm recesses). | On its SIDE — yaw axis horizontal, cradle end wall on the bed |
| `yaw_bearing_cap.stl` | TOP half of the SPLIT yaw-bearing tower; bolts onto each `chassis_bottom` tower with 3 x M3 x 8 SHCS to capture the touching 6805 bearing pair (12 mm ring; Ø32 x 2 mm retaining lip).  Print flat, ring face down (no supports). | Ring face down |
| `yaw_servo_retainer.stl` | Anti-rotation saddle under each yaw cradle + short ground FEET (Aug 2026 v4: four 8 mm corner poles to small Ø12 pads, 34 mm below the belly — the underside stays open so the case-back servo plug is reachable, and the wire exits ride ~9 mm off the floor when the robot belly-sits); bolts to the chassis_bottom anchor bosses and captures the hanging yaw servo body. | Flange down (flipped) |
| `femur_link.stl` | The WHOLE femur, one printed part (Jul 2026): hip moving yoke (symmetric clevis — both arms bolt to a disc horn, driven on the front, passive on the rear boss) + SOLID Ø18 spar (8 mm yoke spine plate, small cone flares at both spar ends — Aug 2026) + knee fixed side (symmetric servo cradle; rear retention tab — 2 × M2.5 × 6 into the servo's rear molded pair, holes fully enclosed since the Aug 17 2026 1 mm-longer tab, heads flush in Ø5.2 × 2 mm recesses). | Yoke spine down, spar horizontal, knee mount plate up (support the knee well through its open back) |
| `tibia_knee_yoke.stl` | Knee moving yoke (symmetric clevis — driven + passive disc horns) + Ø8 CF-tube socket. | Spine down |
| `foot_boot.stl` (**TPU 95A**) | Standard compliant foot boot pressed over every tibia CF-tube end (Aug 2026: replaces `tibia_foot_fitting` + `foot_pad` + the M3 hinge pin/nyloc). Ø8.1 bore over the Ø8 tube, same nominal slip fit as the tibia yoke socket; printed TPU lands snug, CA/epoxy dab for keeps. The boot has a nominal Ø9 rounded/spherical ground-contact end, an 8 mm solid tip section, and puts the furthest foot tip at the shared 150 mm knee-centre-to-tip station on all six legs. Extra slicer copy: regenerate `extra_stl/foot_boot.stl` with `tools/make_extra_foot_boot.py`. | Mouth face on bed, dome up (45° internal blind-end cone, nothing bridges) — no supports |

> **Femur** = `femur_link`, one printed body — nothing to join (no CF,
> no epoxy, no pins; the Jul 2026 merge fused the old hip yoke and knee
> bracket across a solid Ø18 cylinder).
> **Tibia** = `tibia_knee_yoke` + Ø8 CF tube + pressed-on `foot_boot`.
> Cut the tube to length and epoxy it into the printed socket
> (epoxy-only since Aug 2026 — no retention pin, no cross-hole).

### 3.2a Per-joint parts (print 12 of each — 1 per hip + knee joint)

| File | Function | Print orientation |
|---|---|---|
| `servo_clamp_cap.stl` | Sandwich-joint clamp cap that traps the hip/knee servo body; 2 x M3 x 8 SHCS self-tap into the cradle ±X wall-end pilots. Aug 18–19 2026: a 10 mm-wide L-shaped back-face hook (near the wire end of the +Y edge) — a 5.5 mm wall plus a shelf lapping 7.4 mm over the back of the motor toward its middle, riding flush with the back surface (the wire-end band steps up — no bite there), plus a tiny corner pad at the horn end (1 mm press-fit bite, ≥1.2 mm gap to the Ø20 passive horn) that preloads the body against the front retaining lip; the corridor beside the main hook stays open for the 5264 plugs and their cables, clear of the swinging yoke; 1 mm 45° chamfers on the outer face's vertical edges keep the sweeping yoke spine from catching a corner. | Flat |

> **`passive_horn_adapter.stl` retired (Jul 2026 stock-horn refit):** the
> passive side now uses the STS3215's own stock metal rear horn — its
> centre bore slides over the rear idler boss so it seats flush on the
> servo back face (retained by one M2.5 screw).  Nothing printed is
> needed on the passive side.

### 3.3 Visualization (do not print)

| File | Function |
|---|---|
| `assembly_preview.stl` | All parts placed in standing pose. Open in MeshLab or Cursor's STL viewer to sanity-check before printing. |

### 3.4 Print orientation

`stl_prototype/` is the print directory — every STL in it gets printed,
nothing else does.  The canonical print pose for each part (flat face on
the bed, pockets opening up) lives in `scripts/print_orientation.py`;
the verifier's flat-bottom check sweeps that registry so any orientation
regression fails CI before you waste filament.

---

## 4. Bill of materials

### 4.1 Actuators (the long pole)

| Item | Spec | Qty | Approx. cost |
|---|---|---|---|
| Serial-bus servo | **FEETECH STS3215 (ST-3215-C018), 12 V / 30 kg·cm metal-gear smart serial-bus servo**, standard ~40 × 20 × 40 mm case with a 25T output spline carrying the flush 20 mm disc horn; a real END-face 10×10 mm M2.5 hole square for body retention.  Built-in 12-bit encoder; daisy-chained on a 1 Mbps half-duplex TTL bus with closed-loop position/load/voltage/current/temp feedback.  Buy all 20 from the same batch. | 18 + 2 spare | ~$18 – $25 each ($360 – $500 total) |

### 4.2 Power

> Power architecture (Aug 2026 as-built): **two battery domains, no
> external buck, no PDB** — `battery → trunk Wagos → power Wagos →
> servos` (all Wago 221 lever nuts) and
> `battery → Uno Q` (share ground only).  Full detail in
> [`firmware/WIRING.md`](firmware/WIRING.md) §6.  There is **no servo
> BEC** — the STS3215 run directly off the raw 3S (12 V) rail.

| Item | Spec | Qty | Cost |
|---|---|---|---|
| LiPo battery | 3S 2200 mAh shorty, 25C+, XT60, ≤ 75 × 34 × 26.5 mm each (the reserved CAD envelope).  Two in parallel (XT60 Y-harness), velcro'd UNDER `chassis_bottom`'s flat belly, block yawed 30°. | 2 | $30/pair |
| XT60 parallel Y-harness | 1-female → 2-male, 12 AWG; joins the two packs into one trunk feed. | 1 | $8 |
| Industrial hook-and-loop | Adhesive Dual Lock strips, pack tops → flat belly (primary retention). | 1 | $10 |
| Velcro cinch strap | Hook-and-loop strap (~15–20 mm) through the CENTRE chassis_bottom slot pair as the under-belly safety strap. | 1 | $5 |
| Main fuse + holder | 15–20 A blade/ANL fuse + inline holder between the anti-spark switch and the trunk Wagos. | 1 | $8 |
| Per-branch fuse + holder (optional) | 5–7 A mini-blade fuse + holder, one per leg branch. | 6 | $1.50 each |
| Wago 221 lever-nuts | Compact splices — the whole power distribution (as-built: no PDB): a central **trunk** V+/GND pair of **5-port 221-415** near the chassis_top centre where the battery feed lands, **power** 5-port 221-415 nuts press-fit between the tray walls printed into the chassis_bottom top face (ONE per hex corner, entries facing inward; Aug 16 2026 — was a V+/GND pair of 3-port 221-413 per corner), and **data** under the belly at the hex vertices (r = 70; off the leg azimuths, which the battery packs occupy). | 1 pack (~20, incl. 2× 5-port) | $15 |
| 16–18 AWG silicone wire | Red + black, ~5 m each; the six per-leg V+/GND branches from the trunk / power Wagos to each leg's 5264 injection pigtail. | 1 | $10 |
| XT60 pigtails | Male/female, 12–14 AWG silicone (battery cable + trunk-Wago feed + Uno Q battery tap). | 2–3 | $4 |
| Molex 5264 connector + crimp kit | 3-pin 2.5 mm kit (~20 sets) for the per-leg injection pigtails and the leg-to-leg signal+GND-only data jumpers. | 1 | $10 |
| Anti-spark on/off switch | XT60 in/out pigtails (the e-stop).  Snaps into `switch_holster.stl`, velcroed on chassis_top's +X edge; toggle protrudes outside the chassis for user access. | 1 | $10 |
| LiPo charger | iSDT D2, B6AC, or any decent 3S balance charger | 1 | $30 |
| LiPo bag | Fire-safe charging | 1 | $10 |

### 4.3 Control electronics

> The whole control stack is a single **Arduino Uno Q** (on-board Linux
> SoC + MCU) that runs the Python gait/RL/teleop AND drives the
> half-duplex STS3215 TTL bus directly at 1 Mbps.  It replaces the old
> Arduino Mega 2560, the Raspberry Pi, the 2×PCA9685 PWM boards, AND any
> separate USB-to-TTL bus adapter.

| Item | Spec | Qty | Cost |
|---|---|---|---|
| Arduino Uno Q | On-board Linux SoC + MCU; runs Python gait/RL/teleop and drives the STS3215 bus.  Mounts on the round Ø115 `round_mount_plate_115` (with breakout) held by the four magnet posts.  Powered directly from the 3S battery (no external buck). | 1 | $50 |
| microSD card | 32–64 GB, A1/A2 rated (OS/storage if not using on-board eMMC). | 1 | $10 |
| USB-C cable | Flashing / powering / console access to the Uno Q. | 1 | $8 |
| MPU-6050 IMU (GY-521) | 6-DOF gyro + accel, I²C, powered from 3V3.  Glued on `chassis_top` beside the central trunk Wagos, near the robot centre (screen stays on `hex_raised_platform_110` top). | 1 | $4 |
| ST7789 / GMT020 display (optional) | Status screen on the raised-platform top face. | 1 | $10 |
| FEETECH 3-pin serial-bus cables | DATA daisy-chain (servos ship with one each; buy a spare pack for the chassis-to-first-servo runs). | 1 pack | $8 |
| Dupont jumper wires + heat-shrink | I²C / logic wiring cleanup | — | $10 |

### 4.4 Fasteners

The authoritative, auto-generated fastener counts live in the
**Fasteners** table of [`docs/BOM.md`](docs/BOM.md)
(edit `fastener_registry.py`, not the table).  The key items:

| Item | Qty | Notes |
|---|---|---|
| M3 × 10 SHCS disc-horn bolts (`91290A114`) | 48 | DRIVEN hip + knee front-horn bolts on a 14 mm bolt circle into the disc's M3 tapped holes (2 mm longer because the flush output seats the driven disc 2 mm lower). |
| M3 × 8 SHCS disc-horn bolts (`91290A113`) | 72 | Yaw front horn + the two passive rear-boss horns, same 14 mm bolt circle into the disc's M3 tapped holes. |
| M2.5 × 8 SHCS, servo body retention (`91290A104`) | 0 | FULLY RETIRED Aug 16 2026 — the hip cradle (the last holdout at 24) lost its end-face holes with the coxa-hub cleanup. Every servo is held by its clamp cap (with the Aug 18 back-face hook) + retaining lip + output-face seat, plus the hip/knee rear tabs' M2.5 × 6 self-tappers. The YAW cradle uses the `yaw_servo_retainer` strap. |
| M2.5 × 8 spline / passive-horn screws (`91290A104`) | 30 | Passive-horn retention + servo spline center screws (ship with the servos). |
| M3 × 8 SHCS (`91290A113`) | ~52 | 24 `servo_clamp_cap` + 18 `yaw_bearing_cap` self-tap + 4 chassis_top → brass-standoff top bolts (moved from ×10 when the top plate was halved to 2 mm) + 3 screen-stand feet (up from under the round plate into Φ2.5 pilots) + 3 Uno Q mount (down into thumb nuts). |
| M3 × 14 SHCS (`91290A115`) | 4 | Chassis-standoff bottom bolts, UP through chassis_bottom's 8 mm plate + floor stack into the F-F standoffs' bottom female threads (Jul 2026 F/F switch). |
| M3 × 20 mm F-F brass standoffs | 4 | chassis_top ↔ chassis_bottom across `CHASSIS_GAP` = 20 mm (Aug 2026: was 32 mm; shrank when the battery moved under the belly) on the 44-mm-radius diagonal pattern (±31.1, ±31.1).  Same stock as the magnet-post bases below. |
| M3 × 20 mm brass standoffs | 4 | Magnet-post bases at the same `CHASSIS_STANDOFF_HOLES_XY` (±31.1), rising above chassis_top. |
| M3 knurled thumb nuts (~2.5 mm) | 7 | 4 sit on the 20 mm posts under the magnets; 3 hold the Uno Q's three-point mount under the round plate (finger-tight). |
| Ø8 × 8 mm disc magnets | 4 | Top of each post; hold the round Ø115 mount plate. |
| 6805-2RS ball bearings | 12 | Yaw joint touching pair (2 per leg), Ø25 × Ø37 × 7 mm (61805; Aug 2026 — was 6706-2RS 30×37×4); captured by the split tower + `yaw_bearing_cap`. |
| Ø8 mm carbon-fibre tube | ~1 m | Femur + tibia leg segments (epoxy-bonded into the printed sockets). |
| Two-part epoxy | 1 | Bonds the CF tubes into the printed sockets. |

### 4.5 3D-printed material

| Item | Spec | Qty | Cost |
|---|---|---|---|
| PLA / PETG filament | 1.75 mm, any colour | ~ 1 kg | $20 |
| TPU filament | 1.75 mm, 95A | ~ 250 g (foot pads only) | $5 |
| MJF PA12 (optional) | 12 × `servo_clamp_cap` (PLA also OK) | — | print-service |

### 4.6 Total

Cost buckets mirror the **Rough Cost** table in
[`docs/BOM.md`](docs/BOM.md):

| Bucket | STS3215 build |
|---|---:|
| STS3215 serial-bus servos (20) | ~$360 – $500 |
| Arduino Uno Q + microSD + USB-C | ~$60 – $100 |
| Battery + charger + bag + switch + velcro | ~$70 – $120 |
| Power distribution (fuses + Wagos + silicone wire + 5264 kit; no PDB) | ~$30 – $50 |
| Disc horns (30) + CF tube + epoxy | ~$30 – $55 |
| Fasteners / standoffs / magnet posts / wiring | ~$30 – $55 |
| Filament + MJF clamp caps + hex plate / raised platform | ~$30 – $55 |
| **Total** | **~ $615 – $1000** |

---

## 5. Print plan

A single Ender 3-class printer runs the whole BOM in roughly **22 hours**
(see the **Printed Parts** print table + suggested order in
[`docs/BOM.md`](docs/BOM.md)):

| Pass | Parts | Time |
|---|---|---|
| 1 | 6 × `coxa_link` + 6 × `yaw_bearing_cap` | ~ 4 h |
| 2 | 6 × `femur_link` (one-piece femur) | ~ 8 h |
| 3 | 6 × `tibia_knee_yoke` | ~ 4 h |
| 4 | 12 × `servo_clamp_cap` (MJF PA12 or PLA) | ~ 3 h |
| 5 | `chassis_top` + `chassis_bottom` + `switch_holster` (+ `extra_stl` hex plate / raised platform) | ~ 6 h |
| 6 | 6 × `foot_boot` (TPU) | ~ 0.8 h |

Tip: use 4 walls and 30–40 % gyroid infill for the yokes, brackets and
the split yaw-bearing tower cap — the servo cradles and bearing tower
see the most load and the extra walls add substantial stiffness for
very little weight.

---

## 6. Assembly sequence

Allow ~ 4 hours for a first build, ~ 90 min for a second.

> **Chassis_bottom is ONE printed part (Jun 2026 merge).** It is a
> 200 mm flat hex plate with a ~8 mm-thick flat floor that carries the 6
> yaw-servo cradles, the base half of each split yaw-bearing tower, and
> the velcro-strap slots (the centre pair now carries the safety strap
> for the two under-belly LiPo packs).
> It prints face-DOWN flat with no supports.  **Yaw servo retention:**
> each yaw servo seats into its open-bottom cradle and is captured by the
> bolt-on `yaw_servo_retainer` strap + anchor bolts + the output-face
> seat on the mount plate (the yaw cradle takes **no** M2.5 end-face
> bolts — the flush-horn refit dropped the yaw output ~5.5 mm so the
> servo hangs below the chassis floor; see `docs/BOM.md`).

> **The joint is a disc-horn sandwich (Jun 2026).** Each hip and knee
> joint drives a 20 mm aluminium 25T disc horn on the servo's output
> spline and reuses a SECOND disc horn on the servo's rear idler boss
> for passive support, so the moving yoke bolts identically to a disc on
> BOTH faces — no external ball bearing on the hip/knee. The tibia is a
> Ø8 mm carbon-fibre tube epoxy-bonded into the printed end-fittings
> (epoxy-only since Aug 2026 — no retention pin); the femur is ONE printed part
> (`femur_link`, Jul 2026) — nothing to join at all.
> Full joint detail is in the **Bench Test Order** of
> [`docs/BOM.md`](docs/BOM.md).

### 6.1 Per-leg sub-assembly (do all 6 in parallel)

> **Servo body retention.** Every cradle holds its servo with the
> printed `servo_clamp_cap` (seated flush against the body's +Y face,
> retained by 2 × M3 × 8 SHCS self-tapping into the cradle's ±X wall-end
> pilots) plus the retaining lip and the output-face seat.  **End-face
> M2.5 body bolts are fully retired** (Aug 16 2026 — the HIP cradle's
> 4-per-leg set was the last and went with its holes; they were never
> installed on the bench and the empty holes only weakened the wall.
> The KNEE cradle lost its set in the Jul 2026 one-piece femur — the
> fused Ø18 spar covers that wall — and the YAW cradle in the Jun 2026
> flush-horn refit; see the chassis_bottom note above).  There are
> **no heat-set inserts and no vertical tab bolts** anywhere on the
> leg.

> **Hip/knee servo removal (Sep 2026).** Remove the two M3 clamp-cap
> bolts, then lift the servo with its fitted disc horn and clamp cap toward
> the open clamp face. The former rounded bridge over the horn is now a
> straight, 24 mm-wide continuation of the horn opening, so that assembly has
> a monotone lift-out path. The service slot removes one small front-case
> capture location; three remain, together with both clamp-cap bolts and the
> two rear-tab screws.

1. **Join the leg segments first.**  FEMUR: nothing to do — `femur_link`
   comes off the printer as one finished part (Jul 2026 one-piece merge;
   no CF, no epoxy, no pins).  TIBIA:
   cut the Ø8 mm carbon-fibre tube to length and epoxy it into the
   `tibia_knee_yoke` socket (epoxy-only since Aug 2026 — the pin
   cross-hole is gone).  Let the slow-cure epoxy fully set
   before loading the joints.
2. **Hip-pitch servo into the coxa link's hip cradle** — *but only
   AFTER the coxa is bolted to its yaw disc horn* (Aug 2026 one-piece
   coxa: the 5 yaw horn screws drop down head-access shafts that open
   into this well, so the well must still be empty in final-assembly
   step 2 below; do steps 6.2-1/6.2-2 for this leg first).  Then seat
   the servo into the `coxa_link` hip cradle, snap on the
   `servo_clamp_cap` and secure it with 2 × M3 × 8 SHCS self-tapping
   into the cradle wall-end pilots (Aug 16 2026: the 4 × M2.5 end-face
   bolts and their −X-wall holes are RETIRED — cap + lip + output-face
   seat hold the body).  Then drive 2 × M2.5 × 6 self-tappers through
   the cradle's rear retention tab (the vertical plate against the
   servo's back face, inboard end) into the servo's rear molded hole
   pair; the heads seat flush in their recesses — do this BEFORE the
   femur goes on, the yoke arm later swings ~1.5 mm over the tab face
   (Aug 17 2026, matching the femur's knee tab).
3. **Disc horns on the hip servo:** push a 20 mm aluminium 25T disc
   horn onto the output spline at 0° and retain it with the servo's
   M2.5 spline screw; slide the STOCK metal passive horn (ships with
   the servo) over the rear idler boss — its centre bore rides the
   boss so it seats flush on the back face — and retain it with one
   M2.5 screw (Jul 2026 stock-horn refit: no printed adapter).
4. **Femur onto the hip joint:** seat the `femur_link`'s two hip clevis
   arms onto the driven (front) and passive (rear) disc horns and bolt
   each arm to its disc on the Ø14 cross — 4 × M3 × 10 SHCS per horn
   (driven front and passive rear alike) into the discs' M3 tapped
   holes.  Use a 2.5 mm hex key.
5. **Knee servo into the femur's knee cradle:** drop the servo into the
   `femur_link`'s knee cradle, snap on the `servo_clamp_cap` and secure
   it with 2 × M3 × 8 SHCS into the wall-end pilots (NO end-face bolts
   at the knee — the fused spar covers that wall; the cap + lip hold the
   body), drive 2 × M2.5 × 6 self-tappers through the rear retention
   tab under the servo's back face into the rear molded hole pair
   nearer the spar (heads seat flush in their recesses; do it before
   the tibia yoke goes on), then fit the
   driven + passive disc horns as in step 3.
6. **Tibia onto the knee joint:** seat the `tibia_knee_yoke`'s clevis
   arms onto the driven + passive knee disc horns and bolt each arm to
   its disc (4 × M3 × 10 per horn) as in step 4.
7. **Foot boot:** press the TPU `foot_boot` over the free end of each
   tibia CF tube until the tube bottoms in the 20 mm socket (Ø8.1 bore
   over the Ø8 tube, same slip fit as the yoke socket — a drop of isopropyl helps
   it slide, then grips when dry).  No fastener, no hinge (Aug 2026:
   the boot's compliance replaces the old passive hinge).  All six legs
   use the same standard boot and the same 150 mm knee-centre-to-tip
   geometry.

You now have a complete leg (yaw pad → hip → femur → knee → tibia →
foot) ready to drop onto its `chassis_bottom` yaw cradle. Repeat 6
times.

### 6.2 Final assembly

1. **Yaw servos + split bearing towers into chassis_bottom:** lay
   chassis_bottom flat.  Seat each yaw servo into its open-bottom
   cradle and secure it with the `yaw_servo_retainer` strap + anchor
   bolts (no M2.5 end-face bolts on the yaw cradle).  Fit the yaw
   driven disc horn on the output spline.  Assemble the **touching
   6805 bearing pair**: drop the LOWER 6805 race into chassis_bottom's
   open-top Ø37 pocket (its top 3 mm stands proud — that's by design),
   slide both inner races + the loose `yaw_bearing_cap` onto the
   one-piece `coxa_link`'s hub boss from below, set the boss into the
   tower, then pull the cap down with **3 × M3 × 8 SHCS** self-tapping
   into the tower pilots, sandwiching both outer races between the
   chassis seat and the cap lip.
2. **Coxa horn bolts (Aug 2026 one-piece coxa):** with the hip servo
   NOT yet in its cradle, drop the **4 × M3 × 30 SHCS + 1 central M3
   spline screw** down the five head-access shafts that open into the
   empty hip-servo well, and torque them into the yaw disc horn (Aug 17
   2026 sink pass: the well floor sits just 2 mm over the hub platform,
   so the heads are only ~5.3 mm below the shaft mouths — any stubby
   driver works.
   The corner seats are 1.25 mm deeper so the tips get the full 2 mm
   disc bite on the printed part, just breaking the disc's far face;
   the centre seat is 1 mm deeper still, into the spline tap).  Then
   lower the hip servo into the cradle
   (its body covers the shaft mouths) and clamp it.  Each completed
   leg now hangs from its yaw axis.
3. **Battery:** stick Dual Lock strips to the flat belly and to the
   tops of the two 3S 2200 mAh shorty packs, press them on side by side
   with the pack block yawed 30° (long axis toward the leg-0 edge-mid;
   XT60 leads at the +axis end), then thread the safety cinch strap
   down through one CENTRE slot (x = −8), under both packs, and back up
   the other.  Push both packs' XT60 leads nose-first UP through the
   14 × 22 mm battery pass-through at (48, 0) and plug them into the
   XT60 Y-harness above the plate; the Y-harness output is the main
   XT60 that feeds the anti-spark switch.
4. **Chassis standoffs + top plate:** stand 4 × M3 × 20 mm F-F brass
   standoffs on the 44-mm-radius diagonal pattern
   (`CHASSIS_STANDOFF_HOLES_XY` = (±31.1, ±31.1)).
   Drive **4 × M3 × 14 mm SHCS** UP from below chassis_bottom's floor
   face, through the 8 mm plate + floor stack, into each standoff's
   bottom female thread.  Drop `chassis_top.stl` onto the standoff tops
   and drive **4 × M3 × 8 mm SHCS** DOWN from above into the
   standoffs' female top threads.
5. **Switch holster on chassis_top:** velcro the `switch_holster.stl`
   flat to chassis_top's +X edge (toggle pointing outboard past the
   chassis vertex).  (Aug 16 2026: the old boss-insert + 2 × M3 × 10
   bolt-down mount is retired — no inserts, no bolts.)
6. **Electronics deck (magnet plate + raised platform):** at the four
   `CHASSIS_STANDOFF_HOLES_XY` (±31.1) sites above chassis_top, stack
   a **20 mm M3 standoff + ~2.5 mm M3 thumb nut + Ø8×8 mm magnet**.
   VHB-tape a **5-port 3.3 V Wago** flat to the UNDERSIDE of the round
   **`round_mount_plate_115`** (from `extra_stl/`) at (0, −36), wire
   entries facing the south rim.  While the plate is still off the
   robot, bolt the **screen stand** to it — **3 × M3 × 8 SHCS driven UP
   through the plate's three foot holes into the blind Φ2.5 pilots in
   the stand feet** — and mount the **Arduino Uno Q** three-point:
   **3 × M3 × 8 down through the board + plate into M3 thumb nuts**
   (finger-tight; breakout beside it).  Seat the plate assembly on the
   magnets.  Feed the Wago from
   the Uno Q **3V3 pin** down through the plate's **east Ø8 wire port**
   (19, −44); branch **MPU VCC** straight down and **screen VCC** back
   up through the west port.  Status screen on the stand's top disc; glue
   the **MPU-6050 on chassis_top just south of the central trunk
   Wagos** (header row facing the open -X deck so the jumpers have
   room).  Route IMU I²C (GND, SDA, SCL; VCC comes off the 3.3 V Wago)
   to the Uno Q (see `firmware/WIRING.md` Stage F).  Zip-tie the wires
   crossing the magnet interface at the plate's E/W rim slot pairs so
   the plate lifts off cleanly.  Press ONE **5-port power
   Wago (221-415)** into the tray walls printed into chassis_bottom's
   top face at each of the 6 hex **corner flats** (wire entries facing
   the chassis centre; Aug 16 2026 — single press-fit bay, 0.15 mm
   nominal interference: push firmly until it seats on the plate; no
   separate tray, no tape); park **data Wagos** under the chassis near
   the yaw retainers.
7. **Wire it up:** see §7 and `firmware/WIRING.md`.  Two battery
   domains, no buck, no PDB: `battery → trunk Wagos → power Wagos →
   servos` and
   `battery → Uno Q`.  Bench-test all 18 joints on a current-limited
   supply BEFORE the LiPo (WIRING.md Stages A–F).

---

## 7. Wiring

> **Wiring of record: [`firmware/WIRING.md`](firmware/WIRING.md).**
> That doc is the buildable, step-by-step plan (two power domains, the
> half-duplex serial bus, the distributed fused power harness, and the
> bench bring-up checklist). This section is only a summary; when the
> two disagree, `firmware/WIRING.md` wins. See also `docs/BOM.md`
> for the parts.

The robot is **18× FEETECH STS3215** (ST-3215-C018, 12 V / 30 kg·cm)
**serial-bus** smart servos driven **directly by an Arduino Uno Q**
(on-board Linux SoC + MCU). There is **no Raspberry Pi, no PCA9685 PWM
boards, no DS3225 PWM servos, and no FE-URT-1 / Waveshare bus adapter**
— the Uno Q's UART drives the half-duplex TTL bus itself at 1 Mbps.
Each servo has a built-in 12-bit encoder and reports
position/load/voltage/current/temperature back over the bus, so there
are no external encoders.

### 7.1 One-servo bench test first

Before assembling the robot, bring up **one STS3215 on a
current-limited bench supply** (not the LiPo) so a wiring mistake trips
the supply instead of cooking hardware. This proves the exact servo you
bought talks on the bus before you bolt all 18 joints. Full procedure:
`firmware/WIRING.md` §0–§4 (Stages A–C).

In brief, with the bus signal wired to the Uno Q UART (per the FEETECH
half-duplex convention) and 12 V on the bus V+ rail, common ground:

```bash
python -m pip install feetech-servo-sdk pyserial
python motor_setup/feetech_bus.py --port /dev/ttyACM0 scan          # expect [1] (factory ID)
python motor_setup/feetech_bus.py --port /dev/ttyACM0 setid --from 1 --to 1
python motor_setup/feetech_bus.py --port /dev/ttyACM0 joint 0 15 --sweep
python motor_setup/feetech_bus.py --port /dev/ttyACM0 feedback      # reads back angle, volts, temp
```

On the Uno Q's Linux side the port is usually `/dev/ttyACM0` or
`/dev/ttyUSB0` (on a laptop it may be `/dev/cu.usbmodem…`). The
yaw / hip / knee joint order for one leg is validated by re-ID'ing the
three leg-0 servos to IDs 1/2/3 and chaining them (`WIRING.md` Stage D)
— there are no PCA channels to plug into, just unique bus IDs.

### 7.2 Full robot wiring

There are exactly **two domains** that share only ground (full detail
+ ASCII diagrams in `firmware/WIRING.md` §1, §2 and §6):

```
  DATA (one half-duplex TTL bus, daisy-chained, low current):
  Uno Q ─sig+GND─► (underside data Wagos) ─► yaw→hip→knee per leg …
                        leg-to-leg jumper = SIGNAL + GND only (no V+)
  Uno Q MCU Wire SDA/SCL (D20/D21) ─► MPU-6050 (chassis_bottom, phys. leg 1)

  POWER — two battery domains (share ground only; NO external buck, NO PDB):
  3S LiPo ─┬─XT60─► anti-spark switch ─► MAIN FUSE 15–20 A ─► central trunk Wagos (V+/GND pair of 5-port 221-415)
           │         trunk Wagos ─► corner power Wago pairs (12V+G, printed mounts) ─► per-leg branches
           └─► Uno Q VIN (on-board regulator)
```

Why the power bus is **distributed** and not chained: the FEETECH
3-pin bus connector (Molex 5264) is rated ~3 A/pin, but all-18 walking
current is ~9 A and would melt the first upstream connector. So **DATA
stays chained, POWER is injected per-leg from the trunk Wagos via
the corner Wagos** — no single 5264 pin ever carries more than one leg (~1.5 A
walking). The leg-to-leg data jumper carries **signal + GND only
(V+ omitted)**. The Uno Q takes its own battery tap (no buck on the
servo rail). The anti-spark switch is the e-stop. See
`firmware/WIRING.md` §6 for the per-branch current budget, wire gauges
and fuse sizing.

### 7.3 Per-leg harness recipe

With the serial bus, each leg is just **three STS3215s daisy-chained
with stock 3-pin bus cables**; the leg's bundle drops through that
leg's `chassis_bottom` drop slot into the inter-plate volume, where it
meets its **power branch from the bus bar** (V+/GND) and the **data
jumper** to the next leg. There are **no per-servo runs back to a
driver board** — the old PWM extension-cable star is retired. The
printed strain-relief features (zip-tie posts at each cradle wire-exit
+ the per-leg drop slot) still apply: anchor each servo's cable at its
cradle post and the full 3-cable bundle at the drop slot so a leg snag
mid-walk can't yank a connector.

The cable lengths are modelled by `motor_setup/wire_harness_plan.py`
(per-joint serial-bus reach budget: cradle wire-exit → leg drop slot →
electronics-tray bus landing, plus a +30 mm slack loop per joint axis
the harness crosses). Print the BOM with:

```bash
python -m motor_setup.wire_harness_plan
```

It emits one row per servo (joint, leg + axis, servo ID, cradle exit,
drop slot, bus landing, minimum path length, and how many 30 cm
extensions that joint needs on top of the stock STS3215 bus lead).
Every joint currently fits inside the stock bus lead. Re-run after any
electronics-tray or chassis geometry edit; the verifier check **[18]
Harness reach** (`_verify_prototype.check_harness_reach`) asserts the
printed reach budget still covers the geometry.

**Slack-loop sizing** (aim for at least this much "extra" bundle at
each joint axis the harness crosses; err larger — extra slack is
harmless, a deficit fights the kinematics):

* Yaw axis:  ~20 mm slack loop (the bundle twists ~90° across the yaw
  axis at full sweep).
* Hip axis:  ~30 mm slack loop (the hip-pitch range plus the
  `coxa_link`'s WELL_Z_DROP_EXTRA = 4 mm vertical hop).
* Knee axis: ~20 mm slack loop (largest single-axis sweep, but the
  bundle only crosses it on the knee cable, anchored close to the
  joint by the `femur_link` knee-cradle zip-tie post).

---

## 8. Software

There is no PWM pulse-width layer: the **Arduino Uno Q** runs Python on
its Linux side and drives the half-duplex STS3215 TTL bus directly at
1 Mbps via the host-side driver `motor_setup/feetech_bus.py`. Each servo
takes an absolute **position command** (12-bit, count 2048 = 0°) and
reports position/load/voltage/current/temperature back over the same
bus, so the joint loop is closed inside each servo — you command angles,
not microseconds.

The driver maps the 18 logical joints to servo IDs 2..19 (ID 1 left free)
(`ID = leg*3 + axis + 1`), enforces the safe per-axis angle limits
(yaw ±35°, hip −80…+40°, knee −20…+150°), applies per-joint trims from
`motor_setup/feetech_trims.json`, and sync-writes goal positions while
reading back live feedback:

```python
from motor_setup.feetech_bus import FeetechBus

bus = FeetechBus(port="/dev/ttyACM0")   # Uno Q UART → TTL bus
bus.centre()                            # all joints → 0°

# command one joint (degrees; clamped to the safe per-axis limit):
bus.write_joint(joint=2, deg=60)        # leg 0 knee toward stance

# sync-write all 18 goal positions per gait tick:
bus.write_all(goal_degrees)             # list of 18 angles

# read live feedback (angle / load / volts / temp / current):
print(bus.read_feedback(joint=2))
```

Wrap that in a ~ 50 Hz loop that runs the closed-form 3R inverse
kinematics for each leg (coxa yaw, hip pitch, knee pitch — see e.g. the
"Phantom X / Lynxmotion AX hexapod" references) and a tripod-gait
scheduler, sync-writing all 18 goal positions per tick. The same
Python gait/IK code runs on your laptop (bench, via a USB→TTL lead) or
on the Uno Q on-robot — only `--port` changes. See `firmware/WIRING.md`
§2b for how the Uno Q sits in the loop and `feetech_bus.py` for the
full command set (`scan`, `centre`, `joint`, `stance`, `relax`,
`hold`, `feedback`, `trim`).

---

## 9. Tuning notes

* **Per-joint trim:** the first thing you do after assembly is drive
  every joint to 0° (`feetech_bus.py … centre`) and null out any
  mounting offset with `feetech_bus.py … trim <joint> <deg>` (clamped
  ±30°, saved to `feetech_trims.json` and re-applied on every command).
  Mechanical disc-horn/spline mounting lands within ~ one spline tooth,
  so expect a few degrees of trim per joint. Unlike the old PWM servos
  the STS3215 is **absolute** — it knows its angle the instant it
  powers on (12-bit encoder), so you can `relax` it, pose by hand, and
  `feedback` reads exactly where it is (no blind centre-on-boot slam).
* **Travel range:** the safe per-axis limits (yaw ±35°, hip −80…+40°,
  knee −20…+150°) are enforced in software by `feetech_bus.py`. Sweep
  each joint slowly with `joint <j> <deg> --sweep` to confirm no
  mechanical binding before the software limit; fix offsets with
  `trim`, not by re-bolting the horn.
* **Gait period:** start at 2 s per cycle (very slow, easy to debug),
  speed up to 1 s once the IK and trim are dialled.
* **Power supply sag:** if the robot collapses momentarily during
  swing-to-stance transitions, a per-leg power branch or the bus-bar
  feed is undersized. Verify the distributed harness against
  `firmware/WIRING.md` §6 — heavier 16–18 AWG branches, solid Wago
  joins, and the 15–20 A main fuse — and watch each servo's
  reported voltage with `feetech_bus.py … feedback --watch` under load
  (the STS3215 run on the raw 3S rail, so a sag shows up directly in
  the per-servo voltage). The Uno Q is on its own battery tap (separate
  from the Wago→servo path) so servo-rail sag is less likely to brown it
  out.

---

## 10. Migration to the full-size walker

The prototype's value is that **everything you build above the joint
level transfers unchanged** to the full-size walker:

* The IK math is identical (same coxa / femur / tibia ratio).
* The gait scheduler is identical.
* The per-leg state machine (stance / swing / lift / re-plant) is
  identical.
* The body kinematics + IMU fusion are identical.

What changes when you scale up:

* The actuators talk EtherCAT or CAN instead of the prototype's
  half-duplex serial bus, so the joint driver layer changes.
* Joint torque limits are 100× higher, so the gait scheduler needs to
  add a torque-aware planner (don't command a stride that requires
  more than 70 % of peak knee torque).
* Safety: a 280 kg vehicle needs an E-stop, brake-on-fault behaviour,
  and a "freewheel only when stationary" supervisor — none of which
  matter on a 1.3 kg tabletop unit.

Plan to spend ~ 3 months walking on the prototype, ironing the gait
and IK out, before you cut metal for the full-size build.

---

## 11. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Robot "twitches" but won't move | Servo-rail sag, a loose bus-bar/branch crimp, or a blown leg fuse | Check the distributed power harness (`firmware/WIRING.md` §6); watch per-servo voltage with `feetech_bus.py … feedback --watch` under load |
| A servo is silent on the bus / `scan` misses an ID | Duplicate or unset servo ID, or a broken DATA jumper to that leg | Re-ID that servo alone (`setid`), and confirm the leg-to-leg signal+GND jumper is intact |
| One leg drags | Per-joint trim is off | Re-trim with `feetech_bus.py … trim <joint> <deg>` (the encoder is absolute, so this is a software offset, not a re-bolt) |
| Knee servos overheat | Gait is over-loading (too high a chassis, too long a stride) | Lower chassis by re-targeting `STANCE_FEMUR_DEG = -15`, shorter stride |
| Robot drifts sideways | Tripod legs not lifting in sync | Increase swing duration; verify `trim_deg` for hip-pitch is consistent across all 6 legs |
| Foot slips on floor | TPU pad too smooth | Cut a small section of bicycle inner tube, glue inside the pad |

---

## 12. License & Disclaimer

Same as the parent project — personal exploration, no license declared
yet. The prototype is mechanically benign (small servos, small
batteries, no rider) but **3S LiPos can fight back if you puncture
them** — charge in a fire-safe bag and don't leave them charging
unattended.
