# fasteners/ -- explicit fastener geometry

This directory holds the fastener CAD models that the build inspector
and the verifier consume.  Every fastener instance enumerated by
`fastener_registry.build_all_fastener_instances()` resolves through a
McMaster-Carr part number to one cache STL in this directory.

## Default: NopSCADlib via OpenSCAD

`make regen-fasteners` calls the OpenSCAD CLI on each
`fasteners/scad/<spec_id>.scad` file (one per registry spec) and
exports an ISO-accurate STL.  NopSCADlib does the modelling --
ISO 4762 / DIN 912 socket-head caps, ISO 7045 / DIN 7985 pan heads,
DIN 985 nylon-insert lock nuts -- so the cached mesh has the
correct head diameter, cap height, hex socket, and nylon collar
without us having to maintain handwritten primitives.

Install the toolchain once per machine:

```bash
# macOS (Apple Silicon needs the cask; the formula's Intel-only):
brew install --cask openscad
xattr -r -d com.apple.quarantine /Applications/OpenSCAD-2021.01.app
which openscad           # /opt/homebrew/bin/openscad
openscad --version       # OpenSCAD version 2021.01

# Linux / others -- the formula or distro package both ship the CLI.

# NopSCADlib goes into the OpenSCAD user-library directory so
# include <NopSCADlib/...> resolves without -I flags.
OSCAD_LIB="$HOME/Documents/OpenSCAD/libraries"
mkdir -p "$OSCAD_LIB"
git clone --depth 1 https://github.com/nophead/NopSCADlib.git \
    "$OSCAD_LIB/NopSCADlib"
git -C "$OSCAD_LIB/NopSCADlib" rev-parse --short HEAD  # pin in CI
```

The on-disk layout `make regen-fasteners` produces:

```
fasteners/
  scad/
    m2x8_shcs.scad        # one .scad per registry SPEC_*
    m3x8_shcs.scad
    m3x32_shcs.scad
    m3x16_pan.scad
    m2p5x8_shcs.scad
    m3_nyloc_nut.scad
  91290A005.cache.stl     # one per registry part number
  91290A005.cache.source.txt  # breadcrumb: "openscad NopSCADlib c9baa0e"
  ...
```

The `.cache.source.txt` breadcrumb records which stage of the
priority chain (below) produced the cached STL.  Useful when
debugging "did I actually get a NopSCADlib mesh or did it silently
fall back to the parametric placeholder?".

## Source-priority chain

For each unique McMaster part number, `scripts/make_fastener_meshes.py`
looks in priority order:

1. `fasteners/<part_number>.step` -- a real McMaster STEP download
   (highest fidelity; requires `python-occ` so `trimesh.load()` can
   read STEP).
2. `fasteners/<part_number>.stl`  -- a user-supplied STL override.
3. `fasteners/scad/<spec_id>.scad` -- NopSCADlib geometry rendered
   via the OpenSCAD CLI.  **This is the new default in 2026 onward.**
4. `fasteners/_parametric.build_for_spec("<spec>")` -- the
   handwritten parametric fallback.  Used only if (a) OpenSCAD
   isn't installed, OR (b) the NopSCADlib render failed.
   Visualization only.

Whichever source wins gets exported to
`fasteners/<part_number>.cache.stl`.  At runtime the inspector +
verifier load ONLY the cache STL, so swapping a real STEP into place
later just requires re-running `make regen-fasteners`.

## Currently bundled cache STLs

| Part number | Spec | Used for |
|-------------|------|----------|
| 91290A005   | M2 x 8  SHCS, black-oxide steel | **RETIRED (June 2026 disc-horn switch).**  Was the link-to-X-horn self-tapper into the plastic 4-arm X-horn's Phi ~ 2.0 mm arm holes.  The robot no longer uses the plastic X-horn -- each link now bolts to a **20 mm aluminum 25T disc horn** (Amazon B07D56FVK5) with **4 x M3 x 6 SHCS (`91290A111`)** threaded into the disc's M3 tapped holes on a 14 mm bolt circle.  The `m2x8_shcs` cache STL is still bundled but unused; an `m3x6` SCAD cache has not been generated yet, so the registry falls back to parametric geometry -- run `make regen-fasteners` to add a dedicated cache. |
| 91290A113   | M3 x 8  SHCS, black-oxide steel | **cradle servo-mount bolts (72)** -- driven vertically through each servo ear; the 36 -X bolts thread into a brass heat-set insert (`94459A130`) in the cradle boss (`M3x8 SHCS into heat-set insert`), the 36 +X bolts self-tap into a Phi 2.5 mm printed pilot (`M3x8 SHCS self-tap`).  Same SKU for both uses; the distinct spec strings let the verifier / BOM tell them apart. |
| 94459A130   | M3 brass knurled heat-set insert, Phi 4 mm pilot, Phi 5.7 mm OD, 5 mm length | **cradle servo-mount thread carriers (72)** -- one per cradle bolt.  Installed BEFORE the servo is dropped into the cradle with a soldering iron at ~ 220 deg C, light downward pressure, ~ 10-15 s per insert.  May 2026 fix for the cradle pilot's radial-engagement bug; see the Design D blurb in PROTOTYPE.md. |
| 91290A123   | M3 x 30 SHCS, black-oxide steel (treated as M3 x 32 in the BOM -- swap to 91290A126 if you want exactly 35 mm) | coxa-bracket-to-chassis bolts |
| 92010A130   | M3 x 16 pan-head Phillips, A2 stainless | foot hinge pins |
| 91290A104   | M2.5 x 8 SHCS, black-oxide steel | servo spline center screws |
| 90576A102   | M3 nylon-insert lock nut (DIN 985), A2 stainless | coxa-bracket-to-chassis through-bolts (24) + foot hinge pins (6).  Not used in the cradle (Design D: cradle bolts thread into brass heat-set inserts). |

## Cradle bolts thread into M3 brass heat-set inserts (Design D, May 2026)

The 72 vertical M3 x 8 SHCS (`91290A113`) that thread DOWN through
each servo ear now engage an **M3 brass heat-set insert** (McMaster
`94459A130`) pressed into the cradle boss below, instead of
self-tapping into a Phi 2.5 mm printed pilot in the shelf material.
This is **Design D** in the PROTOTYPE.md design-history blurb and
fixes the May 2026 cradle radial-engagement bug.

* The audit (see PROTOTYPE.md Design D) measured the plastic
  remaining between the OLD Phi 2.5 mm self-tap pilot wall and the
  nearest air gap in the +/-Y direction at all 12 cradle sites; 7
  of 12 sites had **0.00-1.50 mm of plastic** on at least one side
  (femur and coxa-link knee/hip outer walls don't extend past the
  servo body footprint, and all three +X columns collided with the
  existing wire channel).  The original `check_cradle_pilot_holes`
  only verified the pilot cylinder existed; it never probed
  radially.
* Switching to a heat-set insert (a) gives real metal threads
  (brass, M3 x 0.5 standard pitch) so the joint survives repeated
  dis-assembly without stripping the PLA / PA12 pilot, and (b)
  forces a Phi `CRADLE_BOSS_OD` = 8 mm printed boss around every
  pilot, which automatically supplies the missing radial material.
* The Phi 4 mm pocket is `INSERT_M3_PILOT_OD` x
  `INSERT_M3_PILOT_DEPTH` = 4 mm x 6 mm (1 mm overdrill on the
  insert's 5 mm body length).  The insert top sits ~ 0.5 mm below
  the boss top so the bolt head clamps the servo ear onto the
  printed plastic, not the brass face (standard heat-set practice).

**Install:** heat a soldering iron to ~ 220 deg C, drop a 94459A130
insert (knurled end down) into the Phi 4 mm pocket, apply light
downward pressure with the iron tip for ~ 10-15 s until the insert
top is ~ 0.5 mm below the boss top, then cool ~ 30 s before
threading the M3 x 8 SHCS in.  McMaster ships a one-page install
guide with every insert order (see
<https://www.mcmaster.com/94459A130/>).

**Verifier checks:** `_verify_prototype.check_cradle_insert_pockets`
probes (a) the Phi 4 mm x 6 mm pocket void, (b) an 8-azimuth radial-
material ring at r = `INSERT_M3_PILOT_OD/2 + CRADLE_BOSS_MIN_WALL_MM`
= 3.5 mm at the pocket top and bottom (this is the check that would
have caught the original bug), and (c) an 8-azimuth knurl-
displacement ring at r = 2.42 mm so the boss has enough plastic for
the brass knurl to displace when the iron is applied.  All three
sub-checks must pass at every 12 (cradle, site) combination.

## Link-to-disc-horn bolts thread into the aluminum disc (June 2026 disc-horn switch)

The 72 link-to-horn bolts no longer self-tap into a plastic X-horn.
Each servo joint now drives a **20 mm aluminum 25T disc horn**
(Amazon B07D56FVK5, MG995 / MG996R type) seated on the 25T spline,
and the printed link bolts to the disc's flat top face with **4 x
M3 x 6 SHCS** on a `XHORN_BOLT_PCD` = 14 mm bolt circle (cross
pattern at 0 / 90 / 180 / 270 deg).  The bolts pass through the
link pad's 4 x Phi `XHORN_BOLT_OD` = 3.4 mm M3 clearance holes and
thread **directly into the disc's M3 tapped holes** -- the aluminum
is the thread-engagement medium (no self-tap into plastic, no
heat-set insert).  M3 x 6 keeps the screw from bottoming past the
disc's tapped depth; the M3 x 6 screws ship with the disc-horn
10-packs, and `91290A111` is the McMaster equivalent for spares.

A Phi `DISC_HORN_COLLAR_OD` = 9 mm x `DISC_HORN_COLLAR_DEPTH` = 2 mm
central bore in the link pad clears the disc's spline collar / the
M2.5 spline center screw head so the pad seats flat on the disc.

The cradle bolts (M3 x 8, `91290A113`) are mechanically symmetric
but ORTHOGONAL to this -- they live in printed shelf material with
brass heat-set inserts, not in the aluminum disc.  Both rows stay
in the BOM.

> Legacy note: the old plastic 4-arm X-horn scheme used 72 x M2 x 8
> SHCS (`91290A005`) self-tapped into the X-horn's Phi ~ 2.0 mm arm
> holes.  That part is retired; see the bundled-cache table above.

## Axis convention

`fastener_registry.FastenerInstance` reports each fastener with:

* `head_world_xyz` -- the centre of the head's outboard face (the
  underside of an SHCS cap that bears on the printed part, or the
  outboard face of a nyloc nut where one is used).
* `axis_world` -- a unit vector pointing FROM the head INTO the
  material (the direction the bolt is driven in; for a nut, INTO
  the wall that holds the nut pocket).

`scripts/inspect_build.py`'s `_axis_to_transform()` then builds a 4 x 4 that
maps the **mesh's local +Z axis** onto `axis_world` and puts the
mesh origin at `head_world_xyz`.  Every cached STL in this directory
must therefore obey:

* Origin = head mating face / nut outboard face.
* +Z = body axis pointing INTO the material.

`fasteners/_parametric.py` already builds its meshes in that frame
(see its top-of-file docstring).  NopSCADlib does NOT: its
`screw(type, length)` module puts the head at +Z and the shank at
-Z, and `nut(type, nyloc=true)` puts the steel body at z = [0, t]
with the nyloc collar above it.  `scripts/make_fastener_meshes.py` therefore
reflects the OpenSCAD-rendered mesh across the XY plane (negate Z,
invert face winding) before writing the cache, so the cached STL
matches the parametric convention exactly.  After this fix:

| Spec | Cached STL bounds (Z) |
|------|----------------------|
| M2 x 8  SHCS    | z in [-2.0,  +8.0] mm  (cap at z<0, shank at z>0) |
| M3 x 8  SHCS    | z in [-3.0,  +8.0] mm |
| M3 x 30 SHCS    | z in [-3.0, +30.0] mm |
| M3 x 16 pan     | z in [-2.0, +16.0] mm |
| M2.5 x 8 SHCS   | z in [-2.5,  +8.0] mm |
| M3 nyloc nut    | z in [-4.0,   0.0] mm  (body at z<0, free side at z=0) |

If you author a new `.scad` recipe under `scad/`, follow the
NopSCADlib convention there (it's the natural one for OpenSCAD);
`_orient_for_registry()` in `scripts/make_fastener_meshes.py` is the single
point of conversion.

## Override with a real STEP

McMaster's product pages are rendered client-side and require a
logged-in session for the CAD download endpoint -- a plain
`WebFetch` of `https://www.mcmaster.com/<PART>/` only returns the
top-level catalog chrome, not the CAD link.  The reliable path is
the manual download:

> 1. In a normal browser, visit `https://www.mcmaster.com/<PART_NUMBER>/`,
>    e.g. <https://www.mcmaster.com/91290A113/>.
> 2. Click **Product Detail**, scroll to the **Drawings and 3D Models**
>    panel.
> 3. Click **3-D STEP**.  (If you want STL instead, pick **3-D STL**.)
> 4. Save the file as `fasteners/91290A113.step` (or `.stl`).
> 5. Run `make -C hexapod_walker/prototype regen-fasteners` to
>    refresh the cache.  `scripts/make_fastener_meshes.py` will pick up the
>    STEP at priority 1 and the breadcrumb will read `step:91290A113.step`.
> 6. Run `make -C hexapod_walker/prototype check-cad-fast` to
>    revalidate.

The same recipe applies to every part number in the table above.
You can drop in real STEPs incrementally; the NopSCADlib renders
cover any missing ones until you do, and the parametric primitives
are the final safety net.

## The parametric fallback

`_parametric.py` builds each spec from `trimesh.creation` primitives:

* `make_m2_shcs(length)` -- M2 cap head (Phi 3.8, h 2.0) with a 1.5 mm
  hex socket carved into the top, plus a Phi 2 shank of the requested
  length.
* `make_m3_shcs(length)` -- M3 cap head (Phi 5.5, h 3.0) with a 2.5 mm
  hex socket carved into the top, plus a Phi 3 shank of the requested
  length.
* `make_m25_shcs(length)` -- M2.5 cap head (Phi 4.5, h 2.5) with a 2 mm
  hex socket, plus a Phi 2.5 shank.
* `make_m3_pan_head(length)` -- M3 pan head (Phi 6.0, h 2.4) with a
  Phillips cross slot, plus a Phi 3 shank.
* `make_m3_nyloc_nut()` -- DIN 985 hex body (AF 5.5, h 2.4) + nylon
  insert ring (Phi 4.7, h 1.6) + Phi 3 through bore.

The parametric meshes are visualization-only -- they are NOT load-
bearing or thread-accurate.  The verifier probes them as bounding
cylinders / cones, so visual fidelity at inspector zoom is all that
matters.  We keep `_parametric.py` around exclusively as the
last-resort fallback for users who don't want to install OpenSCAD.

## Why `91290A123` for the M3 x 32 chassis bolts

McMaster sells M3 SHCS in 1 mm length increments.  The exact stock
that bridges the 16 mm flange + 4 mm chassis plate + 4 mm nyloc nut
budget (= 24 mm engaged + 8 mm head exposure = ~32 mm bolt) is
`91290A124` (M3 x 35 mm; closer to the actual budget).  We use
`91290A123` (M3 x 30 mm) above because the fastener_registry treats
this entry as "the next-stocked length" and it is what is currently
installed in the build photos.  When the real bolt length is
finalised, edit `PN_M3X32_SHCS` and `length_mm` in `fastener_registry.py`
**and** the length in `fasteners/scad/m3x32_shcs.scad`, then re-run
`make regen-fasteners`.

## Pinned NopSCADlib commit

The OpenSCAD renders that ship in this repo were generated against:

    NopSCADlib @ c9baa0ed0faa23e849141c3d8c6728545d6af910

Re-pin this SHA whenever you upgrade NopSCADlib; if the upstream
changes a screw/nut macro signature, the OpenSCAD render will
either error out at regen time (the priority chain then falls back
to the parametric stub and prints a loud `WARN`), or silently
change the geometry -- a quick `view-build` diff against the
previous screenshot will catch the latter.
