"""Raised hex platform: thin top hex + legs that land on the 110 mm plate.

Aug 2026 (user: another hexagonal platform 42 mm above the small hexagon,
thin hexagon with 42 mm legs out of each side, feet within the lower hex
radius so they can attach to it).

Aug 2026 follow-up (user: move supports to the corners, very close to
the corner): legs sit on the six VERTEX rays instead of flat midpoints.

Aug 2026 screen stand (user: longer legs + wire slot on the thin/short
side of a 63×35 mm panel): also emits
``hex_raised_platform_110_h28_screen.stl`` (legs ``hp.HEX_RAISED_LEG_H``,
wire slot only — no screen window; panel sits centered on the solid top).

Aug 2026 follow-up: +10 mm height (62→72), legs 2× thick, top face
perfectly flat (legs stop flush with top), wire slot tucked under the
screen's +X short edge so its Y span matches the screen (35 mm) and
wires drop behind the panel.

Aug 2026 round-plate rework (user: make the mount plate circular to match
the chassis_top disc below, keep the holes): the LOWER plate is now a
ROUND Ø115 disc (= ``hp.HEX_MOUNT_PLATE_DIAM``, same as chassis_top),
plus two Ø8 wire ports for the underside 3.3 V Wago feed/branches
(``hp.MOUNT_PLATE_WIRE_PORT_XY``) and two pairs of zip-tie slots on the
E/W rim for dressing the wires that cross the magnet interface.

Late-Aug 2026 design review:
  * The legacy 49.5 mm bolt square (``hp.ELEC_CHASSIS_MOUNT_HOLES_XY``,
    the pre-standoff chassis generation) is DROPPED -- only the live
    +/-31.1 standoff/magnet-post square remains.
  * Magnet SHEAR REGISTRATION: four printed bosses on the plate
    UNDERSIDE (washer-shaped, OD ``REG_BOSS_OD``, bore ``REG_BORE_D``,
    ``REG_BOSS_H`` tall) socket the top 2 mm of each Ø8 post magnet, so
    the magnets only carry pull and every gait cycle's sideways shake
    goes into the boss walls instead of sliding the deck.  The plate
    still seats directly on the magnet top faces -- stack height is
    unchanged.
  * Screen-stand legs dropped 72 -> 28 mm (the 72 mm tower put the
    robot's highest mass on a magnet-held 2 mm plate).  Leg height
    single source: ``hp.HEX_RAISED_LEG_H``.

Late-Aug 2026 review, round 2 (this file's current form):
  * The stand TOP is a ROUND Ø115 disc matching the plate and
    chassis_top discs below (was a 110 hex -- the "hex" in the file /
    part names is historical now, kept so the loader + viz partType
    stay stable).
  * THREE legs at az 90/210/330 (was 6): a 20 g screen does not need
    six; the south rim (wire ports + underside 3.3 V Wago) stays clear
    of feet.  Each foot carries a Φ2.5 BLIND SELF-TAP PILOT rising into
    the leg blade, and the stand mounts with 3x M3x8 SHCS (the same
    91290A113 stock as the clamp caps) driven UP from under the plate
    -- this CLOSES the old spec gap where 6 M3 went through clearance
    holes into nothing (6 unlisted nuts).
  * The plate gains the Arduino-UNO mounting pattern for the Uno Q at
    the as-built ``hp.UNO_Q_ON_HEX_CENTRE`` = (0, -12): THREE of the
    four standard holes (the SE hole would land ~1.5 mm from the SE
    registration boss, so it is dropped -- three-point mount, M3x8 down
    through the board + plate into an M3 thumb nut below, same thumb
    nut stock as the magnet posts).  This folds the one useful feature
    of the retired ``make_hex_uno_q_io_board.py`` into the live plate.
  * RETIRED outputs: the 42 mm base platform variant (nothing as-built
    used it), the ``hex_uno_q_io_board_110`` board, and the plate's SVG
    cut file (the underside registration bosses cannot be laser-cut;
    shipping a boss-less cut file was a footgun).

Geometry
--------
  * Lower plate: ROUND Ø115 disc, 2 mm thick, matching the chassis_top
    disc below.  Coxa-sweep keep-out starts at r ≈ 59.6 so the 57.5
    radius keeps ≥2 mm (asserted at runtime).
  * Upper plate: ROUND Ø115 disc, 2 mm thick, raised so its BOTTOM sits
    ``LEG_H_SCREEN`` mm above the TOP of the lower plate.  Top face is
    planar (print top-down or feet-down — no stubs).
  * 3 legs on vertex azimuths 90/210/330, vertical blades.
  * Feet: 12 mm pads with Φ2.5 blind self-tap pilots (M3x8 from below
    the plate).
  * Screen variant: solid top (no window) + 24×5 mm wire slot under the
    +X short edge of a centered 63×35 screen (GMT020 / ST7789) — outer
    X edge = screen +X edge, narrowed so the +X screw holes clear it —
    plus 4 Ø1.8 M2 self-tap pilot holes at the panel corner holes
    (2.5 mm inset) to hold the screen down.

Outputs under ``extra_stl/`` (printables only -- no cut files):
  * ``hex_raised_platform_110_h28_screen.stl`` -- 28 mm legs, wire slot
  * ``hex_raised_platform_110_h28_screen_preview.png`` -- top + side
  * ``round_mount_plate_115_with_leg_holes.stl`` -- the magnet plate

Run from the repo root:

    python hexapod_walker/prototype_sts3215/tools/make_xtool_hex_raised_platform.py
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import trimesh
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..")))

import hexapod_walker.prototype_sts3215.hexapod_prototype as hp  # noqa: E402
from hexapod_walker.prototype_sts3215.tools.make_xtool_hex_mount_plate import (  # noqa: E402
    CIRCUMRADIUS, THICKNESS,
)

OUT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "extra_stl"))

# Screen-stand leg height: single source = hexapod_prototype so the
# as-built stack (HEX_RAISED_TOTAL_H) can never drift from the STL.
# History: 62 -> 72 (Aug 2026, +10 mm) -> 28 (late-Aug 2026 design
# review: shorter lever on the magnet-held plate, ~44 mm CoG drop;
# still clears the Uno Q shield headers, ~12 mm tall, below the top).
# (The old 42 mm LEG_H base variant is retired -- nothing as-built
# used it.)
LEG_H_SCREEN = hp.HEX_RAISED_LEG_H
TOP_T = 2.0
FOOT_T = 2.5
LEG_W = 20.0          # tangential width of the leg blade (mm) — 2× prior
LEG_T = 7.0           # radial thickness of the leg blade (mm) — 2× prior
FOOT_PAD = 12.0       # square foot pad edge (mm) — grows with thicker legs
# Pull foot centres inward from the vertex along the corner ray.  Small =
# "very close to the corner" while leaving ≥2 mm past the M3 hole + pad.
CORNER_INSET = 9.5    # mm from circumradius (vertex) to foot-hole centre
FOOT_R = CIRCUMRADIUS - CORNER_INSET
HOLE_R = hp.BRACKET_BOLT_HOLE / 2.0
# Blind self-tap pilot in each stand foot (late-Aug 2026): an M3 x 8
# SHCS self-taps UP from under the plate -- 2 mm plate + ~5.5 mm bite,
# pilot 1 mm deeper than the thread reach.  Same Φ2.5 pilot bore the
# yaw-retainer anchors and cradle wall-end pilots use.
FOOT_PILOT_D = 2.5
FOOT_PILOT_DEPTH = 7.5

# Round lower plate (Aug 2026): Ø115 disc matching the chassis_top disc.
PLATE_R = hp.HEX_MOUNT_PLATE_DIAM / 2.0
WIRE_PORT_R = hp.MOUNT_PLATE_WIRE_PORT_D / 2.0

# Magnet shear-registration bosses on the plate UNDERSIDE (late-Aug
# 2026): one washer-shaped boss per magnet post at +/-31.1.  The Ø8
# magnet's top REG_BOSS_H mm nest in the REG_BORE_D bore (0.4 mm
# diametral clearance); the plate still seats on the magnet top face.
REG_BOSS_OD = 14.0
REG_BORE_D = hp.HEX_POST_MAGNET_OD + 0.4    # 8.4
REG_BOSS_H = 2.0
# Zip-tie slot pairs on the E/W rim: 8 mm (X, along the crossing wire
# bundle) x 3 mm (Y), one slot each side of the bundle path at y = 0.
ZIP_SLOT_L = 8.0
ZIP_SLOT_W = 3.0
ZIP_SLOT_XY = ((50.0, 7.0), (50.0, -7.0), (-50.0, 7.0), (-50.0, -7.0))

# GMT020-02 / ST7789 panel outline (user: 63×35 mm).  Sits centered on
# the top plate; only the wire slot is cut — no screen window.
# Long axis along X so the thin (short, 35 mm) edges face ±X; ribbon
# exits +X into the slot behind the panel.
SCREEN_LONG = 63.0          # along X
SCREEN_SHORT = 35.0         # along Y
# Wire pass-through under the thin (short) +X edge of the centered screen.
# Slot sits under the panel so wires drop behind it.  Aug 2026: narrowed
# from the full 35 mm short edge to 24 mm (the 8-wire pigtail / ribbon is
# ~20 mm) so the +X pair of screen mount screw holes keeps ≥2 mm of
# material beside the slot.
WIRE_SLOT_W = 24.0          # along the thin edge (Y)
WIRE_SLOT_H = 5.0           # radial depth of the slot (X)

# Screen hold-down (Aug 2026, user request): 4 pilot holes in the top
# plate at the panel's corner mounting holes -- M2 self-tapping screws
# straight into the 2 mm PLA top (Ø1.8 pilot).  Corner inset 2.5 mm is
# the common ST7789 / GMT020 module PCB pattern; if the actual panel has
# no corner holes, the pilots still serve for hold-down clips.
SCREEN_HOLE_D = 1.8
SCREEN_HOLE_INSET = 2.5
SCREEN_HOLE_XY = tuple(
    (sx * (SCREEN_LONG / 2.0 - SCREEN_HOLE_INSET),
     sy * (SCREEN_SHORT / 2.0 - SCREEN_HOLE_INSET))
    for sx in (-1.0, 1.0) for sy in (-1.0, 1.0))


def _wire_slot_center_x() -> float:
    """X of slot centre: under the screen, outer edge = screen +X edge."""
    screen_half_x = SCREEN_LONG / 2.0   # short-edge sits at ±31.5
    return screen_half_x - WIRE_SLOT_H / 2.0


# Leg azimuths (late-Aug 2026: 3 legs, was all 6 hex vertices).  90/210/
# 330 keeps the plate's south rim -- the two Ø8 wire ports at (+/-19,
# -44) and the underside 3.3 V Wago at (0, -36) -- free of feet, and
# the 330 foot backs the screen's +X (wire-slot) edge.
CORNER_ANGLES_DEG = (90.0, 210.0, 330.0)

# Arduino-UNO mounting pattern for the Uno Q, on the plate at the
# as-built board centre (hp.UNO_Q_ON_HEX_CENTRE = (0, -12), board long
# axis along X -- same placement the viz + wiring routes use).  Corner
# offsets are the official UNO mechanical drawing, from the board's
# lower-left corner; board envelope 68.58 x 53.34.  Only THREE of the
# four holes are cut: the SE hole (board-local (66.04, 17.78)) would
# land ~1.5 mm from the SE magnet registration boss, and a three-point
# mount holds a 25 g board fine.  Hardware: M3 x 8 down through board +
# plate into an M3 thumb nut below (same thumb-nut stock as the magnet
# posts, finger-tight).
UNO_BOARD_W, UNO_BOARD_D = 68.58, 53.34
UNO_HOLES_CORNER = ((13.97, 2.54), (15.24, 50.8), (66.04, 45.72))


def _uno_hole_xy() -> list[tuple[float, float]]:
    cx = hp.UNO_Q_ON_HEX_CENTRE[0] - UNO_BOARD_W / 2.0
    cy = hp.UNO_Q_ON_HEX_CENTRE[1] - UNO_BOARD_D / 2.0
    return [(cx + hx, cy + hy) for (hx, hy) in UNO_HOLES_CORNER]


def _foot_xy() -> list[tuple[float, float]]:
    out = []
    for deg in CORNER_ANGLES_DEG:
        a = np.deg2rad(deg)
        out.append((FOOT_R * np.cos(a), FOOT_R * np.sin(a)))
    return out


def _assert_feet_inside() -> None:
    """Feet must land fully on the ROUND lower plate with margin."""
    half = FOOT_PAD / 2.0
    for (x, y) in _foot_xy():
        r = float(np.hypot(x, y))
        # hole edge must stay inside the disc with a little margin
        d = PLATE_R - r - HOLE_R
        if d < 2.0:
            raise RuntimeError(
                f"foot at ({x:.1f},{y:.1f}) only {d:.1f} mm from the disc "
                f"edge (need ≥2 mm past the M3 hole)")
        # pad corners (oriented radial-out) must stay inside the disc
        a = np.arctan2(y, x)
        c, s = np.cos(a), np.sin(a)
        for lx, ly in ((half, half), (half, -half), (-half, half), (-half, -half)):
            wx = x + c * lx - s * ly
            wy = y + s * lx + c * ly
            if np.hypot(wx, wy) > PLATE_R:
                raise RuntimeError(
                    f"foot pad corner ({wx:.1f},{wy:.1f}) leaves the disc "
                    f"(increase CORNER_INSET or shrink FOOT_PAD)")


def _assert_round_plate_features() -> None:
    """Wire ports + zip-tie slots: inside the disc, clear of every other
    hole / foot pad / component footprint, and the disc clears the coxa
    sweep (same ≥2 mm policy as the chassis coupons)."""
    from hexapod_walker.prototype_sts3215.tools.make_xtool_chassis_hex import (
        _coxa_sweep_clearance,
    )
    ang = np.linspace(0.0, 2.0 * np.pi, 64, endpoint=False)
    disc = [(PLATE_R * np.cos(a), PLATE_R * np.sin(a)) for a in ang]
    clr = _coxa_sweep_clearance(disc)
    if clr < 2.0:
        raise RuntimeError(
            f"round plate R={PLATE_R:.1f} only {clr:.1f} mm from the coxa "
            f"sweep keep-out (need ≥2 mm)")

    # Live standoff/magnet-post square (with its Ø14 registration boss
    # footprint on the underside) + the platform foot holes.
    others = [(cx, cy, REG_BOSS_OD / 2.0)
              for (cx, cy) in hp.CHASSIS_STANDOFF_HOLES_XY]
    others += [(cx, cy, HOLE_R) for (cx, cy) in _foot_xy()]

    def check_feature(cx, cy, hx, hy, name):
        # corners inside the disc with ≥2 mm rim
        for sx in (-1, 1):
            for sy in (-1, 1):
                if np.hypot(cx + sx * hx, cy + sy * hy) > PLATE_R - 2.0:
                    raise RuntimeError(f"{name} at ({cx},{cy}) too close "
                                       f"to the disc rim")
        for (ox, oy, orr) in others:
            dx = max(abs(ox - cx) - hx, 0.0)
            dy = max(abs(oy - cy) - hy, 0.0)
            if np.hypot(dx, dy) < orr + 2.0:
                raise RuntimeError(f"{name} at ({cx},{cy}) within 2 mm of "
                                   f"the hole at ({ox},{oy})")

    for (cx, cy) in hp.MOUNT_PLATE_WIRE_PORT_XY:
        check_feature(cx, cy, WIRE_PORT_R, WIRE_PORT_R, "wire port")
    for (cx, cy) in ZIP_SLOT_XY:
        check_feature(cx, cy, ZIP_SLOT_L / 2.0, ZIP_SLOT_W / 2.0,
                      "zip-tie slot")
    # Uno Q mounting holes: same rim + keep-out policy as the slots
    # (treated as tiny squares of the hole radius).
    r = hp.BRACKET_BOLT_HOLE / 2.0
    for (cx, cy) in _uno_hole_xy():
        check_feature(cx, cy, r, r, "Uno Q mount hole")


def _top_disc(z0: float, thickness: float) -> trimesh.Trimesh:
    """Round Ø115 top disc (late-Aug 2026: was a 110 hex; now matches
    the round plate and chassis_top discs below)."""
    plate = trimesh.creation.cylinder(
        radius=PLATE_R, height=thickness, sections=128)
    plate.apply_translation([0, 0, z0 + thickness / 2.0])
    return plate


def _leg_and_foot(cx: float, cy: float, *, leg_h: float) -> trimesh.Trimesh:
    """Vertical blade + foot pad at (cx, cy), outward normal away from origin."""
    a = np.arctan2(cy, cx)
    # Local frame: +X radial out, +Y along flat, +Z up.
    c, s = np.cos(a), np.sin(a)
    R = np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ], dtype=float)

    # Leg spans from top of foot to TOP of upper plate (flush — no stubs).
    blade_h = leg_h + TOP_T - FOOT_T
    leg = trimesh.creation.box(extents=[LEG_T, LEG_W, blade_h])
    leg.apply_translation([0.0, 0.0, FOOT_T + blade_h / 2.0])

    foot = trimesh.creation.box(extents=[FOOT_PAD, FOOT_PAD, FOOT_T])
    foot.apply_translation([0.0, 0.0, FOOT_T / 2.0])

    # Blind Φ2.5 self-tap pilot rising from the foot bottom into the leg
    # blade: the mounting M3 x 8 drives UP through the plate into this.
    # Cut AFTER the leg+foot union so the pilot is one clean bore.
    part = trimesh.boolean.union([leg, foot])
    pilot = trimesh.creation.cylinder(
        radius=FOOT_PILOT_D / 2.0, height=FOOT_PILOT_DEPTH + 1.0,
        sections=32)
    # extend 0.5 mm below z=0 so the boolean never sees a coplanar face
    pilot.apply_translation([0.0, 0.0, (FOOT_PILOT_DEPTH + 1.0) / 2.0 - 0.5])
    part = trimesh.boolean.difference([part, pilot])

    part.apply_transform(R)
    part.apply_translation([cx, cy, 0.0])
    return part


def _top_cutouts(*, leg_h: float, wire_slot: bool) -> list[trimesh.Trimesh]:
    """Boxes used to punch the wire slot through the top plate.

    No screen window: the 63×35 panel sits on the solid top, centered.
    The slot sits under the panel's thin (short) +X edge — Y edges match
    the screen; outer X edge = screen +X edge (wires drop behind).
    """
    z_mid = leg_h + TOP_T / 2.0
    tall = TOP_T * 6.0
    out: list[trimesh.Trimesh] = []
    if wire_slot:
        cx = _wire_slot_center_x()
        slot = trimesh.creation.box(
            extents=[WIRE_SLOT_H, WIRE_SLOT_W, tall])
        slot.apply_translation([cx, 0.0, z_mid])
        out.append(slot)
        # Screen hold-down pilot holes (screen variant only).
        for (hx, hy) in SCREEN_HOLE_XY:
            h = trimesh.creation.cylinder(
                radius=SCREEN_HOLE_D / 2.0, height=tall, sections=24)
            h.apply_translation([hx, hy, z_mid])
            out.append(h)
    return out


def make_raised_platform(*, leg_h: float = None,
                         wire_slot: bool = False) -> trimesh.Trimesh:
    """One solid: feet at z=0..FOOT_T, legs, top disc at z=leg_h..leg_h+TOP_T."""
    if leg_h is None:
        leg_h = LEG_H_SCREEN
    top = _top_disc(leg_h, TOP_T)
    parts = [top]
    for (x, y) in _foot_xy():
        parts.append(_leg_and_foot(x, y, leg_h=leg_h))
    solid = trimesh.boolean.union(parts)
    cuts = _top_cutouts(leg_h=leg_h, wire_slot=wire_slot)
    if cuts:
        solid = trimesh.boolean.difference([solid, *cuts])
    solid.merge_vertices()
    return solid


def write_raised_stl(*, leg_h: float = None, wire_slot: bool = False,
                     name: str) -> str:
    solid = make_raised_platform(leg_h=leg_h, wire_slot=wire_slot)
    path = os.path.join(OUT_DIR, name)
    solid.export(path)
    return path


def _round_plate_hole_xy_r():
    """Every circular hole of the round lower plate: the LIVE +/-31.1
    standoff/magnet-post square (late-Aug 2026: the legacy 49.5 mm
    ``ELEC_CHASSIS_MOUNT_HOLES_XY`` square is dropped -- nothing has
    bolted to it since the standoff-era chassis), the 3 raised-platform
    foot holes (M3 x 8 UP into the foot pilots), the 3 Uno Q mounting
    holes (M3 x 8 DOWN into thumb nuts), and the 2 wire ports for the
    underside 3.3 V Wago."""
    r = hp.BRACKET_BOLT_HOLE / 2.0
    for (cx, cy) in hp.CHASSIS_STANDOFF_HOLES_XY:
        yield cx, cy, r
    for (cx, cy) in _foot_xy():
        yield cx, cy, HOLE_R
    for (cx, cy) in _uno_hole_xy():
        yield cx, cy, r
    for (cx, cy) in hp.MOUNT_PLATE_WIRE_PORT_XY:
        yield cx, cy, WIRE_PORT_R


# (The SVG cut-file writer is RETIRED, late-Aug 2026: the underside
# magnet registration bosses cannot be laser-cut, so shipping a flat
# cut file was a footgun.  The plate is print-only now.)


def write_lower_with_leg_holes_stl() -> str:
    plate = trimesh.creation.cylinder(
        radius=PLATE_R, height=THICKNESS, sections=128)
    # Magnet shear-registration bosses on the underside (printed STL
    # only; overlap the plate by 0.5 mm for a clean volumetric union).
    bosses = []
    for (cx, cy) in hp.CHASSIS_STANDOFF_HOLES_XY:
        b = trimesh.creation.cylinder(
            radius=REG_BOSS_OD / 2.0, height=REG_BOSS_H + 0.5, sections=64)
        b.apply_translation(
            [cx, cy, -THICKNESS / 2.0 - (REG_BOSS_H - 0.5) / 2.0])
        bosses.append(b)
    solid = trimesh.boolean.union([plate, *bosses])
    cuts = []
    for (cx, cy, r) in _round_plate_hole_xy_r():
        h = trimesh.creation.cylinder(radius=r, height=THICKNESS * 8)
        h.apply_translation([cx, cy, 0.0])
        cuts.append(h)
    # E/W zip-tie slot pairs.  (Late-Aug 2026 fix: these were only ever
    # cut in the retired SVG file -- the printed STL silently lacked
    # them.  Now that the plate is print-only they are cut here.)
    for (cx, cy) in ZIP_SLOT_XY:
        s = trimesh.creation.box(
            extents=[ZIP_SLOT_L, ZIP_SLOT_W, THICKNESS * 8])
        s.apply_translation([cx, cy, 0.0])
        cuts.append(s)
    # Registration bores: the top REG_BOSS_H mm of each Ø8 post magnet
    # nests here (plate underside still seats on the magnet top face --
    # the bore reaches 0.1 mm into the plate so the boolean never has a
    # coplanar face; the 0.1 mm seat recess is below print tolerance).
    for (cx, cy) in hp.CHASSIS_STANDOFF_HOLES_XY:
        bore_h = REG_BOSS_H + 0.1
        h = trimesh.creation.cylinder(
            radius=REG_BORE_D / 2.0, height=bore_h, sections=64)
        h.apply_translation(
            [cx, cy, -THICKNESS / 2.0 + 0.1 - bore_h / 2.0])
        cuts.append(h)
    solid = trimesh.boolean.difference([solid, *cuts])
    path = os.path.join(OUT_DIR, "round_mount_plate_115_with_leg_holes.stl")
    solid.export(path)
    return path


def write_preview(*, leg_h: float = None, wire_slot: bool = False,
                  show_screen: bool = False, name: str | None = None) -> str:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    from matplotlib.transforms import Affine2D

    if leg_h is None:
        leg_h = LEG_H_SCREEN
    feet = _foot_xy()

    fig, (ax_top, ax_side) = plt.subplots(
        1, 2, figsize=(11.0, 5.2), dpi=120,
        gridspec_kw={"width_ratios": [1.15, 0.85]})

    # --- top view ---
    ax_top.set_aspect("equal")
    ax_top.set_title("top: 3 feet at az 90/210/330 (on round Ø115 plate)",
                     fontsize=11)
    ax_top.add_patch(Circle((0, 0), PLATE_R, fill=True,
                            facecolor="#e0e7ff", edgecolor="#4338ca",
                            alpha=0.5, linewidth=1.2,
                            label="lower round plate (Ø115)"))
    ax_top.add_patch(Circle((0, 0), PLATE_R, fill=True,
                            facecolor="#bfdbfe", edgecolor="#1d4ed8",
                            alpha=0.45, linewidth=1.4, linestyle="--",
                            label="upper round disc (Ø115)"))
    for j, (px, py) in enumerate(_uno_hole_xy()):
        ax_top.add_patch(Circle((px, py), hp.BRACKET_BOLT_HOLE / 2.0,
                                fill=False, edgecolor="#7c3aed",
                                linewidth=1.2,
                                label="Uno Q M3 (3-pt)" if j == 0 else None))
    for j, (px, py) in enumerate(hp.MOUNT_PLATE_WIRE_PORT_XY):
        ax_top.add_patch(Circle((px, py), WIRE_PORT_R, fill=True,
                                facecolor="#fecaca", edgecolor="#991b1b",
                                label="wire port Ø8" if j == 0 else None))
    for j, (px, py) in enumerate(ZIP_SLOT_XY):
        ax_top.add_patch(Rectangle((px - ZIP_SLOT_L / 2, py - ZIP_SLOT_W / 2),
                                   ZIP_SLOT_L, ZIP_SLOT_W, fill=True,
                                   facecolor="#d1d5db", edgecolor="#374151",
                                   label="zip-tie slot" if j == 0 else None))
    for i, (cx, cy) in enumerate(feet):
        a = np.degrees(np.arctan2(cy, cx))
        pad = Rectangle((-FOOT_PAD / 2, -FOOT_PAD / 2), FOOT_PAD, FOOT_PAD,
                        fill=True, facecolor="#fde68a", edgecolor="#b45309",
                        alpha=0.9, linewidth=0.8,
                        label="foot pad + Φ2.5 pilot" if i == 0 else None)
        pad.set_transform(Affine2D().rotate_deg(a) +
                          Affine2D().translate(cx, cy) + ax_top.transData)
        ax_top.add_patch(pad)
        ax_top.add_patch(Circle((cx, cy), FOOT_PILOT_D / 2.0, fill=False,
                                edgecolor="#111827", linewidth=1.0))
        ax_top.plot([0, cx], [0, cy], color="#94a3b8", linewidth=0.5)
    if show_screen:
        # Long axis along X; thin (short) edges at ±X — matches slot.
        ax_top.add_patch(Rectangle((-SCREEN_LONG / 2, -SCREEN_SHORT / 2),
                                   SCREEN_LONG, SCREEN_SHORT,
                                   fill=False, edgecolor="#dc2626",
                                   linewidth=1.4, linestyle="--",
                                   label=f"screen {SCREEN_LONG:.0f}×{SCREEN_SHORT:.0f} (on top)"))
    if wire_slot:
        cx = _wire_slot_center_x()
        ax_top.add_patch(Rectangle((cx - WIRE_SLOT_H / 2, -WIRE_SLOT_W / 2),
                                   WIRE_SLOT_H, WIRE_SLOT_W,
                                   fill=True, facecolor="#fca5a5",
                                   edgecolor="#991b1b", alpha=0.85,
                                   label=f"wire {WIRE_SLOT_W:.0f}×{WIRE_SLOT_H:.0f}"))
    ax_top.set_xlim(-65, 65)
    ax_top.set_ylim(-65, 65)
    ax_top.set_xlabel("X (mm)")
    ax_top.set_ylabel("Y (mm)")
    ax_top.grid(True, linestyle=":", alpha=0.5)
    ax_top.legend(loc="upper right", fontsize=8)

    # --- side view (section through +X) ---
    ax_side.set_aspect("equal")
    ax_side.set_title(f"side: {leg_h:.0f} mm standoff", fontsize=11)
    # lower plate (round Ø115 disc in section)
    ax_side.add_patch(Rectangle((-PLATE_R, -THICKNESS), 2 * PLATE_R, THICKNESS,
                                facecolor="#93c5fd", edgecolor="#1d4ed8",
                                label="lower round plate 2 mm"))
    # upper plate
    ax_side.add_patch(Rectangle((-PLATE_R, leg_h), 2 * PLATE_R, TOP_T,
                                facecolor="#93c5fd", edgecolor="#1d4ed8",
                                label="upper disc 2 mm"))
    # Project the 3 feet onto the side view: 210°/330° at ±|cos 30°|,
    # the 90° foot at x = 0.
    x_proj = FOOT_R * np.cos(np.deg2rad(30.0))
    for x in (-x_proj, 0.0, x_proj):
        ax_side.add_patch(Rectangle((x - LEG_T / 2, FOOT_T), LEG_T,
                                    leg_h + TOP_T - FOOT_T,
                                    facecolor="#fbbf24", edgecolor="#b45309",
                                    alpha=0.85,
                                    label="leg" if x > 0 else None))
        ax_side.add_patch(Rectangle((x - FOOT_PAD / 2, 0), FOOT_PAD, FOOT_T,
                                    facecolor="#f59e0b", edgecolor="#92400e",
                                    label="foot" if x > 0 else None))
    ax_side.annotate("", xy=(0, leg_h), xytext=(0, 0),
                     arrowprops=dict(arrowstyle="<->", color="#111827"))
    ax_side.text(3, leg_h / 2, f"{leg_h:.0f} mm", va="center", fontsize=10)
    ax_side.set_xlim(-60, 60)
    ax_side.set_ylim(-8, leg_h + TOP_T + 8)
    ax_side.set_xlabel("X (mm)")
    ax_side.set_ylabel("Z (mm)")
    ax_side.grid(True, linestyle=":", alpha=0.5)
    ax_side.legend(loc="upper right", fontsize=8)

    bits = [f"Ø{2 * PLATE_R:.0f}", f"legs {leg_h:.0f} mm",
            f"foot r={FOOT_R:.1f}"]
    if show_screen:
        bits.append(f"screen {SCREEN_LONG:.0f}×{SCREEN_SHORT:.0f} centered")
    if wire_slot:
        bits.append(f"wire {WIRE_SLOT_W:.0f}×{WIRE_SLOT_H:.0f} @ +X")
    fig.suptitle("round screen stand  " + "  ".join(bits), fontsize=12)
    fig.tight_layout()
    if name is None:
        name = "hex_raised_platform_110_preview.png"
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path)
    plt.close(fig)
    return path


def main() -> None:
    argparse.ArgumentParser(description=__doc__).parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    _assert_feet_inside()
    _assert_round_plate_features()
    # Retire superseded artifacts: the old Ø110 hex plate, the 72 mm
    # screen-stand files (28 mm variant is live), and -- late-Aug 2026
    # review round 2 -- the 42 mm base platform (nothing used it), the
    # separate Uno Q io board (its UNO hole pattern lives in the round
    # plate now), and the plate's SVG cut file (the underside
    # registration bosses cannot be laser-cut).
    for stale in ("hex_mount_plate_110_with_leg_holes.stl",
                  "hex_mount_plate_110_with_leg_holes.svg",
                  "hex_raised_platform_110_h72_screen.stl",
                  "hex_raised_platform_110_h72_screen_preview.png",
                  "hex_raised_platform_110.stl",
                  "hex_raised_platform_110_preview.png",
                  "hex_uno_q_io_board_110.stl",
                  "hex_uno_q_io_board_110_preview.png",
                  "round_mount_plate_115_with_leg_holes.svg"):
        p = os.path.join(OUT_DIR, stale)
        if os.path.isfile(p):
            os.remove(p)
            print(f"removed stale {p}")
    feet = _foot_xy()
    print(f"round screen stand on round Ø{2 * PLATE_R:.0f} plate")
    print(f"  3 legs @ az 90/210/330, foot centres r={FOOT_R:.1f} mm "
          f"(R {CIRCUMRADIUS:.0f} − inset {CORNER_INSET:.0f})")
    print(f"  foot pads {FOOT_PAD:.0f}×{FOOT_PAD:.0f}×{FOOT_T:.1f} mm, "
          f"blind Φ{FOOT_PILOT_D:.1f}×{FOOT_PILOT_DEPTH:.1f} self-tap "
          f"pilots (3x M3x8 SHCS up from under the plate)")
    for i, (x, y) in enumerate(feet):
        az = CORNER_ANGLES_DEG[i]
        print(f"    foot {i} @ {az:.0f}°: ({x:+.1f}, {y:+.1f})")
    print("  Uno Q 3-pt mount holes (M3x8 down into thumb nuts): "
          + ", ".join(f"({x:+.1f},{y:+.1f})" for x, y in _uno_hole_xy()))

    # The round lower plate is the live as-built part -- always regenerate.
    print(f"\n[lower plate] round Ø{2 * PLATE_R:.0f} (print-only)")
    print(f"wrote {write_lower_with_leg_holes_stl()}")

    slot_cx = _wire_slot_center_x()
    screen_edge = SCREEN_LONG / 2.0
    print(f"\n[screen] legs {LEG_H_SCREEN:.0f} mm · "
          f"no window · screen {SCREEN_LONG:.0f}×{SCREEN_SHORT:.0f} centered · "
          f"wire slot {WIRE_SLOT_W:.0f}×{WIRE_SLOT_H:.0f} under +X edge "
          f"(centre x={slot_cx:.1f}, outer={screen_edge:.1f}) · "
          f"4x Ø{SCREEN_HOLE_D} M2 pilot holes at "
          + ", ".join(f"({x:+.0f},{y:+.0f})" for x, y in SCREEN_HOLE_XY))
    screen_stl = f"hex_raised_platform_110_h{LEG_H_SCREEN:.0f}_screen.stl"
    screen_png = (f"hex_raised_platform_110_h{LEG_H_SCREEN:.0f}"
                  f"_screen_preview.png")
    print(f"wrote {write_raised_stl(wire_slot=True, name=screen_stl)}")
    print(f"wrote {write_preview(wire_slot=True, show_screen=True, name=screen_png)}")
    print("\nPrint the stand feet-down (top is flat; pilots print as "
          "blind holes at the bed).  Mount: lift the plate off its "
          "magnets, drive 3x M3x8 SHCS UP through the plate into the "
          "foot pilots, set the assembly back on the magnets.  Screen: "
          "seat the 63×35 panel centered (long axis along X); pigtail "
          "drops through the 24×5 slot under the +X short edge; hold "
          "down with 4x M2 self-tappers.  Uno Q: 3x M3x8 down through "
          "board + plate into M3 thumb nuts.")


if __name__ == "__main__":
    main()
