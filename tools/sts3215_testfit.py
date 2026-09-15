#!/usr/bin/env python3
"""Standalone, printable FEETECH STS3215 front-face mount + test cradle.

June 2026 redesign foundation
-----------------------------
The prototype is migrating from the DS3225 PWM servo (mounted by its
protruding side TABS onto a printed tab-shelf + heat-set bosses) to the
FEETECH STS3215 serial-bus servo, which has NO tabs.  The STS3215 bolts
via 4x M2.5 THREADED HOLES IN ITS OWN METAL CASE, arranged on a 9.9 mm
square centred on the output shaft, on the front (output) face.

That is a fundamentally different mounting paradigm than the whole
``hexapod_prototype.py`` well/cradle stack is built around, so before
propagating it into all 18 joint cradles (coxa hip / femur knee /
chassis yaw) this module nails down -- and lets you PRINT and validate
against a real servo -- the single reusable mount feature:

    * a front-face PLATE with a small (Phi 10) coupling-clearance bore and
      the 4x case-screw clearance holes on the 9.9 mm square -- the servo
      bolts up through these with M2/M2.5 SELF-TAPPING screws into its own
      (un-tapped, plastic) case; the disc horn mounts on the spline ABOVE
      the plate, plus
    * a body-locating shroud (two side walls) plus a PARTIAL back housing
      that press-fits a BALL BEARING (default 688, 8x16x5) coaxial with the
      output, so the joint is supported on BOTH ends (sandwich), not
      cantilevered.  The back housing covers only the +X (idler-axis) half;
      the -X half stays open for the JST bus connectors, cover and cables.

All STS3215 dimensions below are taken from FEETECH's official STEP
(SO-ARM100 ``STS3215_03a.step``); this file deliberately does NOT import
the (still DS3225-valued) ``SERVO_*`` constants from
``hexapod_prototype`` so it is self-contained and safe to print while the
in-model refit is still in progress.

Frame (matches how the servo is used in a leg):
    +X = body long axis, output shaft offset toward +X
    +Y = body short axis (depth)
    +Z = output-shaft direction (out of the FRONT face)
    origin = centre of the body's BACK (idler) face.

Usage
-----
    python tools/sts3215_testfit.py            # writes stl_prototype/sts3215_testfit.stl
    python tools/sts3215_testfit.py --plate-only   # just the bolt-pattern plate
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

# Reuse the proven manifold-friendly boolean helpers from the main model.
_HERE = Path(__file__).resolve().parent

import hexapod_prototype as HP  # noqa: E402  (_box/_cyl/_union/_diff)

# ---------------------------------------------------------------------------
# STS3215 geometry (FEETECH official STEP: STS3215_03a.step)
# ---------------------------------------------------------------------------
BODY_W = 45.4    # mm -- long axis (X), output-offset direction
BODY_D = 24.8    # mm -- depth (Y)
BODY_H = 35.0    # mm -- back face -> front(output) face (Z), no horn
OUTPUT_X = 12.5  # mm -- output axis offset from body centre (+X)
DISC_HORN_OD = 20.0    # mm -- disc-horn OD (mounts on the spline ABOVE the plate)
SPLINE_OD = 5.9        # mm -- 25T output spline OD

# CENTRAL PLATE BORE.  At the front-face plane only the rotating output
# COUPLING (spline + bushing, ~dia 9) pokes through -- the dia-20 disc
# horn mounts on the spline ABOVE the plate and seats FLAT on the plate
# top, so it is NOT bored out of the plate.  The bore is therefore only
# Phi 10: large enough for the coupling, small enough to leave solid
# plate at the 4x M2.5 holes (centres at r ~7.0; inner edge r ~5.65, just
# clear of the r5 bore).  Boring the full dia-20/22 disc footprint would
# SWALLOW all four screw holes -- which is exactly the bug the first test
# print had (one big hole, nothing to bolt to).  Matches the main model's
# SERVO_OUTPUT_BORE_OD.
OUTPUT_BORE_OD = 10.0   # mm -- central coupling-clearance bore (NOT the disc OD!)

# Case-face mount: 4x on a 9.9 mm square about the output axis (STEP-
# verified: front-face holes at (12.5 +/- 4.95, +/- 4.95), dia 2.5).  Real
# STS3215 projects (SO-ARM100/101, K-Scale) mount the plastic-case housing
# with M2 SELF-TAPPING screws (the case is NOT pre-tapped).  The screws
# pass through the printed plate (clearance) and self-tap into the servo's
# own dia-2.5 case holes.  NOTE: an M2 self-tapper is a little loose in a
# dia-2.5 hole -- M2.5 self-tapping grips the dia-2.5 case holes better;
# the dia-2.6 plate clearance below passes either.
SERVO_CASE_HOLE_OD = 2.5  # mm -- molded hole in the servo case (STEP)
MOUNT_SQUARE = 9.9        # mm centre-to-centre (X and Y)
MOUNT_SCREW_CLEAR = 2.6   # mm -- clearance hole in the printed plate (M2/M2.5 self-tap)
MOUNT_HEAD_OD = 4.2       # mm -- self-tap pan/SHCS head OD (counterbore option)
MOUNT_HEAD_DEPTH = 2.0    # mm -- counterbore depth to sink the head flush

# ---------------------------------------------------------------------------
# Printed cradle parameters
# ---------------------------------------------------------------------------
PLATE_T = 4.0        # mm -- front mount-plate thickness
PLATE_MARGIN = 4.0   # mm -- plate overhang past the body footprint (X & Y)
WALL_T = 3.0         # mm -- side-shroud wall thickness
BODY_CL = 0.7        # mm -- per-face clearance around the body in the shroud

# ---------------------------------------------------------------------------
# Passive side BALL BEARING (the SO-ARM "sandwich" idea, upgraded)
# ---------------------------------------------------------------------------
# Design review: a real BALL BEARING on the passive side is ~an order of
# magnitude stiffer than a POM puck journaling in plastic, and for 100-150
# mm hexapod leg segments it is strongly recommended.  Arrangement: the
# moving link carries a stub shaft coaxial with the output axis; the
# bearing INNER race rides the stub, and the bearing OUTER race press-fits
# into a pocket in this fixed printed housing.  Together with the servo's
# own front-side output bearing, the joint is supported at BOTH ends.
#
# Default 688-2RS (8 mm bore x 16 OD x 5 W) -- the most compact that clears
# the STS3215 back without fouling the -X connectors.  Alternatives that
# also fit: 608 (8x22x7, robust) or 6800 (10x19x5).  Bump BEARING_* to suit.
BEARING_BORE = 8.0        # mm -- inner-race bore (stub-shaft diameter)
BEARING_OD   = 16.0       # mm -- outer-race OD = press-fit pocket diameter
BEARING_W    = 5.0        # mm -- bearing width = pocket depth
BEARING_PRESS = 0.0       # mm -- shrink the pocket dia by this for press fit (tune per printer)
BEARING_SHOULDER_T = 2.0  # mm -- backing wall that seats the outer race
BEARING_STUB_CLEAR = BEARING_OD - 4.0  # mm -- shoulder through-hole (inner-race / stub access)
BACK_PLATE_T = BEARING_W + BEARING_SHOULDER_T  # mm -- back housing thickness

# Carbon-fibre leg-segment tube socketed into the moving yoke.
LEG_TUBE_OD           = 8.0   # mm -- carbon tube OD
LEG_TUBE_SOCKET_CLEAR = 0.15  # mm -- radial glue gap in the socket
LEG_TUBE_SOCKET_DEPTH = 14.0  # mm -- tube engagement depth
LEG_TUBE_PIN_OD       = 2.6   # mm -- transverse retention-pin cross-hole (dia-2.5 roll pin / M2.5)
LEG_TUBE_PIN_INSET    = LEG_TUBE_SOCKET_DEPTH / 2.0  # mm -- pin axis from socket mouth

# The STS3215 BACK (idler) face is NOT flat: the idler hub + axle sit on
# the OUTPUT axis (+X), while the two recessed JST bus connectors, the
# raised PCB/cover block and the cable exit are toward CENTRE / -X.  So the
# back bearing housing is PARTIAL: it spans only the +X (idler-axis) half
# to carry the bearing, leaving the -X half OPEN for connectors and cables.
BACK_OPEN = True


def mount_hole_centres():
    """The 4 case-screw centres (X, Y), a 9.9 mm square about the output."""
    h = MOUNT_SQUARE / 2.0
    return [(OUTPUT_X + sx * h, sy * h) for sx in (-1, 1) for sy in (-1, 1)]


def make_mount_plate(*, counterbore: bool = False) -> "HP.trimesh.Trimesh":
    """The front-face mount plate: output bore + 4x self-tap clearance holes.

    Plate occupies z in [BODY_H, BODY_H + PLATE_T]; the servo's front face
    seats against its underside at z = BODY_H and 4x self-tapping screws
    thread down into the case.  This is the single reusable feature every
    joint cradle will adopt."""
    plate_w = BODY_W + 2 * PLATE_MARGIN
    plate_d = BODY_D + 2 * PLATE_MARGIN
    plate = HP._box((plate_w, plate_d, PLATE_T),
                    center=(0.0, 0.0, BODY_H + PLATE_T / 2.0))

    cuts = []
    # Central coupling-clearance bore (Phi 10) -- ONLY the spline/coupling
    # passes through; the disc horn sits on top.  Small enough to keep the
    # 4x mount holes in solid plate.
    bore = HP._cyl(OUTPUT_BORE_OD / 2.0, PLATE_T * 4)
    bore.apply_translation([OUTPUT_X, 0.0, BODY_H + PLATE_T / 2.0])
    cuts.append(bore)

    # 4x self-tap case-screw clearance holes (+ optional head counterbore).
    for (hx, hy) in mount_hole_centres():
        hole = HP._cyl(MOUNT_SCREW_CLEAR / 2.0, PLATE_T * 4)
        hole.apply_translation([hx, hy, BODY_H + PLATE_T / 2.0])
        cuts.append(hole)
        if counterbore:
            # Recess the head flush so the disc horn seats flat above.
            cb = HP._cyl(MOUNT_HEAD_OD / 2.0, MOUNT_HEAD_DEPTH * 2)
            cb.apply_translation([hx, hy, BODY_H + PLATE_T])  # cuts down from top
            cuts.append(cb)

    return HP._diff(plate, *cuts)


def make_back_bearing_plate() -> "HP.trimesh.Trimesh":
    """Partial back housing carrying the passive-side ball bearing (+X half).

    Occupies z in [-BACK_PLATE_T, 0] (behind the servo's back face) and
    spans only the +X (idler-axis) half so the -X connectors / cover /
    cables stay open.  On the output axis: a Phi BEARING_OD pocket (depth
    BEARING_W, opening on the outer face) press-fits the bearing's outer
    race against a backing shoulder, and a Phi BEARING_STUB_CLEAR hole
    through the shoulder gives the inner race + stub shaft clearance."""
    # Span x from just past body-centre out to the +X plate edge.
    x_lo = 2.0
    x_hi = BODY_W / 2.0 + PLATE_MARGIN
    plate_d = BODY_D + 2 * (BODY_CL + WALL_T)
    plate = HP._box((x_hi - x_lo, plate_d, BACK_PLATE_T),
                    center=(0.5 * (x_lo + x_hi), 0.0, -BACK_PLATE_T / 2.0))

    # Bearing press-fit pocket: opens on the OUTER face (z=-BACK_PLATE_T),
    # depth BEARING_W, leaving a BEARING_SHOULDER_T backing wall.
    pocket = HP._cyl((BEARING_OD - BEARING_PRESS) / 2.0, BEARING_W * 2)
    pocket.apply_translation([OUTPUT_X, 0.0, -BACK_PLATE_T])  # half cuts in by BEARING_W
    # Shoulder through-hole for the inner race / stub shaft.
    stub = HP._cyl(BEARING_STUB_CLEAR / 2.0, BACK_PLATE_T * 4)
    stub.apply_translation([OUTPUT_X, 0.0, -BACK_PLATE_T / 2.0])
    return HP._diff(plate, pocket, stub)


# --- Driven disc-horn bolt pattern (Phi 14 circle; phase to match the real
#     FEETECH horn = SO-ARM's axis-aligned 9.9 mm square = 45 deg). ----------
DISC_HORN_BOLT_PCD = 14.0   # mm -- driven-horn bolt-circle diameter (r7)
DISC_HORN_BOLT_OD  = 3.4    # mm -- M3 clearance for the link-to-horn bolts
DISC_HORN_H        = 5.0    # mm -- disc-horn thickness above the front plate


def horn_bolt_centres():
    """4 driven-horn bolt centres (X,Y): Phi 14 circle at 45 deg => the
    (OUTPUT_X +/- 4.95, +/- 4.95) axis-aligned square the real horn uses."""
    r = DISC_HORN_BOLT_PCD / 2.0
    import math
    return [(OUTPUT_X + r * math.cos(a), r * math.sin(a))
            for a in (math.pi / 4, 3 * math.pi / 4,
                      5 * math.pi / 4, 7 * math.pi / 4)]


def make_testfit_yoke() -> "HP.trimesh.Trimesh":
    """The MOVING link of the yaw joint: a C-clevis that straddles the
    servo, proving the dual-sided sandwich.

    - Top arm (above the disc horn): bolts to the driven disc horn with 4x
      M3 on the Phi 14 circle -> driven by the output.
    - Bottom arm (below the back housing): carries a Phi 8 STUB SHAFT that
      rises into the 688 bearing's inner race -> the passive support.
    - Spine on +X ties the two arms; a Phi 8 carbon-tube SOCKET extends
      outboard as the leg segment.

    Same frame as the cradle (origin = servo back-face centre, +Z = output,
    output axis at x = OUTPUT_X).  Prints separately from the housing."""
    front_top = BODY_H + PLATE_T                 # 39  (disc horn sits 39..44)
    top_z0 = front_top + DISC_HORN_H             # 44  (link top arm underside)
    arm_t = 4.0
    back_out = -BACK_PLATE_T                     # -7  (housing outer face)
    bot_z1 = back_out - 1.0                      # -8  (bottom arm top)
    bot_z0 = bot_z1 - arm_t                      # -12 (bottom arm underside)

    arm_x0, arm_x1 = 2.0, 32.0
    arm_y = 12.0
    spine_x0, spine_x1 = 28.0, 32.0

    # Top arm (bolts to disc horn).
    top = HP._box((arm_x1 - arm_x0, 2 * arm_y, arm_t),
                  center=(0.5 * (arm_x0 + arm_x1), 0.0, top_z0 + arm_t / 2.0))
    cuts = []
    for (hx, hy) in horn_bolt_centres():
        h = HP._cyl(DISC_HORN_BOLT_OD / 2.0, arm_t * 4)
        h.apply_translation([hx, hy, top_z0 + arm_t / 2.0])
        cuts.append(h)
    top = HP._diff(top, *cuts)

    # Bottom arm + stub shaft into the bearing inner race (z in [-7,-2]).
    bot = HP._box((arm_x1 - arm_x0, 2 * arm_y, arm_t),
                  center=(0.5 * (arm_x0 + arm_x1), 0.0, bot_z0 + arm_t / 2.0))
    stub_top = back_out + BEARING_W              # -2
    stub_z0 = bot_z1 - arm_t
    stub = HP._cyl(BEARING_BORE / 2.0 - 0.1, stub_top - stub_z0)
    stub.apply_translation([OUTPUT_X, 0.0, 0.5 * (stub_z0 + stub_top)])

    # Spine tying the two arms on +X (just clear of the housing +X edge).
    spine = HP._box((spine_x1 - spine_x0, 2 * arm_y, (top_z0 + arm_t) - bot_z0),
                    center=(0.5 * (spine_x0 + spine_x1), 0.0,
                            0.5 * (bot_z0 + top_z0 + arm_t)))

    # Phi 8 carbon-tube socket extending outboard (+X) from the spine.
    sock_len = LEG_TUBE_SOCKET_DEPTH + 6.0
    sock_z = 0.5 * (bot_z0 + top_z0 + arm_t)
    Rx = HP.rotation_matrix(np.pi / 2.0, [0, 1, 0])  # cyl axis Z -> X
    boss = HP._cyl(LEG_TUBE_OD / 2.0 + 3.0, sock_len)
    boss.apply_transform(Rx)
    boss.apply_translation([spine_x1 + sock_len / 2.0, 0.0, sock_z])
    tube_bore = HP._cyl(LEG_TUBE_OD / 2.0 + LEG_TUBE_SOCKET_CLEAR, sock_len * 2)
    tube_bore.apply_transform(Rx)
    tube_bore.apply_translation([spine_x1 + sock_len, 0.0, sock_z])

    # Transverse retention-pin cross-hole through socket-wall / tube /
    # socket-wall (epoxy bond + dia-2.5 roll pin locks pull-out AND spin).
    # Pin axis along Y (perpendicular to the tube), at mid-engagement.
    Ry = HP.rotation_matrix(np.pi / 2.0, [1, 0, 0])  # cyl axis Z -> Y
    pin = HP._cyl(LEG_TUBE_PIN_OD / 2.0, (LEG_TUBE_OD / 2.0 + 3.0) * 4)
    pin.apply_transform(Ry)
    pin.apply_translation([spine_x1 + LEG_TUBE_PIN_INSET, 0.0, sock_z])

    yoke = HP._union(top, bot, stub, spine, boss)
    yoke = HP._diff(yoke, tube_bore, pin)
    return yoke


def make_testfit_cradle(*, back_bearing: bool = True) -> "HP.trimesh.Trimesh":
    """Sandwich test cradle: front mount plate + two side walls + a partial
    back housing carrying the passive-side ball bearing.

    - FRONT (output) face: seats against the plate underside (depth datum)
      and the body bolts up with 4x self-tapping screws into the case.  The
      heads are counterbored flush so the driven disc horn seats flat on the
      plate top and spins free.
    - BACK (idler) face: a partial +X-half housing press-fits a ball bearing
      (default 688, 8x16x5) coaxial with the output -> the joint's second
      bearing (dual-sided support).  The -X half stays OPEN for the
      connectors, cover and cables.

    Validate: bolt the servo to the front plate, press the real bearing into
    the back pocket and confirm the OD fit; the moving link's stub shaft
    (next design step) will ride the inner race so the output is supported
    at BOTH ends, not cantilevered."""
    parts = [make_mount_plate(counterbore=True)]

    # Two side walls hugging the body +/-Y faces over the full body height.
    wall_inner_y = BODY_D / 2.0 + BODY_CL
    wall_w = BODY_W + 2 * PLATE_MARGIN
    for sy in (-1, 1):
        wall = HP._box((wall_w, WALL_T, BODY_H),
                       center=(0.0,
                                sy * (wall_inner_y + WALL_T / 2.0),
                                BODY_H / 2.0))
        parts.append(wall)

    if back_bearing:
        parts.append(make_back_bearing_plate())

    return HP._union(*parts)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--part", choices=("housing", "yoke"), default="housing",
                    help="which piece to export: the fixed bearing HOUSING "
                         "(default) or the moving YOKE that straddles it")
    ap.add_argument("--plate-only", action="store_true",
                    help="export just the bolt-pattern plate (fastest print)")
    ap.add_argument("--no-back-bearing", action="store_true",
                    help="omit the passive-side ball-bearing back housing")
    ap.add_argument("--counterbore", action="store_true",
                    help="counterbore the 4 mount heads flush into the plate")
    ap.add_argument("--out", default=None,
                    help="output STL path (default stl_prototype/sts3215_testfit[_yoke].stl)")
    args = ap.parse_args(argv)

    if args.part == "yoke":
        mesh = make_testfit_yoke()
        default_name = "sts3215_testfit_yoke.stl"
    elif args.plate_only:
        mesh = make_mount_plate(counterbore=args.counterbore)
        default_name = "sts3215_testfit.stl"
    else:
        mesh = make_testfit_cradle(back_bearing=not args.no_back_bearing)
        default_name = "sts3215_testfit.stl"

    out = Path(args.out) if args.out else (
        _HERE.parent / "stl_prototype" / default_name)
    out.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(out)

    b = mesh.bounds
    print(f"Wrote {out}")
    print(f"  watertight: {mesh.is_watertight}")
    print(f"  triangles:  {len(mesh.faces):,}")
    print(f"  bbox (mm):  X {b[0][0]:.1f}..{b[1][0]:.1f}  "
          f"Y {b[0][1]:.1f}..{b[1][1]:.1f}  Z {b[0][2]:.1f}..{b[1][2]:.1f}")
    if args.part == "yoke":
        print(f"  driven-horn bolts (x,y): "
              + ", ".join(f"({hx:.2f},{hy:.2f})" for hx, hy in horn_bolt_centres()))
        print(f"  stub-shaft dia: {BEARING_BORE - 0.2:.1f} (rides 688 bore "
              f"{BEARING_BORE:.1f});  tube socket bore: "
              f"{LEG_TUBE_OD + 2 * LEG_TUBE_SOCKET_CLEAR:.1f} (Phi {LEG_TUBE_OD:.0f} CF tube)")
    else:
        print(f"  4x mount centres (x,y): "
              + ", ".join(f"({hx:.2f},{hy:.2f})" for hx, hy in mount_hole_centres()))
        print(f"  mount-hole dia: {MOUNT_SCREW_CLEAR:.1f} (M2/M2.5 self-tap into "
              f"dia-{SERVO_CASE_HOLE_OD:.1f} case holes)")
        if not args.plate_only and not args.no_back_bearing:
            print(f"  bearing pocket: dia {BEARING_OD - BEARING_PRESS:.1f} x "
                  f"{BEARING_W:.1f} deep (bore {BEARING_BORE:.1f}) on the +X back half")


if __name__ == "__main__":
    main()
