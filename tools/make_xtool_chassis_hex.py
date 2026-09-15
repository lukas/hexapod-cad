"""Generate a true-scale SVG of the chassis_bottom hex for xTool laser cutting.

Jul 2026 (user: "make me a design of a hexagon with screw offsets that will
match with the bottom chassis that I can use for a cutting experiment with
xtool").  Two files, both 1 SVG unit = 1 mm (xTool Creative Space imports
SVG at real size when width/height carry mm units):

  * ``xtool/chassis_bottom_hex_simple.svg`` -- a SMALL hexagon coupon
    (SIMPLE_FLAT_TO_FLAT, Jul 2026 user request: "a lot smaller ... fit it
    inside the range of where the coxa could turn to") + the 4 x Phi 3.4 M3
    standoff clearance holes on the CHASSIS_STANDOFF_HOLES_XY diagonal
    pattern (+/-31.1, +/-31.1).  Bolt it to the real chassis through the
    brass F-F standoff columns; the six coxa yaw sweeps clear it.

    Sizing (see check printed at runtime): each coxa yaw axis sits at
    radius 100 (the chassis apothem) on the 30/90/150/... deg azimuths,
    and the WHOLE coxa assembly (printed coxa_link incl. its hip cradle;
    the hip servo envelope stays inside it) sweeps a 40.4 mm-radius disc
    about its axis.  The coupon is deliberately rotated 30 deg vs the big
    chassis outline so a FLAT (not a corner) faces each yaw axis: with
    flat-to-flat 102 the apothem is 51, leaving 100 - 51 - 40.4 = ~8.6 mm
    of clearance to the swept coxa, while the boundary at the 45 deg hole
    azimuths still leaves ~7 mm of material beyond the Phi 3.4 holes.
    (Keeping the chassis' own corner-toward-leg orientation caps the
    corner radius at 100 - 40.4 - margin = ~53, whose apothem ~46 leaves
    < 2 mm around the 44-mm-radius holes -- too flimsy, hence the twist.)

  * ``xtool/chassis_bottom_hex_full.svg`` -- the FULL cross-section of the
    as-built ``make_chassis_bottom()`` mesh, sliced through the flat floor
    slab (z = -5, mid-way through the z in [-6, -2] floor).  Every through
    feature the printed floor carries comes along automatically: the 4
    standoff holes, the 6 servo-body clearance cutouts, the per-leg
    harness-drop slots, the yaw_servo_retainer pilot holes and the battery
    velcro-strap slots.  Because it is cut from the same mesh the printer
    gets, it CANNOT drift out of sync with the CAD.

Both files draw cut lines as 0.1 mm red strokes (fill none), the common
laser-cut convention; assign them a cut layer in xTool Creative Space.
Orientation is the view from ABOVE the plate (+X right, +Y up).

Run from the repo root:

    python hexapod_walker/prototype_sts3215/tools/make_xtool_chassis_hex.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..")))
# hexapod_prototype imports sibling modules (cable_keepouts, ...) by bare
# name, so the project dir itself must be importable too.

import hexapod_walker.prototype_sts3215.hexapod_prototype as hp  # noqa: E402

OUT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "xtool"))

# Slice height: mid-way through the folded floor slab (z in [-6, -2]) so the
# section shows exactly what a flat sheet replacing the floor must contain.
SECTION_Z = -5.0

# Simple-coupon hexagon size (see module docstring for the derivation
# against the coxa yaw-sweep keep-out).
SIMPLE_FLAT_TO_FLAT = 102.0

STROKE = 'fill="none" stroke="#ff0000" stroke-width="0.1"'


def _svg_header(minx: float, miny: float, w: float, h: float) -> str:
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.3f}mm" '
        f'height="{h:.3f}mm" viewBox="{minx:.3f} {miny:.3f} {w:.3f} {h:.3f}">\n'
    )


def _ring_to_path(coords) -> str:
    """A closed polyline as an SVG path (y flipped: SVG y is down, we want
    the view from above the plate with +Y up)."""
    pts = ["%.3f %.3f" % (x, -y) for (x, y) in coords]
    return f'<path {STROKE} d="M {pts[0]} L ' + " L ".join(pts[1:]) + ' Z"/>\n'


def _circle(cx: float, cy: float, r: float) -> str:
    return (f'<circle {STROKE} cx="{cx:.3f}" cy="{-cy:.3f}" '
            f'r="{r:.3f}"/>\n')


def _hexagon_coords(flat_to_flat: float, vertex_offset_deg: float):
    """Hexagon vertex ring.  The big chassis (``_hex_plate``) has vertices
    on the 30/90/150/... deg azimuths (offset 30); the small coupon uses
    offset 0 so its FLATS face the coxa yaw axes instead (see docstring)."""
    apothem = flat_to_flat / 2.0
    circum = apothem / np.cos(np.pi / 6.0)
    angles = np.deg2rad(np.arange(6) * 60.0 + vertex_offset_deg)
    return [(circum * np.cos(a), circum * np.sin(a)) for a in angles]


def _coxa_sweep_clearance(coords) -> float:
    """Min clearance (mm) from the coupon polygon to the six swept-coxa
    keep-out discs (radius = coxa_link's max XY reach about its yaw axis,
    which also envelopes the hip servo)."""
    from shapely.geometry import Point, Polygon

    coxa = hp.make_coxa_link()
    reach = float(np.hypot(coxa.vertices[:, 0], coxa.vertices[:, 1]).max())
    poly = Polygon(coords)
    worst = np.inf
    for _i, edge_mid, _R, _R3 in hp._leg_chassis_frames():
        d = poly.exterior.distance(Point(edge_mid[0], edge_mid[1]))
        if poly.contains(Point(edge_mid[0], edge_mid[1])):
            d = -d
        worst = min(worst, d - reach)
    return worst


def write_simple() -> str:
    """Small hex coupon + the 4 M3 standoff holes only."""
    coords = _hexagon_coords(SIMPLE_FLAT_TO_FLAT, vertex_offset_deg=0.0)

    clearance = _coxa_sweep_clearance(coords)
    print(f"  simple coupon: {SIMPLE_FLAT_TO_FLAT:.0f} mm flat-to-flat, "
          f"min clearance to the swept coxa: {clearance:.1f} mm")
    if clearance < 2.0:
        raise RuntimeError(
            f"coupon too big: only {clearance:.1f} mm to the coxa sweep")

    pad = 2.0
    circum = (SIMPLE_FLAT_TO_FLAT / 2.0) / np.cos(np.pi / 6.0)
    lim = circum + pad
    svg = _svg_header(-lim, -lim, 2 * lim, 2 * lim)
    svg += _ring_to_path(coords)
    for (cx, cy) in hp.CHASSIS_STANDOFF_HOLES_XY:
        svg += _circle(cx, cy, hp.BRACKET_BOLT_HOLE / 2.0)
    svg += "</svg>\n"
    path = os.path.join(OUT_DIR, "chassis_bottom_hex_simple.svg")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(svg)
    return path


def write_full() -> str:
    """Full floor cross-section of the as-built chassis_bottom mesh."""
    mesh = hp.make_chassis_bottom()
    sec = mesh.section(plane_origin=[0.0, 0.0, SECTION_Z],
                       plane_normal=[0.0, 0.0, 1.0])
    if sec is None:
        raise RuntimeError(f"no section at z={SECTION_Z}")
    to_2d = np.eye(4)
    to_2d[2, 3] = -SECTION_Z
    planar, _ = sec.to_2D(to_2D=to_2d)

    rings = []
    minx = miny = np.inf
    maxx = maxy = -np.inf
    for poly in planar.polygons_full:
        for ring in [poly.exterior, *poly.interiors]:
            coords = list(ring.coords)
            rings.append(coords)
            xs, ys = zip(*coords)
            minx, maxx = min(minx, *xs), max(maxx, *xs)
            miny, maxy = min(miny, *ys), max(maxy, *ys)

    pad = 2.0
    svg = _svg_header(minx - pad, -(maxy + pad),
                      (maxx - minx) + 2 * pad, (maxy - miny) + 2 * pad)
    for coords in rings:
        svg += _ring_to_path(coords)
    svg += "</svg>\n"
    path = os.path.join(OUT_DIR, "chassis_bottom_hex_full.svg")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(svg)
    print(f"  full section: {len(rings)} cut paths "
          f"(outline + holes/cutouts), envelope "
          f"{maxx - minx:.1f} x {maxy - miny:.1f} mm")
    return path


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    p1 = write_simple()
    print(f"wrote {p1}")
    p2 = write_full()
    print(f"wrote {p2}")
    print("Import into xTool Creative Space at 100% scale "
          "(the SVGs carry real mm units); red strokes = cut lines.")


if __name__ == "__main__":
    main()
