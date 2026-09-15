"""Generate a 110 mm hex mount plate (SVG cut file + 2 mm STL).

Aug 2026 (user: "make me a flat hexagon plate with max diameter 110 and
screw holes about 48mm apart in a square lining up with the bottom chassis
that I can mount above the bottom chassis, make it 2mm thick so I can mount
through it easily").

Geometry
--------
  * Hexagon: MAX (vertex-to-vertex) diameter = 110 mm -> circumradius 55,
    flat-to-flat 95.3.  Vertices on the 30/90/150/... deg azimuths, the SAME
    orientation as the chassis plates (``_hex_plate``), so the plate reads as
    a shrunk copy of the bottom chassis.  The coxa-sweep keep-out starts
    100 - 40.4 = ~59.6 mm from centre; the 55 mm circumradius stays inside
    it (clearance asserted at runtime below).
  * Screw holes (BOTH patterns, so the plate fits old AND new chassis):
      1. Old chassis (user's print): 4 x Phi 3.4 M3 on the retired
         35-mm-radius / 45-deg square ``ELEC_CHASSIS_MOUNT_HOLES_XY`` =
         (+/-24.75, +/-24.75) -- 49.5 mm sides ("~48 mm apart").
      2. Current chassis: 4 x Phi 3.4 M3 on
         ``CHASSIS_STANDOFF_HOLES_XY`` = (+/-31.1, +/-31.1) -- 62.2 mm
         sides (the live brass F-F standoff columns).
  * Thickness: 2 mm (STL; for the xTool just cut 2 mm stock).

Outputs (both under ``xtool/``):
  * ``hex_mount_plate_110.svg`` -- true-scale cut file (1 unit = 1 mm,
    red 0.1 mm strokes = cut lines), for xTool Creative Space at 100%%.
  * ``hex_mount_plate_110.stl`` -- the same plate as a 2 mm solid, if you
    would rather print it.

Run from the repo root:

    python hexapod_walker/prototype_sts3215/tools/make_xtool_hex_mount_plate.py
"""
from __future__ import annotations

import os
import sys

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..")))

import hexapod_walker.prototype_sts3215.hexapod_prototype as hp  # noqa: E402
from hexapod_walker.prototype_sts3215.tools.make_xtool_chassis_hex import (  # noqa: E402
    OUT_DIR, _circle, _coxa_sweep_clearance, _ring_to_path, _svg_header,
)

MAX_DIAMETER = 110.0            # mm, vertex-to-vertex (user spec)
THICKNESS = 2.0                 # mm (user spec: thin enough to mount through)
CIRCUMRADIUS = MAX_DIAMETER / 2.0
FLAT_TO_FLAT = MAX_DIAMETER * np.cos(np.pi / 6.0)


def _hole_xy_r():
    """(cx, cy, radius) for every M3 clearance hole.

    Emits the old 49.5 mm square first, then the current 62.2 mm
    standoff square -- both Phi BRACKET_BOLT_HOLE so either chassis
    generation can bolt the plate down.
    """
    r = hp.BRACKET_BOLT_HOLE / 2.0
    for (cx, cy) in hp.ELEC_CHASSIS_MOUNT_HOLES_XY:
        yield cx, cy, r
    for (cx, cy) in hp.CHASSIS_STANDOFF_HOLES_XY:
        yield cx, cy, r


def _hex_coords():
    """Vertex ring, chassis orientation (vertices at 30/90/... deg like
    ``_hex_plate`` after its 30 deg rotation)."""
    angles = np.deg2rad(np.arange(6) * 60.0 + 30.0)
    return [(CIRCUMRADIUS * np.cos(a), CIRCUMRADIUS * np.sin(a))
            for a in angles]


def write_svg(coords) -> str:
    pad = 2.0
    lim = CIRCUMRADIUS + pad
    svg = _svg_header(-lim, -lim, 2 * lim, 2 * lim)
    svg += _ring_to_path(coords)
    for (cx, cy, r) in _hole_xy_r():
        svg += _circle(cx, cy, r)
    svg += "</svg>\n"
    path = os.path.join(OUT_DIR, "hex_mount_plate_110.svg")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(svg)
    return path


def write_stl() -> str:
    plate = trimesh.creation.cylinder(radius=CIRCUMRADIUS, height=THICKNESS,
                                      sections=6)
    plate.apply_transform(rotation_matrix(np.pi / 6.0, [0, 0, 1]))
    holes = []
    for (cx, cy, r) in _hole_xy_r():
        h = trimesh.creation.cylinder(radius=r, height=THICKNESS * 4)
        h.apply_translation([cx, cy, 0.0])
        holes.append(h)
    solid = trimesh.boolean.difference([plate, *holes])
    assert solid.is_watertight
    path = os.path.join(OUT_DIR, "hex_mount_plate_110.stl")
    solid.export(path)
    return path


def write_preview(coords) -> str:
    """Top-view PNG overlay of the plate on chassis_bottom + coxa keep-outs."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon as MplPoly

    fig, ax = plt.subplots(figsize=(7.2, 7.2), dpi=120)
    ax.set_aspect("equal")
    ax.set_title("hex_mount_plate_110 over chassis_bottom (top view, mm)",
                 fontsize=11)

    chassis = _hex_coords_at(hp.CHASSIS_FLAT_TO_FLAT)
    ax.add_patch(MplPoly(chassis, closed=True, fill=False,
                         edgecolor="#9ca3af", linewidth=1.2,
                         label="chassis_bottom outline"))

    # Coxa yaw-sweep keep-outs (same as the simple-coupon clearance model).
    coxa = hp.make_coxa_link()
    reach = float(np.hypot(coxa.vertices[:, 0], coxa.vertices[:, 1]).max())
    for i, (_leg, edge_mid, _R, _R3) in enumerate(hp._leg_chassis_frames()):
        ax.add_patch(Circle((edge_mid[0], edge_mid[1]), reach,
                            facecolor="#f9a8d4", edgecolor="#be185d",
                            alpha=0.25, linewidth=0.6,
                            label="coxa yaw sweep keep-outs" if i == 0 else None))

    ax.add_patch(MplPoly(coords, closed=True, fill=True,
                         facecolor="#93c5fd", edgecolor="#1d4ed8",
                         alpha=0.55, linewidth=1.6,
                         label=f"plate (max dia {MAX_DIAMETER:.0f}, "
                               f"{THICKNESS:.0f} mm thick)"))

    old_pitch = 2.0 * abs(hp.ELEC_CHASSIS_MOUNT_HOLES_XY[0][0])
    new_pitch = 2.0 * abs(hp.CHASSIS_STANDOFF_HOLES_XY[0][0])
    for i, (cx, cy) in enumerate(hp.ELEC_CHASSIS_MOUNT_HOLES_XY):
        ax.add_patch(Circle((cx, cy), hp.BRACKET_BOLT_HOLE / 2.0, fill=False,
                            edgecolor="#b45309", linewidth=1.2,
                            label=(f"old chassis: 4x Ø{hp.BRACKET_BOLT_HOLE:.1f} "
                                   f"on {old_pitch:.1f} mm square") if i == 0
                            else None))
    for i, (cx, cy) in enumerate(hp.CHASSIS_STANDOFF_HOLES_XY):
        ax.add_patch(Circle((cx, cy), hp.BRACKET_BOLT_HOLE / 2.0, fill=False,
                            edgecolor="#111827", linewidth=1.0,
                            label=(f"current: 4x Ø{hp.BRACKET_BOLT_HOLE:.1f} "
                                   f"on {new_pitch:.1f} mm square") if i == 0
                            else None))

    ax.set_xlim(-120, 120)
    ax.set_ylim(-120, 120)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.grid(True, which="both", linestyle=":", linewidth=0.5, alpha=0.6)
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    fig.tight_layout()
    path = os.path.join(OUT_DIR, "hex_mount_plate_110_preview.png")
    fig.savefig(path)
    plt.close(fig)
    return path


def _hex_coords_at(flat_to_flat: float):
    """Chassis-orientation hexagon vertices for a given flat-to-flat size."""
    apothem = flat_to_flat / 2.0
    circum = apothem / np.cos(np.pi / 6.0)
    angles = np.deg2rad(np.arange(6) * 60.0 + 30.0)
    return [(circum * np.cos(a), circum * np.sin(a)) for a in angles]


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    coords = _hex_coords()

    clearance = _coxa_sweep_clearance(coords)
    old_pitch = 2.0 * abs(hp.ELEC_CHASSIS_MOUNT_HOLES_XY[0][0])
    new_pitch = 2.0 * abs(hp.CHASSIS_STANDOFF_HOLES_XY[0][0])
    print(f"hex mount plate: max diameter {MAX_DIAMETER:.0f} mm "
          f"(flat-to-flat {FLAT_TO_FLAT:.1f}), {THICKNESS:.0f} mm thick")
    print(f"  old-chassis holes: 4 x Phi {hp.BRACKET_BOLT_HOLE:.1f} M3, "
          f"{old_pitch:.1f} mm square (+/-24.75) -- your ~48 mm pattern")
    print(f"  current-chassis holes: 4 x Phi {hp.BRACKET_BOLT_HOLE:.1f} M3, "
          f"{new_pitch:.1f} mm square (+/-31.1)")
    print(f"  min clearance to the swept coxa: {clearance:.1f} mm")
    if clearance < 2.0:
        raise RuntimeError(
            f"plate too big: only {clearance:.1f} mm to the coxa sweep")

    from shapely.geometry import Point, Polygon
    poly = Polygon(coords)
    margin = min(poly.exterior.distance(Point(cx, cy)) - r
                 for cx, cy, r in _hole_xy_r())
    print(f"  material beyond the closest hole edge: {margin:.1f} mm")

    print(f"wrote {write_svg(coords)}")
    print(f"wrote {write_stl()}")
    print(f"wrote {write_preview(coords)}")
    print("SVG imports into xTool Creative Space at 100% scale "
          "(real mm units); red strokes = cut lines.  On your old "
          "chassis use the inner 49.5 mm square; on a current print use "
          "the outer 62.2 mm standoff square.")


if __name__ == "__main__":
    main()
