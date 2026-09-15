#!/usr/bin/env python3
"""Export STEP machining files for the premade-C-horn fit hardware."""
from __future__ import annotations

import math
import tomllib
from pathlib import Path

from build123d import Align, Box, Cylinder, Pos, Rotation, export_step


HERE = Path(__file__).resolve().parent
STEP_DIR = HERE / "step"
with (HERE / "hardware_config.toml").open("rb") as config_file:
    CONFIG = tomllib.load(config_file)

SPACER_T = float(CONFIG["spacers"]["thickness_mm"])
SPACER_OD = float(CONFIG["spacers"]["outside_diameter_mm"])
SPACER_M3_D = float(CONFIG["spacers"]["m3_clearance_diameter_mm"])
DRIVE_POCKET_D = float(
    CONFIG["spacers"]["driven_head_pocket_diameter_mm"]
)
DRIVE_POCKET_DEPTH = float(CONFIG["spacers"]["driven_head_pocket_depth_mm"])
DRIVE_TOOL_D = float(CONFIG["spacers"]["driven_tool_access_diameter_mm"])
PASSIVE_CENTER_D = float(
    CONFIG["spacers"]["passive_center_through_diameter_mm"]
)
PCD = float(CONFIG["front"]["usual_pattern_pcd_mm"])
FRONT_M3_D = float(CONFIG["front"]["m3_clearance_diameter_mm"])
OUTSIDE_SPAN = float(CONFIG["bracket"]["outside_span_mm"])
FRONT_WIDTH = float(CONFIG["bracket"]["blade_and_front_width_mm"])
FRONT_CENTER_D = float(CONFIG["bracket"]["center_hole_diameter_mm"])
EXTRA_INNER_EDGE_GAP = float(
    CONFIG["front"]["extra_pair_inner_edge_gap_mm"]
)
EXTRA_OUTER_EDGE_SPAN = float(
    CONFIG["front"]["extra_pair_outer_edge_span_mm"]
)
EXTRA_PHYSICAL_HOLE_D = (
    EXTRA_OUTER_EDGE_SPAN - EXTRA_INNER_EDGE_GAP
) / 2.0
EXTRA_CENTRE_SPAN = (
    EXTRA_OUTER_EDGE_SPAN + EXTRA_INNER_EDGE_GAP
) / 2.0
EDGE_INSET = (OUTSIDE_SPAN - EXTRA_CENTRE_SPAN) / 2.0
assert EXTRA_PHYSICAL_HOLE_D > 0.0
assert EXTRA_CENTRE_SPAN < OUTSIDE_SPAN


def _cyl(radius: float, height: float, z0: float = 0.0):
    return Pos(0, 0, z0) * Cylinder(
        radius, height, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )


def _pcd_centres() -> list[tuple[float, float]]:
    radius = PCD / 2.0
    return [
        (radius * math.cos(angle), radius * math.sin(angle))
        for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)
    ]


def _spacer_base():
    part = _cyl(SPACER_OD / 2.0, SPACER_T)
    for x, y in _pcd_centres():
        part -= Pos(x, y, -1.0) * Cylinder(
            SPACER_M3_D / 2.0,
            SPACER_T + 2.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
    return part


def make_driven_spacer():
    part = _spacer_base()
    part -= _cyl(DRIVE_POCKET_D / 2.0, DRIVE_POCKET_DEPTH + 0.1, -0.1)
    part -= _cyl(DRIVE_TOOL_D / 2.0, SPACER_T + 0.2, -0.1)
    return part


def make_passive_spacer():
    return _spacer_base() - _cyl(
        PASSIVE_CENTER_D / 2.0, SPACER_T + 0.2, -0.1
    )


def _cyl_x(radius: float, length: float, x0: float = -1.0,
           y: float = 0.0, z: float = 0.0):
    return Pos(x0, y, z) * Rotation(0, 90, 0) * Cylinder(
        radius, length, align=(Align.CENTER, Align.CENTER, Align.MIN)
    )


def make_front_coupon():
    thickness = 4.0
    part = Pos(0.0, -FRONT_WIDTH / 2.0, -OUTSIDE_SPAN / 2.0) * Box(
        thickness,
        FRONT_WIDTH,
        OUTSIDE_SPAN,
        align=(Align.MIN, Align.MIN, Align.MIN),
    )
    part -= _cyl_x(FRONT_CENTER_D / 2.0, thickness + 2.0)
    for y, z in _pcd_centres():
        part -= _cyl_x(FRONT_M3_D / 2.0, thickness + 2.0, y=y, z=z)
    for z in (-OUTSIDE_SPAN / 2.0 + EDGE_INSET,
              OUTSIDE_SPAN / 2.0 - EDGE_INSET):
        part -= _cyl_x(FRONT_M3_D / 2.0, thickness + 2.0, z=z)
    return part


def main() -> None:
    STEP_DIR.mkdir(parents=True, exist_ok=True)
    stale_support = STEP_DIR / "rear_support_column.step"
    if stale_support.exists():
        stale_support.unlink()
        print(f"removed obsolete separate part {stale_support}")
    parts = {
        "driven_spacer_7mm.step": make_driven_spacer(),
        "passive_spacer_7mm.step": make_passive_spacer(),
        "front_6hole_fit_coupon.step": make_front_coupon(),
    }
    for filename, part in parts.items():
        path = STEP_DIR / filename
        export_step(part, path)
        print(f"wrote {path} ({part.volume:.1f} mm^3)")


if __name__ == "__main__":
    main()
