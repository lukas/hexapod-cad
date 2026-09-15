"""Static PyVista render of the arm assembly preview.

Calls into arm.py to build the per-part meshes in chassis frame
(rather than re-loading the unioned STL), so we can colour the
chassis, frame parts, motors, etc. separately for a cleaner image.

  ./run.sh hexapod_walker/prototype/arm/render_arm_preview.py
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pyvista as pv
import trimesh
from trimesh.transformations import rotation_matrix

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

import arm as ARM   # noqa: E402

RENDER_DIR = os.path.join(THIS_DIR, "renders")
OUT_PATH = os.path.join(RENDER_DIR, "arm_assembly_preview.png")

# Three categories, plain hobby-servo colour palette (same hues as the
# prototype renderer for visual consistency).
FRAME_COLOR    = "#dfe4e7"
CHASSIS_COLOR  = "#b6bdc2"
GRIPPER_COLOR  = "#9aa0a8"


def _to_pv(tm: trimesh.Trimesh) -> "pv.PolyData":
    tm = tm.copy()
    tm.apply_scale(0.001)   # mm -> m
    return pv.wrap(tm)


def main() -> None:
    os.makedirs(RENDER_DIR, exist_ok=True)

    plotter = pv.Plotter(
        off_screen=True,
        window_size=(1920, 1200),
        lighting="three lights",
    )
    plotter.set_background("#b8c5d4")

    # Re-build the meshes in chassis frame so we can colour them
    # separately.
    chassis_top_z = ARM.HP.CHASSIS_PLATE_T / 2.0
    chassis_top = ARM.HP.make_chassis_top()
    arm_parts = ARM._arm_in_chassis_frame(chassis_top_z=chassis_top_z)

    # First arm_part is the base bracket (frame); next two are shoulder
    # horn + shoulder link; next two are upper + forearm; then wrist
    # adapter; gripper_base; then two jaws.
    # We assign rough categories so the user can tell them apart.
    categories = [
        ("base_bracket",   FRAME_COLOR),
        ("j1_horn",        "#2a2c30"),
        ("shoulder_link",  FRAME_COLOR),
        ("upper_arm",      FRAME_COLOR),
        ("forearm",        FRAME_COLOR),
        ("wrist_adapter",  FRAME_COLOR),
        ("gripper_base",   GRIPPER_COLOR),
        ("jaw_left",       GRIPPER_COLOR),
        ("jaw_right",      GRIPPER_COLOR),
    ]
    if len(arm_parts) != len(categories):
        sys.exit(f"part count mismatch: got {len(arm_parts)} parts, "
                 f"expected {len(categories)}")

    # Collect bounds across all meshes so the camera framing is right.
    xmin = ymin = zmin = float("inf")
    xmax = ymax = zmax = float("-inf")

    def _add(mesh: trimesh.Trimesh, color: str, specular: float = 0.35):
        nonlocal xmin, ymin, zmin, xmax, ymax, zmax
        bb = mesh.bounds
        lo, hi = bb[0] * 0.001, bb[1] * 0.001
        xmin, ymin, zmin = min(xmin, lo[0]), min(ymin, lo[1]), min(zmin, lo[2])
        xmax, ymax, zmax = max(xmax, hi[0]), max(ymax, hi[1]), max(zmax, hi[2])
        plotter.add_mesh(
            _to_pv(mesh),
            color=color,
            smooth_shading=False,
            specular=specular,
            specular_power=30.0,
        )

    _add(chassis_top, CHASSIS_COLOR, specular=0.25)
    for (_name, color), mesh in zip(categories, arm_parts):
        _add(mesh, color, specular=0.4 if color == "#2a2c30" else 0.35)

    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    cz = 0.5 * (zmin + zmax)
    span = max(xmax - xmin, ymax - ymin, zmax - zmin, 0.05)

    # Ground plane just below the chassis-top.
    floor = pv.Plane(
        center=(cx, cy, zmin - 0.002),
        direction=(0, 0, 1),
        i_size=span * 3.5,
        j_size=span * 3.5,
    )
    plotter.add_mesh(floor, color="#cfcdc6", opacity=1.0)

    dist = max(span * 1.35, 0.55)
    az = math.radians(45.0)
    el = math.radians(22.0)
    fx = cx + 0.04 * span      # nudge focus toward the arm tip
    fy = cy
    fz = cz + 0.02 * span

    pos = (
        fx + dist * math.cos(el) * math.cos(az),
        fy + dist * math.cos(el) * math.sin(az),
        fz + dist * math.sin(el) + 0.08,
    )
    plotter.camera_position = [pos, (fx, fy, fz), (0, 0, 1)]
    plotter.screenshot(OUT_PATH)
    print(f"Wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
