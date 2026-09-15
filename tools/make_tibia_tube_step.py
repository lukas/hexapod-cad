"""Write ``extra_stl/tibia_tube_printable.step`` (+ matching .stl): the
tibia's Ø8 carbon-fibre tube as a printable replacement part.

Aug 2026 (user: vacation spare -- no access to CF tube stock, so a
3D-printable stand-in for one tibia segment).  Geometry mirrors the
as-designed CF tube exactly:

  * Ø ``hp.LEG_TUBE_OD`` (8 mm) OD x Ø 6 mm ID (``hp.LEG_TUBE_WALL``
    = 1 mm wall), straight tube.
  * Length = ``hp.TIBIA_LENGTH`` - ``hp.FOOT_BOOT_TIP_L`` = 142 mm:
    the tube bottoms flush at the yoke-socket station (joint-local
    x = ``hp._YOKE_SOCKET_X``) and the TPU boot's 8 mm spherical-ended
    solid section
    carries the last stretch to the 150 mm tip station (see the
    ``tibia_tube`` placement in ``leg_named_parts_in_body_frame``).
  * Plain ends: epoxy-only retention at the yoke socket, and the TPU
    ``foot_boot`` press-fits over the last 20 mm with its Ø8.1 bore.

As-built note: the doc prose talks about recutting tubes "at 150 mm"
-- that number is the yoke-face-to-foot-tip SPAN (= TIBIA_LENGTH,
includes the boot's 8 mm rounded solid end), not the raw cut length.  The
model's tube solid is 142 mm; ``--length`` overrides if a bench
measurement of the tube being replaced disagrees.

Printing: PETG (or PLA), 100% infill / all-perimeter, printed LYING
DOWN so the layers run along the tube (vertical printing puts every
layer seam across the bending load of foot strike -- it will snap at a
seam).  Pass ``--solid`` to skip the Ø6 bore for a stronger print (the
bore serves no function on a printed part; both sockets are blind).

Local frame: tube axis = +Z, z = 0 at the KNEE (yoke-socket) end.

The repo ``.venv`` (Python 3.14) has no OCC/STEP kernel and build123d's
OCP wheels stop at 3.12/3.13, so run this through a uv side-environment
(cached after the first run; the .venv is untouched).  From the repo
root:

    uv run --no-project --python 3.12 \
        --with build123d --with trimesh --with numpy \
        python hexapod_walker/prototype_sts3215/tools/make_tibia_tube_step.py
"""
from __future__ import annotations

import argparse
import os


import hexapod_prototype as hp

from build123d import Cylinder, Pos, export_step, export_stl

OUT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "extra_stl"))
OUT_STEM = "tibia_tube_printable"

NOMINAL_LENGTH = hp.TIBIA_LENGTH - hp.FOOT_BOOT_TIP_L   # 142 mm


def make_tibia_tube(length: float, *, solid: bool = False):
    """The tibia leg-segment tube as a build123d BREP solid."""
    r_out = hp.LEG_TUBE_OD / 2.0
    r_in = r_out - hp.LEG_TUBE_WALL
    body = Pos(0, 0, length / 2.0) * Cylinder(r_out, length)
    if not solid:
        body -= Pos(0, 0, length / 2.0) * Cylinder(r_in, length + 2.0)
    return body


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--length", type=float, default=NOMINAL_LENGTH,
                    help=f"tube cut length in mm (default {NOMINAL_LENGTH:.0f})")
    ap.add_argument("--solid", action="store_true",
                    help="omit the Ø6 bore (stronger print, same fit)")
    args = ap.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    part = make_tibia_tube(args.length, solid=args.solid)

    step_path = os.path.join(OUT_DIR, OUT_STEM + ".step")
    stl_path = os.path.join(OUT_DIR, OUT_STEM + ".stl")
    export_step(part, step_path)
    export_stl(part, stl_path)

    bb = part.bounding_box()
    print(f"wrote {step_path}")
    print(f"wrote {stl_path}")
    print(f"  length {args.length:.1f} mm, "
          f"OD {hp.LEG_TUBE_OD:.1f} / "
          f"{'SOLID' if args.solid else f'ID {hp.LEG_TUBE_OD - 2 * hp.LEG_TUBE_WALL:.1f}'} mm")
    print("  plain ends; epoxy-only yoke retention, press-fit boot end")
    print(f"  bbox: {bb.size.X:.2f} x {bb.size.Y:.2f} x {bb.size.Z:.2f} mm")
    print(f"  volume: {part.volume:.0f} mm^3")


if __name__ == "__main__":
    main()
