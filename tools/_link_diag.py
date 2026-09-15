"""Throwaway diagnostic: render a sandwich-joint FIXED side (the servo
bracket used by coxa_link / the femur_link knee cradle) with the STS3215 body
seated and the dia-20 disc horn placed on the output spline, so we can
SEE (1) how the servo inserts and (2) whether the horn fits the plate
opening.  A y=0 cross-section shows the horn-vs-plate collision."""
from __future__ import annotations

import os
import sys

import numpy as np
import pyvista as pv

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import hexapod_prototype as hp  # noqa: E402

pv.global_theme.allow_empty_mesh = True
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_link_diag")
os.makedirs(OUT, exist_ok=True)

fixed = hp._sandwich_fixed_side()

servo = hp._servo_envelope()  # origin = back face centre, body spans z[0,H]
horn_z = hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H
horn = hp.make_disc_horn()
horn.apply_translation([hp.SERVO_OUTPUT_X, 0.0, horn_z])

yoke = hp._sandwich_moving_yoke(tube_socket=False)

print(f"SERVO_BODY_H={hp.SERVO_BODY_H} WELL_RIM_Z={hp.WELL_RIM_Z} "
      f"plate_top={hp.WELL_RIM_Z + hp.WELL_PLATE_T}")
print(f"output spline tip z={hp.SERVO_BODY_H + hp.SERVO_OUTPUT_H} "
      f"(SERVO_OUTPUT_H={hp.SERVO_OUTPUT_H})")
print(f"disc horn z-span: {horn_z}..{horn_z + hp.DISC_HORN_H}  OD={hp.DISC_HORN_OD}")
print(f"plate central bore OD={hp.SERVO_OUTPUT_BORE_OD + 0.8}")
print(f"fixed bounds: {np.round(fixed.bounds,1).tolist()}")


def render(name, cpos, items, clip=False):
    p = pv.Plotter(off_screen=True, window_size=(1100, 850))
    p.set_background("white")
    for mesh, color, op in items:
        m = pv.wrap(mesh.copy())
        if clip:
            m = m.clip(normal="y", origin=(0, 0.1, 0))
        p.add_mesh(m, color=color, opacity=op, smooth_shading=True,
                   show_edges=False)
    p.add_axes()
    p.camera_position = cpos
    path = os.path.join(OUT, name)
    p.screenshot(path)
    p.close()
    print("wrote", path)


cap = hp.make_servo_clamp_cap()

ITEMS = [(fixed, "#cfd8dc", 1.0), (servo, "#2e2e33", 1.0),
         (horn, "#d9a521", 1.0), (cap, "#4a90d9", 0.85)]

# Cross-section at y=0 to verify the horn now seats/spins (no plate over it).
render("xsec.png", [(0, -140, 30), (10, 0, 25), (0, 0, 1)], ITEMS, clip=True)
# Iso of the assembled clamshell.
render("iso.png", [(120, -120, 90), (5, 0, 20), (0, 0, 1)], ITEMS)
# Top-down to verify the dia-24 horn opening clears the dia-20 horn.
render("top.png", [(8, 0, 150), (8, 0, 20), (0, 1, 0)], ITEMS)
# Exploded: servo + cap lifted +Y to show the lateral drop-in.
servo_x = hp._servo_envelope(); servo_x.apply_translation([0, 45, 0])
cap_x = hp.make_servo_clamp_cap(); cap_x.apply_translation([0, 40, 0])
render("exploded.png", [(120, -120, 90), (5, 10, 20), (0, 0, 1)],
       [(fixed, "#cfd8dc", 1.0), (servo_x, "#2e2e33", 1.0),
        (cap_x, "#4a90d9", 0.9)])
# With the moving yoke added, iso.
render("with_yoke.png", [(120, -120, 90), (10, 0, 18), (0, 0, 1)],
       ITEMS + [(yoke, "#9467bd", 0.45)])
