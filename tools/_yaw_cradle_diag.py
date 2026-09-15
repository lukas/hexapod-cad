"""Throwaway diagnostic: render one yaw cradle + placed servo + disc horn
so we can see the retention / horn-clearance / cable-exit problems."""
from __future__ import annotations

import os
import sys

import numpy as np
import pyvista as pv

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import hexapod_prototype as hp  # noqa: E402

pv.global_theme.allow_empty_mesh = True
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_yaw_diag")
os.makedirs(OUT, exist_ok=True)


def _pv(mesh):
    return pv.wrap(mesh.copy())


# --- cradle-local geometry (mirror _chassis_yaw_cradle_solid math) ---
cradle = hp._chassis_yaw_cradle_solid()

out_z = hp.CHASSIS_YAW_OUTPUT_Z - hp.CHASSIS_PLATE_T / 2.0
plate_top_z = out_z - hp.HORN_STACK_H
front_face_z = plate_top_z - hp.WELL_PLATE_T
body_back_z = front_face_z - hp.SERVO_BODY_H

servo = hp.make_servo_body()
servo.apply_translation([-hp.SERVO_OUTPUT_X, 0.0, body_back_z])

horn = hp.make_disc_horn()
horn.apply_translation([0.0, 0.0, plate_top_z])


def _coxa(yaw_deg):
    cl = hp.make_coxa_link()
    cl.apply_transform(hp.rotation_matrix(np.radians(yaw_deg), [0, 0, 1]))
    cl.apply_translation([0.0, 0.0, out_z])
    return cl

# A slab of chassis plate around the cradle for context (z in [-4, 0]).
slab = hp._box((hp.WELL_W + 40.0, hp.WELL_D + 40.0, hp.CHASSIS_PLATE_T),
               center=(-hp.SERVO_OUTPUT_X, 0.0, -hp.CHASSIS_PLATE_T / 2.0))
# punch the cradle footprint out of the slab so they don't z-fight
slab = hp._diff(slab, hp._box((hp.WELL_W + 6.0, hp.WELL_D + 6.0, 10.0),
                              center=(-hp.SERVO_OUTPUT_X, 0.0, 0.0)))

print(f"out_z={out_z}  plate_top_z={plate_top_z}  front_face_z={front_face_z}"
      f"  body_back_z={body_back_z}")
print(f"chassis plate spans z[-4,0]; servo bottom(back) z={body_back_z:.1f}")
print(f"servo body bbox z: {servo.bounds[0][2]:.1f}..{servo.bounds[1][2]:.1f}")
print(f"horn bbox z: {horn.bounds[0][2]:.1f}..{horn.bounds[1][2]:.1f}")


def render(name, cpos, items):
    p = pv.Plotter(off_screen=True, window_size=(1100, 850))
    p.set_background("white")
    for mesh, color, op in items:
        p.add_mesh(pv.wrap(mesh.copy()), color=color, opacity=op,
                   smooth_shading=True)
    p.add_axes()
    p.camera_position = cpos
    path = os.path.join(OUT, name)
    p.screenshot(path)
    p.close()
    print("wrote", path)


retainer = hp.make_yaw_servo_retainer()
retainer.apply_translation([0.0, 0.0, body_back_z - hp.RETAINER_STRAP_T])
print(f"retainer bbox: x {retainer.bounds[0][0]:.1f}..{retainer.bounds[1][0]:.1f}"
      f"  z {retainer.bounds[0][2]:.1f}..{retainer.bounds[1][2]:.1f}")

BASE = [(slab, "#9bb7d4", 1.0), (cradle, "#d0d0d0", 1.0)]
SERVO = [(servo, "#2e2e33", 1.0), (horn, "#c0c0c8", 1.0),
         (retainer, "#d9534f", 1.0)]


# iso from below to show the open-bottom cavity
render("cradle_below.png", [(60, -90, -70), (-12, 0, 0), (0, 0, 1)],
       BASE + SERVO)
# iso from above to show horn + plate top
render("cradle_above.png", [(60, -90, 70), (-12, 0, 10), (0, 0, 1)],
       BASE + SERVO)
# side view to read the z-stack; cradle semi-transparent
render("cradle_side.png", [(0, -160, 5), (-12, 0, 0), (0, 0, 1)],
       [(slab, "#9bb7d4", 1.0), (cradle, "#d0d0d0", 0.35),
        (servo, "#2e2e33", 1.0), (horn, "#c0c0c8", 1.0)])

# coxa_link mounted on the horn, viewed from the side, at yaw 0 and +25 deg,
# to see whether the link sweeps into the raised cradle plateau.
for ang in (0, 25):
    render(f"coxa_side_{ang}.png", [(0, -170, 25), (0, 0, 22), (0, 0, 1)],
           [(slab, "#9bb7d4", 1.0), (cradle, "#d0d0d0", 0.45),
            (horn, "#c0c0c8", 1.0), (_coxa(ang), "#9467bd", 0.85)])
render("coxa_top.png", [(0, 0, 160), (-6, 0, 22), (1, 0, 0)],
       [(slab, "#9bb7d4", 1.0), (cradle, "#d0d0d0", 1.0),
        (_coxa(0), "#9467bd", 0.6)])
