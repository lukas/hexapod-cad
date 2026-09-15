"""Throwaway diagnostic: render the stacked Uno Q + buck electronics decks
(lower uno_q_tray + upper buck_tray on shared standoff columns) with the
board visuals seated on their bosses, plus standoff columns for context."""
from __future__ import annotations

import os
import sys

import pyvista as pv

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import hexapod_prototype as hp  # noqa: E402

pv.global_theme.allow_empty_mesh = True
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_deck_diag")
os.makedirs(OUT, exist_ok=True)

# --- z-stack: chassis_top top face at z=0 reference -----------------------
z0 = 0.0
uno_tray_z = z0 + hp.DECK_LEVEL_1_STANDOFF_H
uno_board_z = uno_tray_z + hp.DECK_TRAY_T + hp.DECK_STANDOFF_BOSS_H
buck_tray_z = uno_tray_z + hp.DECK_LEVEL_2_STANDOFF_H
buck_board_z = buck_tray_z + hp.DECK_TRAY_T + hp.DECK_STANDOFF_BOSS_H

uno_tray = hp.make_uno_q_tray(); uno_tray.apply_translation([0, 0, uno_tray_z])
buck_tray = hp.make_buck_tray(); buck_tray.apply_translation([0, 0, buck_tray_z])
uno = hp.make_uno_q_visual(); uno.apply_translation([0, 0, uno_board_z])
buck = hp.make_buck_converter_visual(); buck.apply_translation([0, 0, buck_board_z])

cols = []
for (cx, cy) in hp.DECK_COLUMN_XY:
    c = hp._cyl(2.5, hp.DECK_LEVEL_2_STANDOFF_H)
    c.apply_translation([cx, cy, z0 + hp.DECK_LEVEL_2_STANDOFF_H / 2.0])
    cols.append(c)

# chassis_top slab for context (a thin disc under the deck).
slab = hp._cyl(70.0, 4.0); slab.apply_translation([0, 0, -2.0])

print(f"uno_tray_z={uno_tray_z} uno_board_z={uno_board_z} "
      f"buck_tray_z={buck_tray_z} buck_board_z={buck_board_z}")


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


ITEMS = [
    (slab, "#9bb7d4", 0.5),
    *[(c, "#bdbdbd", 1.0) for c in cols],
    (uno_tray, "#cfd8dc", 1.0),
    (uno, "#1b7a3d", 0.9),
    (buck_tray, "#cfd8dc", 1.0),
    (buck, "#b5651d", 0.9),
]

render("deck_iso.png", [(150, -170, 120), (0, 0, 25), (0, 0, 1)], ITEMS)
render("deck_side.png", [(0, -200, 35), (0, 0, 25), (0, 0, 1)], ITEMS)
render("deck_top.png", [(0, 0, 230), (0, 0, 25), (1, 0, 0)], ITEMS)
