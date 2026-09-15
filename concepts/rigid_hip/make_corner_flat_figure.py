"""Annotated before/after figure of the FLATTENED wire corridor + cradle
shell (rev 5, Aug 24 + rev 6, Aug 25).

User decision, Aug 24 rev 5: "take a step back that CRADLE WALL isnt
doing shit, just flatten it out and its fine" / "also this isnt wall
flattening, its flatening random bumps in the top of the chassis plate
that serve no purpose".  The wago-era wire-corridor apparatus inboard
of each seated yaw servo (cradle end wall + porch canopy + side-wall
stubs) is cut back to the bare sheet in ``chassis_bottom_rigid``; this
figure shows the leg-0 corridor before (production geometry rebuilt
from hexapod_prototype) and after (the shipped variant STL), plan
sections at the wall band (z=5) and the canopy/deck band (z=9.5).

Run from anywhere:  uv run python make_corner_flat_figure.py
Requires stl/chassis_bottom_rigid.stl (run make_rigid_hip_variant
first).  Output: corner_flattened_annotated.png
"""
import os
import sys

import numpy as np
import trimesh
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402

HERE = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
sys.path.insert(0, HERE)

import hexapod_prototype as hp  # noqa: E402
import make_rigid_hip_variant as mv  # noqa: E402

OUT = os.path.join(HERE, "corner_flattened_annotated.png")

print("building the production chassis (BEFORE panel) ...", flush=True)
before = hp._heal_for_export(hp.make_chassis_bottom())
after = trimesh.load(os.path.join(HERE, "stl", "chassis_bottom_rigid.stl"))

# world -> leg-0 frame (leg azimuths are (i+0.5)*60; leg 0 = az 30)
A = 0.5 * np.pi / 3.0
RINV = np.linalg.inv(mv._rotz(A))
for m in (before, after):
    m.apply_transform(RINV)

# seated yaw servo silhouette in the leg frame (measured: probe3)
SERVO = dict(x0=64.8, x1=110.2, y=12.4)
CUT = dict(x0=mv.CHB_FLAT_X0, x1=mv.CHB_FLAT_X1, y=mv.CHB_FLAT_HALF_Y)
RET_PILOTS = [(100.0 + hp.RETAINER_ANCHOR_RADIAL, sy * hp.RETAINER_ANCHOR_TANG)
              for sy in (+1, -1)] + \
             [(100.0 + hp.RETAINER_ANCHOR_RADIAL_2, sy * hp.RETAINER_ANCHOR_TANG)
              for sy in (+1, -1)]


def sect_polys(mesh, zc):
    sec = mesh.section(plane_origin=[0, 0, zc], plane_normal=[0, 0, 1])
    if sec is None:
        return []
    T = np.eye(4)
    T[2, 3] = -zc
    planar, _ = sec.to_planar(to_2D=T)
    return list(planar.polygons_full)


fig, axes = plt.subplots(2, 2, figsize=(14.5, 9.6), dpi=120)
panels = [
    (axes[0][0], before, 5.0, "BEFORE — wall band (z=5)"),
    (axes[0][1], after, 5.0, "AFTER — wall band (z=5): bare sheet"),
    (axes[1][0], before, 9.5, "BEFORE — canopy/deck band (z=9.5)"),
    (axes[1][1], after, 9.5, "AFTER — canopy/deck band (z=9.5)"),
]
for ax, mesh, zc, title in panels:
    for poly in sect_polys(mesh, zc):
        b = poly.bounds
        if b[2] < 30 or b[0] > 125 or b[3] < -45 or b[1] > 45:
            continue
        xs, ys = np.array(poly.exterior.coords).T
        ax.fill(xs, ys, color="#6b7488" if zc < 8 else "#c5cbd8", zorder=2)
        for hole in poly.interiors:
            hx, hy = np.array(hole.coords).T
            ax.fill(hx, hy, color="white", zorder=3)
    ax.add_patch(Rectangle((SERVO["x0"], -SERVO["y"]),
                           SERVO["x1"] - SERVO["x0"], 2 * SERVO["y"],
                           fc="#3d3d3d", alpha=0.55, zorder=4))
    ax.add_patch(Rectangle((CUT["x0"], -CUT["y"]),
                           CUT["x1"] - CUT["x0"], 2 * CUT["y"],
                           fill=False, ec="#c03030", lw=1.6, ls="--",
                           zorder=6))
    ax.add_patch(Circle((100.0, 0.0), mv.CHB_KEEP_R, fill=False,
                        ec="#c03030", lw=1.6, ls=":", zorder=6))
    for (px, py) in RET_PILOTS:
        ax.plot(px, py, "o", ms=5, mfc="#2c8a2c", mec="#145014", zorder=7)
    ax.set_xlim(38, 122)
    ax.set_ylim(-42, 42)
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3, alpha=0.5)
    ax.set_title(title, fontsize=10, fontweight="bold")
    ax.set_xlabel("leg-frame x (yaw axis at 100) [mm]")
    ax.set_ylabel("y [mm]")

bx = dict(boxstyle="round,pad=0.25", fc="#fff8d8", ec="#a08c30", lw=0.7)
ar = dict(arrowstyle="->", color="#c03030", lw=1.3)
axes[0][0].annotate("cradle END WALL (x 61.5–64.5,\nfull height z 2→10.25,"
                    "\n0.3 mm off the servo face)\n— FLATTENED",
                    xy=(63.0, 8.0), xytext=(40.5, 26), fontsize=7.5,
                    bbox=bx, arrowprops=ar, zorder=9)
axes[0][0].annotate("cradle-shell side walls (|y| 13.2–18.9)\n— rev 5 cut to "
                    "x 64.65, rev 6 removes\nthe WHOLE run to the tower keep",
                    xy=(70.0, -16.0), xytext=(40.5, -35), fontsize=7.5,
                    bbox=bx, arrowprops=ar, zorder=9)
axes[0][0].annotate("seated yaw servo\n(hangs z −28→10.25)",
                    xy=(85.0, 5.0), xytext=(80, 30), fontsize=7.5,
                    bbox=dict(boxstyle="round,pad=0.25", fc="#eeeeee",
                              ec="#555555"), zorder=9,
                    arrowprops=dict(arrowstyle="->", color="#333333", lw=1.1))
axes[1][0].annotate("porch canopy skin (z≈8→10.25)\nover the floorless "
                    "harness corridor\n— FLATTENED",
                    xy=(57.0, 0.0), xytext=(40.5, 26), fontsize=7.5,
                    bbox=bx, arrowprops=ar, zorder=9)
axes[0][1].annotate("cut box x 50→100, |y|≤20.5,\nMINUS the tower keep "
                    "circle\n(r 21.95 about the yaw axis)",
                    xy=(70.0, -20.5), xytext=(40.5, -35), fontsize=7.5,
                    bbox=bx, zorder=9,
                    arrowprops=dict(arrowstyle="->", color="#c03030", lw=1.1))
axes[1][1].annotate("shell INSIDE the keep circle: KEPT\n(well-mouth collar + "
                    "pocket floor —\ncarries the 6805 seat's inboard arc)",
                    xy=(85.0, 15.0), xytext=(40, 28), fontsize=7.5,
                    bbox=dict(boxstyle="round,pad=0.25", fc="#e2f0e2",
                              ec="#2c8a2c"), zorder=9,
                    arrowprops=dict(arrowstyle="->", color="#2c8a2c", lw=1.2))
axes[1][1].annotate("retainer plate pilots (green):\nin-sheet z −2..1, "
                    "outside the cut\n(retainer itself is below the plate)",
                    xy=(RET_PILOTS[1][0], RET_PILOTS[1][1]),
                    xytext=(42, -35), fontsize=7.5,
                    bbox=dict(boxstyle="round,pad=0.25", fc="#e2f0e2",
                              ec="#2c8a2c"), zorder=9,
                    arrowprops=dict(arrowstyle="->", color="#2c8a2c", lw=1.2))

fig.suptitle("rigid_hip rev 5+6: wire-corridor bumps AND the cradle-shell "
             "walls/roof are FLATTENED to the tower keep (user decision) — "
             "leg-0, before (production) vs after (chassis_bottom_rigid)",
             fontsize=12, fontweight="bold")
fig.tight_layout(rect=(0, 0, 1, 0.96))
fig.savefig(OUT, bbox_inches="tight")
print("wrote", OUT)
