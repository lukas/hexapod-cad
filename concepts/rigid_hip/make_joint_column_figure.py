"""Regenerate joint_column_annotated.png for the rigid-hip variant.

Radial section through the leg-0 yaw axis, zoomed on the joint column,
with every z plane annotated: the Aug 25 BEARING-ON-THE-DECK stack (the
6805 race sits 0.5 mm over the servo case top on a deck-level tower
ledge) plus the SHORTENED coxa hub column (Aug 24 rev 3 skirt/platform
delete + Aug 25 M3x30 -> M3x20 horn screws; the cradle slab drops
COL_DROP = 14 mm total, so the hip servo sits just above the bearing).

Run from anywhere:  uv run python make_joint_column_figure.py
Requires the freshly generated stl/ meshes (run make_rigid_hip_variant
first).
"""
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
PROTO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, PROTO)
sys.path.insert(0, HERE)

import hexapod_prototype as hp  # noqa: E402
import make_rigid_hip_variant as rv  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from trimesh.transformations import rotation_matrix  # noqa: E402

STL = os.path.join(HERE, "stl")


def load(key):
    return trimesh.load(os.path.join(STL, rv.MESH_FILES[key][1]))


T = rv.leg_transforms(0)
a = 0.5 * np.pi / 3.0
Rz = rv._rotz(-a)  # leg-0 radial -> +x

yaw_servo_M = T["coxa"] @ rv._trans(
    [-hp.SERVO_OUTPUT_X, 0.0, -(hp.HORN_STACK_H + hp.WELL_RIM_Z)])

sections = [
    ("chassis_bottom", np.eye(4), "#8b93a6", 0.75,
     "chassis_bottom_rigid (static; full-wrap tower)"),
    ("yaw_bearing_upper", T["coxa"] @ rv._trans([0.0, 0.0, rv.YAWBR_DROP]),
     "#303030", 0.9, "6805-2RS -- fully housed, top flush with the rim"),
    ("coxa_link", T["coxa"], "#7ba1d1", 0.75,
     "coxa_link_rigid (rotates; SHORT column, Aug 24 rev 3)"),
    ("servo_body", T["hip_cap"], "#c9a227", 0.55,
     "hip servo (in the lowered cradle)"),
    ("servo_body", yaw_servo_M, "#c9a227", 0.55, "yaw servo (below deck)"),
    ("yaw_servo_retainer", T["cradle"], "#9aa0a6", 0.55, None),
]

fig, ax = plt.subplots(figsize=(11.5, 8.4), dpi=140)
for key, M, color, alpha, label in sections:
    m = load(key)
    m.apply_transform(Rz @ M)
    sec = m.section(plane_origin=[rv.APOTHEM, 0, 0], plane_normal=[0, 1, 0])
    if sec is None:
        continue
    planar, _ = sec.to_2D(to_2D=rotation_matrix(-np.pi / 2.0, [1, 0, 0]))
    first = True
    for poly in planar.polygons_full:
        xs, ys = poly.exterior.xy
        ax.fill(xs, ys, color=color, alpha=alpha,
                label=label if (first and label) else None)
        first = False
        for ring in poly.interiors:
            ax.fill(*ring.xy, color="white")

A = rv.APOTHEM
DECK = 10.25
HORN = hp.CHASSIS_YAW_OUTPUT_Z                        # 15.25 coxa origin
SEAT = HORN + rv.BR_BOT_LOCAL                         # 10.75 race bottom
RIM = rv.CHB_RIM_W                                    # 17.75
BRIM0 = HORN + rv.BRIM_BOT_Z                          # 18.25
BRIM1 = HORN + rv.BRIM_TOP_Z                          # 20.25
SLAB = HORN + rv.SLAB_BOT_Z                           # 19.25
RING1 = HORN + rv.HUB_RING_Z1                         # 20.25 (= brim top)
HEADS = HORN + rv.HORN_HEAD_SEAT_Z + hp.INSERT_M3_BOLT_HEAD_H  # 26.0
FLOOR = HORN + rv.COXA_FLOOR_Z                        # 27.25
HIP = HORN + rv.COXA_HIP_DROP_V                       # 39.65
OLD_SEAT = HORN + hp.YAW_BEARING_LOWER_BOT_Z          # 15.75 (Aug 24 build)
OLD_RIM = OLD_SEAT + hp.YAW_BEARING_W                 # 22.75 (Aug 24 build)
OLD_FLOOR = FLOOR + rv.HORN_SEAT_DROP                 # 37.25 (M3x30 build)

# z-plane lines with staggered, leader-lined labels (crowded stack)
planes = [
    (DECK, 2.6, "z 10.25   servo-mount deck top = yaw servo case top"),
    (SEAT, 6.4, "z 10.75   race seat LEDGE -- 0.5 over the deck/case "
                "(the physical floor)"),
    (HORN, 10.2, "z 15.25   coxa mount plane (the real \u03a620 disc horn "
                 "is recessed at 4.25..6.25, INSIDE the race bore)"),
    (RIM, 14.0, "z 17.75   tower rim = race top -- \u03a644 column ENDS here"),
    (BRIM0, 17.8, "z 18.25   \u03a638 dust brim underside (0.5 running gap)"),
    (SLAB, 21.6, "z 19.25   cradle slab underside -- DROPPED 14 mm total "
                 "(1.5 over the rim)"),
    (BRIM1, 25.4, "z 20.25   brim top = \u03a629 seat ring top "
                  "(1 mm inside the slab)"),
    (HEADS, 29.2, "z 26.00   M3x20 horn screw head tops "
                  "(seats 10 mm deeper, tips/engagement unchanged)"),
    (FLOOR, 33.0, "z 27.25   servo well floor -- 1.25 over the heads"),
    (HIP, 39.65, "z 39.65   hip axis (was 44.65 with M3x25s, 49.65 with "
                 "M3x30s)"),
]
for z, ytxt, lab in planes:
    ax.axhline(z, color="k", lw=0.5, ls=":")
    ax.annotate(lab, xy=(A + 23.5, z), xytext=(A + 29.5, ytxt),
                fontsize=7.5, va="center",
                arrowprops=dict(arrowstyle="-", lw=0.55, color="#555555",
                                shrinkA=0.0, shrinkB=0.0))


# vertical band braces in the empty pocket left of the tower
def brace(x, z0, z1, text, color):
    ax.annotate("", xy=(x, z0), xytext=(x, z1),
                arrowprops=dict(arrowstyle="<->", color=color, lw=1.2))
    ax.annotate(text, (x - 0.9, (z0 + z1) / 2.0), fontsize=7.5, color=color,
                va="center", ha="center", rotation=90)


brace(A - 29.5, DECK, RIM, "\u03a644 tower: 7.5 mm", "#4a5568")
brace(A - 25.5, SEAT, RIM, "6805 fully housed", "#1a1a1a")
brace(A - 33.5, RIM, FLOOR, "bearing \u2192 servo: 9.5 mm", "#166534")
for z, lab in ((OLD_SEAT, "Aug-24 race seat 15.75"),
               (OLD_RIM, "Aug-24 tower rim 22.75"),
               (OLD_FLOOR, "M3x30-era well floor 37.25")):
    ax.axhline(z, color="#b91c1c", lw=0.6, ls="--", alpha=0.6)
    ax.annotate(lab, xy=(A - 20.0, z),
                fontsize=7.0, color="#b91c1c", style="italic",
                va="bottom", ha="left")
ax.annotate("the old \u201crace can't sit lower, it would rub the horn\u201d"
            " claim was architectural:\nthe horn is recessed BELOW the deck,"
            " so the race presses on the same\n\u03a625.15 hub boss 5 mm"
            " lower and the whole robot follows it down",
            xy=(A + 3.0, OLD_SEAT),
            xytext=(A + 14, -4.6), fontsize=7.5,
            color="#b91c1c", style="italic", va="center",
            arrowprops=dict(arrowstyle="-", lw=0.7, color="#b91c1c",
                            ls="--", shrinkA=0.0, shrinkB=0.0))

ax.set_xlim(A - 46, A + 70)
ax.set_ylim(-9, 48)
ax.set_aspect("equal")
ax.set_xlabel("radial position from body centre [mm]  (leg-0 yaw axis at 100)")
ax.set_ylabel("world Z [mm]")
ax.legend(loc="upper left", fontsize=8)
ax.set_title("rigid-hip joint column, Aug 25: bearing ON the deck, M3x20 "
             "horn screws\n(section through the leg-0 yaw axis)")
fig.tight_layout()
out = os.path.join(HERE, "joint_column_annotated.png")
fig.savefig(out)
print("wrote", out)
