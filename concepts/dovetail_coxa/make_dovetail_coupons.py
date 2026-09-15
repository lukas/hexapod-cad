"""Dovetail quick-release COUPONS for the coxa_link split concept.

Concept (Aug 2026, travel/disassembly review): today a leg removal
disturbs the two precision interfaces (bearing cap 3x M3x8 + retainer
4x M3x6) and the horn bolts are deliberately buried.  Proposal: re-split
the merged coxa_link at its historical Part A / Part B plane
(YAW_HUB_PLATFORM_Z1, the flat platform where the hip bracket stacks on
the yaw hub):

  * HUB SIDE (stays on the robot FOREVER): yaw hub, bolted to the disc
    horn with the captive M3 x 20s, riding the 6805 pair -- gains a
    tapered male dovetail RAIL on its platform top.
  * LEG SIDE (removable): hip bracket + everything outboard -- gains
    the female dovetail SOCKET.  Slides on radially (outboard ->
    inboard); the 2 deg taper wedges it snug at full seat.
  * LOCK: one vertical (phi) 4 mm pin through the seated overlap
    (steel dowel or M4 bolt shank), loaded in shear against slide-out.
    Pull the pin and the leg slides off -- zero tools, zero screws.

These COUPONS are print-fit testers, NOT robot parts: one male rail
block at nominal size + three female sliders at 0.10 / 0.20 / 0.30 mm
per-side clearance (side-notch count 1 / 2 / 3 = loosest fit has the
most notches... no: notches = index 1/2/3 in the table below).  Print
all four flat (rail up / slot up), slide-test, and the snuggest slider
that still seats by hand wins; that clearance becomes the real
coxa_link split's constant.

Outputs: stl/*.stl, scene.json (BuildViz project `dovetail-coxa-concept`),
preview.png.

Run from the repo root:

    python hexapod_walker/prototype_sts3215/concepts/dovetail_coxa/make_dovetail_coupons.py
"""
from __future__ import annotations

import json
import os

import numpy as np
import trimesh

HERE = os.path.abspath(os.path.dirname(__file__))
STL_DIR = os.path.join(HERE, "stl")

# ---- representative dimensions (mm) ---------------------------------------
# Footprint ~ the coxa hub platform (pad ~ (phi) 40, YAW_HUB_PAD_T = 6).
BLOCK_W = 40.0        # Y -- across the slide direction
BLOCK_L = 26.0        # X -- along the slide direction (radial on the robot)
BASE_T = 8.0          # male base block thickness (stands in for the hub)
CAP_T = 6.0           # female slider material above the slot roof

RAIL_H = 6.0          # dovetail rail height
RAIL_W_BOT = 24.0     # rail width at its base (narrow face of the trapezoid)
DOVE_OVERHANG = 2.2   # per-side overhang at the top -> flank angle ~20 deg
TAPER_DEG = 2.0       # per-flank taper along the slide (wedges when seated)

PIN_D = 4.2           # clearance for a (phi) 4 steel dowel / M4 shank
PIN_X = -4.0          # pin station, inboard of centre (deep in the overlap)

# Per-side clearances for the three female fit coupons.  The winner
# becomes the real split's DOVETAIL_FIT_CLEARANCE constant.
FIT_CLEARANCES = (0.10, 0.20, 0.30)   # notch count 1 / 2 / 3

NOTCH = 2.0           # fit-index notch size


def _tapered_rail(w_bot: float, h: float, over: float,
                  length: float, extra: float = 0.0) -> trimesh.Trimesh:
    """Convex tapered dovetail rail centred at the origin, axis = X.

    ``extra`` grows the section uniformly (used to carve the female
    slot with clearance).  The -X end is the NARROW (inboard, seated)
    end: each flank pulls in by tan(TAPER_DEG) * length.
    """
    t = np.tan(np.deg2rad(TAPER_DEG)) * length
    pts = []
    for x, shrink in ((-length / 2.0, t), (length / 2.0, 0.0)):
        hb = w_bot / 2.0 - shrink + extra
        ht = w_bot / 2.0 + over - shrink + extra
        # z=0 is the rail base plane; grow downward a hair (extra) so
        # the female slot cut always reaches through its floor.
        z0 = -extra
        z1 = h + extra
        pts += [(x, -hb, z0), (x, hb, z0), (x, -ht, z1), (x, ht, z1)]
    return trimesh.convex.convex_hull(np.array(pts, dtype=float))


def _box(ext, at=(0, 0, 0)) -> trimesh.Trimesh:
    b = trimesh.creation.box(extents=ext)
    b.apply_translation(at)
    return b


def _pin_hole(z0: float, z1: float) -> trimesh.Trimesh:
    h = z1 - z0
    c = trimesh.creation.cylinder(radius=PIN_D / 2.0, height=h, sections=48)
    c.apply_translation([PIN_X, 0.0, (z0 + z1) / 2.0])
    return c


def make_male() -> trimesh.Trimesh:
    """Hub-side coupon: base block + nominal tapered rail on top."""
    base = _box((BLOCK_L, BLOCK_W, BASE_T), (0, 0, BASE_T / 2.0))
    rail = _tapered_rail(RAIL_W_BOT, RAIL_H, DOVE_OVERHANG, BLOCK_L)
    rail.apply_translation([0, 0, BASE_T])
    solid = trimesh.boolean.union([base, rail])
    hole = _pin_hole(-1.0, BASE_T + RAIL_H + 1.0)
    return trimesh.boolean.difference([solid, hole])


def make_female(clear: float, notches: int) -> trimesh.Trimesh:
    """Leg-side coupon: block with the tapered slot cut at ``clear``
    per-side; ``notches`` fit-index grooves on the +Y face."""
    total_h = RAIL_H + CAP_T
    blk = _box((BLOCK_L, BLOCK_W, total_h), (0, 0, total_h / 2.0))
    # slot cut a hair longer than the block so both X ends are open;
    # shift -X so its taper profile lines up with the male rail's
    slot = _tapered_rail(RAIL_W_BOT, RAIL_H, DOVE_OVERHANG,
                         BLOCK_L + 2.0, extra=clear)
    slot.apply_translation([-1.0, 0.0, 0.0])
    cuts = [slot, _pin_hole(-1.0, total_h + 1.0)]
    for k in range(notches):
        cuts.append(_box((NOTCH, NOTCH * 2.0, NOTCH * 2.0),
                         (BLOCK_L / 2.0 - 4.0 - 6.0 * k,
                          BLOCK_W / 2.0, total_h)))
    return trimesh.boolean.difference([blk, *cuts])


def make_pin() -> trimesh.Trimesh:
    """(phi) 4 x 22 dowel stand-in for the scene."""
    return trimesh.creation.cylinder(radius=2.0, height=22.0, sections=48)


def _mat(tx=0.0, ty=0.0, tz=0.0):
    return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
            float(tx), float(ty), float(tz), 1]


def main() -> None:
    os.makedirs(STL_DIR, exist_ok=True)

    male = make_male()
    pin = make_pin()
    females = {c: make_female(c, i + 1)
               for i, c in enumerate(FIT_CLEARANCES)}

    male.export(os.path.join(STL_DIR, "dovetail_male_nominal.stl"))
    pin.export(os.path.join(STL_DIR, "pin_d4x22.stl"))
    for c, mesh in females.items():
        mesh.export(os.path.join(
            STL_DIR, f"dovetail_female_c{int(round(c * 100)):02d}.stl"))

    # ---- BuildViz scene: seated pair + pin, exploded spares beside ----
    meshes = [
        {"id": "stl:male", "name": "dovetail_male_nominal.stl",
         "url": "stl/dovetail_male_nominal.stl"},
        {"id": "stl:pin", "name": "pin_d4x22.stl", "url": "stl/pin_d4x22.stl"},
    ] + [
        {"id": f"stl:female{int(c*100):02d}",
         "name": f"dovetail_female_c{int(c*100):02d}.stl",
         "url": f"stl/dovetail_female_c{int(c*100):02d}.stl"}
        for c in FIT_CLEARANCES
    ]

    def inst(iid, mesh_id, name, ptype, color, tx=0, ty=0, tz=0):
        return {"id": iid, "meshId": mesh_id, "name": name,
                "partType": ptype, "role": "concept", "leg": None,
                "joint": None, "cots": False, "color": color,
                "transform": _mat(tx, ty, tz)}

    instances = [
        # Assembled joint: male at origin, snuggest female seated on it,
        # pin dropped through.  (Female slot z0 == male rail z0 when the
        # female sits at z = BASE_T.)
        inst("male-assembled", "stl:male",
             "hub side (stays bolted to horn)", "dovetail_male",
             "#4878b0"),
        inst("female-assembled", "stl:female10",
             "leg side, seated (c=0.10)", "dovetail_female", "#c44e52",
             tz=BASE_T),
        inst("pin-assembled", "stl:pin", "phi4 lock pin", "pin",
             "#9aa0a6", tx=PIN_X, tz=(BASE_T + RAIL_H + CAP_T) / 2.0 + 2.0),
        # Exploded spares: the two other fit coupons + a spare pin.
        inst("female-c20", "stl:female20", "fit coupon c=0.20 (2 notches)",
             "dovetail_female", "#dd8452", tx=0, ty=60.0, tz=0),
        inst("female-c30", "stl:female30", "fit coupon c=0.30 (3 notches)",
             "dovetail_female", "#e8b04c", tx=0, ty=-60.0, tz=0),
        inst("pin-spare", "stl:pin", "spare pin", "pin", "#9aa0a6",
             tx=40.0, ty=0.0, tz=11.0),
    ]

    scene = {
        "name": "dovetail coxa quick-release — fit coupons",
        "source": "make_dovetail_coupons.py",
        "units": "mm",
        "center": [0, 0, 0],
        "meshes": meshes,
        "instances": instances,
    }
    with open(os.path.join(HERE, "scene.json"), "w", encoding="utf-8") as fh:
        json.dump(scene, fh, indent=1)

    # ---- preview.png: section + top view ------------------------------
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPoly, Circle, Rectangle

    fig, (ax_sec, ax_top) = plt.subplots(1, 2, figsize=(10.5, 4.6), dpi=130)

    # cross-section at the seated (narrow, x=-L/2) end
    t = np.tan(np.deg2rad(TAPER_DEG)) * BLOCK_L
    hb, ht = RAIL_W_BOT / 2.0 - t, RAIL_W_BOT / 2.0 + DOVE_OVERHANG - t
    ax_sec.add_patch(Rectangle((-BLOCK_W / 2, 0), BLOCK_W, BASE_T,
                               facecolor="#4878b0", alpha=.85,
                               label="hub side (permanent)"))
    ax_sec.add_patch(MplPoly([(-hb, BASE_T), (hb, BASE_T),
                              (ht, BASE_T + RAIL_H), (-ht, BASE_T + RAIL_H)],
                             facecolor="#4878b0", alpha=.85))
    ax_sec.add_patch(Rectangle((-BLOCK_W / 2, BASE_T), BLOCK_W,
                               RAIL_H + CAP_T, facecolor="#c44e52",
                               alpha=.45, label="leg side (slides off)"))
    ax_sec.plot([], [])
    ax_sec.set_title(f"section at seated end — flank ≈20°, taper {TAPER_DEG}°/side")
    ax_sec.set_xlim(-28, 28); ax_sec.set_ylim(-2, 24)
    ax_sec.set_aspect("equal"); ax_sec.legend(fontsize=8, loc="upper right")

    # top view: taper + pin
    for sgn in (1, -1):
        ax_top.plot([-BLOCK_L / 2, BLOCK_L / 2],
                    [sgn * (RAIL_W_BOT / 2 + DOVE_OVERHANG - t),
                     sgn * (RAIL_W_BOT / 2 + DOVE_OVERHANG)],
                    color="#4878b0", lw=2)
    ax_top.add_patch(Rectangle((-BLOCK_L / 2, -BLOCK_W / 2), BLOCK_L,
                               BLOCK_W, fill=False, edgecolor="#c44e52",
                               lw=1.5, label="leg-side block"))
    ax_top.add_patch(Circle((PIN_X, 0), PIN_D / 2, facecolor="#9aa0a6",
                            edgecolor="k", label="φ4 lock pin"))
    ax_top.annotate("slide on/off →", (BLOCK_L / 2 + 2, 0), fontsize=9)
    ax_top.annotate("robot centre ←", (-BLOCK_L / 2 - 24, 0), fontsize=9)
    ax_top.set_title("top — rail tapers inboard, pin blocks slide-out")
    ax_top.set_xlim(-52, 42); ax_top.set_ylim(-30, 30)
    ax_top.set_aspect("equal"); ax_top.legend(fontsize=8, loc="lower right")

    fig.suptitle("coxa_link dovetail quick-release — print-fit coupons "
                 f"(clearances {', '.join(f'{c:.2f}' for c in FIT_CLEARANCES)} mm)")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "preview.png"))

    for name in sorted(os.listdir(STL_DIR)):
        m = trimesh.load(os.path.join(STL_DIR, name))
        print(f"{name:36s} watertight={m.is_watertight}  "
              f"ext={np.round(m.extents, 1)}")
    print("wrote scene.json + preview.png")


if __name__ == "__main__":
    main()
