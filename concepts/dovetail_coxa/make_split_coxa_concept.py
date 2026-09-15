"""CONCEPT: the real coxa_link re-split at the platform plane with a
tapered dovetail + pin quick-release.

Takes the actual one-piece ``coxa_link`` from hexapod_prototype.py and
splits it at z = YAW_HUB_PLATFORM_Z1 (+19, the historical Part A / Part B
joint):

  * coxa_hub_qr.stl  -- everything below +19 (yaw hub: horn bolts,
    bearing seats) + a tapered male dovetail RAIL added on the platform
    top.  The 5 vertical head-access shafts stay open through the rail,
    so the M3 x 20 horn bolts are still dropped in from above -- but now
    they are torqued ONCE, with the leg OFF and the whole platform top
    exposed.  This piece never leaves the robot.
  * coxa_leg_qr.stl  -- everything above +19 (foot plate, hip cradle,
    688 housing) with the matching female SOCKET carved into the foot
    plate's underside.  The slot exits outboard (+X), so the whole leg
    slides on radially and the 2 deg/flank taper wedges it snug at full
    seat.  Foot plate is 8 mm thick; a 5 mm slot leaves a 3 mm roof.
  * one vertical (phi) 4 pin through the seated overlap (inboard of the
    bolt circle) blocks slide-out.  Pull pin -> slide leg off.  No tools.

NOT a production change: pure concept viz for BuildViz project
`dovetail-coxa-concept`, build `split-coxa`.  hexapod_prototype.py is
imported read-only; nothing in the verified parts registry moves.

Run:  <prototype .venv python> concepts/dovetail_coxa/make_split_coxa_concept.py
"""
from __future__ import annotations

import json
import os

import numpy as np
import trimesh

HERE = os.path.abspath(os.path.dirname(__file__))
PROTO_DIR = os.path.abspath(os.path.join(HERE, "..", ".."))
STL_DIR = os.path.join(HERE, "stl")

import hexapod_prototype as hp  # noqa: E402  (read-only import)

# ---- dovetail parameters (concept; coupons tune the clearance) -------
RAIL_H = 5.0            # rail height above the platform
RAIL_W_BOT = 20.0       # rail base width
DOVE_OVERHANG = 2.0     # per-side overhang at the rail top (~22 deg flank)
TAPER_DEG = 2.0         # per-flank taper along the slide direction
CLEAR = 0.20            # per-side fit clearance (mid coupon)
PIN_D = 4.2
BIG = 500.0

def _b_union(meshes):
    return trimesh.boolean.union(meshes, engine="manifold")


def _b_diff(meshes):
    return trimesh.boolean.difference(meshes, engine="manifold")


def _b_inter(meshes):
    return trimesh.boolean.intersection(meshes, engine="manifold")


Z_SPLIT = hp.YAW_HUB_PLATFORM_Z1          # +23.0 as of Aug 2026
R_PLAT = max(hp.YAW_HUB_OD, hp.YAW_HUB_DUST_LIP_OD) / 2.0
RAIL_SPAN = 2.0 * R_PLAT                  # rail runs the full platform chord
PIN_X = -(hp.DISC_HORN_BOLT_PCD / 2.0 + 8.0)   # inboard, clear of the shafts


def _tapered_rail(length: float, extra: float = 0.0,
                  taper_deg: float = TAPER_DEG,
                  dip: float = 0.05) -> trimesh.Trimesh:
    """Dovetail rail, axis = X, base plane z=0, narrow end at -X.

    ``dip`` sinks the base below z=0 (volumetric weld for unions --
    never a coplanar kiss for the boolean kernel)."""
    t_total = np.tan(np.deg2rad(taper_deg)) * length
    pts = []
    for x, shrink in ((-length / 2.0, t_total), (length / 2.0, 0.0)):
        hb = RAIL_W_BOT / 2.0 - shrink + extra
        ht = RAIL_W_BOT / 2.0 + DOVE_OVERHANG - shrink + extra
        z0, z1 = -extra - dip, RAIL_H + extra
        pts += [(x, -hb, z0), (x, hb, z0), (x, -ht, z1), (x, ht, z1)]
    return trimesh.convex.convex_hull(np.array(pts, dtype=float))


def _halfspace(below: bool) -> trimesh.Trimesh:
    box = trimesh.creation.box(extents=(BIG, BIG, BIG))
    box.apply_translation([0, 0, Z_SPLIT + (-BIG / 2 if below else BIG / 2)])
    return box


def _pin_cut() -> trimesh.Trimesh:
    c = trimesh.creation.cylinder(radius=PIN_D / 2.0, height=30.0,
                                  sections=48)
    c.apply_translation([PIN_X, 0.0, Z_SPLIT + 5.0])
    return c


def _shaft_cuts() -> list[trimesh.Trimesh]:
    """Keep the 5 horn-bolt head-access shafts open through the rail."""
    r = hp.DISC_HORN_BOLT_PCD / 2.0
    stations = [(0.0, 0.0)] + [(r * np.cos(t), r * np.sin(t))
                               for t in hp.DISC_HORN_BOLT_ANGLES_RAD]
    cuts = []
    for sx, sy in stations:
        c = trimesh.creation.cylinder(
            radius=hp.YAW_HUB_HORN_HEAD_CB_OD / 2.0,
            height=RAIL_H + 4.0, sections=48)
        c.apply_translation([sx, sy, Z_SPLIT + (RAIL_H + 4.0) / 2.0 - 1.0])
        cuts.append(c)
    return cuts


def main() -> None:
    os.makedirs(STL_DIR, exist_ok=True)

    # Load the exported (heal-passed, watertight) STL rather than calling
    # make_coxa_link_part(): the in-memory builder tolerates non-manifold
    # intermediates that the boolean engine here rejects.
    print("loading exported coxa_link.stl ...")
    link = trimesh.load(os.path.join(PROTO_DIR, "stl_prototype",
                                     "coxa_link.stl"))
    assert link.is_volume, "exported coxa_link.stl must be a clean volume"

    print("splitting at z = %+.1f ..." % Z_SPLIT)
    hub = _b_inter([link, _halfspace(below=True)])
    leg = _b_inter([link, _halfspace(below=False)])

    # ---- hub side: + male rail (trimmed to platform), shafts open ----
    # The rail profile must match the female cut exactly, so it can't be
    # dipped into the hub (the hull would widen the flanks).  Instead a
    # plain rectangular ROOT, narrower than the rail base, sinks 1.5 mm
    # below the split plane as the volumetric weld.
    rail = _tapered_rail(RAIL_SPAN)
    rail.apply_translation([0, 0, Z_SPLIT])
    root = trimesh.creation.box(extents=(RAIL_SPAN, RAIL_W_BOT - 4.0, 3.0))
    root.apply_translation([0, 0, Z_SPLIT])
    added = _b_union([rail, root])
    plat = trimesh.creation.cylinder(radius=R_PLAT, height=RAIL_H + 6.0,
                                     sections=96)
    plat.apply_translation([0, 0, Z_SPLIT + (RAIL_H + 6.0) / 2.0 - 2.5])
    added = _b_inter([added, plat])
    print("  added rail volume:", added.is_volume)
    hub_qr = _b_union([hub, added])
    print("  hub+rail volume:", hub_qr.is_volume)
    hub_qr = _b_diff([hub_qr, *_shaft_cuts(), _pin_cut()])

    # ---- leg side: carve the socket, open outboard for the slide -----
    slot = _tapered_rail(RAIL_SPAN, extra=CLEAR)
    exit_prism = _tapered_rail(120.0, extra=CLEAR, taper_deg=0.0)
    exit_prism.apply_translation([RAIL_SPAN / 2.0 + 60.0 - 0.5, 0, 0])
    slot = _b_union([slot, exit_prism])
    slot.apply_translation([0, 0, Z_SPLIT])
    leg_qr = _b_diff([leg, slot, _pin_cut()])

    pin = trimesh.creation.cylinder(radius=2.0, height=16.0, sections=48)

    for name, mesh in (("coxa_hub_qr.stl", hub_qr),
                       ("coxa_leg_qr.stl", leg_qr),
                       ("coxa_qr_pin.stl", pin)):
        mesh.export(os.path.join(STL_DIR, name))
        print(f"  {name:22s} watertight={mesh.is_watertight} "
              f"vol={mesh.volume/1000:.1f}cm3")

    # sanity: seated pair must not interpenetrate
    inter = _b_inter([hub_qr, leg_qr])
    print("seated overlap volume:",
          0.0 if inter.is_empty else round(inter.volume, 3), "mm^3")

    # ---- scene: original | seated | mid-slide -------------------------
    link.export(os.path.join(STL_DIR, "coxa_link_original.stl"))

    def _mat(tx=0.0, ty=0.0, tz=0.0):
        return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
                float(tx), float(ty), float(tz), 1]

    def inst(iid, mesh_id, name, ptype, color, tx=0.0, ty=0.0, tz=0.0):
        return {"id": iid, "meshId": mesh_id, "name": name,
                "partType": ptype, "role": "concept",
                "leg": None, "joint": None, "cots": False,
                "color": color, "transform": _mat(tx, ty, tz)}

    COUPON_Y = 150.0
    scene = {
        "name": "coxa dovetail quick-release — concept",
        "source": "make_split_coxa_concept.py",
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": [0, 0, 0],
        "meshes": [
            {"id": "stl:hub", "name": "coxa_hub_qr.stl",
             "url": "stl/coxa_hub_qr.stl"},
            {"id": "stl:leg", "name": "coxa_leg_qr.stl",
             "url": "stl/coxa_leg_qr.stl"},
            {"id": "stl:pin", "name": "coxa_qr_pin.stl",
             "url": "stl/coxa_qr_pin.stl"},
            {"id": "stl:orig", "name": "coxa_link_original.stl",
             "url": "stl/coxa_link_original.stl"},
            {"id": "stl:cmale", "name": "dovetail_male_nominal.stl",
             "url": "stl/dovetail_male_nominal.stl"},
            {"id": "stl:cpin", "name": "pin_d4x22.stl",
             "url": "stl/pin_d4x22.stl"},
            {"id": "stl:cf10", "name": "dovetail_female_c10.stl",
             "url": "stl/dovetail_female_c10.stl"},
            {"id": "stl:cf20", "name": "dovetail_female_c20.stl",
             "url": "stl/dovetail_female_c20.stl"},
            {"id": "stl:cf30", "name": "dovetail_female_c30.stl",
             "url": "stl/dovetail_female_c30.stl"},
        ],
        "instances": [
            # --- the redesigned part (drive with the Motion scrubber) ---
            inst("hub-qr", "stl:hub",
                 "hub side — stays bolted to horn forever",
                 "coxa_hub_qr", "#4878b0"),
            inst("leg-qr", "stl:leg", "leg side — whole leg slides off",
                 "coxa_leg_qr", "#c44e52"),
            inst("pin-qr", "stl:pin", "phi4 lock pin",
                 "coxa_qr_pin", "#404040", tx=PIN_X, tz=Z_SPLIT + 9.0),
            # --- today's part for comparison ---
            inst("orig-compare", "stl:orig",
                 "TODAY: one-piece coxa_link (comparison)",
                 "coxa_link_original", "#9aa0a6", ty=-130.0),
            # --- print-fit coupons ---
            inst("coupon-male", "stl:cmale",
                 "coupon: male rail (nominal)", "dovetail_male",
                 "#4878b0", ty=COUPON_Y),
            inst("coupon-f10", "stl:cf10",
                 "coupon: female c=0.10 (1 notch), seated",
                 "dovetail_female", "#c44e52", ty=COUPON_Y, tz=8.0),
            inst("coupon-pin", "stl:cpin", "coupon: phi4 pin",
                 "coxa_qr_pin", "#404040", tx=-4.0, ty=COUPON_Y, tz=11.0),
            inst("coupon-f20", "stl:cf20",
                 "coupon: female c=0.20 (2 notches)", "dovetail_female",
                 "#dd8452", tx=70.0, ty=COUPON_Y),
            inst("coupon-f30", "stl:cf30",
                 "coupon: female c=0.30 (3 notches)", "dovetail_female",
                 "#e8b04c", tx=-70.0, ty=COUPON_Y),
        ],
        # Prismatic joints drive the removal in the Motion scrubber.
        "joints": [
            {"id": "qr-pin", "type": "prismatic", "axis": [0, 0, 1],
             "origin": [PIN_X, 0.0, Z_SPLIT], "instances": ["pin-qr"],
             "limits": {"min": 0.0, "max": 30.0}, "home": 0,
             "label": "lock pin (pull up)"},
            {"id": "qr-slide", "type": "prismatic", "axis": [1, 0, 0],
             "origin": [0.0, 0.0, Z_SPLIT], "instances": ["leg-qr"],
             "limits": {"min": 0.0, "max": 70.0}, "home": 0,
             "label": "leg slide (outboard)"},
        ],
        "poses": [
            {"id": "locked", "name": "Locked (riding)",
             "jointValues": {"qr-pin": 0.0, "qr-slide": 0.0}},
            {"id": "pin-pulled", "name": "Pin pulled",
             "jointValues": {"qr-pin": 24.0, "qr-slide": 0.0}},
            {"id": "leg-off", "name": "Leg off",
             "jointValues": {"qr-pin": 24.0, "qr-slide": 65.0}},
        ],
        "animations": [
            {"id": "remove-leg", "name": "Remove leg (pull pin, slide off)",
             "loop": True, "duration": 6.0,
             "keyframes": [
                 {"t": 0.0, "jointValues": {"qr-pin": 0.0, "qr-slide": 0.0}},
                 {"t": 0.8, "jointValues": {"qr-pin": 24.0, "qr-slide": 0.0}},
                 {"t": 1.2, "jointValues": {"qr-pin": 24.0, "qr-slide": 0.0}},
                 {"t": 2.6, "jointValues": {"qr-pin": 24.0, "qr-slide": 65.0}},
                 {"t": 3.4, "jointValues": {"qr-pin": 24.0, "qr-slide": 65.0}},
                 {"t": 4.8, "jointValues": {"qr-pin": 24.0, "qr-slide": 0.0}},
                 {"t": 5.4, "jointValues": {"qr-pin": 0.0, "qr-slide": 0.0}},
                 {"t": 6.0, "jointValues": {"qr-pin": 0.0, "qr-slide": 0.0}},
             ]},
        ],
    }
    with open(os.path.join(HERE, "scene_split.json"), "w",
              encoding="utf-8") as fh:
        json.dump(scene, fh, indent=1)

    # ---- preview: Y=0 cross-section of the seated joint ---------------
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9.5, 6.2), dpi=130)
    # fixed plane frame: plot coords = (coxa X, coxa Z) for BOTH meshes
    T = np.array([[1.0, 0, 0, 0], [0, 0, 1.0, 0],
                  [0, -1.0, 0, 0], [0, 0, 0, 1.0]])
    for mesh, color, label in ((hub_qr, "#4878b0", "hub side (permanent)"),
                               (leg_qr, "#c44e52", "leg side (slides off)")):
        sec = mesh.section(plane_origin=[0, 0, 0], plane_normal=[0, 1, 0])
        if sec is None:
            continue
        planar, _ = sec.to_2D(to_2D=T)
        first = True
        for poly in planar.polygons_full:
            xs, ys = poly.exterior.xy
            ax.fill(xs, ys, color=color, alpha=0.55,
                    label=label if first else None)
            first = False
            for ring in poly.interiors:
                xs, ys = ring.xy
                ax.fill(xs, ys, color="white")
    ax.axhline(Z_SPLIT, color="k", lw=0.8, ls="--")
    ax.annotate("split plane z=+19 (old Part A/B joint)",
                (55, Z_SPLIT + 1), fontsize=9)
    ax.add_patch(plt.Rectangle((PIN_X - PIN_D / 2, Z_SPLIT - 6),
                               PIN_D, 16, facecolor="#404040",
                               label="phi4 lock pin"))
    ax.annotate("dovetail rail\n(taper wedges on slide-in)",
                (0, Z_SPLIT + RAIL_H - 1), fontsize=9, ha="center",
                xytext=(-52, Z_SPLIT + 22),
                arrowprops=dict(arrowstyle="->"))
    ax.annotate("leg slides off outboard →", (36, Z_SPLIT + 8), fontsize=10)
    ax.set_aspect("equal")
    ax.set_xlabel("coxa-local X (outboard →)  [mm]")
    ax.set_ylabel("Z [mm]")
    ax.legend(loc="upper left", fontsize=9)
    ax.set_title("real coxa_link, split at the platform plane — section at Y=0")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "preview_split.png"))
    print("wrote scene_split.json + preview_split.png")


if __name__ == "__main__":
    main()
