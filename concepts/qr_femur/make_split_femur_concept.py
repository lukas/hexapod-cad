"""CONCEPT: the one-piece ``femur_link`` re-split at the inter-well gap
with a drawbored square tenon + quick-release pin.

Takes the actual exported ``femur_link.stl`` (Aug 2026: hip yoke +
solid Phi 18 spar + knee bracket, one printed body) and splits it at
the spar's inter-well gap -- the only free span, joint-local x in
[50, 58.3], whose Phi 24 gusset-cone envelope is already swept-clear:

  * femur_hip_qr.stl  -- the hip moving yoke (clevis arms + doubled
    8 mm spine).  The hip gusset cone is replaced by a Phi 24 SOCKET
    BOSS on the spine face; a rounded-square pocket is broached through
    boss + full 8 mm spine (~15 mm engagement, >= 5 mm walls -- not the
    3 mm hoop that cracked in the field).  This piece never leaves the
    robot: its 8 hip disc-horn bolts become install-once, so horn
    registration / trim is never disturbed.
  * femur_knee_qr.stl -- knee bracket + tibia side, ending in a
    tapered (1 deg/flank) rounded-square TENON, 11 x 11 mm, corner
    r2.  Torsion (the ~1.8 N-m lateral-foot-catch case) and bending
    (~0.5 N-m planar) react as bearing on the flats at a few MPa --
    far under the ~12 MPa PETG threshold used for the crack-fix FEA.
  * one Phi 4 ball-detent pin through boss + tenon blocks pull-out
    (the only load the pin sees).  DRAWBORE: print the tenon's pin
    bore offset 0.2 mm deeper so seating the pin wedges the taper
    snug -- no chopstick play, no tools, no printed threads to wear.

Print-fit coupons (male tenon block + three female sockets at 0.10 /
0.20 / 0.30 mm per-side clearance, edge-notch count 1/2/3) pick the
real clearance for this printer/material, exactly like the dovetail
coxa coupons.

NOT a production change: pure concept viz for BuildViz project
`qr-femur-concept`, build `split-femur`.  hexapod_prototype.py and the
exported STL are read-only inputs; nothing in the verified parts
registry moves.  Gates before production: _closed_yoke_diag sweep
survey (boss mid-span + pin head), femur torque FEA on both halves,
coupon-tuned clearance.

Run:  <repo .venv python> concepts/qr_femur/make_split_femur_concept.py
"""
from __future__ import annotations

import json
import os

import numpy as np
import trimesh

HERE = os.path.abspath(os.path.dirname(__file__))
PROTO_DIR = os.path.abspath(os.path.join(HERE, "..", ".."))
STL_DIR = os.path.join(HERE, "stl")

# ---------------------------------------------------------------------------
# Frames.  The exported femur_link.stl is in the ASSEMBLED-LINK frame:
#   STL x = joint-local x - 12.5   (hip output axis at STL x = 0)
#   STL y = 41.3 - joint-local z   (disc-horn-top plane at y = 0)
#   STL z = joint-local y          (pivot-axis direction)
# All constants below are STL-frame mm.
# ---------------------------------------------------------------------------
X_SPINE_IN = 29.5     # spine inner face   (joint-local 42)
X_SPINE_OUT = 37.5    # spine outer face   (joint-local 50)
X_KNEE_WALL = 45.8    # knee well wall outer face (joint-local 58.3)
SPAR_Y = 24.15        # femur line (joint-local z = 17.15)
SPAR_Z = 0.0

# ---- joint parameters (concept; coupons tune the clearance) --------------
BOSS_OD = 24.0        # socket boss = the swept-clear gusset-cone envelope
BOSS_BITE = 1.0       # volumetric weld into the spine face
FACE_GAP = 0.5        # seated face gap boss->knee wall; drawbore closes it
TENON_W = 11.0        # rounded-square tenon flat-to-flat at the root
TENON_R = 2.0         # corner radius (kills the sharp-corner seam starter)
TAPER_DEG = 1.0       # per-flank taper along the slide; wedges when seated
CLEAR = 0.20          # per-side fit clearance (mid coupon)
TENON_ROOT_BITE = 1.0 # volumetric weld into the knee piece
PIN_D = 4.2           # bore; pin itself is phi 4
PIN_X = 41.0          # near mid of the exposed boss span [37.5, 45.3];
                      # nudged inboard so the phi9 button head clears the
                      # knee wall face (45.8) AND, because the pin runs
                      # along STL z (the pivot-axis direction), the head
                      # sits proud of the boss's widest line at z = 12 --
                      # outside the spine plate's z extent (a y-axis pin's
                      # head clipped the tall spine plate; measured).
DRAWBORE = 0.2        # documented print-time tenon-bore offset (not modeled)
BIG = 600.0

X_BOSS0 = X_SPINE_OUT - BOSS_BITE          # 36.5
X_BOSS1 = X_KNEE_WALL - FACE_GAP           # 45.3
X_TENON_NOSE = X_SPINE_IN + 0.5            # 30.0 -- sub-flush at the inner face
X_TENON_ROOT = X_KNEE_WALL + TENON_ROOT_BITE   # 46.8
TENON_LEN = X_TENON_ROOT - X_TENON_NOSE    # 16.8


def _b_union(meshes):
    return trimesh.boolean.union(meshes, engine="manifold")


def _b_diff(meshes):
    return trimesh.boolean.difference(meshes, engine="manifold")


def _b_inter(meshes):
    return trimesh.boolean.intersection(meshes, engine="manifold")


def _rounded_square_pts(x: float, half: float, r: float,
                        n_arc: int = 8) -> list[tuple[float, float, float]]:
    """3D points of a rounded-square cross-section (in the y-z plane at
    station ``x``), centred on (SPAR_Y, SPAR_Z), half-width ``half``,
    corner radius ``r``.  Convex, so convex_hull of two stations gives
    the exact tapered tenon."""
    pts = []
    c = half - r
    for cy, cz in ((c, c), (-c, c), (-c, -c), (c, -c)):
        for i in range(n_arc + 1):
            # quarter arc centred on the corner-round centre
            a0 = np.arctan2(cz, cy) - np.pi / 4.0
            a = a0 + (np.pi / 2.0) * i / n_arc
            pts.append((x, SPAR_Y + cy + r * np.cos(a),
                        SPAR_Z + cz + r * np.sin(a)))
    return pts


def _tapered_tenon(extra: float = 0.0,
                   x0: float = X_TENON_NOSE,
                   x1: float = X_TENON_ROOT) -> trimesh.Trimesh:
    """Tenon solid, axis = X, fat end at +X (the knee-side root).

    ``extra`` grows the profile per side (female pocket = tenon grown
    by the fit clearance).  Taper: 1 deg/flank, nominal at the root."""
    shrink = np.tan(np.deg2rad(TAPER_DEG)) * (x1 - x0)
    pts = (_rounded_square_pts(x0, TENON_W / 2.0 - shrink + extra, TENON_R)
           + _rounded_square_pts(x1, TENON_W / 2.0 + extra, TENON_R))
    return trimesh.convex.convex_hull(np.array(pts, dtype=float))


def _halfspace_x(keep_below: bool, x_cut: float) -> trimesh.Trimesh:
    box = trimesh.creation.box(extents=(BIG, BIG, BIG))
    box.apply_translation([x_cut + (-BIG / 2 if keep_below else BIG / 2),
                           SPAR_Y, SPAR_Z])
    return box


def _boss() -> trimesh.Trimesh:
    b = trimesh.creation.cylinder(radius=BOSS_OD / 2.0,
                                  height=X_BOSS1 - X_BOSS0, sections=96)
    b.apply_transform(trimesh.transformations.rotation_matrix(
        np.pi / 2.0, [0, 1, 0]))
    b.apply_translation([(X_BOSS0 + X_BOSS1) / 2.0, SPAR_Y, SPAR_Z])
    return b


def _pin_cut() -> trimesh.Trimesh:
    c = trimesh.creation.cylinder(radius=PIN_D / 2.0, height=40.0,
                                  sections=48)   # axis = z already
    c.apply_translation([PIN_X, SPAR_Y, SPAR_Z])
    return c


def _pin_mesh() -> trimesh.Trimesh:
    """Phi 4 detent-pin stand-in: shaft + button head, axis = STL Z (the
    pivot-axis direction; side access through the 8.3 mm inter-well
    slot).  Head seats proud of the boss's widest line at z = +12,
    exported in place (seated)."""
    shaft = trimesh.creation.cylinder(radius=2.0, height=26.0, sections=48)
    shaft.apply_translation([0, 0, -1.0])            # z -14 .. +12
    head = trimesh.creation.cylinder(radius=4.5, height=3.0, sections=48)
    head.apply_translation([0, 0, 13.5])             # z +12 .. +15
    pin = _b_union([shaft, head])
    pin.apply_translation([PIN_X, SPAR_Y, SPAR_Z])
    return pin


def _coupon_male() -> trimesh.Trimesh:
    """Male coupon: 24 x 20 x 11 block + the nominal tenon off one face,
    tenon bottom band coplanar with the block bottom (prints flat)."""
    block = trimesh.creation.box(extents=(20.0, 24.0, 11.0))
    block.apply_translation([-10.0 + 0.5, 0.0, 0.0])   # 0.5 weld overlap
    ten = _tapered_tenon()
    ten.apply_translation([-X_TENON_NOSE, -SPAR_Y, -SPAR_Z])  # nose at x=0
    m = _b_union([block, ten])
    m.apply_translation([0, 0, 11.0 / 2.0])            # bottom on z=0
    return m


def _coupon_female(clear: float, notches: int) -> trimesh.Trimesh:
    """Female coupon: block with the through-pocket at ``clear`` per side;
    ``notches`` shallow grooves on the +Z edge identify the clearance."""
    block = trimesh.creation.box(extents=(22.0, 24.0, 17.0))
    # Cut well PAST both block faces (a 2 mm overshoot left 0.6 mm caps
    # -> a sealed internal void, caught by the hub's geometry check).
    pocket = _tapered_tenon(extra=clear,
                            x0=X_TENON_NOSE - 8.0, x1=X_TENON_ROOT + 8.0)
    pocket.apply_translation(
        [-(X_TENON_NOSE + X_TENON_ROOT) / 2.0, -SPAR_Y, -SPAR_Z])
    cuts = [pocket]
    for i in range(notches):
        n = trimesh.creation.box(extents=(1.6, 26.0, 2.0))
        n.apply_translation([-7.0 + 3.0 * i, 0.0, 17.0 / 2.0])
        cuts.append(n)
    f = _b_diff([block, *cuts])
    f.apply_translation([0, 0, 17.0 / 2.0])
    return f


def main() -> None:
    os.makedirs(STL_DIR, exist_ok=True)

    # Load the exported (heal-passed, watertight) STL rather than calling
    # make_femur_link(): the in-memory builder tolerates non-manifold
    # intermediates that the boolean engine here rejects.
    print("loading exported femur_link.stl ...")
    link = trimesh.load(os.path.join(PROTO_DIR, "stl_prototype",
                                     "femur_link.stl"))
    assert link.is_volume, "exported femur_link.stl must be a clean volume"

    print("splitting at x = %.1f (spine face) and x = %.1f (knee wall) ..."
          % (X_SPINE_OUT, X_KNEE_WALL))
    hip = _b_inter([link, _halfspace_x(True, X_SPINE_OUT)])
    knee = _b_inter([link, _halfspace_x(False, X_KNEE_WALL)])
    # (the exposed spar + gusset cones in between are discarded)

    # ---- hip side: + socket boss, - pocket, - pin bore ----------------
    hip_qr = _b_union([hip, _boss()])
    print("  hip+boss volume:", hip_qr.is_volume)
    pocket = _tapered_tenon(extra=CLEAR, x0=X_SPINE_IN - 0.5,
                            x1=X_BOSS1 + 0.5)
    hip_qr = _b_diff([hip_qr, pocket, _pin_cut()])

    # ---- knee side: + tenon, - pin bore --------------------------------
    knee_qr = _b_union([knee, _tapered_tenon()])
    print("  knee+tenon volume:", knee_qr.is_volume)
    knee_qr = _b_diff([knee_qr, _pin_cut()])

    pin = _pin_mesh()

    coupons = {
        "tenon_male_nominal.stl": _coupon_male(),
        "tenon_female_c10.stl": _coupon_female(0.10, 1),
        "tenon_female_c20.stl": _coupon_female(0.20, 2),
        "tenon_female_c30.stl": _coupon_female(0.30, 3),
    }

    for name, mesh in (("femur_hip_qr.stl", hip_qr),
                       ("femur_knee_qr.stl", knee_qr),
                       ("femur_qr_pin.stl", pin),
                       *coupons.items()):
        mesh.export(os.path.join(STL_DIR, name))
        print(f"  {name:26s} watertight={mesh.is_watertight} "
              f"vol={mesh.volume/1000:.1f}cm3")

    # sanity: seated pair (+ pin) must not interpenetrate
    for a, b, lab in ((hip_qr, knee_qr, "hip vs knee"),
                      (hip_qr, pin, "hip vs pin"),
                      (knee_qr, pin, "knee vs pin")):
        inter = _b_inter([a, b])
        v = 0.0 if inter.is_empty else inter.volume
        print(f"seated overlap {lab}: {v:.3f} mm^3")
        assert v < 0.5, f"seated interpenetration ({lab})"

    # ---- scene: original | seated | coupons ----------------------------
    link.export(os.path.join(STL_DIR, "femur_link_original.stl"))

    def _mat(tx=0.0, ty=0.0, tz=0.0):
        return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
                float(tx), float(ty), float(tz), 1]

    def inst(iid, mesh_id, name, ptype, color, tx=0.0, ty=0.0, tz=0.0):
        return {"id": iid, "meshId": mesh_id, "name": name,
                "partType": ptype, "role": "concept",
                "leg": None, "joint": None, "cots": False,
                "color": color, "transform": _mat(tx, ty, tz)}

    COUPON_Y = 110.0
    scene = {
        "name": "femur quick-release tenon — concept",
        "source": "make_split_femur_concept.py",
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": [50, 24, 0],
        "meshes": [
            {"id": "stl:hip", "name": "femur_hip_qr.stl",
             "url": "stl/femur_hip_qr.stl"},
            {"id": "stl:knee", "name": "femur_knee_qr.stl",
             "url": "stl/femur_knee_qr.stl"},
            {"id": "stl:pin", "name": "femur_qr_pin.stl",
             "url": "stl/femur_qr_pin.stl"},
            {"id": "stl:orig", "name": "femur_link_original.stl",
             "url": "stl/femur_link_original.stl"},
            {"id": "stl:cmale", "name": "tenon_male_nominal.stl",
             "url": "stl/tenon_male_nominal.stl"},
            {"id": "stl:cf10", "name": "tenon_female_c10.stl",
             "url": "stl/tenon_female_c10.stl"},
            {"id": "stl:cf20", "name": "tenon_female_c20.stl",
             "url": "stl/tenon_female_c20.stl"},
            {"id": "stl:cf30", "name": "tenon_female_c30.stl",
             "url": "stl/tenon_female_c30.stl"},
        ],
        "instances": [
            # --- the split femur (drive with the Motion scrubber) -------
            inst("hip-qr", "stl:hip",
                 "hip yoke — stays bolted to the horns forever",
                 "femur_hip_qr", "#4878b0"),
            inst("knee-qr", "stl:knee",
                 "knee + tibia side — pulls off outboard",
                 "femur_knee_qr", "#c44e52"),
            inst("pin-qr", "stl:pin", "phi4 drawbore detent pin",
                 "femur_qr_pin", "#404040"),
            # --- today's part for comparison -----------------------------
            inst("orig-compare", "stl:orig",
                 "TODAY: one-piece femur_link (comparison)",
                 "femur_link_original", "#9aa0a6", ty=-80.0),
            # --- print-fit coupons ---------------------------------------
            inst("coupon-male", "stl:cmale",
                 "coupon: male tenon (nominal)", "tenon_male",
                 "#4878b0", ty=COUPON_Y),
            inst("coupon-f10", "stl:cf10",
                 "coupon: female c=0.10 (1 notch)", "tenon_female",
                 "#c44e52", tx=50.0, ty=COUPON_Y),
            inst("coupon-f20", "stl:cf20",
                 "coupon: female c=0.20 (2 notches)", "tenon_female",
                 "#dd8452", tx=85.0, ty=COUPON_Y),
            inst("coupon-f30", "stl:cf30",
                 "coupon: female c=0.30 (3 notches)", "tenon_female",
                 "#e8b04c", tx=120.0, ty=COUPON_Y),
        ],
        # Prismatic joints drive the removal in the Motion scrubber.
        "joints": [
            {"id": "qr-pin", "type": "prismatic", "axis": [0, 0, 1],
             "origin": [PIN_X, SPAR_Y, SPAR_Z], "instances": ["pin-qr"],
             "limits": {"min": 0.0, "max": 30.0}, "home": 0,
             "label": "detent pin (pull)"},
            {"id": "qr-slide", "type": "prismatic", "axis": [1, 0, 0],
             "origin": [X_KNEE_WALL, SPAR_Y, SPAR_Z],
             "instances": ["knee-qr"],
             "limits": {"min": 0.0, "max": 90.0}, "home": 0,
             "label": "leg slide (outboard)"},
        ],
        "poses": [
            {"id": "locked", "name": "Locked (walking)",
             "jointValues": {"qr-pin": 0.0, "qr-slide": 0.0}},
            {"id": "pin-pulled", "name": "Pin pulled",
             "jointValues": {"qr-pin": 28.0, "qr-slide": 0.0}},
            {"id": "leg-off", "name": "Leg off",
             "jointValues": {"qr-pin": 28.0, "qr-slide": 85.0}},
        ],
        "animations": [
            {"id": "remove-leg", "name": "Remove leg (pull pin, slide off)",
             "loop": True, "duration": 6.0,
             "keyframes": [
                 {"t": 0.0, "jointValues": {"qr-pin": 0.0, "qr-slide": 0.0}},
                 {"t": 0.8, "jointValues": {"qr-pin": 28.0, "qr-slide": 0.0}},
                 {"t": 1.2, "jointValues": {"qr-pin": 28.0, "qr-slide": 0.0}},
                 {"t": 2.6, "jointValues": {"qr-pin": 28.0, "qr-slide": 85.0}},
                 {"t": 3.4, "jointValues": {"qr-pin": 28.0, "qr-slide": 85.0}},
                 {"t": 4.8, "jointValues": {"qr-pin": 28.0, "qr-slide": 0.0}},
                 {"t": 5.4, "jointValues": {"qr-pin": 0.0, "qr-slide": 0.0}},
                 {"t": 6.0, "jointValues": {"qr-pin": 0.0, "qr-slide": 0.0}},
             ]},
        ],
    }
    with open(os.path.join(HERE, "scene.json"), "w", encoding="utf-8") as fh:
        json.dump(scene, fh, indent=1)

    # ---- preview: Z=0 cross-section of the seated joint ----------------
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10.5, 5.8), dpi=130)
    T = np.eye(4)   # z-normal section: planar coords = (x, y)
    for mesh, color, label in (
            (hip_qr, "#4878b0", "hip yoke side (permanent)"),
            (knee_qr, "#c44e52", "knee + tibia side (pulls off)")):
        sec = mesh.section(plane_origin=[0, 0, 0], plane_normal=[0, 0, 1])
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
    ax.add_patch(plt.Circle((PIN_X, SPAR_Y), 2.0, facecolor="#404040",
                            label="phi4 detent pin (axis into the page)"))
    ax.axvline(X_SPINE_OUT, color="k", lw=0.8, ls="--")
    ax.axvline(X_KNEE_WALL, color="k", lw=0.8, ls="--")
    ax.annotate("spine face 37.5 ", (X_SPINE_OUT, -10), fontsize=8,
                ha="right")
    ax.annotate(" knee wall 45.8", (X_KNEE_WALL, -10), fontsize=8,
                ha="left")
    ax.annotate("phi24 socket boss +\n11x11 tapered tenon\n(~15 mm engagement)",
                (33.5, SPAR_Y + 1), fontsize=9, ha="left",
                xytext=(-9, 28), arrowprops=dict(arrowstyle="->"))
    ax.annotate("drawbore: tenon pin bore offset\n0.2 mm so the pin wedges the\ntaper snug (no play, no tools)",
                (PIN_X + 0.5, SPAR_Y + 2.5), fontsize=9,
                xytext=(52, 48), arrowprops=dict(arrowstyle="->"))
    ax.annotate("knee + tibia pull off outboard →", (63, 43), fontsize=10)
    ax.set_ylim(-13, 60)
    ax.set_aspect("equal")
    ax.set_xlabel("femur STL X (hip axis at 0, knee axis at 90)  [mm]")
    ax.set_ylabel("STL Y [mm]")
    ax.legend(loc="upper left", fontsize=9)
    ax.set_title("real femur_link, split at the inter-well gap — section at Z=0")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "preview.png"))
    print("wrote scene.json + preview.png")


if __name__ == "__main__":
    main()
