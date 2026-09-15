"""STOCK-C-HORN VARIANT builder (Aug 2026, user: "I'm also thinking about
buying some stock C shaped horns - can you make a new version of a redesign
that uses stock aluminum horns like that?").

The variant replaces each joint's PRINTED moving clevis (the femur hip yoke
and the tibia knee yoke -- the members behind both field cracks) with a
bought aluminum C-shaped horn: a U/C bracket whose two plates bolt onto the
SAME two 20 mm disc horns the printed yokes bolt to today (driven disc on
the output spline, stock passive disc on the rear idler boss), and whose
outboard web carries a small printed adapter:

  * femur:  ``femur_chorn_body.stl``   = web flange + the Phi 18 solid spar
            (cone flares both ends, same as the shipped femur) + the
            UNCHANGED knee fixed side (cradle, 688 housing, far-wall pad).
  * tibia:  ``tibia_chorn_socket.stl`` = web flange + the Phi 18 CF-tube
            socket boss (Phi 8.1 bore, epoxy-only -- same tube cut length
            and mouth position as the shipped yoke, x = 62 joint-local).

NOTHING in the shipped design changes: the fixed sides (coxa hip cradle,
femur knee cradle), clamp caps, disc horns and every joint frame stay
frozen.  Outputs go to ``extra_stl/chorn/``; the main build, verifier and
BOM are untouched.

WHY SPACERS ARE MANDATORY (measured, not optional): the printed yokes use
reach-down pads because there is NO room for a flat plate riding directly
on the disc faces -- the fixed cradle's own top plate reaches z = 38.3 and
the coxa deck sweeps z 39.3 across r 14..30 (closed-yoke diag), while the
driven disc's top face sits at z = 36.17.  A C-horn plate bolted flush
would collide with both.  Standoff spacers on the four M3 disc bolts lift
the top plate's inner face to 36.17 + t_top; t_top >= 3.6 clears everything
with 0.5 mm margin (we spec 4.0).  The bottom side has free air (the
kept-clear rear volume), so the bottom spacer just absorbs the rest of the
purchased horn's span.

DIMENSIONS MARKED "ASSUMED" BELOW ARE PLACEHOLDERS for the typical 2 mm
aluminum 25T C bracket -- MEASURE THE PURCHASED PART and re-run this
script before printing adapters.  Hard requirements for any candidate horn
are printed by the checks at the end (and in docs/CHORN_VARIANT.md).

Run:  .venv/bin/python tools/make_chorn_variant.py
"""
from __future__ import annotations

import os

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

import hexapod_prototype as hp
from hexapod_prototype import (_box, _cyl, _diff, _union,
                               _disc_horn_bolt_centres)

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "extra_stl", "chorn")

# ---------------------------------------------------------------------------
# Frozen joint geometry this variant must honour (derived, do not edit)
# ---------------------------------------------------------------------------
# Real disc-horn mating faces in the shared joint-local frame (the printed
# yokes' reach-down pads land exactly here):
DISC_TOP_FACE_Z = hp.JOINT_HORN_TOP_Z - (hp.YOKE_ARM_PAD + hp.YOKE_SEAT_INTERF)   # 36.17
DISC_BOT_FACE_Z = hp.JOINT_HORN_BOT_Z + (hp.YOKE_ARM_PAD + hp.YOKE_SEAT_INTERF)   # -1.87
DISC_TO_DISC_SPAN = DISC_TOP_FACE_Z - DISC_BOT_FACE_Z                              # 38.04
AXIS_X = hp.SERVO_OUTPUT_X                       # output axis at (12.5, 0), along Z
CAP_SWEEP_R = 29.1        # clamp-cap flange corner sweep (closed-yoke diag)
CAP_SWEEP_Z = (0.0, hp.WELL_RIM_Z + 4.0 + 0.5)   # cap + cradle plate band, z 0..38.8
DECK_SWEEP_Z = (hp.WELL_RIM_Z + 4.0, 39.8)       # hip coxa deck sweeps z 39.3, r 14..30
DECK_SWEEP_R = (13.5, 30.5)                      # with 0.5 margin each side

# ---------------------------------------------------------------------------
# Stock C horn -- ASSUMED dims for a generic 2 mm aluminum 25T C bracket.
# MEASURE the purchased part and update these before printing adapters.
# ---------------------------------------------------------------------------
CHORN_PLATE_T   = 2.0    # mm plate thickness                     (ASSUMED)
CHORN_SPAN      = 44.0   # mm INNER span between the two plates   (ASSUMED)
CHORN_LEG_R0    = 12.0   # mm plate rounded end radius about axis (ASSUMED)
CHORN_WIDTH     = 24.0   # mm plate / web width (y extent)        (ASSUMED)
CHORN_WEB_X0    = 44.0   # mm web INNER face -- HARD MINIMUM 41.6 + margin
CHORN_WEB_T     = 2.0    # mm web thickness                       (ASSUMED)
CHORN_WEB_HOLE_DY = 7.0  # mm web hole half-spacing along y       (ASSUMED)
CHORN_WEB_HOLE_DZ = 13.0 # mm web hole half-spacing along z       (ASSUMED)
CHORN_PLATE_CENTER_HOLE_D = 10.0  # clears the Phi 9 spline collar (ASSUMED)

# Spacer stack (derived): the top spacer is set by clearance, the bottom
# spacer absorbs whatever span the purchased horn actually has.
SPACER_TOP_T = 4.0                                     # >= 3.6 required
SPACER_BOT_T = CHORN_SPAN - DISC_TO_DISC_SPAN - SPACER_TOP_T
SPACER_OD = 7.0

# Printed web flange (both adapters share it).
FLANGE_T = 6.0
FLANGE_X0 = CHORN_WEB_X0 + CHORN_WEB_T                 # 46
FLANGE_X1 = FLANGE_X0 + FLANGE_T                       # 52
NUT_POCKET_AF = 5.7      # M3 nyloc across-flats 5.5 + 0.2 fit
NUT_POCKET_DEPTH = 4.0
WEB_BOLT_OD = 3.4

Z_TOP_PLATE0 = DISC_TOP_FACE_Z + SPACER_TOP_T                  # 40.17
Z_TOP_PLATE1 = Z_TOP_PLATE0 + CHORN_PLATE_T                    # 42.17
Z_BOT_PLATE1 = DISC_BOT_FACE_Z - SPACER_BOT_T                  # -3.83
Z_BOT_PLATE0 = Z_BOT_PLATE1 - CHORN_PLATE_T                    # -5.83


def _web_hole_centres():
    zc = hp.JOINT_SOCKET_Z
    return [(sy * CHORN_WEB_HOLE_DY, zc + sz * CHORN_WEB_HOLE_DZ)
            for sy in (+1.0, -1.0) for sz in (+1.0, -1.0)]


def _rounded_plate(z0: float, z1: float) -> trimesh.Trimesh:
    """One C-horn plate: rounded end over the disc + straight run to the web."""
    t = z1 - z0
    zc = 0.5 * (z0 + z1)
    end = _cyl(CHORN_LEG_R0, t)
    end.apply_translation([AXIS_X, 0.0, zc])
    run = _box((CHORN_WEB_X0 + CHORN_WEB_T - AXIS_X, CHORN_WIDTH, t),
               center=(0.5 * (AXIS_X + CHORN_WEB_X0 + CHORN_WEB_T), 0.0, zc))
    plate = _union(end, run)
    cuts = []
    c = _cyl(CHORN_PLATE_CENTER_HOLE_D / 2.0, 4 * t)
    c.apply_translation([AXIS_X, 0.0, zc])
    cuts.append(c)
    for (hx, hy) in _disc_horn_bolt_centres():
        h = _cyl(hp.DISC_HORN_BOLT_OD / 2.0, 4 * t)
        h.apply_translation([hx, hy, zc])
        cuts.append(h)
    return _diff(plate, *cuts)


def make_chorn_reference() -> trimesh.Trimesh:
    """The stock aluminum C horn (REFERENCE mesh, not printed): two plates
    + outboard web, holes matching the 25T disc pattern and the web grid."""
    top = _rounded_plate(Z_TOP_PLATE0, Z_TOP_PLATE1)
    bot = _rounded_plate(Z_BOT_PLATE0, Z_BOT_PLATE1)
    web = _box((CHORN_WEB_T, CHORN_WIDTH, Z_TOP_PLATE1 - Z_BOT_PLATE0),
               center=(CHORN_WEB_X0 + CHORN_WEB_T / 2.0, 0.0,
                       0.5 * (Z_BOT_PLATE0 + Z_TOP_PLATE1)))
    cuts = []
    for (wy, wz) in _web_hole_centres():
        h = _cyl(WEB_BOLT_OD / 2.0, 4 * CHORN_WEB_T)
        h.apply_transform(rotation_matrix(np.pi / 2.0, [0, 1, 0]))
        h.apply_translation([CHORN_WEB_X0 + CHORN_WEB_T / 2.0, wy, wz])
        cuts.append(h)
    return _diff(_union(top, bot, web), *cuts)


def make_spacers() -> trimesh.Trimesh:
    """The 8 standoff spacers (4 per side) on the disc M3 bolts."""
    parts = []
    for (hx, hy) in _disc_horn_bolt_centres():
        for (z0, t) in ((DISC_TOP_FACE_Z, SPACER_TOP_T),
                        (DISC_BOT_FACE_Z - SPACER_BOT_T, SPACER_BOT_T)):
            s = _diff(_cyl(SPACER_OD / 2.0, t), _cyl(WEB_BOLT_OD / 2.0, 4 * t))
            s.apply_translation([hx, hy, z0 + t / 2.0])
            parts.append(s)
    return _union(*parts)


def _web_flange(*, keepout_cyls=()) -> trimesh.Trimesh:
    """Printed flange bolting to the C-horn web: 4x M3 clearance holes with
    hex nyloc pockets opening on the OUTBOARD face (screws enter countersunk
    from the web's inner face -- CSK is mandatory there, see the doc)."""
    body = _box((FLANGE_T, CHORN_WIDTH, Z_TOP_PLATE1 - Z_BOT_PLATE0),
                center=(0.5 * (FLANGE_X0 + FLANGE_X1), 0.0,
                        0.5 * (Z_BOT_PLATE0 + Z_TOP_PLATE1)))
    cuts = list(keepout_cyls)
    for (wy, wz) in _web_hole_centres():
        h = _cyl(WEB_BOLT_OD / 2.0, 4 * FLANGE_T)
        h.apply_transform(rotation_matrix(np.pi / 2.0, [0, 1, 0]))
        h.apply_translation([0.5 * (FLANGE_X0 + FLANGE_X1), wy, wz])
        cuts.append(h)
        hexp = trimesh.creation.cylinder(
            radius=NUT_POCKET_AF / np.sqrt(3.0), height=NUT_POCKET_DEPTH,
            sections=6)
        hexp.apply_transform(rotation_matrix(np.pi / 2.0, [0, 1, 0]))
        hexp.apply_translation([FLANGE_X1 - NUT_POCKET_DEPTH / 2.0, wy, wz])
        cuts.append(hexp)
    return _diff(body, *cuts)


def _spar_with_cones(x0: float, x1_wall: float) -> trimesh.Trimesh:
    """Phi 18 solid spar from the flange to the knee wall, cone flares at
    both ends (same recipe as the shipped femur's _femur_fused_spar)."""
    bite = hp._FEMUR_SPAR_WALL_BITE
    length = (x1_wall - x0) + bite
    Rx = rotation_matrix(np.pi / 2.0, [0, 1, 0])
    spar = _cyl(hp.FEMUR_SPAR_OD / 2.0, length)
    spar.apply_transform(Rx)
    spar.apply_translation([x0 + length / 2.0, 0.0, hp.JOINT_SOCKET_Z])
    cone_h = (x1_wall - x0) + bite
    gussets = []
    for base_x, sgn in ((x0 - bite, +1.0), (x1_wall + bite, -1.0)):
        g = trimesh.creation.cone(radius=hp.FEMUR_GUSSET_R_KNEE, height=cone_h,
                                  sections=hp.CYL_SECTIONS)
        g.apply_transform(rotation_matrix(sgn * np.pi / 2.0, [0, 1, 0]))
        g.apply_translation([base_x, 0.0, hp.JOINT_SOCKET_Z])
        gussets.append(g)
    return _union(spar, *gussets)


def make_femur_chorn_body() -> trimesh.Trimesh:
    """Printed femur for the C-horn variant: web flange + Phi 18 spar (cone
    flares both ends) + the UNCHANGED knee fixed side at FEMUR_LENGTH."""
    wall_face = hp._YOKE_SOCKET_X + hp.FEMUR_SPAR_LEN         # 58.3 (frozen)
    flange = _web_flange()
    spar = _spar_with_cones(FLANGE_X0, wall_face)
    kb = hp._femur_knee_fixed_solid()
    kb.apply_translation([hp.FEMUR_LENGTH, 0.0, 0.0])
    return _union(flange, spar, kb)


def make_tibia_chorn_socket() -> trimesh.Trimesh:
    """Printed tibia adapter for the C-horn variant: web flange + Phi 18
    CF-tube socket boss.  The socket MOUTH stays at joint-local x = 62 (same
    as the shipped tibia_knee_yoke), so the CF tube cut length is unchanged.
    Epoxy-only (no pin cross-hole), like the shipped yoke."""
    mouth_x = hp._YOKE_SOCKET_X + 20.0                        # 62 (frozen)
    Rx = rotation_matrix(np.pi / 2.0, [0, 1, 0])
    boss_len = mouth_x - FLANGE_X0                            # 16
    boss = _cyl(hp.LEG_TUBE_OD / 2.0 + hp.LEG_TUBE_SOCKET_WALL, boss_len)
    boss.apply_transform(Rx)
    boss.apply_translation([FLANGE_X0 + boss_len / 2.0, 0.0, hp.JOINT_SOCKET_Z])
    # small cone flare onto the flange face (same spirit as the femur spar)
    cone_h = boss_len + 1.0
    g = trimesh.creation.cone(radius=hp.FEMUR_GUSSET_R_KNEE, height=cone_h,
                              sections=hp.CYL_SECTIONS)
    g.apply_transform(rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    g.apply_translation([FLANGE_X0 - 1.0, 0.0, hp.JOINT_SOCKET_Z])
    bore_depth = hp.LEG_TUBE_SOCKET_DEPTH
    bore = _cyl(hp.LEG_TUBE_OD / 2.0 + hp.LEG_TUBE_SOCKET_CLEAR,
                bore_depth + 0.5)
    bore.apply_transform(Rx)
    bore.apply_translation([mouth_x - (bore_depth + 0.5) / 2.0 + 0.5, 0.0,
                            hp.JOINT_SOCKET_Z])
    flange = _web_flange()
    return _diff(_union(flange, boss, g), bore)


def make_disc_pair() -> trimesh.Trimesh:
    """The two 20 mm disc horns (reference, for the preview scene)."""
    top = _cyl(hp.DISC_HORN_OD / 2.0, hp.DISC_HORN_H)
    top.apply_translation([AXIS_X, 0.0, DISC_TOP_FACE_Z - hp.DISC_HORN_H / 2.0])
    bot = _cyl(hp.DISC_HORN_OD / 2.0, hp.DISC_HORN_H)
    bot.apply_translation([AXIS_X, 0.0, DISC_BOT_FACE_Z + hp.DISC_HORN_H / 2.0])
    return _union(top, bot)


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------
def _check_clearances(meshes: dict[str, trimesh.Trimesh]) -> list[str]:
    problems = []
    for name, m in meshes.items():
        if not m.is_watertight:
            problems.append(f"{name}: NOT watertight")
    # moving geometry vs the fixed clamp-cap / cradle sweep (yoke frame,
    # rotation about Z => planar radius from the output axis is invariant)
    moving = trimesh.util.concatenate([meshes["chorn_reference"],
                                       meshes["spacers"],
                                       meshes["femur_chorn_body"],
                                       meshes["tibia_chorn_socket"]])
    v = moving.vertices
    r = np.sqrt((v[:, 0] - AXIS_X) ** 2 + v[:, 1] ** 2)
    in_cap = (v[:, 2] > CAP_SWEEP_Z[0] - 0.25) & (v[:, 2] < CAP_SWEEP_Z[1])
    # exclude the spacers + plate ends that legitimately ride the disc
    # (they spin WITH the joint inside the Phi 24 horn opening, r <= 12)
    on_disc = r <= hp.DISC_HORN_OD / 2.0 + 2.1
    bad = in_cap & ~on_disc & (r < CAP_SWEEP_R + 0.4)
    if np.any(bad):
        i = int(np.argmin(np.where(bad, r, np.inf)))
        problems.append(f"cap-sweep clearance: {int(bad.sum())} verts inside "
                        f"r {CAP_SWEEP_R + 0.4:.1f} (worst r={r[i]:.1f} at "
                        f"z={v[i, 2]:.1f})")
    in_deck = (v[:, 2] > DECK_SWEEP_Z[0]) & (v[:, 2] < DECK_SWEEP_Z[1])
    bad2 = in_deck & (r > DECK_SWEEP_R[0]) & (r < DECK_SWEEP_R[1])
    if np.any(bad2):
        problems.append(f"coxa-deck band: {int(bad2.sum())} verts sweep the "
                        f"deck annulus r {DECK_SWEEP_R} at z {DECK_SWEEP_Z}")
    if SPACER_BOT_T < 0.0:
        problems.append(f"purchased span {CHORN_SPAN} too small: needs >= "
                        f"{DISC_TO_DISC_SPAN + SPACER_TOP_T:.1f}")
    if CHORN_WEB_X0 < AXIS_X + CAP_SWEEP_R + 0.4:
        problems.append(f"web inner face {CHORN_WEB_X0} inside the cap sweep "
                        f"(needs >= {AXIS_X + CAP_SWEEP_R + 0.4:.1f})")
    return problems


def main() -> int:
    os.makedirs(OUT_DIR, exist_ok=True)
    meshes = {
        "chorn_reference": make_chorn_reference(),
        "spacers": make_spacers(),
        "femur_chorn_body": make_femur_chorn_body(),
        "tibia_chorn_socket": make_tibia_chorn_socket(),
    }
    print("C-horn variant geometry (joint-local frame):")
    print(f"  disc faces z = {DISC_BOT_FACE_Z:.2f} / {DISC_TOP_FACE_Z:.2f} "
          f"(span {DISC_TO_DISC_SPAN:.2f})")
    print(f"  C-horn span {CHORN_SPAN:.1f} -> spacers top {SPACER_TOP_T:.2f} / "
          f"bottom {SPACER_BOT_T:.2f}")
    print(f"  plates z [{Z_BOT_PLATE0:.2f},{Z_BOT_PLATE1:.2f}] / "
          f"[{Z_TOP_PLATE0:.2f},{Z_TOP_PLATE1:.2f}]; web x "
          f"[{CHORN_WEB_X0:.1f},{CHORN_WEB_X0 + CHORN_WEB_T:.1f}]")
    for name, m in meshes.items():
        path = os.path.join(OUT_DIR, name + (
            "_DO_NOT_PRINT.stl" if name in ("chorn_reference",) else ".stl"))
        m.export(path)
        print(f"  wrote {os.path.relpath(path)}  "
              f"({len(m.faces)} tris, {m.volume / 1e3:.1f} cm^3)")

    problems = _check_clearances(meshes)
    if problems:
        print("CHECK FAILURES:")
        for p in problems:
            print("  -", p)
        return 1
    print("All variant checks passed (watertight, cap-sweep, deck band, span).")

    # preview scene (hip joint variant assembly + tibia adapter alongside)
    scene_parts = []
    colors = {"chorn_reference": [180, 185, 195, 255],
              "spacers": [120, 120, 130, 255],
              "femur_chorn_body": [235, 120, 60, 255],
              "tibia_chorn_socket": [80, 160, 235, 255],
              "discs": [90, 90, 95, 255]}
    meshes["discs"] = make_disc_pair()
    tib = meshes["tibia_chorn_socket"].copy()
    tib.apply_translation([0.0, 55.0, 0.0])       # beside, for visibility
    for name in ("chorn_reference", "spacers", "femur_chorn_body", "discs"):
        m = meshes[name].copy()
        m.visual.face_colors = colors[name]
        scene_parts.append(m)
    tib.visual.face_colors = colors["tibia_chorn_socket"]
    scene_parts.append(tib)
    combo = trimesh.util.concatenate(scene_parts)
    combo.export(os.path.join(OUT_DIR, "chorn_preview_DO_NOT_PRINT.stl"))
    try:
        scene = trimesh.Scene(scene_parts)
        png = scene.save_image(resolution=(1400, 900))
        with open(os.path.join(OUT_DIR, "_preview.png"), "wb") as f:
            f.write(png)
        print(f"  wrote {os.path.relpath(os.path.join(OUT_DIR, '_preview.png'))}")
    except BaseException as exc:  # headless GL may be unavailable
        print(f"  (preview render skipped: {exc})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
