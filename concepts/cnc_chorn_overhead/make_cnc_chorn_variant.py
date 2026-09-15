"""VARIANT: CNC aluminum C-clamps sized for LEGS-OVER-HEAD femur travel.

Concept (user, Aug 2026): "make a version of this [the rigid-hip
variant] with metal CNCed C clamps -- a little bigger so this hexapod
can put its legs over its head".

WHAT THE C-CLAMP IS (repo evidence: cad_step_test/build_chorn_step.py,
docs/CHORN_VARIANT.md, tools/make_chorn_variant.py): the C-shaped
bracket at each pitch joint (hip + knee) that clamps the servo's driven
20 mm disc horn on one side and the stock passive disc on the rear
idler boss on the other -- the part that replaces the PRINTED moving
clevis/yoke halves of the production femur and tibia.  The earlier
"C-horn variant" experiment assumed a BOUGHT stock aluminum C horn plus
printed adapters and loose spacers; THIS variant designs the part
proper, as a CNC-machined 6061-T6 bracket with integral standoff
bosses, sized so the femur can swing past vertical.

WHY OVERHEAD WORKS: THE OUTBOARD HIP PIVOT (user course-correction,
Aug 2026: "the coxa link is redesigned to position the hip servo
further out from the hexapod's body").  The production/rigid coxa put
the hip axis a mere 12.5 mm outboard of the yaw axis, i.e. right at
the top plate's rim: any femur-frame material with swing radius above
~40 mm crossed plate structure on the way up (the rigid-hip -47.5
limit, and the reason a previous rev of THIS concept grew a swan-neck
blade + notch + cap shave carved out of a contact map -- superseded,
overcomplicated).  This rev moves the hip pivot COXA_EXT = 42 mm
further outboard by extending the variant coxa's cradle arm, so the
up-swinging femur simply clears the rim because the pivot sits beyond
it, and the C-clamp reverts to a PLAIN C (rectangular blades, full
web, zero contact-map features).

Sweep-derived sizing (see the COXA_EXT block): the plain clamp needs
ext >= 42 for its first plate contact (low corners vs the yaw-tower
collar) to recede past -117.5; the KNEE-end parts saturate at first
contact -117.5 at ANY offset (the plate's Phi 44 dust-roof collar over
the yaw bearing always sits under the knee-end's inboard dip window --
the physics ceiling), which is why the single planar WEDGE chamfer on
the printed knee block + cap is the one overhead feature kept from the
previous rev.  Swept result: first contact -117.5 -> baked limit -110
deg (>= 5 deg true margin), vs -47.5 on rigid_hip and -80 in the
production workspace.

PARTS (this concept's step/ + stl/, BuildViz cnc_chorn_overhead):

  * ``chorn_clamp_cnc``  (12x, CNC 6061-T6, NOT printed) -- the PLAIN
    C-clamp: two 3 mm rectangular blades (y -20..+13) with integral
    Phi 19 ring bosses (4.5 mm below, 4.0 mm above -- replaces the
    chorn experiment's 8 loose spacers per joint), R13 rounded end
    over the disc, and a full-height 3.8 mm web whose outer face is
    the joint's mounting plane, with 4 TAPPED M3 holes (threads in
    metal -- no printed threads, no inserts in the load path).  R2.5
    internal corner gussets + R3 plan fillets are modeled (>= 2 mm
    tool radius everywhere).
  * ``coxa_link_ovh``    (6x, PETG) -- the rigid-hip coxa with its
    cradle arm (hip bracket + foot slab) moved COXA_EXT = 42 mm
    outboard; every yaw-axis interface (horn drive, hub boss, seat
    ring, dust brim) is untouched, byte-compared in check_coxa_ovh.
    The 38.2 mm rotation-envelope trim only applies inboard (the arm
    intentionally exceeds it; see check_yaw_envelope_ovh for the
    replacement guarantees).
  * ``femur_ovh_body``   (6x, PETG) -- the production knee block
    (cradle + 688 housing + far-wall pad, via
    step._femur_knee_fixed_solid) with: the overhead WEDGE chamfer on
    its inboard-top corner, 4 screw cbores for the web joint, the
    knee-cap up-screw bore, and the pocket for the variant cap's
    insert boss.  The yoke arms + spine of the production femur are
    GONE -- the metal clamp is the femur's hip half.
  * ``tibia_ovh_socket`` (6x, PETG) -- flange + CF-tube socket that
    bolts to the tibia-side clamp web the same way (8 mm flange, tube
    mouth pulled 10.6 mm outboard; the tube gets ~16.6 mm shorter and
    almost touches the metal web -- stiffer, lighter).
  * ``knee_clamp_cap_ovh`` (6x, PETG) -- production servo clamp cap
    minus the overhead wedge tail and minus its inboard side-screw
    boss (which died with the tail); retention is redesigned as an
    M3x25 UP-SCREW from the cradle underside into a Phi 4 x 6 heat-set
    insert in a hang-boss, bridged to the surviving tongue rib by a
    riser + a strap over the servo top (see the CAP_* constant block).
    Horn hooks, back hook, outboard screw and the tongue rib's
    outboard run are untouched.  (The previous rev also shaved the
    crown tail; the outboard pivot made that shave unnecessary.)

UNCHANGED: the rigid_hip chassis plates, hatch, hip cap and wago block,
every bearing/servo/COTS item, robot height and the whole vertical
stack.  MOVED: the hip axis (and with it the whole leg beyond it) sits
COXA_EXT further out radially -- stance footprint grows by 2 x 42 mm
across flats and the yaw joint carries more moment (quantified in
report_outboard_geometry + the README).  This driver re-runs
rigid_hip's ENTIRE check suite on the shared parts, then adds the
variant coxa / clamp / joint checks and the overhead sweep.

PRESS-FIT / TOLERANCE WARNING (PETG -> 6061): every bench-tuned
interference in this stack was tuned on printed PETG.  The clamp's
disc-pad span is therefore NOT carried over as an interference: the
production yoke squeezed the discs by 2 x YOKE_SEAT_INTERF = 0.26 mm
(plastic compliance); a rigid aluminum C at that squeeze would bend the
blades, not seat.  The CNC part is dimensioned to the NOMINAL disc span
38.04 mm at +0.00/+0.05, preload comes from the disc screws.  Flagged
as needs-bench-tuning in design_spec.yaml -- do not "fix" it back.

STEP-FIRST PIPELINE: geometry is authored as build123d/OpenCascade BREP
in build_cnc_chorn_step.py (STEP files are the CNC quoting deliverable);
this driver runs that exporter, loads the tessellations, assembles,
checks, sweeps and writes the BuildViz scene.

Run:  uv run python concepts/cnc_chorn_overhead/make_cnc_chorn_variant.py
      (--skip-sweep for fast geometry iterations;
       --skip-brep to reuse existing step/stl/ exports)
"""
from __future__ import annotations

import json
import os
import subprocess
import sys

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

HERE = os.path.abspath(os.path.dirname(__file__))
PROTO_DIR = os.path.abspath(os.path.join(HERE, "..", ".."))
RIGID_DIR = os.path.join(PROTO_DIR, "concepts", "rigid_hip")
STL_DIR = os.path.join(HERE, "stl")
sys.path.insert(0, RIGID_DIR)

import hexapod_prototype as hp  # noqa: E402  (read-only input)
import make_rigid_hip_variant as rv  # noqa: E402  (chassis/stack + checks)

# ---------------------------------------------------------------------------
# Joint-frame constants.  Frame = the femur/tibia LINK frame the production
# yoke parts are authored in (femur_link.stl's frame): +x along the link,
# joint axis along +z at x = AXIS_X, +y = the up-swing (cap) side.
# ---------------------------------------------------------------------------
AXIS_X = hp.SERVO_OUTPUT_X                     # 12.5 -- joint axis station
PAD_STACK = hp.YOKE_ARM_PAD + hp.YOKE_SEAT_INTERF          # 5.13
DISC_TOP_FACE_Z = hp.JOINT_HORN_TOP_Z - PAD_STACK          # 36.17 driven disc
DISC_BOT_FACE_Z = hp.JOINT_HORN_BOT_Z + PAD_STACK          # -1.87 passive disc
DISC_SPAN = DISC_TOP_FACE_Z - DISC_BOT_FACE_Z              # 38.04 NOMINAL --
#   the production yoke adds YOKE_SEAT_INTERF per side (PETG squeeze); the
#   rigid aluminum clamp is cut to the nominal span, see the module doc.

ARM_T = 3.0                    # blade thickness (was 2.0 plate + spacers in
#                                the chorn experiment; 3.0 in metal)
PAD_TOP_H = 4.0                # integral ring boss, driven-disc side
PAD_BOT_H = 4.5                # integral ring boss, passive side
PAD_OD = hp.DISC_HORN_OD - 1.0                 # 19 -- rides inside the same
#                                Phi 24 horn openings the production pads use
ARM_TOP_Z0 = DISC_TOP_FACE_Z + PAD_TOP_H       # 40.17
ARM_TOP_Z1 = ARM_TOP_Z0 + ARM_T                # 43.17 (production yoke: 45.3)
ARM_BOT_Z1 = DISC_BOT_FACE_Z - PAD_BOT_H       # -6.37
ARM_BOT_Z0 = ARM_BOT_Z1 - ARM_T                # -9.37 (production yoke: -11)

WALL_FACE_X = hp._YOKE_SOCKET_X + hp.FEMUR_SPAR_LEN        # 58.3 -- frozen
#   production femur wall plane; the web's OUTER face lands exactly here so
#   the printed knee block needs no adapter flange at the hip.
WEB_T = 3.8
WEB_X1 = WALL_FACE_X                           # 58.3 outer face = DATUM A
WEB_X0 = WEB_X1 - WEB_T                        # 54.5 inner face
CLAMP_R1 = WEB_X1 - AXIS_X                     # 45.8 -- "grown" C reach
CLAMP_R0 = WEB_X0 - AXIS_X                     # 42.0 (chorn experiment: 29.5)

# Blade plan profile (x_rel = x - AXIS_X, y): a PLAIN C.  The outboard
# hip pivot (COXA_EXT below) moved the whole swing beyond the plate rim,
# so the blade is just a full-width rectangle from the disc to the web
# -- no droop, no notch, no swan-neck corridor, no web bevel.  (The
# superseded swan-neck rev carved all of those out of a contact map to
# survive at the production hip station; see README "superseded".)
BLADE_END_R = 13.0             # rounded end over the Phi 20 disc
BLADE_W_UP = 13.0              # leading half-width, full length
BLADE_Y_BOT = -20.0            # trailing/bottom edge, full length
PLAN_FILLET_R = 3.0            # plan-profile fillets (>= 2 mm tool radius)
GUSSET_R = 2.5                 # internal web/blade corner radius (tool R)
GUSSET_Y = (-19.5, 12.5)       # web/blade gusset span (plain: nearly the
#                                whole junction; 0.5 shy of the edges)
WEB_Y_BOT = BLADE_Y_BOT        # plain web: same rectangle as the blades

# --- OUTBOARD HIP PIVOT (the legs-over-head architecture) -----------------
# The variant coxa extends its cradle arm so the hip servo sits COXA_EXT
# further out along the leg (+x in the coxa frame).  Sweep-derived, in
# two passes.  Plan-raster probe (_probe_outboard.py, blade footprint +
# sampled knee-side parts): ext 28 -> blade first plate contact -107.5,
# ext 32 -> -112.5, ext 36 -> blade clear thru -120.  The REAL clamp
# solid (pad bosses + full-height web, swept as meshes on the built
# geometry) needs more: its low corners meet the yaw-tower collar
# (r ~21 from the yaw axis, i.e. a yaw-INVARIANT contact) at -110 for
# ext 36, -117.5 for ext 41, and clear past -117.5 from ext 42 up.
# The KNEE-end parts saturate at first contact -117.5 no matter the
# offset: past ~-102 the knee end dips below the plate-top plane on the
# INBOARD side 31..51 mm from the hip axis, and the plate's Phi 44
# dust-roof collar over the yaw bearing sits in that window for ANY
# offset up to ~+60 (it recedes at the same rate the dip window grows).
# That is the physics ceiling: the single WEDGE chamfer on the printed
# knee block + cap (kept from the previous rev, one plane) holds the
# knee-end swing radius under the collar, and no offset removes the
# need for it.  COXA_EXT = 42 is the minimum at which the saturating
# knee end -- not the clamp -- is the limiter; more offset buys
# nothing but yaw-joint moment.  Expected sweep: first contact -117.5
# (knee block/cap vs collar) => safe up-limit -110.
COXA_EXT = 42.0                # hip pivot outboard extension [mm]
HIP_ANCHOR_OVH = (rv.COXA_HIP_ANCHOR_V[0] + COXA_EXT,
                  rv.COXA_HIP_ANCHOR_V[1], rv.COXA_HIP_ANCHOR_V[2])

# Variant HIP CAP: the rigid-hip cap's yaw-axis furniture (pedestal +
# 6805 press boss + puller notches) must STAY on the yaw axis -- the top
# plate seats on that bearing -- while the servo-clamp jaw moves out
# with the hip.  So hip_clamp_cap_ovh keeps the production jaw at the
# (moved) hip station and reaches COXA_EXT back INBOARD (cap-local
# x = -COXA_EXT) to the pedestal, over a two-step arm.  The arm's plan
# corners are kept outside the clamp's swept band: material at radius r
# from the hip axis is unreachable when its azimuth exceeds
# asin(13/r) + 112.5 deg (leading blade half-width 13, deepest spin
# check -112.5); both boxes below clear that line by > 30 deg.  Load
# path:
# plate -> 6805 -> boss -> arm -> cap -> hip cradle (flagged in the
# README: the arm is a PETG cantilever, sized full pedestal chord).
HIPCAP_ARM_Z = (1.2, 30.1)     # z band = the pedestal chord
_ARM_X0 = -(COXA_EXT + 10.0)   # past the pedestal centre by ~PED_OD/3
HIPCAP_ARM_LO = (_ARM_X0, -24.0, 14.5, 18.5)   # x0, x1, y0, y1
HIPCAP_ARM_HI = (_ARM_X0, -29.0, 14.5, 22.0)   # reaches the ped bottom

# Web joint, 4x M3 in two styles (geometry forced the split: SHCS heads
# on the printed side are only reachable through the open servo well,
# whose floor limits them to y >= ~-10; the web's bevel limits tapped
# metal cover to y <= ~-6.9 -- so the UPPER pair is tapped-metal, and
# the LOWER pair, which lives under the cradle floor, becomes CSK
# through-bolts into T-slot nylocs opened from the printed underside,
# exactly the chorn experiment's nut-pocket scheme):
#   A (2x): M3x10 SHCS from the printed side, head cbore opening into
#      the well cavity (driven before the servo drops in), threads in
#      the TAPPED aluminum web (M3-6H x 3.8 -- threads in metal, the
#      DFM ask).  Tip lands ~1.4 mm below the web on the trailing side
#      (swing-safe, asserted).
#   B (2x): M3x12 CSK from the METAL side (90-deg csk sunk in the web's
#      inner face), through printed clearance into an M3 NYLOC in a
#      T-slot pocket cut from the cradle/flange underside.
# z staggered about JOINT_SOCKET_Z (17.15) so no two head recesses or
# metal cuts ever merge (the B stagger is 7.5, not less: the nyloc
# T-slots must clear the up-screw's underside head cbore at z 13.9..20.4
# by >= 1.4 of printed wall).
WEB_A_YZ = [(-9.4, hp.JOINT_SOCKET_Z - 13.0),
            (-9.4, hp.JOINT_SOCKET_Z + 13.0)]
WEB_B_YZ = [(-15.5, hp.JOINT_SOCKET_Z - 7.5),
            (-15.5, hp.JOINT_SOCKET_Z + 7.5)]
WEB_TAP_D = 3.0                # A: M3-6H modeled at Phi 3.0 (2.5 pilot +
#                                tap is the shop's business)
WEB_A_LEN = 10.0               # M3x10 SHCS
WEB_CBORE_D = 6.0              # A: SHCS head pocket in the printed parts
WEB_CBORE_DEPTH = 3.2
WEB_B_D = 3.4                  # B: clearance through metal AND print
WEB_B_LEN = 12.0               # M3x12 CSK flat-head
WEB_CSK_D = 6.72               # B: 90-deg csk major in the web inner face
NYLOC_AF = 5.5                 # M3 nyloc across flats
NYLOC_H = 4.0                  # M3 nyloc height
NUT_SLOT_X0 = 60.8             # nut pocket: 2.5 printed wall past DATUM A
NUT_SLOT_X1 = NUT_SLOT_X0 + NYLOC_H + 0.3      # 65.1
WEB_SHANK_D = 3.4              # printed-side clearance bores

# Overhead WEDGE chamfer (link coords, z-uniform): cuts the printed knee
# block's inboard-top corner and the knee cap's tail along the SAME plane,
# so the parts stay complementary.  KEPT under the outboard pivot: this
# is the one relief no offset can replace (see the COXA_EXT note -- the
# yaw-tower dust-roof collar always sits under the knee-end's inboard
# dip window), and it is a single planar cut on two printed parts.
WEDGE_XY0 = (WALL_FACE_X, 2.0)
WEDGE_XY1 = (68.0, 22.0)
WEDGE_SLOPE = (WEDGE_XY1[1] - WEDGE_XY0[1]) / (WEDGE_XY1[0] - WEDGE_XY0[0])


def wedge_y(x: float) -> float:
    """Material ceiling of the overhead wedge at link station x."""
    return WEDGE_XY0[1] + WEDGE_SLOPE * (x - WEDGE_XY0[0])


# Knee cap redesign around the wedge: the production INBOARD side screw
# (well x -27.2 -> link 62.8, head on the flange top) dies with the tail.
# Replacement: M3x25 SHCS UP-SCREW from the cradle underside into a
# Phi 4 x 6 heat-set insert in a hang-BOSS that lives in a pocket cut
# into the block's near wall.  The boss cannot weld straight to the
# tongue (the tongue only exists outboard of the servo end face at link
# 67.3, and everything above the wall is wedge-cut inboard of ~66), so
# it bridges over: boss -> RISER (up past the servo top, staying inboard
# of the servo end face) -> STRAP (rides 0.3 over the servo top face,
# welds into the tongue's underside band).  All members stay below the
# wedge plane.  Link coords (cap-local x = link x - FEMUR_LENGTH):
CAP_BOSS_X0, CAP_BOSS_X1 = 60.0, 67.0          # boss: inside the wall span
CAP_BOSS_Y0 = 4.5                              # boss bottom = pocket floor
CAP_BOSS_Y1 = hp.SERVO_BODY_D / 2.0 - hp.CLAMP_TONGUE_INTERF + 0.1  # 11.5
CAP_BOSS_Z0, CAP_BOSS_Z1 = 13.0, 21.3          # >= 2 walls for the insert
CAP_RISER_X0 = 65.0                            # riser: x 65..67 (servo end
#                                                face is 67.3 -- 0.3 clear)
CAP_STRAP_X1 = 68.0                            # strap: x 65..68, welds into
CAP_STRAP_Y0 = hp.SERVO_BODY_D / 2.0 + 0.3     # 12.7 -- 0.3 over the servo
CAP_STRAP_Y1 = 16.5                            # the tongue band (y 11.4-16.9
#                                                at link x >= 67.1)
UPSCREW_X = 63.5                               # link x of the up-screw axis

# (The swan-neck rev also shaved the cap's crown tail with three boxes;
# the outboard pivot made that unnecessary -- the wedge-only cap's crown
# first contacts at -117.5, same as the block, verified by the probe.)
UPSCREW_Z = hp.JOINT_SOCKET_Z                  # 17.15
UPSCREW_D = 3.4                                # clearance bore in the block
UPSCREW_CB_D = 6.5                             # SHCS head cbore, underside
UPSCREW_CB_DEPTH = 4.0
CAP_INSERT_D = 4.0                             # Phi 4 x 6 brass insert
CAP_INSERT_LEN = 6.0
CAP_INSERT_RELIEF_D = 5.5                      # melt relief at entry
CAP_INSERT_RELIEF_DEPTH = 0.4
POCKET_CL = 0.25                               # boss-to-pocket clearance
BLOCK_Y_BOT = -(hp.WELL_D / 2.0)               # -16.9 cradle underside

# Tibia adapter: flange thick enough for the same M3x10 stack, tube mouth
# pulled outboard past the screw hardware.  Socket constants are the
# bench-tuned production tube fit.
TIB_FLANGE_T = 8.0
TIB_FLANGE_X0 = WALL_FACE_X                    # 58.3 (mates DATUM A)
TIB_FLANGE_X1 = TIB_FLANGE_X0 + TIB_FLANGE_T   # 66.3
TIB_FLANGE_Y0, TIB_FLANGE_Y1 = -20.0, 13.0
TIB_FLANGE_Z0, TIB_FLANGE_Z1 = -1.0, 35.3      # covers screws + tube boss
TIB_BORE_X0 = WALL_FACE_X + 0.3                # tube tip 0.3 off the metal
TIB_MOUTH_X = TIB_BORE_X0 + hp.LEG_TUBE_SOCKET_DEPTH       # 72.6 (prod: 62)
TIB_BOSS_R = hp.LEG_TUBE_OD / 2.0 + hp.LEG_TUBE_SOCKET_WALL  # 9.0

# Sweep target + materials
SWEEP_STEP = 2.5
SWEEP_MARGIN = SWEEP_STEP + 5.0                # 1 grid cell + 5 deg true
SWEEP_TARGET = -105.0                          # baked limit must beat this
RHO_ALU = 2.70e-3                              # g/mm3, 6061-T6
RHO_PETG = 1.27e-3                             # g/mm3

# Shared frames from the rigid-hip driver (identical chassis/stack)...
MH = rv.MH
M_KNEE_JP = rv.M_KNEE_JP
_trans = rv._trans
_inter_vol = rv._inter_vol
_bounds_touch = rv._bounds_touch
# ... except the hip anchor, which this variant moves COXA_EXT outboard.
M_HIP_JP_OVH = hp._joint_place(HIP_ANCHOR_OVH, *rv.XZ)


def leg_transforms(i: int, yaw_deg: float = 0.0,
                   femur_deg: float = None,
                   tibia_deg: float = None) -> dict[str, np.ndarray]:
    """rv.leg_transforms with the hip axis at HIP_ANCHOR_OVH (COXA_EXT
    further out along the coxa +x).  Same keys, same conventions."""
    femur_deg = hp.STANCE_FEMUR_DEG if femur_deg is None else femur_deg
    tibia_deg = hp.STANCE_TIBIA_DEG if tibia_deg is None else tibia_deg
    a = (i + 0.5) * np.pi / 3.0
    edge = np.array([rv.APOTHEM * np.cos(a), rv.APOTHEM * np.sin(a),
                     hp.CHASSIS_YAW_OUTPUT_Z])
    p = np.deg2rad(femur_deg)
    pt = np.deg2rad(femur_deg + tibia_deg)
    hip_local = np.array(HIP_ANCHOR_OVH)
    knee_local = hip_local + rotation_matrix(p, [0, 1, 0])[:3, :3] \
        @ np.array([hp.FEMUR_LENGTH, 0.0, 0.0])
    T_coxa = _trans(edge) @ rv._rotz(a) @ rv._rotz(np.deg2rad(yaw_deg))
    T_femur = T_coxa @ _trans(hip_local) @ rotation_matrix(p, [0, 1, 0])
    T_tibia = T_coxa @ _trans(knee_local) @ rotation_matrix(pt, [0, 1, 0])
    T_cradle = _trans([edge[0], edge[1], 0.0]) @ rv._rotz(a)
    return {"coxa": T_coxa, "femur": T_femur, "tibia": T_tibia,
            "cradle": T_cradle,
            "hip_cap": T_coxa @ M_HIP_JP_OVH,
            # the top-plate 6805 stays ON the yaw axis: same frame the
            # rigid concept used (the variant cap's pedestal reaches
            # back to it)
            "yaw_top": T_coxa @ rv.M_HIP_JP,
            "knee_cap": T_femur @ M_KNEE_JP}


# ---------------------------------------------------------------------------
# Mesh registry: rigid_hip's full set + this concept's four BREP parts.
# ---------------------------------------------------------------------------
BREP_STL_DIR = os.path.join(HERE, "step", "stl")
BREP_BUILDER = os.path.join(HERE, "build_cnc_chorn_step.py")
BREP_EXPORT_CMD = ["uv", "run", "--no-project", "--python", "3.12",
                   "--with", "build123d", "--with", "trimesh",
                   "--with", "numpy", "--with", "manifold3d",
                   "python", BREP_BUILDER]
RIGID_BREP_CMD = ["uv", "run", "--no-project", "--python", "3.12",
                  "--with", "build123d", "--with", "trimesh",
                  "--with", "numpy", "--with", "manifold3d",
                  "python", os.path.join(RIGID_DIR, "build_rigid_hip_step.py")]

OVH_PARTS = ("chorn_clamp_cnc", "femur_ovh_body", "tibia_ovh_socket",
             "knee_clamp_cap_ovh", "coxa_link_ovh", "hip_clamp_cap_ovh")


def _load_brep(fname: str) -> trimesh.Trimesh:
    path = os.path.join(BREP_STL_DIR, fname)
    if not os.path.exists(path):
        raise SystemExit(
            f"{fname}: no BREP tessellation at {path} -- run "
            "build_cnc_chorn_step.py first (main() does that for you "
            "unless --skip-brep)")
    m = trimesh.load(path, process=True)
    if isinstance(m, trimesh.Scene):
        m = trimesh.util.concatenate(
            [g for g in m.geometry.values() if len(g.faces) > 0])
    return m


def _tibia_tube() -> tuple[trimesh.Trimesh, np.ndarray]:
    """Shortened tube visual + the UNCHANGED foot frame (kinematics are
    production; only the buried end moves outboard to the new mouth)."""
    ta_prod = (MH @ np.array([hp._YOKE_SOCKET_X, 0.0,
                              hp.JOINT_SOCKET_Z, 1.0]))[:3]
    tube_end = ta_prod + np.array([hp.TIBIA_LENGTH - 8.0, 0.0, 0.0])
    ta = (MH @ np.array([TIB_BORE_X0, 0.0, hp.JOINT_SOCKET_Z, 1.0]))[:3]
    tube = hp._tube_between(ta, tube_end, hp.LEG_TUBE_OD / 2.0)
    foot_frame = hp._frame(tube_end, (1, 0, 0), (0, 0, 1))
    return tube, foot_frame


def build_meshes() -> dict[str, trimesh.Trimesh]:
    print("building rigid_hip mesh set (shared chassis/stack) ...")
    meshes = rv.build_meshes()
    for name in OVH_PARTS:
        print(f"  {name:22s} loading BREP tessellation ...", flush=True)
        m = hp._heal_for_export(_load_brep(f"{name}.stl"))
        assert m.is_volume, f"{name}: not a volume even after heal"
        meshes[name] = m
    tube, _ = _tibia_tube()
    meshes["tibia_tube_ovh"] = tube

    # Mirror everything this concept's scene references into OUR stl/
    # (print set for the new parts, visuals for the rest).
    os.makedirs(STL_DIR, exist_ok=True)
    for key in SCENE_MESH_KEYS:
        fname = SCENE_MESH_FILES[key]
        meshes[key].export(os.path.join(STL_DIR, fname))
    return meshes


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

def _placed(meshes, key, M) -> trimesh.Trimesh:
    m = meshes[key].copy()
    m.apply_transform(M)
    return m


def check_parts(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Part-level gates: watertight, mass budget, DFM-critical dims that
    the BREP builder must have honored (guards future constant edits)."""
    for key, rho, what in (
            ("chorn_clamp_cnc", RHO_ALU, "CNC 6061-T6"),
            ("femur_ovh_body", RHO_PETG, "PETG"),
            ("tibia_ovh_socket", RHO_PETG, "PETG"),
            ("knee_clamp_cap_ovh", RHO_PETG, "PETG"),
            ("coxa_link_ovh", RHO_PETG, "PETG"),
            ("hip_clamp_cap_ovh", RHO_PETG, "PETG")):
        m = meshes[key]
        assert m.is_watertight, f"{key} not watertight"
        print(f"  {key:22s} watertight, vol {m.volume / 1000.0:6.1f} cm3, "
              f"{m.volume * rho:6.1f} g ({what})")

    clamp = meshes["chorn_clamp_cnc"]
    lo, hi = clamp.bounds
    # pads span exactly the nominal disc faces; arms outside them
    assert abs(lo[2] - ARM_BOT_Z0) < 0.02 and abs(hi[2] - ARM_TOP_Z1) < 0.02, \
        f"clamp z envelope {lo[2]:.2f}..{hi[2]:.2f}"
    v = clamp.vertices
    pad_band = v[(v[:, 2] > DISC_BOT_FACE_Z - 0.01)
                 & (v[:, 2] < DISC_TOP_FACE_Z + 0.01)]
    r_pad = np.hypot(pad_band[:, 0] - AXIS_X, pad_band[:, 1])
    # between the disc faces only the Phi 19 bosses (r <= 9.5) and the
    # WEB at the far end (r >= CLAMP_R0) may exist -- the C opening
    inner = r_pad[r_pad < CLAMP_R0 - 0.02]
    assert float(inner.max()) <= PAD_OD / 2.0 + 0.02, \
        f"material at r {inner.max():.2f} inside the disc span (only the " \
        f"Phi {PAD_OD:g} bosses may live there)"
    # pad faces land ON the disc faces (kiss, no PETG-era interference).
    # Probe at azimuth 45 deg -- midway between the 0/90/180/270 deg disc
    # bolt holes, whose Phi 3.4 voids would swallow an on-axis probe.
    px = AXIS_X + (PAD_OD / 2.0 - 1.0) * np.cos(np.pi / 4.0)
    py = (PAD_OD / 2.0 - 1.0) * np.sin(np.pi / 4.0)
    for z, side in ((DISC_BOT_FACE_Z, -1), (DISC_TOP_FACE_Z, +1)):
        probe = np.array([[px, py, z - 0.2 * side],
                          [px, py, z + 0.2 * side]])
        inside = clamp.contains(probe)
        assert not inside[0] and inside[1], \
            f"pad face not exactly on the disc plane z={z:.2f}"
    # web outer face is the wall plane (DATUM A)
    assert abs(hi[0] - WEB_X1) < 0.02, f"web outer face at {hi[0]:.2f}"
    # disc-screw + centre + tap paths open
    pcd_r = hp.DISC_HORN_BOLT_PCD / 2.0
    probes = [[AXIS_X + pcd_r * np.cos(t), pcd_r * np.sin(t), zc]
              for t in hp.DISC_HORN_BOLT_ANGLES_RAD
              for zc in ((ARM_BOT_Z0 + ARM_BOT_Z1) / 2.0,
                         (ARM_TOP_Z0 + ARM_TOP_Z1) / 2.0)]
    probes += [[AXIS_X, 0.0, (ARM_BOT_Z0 + ARM_BOT_Z1) / 2.0],
               [AXIS_X, 0.0, (ARM_TOP_Z0 + ARM_TOP_Z1) / 2.0]]
    assert not clamp.contains(np.asarray(probes)).any(), \
        "a disc-screw / centre bore is blocked in the clamp"
    holes = [[(WEB_X0 + WEB_X1) / 2.0, y, z]
             for (y, z) in WEB_A_YZ + WEB_B_YZ]
    assert not clamp.contains(np.asarray(holes)).any(), \
        "a web screw bore is blocked in the metal"
    # metal cover: full ring around every web hole at the inner face
    # (guards the bevel from biting into a thread/csk boss)
    ring = []
    for (y, z) in WEB_A_YZ + WEB_B_YZ:
        for t in np.linspace(0.0, 2 * np.pi, 8, endpoint=False):
            ring.append([WEB_X0 + 0.4, y + 2.3 * np.cos(t),
                         z + 2.3 * np.sin(t)])
    ring = np.asarray(ring)
    # B holes (the last 2 x 8 points) carry a csk at the inner face:
    # probe their rings at the OUTER face instead, past the csk cone
    ring[16:, 0] = WEB_X1 - 0.4
    assert clamp.contains(ring).all(), \
        "a web screw hole lacks full metal cover (bevel bit the boss ring)"
    print(f"  clamp: pads kiss the discs at span {DISC_SPAN:.2f} "
          f"(nominal, needs-bench-tuning note in the spec), web face on "
          f"DATUM A x={WEB_X1:g}, 2 taps + 2 csk bolt paths open")


def check_clamp_joint(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Seated + spinning clamp vs each joint's fixed side.

    Hip: spin across -112.5..+35 -- one sweep grid cell past the -110
    baked limit.  Deeper is unreachable by design: the sweep's part-A
    pass measures the clamp's first fixed-side contact (the yaw-axis
    6805/pedestal stack, 54.5 mm inboard of the hip axis) at -125,
    past the knee-end-vs-collar -117.5 that actually sets the limit.
    A full 360 was already impossible in production (the yoke crossed
    the fixed cap outside -80..+30).
    Knee: spin across the tibia ROM -50..+40 (beyond the -30..+20 baked
    limits) against femur body + knee servo + variant cap."""
    T = leg_transforms(0)

    fixed_hip = [_placed(meshes, "servo_body", T["hip_cap"]),
                 _placed(meshes, "hip_clamp_cap_ovh", T["hip_cap"]),
                 _placed(meshes, "bearing_6805", T["yaw_top"]),
                 _placed(meshes, "coxa_link_ovh", T["coxa"])]
    for ang in np.arange(-112.5, 35.0 + 1e-9, 2.5):
        Tf = leg_transforms(0, femur_deg=float(ang))
        c = _placed(meshes, "chorn_clamp_cnc", Tf["femur"] @ MH)
        for name, f in zip(("hip servo", "hip cap", "bearing", "coxa"),
                           fixed_hip):
            v = _inter_vol(c, f)
            assert v < 1e-6, \
                f"hip clamp @ {ang:+.0f} deg intersects {name} ({v:.2f} mm3)"
    print("  hip clamp: -112.5..+35 spin clear of servo/cap/bearing/coxa "
          "(first fixed-side contact -125, past the -117.5 ceiling)")

    # knee: the same clamp on the tibia side vs the femur-fixed side
    knee_fixed = [_placed(meshes, "femur_ovh_body", T["femur"] @ MH),
                  _placed(meshes, "servo_body", T["knee_cap"]),
                  _placed(meshes, "knee_clamp_cap_ovh", T["knee_cap"])]
    for td in np.arange(-50.0, 40.0 + 1e-9, 10.0):
        Tt = leg_transforms(0, tibia_deg=float(td))
        c = _placed(meshes, "chorn_clamp_cnc", Tt["tibia"] @ MH)
        sock = _placed(meshes, "tibia_ovh_socket", Tt["tibia"] @ MH)
        for name, f in zip(("femur body", "knee servo", "knee cap"),
                           knee_fixed):
            for part, pm in (("clamp", c), ("socket", sock)):
                v = _inter_vol(pm, f)
                assert v < 1e-6, (f"knee {part} @ tibia {td:+.0f} deg "
                                  f"intersects {name} ({v:.2f} mm3)")
    print("  knee clamp + socket: tibia spin -50..+40 clear of the "
          "femur-fixed side")


def check_web_joint(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Clamp web <-> printed part mating at both joints: faces coplanar,
    zero overlap, both screw styles' paths open end to end, A-screw
    tips swing-safe, B nylocs seated in real pockets with solid bearing
    material above them."""
    clamp = meshes["chorn_clamp_cnc"]
    for key, cav_face in (("femur_ovh_body", hp.FEMUR_LENGTH
                           - hp.SERVO_BODY_W / 2.0),   # 67.3 well face
                          ("tibia_ovh_socket", TIB_FLANGE_X1)):
        part = meshes[key]
        v = _inter_vol(clamp, part)
        assert v < 1e-6, f"clamp overlaps {key} ({v:.2f} mm3)"
        # printed material actually present on the mating plane -- probed
        # as an annulus AROUND each screw (the axis itself is drilled);
        # y-2.6 is skipped because the B row sits 1.4 from the underside
        face_probe = [[WEB_X1 + 0.3, yy, zz]
                      for (y, z) in WEB_A_YZ + WEB_B_YZ
                      for (yy, zz) in ((y + 2.6, z), (y, z + 2.6),
                                       (y, z - 2.6))]
        assert part.contains(np.asarray(face_probe)).all(), \
            f"{key}: no printed material behind the web face at a screw"

        # A screws: axis open from the web's inner face through the tap,
        # printed shank and head cbore out to the cavity/flange face
        for (y, z) in WEB_A_YZ:
            xs = np.arange(WEB_X0 - 0.6, cav_face + 0.5, 0.35)
            line = np.array([[x, y, z] for x in xs])
            blocked = part.contains(line) | clamp.contains(line)
            assert not blocked.any(), \
                f"{key}: A-screw path blocked at (y={y}, z={z})"
            # head seat annulus is solid at the cbore bottom plane
            seat_x = cav_face - WEB_CBORE_DEPTH - 0.4
            seat = [[seat_x, y + 3.6, z], [seat_x, y - 3.6, z],
                    [seat_x, y, z + 3.6], [seat_x, y, z - 3.6]]
            assert part.contains(np.asarray(seat)).all(), \
                f"{key}: A-screw head has no seat at (y={y}, z={z})"
            # M3x10 tip below the web: trailing side, swing-safe
            tip_x = cav_face - WEB_CBORE_DEPTH - WEB_A_LEN
            r = np.hypot(tip_x - AXIS_X, y)
            phi = np.degrees(np.arctan2(y, tip_x - AXIS_X))
            assert r < 44.0 and phi < -12.0, \
                f"A tip at (r {r:.1f}, phi {phi:.1f}) not swing-safe"

        # B bolts: axis open from just outside the web's csk face into
        # the nut pocket; nyloc pocket void present; solid material
        # between pocket top and the mating face for the nut to bear on
        for (y, z) in WEB_B_YZ:
            xs = np.arange(WEB_X0 - 0.6, NUT_SLOT_X1 + 1.2, 0.35)
            line = np.array([[x, y, z] for x in xs])
            blocked = part.contains(line) | clamp.contains(line)
            assert not blocked.any(), \
                f"{key}: B-bolt path blocked at (y={y}, z={z})"
            mid_slot = (NUT_SLOT_X0 + NUT_SLOT_X1) / 2.0
            void = [[mid_slot, y + 2.2, z], [mid_slot, y - 2.2, z]]
            assert not part.contains(np.asarray(void)).any(), \
                f"{key}: nyloc pocket missing at (y={y}, z={z})"
            # probed at z +/- 2.5: outside the Phi 3.4 bore, still under
            # the nyloc face (AF 5.5), inside the wall band either side
            bearing = [[NUT_SLOT_X0 - 1.2, y, z + 2.5],
                       [NUT_SLOT_X0 - 1.2, y, z - 2.5]]
            assert part.contains(np.asarray(bearing)).all(), \
                f"{key}: no bearing wall ahead of the nyloc at (y={y})"
    print(f"  web joint: coplanar mate on DATUM A; 2x M3x{WEB_A_LEN:g} SHCS "
          f"into tapped metal + 2x M3x{WEB_B_LEN:g} CSK into T-slot "
          f"nylocs, per joint; all paths open, tips trailing")


def check_knee_cap(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Variant knee cap: still seats like production (servo squeeze is the
    only contact), boss lands in the block pocket with clearance, the
    up-screw path is open from the cradle underside into the insert,
    drop-in path clear, and the wedge really removed the old blockers."""
    T = leg_transforms(0)
    cap = _placed(meshes, "knee_clamp_cap_ovh", T["knee_cap"])
    block = _placed(meshes, "femur_ovh_body", T["femur"] @ MH)
    servo = _placed(meshes, "servo_body", T["knee_cap"])

    # cap seats FLUSH on the block by design (tongue face on the cavity
    # face, flange on the wall top): coincident faces leave sub-1e-3
    # boolean slivers -- anything real would be orders bigger
    v = _inter_vol(cap, block)
    assert v < 0.01, f"variant cap vs femur body: {v:.3f} mm3"
    # the cap-vs-servo overlap IS the design press: the tongue reaches
    # CLAMP_TONGUE_INTERF = 1.0 past the servo face over the full cavity
    # (~1.6 cm3, production behavior).  Assert it stays in that band --
    # anything above means the new boss/riser/strap bit into the case.
    v = _inter_vol(cap, servo)
    assert 800.0 < v < 1900.0, \
        f"variant cap vs servo: {v:.1f} mm3 (want tongue squeeze only)"

    # production comparison: volume delta = the tail the wedge removed
    dv = (meshes["knee_clamp_cap"].volume
          - meshes["knee_clamp_cap_ovh"].volume) / 1000.0
    assert dv > 0.5, "wedge cut removed almost nothing from the cap"

    # up-screw path (leg frame, along the cap-local -Y... probe in link
    # coords through the block bore into the cap insert bore)
    Mf = T["femur"] @ MH
    ys = np.arange(BLOCK_Y_BOT - 0.5, CAP_BOSS_Y0 + CAP_INSERT_LEN - 0.6, 0.4)
    line = trimesh.transform_points(
        np.array([[UPSCREW_X, y, UPSCREW_Z] for y in ys]), Mf)
    assert not block.contains(line).any(), "up-screw bore blocked in block"
    assert not cap.contains(line).any(), "up-screw bore blocked in cap boss"
    # insert boss really present around the bore
    ringpts = []
    for t in np.linspace(0.0, 2 * np.pi, 6, endpoint=False):
        ringpts.append([UPSCREW_X + 2.6 * np.cos(t),
                        CAP_BOSS_Y0 + 2.0,
                        UPSCREW_Z + 2.6 * np.sin(t)])
    ring = trimesh.transform_points(np.asarray(ringpts), Mf)
    assert cap.contains(ring).all(), "cap insert boss missing around the bore"

    # drop-in: cap lifts straight off (+y in link frame).  The servo is
    # only probed from +2 (the 1 mm tongue press is still engaged below
    # that -- by design); the block must be clear at every height.
    up = Mf[:3, :3] @ np.array([0.0, 1.0, 0.0])
    for d in (0.5, 2.0, 5.0, 15.0):
        c = cap.copy()
        c.apply_translation(up * d)
        fixed = [("block", block, 0.01)] \
            + ([("servo", servo, 25.0)] if d >= 2.0 else [])
        for name, f, lim in fixed:
            # block 0.01: tongue/cavity faces stay COINCIDENT while the
            # cap slides out -- same sub-1e-3 boolean slivers as seated.
            # servo 25.0: the production-style snap hooks sweep a ~1 mm
            # band across the case lip on a STRAIGHT lift (measured
            # 4.2 mm3 at +2); the real PETG cap tilts/flexes over the
            # hooks exactly as in production, so only a small transient
            # is tolerated and +15 must be fully clear
            lim_eff = lim if d < 15.0 else 0.01
            v = _inter_vol(c, f)
            assert v < lim_eff, f"cap lift +{d}: fouls {name} ({v:.2f} mm3)"

    # the wedge did its job: no cap/block/clamp material above the plane
    for key, M in (("knee_clamp_cap_ovh", T["knee_cap"]),
                   ("femur_ovh_body", Mf)):
        m = meshes[key]
        vloc = m.vertices if key == "femur_ovh_body" else \
            trimesh.transform_points(m.vertices,
                                     np.linalg.inv(MH) @ M_KNEE_JP)
        sel = (vloc[:, 0] > WEDGE_XY0[0] - 0.01) \
            & (vloc[:, 0] < WEDGE_XY1[0] - 0.01)
        excess = vloc[sel][:, 1] - wedge_y(vloc[sel][:, 0]) \
            if sel.any() else np.array([-1.0])
        assert float(excess.max()) < 0.05, \
            f"{key}: material {excess.max():.2f} above the overhead wedge"
    print(f"  knee cap OVH: seats + lifts clean, tail wedge removed "
          f"{dv:.1f} cm3, up-screw M3x25 path open into the "
          f"Phi {CAP_INSERT_D:g}x{CAP_INSERT_LEN:g} insert boss")


def _min_gap(a: trimesh.Trimesh, b: trimesh.Trimesh, n: int = 800) -> float:
    """Approximate minimum surface-to-surface gap (mm, negative=overlap):
    UNSIGNED closest distance from a point sample of each surface to the
    other mesh, both directions.  Vertex sets are subsampled (full leg
    assemblies carry 100k+ vertices; a proximity query over all of them
    runs for minutes).  Callers assert gaps >> the sampling error and
    separately assert zero overlap via booleans where it matters."""
    rng = np.random.default_rng(0)

    def pick(m):
        v = m.vertices
        if len(v) > 4000:
            v = v[rng.choice(len(v), 4000, replace=False)]
        return np.vstack([v, trimesh.sample.sample_surface(m, n)[0]])

    d1 = float(trimesh.proximity.ProximityQuery(b).on_surface(pick(a))[1]
               .min())
    d2 = float(trimesh.proximity.ProximityQuery(a).on_surface(pick(b))[1]
               .min())
    return min(d1, d2)


def _hip_fixed_assembly(meshes, i: int, yaw: float) -> trimesh.Trimesh:
    """Everything that rotates with leg i's yaw joint but NOT with the
    femur: variant coxa + hip servo + hip cap + third 6805."""
    T = leg_transforms(i, yaw_deg=yaw)
    return trimesh.util.concatenate([
        _placed(meshes, "coxa_link_ovh", T["coxa"]),
        _placed(meshes, "servo_body", T["hip_cap"]),
        _placed(meshes, "hip_clamp_cap_ovh", T["hip_cap"]),
        _placed(meshes, "bearing_6805", T["yaw_top"])])


def _femur_assembly(meshes, i: int, yaw: float,
                    femur: float = None) -> trimesh.Trimesh:
    """Everything on the femur/tibia side of leg i (clamps, block, knee
    servo/cap, socket, tube, foot) at the given yaw/femur pose."""
    T = leg_transforms(i, yaw_deg=yaw, femur_deg=femur)
    tube = meshes["tibia_tube_ovh"]
    _, foot_frame = _tibia_tube()
    parts = [
        _placed(meshes, "chorn_clamp_cnc", T["femur"] @ MH),
        _placed(meshes, "femur_ovh_body", T["femur"] @ MH),
        _placed(meshes, "servo_body", T["knee_cap"]),
        _placed(meshes, "knee_clamp_cap_ovh", T["knee_cap"]),
        _placed(meshes, "chorn_clamp_cnc", T["tibia"] @ MH),
        _placed(meshes, "tibia_ovh_socket", T["tibia"] @ MH),
        _placed(meshes, "foot_boot", T["tibia"] @ foot_frame)]
    t = tube.copy()
    t.apply_transform(T["tibia"])
    parts.append(t)
    return trimesh.util.concatenate(parts)


def check_coxa_ovh(meshes: dict[str, trimesh.Trimesh]) -> None:
    """The variant coxa: yaw-axis interfaces bit-identical to the rigid
    coxa (horn drive, hub boss, seat ring, dust brim), hip cradle moved
    COXA_EXT out with the same servo squeeze, hip-axis wells open."""
    ovh = meshes["coxa_link_ovh"]
    rig = meshes["coxa_link"]

    # (a) interface band: everything below the slab bottom inside the
    # brim radius is the yaw joint (hub column, seat ring, brim) and
    # must be IDENTICAL -- the bracket move may only touch z >= SLAB
    band = trimesh.creation.cylinder(radius=rv.BRIM_OD / 2.0 + 0.1,
                                     height=80.0)
    band.apply_translation([0.0, 0.0, rv.SLAB_BOT_Z - 0.05 - 40.0])
    v_o = _inter_vol(ovh, band)
    v_r = _inter_vol(rig, band)
    assert abs(v_o - v_r) < max(1.0, 0.002 * v_r), \
        f"yaw interface band differs: {v_o:.1f} vs rigid {v_r:.1f} mm3"

    # (b) horn drive voids: centre + 4 bolt shafts open through the hub
    r = hp.DISC_HORN_BOLT_PCD / 2.0
    stations = [(0.0, 0.0)] + [(r * np.cos(t), r * np.sin(t))
                               for t in hp.DISC_HORN_BOLT_ANGLES_RAD]
    probes = [[sx, sy, z] for sx, sy in stations
              for z in np.arange(rv.HORN_HEAD_SEAT_Z + 0.4, 11.0, 0.8)]
    assert not ovh.contains(np.asarray(probes)).any(), \
        "a horn screw shaft is blocked in the variant coxa"

    # (c) cradle: same servo squeeze as rigid, just COXA_EXT further out
    T = leg_transforms(0)
    Tr = rv.leg_transforms(0)
    servo_o = _placed(meshes, "servo_body", T["hip_cap"])
    servo_r = _placed(meshes, "servo_body", Tr["hip_cap"])
    coxa_o = _placed(meshes, "coxa_link_ovh", T["coxa"])
    coxa_r = _placed(meshes, "coxa_link", Tr["coxa"])
    v_o = _inter_vol(servo_o, coxa_o)
    v_r = _inter_vol(servo_r, coxa_r)
    assert abs(v_o - v_r) < max(20.0, 0.05 * v_r), \
        f"cradle servo squeeze changed: {v_o:.1f} vs rigid {v_r:.1f} mm3"

    # (d) plan reach (the new rotating envelope, replaces the 38.2 trim)
    r_ovh = float(np.linalg.norm(ovh.vertices[:, :2], axis=1).max())
    print(f"  coxa OVH: yaw interfaces identical, servo squeeze "
          f"{v_o:.0f} mm3 (rigid {v_r:.0f}), plan reach {r_ovh:.1f} mm "
          f"(rigid trims at {rv.ROT_ENVELOPE_R:g})")


def check_hip_cap_ovh(meshes: dict[str, trimesh.Trimesh]) -> None:
    """The variant hip cap: production jaw squeeze at the MOVED hip
    station, the yaw-axis furniture (pedestal + 6805 press boss + puller
    notches) at exactly the rigid cap's station and height (the top
    plate seats on that bearing), no plate contact, and the L0 access
    hole still gives a straight driver line to the horn screws."""
    T = leg_transforms(0)
    Tr = rv.leg_transforms(0)
    cap = _placed(meshes, "hip_clamp_cap_ovh", T["hip_cap"])
    cap_r = _placed(meshes, "hip_clamp_cap_rigid", Tr["hip_cap"])

    # (a) jaw squeeze == rigid/production (the jaw IS the stock cap,
    # translated with the hip servo)
    v_o = _inter_vol(cap, _placed(meshes, "servo_body", T["hip_cap"]))
    v_r = _inter_vol(cap_r, _placed(meshes, "servo_body", Tr["hip_cap"]))
    assert abs(v_o - v_r) < max(20.0, 0.05 * v_r), \
        f"hip cap jaw squeeze changed: {v_o:.1f} vs rigid {v_r:.1f} mm3"

    # (b) yaw furniture where rigid put it: same 6805 press engagement,
    # boss tip at the same world height, boss centred on the yaw axis
    br = _placed(meshes, "bearing_6805", T["yaw_top"])
    br_r = _placed(meshes, "bearing_6805", Tr["hip_cap"])
    p_o = _inter_vol(cap, br)
    p_r = _inter_vol(cap_r, br_r)
    assert abs(p_o - p_r) < max(2.0, 0.05 * p_r), \
        f"6805 press engagement changed: {p_o:.1f} vs rigid {p_r:.1f} mm3"
    assert abs(cap.bounds[1][2] - cap_r.bounds[1][2]) < 1e-3, \
        "variant cap boss tip not at the rigid stack height"
    axis = T["coxa"][:2, 3]
    vtx = cap.vertices
    tipband = vtx[vtx[:, 2] > cap.bounds[1][2] - 1.0]
    c = tipband[:, :2].mean(axis=0)
    assert float(np.hypot(*(c - axis))) < 0.2, \
        f"boss tip centred {np.hypot(*(c - axis)):.2f} mm off the yaw axis"

    # (c) plate + hatch untouched by the cap (arm included)
    for key in ("chassis_top_rigid", "top_hatch_rigid"):
        v = _inter_vol(cap, meshes[key])
        assert v < 1e-6, f"hip cap OVH intersects {key} ({v:.2f} mm3)"

    # (d) driver line of sight: Phi 6.5 shaft through the L0 access hole
    # down to the cap face, vs the VARIANT stack (the rv check covers
    # the rigid parts only)
    hx, hy = rv._access_hole_xy()[0]
    shaft = rv._cyl_z(3.25, rv.CAP_FACE_W + 0.05, rv.SHEET_Z1 + 30.0,
                      x=hx, y=hy, sections=48)
    for key, fr in (("coxa_link_ovh", "coxa"), ("servo_body", "hip_cap"),
                    ("hip_clamp_cap_ovh", "hip_cap"),
                    ("bearing_6805", "yaw_top")):
        v = _inter_vol(shaft, _placed(meshes, key, T[fr]))
        assert v < 1e-6, f"driver shaft fouls {key} ({v:.2f} mm3)"
    print(f"  hip cap OVH: jaw squeeze {v_o:.0f} mm3 (rigid {v_r:.0f}), "
          f"6805 press {p_o:.1f} mm3 on the yaw axis, plate/hatch clear, "
          f"driver access open")


def check_yaw_envelope_ovh(meshes: dict[str, trimesh.Trimesh]) -> None:
    """The outboard cradle breaks rigid_hip's 38.2 mm full-circle yaw
    envelope ON PURPOSE.  Replacement guarantees, all measured:
      1. free yaw range of the full leg assembly >= +-40 deg (software
         limit is +-35) against plate/hatch/chassis/wago/neighbors;
      2. worst-case adjacent-leg convergence (+35 vs -35) keeps a real
         gap at stance, mid-lift and overhead;
      3. within the free range the rotating slab stays >= ENV_MIN_CL
         clear of the centre wago block (flipped-lever proxy included).
    """
    # static world: chassis + centre furniture + neighbor legs at yaw 0
    lever_top = rv.BOT_SHEET_TOP_Z + rv.WBLK_FLOOR_T + hp.WAGO5_H + 10.0
    statics = [("plate", meshes["chassis_top_rigid"]),
               ("hatch", meshes["top_hatch_rigid"]),
               ("chassis", meshes["chassis_bottom"]),
               ("wago block", meshes["centre_wago_block"])]
    for k, M in enumerate(rv._wago5_scene_frames()):
        w = _placed(meshes, "wago5", M)
        lo, hi = w.bounds
        proxy = trimesh.creation.box(
            bounds=[[lo[0], lo[1], lo[2]], [hi[0], hi[1], lever_top]])
        statics.append((f"wago5+lever {k}", proxy))
    T0 = leg_transforms(0)
    statics.append(("yaw retainer", _placed(meshes, "yaw_servo_retainer",
                                            T0["cradle"])))
    # leg 0's own yaw servo: the coxa hub ENGAGES its horn by design
    # (constant 862 mm3 at every yaw -- the drive).  Static for the
    # range check, but only overlap GROWTH beyond the engagement counts.
    own_servo = _placed(
        meshes, "servo_body",
        T0["coxa"] @ _trans([-hp.SERVO_OUTPUT_X, 0.0,
                             -(hp.HORN_STACK_H + hp.WELL_RIM_Z)]))
    for i in (1, 5):                     # only neighbors can be reached
        statics.append((f"L{i} hip assembly",
                        _hip_fixed_assembly(meshes, i, 0.0)))
        statics.append((f"L{i} femur assembly",
                        _femur_assembly(meshes, i, 0.0)))

    def leg0(yaw):
        return trimesh.util.concatenate([
            _hip_fixed_assembly(meshes, 0, yaw),
            _femur_assembly(meshes, 0, yaw)])

    v_drive = _inter_vol(leg0(0.0), own_servo)

    free_lo = free_hi = 0.0
    for sgn in (+1, -1):
        for step_deg in np.arange(5.0, 62.6, 5.0):
            m = leg0(sgn * step_deg)
            hit = next((nm for nm, s in statics
                        if _bounds_touch(m, s) and _inter_vol(m, s) > 1.0),
                       None)
            if hit is None and _inter_vol(m, own_servo) > v_drive + 5.0:
                hit = "own yaw servo (beyond drive engagement)"
            if hit:
                print(f"  yaw range: first contact at {sgn * step_deg:+.0f}"
                      f" deg ({hit})")
                break
            if sgn > 0:
                free_hi = step_deg
            else:
                free_lo = -step_deg
    assert free_hi >= 40.0 and free_lo <= -40.0, \
        f"free yaw range only {free_lo:+.0f}..{free_hi:+.0f} (need +-40)"
    print(f"  yaw envelope OVH: free range {free_lo:+.0f}..{free_hi:+.0f}"
          f" deg (software limit +-35)")

    # adjacent-leg worst-case convergence
    worst = None
    for fem in (30.0, None, -60.0, -110.0):
        a = trimesh.util.concatenate([_hip_fixed_assembly(meshes, 0, 35.0),
                                      _femur_assembly(meshes, 0, 35.0, fem)])
        b = trimesh.util.concatenate([_hip_fixed_assembly(meshes, 1, -35.0),
                                      _femur_assembly(meshes, 1, -35.0,
                                                      fem)])
        gap = _min_gap(a, b)
        tag = "stance" if fem is None else f"{fem:+.0f}"
        print(f"  adjacent legs +35/-35 @ femur {tag}: gap {gap:.1f} mm")
        worst = gap if worst is None else min(worst, gap)
    assert worst >= 5.0, f"adjacent-leg gap {worst:.1f} < 5 mm"

    # wago clearance inside the free range
    wago = trimesh.util.concatenate(
        [meshes["centre_wago_block"]]
        + [s for nm, s in statics if nm.startswith("wago5+lever")])
    wmin = None
    for yaw in np.arange(-40.0, 40.1, 10.0):
        g = _min_gap(_hip_fixed_assembly(meshes, 0, float(yaw)), wago)
        wmin = g if wmin is None else min(wmin, g)
    assert wmin >= rv.ENV_MIN_CL, \
        f"rotating coxa within {wmin:.1f} mm of the wago block"
    print(f"  centre wago block: >= {wmin:.1f} mm from the rotating "
          f"coxa across the free range (need {rv.ENV_MIN_CL:g})")


def check_assembly_paths_ovh(meshes: dict[str, trimesh.Trimesh]) -> None:
    """rigid_hip's plate-descent and hatch drop-in re-verified with the
    VARIANT coxa + outboard hip frames (the rv versions check the rigid
    coxa, which this concept no longer uses)."""
    static = []
    for i in range(6):
        T = leg_transforms(i)
        for key, fr in (("coxa_link_ovh", "coxa"), ("servo_body", "hip_cap"),
                        ("hip_clamp_cap_ovh", "hip_cap"),
                        ("bearing_6805", "yaw_top")):
            m = _placed(meshes, key, T[fr])
            static.append((f"L{i}-{key}", m))
    plate = meshes["chassis_top_rigid"]
    for dz in (0.5, 2.0, 5.0, 10.0, 25.0, 50.0):
        p = plate.copy()
        p.apply_translation([0, 0, dz])
        for name, m in static:
            v = _inter_vol(p, m)
            assert v < 1e-6, f"descent +{dz}: plate fouls {name} ({v:.1f})"
    hatch = meshes["top_hatch_rigid"]
    for dz in (0.5, 2.0, 5.0, 20.0, 60.0):
        h = hatch.copy()
        h.apply_translation([0, 0, dz])
        for name, m in static:
            v = _inter_vol(h, m)
            assert v < 1e-6, f"hatch +{dz}: fouls {name} ({v:.1f})"
    print("  plate descent + hatch drop-in: clear with the OVH coxa "
          "(all six legs)")


def sweep_femur_envelope(meshes: dict[str, trimesh.Trimesh],
                         yaw_grid=(-35.0, -20.0, 0.0, 20.0, 35.0),
                         p_hi: float = -140.0, step: float = SWEEP_STEP):
    """The variant's whole point: per yaw angle, the first femur UP angle
    at which ANY femur/tibia-frame part contacts (A) the leg's own
    yaw-rotating fixed side (hip cap + bearing + coxa + hip servo) or
    (B) the world-fixed plate + hatch.  rigid_hip measured -47.5 here;
    the target is past vertical."""
    plate = meshes["chassis_top_rigid"]
    hatch = meshes["top_hatch_rigid"]
    tube = meshes["tibia_tube_ovh"]
    moving_keys = (("chorn_clamp_cnc", "femur_mh"),
                   ("femur_ovh_body", "femur_mh"),
                   ("servo_body", "knee_cap"),
                   ("knee_clamp_cap_ovh", "knee_cap"),
                   ("chorn_clamp_cnc", "tibia_mh"),
                   ("tibia_ovh_socket", "tibia_mh"),
                   ("tibia_tube_ovh", "tibia"),   # tube is authored in the
                   ("foot_boot", "foot"))         # JOINT frame (MH baked in)

    _, foot_frame = _tibia_tube()

    def moving(i, yaw, p):
        T = leg_transforms(i, yaw_deg=yaw, femur_deg=p)
        frames = {"femur_mh": T["femur"] @ MH, "knee_cap": T["knee_cap"],
                  "tibia_mh": T["tibia"] @ MH, "tibia": T["tibia"],
                  "foot": T["tibia"] @ foot_frame}
        for key, fr in moving_keys:
            m = (tube if key == "tibia_tube_ovh" else meshes[key]).copy()
            m.apply_transform(frames[fr])
            yield key, m

    # (A) vs the leg's own yaw-rotating stack -- yaw-independent
    T0 = leg_transforms(0)
    stack = trimesh.util.concatenate([
        _placed(meshes, "hip_clamp_cap_ovh", T0["hip_cap"]),
        _placed(meshes, "bearing_6805", T0["yaw_top"]),
        _placed(meshes, "coxa_link_ovh", T0["coxa"]),
        _placed(meshes, "servo_body", T0["hip_cap"])])
    contact_a, part_a = None, None
    for p in np.arange(0.0, p_hi - 1e-9, -step):
        hits = [k for k, m in moving(0, 0.0, p) if _inter_vol(m, stack) > 1.0]
        if hits:
            contact_a, part_a = float(p), hits[0]
            break
    # (B) vs the world-fixed plate + hatch, per yaw
    contact_b, part_b = {}, {}
    for yaw in yaw_grid:
        contact_b[yaw], part_b[yaw] = None, None
        for p in np.arange(0.0, p_hi - 1e-9, -step):
            hit = None
            for k, m in moving(0, yaw, p):
                if m.bounds[1][2] < rv.RING_BOT_W:      # cheap z prefilter
                    continue
                if _inter_vol(m, plate) > 1.0 or _inter_vol(m, hatch) > 1.0:
                    hit = k
                    break
            if hit:
                contact_b[yaw], part_b[yaw] = float(p), hit
                break
    print(f"  vs own coxa/cap/servo stack: first contact {contact_a} deg"
          f" ({part_a})")
    for yaw in yaw_grid:
        print(f"  vs plate+hatch @ yaw {yaw:+5.1f}: first contact "
              f"{contact_b[yaw]} deg ({part_b[yaw]})")
    worst = [c for c in ([contact_a] + list(contact_b.values()))
             if c is not None]
    limit = (max(worst) + SWEEP_MARGIN) if worst else p_hi
    limit = float(np.ceil(limit / 2.5) * 2.5)
    print(f"  => safe femur UP limit (first grid contact + {SWEEP_MARGIN:g}"
          f" margin): {limit} deg  [rigid_hip: -47.5, production: -80]")
    assert limit <= SWEEP_TARGET, (
        f"overhead sweep only reached {limit} deg -- the clamp growth "
        f"missed the legs-over-head target ({SWEEP_TARGET})")
    return limit, (contact_a, part_a), contact_b, part_b


def check_down_and_knee_range(meshes: dict[str, trimesh.Trimesh]) -> None:
    """The grown clamp must not cost the OLD workspace: femur down-swing
    to +30 and the knee's -30..+20 at stance all stay contact-free
    against chassis bottom + the leg's own fixed side."""
    cb = meshes["chassis_bottom"]
    T0 = leg_transforms(0)
    stack = trimesh.util.concatenate([
        _placed(meshes, "hip_clamp_cap_ovh", T0["hip_cap"]),
        _placed(meshes, "coxa_link_ovh", T0["coxa"]),
        _placed(meshes, "servo_body", T0["hip_cap"])])
    _, foot_frame = _tibia_tube()
    for p in (30.0, 15.0, 0.0):
        T = leg_transforms(0, femur_deg=p)
        for key, M in (("chorn_clamp_cnc", T["femur"] @ MH),
                       ("femur_ovh_body", T["femur"] @ MH),
                       ("chorn_clamp_cnc", T["tibia"] @ MH),
                       ("tibia_ovh_socket", T["tibia"] @ MH)):
            m = _placed(meshes, key, M)
            for name, f in (("chassis_bottom", cb), ("hip stack", stack)):
                v = _inter_vol(m, f)
                assert v < 1e-6, \
                    f"femur {p:+.0f}: {key} hits {name} ({v:.1f} mm3)"
    print("  down-swing +30 and stance knee range: clear "
          "(old workspace preserved)")


def report_masses(meshes: dict[str, trimesh.Trimesh]) -> dict:
    """Mass/BOM delta table vs rigid_hip (which used the production
    printed femur + tibia yoke)."""
    def g(key, rho):
        return meshes[key].volume * rho

    old_femur = g("femur_link", RHO_PETG)
    old_yoke = g("tibia_knee_yoke", RHO_PETG)
    old_cap = g("knee_clamp_cap", RHO_PETG)
    old_coxa = g("coxa_link", RHO_PETG)
    old_hcap = g("hip_clamp_cap_rigid", RHO_PETG)
    clamp = g("chorn_clamp_cnc", RHO_ALU)
    body = g("femur_ovh_body", RHO_PETG)
    sock = g("tibia_ovh_socket", RHO_PETG)
    cap = g("knee_clamp_cap_ovh", RHO_PETG)
    coxa = g("coxa_link_ovh", RHO_PETG)
    hcap = g("hip_clamp_cap_ovh", RHO_PETG)
    per_leg_old = old_femur + old_yoke + old_cap + old_coxa + old_hcap
    per_leg_new = 2 * clamp + body + sock + cap + coxa + hcap
    rows = {
        "chorn_clamp_cnc_g": round(clamp, 1),
        "femur_ovh_body_g": round(body, 1),
        "tibia_ovh_socket_g": round(sock, 1),
        "knee_clamp_cap_ovh_g": round(cap, 1),
        "coxa_link_ovh_g": round(coxa, 1),
        "hip_clamp_cap_ovh_g": round(hcap, 1),
        "old_femur_link_g": round(old_femur, 1),
        "old_tibia_knee_yoke_g": round(old_yoke, 1),
        "old_knee_clamp_cap_g": round(old_cap, 1),
        "old_coxa_link_rigid_g": round(old_coxa, 1),
        "old_hip_clamp_cap_rigid_g": round(old_hcap, 1),
        "per_leg_delta_g": round(per_leg_new - per_leg_old, 1),
        "robot_delta_g": round(6 * (per_leg_new - per_leg_old), 1),
    }
    print(f"  masses: clamp {clamp:.1f} g (alu) x12, femur body {body:.1f}, "
          f"tibia socket {sock:.1f}, knee cap OVH {cap:.1f}, "
          f"coxa OVH {coxa:.1f}, hip cap OVH {hcap:.1f} g")
    print(f"  vs rigid_hip leg parts (femur {old_femur:.1f} + yoke "
          f"{old_yoke:.1f} + knee cap {old_cap:.1f} + coxa {old_coxa:.1f}"
          f" + hip cap {old_hcap:.1f} g): "
          f"{rows['per_leg_delta_g']:+.1f} g/leg, "
          f"{rows['robot_delta_g']:+.1f} g/robot")
    return rows


# COTS masses for the yaw-moment bookkeeping (measured/catalog values;
# printed parts use volume x density).
M_SERVO_G = 60.0               # STS3215 catalog mass
RHO_CF = 1.60e-3               # g/mm3, pultruded CF tube
RHO_TPU = 1.21e-3              # g/mm3, foot boot


def report_outboard_geometry(meshes: dict[str, trimesh.Trimesh]) -> dict:
    """Quantify what the outboard pivot costs/buys vs rigid_hip: hip
    axis station, stance footprint, and the yaw-joint moment growth
    (point-mass approximation at each part's cg -- good to a few %)."""
    def stats(new: bool):
        lt = leg_transforms if new else rv.leg_transforms
        tube = (meshes["tibia_tube_ovh"] if new else rv._tibia_extras()[0])
        _, ff = (_tibia_tube() if new else rv._tibia_extras())
        T = lt(0)
        # (mass g, mesh, frame) for everything rotating with leg 0's yaw
        parts = [
            (meshes["coxa_link_ovh" if new else "coxa_link"].volume
             * RHO_PETG, "coxa_link_ovh" if new else "coxa_link", T["coxa"]),
            (M_SERVO_G, "servo_body", T["hip_cap"]),
            (meshes["hip_clamp_cap_ovh" if new
                    else "hip_clamp_cap_rigid"].volume * RHO_PETG,
             "hip_clamp_cap_ovh" if new else "hip_clamp_cap_rigid",
             T["hip_cap"]),
            (22.0, "bearing_6805", T["yaw_top"] if new else T["hip_cap"]),
            (M_SERVO_G, "servo_body", T["knee_cap"]),
            (tube.volume * RHO_CF, None, T["tibia"]),
            (meshes["foot_boot"].volume * RHO_TPU, "foot_boot",
             T["tibia"] @ ff)]
        if new:
            parts += [
                (meshes["chorn_clamp_cnc"].volume * RHO_ALU,
                 "chorn_clamp_cnc", T["femur"] @ MH),
                (meshes["femur_ovh_body"].volume * RHO_PETG,
                 "femur_ovh_body", T["femur"] @ MH),
                (meshes["knee_clamp_cap_ovh"].volume * RHO_PETG,
                 "knee_clamp_cap_ovh", T["knee_cap"]),
                (meshes["chorn_clamp_cnc"].volume * RHO_ALU,
                 "chorn_clamp_cnc", T["tibia"] @ MH),
                (meshes["tibia_ovh_socket"].volume * RHO_PETG,
                 "tibia_ovh_socket", T["tibia"] @ MH)]
        else:
            parts += [
                (meshes["femur_link"].volume * RHO_PETG, "femur_link",
                 T["femur"] @ MH),
                (meshes["knee_clamp_cap"].volume * RHO_PETG,
                 "knee_clamp_cap", T["knee_cap"]),
                (meshes["tibia_knee_yoke"].volume * RHO_PETG,
                 "tibia_knee_yoke", T["tibia"] @ MH)]
        axis = T["coxa"][:2, 3]
        mass = first = second = 0.0     # g / g*mm / g*mm2 about yaw axis
        for m_g, key, M in parts:
            mm = (tube if key is None else meshes[key]).copy()
            mm.apply_transform(M)
            cg = mm.center_mass
            d = float(np.hypot(cg[0] - axis[0], cg[1] - axis[1]))
            mass += m_g
            first += m_g * d
            second += m_g * d * d
        foot = (T["tibia"] @ ff)[:2, 3]
        lever = float(np.hypot(foot[0] - axis[0], foot[1] - axis[1]))
        hip_r = float(np.linalg.norm(T["femur"][:2, 3]))
        return dict(mass_g=round(mass, 1),
                    yaw_cg_lever_mm=round(first / mass, 1),
                    yaw_first_moment_gmm=round(first, 0),
                    yaw_inertia_gmm2=round(second, 0),
                    stance_foot_lever_mm=round(lever, 1),
                    hip_axis_radial_mm=round(hip_r, 1))

    old, new = stats(False), stats(True)
    rows = {"rigid_hip": old, "cnc_chorn_overhead": new,
            "coxa_ext_mm": COXA_EXT}
    print(f"  outboard geometry: hip axis radial "
          f"{old['hip_axis_radial_mm']} -> {new['hip_axis_radial_mm']} mm, "
          f"stance foot lever about the yaw axis "
          f"{old['stance_foot_lever_mm']} -> {new['stance_foot_lever_mm']}"
          f" mm")
    print(f"  yaw joint: rotating mass {old['mass_g']} -> {new['mass_g']} g,"
          f" cg lever {old['yaw_cg_lever_mm']} -> {new['yaw_cg_lever_mm']}"
          f" mm, first moment {old['yaw_first_moment_gmm']:.0f} -> "
          f"{new['yaw_first_moment_gmm']:.0f} g*mm "
          f"({new['yaw_first_moment_gmm'] / old['yaw_first_moment_gmm']:.2f}"
          f"x), point-mass yaw inertia "
          f"{old['yaw_inertia_gmm2'] / 1e6:.2f} -> "
          f"{new['yaw_inertia_gmm2'] / 1e6:.2f} kg*cm2 "
          f"({new['yaw_inertia_gmm2'] / old['yaw_inertia_gmm2']:.2f}x)")
    return rows


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------
SCENE_MESH_FILES = {
    # this concept's parts (print set / CNC quoting set)
    "chorn_clamp_cnc": "chorn_clamp_cnc_CNC_6061.stl",
    "femur_ovh_body": "femur_ovh_body.stl",
    "tibia_ovh_socket": "tibia_ovh_socket.stl",
    "knee_clamp_cap_ovh": "knee_clamp_cap_ovh.stl",
    "coxa_link_ovh": "coxa_link_ovh.stl",
    "hip_clamp_cap_ovh": "hip_clamp_cap_ovh.stl",
    "tibia_tube_ovh": "tibia_tube_ovh_DO_NOT_PRINT.stl",
    # inherited rigid_hip prints + COTS visuals (copied for the viewer)
    "chassis_top_rigid": "chassis_top_rigid.stl",
    "top_hatch_rigid": "top_hatch_rigid.stl",
    "centre_wago_block": "centre_wago_block.stl",
    "chassis_bottom": "chassis_bottom_rigid.stl",
    "foot_boot": "foot_boot.stl",
    "yaw_servo_retainer": "yaw_servo_retainer.stl",
    "servo_body": "servo_body_DO_NOT_PRINT.stl",
    "yaw_bearing_upper": "yaw_bearing_upper_DO_NOT_PRINT.stl",
    "bearing_6805": "bearing_6805_DO_NOT_PRINT.stl",
    "wago5": "wago5_DO_NOT_PRINT.stl",
}
SCENE_MESH_KEYS = list(SCENE_MESH_FILES)

COLORS = dict(rv.COLORS)
COLORS.update({
    "chorn_clamp_cnc": "#c8ccd2",      # machined aluminum
    "femur_ovh_body": "#7fb069",
    "tibia_ovh_socket": "#7fb069",
    "knee_clamp_cap_ovh": "#9dc183",
    "coxa_link_ovh": "#5f9e6e",        # the outboard-arm coxa (NEW print)
    "hip_clamp_cap_ovh": "#8fbf7f",    # hip cap + yaw pedestal arm (NEW)
    "tibia_tube_ovh": "#404040",
})
COTS = set(rv.COTS) | {"tibia_tube_ovh"}


def _mat16(M: np.ndarray) -> list[float]:
    return [float(x) for x in np.asarray(M, float).T.reshape(-1)]


def build_scene(meshes, femur_up_limit: float) -> dict:
    _, foot_frame = _tibia_tube()
    mesh_defs = [{"id": f"stl:{k}", "name": fn, "url": f"stl/{fn}"}
                 for k, fn in SCENE_MESH_FILES.items()]

    instances, joints = [], []
    n = 0

    def inst(key, name, M, leg=None, part=None):
        nonlocal n
        iid = f"{n:03d}-{name}"
        n += 1
        instances.append({
            "id": iid, "meshId": f"stl:{key}", "name": name,
            "partType": part or key, "role": "variant", "leg": leg,
            "joint": None, "cots": key in COTS, "color": COLORS[key],
            "transform": _mat16(M)})
        return iid

    inst("chassis_bottom", "chassis_bottom RIGID", np.eye(4))
    inst("chassis_top_rigid", "chassis_top_rigid FRAME", np.eye(4))
    inst("top_hatch_rigid", "top_hatch (removable)", np.eye(4))
    inst("centre_wago_block", "central splice block", np.eye(4))
    for M, label in zip(rv._wago5_scene_frames(),
                        ("V+ west", "V+ east", "GND west", "GND east")):
        inst("wago5", f"221-415 {label}", M)
    for i in range(6):
        T = leg_transforms(i)
        a = (i + 0.5) * np.pi / 3.0
        axis_pt = [rv.APOTHEM * np.cos(a), rv.APOTHEM * np.sin(a),
                   hp.CHASSIS_YAW_OUTPUT_Z]
        pitch_ax = (rv._rotz(a)[:3, :3] @ np.array([0.0, 1.0, 0.0])).tolist()
        hip_pt = (T["coxa"] @ np.array([*HIP_ANCHOR_OVH, 1.0]))[:3]
        knee_pt = (T["femur"]
                   @ np.array([hp.FEMUR_LENGTH, 0.0, 0.0, 1.0]))[:3]

        yaw_ids = [
            inst("coxa_link_ovh", f"L{i} coxa OVH outboard arm (NEW)",
                 T["coxa"], leg=i),
            inst("yaw_bearing_upper", f"L{i} yaw bearing (tower-seated)",
                 T["coxa"] @ _trans([0.0, 0.0, rv.YAWBR_DROP]), leg=i),
            inst("servo_body", f"L{i} hip servo", T["hip_cap"], leg=i),
            inst("hip_clamp_cap_ovh", f"L{i} hip cap OVH + yaw pedestal "
                 "(NEW)", T["hip_cap"], leg=i),
            inst("bearing_6805", f"L{i} third 6805", T["yaw_top"], leg=i),
        ]
        inst("yaw_servo_retainer", f"L{i} yaw retainer", T["cradle"], leg=i)
        inst("servo_body", f"L{i} yaw servo",
             T["coxa"] @ _trans([-hp.SERVO_OUTPUT_X, 0.0,
                                 -(hp.HORN_STACK_H + hp.WELL_RIM_Z)]),
             leg=i)
        hip_ids = [
            inst("chorn_clamp_cnc", f"L{i} hip C-clamp (CNC 6061, NEW)",
                 T["femur"] @ MH, leg=i),
            inst("femur_ovh_body", f"L{i} femur body OVH (NEW)",
                 T["femur"] @ MH, leg=i),
            inst("servo_body", f"L{i} knee servo", T["knee_cap"], leg=i),
            inst("knee_clamp_cap_ovh", f"L{i} knee cap OVH (NEW)",
                 T["knee_cap"], leg=i),
        ]
        knee_ids = [
            inst("chorn_clamp_cnc", f"L{i} knee C-clamp (CNC 6061, NEW)",
                 T["tibia"] @ MH, leg=i),
            inst("tibia_ovh_socket", f"L{i} tibia socket OVH (NEW)",
                 T["tibia"] @ MH, leg=i),
            inst("tibia_tube_ovh", f"L{i} tibia tube (shortened)",
                 T["tibia"], leg=i),
            inst("foot_boot", f"L{i} foot boot",
                 T["tibia"] @ foot_frame, leg=i),
        ]
        joints += [
            {"id": f"L{i}-yaw", "type": "revolute", "axis": [0, 0, 1],
             "origin": axis_pt, "instances": yaw_ids,
             "limits": {"min": -35.0, "max": 35.0}, "home": 0,
             "label": f"L{i} yaw"},
            {"id": f"L{i}-hip", "type": "revolute", "axis": pitch_ax,
             "origin": [float(x) for x in hip_pt], "instances": hip_ids,
             "limits": {"min": femur_up_limit, "max": 30.0}, "home": 0,
             "label": f"L{i} hip (OVERHEAD limit {femur_up_limit:g})"},
            {"id": f"L{i}-knee", "type": "revolute", "axis": pitch_ax,
             "origin": [float(x) for x in knee_pt], "instances": knee_ids,
             "limits": {"min": -30.0, "max": 20.0}, "home": 0,
             "label": f"L{i} knee"},
        ]

    up = femur_up_limit - hp.STANCE_FEMUR_DEG   # joint value at the limit
    scene = {
        "name": "sts3215 CNC C-clamp overhead variant -- legs over head",
        "source": "concepts/cnc_chorn_overhead/make_cnc_chorn_variant.py",
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": [0, 0, 55],
        "checksConfig": {
            "overlapMm3": 80.0, "pitchMm": 2.0,
            "ignoreOverlapPairs": [
                ["chassis_top_rigid", "top_hatch_rigid"],
                ["centre_wago_block", "wago5"],
                ["chassis_bottom", "servo_body"],
                ["coxa_link_ovh", "servo_body"],
                ["coxa_link_ovh", "yaw_bearing_upper"],
                ["hip_clamp_cap_ovh", "servo_body"],
                ["femur_ovh_body", "servo_body"],
                ["knee_clamp_cap_ovh", "servo_body"],
                ["foot_boot", "tibia_tube_ovh"],
                ["tibia_ovh_socket", "tibia_tube_ovh"],
            ]},
        "meshes": mesh_defs,
        "instances": instances,
        "joints": joints,
        "poses": [
            {"id": "stance", "name": "Stance (walking)",
             "jointValues": {j["id"]: 0.0 for j in joints}},
            {"id": "legs-over-head",
             "name": f"LEGS OVER HEAD (all hips at {femur_up_limit:g} deg)",
             "jointValues": {f"L{i}-hip": up for i in range(6)}},
            {"id": "femur-up-limit",
             "name": f"L0 femur at the new up limit ({femur_up_limit:g})",
             "jointValues": {"L0-hip": up}},
        ],
        "animations": [
            {"id": "overhead", "name": "all legs: stance -> over the head",
             "loop": True, "duration": 10.0,
             "keyframes": [
                 {"t": 0.0, "jointValues":
                     {f"L{i}-hip": 0 for i in range(6)}},
                 {"t": 2.0, "jointValues": {"L0-hip": up}},
                 {"t": 4.5, "jointValues":
                     {f"L{i}-hip": up for i in range(6)}},
                 {"t": 6.5, "jointValues":
                     {f"L{i}-hip": up for i in range(6)}},
                 {"t": 9.0, "jointValues":
                     {f"L{i}-hip": 0 for i in range(6)}},
             ]},
        ],
    }
    return scene


def render_preview(meshes, limit: float) -> None:
    """Elevation through leg 0's pitch plane at the overhead limit: the
    money shot of the femur past vertical over the top plate."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    a = 0.5 * np.pi / 3.0
    Rz = rv._rotz(-a)
    T = leg_transforms(0, femur_deg=limit)
    Ts = leg_transforms(0)
    _, foot_frame = _tibia_tube()
    sections = [
        ("chassis_bottom", np.eye(4), "#8b93a6", "chassis bottom"),
        ("chassis_top_rigid", np.eye(4), "#5b8fd4", "top plate"),
        ("top_hatch_rigid", np.eye(4), "#6fa8dc", "hatch"),
        ("coxa_link_ovh", Ts["coxa"], "#5f9e6e", "coxa OVH (outboard arm)"),
        ("servo_body", Ts["hip_cap"], "#c9a227", "hip servo"),
        ("hip_clamp_cap_ovh", Ts["hip_cap"], "#8fbf7f",
         "hip cap OVH + yaw pedestal"),
        ("bearing_6805", Ts["yaw_top"], "#303030", "top 6805"),
        ("chorn_clamp_cnc", T["femur"] @ MH, "#8e959e",
         "CNC C-clamp (hip, at the limit)"),
        ("femur_ovh_body", T["femur"] @ MH, "#7fb069", "femur body OVH"),
        ("servo_body", T["knee_cap"], "#c9a227", None),
        ("knee_clamp_cap_ovh", T["knee_cap"], "#9dc183", "knee cap OVH"),
        ("chorn_clamp_cnc", T["tibia"] @ MH, "#8e959e", None),
        ("tibia_ovh_socket", T["tibia"] @ MH, "#7fb069", "tibia socket OVH"),
        ("tibia_tube_ovh", T["tibia"], "#404040", "tibia tube"),
        ("foot_boot", T["tibia"] @ foot_frame, "#5a5f66", "foot"),
    ]
    fig, ax = plt.subplots(figsize=(10.5, 8.0), dpi=130)
    for key, M, color, label in sections:
        m = meshes[key].copy()
        m.apply_transform(Rz @ M)
        sec = m.section(plane_origin=[rv.APOTHEM, 0, 0],
                        plane_normal=[0, 1, 0])
        if sec is None:
            continue
        planar, _ = sec.to_2D(to_2D=rotation_matrix(-np.pi / 2.0, [1, 0, 0]))
        first = True
        for poly in planar.polygons_full:
            xs, ys = poly.exterior.xy
            ax.fill(xs, ys, color=color, alpha=0.65,
                    label=label if first else None)
            first = False if label else first
            for ring in poly.interiors:
                ax.fill(*ring.xy, color="white")
    ax.axhline(rv.SHEET_Z1, color="k", lw=0.5, ls=":")
    ax.annotate(f" plate deck z={rv.SHEET_Z1:.2f}",
                (rv.APOTHEM + 55, rv.SHEET_Z1), fontsize=7, va="bottom")
    ax.set_xlim(rv.APOTHEM - 120, rv.APOTHEM + 90)
    ax.set_ylim(-15, 245)
    ax.set_aspect("equal")
    ax.set_xlabel("radial position from body centre [mm]")
    ax.set_ylabel("world Z [mm]")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title(f"cnc_chorn_overhead -- leg 0 at the {limit:g} deg limit "
                 "(femur past vertical, over the plate)")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "preview.png"))
    print("  wrote preview.png")


def main() -> None:
    skip_sweep = "--skip-sweep" in sys.argv
    skip_brep = "--skip-brep" in sys.argv
    if not skip_brep:
        # rigid_hip's exports are prerequisites (shared chassis/stack)
        need = [k for k in ("chassis_top_rigid", "hip_clamp_cap_rigid",
                            "coxa_link_rigid", "chassis_bottom_rigid",
                            "top_hatch_rigid", "centre_wago_block")
                if not os.path.exists(os.path.join(
                    RIGID_DIR, "step", "stl", f"{k}.stl"))]
        if need:
            print(f"rigid_hip BREP exports missing ({need}) -- building ...")
            subprocess.run(RIGID_BREP_CMD, check=True)
        print("exporting BREP geometry (build_cnc_chorn_step.py) ...")
        subprocess.run(BREP_EXPORT_CMD, check=True)
    else:
        print("BREP EXPORT SKIPPED (--skip-brep): reusing step/stl/")
    meshes = build_meshes()

    print("rigid_hip inherited checks (shared chassis/stack; the coxa-"
          "specific ones exercise the INHERITED rigid coxa -- the variant "
          "coxa has its own suite below) ...")
    rv.check_static(meshes)
    rv.check_bottom_joint(meshes)
    rv.check_coxa_column(meshes)
    rv.check_chassis_variant(meshes)
    rv.check_rot_envelope(meshes)
    rv.check_wago_block(meshes)
    rv.check_hatch(meshes)
    rv.check_yaw_sweep(meshes)
    rv.check_plate_descent(meshes)

    print("cnc_chorn_overhead checks ...")
    check_parts(meshes)
    check_coxa_ovh(meshes)
    check_hip_cap_ovh(meshes)
    check_clamp_joint(meshes)
    check_web_joint(meshes)
    check_knee_cap(meshes)
    check_down_and_knee_range(meshes)
    check_assembly_paths_ovh(meshes)
    check_yaw_envelope_ovh(meshes)
    masses = report_masses(meshes)
    geometry = report_outboard_geometry(meshes)

    if skip_sweep:
        limit = -110.0
        print("SWEEP SKIPPED (--skip-sweep): using placeholder limit -110")
        sweep_info = {"skipped": True}
    else:
        print("overhead femur sweep (the slow part) ...")
        limit, ca, cb, pb = sweep_femur_envelope(meshes)
        sweep_info = {
            "limit_deg": limit,
            "contact_vs_own_stack": {"deg": ca[0], "part": ca[1]},
            "contact_vs_plate_by_yaw": {
                str(y): {"deg": cb[y], "part": pb[y]} for y in cb},
            "margin_deg": SWEEP_MARGIN, "step_deg": SWEEP_STEP,
        }

    print("writing scene.json ...")
    scene = build_scene(meshes, limit)
    with open(os.path.join(HERE, "scene.json"), "w", encoding="utf-8") as fh:
        json.dump(scene, fh, indent=1)
    with open(os.path.join(HERE, "sweep_report.json"), "w",
              encoding="utf-8") as fh:
        json.dump({"sweep": sweep_info, "masses": masses,
                   "outboard_geometry": geometry}, fh, indent=1)

    render_preview(meshes, limit)
    print("done.")


if __name__ == "__main__":
    main()
