#!/usr/bin/env python3
"""THE geometry source for the cnc_chorn_overhead variant's five parts.

STEP-FIRST: the CNC C-clamp and its four printed partners (knee block,
tibia socket, knee cap, OUTBOARD-arm coxa) are authored
HERE as build123d/OpenCascade BREP solids.  The .step files exported to
``step/`` are the CNC-shop quoting deliverable (chorn_clamp_cnc.step is
the one that goes out for machining quotes); the tessellated .stl twins
in ``step/stl/`` feed the assembly/check driver
``make_cnc_chorn_variant.py``, which runs the full geometric suite (web
joint, clamp spins, the overhead femur sweep) and builds the BuildViz
scene.  There is no trimesh twin of these builders -- edit geometry
here, nowhere else.

Constants are imported from ``make_cnc_chorn_variant.py`` (which derives
them from hexapod_prototype + the rigid-hip driver), so there are no
dimension forks.  The production knee block and servo clamp cap start
from the cad_step_test base sidecar's BREP ports
(``build_step_first_test._femur_knee_fixed_solid`` /
``make_servo_clamp_cap``) and receive this variant's boolean edits.

Frames: everything is authored in the femur/tibia LINK frame the
production femur_link is authored in (+x along the link, joint axis
along +z at x = SERVO_OUTPUT_X, +y = the up-swing / cap side), except
``knee_clamp_cap_ovh`` which stays in the production cap's well-local
frame (link x - FEMUR_LENGTH) so it drops into the same scene transform
as the production cap.

Run (build123d needs a 3.12 interpreter; make_cnc_chorn_variant.py runs
this for you by default):

  uv run --no-project --python 3.12 \
    --with build123d --with trimesh --with numpy --with manifold3d \
    python concepts/cnc_chorn_overhead/build_cnc_chorn_step.py
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from build123d import Pos, fillet

HERE = Path(__file__).resolve().parent            # concepts/cnc_chorn_overhead
PROTO_DIR = HERE.parent.parent                    # prototype_sts3215
CAD_STEP_DIR = PROTO_DIR / "cad_step_test"        # base BREP ports + export tail
RIGID_DIR = PROTO_DIR / "concepts" / "rigid_hip"  # rv (constants for cv import)
sys.path.insert(0, str(CAD_STEP_DIR))
sys.path.insert(0, str(RIGID_DIR))
sys.path.insert(0, str(HERE))

from step_common import StepPart, export_all, write_bundle  # noqa: E402

STEP_OUT_DIR = HERE / "step"

import math  # noqa: E402

import build_step_first_test as step  # noqa: E402
import hexapod_prototype as hp  # noqa: E402
import make_cnc_chorn_variant as cv  # noqa: E402
import make_rigid_hip_variant as rv  # noqa: E402  (coxa hub constants)


# --- span-style primitive adapters (same idiom as the rigid-hip builder) ----

_box = step._box


def _cyl_x(r: float, x0: float, x1: float,
           y: float = 0.0, z: float = 0.0) -> object:
    return step._cyl_x(r, x1 - x0, ((x0 + x1) / 2.0, y, z))


def _cyl_y(r: float, y0: float, y1: float,
           x: float = 0.0, z: float = 0.0) -> object:
    return step._cyl_y(r, y1 - y0, (x, (y0 + y1) / 2.0, z))


def _cyl_z(r: float, z0: float, z1: float,
           x: float = 0.0, y: float = 0.0) -> object:
    return step._cyl_z(r, z1 - z0, (x, y, (z0 + z1) / 2.0))


# --- shared cutters ----------------------------------------------------------

def _wedge_prism_link() -> object:
    """The overhead WEDGE half-space (link coords, z-uniform): removes
    everything ABOVE the line WEDGE_XY0 -> WEDGE_XY1 for x >= the wall
    plane.  One cutter, applied to BOTH the knee block and the cap (the
    cap gets it shifted into its well-local frame), so the mating parts
    stay complementary along the same plane."""
    x0, y0 = cv.WEDGE_XY0
    x1 = cv.WEDGE_XY1[0] + 8.0                    # 76 -- past the last point
    pts = [(x0, y0), (x1, cv.wedge_y(x1)), (x1, 80.0), (x0, 80.0)]
    return Pos(0.0, 0.0, 20.0) * step._xy_polygon_prism(pts, 90.0)


# --- chorn_clamp_cnc (CNC 6061-T6) ------------------------------------------

# PLAIN C: the blade plan is just a rectangle from the disc to the web
# (the outboard hip pivot cleared the plate; no droop/notch/corridor).
_BLADE_PTS = [
    (cv.AXIS_X, cv.BLADE_W_UP),
    (cv.WEB_X1, cv.BLADE_W_UP),
    (cv.WEB_X1, cv.BLADE_Y_BOT),
    (cv.AXIS_X, cv.BLADE_Y_BOT),
]
# plan corner that gets the R3 (external) fillet -- the trailing corner
# under the disc end (the leading corner at (AXIS_X, +13) is tangent to
# the R13 disc round; the web-end corners weld into the web block)
_BLADE_FILLET_XY = [(cv.AXIS_X, cv.BLADE_Y_BOT)]


def _blade(z0: float) -> object:
    """One clamp blade: plain rectangular plan + R13 rounded end over
    the disc, extruded ARM_T, plan fillet on the vertical edge."""
    poly = step._xy_polygon_prism(_BLADE_PTS, cv.ARM_T)
    disc = step._cyl_z(cv.BLADE_END_R, cv.ARM_T, (cv.AXIS_X, 0.0, 0.0))
    blade = poly + disc
    edges = []
    for e in blade.edges():
        p0, p1 = e.position_at(0.0), e.position_at(1.0)
        if abs(p0.X - p1.X) > 0.01 or abs(p0.Y - p1.Y) > 0.01:
            continue                              # not a vertical edge
        if any(abs(p0.X - tx) < 0.05 and abs(p0.Y - ty) < 0.05
               for tx, ty in _BLADE_FILLET_XY):
            edges.append(e)
    assert len(edges) == len(_BLADE_FILLET_XY), \
        f"blade plan fillet edge pick found {len(edges)}"
    blade = fillet(edges, cv.PLAN_FILLET_R)
    return Pos(0.0, 0.0, z0 + cv.ARM_T / 2.0) * blade


def _gusset(z_face: float, sign: int) -> object:
    """R2.5 quarter-round gusset along the internal web/blade corner (the
    3-axis slot tool's corner radius, modeled so the DFM is explicit).
    Plain C: runs nearly the whole junction (cv.GUSSET_Y)."""
    r = cv.GUSSET_R
    y0, y1 = cv.GUSSET_Y
    box = _box((r, y1 - y0, r),
               (cv.WEB_X0 - r / 2.0, (y0 + y1) / 2.0, z_face + sign * r / 2.0))
    rod = _cyl_y(r, y0 - 0.5, y1 + 0.5, x=cv.WEB_X0 - r, z=z_face + sign * r)
    return box - rod


def make_chorn_clamp_cnc() -> object:
    """The CNC 6061-T6 C-clamp, PLAIN C: two rectangular blades with
    integral Phi 19 ring bosses that land ON the nominal disc faces (no
    PETG-era interference -- see the press-fit warning in the driver),
    a full-height 3.8 mm web whose OUTER face is DATUM A (the
    production femur wall plane), 2x M3-6H tapped web holes + 2x M3 csk
    through-holes, and the same disc bolt pattern the production yokes
    use.  No plate-dodging features: the outboard hip pivot
    (cv.COXA_EXT) is what clears the plate."""
    web_pts = [(cv.WEB_X0, cv.WEB_Y_BOT), (cv.WEB_X1, cv.WEB_Y_BOT),
               (cv.WEB_X1, cv.BLADE_W_UP), (cv.WEB_X0, cv.BLADE_W_UP)]
    web = Pos(0.0, 0.0, (cv.ARM_BOT_Z0 + cv.ARM_TOP_Z1) / 2.0) \
        * step._xy_polygon_prism(web_pts, cv.ARM_TOP_Z1 - cv.ARM_BOT_Z0)
    body = step._union(
        _blade(cv.ARM_BOT_Z0),
        _blade(cv.ARM_TOP_Z0),
        _cyl_z(cv.PAD_OD / 2.0, cv.ARM_BOT_Z1, cv.DISC_BOT_FACE_Z,
               x=cv.AXIS_X),
        _cyl_z(cv.PAD_OD / 2.0, cv.DISC_TOP_FACE_Z, cv.ARM_TOP_Z0,
               x=cv.AXIS_X),
        web,
        _gusset(cv.ARM_BOT_Z1, +1),
        _gusset(cv.ARM_TOP_Z0, -1),
    )
    cuts = []
    # disc bolt pattern + centre bore, straight through both blade/pad
    # stacks (M3 screws thread into the metal disc horns, as production)
    for hx, hy in step._disc_horn_bolt_centres():
        cuts.append(_cyl_z(hp.DISC_HORN_BOLT_OD / 2.0,
                           cv.ARM_BOT_Z0 - 1.0, cv.ARM_TOP_Z1 + 1.0,
                           x=hx, y=hy))
    cuts.append(_cyl_z(hp.HORN_CENTRE_OD / 2.0,
                       cv.ARM_BOT_Z0 - 1.0, cv.ARM_TOP_Z1 + 1.0,
                       x=cv.AXIS_X))
    # collar recesses at both pad faces (production yoke policy)
    rr = hp.DISC_HORN_COLLAR_OD / 2.0 + 0.25
    rd = hp.DISC_HORN_COLLAR_DEPTH + 1.0
    cuts.append(_cyl_z(rr, cv.DISC_TOP_FACE_Z - 0.5,
                       cv.DISC_TOP_FACE_Z + rd, x=cv.AXIS_X))
    cuts.append(_cyl_z(rr, cv.DISC_BOT_FACE_Z - rd,
                       cv.DISC_BOT_FACE_Z + 0.5, x=cv.AXIS_X))
    # web A holes: tapped M3-6H (modeled Phi 3.0; drill 2.5, tap = shop)
    for y, z in cv.WEB_A_YZ:
        cuts.append(_cyl_x(cv.WEB_TAP_D / 2.0,
                           cv.WEB_X0 - 1.0, cv.WEB_X1 + 1.0, y=y, z=z))
    # web B holes: Phi 3.4 clearance + 90-deg csk sunk in the INNER face
    for y, z in cv.WEB_B_YZ:
        cuts.append(_cyl_x(cv.WEB_B_D / 2.0,
                           cv.WEB_X0 - 1.0, cv.WEB_X1 + 1.0, y=y, z=z))
        r_csk = cv.WEB_CSK_D / 2.0 + 0.1
        cuts.append(step._cone_x_from_base(
            r_csk, r_csk, base_x=cv.WEB_X0 - 0.1, y=y, z=z, direction=1))
    return step._diff(body, *cuts)


# --- femur_ovh_body (PETG) ---------------------------------------------------

CAV_FACE = hp.FEMUR_LENGTH - hp.SERVO_BODY_W / 2.0        # 67.3 servo end face
A_CBORE_X0 = CAV_FACE - cv.WEB_CBORE_DEPTH                # 64.1 head seat
WALL_X0 = hp.FEMUR_LENGTH - hp.WELL_W / 2.0               # 58.3 = DATUM A mate


def _web_screw_cuts(cav_face: float, a_cbore_x0: float,
                    slot_y0: float) -> list[object]:
    """The printed side of the web joint, shared by the femur body and
    the tibia socket: A shanks + head cbores, B shanks + nyloc T-slots.
    ``slot_y0`` = below the part's own underside face (the slot's
    insertion opening)."""
    cuts = []
    for y, z in cv.WEB_A_YZ:
        cuts.append(_cyl_x(cv.WEB_SHANK_D / 2.0, cv.WEB_X1 - 1.0,
                           a_cbore_x0 + 0.5, y=y, z=z))
        cuts.append(_cyl_x(cv.WEB_CBORE_D / 2.0, a_cbore_x0,
                           cav_face + 1.0, y=y, z=z))
    for y, z in cv.WEB_B_YZ:
        cuts.append(_cyl_x(cv.WEB_B_D / 2.0, cv.WEB_X1 - 1.0,
                           cv.NUT_SLOT_X1 + 1.4, y=y, z=z))
        cuts.append(_box((cv.NUT_SLOT_X1 - cv.NUT_SLOT_X0,
                          -13.2 - slot_y0, cv.NYLOC_AF + 0.2),
                         ((cv.NUT_SLOT_X0 + cv.NUT_SLOT_X1) / 2.0,
                          (slot_y0 - 13.2) / 2.0, z)))
    return cuts


def make_femur_ovh_body() -> object:
    """The production knee block (servo cradle + 688 housing + far-wall
    pad + rear tab, via the base port's _femur_knee_fixed_solid) with the
    variant edits: overhead wedge chamfer, the 4 web-screw paths on the
    hip-side wall (whose OUTER face mates the clamp web at DATUM A), the
    knee-cap up-screw bore, and the pocket for the variant cap's insert
    boss + riser."""
    base = Pos(hp.FEMUR_LENGTH, 0.0, 0.0) * step._femur_knee_fixed_solid()
    cuts = [_wedge_prism_link()]
    cuts += _web_screw_cuts(CAV_FACE, A_CBORE_X0,
                            slot_y0=-(hp.WELL_D / 2.0) - 1.6)      # -18.5
    # knee-cap up-screw: Phi 3.4 from the cradle underside to the boss
    # pocket floor, SHCS head cbore opening at the underside face
    cuts.append(_cyl_y(cv.UPSCREW_D / 2.0, cv.BLOCK_Y_BOT - 1.1,
                       cv.CAP_BOSS_Y0 - 0.2, x=cv.UPSCREW_X,
                       z=cv.UPSCREW_Z))
    cuts.append(_cyl_y(cv.UPSCREW_CB_D / 2.0, cv.BLOCK_Y_BOT - 1.1,
                       cv.BLOCK_Y_BOT + cv.UPSCREW_CB_DEPTH,
                       x=cv.UPSCREW_X, z=cv.UPSCREW_Z))
    # cap boss/riser pocket: open from the boss floor straight up past
    # the wedge face so the cap drops in/out vertically
    cl = cv.POCKET_CL
    cuts.append(_box((CAV_FACE + 0.1 - (cv.CAP_BOSS_X0 - cl),
                      25.0 - (cv.CAP_BOSS_Y0 - cl),
                      (cv.CAP_BOSS_Z1 + cl) - (cv.CAP_BOSS_Z0 - cl)),
                     (((cv.CAP_BOSS_X0 - cl) + CAV_FACE + 0.1) / 2.0,
                      ((cv.CAP_BOSS_Y0 - cl) + 25.0) / 2.0,
                      cv.UPSCREW_Z)))
    return step._diff(base, *cuts)


# --- knee_clamp_cap_ovh (PETG, well-local frame) -----------------------------

def make_knee_clamp_cap_ovh() -> object:
    """Production servo clamp cap minus the overhead wedge (tail + the
    inboard side-screw boss die), plus the replacement retention: a
    hang-boss under the tongue line carrying a Phi 4 x 6 heat-set insert
    for the M3x25 up-screw, bridged to the tongue by a riser (inboard of
    the servo end face) and a strap riding 0.3 over the servo top."""
    L = hp.FEMUR_LENGTH
    cap = step.make_servo_clamp_cap()
    boss = _box((cv.CAP_BOSS_X1 - cv.CAP_BOSS_X0,
                 cv.CAP_BOSS_Y1 - cv.CAP_BOSS_Y0,
                 cv.CAP_BOSS_Z1 - cv.CAP_BOSS_Z0),
                ((cv.CAP_BOSS_X0 + cv.CAP_BOSS_X1) / 2.0 - L,
                 (cv.CAP_BOSS_Y0 + cv.CAP_BOSS_Y1) / 2.0,
                 (cv.CAP_BOSS_Z0 + cv.CAP_BOSS_Z1) / 2.0))
    riser = _box((cv.CAP_BOSS_X1 - cv.CAP_RISER_X0,
                  (cv.CAP_STRAP_Y0 + 0.8) - cv.CAP_BOSS_Y0,
                  cv.CAP_BOSS_Z1 - cv.CAP_BOSS_Z0),
                 ((cv.CAP_RISER_X0 + cv.CAP_BOSS_X1) / 2.0 - L,
                  (cv.CAP_BOSS_Y0 + cv.CAP_STRAP_Y0 + 0.8) / 2.0,
                  (cv.CAP_BOSS_Z0 + cv.CAP_BOSS_Z1) / 2.0))
    strap = _box((cv.CAP_STRAP_X1 - cv.CAP_RISER_X0,
                  cv.CAP_STRAP_Y1 - cv.CAP_STRAP_Y0,
                  cv.CAP_BOSS_Z1 - cv.CAP_BOSS_Z0),
                 ((cv.CAP_RISER_X0 + cv.CAP_STRAP_X1) / 2.0 - L,
                  (cv.CAP_STRAP_Y0 + cv.CAP_STRAP_Y1) / 2.0,
                  (cv.CAP_BOSS_Z0 + cv.CAP_BOSS_Z1) / 2.0))
    body = step._union(cap, boss, riser, strap)
    cuts = [
        Pos(-L, 0.0, 0.0) * _wedge_prism_link(),
        # heat-set insert bore + melt relief, up from the boss underside
        _cyl_y(cv.CAP_INSERT_D / 2.0, cv.CAP_BOSS_Y0 - 0.2,
               cv.CAP_BOSS_Y0 + cv.CAP_INSERT_LEN,
               x=cv.UPSCREW_X - L, z=cv.UPSCREW_Z),
        _cyl_y(cv.CAP_INSERT_RELIEF_D / 2.0, cv.CAP_BOSS_Y0 - 0.2,
               cv.CAP_BOSS_Y0 + cv.CAP_INSERT_RELIEF_DEPTH,
               x=cv.UPSCREW_X - L, z=cv.UPSCREW_Z),
    ]
    return step._diff(body, *cuts)


# --- tibia_ovh_socket (PETG) -------------------------------------------------

def make_tibia_ovh_socket() -> object:
    """Flange + CF-tube socket that bolts to the tibia-side clamp web
    with the same 4-screw pattern as the femur body.  The tube mouth is
    pulled outboard so the (16.6 mm shorter) tube tip lands 0.3 off the
    metal web."""
    flange = _box((cv.TIB_FLANGE_X1 - cv.TIB_FLANGE_X0,
                   cv.TIB_FLANGE_Y1 - cv.TIB_FLANGE_Y0,
                   cv.TIB_FLANGE_Z1 - cv.TIB_FLANGE_Z0),
                  ((cv.TIB_FLANGE_X0 + cv.TIB_FLANGE_X1) / 2.0,
                   (cv.TIB_FLANGE_Y0 + cv.TIB_FLANGE_Y1) / 2.0,
                   (cv.TIB_FLANGE_Z0 + cv.TIB_FLANGE_Z1) / 2.0))
    boss, bore, _pin = step._leg_tube_socket_x(
        cv.TIB_BORE_X0, hp.JOINT_SOCKET_Z, direction=1)
    body = flange + boss
    cuts = [bore]
    cuts += _web_screw_cuts(cv.TIB_FLANGE_X1,
                            cv.TIB_FLANGE_X1 - cv.WEB_CBORE_DEPTH,
                            slot_y0=cv.TIB_FLANGE_Y0 - 0.5)       # -20.5
    return step._diff(body, *cuts)


# --- hip_clamp_cap_ovh (PETG, well-local frame) ------------------------------

def make_hip_clamp_cap_ovh() -> object:
    """Production servo clamp cap for the OUTBOARD hip station, with the
    rigid-hip yaw-axis furniture pulled back to where the yaw axis now
    is: pedestal + 6805 inner-race press boss + puller notches sit at
    cap-local x = -COXA_EXT, tied to the cap crown by a two-step arm
    (constants + swept-band reasoning in the driver's HIPCAP_ARM_*
    block).  Same removable-part logic as rigid_hip: the pedestal drops
    over the trimmed hub top AFTER the coxa horn screws are installed,
    so the yaw drive stays serviceable."""
    X = -cv.COXA_EXT
    cap = step.make_servo_clamp_cap()
    ped = _cyl_y(rv.PED_OD / 2.0, rv.PED_Y0, rv.PED_Y1, x=X, z=rv.AXIS_Z)
    boss = _cyl_y(rv.BOSS_OD / 2.0, rv.PED_Y1 - 1.0, rv.BOSS_Y1,
                  x=X, z=rv.AXIS_Z)
    tip = _cyl_y(rv.BOSS_OD / 2.0 - rv.BOSS_TIP_STEP, rv.BOSS_Y1 - 0.1,
                 rv.TIP_Y1, x=X, z=rv.AXIS_Z)
    arms = []
    for x0, x1, y0, y1 in (cv.HIPCAP_ARM_LO, cv.HIPCAP_ARM_HI):
        z0, z1 = cv.HIPCAP_ARM_Z
        arms.append(_box((x1 - x0, y1 - y0, z1 - z0),
                         ((x0 + x1) / 2.0, (y0 + y1) / 2.0,
                          (z0 + z1) / 2.0)))
    body = step._union(cap, ped, boss, tip, *arms)
    notches = []
    for sx in (+1.0, -1.0):
        ext_x = rv.PED_OD / 2.0 - rv.PULLER_NOTCH_R0 + 3.0
        notches.append(_box(
            (ext_x, rv.PULLER_NOTCH_DEPTH + 0.05, rv.PULLER_NOTCH_W),
            (X + sx * (rv.PULLER_NOTCH_R0 + ext_x / 2.0),
             rv.PED_Y1 - rv.PULLER_NOTCH_DEPTH / 2.0 + 0.025,
             rv.AXIS_Z),
        ))
    return step._diff(body, *notches)


# --- coxa_link_ovh (PETG) ----------------------------------------------------

def make_coxa_link_ovh() -> object:
    """The rigid-hip coxa (build_rigid_hip_step.make_coxa_link_rigid)
    with the whole cradle arm -- hip bracket + foot slab -- translated
    cv.COXA_EXT outboard along +x, which puts the hip pivot beyond the
    top plate's rim (THE legs-over-head move).  The slab already spans
    from ~x -35 around the cradle, so after the shift it still overlaps
    the hub column plan by ~20 mm and unions into one rigid body -- no
    separate bridge member.  Yaw-axis interfaces (horn drive cuts, hub
    boss, seat ring, dust brim, the shortened-hub trim) are copied
    verbatim from the rigid builder; the Phi 33.5 hip-axis clearance
    wells and the rotation-envelope trim follow the moved anchor (the
    trim keeps its 38.2 mm circle INBOARD only -- outboard the arm is
    meant to exceed it, see check_yaw_envelope_ovh in the driver)."""
    hub = step.make_coxa_yaw_hub(one_piece=True)
    skirt_cut = step._diff(
        _cyl_z(hp.YAW_HUB_DUST_LIP_OD / 2.0 + 2.0, rv.SLAB_BOT_Z - 1.5,
               hp.YAW_HUB_BOSS_TOP_Z + 1.0),
        _cyl_z(20.0, rv.SLAB_BOT_Z - 2.5, hp.YAW_HUB_BOSS_TOP_Z + 2.0),
    )
    hub = step._diff(hub, skirt_cut)
    bb = hub.bounding_box()
    hub = step._intersect(hub, _cyl_z(60.0, bb.min.Z - 1.0, rv.HUB_TRIM_Z))
    bracket = (Pos(cv.COXA_EXT, 0.0, -rv.COL_DROP)
               * step.make_coxa_hip_bracket(one_piece=True))
    ring = step._diff(
        _cyl_z(rv.HUB_RING_OD / 2.0, rv.HUB_RING_Z0, rv.HUB_RING_Z1),
        _cyl_z(rv.HUB_RING_ID / 2.0, rv.HUB_RING_Z0 - 1.0,
               rv.HUB_RING_Z1 + 1.0),
    )
    brim = step._diff(
        _cyl_z(rv.BRIM_OD / 2.0, rv.BRIM_BOT_Z, rv.BRIM_TOP_Z),
        _cyl_z(rv.HUB_RING_ID / 2.0, rv.BRIM_BOT_Z - 1.0,
               rv.BRIM_TOP_Z + 1.0),
    )
    body = step._union(hub, bracket, ring, brim)
    hip_ax_x = rv.COXA_HIP_ANCHOR_V[0] + cv.COXA_EXT
    hip_ax_z = rv.COXA_HIP_ANCHOR_V[2]
    cuts = [_cyl_y(16.75, ylo, yhi, x=hip_ax_x, z=hip_ax_z)
            for (ylo, yhi) in ((21.75, 30.0), (-31.0, -24.75))]
    shaft_top_z = 80.0
    drive_clear = hp.DISC_HORN_BOLT_OD + 0.3
    stations = [(0.0, 0.0, rv.HORN_CENTRE_SEAT_Z,
                 hp.HORN_CENTRE_OD)]
    r = hp.DISC_HORN_BOLT_PCD / 2.0
    stations.extend(
        (r * math.cos(t), r * math.sin(t), rv.HORN_HEAD_SEAT_Z,
         drive_clear)
        for t in hp.DISC_HORN_BOLT_ANGLES_RAD)
    for sx, sy, seat_z, shank_d in stations:
        cuts.append(_cyl_z(hp.YAW_HUB_HORN_HEAD_CB_OD / 2.0, seat_z,
                           shaft_top_z, sx, sy))
        cuts.append(_cyl_z(shank_d / 2.0, rv.SLAB_BOT_Z - 1.0,
                           seat_z + 0.5, sx, sy))
    body = step._diff(body, *cuts)
    bb = body.bounding_box()
    keep = step._union(
        _cyl_z(rv.ROT_ENVELOPE_R, bb.min.Z - 1.0, bb.max.Z + 1.0),
        _box((150.0, 96.0, bb.max.Z - bb.min.Z + 2.0),
             (4.0 + 75.0, 0.0, (bb.min.Z + bb.max.Z) / 2.0)),
    )
    return step._intersect(body, keep)


# --- specs, checks, main -----------------------------------------------------

def cnc_chorn_part_specs() -> list[StepPart]:
    return [
        StepPart(
            "chorn_clamp_cnc",
            make_chorn_clamp_cnc,
            None,
            "CNC 6061-T6 C-clamp (12x): grown joint clamp, integral disc "
            "bosses, tapped M3 web -- THE machining quote part.",
            printable=False,
        ),
        StepPart(
            "femur_ovh_body",
            make_femur_ovh_body,
            None,
            "Production knee block + overhead wedge + web joint bores + "
            "cap up-screw + boss pocket (6x, PETG).",
        ),
        StepPart(
            "tibia_ovh_socket",
            make_tibia_ovh_socket,
            None,
            "Tibia web flange + shortened CF-tube socket (6x, PETG).",
        ),
        StepPart(
            "knee_clamp_cap_ovh",
            make_knee_clamp_cap_ovh,
            None,
            "Production knee cap minus the overhead wedge tail, plus the "
            "insert-boss up-screw retention (6x, PETG).",
        ),
        StepPart(
            "coxa_link_ovh",
            make_coxa_link_ovh,
            None,
            "Rigid-hip coxa with the cradle arm moved COXA_EXT=42 mm "
            "outboard -- the legs-over-head hip pivot (6x, PETG).",
        ),
        StepPart(
            "hip_clamp_cap_ovh",
            make_hip_clamp_cap_ovh,
            None,
            "Production hip cap + yaw-axis pedestal/press-boss reached "
            "back over a two-step arm (6x, PETG).",
        ),
    ]


def _sanity_problems(rows: list[dict]) -> list[str]:
    """Part-level gates that don't need the assembled robot: every
    derived STL must HEAL into a closed volume (the same
    hp._heal_for_export pass the driver applies).  The full geometric
    suite runs in make_cnc_chorn_variant.py."""
    import trimesh

    problems = []
    for row in rows:
        raw = trimesh.load(STEP_OUT_DIR / row["stl"], process=True)
        mesh = hp._heal_for_export(raw)
        if not mesh.is_volume:
            problems.append(
                f"{row['name']}: derived STL does not heal into a closed "
                "volume (genuine tessellation crack)")
    return problems


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()

    exported = export_all(cnc_chorn_part_specs(), out_dir=STEP_OUT_DIR,
                          step_dir=STEP_OUT_DIR,
                          stl_dir=STEP_OUT_DIR / "stl")
    problems = _sanity_problems(exported)
    manifest = {
        "units": "mm",
        "source": (
            "build123d/OpenCascade BREP (CANONICAL geometry source), "
            "constants imported from "
            "concepts/cnc_chorn_overhead/make_cnc_chorn_variant.py"
        ),
        "exported_parts": exported,
        "checks": {
            "passed": not problems,
            "problems": problems,
        },
        "files": [rel for row in exported for rel in (row["step"], row["stl"])],
    }
    manifest_path = STEP_OUT_DIR / "cnc_chorn_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    bundle = write_bundle(manifest, "cnc_chorn_step_first_bundle.zip",
                          "cnc_chorn_manifest.json", out_dir=STEP_OUT_DIR)
    print(f"wrote {manifest_path.relative_to(PROTO_DIR)}")
    print(f"wrote {bundle.relative_to(PROTO_DIR)}")
    if problems:
        print("cnc-chorn STEP part checks failed:")
        for problem in problems:
            print(f"  - {problem}")
        raise SystemExit(1)
    print("cnc-chorn BREP export complete; run make_cnc_chorn_variant.py "
          "for the assembly checks + scene.")


if __name__ == "__main__":
    main()
