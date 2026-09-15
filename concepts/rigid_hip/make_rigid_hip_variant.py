"""VARIANT: rigid-hip yaw axis -- a THIRD 6805 bearing above each hip,
carried by an extended hip clamp cap, seated in a full-size top chassis.

Concept (user, Aug 2026): the production yaw joint carries the coxa on a
6805-2RS pair in chassis_bottom's tower, but the hip servo above it is
still a cantilever -- leg loads twist the coxa link about the bearing
pair.  This variant closes the loop from the TOP:

  * ``hip_clamp_cap_rigid`` -- the stock ``make_servo_clamp_cap()`` (the
    hip clamshell lid) grown UP along the yaw axis: a Phi 29 pedestal
    (inner-race seat shoulder, same role as the yaw hub's uflange) and a
    Phi 25.15 press boss (= ``YAW_HUB_BOSS_OD``, the bench-tuned +0.15
    interference the lower hub boss already uses) that a third 6805-2RS
    presses onto.  Knee caps stay stock.
  * ``chassis_top_rigid`` -- a SECOND 200 mm flat-to-flat hex plate
    (same footprint as chassis_bottom's sheet, 50% thicker at 6 mm
    since the pillar delete left it alone in the load path) whose six
    Phi 44 bosses pocket the bearings' outer races at Phi 37.15
    (= ``YAW_TOWER_BORE_OD``, the bench-tuned firm finger-press fit),
    race retained by the Phi 34 shoulder (= ``YAW_TOWER_SHOULDER_OD``).
    Six Phi 7 DRIVER ACCESS holes sit above the inboard cap bolts (legs
    at yaw 0): the cap is a CAPTIVE BEARING CARRIER -- its 6805 is
    pressed on once and never removed; all service unbolts the cap from
    the coxa cradle (both bolts reachable with the plate on) and lifts
    the plate + caps + bearings off as one rigid unit.
  * NO CORNER PILLARS (user, Aug 26 pm: "I actually think its sturdy
    enough without the columns") -- the Aug-23 rim columns are
    deleted.  The plate-to-bottom tie is the bearing sandwich itself:
    each hip moment closes through its own two 6805s, and the plate
    couples to chassis_bottom through the other five legs' towers
    (plus the four central 90 mm standoffs, which anchor the hatch /
    electronics).  The hatch perimeter screws that used to thread
    into the pillar tops now thread into M3 BRASS HEAT-SET INSERTS in
    six FRAME_BOSS bosses hanging under the frame sheet (user, Aug 26
    am: frequent removal must not cycle printed threads).
  * ``coxa_link_rigid`` (x6) -- the production coxa with four variant
    edits: the vertical hub column SHORTENED by COL_DROP (the Phi 52.4
    platform disc, dust-lip skirt and uflange -- all sized around the
    deleted production cap -- are removed, the horn screws swap
    M3x30 -> M3x20 (same thread engagement, seats 10 mm deeper), and
    the slab + cradle unit drops so the well floor sits just over the
    screw heads; see the COXA SHORTEN constants), servo-cradle corners
    rounded to the 38.2 mm yaw envelope (max 2.16 mm off two vertical
    wall corners), the hub's existing Phi 25.15 boss pressing the
    relocated bottom race 0.5 mm over the deck (user, Aug 25: "why
    cant the bearing come down even more and be right on top of the
    servo?" -- see BEARING COUNT), and a small Phi 38 DUST BRIM
    hovering 0.5 above the race-top / tower-rim plane -- non-contact,
    stepped 3 mm INSIDE the tower Phi 44 so the coxa reads as coxa,
    not as more chassis column (see the BRIM_* constants).  Horn
    interface, cradle pilots and cap seat untouched.
  * ``chassis_bottom_rigid`` (x1) -- the production chassis with the
    six tower platforms trimmed to the tower's own Phi 44 cylinder
    (user, Aug 24: corners rounded; rev 2: all corner curves must
    match -- so the tower curve is now the ONLY curve), each tower
    band REBUILT LOW so the single bearing sits on a deck-level seat
    ledge and is FULLY housed (full 7 mm outer-race wrap; the Phi 44
    column ends exactly at the bearing top, now world 17.75 -- user,
    Aug 25: bearing right on top of the servo, race bottom 0.5 mm
    over the case), the dead cap-bolt ear lugs shaved (outboard +
    tangential + the inboard one, whose below-deck root now goes with
    the rev-6 shell flatten), the six corner Wago TRAY WALL SETS DELETED
    (user, Aug 24 -- the splices live in centre_wago_block now, so
    the trays are dead geometry), and the wago-era WIRE-CORRIDOR
    APPARATUS plus the whole CRADLE-SHELL run outboard of it (side
    walls + deck-skin roof, the "two gray things" at each corner
    flat) FLATTENED to the sheet up to the tower keep
    cylinder (user, Aug 24 rev 5 -- "that CRADLE WALL isnt doing
    shit, just flatten it out" -- and Aug 25 rev 6; see the
    CHB_FLAT_* constants).  See the CHB_* constant block.  Every
    remaining functional surface (pocket, seat, well collar, slots)
    is production geometry.
  * ``centre_wago_block`` -- the corner Wagos are gone (their trays
    are deleted and with the frame on they had no lever access), so
    the power tree consolidates: 4x 5-port 221-415 (two per
    net, jumpered) in one printed press-fit block at the chassis
    centre, under the open hatch, replacing the 6 corner + 2 trunk
    nuts.

  Load path: hip moment -> cap boss -> top bearing -> top plate ->
  the five other legs' bearing towers -> chassis_bottom.  Each yaw
  axis becomes simply-supported (one bearing below, one above, ~65 mm
  mid-plane to mid-plane) instead of cantilevered.

  BEARING COUNT (user, Aug 24: "we really only need one bearing on
  the bottom -- simplify"): ONE bottom bearing per leg, seated
  directly in chassis_bottom's own open-top Phi 37.15 pocket (the
  production LOWER-race position: drop-in path and z=0.5 seat are
  print-proven).  The bolt-on ``yaw_bearing_cap`` is DELETED -- it
  only existed to house the second race and retain the pair, and the
  top plate now does both jobs: standing loads go UP through the top
  bearing into the plate shoulder, hanging loads go DOWN through the
  hub's new Phi 29 seat ring -> inner race -> outer race -> tower
  seat.  -6 printed caps, -18 M3x8 cap screws, and the leg + bearing
  lift straight out once the plate is off (horn centre screw only).
  The tower rim is RAISED to the race-top plane (full 7 mm wrap), so
  nothing stands proud and the coxa sits directly on the bearing
  (user, Aug 24 -- see the TOWER RIM / BRIM constants).  Net bearings
  per robot: 12, same as production.

TRADE-OFF (measured by the sweep in this script): the full-size top
plate caps the femur's UP-swing.  The production workspace envelope is
femur in [-80, +30] deg; the plate + its bearing bosses cut the up
limit to roughly -55 deg (exact per-yaw contact angles printed at
build time and baked into the scene's joint limits).  Walking gaits
around STANCE_FEMUR_DEG = -25 are unaffected; deep leg tucks (e.g.
stand-up curls) must be re-checked against the new envelope before
this variant goes on the robot.

NOT a production change: nothing in the verified parts registry moves.
``hexapod_prototype.py`` is a read-only input; the printed parts unique
to this variant live in THIS directory's ``stl/`` (never in
``stl_prototype/``).  BuildViz build id: ``prototype_sts3215/rigid-hip``
(a sibling build in the STS hub project; the standalone
``sts3215-rigid-hip`` id was retired 2026-08-27).

STEP-FIRST PIPELINE (user, Aug 2026: "the official way is to make step
files"): the six variant printables are authored as build123d /
OpenCascade BREP solids in ``build_rigid_hip_step.py`` (same dir), which
exports .step + tessellated .stl into ``step/``.  THIS script is the
assembly/check driver: it runs that exporter, loads its tessellations,
builds the production/visual meshes, runs the full geometric check
suite, copies the printables into the print set (``stl/``) and writes
the BuildViz scene.  Edit printable geometry in build_rigid_hip_step.py
-- the trimesh builders that used to live here are retired.

Run:  uv run python concepts/rigid_hip/make_rigid_hip_variant.py
      (--skip-sweep for fast geometry-only iterations;
       --skip-brep to reuse the existing step/stl/ exports)
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
STL_DIR = os.path.join(HERE, "stl")

import hexapod_prototype as hp  # noqa: E402  (read-only input)

# ---------------------------------------------------------------------------
# Derived constants -- every fit re-uses a bench-tuned production constant.
# ---------------------------------------------------------------------------
BEARING_ID = hp.YAW_BEARING_ID            # 25 (6805-2RS)
BEARING_OD = hp.YAW_BEARING_OD            # 37
BEARING_W = hp.YAW_BEARING_W              # 7
BEARING_INNER_OD = hp.YAW_BEARING_INNER_OD  # 29 -- inner-race outer diameter

BOSS_OD = hp.YAW_HUB_BOSS_OD              # 25.15 -- inner-race press (+0.15)
POCKET_BORE = hp.YAW_TOWER_BORE_OD        # 37.15 -- outer-race press bore
SHOULDER_OD = hp.YAW_TOWER_SHOULDER_OD    # 34   -- race retaining lip
RING_OD = BEARING_OD + 2.0 * hp.YAW_TOWER_WALL  # 44 -- same wall as the tower

PED_OD = BEARING_INNER_OD                 # 29 -- pedestal = inner-race seat
PED_H = 5.5                               # pedestal height above the cap face
PULLER_NOTCH_W = 8.0                      # two pry slots under the inner race
PULLER_NOTCH_DEPTH = 2.2                  # slot depth below the race seat
PULLER_NOTCH_R0 = 13.0                    # slot inner face: exposes the race
                                          # underside r 13.0..14.5, stays
                                          # 0.4 mm clear of the Phi 25.15 boss
RING_BOT_CL = 0.5                         # ring bottom vs race bottom (race
                                          # protrudes -> shoulder seats first)
BOSS_TIP_STEP = 0.575                     # stepped lead-in: tip at Phi 24.0
BOSS_TIP_H = 0.8
POCKET_LEADIN = 0.8                       # pocket-mouth lead-in (per side)

# DUST ROOF over each top bearing (user, Aug 26): the Phi 34 shoulder
# opening used to run clean through the sheet, leaving the bearing top
# open to the sky.  Nothing ever needed that opening (the boss lead-in
# tip stops exactly at the sheet BOTTOM plane, the driver holes are
# separate), so the plate now keeps a thin integral roof at the deck
# face -- the whole ring top is one flush surface ("all go up to the
# same height") and dirt cannot reach the bearing from above.  The
# roof is thinner than the sheet so the rotating parts keep an air gap
# under it: tip top 74.05, roof underside 77.65 -> 3.6 mm running
# clearance (asserted >= BR_ROOF_MIN_CL in check_yaw_sweep).
BR_ROOF_T = 2.4                           # roof thickness, scaled with the
                                          # 6 mm sheet (was 1.6 on the 4 mm
                                          # sheet; prints flat on the bed:
                                          # the plate prints deck face down,
                                          # so the pocket is a true blind
                                          # bore, no bridging)
BR_ROOF_MIN_CL = 0.5                      # min rotating-vs-roof air gap

PLATE_T = hp.CHASSIS_PLATE_T              # 4 -- bottom sheet + hatch lid
# TOP PLATE 50% THICKER (user, Aug 26 pm, after the pillar delete: "make
# it say 50% thicker now that the pillars are gone").  With no rim
# pillars the top plate alone carries the hip-moment couple between the
# six bearing towers, so the sheet goes 4 -> 6 mm.  The sheet UNDERSIDE
# is pinned to the race-top/shoulder plane (74.05, that is where the
# pockets seat), so the extra 2 mm grows UPWARD: deck face 78.05 ->
# 80.05.  The hatch LID stays at PLATE_T = 4 (it is not in the bearing
# load path; keeping it thin keeps the lid light and the screws short).
TOP_PLATE_T = 1.5 * hp.CHASSIS_PLATE_T    # 6 -- top frame sheet only
CENTRE_HOLE_D = 40.0                      # wiring / standoff-wrench access
HOLE_D = hp.BRACKET_BOLT_HOLE             # 3.4 -- M3 clearance
ACCESS_HOLE_D = 7.0                       # driver pass-through above the
                                          # INBOARD cap bolt (at yaw 0): the
                                          # cap unbolts from the coxa with
                                          # the plate installed, so the cap +
                                          # its pressed bearing come off as
                                          # one captive unit -- the bearing
                                          # fits are never fought in service

# Removable service hatch (user, Aug 2026): a large hex chunk of the top
# plate is cut out and replaced by a screw-down lid, restoring interior
# access (electronics, wiring, yaw-cap bolts, standoff screws) that the
# full-size plate had buried.  The structural work happens at the RIM --
# the six bearing rings at the edge midpoints -- so the middle can open.
# Opening flats face the rings (14 mm bottom web kept at every ring);
# opening vertices point at the plate corners where the frame is widest.
HATCH_OPEN_APO = 64.0                     # opening apothem (through-cut)
HATCH_APO = 68.0                          # lid apothem: 4 mm overlap onto the
                                          # deck, and its az-30 flats stop
                                          # 1.3 mm short of the Phi 7 driver
                                          # holes so cap access stays clear
HATCH_LIP_CL = 0.3                        # lid registration lip vs opening
HATCH_LIP_H = 1.5                         # lip drop into the opening
HATCH_LIP_W = 2.7                         # lip ring radial width
HATCH_SCREW_RHO = 78.0                    # 6x M3 at the opening's VERTEX
                                          # azimuths (0..300 deg): clear of
                                          # rings and driver holes; moved
                                          # 76.2 -> 78.0 when the pillars
                                          # were deleted so the FRAME's own
                                          # insert boss keeps a 2.1 mm bore
                                          # wall to the opening corner
                                          # (73.90); the lid ear covers the
                                          # bare-corner shortfall
HATCH_EAR_OD = 9.0                        # round lid ears around the screws:
                                          # the bare hex corner (78.52) no
                                          # longer covers the hole at rho 78;
                                          # the ear does (2.8 mm annulus,
                                          # bearing on solid frame deck)
# HEAT-SET INSERTS for the hatch screws (user, Aug 26: the hatch comes
# off "a lot" -- self-tapped PETG threads would strip; "but I also want
# the stiffness of the plate" rules out magnets / quarter-turn latches,
# which cannot deliver full screw clamp preload).  M3 x 5.7 brass
# inserts (Ruthex-style, Phi 4.6 knurl): steel-into-brass threads
# survive unlimited service cycles at the SAME screw clamping.
# NO PILLARS (user, Aug 26 pm: "sturdy enough without the columns"):
# the inserts moved from the deleted pillar tops into six FRAME_BOSS
# bosses hanging under the frame sheet at the hatch-screw stations.
# The insert installs from BELOW, so screw tension pulls it against
# the sheet (compression -- the strong direction).  Screw stack: lid 4
# + sheet 6 + 0.3 air + 5.7 insert = M3x16, tip flush at the boss
# bottom.  (Was M3x14 on the 4 mm sheet.)
INSERT_BORE_OD = 4.0                      # install bore for the M3 insert
INSERT_LEN = 5.7                          # insert length (flush at the
                                          # boss BOTTOM face)
INSERT_RELIEF_OD = 5.5                    # shallow entry counterbore: melt
INSERT_RELIEF_DEPTH = 0.4                 # displacement stays behind the
                                          # boss face, never proud
FRAME_BOSS_OD = 10.0                      # insert boss under the frame sheet
FRAME_BOSS_H = 6.0                        # boss drop below the sheet =
                                          # through-bore depth (insert 5.7
                                          # + 0.3 air up to the sheet); the
                                          # hatch-opening cutter trims the
                                          # boss flush at the opening wall,
                                          # keeping the lid lip's 0.3 mm
                                          # drop-in clearance

# PILLARS DELETED (user, Aug 26 pm: "I actually think its sturdy
# enough without the columns").  The Aug-23 rim pillars tied the frame
# to chassis_bottom at the corner azimuths; the user judges the
# bearing sandwich alone sufficient: each hip moment still closes
# through its own two 6805s into the plate and bottom sheet, and the
# plate couples to the bottom through the OTHER five legs' bearing
# towers (plus the four central standoffs via the hatch).  The frame
# is retained on the six races by the pocket press fits and the
# bolted caps below them.  What went with the pillars: 6 prints,
# 18 M3x12 + belly nylocs, 18 printed foot holes in the bottom sheet,
# 6 dedicated frame->pillar screws (+ their frame holes), and 6 of
# the 12 pillar-top inserts.
# ROUNDED-CORNER ENVELOPE (user, Aug 24, for the since-deleted
# pillars): everything that rotates with a yaw joint is kept inside a
# 38.2 mm cylinder about its own axis (the coxa's servo-cradle corners
# were rounded back from 40.36 to the envelope arc, max 2.16 mm off
# two vertical wall corners).  The pillars this cleared are gone, but
# the trim is KEPT: it costs nothing, the coxa is a variant print
# anyway, and any future rim furniture inherits the guarantee.
ROT_ENVELOPE_R = 38.2                     # max rotating reach (coxa trim)
ENV_MIN_CL = 5.0                          # generic static-vs-envelope margin
                                          # (wago lever check)
ROT_BAND_Z0 = 19.0                        # lowest z where anything rotates
                                          # outside the (static) tower --
                                          # the M3x20 drop puts the slab
                                          # bottom at world 19.25, right on
                                          # this bound (tied to the real slab
                                          # in check_coxa_column; the wago
                                          # levers now clear RADIALLY, see
                                          # check_wago_block)

# BOTTOM JOINT (user, Aug 24: "we really only need one bearing on the
# bottom -- so maybe its possible to simplify the design"; Aug 25:
# "why cant the bearing come down even more and be right on top of
# the servo?").  The variant first moved the single race into the
# production LOWER-race pocket (coxa-local z 0.5) and DELETED the
# bolt-on yaw_bearing_cap (-6 prints, -18 screws).  The Aug 25 rev
# drops the race a further BR_DROP = 5 to the true physical floor:
# race bottom RACE_DECK_CL = 0.5 above the servo-mount deck-top
# plane (world 10.25), which is simultaneously the yaw servo's
# case-top plane and the plane where the hub's Phi 25.15 press boss
# necks to its Phi 20 drive nub.  Race band: coxa-local -4.5..+2.5,
# world 10.75..17.75.
# WHY IT CAN SIT THIS LOW (corrects the old "race cannot sit lower,
# it would rub the horn" claim -- that was ARCHITECTURAL, not
# physical): the disc horn is not at the z=0 mount plane at all.
# The real Phi 20 disc sits RECESSED in the servo's output face at
# coxa-local -11..-9 (world 4.25..6.25, under the deck plate), and
# everything between the deck plane and the old race position is
# the coxa hub's OWN Phi 25.15 wide boss -- which production already
# runs down to the deck plane (hp.YAW_HUB_BOSS_WIDE_BOT_Z = -5),
# necking to the Phi 20 nub that reaches the horn through the
# deck's Phi 24 bore.  The horn and its screw heads (which seat
# HIGH in the slab, never down here) are inside/below that boss, so
# the inner race presses on the SAME bench-tuned +0.15 boss band as
# before, just 5 mm lower on the same cylinder.  NO new collar, no
# new press geometry: the "collar" IS the production boss.  The
# floor is set by the static deck/case-top plane at world 10.25
# (the outer race and its printed ledge must clear the servo case
# by RACE_DECK_CL) -- to go lower the servo itself would have to
# sink into the chassis.
# The hub's Phi 29 seat ring (production uflange OD, bears only on
# the Phi 25..29 inner-race land) follows the race top down to
# coxa-local 2.5.  Axial: hanging legs load hub -> ring -> inner
# race -> outer race -> tower LEDGE at world 10.75; standing loads
# go up through the TOP bearing into the plate shoulder -- each
# bearing takes one direction, no cap lip needed.
# TOWER LEDGE + RIM: the chassis tower's Phi 34 shoulder relief
# (production band world 10.25..15.75) is bored out to the
# Phi 37.15 press bore down to world 10.75, leaving a 0.5 mm-proud
# Phi 34/Phi 37.15 step -- the new outer-race seat ledge (face
# r 17.0..18.575, same 1.5 mm race land as the production seat) --
# sitting directly on the 4 mm servo-mount deck, solid at every
# azimuth (probed: the well breach lives below world ~8.5).  The
# tower above is trimmed so the Phi 44 column ends exactly at the
# new race-top plane (world 17.75, full 7 mm wrap, hoop wall
# 3.425 mm -- identical section to the production pocket band).
# The old raised-rim rings are gone; the rim IS the trimmed tower.
# What remains above is the small Phi 38 DUST BRIM on the coxa,
# hovering 0.5 above the rim / race-top plane and stepped 3 mm
# inside the tower OD, so the visual stack is deck -> bearing (in
# the tower) -> coxa.  Grit still has one turn (under the brim,
# through the 0.5 axial gap) to reach the 2RS seal -- the seal is
# the real barrier, and the brim still roofs the seal + outer-race
# band completely.  Non-contact everywhere: brim-to-rim and
# brim-to-race gaps are both BRIM_GAP, the rotating seat ring keeps
# 4.0 mm radial to the pocket bore, and the rotating inner race /
# boss keep RACE_DECK_CL axial to the deck and 4.4 mm radial to the
# ledge lip.  The az-210 cap-bolt ear root is below the deck
# (z 10.25 world), under the ledge band, untouched.
#
# COXA SHORTEN (user, Aug 24 rev 3: "from around 25mm to 40mm above
# the platform top seems largely unnecessary.  You need to redesign
# the coxa link to make it shorter, theres a bunch of unneccessary
# stuff there").  Audit of that band (coxa-local z ~10..25) found the
# production coxa's vertical hub column was sized around apparatus
# this variant DELETED:
#   * the Phi 52.4 dust-lip SKIRT (z 14..18) labyrinthed over the
#     production cap ring -- the cap is gone and the variant has its
#     own Phi 38 brim at z 3..5, so the skirt hung in mid-air;
#   * the Phi 52.4 platform DISC (z 18..24) existed to clear the
#     production cap rim at z 16.5 by YAW_HUB_CAP_AXIAL_CL and to
#     hang that skirt -- with the cap deleted and the bearing down at
#     z -4.5..+2.5, that clearance stack is filler;
#   * the production uflange (z 14.5..16.5) retained the deleted
#     UPPER race -- the variant's seat ring at the relocated race
#     already does its job.
# What is NOT filler is the horn-screw stack: production bench-pinned
# the M3x30 head seats at YAW_HUB_HORN_HEAD_SEAT_Z = +17.75 (screw
# length + measured print shortfall + tip poke -- the tips cannot go
# deeper without jamming the disc off its seat, see the constants in
# hexapod_prototype).  The redesign drops the whole slab + cradle
# unit (a rigid body: cradle pilots, cap seat, 688 housing, rear tab
# all ride along) by COL_DROP so the well floor lands COL_HEAD_CL
# above the screw heads, deletes the skirt + disc outright, and
# truncates the hub boss at the dropped slab.  The hip axis and
# EVERYTHING keyed to it (cap boss, top bearing, top plate,
# hatch, standoffs) drop by the same COL_DROP -- a
# shorter robot, same simply-supported stack.
#
# HORN SCREWS SHORTENED M3x30 -> M3x20 (user, Aug 25: "worry about
# the screws last -- I can buy any type of screw and shorter ones
# would be better, so the shorter we can make this the better").
# The Aug 24 rev-4 step went M3x30 -> M3x25 because the tower rim
# (then at the race top, world 22.75) capped how far the rotating
# slab could drop.  With the BEARING itself dropped onto the deck
# (see the BOTTOM JOINT block: race now at world 10.75..17.75, rim
# at 17.75), the rim moved down 5 mm and the slab can follow: all 5
# horn screws per hub (4 drive + 1 centre; 30 per robot --
# PURCHASED-HARDWARE CHANGE) become M3x20, and every seat plane
# drops by EXACTLY the 10 mm length delta vs production.  Seat -
# length = tip, so the tip planes and the thread engagement are
# IDENTICAL to the bench-tuned production stack: corner tips still
# break the disc horn's far face by TIP_POKE (full 2 mm tapped-disc
# engagement), the centre screw still reaches the same depth in the
# output spline's tapped bore.  check_coxa_column asserts the seat
# drop == the length delta.
# Why not M3x16: another -4 would put SLAB_BOT_Z at 0.0 (world
# 15.25), i.e. the rotating cradle slab 2.5 mm BELOW the static
# tower rim (17.75) -- collision.  M3x20 is the shortest standard
# length that fits this joint; it lands the slab 1.5 mm over the
# rim (>= 0.5 required), the same margin the M3x25 revision had.
HORN_BOLT_LEN = 20.0                      # M3x20 SHCS -- was M3x25 (30x/robot)
HORN_SEAT_DROP = hp.YAW_HUB_HORN_BOLT_LEN - HORN_BOLT_LEN    # 10.0 -- seats
                                          # follow the screws down 1:1 so the
                                          # tips (= seat - length) never move
HORN_HEAD_SEAT_Z = (hp.YAW_HUB_HORN_HEAD_SEAT_Z
                    - HORN_SEAT_DROP)     # 7.75 -- 4 corner-drive seats
HORN_CENTRE_SEAT_Z = (hp.YAW_HUB_HORN_CENTRE_SEAT_Z
                      - HORN_SEAT_DROP)   # 6.75 -- centre spline-screw seat
COL_HEAD_CL = 1.25                        # servo belly over the M3 heads
                                          # (>= the ~1 mm FDM axial stack-up
                                          # this axis measured on the bench)
COXA_FLOOR_Z = (HORN_HEAD_SEAT_Z
                + hp.INSERT_M3_BOLT_HEAD_H + COL_HEAD_CL)   # 12.0 well floor
COL_DROP = (hp.YAW_HUB_PLATFORM_Z1
            + hp.COXA_WELL_FLOOR_LIFT) - COXA_FLOOR_Z       # 14.0 -- the shrink
SLAB_BOT_Z = hp.YAW_HUB_BOSS_TOP_Z - COL_DROP     # 4 -- dropped slab bottom
HUB_TRIM_Z = SLAB_BOT_Z + 0.75            # hub boss keeps 0.75 into the slab
COXA_HIP_DROP_V = hp.COXA_HIP_DROP - COL_DROP     # 24.4 -- hip axis, coxa z
COXA_HIP_ANCHOR_V = (hp.COXA_HIP_ANCHOR[0], hp.COXA_HIP_ANCHOR[1],
                     COXA_HIP_DROP_V)     # the variant's hip joint anchor
# BEARING ON THE DECK (user, Aug 25: "why cant the bearing come down
# even more and be right on top of the servo?").  The race bottom
# drops from the production LOWER-race seat (coxa-local +0.5) to
# RACE_DECK_CL above the servo-mount deck-top plane -- which is also
# the yaw servo's case-top plane AND the plane where the coxa hub's
# Phi 25.15 wide boss already necks to its Phi 20 drive nub
# (hp.YAW_HUB_BOSS_WIDE_BOT_Z, all three coincide at world 10.25).
# See the BOTTOM JOINT block for why nothing below stops it sooner.
RACE_DECK_CL = 0.5                        # race bottom over deck/case top
BR_BOT_LOCAL = (hp.YAW_HUB_BOSS_WIDE_BOT_Z
                + RACE_DECK_CL)           # -4.5 -- race bottom, coxa-local
BR_DROP = hp.YAW_BEARING_LOWER_BOT_Z - BR_BOT_LOCAL   # 5.0 below production
YAWBR_DROP = -(hp.YAW_BEARING_W + BR_DROP)  # -12: race visual to the deck seat
HUB_RING_OD = hp.YAW_BEARING_INNER_OD     # 29 -- production uflange OD
HUB_RING_ID = 24.0                        # overlaps the boss wall (22..25.15)
HUB_RING_Z0 = (hp.YAW_BEARING_LOWER_TOP_Z
               - BR_DROP)                 # 2.5 -- relocated race TOP
HUB_RING_Z1 = SLAB_BOT_Z + 1.0            # 5 -- 1 mm into the dropped slab
                                          # (was uflange-referenced; the
                                          # uflange is deleted with the
                                          # shortened column)
BRIM_GAP = 0.5                            # running gap, axial + radial
TOWER_RIM_Z = HUB_RING_Z0                 # 2.5 -- rim = race top
                                          # (coxa-local; world 17.75)
BRIM_OD = hp.YAW_BEARING_OD + 2.0 * BRIM_GAP     # 38 -- roofs the seal + outer
                                          # race (r 18.5) plus the gap; 3 mm
                                          # inside the Phi 44 tower so the
                                          # brim reads as coxa, not chassis
BRIM_BOT_Z = TOWER_RIM_Z + BRIM_GAP       # 3.0 -- hovers 0.5 over rim AND race
BRIM_TOP_Z = SLAB_BOT_Z + 1.0             # 5.0 -- 2 mm brim, fuses 1 mm into
                                          # the slab (tracks SLAB_BOT_Z)

# CHASSIS VARIANT (user, Aug 24: "round the corners on the chassis
# bottom below where the bearings go"; rev 2 same day: "I don't like
# the multiple curves here, they should all match").  Each production
# yaw tower stands on a SQUARE outboard platform (leg-frame face
# x 121.2; half-width 21.25 above the sheet, 18.9 for the z -6..-2
# belly skirt; top face z 6.25) whose corners poked 8.4 mm diagonally
# past the Phi 44 tower.  Rev 1 rounded them with a bolt-on R 10
# corner radius, which stacked THREE different arcs at each corner
# (platform R 10 arc, the shallower chord the same cylinder cut into
# the narrower skirt, and the r 22 tower crown right above) -- the
# user rejected the mismatch.  Now the corners are simply TRIMMED TO
# THE TOWER'S OWN CYLINDER (r 22.02 about the yaw axis; 0.02 proud so
# the boolean never grazes the tower's own tessellation).  In
# production the tower already bulges through the platform band (its
# r 22 circle crosses the +-21.25 side faces at x 100+-5.9), so the
# trim just continues that same curve around the end and down the
# skirt: every band shows ONE curve -- the tower's -- and the
# side-face junction is the crease production already has.  The
# bearing pocket, well walls and rim are untouched.  With the
# yaw_bearing_cap deleted, the three M3 cap-bolt ear lugs per tower
# (PCD 47, z 6.25..19.75) are dead weight:
#   * the OUTBOARD ear (coxa az 330) sat ON the trimmed corner --
#     shaved flush to the tower cylinder (0.05 bite into the tower
#     skin for clean topology);
#   * the TANGENTIAL ear (az 90) poked 6.8 mm past the platform
#     silhouette -- shaved flush to the rim-wall face (y 20.45);
#   * the INBOARD ear (az 210, r to 28 from the yaw axis) is shaved
#     flush to the SERVO-MOUNT DECK TOP (z 10.25): the free-standing
#     column above the deck (z 10.25..19.75, its M3 pilot served the
#     deleted cap) is cut away, while the below-deck root (6.25..
#     10.25) STAYS merged with the well-mouth collar -- cutting there
#     risks gouging the collar, and it invisibly stiffens the deck.
#     After this, NOTHING pokes past the tower cylinder above the
#     deck at any azimuth: the six towers read as clean bare columns.
CHB_PLATE_TOP = 6.25          # platform top face = ear-lug bottom (measured)
CHB_DECK_TOP = 10.25          # servo-mount deck top face (measured); the
                              # az-210 ear is shaved flush to THIS plane
CHB_TOWER_R = hp.YAW_BEARING_OD / 2.0 + hp.YAW_TOWER_WALL   # 22.0
CHB_TRIM_R = CHB_TOWER_R + 0.02   # corner trim = the tower cylinder itself
CHB_KEEP_R = CHB_TOWER_R - 0.05   # ear cuts bite 0.05 into the tower skin
CHB_RIM_OLD_W = hp.CHASSIS_YAW_OUTPUT_Z + hp.YAW_SPLIT_Z    # 19.75 -- the
                              # production tower top (4 mm race wrap at the
                              # OLD seat), world z; the Aug 25 rebuild cuts
                              # the whole band above the new seat and
                              # re-rings it lower (see the tower cuts)
CHB_SEAT_W = hp.CHASSIS_YAW_OUTPUT_Z + BR_BOT_LOCAL         # 10.75 -- new
                              # outer-race seat ledge, world z: the Phi 34
                              # shoulder relief keeps only a 0.5 mm band
                              # over the deck; Phi 37.15 press bore above
CHB_RIM_W = hp.CHASSIS_YAW_OUTPUT_Z + TOWER_RIM_Z           # 17.75 -- rim =
                              # race-top plane, world z (full 7 mm wrap:
                              # the Phi 44 column now ends 5 mm lower)
CHB_EAR_R = 6.6               # covers the Phi 9 ear boss with margin
CHB_WALL_FACE_Y = 20.45       # rim-wall outer face (20.33 measured) + cl
# TOWER-FLANK BUMP SHAVE (rev 7 user, Aug 25: "remove this weird bump
# outside the bottom chassis in the part that holds the bearing and
# make it vertically smooth").  Production's +X horn swing relief
# spares a protect cylinder 1.5 mm FATTER than the tower (r 23.5 =
# YAW_BEARING_OD/2 + YAW_TOWER_WALL + 1.5, the ``tower_protect``
# literal in hexapod_prototype._chassis_yaw_cradle_solid), so the
# cradle-shell box survives out to r 23.5 across the mount-plate band
# (z CHB_PLATE_TOP..CHB_DECK_TOP).  After the ear + flatten cuts, two
# Phi 47 ring-sector arcs per leg remained on the outboard flank
# (leg-frame az ~17..55 and ~300..314, measured on the shipped STL,
# identical on all six legs) -- pure leftovers: the bearing seat,
# hoop wall and deck all live inside r 22, and the cap bolts that
# once justified r 23.5 are deleted in this variant.  A per-leg ring
# cutter (same footprint as the corner trim, z band padded 0.25 past
# the bump) shaves them to CHB_TRIM_R, so the tower outer profile is
# ONE vertical cylinder (r 22.00..22.02) from the sheet top to the
# rim (enforced by the cylindricity assert in check_chassis_variant).
# CORRIDOR + CRADLE-SHELL FLATTEN (rev 5 user, Aug 24: "take a step
# back that CRADLE WALL isnt doing shit, just flatten it out and its
# fine" / "its flatening random bumps in the top of the chassis plate
# that serve no purpose"; rev 6 user, Aug 25: "how are there still
# these two gray things from the waygo sticking up on each side of
# the column" -- the cradle-shell ends rev 5 left standing at each
# corner flat).  Rev 5 cut the wago-era WIRE-
# CORRIDOR apparatus (cradle end wall at x 61.5..64.5, porch canopy,
# inboard side-wall stubs) back to the bare sheet with a box ending
# at x 64.65.  Rev 6 extends that same box to the yaw axis and
# subtracts the tower KEEP cylinder (r CHB_KEEP_R about x 100), which
# removes the remaining cradle SHELL outboard of the old cut face:
# the 5.6 mm side walls at leg-frame |y| 13.2..18.8 (z 2 -> 10.25,
# standing 0.85 mm off the servo case flanks) and the 2.2 mm deck-
# skin roof they carried (z ~8.05 -> 10.25, spanning |y| <= 18.8,
# hovering 1.8 mm above the REAL case top at z 6.25 -- the seated
# servo's "case top = deck top 10.25" plane is only the horn-boss
# region inside the tower; the visual servo block is a fat envelope).
# Measured before cutting (Aug 25): nothing bears on that roof (the
# bearing and the top plate ride the towers), the roof
# registers nothing, and the walls' only job was holding the roof up.
# The servo is REGISTERED elsewhere, all untouched by this cut: the
# sheet-level well (z 0..2) hugs the case at 0.75 mm/side for its
# full length, the belly-bolted yaw_servo_retainer takes lateral +
# drop-out loads (plate pilots at leg-frame x 71/87.5, |y| 21,
# z -2..1 -- outside the cut in x, y AND z, verified), and the shell
# INSIDE the tower keep cylinder (well-mouth collar, pocket floor,
# seat ledge) stays: it carries the inboard arc of the 6805 seat over
# the servo tunnel, so the keep cylinder is a hard structural
# boundary, not styling.
CHB_FLAT_X0 = 50.0            # inboard face: past the porch's x~52 lip
CHB_FLAT_X1 = hp.CHASSIS_FLAT_TO_FLAT / 2.0   # 100 -- the yaw axis;
                              # the tower keep cylinder bounds the cut
                              # (rev 5 stopped at 64.65, splitting the
                              # old wall-to-servo gap)
CHB_FLAT_HALF_Y = 20.5        # covers walls to |y| 18.8; the rim-wall
                              # face at 20.45 near the tower is inside
                              # the keep cylinder at every covered x
CHB_FLAT_Z0 = hp.CHASSIS_PLATE_T / 2.0    # 2.0 -- the bare sheet top
CHB_FLAT_Z1 = 12.0            # clears the deck top (10.25); the only
                              # taller corner material is the tower
                              # ring, inside the keep cylinder
# ABOVE-SHEET WHITELIST (rev 8 user, Aug 25: "get rid of these L shaped
# bumps I circled in red ... all of them, not just the ones i circled").
# Every earlier pass was a BLACKLIST -- name a leftover, cut it -- and
# every pass left something standing (rev 8's L-stubs lived only in the
# STEP sidecar: the base STEP chassis still modeled the RETIRED two-bay
# WAGO3 corner tray, 4.8 mm wider than the production WAGO5 tray the
# variant's tray-delete cutter is sized for, so the old side walls at
# tray-local |y| 19.75..22.15 survived at all six corners).  Rev 8 flips
# the rule to a WHITELIST: above the bare sheet top (z 2) the ONLY
# material allowed on chassis_bottom_rigid is
#   1. the six yaw towers -- everything within CHB_TRIM_R (22.02) of a
#      yaw axis: pocket, seat ledge, deck plateau, rim;
#   2. the servo deck plateaus -- measured from the servo well geometry
#      this category is DEGENERATE: the only deck the seated servo
#      still needs (the 6805 seat's inboard arc over the servo tunnel
#      + the well-mouth collar that registers the case at the output
#      end) lives entirely inside the tower keep cylinder, i.e. inside
#      category 1 (the rev-6 measurements showed nothing bears on the
#      roof outside it, and the servo is registered by the sheet-level
#      well + the belly retainer);
#   3. features a placed part verifiably mates with -- measured EMPTY:
#      the retainer bolts from BELOW the sheet, the wago block is taped
#      to the sheet at the centre (nothing else stands on the plate
#      since the pillars were deleted).
# So the whitelist cutter is one global box (sheet top -> above the
# rim) minus the six tower cylinders, applied in BOTH pipelines before
# the rim re-union; check_chassis_variant enforces it on every build
# (no vertex above z 2.05 outside every tower cylinder + CHB_WL_TOL).
CHB_WL_Z1 = 26.0              # cutter top: clears the rim (17.75) + slack
CHB_WL_KEEP_R = CHB_TRIM_R + 0.01   # 22.03: 0.01 proud of the trim
                              # cylinder so the cutter's inner wall sits
                              # in AIR next to the r 22.02 trim surface
                              # (no coincident-face boolean, towers and
                              # pockets untouched by construction)
CHB_WL_TOL = 0.03             # assert slack over CHB_TRIM_R (matches
                              # the rev-7 cylindricity assert)
BOT_SHEET_TOP_Z = hp.CHASSIS_PLATE_T / 2.0   # +2.0 -- bottom sheet top face

# The corner WAGO TRAYS are deleted from the chassis print (user,
# Aug 24: with the top frame installed those corner Wagos would be
# buried under solid deck, no lever access -- the corner + trunk
# splices CONSOLIDATE into the central block, see WBLK_* below).  The
# tray-cut block lives in build_rigid_hip_step.py's
# make_chassis_bottom_rigid.  (The Aug-23..26 corner pillars that
# briefly stood in the vacated bays are deleted too -- see the
# no-pillars note above -- so the bays are now simply bare sheet.)
_WAGO_BAY_W = hp.WAGO5_W + hp.WAGO_MOUNT_BAY_CLEAR    # 29.85 tangential
# Tray wall-set envelope (for the delete cut + the gone-check): the
# production tray is 2 side walls + 1 outer wall, 2.4 thick, 6.5 tall,
# on the plate top face at each corner flat (az 0/60/...).
TRAY_HALF_X = (hp.WAGO5_D + hp.WAGO_MOUNT_BAY_CLEAR) / 2.0 \
    + hp.WAGO_MOUNT_WALL_T                # 11.625 radial half-extent
TRAY_HALF_Y = _WAGO_BAY_W / 2.0 + hp.WAGO_MOUNT_WALL_T   # 17.325 tangential

# CENTRAL SPLICE BLOCK (user, Aug 24: consolidate).  With the corner
# trays deleted, the power tree collapses into ONE
# printed block at the chassis centre: 4x 5-port 221-415 in two
# back-to-back press-fit rows (north pair = V+, south pair = GND; the
# two nuts of a net are jumpered, leaving battery-in + 6 leg branches
# + a spare port per net).  It replaces the 6 corner nuts AND the two
# trunk nuts: 8 -> 4.  Bays reuse the bench-tuned production tray
# constants (0.15 mm wedge, 2.4 mm walls, walls stop ~1.9 mm below the
# nut top).  The floor sits on probed-solid sheet between the battery
# strap slots (inner edge y = 24), west of the battery-lead trunk pass
# (x 41..55) whose leads enter the east ports directly; the whole
# block is under the open hatch, levers up.  Mounted with a VHB pad --
# the exact scheme the production corner trays used for months before
# they merged into the chassis print (and here the wire pull is a
# gentle radial fan, not one corner's inward yank).
WBLK_BAY_W = hp.WAGO5_W + hp.WAGO_MOUNT_BAY_CLEAR   # 29.85 (block X)
WBLK_BAY_D = hp.WAGO5_D + hp.WAGO_MOUNT_BAY_CLEAR   # 18.45 (block Y)
WBLK_WALL_T = hp.WAGO_MOUNT_WALL_T                  # 2.4
WBLK_WALL_H = hp.WAGO_MOUNT_WALL_H                  # 6.5 above the floor
WBLK_FLOOR_T = 2.0
WBLK_HALF_X = WBLK_BAY_W + 1.5 * WBLK_WALL_T        # 33.45
WBLK_HALF_Y = WBLK_WALL_T / 2.0 + WBLK_BAY_D + WBLK_WALL_T   # 22.05

# Cap-local (well-frame) geometry.  Well frame: origin = hip servo back-face
# centre, +X = body long axis, +Y = out of the open face (world UP at the
# hip), +Z = output axis.  The yaw axis pierces the cap at (x=0, z=AXIS_Z).
CAP_FACE_Y = hp.WELL_D / 2.0 + hp.CLAMP_CAP_T             # 21.9 outer face
AXIS_Z = hp.JOINT_HORN_TOP_Z + hp.COXA_HIP_ANCHOR_Y      # 15.65
PED_Y0 = CAP_FACE_Y - 1.0                                 # 1 mm weld bite
PED_Y1 = CAP_FACE_Y + PED_H                               # race seat plane
BOSS_Y1 = PED_Y1 + BEARING_W - BOSS_TIP_H                 # press band top
TIP_Y1 = PED_Y1 + BEARING_W                               # lead-in tip top

# World-frame stack (z up, chassis_bottom sheet mid-plane at z = 0).
# Uses the variant's SHORTENED hip drop: the whole top stack rides
# COL_DROP lower than the production-coxa numbers.
CAP_FACE_W = hp.CHASSIS_YAW_OUTPUT_Z + COXA_HIP_DROP_V + CAP_FACE_Y  # 61.55
BR_BOT_W = CAP_FACE_W + PED_H             # 67.05 -- race bottom / seat
BR_TOP_W = BR_BOT_W + BEARING_W           # 74.05 -- race top / shoulder
RING_BOT_W = BR_BOT_W + RING_BOT_CL       # 67.55 -- ring bottom face
SHEET_Z0 = BR_TOP_W                       # sheet bottom = race top plane
SHEET_Z1 = SHEET_Z0 + TOP_PLATE_T         # 80.05 -- deck face (6 mm sheet)

APOTHEM = hp.CHASSIS_FLAT_TO_FLAT / 2.0   # 100 -- yaw axes sit ON this line

XZ = ((1.0, 0.0, 0.0), hp.LEG_PITCH_AXIS)
M_HIP_JP = hp._joint_place(COXA_HIP_ANCHOR_V, *XZ)      # well -> coxa (hip)
M_KNEE_JP = hp._joint_place((hp.FEMUR_LENGTH, 0.0, 0.0), *XZ)
MH = hp._joint_place((0.0, 0.0, 0.0), *XZ)              # joint -> link local


def _trans(v) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = np.asarray(v, float)
    return T


def _rotz(rad: float) -> np.ndarray:
    return rotation_matrix(rad, [0, 0, 1])


def _cyl_z(r: float, z0: float, z1: float, x: float = 0.0, y: float = 0.0,
           sections: int = 128) -> trimesh.Trimesh:
    c = trimesh.creation.cylinder(radius=r, height=z1 - z0, sections=sections)
    c.apply_translation([x, y, 0.5 * (z0 + z1)])
    return c


def _cyl_y(r: float, y0: float, y1: float, x: float = 0.0, z: float = 0.0,
           sections: int = 128) -> trimesh.Trimesh:
    c = trimesh.creation.cylinder(radius=r, height=y1 - y0, sections=sections)
    c.apply_transform(rotation_matrix(-np.pi / 2.0, [1, 0, 0]))  # axis -> +Y
    c.apply_translation([x, 0.5 * (y0 + y1), z])
    return c


def _box(extents, center) -> trimesh.Trimesh:
    b = trimesh.creation.box(extents=extents)
    b.apply_translation(list(center))
    return b


def _union(meshes):
    return trimesh.boolean.union(meshes, engine="manifold")


def _diff(base, cutters):
    return trimesh.boolean.difference([base, *cutters], engine="manifold")


def _inter_vol(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    """Boolean intersection volume with a cheap AABB prefilter."""
    if (a.bounds[1] < b.bounds[0]).any() or (b.bounds[1] < a.bounds[0]).any():
        return 0.0
    m = trimesh.boolean.intersection([a, b], engine="manifold")
    return 0.0 if m.is_empty else float(m.volume)


# ---------------------------------------------------------------------------
# Placement helpers + visual (non-printed) parts.
#
# The six variant PRINTABLES are no longer built here: geometry is
# authored STEP-FIRST as build123d/OpenCascade BREP solids in
# build_rigid_hip_step.py (user, Aug 2026: "the official way is to make
# step files").  This driver loads that script's tessellations from
# step/stl/ (see the mesh registry below), checks them as an assembly and
# copies them into the print set.  The old trimesh twins live in git
# history (pre-flip) if the rationale comments are ever needed.
# ---------------------------------------------------------------------------

def _hex_prism(apothem: float, z0: float, z1: float,
               flats_at_rings: bool = False) -> trimesh.Trimesh:
    """Hexagonal prism.  Default = the PLATE's as-built orientation
    (measured: VERTICES at the ring azimuths 30/90/... deg).  With
    ``flats_at_rings`` the hex is rotated 30 deg so its FLATS face the
    rings -- the hatch orientation, keeping the web at every ring wide
    and reaching toward the inter-leg directions instead."""
    p = _cyl_z(apothem / np.cos(np.pi / 6.0), z0, z1, sections=6)
    if not flats_at_rings:
        p.apply_transform(rotation_matrix(np.pi / 6.0, [0, 0, 1]))
    return p


def _hatch_screw_xy() -> list[tuple[float, float]]:
    return [(HATCH_SCREW_RHO * np.cos(np.deg2rad(az)),
             HATCH_SCREW_RHO * np.sin(np.deg2rad(az)))
            for az in range(0, 360, 60)]


def _access_hole_xy() -> list[tuple[float, float]]:
    """World XY of each leg's INBOARD hip-cap clamp bolt at yaw 0.

    ``hp.servo_clamp_bolt_centres()`` is the single source of truth for
    the (x, z) bolt pattern in the well frame (bolt axis = well +/-Y =
    world vertical at the hip).  One bolt of each pair lands outboard of
    the hex edge (open sky, always reachable); the other hides under the
    plate -- THAT one gets a driver pass-through hole so the cap can be
    unbolted with the plate installed."""
    out = []
    for i in range(6):
        Tc = leg_transforms(i)["hip_cap"]
        for (bx, bz) in hp.servo_clamp_bolt_centres():
            w = Tc @ np.array([bx, 0.0, bz, 1.0])
            if np.hypot(w[0], w[1]) < APOTHEM:      # under the hex sheet
                out.append((float(w[0]), float(w[1])))
    assert len(out) == 6, f"expected 6 under-plate cap bolts, got {len(out)}"
    return out


def make_bearing_6805() -> trimesh.Trimesh:
    """Visual 6805-2RS stand-in in the CAP frame, seated on the pedestal.

    NOT PRINTED.  Bore modeled at BOSS_OD (25.15) so the seated scene has
    zero mesh overlap; the real bearing bore is Phi 25.00 and the 0.15
    diametral interference is the press."""
    outer = _cyl_y(BEARING_OD / 2.0, PED_Y1, PED_Y1 + BEARING_W,
                   x=0.0, z=AXIS_Z)
    bore = _cyl_y(BOSS_OD / 2.0, PED_Y1 - 1.0, PED_Y1 + BEARING_W + 1.0,
                  x=0.0, z=AXIS_Z)
    return _diff(outer, [bore])


def chassis_whitelist_violations(mesh: trimesh.Trimesh,
                                 z_min: float = 2.05) -> tuple[int, float]:
    """Rev-8 above-sheet whitelist census on a chassis_bottom mesh.

    Returns ``(n_bad, worst_r)``: the number of vertices above ``z_min``
    that lie OUTSIDE every tower cylinder (distance to the nearest yaw
    axis > CHB_TRIM_R + CHB_WL_TOL), and the worst such distance.  The
    deck-plateau and mates whitelist categories are degenerate/empty
    (see the CHB_WL_* constant block), so the towers are the whole
    whitelist.  Shared by check_chassis_variant, the STEP sidecar's
    derived-STL check and probe_above_sheet.py."""
    v = np.asarray(mesh.vertices)
    above = v[v[:, 2] > z_min]
    if len(above) == 0:
        return 0, 0.0
    d = np.full(len(above), np.inf)
    for i in range(6):
        a = (i + 0.5) * np.pi / 3.0
        d = np.minimum(d, np.hypot(above[:, 0] - APOTHEM * np.cos(a),
                                   above[:, 1] - APOTHEM * np.sin(a)))
    bad = d > CHB_TRIM_R + CHB_WL_TOL
    return int(bad.sum()), (float(d[bad].max()) if bad.any() else 0.0)


def _wago5_scene_frames() -> list[np.ndarray]:
    """World 4x4 for the four seated splice nuts (visuals).  The wago5
    visual's local -X is the wire-entry face; rotate it outward."""
    y_c = WBLK_WALL_T / 2.0 + WBLK_BAY_D / 2.0     # 10.425
    zf = BOT_SHEET_TOP_Z + WBLK_FLOOR_T
    x_c = WBLK_WALL_T / 2.0 + WBLK_BAY_W / 2.0     # 16.125
    out = []
    for sy in (+1.0, -1.0):                         # north row, south row
        for sx in (-1.0, +1.0):
            out.append(_trans([sx * x_c, sy * y_c, zf])
                       @ _rotz(-sy * np.pi / 2.0))
    return out


# ---------------------------------------------------------------------------
# Assembly frames (mirrors tools/full_robot_viz_build.py::_servo_M0s /
# _leg0_local_link_parts, with parametric yaw + pitch for the sweeps)
# ---------------------------------------------------------------------------

def leg_transforms(i: int, yaw_deg: float = 0.0,
                   femur_deg: float = None,
                   tibia_deg: float = None) -> dict[str, np.ndarray]:
    femur_deg = hp.STANCE_FEMUR_DEG if femur_deg is None else femur_deg
    tibia_deg = hp.STANCE_TIBIA_DEG if tibia_deg is None else tibia_deg
    a = (i + 0.5) * np.pi / 3.0
    edge = np.array([APOTHEM * np.cos(a), APOTHEM * np.sin(a),
                     hp.CHASSIS_YAW_OUTPUT_Z])
    p = np.deg2rad(femur_deg)
    pt = np.deg2rad(femur_deg + tibia_deg)
    hip_local = np.array(COXA_HIP_ANCHOR_V)   # SHORTENED coxa: hip axis
    #                                           rides COL_DROP lower
    knee_local = hip_local + rotation_matrix(p, [0, 1, 0])[:3, :3] \
        @ np.array([hp.FEMUR_LENGTH, 0.0, 0.0])

    T_coxa = _trans(edge) @ _rotz(a) @ _rotz(np.deg2rad(yaw_deg))
    T_femur = T_coxa @ _trans(hip_local) @ rotation_matrix(p, [0, 1, 0])
    T_tibia = T_coxa @ _trans(knee_local) @ rotation_matrix(pt, [0, 1, 0])
    # Chassis-mounted cradle frame: leg azimuth rotation at chassis z=0, NO
    # yaw and NO yaw-output lift. This is where the yaw_servo_retainer lives
    # (production places it at exactly this frame; parking it on T_coxa put
    # it CHASSIS_YAW_OUTPUT_Z = 15.25 mm too high, jammed into the platform).
    T_cradle = _trans([edge[0], edge[1], 0.0]) @ _rotz(a)
    return {"coxa": T_coxa, "femur": T_femur, "tibia": T_tibia,
            "cradle": T_cradle,
            "hip_cap": T_coxa @ M_HIP_JP,
            "knee_cap": T_femur @ M_KNEE_JP}


def _tibia_extras() -> tuple[trimesh.Trimesh, np.ndarray]:
    """(tube mesh in tibia-local frame, foot-boot local frame)."""
    ta = (MH @ np.array([hp._YOKE_SOCKET_X, 0.0, hp.JOINT_SOCKET_Z, 1.0]))[:3]
    tube_end = ta + np.array([hp.TIBIA_LENGTH - 8.0, 0.0, 0.0])
    tube = hp._tube_between(ta, tube_end, hp.LEG_TUBE_OD / 2.0)
    foot_frame = hp._frame(tube_end, (1, 0, 0), (0, 0, 1))
    return tube, foot_frame


# ---------------------------------------------------------------------------
# Mesh registry (build once, cache to stl/; REBUILD=1 forces)
# ---------------------------------------------------------------------------
BREP_STL_DIR = os.path.join(HERE, "step", "stl")
BREP_BUILDER = os.path.join(HERE, "build_rigid_hip_step.py")
# build123d has no Python-3.14 wheels yet, so the exporter runs in its own
# uv-provisioned 3.12 interpreter (same incantation as cad_step_test) and
# talks to this driver purely through the step/ files.
BREP_EXPORT_CMD = ["uv", "run", "--no-project", "--python", "3.12",
                   "--with", "build123d", "--with", "trimesh",
                   "--with", "numpy", "--with", "manifold3d",
                   "python", BREP_BUILDER]


def _brep(fname: str):
    """Loader factory for a variant printable: the geometry is authored in
    build_rigid_hip_step.py (BREP) and consumed here as its step/stl/
    tessellation.  process=True welds the STL triangle soup so the
    checks' watertightness/containment probes work."""
    def _load() -> trimesh.Trimesh:
        path = os.path.join(BREP_STL_DIR, fname)
        if not os.path.exists(path):
            raise SystemExit(
                f"{fname}: no BREP tessellation at {path} -- run "
                "build_rigid_hip_step.py first (main() does that for you "
                "unless --skip-brep)")
        m = trimesh.load(path, process=True)
        if isinstance(m, trimesh.Scene):
            m = trimesh.util.concatenate(
                [g for g in m.geometry.values() if len(g.faces) > 0])
        return m
    return _load


MESH_FILES = {
    # Variant printables -- GEOMETRY LIVES IN build_rigid_hip_step.py
    # (STEP-first); loaded from its step/stl/ tessellations, checked as
    # an assembly below, then exported into the print set (stl/).
    "hip_clamp_cap_rigid": (_brep("hip_clamp_cap_rigid.stl"),
                            "hip_clamp_cap_rigid.stl"),
    "chassis_top_rigid": (_brep("chassis_top_rigid.stl"),
                          "chassis_top_rigid.stl"),
    "top_hatch_rigid": (_brep("top_hatch_rigid.stl"), "top_hatch_rigid.stl"),
    "centre_wago_block": (_brep("centre_wago_block.stl"),
                          "centre_wago_block.stl"),
    # VARIANT reprints of production parts: the coxa gets its cradle
    # corners rounded to the 38.2 mm yaw envelope + a Phi 29 hub seat
    # ring down to the tower-seated bottom race; the chassis gets its
    # tower platforms trimmed to the tower's own cylinder and the dead
    # cap-bolt ears shaved.
    "coxa_link": (_brep("coxa_link_rigid.stl"), "coxa_link_rigid.stl"),
    "chassis_bottom": (_brep("chassis_bottom_rigid.stl"),
                       "chassis_bottom_rigid.stl"),
    # Unchanged production prints (print from the MAIN stl_prototype/).
    "femur_link": (hp.make_femur_link_part, "femur_link.stl"),
    "tibia_knee_yoke": (hp.make_tibia_knee_yoke, "tibia_knee_yoke.stl"),
    "foot_boot": (hp.make_foot_boot, "foot_boot.stl"),
    "knee_clamp_cap": (hp.make_servo_clamp_cap, "knee_clamp_cap.stl"),
    # NOTE: the production yaw_bearing_cap is DELETED in this variant --
    # the ONE bottom bearing sits in the chassis tower's own pocket
    # (production LOWER-race position) and the top plate provides the
    # retention the cap used to (see the BOTTOM JOINT constant block).
    "yaw_servo_retainer": (hp.make_yaw_servo_retainer,
                           "yaw_servo_retainer.stl"),
    # COTS / visual only.  ONE bottom bearing per leg: the production
    # UPPER-race mesh (built at coxa-local z 7.5..14.5) is instanced
    # with a -7 z drop into the LOWER position, seated on the tower
    # pocket's z=0.5 shoulder.  Net robot bearing count stays 12
    # (6 bottom + 6 top-plate).
    "servo_body": (hp.make_servo_body, "servo_body_DO_NOT_PRINT.stl"),
    "yaw_bearing_upper": (hp.make_yaw_bearing_upper,
                          "yaw_bearing_upper_DO_NOT_PRINT.stl"),
    "bearing_6805": (make_bearing_6805, "bearing_6805_DO_NOT_PRINT.stl"),
    "wago5": (hp.make_wago5_visual, "wago5_DO_NOT_PRINT.stl"),
}
# The BREP-loaded printables are "always rebuilt" so a stale stl/ copy can
# never mask a fresh step/stl/ export; loading is cheap.
BREP_PARTS = {"hip_clamp_cap_rigid", "chassis_top_rigid", "top_hatch_rigid",
              "centre_wago_block", "coxa_link", "chassis_bottom"}
ALWAYS_REBUILD = BREP_PARTS | {"bearing_6805"}


def build_meshes() -> dict[str, trimesh.Trimesh]:
    os.makedirs(STL_DIR, exist_ok=True)
    rebuild_all = os.environ.get("REBUILD", "") == "1"
    out: dict[str, trimesh.Trimesh] = {}
    for key, (factory, fname) in MESH_FILES.items():
        path = os.path.join(STL_DIR, fname)
        if key not in ALWAYS_REBUILD and not rebuild_all and os.path.exists(path):
            out[key] = trimesh.load(path)
            print(f"  {key:22s} loaded from cache")
            continue
        verb = ("loading BREP tessellation" if key in BREP_PARTS
                else "building")
        print(f"  {key:22s} {verb} ...", flush=True)
        # Heal EVERYTHING, BREP loads included: OpenCascade tessellation
        # occasionally emits STL-grid slivers (the coxa welds 2 non-manifold
        # edges) exactly like the manifold boolean kernel does, and the same
        # guarded pass dissolves them.  stl/ therefore always holds
        # slicer-clean meshes.
        m = hp._heal_for_export(factory())
        assert m.is_volume, f"{key}: not a volume even after heal"
        m.export(path)
        out[key] = m
    # tibia tube (per-leg primitive, cheap, not cached)
    tube, _ = _tibia_extras()
    tube.export(os.path.join(STL_DIR, "tibia_tube_DO_NOT_PRINT.stl"))
    out["tibia_tube"] = tube
    return out


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

def check_static(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Seated-assembly sanity: new parts watertight, stack lands where the
    constants say, nothing but the boss/bearing enters the plate bore."""
    for key in ("hip_clamp_cap_rigid", "chassis_top_rigid",
                "top_hatch_rigid"):
        m = meshes[key]
        assert m.is_watertight, f"{key} not watertight"
        print(f"  {key:22s} watertight, vol {m.volume / 1000.0:.1f} cm3")

    T = leg_transforms(0)
    cap = meshes["hip_clamp_cap_rigid"].copy()
    cap.apply_transform(T["hip_cap"])
    assert abs(cap.bounds[1][2] - TIP_Y1 - (CAP_FACE_W - CAP_FACE_Y)) < 1e-3, \
        "placed cap top does not match the derived world stack"
    plate = meshes["chassis_top_rigid"]
    v = _inter_vol(cap, plate)
    assert v < 1e-6, f"rigid cap intersects top plate ({v:.2f} mm3)"

    br = meshes["bearing_6805"].copy()
    br.apply_transform(T["hip_cap"])
    v = _inter_vol(br, plate)
    assert v < 1e-6, f"bearing intersects top plate ({v:.2f} mm3)"
    print(f"  seated cap/bearing vs plate: no overlap "
          f"(race top z = {br.bounds[1][2]:.2f}, shoulder at {SHEET_Z0:.2f})")

    # dust roof: solid plate directly over every bearing at the deck,
    # with the air gap below it actually open (probe both sides)
    roof_bot = SHEET_Z1 - BR_ROOF_T
    probes_solid, probes_air = [], []
    for _i, edge, _R, _R3 in hp._leg_chassis_frames():
        x, y = float(edge[0]), float(edge[1])
        probes_solid.append([x, y, (roof_bot + SHEET_Z1) / 2.0])
        probes_air.append([x, y, roof_bot - 0.2])
    assert plate.contains(np.asarray(probes_solid)).all(), \
        "dust roof missing over a bearing (probe found air at the deck)"
    assert not plate.contains(np.asarray(probes_air)).any(), \
        "no air gap under a dust roof (probe found solid below it)"
    tip_gap = roof_bot - TIP_Y1 - (CAP_FACE_W - CAP_FACE_Y)
    print(f"  dust roof: {BR_ROOF_T:g} mm disc flush at the deck over all "
          f"6 bearings, {tip_gap:.2f} mm air over the boss tip")

    # --- driver access holes: web clearances + clear line of sight ------
    holes = _access_hole_xy()
    open_vertex_r = HATCH_OPEN_APO / np.cos(np.pi / 6.0)   # 73.90
    for k, (hx, hy) in enumerate(holes):
        ax = leg_transforms(k)["coxa"][:2, 3]
        web = np.hypot(hx - ax[0], hy - ax[1]) - RING_OD / 2.0 \
            - ACCESS_HOLE_D / 2.0
        assert web >= 0.5, f"access hole L{k}: only {web:.2f} mm to the ring"
        rho = np.hypot(hx, hy)
        # the hole must stay clear of the hatch opening AND outboard of
        # the hatch lid's az-30 flat (extent = HATCH_APO there)
        assert rho - ACCESS_HOLE_D / 2.0 - HATCH_OPEN_APO >= 1.0, \
            f"access hole L{k} breaks into the hatch opening"
        assert rho - ACCESS_HOLE_D / 2.0 - HATCH_APO >= 1.0, \
            f"access hole L{k} covered by the hatch lid"
    # hatch screw geometry at the opening's vertex direction: the
    # insert bore must keep a real wall to the opening corner, and the
    # lid ear must cover whatever the bare hex corner does not
    assert HATCH_SCREW_RHO - INSERT_BORE_OD / 2.0 - open_vertex_r >= 1.5, \
        "frame insert bore too close to the opening corner"
    # the ear is centred on the hole, so a >= 2.5 mm annulus covers the
    # hole even where the bare lid corner (78.52) falls short of rho 78
    assert HATCH_EAR_OD / 2.0 - HOLE_D / 2.0 >= 2.5, \
        "lid ear annulus under 2.5 mm around the screw hole"
    # frame insert boss: solid at mid-boss beside the bore, air on the
    # opening side of the trim wall (probe the real solid)
    frame = meshes["chassis_top_rigid"]
    bz = SHEET_Z0 - FRAME_BOSS_H / 2.0
    p_solid = [[(HATCH_SCREW_RHO + 3.2) * np.cos(np.deg2rad(az)),
                (HATCH_SCREW_RHO + 3.2) * np.sin(np.deg2rad(az)), bz]
               for az in range(0, 360, 60)]
    p_air = [[(open_vertex_r - 0.6) * np.cos(np.deg2rad(az)),
              (open_vertex_r - 0.6) * np.sin(np.deg2rad(az)), bz]
             for az in range(0, 360, 60)]
    assert frame.contains(np.asarray(p_solid)).all(), \
        "frame insert boss missing under a hatch screw"
    assert not frame.contains(np.asarray(p_air)).any(), \
        "frame insert boss pokes past the opening wall (lip collision)"
    print(f"  hatch screws: 6x M3x16 into Phi {INSERT_BORE_OD:g} insert "
          f"bores in Phi {FRAME_BOSS_OD:g} under-frame bosses "
          f"(bore wall to opening {HATCH_SCREW_RHO - INSERT_BORE_OD / 2.0 - open_vertex_r:.2f} mm)")
    # A Phi 6.5 driver shaft dropped through the L0 hole down to the cap
    # face must touch nothing (leg at yaw 0) -- the screw head sits in the
    # cap flange counterbore right below.  The HATCH must not cover it.
    hx, hy = holes[0]
    shaft = _cyl_z(3.25, CAP_FACE_W + 0.05, SHEET_Z1 + 30.0, x=hx, y=hy,
                   sections=48)
    for key, frame in (("coxa_link", "coxa"), ("servo_body", "hip_cap"),
                       ("hip_clamp_cap_rigid", "hip_cap"),
                       ("bearing_6805", "hip_cap")):
        m = meshes[key].copy()
        m.apply_transform(leg_transforms(0)[frame])
        v = _inter_vol(shaft, m)
        assert v < 1e-6, f"driver shaft fouls {key} ({v:.2f} mm3)"
    for key in ("chassis_top_rigid", "top_hatch_rigid"):
        v = _inter_vol(shaft, meshes[key])
        assert v < 1e-6, f"driver shaft fouls {key} ({v:.2f} mm3)"
    print(f"  cap-bolt driver access: 6 holes Phi {ACCESS_HOLE_D:g}, "
          f"ring web {web:.2f} mm, line of sight clear (hatch ON)")


def check_bottom_joint(meshes: dict[str, trimesh.Trimesh]) -> None:
    """ONE deck-seated bottom bearing, NO cap (user, Aug 24 + Aug 25
    drop): the race sits RACE_DECK_CL over the servo-mount deck / yaw
    servo case top on the new Phi 34/Phi 37.15 ledge at world 10.75,
    with only the Phi 25.15 boss press touching the coxa, the ledge is
    solid at every azimuth with air above it inside the pocket bore
    and air below the inner-race band, the rebuilt tower wraps the
    FULL race width and stops exactly at the race-top plane (the
    Phi 44 column ends at the bearing top -- nothing continues it,
    and nothing remains of the OLD 15.75 seat), the hub's Phi 29 ring
    lands exactly on the race top, the Phi 38 dust brim hovers its
    BRIM_GAP over rim and race without touching either, and with the
    plate off the leg + bearing lift straight out of the pocket (horn
    centre screw only)."""
    T = leg_transforms(0)
    br = meshes["yaw_bearing_upper"].copy()
    br.apply_transform(T["coxa"] @ _trans([0.0, 0.0, YAWBR_DROP]))
    seat_w = CHB_SEAT_W
    assert abs(seat_w - (hp.CHASSIS_YAW_OUTPUT_Z + BR_BOT_LOCAL)) < 1e-9
    assert abs(br.bounds[0][2] - seat_w) < 1e-3, \
        f"race bottom {br.bounds[0][2]:.2f} not on the tower seat {seat_w:.2f}"
    v_pocket = _inter_vol(br, meshes["chassis_bottom"])
    assert v_pocket < 1.0, f"race overlaps the tower pocket ({v_pocket:.2f} mm3)"

    # the race floor: RACE_DECK_CL over the deck top AND the seated yaw
    # servo's case top (the two planes coincide by construction; probe
    # the real servo mesh so a future servo-model edit cannot silently
    # eat the running gap under the rotating inner race)
    ys = meshes["servo_body"].copy()
    ys.apply_transform(T["coxa"] @ _trans([-hp.SERVO_OUTPUT_X, 0.0,
                                           -(hp.HORN_STACK_H
                                             + hp.WELL_RIM_Z)]))
    case_top = float(ys.bounds[1][2])
    assert br.bounds[0][2] - case_top >= RACE_DECK_CL - 1e-3, \
        f"race bottom only {br.bounds[0][2] - case_top:.2f} over the case top"
    assert abs(CHB_DECK_TOP - case_top) < 0.05, \
        "deck top and yaw servo case top no longer coincide"

    coxa = meshes["coxa_link"].copy()
    coxa.apply_transform(T["coxa"])
    v_press = _inter_vol(coxa, br)
    assert v_press < 60.0, \
        f"coxa vs seated race: {v_press:.1f} mm3 (want the boss press only)"
    v = _inter_vol(coxa, meshes["chassis_bottom"])
    assert v < 1e-6, f"ringed coxa overlaps chassis_bottom ({v:.2f} mm3)"

    # the seat ring lands ON the race top (kiss): material just above it
    # on the inner-race land, none below it
    ax, ay = T["coxa"][:2, 3]
    race_top = seat_w + hp.YAW_BEARING_W
    r_mid = (HUB_RING_ID + HUB_RING_OD) / 4.0
    got = coxa.contains(np.array([[ax + r_mid, ay, race_top + 0.3],
                                  [ax + r_mid, ay, race_top - 0.3]]))
    assert got[0] and not got[1], "hub seat ring does not land on the race top"

    # full-wrap tower: the rim IS the race-top plane -- wall solid just
    # below it and through the whole rebuilt band, open sky just above
    # it (at the tower radius: neither chassis nor coxa continues the
    # Phi 44 column past the bearing), NOTHING left of the old tower
    # band above the new rim, and the brim constants keep their
    # derivations
    assert abs(CHB_RIM_W - race_top) < 1e-9, "tower rim is not the race top"
    assert BRIM_BOT_Z - TOWER_RIM_Z == BRIM_GAP
    assert BRIM_OD / 2.0 - hp.YAW_BEARING_OD / 2.0 == BRIM_GAP
    assert BRIM_OD / 2.0 <= CHB_TOWER_R - 2.0, "brim reads as chassis column"
    r_wall = (POCKET_BORE / 2.0 + CHB_TOWER_R) / 2.0          # mid rim wall
    probes = np.array([
        [ax + r_wall, ay, CHB_RIM_W - 0.3],          # in the rim wall top
        [ax + r_wall, ay, CHB_RIM_W + 0.3],          # air above the rim
        [ax + r_wall, ay, (CHB_SEAT_W + CHB_RIM_W) / 2.0],   # mid rebuilt band
        [ax + r_wall, ay, CHB_RIM_OLD_W + 0.3],      # air: old tower top gone
    ])
    in_ch = meshes["chassis_bottom"].contains(probes)
    assert in_ch[0], "tower does not wrap the full race width"
    assert not in_ch[1], "tower rises past the race top"
    assert in_ch[2], "rebuilt ring did not fuse across the pocket band"
    assert not in_ch[3], "old tower band survives above the new rim"
    assert not coxa.contains(probes[1:2])[0], \
        "coxa continues the tower column above the rim (curtain back?)"

    # the new LEDGE: face solid just below the seat plane all the way
    # around (and supported by the deck below), air just above it
    # inside the pocket bore, air below the inner-race / seal band
    # (the Phi 34 relief keeps the rotating race bottom over open air
    # down to the deck), and the OLD 15.75 seat fully bored out
    ang = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
    r_ledge = (hp.YAW_TOWER_SHOULDER_OD / 2.0 + POCKET_BORE / 2.0) / 2.0
    ring = lambda r, z: np.column_stack([ax + r * np.cos(ang),
                                         ay + r * np.sin(ang),
                                         np.full_like(ang, z)])
    ch = meshes["chassis_bottom"]
    assert ch.contains(ring(r_ledge, seat_w - 0.25)).all(), \
        "ledge face not solid all the way around"
    assert ch.contains(ring(r_ledge, CHB_DECK_TOP - 0.5)).all(), \
        "ledge not supported by the deck below"
    assert not ch.contains(ring(r_ledge, seat_w + 0.3)).any(), \
        "material above the ledge inside the pocket bore"
    assert not ch.contains(ring(15.5, (CHB_DECK_TOP + seat_w) / 2.0)).any(), \
        "material under the inner-race band above the deck"
    old_seat_w = hp.CHASSIS_YAW_OUTPUT_Z + hp.YAW_BEARING_LOWER_BOT_Z
    assert not ch.contains(ring(r_ledge, old_seat_w - 0.3)).any(), \
        "the old 15.75 seat / Phi 34 shoulder wall survives in the pocket"
    # dust brim: present over the seal band, hovering BRIM_GAP over the
    # rim/race plane, gap open where brim and rim overlap radially
    r_brim = (HUB_RING_OD / 2.0 + BRIM_OD / 2.0) / 2.0        # mid brim
    r_gap = (POCKET_BORE / 2.0 + BRIM_OD / 2.0) / 2.0         # overlap band
    brim_pts = np.array([
        [ax + r_brim, ay,
         hp.CHASSIS_YAW_OUTPUT_Z + (BRIM_BOT_Z + BRIM_TOP_Z) / 2.0],
        [ax + r_gap, ay, CHB_RIM_W + BRIM_GAP / 2.0],
    ])
    assert coxa.contains(brim_pts[:1])[0], "dust brim missing"
    in_cx = coxa.contains(brim_pts[1:])[0]
    in_ch = meshes["chassis_bottom"].contains(brim_pts[1:])[0]
    assert not in_cx and not in_ch, "brim running gap is not open"

    # service: plate off -> leg + bearing lift straight out of the pocket
    for dz in (2.0, 5.0, 12.0, 30.0):
        for name, m in (("coxa", coxa), ("race", br)):
            mm = m.copy()
            mm.apply_translation([0.0, 0.0, dz])
            v = _inter_vol(mm, meshes["chassis_bottom"])
            assert v < 1e-6, \
                f"lift +{dz}: {name} fouls chassis_bottom ({v:.1f} mm3)"
    print(f"  bottom joint: race on the DECK ledge (world z {seat_w:.2f}, "
          f"{seat_w - case_top:.2f} over the servo case top), NO cap; "
          f"coxa/race contact = boss press ({v_press:.1f} mm3), "
          f"rim at the race top ({CHB_RIM_W:.2f}, full 7 mm wrap), "
          f"Phi {BRIM_OD:g} brim hovers {BRIM_GAP:g} above, "
          f"leg + bearing lift straight out")


def check_coxa_column(meshes: dict[str, trimesh.Trimesh]) -> None:
    """The SHORTENED coxa (user, Aug 24 rev 3/4 + Aug 25 M3x20): the
    cradle floor sits exactly COL_HEAD_CL over the M3x20 horn-screw
    heads, the seats moved down by EXACTLY the M3x30 -> M3x20 length
    delta (so the tips -- and the thread engagement in the horn --
    are the bench-tuned production planes), the screw shanks pass the
    dropped slab, the deleted skirt/platform left nothing below the
    slab outside the seat ring + brim, and the hip axis (and with it
    the seated servo) dropped by exactly COL_DROP."""
    coxa = meshes["coxa_link"]

    # constants: floor derivation and head clearance (guards a future
    # constant edit from silently burying the screw heads)
    assert abs((hp.YAW_HUB_PLATFORM_Z1 + hp.COXA_WELL_FLOOR_LIFT)
               - COXA_FLOOR_Z - COL_DROP) < 1e-9
    head_top = HORN_HEAD_SEAT_Z + hp.INSERT_M3_BOLT_HEAD_H
    assert COXA_FLOOR_Z - head_top >= 1.2, \
        f"only {COXA_FLOOR_Z - head_top:.2f} mm over the M3 heads"

    # thread engagement preserved: the seat plane must drop 1:1 with
    # the screw length so tip = seat - length never moves off the
    # production plane (corner tips at the disc bottom + TIP_POKE,
    # centre at its spline-bore depth).  Guards a future seat/length
    # edit from silently shortening the horn engagement.
    assert abs((hp.YAW_HUB_HORN_HEAD_SEAT_Z - HORN_HEAD_SEAT_Z)
               - (hp.YAW_HUB_HORN_BOLT_LEN - HORN_BOLT_LEN)) < 1e-9, \
        "horn-screw tip plane moved: seat drop != screw length delta"
    assert abs((hp.YAW_HUB_HORN_CENTRE_SEAT_Z - HORN_CENTRE_SEAT_Z)
               - (hp.YAW_HUB_HORN_BOLT_LEN - HORN_BOLT_LEN)) < 1e-9, \
        "centre-screw tip plane moved: seat drop != screw length delta"

    # seat planes: solid seat annulus just under each seat plane,
    # shaft void just above it (probe between the Phi 3.7 shank and
    # the Phi 5.9 head shaft)
    drive_r = hp.DISC_HORN_BOLT_PCD / 2.0
    stations = [(0.0, 0.0, HORN_CENTRE_SEAT_Z)]
    stations += [(drive_r * np.cos(t), drive_r * np.sin(t),
                  HORN_HEAD_SEAT_Z)
                 for t in hp.DISC_HORN_BOLT_ANGLES_RAD]
    for (sx, sy, seat_z) in stations:
        n = np.hypot(sx, sy)
        ux, uy = (sx / n, sy / n) if n > 1e-9 else (1.0, 0.0)
        px, py = sx + 2.4 * ux, sy + 2.4 * uy
        below, above = coxa.contains(np.array([[px, py, seat_z - 0.3],
                                               [px, py, seat_z + 0.3]]))
        assert below and not above, \
            f"horn-screw seat at ({sx:.1f},{sy:.1f}) not on z {seat_z}"
        # shank passage open through the dropped slab band
        assert not coxa.contains(
            np.array([[sx, sy, SLAB_BOT_Z + 1.5]]))[0], \
            f"screw shank blocked in the slab at ({sx:.1f},{sy:.1f})"

    # well floor at COXA_FLOOR_Z: the seated servo's belly IS the floor
    T = leg_transforms(0)
    sv = meshes["servo_body"].copy()
    sv.apply_transform(T["hip_cap"])
    floor_w = hp.CHASSIS_YAW_OUTPUT_Z + COXA_FLOOR_Z
    assert abs(sv.bounds[0][2] - floor_w) < 0.05, \
        f"seated servo belly {sv.bounds[0][2]:.2f} != floor {floor_w:.2f}"

    # deleted skirt/platform: below the dropped slab, nothing reaches
    # past the brim radius (the column is boss + seat ring + brim only)
    v = coxa.vertices
    low = v[v[:, 2] < SLAB_BOT_Z - 0.01]
    r_low = float(np.hypot(low[:, 0], low[:, 1]).max())
    assert r_low <= BRIM_OD / 2.0 + 0.05, \
        f"material at r {r_low:.2f} below the slab (skirt/platform back?)"

    # slab bottom (rotating) vs the static tower rim: the M3x20 drop
    # spends the margin down to the design value of 1.5 mm
    # (>= 0.5 rotating-vs-static required; M3x16 would go NEGATIVE,
    # which is why 20 is the floor -- see the HORN SCREWS constants).
    # ROT_BAND_Z0 (used by the layout constants above) must stay a
    # true lower bound on this rotating band.
    slab_bot_w = hp.CHASSIS_YAW_OUTPUT_Z + SLAB_BOT_Z
    assert slab_bot_w - CHB_RIM_W >= 1.5 - 1e-9, \
        f"slab bottom only {slab_bot_w - CHB_RIM_W:.2f} mm over the rim"
    assert slab_bot_w >= ROT_BAND_Z0, \
        f"slab bottom {slab_bot_w:.2f} fell below ROT_BAND_Z0 {ROT_BAND_Z0:g}"

    print(f"  coxa column: floor z {COXA_FLOOR_Z:g} "
          f"({COXA_FLOOR_Z - head_top:g} mm over the M3x{HORN_BOLT_LEN:g} "
          f"heads), seats at {HORN_HEAD_SEAT_Z:g}/{HORN_CENTRE_SEAT_Z:g} "
          f"(production tips kept: seat drop {HORN_SEAT_DROP:g} = length "
          f"delta), cradle dropped {COL_DROP:g} mm (hip axis world "
          f"{hp.CHASSIS_YAW_OUTPUT_Z + COXA_HIP_DROP_V:g}), "
          f"below-slab column max r {r_low:.1f}, "
          f"slab {slab_bot_w - CHB_RIM_W:g} over the tower rim")


def check_chassis_variant(meshes: dict[str, trimesh.Trimesh]) -> None:
    """chassis_bottom_rigid edits landed as designed: below the platform
    top NOTHING outboard of the hex edge survives past the tower
    cylinder (the corner trim leaves one matching curve), the trim
    never bit the tower wall, above the servo-mount deck nothing pokes
    past the tower cylinder at ANY azimuth all the way to the rim
    (all three dead ears cut, the az-210 root left merged below
    the deck), the tower flank is CYLINDRICAL from the sheet top to
    the rim on all six legs (rev 7: the swing-relief protect-ring
    bump at z 6.25..10.25 is shaved), all six towers carry the rebuilt full-wrap ring to the
    LOWERED race-top plane (Aug 25: rim at world 17.75, pocket seat
    ledge at 10.75 on the deck), the six Wago tray wall
    sets are GONE above the sheet with the sheet still solid
    underneath, and the wire-corridor band PLUS the cradle-shell run
    (side walls + deck-skin roof) around each yaw servo is FLATTENED
    to the bare sheet all the way to the tower keep cylinder
    (rev 5+6) with the sheet-level well and the in-keep shell (well
    collar, pocket floor, 6805-seat support) still standing.
    (The pocket seat ledge itself is verified in check_bottom_joint;
    retainer territory is re-verified against THIS mesh by the other
    checks.)

    Rev 8 adds the ABOVE-SHEET WHITELIST assert: no vertex above
    z 2.05 may lie outside every tower cylinder (CHB_TRIM_R +
    CHB_WL_TOL about the six yaw axes) -- the deck-plateau and mates
    whitelist categories measured degenerate/empty, so the towers ARE
    the whitelist (see the CHB_WL_* constant block)."""
    cb = meshes["chassis_bottom"]
    assert cb.body_count == 1, "chassis variant not a single body"

    # rev-8 whitelist: above the bare sheet, towers only -- rejects any
    # L-bracket / diagonal-wall / lip leftover at ANY azimuth, named or
    # not (this is the assert that caught the STEP sidecar's stale
    # two-bay wago tray walls)
    n_bad, worst_r = chassis_whitelist_violations(cb)
    assert n_bad == 0, (
        f"{n_bad} vertices above the sheet outside the tower whitelist "
        f"(worst r {worst_r:.2f} from the nearest yaw axis) -- something "
        f"is standing on the plate again")
    v = trimesh.transform_points(cb.vertices, _rotz(-0.5 * np.pi / 3.0))
    near = v[(np.abs(v[:, 1]) < 40.0) & (v[:, 0] > 55.0)]
    ax = APOTHEM

    # corner trim: in the platform + skirt band, every vertex outboard
    # of the hex edge (within the platform's own half-width -- the
    # diagonal well walls at |y| ~ 25 are untouched production) lies
    # on/inside the tower cylinder: the tower's curve is the ONLY
    # silhouette curve out there
    band = near[(near[:, 2] > -6.4) & (near[:, 2] < CHB_PLATE_TOP - 0.01)
                & (near[:, 0] > ax + 0.1) & (np.abs(near[:, 1]) < 21.4)]
    r_out = np.hypot(band[:, 0] - ax, band[:, 1])
    r_max = float(r_out.max())
    assert r_max <= CHB_TRIM_R + 0.03, \
        f"corner material at r {r_max:.2f} > tower cylinder -- trim missed"

    # tower wall unbitten by the corner trim: full Phi 44 ring solid
    # mid-way up the rebuilt pocket band
    ang = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False)
    ring = np.column_stack([ax + (CHB_TOWER_R - 0.6) * np.cos(ang),
                            (CHB_TOWER_R - 0.6) * np.sin(ang),
                            np.full(24, (CHB_SEAT_W + CHB_RIM_W) / 2.0)])
    Rz = _rotz(0.5 * np.pi / 3.0)
    got = cb.contains(trimesh.transform_points(ring, Rz))
    assert got.all(), "tower wall bitten by a variant cut"

    # rebuilt ring present all the way around on ALL SIX towers (wall
    # solid just under the race-top rim, air just above it)
    for i in range(6):
        Ri = _rotz((i + 0.5) * np.pi / 3.0)
        r_wall = (hp.YAW_TOWER_BORE_OD / 2.0 + CHB_TOWER_R) / 2.0
        rim_ring = np.column_stack([ax + r_wall * np.cos(ang),
                                    r_wall * np.sin(ang),
                                    np.full(24, CHB_RIM_W - 0.3)])
        assert cb.contains(trimesh.transform_points(rim_ring, Ri)).all(), \
            f"leg {i}: rebuilt tower ring incomplete"
        above = rim_ring + [0.0, 0.0, 0.6]
        assert not cb.contains(trimesh.transform_points(above, Ri)).any(), \
            f"leg {i}: material above the race-top rim"

    # ears: above the servo-mount deck (z 10.25; ears ran to 19.75),
    # NOTHING pokes past the tower cylinder at any azimuth, all the
    # way up to the rim -- az 330/90 were shaved before, the az-210
    # column is cut to the deck, and the rebuilt ring itself stays at
    # the tower radius (the old tower band above the rim is gone, so
    # the band cap is the rim itself)
    band = near[(near[:, 2] > CHB_DECK_TOP + 0.1)
                & (near[:, 2] < CHB_RIM_W + 0.1)]
    d = band[:, :2] - [ax, 0.0]
    rr = np.hypot(d[:, 0], d[:, 1])
    n_out = int((rr > CHB_TOWER_R + 0.05).sum())
    assert n_out == 0, \
        f"{n_out} vertices past the tower cylinder above the deck (an ear " \
        f"cut missed)"

    # tower flank CYLINDRICAL sheet-top -> rim on all six legs (rev 7):
    # within 45 mm of each yaw axis, NO vertex between the sheet top and
    # the rim lies outside the trim cylinder -- this is exactly the band
    # where the production swing-relief protect ring (r 23.5) used to
    # bulge the outboard flank at z 6.25..10.25.  (Nothing else of the
    # chassis lives that close to a tower above the sheet: neighbour
    # towers are 100 mm away, the flatten band is air, trays deleted.)
    for i in range(6):
        vi = trimesh.transform_points(cb.vertices,
                                      _rotz(-(i + 0.5) * np.pi / 3.0))
        di = vi[:, :2] - [ax, 0.0]
        ri = np.hypot(di[:, 0], di[:, 1])
        sel = ((vi[:, 2] > CHB_FLAT_Z0 + 0.1) & (vi[:, 2] < CHB_RIM_W + 0.1)
               & (ri < 45.0))
        r_hi = float(ri[sel].max())
        assert r_hi <= CHB_TRIM_R + 0.03, (
            f"leg {i}: tower flank not cylindrical -- vertex at r {r_hi:.2f}"
            f" in the sheet->rim band (the rev-7 bump shave missed)")

    # az-210 ear spot: the boss column is gone above the deck, and since
    # rev 6 the below-deck root is gone too -- the ear centre sits at
    # r 23.5 from the yaw axis, OUTSIDE the keep cylinder, so the
    # cradle-shell flatten takes the roof band it was merged into
    # (flat to the sheet; the sheet itself must stay solid)
    ear_c = np.array([ax + 23.5 * np.cos(np.pi * 7.0 / 6.0),
                      23.5 * np.sin(np.pi * 7.0 / 6.0)])
    assert np.hypot(ear_c[0] - ax, ear_c[1]) > CHB_KEEP_R  # outside keep
    probes = trimesh.transform_points(
        np.array([[ear_c[0], ear_c[1], CHB_DECK_TOP - 0.5],
                  [ear_c[0], ear_c[1], CHB_DECK_TOP + 0.5]]),
        _rotz(0.5 * np.pi / 3.0))
    root_in, above_in = cb.contains(probes)
    assert not root_in, "az-210 ear root survived the rev-6 shell flatten"
    assert not above_in, "az-210 ear column survived above the deck"
    # (the sheet below is open here anyway: |y| 11.75 is inside the
    # sheet-level well that registers the servo case)

    # wago trays gone: probe mid-height of all three wall positions of
    # every corner tray (must be air) and the sheet just below each
    # (must still be solid -- the cut may not bite the plate)
    t = hp.WAGO_MOUNT_WALL_T
    gone, under = [], []
    for M in hp.wago_tray_corner_transforms():
        for lx, ly in ((TRAY_HALF_X - t / 2.0, 0.0),
                       (0.0, +(TRAY_HALF_Y - t / 2.0)),
                       (0.0, -(TRAY_HALF_Y - t / 2.0))):
            p = np.array([lx, ly, 0.0, 1.0])
            gone.append((M @ (p + [0, 0, hp.WAGO_MOUNT_WALL_H / 2.0, 0]))[:3])
            under.append((M @ (p + [0, 0, -1.0, 0]))[:3])
    assert not cb.contains(np.array(gone)).any(), "a wago tray wall survived"
    assert cb.contains(np.array(under)).all(), \
        "tray delete cut bit into the sheet"

    # corridor + cradle shell flattened (rev 5+6): the ENTIRE cut band
    # -- x 50 to the tower keep cylinder, |y| <= 20.5 -- is air on
    # every leg (grid sample, keep circle excluded with margin);
    # everything below the sheet-top plane is untouched (probed at
    # spots the pre-cut solid owned, including the sheet the shell
    # walls used to stand on -- note the corridor itself was already
    # a floorless tunnel in production); and the structure that DOES
    # do a job still stands: the shell inside the keep cylinder (the
    # 6805 seat's inboard-arc support + well-mouth collar) and the
    # pocket-floor deck inside the bore.
    gx = np.arange(CHB_FLAT_X0 + 0.5, CHB_FLAT_X1 - 0.4, 2.0)
    gy = np.arange(-CHB_FLAT_HALF_Y + 0.5, CHB_FLAT_HALF_Y - 0.4, 2.5)
    gz = (3.0, 6.0, 9.5)
    grid = np.array([[x, y, z] for x in gx for y in gy for z in gz
                     if np.hypot(x - APOTHEM, y) > CHB_KEEP_R + 0.75])
    below_keep = np.array([
        [56.0, 19.0, 1.0],    # bare sheet at the band edge (inboard of
        [56.0, -19.0, 1.0],   # x 52 the trunk pass opens on legs 0/5)
        [55.0, 15.0, 0.5],    # old side-wall roots below the flatten plane
        [55.0, -15.0, 0.5],
        [70.0, 16.0, 1.0],    # sheet the cradle-shell walls stood on
        [70.0, -16.0, 1.0],   # (rev 6 zone -- cut must stop at z 2)
        [84.0, 16.5, 1.0],
        [84.0, -16.5, 1.0],
        [63.0, 0.0, -3.0],    # belly band under the old wire notch
    ])
    keep_local = np.array([
        [90.0, 18.0, 5.0],    # shell wall INSIDE the tower keep cylinder
        [90.0, -18.0, 5.0],   # (r 20.6 < CHB_KEEP_R about the yaw axis)
        [81.0, 0.0, 9.5],     # servo-tunnel roof skin under the ring wall
        [96.0, 12.0, 9.5],    # pocket-floor deck inside the bore
    ])
    for i in range(6):
        Ri = _rotz((i + 0.5) * np.pi / 3.0)
        band = trimesh.transform_points(grid, Ri)
        n_in = int(cb.contains(band).sum())
        assert n_in == 0, \
            f"leg {i}: {n_in} flatten-band points still solid (cut missed)"
        assert cb.contains(trimesh.transform_points(below_keep, Ri)).all(), \
            f"leg {i}: flatten cut bit below the sheet-top plane"
        assert cb.contains(trimesh.transform_points(keep_local, Ri)).all(), \
            f"leg {i}: flatten cut removed keep-cylinder structure"

    print(f"  chassis variant: tower bases trimmed to one r {CHB_TRIM_R:g} "
          f"cylinder (outboard max r {r_max:.2f}), flanks cylindrical "
          f"sheet->rim (rev-7 bump shave), towers rebuilt to the "
          f"LOWERED race top (world {CHB_RIM_W:g}), all 3 dead ears shaved "
          f"(az 210 flush to the deck), "
          f"6 wago trays deleted, corridors + cradle shells flattened "
          f"(x {CHB_FLAT_X0:g}..{CHB_FLAT_X1:g}, |y| {CHB_FLAT_HALF_Y:g}, "
          f"tower keep r {CHB_KEEP_R:g} kept), above-sheet whitelist "
          f"clean (towers only, 0 vertices outside), "
          f"{abs(cb.volume) / 1000.0:.0f} cm3")


def check_rot_envelope(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Rotating parts stay inside the 38.2 mm yaw envelope.  The pillars
    this once cleared are deleted (user, Aug 26), but the coxa keeps its
    envelope trim and this assert keeps the guarantee honest for any
    future rim furniture."""
    r_coxa = float(np.linalg.norm(
        meshes["coxa_link"].vertices[:, :2], axis=1).max())
    assert r_coxa <= ROT_ENVELOPE_R + 0.05, \
        f"rounded coxa reaches {r_coxa:.2f} > envelope {ROT_ENVELOPE_R}"
    vc = meshes["hip_clamp_cap_rigid"].vertices
    r_cap = float(np.hypot(vc[:, 0], vc[:, 2] - AXIS_Z).max())
    assert r_cap <= ROT_ENVELOPE_R + 0.05, \
        f"hip cap reaches {r_cap:.2f} > envelope {ROT_ENVELOPE_R}"
    print(f"  rotating envelope: coxa/cap {r_coxa:.2f}/{r_cap:.2f} <= "
          f"{ROT_ENVELOPE_R:g} (kept after the pillar delete)")


def _bounds_touch(a: trimesh.Trimesh, b: trimesh.Trimesh) -> bool:
    return bool(np.all(a.bounds[0] <= b.bounds[1] + 1.0)
                and np.all(b.bounds[0] <= a.bounds[1] + 1.0))


def check_wago_block(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Central splice block: printable, sits on solid sheet, fits its
    territory (strap slots / standoffs / trunk pass), whole footprint
    under the open hatch, and the nut stack -- even with a lever
    flipped up -- stays RADIALLY clear of every yaw joint's rotating
    envelope (the M3x20 slab drop put the rotating band at world
    19.25, below the flipped levers, so the old axial argument is
    gone; the levers clear because they are nowhere near a leg)."""
    blk = meshes["centre_wago_block"]
    assert blk.is_watertight, "wago block not watertight"
    assert blk.body_count == 1, "wago block not a single body"

    # nothing on the chassis floor where it sits (walls, pads, ports)
    inter = trimesh.boolean.intersection(
        [blk, meshes["chassis_bottom"]], engine="manifold")
    v = 0.0 if inter.is_empty else abs(inter.volume)
    assert v < 1.0, f"wago block overlaps chassis_bottom ({v:.2f} mm3)"

    # solid sheet under the full footprint (VHB backing, no slot bridged)
    xs = np.linspace(-WBLK_HALF_X + 1, WBLK_HALF_X - 1, 23)
    ys = np.linspace(-WBLK_HALF_Y + 1, WBLK_HALF_Y - 1, 15)
    XX, YY = np.meshgrid(xs, ys)
    pts = np.column_stack([XX.ravel(), YY.ravel(),
                           np.full(XX.size, BOT_SHEET_TOP_Z - 1.0)])
    frac = meshes["chassis_bottom"].contains(pts).mean()
    assert frac > 0.999, f"floor under wago block only {frac:.1%} solid"

    # territory: strap slots (inner edge y=24), standoff posts, trunk pass
    slot_y0 = hp.BATTERY_STRAP_SLOT_Y - 2.0                    # 24
    assert WBLK_HALF_Y + 1.5 <= slot_y0, "block reaches the strap slots"
    assert WBLK_HALF_Y + 6.0 <= abs(hp.CHASSIS_STANDOFF_HOLES_XY[0][1]), \
        "block reaches the standoff posts"
    assert WBLK_HALF_X + 2.0 <= (hp.BATTERY_TRUNK_HOLE_CENTRE[0]
                                 - hp.BATTERY_TRUNK_HOLE_X / 2.0), \
        "block reaches the battery trunk pass"

    # fully under the open hatch; the flipped-up levers (top z ~22.4)
    # now sit ABOVE the rotating band floor (slab bottom, world 19.25)
    # but clear every yaw envelope RADIALLY: each yaw axis is APOTHEM
    # from centre and its rotating parts reach at most ROT_ENVELOPE_R
    # inboard, so anything within APOTHEM - ROT_ENVELOPE_R of centre
    # is untouchable at ANY height
    corner_r = float(np.hypot(WBLK_HALF_X, WBLK_HALF_Y))
    assert corner_r <= HATCH_OPEN_APO - 3.0, \
        f"block corner r {corner_r:.1f} not under the hatch opening"
    lever_top = BOT_SHEET_TOP_Z + WBLK_FLOOR_T + hp.WAGO5_H + 10.0
    rot_inboard_r = APOTHEM - ROT_ENVELOPE_R                  # 61.8
    lever_rad_cl = rot_inboard_r - corner_r
    assert lever_rad_cl >= ENV_MIN_CL, \
        f"block corner r {corner_r:.1f} within {lever_rad_cl:.1f} of a " \
        f"yaw rotating envelope (need >= {ENV_MIN_CL:g})"

    # seated nuts: the 0.15 press wedge is the ONLY block contact
    for M in _wago5_scene_frames():
        w = meshes["wago5"].copy()
        w.apply_transform(M)
        inter = trimesh.boolean.intersection([w, blk], engine="manifold")
        v = 0.0 if inter.is_empty else abs(inter.volume)
        assert v < 40.0, f"seated nut vs block: {v:.2f} mm3 (not a wedge)"
    print(f"  wago block: {abs(blk.volume) / 1000.0:.1f} cm3, "
          f"4 bays press-fit, corner r {corner_r:.1f} "
          f"under the {HATCH_OPEN_APO:g} mm opening, "
          f"lever top {lever_top:.1f} clears the yaw envelopes by "
          f"{lever_rad_cl:.1f} mm radial")


def check_hatch(meshes: dict[str, trimesh.Trimesh]) -> None:
    """The hatch must drop STRAIGHT DOWN into the seated frame (lip into
    the opening) with zero overlap, and its screw/standoff/elec holes
    must land on frame bosses / open interior respectively."""
    frame = meshes["chassis_top_rigid"]
    hatch = meshes["top_hatch_rigid"]
    v = _inter_vol(frame, hatch)
    assert v < 1e-6, f"hatch overlaps the frame when seated ({v:.2f} mm3)"
    for dz in (0.5, 2.0, 5.0, 20.0, 60.0):
        h = hatch.copy()
        h.apply_translation([0, 0, dz])
        v = _inter_vol(h, frame)
        assert v < 1e-6, f"hatch descent +{dz}: fouls the frame ({v:.1f} mm3)"
        for i in range(6):
            T = leg_transforms(i)
            for key, fr in (("hip_clamp_cap_rigid", "hip_cap"),
                            ("bearing_6805", "hip_cap"),
                            ("coxa_link", "coxa")):
                m = meshes[key].copy()
                m.apply_transform(T[fr])
                v = _inter_vol(h, m)
                assert v < 1e-6, \
                    f"hatch descent +{dz}: fouls L{i} {key} ({v:.1f} mm3)"
    print("  hatch: seats with zero overlap, straight drop-in verified")


def check_yaw_sweep(meshes: dict[str, trimesh.Trimesh]) -> None:
    """The coxa-mounted assembly must clear the plate at EVERY yaw angle.
    Analytic guard on all rotating vertices + boolean spot checks."""
    plate = meshes["chassis_top_rigid"]
    rotating = [("coxa_link", "coxa"), ("hip_clamp_cap_rigid", "hip_cap"),
                ("servo_body", "hip_cap")]
    axis_xy = leg_transforms(0)["coxa"][:2, 3]
    for key, frame in rotating:
        m = meshes[key].copy()
        m.apply_transform(leg_transforms(0)[frame])
        v = m.vertices
        rad = np.linalg.norm(v[:, :2] - axis_xy, axis=1)
        hi = v[:, 2] > RING_BOT_W - 0.5
        bad = hi & (rad > SHOULDER_OD / 2.0 - 1.0)
        assert not bad.any(), (
            f"{key}: {int(bad.sum())} vertices sweep into the plate "
            f"(z > {RING_BOT_W - 0.5:.2f} outside the shoulder bore)")
        # anything above the ring bottom must also keep the running air
        # gap under the dust roof -- only the boss tip may pass SHEET_Z0,
        # and even the tip must stay BR_ROOF_MIN_CL under the roof
        inbore = hi & (rad <= SHOULDER_OD / 2.0 - 1.0)
        if inbore.any():
            roof_bot = SHEET_Z1 - BR_ROOF_T
            top = v[inbore, 2].max()
            assert top <= roof_bot - BR_ROOF_MIN_CL + 1e-6, (
                f"{key}: rotating material at z {top:.2f} within "
                f"{BR_ROOF_MIN_CL:g} mm of the dust roof ({roof_bot:.2f})")
    for phi in (0.0, 90.0, 180.0, 270.0):
        T = leg_transforms(0, yaw_deg=phi)
        for key, frame in rotating:
            m = meshes[key].copy()
            m.apply_transform(T[frame])
            for pk in ("chassis_top_rigid", "top_hatch_rigid"):
                v = _inter_vol(m, meshes[pk])
                assert v < 1e-6, f"yaw {phi}: {key} hits {pk} ({v:.2f} mm3)"
    print("  yaw sweep (0..360): coxa assembly clears the plate + hatch")


def check_plate_descent(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Assembly: the plate must drop STRAIGHT DOWN onto all six seated
    bearings without fouling anything (races enter the pockets last)."""
    static = []
    for i in range(6):
        T = leg_transforms(i)
        for key, frame in (("coxa_link", "coxa"), ("servo_body", "hip_cap"),
                           ("hip_clamp_cap_rigid", "hip_cap"),
                           ("bearing_6805", "hip_cap")):
            m = meshes[key].copy()
            m.apply_transform(T[frame])
            static.append((f"L{i}-{key}", m))
    plate = meshes["chassis_top_rigid"]
    for dz in (0.5, 2.0, 5.0, 10.0, 25.0, 50.0):
        p = plate.copy()
        p.apply_translation([0, 0, dz])
        for name, m in static:
            v = _inter_vol(p, m)
            assert v < 1e-6, f"descent +{dz}: plate fouls {name} ({v:.1f} mm3)"
    print("  plate descent: clear drop onto all six bearings")


def sweep_femur_envelope(meshes: dict[str, trimesh.Trimesh],
                         yaw_grid=(-35.0, -20.0, 0.0, 20.0, 35.0),
                         p_hi: float = -90.0, step: float = 2.5):
    """Find, per yaw angle, the first femur UP angle that contacts the new
    top plate (or the pedestal/boss/bearing stack).  Production workspace
    is femur in [-80, +30]; this variant trades away the deep-tuck end."""
    plate = meshes["chassis_top_rigid"]
    tube, _ = _tibia_extras()
    moving_keys = (("femur_link", "femur_mh"), ("servo_body", "knee_cap"),
                   ("knee_clamp_cap", "knee_cap"),
                   ("tibia_knee_yoke", "tibia_mh"), ("tibia_tube", "tibia"))

    def moving(i, yaw, p):
        T = leg_transforms(i, yaw_deg=yaw, femur_deg=p)
        frames = {"femur_mh": T["femur"] @ MH, "knee_cap": T["knee_cap"],
                  "tibia_mh": T["tibia"] @ MH, "tibia": T["tibia"]}
        for key, fr in moving_keys:
            m = (tube if key == "tibia_tube" else meshes[key]).copy()
            m.apply_transform(frames[fr])
            yield key, m

    # (A) vs the yaw-rotating boss/bearing stack -- yaw-independent.
    stack = _union([meshes["hip_clamp_cap_rigid"], meshes["bearing_6805"]])
    stack.apply_transform(leg_transforms(0)["hip_cap"])
    contact_a = None
    for p in np.arange(0.0, p_hi - 1e-9, -step):
        if any(_inter_vol(m, stack) > 1.0 for _k, m in moving(0, 0.0, p)):
            contact_a = float(p)
            break
    # (B) vs the world-fixed plate + hatch, per yaw.
    hatch = meshes["top_hatch_rigid"]
    contact_b = {}
    for yaw in yaw_grid:
        contact_b[yaw] = None
        for p in np.arange(0.0, p_hi - 1e-9, -step):
            hit = False
            for _k, m in moving(0, yaw, p):
                if m.bounds[1][2] < RING_BOT_W:      # cheap z prefilter
                    continue
                if _inter_vol(m, plate) > 1.0 or _inter_vol(m, hatch) > 1.0:
                    hit = True
                    break
            if hit:
                contact_b[yaw] = float(p)
                break
    print(f"  femur-vs-boss/bearing first contact: {contact_a} deg")
    for yaw, c in contact_b.items():
        print(f"  femur-vs-plate first contact @ yaw {yaw:+.0f}: {c} deg")
    worst = [c for c in ([contact_a] + list(contact_b.values())) if c is not None]
    limit = (max(worst) + step + 2.5) if worst else p_hi  # margin: 1 grid + 2.5
    limit = float(np.ceil(limit / 2.5) * 2.5)
    print(f"  => safe femur UP limit (with margin): {limit} deg "
          f"(production workspace allowed -80)")
    assert limit <= -45.0, (
        f"rigid-hip plate limits femur up-swing to {limit} deg -- "
        "walking swing headroom (-45) is gone, redesign the stack heights")
    return limit, contact_a, contact_b


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------
COLORS = {
    "hip_clamp_cap_rigid": "#4878b0", "chassis_top_rigid": "#5b8fd4",
    "top_hatch_rigid": "#6fa8dc",
    "centre_wago_block": "#4a90a4", "wago5": "#d9822b",
    "bearing_6805": "#303030",
    "yaw_bearing_upper": "#3a3a3a", "servo_body": "#6b6b6b",
    "chassis_bottom": "#8b93a6", "coxa_link": "#7ba1d1",
    "femur_link": "#9aa0a6", "tibia_knee_yoke": "#9aa0a6",
    "knee_clamp_cap": "#9aa0a6",
    "yaw_servo_retainer": "#9aa0a6", "foot_boot": "#5a5f66",
    "tibia_tube": "#404040",
}
COTS = {"servo_body", "yaw_bearing_upper", "bearing_6805", "tibia_tube",
        "wago5"}


def _mat16(M: np.ndarray) -> list[float]:
    return [float(x) for x in np.asarray(M, float).T.reshape(-1)]


def build_scene(meshes, femur_up_limit: float) -> dict:
    _tube, foot_frame = _tibia_extras()
    mesh_defs = [{"id": f"stl:{k}", "name": fn, "url": f"stl/{fn}"}
                 for k, (_f, fn) in MESH_FILES.items()]
    mesh_defs.append({"id": "stl:tibia_tube", "name": "tibia_tube",
                      "url": "stl/tibia_tube_DO_NOT_PRINT.stl"})

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

    inst("chassis_bottom", "chassis_bottom RIGID (full-wrap towers, NEW)",
         np.eye(4))
    inst("chassis_top_rigid", "chassis_top_rigid FRAME (NEW)", np.eye(4))
    inst("top_hatch_rigid", "top_hatch (NEW, removable)", np.eye(4))
    inst("centre_wago_block", "central splice block (NEW)", np.eye(4))
    for k, (M, label) in enumerate(zip(_wago5_scene_frames(),
                                       ("V+ west", "V+ east",
                                        "GND west", "GND east"))):
        inst("wago5", f"221-415 {label}", M)
    for i in range(6):
        T = leg_transforms(i)
        a = (i + 0.5) * np.pi / 3.0
        axis_pt = [APOTHEM * np.cos(a), APOTHEM * np.sin(a),
                   hp.CHASSIS_YAW_OUTPUT_Z]
        pitch_ax = (_rotz(a)[:3, :3] @ np.array([0.0, 1.0, 0.0])).tolist()
        hip_pt = (T["coxa"] @ np.array([*COXA_HIP_ANCHOR_V, 1.0]))[:3]
        knee_pt = (T["femur"] @ np.array([hp.FEMUR_LENGTH, 0.0, 0.0, 1.0]))[:3]

        yaw_ids = [
            inst("coxa_link", f"L{i} coxa_link RIGID (NEW)",
                 T["coxa"], leg=i),
            inst("yaw_bearing_upper", f"L{i} yaw bearing (tower-seated)",
                 T["coxa"] @ _trans([0.0, 0.0, YAWBR_DROP]), leg=i),
            inst("servo_body", f"L{i} hip servo", T["hip_cap"], leg=i),
            inst("hip_clamp_cap_rigid", f"L{i} hip cap RIGID (NEW)",
                 T["hip_cap"], leg=i),
            inst("bearing_6805", f"L{i} third 6805 (NEW)", T["hip_cap"], leg=i),
        ]
        inst("yaw_servo_retainer", f"L{i} yaw retainer", T["cradle"], leg=i)
        # Yaw servo is CHASSIS-mounted (does not rotate with the leg), so it
        # is placed via the leg frame but NOT listed in the yaw joint.
        inst("servo_body", f"L{i} yaw servo",
             T["coxa"] @ _trans([-hp.SERVO_OUTPUT_X, 0.0,
                                 -(hp.HORN_STACK_H + hp.WELL_RIM_Z)]),
             leg=i)
        hip_ids = [
            inst("femur_link", f"L{i} femur_link", T["femur"] @ MH, leg=i),
            inst("servo_body", f"L{i} knee servo", T["knee_cap"], leg=i),
            inst("knee_clamp_cap", f"L{i} knee cap", T["knee_cap"], leg=i),
        ]
        knee_ids = [
            inst("tibia_knee_yoke", f"L{i} tibia yoke", T["tibia"] @ MH, leg=i),
            inst("tibia_tube", f"L{i} tibia tube", T["tibia"], leg=i),
            inst("foot_boot", f"L{i} foot boot", T["tibia"] @ foot_frame, leg=i),
        ]
        # BuildViz FK: joints compose up the ``parent`` chain and each
        # instance is driven by the deepest joint that lists it -- so the
        # per-link instance lists above only articulate correctly with the
        # yaw -> hip -> knee parent chain declared (same pattern as
        # tools/full_robot_viz_build.py).  Without ``parent`` the hip sweep
        # moved the femur but left the tibia yoke/tube/boot behind.
        joints += [
            {"id": f"L{i}-yaw", "type": "revolute", "axis": [0, 0, 1],
             "origin": axis_pt, "instances": yaw_ids,
             "limits": {"min": -35.0, "max": 35.0}, "home": 0,
             "label": f"L{i} yaw"},
            {"id": f"L{i}-hip", "type": "revolute", "axis": pitch_ax,
             "origin": [float(x) for x in hip_pt], "instances": hip_ids,
             "parent": f"L{i}-yaw",
             "limits": {"min": femur_up_limit, "max": 30.0}, "home": 0,
             "label": f"L{i} hip (up-limit {femur_up_limit:g} due to plate)"},
            {"id": f"L{i}-knee", "type": "revolute", "axis": pitch_ax,
             "origin": [float(x) for x in knee_pt], "instances": knee_ids,
             "parent": f"L{i}-hip",
             "limits": {"min": -30.0, "max": 20.0}, "home": 0,
             "label": f"L{i} knee"},
        ]

    up = femur_up_limit - hp.STANCE_FEMUR_DEG  # joint value at the limit
    scene = {
        "name": "sts3215 rigid-hip variant -- third 6805 above each hip",
        "source": "concepts/rigid_hip/make_rigid_hip_variant.py",
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": [0, 0, 55],
        "checksConfig": {
            "overlapMm3": 80.0, "pitchMm": 2.0,
            "ignoreOverlapPairs": [
                ["chassis_top_rigid", "top_hatch_rigid"],
                ["centre_wago_block", "wago5"],   # 0.15 press wedge
                ["chassis_bottom", "servo_body"],
                ["coxa_link", "servo_body"],
                ["coxa_link", "yaw_bearing_upper"],
                ["femur_link", "servo_body"],
                ["hip_clamp_cap_rigid", "servo_body"],
                ["knee_clamp_cap", "servo_body"],
                ["tibia_knee_yoke", "servo_body"],
                ["foot_boot", "tibia_tube"],
                ["tibia_knee_yoke", "tibia_tube"],
            ]},
        "meshes": mesh_defs,
        "instances": instances,
        "joints": joints,
        "poses": [
            {"id": "stance", "name": "Stance (walking)",
             "jointValues": {j["id"]: 0.0 for j in joints}},
            {"id": "femur-up-limit", "name":
                f"L0 femur at the NEW up limit ({femur_up_limit:g} deg abs)",
             "jointValues": {"L0-hip": up}},
        ],
        "animations": [
            {"id": "sweep", "name": "L0: yaw + femur sweep to the new limits",
             "loop": True, "duration": 8.0,
             "keyframes": [
                 {"t": 0.0, "jointValues": {"L0-yaw": 0, "L0-hip": 0}},
                 {"t": 1.5, "jointValues": {"L0-yaw": 35, "L0-hip": 0}},
                 {"t": 3.0, "jointValues": {"L0-yaw": -35, "L0-hip": 0}},
                 {"t": 4.0, "jointValues": {"L0-yaw": 0, "L0-hip": 0}},
                 {"t": 5.5, "jointValues": {"L0-yaw": 0, "L0-hip": up}},
                 {"t": 6.5, "jointValues": {"L0-yaw": 0, "L0-hip": up}},
                 {"t": 8.0, "jointValues": {"L0-yaw": 0, "L0-hip": 0}},
             ]},
        ],
    }
    return scene


def render_preview(meshes) -> None:
    """Radial cross-section through the leg-0 yaw axis: the money shot of
    cap -> pedestal -> boss -> bearing -> pocket -> plate."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    a = 0.5 * np.pi / 3.0
    Rz = _rotz(-a)  # leg-0 radial -> +x
    T = leg_transforms(0)
    sections = [
        ("chassis_top_rigid", np.eye(4), "#5b8fd4",
         "chassis_top_rigid frame (NEW)"),
        ("top_hatch_rigid", np.eye(4), "#6fa8dc", "service hatch (NEW)"),
        ("hip_clamp_cap_rigid", T["hip_cap"], "#4878b0",
         "hip cap + pedestal + boss (NEW)"),
        ("bearing_6805", T["hip_cap"], "#303030", "top 6805-2RS (NEW)"),
        ("coxa_link", T["coxa"], "#7ba1d1",
         "coxa_link RIGID (rounded + seat ring + dust brim, NEW)"),
        ("yaw_bearing_upper", T["coxa"] @ _trans([0.0, 0.0, YAWBR_DROP]),
         "#303030", "bottom 6805-2RS (fully housed, cap DELETED)"),
        ("chassis_bottom", np.eye(4), "#8b93a6",
         "chassis_bottom RIGID (full-wrap towers, NEW)"),
        ("servo_body", T["hip_cap"], "#c9a227", "hip servo"),
    ]
    fig, ax = plt.subplots(figsize=(10.5, 6.2), dpi=130)
    for key, M, color, label in sections:
        m = meshes[key].copy()
        m.apply_transform(Rz @ M)
        sec = m.section(plane_origin=[APOTHEM, 0, 0], plane_normal=[0, 1, 0])
        if sec is None:
            continue
        planar, _ = sec.to_2D(to_2D=rotation_matrix(-np.pi / 2.0, [1, 0, 0]))
        first = True
        for poly in planar.polygons_full:
            xs, ys = poly.exterior.xy
            ax.fill(xs, ys, color=color, alpha=0.6,
                    label=label if first else None)
            first = False
            for ring in poly.interiors:
                ax.fill(*ring.xy, color="white")
    for z, lab in ((CAP_FACE_W, "cap face"), (BR_BOT_W, "race seat"),
                   (SHEET_Z0, "shoulder / sheet bottom"),
                   (SHEET_Z1, "deck face")):
        ax.axhline(z, color="k", lw=0.5, ls=":")
        ax.annotate(f" {lab} z={z:.2f}", (APOTHEM + 42, z), fontsize=7,
                    va="bottom")
    ax.set_xlim(APOTHEM - 65, APOTHEM + 72)
    ax.set_ylim(0, 108)
    ax.set_aspect("equal")
    ax.set_xlabel("radial position from body centre [mm]")
    ax.set_ylabel("world Z [mm]")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_title("rigid-hip stack -- section through the leg-0 yaw axis")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "preview.png"))
    print("  wrote preview.png")


def main() -> None:
    skip_sweep = "--skip-sweep" in sys.argv
    skip_brep = "--skip-brep" in sys.argv
    if skip_brep:
        print("BREP EXPORT SKIPPED (--skip-brep): reusing existing step/stl/")
    else:
        print("exporting BREP geometry (build_rigid_hip_step.py) ...")
        subprocess.run(BREP_EXPORT_CMD, check=True)
    print("building meshes ...")
    meshes = build_meshes()

    print("static checks ...")
    check_static(meshes)
    check_bottom_joint(meshes)
    check_coxa_column(meshes)
    check_chassis_variant(meshes)
    check_rot_envelope(meshes)
    check_wago_block(meshes)
    check_hatch(meshes)
    check_yaw_sweep(meshes)
    check_plate_descent(meshes)

    if skip_sweep:
        limit = -55.0
        print("SWEEP SKIPPED (--skip-sweep): using placeholder limit -55")
    else:
        print("femur workspace sweep (this is the slow part) ...")
        limit, _ca, _cb = sweep_femur_envelope(meshes)

    print("writing scene.json ...")
    scene = build_scene(meshes, limit)
    with open(os.path.join(HERE, "scene.json"), "w", encoding="utf-8") as fh:
        json.dump(scene, fh, indent=1)

    render_preview(meshes)
    print("done.")


if __name__ == "__main__":
    main()
