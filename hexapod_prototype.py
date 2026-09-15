"""Hexapod walker — PROTOTYPE STL generator for a tabletop, hobby-servo build.

This is a scaled-down sibling of `hexapod_walker.py`.  Same architecture
(regular hex chassis, six identical 3-DOF legs, alternating-tripod
gait), but every dimension is shrunk ~ 6 x and every actuator pocket is
re-shaped for a generic 25 kg-cm hobby servo (DS3225 / MG996R class)
instead of an industrial harmonic-drive servomotor.

Why a separate file?
    The two designs share zero usable geometry: a hobby servo is a
    20 x 40 x 40 mm rectangular brick with two M3 mounting tabs and a
    splined output shaft on top, NOT a 170 mm flanged cylinder.  The
    parts that bolt to it (brackets, links, horn adapters) are
    completely different.  Sharing constants would just make every
    `make_*` function a thicket of `if PROTOTYPE:` branches.

Outputs:
    stl_prototype/   -- slicer-ready printables only
    stl_reference/   -- bought-part / fused-link visuals (``*_DO_NOT_PRINT.stl``)

    Body parts (one each, under stl_prototype/)
        chassis_top.stl         -- 3D-printable hex top plate
        chassis_bottom.stl      -- 3D-printable hex bottom plate
        switch_holster.stl      -- anti-spark switch holster on chassis_top
        yaw_servo_retainer.stl  -- yaw anti-rotation saddle + ground feet (×6)
        yaw_bearing_cap.stl     -- yaw bearing tower cap (×6)

    As-built electronics (extra_stl/ + stl_reference/ visuals, not trays):
        hex_mount_plate_110 + hex_raised_platform_110 (magnet posts),
        PDB + motor controller on chassis_top, Uno Q + breakout on hex,
        screen on raised top; MPU on chassis_bottom behind phys. leg 1; power+data Wagos.
        (uno_q_tray / buck_tray / spider_carapace / imu_pad RETIRED.)

    Per-leg parts (one of each — print 6 sets)
        coxa_bracket.stl        -- bolts to the chassis edge, holds the yaw servo
        coxa_link.stl           -- horn-driven U-bracket; holds the hip-pitch servo
        femur_link.stl          -- thigh; horn-driven by hip, holds the knee servo
        tibia_link.stl          -- shin; horn-driven by knee, ends in the foot socket
        foot_boot.stl           -- TPU boot pressed over the tibia tube end (×6)

    Generic (DEPRECATED -- Design B retired the printed adapter; see
    HORN_ADAPTER_OD / make_servo_horn_adapter below.  Kept only for
    backward-compatible STL references in old quotes.)
        servo_horn_adapter.stl  -- round printed disc that bolted to a standard
                                   25T spline horn and provided 4 x M3 holes on
                                   a 20.8 mm PCD at 0 / 90 / 180 / 270 deg
                                   (aligned with the now-retired plastic X-horn
                                   arms) so a flat printed link could mate to a
                                   servo horn.  That printed adapter AND the
                                   later plastic X-horn scheme are both retired:
                                   each link now bolts DIRECTLY onto a 20 mm
                                   aluminum 25T disc horn (Amazon B07D56FVK5) via
                                   4 x M3 x 6 SHCS that thread into the disc's
                                   M3 tapped holes on a 14 mm bolt circle.

    Assembly preview (everything in standing pose — visualization only)
        assembly_preview.stl

Geometry overview
-----------------

    Six identical legs spaced 60 deg around a hexagonal body.  Each
    leg has the same 3 rotary DOF as the full-size walker:

         coxa  (hip yaw,    vertical axis)
         femur (hip pitch,  horizontal radial axis)
         tibia (knee pitch, horizontal radial axis)

    18 hobby servos total.  Recommended part: ANNIMOS / Miuzei DS3225
    (25 kg-cm, metal gear, ~$13 each on AliExpress / ~$18 on Amazon).
    The MG996R (10 kg-cm) is also fine for the lighter-weight builds
    but leave less torque margin at the knee.

    Power: 1 x 3S 2200 mAh LiPo (or any 6-7.4 V supply >= 5 A).
    Brain: Arduino Mega + PCA9685 16-channel PWM driver (need 2 boards
    for 18 servos, daisy-chained over I2C).

Build envelope
--------------

    Chassis flat-to-flat:    200 mm
    Foot-to-foot extended:   ~ 700 mm
    Standing height:         ~ 100 mm
    Total mass:              ~ 1.3 kg
    Per-leg static load:     ~ 4.3 N (~ 0.4 kg)
    Peak knee torque:        ~ 0.6 N*m (~ 6 kg-cm) -- DS3225 has 4x margin

See PROTOTYPE.md for the BOM, wiring diagram, and gait-controller
starter code.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import trimesh
from trimesh.creation import box as box_mesh
from trimesh.creation import cylinder as cylinder_mesh
from trimesh.transformations import rotation_matrix


# ---------------------------------------------------------------------------
# Geometry constants  (everything in millimetres)
# ---------------------------------------------------------------------------

# ---- Vehicle envelope -----------------------------------------------------
# A regular hexagonal 4 mm 3D-printed top + bottom plate, 200 mm flat-to-flat.
# Comfortably fits on a 220 x 220 mm hobbyist printer bed (Ender 3 / P1S
# size).  At nominal stance the foot pads sit on a circle of radius
# ~ 350 mm, so the vehicle's outer diameter is ~ 700 mm.
CHASSIS_FLAT_TO_FLAT  = 200.0   # mm -- distance between opposite hex edges
CHASSIS_PLATE_T       =   4.0   # mm -- thickness of each 3D-printed plate
CHASSIS_GAP           =  20.0   # mm -- vertical gap between top + bottom plates
                                #     (wiring bay only).  History: 20 mm
                                #     originally, raised to 32 mm (May-Jul
                                #     2026) while the LiPo lived in the
                                #     inter-plate bay (24 mm pack + 8 mm
                                #     headroom).  Aug 2026: the battery
                                #     moved to TWO shorty packs velcro'd
                                #     UNDER the belly (see BATTERY_*), so
                                #     the bay only has to clear the corner
                                #     power-Wago trays -- floor 3 + Wago
                                #     8.3 = 11.3 mm tall -- plus wire runs;
                                #     20 mm leaves 8.7 mm over the Wago
                                #     tops (lever service = lift the top
                                #     plate off its 4 standoff bolts).
                                #     The brass standoffs in docs/BOM.md
                                #     are 20 mm to match.  IMPORTANT: any
                                #     future change to CHASSIS_GAP MUST
                                #     re-check the corner-tray headroom +
                                #     the standoff length
                                #     (CAD_AGENT_INSTRUCTIONS.md rule #9).
# The TOP plate is intentionally smaller than the bottom plate.  The bottom
# plate is the structural one: it sandwiches the coxa-bracket flanges (with
# their four M3 bolts each) and also takes the per-leg servo body cutouts.
# The TOP plate carries only the battery holder, electronics tray and the
# optional arm bracket -- all of which sit within a ~70 mm radius of the
# chassis centre.  Was 200 mm flat-to-flat to match the bottom; the
# check_workspace_self_collision sweep found that across the full
# (yaw, hip, knee) workspace the coxa_link's hip-pitch well and the
# femur_link's hip pad sweep through z ~ 8..36 mm above the bracket
# flange, which is the same height band the chassis_top occupies
# (chassis_top centre at z = chassis_lift + CHASSIS_TOP_CENTRE_Z, see
#  below).  Shrinking the top plate to
# 140 mm flat-to-flat (= apothem 70 mm) keeps the deck for the battery +
# electronics + arm but moves its outer edge inside the radius the leg
# sweeps reach, eliminating ~95 of the 111 collisions found in the
# pre-fix audit.
CHASSIS_TOP_FLAT_TO_FLAT = 140.0  # mm
# Coaxial-coxa clearance: chassis_top is clipped to a disk of this radius
# so the (now yaw-axis-centred) coxa servo body sweep -- which reaches
# inward to ~60 mm in the chassis_top z-band -- clears the deck edge.  All
# deck mounts sit at r <= 52.6 mm, well inside.  See make_chassis_top.
CHASSIS_TOP_RADIUS = 57.5  # mm
# Aug 2026: the TOP plate is HALF the bottom plate's thickness (user: thin
# enough for screw mounts -- an M3 + nut or short self-tapper clamps
# cleanly through 2 mm).  The plate is non-structural (the deck loads are
# light and the 4 inter-plate standoffs support it at r 44); only the
# switch-holster insert bosses need extra local depth, which they get from
# an under-plate pad (see make_chassis_top).  The plate's UNDERSIDE stays
# where it was (top of the 32 mm standoffs) -- the deck face comes DOWN
# 2 mm.  All placement math must use the constants below, never
# CHASSIS_PLATE_T, for the top plate.
CHASSIS_TOP_T        = CHASSIS_PLATE_T / 2.0                    # 2 mm
CHASSIS_TOP_BOT_Z    = CHASSIS_GAP + 0.5 * CHASSIS_PLATE_T      # +34 underside
CHASSIS_TOP_CENTRE_Z = CHASSIS_TOP_BOT_Z + 0.5 * CHASSIS_TOP_T  # +35 (mesh z=0)
CHASSIS_TOP_TOP_Z    = CHASSIS_TOP_BOT_Z + CHASSIS_TOP_T        # +36 deck face

# ---- Leg link lengths -----------------------------------------------------
# Same 1 : 4 : ~5 ratio as the full-size walker, scaled for a tabletop
# build.  Tibia is intentionally a hair longer than 4 x coxa so the
# foot can lift clear over a small obstacle in swing phase.
COXA_LENGTH    =  12.5   # mm -- yaw axis -> hip-pitch axis.  Set to
                         # SERVO_OUTPUT_X so the hip servo BODY centres
                         # over the yaw axis: the hip cradle stacks
                         # coaxially RIGHT ON TOP of the yaw hub (Part A)
                         # instead of cantilevering off to the side.
FEMUR_LENGTH   =  90.0   # mm -- hip-pitch axis -> knee axis
# Knee axis -> furthest boot/contact tip (kinematic).  Bench
# re-measurement on leg 0 (2026-08-25): hip centre -> knee centre =
# 90 mm and knee centre -> furthest straight-out foot tip = 150 mm.
# All six legs now share this same length; the old L0/L4 short-leg
# compensation boot is retired.
TIBIA_LENGTH   = 150.0   # mm -- knee axis -> furthest foot tip

# ---- Coxa link pedestal --------------------------------------------------
# How far above the disc-horn mating face the coxa-link arm + well
# float on a built-in pedestal.  Without the lift the hip-pitch well
# bottom sits ~2.5 mm below the chassis-plate top and the femur's
# hip-pad swings into the bracket flange + well as the femur pitches.
#
# Sizing constraint:  the femur's hip pad/neck is a CYLINDER of radius
# HIP_PAD_R = 17 mm rotating about the +Y joint axis.  The lowest
# coxa-link z it reaches under rotation is hip_drop - HIP_PAD_R =
# -(WELL_D/2 + arm_t/2) + COXA_LIFT - 17 mm; the bracket's flange
# TOP face sits at world z = BRACKET_FLANGE_T = 15 mm (the flange
# was thickened so a solid top cap closes off the body+tab passage --
# see BRACKET_FLANGE_T below), which equals coxa-link z =
# -yaw_output_z + BRACKET_FLANGE_T = -10.75 mm in the standing pose.
# Setting (lowest pad z in coxa-link) >= -10.75 mm gives
#   COXA_LIFT >= BRACKET_FLANGE_T + WELL_D/2 + arm_t/2
#                + HIP_PAD_R - yaw_output_z
#              = 15 + 14.5 + 3 + 17 - 25.75 = 23.75 mm.
# COXA_LIFT = 26 mm gives ~ 2.25 mm of safety margin against FDM
# print tolerances and the voxel sampler's pitch (1.5 mm).  Was 14 mm
# when the flange was a thin 4 mm slab with a through-slot for the
# body+tab passage; bumping the flange to a solid 15 mm slab (with
# the body cavity as a one-sided pocket open only on the bottom of
# the flange and a small Phi 11 mm gear clearance through the top
# cap) requires the matching COXA_LIFT bump so the femur's swept hip
# pad still clears the taller flange.
#
# Bolt screws need to be ~M3 x 32 mm long to reach through the
# (taller) pedestal + disc horn into the servo's
# output gear.  The pedestal's structural Z extent is unchanged
# (still 3.5 mm; the lift increases the air gap below the structural
# slab, not the slab itself).
# COXA_LIFT history:
#   COXA_LIFT = 14 was the original value with a thin 4 mm bracket flange
#   and HIP_PAD_R = 17.  Bumping the flange to a solid 15 mm slab (so a
#   top cap closes the body+tab passage) required COXA_LIFT = 26 to keep
#   the femur's swept hip pad clear of the thicker flange.
#
#   Bumping HIP_PAD_R from 17 to 19.5 (so the hollow-annulus neck has a
#   printable 3 mm wall around the horn-stack clearance void -- see the
#   HIP_PAD_R docstring) puts the pad's swept disk +Z edge at
#       hip_drop + HIP_PAD_R = -17.5 + COXA_LIFT + 19.5 = COXA_LIFT + 2 mm
#   which would COLLIDE with the coxa-link arm's bottom face at z =
#   COXA_LIFT (the arm rests directly on the pedestal top, so arm-bottom
#   and pedestal-top share the COXA_LIFT plane).
#
#   We FIRST drop the well 4 mm deeper than the "arm-bottom = well-top"
#   default (WELL_Z_DROP_EXTRA = 4 mm; see below) so the hip axis lives
#   4 mm below arm-bottom and the pad's +Z edge drops by 4 mm relative
#   to the arm.  This gives 2 mm of vertical air between the pad's +Z
#   edge and the arm's bottom face.
#
#   We THEN bump COXA_LIFT to 32 mm so the pad's -Z edge (hip_drop -
#   HIP_PAD_R = COXA_LIFT - 21.5 - 19.5 = COXA_LIFT - 41 mm) still clears
#   the coxa-bracket flange's top face (at coxa-link z = -10.75 mm in the
#   standing pose).  COXA_LIFT = 32 mm gives:
#     pad -Z edge = -9.5 mm > -10.75 mm  (1.25 mm clearance to bracket)
#     pad +Z edge = +30.5 mm < +32 mm    (1.50 mm clearance to arm)
#   Both clearances are sized for FDM print tolerance + voxel sampler
#   pitch (1.5 mm) so check_workspace_self_collision stays clean.
#
#   Design B (May 2026): the printed servo_horn_adapter has been
#   retired, dropping HORN_STACK_H from 9 mm to 5 mm and therefore
#   dropping the world Z of the coxa_link's frame (and the hip-pitch
#   joint axis) by HORN_ADAPTER_T = 4 mm.  Without compensation, the
#   bracket's flange top in coxa-link Z moves from -10.75 mm to
#   -6.75 mm and the femur's swept pad (lowest coxa-link z = -9.5 mm)
#   now collides with it by ~ 2.75 mm.  COXA_LIFT is therefore bumped
#   by the same 4 mm (32 -> 36) so the hip-pitch joint axis stays at
#   the same WORLD Z as before, the kinematic chain is unchanged for
#   RL / gait / MuJoCo, and the documented +1.25 mm bracket and
#   +1.5 mm arm clearances are restored.
#
# STS3215 refit (Jun 2026): the FEETECH STS3215 body is 24.8 mm deep
# vs the 20 mm Design-B hobby body -- +4.8 mm.  That +4.8 mm shows up
# TWICE in the cap-facing direction: the body's depth half-span about
# the hip-pitch axis grows +2.4 mm (+/-10 -> +/-12.4) AND the well drops
# +2.4 mm deeper (WELL_D 29 -> 33.8, so well_z_drop = -(WELL_D/2 + ...)
# gets 2.4 mm more negative).  Net, the body's lower (cap-facing) edge
# falls a full 4.8 mm, from coxa-link z ~ +4.5 to ~ -0.3, punching it
# through the 4 mm disc-horn bolt cap (z in [0, 4]), which must keep its
# full PEDESTAL_CAP_T for the M3 SHCS counterbores.  Following the exact
# same pattern as the Design-B bump, COXA_LIFT is raised by that same
# +4.8 mm (36 -> 41) so the hip axis (and the rigidly attached servo
# body) lifts back up, restoring body_bot ~ +4.7 mm and the cap
# clearance, while the assembly keeps the world hip-axis Z fixed so
# kinematics / RL / MuJoCo are unchanged.
COXA_LIFT     = 41.0     # mm  (32 -> 36 Design B -> 41 STS3215 refit)

# Extra drop of the hip-pitch well centre BELOW the natural "arm-bottom =
# well-top" plane.  When 0, the well's +Z face (= the hip-pitch axis +
# WELL_D/2) coincides with the arm's bottom face and the bridge member
# is a thin 0.5 mm boolean kiss.  When > 0, the well sits this many mm
# deeper, the hip axis drops by the same amount, and the bridge box
# extends DOWN to overlap the new well-top.  Used to keep the femur hip
# pad's swept +Z edge clear of the coxa-link arm's bottom face -- see
# COXA_LIFT docstring for the geometry.
WELL_Z_DROP_EXTRA = 4.0  # mm

# See COXA_HIP_DROP below for the derived hip-axis Z position.  It needs
# WELL_D and COXA_ARM_T (defined later) so it lives further down.

# ---- Servo (actuator): FEETECH STS3215 serial-bus servo ------------------
# ST-3215-C018, 12 V / 30 kg-cm.  June 2026 redesign replaced the DS3225
# PWM servo (and its 2x PCA9685 drivers + AS5600 add-on encoders) with the
# STS3215 smart serial-bus servo: a single TTL bus, built-in 12-bit
# magnetic-encoder feedback, no PWM boards, no external sensors.
#
# Geometry verified against FEETECH's OFFICIAL STEP (SO-ARM100
# STS3215_03a.step): a rectangular brick 45.4 (X, long) x 24.8 (Y, depth),
# the 25T output shaft on the +Z face offset +12.5 mm in X, an idler shaft
# on the -Z face (the "dual-axis" case), M3 central horn screw.
#
# CRUCIAL MOUNTING DIFFERENCE: the STS3215 has NO protruding mounting tabs.
# It bolts via 4x M2.5 THREADED HOLES IN ITS METAL CASE, present on BOTH
# the front (output) and back (idler) faces, on a SERVO_MOUNT_SQUARE = 9.8
# mm square around the shaft axis (STEP: holes at (12.5 +/- 4.9, +/- 4.9)).
# The printed cradle bolts to the FRONT face: 4x M2.5 pass through the
# cradle's output-side plate into the case (the case IS the thread -- so
# the old heat-set inserts, Phi 8 mm bosses and self-tap pilots are all
# DELETED).  The horn screws onto the 25T spline with the central M3.
SERVO_BODY_W      = 45.4   # mm -- body length along the output-offset (X) axis
SERVO_BODY_D      = 24.8   # mm -- body depth (Y, perpendicular)
SERVO_BODY_H      = 34.3   # mm -- back mount-face -> front(output) mount-face (Z),
                            #      i.e. between the two 4x M2.5 hole planes (STEP)

# --- Wire-exit boot on the servo body (LEGACY STAND-IN) --------------------
# IMPORTANT (Aug 2026 — see firmware/WIRING.md "Where the wires leave an
# STS3215"): the REAL STS3215 has two recessed Molex 5264 ports on the
# BACK (idler) face, centre/−X half, with cables leaving out the back —
# not a DS3225-style molded boot on the +X short end.  These WIRE_BOOT_*
# constants (and the box on ``make_servo_body``) are a DS3225-era probe
# target kept so cradle insertion / clearance channels still have a
# geometric stand-in.  Do NOT treat them as the STS3215 cable exit for
# harness routing — BuildViz attaches at the back-face port cluster
# (``_servo_attach_paths`` in tools/full_robot_viz_build.py).
#
# Stand-in geometry in servo-local coords (origin = back/bottom face centre,
# +X = output-offset direction, +Y = body short axis, +Z = output-shaft up):
#
#     boot footprint on the +X body face:
#         Y span : +/- WIRE_BOOT_W / 2  (centred on y = 0)
#         Z span : [WIRE_BOOT_Z_BASE, WIRE_BOOT_Z_BASE + WIRE_BOOT_H]
#     boot extrudes OUT in +X by WIRE_BOOT_PROTRUSION from the body face
#         (so boot occupies x in [+SERVO_BODY_W/2, +SERVO_BODY_W/2 + WIRE_BOOT_PROTRUSION]).
WIRE_BOOT_W           = 7.0    # mm -- stand-in boot Y width  (centred on y = 0)
WIRE_BOOT_H           = 3.9    # mm -- stand-in boot Z height
WIRE_BOOT_Z_BASE      = 4.1    # mm -- stand-in boot lower-edge Z above body base
WIRE_BOOT_PROTRUSION  = 6.5    # mm -- stand-in protrusion in +X

# Real STS3215 dual 5264 port cluster (well / servo-body frame; origin =
# back-face centre).  Midpoint between the two side-by-side ports on the
# centre/−X half of the back, just past the face so plugs/cables clear.
STS3215_PORT_X_MM     = -10.0  # mm -- toward −X (away from output at +12.5)
STS3215_PORT_Y_MM     =  0.0   # mm -- between the two ports
STS3215_PORT_Z_MM     = -3.0   # mm -- just past the back face (−Z)
SERVO_OUTPUT_H    =  2.0   # mm -- output hub/horn-cap stack above the front face
SERVO_OUTPUT_OD   = 20.0   # mm -- output horn-cap OD (visual; STEP dia-20 hub)
SERVO_SPLINE_OD   =  5.9   # mm -- 25T spline OD (M3 horn screw lives in the bore)
SERVO_OUTPUT_X    = 12.5   # mm -- output shaft offset from body centre, toward +X
                            #      (FEETECH STEP: output axis at body-x = +12.5)
# RESOLVED (Aug 2026): 12.5 mm is CORRECT — do not resurrect the old
# "Waveshare bracket ~3.6 mm" note.  Besides the FEETECH SO-ARM100 STEP,
# the ASSEMBLED robot proves it: the front-face M2.5 mount square at
# (12.5 +/- 4.9, +/- 4.9), the Phi 20 horn-cap wells in every cradle lip,
# and the yaw drive-nub M3 horn screws (fed down the coxa head-access
# shafts, bench-torqued) all land on the real servos — impossible if the
# true offset were ~3.6.  The 3.6 figure was a misreading of a Waveshare
# 2D bracket drawing (measured from a bracket feature, not the body
# centre).  The END-face body-bolt pattern below is centred on the BODY
# and is independent of this offset either way.

# --- Leg pitch-joint output-axis convention --------------------------------
# The hip + knee sandwich joints' OUTPUT (disc-horn) axis is coxa-local
# +/-Y.  Choosing -Y points every cradle's OPEN (servo-insertion) face --
# and therefore its bolt-on clamp cap -- UP (+Z), so the cradle's solid
# back seats DOWN onto its mount (the hub platform for the hip, the femur
# spine for the knee).  This is what lets the hip bracket stack coaxially
# on top of the yaw hub: the wide servo would otherwise open its mouth
# DOWN into the hub and leave no room to bolt the foot.  The leg POSE is
# unchanged -- femur_dir / tibia_dir and all joint points are computed
# geometrically and are independent of this sign; only the cradle/yoke
# ORIENTATION (which way the open face points) flips.
LEG_PITCH_AXIS = (0.0, -1.0, 0.0)

# ---- STS3215 case-face mounting (replaces the DS3225 tab/insert scheme) ---
# The servo bolts to the cradle by 4x M2.5 on a SERVO_MOUNT_SQUARE = 9.8 mm
# square centred on the OUTPUT axis (x = SERVO_OUTPUT_X, y = 0), on the
# FRONT (+Z output) face.  Hole centres (servo-local) are
# (SERVO_OUTPUT_X +/- SERVO_MOUNT_HOLE_X_OFFSET, +/- SERVO_MOUNT_HOLE_Y_OFFSET)
# on the front-face plane (z = SERVO_BODY_H).  The screw threads into the
# servo's own metal case (no heat-set insert, no printed boss, no
# self-tap pilot).  STEP-verified: holes at (12.5 +/- 4.9, +/- 4.9).
SERVO_MOUNT_SQUARE        = 9.8   # mm -- centre-to-centre of the 4 case holes
SERVO_MOUNT_HOLE_X_OFFSET = SERVO_MOUNT_SQUARE / 2.0   # +/-4.9 about the shaft
SERVO_MOUNT_HOLE_Y_OFFSET = SERVO_MOUNT_SQUARE / 2.0   # +/-4.9 about the shaft
SERVO_MOUNT_SCREW_OD      = 2.7   # mm -- M2.5 clearance hole in the printed plate
SERVO_MOUNT_HEAD_OD       = 4.6   # mm -- M2.5 SHCS head OD (counterbore option)
SERVO_MOUNT_THREAD_DEPTH  = 4.0   # mm -- usable M2.5 thread depth into the case

# ---- STS3215 small FRONT-face case-shell screw holes (Aug 2026) -----------
# STEP-verified (SO-ARM100 STS3215_03a.step, same source as SERVO_BODY_*):
# besides the 9.8 mm M2.5 square around the output, the plastic case shell
# carries FOUR small (Phi 1.5 molded pilot, ~1.5 mm deep) screw holes on
# EACH mount face -- the same family the yaw saddle self-taps on the REAR
# face (``yaw_rear_screw_centres`` / SADDLE_CASE_HOLE_*).  Body-frame (x from
# body centre, y depth) positions:
#     REAR  (passive) face: (4.2, +-10.25) and (-20.3, +-10.25)
#                           [= cradle (-8.3, +-10.2) / (-32.8, +-10.2)]
#     FRONT (output)  face: (4.2, +-10.25) and (-16.5, +-10.25)
# i.e. one front pair sits at the SAME (x, y) as the rear landmark pair and
# the other pair is shifted +3.8 mm in x off the rear companion pair.  The
# front holes open on the case-shell DECK that is RECESSED
# SERVO_FRONT_CASE_DECK_DROP below the front mount-hole plane (STEP: shell
# deck z=15.9 vs mount plane z=18.7), so a screw through the well lip
# stands off that extra gap before it bites -- exactly like the end-face
# SERVO_BODY_BOLT_STANDOFF, tension pulls the body front face up against
# the lip underside.  Same screw as the yaw rear capture: M2.5 x 6
# self-tap (PN 96877A150), bite ~= the full molded pilot depth.
SERVO_FRONT_CASE_HOLE_XS   = (4.2, -16.5)  # mm -- body-frame x of the 2 pairs
SERVO_FRONT_CASE_HOLE_Y    = 10.25         # mm -- body-frame |y| of all 4 holes
SERVO_FRONT_CASE_DECK_DROP = 2.8    # mm -- shell deck recess below the front face
FRONT_CASE_SCREW_OD        = SERVO_MOUNT_SCREW_OD  # 2.7 -- M2.5 self-tap clearance
FRONT_CASE_CBORE_OD        = 5.2    # mm -- head counterbore (Phi 4.6 head + 0.6)
FRONT_CASE_CBORE_DEPTH     = 2.2    # mm -- sinks the head FLUSH under the swinging
                                    #       yoke arm (arm underside is only 3 mm
                                    #       above the lip top) and keeps the head
                                    #       edge out of the horn-pad sweep band

# ---- STS3215 BODY-retention bolts on the END (+/-X) faces -----------------
# Measured from the authoritative Waveshare ST3215 mount brackets: each
# END (short, +/-X) face of the servo carries 4x M2.5 threaded holes in a
# 10 x 10 mm square CENTRED on the face (clearance dia 2.8 through the
# printed part).  These are the REAL body-mounting holes (distinct from
# the dia-14 disc-horn cross on the OUTPUT face), so the printed cradles
# can POSITIVELY bolt the servo by its end face instead of only gripping
# it.  The square is centred on the body face, so the hole centres are
# independent of the output offset (SERVO_OUTPUT_X): in the body frame
# (centre at x=y=0, z in [0, SERVO_BODY_H]) they sit at
#   (+/-SERVO_BODY_W/2, +/-SERVO_BODY_BOLT_PITCH/2, SERVO_BODY_H/2 +/-
#    SERVO_BODY_BOLT_PITCH/2)
# i.e. y = +/-5, z = SERVO_BODY_H/2 +/- 5 on each end face.  See
# ``servo_end_face_bolt_centres``.
SERVO_BODY_BOLT_OD    = 2.8   # mm -- M2.5 clearance through the printed wall
SERVO_BODY_BOLT_PITCH = 10.0  # mm -- 10 x 10 mm square, centred on the end face
SERVO_BODY_BOLT_HEAD_OD = 5.0   # mm -- M2.5 SHCS head clearance (counterbore)
SERVO_BODY_BOLT_LEN     = 8.0   # mm -- M2.5 x 8 screw into the servo case
# The printed end WALLS are thick (WELL_WALL_X ~ 8-11 mm), so the M2.5
# head is recessed in a counterbore and the screw stands off the servo
# end face by (SERVO_BODY_BOLT_LEN - SERVO_MOUNT_THREAD_DEPTH) so its tip
# threads SERVO_MOUNT_THREAD_DEPTH into the case.  Keeps a stock M2.5 x 8.
SERVO_BODY_BOLT_STANDOFF = SERVO_BODY_BOLT_LEN - SERVO_MOUNT_THREAD_DEPTH  # 4 mm

# ---- Bearing-SANDWICH joint architecture (Jun 2026 -> symmetric refit) ----
# Real STS3215 legged-robot joints are supported on BOTH ends, not
# cantilevered off the output horn.  The STS3215 already gives us TWO
# coaxial bearing-supported bosses: the 25T DRIVEN spline on the front
# (output) face, and a SMOOTH idler boss on the back face that rotates with
# the output shaft on the servo's own internal rear bearing.  So instead of
# adding an external 688 ball bearing + housing on the back (an asymmetric
# "ring" that only supported the yoke through a loose stub), we mount a
# SECOND disc horn -- the PASSIVE horn -- on the rear idler boss and bolt the
# moving yoke to BOTH horns identically.  The result is a symmetric, fully
# bolted bearing sandwich carried on the servo's own two bearings:
#
#   * DRIVEN side  : STS3215 25T spline -> dia-20 disc horn -> yoke top arm
#                    (4x M3 into the horn's tapped holes + central spline
#                     screw).
#   * PASSIVE side : rear idler boss -> centering ADAPTER -> a SECOND dia-20
#                    disc horn -> yoke bottom arm (4x M3 + a central
#                    retention screw into the rear boss so the horn can't
#                    work loose).  Same horn part, same bolt pattern, same
#                    reach-down pad -- the yoke arms are mirror-identical.
#   * SEGMENTS     : dia-8 CARBON-FIBRE TUBE between joints (femur / tibia),
#                    socketed into printed end-fittings.
SERVO_MOUNT_SELFTAP_OD    = 2.6   # mm -- printed-plate clearance for M2/M2.5 self-tap
SERVO_CASE_HOLE_OD        = 2.5   # mm -- molded case hole the self-tapper bites (STEP)

# --- Passive (rear-boss) horn stack -----------------------------------------
# Jul 2026 (user): the STS3215 ships with a STOCK METAL PASSIVE HORN that
# looks just like the driven disc horn (same 4-hole pattern the yoke bolts
# to).  Its centre bore slides OVER the rear idler boss, so the horn seats
# FLUSH on the servo back face and does NOT protrude beyond the boss tip.
# The boss itself centres the horn; the printed centering adapter
# (make_passive_horn_adapter, retired here) and its 2 mm standoff base are
# GONE, which pulls the yoke bottom-arm seat 2 mm closer to the servo (the
# user measured the adapter-era yoke exactly 2 mm too wide on this stack).
# (Derived constants that depend on the disc-horn dimensions live in the
# JOINT_HORN block below, once those are defined; see BACK_STACK_DEPTH /
# PASSIVE_HORN_FACE_Z there.)
REAR_BOSS_H        = SERVO_OUTPUT_H               # 2 mm -- rear idler boss protrusion
REAR_BOSS_OD       = 4.0                          # mm -- smooth rear boss/shaft dia (MEASURE
                                                  #       on your unit); the stock passive
                                                  #       horn's centre bore rides it directly.

# ===== SPACED YAW BEARING PAIR (cantilever moment support) ===============
# The coxa cantilevers the whole hip/femur/tibia ~COXA_LENGTH out to the
# side of the yaw axis.  A SINGLE thin bearing reacts shear but barely
# reacts the tilting MOMENT (its internal couple is only ~its width).  We
# instead use TWO coaxial deep-groove bearings SPACED apart along the yaw
# axis: tilt stiffness scales with the spacing^2, so the cantilever moment
# is carried into chassis_bottom as an axial couple, not through the servo
# spline/horn.
#
# Geometry cost: a single 6706 fits the original 5 mm disc-horn gap, but a
# spaced pair needs vertical room ABOVE the disc horn (the servo body
# blocks any bearing below the output face).  We therefore LOWER the yaw
# output plane and LIFT the hip the same amount (YAW_TOWER_RAISE) so the
# WORLD hip-pitch axis Z is unchanged -> leg kinematics / RL / MuJoCo are
# unchanged (same trick the COXA_LIFT history used).  See CHASSIS_YAW_OUTPUT_Z
# and COXA_HIP_DROP below.
YAW_TOWER_RAISE           = 14.5  # mm -- yaw output lowered / hip lifted by this
                                  #       (Jun 2026: bumped 9 -> 14.5 = +5.5 when
                                  #       BOTH yaw bearings moved above the flush
                                  #       horn, raising the hub platform +5.5 mm;
                                  #       keeps WORLD hip-axis Z fixed)
                                  #       Aug 2026 6805 swap: deliberately NOT
                                  #       bumped for the +4 mm platform rise.
                                  #       Bumping it would deepen the cradle /
                                  #       tower (CHASSIS_YAW_OUTPUT_Z feeds the
                                  #       chassis_bottom geometry) and force a
                                  #       reprint of the big plate.  Instead the
                                  #       WORLD hip axis rises 54.65 -> 58.65
                                  #       (propagates consistently through
                                  #       COXA_HIP_DROP to every consumer; no
                                  #       trained RL policy exists yet, and the
                                  #       MuJoCo model regenerates from these
                                  #       constants).

# Two coaxial 6805-2RS / 61805 (25 x 37 x 7) deep-groove bearings.  Aug 2026
# thick-section swap (user: same OD, smaller ID, so the printed retaining
# lips can be thicker and reach further over the races): was 6706-2RS
# (30 x 37 x 4).  Same Phi 37 OD keeps the chassis tower bore + Phi 34 seat
# shoulder untouched (chassis_bottom is print-compatible); the 5 mm-smaller
# bore roughly DOUBLES the race cross-section, so the cap lip grabs ~2.5 mm
# of outer race (was 1.5) and the hub flange gets a ~2 mm inner-race shelf
# (was ~1).  6805 is commodity bike-bottom-bracket stock.  The pair sits
# TOUCHING (no spacer gap): 7 mm-wide races stacked face-to-face give the
# same 7 mm centre-to-centre moment-couple spacing the old 4+3+4 stack had,
# so tilt stiffness is preserved while the stack grows only 3 mm.  The
# Phi 25 bore still clears the Phi 20 drive-bolt pattern (head counterbores
# reach r 10.4 < r 12.5); the hub boss (OD 25) rides both inner races and
# the chassis tower keeps its single Phi 37 bore.  Visual-only (NOT
# printed) -- see make_yaw_bearing / NOT_PRINTED_MESHES.
YAW_BEARING_ID            = 25.0  # mm -- inner-race bore (6805)
YAW_BEARING_OD            = 37.0  # mm -- outer-race OD (unchanged vs 6706)
YAW_BEARING_W             =  7.0  # mm -- bearing width (was 4)
YAW_BEARING_INNER_OD      = 29.0  # mm -- approx inner-race OD (rotating band)
YAW_BEARING_OUTER_ID      = 33.0  # mm -- approx outer-race ID (stationary band)
YAW_BEARING_PRESS         = -0.075  # mm -- OD shrink for press fit (tune per printer).
                                  #     Aug 16 2026 bench fit: at 0.0 (bore Phi 37.00
                                  #     exact) the race entered the chassis tower and
                                  #     the cap but BOUND -- could not be pushed home
                                  #     by finger (user).  FDM bores print a touch
                                  #     undersize, so the modeled bore now carries
                                  #     +0.15 diametral clearance (Phi 37.15): a firm
                                  #     finger-press slip fit; the race is located
                                  #     axially by shoulders + cap lip, not by grip.
# Bearing Z bands (coxa-local; z = 0 = disc-horn top).  Jun 2026 FLUSH-HORN
# refit: the real STS3215 disc horn is ~2 mm and sits recessed FLUSH with the
# servo's output face -- there is no protruding horn pedestal, so BOTH
# bearings sit ABOVE the flush horn (z > 0) on the solid hub boss, the LOWER
# race starting 0.5 mm above the horn top.  Aug 2026 6805 swap: the two 7 mm
# races sit TOUCHING (face-to-face, a standard duplex stack -- both outer
# rings stationary, both inner rings rotate together), which keeps the
# centre-to-centre couple spacing at exactly 7 mm (== the old 4 mm race +
# 3 mm gap + 4 mm race stack) while the overall stack grows only 3 mm.  The
# hip platform rises with the stack; YAW_TOWER_RAISE is deliberately NOT
# bumped this time (see the freeze note there).
YAW_BEARING_LOWER_BOT_Z   = 0.5
YAW_BEARING_LOWER_TOP_Z   = YAW_BEARING_LOWER_BOT_Z + YAW_BEARING_W   # +7.5
YAW_BEARING_UPPER_BOT_Z   = YAW_BEARING_LOWER_TOP_Z                   # +7.5 (touching)
YAW_BEARING_UPPER_TOP_Z   = YAW_BEARING_UPPER_BOT_Z + YAW_BEARING_W   # +14.5
YAW_BEARING_SPACING       = (0.5 * (YAW_BEARING_UPPER_BOT_Z + YAW_BEARING_UPPER_TOP_Z)
                             - 0.5 * (YAW_BEARING_LOWER_BOT_Z + YAW_BEARING_LOWER_TOP_Z))  # 7 mm
# Back-compat single-bearing aliases (older references / fastener notes).
YAW_BEARING_TOP_Z         = YAW_BEARING_LOWER_TOP_Z
YAW_BEARING_BOT_Z         = YAW_BEARING_LOWER_BOT_Z

# Chassis bearing tower: a Phi 37 bore holding both outer races on shoulders,
# with a retaining lip over the UPPER race so PLA/PETG creep can't let
# it walk out.  Tower top = upper race top + lip (coxa-local z).
YAW_TOWER_BORE_OD         = YAW_BEARING_OD - 2.0 * YAW_BEARING_PRESS  # press bore
# Aug 2026 6805 swap: FROZEN at Phi 34 (was OUTER_ID - 1) so the chassis_
# bottom tower is print-compatible with the 6706-era part.  Still valid for
# the 6805: it catches the outer race (band Phi 33..37) on a 1.5 mm ledge
# and clears the Phi 29 rotating inner-race band by 2.5 mm.
YAW_TOWER_SHOULDER_OD     = 34.0  # mm -- race-seat lip ID (frozen, chassis compat)
YAW_TOWER_WALL            =  3.5  # mm -- tower wall around the Phi 37 bore
YAW_TOWER_LIP_T           =  2.0  # mm -- retaining lip over the upper outer race
                                  #       (Aug 2026: was 1 -- user asked for a
                                  #       beefier lip; the 6805's taller stack
                                  #       already forces a cap reprint, so the
                                  #       lip thickening is free)
YAW_TOWER_TOP_Z           = YAW_BEARING_UPPER_TOP_Z + YAW_TOWER_LIP_T  # +16.5

# ---- SPLIT bearing tower (Jun 2026): two-piece for INSERTABLE races --------
# Forensic finding: the single-piece tower bored 37 -> 34 -> 37 -> 34 (two
# Phi 37 race pockets each trapped between Phi 34 constrictions), so NEITHER
# 6706 outer race could reach its seat from any direction -- a captured
# pocket the static-assembly CAD never flagged.  The tower is now SPLIT at
# the lower race's top face (coxa-local z = YAW_BEARING_LOWER_TOP_Z = -1):
#
#   * BOTTOM tower (integral to chassis_bottom): an OPEN-TOP Phi 37 pocket
#     from the mount-plate shoulder (z = -5) up to the split (z = -1).  The
#     LOWER race drops straight down onto the z = -5 shoulder -- a clear,
#     monotone Phi 37 insertion path with no constriction above it.
#   * yaw_bearing_cap (NEW bolt-on part, x6): carries the UPPER race in an
#     OPEN-TOP Phi 37 pocket seated on a neck shoulder at z = +2 (clear Phi 37
#     path from above), and a Phi 34 neck whose bottom face (z = -1) caps the
#     lower race once the cap is bolted down.  The old Phi 34 retaining lip
#     over the upper race is RETIRED (it was the very constriction that
#     trapped the race); the upper outer race is now located by the z = +2
#     gravity shoulder and the rotating hub's upper inner-race flange above.
#
# 3 x M3 self-tap join bolts (YAW_CAP_BOLT_*) pull the cap down onto the
# tower.  All join hardware lives BELOW the rotating dust-lip band
# (coxa-local z < YAW_HUB lip z0 = +3.5) so it never fouls the turntable.
# (Jun 2026: the bottom register wrap-lip was removed -- it was flimsy and
# hard to print; concentricity now comes from the 3 bolts into the tower
# pilots, which are coaxial by construction with the bottom-tower ear bosses.)
# Aug 2026 6805 swap: the split plane is FROZEN at the legacy +4.5 (lower-
# race seat + the old 4 mm race width) instead of tracking the new 7 mm
# race top, so the chassis_bottom tower pocket keeps its exact as-printed
# geometry.  The 7 mm lower race simply stands 3 mm proud of the split and
# the cap's Phi 37 bore swallows it -- the race is still fully radially
# housed (4 mm by the tower + 3 mm by the cap) and axially sandwiched
# between the chassis seat below and the upper race + cap lip above.
YAW_SPLIT_Z               = YAW_BEARING_LOWER_BOT_Z + 4.0   # +4.5 (frozen, chassis compat)
YAW_CAP_TOP_Z             = YAW_BEARING_UPPER_TOP_Z   # +14.5 (upper-race top / lip underside)
YAW_CAP_EAR_TOP_Z         = YAW_BEARING_UPPER_BOT_Z + 1.0   # +8.5 (ear top, below dust lip)
YAW_CAP_BOLT_N            = 3
# Bolt circle DIAMETER.  Each bolt sits in an OUTBOARD ear lug so the head
# counterbore fully clears the Phi 37 (r 18.5) central bore: at PCD 47 the
# head edge is at r = 47/2 - HEAD_OD/2 = 23.5 - 3 = 20.5 mm, a 2 mm wall to
# the bore.  (The retired PCD 45 left the head edge at r 19.5 -- only 1 mm to
# the bore and overhanging the Phi 44 cap OD, with no room for the head.)
YAW_CAP_BOLT_PCD          = 47.0  # mm -- join-bolt circle DIAMETER (head clears bore + ring)
YAW_CAP_BOLT_OD           = 3.4   # mm -- M3 clearance (through the cap ear)
YAW_CAP_BOLT_PILOT_OD     = 2.5   # mm -- M3 self-tap pilot (into the tower ear)
YAW_CAP_BOLT_BOSS_OD      = 9.0   # mm -- ear-boss OD (contains the Phi 6 head with 1.5 mm wall)
YAW_CAP_BOLT_HEAD_OD      = 6.0   # mm -- M3 SHCS head clearance (cap counterbore)
YAW_CAP_BOLT_LEN          = 8.0   # mm -- M3 x 8 self-tap SHCS join screw
                                  #       (head recessed in the cap ear, ~7 mm
                                  #       thread bite into the tower pilot)
YAW_CAP_BOLT_ANGLES_RAD   = (np.pi / 2.0,            # +Y
                             np.pi * 7.0 / 6.0,      # 210
                             np.pi * 11.0 / 6.0)     # 330  (none on +X swing axis)

# Inner retaining lip on the cap bore TOP (Jun 2026): a thin Phi YAW_CAP_LIP_ID
# shoulder that the UPPER outer race seats UP against, so the cap positively
# HOLDS the bearing rather than being radial-only support.  It overhangs ONLY
# the stationary outer race -- ID Phi 34 clears the Phi 32 rotating inner race
# / hub uflange by ~1 mm -- and sits ABOVE the seated race (z in
# [YAW_CAP_TOP_Z, YAW_CAP_RIM_Z]).  Because it is at the cap TOP it only
# contacts the race at the END of the cap's straight-down descent over the
# already-seated race, so it retains WITHOUT re-introducing the retired
# bottom-neck hard stop (the race never has to pass the lip).  This restores
# the retention the original single-piece tower lip gave, now compatible with
# the lowered-cap assembly.  The cap rim rises to YAW_CAP_RIM_Z (+7) = the old
# YAW_TOWER_TOP_Z, which already cleared the hub platform (z >= +7.5) by 0.5 mm.
YAW_CAP_LIP_T             = YAW_TOWER_LIP_T          # 2 mm retaining lip thickness
# Aug 2026 6805 swap: decoupled from YAW_TOWER_SHOULDER_OD (frozen Phi 34
# for chassis compat).  The thicker-section race lets the cap lip come IN
# to Phi 32: it overhangs the stationary outer race (band Phi 33..37) by
# 2.5 mm radial (was 1.5 on the 6706) while still clearing the Phi 29
# rotating inner race / hub uflange by 1.5 mm.
YAW_CAP_LIP_ID            = 32.0                     # mm -- lip bore (outer-race only)
YAW_CAP_RIM_Z             = YAW_CAP_TOP_Z + YAW_CAP_LIP_T   # +16.5 cap structural rim (lip top)

# Axial running gap between the STATIONARY cap rim / tower lip top (+16.5)
# and the ROTATING hub's platform underside (= foot-slab bottom since the
# Aug 17 2026 sink pass).  Was 0.5 mm; Aug 17 2026 bench (user: "the yaw
# bearing cap scrapes against the coxa link"): FDM stack-up on this axis
# runs ~1 mm (the same error the horn-screw seats showed), so 0.5 mm of
# design gap can vanish on a real print.  1.5 mm keeps the labyrinth
# (the dust skirt still overlaps the cap ring by 2.5 mm axially) while
# guaranteeing a real gap.  The whole hub platform / cradle stack rides
# this constant, so no screws move relative to their targets: the horn
# screw seats stay horn-referenced and the cap's 3 join bolts are on the
# stationary side.
YAW_HUB_CAP_AXIAL_CL      = 1.5  # mm -- rotating platform over cap rim

# Coxa yaw-hub turntable disc.
YAW_HUB_OD                = 44.0  # mm -- hub pad OD (covers the bearing)
# Bench-tuned fit history on the 6706: −0.2 felt loose, −0.05 was a snug
# FDM slip fit that still slid on from below.  Aug 16 2026 bench fit on the
# 6805's Phi 25 bore: −0.05 (Phi 24.95) printed "touching but just barely"
# (user) -- the boss spun freely inside the inner races instead of driving
# them.  +0.05 (Phi 25.05) STILL measured Phi 25.0 printed, identical to
# the 24.95 part: the coxa prints on its SIDE, so the boss is a horizontal
# cylinder whose layer-stack axis quantizes to 0.2 mm layers (a +/-0.05
# design delta vanishes into the process).  Now +0.15 nominal INTERFERENCE
# (Phi 25.15) so even the quantized axis prints >= ~25.1 and the boss gets
# a REAL ~0.1 mm press on the Phi 25.00 inner races -- a firm hand/tap fit
# over the 14 mm band (scrape/sand if a given print runs fat; the races
# still load from below over the lead-in).
YAW_HUB_BOSS_OD           = YAW_BEARING_ID + 0.15  # mm -- boss presses both inner races (Phi 25.15)
YAW_HUB_TOP_Z             = YAW_TOWER_TOP_Z + 1.5  # +8.5 -- hub top platform plane (clears tower lip)
YAW_HUB_SKIRT_BORE        = 11.0  # mm -- r11 skirt bore = DISC_HORN_OD/2 + 1 (clears horn)
YAW_HUB_DUST_LIP_WALL     = 3.2  # mm -- skirt radial wall (>=3 so it is not
                                 #       a flimsy thin flange)
YAW_HUB_DUST_LIP_CL       = 1.0  # mm -- radial running gap over the tower /
                                 #       cap-ring OD (both Phi 44).  Was 0.6;
                                 #       Aug 17 2026 bench scrape fix (user:
                                 #       "the yaw bearing cap scrapes against
                                 #       the coxa link") -- side-printed bores
                                 #       ovalize enough to eat 0.6 mm.  Still
                                 #       a grit labyrinth at 1.0.
YAW_HUB_DUST_LIP_OD       = (YAW_BEARING_OD + 2.0 * YAW_TOWER_WALL
                             + 2.0 * (YAW_HUB_DUST_LIP_CL
                                      + YAW_HUB_DUST_LIP_WALL))  # Phi ~51.6
# Part A <-> Part B vertical join bolts (4 x M3) on this PCD/angles.
COXA_JOIN_BOLT_PCD        = 32.0  # mm -- join-bolt circle DIAMETER on the hub top
COXA_JOIN_BOLT_OD         =  3.4  # mm -- M3 clearance (through Part B foot)
COXA_JOIN_PILOT_OD        =  2.5  # mm -- M3 self-tap pilot (into Part A hub)
COXA_JOIN_BOLT_ANGLES_RAD = (np.pi / 4.0, 3 * np.pi / 4.0,
                             5 * np.pi / 4.0, 7 * np.pi / 4.0)  # 45/135/225/315

# Carbon-fibre leg-segment tube.
LEG_TUBE_OD               = 8.0   # mm -- carbon tube outer diameter
LEG_TUBE_WALL             = 1.0   # mm -- typical CF tube wall (8 OD / 6 ID)
LEG_TUBE_SOCKET_CLEAR     = 0.05  # mm -- radial: snug slip-fit (Ø8.1 bore = 0.1 mm diametral), still slides on over a thin epoxy film, retained by the transverse pin
LEG_TUBE_SOCKET_DEPTH     = 14.0  # mm -- tube engagement depth into a socket
# Aug 2026: a tibia knee yoke split through the socket's front wall in the
# field (crack ran through the pin cross-hole region -- the thinnest hoop
# section, right on the print layer seams).  The wall was a hard-coded 3 mm;
# it is now a named parameter, bumped to 5 mm (boss Phi 14 -> Phi 18, ~2.8x
# the hoop-bending stiffness and far more meat around the Phi 2.6 pin hole).
# (This wall only drives the tibia knee yoke's tube socket boss; the femur
# spar has its own FEMUR_SPAR_OD, bumped to Phi 18 in Aug 2026.)
LEG_TUBE_SOCKET_WALL      = 5.0   # mm -- printed radial wall around the tube bore

# Tube retention: EPOXY ONLY (Aug 2026, user: "remove the tiny hole in the
# tibia knee yoke to attach the carbon fiber?  Its not needed im just gonna
# glue it").  The Jun 2026 transverse Phi 2.6 retention-pin cross-hole is
# REMOVED from the tibia knee yoke -- no pin, no drilling; the Phi 8.1 bore
# is retained by the epoxy bond alone.  Removing the cross-hole also deletes
# the thinnest hoop section / stress riser the Aug 2026 socket crack ran
# through.  The LEG_TUBE_PIN_* constants are KEPT only for
# tools/make_tibia_tube_step.py (the emergency printed stand-in tube, whose
# optional cross-hole is now vestigial).
LEG_TUBE_PIN_OD           = 2.6   # mm -- legacy cross-hole dia (see note above)
LEG_TUBE_PIN_INSET        = LEG_TUBE_SOCKET_DEPTH / 2.0  # mm -- pin axis from socket mouth

# FEMUR: no tube sockets at all (Jul 2026 one-piece femur).  The femur's
# ~19 mm inter-well span is bridged by the SOLID Phi 14 fused spar of the
# one-piece ``femur_link`` (see FEMUR_SPAR_OD / _femur_fused_spar), so the
# short femur sockets + transverse retention pin of the earlier two-part
# design are retired.  Only the 6 TIBIA sockets (x2 ends) remain, keeping
# the full LEG_TUBE_SOCKET_DEPTH + pin above.

# ---- Heat-set brass insert for cradle servo mounts (May 2026) -----------
# McMaster 94459A130: M3 brass heat-set insert, knurled.  Pilot Phi 4.0 mm,
# insert OD 5.7 mm (knurled max), insert length 5.0 mm, recommended pilot
# depth 6.0 mm.  Installed with a soldering iron at ~220 deg C, light
# pressure, ~10-15 s per insert.  After install the M3 x 8 SHCS threads
# into the insert from above; the bolt clamps the servo ear down onto
# the boss top surface.
#
# This replaces the earlier "M3 self-tap into Phi 2.5 mm plastic pilot"
# scheme.  The user's audit (May 2026) found that 7 of 12 cradle bolt
# sites had 0.00-1.50 mm of plastic on one side of the Phi 2.5 mm pilot
# in the Y direction -- the cradle outer walls did not extend in Y
# beyond the body footprint, and on the +X column the wire channel ate
# into the wall right where the bolt lived.  The verifier's vertical
# self-tap check (commit b447f88) only confirmed that the pilot
# CYLINDER existed; it never confirmed that the pilot was SURROUNDED
# by enough material radially, so the structural bug shipped.
#
# Switching to a heat-set insert (a) gives real metal threads (replacing
# the borderline thread engagement that the Phi 2.5 mm plastic pilot
# provided in PLA / PA12), (b) requires a Phi 4 mm pilot which is
# physically wider than the previous Phi 2.5 mm pilot's column anyway,
# and (c) gives us license to add a fat Phi ~8 mm boss around each
# pilot so the radial-engagement bug automatically vanishes.  The
# verifier was also extended with an 8-azimuth radial-material probe
# (see ``check_cradle_insert_pockets`` in ``_verify_prototype.py``) so
# the previously missing structural check is now part of the build.
INSERT_M3_PILOT_OD            =  4.0   # mm -- printed pilot Phi (matches
                                        # McMaster 94459A130's
                                        # recommended pilot diameter).
INSERT_M3_PILOT_DEPTH         =  6.0   # mm -- printed pilot depth (insert
                                        # length 5 mm + 1 mm overdrill so
                                        # debris from heat-set insertion
                                        # has somewhere to go).
INSERT_M3_INSERT_OD           =  5.7   # mm -- knurled OD of the installed
                                        # brass body.  The knurl
                                        # displaces / melts the Phi 4 mm
                                        # pilot wall during installation
                                        # to form a press-fit -- standard
                                        # heat-set behaviour.
INSERT_M3_INSERT_LENGTH       =  5.0   # mm -- physical insert length.
INSERT_M3_BOLT_LENGTH         =  8.0   # mm -- M3 x 8 SHCS (unchanged
                                        # stock); threads into the
                                        # insert from above.
INSERT_M3_BOLT_HEAD_OD        =  5.5   # mm -- M3 SHCS head Phi.
INSERT_M3_BOLT_HEAD_H         =  3.0   # mm -- M3 SHCS head height.

# Boss surrounding each insert pilot.  Pilot Phi 4.0 mm + 2 x 1.5 mm wall
# = Phi 7.0 mm minimum.  Round up to Phi 8.0 mm to leave margin for the
# wire-channel reroute on the +X side AND to absorb FDM line-width
# variation around the curved boundary.
#
# Geometry sketch (well-local, one boss at sx*SERVO_MOUNT_HOLE_X_OFFSET,
# sy*SERVO_MOUNT_HOLE_Y_OFFSET):
#
#       boss top at z = SERVO_TAB_Z = 27 mm (servo ear bottom seats on
#                                            this face)
#       +----+--+   ear (factory part of the servo)
#       |    |  |   bolt (M3 x 8 SHCS, head on top)
#       |    |##|<--- heat-set insert (Phi 5.7 mm x 5 mm)
#       |    |##|   pocket (Phi 4 mm x 6 mm, printed)
#       |    |  |
#       +----+--+
#       boss bottom at z = SERVO_TAB_Z - CRADLE_BOSS_HEIGHT_MM = 17 mm
#       boss OD = CRADLE_BOSS_OD = 8 mm
#
# The boss fuses into the existing well outer rim and / or shelf
# material wherever they intersect, so the boss "grows out" of the
# nearest cradle surface rather than dangling in free air.
CRADLE_BOSS_OD                =  8.0   # mm -- boss outer diameter.
CRADLE_BOSS_MIN_WALL_MM       =  1.5   # mm of plastic required radially
                                        # around the pilot (read by the
                                        # verifier's radial-material
                                        # probe in
                                        # ``check_cradle_insert_pockets``).
CRADLE_BOSS_HEIGHT_MM         = 10.0   # mm; boss extends from
                                        # SERVO_TAB_Z DOWN by this much.
                                        # Boss top face = SERVO_TAB_Z so
                                        # the ear bottom rests on top.

# Chassis_bottom yaw cradle channel-side wall-reduction exemption
# (May 2026, commit 5.5/9)
# ---------------------------------------------------------------
# For the LEGACY cradles (coxa_bracket / coxa_link / femur_link) the
# 2 -X heat-set bosses sit on a column with NO wire-cut intrusion
# (the L-corridor exits +X, the boot channel cuts +X, and the -X
# wall is solid).  The verifier's radial-material probe in
# ``check_cradle_insert_pockets`` requires ALL 32 azimuths sampled
# at r = CRADLE_BOSS_OD/2 - CRADLE_BOSS_MIN_WALL_MM/2 = 3.5 mm to
# be inside boss material -- 0/32 air-flagged is the strict
# (pre-2026) tolerance these cradles continue to enforce.
#
# The NEW chassis_bottom integrated yaw cradle (commit 1/9 +
# commit 5.5/9) mirrors the wire-exit L-CORRIDOR to the -X
# (radially-inward) face so the servo harnesses route to the
# per-leg drop slot at chassis-frame (+46.8, +27, +2) without
# crossing the chassis centre line.  That mirror puts the
# corridor's lateral leg through the -X bosses' inner azimuth
# band (corridor y in [-3.5, +3.5], boss y at +/-5 with OD = 8 mm
# = boss y range [+1, +9] and [-9, -1]); the corridor cut carves
# away the inboard ~25 percent of each -X boss circumference --
# specifically the 8/32 azimuths in [3 pi/4, 5 pi/4] facing the
# cradle interior, where the corridor reaches the boss radius.
#
# Engineering rationale for accepting the wall reduction
# -------------------------------------------------------
# Heat-set retention in printed plastic is dominated by the
# knurl-ring's AXIAL bite into the plastic surrounding the
# insert (the knurl teeth dig into the boss wall as the insert
# is pressed in hot, then anneal in place).  With 75 percent
# (24/32) of the boss circumference RETAINED at full
# 1.5 mm-min radial wall thickness AND the chassis_bottom plate
# bonded to the boss BOTTOM (the boss grows out of the plate
# top face via union, so the plate provides full axial
# retention on the boss bottom plane), losing 25 percent of the
# radial knurl coverage costs approximately 25 percent of the
# pull-out force -- a McMaster 94459A130 brass insert is rated
# for ~250 N pull-out at full coverage in PA-CF, so the
# channel-side reduction degrades that to ~190 N.  Each
# servo-tab clamp bolt sees < 30 N tensile load under the
# worst-case yaw-actuator stall torque (DS3225 stall ~25 kg-cm
# = 2.45 N-m, divided over 4 bolts at SERVO_TAB_HOLE_PCD/2 =
# 24.75 mm lever arm gives 24.7 N per bolt; doubled to 50 N
# for shock loading), so 190 N retention leaves 3.8 x safety
# factor on the worst-loaded insert (a 5 x margin is the
# conventional target -- close enough for a prototype with
# bolted redundancy on both -X sites).
#
# The verifier's ``check_cradle_insert_pockets`` applies a
# CASE-SPECIFIC azimuth tolerance: 0/32 strict for the legacy
# link cradles (no wire intrusion on the boss column) and
# 8/32 (= 25 percent) for the chassis_bottom yaw cradle inserts
# (channel intrusion documented above).  Globally lowering
# CRADLE_BOSS_MIN_WALL_MM would weaken the legacy cradles
# unnecessarily, so the relaxation is kept localised to the
# one case that physically requires it.
#
# See the ``_wire_exit_l_corridor`` / ``_boot_clearance_channel``
# function docstrings (in the cradle helpers section below) for
# the geometric split that motivated the chassis_bottom redesign
# and the per-call-site mirror policy that enforces these
# tolerances.

# ---- +X cradle bolts: SELF-TAP fallback (mixed-mode Design E, May 2026) --
# The 2 +X cradle bolts at (sx, sy) = (+1, +/-1) CANNOT use the heat-set
# scheme above: the M3 brass insert wants a Phi 8 mm boss centred at
# (+24.75, +/-5), spanning well-z [17, 27], and that boss footprint
# intersects the inside-+X-wall WIRE CHANNEL (y in +/-3.5) that the
# servo's molded wire-exit boot has to pass through DURING INSERTION.
# The boot is WIRE_BOOT_W = 7 mm wide in Y so the channel CANNOT be
# narrowed in Y to clear the boss; the M3 bolt CANNOT be moved in Y
# because it has to drop through the DS3225's factory-drilled tab hole
# at the fixed servo SERVO_TAB_HOLE_PCD_Y = +/-5 offset; and the bolt
# CANNOT be moved in X for the same factory-tab-hole reason.  So the
# heat-set scheme is physically incompatible with the wire-exit
# channel on the +X column.
#
# The user explicitly accepted reverting these 2 bolts to the original
# Phi 2.5 mm M3 SELF-TAP pilot scheme to unblock servo seating
# ("Maybe we need to go back to just drilling the screws into the
# mounts on the servos if the heat set insert wont fit with the
# channels added").  This trades pull-out durability on these 2
# specific bolts for being able to assemble the robot at all; the
# 2 -X bolts per cradle still use the heat-set insert (no channel
# conflict there), so half of the structural improvement from
# f03d59b survives.  See ``check_cradle_insert_pockets`` in
# ``_verify_prototype.py`` for the mixed-mode probe -- only the -X
# sites are checked for 1.5 mm radial wall material; the +X sites
# are documented as known-thin (0.25 mm on the channel side) and
# the radial check is skipped there.  See also
# ``check_servo_insertion_path`` which probes the boot's swept
# volume through the +X wall during insertion so this exact
# regression cannot ship again.
#
# Pilot geometry mirrors the original pre-f03d59b SHCS_PILOT_OD
# value (= 2.5 mm) and runs from the shelf top down by
# INSERT_M3_SELFTAP_PILOT_DEPTH so the M3 x 8 SHCS engages 5+ mm
# of plastic below the 2.5 mm-thick servo tab.  No boss is added
# around the self-tap pilot -- the existing well wall material
# carries the surrounding plastic, and adding a boss would only
# enlarge the column's footprint without solving the inboard
# (channel-side) wall problem.
INSERT_M3_SELFTAP_PILOT_OD    =  2.5   # mm -- Phi 2.5 mm self-tap pilot
                                        # for the 2 +X cradle bolts on
                                        # every cradle (mixed-mode).  Same
                                        # value as the legacy
                                        # ``SERVO_PILOT_OD`` constant -- the
                                        # heat-set switch never removed
                                        # the M3 self-tap convention, just
                                        # stopped using it for the cradle
                                        # bolts; the wire-channel collision
                                        # forces a partial revert.
INSERT_M3_SELFTAP_PILOT_DEPTH = 10.0   # mm -- pilot depth (matches
                                        # CRADLE_BOSS_HEIGHT_MM).  M3 x 8
                                        # SHCS engages 5.5 mm of plastic
                                        # below the 2.5 mm servo tab; the
                                        # extra 4.5 mm of pilot depth is
                                        # head-room for overdrive without
                                        # bottoming the screw.

# ---- Servo wire-exit slot ------------------------------------------------
# On DS3225 / MG996R / DS3218-class hobby servos the 3-wire harness is
# rigidly moulded into the back-case at the BOTTOM-OUTBOARD corner of
# the body -- specifically the corner where the back case meets the -X
# short-end face (the end OPPOSITE the output spline).  The bundle is
# glued in place, so the servo physically cannot seat in the cradle
# unless that exact corner is open.  The harness then leaves the cradle
# either by routing straight DOWN out of the corner (through the well
# floor) or by routing HORIZONTALLY out the body's -X short-end face
# (through the bottom of the -X wall).  To support both escape paths
# with one boolean cut we put an L-shaped channel at the well's -X
# bottom-outboard corner -- a single rectangular box that punches
# through the floor at the body's -X end AND through the bottom of
# the -X wall above it.
#
# Slot geometry (well-local frame -- see _wire_exit_slot for details):
#     X / Z: use WIRE_SLOT_X_INBOARD, WIRE_SLOT_X_PAST_WALL, WIRE_SLOT_DEPTH,
#            WIRE_SLOT_Z_BELOW_FLOOR for a larger exit than the old 12 x 6 x 8.5
#            mm box so bundled + jacketed harnesses fit.
#     Y: WIRE_SLOT_W stays ~6 mm (symmetric) so M3 pilots at well-y = +/-5 keep
#        ~1+ mm clearance; open more in X/Z instead.
#     Stays well below WELL_RIM_Z so tab-screw seats stay solid.
WIRE_SLOT_W     = 7.0   # mm wide along the body SHORT axis (well Y).  Pilot
                        # columns at y = +/-5 have radius 1.25 mm, so their
                        # inner edges sit at y = +/-3.75; a 7 mm slot has its
                        # edges at y = +/-3.5, leaving 0.25 mm of Y clearance
                        # to the pilots.  Going wider than 7.5 mm starts
                        # punching into the M3 thread engagement zone.
WIRE_SLOT_DEPTH = 13.5  # mm reach UP into the +X wall above the cavity
                        # floor.  Sized so the WIRE_BOOT_* envelope plus
                        # any bent harness above it clears the slot top.
                        # Still well below WELL_RIM_Z (27.25), so M3
                        # pilot tab seats at the rim are untouched.
WIRE_SLOT_X_PAST_WALL = 4.0   # mm beyond outer +X well face for exit
WIRE_SLOT_X_INBOARD   = 2.5   # mm inside cavity +X body face for opening
WIRE_SLOT_Z_BELOW_FLOOR = 4.0  # mm below outer floor for thick boots

# Vertical channel cut into the inside surface of the +X cavity wall so the
# 3-wire harness has a place to lie flat against the wall as it descends to
# the L-shaped exit at the bottom (instead of being pinched between the
# servo back-case and the wall and squirting out over the well rim).
# Pilot clearance: the M3 pilots at (x = +/-24.75, y = +/-5) have ~1.25 mm
# radius columns, so their inner Y edges sit at y = +/-3.75.  The channel
# only spans y = +/-WIRE_SLOT_W/2 (= +/-3.5 with WIRE_SLOT_W = 7), so it
# stays clear of the pilots in Y even when deepened further in X.
#
# Depth math with WIRE_CHANNEL_DEPTH = 6.5 (Design E, May 2026):
#   cavity face at +SERVO_BODY_W/2 + WELL_BODY_CL = +20.7;
#   channel outer x = +20.7 + 6.5 = +27.2;
#   boot outboard face at +SERVO_BODY_W/2 + WIRE_BOOT_PROTRUSION =
#     +26.5, so the channel clears the boot's outer face by 0.7 mm --
#     the channel is now genuinely wide enough for the boot to slide
#     all the way down to seated during insertion, instead of forcing
#     the boot to compress 0.8 mm against the +X wall (the pre-Design-E
#     5.0 mm depth left a 0.8 mm interference at the boot's outer edge
#     for all insertion offsets above 9.4 mm; ``check_servo_insertion_
#     path`` flags this regression);
#   pilot at x = +SERVO_TAB_HOLE_PCD/2 = +24.75, pilot radius 1.25 mm so
#     pilot outboard X edge sits at +26.0;
#   channel (y in +/-3.5) and pilots (at y = +/-5, pilot y_min 3.75)
#     are fully separated in Y -- the two never touch in 3D regardless
#     of the channel's X depth, so deepening the channel does not eat
#     pilot material.
#   Wall thickness over the pilot threads (face-to-pilot distance):
#     +WELL_W/2 = +29 to pilot at +24.75 = 4.25 mm -- still >= 4 mm.
#   Remaining +X wall material outboard of the channel (face-to-channel):
#     +WELL_W/2 - channel_outer_x = +29 - 27.2 = 1.8 mm, just above the
#     three-perimeter MIN_PRINT_T = 1.6 mm floor.
WIRE_CHANNEL_DEPTH    = 6.5   # mm groove depth INTO the +X wall material
WIRE_CHANNEL_TOP_OVER_RIM = 2.5  # mm extension of the channel ABOVE the well
                                  # rim, so wires exiting near the top of the
                                  # back-case (micro servos) still find it.

# ---- Cable zip-tie post (printed-in strain relief) ----------------------
# Every servo cradle (yaw / hip-pitch / knee) gets a small printed post on
# the well's +X OUTER wall, just past where the wire-exit slot opens to
# free air.  At assembly time the user loops a 2-3 mm wide zip-tie around
# the post AND the 3-wire bundle, so any tug on the harness pulls the
# bundle against the post (a printed-in strain relief) instead of yanking
# the JST plug at the PCA9685 header.  Without these posts a single
# accidental snag on a leg cable can pop a servo plug off the PCA at the
# worst possible moment (mid-walk gait); with them the harness fails
# safely (the zip-tie bears the load).
#
# Geometry summary (all three cradles use the SAME ``_cable_zip_post()``
# helper applied in well-local frame and then transformed alongside the
# wire_slot, so the post stays anchored to the well wall regardless of
# how each cradle is rotated into its link's frame):
#
#   * Anchored at the well's OUTER +X face (well_x = WELL_W/2 = +29).
#     Inboard face of the post sits AT the wall (= +29); outboard face
#     at +29 + CABLE_POST_X = +31.
#   * Offset in well-Y by CABLE_POST_Y_OFFSET = +7 mm so the post lives
#     OUTSIDE the wire-exit slot's Y span (slot Y in
#     [-WIRE_SLOT_W/2, +WIRE_SLOT_W/2] = [-3.5, +3.5]).  Without the
#     offset the post would sit IN the slot exit and physically block
#     the harness from leaving the cradle.  +7 mm Y centre leaves
#     +7 - CABLE_POST_Y/2 = +4 mm of well-Y clearance from the slot
#     edge at +3.5.
#   * Z centre at CABLE_POST_Z_CENTRE = +6 mm in well-local Z (i.e.,
#     +6 mm above the cavity floor).  Range +2..+10 mm sits comfortably
#     INSIDE the well's outer +X wall material (the wall is solid from
#     well-z = -WELL_FLOOR_T = -3 up to well-z = WELL_RIM_Z = +27.25 at
#     this Y offset since the channel cut only spans y in [-3.5, +3.5]).
#   * 2 x 6 x 8 mm extents (X / Y / Z) -- small enough to print
#     supports-free as a horizontal nub off the vertical well wall in
#     every cradle's print orientation (coxa_bracket prints cradle-up,
#     coxa_link prints with hip cradle opening +Z after the +90 deg X
#     rotation, femur_link prints spar-broad-face-on-bed; in all three
#     the post is a 2 mm-deep ledge along a vertical wall, with the
#     2 mm bottom face printing as a tiny horizontal bridge that FDM
#     handles natively).
#
# Keep-out audit (verified by check_workspace_self_collision +
# check_cable_clearance + check_wire_slot):
#
#   * Yaw cradle (coxa_bracket): post sits BELOW the bracket flange
#     (well-z in [+2, +10] is below WELL_RIM_Z = +27.25 in well frame;
#     in bracket frame the post is at bracket-z in [-25.25, -17.25],
#     well clear of the chassis_bottom plate at bracket-z in [-4, 0]).
#   * Hip-pitch cradle (coxa_link): after R + delta + well_z_drop +
#     COXA_LIFT lift the post sits at link (x ~ +45, y ~ -42, z ~ +5)
#     -- > 21 mm from the hip pad sweep cylinder centre at
#     (COXA_LENGTH, 0, COXA_HIP_DROP), so well clear of HIP_PAD_R = 20
#     mm sweep.
#   * Knee cradle (femur_link): post sits at femur (x ~ +110, y ~ -42,
#     z ~ +5), well outside the tibia knee-pad neck sweep at
#     (FEMUR_LENGTH = +90, *, 0) with HIP_PAD_R = 20 mm clearance.
#
# Cradle bolt boss columns at well-y = +/- SERVO_TAB_HOLE_PCD_Y/2 = +/-5,
# well-x = +/- SERVO_TAB_HOLE_PCD/2 = +/-24.75, well-z in
# [SERVO_TAB_Z - CRADLE_BOSS_HEIGHT_MM, SERVO_TAB_Z] = [+17, +27].  Our
# post at (well_x ~ +30, well_y ~ +7, well_z in [+2, +10]) is FAR from
# the +X bosses' XY (+24.75, +/-5): distance ~ sqrt(5.25^2 + 2^2) =
# 5.6 mm in XY from the (+24.75, +5) boss centre, well outside the
# Phi CRADLE_BOSS_OD/2 = 4 mm boss radius.  Z ranges don't even overlap.
CABLE_POST_X            = 2.0   # mm protrusion in well +X (outward from
                                 # the well's outer face)
CABLE_POST_Y            = 6.0   # mm width along well Y (slot transverse)
CABLE_POST_Z            = 8.0   # mm height along well Z (slot depth axis)
CABLE_POST_Y_OFFSET     = 7.0   # mm centre offset in well +Y from slot
                                 # centreline (slot at y = 0); see audit
                                 # above for clearance to slot + bosses.
CABLE_POST_Z_CENTRE     = 6.0   # mm well-local centre Z; range
                                 # [Z_CENTRE - CABLE_POST_Z/2,
                                 #  Z_CENTRE + CABLE_POST_Z/2] = [+2, +10]
                                 # sits inside solid wall material.
                                 # NOTE: the coxa_bracket OVERRIDES this
                                 # with BRACKET_CABLE_POST_Z_CENTRE below
                                 # because its yaw well is trimmed off
                                 # below bracket-z = BRACKET_WELL_TRIM_Z
                                 # = -15 mm, and the default well-local
                                 # +6 maps to bracket-z = -21.25 (post
                                 # range [-25.25, -17.25]), ENTIRELY
                                 # below the trim plane -- the trim's
                                 # boolean diff would silently destroy
                                 # the post.  coxa_link / femur_link
                                 # cradles still use +6 because their
                                 # wells are NOT trimmed.

BRACKET_CABLE_POST_Z_CENTRE = 20.25  # mm well-local Z centre for the
                                      # COXA_BRACKET cable_post ONLY;
                                      # raised from the shared
                                      # CABLE_POST_Z_CENTRE = +6 so the
                                      # post sits ABOVE the
                                      # BRACKET_WELL_TRIM_Z = -15 trim
                                      # plane.  In bracket-local frame
                                      # the post Z range becomes
                                      # [+20.25 - 4 - WELL_RIM_Z,
                                      #  +20.25 + 4 - WELL_RIM_Z]
                                      # = [-11.0, -3.0]: 4 mm clear
                                      # above the trim and 3 mm below
                                      # the bracket-flange top at z =
                                      # +15 (bracket-z bounds after the
                                      # trim are [-15, +15]).  Without
                                      # this override the default post
                                      # at well-local +6 (= bracket-z
                                      # in [-25.25, -17.25]) was
                                      # silently boolean-subtracted
                                      # away by the trim and every
                                      # leg lost its bracket-level
                                      # strain relief -- the wire
                                      # harness then hung off the
                                      # chassis_bottom anchor tab
                                      # alone.  Used only by
                                      # ``make_coxa_bracket`` via
                                      # ``_cable_zip_post(z_centre=...)``;
                                      # coxa_link / femur_link continue
                                      # to use the +6 default because
                                      # their wells are not trimmed and
                                      # the +6 placement is already
                                      # audited against the slot, bosses
                                      # and link sweep in the
                                      # ``CABLE_POST_*`` keep-out block
                                      # at the top of this section.

# ---- Per-leg chassis_bottom drop slot (Part B, May 2026) ----------------
# A small 12 x 6 mm slot cut THROUGH the chassis_bottom plate at each of
# the 6 leg positions, located just INBOARD of each coxa_bracket's body
# cutout (which the plate already carries for the servo body's drop-in
# path; see ``_hex_plate``'s body_cutout block).  The slot lets each
# leg's 3-cable harness drop from the bracket-flange-top side of the
# plate INTO the inter-plate volume between chassis_bottom and
# chassis_top -- where the electronics_tray + PCA9685 boards live --
# without the harness having to share the body cutout with the seated
# yaw servo (the body fills ~ 40 x 20 of the cutout's 60 x 31 footprint,
# leaving only ~ 5 mm of margin around the body; cramming 3 wires into
# the margin works but adds risk of pinching the harness against the
# body during assembly).
#
# Slot orientation: LONG axis is along the chassis-radial (bracket +X)
# direction; SHORT axis is tangential (bracket Y).  In each leg's
# bracket frame the PRIMARY slot is centred at:
#     bracket-x = LEG_HARNESS_DROP_X_CENTRE = -46
#                 (inboard of the body-cutout's -X edge)
#     bracket-y = 0  (on the chassis radial line)
# The same hex-leg iterator that ``_hex_plate`` uses to place the body
# cutouts places these slots, so the drop slots always line up with their
# parent cutouts (a regression here is caught by ``check_leg_harness_drop``
# in ``_verify_prototype.py``, which probes EVERY slot in
# ``leg_harness_drop_slots()``).
#
# Jun 2026 wire-room pass: the harness on a real STS3215 leg is a daisy-chain
# of three 3-pin bus cables (servo connectors ~ 8 x 4 mm each), and the
# original 12 x 6 mm port was a tight single drop.  It grew to a 3-port
# perforated band (18 x 8 primary + two 18 x 6 flanks with ~3 mm ribs).
# Aug 2026: the ribs made pushing the Molex connectors through annoying
# (user), so the band is now ONE open 18 x 28 mm port -- the same overall
# envelope as the old 3-port band, just without the ribs -- and the port
# moved 1.5 mm INBOARD (centre -46 -> -47.5, radial span 43.5..61.5): at
# the old centre the ~2.4 mm ligament between the port's outboard edge
# and the body cutout ran the full 28 mm width and tripped the flimsy-
# joints check (was fine when the 3 ribs broke it up); the shift gives a
# ~3.9 mm outboard rib.  Clear of the yaw retainer pilots (tangential
# +/-21; port edge +/-14) -- but NOT of the standoff holes: the original
# claim of ">17 mm tangential" was wrong (a standoff site sits 15 deg
# off the leg axis -> 44*sin(15) = 11.4 mm < the 14 mm half-width), so
# the inboard shift clipped each standoff seat's corner.  Fixed Aug 16
# 2026 with the CHASSIS_STANDOFF_SEAT_PAD_OD pads (see that block);
# each pad bites a ~3.5 x 7 mm corner off the port, which
# check_leg_harness_drop exempts.
LEG_HARNESS_DROP_X_CENTRE  = -47.5  # mm bracket-x (inboard of body cutout)
LEG_HARNESS_DROP_X_EXTENT  =  18.0  # mm along bracket X (radial)
LEG_HARNESS_DROP_Y_EXTENT  =  28.0  # mm along bracket Y (tangential; was 8
                                    # + two flanking 6s with ribs pre-Aug-2026)

# Aug 2026: the two tangential flanking ports merged INTO the primary slot
# (kept as an empty tuple so older callers of leg_harness_drop_slots()
# keep working).
LEG_HARNESS_DROP_EXTRA_SLOTS = ()


def leg_harness_drop_slots():
    """All per-leg cable pass-through slots as
    ``(bracket_x_centre, bracket_y_centre, x_extent, y_extent)`` tuples in the
    leg bracket frame (x = chassis radial, y = chassis tangential): the primary
    drop slot followed by ``LEG_HARNESS_DROP_EXTRA_SLOTS``.

    Single source of truth shared by every chassis_bottom builder that cuts the
    slots (``_hex_plate``, ``_chassis_bottom_full_solid``,
    ``_chassis_bottom_floor_solid``) and by ``check_leg_harness_drop`` so the
    cut geometry and the verifier probe can never drift apart."""
    return ((LEG_HARNESS_DROP_X_CENTRE, 0.0,
             LEG_HARNESS_DROP_X_EXTENT, LEG_HARNESS_DROP_Y_EXTENT),
            *LEG_HARNESS_DROP_EXTRA_SLOTS)

# ---- Chassis_bottom cable anchor tab (Part A continued) -----------------
# RETIRED (May 2026): the per-leg vertical anchor tab that used to hang
# DOWN from chassis_bottom's BOTTOM face was removed so the plate's
# bottom face is fully flat (chassis-z = -CHASSIS_PLATE_T/2 = -2 mm
# everywhere) for easier FDM printing.  The 3-cable harness is now
# anchored by looping a zip-tie THROUGH the per-leg drop slot itself --
# no sub-plate geometry required.  The CABLE_ANCHOR_TAB_* constants
# below are PRESERVED so external references (docs, verifier comments,
# historical commits) keep resolving; they are no longer consumed by
# ``make_chassis_bottom``.
#
# Historical placement rationale (kept verbatim for reference):
# Each leg also gets a SMALL printed tab hanging DOWN from chassis_bottom's
# BOTTOM face, immediately adjacent to its ``leg_harness_drop_*`` slot, so
# the assembler can loop a 2-3 mm zip-tie around the tab AND the 3-cable
# harness BELOW the chassis plate -- exactly where the cradle wire-exit
# bundle makes its U-turn from the cradle's lateral wire-exit (at well +X)
# back inboard to the drop slot.  Anchoring the bundle here rather than
# on chassis_bottom's TOP face dodges the electronics_tray (chassis-z in
# [+5, +8] over chassis-x in [-80, +80] x y in [-70, +65], which fully
# blankets the inter-plate volume directly above each drop slot), and it
# matches the harness's natural routing: the cradle wire-exit fires at
# bracket-z ~ -21 (= chassis-z ~ -19, well below the plate), the wire
# hugs the underside of the plate inboard to bracket-x ~ -46, then comes
# UP through the drop slot.  The tab grabs the bundle just BEFORE the
# slot, so any cable tug puts load on the tab + plate instead of the
# servo's JST plug or the cradle wire-exit boot.
#
# Geometry:
#   * Anchored at chassis_bottom's BOTTOM face (chassis-z =
#     -CHASSIS_PLATE_T/2 = -2 mm); extends DOWN by CABLE_ANCHOR_TAB_H so
#     the bottom face sits at chassis-z = -CHASSIS_PLATE_T/2 -
#     CABLE_ANCHOR_TAB_H = -12 mm.  In bracket-local frame the same range
#     is bracket-z in [-12, -2].
#   * Placed at bracket-x = LEG_HARNESS_DROP_X_CENTRE (= -46 mm), so the
#     tab's radial position is centred on the drop slot itself.
#   * Offset to the +Y side of the drop slot by CABLE_ANCHOR_TAB_Y_OFFSET
#     = +5 mm (slot Y in [-3, +3]; tab centre at +5 leaves a 1 mm gap
#     between the slot's +Y edge at +3 and the tab's -Y face at
#     +5 - CABLE_ANCHOR_TAB_W/2 = +1).  The harness exits the slot in
#     the slot's Y span, so the tab's -Y face is the natural surface
#     for the zip-tie to bear against.
#
# Keep-out audit (verified by check_workspace_self_collision +
# check_cable_clearance):
#   * Bracket-x = -46 is 7 mm INBOARD of the yaw well's inboard wall
#     (well outer face at bracket-x = body_centre_x - WELL_W/2 = -39).
#     Below the plate the well + seated yaw servo body occupy
#     bracket-x in [-39, +19]; tab at bracket-x = -46 sits in OPEN
#     air well clear of that volume.
#   * Bracket-x = -46 is 16 mm INBOARD of the bracket flange's inboard
#     edge at bracket-x = -30; the flange is ABOVE the plate (bracket-z
#     in [+2, +17]) so there's no overlap in either X or Z.
#   * Bracket-y = +5 sits outside the drop slot's Y span [-3, +3] and
#     well inside the chassis_bottom plate footprint.  Below the plate
#     in the inter-plate region just inboard of the bracket nothing
#     else exists -- the chassis_bottom plate's underside between the
#     legs is otherwise empty.
#   * The dynamic leg workspace (coxa_link, femur_link, tibia_link)
#     sweeps OUTBOARD from the yaw axis; the inboard region around
#     bracket(-46, *) is not reachable by any leg part.
#     check_workspace_self_collision catches any regression.
CABLE_ANCHOR_TAB_T   = 2.0    # mm thickness (along bracket X = chassis
                               # radial); thin enough to print supports-
                               # free as a horizontal nub off the plate
                               # underside.
CABLE_ANCHOR_TAB_W   = 8.0    # mm width (along bracket Y = chassis
                               # tangential); enough to anchor a 2-3 mm
                               # zip-tie loop with a comfortable bearing
                               # surface on the -Y face.
CABLE_ANCHOR_TAB_H   = 10.0   # mm height (chassis -Z direction, below
                               # the plate bottom face); enough vertical
                               # purchase to anchor a zip-tie without
                               # snagging on the lower edge.
CABLE_ANCHOR_TAB_X   = LEG_HARNESS_DROP_X_CENTRE  # = -46 mm bracket-x;
                               # tab centred on the drop slot (so the
                               # bundle exits the slot ABOVE the tab
                               # and the zip-tie loop grabs it at the
                               # plate level).
CABLE_ANCHOR_TAB_Y_OFFSET = 5.0  # mm bracket +Y offset from drop slot
                                  # centreline (slot Y in [-3, +3]; tab
                                  # centred at +5 with width 8 spans
                                  # bracket-y in [+1, +9], with a 1 mm
                                  # gap to the slot's +Y edge so the
                                  # cradle harness can sit against the
                                  # tab's -Y face without binding in
                                  # the slot itself).

# ---- Servo well (open-topped bucket holding ONE servo) -------------------
# The well is the structural pocket that the servo drops into during
# assembly:
#
#     1.  The servo body slides DOWN through the well's wide-open top.
#     2.  Its mounting tabs land on the well's rim at z = WELL_RIM_Z.
#     3.  The user drives 4 standard M3 self-tapping screws DOWN through
#         the tab clearance holes into 4 pilot holes drilled vertically
#         through the well's side walls (positioned at x=+/-SERVO_TAB_
#         HOLE_PCD/2, y=+/-SERVO_TAB_HOLE_PCD_Y/2).
#     4.  Above the rim the body extends another (BODY_H - TAB_Z) mm into
#         open air; the gear stack and horn adapter sit on top of that.
#
# The +X / -X walls (where the M3 pilots live) MUST be thick enough to
# hold the pilot hole AT a comfortable distance inside the wall material.
# With WELL_WALL_X = 9 mm: pilot at x = +/-24.75, wall outer at x =
# +/-29, so 4.25 mm of material sits between the pilot and the outer
# face -- plenty for FDM in PLA / MJF in PA12.
WELL_WALL_X  = 9.0   # mm thick on +X / -X faces (must hold the M3 pilot)
WELL_WALL_Y  = 4.5   # mm thick on +Y / -Y faces.  Was 2.5 mm, which gave a
                     # FINAL printed wall of WELL_WALL_Y - WELL_BODY_CL = 1.8 mm
                     # after the body cavity is cut (the inner face of the
                     # wall sits 0.7 mm inboard of the SERVO_BODY_D/2 line to
                     # leave drop-in clearance for the body).  With FDM
                     # nozzle = 0.4 mm we need >= 3 mm of solid wall to fit
                     # three perimeters with margin.  4.5 mm gives 3.0 mm
                     # of real wall AT THE RIM (where the WELL_LEAD_IN_EXTRA
                     # = 0.8 mm chamfer eats an additional 0.8 mm of wall
                     # over the top 1.8 mm of the cavity) and 3.8 mm of wall
                     # everywhere else, comfortably above the three-perimeter
                     # threshold even with FDM line-width variation.
WELL_FLOOR_T = 3.0   # mm bottom-plate thickness.  Was 2.5 mm which printed as
                     # a 2.5 mm-thick frame strip in Z; 3.0 mm gives the
                     # FDM-friendly three-perimeter minimum.  The plate exists
                     # only as a perimeter FRAME around the (open) cavity
                     # opening, tying the +X/-X tab posts to the +Y/-Y walls
                     # at the bracket's print-bed face.  The servo body does
                     # NOT rest on the floor -- it hangs from its mounting
                     # tabs on the rim, with WELL_TAB_FLOAT mm of clearance
                     # from the nominal floor plane (and the cavity itself
                     # punches all the way through the floor, so a body that
                     # exceeds the nominal SERVO_TAB_Z depth simply pokes out
                     # the bottom of the bracket instead of bottoming the tab
                     # above the rim).
WELL_TAB_FLOAT = 1.5  # mm float distance between body bottom and the nominal
                      # cavity floor (z = 0) when the tabs are seated on the
                      # rim.  Was 0 (the body sat on the floor at the same
                      # instant the tabs reached the rim, so any extra body
                      # height left the tabs hanging above the rim).  Bumping
                      # this means the rim, not the floor, defines seating
                      # depth.
WELL_W       = SERVO_BODY_W + 2 * WELL_WALL_X     # 58 mm
WELL_D       = SERVO_BODY_D + 2 * WELL_WALL_Y     # 29 mm (was 25 mm
                                                  # when WELL_WALL_Y = 2.5;
                                                  # the chassis_bottom body
                                                  # cutout below is WELL_D + 2
                                                  # = 31 mm and grew with it,
                                                  # so older chassis_bottom
                                                  # prints made before the
                                                  # wall bump have a 27 mm
                                                  # cutout that the current
                                                  # bracket no longer fits
                                                  # through -- reprint the
                                                  # bottom plate.)
# STS3215 front-face mount: the "rim" is now the FRONT (output) face
# plane, where the printed mount PLATE caps the well and the 4x M2.5
# bolt up into the servo case.  The body inserts from the BACK (-Z,
# open) side and seats its front face flush against the plate underside
# at z = WELL_RIM_Z = SERVO_BODY_H.  (No tab-shelf, no heat-set bosses.)
WELL_PLATE_T = 4.0                # mm -- front mount-plate thickness
WELL_RIM_Z   = SERVO_BODY_H       # mm: front-face plane = plate underside
WELL_H       = WELL_RIM_Z + WELL_PLATE_T          # mm: outer height (plate on top)
# Radius of the compact central mounting pedestal kept at full plate
# height in ``_chassis_yaw_cradle_solid``'s Jun 2026 horn-clearance
# relief.  Must clear the 4 case-screw holes (on the +/-SERVO_MOUNT_SQUARE/2
# = +/-4.9 square, corner radius ~6.9) AND seat the Phi DISC_HORN_OD = 20 mm
# disc horn's full underside -> radius 12 (Phi 24 boss).  The tower top
# OUTBOARD of this pedestal drops to the servo front face so the horn +
# coxa_link hub have a clear swing cone.
HORN_CLEAR_PEDESTAL_R = 12.0      # mm

# ---- Yaw-servo retention floor (RETIRED) --------------------------------
# Historical: a brief two-part build closed the yaw cradle bottom with a solid
# bucket-bottom slab.  Both the deep bucket and that slab are gone -- the
# merged chassis_bottom is a flat plate + folded-in flat floor, and the yaw
# servo is captured from below by the bolt-on yaw_servo_retainer stirrup.  The
# constant is unused (kept only to preserve historical context).
YAW_CRADLE_FLOOR_T = 4.0          # mm -- (retired) bucket-bottom slab thickness

# ---- Yaw-servo retainer strap (Jun 2026) --------------------------------
# LEGACY (pre flat-plate) constants for the original deep-cradle strap; see
# the "Flat-plate retainer re-anchor" block below and make_yaw_servo_retainer
# for the CURRENT capture-stirrup design that supersedes the end-wall anchors.
# A small printed strap (``make_yaw_servo_retainer``) bridges the open
# bottom of each chassis_bottom yaw cradle.  It bolts to the cradle's two
# +/-X end walls (M3 self-tap pilots) and to the servo's 4 bottom-face M3
# holes, so the servo can no longer drop out the open bottom.  The strap
# has an open central window so the rear STS3215 cable bundle passes
# straight through.
RETAINER_ANCHOR_PILOT_OD    = 2.5   # mm -- M3 self-tap pilot in the cradle walls
RETAINER_ANCHOR_PILOT_DEPTH = 8.0   # mm
RETAINER_STRAP_T            = 4.0   # mm -- strap thickness (print-flat)
RETAINER_STRAP_W           = 24.0   # mm -- strap width (Y, across the body).  Was
                                    # 20; widened so the +/-X end-anchor bolts can
                                    # straddle the central +X wire-boot exit channel
                                    # at y = +/-RETAINER_ANCHOR_Y_OFFSET with a
                                    # healthy strap edge wall (Jun 2026 insertion fix)
RETAINER_ANCHOR_Y_OFFSET    = 8.0   # mm -- the cradle's end-wall anchor pilots are
                                    # shifted off the body centreline to y = +/-this
                                    # so they clear the central wire-boot insertion
                                    # channel cut through the +X end wall (the boot
                                    # rides up the cradle's open bottom on the y=0
                                    # axis during insertion).
RETAINER_BOLT_CLEAR_OD      = 3.4   # mm -- M3 clearance through the strap
# Central cable DROP WINDOW (Jun 2026 bottom-exit rework).  The two 5264-3P
# serial-bus connectors are on the STS3215's BACK face and the cables exit
# toward the servo BOTTOM, i.e. straight down onto the capture bar.  The old
# 12 x 6 mm slot was far too small for two 3-wire bundles + their connector
# housings, so it is enlarged to a generous window that clears the bundle
# wherever it drops on the body-bottom footprint while leaving a capture frame
# (the servo 24.8 x 45 mm bottom cannot fall through a 20 x 22 mm hole).
RETAINER_CABLE_SLOT_L      = 22.0   # mm -- drop window RADIAL span (cradle X)
RETAINER_CABLE_SLOT_W      = 20.0   # mm -- drop window TANGENTIAL span (cradle Y)

# ---- Flat-plate retainer re-anchor (Jun 2026 flat-chassis redesign) ------
# The merged chassis_bottom's floor is a SIMPLE FLAT hex slab (z in [-6, -2]);
# it has no deep cradle side walls for the old strap to bolt to, and the
# legacy anchor pattern (yaw_retainer_anchor_centres) falls inside
# the plate's body cutout (no material).  So the strap is re-built as a CAPTURE
# STIRRUP: two arms rise ~20 mm from a cross-bar under the servo body bottom up
# to the flat plate's underside, where they bolt into two blind M3 self-tap
# pilots cut in REAL plate material on the TANGENTIAL flanks of the body cutout.
# The cross-bar physically blocks the hanging yaw servo from dropping (the
# servo is captured between the HIGH-half mount plate on top and this bar
# below).  All coords below are cradle-local: +X = radial outboard (the
# yaw-axis -> hex-edge-midpoint direction), +Y = tangential, origin = yaw axis.
RETAINER_ANCHOR_RADIAL  = -SERVO_OUTPUT_X  # -12.5 mm -- OUTBOARD anchor radial pos
                                           # (under the servo body centreline)
# Jul 2026: a SECOND, inboard anchor pair was added so the retainer bolts to the
# chassis at FOUR points instead of two.  The original two anchors both sat on
# one tangential line at RADIAL = -12.5, so the saddle could still rock about
# that line (pivot in the radial/X direction).  Adding an inboard pair at
# RADIAL_2 = -29 makes a ~16.5 mm x 42 mm rectangular 4-bolt pattern that reacts
# rocking in BOTH axes.  -29 keeps each Phi 12 flange pad fully on the saddle's
# +/-Y side walls (walls span cradle-x [-35.4, +10.2]) and clear of the harness
# drop slots (x in [-55, -37]); the pilot still lands in solid floor because
# y = +/-TANG = 21 is outboard of the body cutout in Y.
RETAINER_ANCHOR_RADIAL_2 = -29.0  # mm -- inboard anchor radial pos (2nd pair)
RETAINER_ANCHOR_TANG    = 21.0   # mm -- anchor tangential pos; > body-cutout
                                 # half-depth (17.9) so the pilot lands in solid
                                 # plate material beside the cutout
RETAINER_PLATE_PILOT_OD = 2.5    # mm -- M3 self-tap pilot in the flat plate
RETAINER_PLATE_PILOT_DEPTH = 3.0 # mm -- blind into the 4 mm plate (>=1 mm cap)
RETAINER_ARM_RADIAL     = 9.0    # mm -- stirrup arm cross-section (radial)
RETAINER_ARM_TANG       = 8.0    # mm -- stirrup arm cross-section (tangential)
RETAINER_CAPTURE_GAP    = 1.0    # mm -- gap below the REAL yaw-servo body bottom
                                 # to the capture bar TOP.  The bar sits this far
                                 # UNDER the servo so it positively blocks drop-out
                                 # (servo can fall <= this then is stopped) without
                                 # preloading the body.  The bar Z is derived from
                                 # the actual placed servo bottom in
                                 # make_yaw_servo_retainer (auto-tracks the yaw
                                 # hang depth), NOT a fixed arm length -- the old
                                 # fixed 14 mm rise left the bar ~6.5 mm ABOVE the
                                 # servo bottom (buried in the body, not under it).
RETAINER_BAR_RADIAL     = 30.0   # mm -- capture cross-bar span (radial)
RETAINER_BAR_TANG       = 46.0   # mm -- capture cross-bar span (tangential,
                                 # reaches both arms at y = +/-RETAINER_ANCHOR_TANG)
# ---- Tangential GRIP JAWS (Jun 2026 anti-wobble) ----------------------------
# The yaw STS3215 body hangs BELOW the chassis floor (z < plate_bot), so once
# clear of the well the only lateral support is this retainer.  The two arms
# anchor at y = +/-RETAINER_ANCHOR_TANG (= 21, out in solid plate beside the
# body cutout), leaving their inner faces 4.6 mm OFF the servo's +/-Y (depth)
# faces -- the dangling body could rock tangentially in that slop (user report:
# "the retainer is loose, the servo wobbles").  Each arm now carries an inward
# GRIP JAW that closes that gap to a snug-but-assemblable slip fit: the jaw
# inner face lands RETAINER_BODY_GRIP_CL off the SERVO_BODY_D/2 face, so the
# body still slides straight DOWN the Z channel into / out of the jaws (no
# interference, no insertion block) but cannot rock sideways once seated.
# 0.15 mm/side is tight enough to kill the wobble yet > one FDM line-width of
# play so the body seats by hand; this is INTENTIONALLY far tighter than the
# shared WELL_BODY_CL (0.7) cavity clearance, which is left untouched so the
# cradle/well cavities keep their drop-in fit.
RETAINER_BODY_GRIP_CL   = 0.15   # mm -- jaw inner face clearance per +/-Y face
RETAINER_GRIP_WELD      = 1.0    # mm -- jaw overlap INTO the arm (clean weld)

# ---- Stand-off base for cable run-out (Jun 2026 bottom-exit rework) ----------
# The cables drop straight down through the enlarged window and would be pinned
# against whatever the bar rests on.  So the bar stands RETAINER_LEG_H above a
# flat BASE plate on 4 corner posts, opening a cable run-out cavity between the
# base and the bar (OPEN on all four sides between the posts) where the bundle
# turns and exits laterally instead of being crushed.  The base carries the
# full bar footprint so it is the LARGEST downward face -> it becomes the print
# bed plane and the part stays flat-bottom printable (4 bare legs would leave
# the big bar face floating > the 5 mm support tolerance above small feet).
RETAINER_LEG_H      = 7.0   # mm -- cavity height (bar bottom -> base top)
RETAINER_LEG_W      = 6.0   # mm -- corner post square cross-section
RETAINER_LEG_INSET  = 2.0   # mm -- post inset from the base/bar outer edges
RETAINER_BASE_T     = 2.5   # mm -- stand-off base plate thickness

# ---- Anti-rotation SADDLE (Jun 2026 yaw-servo-retainer redesign) ------------
# Replaces the capture-stirrup above.  Root cause it fixes (user report + the
# _chk diagnostic): the old stirrup threaded its 2 anchor bolts UP through a
# ~20 mm arm into the floor with a SOLID stand-off base SEALING the bolt axis
# from below -> undrivable (a ~38 mm screw down 34 mm of part), and held the
# case with 1 mm of axial slop + only two 0.15 mm-clearance side jaws (no
# radial / rotational key).  The yaw CASE cannot actually fall -- it hangs from
# the output shaft via the servo's internal bearings -> disc horn -> coxa_link
# -> 6805 yaw tower -> chassis -- so the real need is ANTI-ROTATION (reaction
# torque) + anti-wobble + a DRIVABLE chassis anchor.  (The rear idler boss
# rotates WITH the output, so a passive horn there would LOCK yaw -- hence we
# key the CASE, not the boss.)
SADDLE_BODY_CL      = 0.12  # mm -- snug wall clearance per case face (anti-rotation)
SADDLE_WALL_T       = 3.0   # mm -- saddle wall thickness
SADDLE_FLANGE_T     = 5.0   # mm -- top-flange / screw-hole boss depth, z[-11, -6].
                            #       Jun 2026: kept THICK (user: "I liked the thicker
                            #       walls for the screw holes") -- the head does NOT bear
                            #       on the bottom face; instead a COUNTERBORE recesses it
                            #       (SADDLE_HEAD_CB_*), so the head seat is at z=-9 and the
                            #       M3x6 tip still lands at the -3 pilot bottom (3 mm bite,
                            #       1 mm blind cap preserved).  An M3x8 would need the seat
                            #       at -11 (no recess) or overshoot the 4 mm plate.
# Recessed-head counterbore in each thick screw-hole boss (user: "leave room for
# the screw head to go lower").  A Phi SADDLE_HEAD_CB_OD bore opens DOWNWARD from
# the flange bottom (-11) up SADDLE_HEAD_CB_DEPTH, so the M3 SHCS head recesses
# into the boss and bears on the counterbore shoulder at z = -11 + depth = -9
# (driver still enters straight up the open cavity into the down-facing socket).
SADDLE_HEAD_CB_OD    = 6.0  # mm -- counterbore dia (M3 SHCS head Phi 5.5 + clearance)
SADDLE_HEAD_CB_DEPTH = 2.0  # mm -- counterbore depth from the flange bottom (head recess)
SADDLE_FLOOR_T      = 3.0   # mm -- anti-drop backstop floor under the case back face
SADDLE_ANCHOR_PAD_R = 6.0   # mm -- solid flange-tab radius (head bearing + wall weld)
SADDLE_FLOOR_X_OUT  = -2.0  # mm -- +X edge of the backstop floor
# Jun 2026 case-length reconciliation (USER MEASURED the real STS3215, 1 horn on:
# 39 mm from the driven-horn TOP to the FLAT case bottom, centre rear boss lower
# still).  The model's driven-horn top sits SERVO_BODY_H + DISC_HORN_H = 36.3 mm
# above the case back face (SERVO_BODY_H = 34.3 is the STEP mount-HOLE-plane gap,
# NOT the full case length), so the REAL flat bottom hangs ~2.7 mm BELOW the
# modeled back face.  SERVO_BODY_H is the FROZEN kinematic anchor (JOINT_HORN_TOP_Z,
# passive-horn planes, well, clamp caps) so we do NOT move it globally; instead the
# yaw SADDLE references the real bottom via this local drop so its case-hugging
# walls run ~3 mm TALLER and SEAT FLUSH (the old short ref propped the saddle low,
# the backstop hitting the longer real case ~3 mm before the flange reached the
# floor).  Rounded to 3.0 (>= the user's ">=3 mm" ask; +0.3 mm anti-drop gap also
# clears the lower rear boss through the central drop window).
SADDLE_CASE_LEN_FIX  = 3.0  # mm -- extra case-bottom drop for the yaw saddle walls/backstop
# Jun 2026 wire-exit follow-up (user: "doesnt leave space for the wires that come
# out of the center of the bottom").  The harness exits the CENTER of the case
# back (bottom) face and drops STRAIGHT DOWN, so the backstop floor is opened
# into a perimeter FRAME: a SADDLE_FLOOR_RIM-wide ledge on the -X + -/+Y edges
# (still catches the case back-face rim as an anti-drop backstop AND is the
# print bed face) with a big central DROP WINDOW so the wire falls free into the
# open under-chassis cavity.  (We do NOT screw into the rear boss/horn to "cover"
# it: that boss rotates 1:1 with the yaw output, so a chassis-fixed screw into it
# would LOCK the yaw DOF -- the case-keyed walls give the anti-rotation instead.)
SADDLE_FLOOR_RIM    = 5.0   # mm -- backstop-frame ledge width on the -X / +-Y edges

# ---- Aug 2026 retainer ground FEET (v4, four corner poles) --------------------
# History: 38 mm open-cage "permanent ground stand" (replaced the belly
# stilts) -> deleted in the flat-belly rework -> v2 C-shell foot (flimsy)
# -> v3 tripod to a central Ø30 disk -> user: the disk sits directly
# under the case-back servo PLUG (the harness plugs into the case bottom
# centre, through the floor drop window) and makes it hard to reach.  v4
# clears the whole underside: FOUR slender corner POLES, each ending in a
# small ground pad, so the plug can be (un)mated from below/side with
# nothing in the way.  Depths below the chassis underside (plate_bot =
# -6): saddle floor bottom ~24.1 mm, case-boss head plane ~24.6 mm.  A
# 34 mm foot leaves ~9 mm of protected air under the wire exits when the
# robot belly-sits.
#
# Pole placement must dodge EIGHT vertical driver corridors (verifier):
#   * Φ12 PHILLIPS under each M2.5 case screw (-8.3/-32.8, +-10.2),
#     from the boss head plane (~-30.6) down 80 mm;
#   * Φ8 HEX_KEY under each M3 flange anchor (-12.5/-29, +-21),
#     from the head recess (-9) down 30 mm (reaches z -39!).
# All four poles run FULL HEIGHT, tops flush with plate_bot (same top Z,
# user Aug 2026 -- all posts bear on the chassis underside).  FRONT poles
# weld flat onto the +-Y side-wall outer faces near the +X end (full wall
# height).  REAR poles sit diagonally off the saddle's rear corners --
# outside the x2 Φ12 corridors -- tied in by a corner GUSSET in the
# backstop-floor z band, whose underside stays ABOVE the M2.5 head plane,
# so it may legally bridge OVER the screw corridor.
#
# Print orientation: v4 flips the part FLANGE-DOWN (see
# print_orientation._reorient_yaw_servo_retainer) -- four small pads can
# no longer be the largest downward plane, so feet-down would trip the
# flat-bottom guard on the floating backstop floor; flange-down the big
# flange plane is the bed and the poles print as clean vertical columns.
RETAINER_FOOT_H       = 34.0    # mm -- chassis underside -> foot tip
RETAINER_POLE_W       =  8.0    # mm -- square pole side
RETAINER_POLE_FRONT   = (5.5, 17.5)    # cradle (x, |y|) of the +X pole pair
RETAINER_POLE_REAR    = (-41.5, 19.5)  # cradle (x, |y|) of the -X pole pair
RETAINER_PAD_OD       = 12.0    # mm -- small ground pad under each pole
RETAINER_PAD_H        =  3.5    # mm -- pad thickness
RETAINER_PAD_CHAMFER  =  0.8    # mm -- pad bottom edge break

# Jun 2026 STEP-driven rear case-mount capture (CORRECTED, 3rd pass; the prior two
# passes wrongly bolted the HORN bolt circle).  Re-parsed STS3215_c.step and
# classified EVERY hole by solid: the only Phi 2.5 four-hole crosses (STEP
# (X,Z)=(+-7,0),(0,+-7) at the OUTPUT axis, both the +Y front face s17 and the -Y
# rear face s18) are the DISC-HORN bolt circle -- NOT a case mount.  The model's
# own SERVO_BODY_H constant documents the real case mounting holes: it is the gap
# "between the two 4x M2.5 hole planes (STEP)" = the BACK and FRONT(output) case
# faces.  On the FIXED rear (back) case face (STEP solid 1) those 4 holes sit at
# STEP (X,Z) = (8.3, +-10.2) and (32.8, +-10.2), drilled along the output axis.
# Frame: cradle_x = -STEP_X, cradle_y = STEP_Z, cradle_z = STEP_Y (output, UP), so
# they map to cradle (x,y) = (-8.3,+-10.2) [18.5 mm from the +X/output-end "top",
# the user's landmark, just above the centre wire boot] and (-32.8,+-10.2) [43 mm].
# All 4 are on the FIXED case shell, clear of the rotating idler (at the axis,
# r~0) and the centre-bottom wire exit (cradle x~-14).  Their axis IS the output
# axis = WORLD Z (vertical), so the saddle drives 4 M2.5 SELF-TAPPERS straight UP
# from a backstop boss under each hole, head recessed in a counterbore, driven up
# the open under-chassis cavity (the +Y/-Y depth side-walls run PARALLEL to these
# holes, so they cannot host them -- the capture is vertical, not through a wall).
SADDLE_CASE_HOLE_X1     = -8.3   # mm -- cradle x of the landmark rear-face hole pair
SADDLE_CASE_HOLE_X2     = -32.8  # mm -- cradle x of the companion rear-face hole pair
SADDLE_CASE_HOLE_Y      = 10.2   # mm -- cradle |y| of the rear-face holes (STEP Z)
SADDLE_CASE_SCREW_OD    = SERVO_MOUNT_SCREW_OD  # 2.7 mm -- M2.5 clearance bore in boss
SADDLE_CASE_SCREW_BITE  = 2.5    # mm -- self-tap bite into the shallow (~2.8) rear case hole
SADDLE_CASE_SHANK       = 3.5    # mm -- real-case face down to the head bearing (= boss height)
SADDLE_CASE_SCREW_LEN   = SADDLE_CASE_SHANK + SADDLE_CASE_SCREW_BITE  # 6.0 -- M2.5x6 self-tap
SADDLE_CASE_BOSS_R      = 3.4    # mm -- backstop boss pad radius (hosts bore, welds to wall/rim)
# The head bears FLAT on the boss UNDERSIDE (boss bottom = head plane = zc - SHANK)
# -- NO counterbore, so no boss material sits below the head to foul the driver
# cone (Phi-12 Phillips clears straight up the open cavity).  Boss height = SHANK.

WELL_BODY_CL = 0.7   # mm clearance on every body face inside the well.
                     # FDM in PLA / PETG can swallow ~0.3 mm per side just in
                     # line-width / shrinkage, so 0.4 mm is too tight for a
                     # drop-in fit on most desktop printers.  0.7 mm leaves
                     # 0.4 mm of real-world wiggle without the body rattling.

# ---- Clamshell servo retention (Jun 2026 sandwich-link redesign) ---------
# The earlier front-face screw mount could NOT coexist with the dia-20 disc
# horn: the 4 case screws sit on the 9.8 mm square (radius ~6.9 mm) which is
# INSIDE the horn's 10 mm radius, so the horn could neither seat nor spin --
# and the body had to be pressed ~34 mm into a closed tube to reach the
# plate.  The sandwich fixed side now holds the servo by its BODY:
#   * 3 walls (both +/-X end walls + the -Y long wall) locate the body;
#   * the +Y long face is OPEN so the body drops in LATERALLY;
#   * a separate ``make_servo_clamp_cap`` bolts over the +Y face and clamps
#     the body against the -Y wall (2x M3 into the +/-X wall +Y ends);
#   * the output face is left OPEN (Phi HORN_CLEAR_OPENING_OD bore through
#     the top lip) so the disc horn seats on the spline and spins free; the
#     bore opens into the +Y mouth so the fitted horn can lift out with the
#     servo and clamp cap.
HORN_CLEAR_OPENING_OD  = 24.0   # = DISC_HORN_OD (20) + HORN_CLEAR_OPENING_MARGIN
                                #   (4 mm); tied to DISC_HORN_OD by an assert at
                                #   the DISC_HORN_OD definition (single source)
# Sep 2026 serviceability: the circular output opening continues toward the
# OPEN +Y clamp face as a straight, same-width slot through the output lip.
# With the clamp bolts removed, the servo + fitted disc horn + clamp cap can
# therefore lift together along +Y; the old closed, rounded bridge over the
# horn trapped the fitted disc behind the output lip.  The swept slot consumes
# exactly one of the four small front-case capture sites, (x, y) =
# (+4.2, +10.25); the other three remain.  It stops at y=0 because the circular
# opening already clears the starting half of the horn envelope.
HORN_SERVICE_SLOT_W   = HORN_CLEAR_OPENING_OD
WELL_LIP_SLIDE_CL      = 0.4    # lift the top retaining lip off the front face
CLAMP_CAP_T            = 5.0    # clamp-cap flange thickness (Y)
CLAMP_TONGUE_INTERF    = 0.5    # mm -- PRESS-FIT: the tongue reaches this far PAST
                                #       the SEATED body +Y face (into the body) for a
                                #       snug press fit.  History: 0 (flush) left the
                                #       old ~1.5-2 mm rattle -- the tongue stopped at
                                #       the +Y CAVITY face and the body floated.
                                #       Jun 2026 went to 1.0 (user: "the part that
                                #       comes down and touches the servo should come
                                #       down 1 more mm"); Aug 26 2026 bench: 1.0 was
                                #       too much -- the tongue bottoms on the rigid
                                #       body ~1 mm before the flange meets the wall
                                #       ends, so tightening the 2 cap bolts visibly
                                #       BOWED the cap top around the screws (the
                                #       counterbore leaves only 2 mm of flange under
                                #       each head).  Halved to 0.5: still a real
                                #       interference (preload from bolt tension +
                                #       print fit forces the body against the -Y
                                #       wall) but the forced deflection is halved.
CLAMP_BOLT_PILOT_OD    = 2.5    # M3 self-tap pilot in the +/-X wall +Y ends
CLAMP_BOLT_PILOT_DEPTH = 12.0   # mm thread engagement into the wall
CLAMP_BOLT_CLEAR_OD    = 3.4    # M3 clearance through the cap flange
CLAMP_BOLT_X           = WELL_W / 2.0 - WELL_WALL_X / 2.0   # +/-27.2 (wall centre)
CLAMP_BOLT_Z           = SERVO_BODY_H / 2.0                 # ~17.15 (mid-body)
# Jun 2026 head-inset fix: the 2 M3x8 SHCS heads (Phi 5.5, 3.0 mm tall) bore on
# the flange OUTER (+Y) face and stood ~3 mm PROUD.  The femur hip yoke (user-
# confirmed; since Jul 2026 part of the one-piece femur_link) and the
# tibia_knee_yoke sweep PAST that +Y face and clear the
# flange itself by only ~3 mm at the closest ROM pose, so the proud head ate the
# whole margin and BRUSHED.  A counterbore opens from the +Y outer face and runs
# CLAMP_HEAD_CB_DEPTH inward (-Y) so the head recesses FLUSH with the face; the
# M3x8 simply threads CLAMP_HEAD_CB_DEPTH deeper into the +/-X wall pilot
# (engagement ~3 -> ~6 mm; CLAMP_BOLT_PILOT_DEPTH = 12 mm has room) so the screw
# length is UNCHANGED.  Counterbore r = 3 at the bolt (4.5 mm inboard of the
# flange edge) leaves a 1.5 mm rim, and the 2 mm of flange surviving below the
# shoulder carries the clamp preload.
CLAMP_HEAD_CB_OD       = 6.0    # mm -- head counterbore dia (M3 SHCS head Phi 5.5 + clearance)
CLAMP_HEAD_CB_DEPTH    = 3.0    # mm -- recess from flange OUTER face (= M3 head height -> flush)
# Jun 2026 snug-fit (user, on the femur KNEE bracket: "let the clamp come down
# 1 mm lower for a snug fit; the inside on the x dimension should also be reduced
# by 1 mm").  The cradle (_servo_well_solid) + clamp cap (make_servo_clamp_cap)
# are ONE shared part used at BOTH the hip and knee sandwich joints (same
# STS3215), so these snug-fit tweaks are applied to the shared part -> both
# joints get the tighter fit.  Both are sub-mm-to-1 mm and stay inside positive
# clearance (servo still inserts; guarded by check_servo_clearance /
# check_servo_insertion_path).
#   * INSIDE_X_TIGHTEN: shrink the body cavity (and the matching clamp tongue /
#     drop-in slot) in X by this TOTAL (0.5 mm/side), 0.7 -> 0.2 mm/side body
#     clearance, killing the X slop the user felt.  Interpreted as 1 mm TOTAL.
#   * CLAMP_SEAT_DROP: the clamp-cap pressing TONGUE reaches this much FURTHER
#     (in -Z, past the body back-face plane) so it "comes down lower" and adds a
#     -Z backstop lip behind the body's outboard +Y edge for a snug seat.  The
#     bolt-bearing flange + the 2 cap bolt holes are UNCHANGED (still coaxial
#     with the wall pilots -- guarded by check_clamp_cap_alignment).
WELL_INSIDE_X_TIGHTEN  = 1.0    # mm -- total X-cavity reduction (0.5/side)
CLAMP_SEAT_DROP        = 1.0    # mm -- clamp tongue reaches this far past z=0 (-Z)
#
# BACK-FACE HOOK (Aug 18 2026, user: "make the hip clamp cap and the knee
# clamp cap have a small extra part that goes over part of the back of the
# motor to help hold it in place without blocking where the wires come
# out"; rev 3 same day: "it has to go over the back of the servo a bit ...
# otherwise it isnt doing anything"; rev 4, Aug 19: "reduce the width of
# that tab to 10mm to give more room for the servo plug but have it come
# down 5mm further closer to the middle of the back of the servo").  An
# L-shaped hook near the -X end of the +Y edge:
#   * a WALL dropping CLAMP_BACK_HOOK_T = 5.5 mm past the back plane along
#     the +Y edge strip (y >= tongue_y0 = 11.9 -- flat rim at z = 0, proven
#     by the seated CLAMP_SEAT_DROP lip; the raised ~1.8 mm centre platform
#     around the 5264 bay stops short of it);
#   * a SHELF turning inboard (-Y) to y = CLAMP_BACK_HOOK_Y0 = 5.0, i.e.
#     lapping 7.4 mm OVER the back of the servo, well toward its middle.
#     Its top face sits FLUSH with the back plane at z =
#     -CLAMP_BACK_HOOK_SHELF_Z = 0 (rev 8, Aug 19, user: "the bigger hook
#     cant come 1mm [inside] it needs to be flush with the top" -- the STS
#     back DOES step up in this wire-end band, so the rev-6 1 mm press
#     bite jammed; flush rides the surface and still blocks any -Z
#     back-out immediately).  History: rev 3 started 2.2 mm down "to
#     clear the platform" (let the body float ~2 mm -- not a hold), rev 5
#     rode 1 mm down, rev 6 bit 1 mm inside (jammed on the bump).  The
#     HORN-SIDE mini hook below keeps its 1 mm press bite -- that end of
#     the back is genuinely flat ("the small hook is perfect because the
#     sts motor doesnt have a bump there"), so it alone preloads the body
#     against the cradle's front retaining lip while this shelf takes the
#     back-out load.
# Keep-outs boxing it in:
#   * the 10 mm x band [X0, X1] = [-17.7, -7.7] sits at the -X end of the
#     edge: x >= -17.7 stays 0.5 mm clear of the cradle's rear retention
#     tab (+X edge at -18.2), and stopping at -7.7 leaves the whole
#     x > -7.7 corridor OPEN for the 5264 plugs' cables to bend out
#     sideways (rev 4: the deep shelf now reaches beside the plug bay --
#     ports centred (x -10, y 0), housings to y ~ +/-5 -- so the plugs
#     still plug straight in below the shelf and their wires escape
#     through the open corridor, not under the tab);
#   * the deep corner (-7.7, 5.0) is r ~20.8 from the output axis at
#     (+12.5, 0) -- comfortably outside the swinging yoke pad sweep
#     (r 16.75) and the Phi 20 passive horn;
#   * the depth stops at the REAR TAB's proven plane: the swinging yoke
#     arm's inner face passes at z = -7, so 5.5 keeps the same 1.5 mm
#     running clearance the tab has.
# Cap install (slide -Y onto the well): the flush shelf skims the back
# surface on the way in.  Plugs are inserted straight up into the
# recessed bay AFTER capping (plug bodies stay at y <= ~5, beside the
# shelf).
CLAMP_BACK_HOOK_T       = 5.5  # mm -- hook depth past the back plane
CLAMP_BACK_HOOK_X0      = -17.7 # mm -- -X end (0.5 mm clear of the rear tab edge)
CLAMP_BACK_HOOK_X1      = -7.7  # mm -- +X end (10 mm wide; plug-cable corridor beyond)
CLAMP_BACK_HOOK_Y0      = 5.0   # mm -- shelf inner reach: 7.4 mm over the back face
CLAMP_BACK_HOOK_SHELF_Z = 0.0   # mm -- shelf top FLUSH with the back plane (rev 8;
                                #       sign: top face = -CLAMP_BACK_HOOK_SHELF_Z)
#
# HORN-SIDE MINI HOOK (Aug 19 2026, user: "add a tiny hook on the other
# side where the passive servo horn is?  It needs to be right at the end
# and leave a gap for the servo horn").  A second, much smaller hook at
# the +X (output) end corner of the +Y edge.  It MUST be tiny -- the
# depth budget there is nothing like the wire end's:
#   * the yoke pads sweep a FULL r 16.75 circle about the output axis
#     (+12.5, 0) from z = -2 down to -7, and that circle covers the whole
#     +X end of the back face (the body corner (22.7, 11.4) is at r 15.3)
#     -- so NOTHING at this end may reach below z = -2.  The hook bottoms
#     at z = -CLAMP_SEAT_DROP = -1.0, the plane the full-width tongue lip
#     has bench-proven over the pad sweep (1 mm running clearance);
#   * the Phi 20 passive horn itself stands 2 mm proud (z -2..0, r 10):
#     the hook's inner corner (18.5, 9.4) sits at r 11.2 -- a >= 1.2 mm
#     GAP all round the horn (and clear of the idler hub under it);
#   * the top face bites 1 mm INSIDE the back surface, same press-fit as
#     the big shelf, so BOTH ends of the body get preloaded against the
#     cradle's front retaining lip.
# Net: a 4.4 x 3 mm corner pad, 2 mm tall (z -1..+1), gripping the back
# corner right at the end of the servo.
CLAMP_HORN_HOOK_X0 = 18.5  # mm -- inner x edge (gap to the horn: r >= 11.2)
CLAMP_HORN_HOOK_Y0 = 9.4   # mm -- inner y reach: 3 mm lap over the back face
#
# YOKE-SWEEP EDGE CHAMFER (Aug 19 2026, user: "rounding the edge on the
# side where the yoke passes helps make sure theres not small friction").
# The swinging yoke's SPINE sweeps across the cap's outer (+Y) face with
# only ~3 mm at closest ROM, travelling in x -- so any graze catches on
# the VERTICAL (z-parallel) edges standing on that face.  All six get a
# 45 deg chamfer (leg = this): the two flange ends, the two wire-end hook
# wall edges (below z = 0 only -- the flange face above stays flat), and
# the two horn-end pad edges.  Horizontal edges are parallel to the sweep
# and cannot snag; mating faces (tongue / lip / hook undersides) stay
# sharp for registration.  Prints cleanly flange-down (edges rise at
# 45 deg from the bed).
CLAMP_YOKE_EDGE_CHAMFER = 1.0  # mm -- 45 deg chamfer leg on outer-face vertical edges


def servo_clamp_bolt_centres():
    """SINGLE SOURCE OF TRUTH for the sandwich-joint (hip + knee) clamp-cap
    bolt pattern, expressed in the shared well-local frame (origin = servo
    body BACK-face centre, +X = body long axis, +Y = body depth, +Z =
    output).

    Returns a list of ``(x, z)`` bolt centres; the bolt AXIS is +/-Y for
    every bolt (the cap pulls -Y into the +Y wall-end pilots).  The mating
    Y coordinate differs per side of the joint -- the cradle pilot threads
    into the wall END face, the cap clearance hole sits in the flange -- so
    only the (x, z) pair and the Y axis are shared; that pair is the thing
    that has to stay coaxial.

    Consumed by ``_servo_well_solid`` (cradle/bracket PILOTS),
    ``make_servo_clamp_cap`` (cap CLEARANCE holes) and
    ``fastener_registry._emit_clamp_cap_fasteners`` (the fastener
    instances) so the cap holes and the cradle pilots cannot drift out of
    coaxiality: every consumer reads the SAME (x, z) here.  A drift is
    additionally guarded at build time by
    ``_verify_prototype.check_clamp_cap_alignment``.
    """
    return [(sx * CLAMP_BOLT_X, CLAMP_BOLT_Z) for sx in (-1, 1)]


def yaw_retainer_anchor_centres():
    """SINGLE SOURCE OF TRUTH for the yaw cradle <-> ``yaw_servo_retainer``
    strap END-WALL anchor-bolt pattern, in cradle-local XY (origin on the
    yaw/output axis, +X = outboard radial, +Y = tangential).

    The yaw servo has NO clamshell clamp cap (its output points up and the
    body inserts from the open bottom); it is trapped instead by the bolt-on
    ``yaw_servo_retainer`` strap, whose 2 end-wall anchor bolts thread up
    into ``RETAINER_ANCHOR_PILOT`` pilots in the +/-X wall bottoms.  The bolt
    AXIS is Z for both anchors.

    Returns ``[(x, y), ...]``.  Consumed by ``_chassis_yaw_cradle_solid``
    (the cradle PILOTS) and ``make_yaw_servo_retainer`` (the strap clearance
    holes) so the two cannot drift apart; also guarded by
    ``check_clamp_cap_alignment``.
    """
    body_centre_x = -SERVO_OUTPUT_X
    outer_w = WELL_W + 2.0 + 2.0 * CRADLE_BOND_STRIP_MM
    cav_half_x = SERVO_BODY_W / 2.0 + WELL_BODY_CL
    ax_minus = body_centre_x - 0.5 * (cav_half_x + outer_w / 2.0)
    ax_plus = body_centre_x + 0.5 * (cav_half_x + outer_w / 2.0)
    return [(ax_minus, -RETAINER_ANCHOR_Y_OFFSET),
            (ax_plus, +RETAINER_ANCHOR_Y_OFFSET)]


def chassis_lower_retainer_anchor_centres():
    """SINGLE SOURCE OF TRUTH for the FLAT-PLATE yaw retainer anchor pattern
    (Jun 2026 flat-chassis redesign; Jul 2026 4-point rework), in cradle-local
    XY (origin on the yaw/output axis, +X = outboard radial, +Y = tangential).

    FOUR anchors form a rectangle: an OUTBOARD pair at RADIAL = -12.5 and an
    INBOARD pair at RADIAL_2 = -29, each straddling the body centreline
    tangentially at y = +/-TANG = 21 -- just OUTSIDE the plate's body cutout
    (|y| > body-cutout half-depth) so every pilot lands in solid plate
    material and clear of the harness drop slots (x in [-55, -37]).  The
    two-per-line -> four-point pattern reacts saddle rock in BOTH axes (the
    old 2-bolt line could still pivot about itself in the radial direction).

    Consumed by ``_chassis_bottom_floor_solid`` (the floor PILOTS folded into
    the merged ``make_chassis_bottom``), ``make_yaw_servo_retainer`` (the
    flange-tab clearance holes + counterbores), ``_emit_yaw_retainer_anchor_
    fasteners`` (the BOM/engagement/screwdriver-access fasteners) and
    ``check_clamp_cap_alignment`` so they cannot drift apart.  The bolt AXIS is
    Z for all four anchors.

    Returns ``[(x, y), ...]``.
    """
    return [(RETAINER_ANCHOR_RADIAL,   -RETAINER_ANCHOR_TANG),
            (RETAINER_ANCHOR_RADIAL,   +RETAINER_ANCHOR_TANG),
            (RETAINER_ANCHOR_RADIAL_2, -RETAINER_ANCHOR_TANG),
            (RETAINER_ANCHOR_RADIAL_2, +RETAINER_ANCHOR_TANG)]

# ---- "Drop-in" assembly features ----------------------------------------
# Two changes to the otherwise solid bucket geometry that make seating the
# servo body MUCH easier without compromising structural function:
#   1. Finger-access notches cut through the +Y / -Y walls above a short
#      bottom pocket.  The bottom pocket guides the servo into the cavity;
#      the notches let you reach in from the sides with thumb + forefinger
#      to pinch / wiggle the body the rest of the way down.  The notches
#      do NOT touch the +X / -X walls (where the M3 pilots and tab seats
#      live), so the bracket's structural posts are unchanged.
#   2. A short lead-in chamfer at the top of the cavity that opens the
#      mouth slightly wider than the body so the body can self-align on
#      its way down rather than catching on a sharp inside corner.
WELL_BOTTOM_POCKET_H = 6.0   # mm of full +Y/-Y wall below the notch (guides
                              # the bottom of the servo body into the cavity)
WELL_FINGER_NOTCH_W  = 24.0  # mm wide (along body long axis X) — wide enough
                              # to pinch the body but narrow enough to leave
                              # ~8 mm of wall material at each +X/-X tab post
WELL_LEAD_IN_H       = 1.8   # mm tall rim chamfer
WELL_LEAD_IN_EXTRA   = 0.8   # mm extra clearance per side at the very top of
                              # the cavity, tapering down to WELL_BODY_CL

# ---- Chassis_bottom-integrated yaw-servo cradle (May 2026) --------------
# Replaces the role of the legacy ``make_coxa_bracket()`` part: instead of
# a separate flange+well that bolts to chassis_bottom from above, the yaw
# servo's cradle is now printed AS PART OF chassis_bottom -- a printed
# boss rises ``CRADLE_BOSS_H_MM`` mm above the plate's top face, with a
# tab shelf at ``CRADLE_TAB_SHELF_Z`` where the servo's mounting ears
# land.  The body's bulk hangs DOWN through the plate's existing per-leg
# body cutout (the same ``WELL_W + 2`` x ``WELL_D + 2`` rectangle cut by
# ``_hex_plate`` for the legacy bracket's well drop-in).
#
# Cradle-local frame: origin at the YAW AXIS (= chassis hex edge
# midpoint, where the output spline pokes up); +X = outboard radial;
# +Y = tangential along the chassis edge; +Z = up.  Cradle-z = 0
# coincides with chassis_bottom's TOP face -- pick this anchor instead
# of the plate centre so the boss height value matches the user-facing
# spec ("11 mm above the plate top"; see CRADLE_BOSS_H_MM below for
# the history of the original +19 spec and why it dropped to +11).
#
# Z layout (cradle-local; cradle-z = 0 = chassis_bottom_top):
#     cradle-z = +CRADLE_BOSS_H_MM   = +11  boss top (cradle rim; set
#                                            by the cable-clearance
#                                            floor at chassis-z = +13)
#     cradle-z = +CRADLE_TAB_SHELF_Z =  +6  tab shelf (where the servo
#                                            ears bottom-out)
#     cradle-z =                         0  chassis_bottom_top
#     cradle-z = -CHASSIS_PLATE_T    =  -4  chassis_bottom_bot (cradle
#                                            ends here; body bulk hangs
#                                            below in free air)
#
# Body Z layout (with tabs landing on the shelf):
#     body bottom at cradle-z = +CRADLE_TAB_SHELF_Z - SERVO_TAB_Z = -21
#     body top    at cradle-z = +CRADLE_TAB_SHELF_Z + (SERVO_BODY_H
#                                                       - SERVO_TAB_Z) = +17
# The body's gear housing top at cradle-z = +17 now pokes 6 mm ABOVE
# the cradle rim (+11) into the inter-plate gap.  chassis_top's
# underside sits at cradle-z = CHASSIS_GAP - 2 = +18 (Aug 2026 gap
# shrink, was +30), i.e. only ~1 mm above the housing -- but the
# housings are at the yaw cradles (r ~ 100), 40+ mm OUTSIDE the Ø115
# top disk's footprint (r 57.5), so there is no vertical stack-up to
# clear.  See the CRADLE_BOSS_H_MM docstring below for the Path-A fix
# that took the rim from +19 down to +11.
#
# Bonding: the cradle's outer footprint is wider than the plate's body
# cutout (``WELL_W + 2`` x ``WELL_D + 2`` = 60 x 31 mm) by
# ``CRADLE_BOND_STRIP_MM`` on each side, so the cradle overhangs the
# cutout edges and bonds to plate material via boolean union when the
# meshes are joined in ``make_chassis_bottom``.  The +X bond strip
# overhangs PAST the chassis edge (cradle-x > 0) into empty air -- no
# plate material there to bond with -- so only the -X / +Y / -Y bond
# strips carry actual bonding load.  Adequate given the cradle's
# modest 11 mm height above the plate.
#
# Wire features: the -X (radially INWARD) wall carries the wire-exit
# slot and the printed zip-tie strain-relief post (mirror-flipped
# copies of ``_wire_exit_slot()`` / ``_cable_zip_post()`` -- both
# helpers are hard-coded to well-+X, so the cradle helper applies an
# x-reflection transform before unioning).  Routing the 3-wire servo
# harness OUT on the cradle's INBOARD face sends it toward the
# chassis centre, where it follows chassis_bottom's top face to the
# per-leg cable-drop slot at chassis-frame
# (LEG_HARNESS_DROP_X_CENTRE, 0) = (-46, 0) in cradle-x / cradle-y,
# then drops into the inter-plate gap.  The legacy bracket's wire
# slot is on the +X (OUTBOARD) face -- its docstring claims inboard
# routing but the actual mesh has always exited outboard.  That
# inconsistency stays unresolved on the bracket since the part
# retires entirely in the final cleanup commit; the new integrated
# cradle gets the inboard exit the project always wanted.
#
# Heat-set inserts: the cradle reuses the same MIXED-MODE Design E
# pattern as every other servo cradle in the project: 2 -X bolts per
# cradle get a Phi CRADLE_BOSS_OD = 8 mm boss + Phi INSERT_M3_PILOT_OD
# = 4 mm heat-set insert pocket (McMaster 94459A130), 2 +X bolts get a
# bare Phi INSERT_M3_SELFTAP_PILOT_OD = 2.5 mm self-tap pilot (no boss,
# because the Phi 8 mm boss footprint cannot coexist with the +X wire
# channel that the servo's molded wire boot has to pass through during
# insertion).  See the ``CRADLE_BOSS_*`` / ``INSERT_M3_*`` /
# ``INSERT_M3_SELFTAP_*`` constant blocks above for the full rationale.
CRADLE_BOSS_H_MM     = 11.0   # mm above chassis_bottom_top to the boss
                               # top (= cradle rim).  Chosen (May 2026)
                               # to preserve the then-32 mm CHASSIS_GAP;
                               # still fine under the Aug 2026 20 mm gap
                               # since the cradles sit outside the top
                               # disk's footprint.
                               #
                               # History.  The original planning spec
                               # for this redesign (May 2026) was
                               # BOSS_H = 19 mm so the cradle rim
                               # wrapped the body's gear housing
                               # (gear-housing top at cradle-z = +17)
                               # with 2 mm of clearance above it.
                               # Once the integrated cradle was
                               # unioned into chassis_bottom (the
                               # full plate mesh, all 6 legs), the
                               # ``check_cable_clearance`` verifier
                               # caught the cradle's inboard wings
                               # poking into the Pi4 Ethernet, Pi4
                               # USB-C, Pi4 Micro-HDMI 0 and
                               # Mega2560 USB-B plug envelopes
                               # (~2750 mm^3 of total intrusion).
                               # Those keepouts have their FLOOR at
                               # chassis-z = +13 to +15 mm (= cradle-
                               # z = +11 to +13), so any cradle
                               # material above cradle-z = +11
                               # outside the body-cavity outline
                               # blocks a real cable plug.
                               #
                               # The legacy ``coxa_bracket`` flange
                               # on legs 1-5 has the same physical
                               # conflict (flange top at chassis-z =
                               # +17) but the cable-clearance check
                               # only loaded leg 0's bracket via
                               # ``_build_world_leg0_printed_parts``;
                               # the missing legs let the bracket
                               # design quietly violate the keepouts
                               # without firing the check.  The new
                               # integrated cradle, sitting inside
                               # chassis_bottom which IS in every
                               # check's parts dict, can no longer
                               # hide.
                               #
                               # Path-A fix (May 22 2026, agreed
                               # with the user): drop BOSS_H from
                               # +19 to +11 so the cradle rim lands
                               # AT the keepout floor.  The body's
                               # gear housing now pokes 6 mm above
                               # the cradle rim into the inter-plate
                               # gap instead of sitting under a 2 mm
                               # visual cap; the gear housing isn't
                               # load-bearing -- the 4 servo
                               # mounting tabs clamping down on the
                               # +6 mm tab shelf carry the entire
                               # reaction torque -- so the 5 mm of
                               # wall above the shelf (= +11 - +6)
                               # is purely a dust / visual shroud
                               # strip, not a structural feature.
                               # Tab shelf, bosses, fasteners, bond
                               # strip and the outboard trim
                               # threshold are all UNCHANGED by
                               # this Path-A fix.
                               #
                               # Two alternatives we explicitly
                               # rejected: (B) "step-narrow the
                               # cradle above the shelf to a ~44 x
                               # 24 mm body-cavity outline" was a
                               # substantial geometry rewrite to
                               # preserve BOSS_H = +19; (C) "mark
                               # the 4 affected keepouts as exempt"
                               # kept BOSS_H = +19 but blocked the
                               # user from plugging in an Ethernet
                               # cable without lifting chassis_top
                               # off the standoffs.  Both were
                               # strictly worse than Path A on the
                               # design / code-size axis.
CRADLE_TAB_SHELF_Z   =  6.0   # mm above chassis_bottom_top to the tab
                               # shelf (where the servo's 4 mounting
                               # ears land).  CRADLE_BOSS_H_MM -
                               # CRADLE_TAB_SHELF_Z = 5 mm of wall
                               # extension above the shelf (purely a
                               # dust/visual shroud strip; see the
                               # CRADLE_BOSS_H_MM Path-A docstring
                               # above for why this is no longer
                               # wrapping the body's gear housing).
                               #
                               # May 24 2026 fix: the 12 -X heat-set
                               # insert pockets (and, for consistency,
                               # the 12 +X self-tap pockets) are now
                               # drilled THROUGH the 5 mm shroud
                               # above this shelf so the pocket
                               # cylinder opens at the cradle rim
                               # (cradle-z = +CRADLE_BOSS_H_MM = +11)
                               # instead of being capped at +8 by
                               # ~3 mm of shroud material.  This
                               # lets the operator press the M3
                               # brass heat-set insert (McMaster
                               # 94459A130) DOWN through the cradle
                               # rim with a soldering iron at
                               # ~250 deg C; before the fix the
                               # pocket was sandwiched between 4 mm
                               # of plate below and 3 mm of shroud
                               # above, with no way for the operator
                               # to seat the insert.  User feedback
                               # (May 24 2026): "the bottom chassis
                               # really doesn't seem like it has the
                               # inside holes for the heat set
                               # inserts to hold the servo -- maybe
                               # they are covered so they are
                               # impossible to see?  Either way I
                               # need a hole on the top to put the
                               # heat set insert into".  The fix is
                               # plumbed via the
                               # ``heatset_pocket_z_top_override``
                               # and ``selftap_pocket_z_top_override``
                               # args on
                               # ``_servo_cradle_insert_pockets``;
                               # see that helper's docstring for the
                               # parameter contract and rationale.
                               # The shroud's OUTER perimeter
                               # (everything past the Phi 8 mm boss
                               # footprint) is unchanged -- only the
                               # 24 Phi-2.5 / Phi-4 holes through
                               # the boss interior are added.
CRADLE_BOND_STRIP_MM =  1.0   # mm of bond strip per side: cradle outer
                               # footprint is (WELL_W + 2 + 2*this) x
                               # (WELL_D + 2 + 2*this) so the cradle
                               # overhangs the plate's body cutout
                               # edges by this much on each side, for
                               # a boolean-union bond between cradle
                               # walls and plate material.  Sized to
                               # leave a healthy bonding strip without
                               # widening the cradle so far that it
                               # collides with the per-leg cable
                               # anchor tab (CABLE_ANCHOR_TAB_X = -46,
                               # comfortably inboard of the cradle's
                               # -X face at cradle-x = -41).
CRADLE_OUTBOARD_TRIM_Z = 15.0  # mm cradle-z above which the cradle's
                                # OUTBOARD region (cradle-x > 0 =
                                # outside the chassis hex perimeter)
                                # is trimmed off.  Mirrors the legacy
                                # ``BRACKET_FLANGE_T = 15 mm`` flange-
                                # top height so the femur_link's
                                # hip-pad sweep (which already cleared
                                # the bracket at chassis-z = +17)
                                # continues to clear the integrated
                                # cradle without a workspace-collision
                                # regression.  Under Path A
                                # (CRADLE_BOSS_H_MM = +11, May 22
                                # 2026 docstring) the cradle's rim
                                # is now BELOW this trim threshold,
                                # so the outboard trim subtraction
                                # is a NO-OP at runtime -- the
                                # helper's box-extent clamp handles
                                # TRIM_Z >= BOSS_H cleanly.  Kept as
                                # a constant + active diff so a
                                # future taller-cradle revision
                                # already has the femur-sweep floor
                                # named in the code; lifting BOSS_H
                                # back above +15 would also need to
                                # revisit the cable-clearance
                                # keepout intrusions documented in
                                # the CRADLE_BOSS_H_MM history.
                                # Audit-pass before lifting this:
                                # ``check_workspace_self_collision``
                                # in ``_verify_prototype.py`` is the
                                # regression probe; it fires the
                                # moment the cradle pokes more than
                                # ~CRADLE_OUTBOARD_TRIM_Z mm above
                                # chassis_bottom_top outside the
                                # chassis hex.

# ---- Coxa bracket (yaw-motor housing) -----------------------------------
# A horizontal flange that bolts to the chassis edge plus a servo well
# that hangs from it.  The yaw axis (the output spline of the servo)
# coincides with the chassis hexagon's apothem line, so the whole bracket
# is rotationally symmetric about the chassis perimeter.
BRACKET_FLANGE_T   = 15.0   # mm thick mounting flange.  Was 4 mm; bumped to
                            # 15 mm so the body+tab clearance is a one-sided
                            # POCKET cut into the FLANGE BOTTOM (z in
                            # [0, +12.5]) rather than a through-hole, leaving
                            # a solid 2.5 mm top cap above the entire body
                            # footprint.  Only a small Phi 11 mm gear /
                            # spline clearance hole punches through the top
                            # of the cap, so the chassis-bolt corners and
                            # the perimeter of the bracket are tied together
                            # by a continuous slab of material instead of
                            # two disconnected (+Y, -Y) half-pads with a
                            # rectangular gap between them.  Greatly stiffer
                            # in torsion.  Cascades:
                            #   * COXA_LIFT bumped 14 -> 26 (femur hip pad
                            #     must still clear the taller flange top).
                            #   * Chassis bolt screws need to be M3 x 32
                            #     instead of M3 x 20 (an extra ~ 12 mm of
                            #     flange material to reach through).
BRACKET_FLANGE_X   = 30.0   # mm long (radial -- inboard from chassis edge)
BRACKET_FLANGE_Y   = 52.0   # mm wide (tangential).  Sized so the slot and
                            # the four chassis bolt holes each have >= 4 mm
                            # of material around them.  Was 48 mm; bumped to
                            # 52 mm when BRACKET_BOLT_PCD_Y grew from 36 to
                            # 40 (see below) so the flange still has >= 4 mm
                            # of material outboard of each bolt-circle hole.
BRACKET_BOLT_PCD_X = 16.0   # mm centre-to-centre, inboard vs. outboard bolt pair
BRACKET_BOLT_PCD_Y = 40.0   # mm centre-to-centre, +Y vs. -Y bolt line.  Was
                            # 36 mm; bumped to 40 mm so the bolt clearance
                            # holes stay >= 2.5 mm clear of the chassis-plate
                            # body+tab cutout, whose Y span now grows to
                            # WELL_D + 2 = 31 mm with WELL_D = 29 mm.  At
                            # BRACKET_BOLT_PCD_Y = 36 the bolt holes would
                            # have come within 0.8 mm of the cutout edge --
                            # not enough chassis material around the bolt
                            # head's bearing surface.
BRACKET_BOLT_HOLE  =  3.4   # mm clearance for M3 chassis bolts
BRACKET_FLANGE_INSET = 8.0  # mm distance from the chassis edge (= bracket
                            # origin's outboard face) to the OUTBOARD bolt
                            # line.  >= BRACKET_BOLT_HOLE so the bolt is on
                            # solid chassis material, not in mid-air.

# ---- Bracket yaw-well TRIM plane ----------------------------------------
# May 2026 user-driven simplification:  the bracket's yaw-servo well used
# to extend ``WELL_RIM_Z + WELL_FLOOR_T = 30.25 mm`` below the chassis-
# plate top face -- a 38 mm DS3225 body fully enclosed by ~27 mm of side
# walls plus a 3 mm perimeter floor.  The user pointed out that the
# walls only need to extend deep enough to host the bracket-level
# heat-set insert bosses at bracket-z in [-13, -3] (= 10 mm boss height
# starting 3 mm below the effective shelf top); below that, the wall
# material is just lateral confinement for a servo body that the
# four tabs already hold up structurally.
#
# ``BRACKET_WELL_TRIM_Z`` is the bracket-z plane below which all bracket
# material is removed by the final boolean subtraction in
# ``make_coxa_bracket``.  Set to -15 mm:
#
#   * 2 mm of margin BELOW the bracket-level insert bosses (boss bottom
#     at bracket-z = -13).
#   * 5 mm of margin below the WELL-level inboard-bolt insert bosses
#     emitted by ``_servo_well_solid`` (bracket-z in [-10, 0]).
#   * The DS3225 wire-exit boot, centred at bracket-z ~ -19, hangs
#     EXPOSED below the trim plane.  The +X insertion channel
#     (boot-drop path) above the trim is preserved -- the boot still
#     slides freely from the rim down to its seated position.
#   * The bracket's lowest point moves from bracket-z = -30.25 to
#     bracket-z = -15 (a 15.25 mm reduction in vertical extent).
#
# The trim is applied to the BRACKET ONLY (``make_coxa_bracket``); the
# coxa_link (hip-pitch cradle) and femur_link (knee cradle) keep their
# existing full-depth walls because their wells participate in those
# links' structural geometry differently.
BRACKET_WELL_TRIM_Z = -15.0

# ---- Servo horn adapter (DEPRECATED -- retained for backward compat) -----
# Design B (May 2026): the printed servo_horn_adapter has been RETIRED.
# Each driven link (coxa_link, femur_link, tibia_link) now bolts DIRECTLY
# onto a 20 mm aluminum 25T DISC horn (Amazon B07D56FVK5): 4 x M3
# clearance holes are carved straight into the link's mating pad on the
# DISC_HORN_BOLT_PCD = 14 mm pattern, and a DISC_HORN_COLLAR_OD x
# DISC_HORN_COLLAR_DEPTH bore clears the disc's central spline collar.
#
# Retired-scheme history (kept for context): an intermediate Design B
# bolted the link onto the plastic 4-arm X-horn that ships with the
# servo.  Those X-horn bolts were originally drawn as M3 SHCS
# (Phi 3.2 mm clearance), which was WRONG -- the X-horn that shipped
# with DS3225 / MG996R / DS3218-class servos had Phi ~ 2.0 mm untapped
# through-holes in its arms (M2 self-tap-sized), not Phi 3.2 mm
# clearance, so that pad bored Phi 2.2 mm and used M2 x 8 SHCS as a
# self-tapper into the X-horn plastic.  That whole plastic-X-horn
# scheme is RETIRED in favour of the aluminum disc horn (4 x M3 x 6
# SHCS into the disc's M3 TAPPED holes).  The cradle bolts (4 per servo
# into a Phi 2.5 mm printed shelf pilot) were always orthogonal to this
# and stay M3 x 8 (b447f88).  The adapter geometry below is left in
# place so old STL references / spare-stock screenshots
# keep resolving, but ``make_servo_horn_adapter`` is no longer called
# from any printable output path and no ``servo_horn_adapter.stl`` is
# written by ``main()``.
HORN_ADAPTER_OD     = 32.0   # mm -- plate OD; gives (32 - 20.8) / 2 - 1.1
                              # = 4.5 mm wall outboard of each M2 bolt
                              # hole on the retired adapter's 20.8 mm PCD
HORN_ADAPTER_T      =  4.0   # mm -- thickness (LEGACY).  No longer added
                              # to any joint output Z stack.
# Servo horn stack height.  Historically the hobby-servo plastic horn;
# the robot now drives a 20 mm aluminum 25T disc horn of the same ~5 mm
# thickness that screws onto the 25T spline with an M3 centre screw.
# Adds 5 mm of "stack" along the output-shaft direction between the
# servo's gear-stack top face and the link's pad MATING face.  With the
# printed adapter retired, this IS the entire horn stack (it used to be
# PLASTIC_HORN_H + HORN_ADAPTER_T = 9 mm).
PLASTIC_HORN_H      =  5.0
# Now-retired plastic 4-arm X-shaped horn (DS3225 / MG996R / DS3218 -class
# hardware): the arms extended ~18 mm from the spline centre, so that horn
# swept a Phi 36 mm cylinder as the servo rotated.  The current 20 mm
# aluminum disc horn sweeps only Phi 20 mm, but this larger X-horn sweep
# is retained as a CONSERVATIVE clearance envelope.  It is bigger than
# the retired X-horn's 20.8 mm bolt circle (radius 10.4 mm): each arm ran
# past the bolt position to give the user a thumb-purchase for hand-
# tightening + spare hole positions.  Drives:
#   * ``make_servo_horn`` arm length (the visual mesh's bounding
#     cylinder reflects the real-hardware sweep, so any verifier that
#     reads the mesh extents picks up the correct horn radius).
#   * ``check_horn_sweep_clearance`` in _verify_prototype.py (the
#     cylinder void that the bracket's flange / walls must respect
#     above the seated servo, so the horn can rotate freely without
#     clashing with printed material).
# History: an earlier draft used a 20 mm-long arm (tip at radius
# 10 mm = INSIDE the bolt circle) which silently understated the
# sweep radius by ~9 mm.  The horn-sweep clearance test would have
# missed the recurring "the servo motor doesn't stick out high
# enough" failure if it had read that mesh's bounding cylinder.
#
# User-measured May 2026: 36 mm horn diameter on the actual hardware
# (user has the physical plastic X-horns in hand and measured them
# with calipers).  Was 19.0 mm tip radius / Phi 38 mm sweep; now
# 18.0 mm / Phi 36 mm.
PLASTIC_HORN_X_TIP_R = 18.0
# Total height of the servo output stack ABOVE the spline tip.  In the
# hip-pitch and knee-pitch joints this is the offset between the joint
# AXIS (where the spline pokes out of the servo body) and the link's
# pad MATING face (where the link bolts directly onto the disc horn).
# Driven links must offset their pad along the joint axis by this much,
# then bridge the gap back to the spar with a short "neck" -- without
# this offset the link's pad sits directly on the joint axis and
# overlaps the cradle's "swept volume" (the bridge cap / well wall
# material right above the body).
#
# May 2026: HORN_STACK_H collapses to PLASTIC_HORN_H = 5 mm (was
# PLASTIC_HORN_H + HORN_ADAPTER_T = 9 mm) now that the printed
# horn-adapter disc has been removed.  The link's pad bolts directly
# to the disc horn's top face.
HORN_STACK_H        = PLASTIC_HORN_H



# ---- Yaw-axis output Z (chassis frame, May 2026 chassis_bottom cradle) ---
# World-z (pre-lift) of the yaw disc horn's TOP mating face, where the
# coxa_link's pedestal-bottom mating plane sits.  This is THE most-
# duplicated coordinate in the codebase -- the same formula recurs in
# ``build_prototype_assembly``, ``fastener_registry`` (yaw / hip /
# knee cradle transforms), ``_verify_prototype`` (5 separate places),
# ``inspect_build``, ``render_leg_assembly``, and ``arm/integrate``.
# Centralising it here keeps the May 2026 chassis-bottom-integrated
# yaw-cradle redesign (which lifted yaw_output_z by +8 mm from the
# legacy bracket layout) confined to a single constant edit.
#
# Z stack (chassis-z = 0 = chassis_bottom CENTRE; world-z post-lift
# is chassis-z + chassis_lift):
#
#   chassis_bottom top         z = CHASSIS_PLATE_T/2          = +2
#   cradle tab shelf           z = CHASSIS_PLATE_T/2
#                                    + CRADLE_TAB_SHELF_Z      = +8
#   servo body bottom          z = shelf - WELL_RIM_Z          = -19.25
#                                  (= +8 - +27.25)
#   servo body top             z = body_bottom + SERVO_BODY_H  = +18.75
#   output-gear top            z = body_top + SERVO_OUTPUT_H   = +24.75
#   disc-horn top (yaw_output) z = gear_top + HORN_STACK_H     = +29.75
#
# Legacy formula (bracket-based, pre-May-2026):
#   yaw_output_z_legacy = (SERVO_BODY_H - WELL_RIM_Z)
#                          + SERVO_OUTPUT_H + HORN_STACK_H = +21.75
# New formula simply adds the cradle-shelf offset
# (CHASSIS_PLATE_T/2 + CRADLE_TAB_SHELF_Z = +8) on top of the legacy
# stack: the servo's mounting tabs no longer rest at chassis-z = 0
# (= chassis_bottom CENTRE, the legacy bracket-flange reference
# plane) but on the new cradle shelf at chassis-z = +8 (= +6 mm above
# chassis_bottom TOP).
# STS3215 refit (Jun 2026): the legacy tab-shelf formula above collapses
# under the front-face mount -- with WELL_RIM_Z == SERVO_BODY_H the
# ``(SERVO_BODY_H - WELL_RIM_Z)`` term goes to 0 and the result silently
# drops to 15.0 mm, sinking the whole leg-mount height and folding the
# coxa into the chassis plates.  Since this coordinate sets where every
# leg bolts onto the chassis, it is PINNED to the proven DS3225 value
# (29.75 mm) so the kinematic chain / RL policy / MuJoCo model and the
# horn-to-chassis_top clearance are all preserved.  The front-face yaw
# cradle (``_chassis_yaw_cradle_solid``) is built to deliver the disc-
# horn top mating face exactly at this height, so the constant and the
# cradle geometry stay consistent.
#
# SPACED-PAIR refit (Jun 2026): to fit the second yaw bearing above the
# disc horn we LOWER this output plane by YAW_TOWER_RAISE and LIFT the hip
# the same amount (COXA_HIP_DROP += YAW_TOWER_RAISE), so the WORLD hip-pitch
# axis Z (29.75 + 17.1 = 46.85) is unchanged and the kinematic chain / RL /
# MuJoCo model stay fixed.  Both the cradle geometry and the coxa hub are
# built from this constant so they track the change automatically.
CHASSIS_YAW_OUTPUT_Z = 29.75 - YAW_TOWER_RAISE   # 20.75
# Retired link-to-horn bolt-circle history (superseded by the disc-horn
# block below -- see DISC_HORN_BOLT_* for the live values).  The
# now-retired plastic 4-arm X-horn put its 4 bolt holes on a ~20.8 mm
# PCD (the SECOND hole position out from the spline on each arm of a
# standard DS3225 / MG996R / DS3218 plastic horn) at 0 / 90 / 180 / 270
# deg.  The CURRENT disc horn keeps the same 0 / 90 / 180 / 270 cross
# angles (DISC_HORN_BOLT_ANGLES_RAD) but on a 14 mm bolt circle.
#
# Old May 2026 fastener-spec note (the retired X-horn scheme, kept for
# history):
#     The plastic X-horn that shipped with DS3225 / MG996R / DS3218-class
#     hobby servos had Phi ~ 2.0 mm UNTAPPED through-holes in its arms
#     (intended for M2 self-tap), NOT Phi 3.2 mm clearance for M3, so the
#     old link pad bored Phi 2.2 mm and bolted on with 4 x M2 SHCS used
#     as self-tappers into the X-horn's plastic arm.  That whole scheme
#     is RETIRED: the link now bolts with 4 x M3 x 6 SHCS straight into
#     the aluminium disc's M3 TAPPED holes (DISC_HORN_BOLT_OD = 3.4 mm
#     clearance).  The cradle bolts were always orthogonal to this and
#     stay M3 x 8 (they self-tap into a Phi 2.5 mm printed shelf pilot).
#     See fastener_registry.py and fasteners/README.md for the SKUs.
# ---- Disc-horn bolt circle (June 2026: 20 mm aluminum 25T disc) ----------
# The servo joints now drive a 20 mm aluminum 25T DISC horn (MG995 /
# MG996R type, Amazon B07D56FVK5), NOT the plastic 4-arm X-horn.  The
# disc presents 4 x M3 TAPPED mounting holes on a cross pattern at
# 0 / 90 / 180 / 270 deg.  The vendor lists "14 mm hole spacing"; we
# interpret that as a 14 mm BOLT-CIRCLE DIAMETER (radius 7), which is
# the only reading that physically fits a 20 mm disc -- holes at r=7
# leave 3 mm to the disc edge.
#
# ALTERNATIVE interpretation (reject unless the user's calipers say
# so): "14 mm" = the side of a 14 mm SQUARE, putting the 4 holes at
# (+/-7, +/-7) = radius 9.9 mm -- basically off the 20 mm disc edge.
# To switch to the square reading it is a ONE-LINE change: set
# DISC_HORN_BOLT_PCD = 14.0 * 2**0.5 (= 19.8 mm diagonal) and change
# DISC_HORN_BOLT_ANGLES_RAD to (pi/4, 3pi/4, 5pi/4, 7pi/4).
#
# DISC_HORN_BOLT_PCD is THE single source of truth for the link-to-horn
# bolt circle (referenced in ~30 places).  Changing the one value here
# moves every link pad, every fastener, the disc visual and the spec
# together.
DISC_HORN_BOLT_PCD    = 14.0   # mm -- disc-horn 4 x M3 bolt-circle DIAMETER
DISC_HORN_BOLT_ANGLES_RAD = (0.0, np.pi / 2.0, np.pi, 3.0 * np.pi / 2.0)
DISC_HORN_BOLT_OD     =  3.4   # mm -- M3 clearance (0.2 mm FDM tolerance)
                              # for the M3 SHCS shank passing through the
                              # link pad; the bolt threads into the disc's
                              # M3 TAPPED hole below (the aluminium disc IS
                              # the thread-engagement medium -- no heat-set,
                              # no self-tap into plastic).
HORN_CENTRE_OD      =  3.4   # mm -- M3 centre clearance (for the spline
                              # centre screw that holds the disc on the
                              # 25T spline; reachable from the pad outer
                              # face for serviceability)
# ---- Spline-collar / centre-screw clearance bore -------------------------
# The aluminium disc seats FACE-TO-FACE on the link pad (flat 20 mm disc
# against the flat pad), so the wide Phi 16 mm hub recess used for the
# old plastic X-horn is GONE.  All that has to be cleared at the pad's
# mating face is the disc's raised central spline collar (~Phi 8-9 mm
# knurled hub) and the M3 centre screw head, so the disc still seats
# flat.  A small Phi DISC_HORN_COLLAR_OD x DISC_HORN_COLLAR_DEPTH bore
# centred on the joint axis does that and nothing more.
#
# Sizing: collar bore radius (4.5) sits INSIDE the bolt circle's inner
# rim (DISC_HORN_BOLT_PCD/2 - DISC_HORN_BOLT_OD/2 = 7 - 1.7 = 5.3 mm) so it
# never punches into the 4 M3 clamp holes (0.8 mm radial gap).
DISC_HORN_COLLAR_OD    = 9.0   # mm -- spline-collar / screw-head clearance
DISC_HORN_COLLAR_DEPTH = 2.0   # mm -- depth of the clearance bore at the
                               #       pad mating face

# Legacy plastic-horn hub recess (retained only for
# ``make_servo_horn_adapter`` and the now-orphaned ``make_servo_horn``
# X-horn visual; no robot part uses it any more).
HORN_RECESS_OD      = 16.0   # mm
HORN_RECESS_DEPTH   =  1.2   # mm

# ---- 20 mm aluminum 25T disc horn (the part the robot actually uses) -----
# Amazon B07D56FVK5 ("10Pcs Servo Horn Metal Aluminum 25T Silvery Servo
# Disc ... MG945 MG995 MG996").  Confirmed across several vendor
# datasheets: Phi 20 mm x ~5 mm silver-anodised aluminium disc, central
# 25T female spline bore Phi 5.5 mm, 4 x M3 tapped holes on a 14 mm
# bolt circle, ships with M3 x 6 screws.
DISC_HORN_OD        = 20.0   # mm -- disc outer diameter
# Jun 2026 flush-horn correction: the aluminium 25T disc is RECESSED onto the
# servo's output boss, so its flat mating top sits ~2 mm above the front face --
# essentially FLUSH with the SERVO_OUTPUT_H output bump -- NOT 5 mm proud as the
# old model drew it.  DISC_HORN_H is now the TRUE visible horn height above the
# front face (the link/hub mating plane).  HORN_STACK_H (5 mm) is kept as the
# FROZEN output-plane offset (kinematics): the (HORN_STACK_H - DISC_HORN_H) =
# 3 mm difference is now carried by a printed reach-down boss on each driven
# mount (coxa_yaw_hub / hip + knee yokes) so they physically seat on the real
# thin horn while every joint axis / link length stays exactly where it was.
DISC_HORN_H         =  2.0   # mm -- TRUE disc mating-top height above front face
HORN_REACH_DOWN     = HORN_STACK_H - DISC_HORN_H   # 3 mm printed boss bridge
DISC_HORN_SPLINE_OD =  5.5   # mm -- 25T female spline bore (visualisation;
                              #       teeth not modelled)
DISC_HORN_TAP_OD    =  2.5   # mm -- M3 tap-drill diameter, used to draw the
                              #       4 tapped holes in the disc VISUAL (the
                              #       real holes are tapped M3; the bolt
                              #       threads into them)

# ---- Single-source guard for the disc-horn clearance opening (Jun 2026) ----
# Every printed cradle that the disc horn passes through / seats in opens a
# Phi HORN_CLEAR_OPENING_OD bore (the Jun 2026 "10 mm hole for a 20 mm horn"
# fix on chassis_bottom / hip / knee cradles).  HORN_CLEAR_OPENING_OD is
# defined far above (with the clamshell-retention block) for layout reasons,
# so it can't textually reference DISC_HORN_OD; this assert ties the two
# together as a SINGLE SOURCE OF TRUTH (clearance = horn OD + radial margin)
# so the undersized-clearance bug can never silently reappear.
HORN_CLEAR_OPENING_MARGIN = 4.0   # mm -- diametral clearance over the disc OD
assert abs(HORN_CLEAR_OPENING_OD
           - (DISC_HORN_OD + HORN_CLEAR_OPENING_MARGIN)) < 1e-9, (
    "HORN_CLEAR_OPENING_OD must stay = DISC_HORN_OD + "
    "HORN_CLEAR_OPENING_MARGIN; the disc-horn clearance bore must track the "
    "disc OD (see the Jun 2026 chassis_bottom / hip / knee horn-hole fix)")

# ---------------------------------------------------------------------------
# goBILDA 1906 25T aluminum servo hub
# ---------------------------------------------------------------------------
# An aluminum (anodized) low-profile servo hub from goBILDA.  Same H25T
# spline as the now-retired DS3225 / MG996R / DS3218 plastic X-horn we
# still model with ``make_servo_horn`` -- so it can SHARE the servo, but
# the bolt pattern is COMPLETELY DIFFERENT from DISC_HORN_BOLT_PCD.  The
# 1906 hub presents 4 x M4 threaded holes on a 16 mm SQUARE pattern on
# its TOP face: bolts at (+/-8, +/-8) mm from the spline centre.  Compare
# the retired X-horn's 20.8 mm PCD at 0 / 90 / 180 / 270 degrees -- the
# two are not interchangeable.
#
# Why we're modeling it (user report, May 2026):
#     The OEM plastic 4-arm X-horn that ships with hobby servos
#     ("make_servo_horn") strips its spline after a few load cycles at
#     the hexapod's femur joint.  Replacing it with the goBILDA 1906
#     aluminum hub (4.7 g, $4.99) is the upgrade path.  Using this hub
#     requires REDESIGNING the link's mounting pad (DISC_HORN_BOLT_PCD ->
#     16 mm square M4 grid) which is a separate decision; for now we
#     only PROVIDE the parametric factory + STL so a user can preview
#     the swap visually.
#
# Source: https://www.gobilda.com/1906-series-lightweight-servo-hub-25-tooth-spline-32mm-diameter/
#   * Outer diameter:        32 mm  (SKU last segment "0032")
#   * Spline:                H25T (25-tooth, same as servo X-horn)
#   * Bolt pattern:          4 x M4 on a 16 mm SQUARE (bolts at +/-8, +/-8)
#   * Centering protrusion:  14 mm OD boss on the TOP face (for mating
#                            parts with a 14 mm bore)
#   * Material:              anodized aluminum
#   * Mass:                  4.7 g  -> ~1.74 cm^3 of aluminum
#
# The product page lists no exact thickness number.  Working backwards from
# 4.7 g / (2.70 g/cm^3) = 1.74 cm^3 and a ~32 mm OD x ~4 mm disc minus the
# spline bore and 4 x M4 holes lands in the 4-6 mm range; goBILDA calls
# the hub "low-profile", so we model a 4 mm main disc + a 2 mm tall
# 14 mm centering boss for a 6 mm overall stack.  These are visualization-
# grade estimates.  No spline TEETH are modeled (just a Phi ~6 mm smooth
# female bore standing in for the 25T spline).
GOBILDA_1906_HUB_OD       = 32.0   # mm -- outer diameter, from SKU
GOBILDA_1906_HUB_H        =  6.0   # mm -- overall hub height (main disc + boss).
                                   # Main disc is 4 mm, boss adds another 2 mm.
GOBILDA_1906_DISC_H       =  4.0   # mm -- main 32 mm disc thickness
GOBILDA_1906_BOSS_OD      = 14.0   # mm -- top centering protrusion (mates to
                                   # parts with a 14 mm bore)
GOBILDA_1906_BOSS_H       =  2.0   # mm -- boss height above the main disc
GOBILDA_1906_BOLT_GRID    = 16.0   # mm -- 4 x M4 threaded holes at +/-GRID/2
                                   # in X and Y.  NOT the same pattern as the
                                   # disc horn's DISC_HORN_BOLT_PCD = 14 mm cross
                                   # (nor the retired X-horn's 20.8 mm 4-arm PCD).
GOBILDA_1906_M4_OD        =  4.3   # mm -- M4 threaded-hole clearance for the
                                   # visualization mesh (the real hub is
                                   # tapped M4, not a clearance hole).
GOBILDA_1906_M4_DEPTH     =  3.0   # mm -- blind pocket depth from the top face
GOBILDA_1906_SPLINE_OD    =  6.0   # mm -- 25T spline female bore (major Phi
                                   # ~5.92 mm on H25T).  Modeled as a smooth
                                   # cylinder; the real part has 25 internal
                                   # spline teeth that we don't draw.

# Femur hip-pad mating-face recess.  The coxa_link's pedestal-bottom
# horn_hub_recess at DISC_HORN_COLLAR_DEPTH = 2.0 mm is sized for the
# nominal "spline-collar / screw-head protrusion + FDM tolerance"
# case (see the disc-horn block above).  User-flagged May 2026: on the
# femur_link side the disc horn can sit a few mm LOWER than nominal due
# to spline-cap manufacturing variation, gear backlash, and how
# tightly the central M3 screw is torqued down on the spline.  If
# the disc horn slides ~3 mm down its spline relative to the servo
# output disc, the disc horn's central screw head also sits ~3 mm
# closer to the pad's mating face -- so the femur_link needs a
# deeper recess than the coxa_link to keep the pad seating flush
# against the disc-horn top plane.
#
# Depth 4.0 mm = 1.0 mm nominal protrusion + 3.0 mm extra slack for
# the horn's vertical position uncertainty.  (The bolt-interference
# sizing below is from the now-RETIRED Phi 16 mm plastic-X-horn hub
# recess: at Phi HORN_RECESS_OD = 16 mm that legacy recess sat well
# inside the retired X-horn's 20.8 mm bolt circle -- recess outer rim
# r = 8 mm vs PCD inner rim r = 10.4 - 1.1 = 9.3 mm, 1.3 mm radial
# gap -- so it never punched into the 4 M2 X-horn clamp bolts.  The
# current disc horn instead clears its central collar with the small
# Phi 9 mm DISC_HORN_COLLAR_OD bore, well inside the 14 mm M3 bolt
# circle.)  Material budget at the pad's central annulus (radius r in
# [HORN_CENTRE_OD/2, HORN_RECESS_OD/2] = [1.7, 8] mm): y in [+4, +6]
# = 2 mm of pad cap left between this recess and the +Y outer face --
# thin, but acceptable since the central annulus carries no clamp load
# (the 4 clamp bolts sit outside the recess and clamp through full
# 6 mm pad thickness minus the COUNTERBORE_DEPTH = 2.5 mm head pocket).
FEMUR_HIP_HUB_RECESS_DEPTH = 4.0   # mm -- see comment above

# Radius of the cylindrical CLEARANCE VOID inside the hip/knee link's
# flange ring.  The void is the air gap above the horn during assembly.
# It is still sized to the now-RETIRED plastic X-horn's swept envelope
# (arm tips at PLASTIC_HORN_X_TIP_R = 18 mm) as a CONSERVATIVE keep-out;
# the current 20 mm aluminum disc horn (radius 10 mm) drops in with room
# to spare.
#
# May 2026 (post-shorten-neck refactor): the void switched from
# HORN_ADAPTER_OD/2 + 0.5 = 16.5 mm (sized for the now-retired
# printed servo_horn_adapter disc) to PLASTIC_HORN_X_TIP_R + 0.5 =
# 18.5 mm (sized for the retired plastic X-horn's Phi 36 mm sweep).
# The user found that the previous Phi 33 mm cup physically blocked
# the Phi 36 mm plastic X-horn from fitting -- the horn arm tips
# slammed into the cup's inner wall.  Bumping to Phi 37 mm ID gave
# the horn 0.5 mm radial clearance per side at the arm tips; the
# smaller disc horn keeps that envelope as headroom.
#
# Consumers: ``make_femur_link`` + ``make_tibia_link`` (flange-ring
# inner radius), ``keepout_volumes.py`` (femur/tibia horn-stack
# void registry entries), and ``_verify_prototype.py``
# (``check_horn_stack_clearance`` probe radius).
HORN_STACK_VOID_R   = PLASTIC_HORN_X_TIP_R + 0.5

# ---- M3 into the aluminium disc horn (June 2026 disc-horn switch) --------
# The link-to-horn bolt is now an M3 SHCS that threads into the disc's
# M3 TAPPED hole.  The aluminium disc IS the thread-engagement medium
# (no heat-set insert, no self-tap into plastic).  The link's pad just
# needs a clean Phi 3.4 mm clearance through-hole for the M3 shank.
# Bottom-cap geometry on driven link pads.  Each printed link pad has
# to:
#   (a) be solid in the central region where the 4 M3 disc clamp
#       bolts seat (PCD = DISC_HORN_BOLT_PCD = 14 mm, radius 7, well
#       within every pad footprint),
#   (b) recess each M3 SHCS head into a counter-bore so the head
#       doesn't protrude into the assembly trough above and block
#       the hip-pitch / knee servo body's insertion path.
# These two requirements drive the cap thickness PEDESTAL_CAP_T and
# the counter-bore depth COUNTERBORE_DEPTH below.  See the
# make_coxa_link / make_femur_link / make_tibia_link docstrings for
# the full geometry.
PEDESTAL_CAP_T          = 4.0   # mm -- thickness of the solid bottom
                                #       cap that carries the 4 M3
                                #       disc clamp bolts.  4 mm gives
                                #       a 1.0 mm shaft-clearance run
                                #       + 3.0 mm counter-bore.
COUNTERBORE_DEPTH       = 3.0   # mm -- counter-bore depth (M3 SHCS
                                #       head height = 3 mm).  Bolt head
                                #       TOP sits flush with the cap /
                                #       pad outer face.
M2_HEAD_OD_CLEARANCE    = 5.7   # mm -- Phi 5.7 mm clearance pocket for
                                #       the M3 SHCS head (head OD =
                                #       5.5 mm + 0.2 mm FDM tolerance).
                                #       (Legacy name kept so the ~6
                                #       call-sites keep resolving;
                                #       DISC_HORN_BOLT_HEAD_OD is the
                                #       self-documenting alias.)
DISC_HORN_BOLT_HEAD_OD  = M2_HEAD_OD_CLEARANCE   # explicit alias
DISC_HORN_BOLT_THREAD_ENGAGEMENT_MM = DISC_HORN_H  # mm -- depth of M3
                                       #       thread engagement INTO the
                                       #       aluminium disc.  Jun 2026
                                       #       flush-horn fix: the real disc
                                       #       is only DISC_HORN_H = 2 mm, so
                                       #       the bolt threads the FULL disc
                                       #       thickness (2 mm) -- the driven
                                       #       mount's printed reach-down boss
                                       #       carries the remaining
                                       #       HORN_REACH_DOWN = 3 mm of grip.
                                       #       Drives the M3 x 8 disc-horn SHCS
                                       #       length pick in
                                       #       fastener_registry.py.

# ---- Link cross-sections -------------------------------------------------
# All printed in PLA / PETG.  The femur and tibia are flat plates that
# lie in the leg's plane of motion (X-Z plane in leg-local coords) so
# they're stiff against vertical bending.  In LINK-local coords:
#     +X = spar long axis
#     +Y = joint-axis direction (the link's THICKNESS direction;
#          this is also the servo output-shaft direction at each joint)
#     +Z = perpendicular to spar in the leg's motion plane (the spar's
#          structural HEIGHT)
# The link's hip end is a square pad centred on the joint axis (x=0,
# z=0) that contains the 4 horn bolts.  The link rotates rigidly with
# the horn, so the bolt-circle centre MUST coincide with the joint
# axis or the bolts can't physically stay attached.
LINK_THICKNESS   =  6.0   # mm -- Y-direction thickness of every link
FEMUR_SPAR_H     = 34.0   # mm -- Z-direction height of the femur spar.
                          # Must be > SERVO_BODY_D + 8 so the spar's top
                          # and bottom flanges remain after we cut a slot
                          # through it for the knee servo's body to slide
                          # through during assembly.
                          # Was 30 mm -> 4 mm of flange material on each
                          # side of the 22 mm-tall insertion slot, i.e.
                          # 4 mm-tall bridges joining spar to knee well.
                          # Bumped to 34 mm -> 6 mm flanges (50 % taller,
                          # ~ 3.4x stiffer in out-of-plane bending since
                          # I scales with t^3 for a flange in bending).
                          # The hip pad is 2*HIP_PAD_R = 34 mm in Z, so
                          # the spar now exactly fills the hip-pad Z
                          # extent -- no step at the hip end.
TIBIA_SPAR_H     = 18.0   # mm -- Z-direction height of the tibia spar
# Pad radius.  Must satisfy:
#
#   HIP_PAD_R >= HORN_STACK_VOID_R + MIN_PRINTABLE_NECK_WALL_T
#
# so the hollow-annulus neck (see make_femur_link / make_tibia_link, "flange
# ring" block) has a printable wall around the horn-stack clearance void.
# With HORN_STACK_VOID_R = PLASTIC_HORN_X_TIP_R + 0.5 = 18.5 mm and a
# 1.5 mm wall the minimum is 18.5 + 1.5 = 20.0 mm.  A SHRUNK pad
# (HIP_PAD_R = 17, the very old value) leaves only 0.5 mm of wall,
# which cannot be FDM-printed reliably at 0.4 mm nozzle.
#
# May 2026 (post-flange-ring-fuse refactor): HIP_PAD_R bumped from
# 19.5 to 20.0 mm.  The previous 19.5 mm value left a 1.0 mm-thick
# neck wall (HIP_PAD_R - HORN_STACK_VOID_R = 1.0 mm), which prints
# as a single perimeter at 0.4 mm nozzle and is fragile.  More
# importantly, the May 2026 "shorten-neck" refactor (7e2ca87)
# collapsed the neck to a 2 mm flange ring at y in [+3, +5] with
# only a 0.5 mm "boolean kiss" volumetric fuse with the spar at
# y in [+2.5, +3].  The user complained twice that the tibia
# spar <-> knee pad joint looked like it would break in service
# (TIBIA_SPAR_H = 18 mm has only ~half the femur's pad-side fuse
# cross-section to work with), and a Y-sweep probe of the cross-
# section confirmed the fuse area dropped from a tall column to
# a 0.5 mm sliver.  The fix is Option A from the May 2026
# pad <-> spar fuse audit: bump HIP_PAD_R to 20.0 (wall thickness
# 1.5 mm, 3 perimeters @ 0.4 mm nozzle) AND deepen the neck back
# to ~8 mm (y in [-LINK_THICKNESS/2, HORN_STACK_H] = [-3, +5]) so
# the full volumetric fusion with the spar at y in [-3, +3] is
# restored.  Cup ID stays at PLASTIC_HORN_X_TIP_R + 0.5 = 18.5 mm,
# the conservative envelope of the now-retired Phi 36 mm plastic
# X-horn; the current 20 mm disc horn clears it easily.
#
# Bumping HIP_PAD_R increases the pad's swept disk diameter (39 mm
# -> 40 mm) which means the pad reaches 0.5 mm further toward the
# coxa_link arm's bottom face and the bracket flange.  COXA_LIFT
# (36 mm) and WELL_Z_DROP_EXTRA (4 mm) were sized with ~2 mm of
# headroom above the swept disk's +Z edge at HIP_PAD_R = 19.5, so
# the 0.5 mm bump shrinks the headroom to 1.5 mm -- still well
# above the WORKSPACE_VOXEL_PITCH = 2.5 mm artefact band.  See
# check_workspace_self_collision for the runtime verification.
HIP_PAD_R        = 20.0

# ---- Femur hip-pad radius (May 2026 user-flagged shrink) ----------------
# The femur's hip pad is a SEPARATE Phi-2*FEMUR_HIP_PAD_R disc that
# clamps onto the hip-pitch disc horn at the link's NEW local origin.
# Decoupled from HIP_PAD_R (= the tibia knee pad radius) so the femur
# can be shrunk independently of the tibia.
#
# The FEMUR_HIP_PAD_R = 14 mm value dates from the now-retired plastic
# X-horn era.  User-flagged May 2026: "the 20 mm from the femur link
# hip pitch to knee is too long, it blocks the X horn, you need to
# shorten it".  The previous FEMUR_HIP_PAD_R = HIP_PAD_R = 20 mm covered
# the (then-current) X-horn arm tips (PLASTIC_HORN_X_TIP_R = 18 mm) in
# the assembled BuildViz view from +Y -- the Phi 40 mm pad disc fully
# hid the Phi 36 mm X-horn (and all 4 arms) behind it, making the
# X-horn invisible during inspection.  It was reduced to 14 mm so each
# X-horn arm tip poked out PLASTIC_HORN_X_TIP_R - FEMUR_HIP_PAD_R =
# 18 - 14 = 4 mm past the pad's outer edge, restoring visibility while
# still covering the 4 retired-X-horn PCD bolts at radius 10.4 mm with
# safe margin (wall thickness = 14 - 10.4 - M2_HEAD_OD_CLEARANCE/2 =
# 14 - 10.4 - 2.0 = 1.6 mm of pad material outboard of each head
# counter-bore; well above the 1.5 mm = 3 perimeter @ 0.4 mm nozzle
# minimum).  The current 14 mm disc-horn M3 bolt circle (radius 7) sits
# even further inside this pad, so the coverage margin only improves.
#
# Why the constraint HIP_PAD_R >= HORN_STACK_VOID_R + 1.5 mm wall
# (= 20 mm, see the HIP_PAD_R docstring above) does NOT apply here:
# that constraint was for the pre-May-2026 HOLLOW ANNULUS NECK
# design where the pad enclosed the X-horn inside a printed cup
# (the cup wall had to be >= 1.5 mm thick at FDM tolerance).  The
# post-2026 COLLINEAR-PAD refactor moved the pad from y in
# [-HORN_STACK_H, +LINK_THICKNESS+5] to a flat solid disc at NEW y
# in [0, +LINK_THICKNESS], sitting ABOVE the X-horn (X-horn at y in
# [-HORN_STACK_H, 0]).  The pad no longer encloses the horn, so the
# wall-thickness constraint vanishes.  See make_femur_link's
# "NEW (May 2026 collinear-pad refactor)" comment block.
#
# The pad's swept clearance cut in make_coxa_link (pad_sweep_clear =
# HIP_PAD_R + 0.5) is left at HIP_PAD_R = 20 mm + 0.5 mm = 20.5 mm
# even though the femur's actual sweep is now only 14 mm radius;
# the cut just removes more coxa_link pedestal material than strictly
# needed (no structural concern -- only fewer mm^3 of plastic) and
# stays consistent with the historical sizing.  The tibia's knee pad
# clearance cut (knee_clear_R = HIP_PAD_R + 2.5) is sized for the
# TIBIA's pad (still HIP_PAD_R = 20 mm) so it stays unchanged.
FEMUR_HIP_PAD_R  = 14.0

# ---- Coxa-link bridge stiffener -----------------------------------------
# The coxa link's "bridge" -- the flat 4 mm-thick arm + 6.5 mm-tall
# bridge member that connects the horn-mating yoke (the inboard 4-bolt
# hub pad that clamps onto the yaw servo's horn adapter) to the
# hip-pitch servo cradle at the outboard end -- carries up to ~2.5 N*m
# of hip-pitch reaction torque about the link's local +Y axis.  In that
# bending direction the arm is the THIN dimension: only 4 mm of Z height
# at the +X end of the bridge (just inboard of the cradle's outer +X
# wall), giving I_y = 22*4^3/12 = 117 mm^4, sigma_max = 2.5 N*m * 2 mm /
# 117 mm^4 = 43 MPa -- a 1.2x safety factor against PA12's 50 MPa yield
# and visibly flexible under hand fit-check load.  For comparison the
# femur's tall 30 mm spar runs sigma_max ~ 3 MPa in the same bending
# direction (SF ~ 18x).
#
# Adding a thin gusset hanging BELOW the arm in the bridge region
# extends the cross-section's Z range to ~8 mm, raising I_y to
# ~570 mm^4 and dropping sigma_max to ~20 MPa (SF ~ 2.5x).  Geometric
# clearance constraints:
#   - Inboard X start >= HIP_PAD_R + 1 mm so the gusset doesn't clip
#     the horn adapter's 32 mm OD circular footprint underneath the hub.
#   - Z depth <= servo-body-top clearance so the gusset's bottom face
#     stays above the hip-pitch servo body's top face during insertion
#     (with arm_t = 6 mm and the well dropped by -(WELL_D/2 + arm_t/2)
#     = -15.5 mm, the seated body top sits at link-z = -5.5 mm; a 5 mm
#     gusset puts the bottom face at -5.0, leaving 0.5 mm of clearance).
#   - Outboard X end stays inside the arm's +X tail (link-x <= 41) so
#     the gusset doesn't overhang past the cradle's +X wall.
#   - Far from the wire-exit slot (which lives at link-x in [-15, -4]).
COXA_BRIDGE_GUSSET_H = 5.0   # mm -- gusset depth in -Z below arm bottom.
                             # Was 4 mm (matched the old arm_t = 4 mm).
                             # With arm_t = 6 mm the seated servo body
                             # top drops to link-z = -5.5, so we can go
                             # 1 mm deeper without violating insertion
                             # clearance.
COXA_BRIDGE_GUSSET_L = 23.0  # mm -- gusset length along the spar +X

# Coxa-link arm thickness in Z.  Originally 4 mm, raised to 6 mm to
# bring the arm's I = arm_w * arm_t^3 / 12 up by 3.4x for cantilever
# bending stiffness.  Bumping to 8 mm would clear the
# ``check_thin_sheets`` verifier directly but drops the servo-well
# by 1 mm (-> coxa_link clips the electronics tray in the
# workspace sweep) and squeezes the femur-pad-vs-bracket-flange
# margin from 1.25 mm to 0.25 mm.  Instead we keep arm_t = 6 and
# stiffen the +Y side of the arm with a top cap rib in
# ``make_coxa_link`` (see COXA_ARM_CAP_T below).
COXA_ARM_T              = 6.0    # mm -- arm slab thickness in +Z.

# Top cap rib stacked ABOVE the arm slab on the +Y side of the arm
# only.  Covers link y in [pad_y_min, arm_w/2] and link x in
# [hub edge, arm +X end].  Stays clear of the femur spar (which
# sits at link y in [-3, +3] at any pitch) and stays above the
# femur hip pad's max Z reach (pad swept disk reaches link z =
# hip_drop + HIP_PAD_R = -3.5 + 17 = +13.5 mm lifted, well below
# the cap bottom at z = arm_t + COXA_LIFT = 20 mm lifted).
#
# What this fixes: the ``check_thin_sheets`` cluster identifies the
# arm slab's +Y portion (cluster centroid at y = +1.2 mm, bbox y
# from -8.4 to +10.8 mm) as a 6 mm-thick (chord_z = 7.2 mm
# voxelized) structural neck between the yaw hub and the hip-pitch
# servo well.  Adding a cap on the +Y side raises chord_z to arm_t
# + COXA_ARM_CAP_T = 10 mm in the cap region.
#
# What this leaves unfixed: the y in [-3, +3] strip of the arm
# (where the femur spar swings through at extreme pitches) is NOT
# capped; a top cap there would collide with the spar at fem = -80
# deg (the firmware's negative-pitch joint limit).  The cluster's
# +Y half is fixed by the cap and the -Y half is already covered
# by the existing bridge / gusset structure (chord_z >= 10 mm in
# the bridge region).  The remaining "uncapped strip" voxels
# represent ~5/16 of the original cluster -- below the
# ``MAX_SHEET_BUDGET_VOX`` = 250 voxel budget.
COXA_ARM_CAP_T          = 4.0    # mm -- cap thickness in +Z above
                                  # arm.  Cap top at z = arm_t +
                                  # COXA_ARM_CAP_T = 10 mm unlifted,
                                  # lifted to z = COXA_LIFT + 10 =
                                  # 24 mm.

# ----- Well-top-wall thickening pad (coxa_link only) ---------------------
# Why this exists.  In the BASELINE ``make_coxa_link`` geometry the
# bridge slab that ties the horn yoke (top hub) to the hip-pitch servo
# well box overlaps the well's outer body by only 0.5 mm in Y (the
# 0.5 mm of bridge that punches past the well's outer +Y face) and
# 1.5 mm in Z (the 1.5 mm of bridge below the well's outer top face),
# giving a 53 x 0.5 x 1.5 = 40 mm^3 bonded interface holding the
# entire leg load.  ``pad_sweep_clear`` -- the cylindrical void that
# lets the femur hip pad swing through the link's interior -- then
# eats a circular hole through the bridge across most of its X span,
# leaving 19 mm^2 of bridge cross-section at the worst Z slice (just
# above the well top face, where the cylinder cut is widest).  The
# user has complained about this exact failure mode multiple times
# ("the top of coxa_link is not attached strongly to the part housing
# the servo, theres an obvious fix of thickening the wall on the
# motor housing so a bigger surface area attaches to the top piece").
#
# This pad is the user's preferred fix.  It's a rectangular block of
# plastic that sits ON TOP of the well's outer top face and rises up
# into the bridge gap, fusing both with the well's outer body (below)
# and the existing bridge / arm (above).
#
# Sizing notes (May 18 2026 revision).  An earlier 5 mm version of the
# pad left a 4.25 mm-wide safe spine outside the ``pad_sweep_clear``
# cylinder's y range of [-18, +18] -- BUT the bridge + pad at x ~
# COXA_LENGTH = +25 mm (where the cylinder cuts the deepest into the
# bridge gap) collapsed to a ~4 mm-thin vertical strip that the user
# flagged as visually + structurally "wrong wall thickened".  See the
# user-pointed screenshot at hexapod_walker/prototype/renders/
# _user_pointed_thin_neck.png and the ASCII diagnostic in
# _find_thin_neck.py (YZ slice at x = +22, x = +25): the surviving
# spine in the middle X range was only 4 mm wide in Y for a 6 mm-tall
# vertical column, which neither the original XY-slice bridge check
# nor casual inspection picked up.
#
# The fix is to extend ``WELL_TOP_PAD_Y_EXT`` to 13.25 mm so the pad's
# far-Y edge lands at y = -32 (= bridge_y_min - 13.25, = well_near_y -
# 13.75).  The pad's far-Y edge stays comfortably INSIDE the well's
# outer footprint (well's far-Y face is at y = -46.5) and OUTSIDE the
# cavity (the cavity sits at y in [-44, -6], BUT at z in [+2.5,
# +22.5] -- well below the pad's z range of [+23.5, +32], so the pad
# does not intrude into the cavity).  The new pad supplies ~14 mm of
# surviving safe spine in Y at every x in the bridge x range, holding
# the well to the bridge / arm / hub through ~14 x 8.5 = 119 mm^2 of
# cross-section at the cylinder-cut middle (vs. ~24 mm^2 with the
# 5 mm pad).
#
# Verified by ``_check_coxa_link_bridge_joint`` AND
# ``_check_coxa_link_bridge_yz_thickness`` in _verify_prototype.py
# (called as part of ``check_flimsy_joints``).  The YZ-thickness
# check FAILS on the 5 mm pad geometry (min YZ-area = 40 mm^2 at x =
# +22) and PASSES on the 13.25 mm pad.
WELL_TOP_PAD_Y_EXT = 13.25 # mm -- distance the pad extends PAST the
                            # bridge's existing -Y face into -Y
                            # direction (= INTO the well's outer body
                            # footprint).  See big docstring above.

# ---- Derived hip-pitch geometry ----------------------------------------
# Hip-pitch axis Z position in coxa-link local frame, AFTER lift.  Equal
# to (well_z_drop + COXA_LIFT) inside make_coxa_link, where
# well_z_drop = -(WELL_D/2 + COXA_ARM_T/2 + WELL_Z_DROP_EXTRA).  Exposed
# as a module-level constant so downstream callers (workspace tests,
# assemblies, renders, integrators) don't have to duplicate the formula
# (which got out of sync when WELL_Z_DROP_EXTRA was introduced).
# NB: + YAW_TOWER_RAISE lifts the hip-pitch axis in the coxa-local frame so
# the spaced yaw-bearing tower fits below it; CHASSIS_YAW_OUTPUT_Z is lowered
# by the same amount so the WORLD hip-axis Z is unchanged (kinematics fixed).
# COXA_COAXIAL_FOOT_RAISE lifts the hip-pitch axis so the coaxially-stacked
# hip cradle's solid back seats on the hub.  The seated servo-body bottom
# (= hip-servo well floor) is at COXA_HIP_DROP - SERVO_BODY_D/2; we want
# that = yaw-hub platform TOP (YAW_HUB_PLATFORM_Z1, defined late)
# + COXA_WELL_FLOOR_LIFT.
#
# History: the two-part era stacked an 8 mm foot plate (COXA_JOIN_FOOT_T)
# ON TOP of the platform, putting the well floor at platform_top + 8 (+31)
# -- the sole Part A<->B bolted join needed its own thick slab.  Aug 17
# 2026 SINK PASS (user: the M3x30 heads were hard to reach down the 12 mm
# shafts; "push the bracket holding the servo down to overlap with the cap
# more ... making the height lower would help a lot getting the screws in
# and remove unnecessary material"): the link has been one piece since the
# Aug merge, so the separate raised slab serves nothing -- the foot now
# SINKS INTO the platform band (see make_coxa_hip_bracket), spanning
# z +18..+26.  The floor cannot drop all the way onto the platform top:
# the slab bottom is pinned at +18 (YAW_HUB_CAP_AXIAL_CL = 1.5 mm running
# gap over the stationary cap rim at +16.5 -- widened from 0.5 by the Aug
# 17 2026 bench scrape fix), and a flush floor left a 6 mm sheet carrying
# the whole cradle (flagged by the thin-sheet check, 43 x 40 mm cluster),
# so the floor sits COXA_WELL_FLOOR_LIFT = 2 mm above the platform top
# for the proven 8 mm slab.  Net: the cradle rides 5 mm lower, the screw
# heads sit ~5.3 mm below the shaft mouths instead of ~9, and the world
# hip axis drops 5 mm (58.65 -> 53.65 -- the verifier / sim re-derive the
# pose consistently).  The INNER bearing interfaces (boss, uflange, dust
# skirt) are untouched.
COXA_WELL_FLOOR_LIFT = 2.0   # mm -- well floor above the platform top
#
# Jun 2026 flush-horn refit: the platform top is no longer the literal 13.5;
# it tracks the bearing stack via YAW_TOWER_TOP_Z (= UPPER_race_top + lip).
# YAW_HUB_PLATFORM_Z1 = YAW_TOWER_TOP_Z + YAW_HUB_CAP_AXIAL_CL (boss over
# the cap rim -- 1.5 since the Aug 17 2026 scrape fix) + YAW_HUB_PAD_T (6)
# -- expressed here from the EARLY-defined constants because
# YAW_HUB_PLATFORM_Z1 itself is defined further down.
_YAW_HUB_PLATFORM_TOP_EARLY = (YAW_TOWER_TOP_Z + YAW_HUB_CAP_AXIAL_CL
                               + 6.0)   # == YAW_HUB_PLATFORM_Z1
COXA_COAXIAL_FOOT_RAISE = (_YAW_HUB_PLATFORM_TOP_EARLY + COXA_WELL_FLOOR_LIFT
                           + SERVO_BODY_D / 2.0) - (
    -(WELL_D / 2.0 + COXA_ARM_T / 2.0 + WELL_Z_DROP_EXTRA)
    + COXA_LIFT + YAW_TOWER_RAISE)
COXA_HIP_DROP = (-(WELL_D / 2.0 + COXA_ARM_T / 2.0 + WELL_Z_DROP_EXTRA)
                 + COXA_LIFT + YAW_TOWER_RAISE + COXA_COAXIAL_FOOT_RAISE)

# ---- Hip-bracket CENTERING on the yaw axis (Jun 2026 re-centre) ---------
# The user requirement: coxa_hip_bracket (Part B) must sit CENTERED RIGHT
# OVER THE YAW SERVO -- its body/footprint centred on the yaw axis (coxa
# x=0, y=0), a clean coaxial vertical stack, NOT cantilevered off to one
# side.
#
# The servo BODY is already centred in coxa-X (the STS3215 long axis spans
# coxa-X symmetric about 0 when x_dir=(1,0,0)).  The off-axis sprawl is
# entirely in coxa-Y: with the disc-horn-top (hip axis) anchored at y=0 the
# fixed side spans coxa-Y in
#     [JOINT_HORN_TOP_Z - WELL_H, JOINT_HORN_TOP_Z + PASSIVE_BACK_PLATE_T]
#   = [3.0, 48.3] mm  (centre +25.65),
# i.e. the whole servo tower hangs to +Y.  Anchoring the hip joint at
# COXA_HIP_ANCHOR_Y instead slides that footprint so its centre lands on
# the yaw axis (coxa y=0).
#
# KINEMATIC CONSEQUENCE (documented, intentional): the STS3215 output face
# is on ONE side of the body, so a body that is centred on the yaw axis
# necessarily puts the hip-PITCH axis at the body's -Y edge -- about
# |COXA_HIP_ANCHOR_Y| mm tangential (and COXA_LENGTH mm radial) from the
# yaw axis.  This is the unavoidable minimum offset (you cannot have BOTH
# the body centred AND the output exactly on the axis).  Every leg-
# kinematics consumer anchors the hip joint at (COXA_LENGTH,
# COXA_HIP_ANCHOR_Y, COXA_HIP_DROP) so the femur/tibia chain, the verifier
# pose grid, fasteners, mujoco and the assembly all stay consistent and
# the foot just lands at the (re-derived) tangentially-shifted stance.
#
# (The actual value depends on JOINT_HORN_TOP_Z / PASSIVE_BACK_PLATE_T,
# which are defined further down with the sandwich-joint geometry, so the
# assignment of COXA_HIP_ANCHOR_Y / COXA_HIP_ANCHOR lives just below them.)

# ---- Foot ----------------------------------------------------------------
# Aug 2026: the hinged foot is RETIRED.  The tibia now ends in a single
# TPU 95A ``foot_boot`` pressed (optionally epoxy-dabbed) straight over
# the Ø8 CF tube end -- no tibia_foot_fitting, no foot_pad, no M3x16
# pan-head hinge pin, no nyloc.  Rationale (user, Aug 2026 design
# review): the hinged pad's angled tang dug into the textured garage
# floor and caught; the loose hinge added 2 printed parts + 2 fasteners
# per leg and nothing measurable in traction.
#
# Current as-built boot (user bench note, 2026-08-25): standard
# spherical-ended TPU boot, about 8.5-9 mm diameter; CAD uses 9.0 mm
# nominal.  The rounded end is the ground contact and matches MuJoCo's
# spherical foot contact model.  The dome centre lands at tibia-local
# x = TIBIA_LENGTH - FOOT_BOOT_OD/2 = mujoco's BOOT_TIP_CTR.
#
# Boot local frame (same convention the old fitting used): origin at the
# tube end on the tube axis, tube enters from -X, ground tip toward +X.
# The dome APEX lands exactly at tibia-local x = TIBIA_LENGTH, so the
# knee-axis -> tip kinematic length is controlled by the measured
# constant above.
FOOT_BOOT_OD           =  9.0  # mm -- nominal spherical-end/outer diameter
                               #        (bench-estimated 8.5-9 mm)
FOOT_BOOT_BORE_D       =  8.1  # mm -- bore over the Ø8 tube: SAME Ø8.1 as
                               #        the tibia yoke's tube socket
                               #        (LEG_TUBE_OD + 2*LEG_TUBE_SOCKET_CLEAR
                               #        -- the fit the bench already proved
                               #        slides on).  Aug 17 2026, two rounds:
                               #        7.7 (0.3 interference) was unpressable
                               #        by hand, 7.9 still too tight -- TPU
                               #        bores print undersized from extrusion
                               #        bulge, so a nominal slip fit still
                               #        ends up snug.  Dab of CA/epoxy for
                               #        keeps.
FOOT_BOOT_SOCKET_DEPTH = 20.0  # mm -- tube insertion depth
FOOT_BOOT_TIP_L        =  8.0  # mm -- solid tip beyond the tube end;
                               #        tube end at x = TIBIA_LENGTH - 8
                               #        puts the dome apex at TIBIA_LENGTH.
                               #        (>= 3-5 mm of material stays below
                               #        the rod end: the 45-deg internal
                               #        blind cone bottoms ~3.95 mm short
                               #        of the apex.)
FOOT_BOOT_TIP_R        = FOOT_BOOT_OD / 2.0
FOOT_BOOT_MOUTH_LEAD   =  0.4  # mm -- bore lead-in chamfer at the mouth;
                               #        limited by the thin wall of the
                               #        9 mm standard boot.
# (FOOT_BOOT_TIP_CHAMFER RETIRED Aug 19 2026 with the flat tip face --
# the dome has no rim to chamfer.)
#
# RETIRED hinged-foot constants below (FOOT_PAD_* / FOOT_HINGE_* /
# FOOT_TANG_*): kept only because the legacy pre-sandwich cantilever
# tibia builder (shadowed dead code) still compiles against them.  No
# emitted part, fastener, or check references them any more.
FOOT_PAD_OD          = 28.0   # mm -- outer diameter of the ground-contact disk
FOOT_PAD_BASE_H      =  4.0   # mm -- thickness of the disk (TPU spring)
FOOT_PAD_BOSS_OD     = 14.0   # mm -- short stiffening boss between disk top
                              #        and fork (gives the fork a wider
                              #        root than its 10 x 13.4 mm
                              #        cross-section; circular 14 mm OD
                              #        easily contains the rectangular
                              #        fork footprint)
FOOT_PAD_BOSS_H      =  3.0   # mm -- boss height; fork cheeks start at
                              #        base_h + FOOT_PAD_BOSS_H

# Hinge geometry.
#
# May 2026 inversion: the foot/tibia hinge used to be "TIBIA = fork,
# FOOT = tongue" -- the tibia ended in a 2-cheek + 4 mm tongue-slot
# clevis (12 mm wide in tibia Y) and the foot_pad had a 4 mm-thick
# vertical tongue that slid up into the slot.  That made the tibia
# 12 mm wide in Y at the foot end but only LINK_THICKNESS = 6 mm
# wide everywhere else, so the clevis protruded 6 mm above the
# spar's broad face in the "spar broad face on bed" print orientation
# and forced supports under the clevis.
#
# After the inversion the geometry is "TIBIA = single tang, FOOT =
# fork": the tibia spar terminates in a single LINK_THICKNESS-wide
# tang centred on tibia y=0 (so the tibia is now 6 mm wide in Y
# EVERYWHERE and prints fully flat), and the foot_pad gains a 2-cheek
# fork with a LINK_THICKNESS + clearance slot that captures the tang.
# Same M3 x 16 pan-head pin + M3 nylock, same hinge axis -- just
# inverted geometry; no BOM change for fasteners.  The foot_pad's
# Y extent across the fork is 2 * FOOT_HINGE_CHEEK_T + SLOT_W =
# 2 * 3.5 + 6.4 = 13.4 mm, which fits comfortably within the disk's
# 28 mm OD (the fork lives above the disk in foot-local +Z anyway).
FOOT_HINGE_CHEEK_T   =  3.5   # mm -- each foot-pad fork cheek thickness in
                              #        the knee-axis direction (Y).  Above
                              #        MIN_PRINT_T = 3.0 mm with margin.
                              #        Unchanged across the inversion --
                              #        the cheeks just live on the FOOT
                              #        now instead of the tibia.
FOOT_HINGE_SLOT_W    = LINK_THICKNESS + 0.4   # = 6.4 mm -- fork slot
                              #        clearance for the tibia's tang.
                              #        Tang nominal width = LINK_THICKNESS
                              #        = 6 mm, so 0.2 mm of clearance per
                              #        side.  Renamed from the pre-2026
                              #        ``FOOT_HINGE_GAP`` (5 mm of slop
                              #        around a separate 4 mm tongue);
                              #        with the tongue gone the slot
                              #        accepts the tang directly.
FOOT_HINGE_PIN_HOLE_D =  3.4  # mm -- through-hole diameter for the M3
                              #        hinge bolt; 0.2 mm clearance over a
                              #        nominal 3.2 mm M3 shank so the joint
                              #        rotates freely.
FOOT_HINGE_PIN_LEN    = 16.0  # mm -- pin length specification (M3 x 16
                              #        pan-head + M3 nylock nut).  Stack
                              #        = 3.5 + 6.4 + 3.5 = 13.4 mm of fork
                              #        + tang plus ~2.6 mm into the nylock
                              #        nut (still well within the nylock's
                              #        engagement range).

# Hinge axis Z position in tibia local: 10 mm below the tibia spar
# centreline (= 1 mm below the spar's bottom face at z = -9).  This
# pulls the tang down past the spar so the foot's fork mouth opens
# clearly onto free space below the spar and the foot pad has room
# to swing.
FOOT_HINGE_TIBIA_Z   = -10.0  # mm -- pin axis z in tibia-local

# Hinge axis Z above the disk bottom when the foot stands.
def foot_hinge_foot_z(base_extra_h: float = 0.0) -> float:
    return (FOOT_PAD_BASE_H + float(base_extra_h)
            + FOOT_PAD_BOSS_H + 7.0)


FOOT_HINGE_FOOT_Z    = foot_hinge_foot_z(0.0)   # = 14.0 (nominal pad)
FOOT_HINGE_FORK_OVER_PIN = 4.0    # mm of fork-cheek material above the pin
                                  #        axis (so the M3 hole has a
                                  #        continuous ring of plastic
                                  #        around it on the top side).

# Foot-pad fork X extent (along the foot's +X = spar direction).
# Sized so the M3 hole has > 3 mm of material on each side of the
# bore axis in X.  Matches the pre-inversion FOOT_HINGE_TONGUE_X
# value so the fork preserves the same rotational clearance the old
# tongue + slot pair had.
FOOT_HINGE_FORK_X    = 10.0       # mm

# Tibia tang bulk extent.  The tang is the SINGLE tongue at the spar's
# far end (around x = TIBIA_LENGTH); the FOOT_TANG_X_* constants set
# how far the tang extends inboard / past the spar tip and how far
# it reaches below the pin axis.  Renamed from the pre-2026
# ``FOOT_CLEVIS_X_*`` block when the tibia's clevis became a single
# tang; numeric values are unchanged so the tang occupies the same
# X / Z footprint as the old clevis bulk.
# Foot-fitting local X of the hinge pin (from tube-socket mouth).
# RETIRED with the hinged foot (Aug 2026).
FOOT_FITTING_HINGE_X = 16.0   # mm -- nominal tang hinge centre
FOOT_FITTING_TANG_L  = 22.0   # mm -- nominal tang length along +X

FOOT_TANG_X_INBOARD       = 12.0  # mm -- tang inboard of x=TIBIA_LENGTH
FOOT_TANG_X_BEYOND_TIP    =  6.0  # mm -- tang extending past x=TIBIA_LENGTH
FOOT_TANG_BELOW_PIN       = 5.0   # mm -- material below the pin axis in
                                  #        the tang (gives a ~1.5 mm ring
                                  #        of plastic around the M3 hole
                                  #        on the bottom side)

# ---- Battery (two under-belly shorty packs, no printed holder) ------------
# Aug 2026: the single 138 x 46 x 24 mm 3S pack in the inter-plate bay is
# RETIRED (it made the top deck unwieldy and would NOT fit under the
# belly -- verified by projecting every under-belly obstruction at the
# pack's Z band).  Replaced by TWO "shorty" 3S 2200 mAh packs (Zeee
# 75 x 34 x 26.5 mm, ~137 g each) wired in PARALLEL via an XT60 Y-harness
# and mounted UNDER chassis_bottom's flat belly (z = -6), side by side.
#
# Placement (chassis frame, verified against all 6 legs' servo + retainer
# footprints sliced over the pack Z band): block centred on the ORIGIN,
# long axis yawed +30 deg (pointing at the leg-0 edge-mid direction --
# the only orientation that fits; 0/60 deg clips the diagonal saddles).
# Exact-mesh clearance to the nearest retainer pole: ~5 mm.  XT60 leads
# exit the block's +axis end (r = 37.5 toward az 30) into the open
# pocket inboard of the leg-0 saddle (inboard edge r = 53).
#
# Retention: industrial hook-and-loop (Dual Lock) between pack tops and
# the flat belly, plus the CENTRE velcro-strap slot pair (x = -8,
# y = +/-26 -- cut for the old bay pack and kept) as a safety strap: its
# straight y-run crosses the middle of the 30-deg block, wrapping both
# packs.  No chassis_bottom change needed.
#
# Ground note: pack undersides sit at z = -32.5 chassis-local, ~26 mm
# above the floor at the nominal STANCE chassis lift -- and the belly
# packs drop the CG vs the old top-side pack.
BATTERY_W = 75.0    # mm -- ONE pack, length (along the block's yawed axis)
BATTERY_D = 34.0    # mm -- ONE pack, width
BATTERY_H = 26.5    # mm -- ONE pack, height (hangs below the belly)
BATTERY_N_PACKS = 2
BATTERY_PACK_GAP = 4.0            # mm between the two packs' long edges
BATTERY_UNDER_YAW_DEG = 30.0      # block long-axis azimuth (leg-0 edge-mid)
BATTERY_UNDER_CENTRE = (0.0, 0.0)  # block centre, chassis XY
BATTERY_STRAP_W = 10.0   # velcro slot width
# X offsets (relative to BATTERY_HOLDER_CENTRE_X) of the 3 velcro-strap
# slot pairs cut through chassis_bottom.  Sized/placed for the retired
# 138 mm bay pack; kept as-is (chassis_bottom unchanged).  The CENTRE
# pair doubles as the under-belly safety-strap + battery-lead pass;
# +/-30 keeps the +X slot pair clear of the (+31.1, +/-31.1)
# brass-standoff holes (inner hole edge at 31.1 - 1.7 = 29.4 mm).
BATTERY_STRAP_DX = (-30.0, 0.0, 30.0)
# Slot-pair half-spacing: FROZEN at the retired bay pack's
# 46/2 + 3 = 26 mm (was derived from BATTERY_D, which now describes the
# 34 mm shorty packs) so the strap-slot pattern matches earlier prints.
BATTERY_STRAP_SLOT_Y = 26.0

# ---- Battery trunk pass-through hole (Aug 2026) ---------------------------
# The two under-belly packs each send a thick 2-wire lead (12-14 AWG
# V+/GND, XT60-terminated) UP through the chassis to the main XT60 /
# anti-spark switch on the top deck.  One dedicated port through
# chassis_bottom (plate + floor, z -6..+2) passes both terminated XT60s
# nose-first (housing face ~16 x 8 mm).
#
# Placement: centred on the +X VERTEX axis at (48, 0) -- a straight shot
# from the pack block's lead end (az 30, r ~37.5) under the belly, and
# straight toward the switch holster on chassis_top's +X edge above.
# Clearances: brass-standoff holes at (31.1, +/-31.1) ~22 mm from the
# hole corner; +X strap-slot pair at (22, +/-26) well away; the az-0
# under-belly data Wago (r = 70, spans r 61-79) starts 6 mm outboard of
# the hole's outer edge (55); the hole sits inside the chassis_top disk
# (r 57.5) so the leads surface in the inter-plate bay and hop the deck
# rim at az 0.  Cut as an off-centre through-feature in
# make_chassis_bottom (like the strap slots), so the C6 floor solid
# stays six-fold symmetric.
BATTERY_TRUNK_HOLE_CENTRE = (48.0, 0.0)   # chassis XY
BATTERY_TRUNK_HOLE_X = 14.0               # mm radial extent (chassis X)
BATTERY_TRUNK_HOLE_Y = 22.0               # mm tangential extent (chassis Y)

# RETIRED (Jul 2026): the BATTERY_FOOT_* / BATTERY_WALL /
# BATTERY_BOLT_ACCESS_OD constants and ``make_battery_holder`` are gone
# along with the clip-in battery_holder itself (retired Jun 2026 when
# the pack moved to velcro straps).  The chassis_bottom hole pattern
# for the holder feet (``with_battery_holder_holes``) and the 4 hex-key
# access bores through the leg-2/3 cradle shells were deleted with it.

# Historical strap-slot datum.  The retired bay pack sat at
# X = BATTERY_HOLDER_CENTRE_X = -8 (balanced between the old bus-bar
# strip at +X and the diagonal yaw-hub swings at -X); the 3 strap-slot
# PAIRS in chassis_bottom are cut about this X, so it must stay -8 to
# describe the as-printed plate even though the packs themselves now
# hang under the belly centred on the origin (BATTERY_UNDER_CENTRE).
BATTERY_HOLDER_CENTRE_X = -8.0   # mm -- strap-slot pattern centre, NOT the
                                 #       battery centre (Aug 2026)


def battery_pack_transforms() -> "list[np.ndarray]":
    """Chassis-frame (pre-lift) 4x4 transforms of the two under-belly
    shorty packs' CENTRES.  z = 0 at chassis_bottom's mesh centre, so the
    flat belly is z = -6 and each pack top sticks to it; the block is
    yawed BATTERY_UNDER_YAW_DEG with the packs offset +/- half the
    (width + gap) perpendicular to the long axis."""
    from trimesh.transformations import (
        translation_matrix as _T, rotation_matrix as _R)
    cz = -(CHASSIS_PLATE_T / 2.0 + 4.0) - BATTERY_H / 2.0   # belly -6, -19.25
    yaw = np.deg2rad(BATTERY_UNDER_YAW_DEG)
    off = (BATTERY_D + BATTERY_PACK_GAP) / 2.0
    out = []
    for s in (-1.0, 1.0):
        out.append(_T([BATTERY_UNDER_CENTRE[0], BATTERY_UNDER_CENTRE[1], cz])
                   @ _R(yaw, [0, 0, 1])
                   @ _T([0.0, s * off, 0.0]))
    return out

# ---- M2.5 brass heat-set insert (Raspberry Pi mount) ---------------------
# McMaster 94459A106: M2.5 brass knurled heat-set insert.  Specs:
#   pilot Phi 3.0 mm, recommended pilot depth 4.5 mm,
#   insert OD (knurled max) 3.6 mm, insert length 4.0 mm.
# Installed with a soldering iron (same technique as the M3 inserts that
# carry the cradle servo bolts and the battery_holder foot bolts).  The
# Pi 4 / Pi 5 mounts on the electronics_tray via 4 of these inserts +
# M2.5 x 8 SHCS (PN_M25X8_SHCS).  Same fastener stock as the servo
# spline center screw -- a plain M2.5 x 8 SHCS with the role label
# distinguishing it from the captive spline screw.
INSERT_M25_PILOT_OD       = 3.0   # mm -- printed pilot Phi
INSERT_M25_PILOT_DEPTH    = 4.5   # mm -- printed pilot depth
INSERT_M25_INSERT_OD      = 3.6   # mm -- installed brass body knurled OD
INSERT_M25_INSERT_LENGTH  = 4.0   # mm -- physical insert length
INSERT_M25_BOLT_LENGTH    = 8.0   # mm -- M2.5 x 8 SHCS clamps the Pi
INSERT_M25_BOLT_HEAD_OD   = 4.5   # mm -- M2.5 SHCS head Phi
INSERT_M25_BOLT_HEAD_H    = 2.5   # mm -- M2.5 SHCS head height

# ---- Electronics tray ----------------------------------------------------
#
# Carries the THREE control boards used in the prototype (May 2026
# upgrade -- replaces the Arduino-Nano-only layout):
#
#   * Arduino Mega 2560 R3 (ELEGOO clone) -- 101.5 x 53.3 mm PCB with
#     the standard 4-hole Mega 2560 R3 footprint at offsets
#     MEGA_HOLES below (asymmetric pattern; see Arduino's reference
#     drawing).  4 x M3 SHCS into 4 x M3 brass heat-set inserts
#     (McMaster 94459A130) embedded in printed tray bosses.  Replaces
#     the old Arduino Nano hole pattern; the firmware moved to the
#     Mega so the Nano is no longer on the BOM.
#
#   * Raspberry Pi 4 Model B (or Pi 5; same 85 x 56 mm footprint with
#     the same 49 x 58 mm 4-hole pattern at offsets PI_HOLES below).
#     4 x M2.5 SHCS into 4 x M2.5 heat-set inserts (McMaster
#     94459A106) embedded in printed tray bosses.  NEW -- no Pi
#     mounting existed before the May 2026 hardware-arrival pass.
#
#   * 2 x Adafruit PCA9685 16-channel PWM driver -- 62 x 25 mm PCB
#     with the standard 54 x 16 mm 4-hole pattern (PCA_HOLES below).
#     4 x M3 SHCS into 4 x M3 heat-set inserts per board.  May 2026
#     "essentials" pass: the second PCA9685 (was previously cable-
#     tied / unbolted) now gets its own bolted hole pattern at
#     PCA2_CENTRE.  Two PCA9685 boards together carry the full 18
#     servos (9 per driver, daisy-chained over I2C: primary at
#     ``0x40`` and secondary at ``0x41``).
#
# Board placement (tray-local coords; tray origin at chassis (0, 0);
# tray-local +X = chassis +X):
#
#                    +Y (tray, chassis)
#                       ^
#       Mega2560        |     PCA9685 (long
#       (long axis      |      axis along Y;
#       along X; USB-B  |      servo-pin headers
#       + barrel jack   |      ride on +Z face)
#       on +X edge)     |
#                       |
#   ---------------------(0,0)------------------ +X
#                       |
#       Pi 4            |
#       (long axis      |
#       along X;        |
#       USB-A +         |
#       Ethernet on -Y  |
#       edge)           |
#
# Tray-local board centres come from MEGA_CENTRE / PI_CENTRE /
# PCA_CENTRE; the per-board mount holes come from MEGA_HOLES /
# PI_HOLES / PCA_HOLES (offsets from each board's centre).
#
# Tray sits in the same middle-bay z slot as the original 100 x 70 mm
# tray (z = CHASSIS_PLATE_T / 2 + 3 = 5 mm above chassis_bottom top
# face); the boards sit on ELEC_STANDOFF_H = 5 mm tall bosses.  Each
# boss is Phi CRADLE_BOSS_OD = 8 mm (M3 sites) or Phi 6 mm (M2.5
# sites) so the heat-set insert pocket has >= 1.5 mm of radial
# plastic.  The chassis-side mount holes still match the
# 35-mm-radius / 45-deg square pattern shared with chassis_bottom +
# chassis_top (and the arm baseplate, if present), with a Phi 5.5 mm
# x 3 mm-deep counterbore from the tray's TOP face so the chassis
# bolt heads sit FLUSH with the tray top -- the board standoff bosses
# on top of the tray then have an unobstructed 5 mm of clear air
# above the tray face for the M3 / M2.5 SHCS heads that clamp the
# boards onto the heat-set inserts.
#
# The tray's footprint OVERLAPS the battery_holder body and the two
# +X-side coxa_bracket flanges at the z = 5..8 slab the way the prior
# 100 x 70 mm tray did (the verifier does not flag chassis-fixed
# parts against each other, only against dynamic leg parts via
# ``check_workspace_self_collision``).  Cable-clearance keep-out
# volumes for the Mega's USB-B + barrel jack and the Pi's USB-C /
# HDMI / USB-A / Ethernet stack ARE modelled (May 2026
# "essentials" pass) -- see ``cable_keepouts.py`` for the
# registry and ``_verify_prototype.check_cable_clearance`` for
# the verifier.  Each keep-out is a simple box extending from the
# connector face along the cable's natural exit direction; the
# verifier asserts each keep-out has < 50 mm^3 of overlap with
# every printed STL, every modelled-electronics STL and every
# fastener mesh.
ELEC_TRAY_W = 160.0  # mm -- X dimension (long axis; spans chassis +/-80 mm)
# Tray Y span: May 2026 "Pi cantilever" pass grew the tray from 130 ->
# 135 mm (asymmetric, only in -Y) so the Pi's -Y mount-boss pair at
# tray-local y = -66.5 still lands on tray plastic with ~ 1 mm of
# radial wall around each boss.  The tray's chassis-frame Y centre
# was shifted from 0 -> -2.5 in lockstep so the tray's +Y edge stays
# at chassis y = +65 (the Mega + BEC cradle + PCA9685 layout on the
# +Y half doesn't move in chassis frame); only the -Y edge extends
# from -65 to -67.5 in chassis frame.  See ELEC_TRAY_CENTRE_Y +
# PI_CENTRE comments below for the cantilever rationale.
ELEC_TRAY_D = 135.0  # mm -- Y dimension (spans tray-local +/- 67.5 mm)
ELEC_TRAY_T =   3.0  # mm -- thickness (unchanged from the original tray)

ELEC_STANDOFF_H       = 5.0   # mm -- printed boss height above the tray top
                                # (boards sit ELEC_STANDOFF_H mm above the
                                # tray face so solder joints + headers
                                # underneath the PCB clear the tray).
ELEC_BOSS_OD_M3       = CRADLE_BOSS_OD   # 8 mm -- reuse the cradle constant
ELEC_BOSS_OD_M25      = 6.0    # mm -- smaller M2.5 boss (Phi 3 mm pilot +
                                # 1.5 mm wall = Phi 6 mm minimum).
ELEC_CHASSIS_COUNTERBORE_OD   = 5.5    # mm -- M3 SHCS head clearance Phi
ELEC_CHASSIS_COUNTERBORE_DEPTH = 2.0   # mm -- counterbore depth from tray top.
                                        # MUST be < ELEC_TRAY_T (= 3 mm) so a
                                        # plastic rim survives at the cbore
                                        # floor for the M3 SHCS head to clamp
                                        # against.  With ELEC_TRAY_T = 3 mm and
                                        # cbore_depth = 2 mm, the cbore floor
                                        # sits 1 mm above the tray's bottom
                                        # face -- the bolt head bears on that
                                        # 1 mm rim (annulus radius 1.7..2.75
                                        # mm) and the tray bolts DOWN onto an
                                        # M3 heat-set insert in a chassis_
                                        # bottom mounting boss (see
                                        # TRAY_MOUNT_BOSS_* below).  May 2026
                                        # tray-mount fix: cbore_depth was
                                        # 3.0 mm before; the tray-bolt
                                        # audit caught that the cbore ate the
                                        # full tray thickness, leaving NO rim
                                        # for the head to clamp against.

# RETIRED (Jul 2026): the TRAY_MOUNT_BOSS_* constants and the tray-mount
# bosses + insert pockets they described on chassis_bottom are deleted.
# The in-gap electronics_tray they mounted was retired in the Jun 2026
# deck redesign (electronics moved to the standoff-column deck trays
# above chassis_top), and the leftover 3-mm-tall boss bodies at
# (+/-24.75, +/-24.75) sat inside the real 138 x 46 mm battery's
# footprint on the plate top face.

# Mega 2560 R3 mounting holes (Arduino reference drawing).  Listed in
# the BOARD'S OWN coordinate frame with origin at the PCB's bottom-left
# corner, +Y = USB-B end, +X = right edge as drawn.  All four are
# Phi 3.2 mm M3 clearance.
MEGA_PCB_W = 53.3   # mm -- PCB width  (board X dimension in board frame)
MEGA_PCB_D = 101.5  # mm -- PCB depth  (board Y dimension; USB-B at +Y end)
_MEGA_HOLES_BOARD_FRAME = (
    ( 2.54, 15.24),     # bottom-left
    (50.80, 15.24),     # bottom-right
    ( 7.62, 66.04),     # mid-left
    (50.80, 90.17),     # top-right (near USB-B)
)
# Hole offsets relative to BOARD CENTRE in the board's frame:
_MEGA_HOLES_BOARD_CENTRE = tuple(
    (bx - MEGA_PCB_W / 2.0, by - MEGA_PCB_D / 2.0)
    for bx, by in _MEGA_HOLES_BOARD_FRAME
)
# 90 deg CW rotation maps board +Y (USB-B end) to tray +X so the USB-B
# + barrel jack faces the +X chassis edge: (bx, by) -> (by, -bx).
MEGA_HOLES = tuple(
    (by, -bx) for (bx, by) in _MEGA_HOLES_BOARD_CENTRE
)
# Mega centre in tray-local coords.  Placed on the -X +Y quadrant so
# the bolt heads at the 35-mm-radius chassis mount pattern don't poke
# up through the Mega PCB (the tray-top counterbore recess takes
# care of the head clearance regardless; the offset just keeps the
# layout tidy).  May 2026 "essentials" pass: nudged 3 mm in -X (from
# -25 to -28) so the corridor between Mega's +X edge and PCA1's
# new -X edge widens to ~ 28 mm -- enough to drop in a snap-fit
# BEC cradle for the 2 x switching BEC bodies (24 x 15 x 8 mm each).
# Mega's -X edge at MEGA_CENTRE_X - MEGA_PCB_D/2 = -78.75 mm sits
# 1.25 mm inside the tray's -X edge at -80, with the Mega's own
# -X-most chassis-mount hole at -63.51 mm (well inboard of the
# tray edge), so the shift doesn't cantilever any board feature
# past the printed plate.
#
# May 2026 "Pi cantilever" pass: tray-local Y bumped from +28 -> +30.5
# to compensate for the tray's chassis-frame Y centre shift (0 ->
# -2.5).  The Mega's chassis-frame position (-28, +28) is UNCHANGED.
MEGA_CENTRE = (-28.0, +30.5)

# Raspberry Pi 4 / Pi 5 mounting holes.  Pi PCB is 85 x 56 mm with
# 4 x M2.5 holes on a 58 x 49 mm rectangle (asymmetric -- the long-
# axis hole pair is offset toward one end).  Listed in board frame
# with +X = PCB long axis (85 mm), +Y = short axis (56 mm), origin
# at the bottom-left corner.
PI_PCB_W = 85.0     # mm -- long axis
PI_PCB_D = 56.0     # mm -- short axis
_PI_HOLES_BOARD_FRAME = (
    ( 3.5,  3.5),
    (61.5,  3.5),
    ( 3.5, 52.5),
    (61.5, 52.5),
)
_PI_HOLES_BOARD_CENTRE = tuple(
    (bx - PI_PCB_W / 2.0, by - PI_PCB_D / 2.0)
    for bx, by in _PI_HOLES_BOARD_FRAME
)
# Pi long axis stays along tray +X so the USB-A + Ethernet bank (on
# the -Y long edge) faces chassis -Y, where the user can plug cables
# in from outside the chassis past the chassis_top -Y apothem at
# y = -70.  The USB-C + HDMI bank (on the -X short edge) faces
# chassis -X so HDMI/USB-C cables exit in a separate corridor from
# the +X PCA9685 servo header bank.
PI_HOLES = _PI_HOLES_BOARD_CENTRE
# Pi centre in tray-local coords.  -X -Y quadrant so the Pi's
# connector bank sits opposite the Mega's USB-B + barrel jack.
#
# May 2026 "Pi cantilever" pass: Y moved from -33 -> -42 (in tray-
# local frame, with the tray centre also shifted from chassis y = 0
# to chassis y = -2.5) so the Pi cantilevers south past the
# chassis_top -Y apothem.  Pi PCB -Y edge in chassis frame =
# ELEC_TRAY_CENTRE_Y + PI_CENTRE_Y - PI_PCB_D/2 = -2.5 + (-42) - 28
# = -72.5 mm -- 2.5 mm PAST chassis_top's -Y apothem at -70.  The
# entire USB-A/Ethernet connector bank (which is on the Pi's -Y
# long edge, 16 mm of body sticking off the edge -- chassis y range
# = -72.5 .. -88.5) is OUTSIDE the chassis_top hexagon silhouette
# in plan view, so the 17 mm-tall USB-A stacks + 14 mm-tall Ethernet
# RJ45 tops (which reach into the chassis_top z-band at z in
# [34, 38]) are in free air with no chassis_top above them.  Pre-
# cantilever (PI_CENTRE_Y = -33, tray-centre at 0) the PCB -Y edge
# was at chassis y = -61, inside the chassis_top hexagon, and the
# Ethernet jack body punched THROUGH chassis_top.
#
# Pi -Y mount holes at PI_CENTRE_Y + PI_HOLES y = -42 + (-24.5) =
# -66.5 in tray-local; with the tray's -Y edge at tray-local
# y = -ELEC_TRAY_D/2 = -67.5, the boss has ~ 1 mm of plastic wall
# around its -Y side.  Pi +Y mount holes at -42 + 24.5 = -17.5 sit
# well inboard of the tray's +Y edge.
PI_CENTRE = (-20.0, -42.0)

# ---------------------------------------------------------------------------
# Arduino Uno Q -- the SINGLE compute + servo-bus controller (Jun 2026 refit).
# ---------------------------------------------------------------------------
# The Uno Q carries an on-board Linux SoC AND an MCU, so it REPLACES the
# Raspberry Pi + the USB-to-TTL bus adapter entirely: it talks to the
# STS3215 serial bus directly.  UNO form factor: 68.58 x 53.34 mm PCB,
# bottom-side components < 2 mm tall.
#
# Mounting holes use the standard Arduino UNO pattern, which is deliberately
# NON-SQUARE (user note): the two +X holes are offset in Y from each other
# and from the -X pair.  Coordinates below are the canonical UNO hole
# template in board-corner frame, then recentred on the board centre so the
# pattern can be dropped onto a tray with ``_absolute_xy(UNO_Q_CENTRE, ...)``.
UNO_Q_PCB_W = 68.58    # long axis (X)
UNO_Q_PCB_D = 53.34    # short axis (Y)
_UNO_Q_HOLES_BOARD_FRAME = (
    (15.24,  2.54),
    (15.24, 50.80),
    (66.04,  7.62),
    (66.04, 35.56),
)
UNO_Q_HOLES = tuple((bx - UNO_Q_PCB_W / 2.0, by - UNO_Q_PCB_D / 2.0)
                    for (bx, by) in _UNO_Q_HOLES_BOARD_FRAME)

# ---------------------------------------------------------------------------
# XINGYHENG step-down (buck) converter -- 12 V LiPo -> 5 V logic rail.
# ---------------------------------------------------------------------------
# Same module the prototype_v1 tooling plate is cut for: ~66 x 52 mm body
# with 4 x M3 mounting holes on a 53 x 39 mm rectangle (see
# prototype_v1/make_tooling_plate.py BUCK_HOLE_X / BUCK_HOLE_Y).
BUCK_PCB_W = 66.0
BUCK_PCB_D = 52.0
BUCK_PCB_H = 21.0      # tallest component (inductor / trimpot) envelope
BUCK_HOLE_X = 53.0     # hole centre-to-centre across the long axis
BUCK_HOLE_Y = 39.0     # hole centre-to-centre across the short axis
BUCK_HOLES = tuple((sx * BUCK_HOLE_X / 2.0, sy * BUCK_HOLE_Y / 2.0)
                   for sx in (-1, 1) for sy in (-1, 1))

# ---------------------------------------------------------------------------
# Stacked electronics decks -- RETIRED (Aug 2026 as-built stack).
# ---------------------------------------------------------------------------
# The printed uno_q_tray / buck_tray / spider_carapace / imu_pad stack is
# retired.  Live robot: 20 mm posts + magnets hold a Ø110 hex board (Uno Q
# + breakout), with hex_raised_platform above (screen on top); MPU on chassis_bottom.
# Constants below remain only so legacy builders / old quotes still resolve.
DECK_TRAY_T = 3.0                       # tray plate thickness (legacy)
DECK_BOSS_OD_M3 = 6.5                   # board-standoff boss OD (M3 insert)
DECK_STANDOFF_BOSS_H = 4.0              # printed boss lifting board off plate
DECK_COLUMN_DX = 41.0                   # +/-X column position (clears 68.6 board)
DECK_COLUMN_DY = 33.0                   # +/-Y column position (clears 53.3 board)
DECK_COLUMN_XY = tuple((sx * DECK_COLUMN_DX, sy * DECK_COLUMN_DY)
                       for sx in (-1, 1) for sy in (-1, 1))
DECK_COLUMN_BOSS_OD = 8.0               # column-bolt boss OD on each tray
DECK_TRAY_W = 2 * DECK_COLUMN_DX + 14.0  # 96 mm
DECK_TRAY_D = 2 * DECK_COLUMN_DY + 14.0  # 80 mm
DECK_LEVEL_1_STANDOFF_H = 16.0          # chassis_top -> uno_q tray gap (legacy)
DECK_LEVEL_2_STANDOFF_H = 22.0          # uno_q tray -> buck tray gap (legacy)
DECK_ZIPTIE_SLOT_W = 3.0                # zip-tie slot width
DECK_ZIPTIE_SLOT_L = 10.0               # zip-tie slot length
DECK_SPARE_HOLE_OD = 3.4               # M3 clearance for spare bolt-down holes
DECK_VENT_SLOT_W = 4.0                  # buck-tray cooling vent slot width
DECK_VENT_SLOT_L = 26.0                 # buck-tray cooling vent slot length

# ---------------------------------------------------------------------------
# As-built electronics stack (Aug 2026) -- match the real robot.
# ---------------------------------------------------------------------------
# Four posts at CHASSIS_STANDOFF_HOLES_XY (±31.1): 20 mm M3 standoff +
# ~2.5 mm M3 thumb nut + Ø8×8 mm magnet.  Magnets hold the Ø110 hex mount
# plate (Uno Q + breakout).  hex_raised_platform_110 (screen variant, 72 mm
# legs) sits on that plate with the screen on the top face.  MPU-6050 is
# glued on chassis_bottom (inter-plate bay), inboard of physical leg 1 —
# see MPU_ASBUILT_*.  No buck -- battery feeds PDB and Uno Q directly.
# Power Wagos on the chassis-top periphery; data Wagos under chassis near
# the yaw retainers.
HEX_POST_STANDOFF_H = 20.0
HEX_POST_STANDOFF_OD = 5.0              # M3 brass hex standoff body OD approx
HEX_POST_THUMB_NUT_T = 2.5
HEX_POST_THUMB_NUT_OD = 8.0
HEX_POST_MAGNET_OD = 8.0
HEX_POST_MAGNET_H = 8.0
HEX_POST_STACK_H = (HEX_POST_STANDOFF_H
                    + HEX_POST_THUMB_NUT_T
                    + HEX_POST_MAGNET_H)   # 30.5 mm

HEX_MOUNT_PLATE_T = 2.0                 # extra_stl / xtool mount plate
# Aug 2026: the magnet-held mount plate is now a ROUND Ø115 disc matching
# the chassis_top disc below it (was a Ø110 hex).  Worst-case coxa sweep
# keep-out starts at r ≈ 59.6, so the 57.5 radius keeps ≥2 mm margin.
HEX_MOUNT_PLATE_DIAM = 115.0
# Late-Aug 2026 design review: the magnet-held plate gets SHEAR
# REGISTRATION -- 4 printed underside bosses (OD 14, bore Ø8.4, 2 mm)
# that socket the top of each Ø8 post magnet so gait shake can't slide
# the deck (magnets only carry pull); the legacy 49.5 mm bolt square is
# dropped from the plate.  Review round 2: the plate also carries the
# Uno Q's 3-point UNO mount holes (M3x8 into thumb nuts; the separate
# hex_uno_q_io_board is retired) and 3 stand-foot holes at az
# 90/210/330 (M3x8 up into the screen stand's blind self-tap pilots).
# Print-only -- the SVG cut file is retired with the bosses.
# See tools/make_xtool_hex_raised_platform.py.
HEX_RAISED_LEG_H = 28.0                 # screen-stand legs (single source
                                        # for the tool's LEG_H_SCREEN).
                                        # 62 -> 72 (Aug 2026) -> 28
                                        # (late-Aug 2026 review: a 72 mm
                                        # tower on a magnet-held 2 mm
                                        # plate was the robot's highest
                                        # mass on its weakest joint; 28
                                        # still clears the Uno Q's ~12 mm
                                        # shield headers below the top)
HEX_RAISED_TOP_T = 2.0
HEX_RAISED_TOTAL_H = HEX_RAISED_LEG_H + HEX_RAISED_TOP_T
                                        # foot z=0 .. top of upper plate
                                        # (= _h28_screen.stl exact z-span;
                                        # the screen sits ON this face --
                                        # derived, so it can't drift from
                                        # the leg-height constant again)

SCREEN_PCB_W = 63.0                     # GMT020 / ST7789 long axis (X)
SCREEN_PCB_D = 35.0
SCREEN_PCB_T = 4.0

# As-built power distribution (Aug 2026): NO PDB.  The battery trunk
# lands on a CENTRAL pair of 5-port Wago 221-415 lever nuts on
# chassis_top -- one V+ splice + one GND splice, side by side near the
# plate centre (nudged +X so the motor controller's servo-plug face
# gets wire clearance to its west), wire entries facing +X (toward the
# switch / battery trunk) -- which feed the 6 corner power Wago pairs.
WAGO_TRUNK_CENTRE = (16.0, 0.0)         # chassis XY on chassis_top
WAGO_TRUNK_DY = 32.0                    # V+/GND nut centre-to-centre (Y);
                                        # 30 mm wide 221-415 + 2 mm gap

# Motor controller = Waveshare Bus Servo Adapter (A), real datasheet
# footprint 42 x 33 mm.  Its connectors live on the two 42 mm edges:
# one carries the latching 3-pin servo-bus plugs + UART header, the
# other the DC jack + power screw terminal + USB-C -- BOTH faces need
# free air for wires or the assembly does not work (user, Aug 2026).
#
# Placement: long axis on Y (MOTOR_CTRL_YAW_DEG = 90), centred in the
# WEST band of chassis_top at (-24, 0).  World +X face = servo/UART
# side (plugs exit toward the trunk-Wago gap); world -X face = power /
# USB side (wires exit toward the open west rim, still on-plate so
# nothing dangles into the leg-sweep airspace outside r = 57.5).
# The wire clearance zones below are enforced by the viz suite's
# connector_clearance check (tools/full_robot_viz_build.py).
MOTOR_CTRL_W = 42.0
MOTOR_CTRL_D = 33.0
MOTOR_CTRL_H = 12.0
MOTOR_CTRL_CENTRE = (-24.0, 0.0)
MOTOR_CTRL_YAW_DEG = 90.0
MOTOR_CTRL_SERVO_CLEAR = 14.0   # mm free air off the +X (servo-plug) face
MOTOR_CTRL_PWR_CLEAR = 13.0     # mm free air off the -X (terminal/USB) face
# Bench-plug corridor (Aug 2026, user): a laptop must be able to plug
# into the adapter's USB-C on the -X face, so the port needs the full
# plug-body depth of free air -- ~30 mm covers a USB-C overmold plus
# strain relief.  Unlike the 13 mm PWR zone (permanent wiring, must
# stay over the deck disc), this corridor is TEMPORARY-use: it may
# extend past the chassis_top rim into leg-sweep airspace because the
# robot is parked/limp whenever a bench cable is attached.  Enforced as
# its own connector_clearance zone (no permanent part may intrude).
MOTOR_CTRL_USB_CLEAR = 30.0     # mm bench USB-C plug corridor, -X face

# Generic shield / breakout next to Uno Q on the hex plate.
BREAKOUT_W = 45.0
BREAKOUT_D = 30.0
BREAKOUT_H = 10.0
BREAKOUT_CENTRE = (0.0, 36.0)           # north of Uno Q, inside hex
UNO_Q_ON_HEX_CENTRE = (0.0, -12.0)      # south of centre under the screen
                                        # stand.  The plate's 3-point UNO
                                        # mount holes are derived from this
                                        # centre (make_xtool_hex_raised_
                                        # platform._uno_hole_xy) -- move it
                                        # and the holes follow.

# As-built MPU-6050 (GY-521): glued to chassis_top's TOP face just SOUTH
# of the central trunk Wago pair (Aug 2026; earlier near-centre spots
# conflicted first with the LiPo footprint on the bay floor, then with
# the motor controller's wire-clearance zones on the west band).  Still
# well inboard (r = 43) so the gyro stays close to the yaw axis, with a
# short I2C run up to the Uno Q.
#
# The GY-521's right-angle 8-pin header runs along one LONG (21 mm)
# edge; with yaw = 90 the long axis is on Y and the header row faces
# world -X, exiting into the open south-west deck (MPU_WIRE_CLEAR zone,
# enforced by the viz connector_clearance check).
MPU_ASBUILT_CENTRE = (2.0, -43.0)       # chassis XY on chassis_top top face
MPU_ASBUILT_YAW_DEG = 90.0
MPU_WIRE_CLEAR = 14.0                   # mm free air off the -X header face

# Wago 221-style lever-nut placeholders (simple box).
WAGO_W = 18.0
WAGO_D = 13.0
WAGO_H = 10.0
# Data: under chassis.  Aug 2026: moved from r = 48 on the LEG azimuths
# (which the two under-belly battery packs now occupy -- block corner
# reach ~r 52) to r = 70 on the hex VERTEX azimuths (i*60 deg), in the
# open belly pockets between adjacent retainer saddles: clear of the
# 30-deg pack block by >~ 12 mm, ~17 mm from the flanking saddle walls,
# and inboard of the corner power-Wago tray lips (r ~ 98).
WAGO_DATA_R = 70.0

# 3-conductor Wago 221-413 (datasheet dims): 18.7 wide across the three
# ports x 18.6 deep along the wire direction x 8.3 high (levers up).
WAGO3_W = 18.7
WAGO3_D = 18.6
WAGO3_H = 8.3

# 5-conductor Wago 221-415 (datasheet dims): 30.0 wide across the five
# ports x 18.6 deep x 8.4 high.  Two of these are the central battery
# trunk splices (one V+, one GND) that fan out the six corner branches.
WAGO5_W = 30.0
WAGO5_D = 18.6
WAGO5_H = 8.4

# 3.3 V rail splice (Aug 2026): ONE more 5-port 221-415 VHB-taped flat to
# the UNDERSIDE of the round mount plate near its south rim, wire entries
# facing -Y (the rim).  Feed = Uno Q 3V3 pin (drops through the plate's
# east wire port); loads = GY-521 MPU VCC (straight down to the deck) +
# screen VCC (back up through the west wire port and up a platform leg),
# with two spare ports.  The plate top there is Uno-Q territory and the
# rim channels between the raised-platform leg blades are too narrow for
# a 30 mm nut, so the open underside (only the 4 magnet pads live there)
# is the flat spot -- and the plate pulls off its magnets, so the levers
# stay serviceable.  Hanging 8.4 mm below the plate it bottoms at
# deck0 + 22.1, above everything on chassis_top at that XY.
WAGO_V33_CENTRE = (0.0, -36.0)          # mount-plate XY of the nut centre
# Ø8 wire pass-through ports flanking the nut's south wire face (also cut
# by tools/make_xtool_hex_raised_platform.py into the plate).
MOUNT_PLATE_WIRE_PORT_D = 8.0
MOUNT_PLATE_WIRE_PORT_XY = ((19.0, -44.0), (-19.0, -44.0))

# As-built per-leg power splices: ONE 5-port Wago 221-415 at each of the
# 6 chassis_bottom hex CORNER FLATS -- the straight plate-edge segments
# BETWEEN adjacent yaw cradles, outward normals at az = i*60 deg, edge at
# chassis r = WAGO_MOUNT_EDGE_R -- with the wire entries facing the
# chassis centre.  (Aug 16 2026, user: "5 slot waygos at each edge on the
# hexagon instead of 2 3 slot waygos" -- replaces the previous V+/GND
# PAIR of 3-port 221-413 per corner.  A side-by-side PAIR of 5-slot bays
# would need ~67 mm but the corner flat is only ~53.6 mm long, so each
# corner carries a single 30 mm bay, comfortably inside the flat.)  The
# nut PRESS-FITS into an open-top wall set INTEGRATED into the
# chassis_bottom top face (``_chassis_wago_tray_solid``; late-Aug 2026 --
# replaces the 6 separately printed, VHB-taped ``make_wago_mount`` trays).
# The chassis prints face-down, so the top-face walls print clean with no
# supports; the nut sits directly on the plate top (no tray floor) and
# the inward wire pull is reacted by solid plate material.
WAGO_MOUNT_BAY_CLEAR = -0.15  # mm total bay allowance on width and depth.
                              #   Aug 16 2026 (user: "make the holders
                              #   about 0.5-1mm tighter so I can press fit
                              #   in the waygo"): was +0.6 -- a drop-in
                              #   slip fit that sat loose on the bench --
                              #   now 0.75 tighter = 0.15 nominal
                              #   INTERFERENCE, a light press fit
WAGO_MOUNT_WALL_T    = 2.4    # mm side / outer wall thickness
                              #   (generous: a broken wall = chassis reprint)
WAGO_MOUNT_WALL_H    = 6.5    # mm wall height above the plate top face --
                              #   stops ~1.9 mm below the 8.4 mm nut top
                              #   so the nut stays easy to grip
WAGO_MOUNT_EDGE_R    = 100.0  # mm chassis r of the corner-flat plate edge;
                              #   the outer wall's outboard face is flush
                              #   with this edge
# Retired late-Aug 2026 with the separate taped tray: WAGO_MOUNT_FLOOR_T
# (= 3, tray floor -- the plate top face is the floor now),
# WAGO_MOUNT_LIP_DROP (= 8) and WAGO_MOUNT_LIP_CLEAR (= 0.15, the
# over-edge registration lip -- solid integration needs no lip).

# ---------------------------------------------------------------------------
# Spider carapace dome (Jun 2026) -- a domed cephalothorax/prosoma shell with
# the classic 8-eye spider face, bolted on as a THIRD deck level ABOVE the
# buck tray.  It reuses the 4-point DECK_COLUMN_XY (+/-41, +/-33) column
# pattern: 4 M3 standoffs rise from the buck-tray column bosses up to the
# carapace feet, so the dome bolts on / lifts off as one piece.  The dome
# floats high enough that its open skirt + rear window leave the whole
# electronics stack (Uno Q tray, buck converter at z<=104, battery, all wire
# exits and the per-leg cable drops) uncovered and vented.
#
# Z reference (chassis frame, no lift): the carapace SEAT/RIM plane (the
# part's local z = 0) sits at deck_top_face + L1 + L2 + L3 above chassis,
# i.e. DECK_LEVEL_3_STANDOFF_H above the buck-tray BASE.  Chosen so the rim
# clears the tallest electronics point (buck converter envelope top, z=104).
DECK_LEVEL_3_STANDOFF_H = 31.0          # buck-tray base -> carapace seat/rim
# Outer half-ellipsoid (prosoma) semi-axes, local frame: +X = forward
# (anterior; the eyes live on this front slope), +Y = lateral, +Z = up.
CARAPACE_AX = 72.0                      # mm -- fore/aft outer semi-axis
CARAPACE_AY = 62.0                      # mm -- lateral outer semi-axis
CARAPACE_HD = 34.0                      # mm -- dome height (rim -> apex)
CARAPACE_WALL = 5.0                     # mm -- shell wall (>= MIN_PRINT_T)
CARAPACE_FOOT_OD = 8.0                  # mm -- mount-foot boss OD at columns
# 8-eye layout (plan-view (x, y) on the front slope + lens radius).  Two
# rows: a large anterior-median pair (AME) + flanking anterior laterals
# (ALE) low on the face, and a posterior-median (PME) + wider posterior-
# lateral (PLE) pair set higher/back -- reads as a jumping-spider face.
CARAPACE_EYES = (
    # (x, y, radius) -- mirrored in +/-Y below for the paired eyes.
    (58.0,  9.0, 6.5),   # AME -- big forward pair
    (52.0, 23.0, 4.3),   # ALE -- smaller, flanking + slightly outboard
    (41.0, 14.0, 3.4),   # PME -- posterior median (higher up)
    (38.0, 31.0, 3.2),   # PLE -- posterior lateral (wider, higher)
)

# Adafruit PCA9685 PWM driver -- 62 x 25 mm PCB with 4 x M3 holes
# on a 54 x 16 mm rectangle centred on the board.  Long axis = 62.
PCA_PCB_W = 62.0
PCA_PCB_D = 25.0
PCA_HOLES_BOARD_CENTRE = (
    (-27.0, -7.5),
    (+27.0, -7.5),
    (-27.0, +7.5),
    (+27.0, +7.5),
)
# Rotate PCA 90 deg so its long axis lies along tray +Y -- tucks
# into the +X strip of the tray between the Mega's +X edge and the
# tray's +X edge without touching either board.
PCA_HOLES = tuple((by, -bx) for (bx, by) in PCA_HOLES_BOARD_CENTRE)
# Primary PCA9685 (I2C 0x40) centre in tray-local coords.  May 2026
# "essentials" pass: shifted from (+55, +20) to (+64, +20) so the
# corridor between the Mega's +X edge and PCA1's -X edge widens
# enough to drop in the snap-fit BEC cradle (see BEC_CRADLE_*
# constants) AND so PCA2 (I2C 0x41) can fit a symmetric hole
# pattern on the -Y half of the tray at PCA2_CENTRE.  PCA1 -X
# edge at PCA_CENTRE_X - PCA_PCB_D/2 = +51.5 mm, +X edge at
# +76.5 mm (3.5 mm inside the tray's +X edge at +80).  +X-most
# mount-boss outer face at +71.5 + CRADLE_BOSS_OD/2 = +75.5 mm,
# 4.5 mm inside the tray's +X edge.
#
# May 2026 "Pi cantilever" pass: tray-local Y bumped from +20 ->
# +22.5 to compensate for the tray's chassis-frame Y centre shift
# (0 -> -2.5).  PCA1's chassis-frame position (+64, +20) unchanged.
PCA_CENTRE = (+64.0, +22.5)
# Secondary PCA9685 (I2C 0x41) centre in tray-local coords.  May
# 2026 "essentials" pass: previously this PCA9685 was on the BOM
# but was just cable-tied to the chassis_top deck with no printed
# hole pattern; the "essentials" pass adds a proper bolted mount.
# Placed symmetrically opposite the primary PCA9685 across the
# tray's X axis (PCA1 at +Y, PCA2 at -Y, both in the +X strip
# beyond x = +51.5 mm).  PCA2 servo headers (along its long edge,
# i.e. tray +/-X faces of the rotated PCA9685 PCB) fan out into
# the -Y leg cable routing direction; PCA1's headers do the same
# in +Y.  Together the two boards drive all 18 servos (9 per
# board) without any header crossing the tray midline.
#
# May 2026 "Pi cantilever" pass: tray-local Y bumped from -20 ->
# -17.5 (+2.5) to compensate for the tray's chassis-frame Y centre
# shift (0 -> -2.5).  PCA2's chassis-frame position (+64, -20)
# unchanged.
PCA2_CENTRE = (+64.0, -17.5)

# ---- STS3215 serial-bus adapter (Jun 2026 refit) ------------------
# A single small USB-to-TTL half-duplex bus-servo adapter (Waveshare
# "Bus Servo Adapter (A)"-class, ~30 x 24 mm PCB) REPLACES the 2x
# PCA9685 PWM drivers AND the Arduino Mega: the STS3215s are serial
# bus servos, so the Pi drives them directly over one half-duplex TTL
# bus fanned out by this adapter (see motor_setup/feetech_bus.py and
# firmware/WIRING.md).  Mounted on 4 x M2.5 standoffs (Phi 6 mm boss,
# Phi 3 mm heat-set pocket -- same geometry as the Pi sites).  Sits in
# the +X strip the PCAs used to occupy, near the Pi's USB ports so the
# short USB lead reaches.
BUS_ADAPTER_PCB_W = 30.0
BUS_ADAPTER_PCB_D = 24.0
BUS_ADAPTER_HOLES = (
    (-11.0, -8.0),
    (+11.0, -8.0),
    (-11.0, +8.0),
    (+11.0, +8.0),
)
BUS_ADAPTER_CENTRE = (+55.0, -10.0)

# 35-mm-radius / 45-deg-square chassis-mount hole pattern (matches
# ``_hex_plate(with_centre_holes=True)`` on chassis_top + chassis_
# bottom).  The 4 M3 SHCS tray-mount bolts thread DOWN through the
# tray's cbore floor into M3 heat-set inserts captive in chassis_
# bottom's tray-mount bosses (May 2026 tray-mount fix; the brass
# standoffs no longer share this XY pattern -- see
# CHASSIS_STANDOFF_HOLES_XY below for the new standoff pattern).
ELEC_CHASSIS_MOUNT_R       = 35.0
ELEC_CHASSIS_MOUNT_HOLES_XY = tuple(
    (ELEC_CHASSIS_MOUNT_R * np.cos(np.pi / 4 + i * np.pi / 2),
     ELEC_CHASSIS_MOUNT_R * np.sin(np.pi / 4 + i * np.pi / 2))
    for i in range(4)
)

# Brass-standoff column pattern.  4 M3 F-F brass standoffs span the
# inter-plate gap on a 44 mm radius square pattern at the diagonals:
# (+/-31.1, +/-31.1).
#
# Jul 2026 battery-fit rework: the pattern moved OFF the old
# (+/-35, 0) / (0, +/-35) rotated-45-deg layout because the then-bay
# 138 x 46 mm pack (retired Aug 2026 -- battery is now two shorty
# packs UNDER the belly) needed a clear straight lane
# through the inter-plate bay along Y in [-23, +23] -- the old +/-X
# standoffs at (+/-35, 0) sat dead inside that lane.  At the
# diagonals the standoff bodies (hex AF 5.5 -> circumradius ~3.2 mm)
# clear the pack's long edge by 31.1 - 23 - 3.2 = 4.9 mm, stay 10 mm
# from the DECK_COLUMN_XY (+/-41, +/-33) clearance holes in
# chassis_top, and remain well inside chassis_top's 57.5 mm
# circumradius.  (The original May 2026 reason for the rotated-45
# pattern -- dodging the in-gap electronics_tray insert pockets at
# (+/-24.75, +/-24.75) -- is gone: that tray and its chassis_bottom
# bosses are retired.)
#
# Hardware (Jul 2026 F/F switch): the user's M-F studs cannot work
# here -- chassis_bottom is 8 mm thick (plate + merged floor) so a
# standard 6 mm male stud can't reach a nut on the -6 mm face.  Each
# standoff is now FEMALE-FEMALE and is bolted from BOTH faces:
#     * An M3 x 14 SHCS enters from BELOW chassis_bottom (head on the
#       -6 mm floor face), passes through the 8 mm plate stack and
#       threads ~6 mm into the standoff's bottom female thread.
#     * An M3 x 10 SHCS drops DOWN from above chassis_top into the
#       standoff's top female thread (unchanged).
# Both chassis_top and chassis_bottom carry these 4 clearance holes
# via ``_hex_plate(with_chassis_standoffs=True)``.
CHASSIS_STANDOFF_R          = 44.0
_STANDOFF_D                 = CHASSIS_STANDOFF_R / np.sqrt(2.0)  # 31.11 mm
CHASSIS_STANDOFF_HOLES_XY   = (
    (+_STANDOFF_D, +_STANDOFF_D),
    (-_STANDOFF_D, +_STANDOFF_D),
    (-_STANDOFF_D, -_STANDOFF_D),
    (+_STANDOFF_D, -_STANDOFF_D),
)

# Aug 16 2026 standoff SEAT PADS (chassis_bottom only).  The Aug 2026
# harness-port merge shifted each leg's open 18 x 28 drop port 1.5 mm
# INBOARD (radial span 43.5..61.5), and that quietly broke the 4
# standoff seats: each site sits 15 deg off a leg axis (44*sin(15) =
# 11.4 mm tangential < the port's 14 mm half-width; the old block
# comment claimed ">17 mm" -- a bad trig shortcut), so the port corner
# clipped through the hole's annulus.  The Phi 3.4 bore survived as a
# keyhole but a Phi 6 brass-standoff base / M3x14 head had nothing to
# seat on ("no longer really has holes where I can put a spacer" --
# user, bench).  Fix: union a Phi CHASSIS_STANDOFF_SEAT_PAD_OD pad
# through the full -6..+2 floor stack at each site (re-drilled Phi 3.4),
# restoring a >= 2.8 mm seat annulus.  Each pad nibbles a ~3.5 x 7 mm
# bite out of ONE corner of the adjacent port (legs az 30/150/210/330);
# the remaining L-shaped opening still passes the Molex connectors with
# room to spare.  check_leg_harness_drop exempts probe points inside
# the pads (it reads this constant -- single source of truth).
CHASSIS_STANDOFF_SEAT_PAD_OD = 9.0  # mm -- seat annulus around the Phi 3.4 bore

# Chassis-frame translation applied to the tray mesh by
# ``build_prototype_assembly._body_battery_parts`` and the verifier's
# ``_build_chassis_world`` / ``_build_world_leg0_printed_parts``.
# Moved from the original (+35, 0) to (0, 0) so the tray's chassis-
# mount holes line up CORRECTLY with the chassis-side
# 35-mm-radius / 45-deg-square pattern at chassis (+/-24.75, +/-24.75).
# The old +35 mm offset had the tray's bolt holes mapping to
# (+10.25, +/-24.75) and (+59.75, +/-24.75) instead -- a longstanding
# bug masked because the assembly preview is purely visual.
#
# May 2026 "Pi cantilever" pass: the Y centre shifted from 0 -> -2.5
# (paired with the asymmetric +5 mm -Y growth of ELEC_TRAY_D) so the
# Pi can cantilever 2.5 mm past the chassis_top -Y apothem at chassis
# y = -70.  The Pi sits at PI_CENTRE = (-20, -42) in tray-local coords,
# so its PCB -Y edge is at chassis y = -2.5 + (-42) - 28 = -72.5 --
# the entire USB-A/Ethernet connector bank sits OUTSIDE the chassis_
# top hexagon silhouette in plan view, so the 17 mm-tall USB-A stacks
# and 14 mm-tall Ethernet jack tops are in free air with no chassis_
# top above them to collide with.  Mega + PCA + BEC cradle tray-local
# Y centres bumped by +2.5 to keep their chassis-frame Y unchanged
# (the only board that actually moves in chassis frame is the Pi --
# everything else stays put; the 2.5 mm tray-local shift just keeps
# the chassis-frame positions invariant against the tray centre move).
# The chassis-mount holes in ``make_electronics_tray`` subtract
# ELEC_TRAY_CENTRE_* from the chassis-frame ELEC_CHASSIS_MOUNT_HOLES_XY
# positions so they STILL land on the 35-mm-radius chassis pattern
# after the tray is translated.
ELEC_TRAY_CENTRE_X = 0.0
ELEC_TRAY_CENTRE_Y = -2.5


# ---- BEC cradle ----------------------------------------------------------
#
# May 2026 "essentials" pass: 2 x 5V 5A switching BECs (Hobbywing
# UBEC form factor: ~ 24 x 15 x 8 mm body, ~ 5 cm input pigtail to
# the anti-spark switch's XT60 output and ~ 10 cm output pigtail to
# the nearest PCA9685's V+ / GND rail).  The BECs used to be cable-
# tied wherever; now they snap into a printed cradle that sits on
# top of the electronics_tray in the corridor between the Mega's
# +X edge and PCA1's -X edge.
#
# Cradle geometry summary:
#
#   * 2 BECs laid flat side-by-side along Y, BEC long axis (24 mm)
#     along tray X so the input and output pigtails exit naturally
#     out the cradle's +/- X end faces.
#   * Inner cavity:
#         X: BEC_BODY_L (24 mm) -- one BEC body's long axis
#         Y: 2 * BEC_BODY_W - BEC_CRADLE_INTERFERENCE (= 29.6 mm) so
#            the 2 BEC bodies share a 0.4 mm friction interference
#            fit across the inner span (no divider wall; the BECs
#            wedge against each other).
#         Z: BEC_BODY_H (8 mm) -- BEC body height
#   * Walls:
#         +/- X end walls: BEC_CRADLE_WALL (1.5 mm) thick, pierced
#                          by a Phi BEC_PIGTAIL_OD = 5 mm wire-
#                          exit channel centred on the cavity.
#         +/- Y side walls: BEC_CRADLE_WALL (1.5 mm) thick, full
#                           cavity height + lip.
#         Floor:           BEC_CRADLE_FLOOR (2 mm) thick.
#         +Z top lip:      BEC_CRADLE_LIP_H (2 mm) of inward
#                          overhang on each +/- Y side wall so the
#                          BEC body snaps in under the lip and
#                          stays captive.  The lip's inner edge
#                          is BEC_CRADLE_LIP_INSET (1.0 mm) inboard
#                          of the cavity wall, giving the BEC body
#                          a 1.0 mm-wide x 2 mm-tall retention tab
#                          to flex past on insertion.
#   * No bolts -- friction fit only.  The cradle sits free on the
#     tray's top face; gravity + the lip tabs hold the BECs and the
#     cradle itself stays in place because the corridor is bounded
#     by the Mega's +X edge to one side and PCA1's -X edge to the
#     other (the cradle floor's X extent is 1 mm shy of each
#     adjacent board's footprint for clearance).
BEC_BODY_L            = 24.0   # mm -- BEC PCB long axis (along cradle X)
BEC_BODY_W            = 15.0   # mm -- BEC PCB short axis (along cradle Y)
BEC_BODY_H            =  8.0   # mm -- BEC body height (along cradle Z)
BEC_PIGTAIL_OD        =  5.0   # mm -- Phi 5 mm wire-exit channel diameter
BEC_CRADLE_WALL       =  1.5   # mm -- side/end wall thickness
BEC_CRADLE_FLOOR      =  2.0   # mm -- floor thickness
BEC_CRADLE_LIP_H      =  2.0   # mm -- retention-lip overhang height
BEC_CRADLE_LIP_INSET  =  1.0   # mm -- lip inboard offset from cavity wall
BEC_CRADLE_INTERFERENCE = 0.4  # mm -- 2 BECs share this much overlap so
                                #     they wedge against each other for a
                                #     friction fit (positive = squeeze).
# Cradle centre in tray-local coords.  Placed in the corridor between
# the Mega's +X edge (at +22.75 mm = MEGA_CENTRE_X + MEGA_PCB_D/2)
# and PCA1's -X edge (at +51.5 mm = PCA_CENTRE_X - PCA_PCB_D/2).  The
# corridor is 28.75 mm wide; the cradle's outer X extent is
# BEC_BODY_L + 2 * BEC_CRADLE_WALL = 27 mm so there's ~ 0.9 mm of
# clearance on each side.  Chassis-frame Y position is +20 so the
# cradle's pigtail exits land within easy cable-routing distance of
# the nearest PCA9685's V+ / GND rail (PCA1's -X servo header row).
#
# May 2026 "Pi cantilever" pass: tray-local Y bumped from +20 ->
# +22.5 (+2.5) to compensate for the tray's chassis-frame Y centre
# shift (0 -> -2.5).  Cradle's chassis-frame position (+37, +20)
# unchanged.
BEC_CRADLE_CENTRE     = (+37.0, +22.5)


# ---- Anti-spark switch holster -------------------------------------------
#
# May 2026 "essentials" pass: the anti-spark XT60 on/off switch
# ("LowPro RC", "HRB", typical body ~ 30 x 15 x 15 mm with 2 short
# XT60 pigtails) used to be cable-tied wherever the user could fit
# it.  Now it lives in a printed holster on chassis_top's +X edge
# between the L0 and L5 coxa_brackets, with the toggle protruding
# past the chassis vertex so the user can flip it from outside the
# chassis without opening the stack.
#
# Aug 16 2026 VELCRO pass (user): the bolt-down mount is RETIRED.
# The old design had a -X mounting ear on the holster + 2 Phi 8 mm
# insert bosses standing 5 mm proud of chassis_top, 2 M3 heat-set
# inserts and 2 M3 x 10 SHCS.  The user velcros the holster to the
# deck instead, so the holster is now JUST the socket (flat bottom
# for the velcro patch) and chassis_top's top face is completely
# flat again -- no bosses, no inserts, no ear, no bolts.
#
# Holster geometry summary (post-velcro):
#
#   * SOCKET (the whole body): a SWITCH_BODY_L x _W x _H box-with-
#     5-walls (open top so the switch body drops in from above)
#     that holds the switch body snugly (SWITCH_BODY_CL mm
#     clearance per axis).  The +X end wall has a SWITCH_TOGGLE_W
#     x SWITCH_TOGGLE_H rectangular cutout so the toggle protrudes
#     for user access.  The -X end wall has 2 x Phi
#     SWITCH_PIGTAIL_OD = 6 mm holes for the XT60 pigtails.  The
#     SWITCH_HOLSTER_FLOOR = 4 mm bottom slab is solid and FLAT --
#     that is the velcro face.
SWITCH_BODY_L         = 32.0   # mm -- switch body length (along X)
SWITCH_BODY_W         = 17.0   # mm -- switch body width (along Y)
SWITCH_BODY_H         = 17.0   # mm -- switch body height (along Z)
SWITCH_BODY_CL        =  0.5   # mm clearance per axis between body and cavity
SWITCH_HOLSTER_WALL   =  2.0   # mm -- holster outer wall thickness
SWITCH_HOLSTER_FLOOR  =  4.0   # mm -- solid floor slab under the switch
                                #     cavity.  Flat bottom = velcro face
                                #     (Aug 16 2026: the -X mounting ear
                                #     + bolt-down hardware are retired).
SWITCH_TOGGLE_W       = 14.0   # mm -- toggle cutout width (Y) in +X face
SWITCH_TOGGLE_H       = 10.0   # mm -- toggle cutout height (Z) in +X face
SWITCH_PIGTAIL_OD     =  6.0   # mm -- Phi 6 mm pigtail exit (clears a
                                #     12 AWG XT60 silicone-wire pigtail)
SWITCH_PIGTAIL_DY     =  5.0   # mm -- spacing between the 2 pigtail
                                #     channels on the -X face (centres
                                #     at +/- DY)

# Derived outer envelope.  (Aug 16 2026: OUTER_L = SOCKET_OUTER_L --
# the +14 mm mounting ear is retired with the bolt-down mount.)
SWITCH_SOCKET_OUTER_L = (SWITCH_BODY_L + 2.0 * SWITCH_BODY_CL
                          + 2.0 * SWITCH_HOLSTER_WALL)          # 37 mm
SWITCH_HOLSTER_OUTER_L = SWITCH_SOCKET_OUTER_L                  # 37 mm (was 51 with the ear)
SWITCH_HOLSTER_OUTER_W = (SWITCH_BODY_W + 2.0 * SWITCH_BODY_CL
                          + 2.0 * SWITCH_HOLSTER_WALL)          # 22 mm
SWITCH_HOLSTER_OUTER_H = (SWITCH_BODY_H + SWITCH_BODY_CL
                          + SWITCH_HOLSTER_FLOOR)               # 23.5 mm

# Holster placement in CHASSIS frame (chassis_top's TOP face sits
# at world z = CHASSIS_TOP_TOP_Z = 36 mm; the holster's FLOOR bottom
# mates with chassis_top's TOP face there).  Placed so the SOCKET
# +X face is flush with
# the chassis_top vertex at x = CHASSIS_TOP_FLAT_TO_FLAT / 2 /
# cos(30 deg) = 80.83 mm in +X; the toggle protrudes past the
# vertex for user access.  L0 and L5 coxa_bracket flanges are at
# (+86.6, +/- 50) -- well away from y = 0 so the holster's full
# 22 mm Y extent stays clear of them.
# chassis_top has FLATS on the +/-X side (at x = +/- apothem =
# CHASSIS_TOP_FLAT_TO_FLAT / 2 = +/- 70 mm; the flat spans
# y in [-40.41, +40.41] at x = +/- 70).  VERTICES at angles
# +/- 30, +/- 90, +/- 150 deg at distance circum = 80.83 mm.
# The "+X edge between L0 and L1 coxa_brackets" the user refers
# to is the +X flat (L5 is at angle 330 deg = (86.6, -50), L0 at
# angle 30 deg = (86.6, +50); the +X flat spans the gap between
# them along chassis_top's +X face).
SWITCH_CHASSIS_EDGE_X = CHASSIS_TOP_FLAT_TO_FLAT / 2.0  # = 70 mm
# Toggle must be reachable from outside the chassis: cantilever
# the holster so its +X face protrudes past chassis_top's +X
# flat by SWITCH_TOGGLE_REACH mm.  Toggle cutout itself adds
# another few mm of protrusion for the toggle stem.
SWITCH_TOGGLE_REACH    = 15.0  # mm -- holster +X face past chassis edge
# Holster MESH origin = holster X centre (midway between the
# socket's +X and -X faces; ear retired Aug 16 2026).  Place that
# origin so the holster's +X face is at SWITCH_CHASSIS_EDGE_X +
# TOGGLE_REACH -- the toggle keeps its exact as-built reach past the
# chassis edge; only the -X tail of the part got shorter.
SWITCH_HOLSTER_CENTRE_X = (SWITCH_CHASSIS_EDGE_X
                           + SWITCH_TOGGLE_REACH
                           - SWITCH_HOLSTER_OUTER_L / 2.0)
SWITCH_HOLSTER_CENTRE_Y = 0.0

# (Aug 16 2026: SWITCH_HOLSTER_BOLT_* / SWITCH_HOLSTER_BOSS_* /
# SWITCH_EAR_L are RETIRED with the bolt-down mount -- the holster
# velcros to the flat deck.  chassis_top carries no holster bosses,
# inserts, or clearance holes any more.)

# ---- IMU pad (MPU-6050 vibration-isolated mount) ------------------------
#
# May 2026: the MPU-6050 / GY-521 was the last "optional" item in the
# parts list without a fixed mounting location.  Promoted to standard
# kit now that the Pi 4 / Pi 5 is on the robot (ROS 2 / Python brain
# expects orientation feedback for closed-loop body-attitude control on
# uneven terrain).
#
# Vibration isolation: the MPU-6050's HF gyro noise floor is very
# sensitive to high-frequency mechanical vibration, and brushed /
# digital hobby servos generate plenty of it.  The IMU must be
# MECHANICALLY DECOUPLED from the chassis -- bolting the breakout
# rigidly to chassis_top would feed every PWM-update jitter and tooth-
# slop pulse straight into the gyro.
#
# Damping strategy: a small printed pad (``imu_pad.stl``) carries the
# IMU on 4 x M3 brass heat-set inserts, and the pad ITSELF is glued
# to chassis_top via a 3 mm-thick strip of 3M VHB / generic double-
# sided mounting foam tape.  The foam tape doubles as the mount AND
# the vibration damper.  No fasteners between the pad and chassis_top.
#
# (Alternative considered: a 4-corner printed standoff with TPU 95A
# sleeves.  Mechanically nicer but complicates the build -- needs a
# second filament + a separate slicer profile for the TPU sleeves --
# and the foam-tape variant gets you 80 % of the damping with zero
# extra print effort.  See PROTOTYPE.md step 12 for the assembly
# rationale.)
#
# Standard GY-521 breakout PCB dimensions:
#
#     PCB:       21.2 x 16.4 x 1.6 mm
#     Holes:     4 holes in a 15.0 x 11.0 mm pattern, Phi 3.0 mm
#                clearance (most boards; a minority use Phi 2.5 mm --
#                we treat as Phi 3.0 for compatibility).
#     Header:    1 x 8-pin male header along one long edge (VCC, GND,
#                SCL, SDA, XDA, XCL, AD0, INT).
#
# Pad geometry (printed):
#
#     Floor:       IMU_PAD_W x IMU_PAD_D x IMU_PAD_T mm, flat smooth
#                  bottom for foam-tape adhesion.  Slightly LARGER
#                  than the IMU PCB so the foam-tape footprint
#                  extends past the board edges.
#     Bosses:      4 x Phi IMU_PAD_BOSS_OD mm cylinders at the IMU's
#                  15 x 11 mm hole pattern, rising IMU_PAD_BOSS_H mm
#                  above the floor's top face.  Each boss carries
#                  an M3 brass heat-set insert (McMaster 94459A130)
#                  in a Phi INSERT_M3_PILOT_OD x INSERT_M3_PILOT_DEPTH
#                  pocket opening DOWNWARD from the boss top.  The
#                  IMU PCB rides on the 4 boss tops; M3 x 8 SHCS
#                  thread DOWN through the PCB into the inserts.
#
# IMU_PAD_BOSS_H is deliberately TALLER than the (now retired)
# switch-holster boss pattern (5 mm vs 3 mm) so a full 6 mm-deep insert pocket fits
# ENTIRELY above the pad's bottom face.  With the standard 2 mm pad
# floor + 5 mm bosses = 7 mm total, the 6.4 mm pocket overdrill ends
# at z = 0.6 mm above the pad bottom, leaving the bottom face FLAT
# for foam-tape adhesion.  A nominal "3 mm boss" would leave the
# pocket overhanging the foam-tape face by 1.4 mm -- which is why
# the convention diverges from the cradle / switch_holster boss
# heights here.
IMU_PCB_W = 21.2   # mm -- IMU breakout PCB long axis (X in pad frame)
IMU_PCB_D = 16.4   # mm -- IMU breakout PCB short axis (Y in pad frame)
IMU_PCB_T =  1.6   # mm -- IMU breakout PCB thickness (visual mesh only)
IMU_HOLE_PCD_X = 15.0   # mm -- IMU mounting-hole pattern X spacing
IMU_HOLE_PCD_Y = 11.0   # mm -- IMU mounting-hole pattern Y spacing

IMU_PAD_W       = 25.0   # mm -- pad X extent (slightly > IMU_PCB_W)
IMU_PAD_D       = 20.0   # mm -- pad Y extent (slightly > IMU_PCB_D)
IMU_PAD_T       =  2.0   # mm -- pad floor thickness (thin so foam tape
                          #      dominates the bond compliance)
IMU_PAD_BOSS_OD =  8.0   # mm -- boss OD around each insert pocket
IMU_PAD_BOSS_H  =  5.0   # mm -- boss height above the pad's top face
                          #      (= IMU PCB-to-pad standoff).  See the
                          #      block comment above for why we use
                          #      5 mm here instead of the nominal 3 mm
                          #      that the switch_holster / cradle
                          #      bosses use.
IMU_BOLT_CL     =  3.0   # mm -- Phi 3.0 mm clearance hole through each
                          #      boss for the IMU's M3 SHCS (M3 = Phi
                          #      3.0 mm body, +0 mm slop).  The IMU
                          #      PCB itself has the matching Phi 3.0
                          #      clearance hole.

# IMU pad chassis-frame placement.  Sit the pad on top of chassis_top
# at chassis (0, 0) so the gyro sits at the chassis CENTRE OF MASS --
# putting it at the chassis centre means the rotation rates are NOT
# contaminated by linear acceleration from the body swing.
# IMU_PAD_TAPE_T mm of double-sided mounting foam tape sits between
# the pad bottom and chassis_top top.  The pad's MESH ORIGIN is at
# the pad floor's BOTTOM face (z = 0 in pad-local), so
# build_prototype_assembly / inspect_build / verifier translate the
# pad to world z = chassis_top_top + IMU_PAD_TAPE_T.
IMU_PAD_CENTRE_X = 0.0    # mm -- chassis centre, no horizontal offset
IMU_PAD_CENTRE_Y = 0.0    # mm
IMU_PAD_TAPE_T   = 3.0    # mm -- nominal foam-tape thickness (3M VHB
                          #      / generic double-sided mounting foam).

# IMU hole positions in pad-local frame (mesh origin = pad floor centre).
IMU_PAD_HOLE_OFFSETS = (
    (-IMU_HOLE_PCD_X / 2.0, -IMU_HOLE_PCD_Y / 2.0),
    (-IMU_HOLE_PCD_X / 2.0, +IMU_HOLE_PCD_Y / 2.0),
    (+IMU_HOLE_PCD_X / 2.0, -IMU_HOLE_PCD_Y / 2.0),
    (+IMU_HOLE_PCD_X / 2.0, +IMU_HOLE_PCD_Y / 2.0),
)

# CHASSIS-frame XY of the 4 IMU bolt sites.  ``fastener_registry``
# enumerates the 4 bolts + 4 inserts at these positions.
IMU_PAD_BOLT_CHASSIS_XY = tuple(
    (IMU_PAD_CENTRE_X + ox, IMU_PAD_CENTRE_Y + oy)
    for ox, oy in IMU_PAD_HOLE_OFFSETS
)


# ---- Resolutions ---------------------------------------------------------
CYL_SECTIONS = 48     # cylinder facet count -- smooth STL, fast booleans

# ---- Standing pose used for the assembly preview ------------------------
# Same conventions as hexapod_walker.py: pitch angles measured from the
# coxa link's local horizontal X axis.
# Aug 2026 boot refit: the TPU foot_boot contacts AT the tibia tip
# (the old hinged pad hung ~24 mm below the centerline), so the same
# angles stand ~20 mm lower -- at 60 deg the hanging yaw servos and
# retainer feet grounded out (MuJoCo showed the robot resting on the
# servo boxes with the feet 6 mm in the air).  75 deg knee restores
# the ~58 mm hip height / ~20 mm belly clearance of the pad era.
STANCE_FEMUR_DEG = -25.0
STANCE_TIBIA_DEG =  75.0

# Output directories -- next to this script
#   stl_prototype/  -- slicer-ready printables only
#   stl_reference/  -- bought-part / fused-link visuals for MuJoCo & BuildViz
#                     (filenames keep the loud ``_DO_NOT_PRINT`` suffix)
STL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "stl_prototype")
REF_STL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "stl_reference")
EXTRA_STL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "extra_stl")
os.makedirs(STL_DIR, exist_ok=True)
os.makedirs(REF_STL_DIR, exist_ok=True)


# ---------------------------------------------------------------------------
# "Do not print" mesh registry
# ---------------------------------------------------------------------------
#
# Some meshes are emitted into stl_reference/ purely for SIMULATION, FIT-CHECK
# or RENDER purposes and must never be sent to a slicer / print service:
#
#   * Assembled-link sim mesh -- ``tibia_link`` is really a printed yoke
#     bonded onto a bought Ø8 carbon-fibre tube plus a pressed-on TPU
#     foot boot; the one-piece STL fuses a SOLID plastic tube and would
#     print as a useless solid rod.  The real tibia printables are
#     tibia_knee_yoke / foot_boot.  (``femur_link`` is NOT in this
#     category since the Jul 2026 merge #2 -- the femur genuinely IS one
#     printed part.)
#   * Commodity-hardware visuals -- the servo body, the aluminium disc horn,
#     the MPU-6050 / Uno Q / bus adapter / anti-spark switch / LiPo
#     battery: these are BOUGHT, not printed; the meshes exist only so the
#     inspector / MuJoCo / BuildViz can show where the bought part sits.
#   * ``assembly_preview`` -- the whole standing robot fused into one mesh.
#
# These are written under ``stl_reference/`` with a loud ``_DO_NOT_PRINT``
# filename suffix so nobody can mistake them for a printable in a file
# picker.  Resolve a logical mesh base name with ``stl_filename`` /
# ``stl_path`` -- printables stay in ``stl_prototype/<name>.stl``.
NOPRINT_SUFFIX = "_DO_NOT_PRINT"

NOT_PRINTED_MESHES = frozenset({
    # femur_link left OUT deliberately: since the Jul 2026 merge #2 the
    # femur IS one printed part, so femur_link.stl is a real printable.
    "tibia_link", "assembly_preview",
    "servo_body", "servo_horn", "disc_horn", "yaw_bearing", "mpu6050",
    "uno_q", "buck_converter",
    "antispark_switch_body", "antispark_switch_toggle",
    "lipo_battery_body", "lipo_xt60",
    # As-built electronics stack visuals (Aug 2026; "pdb" retired --
    # power distribution is all Wago lever nuts now).
    "motor_controller", "breakout", "screen",
    "hex_post_standoff", "hex_post_thumb_nut", "hex_post_magnet",
    "chassis_standoff",
    "wago", "wago3", "wago5",
})


def stl_filename(base: str) -> str:
    """Map a logical mesh base name (no extension) to its on-disk STL
    filename, appending the loud ``_DO_NOT_PRINT`` marker for the
    reference / simulation / visual meshes in ``NOT_PRINTED_MESHES``."""
    base = base[:-4] if base.endswith(".stl") else base
    if base in NOT_PRINTED_MESHES:
        return f"{base}{NOPRINT_SUFFIX}.stl"
    return f"{base}.stl"


def stl_dir_for(base: str) -> str:
    """Directory that holds the on-disk STL for a logical mesh base name."""
    base = base[:-4] if base.endswith(".stl") else base
    return REF_STL_DIR if base in NOT_PRINTED_MESHES else STL_DIR


def stl_path(base: str) -> str:
    """Absolute path to the on-disk STL for a logical mesh base name."""
    base = base[:-4] if base.endswith(".stl") else base
    return os.path.join(stl_dir_for(base), stl_filename(base))


def stl_location(base: str) -> tuple[str, str]:
    """``(directory, filename)`` for a logical mesh base name."""
    base = base[:-4] if base.endswith(".stl") else base
    return stl_dir_for(base), stl_filename(base)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _cyl(radius: float, height: float, *,
         sections: int = CYL_SECTIONS) -> trimesh.Trimesh:
    """A cylinder along +Z, centred at the origin."""
    return cylinder_mesh(radius=radius, height=height, sections=sections)


def _cyl_along(radius: float, length: float, axis: str = "x",
               *, sections: int = CYL_SECTIONS) -> trimesh.Trimesh:
    """A cylinder lying along the named axis, one end at origin, the
    other at +length on that axis."""
    m = _cyl(radius, length, sections=sections)
    m.apply_translation([0, 0, length / 2.0])
    if axis == "x":
        m.apply_transform(rotation_matrix(np.pi / 2, [0, 1, 0]))
    elif axis == "y":
        m.apply_transform(rotation_matrix(-np.pi / 2, [1, 0, 0]))
    elif axis == "z":
        pass
    else:
        raise ValueError(f"axis must be x|y|z, got {axis}")
    return m


def _box(extents, center=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    """Axis-aligned box with given extents (w, d, h), centred on *center*."""
    m = box_mesh(extents=extents)
    m.apply_translation(center)
    return m


def _union(*meshes) -> trimesh.Trimesh:
    """Boolean union with a manifold-friendly fallback."""
    parts = [m for m in meshes if m is not None]
    if len(parts) == 1:
        return parts[0]
    try:
        return trimesh.boolean.union(parts)
    except Exception:
        return trimesh.util.concatenate(parts)


def _diff(a: trimesh.Trimesh, *cuts) -> trimesh.Trimesh:
    """Boolean difference a - cuts, with a fallback to the original."""
    parts = [m for m in cuts if m is not None]
    if not parts:
        return a
    try:
        return trimesh.boolean.difference([a, *parts])
    except Exception:
        return a


def _intersection(a: trimesh.Trimesh, b: trimesh.Trimesh) -> trimesh.Trimesh:
    """Boolean intersection a & b, with a fallback to the original ``a``."""
    try:
        return trimesh.boolean.intersection([a, b])
    except Exception:
        return a


# Sub-tolerance below which collinear/coincident features are collapsed by the
# manifold simplify pass during export healing.  0.1 um is far below both the
# float32-STL quantisation floor and any real printed feature, so the simplify
# only ever removes degenerate boolean slivers, never real geometry.
MANIFOLD_SIMPLIFY_TOL = 1e-4   # mm


def _exported_mesh_defects(mesh: trimesh.Trimesh) -> dict:
    """Count the mesh defects a SLICER sees after welding a binary STL.

    Binary STL is a float32 triangle soup with no shared-vertex topology, so a
    slicer (Bambu Studio, Cura, ...) re-welds vertices by quantised position.
    We reproduce that here: quantise to float32, weld identical coordinates,
    drop any face that collapsed to <3 distinct vertices, and tally the edges.

    Returns ``{"degenerate": .., "open_edges": .., "nonmanifold_edges": ..}``
    where ``nonmanifold_edges`` is edges shared by >2 faces (what Bambu Studio
    flags) and ``open_edges`` is edges shared by exactly 1 face (holes).  A
    clean, printable, watertight 2-manifold has 0 of all three.  This is the
    check ``trimesh.is_watertight`` is NOT strong enough to make -- it tolerates
    the sub-micron boolean slivers that crack on float32 export.
    """
    v = np.asarray(mesh.vertices, dtype=np.float32)
    faces = np.asarray(mesh.faces)
    uniq: dict = {}
    idx = np.empty(len(v), dtype=np.int64)
    for i, row in enumerate(map(tuple, v)):
        idx[i] = uniq.setdefault(row, len(uniq))
    welded = idx[faces]
    edge_count: dict = {}
    degenerate = 0
    for f in welded:
        a, b, c = int(f[0]), int(f[1]), int(f[2])
        if a == b or b == c or a == c:
            degenerate += 1
            continue
        for u, w in ((a, b), (b, c), (c, a)):
            e = (u, w) if u < w else (w, u)
            edge_count[e] = edge_count.get(e, 0) + 1
    open_edges = sum(1 for n in edge_count.values() if n == 1)
    nonmanifold = sum(1 for n in edge_count.values() if n > 2)
    return {"degenerate": degenerate,
            "open_edges": open_edges,
            "nonmanifold_edges": nonmanifold}


def _is_export_clean(mesh: trimesh.Trimesh) -> bool:
    """True iff the mesh welds to a defect-free 2-manifold on float32 export."""
    d = _exported_mesh_defects(mesh)
    return (d["degenerate"] == 0 and d["open_edges"] == 0
            and d["nonmanifold_edges"] == 0)


def _simplify_via_manifold(mesh: trimesh.Trimesh,
                           tol: float = MANIFOLD_SIMPLIFY_TOL):
    """Round-trip a mesh through the manifold3d kernel and collapse sub-``tol``
    features, returning a guaranteed-2-manifold ``Trimesh`` (or ``None`` if the
    input is not a valid manifold the kernel will accept).

    The manifold boolean kernel can leave vertices that are DISTINCT in float64
    yet closer together than the float32 STL grid; ``is_watertight`` is happy
    but the slicer welds them and gets coincident/sliver faces -> non-manifold
    edges.  ``Manifold.simplify(tol)`` dissolves exactly those sub-tolerance
    slivers while preserving the topology, so the float32 export stays clean."""
    try:
        import manifold3d
        mg = manifold3d.Mesh(
            vert_properties=np.asarray(mesh.vertices, dtype=np.float32).reshape(-1, 3),
            tri_verts=np.asarray(mesh.faces, dtype=np.uint32).reshape(-1, 3),
        )
        man = manifold3d.Manifold(mg)
        if man.status() != manifold3d.Error.NoError or man.is_empty():
            return None
        man = man.simplify(tol)
        g = man.to_mesh64() if hasattr(man, "to_mesh64") else man.to_mesh()
        h = trimesh.Trimesh(
            vertices=np.asarray(g.vert_properties)[:, :3].astype(np.float64),
            faces=np.asarray(g.tri_verts).reshape(-1, 3),
            process=False,
        )
        trimesh.repair.fix_normals(h)
        return h
    except Exception:
        return None


def _heal_for_export(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Return a slicer-clean mesh for STL export (0 non-manifold edges).

    The manifold boolean kernel occasionally emits zero-area faces and
    vertices spaced finer than the float32-STL grid.  ``is_watertight``
    tolerates them in memory, but binary STL quantises to float32 and a slicer
    re-welds them into coincident/sliver faces -> NON-MANIFOLD edges that block
    slicing (Bambu Studio rejected chassis_bottom_lower with 29 such edges).

    Strategy, in order of preference:

      1. Route through ``manifold3d.Manifold.simplify`` -- this dissolves the
         sub-tolerance slivers while *guaranteeing* a 2-manifold result.  Adopt
         it only when it welds export-clean (``_is_export_clean``), stays
         watertight, and preserves volume -- so genuine assembly meshes
         (femur_link's CF-tube-in-bore gap, which is not a closed solid) are
         never silently mangled.
      2. Fall back to the legacy merge / drop-degenerate / drop-duplicate pass,
         adopted under the same guard.
      3. Otherwise return the mesh untouched.

    The volume guard is relative (<=0.5 %): a sliver-ridden raw mesh slightly
    MIS-measures its own volume, so the clean mesh legitimately differs by a
    fraction of a percent -- that is the corrected value, not lost material."""
    def _vol_ok(h: trimesh.Trimesh) -> bool:
        return abs(h.volume - mesh.volume) <= max(0.05, 5e-3 * abs(mesh.volume))

    # 0) If the raw boolean output ALREADY welds slicer-clean, return it
    # UNTOUCHED.  ``manifold3d.simplify`` can re-triangulate an already-clean
    # mesh into a geometrically self-overlapping soup that still welds to a
    # valid 2-manifold EDGE graph (so ``_is_export_clean`` / the manifold guard
    # pass) yet renders as shredded / inverted triangles in the slicer -- this
    # is exactly what mangled the old multi-boolean chassis_bottom_lower.  Only
    # run the simplify / merge heal on meshes that genuinely need repair, so a
    # clean single-prism-minus-diff part can never be silently shredded.
    if mesh.is_watertight and _is_export_clean(mesh):
        return mesh

    # 1) manifold3d simplify -- guaranteed-manifold, removes sub-tol slivers.
    h = _simplify_via_manifold(mesh)
    if h is not None and h.is_watertight and _vol_ok(h) and _is_export_clean(h):
        return h

    # 2) legacy merge-based heal.
    try:
        h = mesh.copy()
        h.merge_vertices(digits_vertex=3)
        h.update_faces(h.nondegenerate_faces())
        h.update_faces(h.unique_faces())
        h.remove_unreferenced_vertices()
        trimesh.repair.fix_normals(h)
        if h.is_watertight and _vol_ok(h) and _is_export_clean(h):
            return h
    except Exception:
        pass

    # 3) give up -- export the mesh as-is (the verifier guard will flag it).
    return mesh


def _save(mesh: trimesh.Trimesh, name: str) -> str:
    """Write ``mesh`` to stl_prototype/ or stl_reference/ based on name.

    ``name`` is usually already ``stl_filename(base)`` (may include
    ``_DO_NOT_PRINT``).  Reference / bought-part visuals go under
    ``stl_reference/``; everything else under ``stl_prototype/``.
    """
    if name.endswith(".stl"):
        fname = name
        base = name[:-4].replace(NOPRINT_SUFFIX, "")
    else:
        base = name
        fname = stl_filename(base)
    out_dir = stl_dir_for(base)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, fname)
    mesh = _heal_for_export(mesh)
    mesh.export(path)
    n_faces = len(mesh.faces)
    extents = mesh.extents
    rel = os.path.relpath(path, os.path.dirname(STL_DIR))
    print(f"  wrote {rel:44s}"
          f"  {n_faces:>6d} faces"
          f"  envelope {extents[0]:5.1f} x {extents[1]:5.1f} x {extents[2]:5.1f} mm")
    return path


# ---------------------------------------------------------------------------
# Servo-mounting primitives
# ---------------------------------------------------------------------------

def _servo_cradle_insert_pockets(
    shelf_top_z: float = None,
    heatset_pocket_z_top_override: float = None,
    selftap_pocket_z_top_override: float = None,
) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """Return ``(bosses_union, pockets_union)`` for the 4 cradle bolt
    sites in a cradle shelf (MIXED MODE -- Design E, May 2026).

    Local frame: matches ``_servo_well_solid`` (origin at the body's
    bottom face centre; +X = body long axis = tab-span direction;
    +Y = body short axis; +Z = output shaft direction).

    Two distinct schemes are emitted, one per X column:

      * ``sx == -1`` (the 2 -X bolts per cradle): HEAT-SET INSERT scheme.

        - Boss: a vertical Phi ``CRADLE_BOSS_OD`` cylinder centred on
          ``(sx * SERVO_MOUNT_HOLE_X_OFFSET, sy * SERVO_MOUNT_HOLE_Y_OFFSET)``,
          spanning well-z in ``[shelf_top_z - CRADLE_BOSS_HEIGHT_MM,
          shelf_top_z]``.  Unioned into the cradle so the printed
          material around the insert pocket is at least
          ``CRADLE_BOSS_MIN_WALL_MM`` thick radially.
        - Pocket: a vertical Phi ``INSERT_M3_PILOT_OD`` cylinder
          centred on the same (x, y) axis, spanning well-z in
          ``[shelf_top_z - INSERT_M3_PILOT_DEPTH, shelf_top_z + 2.0]``
          BY DEFAULT.  The pocket extends 2 mm ABOVE the shelf top so
          the boolean cut leaves a clean rim and 1 mm BELOW the
          insert length ``INSERT_M3_INSERT_LENGTH`` so debris from
          the heat-set installation has somewhere to go.  The pocket
          is subtracted from the cradle.

          The ``heatset_pocket_z_top_override`` arg lifts the
          pocket's TOP cut plane above the default shelf_top_z +
          2.0 mm.  Use this from the ``_chassis_yaw_cradle_solid``
          call site -- and ONLY there -- to drill the pocket UP
          through the cradle's non-load-bearing 5-mm shroud that
          caps the shelf between cradle-z = +6 (shelf top) and
          cradle-z = +11 (cradle rim, = CRADLE_BOSS_H_MM).  Without
          the override the 12 chassis_bottom heat-set pockets are
          fully CAPPED from above by ~3 mm of shroud material, and
          the operator has no opening from which to press the
          M3 brass insert (McMaster 94459A130) down into the
          pocket with a soldering iron.  See the ``CRADLE_BOSS_H_MM``
          constants block for the shroud's history (it used to wrap
          the gear housing at BOSS_H = 19 mm; Path-A dropped it to
          11 mm but left a 5 mm dust-shroud strip above the shelf
          which the heat-set pocket geometry never adapted to).
          User feedback (May 24 2026): "the bottom chassis really
          doesn't seem like it has the inside holes for the heat
          set inserts to hold the servo -- maybe they are covered
          so they are impossible to see? Either way I need a hole
          on the top to put the heat set insert into".  Confirmed
          via Z-sweep probe at the bolt PCD before the fix.

          The legacy ``_servo_well_solid`` cradles (coxa_bracket,
          coxa_link, femur_link, tibia_link) have NO shroud above
          the shelf -- their rims sit at well-z = WELL_RIM_Z which
          equals the shelf top -- so the default +2.0 mm overshoot
          suffices there and they pass ``shelf_top_z`` only.

      * ``sx == +1`` (the 2 +X bolts per cradle): SELF-TAP PILOT scheme.

        - No boss is emitted -- the Phi ``CRADLE_BOSS_OD`` = 8 mm boss
          footprint physically intersects the +X WIRE_CHANNEL that the
          servo's molded wire boot (Phi ``WIRE_BOOT_W`` = 7 mm in Y)
          has to pass through during insertion, so the heat-set boss
          cannot coexist with the channel on this column.
        - A bare Phi ``INSERT_M3_SELFTAP_PILOT_OD`` = 2.5 mm pilot is
          sunk straight into the existing well-wall material below the
          servo tab, spanning well-z in
          ``[shelf_top_z - INSERT_M3_SELFTAP_PILOT_DEPTH,
            shelf_top_z + 2.0]``.  The M3 x 8 SHCS self-taps into this
          pilot -- the surrounding plastic is the thread engagement
          medium, NOT a brass insert.

        See the ``INSERT_M3_SELFTAP_*`` constant block near the top of
        this file for the design rationale (mixed-mode Design E,
        May 2026: restored wire channel + +X self-tap fallback) and
        ``check_servo_insertion_path`` in ``_verify_prototype.py`` for
        the regression probe that catches the heat-set-boss-vs-boot
        collision if it ever recurs.

        The ``selftap_pocket_z_top_override`` arg behaves
        symmetrically to ``heatset_pocket_z_top_override`` above: it
        lifts the +X self-tap pocket's TOP cut plane above the
        default shelf_top_z + 2.0 mm.  The
        ``_chassis_yaw_cradle_solid`` call site passes this so the
        2 +X self-tap columns also open through the shroud cap,
        keeping all 4 bolt sites consistent (clean Phi-2.5 holes
        from rim to shelf top) and giving the bolt head 3 mm of
        head-room above the seated servo ear during installation.
        See ``heatset_pocket_z_top_override`` above for the full
        rationale -- the shroud above the +X column is identical
        to the shroud above the -X column.

    ``shelf_top_z`` defaults to ``WELL_RIM_Z`` (the nominal cradle rim
    height in well-local).  ``make_coxa_bracket`` passes a lower value
    when the bracket's drop-in slot has eaten the rim down by a known
    amount so its bolt-site bosses + pockets stay correctly aligned
    with the bracket's effective shelf top.

    ``heatset_pocket_z_top_override`` / ``selftap_pocket_z_top_override``
    both default to ``None`` which means "use ``shelf_top_z + 2.0``"
    (the legacy +2 mm clean-rim overshoot).  Pass a numeric value
    only from cradle call sites where the shelf is capped from above
    by a non-load-bearing shroud that has to be punched through to
    give the operator visual / mechanical access to the pocket --
    i.e., the chassis_bottom integrated yaw cradle and only that
    cradle.

    Use as::

        bosses, pockets = _servo_cradle_insert_pockets()
        body = _union(body, bosses)
        body = _diff(body, pockets)

    Caller responsibility: apply the same well-to-cradle ``R`` /
    translation chain to BOTH meshes so the boss/pocket pair lands on
    the bolt PCD in the cradle's local frame.

    History: this helper replaced ``_servo_cradle_pilot_holes`` (which
    returned only the 4 Phi 2.5 mm self-tap pilot cylinders) in the
    May 2026 heat-set switch (commit f03d59b).  Design E (May 2026)
    REVERTED the 2 +X sites to the Phi 2.5 mm self-tap scheme because
    the heat-set boss + restored wire channel cannot coexist on the
    +X column; the 2 -X sites kept the heat-set scheme intact.
    """
    if shelf_top_z is None:
        shelf_top_z = WELL_RIM_Z

    # STS3215 front-face mount (June 2026): NO bosses / inserts / self-tap
    # pilots.  The plate carries the central output-coupling clearance
    # bore plus 4x M2.5 countersunk case-screw holes on the 9.8 mm square
    # around the output axis, drilled through the plate from above (+Z).
    # The screws thread into the servo's own metal case, so the printed
    # part only needs clearance + a flat-head counterbore.  ``shelf_top_z``
    # is the plate UNDERSIDE plane (= front face = WELL_RIM_Z); the holes
    # run from above the plate top down through the seat.
    pocket_parts: list[trimesh.Trimesh] = []

    plate_top = shelf_top_z + WELL_PLATE_T
    z_top = plate_top + 1.0          # overshoot above plate top
    z_bot = shelf_top_z - 2.0        # through the seat into the body cavity
    h = z_top - z_bot
    z_cen = 0.5 * (z_top + z_bot)

    # Central output-coupling clearance bore.
    bore = _cyl(SERVO_OUTPUT_BORE_OD / 2.0 + 0.4, h)
    bore.apply_translation([SERVO_OUTPUT_X, 0.0, z_cen])
    pocket_parts.append(bore)

    # 4x M2.5 clearance holes + flat-head countersink at the plate top.
    cs_r = SERVO_MOUNT_HEAD_OD / 2.0
    for (hx, hy) in servo_mount_hole_centres():
        hole = _cyl(SERVO_MOUNT_SCREW_OD / 2.0, h)
        hole.apply_translation([hx, hy, z_cen])
        pocket_parts.append(hole)
        # 90-deg flat-head countersink approximated by a short straight
        # counterbore at the plate top (printable + screwdriver clear).
        cbore = _cyl(cs_r, WELL_PLATE_T + 1.0)
        cbore.apply_translation([hx, hy, plate_top - (WELL_PLATE_T + 1.0) / 2.0 + 0.5])
        pocket_parts.append(cbore)

    # ``bosses`` is now empty (front-face mount has no printed bosses);
    # an empty mesh keeps the legacy ``.copy()`` / ``_union`` / ``_diff``
    # call sites (e.g. the chassis yaw cradle) working as harmless no-ops.
    return trimesh.Trimesh(), _union(*pocket_parts)


def _servo_well_solid(*, remove_floor: bool = False,
                      end_face_bolts: bool = True) -> trimesh.Trimesh:
    """STS3215 front-face mount cradle, returned as one watertight mesh
    in the well's local frame.

    ``end_face_bolts=False`` skips the 4x M2.5 end-face bolt holes +
    counterbores through the -X wall (Jul 2026 one-piece femur: the fused
    Phi 14 spar covers the knee cradle's -X wall from outside, so those
    screws are impossible to install there and the empty holes only
    weakened the spar-to-wall junction; the knee servo is retained by the
    clamp cap + retaining lip alone, like the yaw cradle).

    Local frame (matches `_servo_envelope` *body* axes):
        Origin: centre of the body's BACK face.
        +X = body long axis (output-offset direction).
        +Y = body short axis (depth).
        +Z = output-shaft direction.

    June 2026 STS3215 redesign
    --------------------------
    The DS3225 dropped INTO an open-topped bucket and hung from its side
    TABS on a shelf.  The STS3215 has no tabs; it bolts by 4x M2.5
    through a PLATE on its FRONT (output) face.  So the cradle is now an
    inverted bucket:

        - 4 side WALLS span z in [0, WELL_RIM_Z] (= body height) and
          locate the body laterally with WELL_BODY_CL clearance.
        - A front mount PLATE caps the top, z in [WELL_RIM_Z,
          WELL_RIM_Z + WELL_PLATE_T].  The body's front face seats flush
          against the plate underside at z = WELL_RIM_Z.
        - The plate carries the central output-coupling clearance bore
          and the 4x M2.5 (countersunk) case-screw holes on the 9.8 mm
          square around the output axis (see ``servo_mount_hole_centres``).
        - The cradle is OPEN at the BACK (-Z): the servo is slid in from
          behind and pushed forward against the plate, then the 4 M2.5
          drive in from the front (output) side.

    ``remove_floor`` is accepted for backward compatibility (the femur's
    supports-free print) but is a no-op now.

    Jun 2026 clamshell redesign (see the ``CLAMP_*`` constants block): the
    full front-face mount PLATE is gone.  The cradle is now open on the +Y
    long face (lateral drop-in) and open at the centre of the output face
    (Phi ``HORN_CLEAR_OPENING_OD`` bore) so the disc horn seats + spins.
    Since Sep 2026 that bore continues into the +Y mouth as a straight
    service slot: after removing the two clamp bolts, the servo, fitted horn
    and clamp cap lift out together.  The slot deletes one small front-case
    capture site, leaving three.  The body is captured by the -Y + corner
    top LIP and held by the bolt-on ``make_servo_clamp_cap`` on the +Y face.
    """
    outer = _box((WELL_W, WELL_D, WELL_H),
                 center=(0, 0, WELL_H / 2.0))
    cuts: list[trimesh.Trimesh] = []

    # Body cavity, open at the BACK (-Z).  The top is lifted
    # WELL_LIP_SLIDE_CL above the front-face plane so the body slides in
    # laterally (in -Y) without binding under the retaining lip.
    cav_z_bot = -1.0
    cav_z_top = WELL_RIM_Z + WELL_LIP_SLIDE_CL
    # Inside-X tightened WELL_INSIDE_X_TIGHTEN total (snug-fit; see CLAMP block).
    cav_w = SERVO_BODY_W + 2 * WELL_BODY_CL - WELL_INSIDE_X_TIGHTEN
    cav_d = SERVO_BODY_D + 2 * WELL_BODY_CL
    cuts.append(_box((cav_w, cav_d, cav_z_top - cav_z_bot),
                     center=(0, 0, 0.5 * (cav_z_top + cav_z_bot))))

    # OPEN the +Y wall + the +Y strip of the top lip for the lateral
    # drop-in (the clamp cap closes this face).  The +/-X end walls and
    # the -Y wall survive.
    open_y0 = cav_d / 2.0 - 0.01
    open_y1 = WELL_D / 2.0 + 1.0
    cuts.append(_box((cav_w, open_y1 - open_y0, WELL_H + 2.0),
                     center=(0.0, 0.5 * (open_y0 + open_y1), WELL_H / 2.0)))

    # Central output-face opening: a Phi HORN_CLEAR_OPENING_OD bore through
    # the top lip (centred on the output axis at +SERVO_OUTPUT_X) so the
    # dia-20 disc horn seats on the spline and spins in free air.
    horn = _cyl(HORN_CLEAR_OPENING_OD / 2.0, WELL_PLATE_T * 4.0)
    horn.apply_translation([SERVO_OUTPUT_X, 0.0, WELL_RIM_Z])
    cuts.append(horn)

    # Horn service slot (Sep 2026, user): open the circular horn bore into
    # the +Y clamp mouth.  This removes the rounded bridge that trapped a
    # fitted horn behind the fixed output lip, making the +Y extraction path
    # monotone for the servo + horn + attached clamp cap.  A same-width box
    # from the horn centreline outward is exactly the straight part of the
    # swept capsule; the circular cut above supplies its rounded starting end.
    service_y0 = 0.0
    service_y1 = WELL_D / 2.0 + 1.0
    service_slot = _box(
        (HORN_SERVICE_SLOT_W, service_y1 - service_y0,
         WELL_PLATE_T * 4.0),
        center=(SERVO_OUTPUT_X, 0.5 * (service_y0 + service_y1),
                WELL_RIM_Z))
    cuts.append(service_slot)

    # FRONT-face case-screw capture (Aug 2026; Sep 2026 service refit): 3x
    # small self-tap screws
    # drive -Z through the top lip into the servo's molded front-shell
    # pilots (``servo_holder_front_case_hole_centres`` -- the three surviving
    # sites from the FRONT twin of the
    # yaw saddle's rear capture; one pair matches the rear pattern, one
    # pair is 3.8 mm offset).  The three retained sites clear the Phi 24 horn
    # opening/service slot and sit under the surviving lip.
    # Heads are counterbored FLUSH: the moving yoke's arm sweeps only
    # 3 mm above the lip top and its horn pad (r 10.5) spins 0.4 mm
    # inside the nearest head edge, so a proud head would foul.  The
    # screw stands off the recessed shell deck (SERVO_FRONT_CASE_DECK_DROP
    # below the front face) and its tension pulls the body's front face
    # up against the lip underside -- positive retention on top of the
    # clamp cap, which the knee cradle (no end-face bolts) lacked.
    # The fourth physical case site at (+4.2, +10.25) lies in the horn service
    # slot and is intentionally omitted; see
    # ``servo_holder_front_case_hole_centres``.
    for (fx, fy) in servo_holder_front_case_hole_centres():
        bore = _cyl(FRONT_CASE_SCREW_OD / 2.0, WELL_PLATE_T + 4.0)
        bore.apply_translation([fx, fy, WELL_RIM_Z + WELL_PLATE_T / 2.0])
        cuts.append(bore)
        cbore = _cyl(FRONT_CASE_CBORE_OD / 2.0, FRONT_CASE_CBORE_DEPTH + 1.0)
        cbore.apply_translation(
            [fx, fy, WELL_H - (FRONT_CASE_CBORE_DEPTH - 1.0) / 2.0])
        cuts.append(cbore)

    # Clamp-cap bolt pilots: M3 self-tap into the +Y END faces of the +/-X
    # walls (axis along Y).  The cap's 2 bolts thread in here and pull the
    # cap -Y to clamp the body against the -Y wall.  The (x, z) centres come
    # from ``servo_clamp_bolt_centres`` -- the SAME source the clamp cap and
    # the fastener registry read -- so the pilots stay coaxial with the cap.
    pilot_y = WELL_D / 2.0 - CLAMP_BOLT_PILOT_DEPTH / 2.0 + 1.0
    for (bx, bz) in servo_clamp_bolt_centres():
        pilot = _cyl(CLAMP_BOLT_PILOT_OD / 2.0, CLAMP_BOLT_PILOT_DEPTH + 2.0)
        pilot.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))   # axis -> Y
        pilot.apply_translation([bx, pilot_y, bz])
        cuts.append(pilot)

    # POSITIVE body retention: 4x M2.5 bolts through the -X (spine/back)
    # end WALL into the servo's real END-face 10x10 square
    # (``servo_end_face_bolt_centres``).  Screws pass +X through the wall
    # and thread into the servo's -X end-face M2.5 case holes, bolting the
    # body to the cradle (the clamp cap + lip still grip the +Y/-Y faces).
    # The -X end face is the back (spine) face, away from the +Y lateral
    # mouth and the disc horn, so it stays accessible.  The bus harness
    # window (y in [-3.5, 3.5]) does not reach these y = +/-5 holes.  The
    # head is recessed in a counterbore so a stock M2.5 x 8 still reaches
    # the case through the thick wall.  (Skipped when ``end_face_bolts``
    # is False -- one-piece femur knee cradle, see docstring.)
    if end_face_bolts:
        body_face_x = -SERVO_BODY_W / 2.0           # servo -X end face plane
        wall_outer_x = -WELL_W / 2.0                # -X wall outer face
        head_plane_x = body_face_x - SERVO_BODY_BOLT_STANDOFF  # counterbore floor
        cl_outer = wall_outer_x - 1.0
        cl_inner = body_face_x + SERVO_MOUNT_THREAD_DEPTH       # into the case
        for (by, bz) in servo_end_face_bolt_centres():
            hole = _cyl(SERVO_BODY_BOLT_OD / 2.0, cl_inner - cl_outer)
            hole.apply_transform(rotation_matrix(np.pi / 2.0, [0, 1, 0]))   # axis -> X
            hole.apply_translation([0.5 * (cl_outer + cl_inner), by, bz])
            cuts.append(hole)
            cbore = _cyl(SERVO_BODY_BOLT_HEAD_OD / 2.0, head_plane_x - cl_outer)
            cbore.apply_transform(rotation_matrix(np.pi / 2.0, [0, 1, 0]))
            cbore.apply_translation([0.5 * (cl_outer + head_plane_x), by, bz])
            cuts.append(cbore)

    return _diff(outer, *cuts)


def _wire_exit_l_corridor() -> trimesh.Trimesh:
    """Cutting volume for the L-shaped wire-EXIT corridor at the
    bottom-OUTBOARD corner of a servo well.

    Local frame: same as ``_servo_well_solid`` / ``_servo_envelope``.

    This is HALF of the legacy ``_wire_exit_slot()`` mesh.  The other
    half -- the inner +X wall channel that clears the molded wire
    boot during insertion -- is returned by ``_boot_clearance_channel()``.

    Background
    ----------
    DS3225-class hobby servos route their 3-wire harness out of a
    RECTANGULAR MOLDED BOOT on the body's **+X SHORT face** -- i.e. the
    SAME X-end as the output gear (which sits at +SERVO_OUTPUT_X).  The
    boot dimensions (WIRE_BOOT_* near the top of this file) are baked
    into ``_servo_envelope`` so this is visually unambiguous in renders.

    Function split rationale (May 2026, chassis_bottom yaw cradle
    redesign, commit 5.5/9)
    ----------------------------------------------------------------
    The legacy ``_wire_exit_slot()`` returned a single mesh that
    combined TWO independent functions:

      1. an L-shaped corridor through the EXIT face of the cradle that
         lets the wire bundle leave the cradle into open air, AND
      2. a vertical channel cut INTO the wall facing the wire boot
         (always +X by hardware) that lets the boot descend through
         the wall during servo insertion.

    These were bundled because for the legacy cradles (coxa_bracket,
    coxa_link, femur_link) BOTH ended up on the +X face and so could
    share a single boolean cut.  The chassis_bottom integrated yaw
    cradle moved the EXIT corridor to the radially-INWARD (-X) face
    (so the harness routes to the per-leg drop slot at chassis-frame
    +46.8, +27, +2) but the BOOT is still hardware-anchored to +X.
    Without separating the two functions, mirroring the legacy
    bundle to -X moved both pieces and left the +X wall solid in
    the boot's swept volume; real DS3225 servos could not be seated.

    Splitting the helper into two functions lets each cradle's
    builder mirror / translate / rotate the two pieces independently.

    The L-corridor must:

      * give the wire bundle a clean exit path through the chosen
        EXIT face (defaults to +X for legacy cradles; mirrored to -X
        by the chassis_bottom yaw cradle), AND
      * give the bundle a downward drop path through the well floor
        at the corridor's inboard end (so a harness that bends 90 deg
        at the boot can drop out the bottom of the cradle without
        sharing the boot's swept volume).

    Use as::

        well = _servo_well_solid()
        corridor = _wire_exit_l_corridor()
        # apply the same R / mirror / translation to corridor as to the well
        body = _diff(body, corridor)

    PRE-2026 versions of this file put the slot on the **-X side** of
    the well (opposite the output gear) on the incorrect assumption
    that the wires emerged from the back of the case.  That left the
    boot punched against solid +X wall material and the slot punched
    against solid +X cavity material -- the user could not seat the
    servo body fully in the cradle without bending or shearing the
    boot.  Fixed by mirroring the slot to the +X face that matches
    the boot.
    """
    # X span: from the body's +X face (inboard end of the slot, slightly
    # inside the cavity so the slot opens cleanly into the cavity wall)
    # out past the well's outer +X face by WIRE_SLOT_X_PAST_WALL so the
    # bundle exits into free air.
    slot_x_min = +SERVO_BODY_W / 2.0 - WIRE_SLOT_X_INBOARD
    slot_x_max = +WELL_W / 2.0 + WIRE_SLOT_X_PAST_WALL
    slot_x_extent = slot_x_max - slot_x_min
    slot_x_centre = 0.5 * (slot_x_min + slot_x_max)

    slot_y_extent = WIRE_SLOT_W                 # centred on y = 0

    # Z span: must reach BELOW the well floor (to give the downward escape
    # path) AND reach UP at least past the top of the wire boot
    # (WIRE_BOOT_Z_BASE + WIRE_BOOT_H) plus a comfortable margin -- we use
    # WIRE_SLOT_DEPTH which already includes that headroom for a molded
    # boot + bent harness.
    slot_z_bottom = -WELL_FLOOR_T - WIRE_SLOT_Z_BELOW_FLOOR
    slot_z_top    = WIRE_SLOT_DEPTH
    slot_z_extent = slot_z_top - slot_z_bottom
    slot_z_centre = 0.5 * (slot_z_bottom + slot_z_top)

    return _box((slot_x_extent, slot_y_extent, slot_z_extent),
                center=(slot_x_centre, 0.0, slot_z_centre))


def _boot_clearance_channel() -> trimesh.Trimesh:
    """Cutting volume for the vertical channel on the INSIDE surface of
    the +X wall that lets the servo's molded wire boot descend through
    the wall during INSERTION.

    Local frame: same as ``_servo_well_solid`` / ``_servo_envelope``.

    This is the SECOND half of the legacy ``_wire_exit_slot()`` mesh.
    The first half -- the L-shaped wire-EXIT corridor on the chosen
    exit face -- is returned by ``_wire_exit_l_corridor()``.  See
    that function's docstring for the May 2026 chassis_bottom yaw
    cradle redesign rationale that motivated the split.

    The channel cuts INTO the +X wall material from the cavity face
    (so it merges seamlessly with the body cavity) out to
    ``WIRE_CHANNEL_DEPTH`` into the wall.  The +X anchor is FIXED by
    hardware (the DS3225 / MG996R / DS3218-class boot always points
    +X in well-local coords) and so this helper does NOT get mirrored
    along with the L-corridor when the EXIT face changes -- each
    cradle's builder applies whatever transforms it needs to the
    L-corridor INDEPENDENTLY of this channel.

    Channel-vs-boss reroute history (May 2026)
    -------------------------------------------
    The May 2026 heat-set switch (commit f03d59b) shortened the
    channel's top from ``WELL_RIM_Z + WIRE_CHANNEL_TOP_OVER_RIM =
    +29.5 mm`` down to ``(WELL_RIM_Z - CRADLE_BOSS_HEIGHT_MM) -
    0.5 = +16.5 mm`` so the channel cut would not graze the new
    Phi CRADLE_BOSS_OD = 8 mm heat-set bosses at
    (+SERVO_TAB_HOLE_PCD/2, +/-SERVO_TAB_HOLE_PCD_Y/2) =
    (+24.75, +/-5).  That worked for the SEATED boot
    (z ~= [4.1, 8.0] in body-local, comfortably below the new
    16.5 mm cap) but BROKE the INSERTION path: the boot has to
    slide DOWN through the +X wall from z = WELL_RIM_Z + boot_h
    (above the rim) all the way to its seated z, and the 10.5 mm
    of +X wall material between z = 16.5 and z = WELL_RIM_Z =
    27.25 stopped the boot from descending past the rim.  Real
    DS3225 servos could not be seated in their printed cradles.

    Fix (Design E, May 2026 mixed-mode): restore the channel cap
    to its pre-f03d59b value (``WELL_RIM_Z +
    WIRE_CHANNEL_TOP_OVER_RIM``) so the boot has the full
    insertion path AND switch the 2 +X heat-set sites per cradle
    to Phi 2.5 mm M3 SELF-TAP pilots (no boss), since the
    restored channel would have eaten the inboard half of each
    +X boss.  Detailed rationale + alternative options
    considered live in the INSERT_M3_SELFTAP_* constant block
    near the top of this file; the verifier's
    ``check_servo_insertion_path`` probe catches this regression
    if it ever ships again.

    The 2 -X bolts per cradle KEEP their heat-set inserts.  For
    the LEGACY cradles (coxa_bracket / coxa_link / femur_link)
    the wire-exit corridor stays on +X (no mirror) so the -X
    column has no channel cut and the -X bosses keep their full
    8-azimuth radial wall.  The NEW chassis_bottom integrated
    yaw cradle mirrors the L-corridor to -X (so wires route
    radially inward toward the per-leg harness drop slot), which
    puts the L-corridor's lateral leg through the -X column;
    the -X heat-set bosses there get the channel-side wall
    reduction documented in the ``CRADLE_BOSS_*`` constants
    block: 25 percent (8/32) of the 1.5-mm-min radial-wall
    azimuths get punched out by the mirrored corridor.  See that
    constants block + the ``check_cradle_insert_pockets``
    preamble for the engineering judgement that retaining 75
    percent of the boss circumference + the plate-bonded boss
    bottom leaves >> 5x the actual servo-tab clamping load on
    each insert.

    ``_servo_cradle_insert_pockets`` builds heat-set bosses ONLY
    on the -X sites; the +X sites get a bare Phi 2.5 mm pilot
    column in the wall material that the boot's swept volume
    passes cleanly through.
    """
    ch_x_min = +SERVO_BODY_W / 2.0 - WIRE_SLOT_X_INBOARD
    ch_x_max = +SERVO_BODY_W / 2.0 + WELL_BODY_CL + WIRE_CHANNEL_DEPTH
    ch_x_extent = ch_x_max - ch_x_min
    ch_x_centre = 0.5 * (ch_x_max + ch_x_min)

    ch_z_bottom = 0.0
    # Channel top sits ``WIRE_CHANNEL_TOP_OVER_RIM`` above the well
    # rim so the boot's swept volume during insertion clears the
    # +X wall all the way from the seated position
    # (boot bottom ~ z = WIRE_BOOT_Z_BASE + WELL_TAB_FLOAT = 5.6 mm)
    # up past the rim (z = WELL_RIM_Z = 27.25 mm) and a comfortable
    # exit margin above it.  The +X bolts in this z range now use
    # SELF-TAP pilots instead of heat-set bosses, so the channel
    # cut does not collide with any structural boss material on
    # this column.
    ch_z_top = WELL_RIM_Z + WIRE_CHANNEL_TOP_OVER_RIM
    ch_z_extent = ch_z_top - ch_z_bottom
    ch_z_centre = 0.5 * (ch_z_bottom + ch_z_top)

    return _box((ch_x_extent, WIRE_SLOT_W, ch_z_extent),
                center=(ch_x_centre, 0.0, ch_z_centre))


def _wire_exit_slot() -> trimesh.Trimesh:
    """Backwards-compatible wrapper that returns the union of the new
    ``_wire_exit_l_corridor()`` + ``_boot_clearance_channel()`` helpers.

    Kept for any external scripts (``arm/arm.py``, future test
    utilities) that still call the legacy single-mesh form.  The four
    in-this-file call sites (``make_coxa_bracket``, ``make_coxa_link``,
    ``make_femur_link``, ``_chassis_yaw_cradle_solid``) now use the two
    split helpers directly so the boot channel can be anchored to +X
    independently of where the wire-exit corridor terminates.  See the
    ``_wire_exit_l_corridor`` docstring for the split rationale (May
    2026 chassis_bottom yaw cradle redesign, commit 5.5/9).

    Byte-equivalent to the pre-split implementation: the mesh produced
    by ``_union(_wire_exit_l_corridor(), _boot_clearance_channel())``
    has identical bounds, faces and volume to the legacy single-mesh
    version (verified with ``mesh.contains`` probes on the bracket,
    coxa_link and femur_link STLs).
    """
    return _union(_wire_exit_l_corridor(), _boot_clearance_channel())


def _cable_zip_post(
    z_centre: float = CABLE_POST_Z_CENTRE,
) -> trimesh.Trimesh:
    """Return the printed-in zip-tie strain-relief post for one servo
    cradle.

    Local frame: same as ``_servo_well_solid`` / ``_wire_exit_slot``.
    The post is a small rectangular nub that protrudes from the well's
    OUTER +X face just past the wire-exit slot, so the assembler can
    loop a 2-3 mm zip-tie around the post AND the 3-wire harness as a
    printed-in strain relief.  Used by ``make_coxa_bracket`` (yaw
    cradle), ``make_coxa_link`` (hip-pitch cradle) and
    ``make_femur_link`` (knee cradle) -- each calls this helper, applies
    the SAME transform that the matching ``_wire_exit_slot()`` cut
    received, and UNIONs the result into the link's body.  See the
    ``CABLE_POST_*`` constants block near the top of this file for the
    geometry rationale, keep-out audit and printability argument.

    The post is anchored to the wall on its -X face (well_x = WELL_W/2)
    and extends OUTWARD by ``CABLE_POST_X``; the centre Y is offset to
    the +Y side of the slot by ``CABLE_POST_Y_OFFSET`` so the post does
    not sit IN the wire-exit slot's Y span; centre Z is ``z_centre``
    (well-local Z above the cavity floor; defaults to the shared
    ``CABLE_POST_Z_CENTRE`` for coxa_link / femur_link).  All three
    NOMINAL numeric values live in the constants block; do not hard-
    code here.

    The ``z_centre`` parameter exists so ``make_coxa_bracket`` can pass
    ``BRACKET_CABLE_POST_Z_CENTRE = 20.25`` (instead of the default +6)
    to LIFT the post above the bracket's ``BRACKET_WELL_TRIM_Z = -15``
    cut.  Without that override the default +6 post lands at bracket-
    z in [-25.25, -17.25], entirely below the trim plane, and the
    trim's boolean subtraction silently destroys the post material.
    See the ``BRACKET_CABLE_POST_Z_CENTRE`` docstring for the full
    derivation.
    """
    px_min = WELL_W / 2.0
    px_max = px_min + CABLE_POST_X
    py_centre = CABLE_POST_Y_OFFSET
    pz_centre = z_centre
    return _box(
        (CABLE_POST_X, CABLE_POST_Y, CABLE_POST_Z),
        center=(0.5 * (px_min + px_max), py_centre, pz_centre),
    )


def _chassis_yaw_cradle_solid() -> trimesh.Trimesh:
    """Yaw-servo cradle integrated into ``chassis_bottom``.

    STS3215 front-face mount (Jun 2026 refit)
    -----------------------------------------
    Returns the cradle for ONE leg, in cradle-local frame:

      Origin: at the YAW AXIS (= chassis hex edge midpoint).
      +X = outboard radial (away from chassis centre).
      +Y = tangential (along the chassis edge).
      +Z = up.  cradle-z = 0 coincides with chassis_bottom's TOP face.

    The DS3225 tab-shelf cradle (servo dropped into an open-top bucket,
    held by its side ears on a shelf, heat-set inserts + a molded wire
    boot) is replaced by a TOWER that carries an elevated FRONT-FACE
    mount plate.  The STS3215 has no tabs; it bolts via 4x M2.5 through
    a plate on its FRONT (output) face.  The yaw servo's output points
    UP; the servo is inserted from BELOW (open bottom), pushed up until
    its front face seats against the plate underside, then fixed with
    the 4 M2.5 case screws driven down through the plate.  The output
    coupling pokes up through the plate's central bore and the dia-20
    disc horn seats on the plate TOP; the coxa_link mounts on the horn
    at ``CHASSIS_YAW_OUTPUT_Z`` (pinned to the DS3225 value so the leg-
    mount height / kinematics are unchanged).

    Used by ``make_chassis_bottom`` which iterates the 6 leg azimuths
    via ``_leg_chassis_frames`` and unions one cradle per leg into the
    hex plate.  The shell footprint is 1 mm proud of the plate's
    ``WELL_W+2 x WELL_D+2`` body cutout on each side so the boolean
    union bonds the tower walls into the plate around the cutout.

    Structure (cradle-local), see the inline Z-layout block below:
      * Outer shell tower: bond-strip footprint, spanning from the body
        back face (below the plate, hanging into the inter-plate gap)
        up to the front mount plate top.
      * Body cavity: open at the BOTTOM in this shared solid; the servo
        hangs through the merged chassis_bottom floor's body cutout and is
        captured from below by the bolt-on ``yaw_servo_retainer`` stirrup
        (which anchors into 2 M3 pilots in the floor) so it cannot drop out.
      * Front mount plate: central output-coupling bore + 4x M2.5
        countersunk case-screw holes + a disc-horn collar recess.
      * Inboard (-X) wire-exit window for the serial-bus harness.
    """
    body_centre_x = -SERVO_OUTPUT_X

    # ---- STS3215 front-face mount tower (Jun 2026 refit) -------------
    # Z layout (cradle-local; cradle-z = 0 = chassis_bottom TOP face):
    #   horn top (yaw output) = CHASSIS_YAW_OUTPUT_Z - CHASSIS_PLATE_T/2
    #   plate top             = horn top - HORN_STACK_H
    #   front face (= plate underside, the servo's output face seats
    #                here)    = plate top - WELL_PLATE_T
    #   body back face        = front face - SERVO_BODY_H  (hangs below)
    # CHASSIS_YAW_OUTPUT_Z is pinned to the DS3225 value so the leg-mount
    # height / kinematics are unchanged across the refit.
    out_z        = CHASSIS_YAW_OUTPUT_Z - CHASSIS_PLATE_T / 2.0
    plate_top_z  = out_z - HORN_STACK_H
    front_face_z = plate_top_z - WELL_PLATE_T
    body_back_z  = front_face_z - SERVO_BODY_H

    # ---- Outer shell: bond-strip footprint (1 mm proud of the
    # WELL_W+2 x WELL_D+2 hex-plate body cutout on each side) so the
    # boolean union with chassis_bottom fuses the tower walls into the
    # plate around the cutout perimeter.  Spans from the body back face
    # (below the plate) up to the front mount plate top.
    outer_w = WELL_W + 2.0 + 2.0 * CRADLE_BOND_STRIP_MM
    outer_d = WELL_D + 2.0 + 2.0 * CRADLE_BOND_STRIP_MM
    outer = _box((outer_w, outer_d, plate_top_z - body_back_z),
                 center=(body_centre_x, 0.0,
                         0.5 * (body_back_z + plate_top_z)))

    # ---- Body cavity: OPEN at the BOTTOM in this shared cradle solid.  The
    # merged chassis_bottom floor (``_chassis_bottom_floor_solid``) keeps a
    # body cutout here, so the servo body hangs through and is captured from
    # below by the bolt-on ``yaw_servo_retainer`` stirrup -- this shared solid
    # (consumed by the chassis + every verifier cradle probe) stays cavity-
    # open.  Capped at the front face by the plate.  WELL_BODY_CL clearance on
    # each body face.
    cav_z_min = body_back_z - 5.0
    cav_z_max = front_face_z
    cavity = _box((SERVO_BODY_W + 2.0 * WELL_BODY_CL,
                   SERVO_BODY_D + 2.0 * WELL_BODY_CL,
                   cav_z_max - cav_z_min),
                  center=(body_centre_x, 0.0,
                          0.5 * (cav_z_min + cav_z_max)))

    # ---- Front mount plate: NO front-face case screws and NO raised
    # disc-horn pedestal (Jun 2026 fix; user photo of the real STS3215).
    # The 25T disc horn rides FLUSH on the output face (only ~DISC_HORN_H
    # = 5 mm proud) and the dia-DISC_HORN_OD (20 mm) horn fully covers the
    # dia-SERVO_MOUNT_SQUARE (9.8 mm) case-screw square -- so the servo
    # CANNOT be bolted through its output face, and there is no room there
    # for a pedestal/collar boss.  The coxa hub bolts straight onto the
    # flush horn (DISC_HORN_BOLT_PCD) and the servo is now POSITIVELY
    # captured between the chassis_bottom mount plate on top and the bolt-on
    # ``yaw_servo_retainer`` capture stirrup underneath (which bolts into 2 M3
    # pilots in the merged floor), so it cannot fall out the bottom.  The
    # output-face region is left fully open by the swing relief below.

    # ---- Inboard (-X) wire-exit window for the STS3215 serial-bus
    # daisy-chain harness.  The STS3215 routes its bus through 2-pin
    # connectors on the body ENDS (not a DS3225-style molded boot), so
    # the cradle only needs a clear window through the -X (radially
    # INWARD) tower wall at the chassis-plate level: the harness leaves
    # the body cavity, turns inboard, and follows the chassis_bottom top
    # face to the per-leg cable-drop slot.  The window runs from inside
    # the body cavity out 2 mm past the -X outer wall, over cradle-z in
    # [-CHASSIS_PLATE_T, +6] (the band that overlaps both the chassis
    # plate and the lower tower wall).
    wire_w = WIRE_SLOT_W
    wire_z_min = -CHASSIS_PLATE_T
    wire_z_max = 6.0
    wire_x_inner = body_centre_x + 5.0                       # into the cavity
    wire_x_outer = body_centre_x - outer_w / 2.0 - 2.0       # 2 mm past -X wall
    wire_slot = _box((wire_x_inner - wire_x_outer, wire_w,
                      wire_z_max - wire_z_min),
                     center=(0.5 * (wire_x_inner + wire_x_outer),
                             0.0, 0.5 * (wire_z_min + wire_z_max)))

    # ---- Retainer-strap anchor pilots (LEGACY end-wall pattern) ----------
    # These 2x M3 self-tap pilots in the thick +/-X end-wall bottoms are the
    # LEGACY anchor for the old flat retainer strap.  The current capture
    # stirrup (``make_yaw_servo_retainer``) instead anchors into the merged
    # floor pilots (``chassis_lower_retainer_anchor_centres``), so these
    # end-wall pilots are vestigial belt-and-suspenders holes; they sit in the
    # wall stock OUTBOARD of the body cavity.
    # Diagonal Y offset (+ on the +X end, - on the -X end) so the two anchor
    # pilots straddle the central y=0 wire-boot insertion channel cut through
    # the +X end wall below (the boot rides straight up the open bottom on the
    # y=0 axis, so a y=0 pilot would sit in its swept volume).  The (x, y)
    # centres come from ``yaw_retainer_anchor_centres`` -- the SAME source the
    # ``yaw_servo_retainer`` strap reads -- so the cradle pilots stay coaxial
    # with the strap clearance holes (guarded by check_clamp_cap_alignment).
    anchor_specs = yaw_retainer_anchor_centres()
    anchor_pilots = []
    for ax, ay in anchor_specs:
        pilot = _cyl(RETAINER_ANCHOR_PILOT_OD / 2.0, RETAINER_ANCHOR_PILOT_DEPTH + 1.0)
        pilot.apply_translation(
            [ax, ay, body_back_z - 0.5 + (RETAINER_ANCHOR_PILOT_DEPTH + 1.0) / 2.0])
        anchor_pilots.append(pilot)

    # END-face M2.5 body screws RETIRED on the yaw cradle (Jun 2026 flush-horn
    # refit).  Moving both 6706 bearings above the flush horn raised the hub
    # stack +5.5 mm; to hold the WORLD hip axis fixed the yaw output was lowered
    # the same amount (YAW_TOWER_RAISE 9 -> 14.5), which drops the whole yaw
    # servo 5.5 mm.  The servo body bottom now hangs ~20 mm below the -6 chassis
    # floor, and even the UPPER end-face bolt row lands right at the floor edge
    # (world z ~= -5.9) with no -X wall left behind its head ("head bearing in
    # air", caught by check_fastener_engagement).  Positive body retention is
    # therefore carried entirely by the bolt-on ``yaw_servo_retainer`` strap +
    # its anchor bolts + the output-face seat on the mount plate + the
    # open-bottom pocket grip -- the supplementary end-face screws are dropped
    # (their reaction was X-translation, not yaw torque, so no torque path is
    # lost).  No end-face bolt holes are cut into this cradle.
    end_bolts = []

    # ---- Disc-horn clearance bore through the mount plate (Jun 2026 fix) ---
    # The mount-plate cap (front_face_z .. plate_top_z) is SOLID on the output
    # axis -- the +X swing relief that opens the rest of the output face is
    # carved away from a tower-protect cylinder that re-covers this central
    # ring.  The dia-DISC_HORN_OD (20 mm) disc horn screws onto the spline and
    # rides on the servo's output face -- so its body OCCUPIES this very plate
    # band (its underside sits at ~ front_face_z + SERVO_OUTPUT_H, ~2 mm into
    # the plate).  A small SERVO_OUTPUT_BORE_OD (Phi 10) coupling bore is WRONG
    # here: the 20 mm disc rams the plate (only the +/-X-half swing relief
    # opens past r ~ 23.5 mm, and the tower-protect cylinder re-covers the disc
    # footprint, so the plate stays solid right where the horn sits).  This is
    # the SAME front-face-plate-vs-disc-horn conflict the sandwich hip/knee
    # cradles already resolved with the Phi HORN_CLEAR_OPENING_OD bore; the yaw
    # cradle was missed.  Bore the FULL disc-horn clearance (Phi
    # HORN_CLEAR_OPENING_OD = DISC_HORN_OD + 4 mm) through the plate band so the
    # disc seats + spins.  The plate still backs the lower bearing outer race
    # (race face at r in [15, 18.5] mm) since material survives outboard of the
    # Phi 24 bore, and the body still seats on the plate underside ring (the
    # STS3215 body is far wider than Phi 24).
    out_bore = _cyl(HORN_CLEAR_OPENING_OD / 2.0,
                    (plate_top_z + 1.0) - (front_face_z - 1.0))
    out_bore.apply_translation([0.0, 0.0,
                                0.5 * ((front_face_z - 1.0) + (plate_top_z + 1.0))])

    # ---- +X wire-boot insertion / exit channel (Jun 2026 insertion fix) ---
    # The STS3215 carries a molded wire-exit boot on its +X short face
    # (WIRE_BOOT_*).  Seated output-UP and inserted from the open bottom, the
    # boot rides straight up the +X end wall on the y=0 axis -- but that wall
    # was solid, so the boot shears against it and the body cannot be pushed
    # home.  Cut a y=0 channel through the +X end wall, open at the bottom and
    # past the outer face, spanning the boot's full insertion sweep, so the
    # boot slides up freely and the bus cable exits radially-outward into the
    # inter-plate gap (the inboard -X window + per-leg drop slot still serve a
    # harness routed the other way).  The two retainer anchor pilots were moved
    # to y=+/-RETAINER_ANCHOR_Y_OFFSET so they straddle (don't block) this slot.
    boot_x_face = body_centre_x + SERVO_BODY_W / 2.0          # +X body face
    boot_ch_x0 = boot_x_face - 1.0                            # open into cavity
    boot_ch_x1 = body_centre_x + outer_w / 2.0 + 2.0         # past outer wall
    boot_seat_z0 = body_back_z + WIRE_BOOT_Z_BASE
    boot_seat_z1 = boot_seat_z0 + WIRE_BOOT_H
    boot_ch_z0 = body_back_z - 1.0                            # open bottom mouth
    boot_ch_z1 = boot_seat_z1 + 4.0                           # margin above seated
    boot_channel = _box((boot_ch_x1 - boot_ch_x0,
                         WIRE_BOOT_W + 1.0,
                         boot_ch_z1 - boot_ch_z0),
                        center=(0.5 * (boot_ch_x0 + boot_ch_x1), 0.0,
                                0.5 * (boot_ch_z0 + boot_ch_z1)))

    body = _diff(outer, cavity, wire_slot, out_bore, boot_channel,
                 *anchor_pilots, *end_bolts)

    # ---- Horn / leg swing clearance ---------------------------------------
    # Drop the OUTBOARD (+X, the direction the leg points + sweeps) portion
    # of the tower top -- INCLUDING the output-face centre -- down to the
    # servo front-face plane, so the FLUSH disc horn and the coxa hub bolted
    # onto it spin in open air.  (Jun 2026: the old central pedestal dome +
    # 4 case-screw counterbores were removed -- the dia-20 flush horn sits
    # right on the face, leaving no room for them.)  The -X half stays
    # capped so the servo body is enclosed and seats on that ring.
    clear_floor_z = front_face_z
    cut_h = (plate_top_z + 30.0) - clear_floor_z
    cut_z_cen = clear_floor_z + cut_h / 2.0
    relief = _box((outer_w + 6.0, outer_d + 6.0, cut_h),
                  center=(body_centre_x + (outer_w + 6.0) / 2.0, 0.0, cut_z_cen))
    # PROTECT the coaxial bearing-tower region (r <= tower OD/2 + margin) from
    # the +X swing relief so the spaced-pair tower below survives.
    tower_protect = _cyl(YAW_BEARING_OD / 2.0 + YAW_TOWER_WALL + 1.5, cut_h * 2.0)
    tower_protect.apply_translation([0.0, 0.0, cut_z_cen])
    relief = _diff(relief, tower_protect)
    body = _diff(body, relief)

    # ---- SPLIT yaw-bearing tower: BOTTOM half (Jun 2026 flush-horn refit) ---
    # Both bearing races (6805 pair) sit ABOVE the flush disc horn (z > 0), so this BOTTOM
    # half is no longer a short ring seated on the mount plate -- it RISES from
    # the mount-plate top (z = -HORN_STACK_H) up to the split plane between the
    # two bearings (z = YAW_SPLIT_Z = +4.5), carrying the LOWER race in an
    # OPEN-TOP Phi 37 pocket whose seat is a printed shoulder at the lower-race
    # bottom (z = YAW_BEARING_LOWER_BOT_Z = +0.5).  Below that shoulder the bore
    # steps IN to Phi YAW_TOWER_SHOULDER_OD (34): wide enough to clear BOTH the
    # rotating hub boss (Phi 29.8) and the flush horn (Phi 20), narrow enough to
    # catch the Phi 37 outer race on a ~1 mm ledge.  The race drops straight
    # down the Phi 37 pocket onto that ledge -- a clear, monotone insertion path
    # with no constriction above it.  The UPPER race + retaining lip live on the
    # bolt-on ``yaw_bearing_cap`` that lands on the split plane.  Coxa-local Z
    # maps to cradle Z via ``out_z`` (= disc-horn top).
    def _cz(coxa_z):
        return out_z + coxa_z
    r_out = YAW_BEARING_OD / 2.0 + YAW_TOWER_WALL          # Phi 44
    r_bore = YAW_TOWER_BORE_OD / 2.0                       # Phi 37 race pocket
    r_shoulder = YAW_TOWER_SHOULDER_OD / 2.0               # Phi 34 seat ledge / boss clear
    # Jun 2026 (floating-ring fix): the tower used to start at
    # ``plate_top_z - 0.5`` (cradle +7.75), which left it cantilevered ~3.5 mm
    # ABOVE the +X swing-relief floor (front_face_z = +4.25) -- a thin Phi 44
    # ring visibly floating over a moat, structurally tied to the chassis only
    # through the mount plate.  EXTEND the solid cylinder all the way DOWN to
    # the chassis FLAT BOTTOM (cradle z = -(CHASSIS_PLATE_T + CHASSIS_BOTTOM_
    # FLOOR_T) = chassis-local z = -6, the print-bed face) so the bearing
    # housing is a continuous column grounded into the bed on the loaded
    # outboard + tangential sides instead of a perched ring stopping 4 mm short
    # at the plate underside.  The servo cavity / horn bore / wire routes are
    # re-opened after the union below so the lowered cylinder doesn't plug the
    # servo that shares this volume.
    tower_bot = -(CHASSIS_PLATE_T + CHASSIS_BOTTOM_FLOOR_T)  # to the flat bottom (chassis z=-6)
    race_seat_z = _cz(YAW_BEARING_LOWER_BOT_Z)             # +0.5 lower-race seat shoulder
    tower_top = _cz(YAW_SPLIT_Z)                           # +4.5 split plane (open top)
    tower = _cyl(r_out, tower_top - tower_bot)
    tower.apply_translation([0.0, 0.0, 0.5 * (tower_bot + tower_top)])
    # Lower outer-race pocket: open-top Phi 37 bore from the seat up to the top.
    race_pocket = _cyl(r_bore, (tower_top + 0.02) - race_seat_z)
    race_pocket.apply_translation([0.0, 0.0, 0.5 * (race_seat_z + tower_top + 0.02)])
    tower = _diff(tower, race_pocket)
    # Relief bore BELOW the lower-race seat -- STEPPED so the tower wall stays
    # THICK and grounds into the deck instead of being a thin ring perched over
    # an empty round moat (Jun 2026 floating-ring/gap fix).  The old single
    # Phi 34 (YAW_TOWER_SHOULDER_OD) relief ran the full height to the deck,
    # but the only thing that needs that width is the rotating hub boss
    # (Phi 29.8), which descends ONLY to the disc-horn top (coxa z = 0).  Below
    # that just the spinning disc horn (Phi 20) + servo output coupling live on
    # the axis, so the bore steps in to the small Phi HORN_CLEAR_OPENING_OD (24)
    # disc-horn clearance, leaving the surrounding wall SOLID out to Phi 44.
    # Combined with the servo-body cutout (which opens the inboard half for the
    # hanging servo), the lower tower becomes a thick GROUNDED horseshoe
    # buttress that carries the yaw-bearing load into the floor on the outboard
    # + tangential sides -- no more floating ring, no more dead gap.
    #   * UPPER step Phi 34: from the disc-horn BASE (= mount-plate top) up to
    #     the race seat -- spans the full rotating disc-horn + hub-boss sweep
    #     (forms the outer-race seat ledge, clears the Phi 29.8 hub boss AND the
    #     spinning Phi 20 disc horn that the horn-sweep check guards).
    #   * LOWER step Phi 24: from the mount-plate top down to the deck -- only
    #     the small output coupling lives this low (the same bore the mount-plate
    #     out_bore already cuts), so the wall stays SOLID out to Phi 44 here.
    # The step sits at the mount-plate top so the wide relief never narrows
    # inside the disc-horn sweep envelope (a higher step clipped that sweep).
    relief_step_z = plate_top_z                        # disc-horn base = mount-plate top
    relief_upper = _cyl(r_shoulder, race_seat_z - (relief_step_z - 0.02))
    relief_upper.apply_translation(
        [0.0, 0.0, 0.5 * ((relief_step_z - 0.02) + race_seat_z)])
    relief_lower = _cyl(HORN_CLEAR_OPENING_OD / 2.0,
                        (relief_step_z + 0.02) - (tower_bot - 0.02))
    relief_lower.apply_translation(
        [0.0, 0.0, 0.5 * ((tower_bot - 0.02) + (relief_step_z + 0.02))])
    tower = _diff(tower, relief_upper, relief_lower)

    # ---- 3 x M3 join-bolt ear bosses (cap pulls DOWN into these pilots) ----
    # Solid bosses straddle the tower wall at YAW_CAP_BOLT_PCD, extending from
    # the mount-plate underside (front_face_z) up to the split face so the
    # self-tap pilots get ~8 mm of engagement.  They sit at coxa-local
    # z <= -1, well BELOW the rotating dust-lip band.
    ear_bot_z = front_face_z
    ear_top_z = tower_top
    for ang in YAW_CAP_BOLT_ANGLES_RAD:
        ex = YAW_CAP_BOLT_PCD / 2.0 * np.cos(ang)
        ey = YAW_CAP_BOLT_PCD / 2.0 * np.sin(ang)
        boss = _cyl(YAW_CAP_BOLT_BOSS_OD / 2.0, ear_top_z - ear_bot_z)
        boss.apply_translation([ex, ey, 0.5 * (ear_bot_z + ear_top_z)])
        tower = _union(tower, boss)
        pilot = _cyl(YAW_CAP_BOLT_PILOT_OD / 2.0, (ear_top_z - ear_bot_z) + 0.5)
        pilot.apply_translation([ex, ey,
                                 ear_top_z - ((ear_top_z - ear_bot_z) + 0.5) / 2.0 + 0.25])
        tower = _diff(tower, pilot)

    body = _union(body, tower)
    # Re-open the servo cavity / disc-horn bore / wire routes that the
    # now-grounded tower cylinder reaches down into and would otherwise plug
    # (the tower and the offset servo body share this z < front_face volume).
    body = _diff(body, cavity, out_bore, boot_channel, wire_slot)
    return body


def yaw_servo_real_back_z() -> float:
    """World Z of the yaw servo's REAL flat case-back (rear flange) face.

    The frozen-kinematic SERVO_BODY_H is the STEP mount-HOLE-plane gap (34.3),
    ~SADDLE_CASE_LEN_FIX shorter than the real case, so the real rear flange
    (where the rear case-mount screws thread) hangs that much below the modeled
    servo_body back face.  The saddle floor + rear screw fingers reference THIS
    plane so the printed holes line up with the real STS3215 rear flange."""
    return (CHASSIS_YAW_OUTPUT_Z - HORN_STACK_H
            - SERVO_BODY_H - SADDLE_CASE_LEN_FIX)


def yaw_rear_screw_centres():
    """The 4 FIXED REAR (back) case-mount screw centres in saddle cradle-local XY
    (origin on the output/yaw axis, +X outboard radial, +Y tangential).

    These are the standard STS3215 case mounting holes on the rear face (STEP
    solid 1, NOT the horn bolt circle): STEP (X,Z) = (8.3, +-10.2) and
    (32.8, +-10.2) -> cradle (x,y) = (-8.3, +-10.2) [the landmark pair, 18.5 mm
    from the +X output-end "top"] and (-32.8, +-10.2) [the companion pair, 43 mm].
    Each is r~13.1 / 34.4 mm from the output axis, so all clear the central
    rotating idler boss; the screws self-tap UP (+Z, the output axis) into the
    rear case face."""
    return [(SADDLE_CASE_HOLE_X1, +SADDLE_CASE_HOLE_Y),
            (SADDLE_CASE_HOLE_X1, -SADDLE_CASE_HOLE_Y),
            (SADDLE_CASE_HOLE_X2, +SADDLE_CASE_HOLE_Y),
            (SADDLE_CASE_HOLE_X2, -SADDLE_CASE_HOLE_Y)]


def make_yaw_servo_retainer() -> trimesh.Trimesh:
    """Printed ANTI-ROTATION SADDLE + short ground FEET for each yaw STS3215.

    Keys the yaw CASE to the chassis (Jun 2026 saddle).  Aug 2026 v4 feet:
    the flat-belly rework deleted the 38 mm ground stand entirely, which
    left the M2.5 case bosses as the robot's resting surface -- the harness
    exiting each case-bottom centre was smashed against the floor (user
    report).  A SHORT foot (RETAINER_FOOT_H = 34 mm below the chassis
    underside) returns to lift the wire exits ~9 mm off the ground; the
    rest of the belly stays flat.  v4 replaces the v3 central-disk tripod
    (it sat directly under the case-back servo PLUG and blocked access,
    user) with FOUR corner poles + small pads -- open underside.

    Geometry -- a U-channel open on +X, with a clear central wire drop:
        * +/-Y SIDE WALLS + a -X END WALL snugly hug the hanging lower case;
        * a BACKSTOP FRAME under the case BACK face with a central DROP
          WINDOW so the harness falls straight down;
        * 4 REAR-CASE CAPTURE SCREWS (M2.5 self-tap up into the case);
        * 4 TOP-FLANGE M3 chassis anchors (driven up from below, all four
          clear of the foot -- open air below every head);
        * FOUR CORNER POLES (RETAINER_POLE_*): full-height square posts
          (all four tops flush with plate_bot) off the side-wall +X ends
          and the rear corners (rear pair tied in by floor-band
          gussets), each ending in a small Ø12 ground pad.  The
          whole underside stays open so the case-back servo plug is
          reachable, and every pole/pad clears the M2.5 Phillips and
          M3 hex-key driver corridors by position.

    Prints FLANGE-DOWN (flipped; the pads are too small to be the bed
    plane), see print_orientation.

    Frame: cradle-local XY (origin on the yaw/output axis, +X = outboard
    radial, +Y = tangential) and WORLD Z.
    """
    plate_bot = CHASSIS_SPLIT_Z - CHASSIS_BOTTOM_FLOOR_T         # -6 (merged floor face)
    # Real seated yaw case (SAME placement _place_servo_bodies uses): the
    # output/front face lands at CHASSIS_YAW_OUTPUT_Z - HORN_STACK_H and the
    # case hangs below it.  The modeled SERVO_BODY_H (34.3) is the STEP mount-
    # HOLE-plane gap, but the user MEASURED the real case ~SADDLE_CASE_LEN_FIX mm
    # longer to the flat bottom (39 mm horn-top -> flat bottom), so we reference
    # the REAL bottom here -- the walls run that much TALLER and the saddle SEATS
    # FLUSH up against the floor instead of being propped low by the backstop
    # hitting the longer real case early.  (SERVO_BODY_H stays frozen globally.)
    servo_back_z = yaw_servo_real_back_z()                          # real flange face

    # Case footprint, cradle-local: the output axis is x=0 but the STS3215
    # output sits off-centre, so the body centre is at x=-SERVO_OUTPUT_X.
    bx_c = -SERVO_OUTPUT_X                                       # -12.5
    x_in = bx_c - SERVO_BODY_W / 2.0                             # -35.2  (-X end, NON-wire)
    x_out = bx_c + SERVO_BODY_W / 2.0                            # +10.2  (+X end, wire boot)
    yf = SERVO_BODY_D / 2.0                                      # 12.4

    yi = yf + SADDLE_BODY_CL                                     # snug inner face
    yo = yi + SADDLE_WALL_T                                      # wall outer face
    xi = x_in - SADDLE_BODY_CL                                   # -X wall inner face
    xo = xi - SADDLE_WALL_T                                      # -X wall outer face

    wall_z0, wall_z1 = servo_back_z, plate_bot                   # ~[-24.05, -6]

    parts = []
    # +/-Y side walls (run the body length; +X stays open for the boot).
    for sgn in (-1.0, +1.0):
        wall = _box((x_out - xi, SADDLE_WALL_T, wall_z1 - wall_z0),
                    center=(0.5 * (xi + x_out), sgn * 0.5 * (yi + yo),
                            0.5 * (wall_z0 + wall_z1)))
        parts.append(wall)
    # -X end wall ties the side walls + adds a third anti-rotation key face.
    endw = _box((SADDLE_WALL_T, 2 * yo, wall_z1 - wall_z0),
                center=(0.5 * (xo + xi), 0.0, 0.5 * (wall_z0 + wall_z1)))
    parts.append(endw)

    # Anti-drop BACKSTOP FRAME under the case back face (z = servo_back_z): the
    # full plate spans out to the wall outers (welds them into one body) but a
    # central DROP WINDOW is cut from it below (see `cuts`) so the centrally-
    # exiting harness falls straight down -- only a SADDLE_FLOOR_RIM-wide ledge
    # remains on the -X + -/+Y edges to catch the back-face rim.
    fl_z1 = servo_back_z
    fl_z0 = fl_z1 - SADDLE_FLOOR_T
    floor = _box((SADDLE_FLOOR_X_OUT - xo, 2 * yo, SADDLE_FLOOR_T),
                 center=(0.5 * (xo + SADDLE_FLOOR_X_OUT), 0.0,
                         0.5 * (fl_z0 + fl_z1)))
    parts.append(floor)

    # Top-flange anchor tabs: a THICK solid pad (radius SADDLE_ANCHOR_PAD_R,
    # reaching IN to weld its +/-Y side wall) in the z[flange_z0, plate_bot] band.
    # The head does NOT bear on the bottom face -- a counterbore (cut below)
    # recesses it so the seat sits at flange_z0 + SADDLE_HEAD_CB_DEPTH = -9, with
    # the down-facing socket open to the clear driver cone in the cavity below.
    flange_z0 = plate_bot - SADDLE_FLANGE_T                      # -11 (boss bottom)
    flange_z1 = plate_bot                                        # -6  (mates floor)
    for (ax, ay) in chassis_lower_retainer_anchor_centres():
        pad = _cyl(SADDLE_ANCHOR_PAD_R, SADDLE_FLANGE_T)
        pad.apply_translation([ax, ay, 0.5 * (flange_z0 + flange_z1)])
        parts.append(pad)

    # ---- Ground FEET (Aug 2026 v4: FOUR corner poles + small pads) --------
    # See the RETAINER_POLE_* constants comment: the whole underside stays
    # open so the case-back servo plug can be (un)mated freely; poles and
    # pads dodge all eight driver corridors.
    tip_z = plate_bot - RETAINER_FOOT_H                          # -40
    pad_top_z = tip_z + RETAINER_PAD_H

    def _corner_pad(px: float, py: float) -> trimesh.Trimesh:
        pad = _cyl(RETAINER_PAD_OD / 2.0, RETAINER_PAD_H)
        pad.apply_translation([px, py, tip_z + RETAINER_PAD_H / 2.0])
        ch = RETAINER_PAD_CHAMFER
        ring = _cyl(RETAINER_PAD_OD / 2.0 + 0.2, ch)
        ring.apply_translation([px, py, tip_z + ch / 2.0])
        keep = _cyl(RETAINER_PAD_OD / 2.0 - ch, ch + 0.2)
        keep.apply_translation([px, py, tip_z + ch / 2.0])
        return _diff(pad, _diff(ring, keep))

    pw = RETAINER_POLE_W
    # FRONT pole pair: welds flat onto the side-wall outer face over the
    # full wall height (top flush with plate_bot), then drops to its pad.
    fx, fy = RETAINER_POLE_FRONT
    for sgn in (-1.0, +1.0):
        pole = _box((pw, pw, plate_bot - pad_top_z),
                    center=(fx, sgn * fy, 0.5 * (plate_bot + pad_top_z)))
        parts.append(pole)
        parts.append(_corner_pad(fx, sgn * fy))
    # REAR pole pair: diagonally off the saddle's rear corners (outside
    # the x2 Φ12 corridors).  Full height like the front pair -- top flush
    # with plate_bot so all four posts bear on the chassis underside (user
    # Aug 2026: same top Z on all feet for structural stability).  Since
    # the pole body sits diagonally OFF the wall corner (only a sliver of
    # direct wall contact), a corner GUSSET ties it into the end-wall /
    # side-wall / floor corner; the gusset underside sits at fl_z0, ABOVE
    # the M2.5 head plane (fl_z0 - 0.5), so bridging over the screw
    # corridor is legal -- the driver only needs the space BELOW the head.
    # The pole runs THROUGH the gusset for a solid volumetric weld.
    rx, ry = RETAINER_POLE_REAR
    gz1 = plate_bot - 18.0          # gusset top (-24): 3 mm into the walls
    # Gusset bounds: from the pole centre diagonally into the end-wall /
    # side-wall / floor corner.  x reaches -34 (4.3 mm into the end wall +
    # floor rim), y starts at 14 (1.5 mm into the wall band).  Nearest
    # approach to the M3 anchor corridor at (-29, +-21) is 5.2 mm > Φ8/2.
    gx0, gx1 = rx, -34.0
    gy0, gy1 = 14.0, ry
    for sgn in (-1.0, +1.0):
        gusset = _box((gx1 - gx0, gy1 - gy0, gz1 - fl_z0),
                      center=(0.5 * (gx0 + gx1), sgn * 0.5 * (gy0 + gy1),
                              0.5 * (gz1 + fl_z0)))
        parts.append(gusset)
        pole = _box((pw, pw, plate_bot - pad_top_z),
                    center=(rx, sgn * ry, 0.5 * (plate_bot + pad_top_z)))
        parts.append(pole)
        parts.append(_corner_pad(rx, sgn * ry))

    saddle = _union(*parts)

    cuts = []
    # Central DROP WINDOW through the backstop floor: opens the bottom-centre so
    # the harness exiting the case-bottom centre drops straight down into the
    # stand shaft.  Leaves a SADDLE_FLOOR_RIM ledge on the -X + -/+Y edges;
    # the +X side is already open.
    win_x0 = xo + SADDLE_FLOOR_RIM
    win_x1 = SADDLE_FLOOR_X_OUT + 1.0
    win_y = yo - SADDLE_FLOOR_RIM
    window = _box((win_x1 - win_x0, 2 * win_y, SADDLE_FLOOR_T + 2.0),
                  center=(0.5 * (win_x0 + win_x1), 0.0, 0.5 * (fl_z0 + fl_z1)))
    cuts.append(window)
    # (v4: no wire-exit slot cut needed -- the corner poles leave the whole
    # underside below the floor open; the harness dropping through the
    # window can run out in any horizontal direction between the poles.)

    # 4x M3 anchor-bolt clearance holes through the flange tabs (axis Z) + a
    # head COUNTERBORE opening downward from the boss bottom so the M3 SHCS head
    # recesses (seat at flange_z0 + SADDLE_HEAD_CB_DEPTH = -9; the M3x6 tip still
    # reaches the -3 pilot for a full 3 mm bite).
    for (ax, ay) in chassis_lower_retainer_anchor_centres():
        hole = _cyl(RETAINER_BOLT_CLEAR_OD / 2.0, SADDLE_FLANGE_T + 2.0)
        hole.apply_translation([ax, ay, 0.5 * (flange_z0 + flange_z1)])
        cuts.append(hole)
        cb_h = SADDLE_HEAD_CB_DEPTH + 1.0   # +1 so the bottom mouth cuts cleanly
        cbore = _cyl(SADDLE_HEAD_CB_OD / 2.0, cb_h)
        cbore.apply_translation([ax, ay, flange_z0 + SADDLE_HEAD_CB_DEPTH - cb_h / 2.0])
        cuts.append(cbore)

    # (v4: no driver relief cuts needed -- the corner poles/pads are placed
    # clear of every M2.5 Phillips and M3 hex-key corridor by position;
    # check_screwdriver_access verifies this instead of geometry carving.)

    body = _diff(saddle, *cuts)

    # 4x FIXED-REAR-CASE-MOUNT capture screws (CORRECTED Jun 2026): a printed
    # boss under EACH of the 4 rear-face case holes (yaw_rear_screw_centres ->
    # cradle (-8.3,+-10.2) landmark pair + (-32.8,+-10.2) companion pair), each
    # spanning z[servo_back_z - SADDLE_CASE_SHANK, servo_back_z] up to the real
    # rear case face.  An M2.5 self-tapper drives straight UP (the holes' axis IS
    # the output axis) through the boss into the case (SADDLE_CASE_SCREW_BITE); its
    # head bears FLAT on the boss UNDERSIDE (boss bottom = head plane) -- NO
    # counterbore, so nothing sits below the head to foul the driver cone, which
    # enters straight up the open under-chassis cavity.  Each boss reaches r
    # SADDLE_CASE_BOSS_R, welding into the +-Y side-wall / floor rim (and the -X
    # end wall for the x2 pair) for a single body.  Bosses are unioned AFTER the
    # central drop-window cut so the window can't carve them; their clearance bores
    # are cut last.  They sit at |y| ~ 10.2 (out by the depth walls), well clear of
    # the centre wire exit (cradle x~-14, y~0) and the central idler (axis, r~0).
    zc = servo_back_z                                           # real rear case face
    boss_z0 = zc - SADDLE_CASE_SHANK                            # boss bottom = head plane
    bosses = []
    boss_cuts = []
    for (rx, ry) in yaw_rear_screw_centres():
        boss = _cyl(SADDLE_CASE_BOSS_R, SADDLE_CASE_SHANK)
        boss.apply_translation([rx, ry, 0.5 * (boss_z0 + zc)])
        bosses.append(boss)
        # M2.5 clearance bore (full boss height + a touch into the case region).
        bore = _cyl(SADDLE_CASE_SCREW_OD / 2.0, SADDLE_CASE_SHANK + 2.0)
        bore.apply_translation([rx, ry, 0.5 * (boss_z0 + zc)])
        boss_cuts.append(bore)

    body = _union(body, *bosses)
    return _diff(body, *boss_cuts)


def _yaw_cap_bolt_centres():
    """yaw_bearing_cap <-> bottom-tower M3 join-bolt centres (coxa-local XY)."""
    r = YAW_CAP_BOLT_PCD / 2.0
    return [(r * np.cos(t), r * np.sin(t)) for t in YAW_CAP_BOLT_ANGLES_RAD]


def make_yaw_bearing_cap() -> trimesh.Trimesh:
    """TOP half of the SPLIT yaw-bearing tower (Jun 2026 insertion fix).

    The single-piece chassis tower bored 37 -> 34 -> 37 -> 34 and trapped
    BOTH 6706 outer races between Phi 34 constrictions, so neither race could
    reach its seat -- physically un-assemblable, yet the static CAD validated
    the seated state.  The tower is now split at the lower race's top face
    (coxa-local z = YAW_SPLIT_Z = -1); the BOTTOM half (in ``chassis_bottom``)
    holds the lower race in an open-top pocket, and THIS bolt-on cap holds the
    UPPER race.

    Coxa-local frame (z = 0 = disc-horn top), coaxial with the yaw axis --
    identical to ``make_coxa_yaw_hub`` so the same per-leg placement transform
    positions both.  Vertical layout:

        z[-1, +6]    Phi 37 CLEAN through-bore (radial housing for the upper
                     outer race; NO sub-Phi-37 constriction below the race)
        z[+6, +7]    Phi 34 inner RETAINING LIP -- a thin shoulder the upper
                     outer race seats UP against, holding the bearing in (the
                     lip clears the Phi 32 rotating inner race / hub uflange)
        z[-1, +3]    3 x M3 outboard ear lugs (bolt DOWN into the tower pilots)

    (Jun 2026: the bottom register wrap-lip at z[-3.5, -1] was removed -- it
    was a thin, hard-to-print floating ring; the two halves are now kept
    concentric by the 3 join bolts seating into the tower pilots.)

    ASSEMBLY (Jun 2026 assemblability fix): the cap is LOWERED straight down
    over the upper outer race already seated on the hub boss, so the bore is
    Phi 37 all the way DOWN from the lip to the split plane.  The retired Phi 34
    NECK (which capped the LOWER race + seated the upper race from BELOW) was an
    85.5 mm^3 hard stop on that descent -- see ``check_bearing_cap_descent_path``.
    The new Phi 34 lip is the OPPOSITE feature: it sits ABOVE the seated race
    (z[+6, +7]) so the cap clears the race all the way down and the lip only
    lands on the race TOP at full descent -- positive axial retention with no
    descent hard stop.  The upper bearing is now POSITIVELY held: located
    axially by its inner race against the hub uflange (z=+6) + the M3 disc-horn
    clamp preload, and capped by this lip on the outer race.  The lower
    (floating) outer race is held by the tower press-fit + its z=-5 plate seat.

    The 3 M3 head counterbores are cut as COMPLETE circles up through the (now
    +7) ring rim: since the bolt circle (PCD 47) is ~1.5 mm outboard of the
    Phi 44 ring OD, each head pocket carves a small scallop into the ring wall,
    giving the screw head a clean full-circle seat + clear driver access from
    above instead of being crowded by the taller ring wall.
    """
    r_out = YAW_BEARING_OD / 2.0 + YAW_TOWER_WALL          # Phi 44
    r_bore = YAW_TOWER_BORE_OD / 2.0                       # Phi 37 race seat

    split_z = YAW_SPLIT_Z                                  # -1
    top_z = YAW_CAP_TOP_Z                                  # +6 (race top / lip underside)
    rim_z = YAW_CAP_RIM_Z                                  # +7 (rim above the lip)

    # ---- Main ring (Phi 44 OD, split plane up to the rim above the lip) ----
    ring = _cyl(r_out, rim_z - split_z)
    ring.apply_translation([0.0, 0.0, 0.5 * (split_z + rim_z)])
    cap = ring

    # ---- 3 x M3 join-bolt OUTBOARD ear lugs ----
    # Each ear boss (YAW_CAP_BOLT_BOSS_OD) is centred on YAW_CAP_BOLT_PCD,
    # straddling the ring OD so its inner flank merges solidly into the ring
    # (single connected body) and its outboard flank carries the bolt head
    # counterbore clear of the Phi 37 bore.  The ears top out at
    # YAW_CAP_EAR_TOP_Z (+3), below the rotating dust-lip band (z >= +3.5),
    # so the radial growth never fouls the turntable skirt.
    ear_top = YAW_CAP_EAR_TOP_Z                            # +3 (below dust lip)
    for (ex, ey) in _yaw_cap_bolt_centres():
        boss = _cyl(YAW_CAP_BOLT_BOSS_OD / 2.0, ear_top - split_z)
        boss.apply_translation([ex, ey, 0.5 * (split_z + ear_top)])
        cap = _union(cap, boss)

    cuts = []
    # CLEAN Phi 37 through-bore from the split plane to the open top.
    #
    # ASSEMBLABILITY (Jun 2026 fix): a bearing is ONE rigid unit, so the upper
    # inner race must slide onto the hub boss (from the boss BOTTOM, the only
    # open end -- the platform caps the top) AND the upper outer race must end
    # up in this cap.  The two races move together, so the ONLY valid sequence
    # is: slide both bearings up the hub boss, drop the hub+bearings into the
    # bottom tower, then LOWER THIS CAP straight down over the already-seated
    # upper outer race.  The retired Phi 34 neck (a lower-race cap + z=+2 seat
    # shoulder) sat BELOW the Phi 37 pocket, so lowering the cap forced its
    # Phi 34 wall down past the Phi 37 race -- an 85.5 mm^3 hard stop the cap
    # could not pass (measured; see check_bearing_cap_descent_path).  The old
    # check only swept the race DOWN into a free-standing cap (race-into-cap,
    # which never happens in the robot) so it stayed blind to this.
    #
    # The bore is now Phi 37 ALL THE WAY DOWN: the cap slips over the Phi 37
    # outer race with a clear monotone path.  The upper bearing is located
    # axially by its INNER race seating up against the hub uflange (z=+6) plus
    # the 4 x M3 disc-horn clamp preload -- the cap provides RADIAL (moment)
    # support only, which is the bearing pair's job here.  The lower (floating)
    # outer race is retained by the tower press-fit + its z=-5 plate seat, so
    # dropping its cap is also mechanically correct for a floating race.
    bore = _cyl(r_bore, (top_z - split_z) + 0.02)
    bore.apply_translation([0.0, 0.0, 0.5 * (split_z + top_z)])
    cuts.append(bore)

    # Inner retaining lip: ABOVE the race top the bore steps IN from Phi 37 to
    # Phi YAW_CAP_LIP_ID, so the lip underside (z = top_z) caps the upper outer
    # race.  The cap lowers straight down over the seated race (clean Phi 37 all
    # the way to top_z), and the lip lands on the race top only at full descent
    # -- positive axial retention with NO bottom-neck hard stop (the race never
    # passes the lip).  Phi 34 clears the Phi 32 rotating inner race / uflange.
    lip_bore = _cyl(YAW_CAP_LIP_ID / 2.0, (rim_z - top_z) + 0.04)
    lip_bore.apply_translation([0.0, 0.0, 0.5 * (top_z + rim_z)])
    cuts.append(lip_bore)

    # Join-bolt clearance + head counterbore.  The M3 self-tap join screw
    # enters from the ear TOP (z = ear_top), its head recessed onto the
    # cb_floor bearing plane, shank clearing the cap, threads biting the tower
    # pilot below the split plane.
    cb_floor = ear_top - (INSERT_M3_BOLT_HEAD_H + 0.3)    # head bearing plane
    for (ex, ey) in _yaw_cap_bolt_centres():
        clr = _cyl(YAW_CAP_BOLT_OD / 2.0, (ear_top - split_z) + 2.0)
        clr.apply_translation([ex, ey, 0.5 * (split_z + ear_top)])
        cuts.append(clr)
        # Head counterbore: a COMPLETE circular seat for the M3 head at
        # cb_floor, cut all the way UP through the (now taller) ring rim so the
        # ring no longer towers beside the head.  Because the bolt circle (PCD
        # 47) sits ~1.5 mm OUTBOARD of the Phi 44 ring OD, the head circle
        # overlaps the ring slightly, so this carves a small scallop INTO the
        # ring wall -- giving a clean full-circle head pocket with clear
        # driver access from above, while keeping a >= 2 mm wall to the bore.
        cb_top = rim_z + 0.02
        cb = _cyl(YAW_CAP_BOLT_HEAD_OD / 2.0, cb_top - cb_floor)
        cb.apply_translation([ex, ey, 0.5 * (cb_floor + cb_top)])
        cuts.append(cb)

    return _diff(cap, *cuts)


# Output-coupling clearance bore through the front mount plate.  At the
# front-face plane only the output COUPLING rotates (spline + bushing,
# ~dia 9); the dia-20 disc horn sits ABOVE the plate, so the plate bore
# need only clear the coupling.  Bore dia 10 (r5) stays clear of the
# 4 M2.5 holes' inner edges (r 6.9 - 1.35 = 5.55).
SERVO_OUTPUT_BORE_OD = 10.0   # mm -- central clearance bore in the mount plate


def servo_mount_hole_centres():
    """The 4 STS3215 case-face mount-hole centres (servo-local XY).

    A SERVO_MOUNT_SQUARE = 9.8 mm square centred on the output axis
    (x = SERVO_OUTPUT_X, y = 0).  Returns [(x, y), ...].  The holes are
    on the FRONT (output, +Z) face -- the cradle's output-side plate
    bolts to them with 4x M2.5 (countersunk flush) into the servo's own
    metal case."""
    return [
        (SERVO_OUTPUT_X + sx * SERVO_MOUNT_HOLE_X_OFFSET,
         sy * SERVO_MOUNT_HOLE_Y_OFFSET)
        for sx in (-1, 1) for sy in (-1, 1)
    ]


def servo_front_case_hole_centres():
    """The 4 small FRONT-face case-shell screw-hole centres (servo-local XY).

    STEP-verified (see the SERVO_FRONT_CASE_* constants block): the same
    small self-tap hole family the yaw saddle uses on the REAR face
    (``yaw_rear_screw_centres``), but on the FRONT (output) shell deck.
    One pair shares the rear pair's (x, y) = (4.2, +-10.25); the other
    pair sits at (-16.5, +-10.25), 3.8 mm off the rear companion pair.
    Returns [(x, y), ...] in the well-local / body frame."""
    return [
        (hx, sy * SERVO_FRONT_CASE_HOLE_Y)
        for hx in SERVO_FRONT_CASE_HOLE_XS for sy in (-1, 1)
    ]


def servo_holder_front_case_hole_centres():
    """The three front-case capture sites retained by the serviceable holder.

    ``servo_front_case_hole_centres`` describes all four molded sites on the
    physical STS3215.  The hip/knee holder deliberately omits the one site
    swept away by the +Y horn service slot so the servo, fitted disc horn and
    clamp cap can lift out together.  Keep the filter geometric: if the horn
    opening or the physical case pattern changes, this helper continues to
    identify the site that actually lies in the extraction slot.
    """
    return [
        (x, y) for x, y in servo_front_case_hole_centres()
        if not (y > 0.0 and
                abs(x - SERVO_OUTPUT_X) < HORN_SERVICE_SLOT_W / 2.0)
    ]


def servo_end_face_bolt_centres():
    """The 4 STS3215 body-retention hole centres on ONE END (+/-X) face.

    A SERVO_BODY_BOLT_PITCH = 10 mm square CENTRED on the END face, so
    the centres are independent of which +/-X face (the caller picks the
    X plane) and of the output offset.  Returns [(y, z), ...] in the
    well-local / body frame (body centred at x=y=0, z in [0,
    SERVO_BODY_H]): y = +/-5, z = SERVO_BODY_H/2 +/- 5.  Callers place
    these on whichever end-face X plane and add their own frame's Z
    offset for the body back face."""
    h = SERVO_BODY_BOLT_PITCH / 2.0
    return [
        (sy * h, SERVO_BODY_H / 2.0 + sz * h)
        for sy in (-1, 1) for sz in (-1, 1)
    ]


# ===========================================================================
# Bearing-SANDWICH joint primitives (Jun 2026 production refit)
# ===========================================================================
# All built in the JOINT-LOCAL frame shared with ``_servo_well_solid`` /
# ``_servo_envelope``:
#     origin = centre of the servo BACK (idler) face
#     +X     = body long axis (output-offset side; output axis at +SERVO_OUTPUT_X)
#     +Z     = output-shaft direction
# Z landmarks: front(output) face = SERVO_BODY_H; mount-plate top =
# SERVO_BODY_H + WELL_PLATE_T; disc-horn top = + HORN_STACK_H above that.
# These primitives are the validated proof geometry (tools/sts3215_testfit.py)
# promoted into the model so every driven joint shares one reusable pattern.

# Back-side stack depth (frozen).  The back mirrors the front: rear boss
# (REAR_BOSS_H) + frozen horn offset (HORN_STACK_H) measured from the back
# face.  Kept = SERVO_OUTPUT_H + HORN_STACK_H = 7 mm so the yoke bottom-arm
# seat / COXA_HIP_ANCHOR_Y / joint envelope are byte-identical to the
# retired 688-bearing era (kinematics frozen).  Retired name was
# PASSIVE_BACK_PLATE_T.
BACK_STACK_DEPTH     = REAR_BOSS_H + HORN_STACK_H                      # 7 mm
PASSIVE_BACK_PLATE_T = BACK_STACK_DEPTH                                # 7 mm

# Joint-local Z of the disc-horn TOP plane on the output axis (the moving
# link / yoke top arm seats here).  The disc horn seats on the servo OUTPUT
# boss (SERVO_OUTPUT_H above the front face) -- NOT on the bracket mount
# plate -- exactly as the yaw joint does (CHASSIS_YAW_OUTPUT_Z = gear_top +
# HORN_STACK_H).  Using WELL_PLATE_T here put the yoke seat 2 mm
# (= WELL_PLATE_T - SERVO_OUTPUT_H) above the real horn top, which showed up
# as a +2 mm femur/tibia mating-face gap.
JOINT_HORN_TOP_Z = SERVO_BODY_H + SERVO_OUTPUT_H + HORN_STACK_H        # 41.3 mm

# Reach-down depth for the DRIVEN (front) yoke top-arm pad.  JOINT_HORN_TOP_Z
# stays FROZEN (kinematic seat plane / link mount), but the real STS3215 output
# spline is FLUSH with the body front face -- it does NOT protrude SERVO_OUTPUT_H
# -- so the driven disc horn actually tops out at SERVO_BODY_H + DISC_HORN_H =
# 36.3 mm, i.e. SERVO_OUTPUT_H (2 mm) BELOW where the generic HORN_REACH_DOWN (3)
# assumed.  The top-arm pad must therefore bridge that extra 2 mm so it seats
# FLUSH on the real horn instead of leaving the user-reported ~2 mm clevis gap.
# The yaw joint already models this flush output (its _horn_world_transform uses
# horn_base_dz = 0); the hip/knee driven horn now does too.
DRIVEN_HORN_REACH_DOWN = JOINT_HORN_TOP_Z - (SERVO_BODY_H + DISC_HORN_H)  # 5 mm

# Jun 2026 symmetric-yoke refit (user: "same screw on each side").  BOTH clevis
# arms now use ONE reach-down pad depth so a single screw (M3 x 10) bolts the
# yoke to the disc horn on each side.  The DRIVEN top arm already needs 5 mm (the
# flush output sits the driven disc 5 mm below the frozen JOINT_HORN_TOP_Z seat),
# so 5 mm is the shared pad; the PASSIVE bottom-arm seat is moved DOWN to put the
# same 5 mm pad onto the real passive disc face (see JOINT_HORN_BOT_Z below).
YOKE_ARM_PAD = DRIVEN_HORN_REACH_DOWN                                  # 5 mm (both arms)
# Tiny clamp preload (user: "maybe a very tiny bit loose still"): each pad reaches
# YOKE_SEAT_INTERF PAST the rigid disc face so the printed arms seat snug with a
# light squeeze instead of dead-flush.  ~0.13 mm/side = ~0.26 mm off the inner
# span -- well inside the +/-0.5 mm mating-face tolerance.
YOKE_SEAT_INTERF = 0.13                                               # mm per side
# Aug 2026 bench fit (user: "the tibia knee yoke still has a 1mm gap with
# the two horns on the servo - can make each pad 0.5mm thicker?", then
# "the femur link has the same issue"): BOTH printed clevises sat
# ~0.5 mm/side off the real disc faces (FDM tolerance stack across the
# clevis span), so both yokes' reach-down pads grow 0.5 mm/side PAST
# nominal.  In nominal CAD that is ~0.63 mm/side of pad-into-disc
# interference (an intended, bench-measured preload fill; the viz overlap
# check ignores the yoke/horn pairs for this reason) -- on the real parts
# it just closes the measured gaps, so screw grip lengths are unchanged
# in practice.
YOKE_PAD_EXTRA_REACH = 0.5                                            # mm per side
# Real PASSIVE disc mating face (joint-local).  Jul 2026 stock-horn refit
# (user): the STOCK metal passive horn's centre bore slides OVER the rear
# idler boss, so the horn seats FLUSH on the servo back face -- its mating
# face is only DISC_HORN_H below the back face (flush with the boss tip),
# NOT (REAR_BOSS_H + DISC_HORN_H) as in the retired printed-adapter stack.
# This narrows the yoke clevis opening by exactly REAR_BOSS_H = 2 mm (the
# user measured the adapter-era yoke 2 mm too wide on the stock horn).
PASSIVE_HORN_FACE_Z = -DISC_HORN_H                                    # -2 mm

# Joint-local Z of the PASSIVE disc-horn seat plane (the yoke BOTTOM arm body's
# inner face).  Symmetric refit: the bottom arm is a true MIRROR of the top arm
# about the real disc-stack mid-plane -- it carries the SAME YOKE_ARM_PAD (5 mm)
# down onto the real passive disc face, so the seat is PASSIVE_HORN_FACE_Z minus
# one pad depth.  (BACK_STACK_DEPTH / COXA_HIP_ANCHOR_Y / the joint envelope stay
# FROZEN -- the stock-horn refit only pulls the printed bottom arm 2 mm closer.)
JOINT_HORN_BOT_Z = PASSIVE_HORN_FACE_Z - YOKE_ARM_PAD                 # -7 mm

# Hip-bracket centering on the yaw axis (see the big note next to
# COXA_HIP_DROP for the rationale + kinematic consequence).  With the hip
# disc-horn-top anchored at y=0 the fixed-side footprint spans coxa-Y in
# [JOINT_HORN_TOP_Z - WELL_H, JOINT_HORN_TOP_Z + PASSIVE_BACK_PLATE_T];
# this anchor Y slides that span so its centre lands on the yaw axis.
COXA_HIP_ANCHOR_Y = -(JOINT_HORN_TOP_Z - (WELL_H - PASSIVE_BACK_PLATE_T) / 2.0)
COXA_HIP_ANCHOR = (COXA_LENGTH, COXA_HIP_ANCHOR_Y, COXA_HIP_DROP)


def _disc_horn_bolt_centres():
    """The 4 driven-horn bolt centres (joint-local XY) on the Phi
    DISC_HORN_BOLT_PCD circle about the output axis, phased to the
    production disc horn (DISC_HORN_BOLT_ANGLES_RAD = 0/90/180/270)."""
    r = DISC_HORN_BOLT_PCD / 2.0
    return [(SERVO_OUTPUT_X + r * np.cos(t), r * np.sin(t))
            for t in DISC_HORN_BOLT_ANGLES_RAD]


# make_passive_horn_adapter is RETIRED (Jul 2026 stock-horn refit): the
# STS3215's stock metal passive horn centres itself directly on the rear
# idler boss (its bore rides the boss, seating flush on the back face), so
# no printed centering/standoff bushing is needed.


# ---- Sandwich-cradle REAR TAB (Aug 2026, user: "add two holes on the other
# side of the femur link - the one closer to the yoke because i dont want to
# block the electronics coming out ... so i can screw into the body of the
# sts on the other side"; Aug 17 2026 the coxa hip cradle grew the SAME tab
# -- user: "copy that same part to be on the coxa link as well so I can
# screw into the servo from both sides") -------------------------------------
# The knee cradle holds its servo by the clamp cap + lip + the 3 retained
# front-case self-tappers on the OUTPUT side (the Sep 2026 horn service slot
# consumes the fourth; its end-face bolts were dropped in
# Jul 2026 -- the fused spar covers that wall).  This tab hangs under the
# OPEN back (idler) face and picks up the servo's rear molded screw-hole
# PAIR nearer the -X (spar / hip-yoke) end -- body-frame
# (SADDLE_CASE_HOLE_X2 + SERVO_OUTPUT_X = -20.3, +/-10.2), the same hole
# family the yaw saddle self-taps -- with 2x M2.5 x 6 self-tappers
# (PN 96877A150, ~2.5 mm bite into the ~2.8 mm molded pilots).  The pair
# nearer the output end (+4.2) is left OPEN so the back-face 5264 connector
# bay / bus harness stays unblocked (user).  Clearances: the tab stays
# z >= -FEMUR_REAR_TAB_T = -5.5 with the heads FLUSH in their recesses (see
# the head-recess note below), the swinging tibia arm's inner face passes
# at z = -7 (~1.5 mm running clearance), and the reachable tibia-spine sweep
# sector never points at the -X azimuth (the well walls already stand
# there).  The +Y corner is relieved 0.25 mm below the knee clamp cap's
# dropped tongue (CLAMP_SEAT_DROP) so the cap still seats.
#
# THE 4 mm BUMP, clarified (Aug 15 2026 bench photo, user: "4 mm ... this is
# absolute max 4 mm and should probably be 3.5 to give some tolerance for the
# bump on the sts motor on this side" / "this green part sticks out too far"):
# the bump is ON THE BACK FACE, not just the end flare.  Measured from the
# servo's -X (wire) END FACE across the back face there is only a short FLAT
# recessed ledge -- it carries the rear molded hole pair (centres 2.4 mm
# in) -- and then the case shell STEPS UP (~1.8 mm proud, the raised centre
# platform around the 5264 bay).  Any printed tab lying on the back face must
# stop on that ledge, else it lands on the step and the screws can never pull
# it flush.  Aug 17 2026 bench remeasure with the tab printed (user: "make
# [it] 1 mm longer? It has a tiny bit more space"): the flat ledge is a hair
# under 5 mm, not the ~4 mm read off the photo, so the reach grew 3.5 -> 4.5
# past the end face (x <= -18.2).  That retires the open KEYHOLE compromise
# of the 3.5 mm tab: the Phi 2.7 holes (rims at -18.95) are now fully
# ENCLOSED with 0.75 mm of rim wall, and the Phi 4.6 pan head overhangs the
# +X edge by only 0.2 mm (effectively fully seated), still in free air BELOW
# the step (head bears at z = -3.5; the step only reaches -1.8).
#
# HEAD RECESS (Aug 17 2026, user: "you also need to have room to countersink
# the screw head on the side with two holes just like on the side with four
# holes"): the tab is 2 mm THICKER than the screw shank and each hole gets a
# Phi 5.2 straight counterbore 2 mm deep from the outer face, so the Phi 4.6
# pan head sits FLUSH in a pocket (like the output plate's 4 countersunk
# case screws) instead of proud.  The counterbore floor stays at z = -3.5 =
# -SADDLE_CASE_SHANK, so the clamp shank (3.5 mm of plastic under the head)
# and the 2.5 mm self-tap bite into the ~2.8 mm molded pilot are UNCHANGED --
# recessing into the old 3.5 mm tab instead would have bottomed the M2.5 x 6
# out in the pilot.  The outer face moves to z = -5.5; the swinging yoke
# arm's inner face passes at z = -7, so the flush head keeps the same
# ~1.5 mm running clearance the proud head had.  The Phi 5.2 counterbore rim
# (-17.7) breaks out of the +X edge by 0.5 mm -- a shallow cosmetic notch in
# the recess wall only (z -3.5..-5.5, below the case step); the head still
# bears on a nearly-full-ring counterbore floor.
FEMUR_REAR_TAB_SHANK_T    = 3.5  # mm -- plastic under the head = SADDLE_CASE_SHANK
FEMUR_REAR_TAB_HEAD_CB    = 2.0  # mm -- head-recess counterbore depth (~pan head H)
FEMUR_REAR_TAB_HEAD_CB_OD = 5.2  # mm -- counterbore bore over the Phi 4.6 pan head
FEMUR_REAR_TAB_T = FEMUR_REAR_TAB_SHANK_T + FEMUR_REAR_TAB_HEAD_CB  # 5.5 (z -5.5..0)
FEMUR_REAR_TAB_X1     = -18.2 # mm -- +X edge = -SERVO_BODY_W/2 + 4.5: the tab
                              #       stops on the flat ledge, clear of the
                              #       back-face step (bump) ~5 mm in (Aug 17
                              #       2026 bench remeasure; was 3.5/-19.2)
FEMUR_REAR_TAB_Y1     = 12.8  # mm -- +Y edge (Phi 4.6 screw head fully seated);
                              #       -Y edge is flush under the -Y wall (-16.9)
# Riser height is CAPPED at 3.5 for the same bump family: on the wire (-X)
# end the shell also flares outward starting ~3.7-4 mm above the rear mount
# plane, so printed material standing next to that end face must stay BELOW
# z ~ 3.5.  The riser exists only to fuse the tab into the -X wall bottom --
# 3.5 mm of bury plus the full plate seam is plenty for two M2.5 self-tappers.
FEMUR_REAR_TAB_FUSE_Z = 3.5   # mm -- riser burying into the -X wall bottom


def _sandwich_fixed_side(*, end_face_bolts: bool = True,
                         farwall_pad: bool = False,
                         rear_tab: bool = False,
                         wire_exit: bool = True) -> trimesh.Trimesh:
    """The FIXED printed side of a sandwich joint: JUST the STS3215
    front-face mount cradle (4 walls + output-face mount lip + end-face body
    bolts).  The back is now OPEN -- the PASSIVE disc horn rides the servo's
    own internal rear-bearing idler boss (no external 688 bearing or printed
    housing), so nothing printed lives behind the body.  The moving yoke
    wraps the DRIVEN horn on the front (+Z) and the PASSIVE horn on the back
    (-Z), bolting to both identically.

    ``end_face_bolts=False`` drops the 4x M2.5 end-face bolt holes through
    the -X wall (one-piece femur knee cradle -- the fused spar covers that
    wall; see ``_servo_well_solid``).

    ``farwall_pad=True`` adds the external buttress pad on the +X (far)
    wall's outer face (Aug 2026 femur knee field crack; see the
    FEMUR_KNEE_FARWALL_PAD_* constants block for the failure mode and
    the FEA).  The pad is unioned BEFORE the wire-exit diff so the
    harness corridor still pierces it.

    ``rear_tab=True`` adds the Aug 2026 rear retention tab under the open
    back face with 2x M2.5 self-tap holes into the servo's rear molded
    hole pair nearer the -X end (see the FEMUR_REAR_TAB_* block).  Both
    remaining callers use it: the femur knee cradle (Aug 2026) and, since
    Aug 17 2026, the coxa hip cradle (user: "copy that same part to be on
    the coxa link as well").

    ``wire_exit=False`` (Aug 2026 flatten pass, knee cradle; Aug 16 2026
    also the hip cradle -- user: "a weird cutout channel on the opposite
    side ... pointless") skips the L-shaped wire-exit corridor +
    boot-clearance channel entirely: the real STS3215 has NO +X molded
    boot -- its bus cables leave via the BACK-face 5264 ports through
    the sandwich's open back -- so the corridor only pierced solid
    walls for nothing.  NO caller passes wire_exit=True any more; the
    default survives only so the legacy geometry stays reproducible for
    verifier self-tests."""
    body = _servo_well_solid(end_face_bolts=end_face_bolts)
    if farwall_pad:
        pad_x0 = WELL_W / 2.0 - 1.0                       # 1 mm wall bite
        pad_x1 = WELL_W / 2.0 + FEMUR_KNEE_FARWALL_PAD_T
        pad_z1 = WELL_RIM_Z                               # stop at plate underside
        pad = _box((pad_x1 - pad_x0, 2.0 * FEMUR_KNEE_FARWALL_PAD_HALF_Y,
                    pad_z1),
                   center=(0.5 * (pad_x0 + pad_x1), 0.0, pad_z1 / 2.0))
        body = _union(body, pad)
    cuts = [_wire_exit_slot()] if wire_exit else []
    if rear_tab:
        tab_x0 = -WELL_W / 2.0                            # flush with -X wall outer
        tab_y0 = -WELL_D / 2.0                            # flush under the -Y wall
        plate = _box((FEMUR_REAR_TAB_X1 - tab_x0,
                      FEMUR_REAR_TAB_Y1 - tab_y0, FEMUR_REAR_TAB_T),
                     center=(0.5 * (tab_x0 + FEMUR_REAR_TAB_X1),
                             0.5 * (tab_y0 + FEMUR_REAR_TAB_Y1),
                             -FEMUR_REAR_TAB_T / 2.0))
        # Riser burying into the -X wall bottom so the tab is solidly fused
        # (not just a coplanar face seam at z = 0).  Entirely inside the
        # wall's own footprint, so it adds no outside material.
        wall_in_x = -(SERVO_BODY_W / 2.0 + WELL_BODY_CL)
        riser = _box((wall_in_x - tab_x0, FEMUR_REAR_TAB_Y1 - tab_y0,
                      FEMUR_REAR_TAB_FUSE_Z),
                     center=(0.5 * (tab_x0 + wall_in_x),
                             0.5 * (tab_y0 + FEMUR_REAR_TAB_Y1),
                             FEMUR_REAR_TAB_FUSE_Z / 2.0))
        body = _union(body, plate, riser)
        # Relief shelf under the knee clamp cap's dropped tongue (+Y corner):
        # the tongue occupies y >= SERVO_BODY_D/2 - CLAMP_TONGUE_INTERF down
        # to z = -CLAMP_SEAT_DROP; shave the tab 0.25 below it there.
        tongue_y0 = SERVO_BODY_D / 2.0 - CLAMP_TONGUE_INTERF   # +11.9
        relief = _box((abs(wall_in_x - (FEMUR_REAR_TAB_X1 + 0.5)) + 1.0,
                       (FEMUR_REAR_TAB_Y1 + 2.0) - (tongue_y0 - 0.25),
                       CLAMP_SEAT_DROP + 0.25),
                      center=(0.5 * (wall_in_x + FEMUR_REAR_TAB_X1 + 0.5),
                              0.5 * ((tongue_y0 - 0.25) + FEMUR_REAR_TAB_Y1 + 2.0),
                              -(CLAMP_SEAT_DROP + 0.25) / 2.0))
        cuts.append(relief)
        # 2x M2.5 self-tap clearance holes onto the rear molded pair nearer
        # the -X end (body-frame x = SADDLE_CASE_HOLE_X2 + SERVO_OUTPUT_X),
        # each with a head-recess counterbore from the OUTER face up to the
        # z = -3.5 shank plane so the pan head sits flush (Aug 17 2026,
        # user: "room to countersink the screw head ... just like on the
        # side with four holes").
        hx = SADDLE_CASE_HOLE_X2 + SERVO_OUTPUT_X          # -20.3
        for sy in (+1.0, -1.0):
            h = _cyl(SADDLE_CASE_SCREW_OD / 2.0, FEMUR_REAR_TAB_T + 4.0)
            h.apply_translation([hx, sy * SADDLE_CASE_HOLE_Y,
                                 -FEMUR_REAR_TAB_T / 2.0])
            cuts.append(h)
            cb_h = FEMUR_REAR_TAB_HEAD_CB + 1.0            # 1 mm overshoot below
            cb = _cyl(FEMUR_REAR_TAB_HEAD_CB_OD / 2.0, cb_h)
            cb.apply_translation([hx, sy * SADDLE_CASE_HOLE_Y,
                                  -FEMUR_REAR_TAB_SHANK_T - cb_h / 2.0])
            cuts.append(cb)
    return _diff(body, *cuts)


def make_servo_clamp_cap() -> trimesh.Trimesh:
    """Bolt-on clamp cap that closes the OPEN +Y face of a sandwich-joint
    servo cradle (``_servo_well_solid``) and clamps the STS3215 body against
    the cradle's -Y wall.

    Well-local frame (same as ``_servo_well_solid``): origin at the body
    back-face centre, +X = body long axis, +Y = body depth, +Z = output.

    Shape = a T:
      * a full-width FLANGE bar just +Y of the +/-X wall ends, carrying the
        2x M3 clearance holes that thread -Y into the wall-end pilots;
      * a centre TONGUE that reaches -Y into the open face and presses the
        servo body's +Y face (slight interference so the bolts clamp it);
      * a top LIP that laps the body's +Y front-face edge to stop +Z
        pull-out, matching the cradle's -Y / corner lip;
      * a back-face HOOK (Aug 18-19 2026) near the -X end of the +Y edge:
        a 10 mm-wide wall dropping CLAMP_BACK_HOOK_T = 5.5 mm past the back
        plane plus an inboard SHELF whose top face rides FLUSH with the
        back surface (the wire-end band steps up, so a press bite jams
        there), lapping 7.4 mm over the back of the motor toward its
        middle -- an immediate -Z backstop.
        Placed to leave the plug-cable corridor open beside it and to
        clear the yoke pad sweep and the cradle's rear tab (see the
        CLAMP_BACK_HOOK_* constants);
      * a horn-side MINI hook (Aug 19 2026) at the +X end corner: a tiny
        pad (z -1..+1, 1 mm bite) gripping the back corner right at the
        end, with a >= 1.2 mm gap to the Phi 20 passive horn -- the yoke
        pad sweep forbids anything deeper at that end (see the
        CLAMP_HORN_HOOK_* constants);
      * 1 mm 45-deg chamfers on the six vertical edges of the outer +Y
        face (flange ends, hook wall edges, horn pad edges) so the
        sweeping yoke spine cannot catch a corner (see
        CLAMP_YOKE_EDGE_CHAMFER).

    Same part for the hip-pitch (coxa_link) and knee (femur_link's knee
    cradle) joints -- 2 per leg, 12 per robot.  Prints flat on its +Z face.

    Jun 2026 clamp-fit fix: the sandwich cradle's -Y wall sits right at the
    body's -Y face (the WELL_BODY_CL clearance is on the OPEN +Y / cap side
    and laterally, NOT behind the body), so the body's seated +Y face is at
    +SERVO_BODY_D/2.  The OLD tongue stopped at the +Y CAVITY face
    (SERVO_BODY_D/2 + WELL_BODY_CL = +13.1), i.e. WELL_BODY_CL OUTBOARD of the
    body, so the     body could float toward +Y into that gap -> the user's
    measured ~1.5-2 mm rattle.  The tongue now reaches CLAMP_TONGUE_INTERF
    PAST the body's seated +Y face (a press-fit interference, user request),
    so tightening the 2 cap bolts traps the body between the -Y wall and the
    tongue with a snug press fit and zero slop.  (Aug 26 2026: interference
    reduced 1.0 -> 0.5 mm -- at 1 mm the flange visibly bowed around the
    screws when tightened; see the CLAMP_TONGUE_INTERF comment.)"""
    # +Y face of the body, seated against the cradle's -Y wall.
    body_face_y = SERVO_BODY_D / 2.0                       # +12.4 (seated body +Y face)
    wall_end_y = WELL_D / 2.0                              # +16.9 (wall +Y ends)
    flange_y0 = wall_end_y
    flange_y1 = wall_end_y + CLAMP_CAP_T                   # outer face
    cap_z1 = WELL_RIM_Z + WELL_LIP_SLIDE_CL                # tongue top (under lip)
    # Inside-X tightened WELL_INSIDE_X_TIGHTEN total so the tongue + lip match
    # the narrower cavity / drop-in slot (snug-fit; see the CLAMP_* block).
    cav_w = SERVO_BODY_W + 2 * WELL_BODY_CL - WELL_INSIDE_X_TIGHTEN

    # Flange bar (spans the full well width so it covers both wall ends).
    flange = _box((WELL_W, flange_y1 - flange_y0, cap_z1),
                  center=(0.0, 0.5 * (flange_y0 + flange_y1), cap_z1 / 2.0))

    # Centre tongue: reaches the SEATED body face and presses it with
    # CLAMP_TONGUE_INTERF interference so the 2 bolts genuinely clamp it.  The
    # tongue also drops CLAMP_SEAT_DROP PAST the body back-face plane (z=0) so it
    # "comes down lower" -- a -Z backstop lip for a snug seat (user request).
    tongue_y0 = body_face_y - CLAMP_TONGUE_INTERF
    tongue_z0 = -CLAMP_SEAT_DROP
    tongue = _box((cav_w, wall_end_y - tongue_y0, cap_z1 - tongue_z0),
                  center=(0.0, 0.5 * (tongue_y0 + wall_end_y),
                          0.5 * (tongue_z0 + cap_z1)))

    # Top retaining lip that ROOFS the cradle's OPEN +Y drop-in slot.  It
    # tiles FLUSH against the cradle's surviving top plate instead of
    # interpenetrating it: ``_servo_well_solid`` removes its top plate only
    # for y >= cav_d/2 (the lateral drop-in slot), so the cap lip starts
    # exactly at that cut edge and spans out to the wall ends, where the
    # flange takes over.  (It used to overhang -Y to ``body_face_y - 4``
    # = +9.1 mm, burying ~560 mm^3 INSIDE the cradle's own retaining plate
    # -- a real printed-part interference.  The cradle plate already laps
    # the servo's +Y front-face edge for +Z pull-out retention, so the cap
    # only needs to close the slot.)
    cav_d = SERVO_BODY_D + 2 * WELL_BODY_CL
    lip_y0 = cav_d / 2.0 - 0.01           # flush with the cradle plate +Y edge
    lip_y1 = wall_end_y                   # meet the flange at the wall ends
    lip = _box((cav_w, lip_y1 - lip_y0, WELL_H - WELL_RIM_Z),
               center=(0.0, 0.5 * (lip_y0 + lip_y1),
                       0.5 * (WELL_RIM_Z + WELL_H)))

    # Back-face HOOK (Aug 18 2026, user; rev 3): an L over the back of the
    # motor on the -X half of the +Y edge -- a WALL dropping past the back
    # plane plus a SHELF turning inboard UNDER the back face (below the
    # raised centre platform) so the cap positively catches the body.  See
    # the CLAMP_BACK_HOOK_* constants block for the exact keep-outs.  The
    # wall spans y from the tongue's press face out to the flange outer face
    # and buries 1 mm up into both (z +1) so it is solidly fused, not a
    # coplanar seam; the shelf overlaps the wall 0.5 mm in y for the same
    # reason.
    hook_wall = _box((CLAMP_BACK_HOOK_X1 - CLAMP_BACK_HOOK_X0,
                      flange_y1 - tongue_y0, CLAMP_BACK_HOOK_T + 1.0),
                     center=(0.5 * (CLAMP_BACK_HOOK_X0 + CLAMP_BACK_HOOK_X1),
                             0.5 * (tongue_y0 + flange_y1),
                             0.5 * (1.0 - CLAMP_BACK_HOOK_T)))
    shelf_y1 = tongue_y0 + 0.5
    hook_shelf = _box((CLAMP_BACK_HOOK_X1 - CLAMP_BACK_HOOK_X0,
                       shelf_y1 - CLAMP_BACK_HOOK_Y0,
                       CLAMP_BACK_HOOK_T - CLAMP_BACK_HOOK_SHELF_Z),
                      center=(0.5 * (CLAMP_BACK_HOOK_X0 + CLAMP_BACK_HOOK_X1),
                              0.5 * (CLAMP_BACK_HOOK_Y0 + shelf_y1),
                              -0.5 * (CLAMP_BACK_HOOK_SHELF_Z + CLAMP_BACK_HOOK_T)))

    # Horn-side MINI hook (Aug 19 2026, user): a tiny corner pad at the +X
    # end -- z -1 (the proven tongue-lip plane over the yoke pad sweep) up
    # to +1 (press-fit bite), reaching to the cavity edge in x and 3 mm
    # over the back face in y, with a >= 1.2 mm radial gap to the Phi 20
    # passive horn.  See the CLAMP_HORN_HOOK_* constants block.
    horn_hook = _box((cav_w / 2.0 - CLAMP_HORN_HOOK_X0,
                      flange_y1 - CLAMP_HORN_HOOK_Y0,
                      CLAMP_SEAT_DROP + 1.0),
                     center=(0.5 * (CLAMP_HORN_HOOK_X0 + cav_w / 2.0),
                             0.5 * (CLAMP_HORN_HOOK_Y0 + flange_y1),
                             0.5 * (1.0 - CLAMP_SEAT_DROP)))

    # The lip must clear the dia-20 disc horn (centred on +SERVO_OUTPUT_X).
    body = _union(flange, tongue, lip, hook_wall, hook_shelf, horn_hook)
    horn = _cyl(HORN_CLEAR_OPENING_OD / 2.0, (WELL_H - WELL_RIM_Z) * 4.0)
    horn.apply_translation([SERVO_OUTPUT_X, 0.0, WELL_RIM_Z])

    # Yoke-sweep edge chamfers (Aug 19 2026, user): 45 deg cuts along the
    # six vertical edges standing on the outer (+Y) face, so the sweeping
    # yoke spine cannot catch a sharp corner if it grazes.  Each cutter is
    # a square prism rotated 45 deg about Z, centred on the edge line; the
    # hook-edge cutters stop at z = 0 so the flat flange face above keeps
    # its full bed contact.  See CLAMP_YOKE_EDGE_CHAMFER.
    c = CLAMP_YOKE_EDGE_CHAMFER
    chamfers = []
    for (ex, ez0, ez1) in (
        (-WELL_W / 2.0,       -1.0, cap_z1 + 1.0),   # flange -X end (full height)
        (WELL_W / 2.0,        -1.0, cap_z1 + 1.0),   # flange +X end (full height)
        (CLAMP_BACK_HOOK_X0,  -CLAMP_BACK_HOOK_T - 1.0, 0.0),  # hook wall -X edge
        (CLAMP_BACK_HOOK_X1,  -CLAMP_BACK_HOOK_T - 1.0, 0.0),  # hook wall +X edge
        (CLAMP_HORN_HOOK_X0,  -CLAMP_SEAT_DROP - 1.0, 0.0),    # horn pad -X edge
        (cav_w / 2.0,         -CLAMP_SEAT_DROP - 1.0, 0.0),    # horn pad +X edge
    ):
        prism = _box((c * np.sqrt(2.0), c * np.sqrt(2.0), ez1 - ez0),
                     center=(0.0, 0.0, 0.5 * (ez0 + ez1)))
        prism.apply_transform(rotation_matrix(np.pi / 4.0, [0, 0, 1]))
        prism.apply_translation([ex, flange_y1, 0.0])
        chamfers.append(prism)

    # 2x M3 clearance holes through the flange (axis Y) onto the wall pilots.
    # The (x, z) centres come from ``servo_clamp_bolt_centres`` -- the SAME
    # source ``_servo_well_solid`` reads for the cradle pilots -- so the cap
    # holes can never drift out of coaxiality with the pilots they thread
    # into (guarded by check_clamp_cap_alignment).
    cuts = [horn] + chamfers
    flange_mid_y = 0.5 * (flange_y0 + flange_y1)
    for (bx, bz) in servo_clamp_bolt_centres():
        hole = _cyl(CLAMP_BOLT_CLEAR_OD / 2.0, (flange_y1 - flange_y0) + 4.0)
        hole.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))   # axis -> Y
        hole.apply_translation([bx, flange_mid_y, bz])
        cuts.append(hole)
        # Head counterbore: opens at the flange OUTER (+Y) face and runs
        # CLAMP_HEAD_CB_DEPTH inward (-Y) so the M3 SHCS head recesses FLUSH
        # with the face (head no longer brushes the sweeping yoke).  The head
        # bears on the shoulder at flange_y1 - CLAMP_HEAD_CB_DEPTH; +1 mm of
        # over-length cuts the outer mouth cleanly.
        cb_h = CLAMP_HEAD_CB_DEPTH + 1.0
        cbore = _cyl(CLAMP_HEAD_CB_OD / 2.0, cb_h)
        cbore.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))  # axis -> Y
        cbore.apply_translation([bx, flange_y1 - CLAMP_HEAD_CB_DEPTH + cb_h / 2.0, bz])
        cuts.append(cbore)
    return _diff(body, *cuts)


def _leg_tube_socket_x(x_mouth: float, sock_z: float, *, direction: int = 1,
                       length: float = None, pin_inset: float = None):
    """A Phi LEG_TUBE_OD carbon-tube socket extending along +/-X from
    ``x_mouth`` at height ``sock_z`` (joint-local).  ``direction`` = +1
    (toward +X) or -1 (toward -X).  Returns ``(boss, bore, pin)``: union
    the boss into the part, then diff the bore + transverse retention-pin
    cross-hole (epoxy bond + dia-2.5 pin retains the tube).

    ``length`` overrides the default boss length; pass the matching
    ``pin_inset`` so the transverse pin still lands at the boss
    mid-engagement (not past its end).  (Since the Jul 2026 one-piece femur
    only the TIBIA uses tube sockets, at the default size.)"""
    sock_len = length if length is not None else (LEG_TUBE_SOCKET_DEPTH + 6.0)
    inset = pin_inset if pin_inset is not None else LEG_TUBE_PIN_INSET
    s = float(direction)
    Rx = rotation_matrix(np.pi / 2.0, [0, 1, 0])   # cyl axis Z -> X
    boss = _cyl(LEG_TUBE_OD / 2.0 + LEG_TUBE_SOCKET_WALL, sock_len)
    boss.apply_transform(Rx)
    boss.apply_translation([x_mouth + s * sock_len / 2.0, 0.0, sock_z])
    bore = _cyl(LEG_TUBE_OD / 2.0 + LEG_TUBE_SOCKET_CLEAR, sock_len * 2)
    bore.apply_transform(Rx)
    bore.apply_translation([x_mouth + s * sock_len, 0.0, sock_z])
    pin = _cyl(LEG_TUBE_PIN_OD / 2.0, (LEG_TUBE_OD / 2.0 + LEG_TUBE_SOCKET_WALL) * 4)
    pin.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))   # axis Y
    pin.apply_translation([x_mouth + s * inset, 0.0, sock_z])
    return boss, bore, pin


# Yoke arm geometry (joint-local).
_YOKE_ARM_T = 4.0
_YOKE_ARM_Y = 12.0
# Jun 2026 (user: "add 1mm to each side of the bump that goes out to connect to
# the servos ... on each of the yokes").  The horn-contact pad is the cylindrical
# "bump" on each clevis arm that reaches inboard to seat on (and bolt to) the
# disc horn.  Grow it YOKE_PAD_RADIAL_GROW per side -- i.e. +1 mm of material on
# every radial face, so the pad diameter grows by 2x this.  The pad OD goes from
# (DISC_HORN_OD - 1) = 19 mm to 21 mm; it still clears the fixed cradle's
# Phi HORN_CLEAR_OPENING_OD (24 mm) running bore by ~1.5 mm/side as it sweeps, and
# the reach-down depth (seating against the disc face) is unchanged, so screw
# engagement and every frozen joint axis / link length stay put.  Applies to
# BOTH yokes (hip femur yoke + knee tibia yoke share this component).
YOKE_PAD_RADIAL_GROW = 1.0   # mm per side added to the horn-contact pad radius
# Spine sits OUTBOARD of the disc-horn sweep: the Phi HORN_STACK_VOID_R
# (= 18.5 mm) horn envelope about the output axis (joint-local x =
# SERVO_OUTPUT_X = 12.5) reaches x = 31 mm, and the fixed-side servo well
# half-width is WELL_W/2 = 29 mm; starting the spine at x = 32 keeps the
# moving yoke clear of BOTH the horn-stack clearance probe (5b) and the
# fixed bracket (self-collision), while the arms still cover the Phi 14
# bolt circle (x in [5.5, 19.5]) on the driven horn.
# Jun 2026 ROM-clearance fix (user: "they bang into the cap... need to be a
# little longer").  At hip femur pitch ~ -30..-65 deg (and the mirrored knee
# pose) the mid-height connecting WEB (spine) swept into the fixed servo clamp
# cap (~400 mm^3 clash at -40 deg).  Moving the web +4 mm OUTBOARD (vacating the
# old x[38,42] band where it banged the cap) clears the full ROM with margin.
# The arms extend to the new web; the CF-tube socket MOUTH stays at the old face
# (_YOKE_SOCKET_X) so the femur tube span is unchanged.
_YOKE_ARM_X0, _YOKE_ARM_X1 = 2.0, 46.0
_YOKE_SOCKET_X = 42.0                       # CF-tube socket mouth (femur span frozen)
_YOKE_SPINE_X0, _YOKE_SPINE_X1 = 42.0, 46.0
# Aug 2026, field crack #1 follow-up (user: "I dont think you actually
# thickened the front of the tibia knee yoke where it also broke"): the
# tibia crack photo shows VERTICAL splits through the 4 mm spine PLATE
# (the yoke's flat front face), above and below the tube socket, out to
# the arm edges -- the Ø18 socket-boss thickening (LEG_TUBE_SOCKET_WALL
# 3 -> 5) fattened the boss but left the plate it tees into untouched.
# Mechanism: a motion-plane foot strike bends the tube about the pivot
# axis and the boss pries the plate top/bottom, splitting it along the
# vertical layer seams (same thin-plate T-junction failure as the femur
# far wall).  Fix: DOUBLE the spine plate for the TIBIA yoke only
# (4 -> 8 mm, extending OUTBOARD x 46 -> 50 -- inboard is forbidden by
# the Jun 2026 clamp-cap ROM refit; outboard is measured-free: the
# closed-yoke diag maps the knee joint clear to r ~ 39.5+ in this
# sector, and the thicker plate's swept corner reaches r ~ 39.4).
# CalculiX before/after (2g foot strike, motion-plane): see
# artifacts/strength/tibiayoke_*.  The femur hip yoke keeps the stock
# 4 mm spine: its spar passes through the plate's full thickness and
# that junction never cracked.  Bore and socket mouth (x = 42) are
# unchanged, so the CF tube cut length stays put.  (The retention-pin
# cross-hole at x = 49 was later REMOVED entirely -- epoxy-only, see the
# LEG_TUBE_PIN_OD block.)
TIBIA_YOKE_SPINE_PAD_T = 4.0   # mm added OUTBOARD to the tibia yoke spine
# Socket height = mid-plane of the yoke (between the top + bottom arms).
_YOKE_BOT_Z0 = JOINT_HORN_BOT_Z - _YOKE_ARM_T                          # -11
_YOKE_TOP_Z1 = JOINT_HORN_TOP_Z + _YOKE_ARM_T                          # 45.3
JOINT_SOCKET_Z = 0.5 * (_YOKE_BOT_Z0 + _YOKE_TOP_Z1)                   # ~17.15

# ---------------------------------------------------------------------------
# REINFORCED-YOKE TEST VARIANT (Aug 2026 v3, after the bench yoke-bend
# incident).  The weak member is the SPINE -- the plate connecting the two
# clevis arms (user, after v2: "you dont need to reinforce the arms, you
# need to reinforce the plate connecting the arms").  The evidence agrees:
# the arms are bolted flat onto the disc horns (well supported), the load
# path BETWEEN them is the 4 mm spine plate, twisting/bending of which is
# exactly what lets the arms go non-parallel -- and the tibia yoke's spine
# plate is the part that actually CRACKED in the field (see
# TIBIA_YOKE_SPINE_PAD_T above).  v3 therefore:
#   * DOUBLES the spine plate to YOKE_SPINE_T_REINF = 8 mm total on EVERY
#     yoke (the tibia already ships 8 mm from the crack fix; the variant
#     brings the femur hip yoke to match).  Growth is OUTBOARD only
#     (x 46 -> 50) -- inboard is forbidden by the Jun 2026 clamp-cap ROM
#     refit (the spine face at x 42 clears the swept cap by 0.39 mm).
#     Plate bending/torsion stiffness ~ t^3 => ~8x.
#   * KEEPS the triangular root fillets at the arm/spine inner corners --
#     they reinforce the plate-to-arm junction where the yield started.
#     Z-legs differ per corner because the measured swept gaps differ:
#     below the BOTTOM arm face (-7) the servo rear boss tops out at -4
#     (3 mm gap -> LZ 2.5); above the TOP arm face (41.3) the HIP's coxa
#     deck plate sweeps to z 39.3 across r 14-30 (2 mm gap -> LZ 1.5).
#   * Arms stay stock 4 mm (v2's 7 mm arms retired: they stiffened the
#     wrong member and needed +3 mm horn screws).
# History: v1 (outboard drum "closed yoke", ~23 g/joint) and v2 (7 mm
# arms) are retired; each stiffened something other than the plate that
# fails.
#
# Where reinforcement may live was measured, not guessed
# (tools/_closed_yoke_diag.py sweeps the REAL fixed-side meshes through the
# joint ROM in the yoke frame, +5 deg margin each end):
#   * Outboard of the spine, |phi| <= ~45 deg about +X is free out to
#     r ~ 39.5 (hip-limited); the 8 mm plate's corners reach r 39.4 --
#     the same envelope the tibia crack-fix pad already occupies.
#   * A hoop closing the open (-X) end is IMPOSSIBLE at any radius (the
#     incoming femur spar / coxa arm sweeps that half-space out to
#     r ~ 90 mm over the ROM); side webs near the arm edges are blocked by
#     the clamp-cap corner sweep (r <= 29.1 mm across phi +/-105 deg).
#
# Enable with env HEX_YOKE_REINFORCED=1 (test builds) or per-call
# reinforced=True.  Assembly/hardware unchanged (stock screws fit).
# Aug 2026: the SHIPPED femur_link now also carries the 8 mm spine
# (FEMUR_YOKE_SPINE_PAD_T, user request), so for the femur yoke this
# variant only adds the root fillets.
YOKE_REINFORCED   = os.environ.get("HEX_YOKE_REINFORCED", "") == "1"
YOKE_SPINE_T_REINF  = 8.0   # mm TOTAL spine plate thickness (stock 4)
_YOKE_FILLET_LX     = 6.0   # mm fillet leg along the arm (inboard from spine)
_YOKE_FILLET_LZ_TOP = 1.5   # mm z-leg, top corner (hip coxa deck at 39.3)
_YOKE_FILLET_LZ_BOT = 2.5   # mm z-leg, bottom corner (servo rear boss at -4)


def _yoke_root_fillet(z_face: float, d: float, lz: float) -> trimesh.Trimesh:
    """Triangular fillet prism at one arm/spine inner corner, full arm
    width (see the REINFORCED-YOKE note above).  ``z_face`` is the arm's
    inner face plane; ``d`` = +1 for the top arm (fillet hangs below it),
    -1 for the bottom arm (mirrored); ``lz`` is the corner's z-leg.
    Triangle verts in x-z: (spine_x0 - LX, z_face), (spine_x0, z_face),
    (spine_x0, z_face - d*lz)."""
    lx = _YOKE_FILLET_LX
    x1 = _YOKE_SPINE_X0
    block = _box((lx, 2 * _YOKE_ARM_Y, lz),
                 center=(x1 - lx / 2.0, 0.0, z_face - d * lz / 2.0))
    # Hypotenuse: half-space cut through (x1-LX, z_face) and
    # (x1, z_face - d*LZ), removing the corner block's far half.
    cutter = _box((400.0, 400.0, 400.0), center=(0.0, 0.0, -d * 200.0))
    cutter.apply_transform(rotation_matrix(
        d * np.arctan2(lz, lx), [0, 1, 0]))
    cutter.apply_translation([x1 - lx, 0.0, z_face])
    return _diff(block, cutter)


def _sandwich_moving_yoke(*, tube_socket: bool = True,
                          socket_length: float = None,
                          socket_pin_inset: float = None,
                          socket_pin: bool = True,
                          reinforced: bool = None,
                          spine_extra_t: float = 0.0,
                          pad_extra_reach: float = 0.0) -> trimesh.Trimesh:
    """The MOVING printed link of a sandwich joint: a C-clevis straddling
    the fixed servo/housing stack.

    ``reinforced`` (default: the HEX_YOKE_REINFORCED env flag) builds the
    Aug 2026 v3 test reinforcement -- spine plate grown OUTBOARD to
    YOKE_SPINE_T_REINF total + root fillets at the arm/spine inner
    corners (see the REINFORCED-YOKE note above).

    ``socket_length`` / ``socket_pin_inset`` override the CF-tube socket size
    (historical: the two-part femur used SHORT sockets; since the Jul 2026
    one-piece femur only the tibia knee yoke sockets a tube, at the default
    size).

    ``pad_extra_reach`` deepens BOTH arms' reach-down pads by that many mm
    past the nominal disc faces (Aug 2026 bench fit: the tibia knee yoke
    AND the femur hip yoke both pass YOKE_PAD_EXTRA_REACH = 0.5 to close
    a measured ~1 mm total clevis gap on each).

    ``spine_extra_t`` thickens the spine plate OUTBOARD (+X) by that many
    mm (Aug 2026 tibia spine-plate field crack; the tibia knee yoke passes
    TIBIA_YOKE_SPINE_PAD_T and the femur hip yoke passes
    FEMUR_YOKE_SPINE_PAD_T -- both 8 mm total plates).

    ``socket_pin=False`` (Aug 2026, user: epoxy-only tube retention) drops
    the transverse Phi 2.6 retention-pin cross-hole from the tube socket.

    - TOP arm seats on the DRIVEN disc-horn top (z = JOINT_HORN_TOP_Z) and
      bolts to it with 4x M3 on the Phi DISC_HORN_BOLT_PCD circle.
    - BOTTOM arm is the MIRROR of the top: it seats on the PASSIVE disc horn
      (z = JOINT_HORN_BOT_Z) and bolts to it with the SAME 4x M3 pattern +
      reach-down pad.  Both arms clamp a disc horn identically; the passive
      horn rides the servo's rear idler boss (no external bearing).
    - +X spine ties the arms and (optionally) sockets the Phi 8 CF tube
      (epoxy bond; ``socket_pin`` adds the legacy pin cross-hole).
    """
    if reinforced is None:
        reinforced = YOKE_REINFORCED
    arm_t = _YOKE_ARM_T

    def _disc_arm(seat_z, arm_dir, reach):
        """One disc-clamping clevis arm.  ``seat_z`` is the seat plane (the arm
        body's inner face); ``arm_dir`` is +1 for the TOP arm (pad reaches DOWN
        -Z to the horn, arm body +Z) and -1 for the BOTTOM arm (mirrored).
        ``reach`` is the reach-down-pad depth bridging the seat plane to the REAL
        disc mating face.  Symmetric refit: BOTH arms use the SAME reach
        (YOKE_ARM_PAD + YOKE_SEAT_INTERF) so one screw bolts each side -- the
        driven seat (JOINT_HORN_TOP_Z) and the moved passive seat
        (JOINT_HORN_BOT_Z) are each one pad above/below their disc face, and the
        pad reaches YOKE_SEAT_INTERF PAST the rigid disc for a snug clamp."""
        d = float(arm_dir)
        # Arm body: a slab on the OUTBOARD side of the seat plane.
        arm = _box((_YOKE_ARM_X1 - _YOKE_ARM_X0, 2 * _YOKE_ARM_Y, arm_t),
                   center=(0.5 * (_YOKE_ARM_X0 + _YOKE_ARM_X1), 0.0,
                           seat_z + d * arm_t / 2.0))
        # Reach-down pad: the real aluminium disc horn is only DISC_HORN_H
        # (2 mm), so its mating face sits ``reach`` INBOARD of the frozen seat
        # plane.  A concentric pad bridges the arm to the real horn so the yoke
        # actually bolts onto the disc -- every joint axis / link length stays
        # frozen.  Sized to the disc OD (+ YOKE_PAD_RADIAL_GROW per side, user
        # request) so it still clears the fixed cradle's running bore as it
        # rotates.
        pad = _cyl(DISC_HORN_OD / 2.0 - 0.5 + YOKE_PAD_RADIAL_GROW, reach)
        pad.apply_translation([SERVO_OUTPUT_X, 0.0,
                               seat_z - d * reach / 2.0])
        arm = _union(arm, pad)
        cuts = []
        for (hx, hy) in _disc_horn_bolt_centres():
            h = _cyl(DISC_HORN_BOLT_OD / 2.0, arm_t * 4 + 2 * reach)
            h.apply_translation([hx, hy, seat_z + d * arm_t / 2.0])
            cuts.append(h)
        # Central spline-collar clearance through the reach-down pad face.
        collar = _cyl(DISC_HORN_COLLAR_OD / 2.0 + 0.25, DISC_HORN_COLLAR_DEPTH + 1.0)
        collar.apply_translation([SERVO_OUTPUT_X, 0.0,
                                  (seat_z - d * reach)
                                  + d * (DISC_HORN_COLLAR_DEPTH + 1.0) / 2.0])
        cuts.append(collar)
        # Through-centre clearance: the servo spline / rear-boss screw clamps
        # plastic + horn to the servo (not horn-only retention).  Head sits on
        # the arm outer face; shank passes pad → horn → servo.  Used by both
        # the tibia knee yoke and the femur hip yoke (same _disc_arm).
        centre = _cyl(HORN_CENTRE_OD / 2.0, arm_t * 4 + 2 * reach + 4.0)
        centre.apply_translation([SERVO_OUTPUT_X, 0.0,
                                  seat_z + d * arm_t / 2.0])
        cuts.append(centre)
        return _diff(arm, *cuts)

    reach = YOKE_ARM_PAD + YOKE_SEAT_INTERF + pad_extra_reach      # symmetric, snug
    top = _disc_arm(JOINT_HORN_TOP_Z, +1, reach)                   # driven (flush front)
    bot = _disc_arm(JOINT_HORN_BOT_Z, -1, reach)                   # passive (rear boss)
    top_z0 = JOINT_HORN_TOP_Z

    spine_x1 = _YOKE_SPINE_X1 + spine_extra_t
    if reinforced:
        # Spine plate to >= YOKE_SPINE_T_REINF total, growing OUTBOARD
        # (the tibia crack-fix pad already puts the tibia yoke at 8 mm;
        # this brings the femur hip yoke to the same plate).
        spine_x1 = max(spine_x1, _YOKE_SPINE_X0 + YOKE_SPINE_T_REINF)
    spine = _box((spine_x1 - _YOKE_SPINE_X0, 2 * _YOKE_ARM_Y,
                  (top_z0 + arm_t) - _YOKE_BOT_Z0),
                 center=(0.5 * (_YOKE_SPINE_X0 + spine_x1), 0.0,
                         0.5 * (_YOKE_BOT_Z0 + top_z0 + arm_t)))

    parts = [top, bot, spine]
    if reinforced:
        parts.append(_yoke_root_fillet(JOINT_HORN_TOP_Z, +1.0,
                                       _YOKE_FILLET_LZ_TOP))
        parts.append(_yoke_root_fillet(JOINT_HORN_BOT_Z, -1.0,
                                       _YOKE_FILLET_LZ_BOT))
    if tube_socket:
        boss, bore, pin = _leg_tube_socket_x(
            _YOKE_SOCKET_X, JOINT_SOCKET_Z,
            length=socket_length, pin_inset=socket_pin_inset)
        parts.append(boss)
        cuts = [bore] + ([pin] if socket_pin else [])
        return _diff(_union(*parts), *cuts)
    return _union(*parts)


# ---------------------------------------------------------------------------
# ONE-PIECE printed femur (Jul 2026) -- yoke + spar + knee bracket, one body
# ---------------------------------------------------------------------------
# History: the femur segment started as a cut Phi 8 CF tube, then became a
# separate printed Phi 7.8 drop-in strut (Jun 2026, user: "the femur tube is
# so ridiculously short and cutting the carbon fiber is kind of a pain"),
# then the strut was fused into the hip yoke leaving a slip-fit + single-pin
# joint at the knee bracket (Jul 2026 merge #1, user: "combine femur hip yoke
# and femur strut into one stronger femur hip yoke part").  Jul 2026 merge #2
# (user: "combine the femur knee bracket with the femur hip yoke and make
# that connection very solid (cylinder size of our diameter)"): the knee
# bracket is fused in too, so the WHOLE FEMUR is one printed part --
# ``femur_link`` -- and the yoke-to-bracket connection is a SOLID
# Phi FEMUR_SPAR_OD cylinder spanning the full inter-well gap.  No socket
# bore, no slip fit, no retention pin: the printed cross-section between
# the joints is solid Phi FEMUR_SPAR_OD (18 since Aug 2026) everywhere.
# Assembly is unaffected -- both ends still attach via bolted disc horns
# (hip) and the drop-in servo + clamp caps (knee).  The longer 130 mm
# tibia stays a CF tube between two separate printed fittings.
FEMUR_SPAR_LEN = FEMUR_LENGTH - _YOKE_SOCKET_X - WELL_W / 2.0
                 # yoke-spine face to knee-bracket well wall (~19 mm).
FEMUR_SPAR_OD  = LEG_TUBE_OD + 10.0
                 # Phi 18 (Aug 2026, user: "make the connector coming out of
                 # it thicker").  Was Phi 14 = the old socket-boss OD from the
                 # Jul 2026 merge; 18/14 in solid bending stiffness ~ (18/14)^4
                 # = 2.7x.  Still SOLID, no bore.  Clearance: the exposed spar
                 # (x >= the 8 mm spine's outer face at 50) never dips below
                 # planar r 37.5 from the hip axis vs the clamp-cap sweep at
                 # r <= 29.1, and Phi 18 (z 8.15..26.15, y +/-9) stays inside
                 # the knee well wall's y +/-16.9 / z 0..34.3 face.
_FEMUR_SPAR_WALL_BITE = 1.0   # mm the spar extends INTO the knee well wall
                              # so the boolean union is volumetric (not a
                              # coincident-face contact); stays well clear of
                              # the servo cavity behind the 6 mm wall.
# Aug 2026: field crack #2 -- a femur knee well crack one week after the
# identical tibia socket-wall crack.  (First read as radiating from the
# spar landing; the user corrected that -- the split actually runs along
# the FAR wall, see the FEMUR_KNEE_FARWALL_PAD_* block below, which is
# the fix for the crack itself.)  The spar junction cones kill the sharp
# 90-deg T-junctions where the spar meets the yoke spine and the knee
# well wall: torque-reaction FEA shows the near-wall junction is the
# highest-stressed region of the whole part under a 2g foot strike --
# every bending moment funnels through that re-entrant corner (Kt ~ 3)
# across the FDM layer seams (the part prints spar-horizontal).  Each
# cone: base radius buried _FEMUR_SPAR_WALL_BITE into the spine / wall,
# apex exactly at the OPPOSITE junction face, so it tapers past the
# Phi 18 spar ~3 mm out (a small ~73-deg flare) and NEVER pokes through
# the far wall / spine into the clamp-cap or clevis airspace.
# History of the per-end radii:
# * KNEE end R = 12 since the first Aug 2026 study (base on the 9 mm well
#   wall backed by the whole well box; FEA: junction stress -31%).
# * HIP end was R = 0 while the spine was the stock 4 mm plate: FEA
#   showed a stiff cone rim prying that thin plate (7.7 MPa hot zone at
#   the rim, worse than the 5.3 MPa corner it replaced).  Later in
#   Aug 2026 the user asked for the yoke bar to be thickened and a small
#   taper "on both sides": the femur spine is now a DOUBLED 8 mm plate
#   (FEMUR_YOKE_SPINE_PAD_T), which removes the thin-plate objection, so
#   the hip cone is reinstated at the same R = 12 -- re-checked by FEA
#   against the 8 mm plate (see artifacts/strength/femur_tq_*).
FEMUR_GUSSET_R_HIP  = 12.0  # mm -- cone base radius at the hip spine face
FEMUR_GUSSET_R_KNEE = 12.0  # mm -- cone base radius at the knee well wall

# Aug 2026 (user: "thicken the bar between the two arms ... on the femur
# link"): the femur hip yoke's spine plate -- the bar connecting the two
# clevis arms -- is DOUBLED 4 -> 8 mm, growing OUTBOARD x 46 -> 50 exactly
# like the tibia yoke's crack-fix pad (TIBIA_YOKE_SPINE_PAD_T) and the v3
# reinforced-yoke variant (YOKE_SPINE_T_REINF); the same _closed_yoke_diag
# survey that cleared those clears this (outboard |phi| <= 45 deg free to
# r ~ 39.5; the 8 mm plate's swept corner reaches r ~ 39.4).  The spar and
# both gusset cones root on the new x = 50 outer face.
FEMUR_YOKE_SPINE_PAD_T = 4.0   # mm added OUTBOARD to the femur yoke spine

# Aug 2026, crack-location correction (user: "the crack was against the
# other wall"): the field crack is NOT at the spar landing -- the photo
# shows a layer-seam split just under the clamp-cap seam wrapping the
# FAR (+X well-local) wall corner, the wall OPPOSITE the spar.  A
# torque-reaction FEA (knee moment = F_impact * TIBIA_LENGTH ~ 1.8 N-m
# applied as the servo's bearing couple on the well walls + cap-pilot
# pull, artifacts/strength/femur_tq_*) shows why: pure knee torque only
# puts ~2 MPa in that wall, but a LATERAL foot catch reacts its
# out-of-plane moment as an ~80 N bearing couple of the servo case
# against the far wall's TOP band plus a pull against the retaining
# lip -- 5..17 MPa at the wall top / plate junction, marginal against
# PETG LAYER-ADHESION strength (well under the 22.7 MPa derated bulk
# yield, but the load pries straight across the seams, and the Phi 2.5
# cap-bolt pilot pre-stresses the same section).  Fix: an external
# BUTTRESS PAD (a doubler) on the far wall's outer face over the WALL
# BAND ONLY, z in [0, WELL_RIM_Z] -- STOPPING at the plate underside.
#   * FEA-tuned height: a first full-height pad (flush with the plate
#     top) halved the wall stress but hard-coupled the lateral couple
#     into the 4 mm top plate -- its drop-in-slot edge strip went +65%
#     (p95 9.8 -> 16.3 MPa), the same stiff-addition-moves-the-hot-spot
#     lesson as the abandoned hip-end spar cone.  Stopping the pad at
#     the plate underside keeps the wall doubled (far-wall mid-section
#     p95 1.48 -> 0.81 MPa, region peak 17 -> 9.7; pilot vicinity -70%;
#     lateral-case global peak 36 -> 16 MPa vs 22.7 derated yield) with
#     the plate strip unchanged from baseline.
#   * T = 3.5 (1 mm bite into the wall for a volumetric union).
#   * half-Y: FULL wall width (WELL_D/2 = 16.9) since the Aug 2026
#     flatten pass (user: "this top can just be a flat wall with no
#     weird cutouts") -- the outer face is now ONE flat rectangle over
#     the whole wall band instead of a raised 26 mm pad ("two layers of
#     a wall").  Clearances re-checked at full width: the pad's swept
#     corner (r ~28.3 about the knee axis) stays inside the cap
#     flange's existing r ~29.1 sweep envelope, and its +Y face is
#     exactly COPLANAR with the wall end face the cap flange seats on
#     (no overlap; mesh_overlap guards it).  The z span still STOPS at
#     the plate underside (WELL_RIM_Z) -- that is the FEA lesson above,
#     not a styling choice: the remaining small step under the 4 mm top
#     plate must stay.
# The same Aug 2026 pass DELETED the wire-exit L-corridor + boot channel
# from the knee cradle (``_sandwich_fixed_side(wire_exit=False)``): the
# real STS3215 exits its bus cables via the BACK-face 5264 ports through
# the sandwich's open back (see the WIRE_BOOT_* legacy note), so the
# DS3225-era corridor pierced the crack wall for nothing.
# Knee cradle only -- the coxa hip cradle keeps the stock wall (its far
# wall never cracked; revisit if it does).
FEMUR_KNEE_FARWALL_PAD_T      = 3.5   # mm proud of the far wall outer face
FEMUR_KNEE_FARWALL_PAD_HALF_Y = WELL_D / 2.0  # 16.9 -- full flat wall face


def _femur_fused_spar() -> trimesh.Trimesh:
    """The solid femur spar of the one-piece femur (hip-joint-local frame),
    plus its two junction gussets (Aug 2026, see FEMUR_GUSSET_R above).

    A Phi FEMUR_SPAR_OD solid cylinder at z = JOINT_SOCKET_Z running from
    the hip yoke's spine face (x = _YOKE_SOCKET_X, embedding into the
    x[42,50] doubled spine) to _FEMUR_SPAR_WALL_BITE past the knee
    bracket's well wall face (x = _YOKE_SOCKET_X + FEMUR_SPAR_LEN), fusing
    yoke and bracket into one printed body.  A cone at each end (base at
    the spine / wall, apex at the opposite face) fillets both
    T-junctions."""
    Rx = rotation_matrix(np.pi / 2.0, [0, 1, 0])   # cyl axis Z -> X
    length = FEMUR_SPAR_LEN + _FEMUR_SPAR_WALL_BITE
    spar = _cyl(FEMUR_SPAR_OD / 2.0, length)
    spar.apply_transform(Rx)
    spar.apply_translation([_YOKE_SOCKET_X + length / 2.0, 0.0,
                            JOINT_SOCKET_Z])
    spine_face = _YOKE_SPINE_X1 + FEMUR_YOKE_SPINE_PAD_T   # 50 (8 mm plate)
    wall_face = _YOKE_SOCKET_X + FEMUR_SPAR_LEN       # 58.3 (well wall outer)
    bite = _FEMUR_SPAR_WALL_BITE
    cone_h = (wall_face - spine_face) + bite          # apex at opposite face
    gussets = []
    for base_x, sgn, base_r in ((spine_face - bite, +1.0, FEMUR_GUSSET_R_HIP),
                                (wall_face + bite, -1.0, FEMUR_GUSSET_R_KNEE)):
        if base_r <= 0.0:
            continue
        g = trimesh.creation.cone(radius=base_r, height=cone_h,
                                  sections=CYL_SECTIONS)
        g.apply_transform(rotation_matrix(sgn * np.pi / 2.0, [0, 1, 0]))
        g.apply_translation([base_x, 0.0, JOINT_SOCKET_Z])
        gussets.append(g)
    return _union(spar, *gussets)


def _servo_envelope() -> trimesh.Trimesh:
    """Return the bounding-volume of the FEETECH STS3215 servo.

    Local frame (matches the way the servo is *used*):
        +X = body long axis (output shaft offset toward +X)
        +Y = body short axis (depth)
        +Z = output-shaft direction (out of the FRONT face)
        Origin at the centre of the body's BACK (idler, -Z) face.

    The STS3215 is a tab-less brick: it mounts via 4x M2.5 case-face
    holes (see ``servo_mount_hole_centres``), not protruding ears, so
    there are no tabs in this envelope -- just the body, the output hub
    + spline on +Z, a small idler hub on -Z, and the bus-cable exit."""
    body = _box((SERVO_BODY_W, SERVO_BODY_D, SERVO_BODY_H),
                center=(0, 0, SERVO_BODY_H / 2.0))

    # Output spline/coupling on the FRONT (+Z) face.  Jun 2026 flush-output
    # correction (user, repeated): the STS3215 output does NOT protrude beyond
    # the body front face -- the 25T spline is essentially flush and the Phi20
    # aluminium disc horn (a SEPARATE part, ``make_disc_horn``) RECESSES onto it,
    # seating FLUSH on the front face.  The old model drew the coupling/spline
    # SERVO_OUTPUT_H+ proud, which (a) contradicted the hardware and (b) would
    # now ram the flush-seated disc horn.  Model the output as a flush boss whose
    # TOP is the front face (z = SERVO_BODY_H), extending DOWN into the body, so
    # it never pokes up into the disc horn or the mount-plate bore.
    OUT_BOSS_H = 2.0
    coupling = _cyl((SERVO_OUTPUT_BORE_OD - 1.0) / 2.0, OUT_BOSS_H)
    coupling.apply_translation([SERVO_OUTPUT_X, 0, SERVO_BODY_H - OUT_BOSS_H / 2.0])
    spline = _cyl(SERVO_SPLINE_OD / 2.0, OUT_BOSS_H)
    spline.apply_translation([SERVO_OUTPUT_X, 0, SERVO_BODY_H - OUT_BOSS_H / 2.0])

    # Idler coupling on the BACK (-Z) face (open air -- the cradle is open
    # behind the body), same axis.
    idler = _cyl((SERVO_OUTPUT_BORE_OD - 1.0) / 2.0, 1.5)
    idler.apply_translation([SERVO_OUTPUT_X, 0, -0.75])

    # Legacy +X stand-in "boot" REMOVED (Aug 2026 flatten pass): the real
    # STS3215 has NO DS3225-style molded boot on the +X end -- its dual
    # 5264 ports are on the BACK face (−Z), centre/−X half (STS3215_PORT_*),
    # and harness routes attach there.  The WIRE_BOOT_* constants survive
    # only for the legacy corridor cuts still present in the hip/yaw
    # cradles (harmless routing room); the knee cradle's far wall is now
    # solid, which a modeled boot box would have interpenetrated.
    # Tiny visual markers at the real port cluster (back face, −X half).
    port_a = _box((6.0, 4.0, 2.0),
                  center=(STS3215_PORT_X_MM, 3.5, STS3215_PORT_Z_MM))
    port_b = _box((6.0, 4.0, 2.0),
                  center=(STS3215_PORT_X_MM, -3.5, STS3215_PORT_Z_MM))
    return _union(body, coupling, spline, idler, port_a, port_b)


def _servo_pocket() -> trimesh.Trimesh:
    """Return the void volume of an STS3215 (slightly oversized, slip
    fit) for cutting into a bracket.  Same local frame as
    `_servo_envelope`.

    Includes:
        - body cavity (0.4 mm clearance on every face)
        - output-coupling clearance bore through the front plate
        - 4 x M2.5 case-mount clearance holes through the front plate

    Returns a single union mesh; pass to _diff(bracket, _servo_pocket())."""
    CL = 0.4   # mm clearance
    body = _box((SERVO_BODY_W + 2 * CL,
                 SERVO_BODY_D + 2 * CL,
                 SERVO_BODY_H + 2 * CL),
                center=(0, 0, (SERVO_BODY_H + 2 * CL) / 2.0 - CL))

    # Output-coupling clearance bore (through the front plate, well above).
    bore = _cyl(SERVO_OUTPUT_BORE_OD / 2.0 + 0.4, SERVO_OUTPUT_H * 8)
    bore.apply_translation([SERVO_OUTPUT_X, 0, SERVO_BODY_H])

    # 4 x M2.5 case-mount clearance holes through the front-side plate,
    # on the 9.8 mm square around the output axis.
    mount_holes = []
    for (hx, hy) in servo_mount_hole_centres():
        h = _cyl(SERVO_MOUNT_SCREW_OD / 2.0, 14.0)
        h.apply_translation([hx, hy, SERVO_BODY_H])
        mount_holes.append(h)

    return _union(body, bore, *mount_holes)


def make_servo_body() -> trimesh.Trimesh:
    """Visual hobby-servo envelope for MuJoCo/CAD fit-checking.

    Same local frame as ``_servo_envelope``:
        +X = body long axis (mounting-tab span; output shaft offset toward +X)
        +Y = body short axis (depth)
        +Z = output-shaft direction (out of body top face)
        origin at the centre of the body's bottom face.

    Intended use:
        - Visual mesh asset in ``mujoco_prototype.py`` (contact-free).
        - Fit-check input for ``_verify_prototype.py``'s servo-clearance test.
    Not for printing -- the parametric well/cradle parts already cut the
    matching pocket out of the printed part, so a printed servo body would
    just clash with itself.
    """
    return _servo_envelope()


def make_servo_horn() -> trimesh.Trimesh:
    """Plastic 4-arm output horn -- the now-RETIRED part (kept for compat).

    The robot now drives a 20 mm aluminum 25T disc horn (``make_disc_horn``);
    this X-horn factory is retained only for backward-compatible visuals and
    old quotes.

    Local frame:
        +Z = output shaft axis (mates to the servo spline at z = 0)
        Origin at the bottom face of the horn hub.

    The 4 arms point along +X, +Y, -X, -Y (DISC_HORN_BOLT_ANGLES_RAD).  Each
    arm carries a row of mounting holes; the SECOND hole out from the
    spline on each arm sits on DISC_HORN_BOLT_PCD -- the pattern the retired
    printed ``servo_horn_adapter`` clamped onto.  This visual mesh drills
    those 4 bolt holes so any legacy render that includes both the horn and
    the adapter shows the bolts lining up.
    Used as a visual stand-in only; the retired printed ``servo_horn_adapter``
    bolted on top of this.
    """
    parts: list[trimesh.Trimesh] = []
    # Hub spans z=[0, PLASTIC_HORN_H] so the mesh's bounding cylinder
    # matches the placement math (HORN_STACK_H = PLASTIC_HORN_H = 5 mm).
    # The pre-2026 mesh had the hub at z=[0, 2] and the arms ALSO at
    # z=[0, 1.6], so the visual stack was 3 mm SHORTER than the math
    # said (HORN_STACK_H = 5).  Inspectors saw a 3 mm air gap between
    # the horn top and the link bottom even when the placement math
    # was correct.  Now the hub fills the full 5 mm and the arms sit
    # on the hub TOP at z=[3.4, 5.0] so the link's bottom face mates
    # to the arms at z = PLASTIC_HORN_H = 5 (= link-local z = 0).
    hub_h = PLASTIC_HORN_H
    arm_t = 1.6
    hub = _cyl(8.0, hub_h)
    hub.apply_translation([0, 0, hub_h / 2.0])
    parts.append(hub)
    # Arm length 2 * PLASTIC_HORN_X_TIP_R so each arm's tip sits at the
    # real-hardware sweep radius (= 19 mm from the spline centre on a
    # standard DS3225 / MG996R / DS3218 horn).  Previously hard-coded
    # to 20 mm (tip at radius 10 mm), which was INSIDE the
    # retired X-horn's 20.8 mm bolt circle -- nonsensical, and made the
    # mesh's bounding cylinder understate the horn's swept volume.
    # check_horn_sweep_clearance reads the bounding cylinder of this
    # mesh; keep it in sync with real hardware so the verifier
    # measures the right sweep radius.
    arm_z_centre = hub_h - arm_t / 2.0
    for a in DISC_HORN_BOLT_ANGLES_RAD:
        arm = _box((2.0 * PLASTIC_HORN_X_TIP_R, 4.0, arm_t))
        arm.apply_transform(rotation_matrix(a, [0, 0, 1]))
        arm.apply_translation([0, 0, arm_z_centre])
        parts.append(arm)
    horn = _union(*parts)

    # Bolt holes drilled down through the arms, slightly into the hub
    # top (z range = [hub_h - arm_t - 0.4, hub_h]) so the CSG cut
    # cleanly punches through the arm even with FDM/voxel tolerance.
    bolt_holes = []
    hole_h = arm_t + 0.4
    hole_z = hub_h - hole_h / 2.0
    for a in DISC_HORN_BOLT_ANGLES_RAD:
        h = _cyl(DISC_HORN_BOLT_OD / 2.0, hole_h)
        h.apply_translation([DISC_HORN_BOLT_PCD / 2.0 * np.cos(a),
                              DISC_HORN_BOLT_PCD / 2.0 * np.sin(a),
                              hole_z])
        bolt_holes.append(h)

    # Central screw clearance hole through the hub.  The real DS3225 /
    # MG996R / DS3218 plastic X-horn has a Phi ~5.5 mm bore at the
    # centre so an M3 SHCS head + driver passes through and threads
    # into the servo's spline cap screw.  Without this hole the visual
    # hub is a solid Phi 16 mm slug that occludes the link pad's
    # central screw clearance hole (HORN_CENTRE_OD = 3.4 mm) in the
    # assembled view, making it appear that the link's screw clearance
    # dead-ends at the link's mating face.  Drill through the full
    # hub_h + 1 mm overshoot on each face for clean boolean diff.
    centre_hole = _cyl(HORN_CENTRE_OD / 2.0, hub_h + 2.0)
    centre_hole.apply_translation([0, 0, hub_h / 2.0])
    return _diff(horn, *bolt_holes, centre_hole)


def make_disc_horn() -> trimesh.Trimesh:
    """20 mm aluminum 25T DISC servo horn (the part the robot uses).

    Models Amazon B07D56FVK5 ("10Pcs Servo Horn Metal Aluminum 25T
    Silvery Servo Disc ... MG945 MG995 MG996"): a Phi DISC_HORN_OD =
    20 mm x DISC_HORN_H = 5 mm silver-anodised aluminium disc with a
    central 25T female spline bore (Phi DISC_HORN_SPLINE_OD = 5.5 mm)
    and 4 x M3 TAPPED mounting holes on the DISC_HORN_BOLT_PCD = 14 mm
    bolt circle (cross pattern at DISC_HORN_BOLT_ANGLES_RAD).

    This replaces the plastic 4-arm X-horn (``make_servo_horn``) at
    every servo joint (yaw / hip-pitch / knee-pitch).  The driven link
    bolts onto the disc's flat top face with 4 x M3 SHCS that thread
    DIRECTLY into the disc's tapped holes -- the aluminium IS the
    thread-engagement medium.

    Local frame (matches ``make_servo_horn`` so the placement math is
    unchanged):
        +Z = servo output-shaft axis.
        Origin at the BOTTOM face (spline / servo side) at z = 0; the
        flat TOP mating face (where the link clamps) sits at
        z = DISC_HORN_H = HORN_STACK_H = 5 mm.

    The 4 tapped holes are drawn at the M3 tap-drill diameter
    (DISC_HORN_TAP_OD = 2.5 mm) rather than M3 clearance, because they
    are TAPPED -- the bolt's thread engages the hole wall.  Spline
    teeth are not modelled (a smooth Phi 5.5 mm bore stands in for the
    25T female spline).
    """
    h = DISC_HORN_H
    disc = _cyl(DISC_HORN_OD / 2.0, h)
    disc.apply_translation([0, 0, h / 2.0])

    # Central 25T spline bore (through), smooth cylinder stand-in.
    spline = _cyl(DISC_HORN_SPLINE_OD / 2.0, h + 2.0)
    spline.apply_translation([0, 0, h / 2.0])

    # 4 x M3 tapped holes on the 14 mm bolt circle (cross pattern).
    holes = []
    for a in DISC_HORN_BOLT_ANGLES_RAD:
        hole = _cyl(DISC_HORN_TAP_OD / 2.0, h + 2.0)
        hole.apply_translation([DISC_HORN_BOLT_PCD / 2.0 * np.cos(a),
                                 DISC_HORN_BOLT_PCD / 2.0 * np.sin(a),
                                 h / 2.0])
        holes.append(hole)

    return _diff(disc, spline, *holes)


def make_gobilda_1906_hub() -> trimesh.Trimesh:
    """goBILDA 1906 Series 25T aluminum servo hub (visualization mesh).

    Source: https://www.gobilda.com/1906-series-lightweight-servo-hub-25-tooth-spline-32mm-diameter/
    SKU:    1906-0025-0032 ("1906 Series Low-Profile Servo Hub, 25-Tooth
            Spline, 32 mm Diameter"; aluminum, clear-anodized; 4.7 g; $4.99)

    Why this exists (user report, May 2026):
        The OEM plastic 4-arm X-horn that ships with hobby servos
        (``make_servo_horn``) strips its spline after a few load cycles
        at the hexapod's femur joint.  This aluminum hub is the
        higher-torque replacement.  Using it requires redesigning the
        link mounting pad -- the goBILDA bolt pattern is a 16 mm SQUARE
        (M4 at +/-8, +/-8) which is COMPLETELY DIFFERENT from the
        current ``DISC_HORN_BOLT_PCD`` = 14 mm disc-horn cross PCD that the
        link pad mates against -- so this factory is offered in parallel
        to ``make_disc_horn`` and the assembly/inspector still call the
        disc horn for now.

    Local frame (matches ``make_servo_horn``):
        +Z = servo output shaft axis
        Origin at the BOTTOM face (the spline-engagement side that mates
            to the servo).  Top face sits at z = GOBILDA_1906_HUB_H.

    Modeled features:
        * Main hub disc: Phi GOBILDA_1906_HUB_OD = 32 mm, height
          GOBILDA_1906_DISC_H = 4 mm, spanning z = [0, 4].
        * Top centering boss: Phi GOBILDA_1906_BOSS_OD = 14 mm, height
          GOBILDA_1906_BOSS_H = 2 mm, on top of the main disc spanning
          z = [4, 6].  This is goBILDA's 14 mm "centering mechanism for
          mating components that have a 14 mm bore."
        * Central 25T spline female bore: Phi GOBILDA_1906_SPLINE_OD =
          6 mm (smooth cylinder; the real part has 25 internal spline
          teeth that we DON'T draw because this is a visualization mesh).
          Modeled as a blind pocket from the BOTTOM face spanning the
          full main-disc thickness (z = [0, GOBILDA_1906_DISC_H]) so
          the spline engages the servo output shaft.
        * Central M3 cap-screw clearance hole: Phi HORN_CENTRE_OD =
          3.4 mm, drilled THROUGH the full hub from bottom to top so the
          servo's M3 spline-cap screw passes through.
        * 4 x M4 threaded holes: Phi GOBILDA_1906_M4_OD = 4.3 mm
          (clearance for the M4 screw; the real hub is tapped M4),
          at (+/-GOBILDA_1906_BOLT_GRID/2, +/-GOBILDA_1906_BOLT_GRID/2)
          = (+/-8, +/-8) mm from the spline centre.  Important: the
          bolts sit at radius sqrt(8^2 + 8^2) ~= 11.31 mm, which is
          OUTSIDE the 14 mm boss footprint (boss radius = 7).  So the
          M4 holes can only tap into the 32 mm DISC's flat top
          shoulder, not the boss -- modeled as blind pockets going
          DOWN GOBILDA_1906_M4_DEPTH = 3 mm from the disc top face
          (z = GOBILDA_1906_DISC_H = 4 mm), spanning roughly z = [1, 4]
          inside the main disc.

    Returns:
        A single watertight ``trimesh.Trimesh`` ~32 mm OD x 6 mm tall.

    NOTE: This is a VISUALIZATION mesh.  Don't use it for printable-part
    fit checks: spline teeth are omitted, hole positions are nominal,
    and the boss/disc step is an estimate (the goBILDA product page
    doesn't publish a sectional dimension drawing).  When goBILDA's
    STEP file is available offline, prefer it for any production check.
    """
    disc_h = GOBILDA_1906_DISC_H
    boss_h = GOBILDA_1906_BOSS_H
    hub_h = GOBILDA_1906_HUB_H  # = disc_h + boss_h
    assert abs((disc_h + boss_h) - hub_h) < 1e-9, (
        "GOBILDA_1906_HUB_H must equal DISC_H + BOSS_H")

    # Main 32 mm disc, z = [0, disc_h]
    disc = _cyl(GOBILDA_1906_HUB_OD / 2.0, disc_h)
    disc.apply_translation([0, 0, disc_h / 2.0])

    # 14 mm centering boss on top of the disc, z = [disc_h, hub_h]
    boss = _cyl(GOBILDA_1906_BOSS_OD / 2.0, boss_h)
    boss.apply_translation([0, 0, disc_h + boss_h / 2.0])

    hub = _union(disc, boss)

    # Spline bore (blind from BOTTOM face, full disc thickness).  Smooth
    # cylinder standing in for the 25T H25T female spline.
    spline_bore = _cyl(GOBILDA_1906_SPLINE_OD / 2.0, disc_h + 0.2)
    spline_bore.apply_translation([0, 0, (disc_h + 0.2) / 2.0 - 0.1])

    # M3 spline cap-screw clearance hole, through the full hub.
    centre_hole = _cyl(HORN_CENTRE_OD / 2.0, hub_h + 2.0)
    centre_hole.apply_translation([0, 0, hub_h / 2.0])

    # 4 x M4 threaded holes at +/-8, +/-8 on a 16 mm SQUARE grid.
    # Bolts sit at radius ~11.3 mm = OUTSIDE the 14 mm boss (radius 7),
    # so the holes tap into the 32 mm disc's flat top shoulder at
    # z = disc_h.  Pocket spans z = [disc_h - M4_DEPTH, disc_h + 0.1]
    # (the 0.1 overshoot at the disc top gives a clean CSG cut).
    g = GOBILDA_1906_BOLT_GRID / 2.0
    m4_depth = GOBILDA_1906_M4_DEPTH
    m4_holes = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            h = _cyl(GOBILDA_1906_M4_OD / 2.0, m4_depth + 0.2)
            h.apply_translation([sx * g, sy * g,
                                  disc_h - m4_depth / 2.0 + 0.1])
            m4_holes.append(h)

    return _diff(hub, spline_bore, centre_hole, *m4_holes)


def make_servo_horn_adapter() -> trimesh.Trimesh:
    """Round servo-horn adapter plate -- RETIRED (kept for backward compat).

    Models the now-retired printed adapter.  It bolted to a standard
    plastic servo horn from below (single M3 centre screw + a counter-bored
    recess for the horn body) and presented a flat 4 x M3 bolt pattern on
    the bolt circle so any link with the matching hole pattern could clamp
    onto it.  The robot now bolts each link straight onto a 20 mm aluminum
    25T disc horn instead.

    Local frame:
        +Z = servo output axis
        Origin at the bottom face (mating to the plastic horn)
        Bolt holes on DISC_HORN_BOLT_PCD at DISC_HORN_BOLT_ANGLES_RAD
        (= 0 / 90 / 180 / 270 deg), aligned with the retired plastic
        horn's 4 X-shaped arms so each bolt dropped straight into the
        second hole-position out from the spline on each arm.

    Why a solid disc (and not the 4-arm cross the BOM used to advertise):
        The previous design had four "lightening cuts" sized so they sliced
        straight through the arms, leaving 4 corner blocks disconnected
        from the centre.  A 32 mm OD x 4 mm disc is 3 cm^3 of plastic per
        copy -- lightening it saves nothing and prints far more reliably.
    """
    plate_r = HORN_ADAPTER_OD / 2.0
    plate = _cyl(plate_r, HORN_ADAPTER_T)
    plate.apply_translation([0, 0, HORN_ADAPTER_T / 2.0])

    recess = _cyl(HORN_RECESS_OD / 2.0, HORN_RECESS_DEPTH)
    recess.apply_translation([0, 0, HORN_RECESS_DEPTH / 2.0])

    centre = _cyl(HORN_CENTRE_OD / 2.0, HORN_ADAPTER_T * 4)

    bolts = []
    for a in DISC_HORN_BOLT_ANGLES_RAD:
        h = _cyl(DISC_HORN_BOLT_OD / 2.0, HORN_ADAPTER_T * 4)
        h.apply_translation([DISC_HORN_BOLT_PCD / 2.0 * np.cos(a),
                              DISC_HORN_BOLT_PCD / 2.0 * np.sin(a),
                              0])
        bolts.append(h)

    return _diff(plate, recess, centre, *bolts)


# ---------------------------------------------------------------------------
# Body parts
# ---------------------------------------------------------------------------

def _leg_chassis_frames():
    """Yield ``(leg_index, edge_mid_xyz, R_a, R3)`` for each of the 6
    legs in the hex chassis pattern.  Single source of truth for the
    leg-azimuth iteration: ``_hex_plate``, ``make_chassis_bottom``, the
    workspace verifier and the wire-harness planner ALL go through this
    helper so a future change to the leg layout (different leg count,
    different azimuth offset) only happens here.

    Mirrors ``_build_workspace_leg`` / ``_build_standing_leg`` /
    ``build_prototype_assembly._build_leg`` which all derive the same
    azimuth ``a = (leg_index + 0.5) * pi / 3`` and radius
    ``apothem = CHASSIS_FLAT_TO_FLAT / 2``.
    """
    apothem = CHASSIS_FLAT_TO_FLAT / 2.0
    for i in range(6):
        a = (i + 0.5) * np.pi / 3.0
        edge_mid = np.array([apothem * np.cos(a),
                              apothem * np.sin(a),
                              0.0])
        R = rotation_matrix(a, [0, 0, 1])
        yield i, edge_mid, R, R[:3, :3]


def _hex_plate(flat_to_flat: float, thickness: float,
               with_centre_holes: bool = False,
               with_leg_features: bool = True,
               with_leg_harness_drops: bool = False,
               with_chassis_standoffs: bool = False) -> trimesh.Trimesh:
    """Return a flat hexagonal plate, centred on origin, axis = +Z.

    Hole pattern (per leg, 6 legs total):
        4 vertical M3 clearance holes (Φ BRACKET_BOLT_HOLE) drilled all
        the way through.  Pattern matches `make_coxa_bracket()`'s flange:
        2 holes on the OUTBOARD edge of the bolt rectangle (just inboard
        of the chassis perimeter) and 2 on the INBOARD edge.  All four
        holes are inboard of the apothem line so the bolt heads have
        chassis material under them.

    Optional inboard hole patterns:
        ``with_centre_holes``: 4 vertical M3 clearance holes on a
            35-mm-radius / 45-deg square (= ELEC_CHASSIS_MOUNT_HOLES_XY
            = (+/-24.75, +/-24.75) mm).  Only chassis_top still carries
            this pattern: the holes were used by the retired in-gap
            electronics_tray's chassis-mount bolts and are kept on the
            top plate for retrofit compatibility with older prints.
            (Jul 2026: chassis_bottom no longer carries the pattern --
            its tray-mount bosses + insert pockets were vestigial after
            the tray's retirement and their boss bodies sat inside the
            real 138 x 46 mm battery's footprint.)
        ``with_chassis_standoffs``: 4 vertical M3 clearance holes on
            the 44-mm-radius diagonal pattern
            (= CHASSIS_STANDOFF_HOLES_XY = (+/-31.1, +/-31.1)).
            chassis_top + chassis_bottom both carry this pattern.  The
            4 M3 F-F brass standoffs span the inter-plate gap on this
            pattern: an M3 x 14 SHCS enters from BELOW chassis_bottom
            into the standoff's bottom female thread and an M3 x 10
            SHCS drops DOWN from above chassis_top into its top
            female thread (see the CHASSIS_STANDOFF_R block).
    """
    apothem = flat_to_flat / 2.0
    circum = apothem / np.cos(np.pi / 6)
    plate = _cyl(circum, thickness, sections=6)
    plate.apply_transform(rotation_matrix(np.pi / 6, [0, 0, 1]))

    # The bracket's flange occupies bracket-local x in [-FLANGE_X, 0]
    # = [-30, 0] (origin = yaw axis = chassis edge).  Bolts at:
    #   outboard pair: x = -BRACKET_FLANGE_INSET (8 mm inboard of edge)
    #   inboard pair:  x = -BRACKET_FLANGE_INSET - BRACKET_BOLT_PCD_X
    #                       (8 + 20 = 28 mm inboard of edge)
    bolt_x_outboard = -BRACKET_FLANGE_INSET
    bolt_x_inboard  = -BRACKET_FLANGE_INSET - BRACKET_BOLT_PCD_X
    bolt_ys         = (-BRACKET_BOLT_PCD_Y / 2.0,
                       +BRACKET_BOLT_PCD_Y / 2.0)

    # Servo body+tab cutout per bracket so the yaw servo's body can
    # hang BELOW the chassis plate while the bracket flange clamps the
    # plate from above.  Without this cutout the user has to drill the
    # opening manually (which was issue #1 the user hit).
    # Size = WELL_W x WELL_D + 1 mm clearance per side so the
    # bracket's well slides through without rubbing.  Sits at bracket-
    # local (body_centre_x, 0) = (-10, 0) -- the well's centre.  Bolts
    # at y = +/-18 stay outside the cutout's y range (+/-13.5), so the
    # 4 chassis bolts still bite into solid chassis material.
    body_centre_x  = -SERVO_OUTPUT_X
    body_cutout_w  = WELL_W + 2.0            # 60 mm along bracket X
    body_cutout_d  = WELL_D + 2.0            # 31 mm along bracket Y

    holes = []
    if with_leg_features:
        for i, edge_mid, R, R3 in _leg_chassis_frames():
            # ---- Coxa-bracket -> chassis bolt holes (Phi 3.4 mm) ----
            # Commit 5 of the May 2026 chassis-bottom-integrated yaw-
            # cradle redesign retired the 4-bolt-per-leg bracket-to-
            # chassis pattern.  The yaw cradle no longer drops into
            # the bracket flange + nyloc-nut sandwich; the cradle's
            # heat-set inserts + cradle bolts (see
            # ``_chassis_yaw_cradle_solid``) now do all the structural
            # work.  The 4-bolt drill loop is intentionally GONE;
            # ``bolt_x_outboard / inboard / bolt_ys`` above are still
            # computed (cheap) so commit 8 can delete BRACKET_BOLT_PCD_X
            # / PCD_Y / FLANGE_INSET / BOLT_HOLE alongside the rest of
            # the BRACKET_* constants in one sweep.  Until commit 8 the
            # bracket itself is still PLACED on the chassis edge -- its
            # well hangs through the ``cutout`` carved below -- but no
            # fastener clamps the flange to chassis_bottom, so the
            # bracket is held by visual overlap only during commits 5-7.
            # This is fine: commits 6-7 only touch the verifier +
            # scripts; commit 8 removes the bracket entirely.

            cutout = _box((body_cutout_w, body_cutout_d, thickness * 4))
            cutout.apply_transform(R)
            cutout_world = edge_mid + R3 @ np.array([body_centre_x, 0.0, 0.0])
            cutout.apply_translation(cutout_world)
            holes.append(cutout)

            if with_leg_harness_drops:
                # Per-leg cable-drop slots through the plate, just INBOARD of
                # the body cutout, on the chassis radial axis (slot long axis =
                # bracket +X = chassis radial, short axis = bracket Y = chassis
                # tangential).  ``leg_harness_drop_slots()`` enumerates the
                # port(s) -- since Aug 2026 a single open 18 x 28 slot; see
                # the LEG_HARNESS_DROP_* constants block for the rationale.
                for sx, sy, sxe, sye in leg_harness_drop_slots():
                    drop = _box((sxe, sye, thickness * 4))
                    drop.apply_transform(R)
                    drop_world = edge_mid + R3 @ np.array([sx, sy, 0.0])
                    drop.apply_translation(drop_world)
                    holes.append(drop)

    if with_centre_holes:
        # 4 holes on the 35 mm radius / 45 deg square pattern
        # (= ELEC_CHASSIS_MOUNT_HOLES_XY = (+/-24.75, +/-24.75) mm).
        # Only chassis_top requests these (retrofit-compat holes for
        # the retired in-gap electronics_tray's mount bolts).
        for (cx, cy) in ELEC_CHASSIS_MOUNT_HOLES_XY:
            h = _cyl(BRACKET_BOLT_HOLE / 2.0, thickness * 4)
            h.apply_translation([cx, cy, 0])
            holes.append(h)

    if with_chassis_standoffs:
        # 4 holes for the brass M3 F-F standoff columns on the
        # 44-mm-radius diagonal pattern (= CHASSIS_STANDOFF_HOLES_XY =
        # (+/-31.1, +/-31.1) mm).  chassis_top + chassis_bottom both
        # carry this pattern; an M3 x 14 SHCS enters from below
        # chassis_bottom into the standoff's bottom female thread, an
        # M3 x 10 SHCS from above chassis_top into its top thread.
        for (cx, cy) in CHASSIS_STANDOFF_HOLES_XY:
            h = _cyl(BRACKET_BOLT_HOLE / 2.0, thickness * 4)
            h.apply_translation([cx, cy, 0])
            holes.append(h)

    return _diff(plate, *holes)


def make_chassis_top() -> trimesh.Trimesh:
    """Top deck plate.  3D-printed, CHASSIS_TOP_T = 2 mm thick (Aug 2026:
    halved from 4 mm so the user can add screw mounts -- an M3 + nut or a
    short self-tapper clamps cleanly through 2 mm of plate).

    Intentionally SMALLER than the bottom plate (CHASSIS_TOP_FLAT_TO_FLAT
    = 140 mm vs CHASSIS_FLAT_TO_FLAT = 200 mm).  The bottom plate carries
    the structural load (it sandwiches the coxa brackets and takes their
    M3 bolts) while the top plate just provides a deck for the battery
    holder, electronics tray and optional arm.  The smaller hexagon keeps
    the top plate inside a 70 mm apothem so the legs' hip-pitch wells +
    femur hip pads sweep clear of it across the full joint workspace
    (see check_workspace_self_collision in _verify_prototype.py).  No
    per-leg bracket cutouts or M3 bolt holes are needed because the
    bottom plate already takes them.  Only the 4 centre-hole standoff
    bolts (battery/electronics tray + arm baseplate) remain.

    (Aug 16 2026: the 2 switch_holster mount bosses -- Phi 8 mm, 5 mm
    proud, with M3 heat-set inserts -- are RETIRED.  The user velcros
    the holster to the deck, so the plate top face is completely flat.)

    Yaw-shaft pass-through cutouts (May 2026 chassis-bottom-integrated
    yaw-cradle redesign):

      The redesigned yaw-servo cradle sits inside ``chassis_bottom``
      and routes the servo's output spline UP toward chassis_top.  In
      the legacy ``coxa_bracket`` design the spline + X-horn stack
      lived ENTIRELY below the bracket flange (chassis-z <= +20),
      below chassis_top's bottom face (chassis-z = +22 at the Aug 2026
      20 mm gap; +32 historically); no top-plate hole was needed.  Under the
      new design the spline still terminates at chassis-z = +20
      (= cradle-z +6 tab shelf + SERVO_BODY_H - SERVO_TAB_Z + spline
      length), but the project's CAD convention is to add an explicit
      pass-through cutout in chassis_top for any output shaft whose
      AXIS is below the top plate, so the design intent is documented
      in code instead of relying on a footprint-aware reader.

      Geometry: 6 x Phi (DISC_HORN_OD + 2 mm) = Phi 22 mm cylindrical
      cutouts -- 1 mm radial clearance on the Phi DISC_HORN_OD = 20 mm
      disc horn / coxa yaw-hub stack that rotates on the yaw axis --
      placed at the yaw-axis locations (= chassis hex edge midpoints,
      apothem 100 mm).  The cylinders have height ``CHASSIS_PLATE_T * 4``
      mm for a clean through-cut.  (Sized to DISC_HORN_OD, NOT the
      Phi 10 mm output gear stack: if chassis_top ever grew to reach the
      yaw axis, the 20 mm disc horn -- not the spline -- is what must
      clear it.)

      Geometric reality: chassis_top's hex perimeter sits at apothem
      = CHASSIS_TOP_FLAT_TO_FLAT / 2 = 70 mm, so the 6 yaw axes at
      apothem 100 mm are 30 mm OUTSIDE the plate's footprint.  The
      Phi 22 mm cylinders cut empty air; the diff is a geometric
      no-op against the current chassis_top hex.  Kept as an active
      diff so a future enlarged chassis_top (say apothem 110 mm) will
      automatically gain the clearance holes without a code change.
      The runtime cost of 6 empty diffs is negligible (well under
      1 ms in trimesh's CSG path).
    """
    plate = _hex_plate(CHASSIS_TOP_FLAT_TO_FLAT, CHASSIS_TOP_T,
                       with_centre_holes=True,
                       with_chassis_standoffs=True,
                       with_leg_features=False)

    # Yaw-shaft pass-through cutouts at every leg's yaw axis (= chassis
    # edge midpoint).  See the docstring above for why this is a
    # geometric no-op against the current 70 mm apothem chassis_top
    # (and why we keep the diff anyway).
    yaw_passthroughs: list[trimesh.Trimesh] = []
    yaw_clearance = 1.0  # mm radial clearance on the Phi DISC_HORN_OD disc
    yaw_hole_r = DISC_HORN_OD / 2.0 + yaw_clearance          # r11 -> Phi 22
    for _i, edge_mid, _R, _R3 in _leg_chassis_frames():
        hole = _cyl(yaw_hole_r, CHASSIS_PLATE_T * 4.0)
        hole.apply_translation([edge_mid[0], edge_mid[1], 0.0])
        yaw_passthroughs.append(hole)
    plate = _diff(plate, *yaw_passthroughs)

    # (Aug 16 2026: the 2 switch_holster insert bosses + pockets that
    # used to be unioned here are RETIRED -- the holster velcros to the
    # flat deck now.)

    # Electronics-deck standoff-column through-holes (Jun 2026 deck
    # redesign).  The Uno Q tray (lower) + buck tray (upper) bolt onto 4
    # standoff columns that rise ABOVE chassis_top from these 4 sites
    # (DECK_COLUMN_XY = (+/-DECK_COLUMN_DX, +/-DECK_COLUMN_DY) =
    # (+/-41, +/-33) mm -- well inside the 70 mm-apothem plate, radius
    # ~52.6 mm from centre).  Each lowest M-F standoff drops its male
    # stud DOWN through a Phi BRACKET_BOLT_HOLE = 3.4 mm clearance hole
    # here, captured by an M3 nyloc on the plate's UNDER face (same
    # retention scheme as the 4 brass inter-plate standoffs).
    column_holes = []
    for (cx, cy) in DECK_COLUMN_XY:
        h = _cyl(BRACKET_BOLT_HOLE / 2.0, CHASSIS_PLATE_T * 4.0)
        h.apply_translation([cx, cy, 0.0])
        column_holes.append(h)

    top = _diff(plate, *column_holes)

    # Coaxial-coxa clearance clip (Jun 2026 re-centre): with the hip
    # bracket centred on the yaw axis, the coxa servo body sweeps INWARD
    # to radius ~60 mm in the chassis_top z-band (z 34-36) over the yaw
    # workspace.  The legacy 70 mm-apothem hexagon (corners at ~81 mm)
    # therefore intrudes into that sweep at every leg.  Clip the deck to a
    # CHASSIS_TOP_RADIUS = 57.5 mm DISK: that clears the coxa sweep with a
    # ~2.5 mm margin while still carrying ALL the deck mounts -- the 4
    # electronics-deck standoff columns (DECK_COLUMN_XY, r = 52.6 mm), the
    # 2 switch-holster bosses (r = 41 mm), the 4 inter-plate centre
    # standoffs and the centre-hole cluster are all at r <= 52.6 mm, well
    # inside the disk.  (The columns rise INBOARD of the r >= 60 mm coxa
    # sweep, so they too stay clear.)
    clip = _cyl(CHASSIS_TOP_RADIUS, CHASSIS_PLATE_T * 6.0 + 40.0)
    return _intersection(top, clip)


def _chassis_bottom_full_solid() -> trimesh.Trimesh:
    """Full bottom hex plate INCLUDING the 6 below-plate yaw-servo cradle
    buckets -- the pre-merge construction base.

    INTERNAL CONSTRUCTION HELPER (Jun 2026 single-part merge).  This is the
    plate + tray-mount bosses + upward bearing tower + the 6 deep cradle
    buckets.  ``make_chassis_bottom`` builds the real printed part by cutting
    everything below ``CHASSIS_SPLIT_Z`` off this solid (discarding the deep
    buckets, keeping the flat plate + cradle WELL walls + tower) and folding a
    flat floor slab onto the underside.  This solid is NOT printed and is NO
    LONGER the verifier's assembled reference -- the cradle-feature probes now
    load the REAL merged ``make_chassis_bottom`` (via the ``chassis_assembled``
    key) so they test the part that actually prints.  Its one remaining
    standalone use is as the known-bad flat-bottom fixture in
    ``check_flat_bottom`` (the deep buckets MUST be rejected by the guard --
    see ``_flatbottom_check.py``).

    Structural carrier for the coxa-bracket flanges and the brass
    standoff columns (44-mm-radius diagonal pattern via
    ``with_chassis_standoffs``).  Jul 2026 battery-fit rework: the
    tray-mount bosses + heat-set insert pockets + centre holes at
    ELEC_CHASSIS_MOUNT_HOLES_XY and the battery_holder foot-bolt
    pattern + hex-key access bores are all DELETED -- the in-gap
    electronics_tray and the clip-in battery_holder are both retired,
    and the boss bodies at (+/-24.75, +/-24.75) sat inside the real
    138 x 46 mm pack's footprint (the pack corner would have rested
    on them instead of lying flat on the plate).

    Cable management (Part A + Part B, May 2026):

    * Each leg gets a small ``LEG_HARNESS_DROP_X_EXTENT`` x
      ``LEG_HARNESS_DROP_Y_EXTENT`` mm slot cut THROUGH the plate just
      INBOARD of the body cutout (long axis = chassis radial = bracket
      +X), so the leg's 3-cable harness can drop from the cradle side
      of the plate into the inter-plate volume without sharing the
      body cutout with the seated yaw servo body.  Applied by
      ``_hex_plate`` via ``with_leg_harness_drops=True``.  The slot
      itself is also the per-leg zip-tie anchor: the assembler loops
      a zip-tie through the slot to bundle the 3-cable harness as it
      makes its U-turn from the cradle wire-exit into the inter-plate
      volume.  (A previous revision printed a vertical anchor tab
      below the plate; it was retired in May 2026 to keep the plate's
      bottom face flat for easier FDM printing -- see the
      CABLE_ANCHOR_TAB_* constants block above.)
    """
    plate = _hex_plate(CHASSIS_FLAT_TO_FLAT, CHASSIS_PLATE_T,
                       with_chassis_standoffs=True,
                       with_leg_harness_drops=True)

    # (Jul 2026: the May 2026 tray-mount bosses + heat-set insert
    # pockets at ELEC_CHASSIS_MOUNT_HOLES_XY are deleted -- see the
    # docstring above.  The battery must lie FLAT on the plate top
    # face; the 3-mm-tall boss bodies sat inside its footprint.)

    # Per-leg integrated yaw-servo cradles (May 2026 redesign).
    # ``_chassis_yaw_cradle_solid`` returns the cradle for ONE leg in
    # cradle-local frame (origin at the yaw axis / chassis edge
    # midpoint; cradle-z = 0 = chassis_bottom_top face).  The plate's
    # own z = 0 sits at the plate's CENTRE, so we translate each cradle
    # UP by ``CHASSIS_PLATE_T / 2`` to put cradle-z = 0 at the plate's
    # TOP face (plate-local z = +2 mm).
    #
    # During the May 2026 transition commits (between this commit and
    # the final cleanup), the legacy ``make_coxa_bracket`` still gets
    # placed on top of chassis_bottom by ``build_prototype_assembly``;
    # its flange overlaps the cradle's outer shell at chassis-z in
    # [+2, +17].  This overlap is intentional and harmless: no
    # verifier check probes the bracket-flange-vs-chassis_bottom
    # interface (see the explicit NOTE in
    # ``_verify_prototype.check_mating_face_contact``).  The bracket
    # holds the yaw servo until commit 4 swaps the assembly to use
    # the integrated cradle instead.
    cradles: list[trimesh.Trimesh] = []
    for _i, edge_mid, R, R3 in _leg_chassis_frames():
        cradle = _chassis_yaw_cradle_solid()
        cradle.apply_transform(R)
        cradle.apply_translation(
            [edge_mid[0], edge_mid[1], CHASSIS_PLATE_T / 2.0],
        )
        cradles.append(cradle)
    plate = _union(plate, *cradles)

    # Re-cut the per-leg cable-drop slots.  ``_hex_plate`` already
    # cuts them when ``with_leg_harness_drops=True``, but the cradle's
    # -X bond-strip overhang at cradle-x in [-41, -40] crosses the
    # outboard 1 mm of each slot at bracket-x = [-52, -40].  The
    # boolean union of the cradles with the plate would re-fill that
    # 1 mm strip and partially plug the slot (verified by
    # ``check_leg_harness_drop`` firing on 12 of 12 probe points per
    # leg when the slots are NOT re-cut).  Re-cutting after the union
    # is an idempotent diff: the slot is already open in the original
    # plate, the bond-strip union fills its outboard edge, and this
    # diff carves it back out.  Geometry matches the cuts in
    # ``_hex_plate(with_leg_harness_drops=True)`` so the post-union
    # plate has identical drop-slot openings to the pre-union plate.
    drops: list[trimesh.Trimesh] = []
    for _i, edge_mid, R, R3 in _leg_chassis_frames():
        for sx, sy, sxe, sye in leg_harness_drop_slots():
            drop = _box((sxe, sye, CHASSIS_PLATE_T * 4.0))
            drop.apply_transform(R)
            drop_world = edge_mid + R3 @ np.array([sx, sy, 0.0])
            drop.apply_translation(drop_world)
            drops.append(drop)
    plate = _diff(plate, *drops)

    # (Jul 2026: the 4 battery-bolt hex-key access bores are deleted
    # along with the retired battery_holder's foot-bolt pattern.)

    # Cable-corridor relief (Jun 2026).  The Pi4 / bus-adapter connector
    # cable corridors (defined in cable_keepouts.py, the same airspaces
    # check_cable_clearance probes) exit radially past the chassis edge,
    # but the integrated yaw cradles rise into those reserved airspaces
    # (cradle top reaches chassis-z +24.8 mm; the connector corridors span
    # +13..+31 mm).  Carve the (1 mm-padded) corridor volumes out of the
    # cradle shells so each cable can physically be plugged in.  Deferred
    # import avoids a module-load cycle (cable_keepouts imports this
    # module at top level).
    import cable_keepouts as _ck  # noqa: WPS433
    corridor_cuts: list[trimesh.Trimesh] = []
    for _ko in _ck.build_cable_keepouts():
        _lo, _hi = _ko.mesh.bounds
        # Pad generously so each corridor punches CLEAN through the cradle
        # shell instead of leaving a < MIN_PRINT_T sliver wall beside the
        # tunnel (those slivers trip check_flimsy_joints).  3.5 mm clears
        # the ~1-2 mm remnants without reaching the central servo well.
        _pad = 3.5
        _ext = (_hi - _lo) + 2.0 * _pad
        corridor_cuts.append(
            _box(tuple(_ext), center=tuple((_lo + _hi) / 2.0)))
    plate = _diff(plate, *corridor_cuts)

    # Battery velcro-strap slots (Jun 2026 deck redesign; positions FROZEN).
    # Cut for the retired 138 x 46 mm bay pack: 3 pairs of slots straight
    # THROUGH the plate at x = -8 + (-30, 0, +30), y = +/-26.  Aug 2026:
    # the battery is now two shorty packs velcro'd UNDER the belly; the
    # CENTRE pair carries the safety strap that wraps the under-slung
    # 30-deg pack block (and passes the battery leads up into the bay),
    # the outer pairs are spare tie points.  Slot geometry is untouched so
    # the as-printed plate stays valid.
    velcro_slots: list[trimesh.Trimesh] = []
    strap_y = BATTERY_STRAP_SLOT_Y
    for strap_dx in BATTERY_STRAP_DX:
        sx_centre = BATTERY_HOLDER_CENTRE_X + strap_dx
        for sy in (-1, 1):
            slot = _box((BATTERY_STRAP_W, 4.0, CHASSIS_PLATE_T * 4.0),
                        center=(sx_centre, sy * strap_y, 0.0))
            velcro_slots.append(slot)
    # Battery trunk pass-through (Aug 2026): one 14 x 22 mm port at
    # (48, 0) on the +X vertex axis so both under-belly packs' terminated
    # XT60 leads pass up to the switch / main XT60 on the deck (see the
    # BATTERY_TRUNK_HOLE_* constants block).
    velcro_slots.append(
        _box((BATTERY_TRUNK_HOLE_X, BATTERY_TRUNK_HOLE_Y,
              CHASSIS_PLATE_T * 4.0),
             center=(BATTERY_TRUNK_HOLE_CENTRE[0],
                     BATTERY_TRUNK_HOLE_CENTRE[1], 0.0)))
    plate = _diff(plate, *velcro_slots)

    # May 2026: the per-leg vertical anchor tabs that used to hang
    # below the plate's bottom face have been RETIRED so the
    # chassis_bottom mesh is fully flat at chassis-z =
    # -CHASSIS_PLATE_T/2 (= -2 mm) for easier FDM printing.  The
    # zip-tie now loops through the drop slot itself to bundle the
    # per-leg harness -- no sub-plate geometry required.  See the
    # CABLE_ANCHOR_TAB_* constants block above for the (preserved)
    # historical dimensions.
    return plate


# ===========================================================================
# Chassis-bottom FLOOR -- single merged part (Jun 2026)
# ===========================================================================
# Forensic context: ``_flatbottom_check.py`` proved the ORIGINAL single-piece
# chassis_bottom did NOT print flat -- the 6 integrated yaw-servo cradle
# buckets hung ~20.5 mm BELOW the main hex plate, so the part rested on 6
# small rings (~16% bed contact) with the whole 200 mm plate floating as a
# support-needing overhang.  That drove a temporary HIGH/LOW print split.
#
# The LOW half was then simplified to a plain flat hex slab (the deep buckets
# were abandoned; the yaw servo is retained by the bolt-on
# ``yaw_servo_retainer`` capture stirrup instead).  At that point the split
# was VESTIGIAL: the HIGH half already prints flat (bearing tower up, broad
# flat underside on the bed), so the horizontal split bought nothing.
#
# So the two halves are MERGED back into ONE printed ``make_chassis_bottom``:
# the flat plate + everything above it, plus a solid flat FLOOR slab
# (``_chassis_bottom_floor_solid`` -- the old LOW plate, minus all the now-dead
# join hardware) folded onto its underside.  The result is a single part whose
# perimeter is ~8 mm thick with a broad flat -6 mm bed face -- still flat-bottom
# printable, tower up, no supports.  CHASSIS_SPLIT_Z is retained only as the
# plate-underside / floor-step reference plane.
CHASSIS_SPLIT_Z = -CHASSIS_PLATE_T / 2.0     # = -2.0 mm (plate underside; the
                                             # old print-split cut plane, kept as
                                             # the floor-step reference)
CHASSIS_BOTTOM_FLOOR_T = 4.0                 # mm -- solid flat floor folded onto
                                             # the plate underside (the old LOW
                                             # plate).  Bottom face at
                                             # CHASSIS_SPLIT_Z - this = -6 mm, so
                                             # the perimeter is ~8 mm thick.  The
                                             # yaw_servo_retainer stirrup bolts UP
                                             # into 2-per-leg self-tap pilots in
                                             # this floor (see
                                             # _chassis_bottom_floor_solid).


def make_chassis_bottom() -> trimesh.Trimesh:
    """The single, merged chassis_bottom (Jun 2026 -- the temporary HIGH/LOW
    print split has been re-merged into ONE printed part; see the
    "Chassis-bottom FLOOR" block above).

    Three steps:

      1. take the full integrated solid (``_chassis_bottom_full_solid``: the
         flat hex plate + the upward yaw-bearing tower +
         ``yaw_bearing_cap`` interface + the 6 yaw cradles + velcro/battery
         cutouts) and cut away everything BELOW the plate underside
         (``CHASSIS_SPLIT_Z`` = -2 mm).  That removes the deep cradle buckets
         that used to hang ~20.5 mm down, leaving the flat plate + the cradle
         WELL walls above the cut + the bearing tower -- a broad flat -2 face.

      2. fold a solid flat FLOOR slab (``_chassis_bottom_floor_solid``, the old
         LOW plate minus all join hardware) onto that underside, z in
         ``[-6, -2]`` (overlapping the plate by 1 mm for a clean volumetric
         union, never a coplanar kiss).  The perimeter is now ~8 mm thick with
         a broad flat -6 mm bed face -- still flat-bottom printable, tower up,
         no supports.  The floor carries the 6 per-leg servo-body clearance
         cutouts + harness-drop slots + the 2-per-leg ``yaw_servo_retainer``
         self-tap pilots.

      3. re-cut the 3 pairs of battery velcro-strap slots through the merged
         solid so a strap can still loop under the (now thicker) plate.

    The yaw servo body passes down through the floor's body cutout and hangs
    below the chassis; the bolt-on ``yaw_servo_retainer`` capture stirrup bolts
    UP into the floor pilots and blocks the servo from dropping (real retention
    preserved -- the pilots migrated here unchanged from the old LOW plate).
    """
    full = _chassis_bottom_full_solid()

    big = 800.0
    below = _box((big, big, big), center=(0.0, 0.0, CHASSIS_SPLIT_Z - big / 2.0))
    high = _diff(full, below)

    # Fold the flat floor slab onto the underside -> single ~8 mm part.
    merged = _union(high, _chassis_bottom_floor_solid())

    # Standoff SEAT PADS (Aug 16 2026): the inboard-shifted harness ports
    # clip the corner of every standoff site's seat annulus (see the
    # CHASSIS_STANDOFF_SEAT_PAD_OD block).  Union a full-stack Phi 9 pad
    # at each site BEFORE the through-cuts below re-drill the Phi 3.4
    # bore, restoring a solid ring for the brass standoff base (+2 face)
    # and the M3 x 14 head (-6 face).  Pad spans z [-6, +2] exactly so
    # the bed face stays flat and nothing pokes into the inter-plate bay.
    pad_top_z = CHASSIS_PLATE_T / 2.0                             # +2
    pad_bot_z = CHASSIS_SPLIT_Z - CHASSIS_BOTTOM_FLOOR_T          # -6
    pads = []
    for (cx, cy) in CHASSIS_STANDOFF_HOLES_XY:
        pad = _cyl(CHASSIS_STANDOFF_SEAT_PAD_OD / 2.0, pad_top_z - pad_bot_z)
        pad.apply_translation([cx, cy, 0.5 * (pad_top_z + pad_bot_z)])
        pads.append(pad)
    merged = _union(merged, *pads)

    # Re-cut the through-features the floor slab would otherwise plug below the
    # plate underside.  The cutters are tall enough to clear the full merged
    # thickness (z in [-6, +2]); cutting them in make_chassis_bottom (NOT in the
    # C6 floor slab) keeps the floor's C6 symmetry intact while these
    # off-centre / 4-fold features punch through the merged part.
    through_cuts: list[trimesh.Trimesh] = []

    # (a) 3 pairs of battery velcro-strap slots (BATTERY_HOLDER_CENTRE_X +
    #     BATTERY_STRAP_DX at y = +/-BATTERY_STRAP_SLOT_Y, frozen at the
    #     retired bay pack's edges) so a strap can loop UNDER the plate --
    #     now the safety strap for the under-belly packs.
    strap_y = BATTERY_STRAP_SLOT_Y
    for strap_dx in BATTERY_STRAP_DX:
        sx_centre = BATTERY_HOLDER_CENTRE_X + strap_dx
        for sy in (-1, 1):
            through_cuts.append(
                _box((BATTERY_STRAP_W, 4.0, CHASSIS_PLATE_T * 8.0),
                     center=(sx_centre, sy * strap_y, 0.0)))

    # (b) the 4 brass-standoff bolt clearance holes (CHASSIS_STANDOFF_
    #     HOLES_XY = (+/-31.1, +/-31.1)).  ``with_chassis_standoffs`` only
    #     drilled them through the z[-2,+2] plate; extend them through the
    #     floor so the M3 x 14 SHCS entering from the -6 bottom face reaches
    #     the F-F standoff's bottom female thread on the +2 top face.
    for (cx, cy) in CHASSIS_STANDOFF_HOLES_XY:
        h = _cyl(BRACKET_BOLT_HOLE / 2.0, CHASSIS_PLATE_T * 8.0)
        h.apply_translation([cx, cy, 0.0])
        through_cuts.append(h)

    # (c) the battery trunk pass-through at (48, 0): both under-belly
    #     packs' terminated XT60 leads climb through here to the switch /
    #     main XT60 on the deck (Aug 2026; see BATTERY_TRUNK_HOLE_*).
    through_cuts.append(
        _box((BATTERY_TRUNK_HOLE_X, BATTERY_TRUNK_HOLE_Y,
              CHASSIS_PLATE_T * 8.0),
             center=(BATTERY_TRUNK_HOLE_CENTRE[0],
                     BATTERY_TRUNK_HOLE_CENTRE[1], 0.0)))

    merged = _diff(merged, *through_cuts)

    # (d) late-Aug 2026: the 6 corner power-Wago tray WALL SETS grow
    #     straight out of the plate top face (integrated replacement for
    #     the separately printed + taped ``make_wago_mount`` trays).
    #     They sit at the hex corner flats (az = i*60 deg, r ~ 100),
    #     well outboard of every through-cut above, and print as plain
    #     vertical walls in the normal belly-down orientation.
    trays = []
    for M in wago_tray_corner_transforms():
        tw = _chassis_wago_tray_solid()
        tw.apply_transform(M)
        trays.append(tw)
    merged = _union(merged, *trays)
    return merged


def _chassis_bottom_floor_solid() -> trimesh.Trimesh:
    """Solid flat hex FLOOR slab folded onto ``make_chassis_bottom``'s
    underside (Jun 2026 single-part merge -- replaces the old bolt-on
    ``chassis_bottom_lower`` LOW half).

    GEOMETRY
    --------
    A solid regular hex prism, outer footprint = the chassis hex
    (``CHASSIS_FLAT_TO_FLAT``), spanning z in
    ``[CHASSIS_SPLIT_Z - CHASSIS_BOTTOM_FLOOR_T, CHASSIS_SPLIT_Z + 1]``
    (= [-6, -1]).  The nominal floor is [-6, -2]; the extra +1 mm overlaps the
    plate so ``make_chassis_bottom``'s union is a clean volumetric merge rather
    than a fragile coplanar kiss.

    Functional negative space (ONE batched diff -- no coplanar-faced unions, so
    nothing for the boolean kernel to shred):
      * 6 per-leg servo-body clearance cutouts (sized to the servo BODY +
        ``WELL_BODY_CL``, == the cradle's own body cavity -- NOT the larger
        ``WELL_W+2 x WELL_D+2`` well) so each yaw servo body passes down through
        the floor and hangs below while the floor still fills in UNDER the
        cradle walls to keep the -6 bottom face flush/flat for printing;
      * 6 per-leg harness-drop slots (the serial-bus wire exits);
      * 4-per-leg ``yaw_servo_retainer`` self-tap pilots (blind, opening at the
        -6 bottom face) in solid plate material on the tangential flanks of
        each body cutout, in two radial rows (Jul 2026 4-point anchor rework).

    NO join hardware: the HIGH/LOW split is gone, so the 12 join-bolt
    counterbores/clearances + 6 register dowels of the old LOW plate are
    deleted.

    STRUCTURAL 6-FOLD (C6) SYMMETRY
    -------------------------------
    ONE canonical 60-deg sector of cutters (one leg's body cutout +
    harness-drop slot + the 2 retainer pilots) is built ONCE, then replicated
    by an EXACT ``k * 60 deg`` rotation about the central Z axis for
    ``k = 0..5`` -- every leg is bit-identical up to its rotation and the hex
    prism is itself C6.  ``check_c6_symmetry`` probes THIS slab (the
    C6-bearing portion of the merged chassis_bottom); the full chassis_bottom
    legitimately breaks C6 only via the off-centre battery/electronics
    features, which live in the plate above, not in this floor.
    """
    floor_bot = CHASSIS_SPLIT_Z - CHASSIS_BOTTOM_FLOOR_T   # -6
    floor_top = CHASSIS_SPLIT_Z + 1.0                      # -1 (1 mm plate overlap)
    z_c = 0.5 * (floor_bot + floor_top)
    t = floor_top - floor_bot

    apothem = CHASSIS_FLAT_TO_FLAT / 2.0
    circum = apothem / np.cos(np.pi / 6.0)

    # ---- The slab: ONE solid regular-hex prism (inherently C6) ------------
    plate = _cyl(circum, t, sections=6)
    plate.apply_transform(rotation_matrix(np.pi / 6, [0, 0, 1]))
    plate.apply_translation([0.0, 0.0, z_c])

    # ---- Per-leg cradle-footprint pads ground the OUTBOARD cradle walls ----
    # The hex prism only reaches the apothem (radial 100 mm at each edge
    # midpoint), but each yaw cradle's outer shell OVERHANGS outboard past the
    # hex edge (to radial ~ apothem + outer_w/2 - SERVO_OUTPUT_X = 118.5 mm).
    # With no floor out there the cradle's outboard end-wall underside floats
    # at the -2 split plane (world +23.5).  Union a pad matching the cradle
    # outer-shell footprint under each leg so the floor reaches the FULL
    # cradle silhouette and grounds those outboard walls to the -6 bed.  This
    # is purely BELOW the deck (z in [-6, -1]); the leg/coxa mounts ~20 mm
    # ABOVE the deck and the swing relief sits at z = +4.25, so the pad adds
    # no collision envelope (the cradle TOWER already occupies this silhouette
    # above the plate).  The servo-body cutout below still punches through.
    outer_w = WELL_W + 2.0 + 2.0 * CRADLE_BOND_STRIP_MM
    outer_d = WELL_D + 2.0 + 2.0 * CRADLE_BOND_STRIP_MM
    pad_r = apothem - SERVO_OUTPUT_X            # cradle outer-shell centre radius
    pads: list[trimesh.Trimesh] = []
    for k in range(6):
        az = np.radians(30.0 + 60.0 * k)
        pad = _box((outer_w, outer_d, t))
        pad.apply_transform(rotation_matrix(az, [0, 0, 1]))
        pad.apply_translation([pad_r * np.cos(az), pad_r * np.sin(az), z_c])
        pads.append(pad)
    plate = _union(plate, *pads)

    # ---- ONE canonical 60-deg sector of cutters (built ONCE) --------------
    sector: list[trimesh.Trimesh] = []

    # Leg body-clearance cutout + serial-bus harness-drop slot at the canonical
    # leg azimuth (legs sit at the 6 hex-edge midpoints, 30 + k*60 deg).
    leg_az = np.radians(30.0)
    body_r = apothem - SERVO_OUTPUT_X
    # Servo-BODY clearance cutout (Jun 2026 flush-bottom fix).  Sized to the
    # servo body + WELL_BODY_CL on each face -- i.e. the SAME body cavity the
    # cradle uses in ``_chassis_yaw_cradle_solid`` -- NOT the larger
    # ``WELL_W+2 x WELL_D+2`` plate body cutout the cradle TOWER bonds into.
    # The old well-sized cutout stripped floor out under the cradle WALLS,
    # leaving their undersides floating at the split plane (z = CHASSIS_SPLIT_Z
    # = -2, world +23.5 -- 4 mm proud of the -6 / world +19.55 bed).  Sizing it
    # to the body lets the floor fill in UNDER the cradle walls all the way to
    # the -6 bed, so the chassis_bottom underside is ONE flush flat face (no
    # 4-mm-deep recesses -> fewer FDM supports); the servo body still drops
    # through with its usual 0.7 mm/face clearance.
    body_cut_w = SERVO_BODY_W + 2.0 * WELL_BODY_CL
    body_cut_d = SERVO_BODY_D + 2.0 * WELL_BODY_CL
    cutout = _box((body_cut_w, body_cut_d, t * 4.0))
    cutout.apply_transform(rotation_matrix(leg_az, [0, 0, 1]))
    cutout.apply_translation([body_r * np.cos(leg_az), body_r * np.sin(leg_az), z_c])
    sector.append(cutout)

    # Per-leg cable pass-through port (one open 18 x 28 slot since Aug 2026);
    # x = chassis radial (along leg_az), y = chassis tangential.  Shared with
    # the plate cuts via ``leg_harness_drop_slots()``.
    rad = np.array([np.cos(leg_az), np.sin(leg_az)])
    tan = np.array([-np.sin(leg_az), np.cos(leg_az)])
    edge = apothem * rad
    for sx, sy, sxe, sye in leg_harness_drop_slots():
        pxy = edge + sx * rad + sy * tan
        drop = _box((sxe, sye, t * 4.0))
        drop.apply_transform(rotation_matrix(leg_az, [0, 0, 1]))
        drop.apply_translation([pxy[0], pxy[1], z_c])
        sector.append(drop)

    # Yaw-servo retainer anchor pilots: 4 blind M3 self-tap holes (Jul 2026
    # 4-point rework -- was 2) in the floor BOTTOM at the tangential flanks of
    # the leg's body cutout, in two radial rows (outboard -12.5 + inboard -29),
    # where the retainer saddle (make_yaw_servo_retainer) bolts UP to tie the
    # hanging yaw servo to the chassis.  chassis_lower_retainer_anchor_centres
    # is the shared single source of truth for the saddle tabs + alignment guard.
    pil_h = RETAINER_PLATE_PILOT_DEPTH + 0.2
    for (cxr, cyt) in chassis_lower_retainer_anchor_centres():
        pxy = edge + cxr * rad + cyt * tan
        pil = _cyl(RETAINER_PLATE_PILOT_OD / 2.0, pil_h)
        pil.apply_translation([pxy[0], pxy[1], floor_bot - 0.2 + pil_h / 2.0])
        sector.append(pil)

    # ---- Replicate the sector by EXACT k*60 deg rotation, batch diff ------
    cutters: list[trimesh.Trimesh] = []
    for k in range(6):
        Rk = rotation_matrix(k * np.pi / 3.0, [0, 0, 1])
        for c in sector:
            ck = c.copy()
            ck.apply_transform(Rk)
            cutters.append(ck)

    return _diff(plate, *cutters)


# RETIRED (Jul 2026): ``make_battery_holder`` (the clip-in open-top LiPo
# tray) is deleted.  The holder was already out of the build list since
# the Jun 2026 deck redesign (the pack velcro-straps directly onto
# chassis_bottom); the dead builder and its BATTERY_FOOT_* / BATTERY_WALL
# constants went stale when BATTERY_W/D/H were re-pointed at the real
# 138 x 46 x 24 mm pack.


def _board_standoff_boss_and_pocket(
    x: float, y: float, *,
    pilot_od: float,
    pilot_depth: float,
    boss_od: float,
    boss_height: float,
    tray_top_z: float,
) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """Return ``(boss, pocket)`` for one printed board-standoff site.

    The boss is a vertical Phi ``boss_od`` cylinder spanning
    ``[tray_top_z, tray_top_z + boss_height]``.  The pocket is a Phi
    ``pilot_od`` cylinder spanning ``[tray_top_z + boss_height -
    pilot_depth - 0.5, tray_top_z + boss_height + 0.5]``.  The boss
    is to be UNIONed onto the tray and the pocket SUBTRACTed.  The
    pocket extends 0.5 mm above the boss top so the boolean diff
    clears the rim and 0.5 mm below the insert (room for debris).

    Mirrors the cradle-bolt boss/pocket pair (``_servo_cradle_
    insert_pockets``) but lives on top of the electronics tray
    instead of inside a servo cradle.  Used by ``make_electronics_
    tray`` for all 12 board-mount sites (4 x Mega, 4 x Pi, 4 x
    PCA9685).
    """
    boss = _cyl(boss_od / 2.0, boss_height)
    boss.apply_translation([x, y, tray_top_z + boss_height / 2.0])

    pocket_h = pilot_depth + 1.0
    pocket = _cyl(pilot_od / 2.0, pocket_h)
    pocket_z_centre = tray_top_z + boss_height - pilot_depth / 2.0
    pocket.apply_translation([x, y, pocket_z_centre])
    return boss, pocket


def _absolute_xy(centre: tuple[float, float],
                  offsets: tuple[tuple[float, float], ...]):
    """Helper: translate per-board hole offsets into tray-local
    absolute (x, y) tuples."""
    cx, cy = centre
    return tuple((cx + ox, cy + oy) for ox, oy in offsets)


def make_electronics_tray() -> trimesh.Trimesh:
    """Flat 3D-printed deck that carries the Raspberry Pi 4 (or Pi 5)
    + a single STS3215 serial-bus adapter.

    STS3215 refit (Jun 2026): the FEETECH STS3215 servos are serial
    bus servos driven DIRECTLY by the Pi over one half-duplex TTL bus,
    so the Arduino Mega 2560 and BOTH PCA9685 PWM drivers (and their
    16 board-mount fasteners) are GONE.  What remains is the Pi (its
    parametric PI_HOLES / PI_CENTRE pattern) plus the small USB-to-TTL
    bus adapter on the BUS_ADAPTER_HOLES / BUS_ADAPTER_CENTRE pattern.
    The 2x switching BECs / BEC cradle are also retired (servos run
    12 V direct from the 3S LiPo; the Pi is powered separately).

    Geometry summary:

        * Plate: ELEC_TRAY_W x ELEC_TRAY_D x ELEC_TRAY_T centred on
          tray-local (0, 0).  Tray-local origin = chassis (0, 0)
          after ``build_prototype_assembly`` places the tray.
        * 4 chassis-mount holes on the 35-mm-radius / 45-deg square
          pattern (= ELEC_CHASSIS_MOUNT_HOLES_XY, shared with
          chassis_top + chassis_bottom).  Each hole is Phi
          BRACKET_BOLT_HOLE = 3.4 mm with a Phi ELEC_CHASSIS_
          COUNTERBORE_OD = 5.5 mm x ELEC_CHASSIS_COUNTERBORE_DEPTH =
          2.0 mm counterbore from the tray's TOP face.  The cbore
          depth is INTENTIONALLY shallower than ELEC_TRAY_T = 3 mm so
          a 1 mm plastic rim survives at the cbore floor (annulus
          radius 1.7..2.75 mm) for the M3 SHCS head to clamp against
          -- boards on the ELEC_STANDOFF_H = 5 mm standoff bosses
          then clear the chassis bolt heads (which sit 1 mm below
          the tray top face) entirely.  The bolt threads DOWN
          through the tray and into an M3 heat-set insert captive in
          chassis_bottom's tray-mount boss (see ``make_chassis_
          bottom`` for the TRAY_MOUNT_BOSS_* geometry).  May 2026
          tray-mount fix: cbore depth was 3.0 mm before; the tray-
          mount audit caught that the cbore ate the full tray
          thickness, leaving NO rim for the head to clamp.
        * 4 printed bosses + heat-set insert pockets for the Pi 4
          (Phi 6 mm boss, Phi 3 mm pocket, McMaster 94459A106
          insert) -- 4 x M2.5 SHCS clamps the Pi onto the boss tops.
        * 4 printed bosses + heat-set insert pockets for the STS3215
          bus adapter at BUS_ADAPTER_CENTRE (same Phi 6 / 3 mm M2.5
          geometry as the Pi).

    All 8 board-mount fasteners are CAPTIVE SUB-ASSEMBLY fasteners:
    they are torqued during the electronics install BEFORE the
    chassis_top + arm stack closes over the tray; once the top plate
    is on, a hex key cannot reach the heads.  ``_emit_electronics_
    tray_fasteners`` in fastener_registry.py flags them with
    ``skip_screwdriver_reason`` accordingly.
    """
    plate = _box((ELEC_TRAY_W, ELEC_TRAY_D, ELEC_TRAY_T),
                 center=(0, 0, ELEC_TRAY_T / 2.0))

    tray_top_z = ELEC_TRAY_T

    bosses: list[trimesh.Trimesh] = []
    pockets: list[trimesh.Trimesh] = []

    # --- Pi 4 board-mount sites (M2.5 inserts) ---
    for (hx, hy) in _absolute_xy(PI_CENTRE, PI_HOLES):
        boss, pocket = _board_standoff_boss_and_pocket(
            hx, hy,
            pilot_od=INSERT_M25_PILOT_OD,
            pilot_depth=INSERT_M25_PILOT_DEPTH,
            boss_od=ELEC_BOSS_OD_M25,
            boss_height=ELEC_STANDOFF_H,
            tray_top_z=tray_top_z,
        )
        bosses.append(boss)
        pockets.append(pocket)

    # --- STS3215 serial-bus adapter board-mount sites (M2.5 inserts) ---
    # Replaces the 2x PCA9685 + Arduino Mega sites (STS3215 refit).
    for (hx, hy) in _absolute_xy(BUS_ADAPTER_CENTRE, BUS_ADAPTER_HOLES):
        boss, pocket = _board_standoff_boss_and_pocket(
            hx, hy,
            pilot_od=INSERT_M25_PILOT_OD,
            pilot_depth=INSERT_M25_PILOT_DEPTH,
            boss_od=ELEC_BOSS_OD_M25,
            boss_height=ELEC_STANDOFF_H,
            tray_top_z=tray_top_z,
        )
        bosses.append(boss)
        pockets.append(pocket)

    # --- 4 chassis-mount holes (35 mm radius / 45 deg square) ---
    # Phi BRACKET_BOLT_HOLE = 3.4 mm M3 clearance through the full
    # ELEC_TRAY_T thickness, with a Phi ELEC_CHASSIS_COUNTERBORE_OD =
    # 5.5 mm x ELEC_CHASSIS_COUNTERBORE_DEPTH = 3.0 mm counterbore
    # from the tray's TOP face so the chassis-side M3 SHCS head sits
    # flush with the tray top (the head face ends up coincident with
    # tray-local z = 0).  Boards on the 5 mm standoffs above the
    # tray then have an unobstructed 5 mm of clear air above the
    # tray face -- no PCB-vs-bolt-head conflict.
    #
    # ELEC_CHASSIS_MOUNT_HOLES_XY is the CHASSIS-frame pattern (35 mm
    # radius / 45 deg square = (+/-24.75, +/-24.75)).  We subtract
    # (ELEC_TRAY_CENTRE_X, ELEC_TRAY_CENTRE_Y) so the holes' tray-
    # local positions still map onto the chassis pattern after the
    # tray is translated.  Pre-May-2026 the tray centre was (0, 0)
    # and chassis-frame == tray-local; the May 2026 "Pi cantilever"
    # pass shifted the tray centre to (0, -2.5) -- without this
    # subtraction the tray bolts would no longer line up with
    # chassis_top + chassis_bottom's centre hole pattern.
    mount_holes: list[trimesh.Trimesh] = []
    for (mx_chassis, my_chassis) in ELEC_CHASSIS_MOUNT_HOLES_XY:
        mx = mx_chassis - ELEC_TRAY_CENTRE_X
        my = my_chassis - ELEC_TRAY_CENTRE_Y
        through = _cyl(BRACKET_BOLT_HOLE / 2.0, ELEC_TRAY_T * 4.0)
        through.apply_translation([mx, my, ELEC_TRAY_T / 2.0])
        mount_holes.append(through)

        cbore = _cyl(ELEC_CHASSIS_COUNTERBORE_OD / 2.0,
                     ELEC_CHASSIS_COUNTERBORE_DEPTH + 0.4)
        cbore_z_centre = ELEC_TRAY_T - ELEC_CHASSIS_COUNTERBORE_DEPTH / 2.0
        cbore.apply_translation([mx, my, cbore_z_centre + 0.2])
        mount_holes.append(cbore)

    body = _union(plate, *bosses)
    return _diff(body, *pockets, *mount_holes)


def _deck_tray_base(
    board_centre: tuple[float, float],
    board_holes: tuple[tuple[float, float], ...],
    *,
    extra_cuts: tuple[trimesh.Trimesh, ...] = (),
    ziptie_rows: tuple[float, ...] = (),
) -> trimesh.Trimesh:
    """Shared builder for the two stacked electronics decks.

    Produces a ``DECK_TRAY_W x DECK_TRAY_D x DECK_TRAY_T`` plate with:

        * 4 printed board-standoff bosses (M3 heat-set inserts) on the
          ``board_holes`` pattern offset to ``board_centre`` -- the board
          bolts DOWN into these from above.
        * 4 column-bolt bosses + M3 through-holes on ``DECK_COLUMN_XY`` so
          the tray clamps onto the standoff columns that rise from
          chassis_top (lower tray) or from the tray below (upper tray).
        * Zip-tie slot pairs on each row in ``ziptie_rows`` (Y positions)
          flanking the board for cable strain relief, plus a row of spare
          M3 bolt-down holes between the columns.
        * Any ``extra_cuts`` the caller wants (e.g. cooling vents).

    The board sits on bosses ``DECK_STANDOFF_BOSS_H`` above the plate top so
    its bottom-side components clear the plate; the column bosses are taller
    so a brass standoff seats flat against the boss top, not the thin plate.
    """
    plate = _box((DECK_TRAY_W, DECK_TRAY_D, DECK_TRAY_T),
                 center=(0, 0, DECK_TRAY_T / 2.0))
    tray_top_z = DECK_TRAY_T

    bosses: list[trimesh.Trimesh] = []
    pockets: list[trimesh.Trimesh] = []
    cuts: list[trimesh.Trimesh] = list(extra_cuts)

    # Board-standoff bosses (M3 inserts) on the board's own hole pattern.
    for (hx, hy) in _absolute_xy(board_centre, board_holes):
        boss, pocket = _board_standoff_boss_and_pocket(
            hx, hy,
            pilot_od=INSERT_M3_PILOT_OD,
            pilot_depth=INSERT_M3_PILOT_DEPTH,
            boss_od=DECK_BOSS_OD_M3,
            boss_height=DECK_STANDOFF_BOSS_H,
            tray_top_z=tray_top_z,
        )
        bosses.append(boss)
        pockets.append(pocket)

    # Column-bolt bosses + M3 through-holes (clamp onto the standoff column).
    for (cx, cy) in DECK_COLUMN_XY:
        col_boss = _cyl(DECK_COLUMN_BOSS_OD / 2.0, DECK_STANDOFF_BOSS_H)
        col_boss.apply_translation([cx, cy, tray_top_z + DECK_STANDOFF_BOSS_H / 2.0])
        bosses.append(col_boss)
        through = _cyl(BRACKET_BOLT_HOLE / 2.0,
                       DECK_TRAY_T + DECK_STANDOFF_BOSS_H + 2.0)
        through.apply_translation([cx, cy,
                                   (DECK_TRAY_T + DECK_STANDOFF_BOSS_H) / 2.0])
        cuts.append(through)

    # Zip-tie slot pairs flanking the board + a spare bolt-down hole grid.
    for row_y in ziptie_rows:
        for sx in (-1, 1):
            slot = _box((DECK_ZIPTIE_SLOT_W, DECK_ZIPTIE_SLOT_L,
                         DECK_TRAY_T + 2.0),
                        center=(sx * (DECK_TRAY_W / 2.0 - 6.0), row_y,
                                DECK_TRAY_T / 2.0))
            cuts.append(slot)
    for sx in (-1, 1):
        hole = _cyl(DECK_SPARE_HOLE_OD / 2.0, DECK_TRAY_T + 2.0)
        hole.apply_translation([sx * 9.0, 0.0, DECK_TRAY_T / 2.0])
        cuts.append(hole)

    body = _union(plate, *bosses)
    return _diff(body, *pockets, *cuts)


def make_uno_q_tray() -> trimesh.Trimesh:
    """RETIRED (Aug 2026 as-built stack) — kept for legacy quotes only.

    Live robot uses the magnet-held Ø110 hex mount plate instead of a
    printed Uno Q tray.  See ``asbuilt_electronics_local_parts``.
    """
    return _deck_tray_base(
        (0.0, 0.0), UNO_Q_HOLES,
        ziptie_rows=(-UNO_Q_PCB_D / 2.0 - 4.0, UNO_Q_PCB_D / 2.0 + 4.0),
    )


def make_buck_tray() -> trimesh.Trimesh:
    """RETIRED (Aug 2026 as-built stack) — no buck on the live robot.

    Battery feeds the trunk Wagos and Uno Q directly.  Builder kept only
    so old tray references can still regenerate the mesh if needed.
    """
    vents: list[trimesh.Trimesh] = []
    for vx in (-12.0, 0.0, 12.0):
        vent = _box((DECK_VENT_SLOT_W, DECK_VENT_SLOT_L, DECK_TRAY_T + 2.0),
                    center=(vx, 0.0, DECK_TRAY_T / 2.0))
        vents.append(vent)
    return _deck_tray_base(
        (0.0, 0.0), BUCK_HOLES,
        extra_cuts=tuple(vents),
        ziptie_rows=(-BUCK_PCB_D / 2.0 - 4.0, BUCK_PCB_D / 2.0 + 4.0),
    )


def _half_ellipsoid(ax: float, ay: float, hd: float,
                    subdivisions: int = 4) -> trimesh.Trimesh:
    """Solid upper half-ellipsoid (a dome) with its flat rim on z = 0 and
    apex at z = hd.  Built from a unit icosphere scaled to (ax, ay, hd),
    then the lower half (z < 0) sliced off so the rim lands on z = 0."""
    sph = trimesh.creation.icosphere(subdivisions=subdivisions, radius=1.0)
    sph.apply_scale((ax, ay, hd))
    # Slice off everything below z = 0 (keep the dome).
    cutter = _box((4.0 * ax, 4.0 * ay, 4.0 * hd),
                  center=(0.0, 0.0, -2.0 * hd))
    return _diff(sph, cutter)


def make_spider_carapace() -> trimesh.Trimesh:
    """RETIRED (Aug 2026 as-built stack) — cosmetic dome no longer printed.

    Live robot uses the hex mount plate + raised platform instead.
    Builder kept for legacy quotes / mesh regeneration only.

    Local frame: origin at the chassis centre on the carapace SEAT/RIM
    plane (local z = 0), +X = forward (anterior; the 8-eye face lives on
    this front slope), +Y = lateral, +Z = up.
    """
    ax, ay, hd = CARAPACE_AX, CARAPACE_AY, CARAPACE_HD
    w = CARAPACE_WALL
    outer = _half_ellipsoid(ax, ay, hd)
    inner = _half_ellipsoid(ax - w, ay - w, hd - w)
    shell = _diff(outer, inner)

    # --- Mount feet at the 4 deck columns -------------------------------
    feet: list[trimesh.Trimesh] = []
    pockets: list[trimesh.Trimesh] = []
    for (cx, cy) in DECK_COLUMN_XY:
        # Inner-surface height at this column so the boss fuses into the
        # shell; add a margin and clamp so it always reaches the shell.
        frac = 1.0 - (cx / (ax - w)) ** 2 - (cy / (ay - w)) ** 2
        inner_z = (hd - w) * np.sqrt(max(frac, 0.04))
        boss_h = inner_z + 4.0
        boss = _cyl(CARAPACE_FOOT_OD / 2.0, boss_h)
        boss.apply_translation([cx, cy, boss_h / 2.0])
        feet.append(boss)
        # M3 heat-set insert pocket opening DOWN from the seat face.
        pocket = _cyl(INSERT_M3_PILOT_OD / 2.0, INSERT_M3_PILOT_DEPTH + 0.4)
        pocket.apply_translation(
            [cx, cy, (INSERT_M3_PILOT_DEPTH + 0.4) / 2.0 - 0.2])
        pockets.append(pocket)

    body = _union(shell, *feet)

    # --- Rear (-X) wire-exit / vent window through the lower shell ------
    rear_window = _box((40.0, 34.0, 2.0 * (hd - w) + 4.0),
                       center=(-(ax - 6.0), 0.0, (hd - w)))
    # Trim the window so it only opens the lower band (keep an arch above).
    rear_cap = _box((60.0, 40.0, 40.0), center=(-(ax - 6.0), 0.0, 17.0 + 20.0))
    rear_window = _diff(rear_window, rear_cap)
    cuts: list[trimesh.Trimesh] = [rear_window]

    # --- Lateral cooling-vent slots on each flank ----------------------
    for sy in (-1.0, 1.0):
        for vx in (-18.0, 0.0, 18.0):
            slot = _cyl(2.0, 30.0)
            slot.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))
            slot.apply_translation([vx, sy * (ay - 8.0), 9.0])
            cuts.append(slot)

    body = _diff(body, *pockets, *cuts)

    # --- 8 spider eyes: raised domed lenses on the front slope ----------
    eyes: list[trimesh.Trimesh] = []
    for (ex, ey, er) in CARAPACE_EYES:
        for sy in (-1.0, 1.0):
            yy = sy * ey
            frac = 1.0 - (ex / ax) ** 2 - (yy / ay) ** 2
            if frac <= 0.0:
                continue
            ez = hd * np.sqrt(frac)
            lens = trimesh.creation.icosphere(subdivisions=3, radius=er)
            lens.apply_translation([ex, yy, ez])
            eyes.append(lens)
    carapace = _union(body, *eyes)
    return carapace


def make_uno_q_visual() -> trimesh.Trimesh:
    """Visual-only proxy for the Arduino Uno Q (NOT printed).  PCB slab +
    a USB-C jack stub + a header-strip block so cable-clearance checks and
    BuildViz have a real envelope to test against."""
    pcb_t = 1.6
    pcb = _box((UNO_Q_PCB_W, UNO_Q_PCB_D, pcb_t), center=(0, 0, pcb_t / 2.0))
    parts = [pcb]
    # USB-C power/data jack on the -X short edge.
    usb = _box((8.0, 9.0, 3.2),
               center=(-UNO_Q_PCB_W / 2.0 + 4.0, 14.0, pcb_t + 1.6))
    parts.append(usb)
    # Header pin strips along both long edges (Arduino shield headers).
    for sy in (-1, 1):
        strip = _box((50.0, 2.6, 9.0),
                     center=(0.0, sy * (UNO_Q_PCB_D / 2.0 - 3.0),
                             pcb_t + 4.5))
        parts.append(strip)
    return _union(*parts)


def make_buck_converter_visual() -> trimesh.Trimesh:
    """Visual-only proxy for the XINGYHENG buck converter (NOT printed).

    RETIRED from the live as-built stack (Aug 2026: no buck — battery
    feeds PDB and Uno Q directly).  Kept so legacy meshes / MuJoCo
    fallbacks can still rebuild the envelope.
    """
    pcb_t = 1.6
    pcb = _box((BUCK_PCB_W, BUCK_PCB_D, pcb_t), center=(0, 0, pcb_t / 2.0))
    parts = [pcb]
    inductor = _box((18.0, 18.0, BUCK_PCB_H - pcb_t),
                    center=(6.0, 0.0, pcb_t + (BUCK_PCB_H - pcb_t) / 2.0))
    parts.append(inductor)
    trimpot = _box((8.0, 8.0, 6.0),
                   center=(-14.0, 10.0, pcb_t + 3.0))
    parts.append(trimpot)
    for sx in (-1, 1):
        term = _box((10.0, 9.0, 9.0),
                    center=(sx * (BUCK_PCB_W / 2.0 - 6.0), -14.0, pcb_t + 4.5))
        parts.append(term)
    return _union(*parts)


# ---------------------------------------------------------------------------
# As-built electronics visuals (Aug 2026)
# ---------------------------------------------------------------------------

# Aug 2026: ``make_pdb_visual`` DELETED -- the as-built robot has no
# PDB.  The battery trunk splices straight into Wago 221 lever nuts
# (see ``WAGO_TRUNK_CENTRE`` and ``asbuilt_electronics_local_parts``).


def make_motor_controller_visual() -> trimesh.Trimesh:
    """USB/TTL motor-controller brick placeholder (NOT printed).
    Origin = body bottom centre."""
    return _box((MOTOR_CTRL_W, MOTOR_CTRL_D, MOTOR_CTRL_H),
                center=(0, 0, MOTOR_CTRL_H / 2.0))


def make_breakout_visual() -> trimesh.Trimesh:
    """Generic shield / breakout next to Uno Q on the hex plate.
    Origin = PCB bottom."""
    pcb = _box((BREAKOUT_W, BREAKOUT_D, 1.6), center=(0, 0, 0.8))
    header = _box((BREAKOUT_W - 6.0, 6.0, BREAKOUT_H - 1.6),
                  center=(0, 0, 1.6 + (BREAKOUT_H - 1.6) / 2.0))
    return _union(pcb, header)


def make_screen_visual() -> trimesh.Trimesh:
    """63×35 mm display panel on the raised platform (NOT printed).
    Origin = panel bottom centre."""
    return _box((SCREEN_PCB_W, SCREEN_PCB_D, SCREEN_PCB_T),
                center=(0, 0, SCREEN_PCB_T / 2.0))


def make_hex_post_standoff_visual() -> trimesh.Trimesh:
    """20 mm M3 brass standoff body (NOT printed).  Origin = bottom."""
    return _cyl_along(HEX_POST_STANDOFF_OD / 2.0, HEX_POST_STANDOFF_H,
                      axis="z", sections=12)


def make_chassis_standoff_visual() -> trimesh.Trimesh:
    """32 mm M3 brass standoff between chassis_bottom and chassis_top
    (NOT printed; BOM hardware).  Spans CHASSIS_GAP exactly -- bottom
    plate's top face up to the top plate's underside.  Origin = bottom."""
    return _cyl_along(HEX_POST_STANDOFF_OD / 2.0, CHASSIS_GAP,
                      axis="z", sections=12)


def make_hex_post_thumb_nut_visual() -> trimesh.Trimesh:
    """M3 knurled thumb nut ~2.5 mm thick (NOT printed).  Origin = bottom."""
    return _cyl_along(HEX_POST_THUMB_NUT_OD / 2.0, HEX_POST_THUMB_NUT_T,
                      axis="z", sections=16)


def make_hex_post_magnet_visual() -> trimesh.Trimesh:
    """Ø8 × 8 mm disc magnet (NOT printed).  Origin = bottom."""
    return _cyl_along(HEX_POST_MAGNET_OD / 2.0, HEX_POST_MAGNET_H,
                      axis="z", sections=24)


def make_wago_visual() -> trimesh.Trimesh:
    """Wago 221-style lever-nut placeholder box (NOT printed).
    Origin = body centre."""
    return _box((WAGO_W, WAGO_D, WAGO_H), center=(0, 0, 0))


def make_wago3_visual() -> trimesh.Trimesh:
    """3-port Wago 221-413 placeholder box (NOT printed).

    Local frame: +X = wire direction (depth 18.6), Y = across the three
    ports (width 18.7), origin = body BOTTOM centre (drops straight onto
    the plate top face between the integrated corner-tray walls).
    """
    return _box((WAGO3_D, WAGO3_W, WAGO3_H), center=(0, 0, WAGO3_H / 2.0))


def make_wago5_visual() -> trimesh.Trimesh:
    """5-port Wago 221-415 placeholder box (NOT printed).

    Local frame: +X = wire direction (depth 18.6), Y = across the five
    ports (width 30.0), origin = body BOTTOM centre.
    """
    return _box((WAGO5_D, WAGO5_W, WAGO5_H), center=(0, 0, WAGO5_H / 2.0))


def _chassis_wago_tray_solid() -> trimesh.Trimesh:
    """Wall set of ONE single-bay corner power-Wago tray, INTEGRATED into
    the chassis_bottom top face (late-Aug 2026 -- replaces the separate
    printed + VHB-taped ``make_wago_mount``; -6 parts, no tape, and the
    inward wire pull is reacted by solid plate instead of adhesive).

    Aug 16 2026: ONE 5-port 221-415 bay per corner (user -- was a two-bay
    3-port V+/GND pair), PRESS-FIT: WAGO_MOUNT_BAY_CLEAR is now a 0.15 mm
    nominal interference, so the nut wedges between the walls instead of
    rattling.

    Two side walls + outer (outboard) wall growing straight out of the
    plate top face; no floor (the plate IS the floor), no divider, no
    over-edge lip.  Wire entries + levers face INBOARD (local -X = toward
    the chassis centre); the walls stop ~1.9 mm below the nut top so the
    nut is easy to grip.  The chassis prints belly on the bed / tower up,
    so these top-face walls print as plain vertical extrusions -- no
    supports.

    Local frame: origin = bay-envelope centre ON the plate top face,
    +X = outboard (radial), +Y = tangential, +Z = up.  Walls extend
    1 mm below z = 0 so the union into the plate is volumetric, never a
    coplanar kiss.
    """
    bay_w = WAGO5_W + WAGO_MOUNT_BAY_CLEAR      # 29.85 tangential (Y)
    bay_d = WAGO5_D + WAGO_MOUNT_BAY_CLEAR      # 18.45 radial (X)
    t = WAGO_MOUNT_WALL_T
    half_x = bay_d / 2.0 + t                    # outer wall + front ledge
    half_y = bay_w / 2.0 + t                    # 1 bay + 2 side walls
    emb = 1.0                                   # plate-overlap embed depth
    h = WAGO_MOUNT_WALL_H + emb
    z_c = WAGO_MOUNT_WALL_H / 2.0 - emb / 2.0
    outer = _box((t, 2 * half_y, h), center=(half_x - t / 2.0, 0, z_c))
    walls = [
        _box((2 * half_x, t, h),
             center=(0, s * (half_y - t / 2.0), z_c))
        for s in (-1.0, 1.0)
    ]
    return _union(outer, *walls)


def wago_tray_corner_transforms() -> "list[np.ndarray]":
    """Chassis-frame (pre-lift) 4x4 for each of the 6 corner-flat Wago
    tray wall sets (and their nut pairs): local +X rotated to the
    corner's outward normal (az = i*60 deg), local z = 0 on the plate
    TOP face, outer wall's outboard face flush with the plate edge at
    chassis r = ``WAGO_MOUNT_EDGE_R``."""
    from trimesh.transformations import (
        translation_matrix as _T,
        rotation_matrix as _R,
    )
    bay_d = WAGO5_D + WAGO_MOUNT_BAY_CLEAR
    r0 = WAGO_MOUNT_EDGE_R - WAGO_MOUNT_WALL_T - bay_d / 2.0
    out = []
    for i in range(6):
        a = i * np.pi / 3.0
        out.append(_T([r0 * np.cos(a), r0 * np.sin(a),
                       CHASSIS_PLATE_T / 2.0]) @ _R(a, [0, 0, 1]))
    return out


def load_hex_mount_plate() -> trimesh.Trimesh:
    """Round Ø115 mount plate from ``extra_stl/`` (3 raised-stand foot
    holes, the live +/-31.1 standoff square, the Uno Q's 3-point
    Arduino-UNO mount holes (late-Aug 2026 review round 2: the separate
    hex_uno_q_io_board is retired; M3 x 8 down into thumb nuts), 3.3 V-
    Wago wire ports, zip-tie slots, and 4 underside magnet shear-
    registration bosses; the legacy 49.5 mm bolt square is dropped.
    Print-only -- the SVG cut file is retired (bosses can't be cut).

    Plate midplane at local z = 0 (plate z in [-1, +1]; the registration
    bosses reach down to z = -3 and socket the magnet tops).
    """
    path = os.path.join(EXTRA_STL_DIR,
                        "round_mount_plate_115_with_leg_holes.stl")
    if not os.path.isfile(path):
        # Fallback: plain 2 mm disc, no holes.
        return trimesh.creation.cylinder(
            radius=HEX_MOUNT_PLATE_DIAM / 2.0,
            height=HEX_MOUNT_PLATE_T, sections=64)
    return trimesh.load(path, force="mesh")


def load_hex_raised_platform() -> trimesh.Trimesh:
    """Screen stand from ``extra_stl/`` ("hex" name is historical --
    since the late-Aug 2026 review round 2 the top is a ROUND Ø115 disc
    matching the plate below, on 3 legs at az 90/210/330 whose feet
    carry blind self-tap pilots: 3x M3 x 8 drive UP through the plate).

    Foot bottoms at local z = 0; top of upper disc at
    z = HEX_RAISED_TOTAL_H (30 since the late-Aug 2026 72 -> 28 mm
    leg shortening).
    """
    path = os.path.join(
        EXTRA_STL_DIR,
        f"hex_raised_platform_110_h{HEX_RAISED_LEG_H:.0f}_screen.stl")
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"missing {path} — run tools/make_xtool_hex_raised_platform.py")
    return trimesh.load(path, force="mesh")


def asbuilt_electronics_local_parts(
) -> list[tuple[str, trimesh.Trimesh, np.ndarray]]:
    """``(name, local_mesh, M0)`` for the as-built electronics stack.

    ``M0`` is the PRE-LIFT chassis-frame transform (z = 0 at chassis_bottom
    mesh centre), matching ``_body_local_parts`` / BuildViz convention.
    Chassis_top top face = ``deck0 = CHASSIS_TOP_TOP_Z``.
    """
    from trimesh.transformations import (
        translation_matrix as _T,
        rotation_matrix as _R,
    )

    gap = CHASSIS_GAP
    plate = CHASSIS_PLATE_T
    deck0 = CHASSIS_TOP_TOP_Z          # chassis_top top face

    out: list[tuple[str, trimesh.Trimesh, np.ndarray]] = []

    # Central trunk power Wagos (V+ / GND splice pair of 5-port 221-415)
    # + motor controller on chassis_top.  As-built Aug 2026: no PDB --
    # the fused battery trunk lands on these two lever nuts near the
    # plate centre (entries facing +X, toward the switch), which fan out
    # the six branches to the corner power Wago pairs.
    for dy in (-WAGO_TRUNK_DY / 2.0, WAGO_TRUNK_DY / 2.0):
        out.append(("wago_trunk", make_wago5_visual(),
                    _T([WAGO_TRUNK_CENTRE[0], WAGO_TRUNK_CENTRE[1] + dy,
                        deck0])))
    mc = make_motor_controller_visual()
    out.append(("motor_controller", mc,
                _T([MOTOR_CTRL_CENTRE[0], MOTOR_CTRL_CENTRE[1], deck0])
                @ _R(np.deg2rad(MOTOR_CTRL_YAW_DEG), [0, 0, 1])))

    # Inter-plate structure: four 32 mm M3 brass standoffs (BOM hardware)
    # spanning chassis_bottom's top face up to chassis_top's underside at
    # CHASSIS_STANDOFF_HOLES_XY.  Previously unmodeled, which left the
    # top deck visually levitating in BuildViz.
    for (cx, cy) in CHASSIS_STANDOFF_HOLES_XY:
        out.append(("chassis_standoff", make_chassis_standoff_visual(),
                    _T([cx, cy, plate / 2.0])))

    # Four post stacks at CHASSIS_STANDOFF_HOLES_XY.
    for (cx, cy) in CHASSIS_STANDOFF_HOLES_XY:
        z = deck0
        out.append(("hex_post_standoff",
                    make_hex_post_standoff_visual(),
                    _T([cx, cy, z])))
        z += HEX_POST_STANDOFF_H
        out.append(("hex_post_thumb_nut",
                    make_hex_post_thumb_nut_visual(),
                    _T([cx, cy, z])))
        z += HEX_POST_THUMB_NUT_T
        out.append(("hex_post_magnet",
                    make_hex_post_magnet_visual(),
                    _T([cx, cy, z])))

    # Round mount plate sits on magnet tops (midplane = magnet_top + T/2).
    hex_mid_z = deck0 + HEX_POST_STACK_H + HEX_MOUNT_PLATE_T / 2.0
    out.append(("hex_mount_plate", load_hex_mount_plate(),
                _T([0.0, 0.0, hex_mid_z])))
    hex_top_z = hex_mid_z + HEX_MOUNT_PLATE_T / 2.0

    # 3.3 V rail splice: one 5-port Wago VHB'd flat to the plate UNDERSIDE
    # near the south rim, entries facing -Y (see WAGO_V33_CENTRE).
    plate_under_z = hex_mid_z - HEX_MOUNT_PLATE_T / 2.0
    out.append(("wago_v33", make_wago5_visual(),
                _T([WAGO_V33_CENTRE[0], WAGO_V33_CENTRE[1],
                    plate_under_z - WAGO5_H])
                @ _R(-np.pi / 2.0, [0, 0, 1])))

    # Uno Q + breakout on hex plate top.
    out.append(("uno_q", make_uno_q_visual(),
                _T([UNO_Q_ON_HEX_CENTRE[0], UNO_Q_ON_HEX_CENTRE[1],
                    hex_top_z])))
    out.append(("breakout", make_breakout_visual(),
                _T([BREAKOUT_CENTRE[0], BREAKOUT_CENTRE[1], hex_top_z])))

    # Raised platform + screen (MPU is NOT under the platform — see below).
    out.append(("hex_raised_platform", load_hex_raised_platform(),
                _T([0.0, 0.0, hex_top_z])))
    raised_top_z = hex_top_z + HEX_RAISED_TOTAL_H
    out.append(("screen", make_screen_visual(),
                _T([0.0, 0.0, raised_top_z])))

    # MPU-6050 glued on chassis_top just south of the trunk Wago pair
    # (inboard, close to the yaw axis; header row faces the open -X deck).
    mpu_z = deck0 + IMU_PCB_T / 2.0
    mpu_M = (_T([MPU_ASBUILT_CENTRE[0], MPU_ASBUILT_CENTRE[1], mpu_z])
             @ _R(np.deg2rad(MPU_ASBUILT_YAW_DEG), [0, 0, 1]))
    out.append(("mpu6050", make_mpu6050_visual(), mpu_M))

    # Per-leg power splices: ONE 5-port Wago 221-415 press-fit in the
    # single-bay tray walls INTEGRATED into the chassis_bottom top face
    # at each hex corner flat (Aug 16 2026, user -- was a V+/GND pair of
    # 3-port nuts in a two-bay tray), wire entries facing the chassis
    # centre.  The walls are part of the chassis mesh; only the nut is
    # placed here, sitting directly on the plate top face.
    for M in wago_tray_corner_transforms():
        out.append(("wago_power", make_wago5_visual(), M))

    # Data Wagos under chassis, one per hex VERTEX azimuth (see WAGO_DATA_R:
    # the leg azimuths at low r now belong to the under-belly battery
    # packs).  Hung from the TRUE flat belly at z = -6 (plate + 4 mm floor
    # band), not the bare plate underside.
    belly_z = -(plate / 2.0 + 4.0)
    for i in range(6):
        a = i * np.pi / 3.0
        x = WAGO_DATA_R * np.cos(a)
        y = WAGO_DATA_R * np.sin(a)
        out.append(("wago_data", make_wago_visual(),
                    _T([x, y, belly_z - WAGO_H / 2.0])))

    # Two under-belly shorty LiPo packs (see BATTERY_* constants).
    from trimesh.creation import box as _box_mesh
    for M in battery_pack_transforms():
        out.append(("lipo_battery",
                    _box_mesh(extents=(BATTERY_W, BATTERY_D, BATTERY_H)), M))

    return out


def make_bec_cradle() -> trimesh.Trimesh:
    """Snap-fit cradle for the 2 x 5V 5A switching BECs (Hobbywing
    UBEC form factor, ~ 24 x 15 x 8 mm body).

    Sits on top of the electronics_tray in the corridor between the
    Mega's +X edge and PCA1's -X edge.  No bolts -- the 2 BEC bodies
    wedge in side-by-side along Y with a BEC_CRADLE_INTERFERENCE-mm
    interference fit and a 2 mm-tall retention lip on each +/- Y
    side wall snaps over the BEC's top edge so it stays captive
    against vibration.  Both XT60 input pigtails exit out the +X end
    wall and both 3-pin servo-header output pigtails exit out the
    -X end wall via Phi BEC_PIGTAIL_OD = 5 mm wire channels.

    Local frame (mesh origin = cradle centre on the tray's top face):
        +X = pigtail axis (input on +X end, output on -X end)
        +Y = wider axis -- 2 BECs span this dimension
        +Z = UP (cradle floor at z = 0, cavity extends to z =
              BEC_CRADLE_FLOOR + BEC_BODY_H)

    The cradle's CHASSIS-frame placement is
    ``(ELEC_TRAY_CENTRE_X + BEC_CRADLE_CENTRE[0],
       ELEC_TRAY_CENTRE_Y + BEC_CRADLE_CENTRE[1],
       CHASSIS_PLATE_T / 2 + 3 + ELEC_TRAY_T)`` -- i.e. on the
    electronics_tray's top face, in the corridor between Mega +X
    edge (+22.75 mm) and PCA1 -X edge (+51.5 mm).
    """
    cavity_l = BEC_BODY_L
    cavity_w = 2.0 * BEC_BODY_W - BEC_CRADLE_INTERFERENCE
    cavity_h = BEC_BODY_H

    outer_l = cavity_l + 2.0 * BEC_CRADLE_WALL
    outer_w = cavity_w + 2.0 * BEC_CRADLE_WALL
    outer_h = BEC_CRADLE_FLOOR + cavity_h + BEC_CRADLE_LIP_H

    # Solid outer block.
    block = _box((outer_l, outer_w, outer_h),
                  center=(0.0, 0.0, outer_h / 2.0))

    # Subtract the main cavity (BEC body space).  The cavity opens
    # UP to z = floor + body_h; the retention lip lives ABOVE that
    # at z in [floor + body_h, floor + body_h + lip_h] but tapers
    # inboard by BEC_CRADLE_LIP_INSET on each +/- Y side.
    cavity = _box((cavity_l, cavity_w, cavity_h + 0.5),
                   center=(0.0, 0.0,
                            BEC_CRADLE_FLOOR + cavity_h / 2.0 + 0.25))

    # Upper insertion slot (above the lip): narrower in Y so the
    # BEC body has to flex past the retention lip to drop in.
    insert_slot = _box(
        (cavity_l,
         cavity_w - 2.0 * BEC_CRADLE_LIP_INSET,
         BEC_CRADLE_LIP_H + 0.5),
        center=(0.0, 0.0,
                 BEC_CRADLE_FLOOR + cavity_h
                 + BEC_CRADLE_LIP_H / 2.0 + 0.25),
    )

    # Wire-exit channels on +/- X end walls.  Centred vertically on
    # the BEC body (at z = floor + body_h/2).  One channel per BEC
    # would be ideal but a single wider opening that spans both
    # BECs' Y range is mechanically simpler and the user will fish
    # both pigtails through the same end.  Channel height = Phi 5
    # round; widened in Y to cover both BEC pigtail positions
    # (centres at y = +/- (cavity_w/4)).
    channels = []
    for sx in (-1, 1):
        # Single round Phi 5 channel per BEC -- 2 channels per end,
        # 4 total.
        for sy in (-1, 1):
            ch = _cyl_along(BEC_PIGTAIL_OD / 2.0,
                             BEC_CRADLE_WALL + 0.4,
                             axis="x")
            cx = sx * (cavity_l / 2.0 - 0.2)
            cy = sy * (cavity_w / 4.0)
            cz = BEC_CRADLE_FLOOR + cavity_h / 2.0
            ch.apply_translation([cx, cy, cz])
            channels.append(ch)

    return _diff(block, cavity, insert_slot, *channels)


def make_switch_holster() -> trimesh.Trimesh:
    """Printed holster for the anti-spark XT60 on/off switch.

    VELCROS to chassis_top's +X edge between the L0 and L5
    coxa_brackets (Aug 16 2026: the bolt-down mounting ear + the 2
    chassis_top insert bosses are RETIRED -- the user velcros the
    holster down instead).  Single-part body:

      * SOCKET: a 5-walled open-top box that snugs the
        switch body in with SWITCH_BODY_CL mm clearance per axis.
        Toggle pokes out a SWITCH_TOGGLE_W x SWITCH_TOGGLE_H
        cutout in the +X end wall (= the chassis +X vertex); the
        2 XT60 pigtails exit out 2 x Phi SWITCH_PIGTAIL_OD = 6 mm
        holes in the -X end wall.  The SWITCH_HOLSTER_FLOOR = 4 mm
        bottom slab is solid and dead flat -- the velcro face.

    Local frame (mesh origin = MIDPOINT of the holster's X extent
    on chassis_top's TOP face):
        +X = toggle-exit direction (toggle pokes out +X face)
        +Y = tangential (along the chassis +X edge)
        +Z = UP (floor bottom at z = 0, socket cavity opens UP at z
              = SWITCH_HOLSTER_FLOOR)

    CHASSIS-frame placement: ``(SWITCH_HOLSTER_CENTRE_X,
    SWITCH_HOLSTER_CENTRE_Y, CHASSIS_TOP_TOP_Z)`` -- flat on the deck.
    """
    outer_l = SWITCH_HOLSTER_OUTER_L
    outer_w = SWITCH_HOLSTER_OUTER_W
    socket_l = SWITCH_SOCKET_OUTER_L

    # SOCKET solid block (the whole body; ear retired).
    socket_centre_x = outer_l / 2.0 - socket_l / 2.0    # = 0 now
    socket_outer_h = SWITCH_BODY_H + SWITCH_BODY_CL + SWITCH_HOLSTER_FLOOR
    block = _box(
        (socket_l, outer_w, socket_outer_h),
        center=(socket_centre_x, 0.0, socket_outer_h / 2.0),
    )

    # Switch body cavity (open top, inside socket block).
    cavity_l = SWITCH_BODY_L + 2.0 * SWITCH_BODY_CL
    cavity_w = SWITCH_BODY_W + 2.0 * SWITCH_BODY_CL
    cavity_h = SWITCH_BODY_H + SWITCH_BODY_CL
    cavity = _box(
        (cavity_l, cavity_w, cavity_h + 0.5),
        center=(socket_centre_x, 0.0,
                 SWITCH_HOLSTER_FLOOR + cavity_h / 2.0 + 0.25),
    )

    # Toggle cutout in +X end wall of the socket.
    toggle = _box(
        (SWITCH_HOLSTER_WALL + 0.4,
         SWITCH_TOGGLE_W,
         SWITCH_TOGGLE_H),
        center=(outer_l / 2.0 - (SWITCH_HOLSTER_WALL + 0.4) / 2.0 + 0.2,
                 0.0,
                 SWITCH_HOLSTER_FLOOR + SWITCH_BODY_H / 2.0),
    )

    # 2 XT60 pigtail exit channels through -X end wall of the
    # socket.  Phi 6 mm cylinders along X, centred vertically on the
    # body cavity (at z = floor + body_h/2 = 2.5 + 8.5 = 11 mm).
    pigtails = []
    socket_minus_x_face = socket_centre_x - socket_l / 2.0
    for sy in (-1, 1):
        ch = _cyl_along(SWITCH_PIGTAIL_OD / 2.0,
                         SWITCH_HOLSTER_WALL + 0.4,
                         axis="x")
        cx = socket_minus_x_face - 0.2
        cy = sy * SWITCH_PIGTAIL_DY
        cz = SWITCH_HOLSTER_FLOOR + SWITCH_BODY_H / 2.0
        ch.apply_translation([cx, cy, cz])
        pigtails.append(ch)

    # (Aug 16 2026: the 2 M3 ear clearance holes are retired with the
    # bolt-down mount -- velcro needs a flat, hole-free floor.)
    return _diff(block, cavity, toggle, *pigtails)


def make_belly_stilt() -> trimesh.Trimesh:
    """RETIRED — no printed belly stand exists any more.

    Kept only so old scripts that import the name fail loudly with a
    pointer rather than silently regenerating velcro stilts.  (Ground
    support now comes from the yaw_servo_retainer's built-in short feet --
    Aug 2026 v2, RETAINER_FOOT_H -- which lift the case-bottom wire exits
    off the floor when the robot belly-sits.)
    """
    raise RuntimeError(
        "belly_stilt is retired; ground support is the yaw_servo_retainer's "
        "built-in short feet (RETAINER_FOOT_H).  There is no separate "
        "printed belly stand.")


def make_imu_pad() -> trimesh.Trimesh:
    """RETIRED (Aug 2026 as-built stack) — MPU is glued on chassis_bottom
    behind phys. leg 1 (not on this pad).  Builder kept for legacy quotes.

    Printed vibration-isolated mounting pad for the MPU-6050 / GY-521
    breakout (May 2026 "essentials" pass: IMU promoted from optional to
    standard kit).

    The pad is a thin IMU_PAD_W x IMU_PAD_D x IMU_PAD_T plate with 4
    short Phi IMU_PAD_BOSS_OD bosses at the IMU's 15.0 x 11.0 mm hole
    pattern.  Each boss carries an M3 brass heat-set insert (McMaster
    94459A130) in a Phi INSERT_M3_PILOT_OD x INSERT_M3_PILOT_DEPTH
    pocket opening DOWNWARD from the boss top.  The IMU PCB sits on
    the 4 boss tops; 4 x M3 x 8 SHCS thread DOWN through the PCB
    into the inserts.

    There are NO fasteners between the pad and chassis_top.  The pad
    is glued to chassis_top with a 3 mm-thick strip of 3M VHB /
    generic double-sided mounting foam tape; the foam tape doubles
    as the mount AND the vibration damper that the MPU-6050's HF gyro
    noise floor requires in a servo-driven robot.  The pad's BOTTOM
    face is therefore deliberately FLAT and free of features so the
    foam tape adheres cleanly across its full IMU_PAD_W x IMU_PAD_D
    footprint.

    Local frame (mesh origin = pad floor centre on its BOTTOM face):
        +X = IMU long axis (PCB long edge, = the 8-pin header edge
              after the orientation in build_prototype_assembly)
        +Y = IMU short axis
        +Z = UP (pad floor at z in [0, IMU_PAD_T]; boss tops at
              z = IMU_PAD_T + IMU_PAD_BOSS_H)

    CHASSIS-frame placement (see IMU_PAD_CENTRE_X / Y / IMU_PAD_TAPE_T
    constants): ``(0, 0, chassis_top_top_z + IMU_PAD_TAPE_T)`` -- the
    chassis-frame centre of mass, so rotation rates from the gyro
    are not contaminated by linear-acceleration cross-coupling from
    the body swing.
    """
    # Pad floor.
    floor = _box((IMU_PAD_W, IMU_PAD_D, IMU_PAD_T),
                 center=(0.0, 0.0, IMU_PAD_T / 2.0))

    # 4 bosses + insert pockets at the IMU hole pattern.  Each boss is
    # a Phi IMU_PAD_BOSS_OD cylinder rising IMU_PAD_BOSS_H above the
    # pad's top face (= z = IMU_PAD_T).  The 6 mm insert pocket digs
    # DOWN from the boss top into the boss + pad floor, leaving 0.6
    # mm of plastic above the pad bottom face so the foam-tape side
    # of the pad stays FLAT.
    bosses = []
    pockets = []
    clearance_holes = []
    pocket_overdrill_h = INSERT_M3_PILOT_DEPTH + 0.4   # 0.4 mm slop
                                                       # at the OPEN
                                                       # end of the
                                                       # pocket so the
                                                       # CSG diff
                                                       # clears the
                                                       # boss top rim
    boss_top_z = IMU_PAD_T + IMU_PAD_BOSS_H            # = 7 mm
    pocket_centre_z = boss_top_z - INSERT_M3_PILOT_DEPTH / 2.0 + 0.2
    # ^ pocket centre = (boss_top - depth/2) + small overdrill shift
    #   so the pocket extends from
    #   z = boss_top_z + 0.2 (just above boss top, for clean diff)
    #   down to
    #   z = boss_top_z + 0.2 - pocket_overdrill_h
    #     = boss_top_z + 0.2 - (INSERT_M3_PILOT_DEPTH + 0.4)
    #     = boss_top_z - INSERT_M3_PILOT_DEPTH - 0.2
    # With boss_top_z = 7 and INSERT_M3_PILOT_DEPTH = 6 the pocket
    # bottom is at z = +0.8 mm -- 0.8 mm of pad floor remains below
    # the pocket so the bottom face stays flat for foam-tape adhesion.
    for (bx, by) in IMU_PAD_HOLE_OFFSETS:
        # Boss extends from z = 0 (= pad floor BOTTOM face, so it
        # stays UNIONED with the floor without dropping below it --
        # keeps the pad's BOTTOM face flat for foam-tape adhesion)
        # up to z = boss_top_z + 0.2 (0.2 mm above boss top for
        # clean diff with the insert pocket).
        boss_h_total = boss_top_z + 0.2
        boss = _cyl(IMU_PAD_BOSS_OD / 2.0, boss_h_total)
        boss.apply_translation([bx, by, boss_h_total / 2.0])
        bosses.append(boss)

        pocket = _cyl(INSERT_M3_PILOT_OD / 2.0, pocket_overdrill_h)
        pocket.apply_translation([bx, by, pocket_centre_z])
        pockets.append(pocket)

    # NOTE: the M3 clearance holes for the IMU's bolts are in the
    # IMU PCB itself, not in the pad.  The pad's bosses are SOLID
    # (apart from the insert pocket) -- the bolt threads into the
    # brass insert, not through the boss.  So no additional Phi 3.0
    # clearance cuts are made through the boss material.
    del clearance_holes  # placeholder; left intentionally unused

    body = _union(floor, *bosses)
    return _diff(body, *pockets)


def make_mpu6050_visual() -> trimesh.Trimesh:
    """Visual mesh for the MPU-6050 / GY-521 IMU breakout PCB.

    NOT FOR PRINTING -- this is the commodity blue/red breakout board
    that ships from Amazon; the visual is a simple IMU_PCB_W x
    IMU_PCB_D x IMU_PCB_T flat slab so the build inspector / render
    can show the IMU on top of the printed ``imu_pad``.  Same role
    as ``make_servo_body`` / ``make_servo_horn`` (visual-only meshes
    written to stl_prototype/ alongside the real printables).

    Mesh frame: origin at the PCB centre, +X along the long axis
    (along the 8-pin header edge), +Y along the short axis, +Z UP.
    """
    return _box((IMU_PCB_W, IMU_PCB_D, IMU_PCB_T),
                center=(0.0, 0.0, 0.0))


# ---------------------------------------------------------------------------
# Non-printed electronics visuals (May 2026 follow-up: BuildViz pass)
# ---------------------------------------------------------------------------
#
# These ``make_*_visual()`` functions follow the same convention as
# ``make_servo_body`` / ``make_servo_horn`` / ``make_mpu6050_visual``:
# visual-only meshes written to ``stl_reference/`` (via stl_dir_for),
# NOT added to the printable-parts registry
# (``scripts/print_orientation.PART_REGISTRY``) -- there
# is nothing to print here, the user buys the actual hardware from
# Amazon / McMaster.
#
# Each function returns a mesh whose LOCAL ORIGIN is chosen so the
# inspector's ``_build_assembly_instances`` can translate by a single
# ``_trans(world_x, world_y, world_z)`` without having to apply
# additional local offsets:
#
#   * Bare PCB boards (Mega, Pi, PCA9685) -- origin = PCB midplane,
#     +X = long axis (chassis +X after placement), +Y = short axis,
#     +Z = up.  Connector lumps stick up above the PCB top face.
#   * Free-standing bodies (BEC, anti-spark switch, LiPo) -- origin
#     = body geometric centre.
#
# Triangle counts are kept small: each connector / pigtail is a
# single primitive (box or short cylinder).  These meshes never
# participate in the design verifier -- they are not in
# ``_MESH_BUILDERS`` (see _verify_prototype.py) and the inspector
# marks them as decoration.

# Standard PCB thickness used by the Mega, Pi, and PCA9685 commodity
# breakouts.  IMU_PCB_T (above) already documents the same value for
# the MPU-6050; redefining here so the visual makers don't reach into
# an "imu-only" constant for their generic PCB slab.
COMMODITY_PCB_T = 1.6   # mm


def make_arduino_mega_visual() -> trimesh.Trimesh:
    """Visual mesh for the Arduino Mega 2560 R3 (NOT FOR PRINTING).

    Models the bare PCB (MEGA_PCB_D x MEGA_PCB_W x COMMODITY_PCB_T)
    plus the two -X end connectors that the user wires into: a USB-B
    socket (~ 14 x 13 x 16 mm) and a DC barrel jack (~ Phi 9 x 14 mm,
    axis along the same edge as the USB-B).  The ATmega chip, headers,
    and pin sockets are omitted -- they read as part of the PCB slab
    once colored.

    Mesh frame: origin = PCB midplane.  +X = long axis (chassis +X
    after placement, matching MEGA_PCB_D being the chassis-X extent
    in ``_body_battery_parts``).  +Y = short axis.  +Z = up.
    """
    pcb = _box((MEGA_PCB_D, MEGA_PCB_W, COMMODITY_PCB_T),
               center=(0.0, 0.0, 0.0))

    pcb_top_z = COMMODITY_PCB_T / 2.0
    usb_b_h = 16.0
    usb_b_centre_x = -MEGA_PCB_D / 2.0 + 14.0 / 2.0 + 0.5
    usb_b = _box((14.0, 13.0, usb_b_h),
                 center=(usb_b_centre_x, -10.0,
                          pcb_top_z + usb_b_h / 2.0))

    barrel_d = 9.0
    barrel = _cyl_along(barrel_d / 2.0, 14.0, axis="x")
    barrel.apply_translation([-MEGA_PCB_D / 2.0 - 0.5,
                               +12.0,
                               pcb_top_z + barrel_d / 2.0])

    return _union(pcb, usb_b, barrel)


def make_raspberry_pi_visual() -> trimesh.Trimesh:
    """Visual mesh for the Raspberry Pi 4 / Pi 5 SBC (NOT FOR PRINTING).

    Models the bare PCB (PI_PCB_W x PI_PCB_D x COMMODITY_PCB_T) plus
    the two perpendicular connector edges the user wires into.  Real
    Pi 4 / Pi 5 layout per the published mechanical drawings:

      * -Y LONG edge (chassis -Y after placement; cables exit -Y
        past chassis_top's -Y apothem):
          - USB-A 3.0 PAIR (blue, 2 stacked) ~ 16 x 16 x 17 mm
          - USB-A 2.0 PAIR (black, 2 stacked) ~ 16 x 16 x 17 mm
          - Ethernet RJ45                     ~ 16 x 16 x 14 mm
      * -X SHORT edge (chassis -X after placement; cables exit -X
        in a separate corridor from the +X PCA9685 servo header
        bank):
          - USB-C power           ~ 8 x 12 x 8 mm
          - 2 x micro-HDMI        ~ 8 x 8 x 12 mm each
          - (3.5 mm audio jack omitted -- not used in this robot)
      * +Y LONG edge: 40-pin GPIO header (no jumper cables modelled).
      * +X SHORT edge: micro-SD slot (user reaches in to swap; no
        plug, no keepout needed -- the card sits low on the PCB).

    Mesh frame: origin = PCB midplane.  +X = long axis (PI_PCB_W,
    chassis +X after placement).  +Y = short axis.  +Z = up.

    Pre-May-2026 the visual mesh had USB-C/HDMI on the +Y LONG edge
    and USB-A/Ethernet on the +X SHORT edge -- physically impossible
    for the real Pi (USB-A x2 + Ethernet would overflow the 56 mm
    short edge).  The May 2026 "Pi cantilever" pass corrected the
    bank-to-edge assignment AND fixed the Ethernet RJ45 jack
    out-of-plane height from 25 mm to 14 mm (the 25 mm figure
    confused the in-plane plug depth with the actual ~ 13.5 mm-tall
    metal jack body + ~ 1-2 mm metal EMI shield on top).
    """
    pcb = _box((PI_PCB_W, PI_PCB_D, COMMODITY_PCB_T),
               center=(0.0, 0.0, 0.0))

    pcb_top_z = COMMODITY_PCB_T / 2.0
    half_x = PI_PCB_W / 2.0
    half_y = PI_PCB_D / 2.0

    # USB-C power on -X SHORT edge.  Body sticks 8 mm in -X off the
    # PCB edge; centre at PCB-local y = +half_y - 11 mm (11 mm from
    # the +Y corner along the short axis, matching real Pi 4 USB-C
    # position).
    usb_c_h = 8.0
    usb_c = _box((8.0, 12.0, usb_c_h),
                 center=(-half_x - 4.0,
                          half_y - 11.0,
                          pcb_top_z + usb_c_h / 2.0))

    # Micro-HDMI 0 and 1 on -X SHORT edge.  Same body-depth-in-X
    # convention as USB-C.
    hdmi_h = 12.0
    hdmi_a = _box((8.0, 8.0, hdmi_h),
                   center=(-half_x - 4.0,
                            half_y - 15.0,
                            pcb_top_z + hdmi_h / 2.0))
    hdmi_b = _box((8.0, 8.0, hdmi_h),
                   center=(-half_x - 4.0,
                            0.0,
                            pcb_top_z + hdmi_h / 2.0))

    # USB-A pairs (16 x 16 x 17, 2 receptacles stacked vertically)
    # on the -Y LONG edge.  Body sticks 16 mm in -Y off the PCB
    # edge.  Positions matching real Pi 4 mech drawing: USB 3.0
    # PAIR (blue) at PCB-local x = -3.5 mm, USB 2.0 PAIR (black) at
    # PCB-local x = +13.5 mm.
    usb_a_h = 17.0
    usb_a_3 = _box((16.0, 16.0, usb_a_h),
                    center=(-3.5,
                             -half_y - 8.0,
                             pcb_top_z + usb_a_h / 2.0))
    usb_a_2 = _box((16.0, 16.0, usb_a_h),
                    center=(+13.5,
                             -half_y - 8.0,
                             pcb_top_z + usb_a_h / 2.0))

    # Ethernet RJ45 on -Y LONG edge at PCB-local x = +30 mm.  Body
    # depth 16 mm in -Y; height in Z = 14 mm (~ 13.5 mm jack body
    # + ~ 1-2 mm metal EMI shield, rounded up to 14).
    eth_h = 14.0
    eth = _box((16.0, 16.0, eth_h),
                center=(+30.0,
                         -half_y - 8.0,
                         pcb_top_z + eth_h / 2.0))

    return _union(pcb, usb_c, hdmi_a, hdmi_b, usb_a_3, usb_a_2, eth)


def make_pca9685_visual() -> trimesh.Trimesh:
    """Visual mesh for an Adafruit PCA9685 16-channel PWM driver
    (NOT FOR PRINTING).

    Models the bare PCB (PCA_PCB_D x PCA_PCB_W x COMMODITY_PCB_T --
    short axis along chassis X, long axis along chassis Y to match
    ``_body_battery_parts``'s placement rotation) plus the 3-pin
    servo header row along the +X long edge (modelled as one
    8 x 60 x 8 mm bar so the user sees where the servo jumpers plug
    in).  The I2C terminal block and STEMMA QT connector are omitted.

    Mesh frame: origin = PCB midplane.  +X = chassis short axis.
    +Y = chassis long axis.  +Z = up.

    Same STL is used for both PCA9685(0x40) and PCA9685(0x41) -- the
    address jumper is not visually distinguishable.
    """
    pcb = _box((PCA_PCB_D, PCA_PCB_W, COMMODITY_PCB_T),
               center=(0.0, 0.0, 0.0))

    pcb_top_z = COMMODITY_PCB_T / 2.0
    header_h = 8.0
    header = _box((8.0, 60.0, header_h),
                   center=(PCA_PCB_D / 2.0 + 4.0,
                            0.0,
                            pcb_top_z + header_h / 2.0))

    return _union(pcb, header)


def make_bus_adapter_visual() -> trimesh.Trimesh:
    """Visual mesh for the STS3215 USB-to-TTL serial-bus adapter
    (Waveshare "Bus Servo Adapter (A)"-class, NOT FOR PRINTING).

    Models the bare PCB (BUS_ADAPTER_PCB_W x BUS_ADAPTER_PCB_D x
    COMMODITY_PCB_T) plus a 3-pin servo-bus header bar on the +X edge
    (where the daisy-chain harness plugs in) and a USB stub on the -X
    edge (where the lead to the Pi plugs in).

    Mesh frame: origin = PCB midplane.  +X = PCB long axis.
    +Y = PCB short axis.  +Z = up.  Replaces the retired PCA9685 +
    Arduino Mega visuals (STS3215 refit)."""
    pcb = _box((BUS_ADAPTER_PCB_W, BUS_ADAPTER_PCB_D, COMMODITY_PCB_T),
               center=(0.0, 0.0, 0.0))
    pcb_top_z = COMMODITY_PCB_T / 2.0

    header_h = 8.0
    header = _box((6.0, 14.0, header_h),
                  center=(BUS_ADAPTER_PCB_W / 2.0 - 3.0,
                          0.0, pcb_top_z + header_h / 2.0))

    usb = _box((8.0, 8.0, 4.0),
               center=(-BUS_ADAPTER_PCB_W / 2.0 - 2.0, 0.0, pcb_top_z + 2.0))

    return _union(pcb, header, usb)


def make_bec_visual() -> trimesh.Trimesh:
    """Visual mesh for one 5V switching BEC body (NOT FOR PRINTING).

    Shrink-wrapped BEC_BODY_L x BEC_BODY_W x BEC_BODY_H body plus 2
    short Phi 3 mm pigtail stubs exiting the +/-X ends (the input
    XT60 lead at one end, the 3-pin servo-header output at the other;
    both modelled as plain cylinders, no connectors).

    Mesh frame: origin = body geometric centre.  +X = pigtail axis
    (matches the bec_cradle's local +X = pigtail axis).  +Y = wider
    axis.  +Z = up.

    Same STL is used for both ``bec_a`` and ``bec_b`` -- the cradle
    holds 2 BECs side-by-side along the cradle's +Y axis.
    """
    body = _box((BEC_BODY_L, BEC_BODY_W, BEC_BODY_H),
                center=(0.0, 0.0, 0.0))

    pigtail_r = 1.5
    pigtail_len = 8.0
    p_plus = _cyl_along(pigtail_r, pigtail_len, axis="x")
    p_plus.apply_translation([+BEC_BODY_L / 2.0, 0.0, 0.0])
    p_minus = _cyl_along(pigtail_r, pigtail_len, axis="x")
    p_minus.apply_translation([-BEC_BODY_L / 2.0 - pigtail_len, 0.0, 0.0])

    return _union(body, p_plus, p_minus)


def make_antispark_switch_body_visual() -> trimesh.Trimesh:
    """Visual mesh for the anti-spark XT60 switch BODY (NOT FOR PRINTING).

    SWITCH_BODY_L x _W x _H switch body (~ 32 x 17 x 17 mm) plus two
    short Phi 4 mm XT60 pigtail stubs exiting the +/-Y end faces.
    The toggle lever is a SEPARATE mesh
    (``make_antispark_switch_toggle_visual``) so the inspector can
    paint it a distinct safety-orange against the body's anodized
    grey.

    Mesh frame: origin = body geometric centre, with +X = toggle
    axis (chassis +X after placement, matching the switch_holster's
    local +X = toggle face), +Y = pigtail axis, +Z = up.
    """
    body = _box((SWITCH_BODY_L, SWITCH_BODY_W, SWITCH_BODY_H),
                center=(0.0, 0.0, 0.0))

    pigtail_r = 2.0
    pigtail_len = 10.0
    p_plus_y = _cyl_along(pigtail_r, pigtail_len, axis="y")
    p_plus_y.apply_translation([0.0, +SWITCH_BODY_W / 2.0, 0.0])
    p_minus_y = _cyl_along(pigtail_r, pigtail_len, axis="y")
    p_minus_y.apply_translation([0.0,
                                  -SWITCH_BODY_W / 2.0 - pigtail_len,
                                  0.0])

    return _union(body, p_plus_y, p_minus_y)


def make_antispark_switch_toggle_visual() -> trimesh.Trimesh:
    """Visual mesh for the anti-spark switch TOGGLE lever (NOT FOR
    PRINTING).

    Phi 4 x 8 mm cylinder protruding from the switch body's +X face.
    Mesh frame: origin at the TOGGLE's centre (the cylinder spans -4
    to +4 in its long axis).  +X is the protrusion direction.

    Drawn as a separate mesh so the inspector can color it safety-
    orange against the body's anodized grey.
    """
    toggle = _cyl_along(2.0, 8.0, axis="x")
    toggle.apply_translation([-4.0, 0.0, 0.0])
    return toggle


def make_lipo_battery_body_visual() -> trimesh.Trimesh:
    """Visual mesh for ONE 3S shorty LiPo pack BODY (NOT FOR PRINTING).

    Shrink-wrap slab (BATTERY_W x BATTERY_D x BATTERY_H = 75 x 34 x
    26.5 mm Zeee shorty, Aug 2026 -- TWO of these mount under the
    belly, see ``battery_pack_transforms``).  The XT60 connector +
    balance plug are a SEPARATE mesh (``make_lipo_xt60_visual``) so
    the inspector can paint the XT60 housing safety-yellow against
    the body's "lipo red".

    Mesh frame: origin = body geometric centre.  +X = long axis
    (the pack block's yawed axis after placement).  +Y = short axis.
    +Z = up.
    """
    return _box((BATTERY_W, BATTERY_D, BATTERY_H),
                center=(0.0, 0.0, 0.0))


def make_lipo_xt60_visual() -> trimesh.Trimesh:
    """Visual mesh for the LiPo's XT60 connector + balance plug
    (NOT FOR PRINTING).

    XT60 housing (~ 14 x 17 x 10 mm) plus the smaller balance-plug
    ribbon header (~ 12 x 8 x 4 mm) sitting adjacent.  Both rendered
    safety-yellow.

    Mesh frame: origin at the XT60 housing centre.  +X is the
    direction the connector points (out of the battery body's +X
    short face after placement).
    """
    xt60 = _box((14.0, 17.0, 10.0),
                 center=(0.0, 0.0, 0.0))
    balance = _box((12.0, 8.0, 4.0),
                    center=(0.0, 17.0 / 2.0 + 8.0 / 2.0 + 0.5, -3.0))
    return _union(xt60, balance)


# ---------------------------------------------------------------------------
# Leg parts
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Joint convention (mirrors hexapod_walker.py)
# ---------------------------------------------------------------------------
#
#   - Yaw axis is global +Z (vertical).
#   - Hip and knee pitch axes are along the leg's local +Y (tangential
#     to the chassis, perpendicular to the leg's outboard direction).
#   - In every leg, all three servos hang on the leg's -Y_local side;
#     the spars run along +X_local in the y_local = 0 plane.
#
# Parts are designed for FDM printing in PLA / PETG: minimum wall 1.6 mm,
# minimum hole 3.2 mm, no overhangs steeper than 45 deg without supports.
# ---------------------------------------------------------------------------


def make_foot_boot() -> trimesh.Trimesh:
    """TPU 95A boot pressed over the tibia CF-tube end (Aug 2026 --
    replaces tibia_foot_fitting + foot_pad + the M3x16/nyloc hinge).

    One revolved solid: a standard spherical-ended boot over the last
    FOOT_BOOT_SOCKET_DEPTH mm of the Ø8 tube (Ø8.1 bore -- same slip
    fit as the tibia yoke's tube socket), closed by a FOOT_BOOT_TIP_L
    solid section ending in a hemispherical dome.  The dome IS the
    MuJoCo contact sphere, apex at the measured kinematic tip.  The
    bore's blind end is a 45-deg internal cone so the boot prints
    MOUTH-DOWN with no bridges.

    Local frame matches the old fitting: origin at the tube end on the
    tube axis, tube enters from -X, dome apex at +FOOT_BOOT_TIP_L.
    """
    tip_l = FOOT_BOOT_TIP_L
    total_l = FOOT_BOOT_SOCKET_DEPTH + tip_l
    r_out = FOOT_BOOT_TIP_R
    r_bore = FOOT_BOOT_BORE_D / 2.0
    lead = FOOT_BOOT_MOUTH_LEAD
    # Revolved profile (r, z): z = 0 at the dome apex (ground contact),
    # +z toward the open mouth.  Starts and ends on the axis ->
    # watertight solid.  The dome arc runs apex -> equator at (r_out,
    # r_out); a quarter circle sampled finely enough that the chord
    # error is far below FDM resolution.
    theta = np.linspace(0.0, np.pi / 2.0, 25)
    dome = np.column_stack([r_out * np.sin(theta),
                            r_out * (1.0 - np.cos(theta))])
    profile = np.vstack([
        dome,
        [(r_out,          total_l),
         (r_bore + lead,  total_l),
         (r_bore,         total_l - lead),
         (r_bore,         tip_l),
         # 45-deg internal blind-end cone: the tube's end ring bottoms
         # on the cone/bore corner at the same insertion depth as the
         # old flat blind end, and mouth-down printing self-supports.
         (0.0,            tip_l - r_bore)],
    ])
    boot = trimesh.creation.revolve(profile, sections=64)
    # Dome apex (z=0) -> local +X tip; mouth -> -X over the tube.
    boot.apply_transform(rotation_matrix(-np.pi / 2.0, [0, 1, 0]))
    boot.apply_translation([tip_l, 0.0, 0.0])
    return boot


# ===========================================================================
# Bearing-SANDWICH production leg parts (Jun 2026 refit)
# ===========================================================================
# These builders define the dual-supported production legs used by main()
# and the sandwich assembly. The superseded cantilevered builders remain in
# Git history.
#
#   coxa_link        : yaw-driven pad + arm + HIP fixed side (servo bracket
#                      + 688 bearing housing).
#   femur_link       : the WHOLE femur as ONE printed part (Jul 2026 merge
#                      #2): HIP moving yoke + solid Phi 14 fused spar + KNEE
#                      fixed side (servo bracket + bearing housing).
#   tibia_knee_yoke  : KNEE moving yoke with the tibia CF-tube socket toward
#                      the foot.
#   foot_boot        : TPU boot pressed over the tube end (Aug 2026 --
#                      replaces tibia_foot_fitting + foot_pad + hinge pin).
#
# Femur = femur_link, one printed body (no pins, no sockets).
# Tibia = tibia_knee_yoke + dia-8 CF tube + foot_boot.


def _joint_place(mount, x_dir, z_dir) -> np.ndarray:
    """4x4 mapping a joint-local mesh into a target frame so local +X ->
    ``x_dir``, local +Z -> ``z_dir`` (orthonormalized), and the disc-horn-
    top point (SERVO_OUTPUT_X, 0, JOINT_HORN_TOP_Z) lands on ``mount``.

    Anchoring every sandwich part's disc-horn-top to the joint axis point
    guarantees the fixed side and the moving yoke mate at the same horn
    plane no matter how each is oriented."""
    z = np.asarray(z_dir, float); z = z / np.linalg.norm(z)
    x = np.asarray(x_dir, float); x = x - z * np.dot(x, z); x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    M = np.eye(4)
    M[:3, 0] = x; M[:3, 1] = y; M[:3, 2] = z
    anchor = M[:3, :3] @ np.array([SERVO_OUTPUT_X, 0.0, JOINT_HORN_TOP_Z])
    M[:3, 3] = np.asarray(mount, float) - anchor
    return M


def _frame(origin, x_dir, z_dir) -> np.ndarray:
    """4x4 with local +X -> x_dir, +Z -> z_dir (orthonormalized) and
    local origin -> ``origin``."""
    z = np.asarray(z_dir, float); z = z / np.linalg.norm(z)
    x = np.asarray(x_dir, float); x = x - z * np.dot(x, z); x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    M = np.eye(4)
    M[:3, 0] = x; M[:3, 1] = y; M[:3, 2] = z
    M[:3, 3] = np.asarray(origin, float)
    return M


def _tube_between(p0, p1, radius: float) -> trimesh.Trimesh:
    """A carbon-tube visual: a cylinder of ``radius`` spanning world
    points ``p0`` -> ``p1``."""
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
    axis = p1 - p0
    length = float(np.linalg.norm(axis))
    tube = _cyl(radius, length)
    z = axis / length
    # Align +Z to the tube axis.
    zc = np.array([0.0, 0.0, 1.0])
    v = np.cross(zc, z); s = float(np.linalg.norm(v)); c = float(np.dot(zc, z))
    R = np.eye(4)
    if s > 1e-9:
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R[:3, :3] = np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))
    elif c < 0:
        R[:3, :3] = np.diag([1.0, -1.0, -1.0])
    tube.apply_transform(R)
    tube.apply_translation((p0 + p1) / 2.0)
    return tube


# --- Yaw turntable hub geometry (Part A) --------------------------------
# z = 0 is the disc-horn top (= CHASSIS_YAW_OUTPUT_Z, the coxa pad plane).
YAW_HUB_PAD_T   = 6.0   # mm -- pad thickness (lets the 4 horn-bolt heads
                        #       counterbore FLUSH so Part B sits flat on top)
# COXA_JOIN_FOOT_T: RETIRED Aug 17 2026.  The two-part era stacked an 8 mm
# foot plate ABOVE the platform (well floor +31) because it was the sole
# bolted Part A<->B join.  The one-piece coxa_link instead SINKS the foot
# through the full platform band (z +18..+26) so the hip-servo well floor
# sits just COXA_WELL_FLOOR_LIFT = 2 mm over the platform top -- 5 mm
# shorter part, screw heads ~5.3 mm below the shaft mouths.  See
# COXA_COAXIAL_FOOT_RAISE.
LIP_RELIEF_CHAMFER = 5.0  # mm -- 45-deg chamfer leg cut into the foot's FRONT
                          #       (-Y) bottom-outer edge to kill the proud
                          #       print lip on the high-coxa-Z output face


def make_yaw_bearing(which: str = "lower") -> trimesh.Trimesh:
    """Visual 6805-2RS deep-groove bearing (NOT printed; 25x37x7, Aug 2026
    thick-section swap -- was 6706-2RS 30x37x4).  ``which`` selects
    the LOWER (around the disc horn) or UPPER (on the hub boss) bearing of
    the SPACED PAIR.  Coxa-local frame, coaxial with the yaw axis."""
    bot = (YAW_BEARING_UPPER_BOT_Z if which == "upper"
           else YAW_BEARING_LOWER_BOT_Z)
    inner = _diff(_cyl(YAW_BEARING_INNER_OD / 2.0, YAW_BEARING_W),
                  _cyl(YAW_BEARING_ID / 2.0, YAW_BEARING_W * 2))
    outer = _diff(_cyl(YAW_BEARING_OD / 2.0, YAW_BEARING_W),
                  _cyl(YAW_BEARING_OUTER_ID / 2.0, YAW_BEARING_W * 2))
    brg = _union(inner, outer)
    brg.apply_translation([0.0, 0.0, bot + YAW_BEARING_W / 2.0])
    return brg


def make_yaw_bearing_lower() -> trimesh.Trimesh:
    return make_yaw_bearing("lower")


def make_yaw_bearing_upper() -> trimesh.Trimesh:
    return make_yaw_bearing("upper")


def _coxa_join_bolt_centres():
    """Part A <-> Part B vertical M3 join-bolt centres (coxa-local XY)."""
    r = COXA_JOIN_BOLT_PCD / 2.0
    return [(r * np.cos(t), r * np.sin(t)) for t in COXA_JOIN_BOLT_ANGLES_RAD]


# Coxa-local Z planes for the spaced-pair hub (shared by Part A + Part B so
# the two printed parts mate without interpenetrating).
# Jun 2026 flush-horn refit: the hub seats FLAT on the horn top (z = 0) -- the
# old wrap-around skirt that dipped to z = -4.5 is gone -- and the solid boss
# rides BOTH inner races above it.
# Jun 2026 flush-horn DEPTH fix (user: "the motor tip doesn't protrude; the
# bearing cap can't get flush with the horn").  ROOT CAUSE: the yaw servo's
# front face seats against the mount-plate UNDERSIDE and the output does NOT
# protrude, so the Phi20 disc horn sits FLUSH on the front face -- i.e. RECESSED
# inside the WELL_PLATE_T (4 mm) mount-plate bore, with its top
# (WELL_PLATE_T - DISC_HORN_H) = 2 mm BELOW the plate top, NOT sitting ON the
# plate top.  The frozen output plane (coxa z = 0) is WELL_PLATE_T + HORN_STACK_H
# above the front face, so the REAL flush-horn top is YAW_HORN_REACH_DOWN below
# it -- 4 mm deeper than the generic HORN_REACH_DOWN = 3 mm that the PLATELESS
# hip/knee joints use (their horn rides the bare output boss, no 4 mm plate).
# The old hub reached only -HORN_REACH_DOWN = -3, leaving a 4 mm axial GAP above
# the real horn: the 4 drive bolts never bit the disc and the whole turntable
# stack (incl. the bearing cap) floated, never seating flush.  The hub now
# reaches the real horn through the plate bore via a necked drive nub.
#
# Aug 2026 bench (hub seated on both bearing inner races, lip on upper race):
# formula-only reach (= 7 mm) still left ~2 mm of air between the Phi 20
# drive-nub bottom and the real flush horn.  Tightening the 4 horn screws
# then pulled the hub DOWN off the bearings (heavy/smooth drag + angular
# slop when left loose).  YAW_HORN_BENCH_EXTRA_REACH closes that measured
# gap so the nub meets the horn WHILE the retain lip stays on the race.
YAW_HORN_BENCH_EXTRA_REACH = 2.0  # mm -- measured hub-on-bearings → horn gap
YAW_HORN_REACH_DOWN = (WELL_PLATE_T + HORN_STACK_H - DISC_HORN_H
                       + YAW_HORN_BENCH_EXTRA_REACH)   # 9 mm
YAW_HUB_BOSS_BOT_Z = -YAW_HORN_REACH_DOWN             # -9: real flush-horn top
# The wide Phi YAW_HUB_BOSS_OD boss (rides BOTH inner races) necks DOWN to a
# Phi YAW_HUB_DRIVE_NUB_OD stub that passes THROUGH the Phi HORN_CLEAR_OPENING_OD
# plate bore to seat on the recessed horn (this IS the "~3 mm bump <= the 20 mm
# chassis hole" the user asked for, sized to the disc footprint).  The nub
# carries the 4 x M3 drive bolts (PCD 14).
#
# Jun 2026 (user: "the bottom ring of the coxa yaw hub needs to be 4 mm not
# 2 mm"): the disc-horn-mating "bottom ring" is this necked nub.  Keep the
# ring 4 mm thick and let BOSS_BOT / WIDE_BOT track YAW_HORN_REACH_DOWN --
# Aug 2026's +2 mm reach drops the neck to the plate-top plane (z = -5)
# where the Phi 44 tower relief still clears the wide boss; only the narrow
# Phi 20 nub enters the plate bore.  The wide boss still spans the full
# bearing band (z >= +0.5) untouched.
YAW_HUB_BOTTOM_RING_T = 4.0                            # mm -- disc-horn-mating ring (Z thickness)
YAW_HUB_BOSS_WIDE_BOT_Z = YAW_HUB_BOSS_BOT_Z + YAW_HUB_BOTTOM_RING_T  # -5: neck plane (ring top)
YAW_HUB_DRIVE_NUB_OD    = DISC_HORN_OD                 # Phi 20: == disc footprint
assert YAW_HUB_DRIVE_NUB_OD <= HORN_CLEAR_OPENING_OD - 2.0, (
    "yaw drive nub must clear the chassis mount-plate bore by >=1 mm radial")
# CAD had been calling for M3 x 8 / M3 x 10 with the head near the horn;
# the Aug 2026 bench build used M3 x 20 from a mid-boss counterbore (head
# underside ~z=+9).  Aug 16 2026 (user: "use my m3x30 so that i can fit my
# normal screwdriver in there more easily"): one size longer -- M3 x 30
# lifts the shared head-seat plane to the top of the hub.
#
# Aug 17 2026 seat-depth fix (user: "the m3x30 screws only protrude about
# 1 mm which i dont think is enough to really attach to the horn safely"):
# the NOMINAL tip landed at the disc bottom (z = -11, full 2 mm bite), but
# on the bench the tips protruded only ~1 mm from the drive nub -- the
# printed counterbore floor + the M3x30's under-head length tolerance eat
# about a millimetre.  Compensate in the seat plane:
#   * BENCH_SEAT_SHORTFALL (1.0) sinks every seat by the measured loss;
#   * TIP_POKE (0.25) sinks the 4 CORNER seats a hair further so their
#     tips just break the disc's far face for full thread engagement.
#     They cannot go deeper: the Phi 20 disc sits FLUSH on the servo's
#     front face, so under the PCD-14 holes there is servo case -- a
#     longer poke jams the disc off its seat.
#   * The CENTRE station gets its own seat 1 mm deeper still
#     (YAW_HUB_HORN_CENTRE_SEAT_Z): the centre screw threads into the
#     output-spline's tapped bore (>= 8 mm deep -- the stock spline screw
#     alone reaches ~-14), so "the middle one might go deeper" (user) is
#     free extra engagement with no case-jam risk.
YAW_HUB_HORN_BOLT_LEN = 30.0  # mm -- M3x30 (user's stock, Aug 16 2026)
YAW_HORN_BOLT_BENCH_SEAT_SHORTFALL = 1.0   # mm -- measured print/screw loss
YAW_HORN_BOLT_TIP_POKE = 0.25              # mm -- corner tips past disc bottom
YAW_HUB_HORN_HEAD_SEAT_Z = (YAW_HUB_BOSS_BOT_Z - DISC_HORN_H
                            + YAW_HUB_HORN_BOLT_LEN
                            - YAW_HORN_BOLT_BENCH_SEAT_SHORTFALL
                            - YAW_HORN_BOLT_TIP_POKE)  # +17.75 corner seat
YAW_HUB_HORN_CENTRE_SEAT_Z = YAW_HUB_HORN_HEAD_SEAT_Z - 1.0  # +16.75 centre seat
YAW_HUB_HORN_HEAD_CB_OD = INSERT_M3_BOLT_HEAD_OD + 0.4  # mm -- shared head pocket
YAW_HUB_BOSS_TOP_Z = YAW_TOWER_TOP_Z + YAW_HUB_CAP_AXIAL_CL  # +18.0 (1.5 mm over tower/cap rim; Aug 17 2026 scrape fix)
YAW_HUB_PLATFORM_Z1 = YAW_HUB_BOSS_TOP_Z + YAW_HUB_PAD_T  # +19.0 (Part B seats here)
assert abs(YAW_HUB_PLATFORM_Z1 - _YAW_HUB_PLATFORM_TOP_EARLY) < 1e-9, (
    "YAW_HUB_PLATFORM_Z1 drifted from the early estimate used in "
    "COXA_COAXIAL_FOOT_RAISE -- keep the two formulas in sync")


def make_coxa_yaw_hub(*, one_piece: bool = False) -> trimesh.Trimesh:
    """Part A -- the yaw 'turntable' hub that sits RIGHT ON TOP of the yaw
    servo.  Bolts DOWN onto the yaw disc horn (drive) and rides BOTH inner
    races of the TOUCHING 6805 pair (support), so the cantilevered hip load
    reacts as an axial couple through the bearings into chassis_bottom, not
    through the servo spline.

    Aug 2026 one-piece merge (user: "the hip bracket and coxa yaw hub are
    linked together and theres no reason for them not to be one piece"):
    with ``one_piece=True`` this is a SUB-SOLID of the merged printed
    ``coxa_link`` (see ``make_coxa_link_part``) -- the 4 Part-B join pilots
    are skipped because there is no separate Part B to bolt on.  With the
    default ``one_piece=False`` the standalone Part-A geometry (with join
    pilots) is kept for the verifier's targeted bearing/insertion checks.

    Coxa-local: origin = yaw axis at the disc-horn top; +Z up, +X outboard,
    +Y = hip-pitch axis.  Vertical stack:
        z[-4.5, 0]  lower boss (annular, clears Phi 20 horn) -> LOWER race
        z[0, +7.5]  upper boss (solid) -> UPPER race + clamp body
        z[+7.5, +13.5] top platform (turntable disc, Part B bolts here)
    A single M3-clamp path runs from the platform top down into the disc
    horn (4 bolts) preloading the whole stack.  The Part B mating bolts
    (4 x M3 on COXA_JOIN_BOLT_PCD) thread into the platform."""
    rbore = YAW_HUB_SKIRT_BORE                            # r11 -> Phi 22 clears horn
    rboss = YAW_HUB_BOSS_OD / 2.0
    rinner = YAW_BEARING_INNER_OD / 2.0

    # Jun 2026 flush-horn fix: the real STS3215 disc horn is ~2 mm thick and
    # sits RECESSED into the servo's output face, so the horn top is
    # essentially FLUSH with the servo body -- there is no protruding
    # horn pedestal for the hub to wrap a skirt down around.  The old
    # annular LOWER boss (z[-4.5, 0], Phi 22 bore that "clears the disc horn")
    # was therefore a recess for a horn that does not protrude; on the real
    # hardware that skirt would just crash into the servo face.  Dropped: the
    # hub now seats FLAT on the horn top at z = 0 (its bottom face meets the
    # horn where it actually sits).  The central spline collar / screw bump is
    # cleared by the Phi DISC_HORN_COLLAR_OD recess cut below.
    # Boss: solid, rides BOTH inner races.  Jun 2026 flush-horn DEPTH fix: the
    # real disc horn sits FLUSH on the servo front face -- RECESSED 2 mm into the
    # 4 mm mount-plate bore -- so its top is YAW_HORN_REACH_DOWN below the
    # frozen output plane (z = 0), NOT 3 mm.  The wide boss descends to the
    # plate top (YAW_HUB_BOSS_WIDE_BOT_Z) where the Phi 44 tower relief still
    # clears it; below that it necks to the Phi 20 drive nub that passes
    # through the Phi 24 plate bore down to YAW_HUB_BOSS_BOT_Z, seating on
    # the real horn so the 4 drive bolts bite the disc without pulling the
    # retain lip off the upper race.  The boss stays below the lower bearing
    # (z >= +0.5), so the inner-race insertion path is unchanged.
    # Wide boss (rides BOTH inner races) spans the mount-plate top up to the
    # platform.  Below the plate top it NECKS to the Phi 20 drive nub that
    # threads through the plate bore down to the recessed flush horn so the
    # 4 drive bolts bite the disc and the stack seats flush (the user's
    # "3 mm bump <= the chassis hole").
    uboss = _cyl(rboss, YAW_HUB_BOSS_TOP_Z - YAW_HUB_BOSS_WIDE_BOT_Z)
    uboss.apply_translation([0.0, 0.0,
                             0.5 * (YAW_HUB_BOSS_WIDE_BOT_Z + YAW_HUB_BOSS_TOP_Z)])
    drive_nub = _cyl(YAW_HUB_DRIVE_NUB_OD / 2.0,
                     YAW_HUB_BOSS_WIDE_BOT_Z - YAW_HUB_BOSS_BOT_Z)
    drive_nub.apply_translation(
        [0.0, 0.0, 0.5 * (YAW_HUB_BOSS_BOT_Z + YAW_HUB_BOSS_WIDE_BOT_Z)])
    # Single inner-race retain flange (OD = inner-race OD = Phi 29; clears the
    # Phi 33 outer-race ID so it never rubs the STATIONARY race).  This is the
    # uflange ABOVE the upper race (z[14.5, 16.5]) -- it makes the upper
    # bearing the axially-LOCATED ("fixed") race of a fixed/floating pair.
    # Aug 2026 6805 swap: thickened 1 -> 2 mm (user: the lip that rides the
    # bearing was too thin) and its shelf onto the race grew from ~1 mm
    # (Phi 30..32 band) to ~2 mm (Phi 25..29 band).  Top at 16.5 still
    # clears the platform underside (BOSS_TOP = 17).
    #
    # The old design also carried an lflange ABOVE the lower race (z[-1, 0]).
    # That flange was the "lip" the user could not push the upper bearing over:
    # the boss is capped on top by the Phi 44+ turntable platform (z >= +7.5),
    # so the only open end an inner race can slide on from is the boss BOTTOM
    # (z = -4.5).  The lflange (Phi 32 > the Phi 30 bore) sat squarely in that
    # bottom-load path between the open end and the upper race's z[+2, +6] seat,
    # trapping the upper inner race between it (below) and the uflange/platform
    # (above) on a Phi 29.8 boss -- physically un-assemblable from EITHER end.
    #
    # Dropping the lflange leaves a clean Phi 29.8 slide path from the boss
    # bottom all the way up to the uflange, so BOTH inner races slide on from
    # below (upper race seats UP against the uflange; lower race rides free).
    # The lower bearing becomes the FLOATING race: its OUTER ring is still
    # positively captured in the chassis tower (z = -5 seat) + yaw_bearing_cap
    # neck (z = -1), and the 4 x M3 disc-horn clamp preloads the whole hub down
    # onto the located (upper) bearing -- so no race is left free, and nothing
    # of OD > the bearing bore sits between either seat and the open end it
    # loads from (verified by check_hub_inner_race_insertion_path).
    uflange = _cyl(rinner, 2.0)                    # z[14.5, 16.5], 2 mm thick
    uflange.apply_translation([0.0, 0.0, YAW_BEARING_UPPER_TOP_Z + 1.0])
    # Top platform (turntable disc) above the tower.  Its rim must reach OUT
    # to at least the dust-lip skirt OD so the skirt HANGS from the platform
    # as a SINGLE connected solid.  The skirt clears the Phi 44 chassis tower
    # by a radial running gap (its inner wall sits at tower_OD/2 + clearance =
    # 22.6 mm), so a bare YAW_HUB_OD (Phi 44, r=22) platform stops 0.6 mm
    # short of the skirt's inner wall and leaves it floating (a disconnected
    # island -- caught by the [1c] single-body connectivity guard).  The part
    # footprint is already the skirt OD, so widening the platform adds no
    # bounding-box / nesting / tray cost.
    plat_od = max(YAW_HUB_OD, YAW_HUB_DUST_LIP_OD)
    plat = _cyl(plat_od / 2.0, YAW_HUB_PLATFORM_Z1 - YAW_HUB_BOSS_TOP_Z)
    plat.apply_translation([0.0, 0.0,
                            0.5 * (YAW_HUB_BOSS_TOP_Z + YAW_HUB_PLATFORM_Z1)])
    # Dust labyrinth lip: a STOUT skirt hanging from the platform OUTSIDE
    # the chassis tower OD, overlapping it with a small radial running gap
    # (grit guard).  Sized YAW_HUB_DUST_LIP_WALL thick x LIP_H tall so it
    # is a real wall, not a flimsy 1.5 mm flange.
    lip_inner_r = YAW_BEARING_OD / 2.0 + YAW_TOWER_WALL + YAW_HUB_DUST_LIP_CL
    lip_z1 = YAW_HUB_BOSS_TOP_Z          # +7.5 (under the platform)
    lip_z0 = lip_z1 - 4.0                # 4 mm tall skirt
    lip = _diff(_cyl(YAW_HUB_DUST_LIP_OD / 2.0, lip_z1 - lip_z0),
                _cyl(lip_inner_r, (lip_z1 - lip_z0) * 3))
    lip.apply_translation([0.0, 0.0, 0.5 * (lip_z0 + lip_z1)])

    hub = _union(uboss, drive_nub, uflange, plat, lip)

    cuts = []
    # Disc-horn drive bolts (4 x M3, PCD14).  CLAMP-THROUGH: heads recess at
    # the platform top, shanks run down to the horn, preloading the whole
    # hub onto the disc horn (axial-preload path for the bearing pair).
    # Mostly-compliant interface: clearance is mildly oversized (Phi 3.7)
    # so the BEARINGS still set concentricity and the horn mainly transmits
    # torque; Aug 2026 tightened from Phi 4.2 after bench slop complaints.
    # Spline-collar / centre-screw clearance: the disc's raised central
    # collar (Phi ~9) protrudes ABOVE the horn top (z=0) into the solid
    # upper boss.  Recess it so the hub seats on the disc FACE (via the
    # bearings) and never bottoms on the collar.
    # Recess starts at the (now lowered) boss bottom face = real horn top and
    # rises COLLAR_DEPTH+1 so the disc's raised central spline collar clears.
    collar_h = DISC_HORN_COLLAR_DEPTH + 1.0
    collar = _cyl(DISC_HORN_COLLAR_OD / 2.0 + 0.25, collar_h)
    collar.apply_translation([0.0, 0.0, YAW_HUB_BOSS_BOT_Z + collar_h / 2.0])
    cuts.append(collar)
    # Through-centre + 4 drive bolts: shared M3 x 20 stock, shared head-seat
    # Z, shared counterbore OD so every screw head sits at the same height.
    hub_h = (YAW_HUB_PLATFORM_Z1 - YAW_HUB_BOSS_BOT_Z) + 4.0
    centre = _cyl(HORN_CENTRE_OD / 2.0, hub_h)
    centre.apply_translation(
        [0.0, 0.0, 0.5 * (YAW_HUB_BOSS_BOT_Z + YAW_HUB_PLATFORM_Z1)])
    cuts.append(centre)
    # Head counterbore from the platform top down to the seat plane.
    # Aug 2026 seat fix: the old cut was shifted 0.5 mm LOW ("overshoot below
    # the seat"), so the physical counterbore floor sat 0.5 mm below the
    # declared YAW_HUB_HORN_HEAD_SEAT_Z -- the fastener registry places the
    # M3 head undersides exactly AT the seat constant, so the engagement
    # probe found 0.5 mm of air under every head ("head bearing in air").
    # The bore now bottoms exactly at the seat plane (overshoot upward only).
    # Aug 17 2026: the CENTRE station has its own 1 mm-deeper seat (the
    # spline tap can take the extra engagement; the corner seats are capped
    # by the servo case under the disc -- see the seat constants block).
    cb_h = (YAW_HUB_PLATFORM_Z1 - YAW_HUB_HORN_HEAD_SEAT_Z) + 1.0
    cb_z = YAW_HUB_HORN_HEAD_SEAT_Z + cb_h / 2.0
    cb_c_h = (YAW_HUB_PLATFORM_Z1 - YAW_HUB_HORN_CENTRE_SEAT_Z) + 1.0
    centre_head = _cyl(YAW_HUB_HORN_HEAD_CB_OD / 2.0, cb_c_h)
    centre_head.apply_translation(
        [0.0, 0.0, YAW_HUB_HORN_CENTRE_SEAT_Z + cb_c_h / 2.0])
    cuts.append(centre_head)
    r = DISC_HORN_BOLT_PCD / 2.0
    # Aug 2026: was +0.8 (Phi 4.2) for a very compliant torque-only fit.
    # That much clearance on the Phi 14 PCD alone can feel like ~10 deg of
    # lost motion before the shanks catch.  +0.3 (Phi 3.7) still leaves the
    # bearings as the primary locator but cuts the screw-hole slop a lot.
    drive_clear = DISC_HORN_BOLT_OD + 0.3                 # Phi 3.7
    for t in DISC_HORN_BOLT_ANGLES_RAD:
        cx, cy = r * np.cos(t), r * np.sin(t)
        h = _cyl(drive_clear / 2.0, YAW_HUB_PLATFORM_Z1 * 2 + 4.0)
        h.apply_translation([cx, cy, YAW_HUB_PLATFORM_Z1 / 2.0])
        cuts.append(h)
        cb = _cyl(YAW_HUB_HORN_HEAD_CB_OD / 2.0, cb_h)
        cb.apply_translation([cx, cy, cb_z])
        cuts.append(cb)
    # Part B join pilots: M3 self-tap into the platform from its top face.
    # Skipped in the one-piece coxa_link (no separate Part B any more).
    if not one_piece:
        for (jx, jy) in _coxa_join_bolt_centres():
            p = _cyl(COXA_JOIN_PILOT_OD / 2.0, YAW_HUB_PAD_T)
            p.apply_translation([jx, jy,
                                 YAW_HUB_PLATFORM_Z1 - YAW_HUB_PAD_T / 2.0])
            cuts.append(p)
    return _diff(hub, *cuts)


def _coxa_partA_envelope() -> trimesh.Trimesh:
    """Solid clearance envelope of Part A (turntable + dust lip) used to
    carve Part B so the two printed parts mate without interpenetrating."""
    # Platform now overhangs to the dust-lip skirt OD (see make_coxa_yaw_hub),
    # so the envelope tracks the LARGER of the pad OD / skirt OD.
    env = _cyl(max(YAW_HUB_OD, YAW_HUB_DUST_LIP_OD) / 2.0 + 0.6,
               YAW_HUB_PLATFORM_Z1 + 6.0)
    env.apply_translation([0.0, 0.0, (YAW_HUB_PLATFORM_Z1 + 6.0) / 2.0 - 6.0])
    lip = _cyl(YAW_HUB_DUST_LIP_OD / 2.0 + 0.6, YAW_HUB_BOSS_TOP_Z + 6.0)
    lip.apply_translation([0.0, 0.0, (YAW_HUB_BOSS_TOP_Z + 6.0) / 2.0 - 6.0])
    return _union(env, lip)


def make_coxa_hip_bracket(*, one_piece: bool = False) -> trimesh.Trimesh:
    """Part B -- the hip bracket that carries the HIP joint's FIXED side
    (servo cradle + 688 bearing housing).  Coxa-local frame, identical to
    Part A.  (Legacy two-part path: the foot bolts onto the hub platform
    with 4 x M3 and is carved against the Part A envelope so the two
    printed parts mate flush without interpenetrating.)

    Aug 2026 one-piece merge: with ``one_piece=True`` this is a SUB-SOLID of
    the merged printed ``coxa_link`` (see ``make_coxa_link_part``):
      * the Part-A envelope carve is skipped (interpenetrating the hub is
        the whole point of a union);
      * the 4 M3 join clearance holes + head counterbores are skipped (the
        joint no longer exists).

    Aug 17 2026 sink pass (user: "push the bracket holding the servo down
    to overlap with the cap more ... making the height lower would help a
    lot getting the screws in and remove unnecessary material"): the foot
    slab no longer stacks ABOVE the platform -- it spans the full platform
    band plus COXA_WELL_FLOOR_LIFT (z YAW_HUB_BOSS_TOP_Z +18 ..
    YAW_HUB_PLATFORM_Z1 + 2 = +26), fusing through the platform inside its
    Phi ~52 footprint and hanging as an 8 mm slab outside it (a flush
    floor left a 6 mm sheet carrying the whole cradle -- thin-sheet
    FAIL; 8 mm is the thickness the old raised foot already proved).  The
    hip-servo well floor drops +31 -> +26, so the whole cradle rides 5 mm
    lower.  Clearances of the slab's exposed bottom face (+18):
    stationary cap rim tops at +16.5 (YAW_HUB_CAP_AXIAL_CL = 1.5 mm --
    widened from 0.5 by the Aug 17 2026 scrape fix), cap ear bosses top
    at +8.5, and the chassis_top deck band sits at coxa-local z
    6.75..8.75 and radius >= 42.5 mm (slab corner sweep 40.4).  The inner
    bearing interfaces (boss, uflange, dust skirt) are untouched."""
    foot_z0 = YAW_HUB_BOSS_TOP_Z                          # +18: platform underside plane
    foot_z1 = YAW_HUB_PLATFORM_Z1 + COXA_WELL_FLOOR_LIFT  # +26: well floor

    # Hip fixed side, anchored so its disc-horn-top -> COXA_HIP_ANCHOR =
    # (COXA_LENGTH, COXA_HIP_ANCHOR_Y, DROP).  COXA_HIP_ANCHOR_Y slides the
    # servo tower so its footprint is CENTRED on the yaw axis (coxa y=0)
    # rather than hanging off to +Y.  LEG_PITCH_AXIS (-Y) orients the cradle
    # OPEN face / clamp UP, so the cradle's solid back face seats DOWN onto
    # the foot/hub.
    #
    # Aug 16 2026 flatten pass (user: "the coxa hub has four meaningless
    # holes on one of the shorter sides where the servo sits and a weird
    # cutout channel on the opposite side, both are pointless now"):
    #   * end_face_bolts=False -- the 4x M2.5 end-face body screws through
    #     the -X wall are RETIRED (same story as the knee's Jul 2026
    #     retirement: the hip servo is already held by the clamp cap + lip
    #     + output-face seat, and on the bench the screws were never
    #     installed -- the empty holes only weakened the wall);
    #   * wire_exit=False -- the DS3225-era L-corridor + boot channel
    #     through the +X far wall is DELETED (the real STS3215's bus
    #     cables leave via its BACK-face 5264 ports through the sandwich's
    #     open clamp face; the corridor pierced a solid wall for nothing).
    # This was the LAST cradle carrying either feature -- every sandwich
    # cradle is now a clean 4-wall box.
    #
    # Aug 17 2026 rear tab (user: "copy that same part to be on the coxa
    # link as well so I can screw into the servo from both sides"): the hip
    # cradle grows the SAME rear retention tab as the femur knee cradle --
    # 2x M2.5 self-tappers into the hip servo's rear molded hole pair
    # nearer the -X (inboard/wire) end; the connector-end pair stays open
    # for the bus harness.  See the FEMUR_REAR_TAB_* constants block for
    # the shared geometry + the back-face step ("4 mm bump") constraint.
    # In coxa frame the tab is a VERTICAL 5.5 mm plate standing on the
    # +Y side of the cradle back face (joint-local -Z -> coxa +Y): screw
    # axes run along coxa +Y, driven from outside before the femur yoke
    # goes on; the heads sit FLUSH in 2 mm recesses (Aug 17 2026 head-
    # recess pass) and the femur's swinging passive-side arm passes
    # ~1.5 mm over the tab's outer face, exactly as at the knee.
    fixed = _sandwich_fixed_side(end_face_bolts=False, wire_exit=False,
                                 rear_tab=True)
    M = _joint_place(COXA_HIP_ANCHOR,
                     x_dir=(1, 0, 0), z_dir=LEG_PITCH_AXIS)
    fixed.apply_transform(M)
    # Foot sizing must NOT see the tab: the foot slab spans the CRADLE
    # footprint (fb +/- 1); letting it grow +3.5 under the tab would push
    # its +Y edge into the femur yoke-arm sweep band that the r=16.75
    # relief cylinders keep clear (see make_coxa_link_part).
    fixed_notab = _sandwich_fixed_side(end_face_bolts=False, wire_exit=False)
    fixed_notab.apply_transform(M)
    fb = fixed_notab.bounds

    # Foot plate on the hub platform, directly under the cradle's solid
    # back (now facing DOWN).  It spans the cradle's X/Y footprint so the
    # back seats flat on it, and covers the 4 join-bolt heads.  The servo
    # body is wider than the hub, so the foot overhangs the platform and
    # carries the moment into the 4 join bolts + the bearing pair below.
    foot_x0 = float(fb[0][0]) - 1.0
    foot_x1 = float(fb[1][0]) + 1.0
    foot_y0 = float(fb[0][1]) - 1.0
    foot_y1 = float(fb[1][1]) + 1.0
    foot = _box((foot_x1 - foot_x0, foot_y1 - foot_y0, foot_z1 - foot_z0),
                center=(0.5 * (foot_x0 + foot_x1), 0.5 * (foot_y0 + foot_y1),
                        0.5 * (foot_z0 + foot_z1)))

    # Short pedestal closing the small gap between the foot top and the
    # cradle's down-facing back wall (no long inboard arm/riser any more --
    # the cradle sits coaxially right on top of the hub).
    back_z0 = float(fb[0][2])
    if back_z0 > foot_z1:
        ped = _box((foot_x1 - foot_x0, foot_y1 - foot_y0, back_z0 - foot_z1 + 0.5),
                   center=(0.5 * (foot_x0 + foot_x1), 0.5 * (foot_y0 + foot_y1),
                           0.5 * (foot_z1 + back_z0 + 0.5)))
        body = _union(foot, ped, fixed)
    else:
        body = _union(foot, fixed)
    # Carve away anything that would interpenetrate Part A (turntable/lip).
    # One-piece merge: skipped -- the foot welds straight into the platform.
    if not one_piece:
        body = _diff(body, _coxa_partA_envelope())

    # Femur-swing clearance: at the top of the femur up-pitch workspace
    # (fem=+30) the femur swings DOWN-and-OUTBOARD past the hip and can
    # clip the bracket's OUTBOARD-BOTTOM corner.  Cut that corner back:
    # remove all outboard (x >= FEMUR_CLEAR_X) material below
    # FEMUR_CLEAR_Z.  The sweep band rides RIGIDLY with the hip axis, so
    # the Z threshold is expressed relative to COXA_HIP_DROP (originally
    # bench-derived as 18.5 when the drop was 43.4, i.e. drop - 24.9).
    # After the Aug 17 2026 8 mm sink the band sits at z ~5.5-10 while the
    # foot slab bottom is +17, so the cut is currently a no-op safety
    # margin -- kept so any future re-raise of the cradle stays safe
    # (check_workspace_self_collision re-verifies either way).
    FEMUR_CLEAR_X = 27.5
    FEMUR_CLEAR_Z = COXA_HIP_DROP - 24.9
    clr = _box((40.0, foot_y1 - foot_y0 + 4.0, FEMUR_CLEAR_Z - 4.0),
               center=(FEMUR_CLEAR_X + 20.0, 0.5 * (foot_y0 + foot_y1),
                       0.5 * (4.0 + FEMUR_CLEAR_Z)))
    body = _diff(body, clr)

    # Printability relief (Jun 2026): the foot is sized to the full cradle
    # bbox (fb +/- 1 mm) and is a thick (6 mm platform-band) slab, so on the
    # FRONT (-Y, LOW-Y) side -- the disc-horn / output face, where coxa-Z is
    # HIGHEST -- its bottom-outer edge is a proud, sharp lip that prints as a
    # fragile unsupported ledge.  Chamfer that front-bottom-outer edge back at
    # 45 deg.  This is OUTBOARD of the Phi YAW_HUB hub seat (which is central,
    # within Phi 44) and the 4 join bolts (y = +/-COXA_JOIN_BOLT_PCD/2), and
    # BELOW the cradle's seating plane, so the seating face, bolt pattern,
    # bearing housing and cradle are all untouched.
    chamfer = LIP_RELIEF_CHAMFER
    wedge = _box((foot_x1 - foot_x0 + 2.0, 50.0, 50.0),
                 center=(0.5 * (foot_x0 + foot_x1), 0.0, 0.0))
    wedge.apply_transform(rotation_matrix(np.deg2rad(-45.0), [1, 0, 0]))
    _n = np.array([0.0, -1.0, -1.0]) / np.sqrt(2.0)
    _mid = np.array([0.0, foot_y0 + chamfer / 2.0, foot_z0 + chamfer / 2.0])
    _ctr = _mid + _n * 25.0
    wedge.apply_translation([0.0, _ctr[1], _ctr[2]])
    body = _diff(body, wedge)

    # Join clearance holes + head counterbores (driven from the foot top).
    # One-piece merge: no hub<->bracket joint, so no join hardware.
    if one_piece:
        return body
    cuts = []
    for (jx, jy) in _coxa_join_bolt_centres():
        h = _cyl(COXA_JOIN_BOLT_OD / 2.0, (foot_z1 - foot_z0) * 3)
        h.apply_translation([jx, jy, foot_z0])
        cuts.append(h)
        cb = _cyl(INSERT_M3_BOLT_HEAD_OD / 2.0, INSERT_M3_BOLT_HEAD_H + 0.5)
        cb.apply_translation([jx, jy, foot_z1 - (INSERT_M3_BOLT_HEAD_H + 0.5) / 2.0])
        cuts.append(cb)
    return _diff(body, *cuts)


def make_coxa_link_part() -> trimesh.Trimesh:
    """The ONE-PIECE printed coxa ``coxa_link`` (Aug 2026 merge, user: "the
    hip bracket and coxa yaw hub are linked together and theres no reason
    for them not to be one piece if theres screw holes to get the screws
    all the way down to yaw motor horn").

    One printed body =
      yaw turntable hub (bolts the disc horn, rides the touching 6805 pair)
      + hip bracket (foot slab + hip servo cradle + 688 housing),
    with the foot slab fused through the full platform band (Aug 17 2026
    sink pass -- see make_coxa_hip_bracket).  The 4 M3 hub<->bracket join
    bolts (and their pilots / counterbores) are GONE.

    The split had existed only so the hub could be bolted to the disc horn
    before the bracket went on.  The merged part instead carries 5 vertical
    HEAD-ACCESS SHAFTS (Phi YAW_HUB_HORN_HEAD_CB_OD, same as the head
    counterbores they extend): from each head-seat plane (corner seats at
    YAW_HUB_HORN_HEAD_SEAT_Z = +17.75; centre 1 mm deeper at
    YAW_HUB_HORN_CENTRE_SEAT_Z) straight up through the platform/foot,
    opening at the hip-servo well floor (+26).  Assembly: drop the 4
    drive bolts + centre spline screw down the shafts and torque them
    BEFORE the hip servo is lowered into its cradle -- the heads sit only
    ~5.3 mm below the shaft mouths, so a normal driver bit reaches them.
    The seated hip servo then covers the shaft mouths (head tops clear
    the servo bottom by ~5.3 mm).

    Bearing assembly is unchanged: both 6805 inner races and the loose
    yaw_bearing_cap all slide onto the hub boss from BELOW (the cap's
    Phi 37 bore could never pass the Phi 48 platform anyway, split or not),
    then the whole stack drops into the chassis tower."""
    body = _union(make_coxa_yaw_hub(one_piece=True),
                  make_coxa_hip_bracket(one_piece=True))
    # Yoke-end sweep clearance (Aug 17 2026, consequence of the sink pass):
    # the femur hip yoke's arm ends are full discs of radius ~16 around the
    # hip axis (they carry the horn pads), so their swept volume over ANY
    # pitch angle is a solid cylinder.  Before the sink the slab/platform
    # sat well below the hip axis and those cylinders cleared; now the
    # slab's top corner pokes into the +Y arm's disc (13 mm^3 at the
    # stance pose -- caught by check_interference).  Carve a Y-axis
    # cylinder of r = disc 16 + 0.75 running clearance through each ARM
    # BAND ONLY (walls at y ~[-23,-19]/[15,16] and the dust skirt, z <= 18
    # < cut bottom ~21.65, are untouched; only the slab-edge lens at
    # z ~21.65..26, x ~12.5 +/- 16 in the two narrow bands is removed).
    hip_ax_x, _, hip_ax_z = COXA_HIP_ANCHOR
    for (ylo, yhi) in ((21.75, 30.0), (-31.0, -24.75)):
        sweep = _cyl(16.75, yhi - ylo)
        sweep.apply_transform(rotation_matrix(np.pi / 2.0, [1, 0, 0]))
        sweep.apply_translation([hip_ax_x, 0.5 * (ylo + yhi), hip_ax_z])
        body = _diff(body, sweep)
    # Head-access shafts: extend the 5 shared head counterbores (4 drive
    # bolts on DISC_HORN_BOLT_PCD + the centre spline screw) up through
    # everything the bracket stacked over the platform top.  Bottom lands
    # exactly on the head-seat plane so the seat annulus is untouched.
    shaft_top_z = 80.0   # safely above the cradle's open clamp face
    stations = [(0.0, 0.0, YAW_HUB_HORN_CENTRE_SEAT_Z)]  # centre: deeper seat
    r = DISC_HORN_BOLT_PCD / 2.0
    stations += [(r * np.cos(t), r * np.sin(t), YAW_HUB_HORN_HEAD_SEAT_Z)
                 for t in DISC_HORN_BOLT_ANGLES_RAD]
    cuts = []
    for (sx, sy, seat_z) in stations:
        s = _cyl(YAW_HUB_HORN_HEAD_CB_OD / 2.0, shaft_top_z - seat_z)
        s.apply_translation([sx, sy, 0.5 * (seat_z + shaft_top_z)])
        cuts.append(s)
    return _diff(body, *cuts)


def make_coxa_link() -> trimesh.Trimesh:
    """Yaw-driven coxa.  Aug 2026 merge: the assembled link IS the single
    printed part ``coxa_link`` (see make_coxa_link_part) -- the old
    two-part split (coxa_yaw_hub Part A + coxa_hip_bracket Part B, 4 x M3
    join bolts) is retired.  Kinematics are identical to every prior
    revision (hip fixed side at (COXA_LENGTH, 0, COXA_HIP_DROP))."""
    return make_coxa_link_part()


def _femur_knee_fixed_solid() -> trimesh.Trimesh:
    """The knee joint FIXED side (servo bracket + 688 bearing housing) as a
    SUB-SOLID of the one-piece femur (see make_femur_link_part).  Knee
    joint-local frame (+Z = knee output).  Jul 2026 merge #2: the old femur
    spar socket boss, bore and transverse retention pin are RETIRED -- the
    fused Phi 14 spar arrives from the hip and buries into this side's well
    wall, so no socket geometry exists here at all.  NOT an emitted printed
    part on its own (kept as a builder for the verifier's targeted knee
    cradle / clamp-cap checks).

    ``end_face_bolts=False`` (Jul 2026, user: "theres four holes in the one
    piece femur link where you added the connection which serve no purpose
    and weaken the connection"): the     spar covers this cradle's -X wall from
    outside, so the 4x M2.5 end-face screws could never be installed at the
    knee anyway -- the empty holes + counterbores just punched through the
    spar-to-wall junction.  The knee servo is retained by the clamp cap +
    retaining lip (the hip cradle on coxa_link keeps its 4 bolts).

    ``farwall_pad=True`` (Aug 2026): external buttress pad on the far
    (+X) wall -- the wall the field crack actually ran along; see the
    FEMUR_KNEE_FARWALL_PAD_* constants block.

    ``rear_tab=True`` (Aug 2026, user): rear retention tab under the open
    back face -- 2x M2.5 self-tappers into the servo's rear molded hole
    pair nearer the spar/yoke end (the connector-end pair stays open for
    the bus harness); see the FEMUR_REAR_TAB_* constants block.

    ``wire_exit=False`` (Aug 2026, user: "this top can just be a flat wall
    with no weird cutouts"): no DS3225-era wire corridor at the knee -- the
    STS3215 bus cables leave via the back-face ports through the open back,
    and the far (+X) wall is now one flat full-width doubled surface."""
    return _sandwich_fixed_side(end_face_bolts=False, farwall_pad=True,
                                rear_tab=True, wire_exit=False)


def make_femur_link_part() -> trimesh.Trimesh:
    """The ONE-PIECE printed femur ``femur_link`` (Jul 2026 merge #2, user:
    "combine the femur knee bracket with the femur hip yoke and make that
    connection very solid (cylinder size of our diameter)").  HIP
    joint-local frame; the knee fixed side sits FEMUR_LENGTH out along +X
    (same relative placement make_femur_link always used).

    One printed body =
      hip moving yoke (bolts the driven + passive hip disc horns; Aug 2026:
      DOUBLED 8 mm spine plate, FEMUR_YOKE_SPINE_PAD_T)
      + SOLID Phi FEMUR_SPAR_OD (18) spar bridging the full inter-well gap,
        flared into BOTH end faces by small gusset cones
      + knee fixed side (servo cradle + 688 bearing housing).
    No socket bore, no slip fit, no retention pin anywhere in the femur.

    Jun 2026 ROM-clearance refit (unchanged): the connecting web sits +4 mm
    outboard (see _YOKE_SPINE_X0/X1), keeping the swept web clear of the
    fixed clamp cap and the coxa across the full femur ROM."""
    yoke = _sandwich_moving_yoke(tube_socket=False,
                                 spine_extra_t=FEMUR_YOKE_SPINE_PAD_T,
                                 pad_extra_reach=YOKE_PAD_EXTRA_REACH)
    spar = _femur_fused_spar()
    kb = _femur_knee_fixed_solid()
    kb.apply_translation([FEMUR_LENGTH, 0.0, 0.0])
    return _union(yoke, spar, kb)


def make_tibia_knee_yoke() -> trimesh.Trimesh:
    """Tibia's KNEE end: the moving yoke (driven by the knee disc horn,
    stub into the knee 688 bearing) with the tibia CF-tube socket toward
    the foot.  Joint-local frame.  Aug 2026: the spine plate is doubled
    (TIBIA_YOKE_SPINE_PAD_T) after the field crack through it, the
    retention-pin cross-hole is REMOVED (user: epoxy-only retention), and
    both horn pads reach YOKE_PAD_EXTRA_REACH (0.5 mm) deeper to close
    the bench-measured ~1 mm total clevis-to-horn gap."""
    return _sandwich_moving_yoke(tube_socket=True, socket_pin=False,
                                 spine_extra_t=TIBIA_YOKE_SPINE_PAD_T,
                                 pad_extra_reach=YOKE_PAD_EXTRA_REACH)


# make_tibia_foot_fitting / make_tibia_foot_fitting_plus4 /
# tibia_foot_hinge_local: RETIRED Aug 2026 with the hinged foot.  The
# tibia CF tube now takes a pressed-on TPU ``foot_boot`` (see
# make_foot_boot above) -- no printed fitting, no hinge pin.


# ---- Assembled rigid bodies (for the verifier / inspector / mujoco) ------
# ``make_femur_link`` / ``make_tibia_link`` keep their legacy names and
# legacy LOCAL FRAMES so the verifier's / inspector's transform chains
# (origin = pad mating face; joint axis at (0, -HORN_STACK_H, 0) along +Y;
# next joint / foot at +X * length) keep working.  Since the Jul 2026 merge
# #2 the femur rigid body IS the single printed part (make_femur_link_part);
# the tibia is still the two printed fittings + the CF tube.  The PRINTED
# parts are emitted separately by main() (femur_link / tibia_knee_yoke / ...).

def _femur_socket_world(M):  # spar line at the old socket-mouth station
    return M @ np.array([_YOKE_SOCKET_X, 0.0, JOINT_SOCKET_Z, 1.0])


def make_femur_link() -> trimesh.Trimesh:
    """Assembled femur = the ONE-PIECE printed femur (make_femur_link_part),
    re-anchored.  Local frame: origin = hip disc-horn-top (the moving-link
    mating face) ON the hip-pitch axis (+Y); the knee disc-horn-top sits at
    (FEMUR_LENGTH,0,0).  The servo/horn/bearing stacks extend toward -Y
    (body side)."""
    xz = (1, 0, 0), LEG_PITCH_AXIS
    Mh = _joint_place((0.0, 0.0, 0.0), *xz)
    fl = make_femur_link_part()
    fl.apply_transform(Mh)
    return fl


def make_tibia_link() -> trimesh.Trimesh:
    """Assembled tibia: knee yoke + dia-8 CF tube + TPU foot boot.  Local
    frame: origin = knee disc-horn-top (mating face) ON the knee-pitch
    axis (+Y), foot toward +X."""
    xz = (1, 0, 0), LEG_PITCH_AXIS
    Mk = _joint_place((0.0, 0.0, 0.0), *xz)
    ky = make_tibia_knee_yoke(); ky.apply_transform(Mk)
    a = (Mk @ np.array([_YOKE_SOCKET_X, 0.0, JOINT_SOCKET_Z, 1.0]))[:3]
    tube_end = a + np.array([TIBIA_LENGTH - FOOT_BOOT_TIP_L, 0.0, 0.0])
    boot = make_foot_boot()
    boot.apply_transform(_frame(tube_end, (1, 0, 0), (0, 0, 1)))
    return _union(ky, _tube_between(a, tube_end, LEG_TUBE_OD / 2.0), boot)


# ---------------------------------------------------------------------------
# Assembly preview
# ---------------------------------------------------------------------------

def leg_named_parts_in_body_frame(
    leg_index: int,
) -> list[tuple[str, trimesh.Trimesh]]:
    """Return one leg's INDIVIDUAL printed parts (+ the two CF tube
    visuals), each transformed into the chassis frame in standing pose,
    as a list of ``(part_name, mesh)`` pairs.

    This is the BOM-correct decomposition -- the femur and tibia are NOT
    single parts but ``yoke + dia-8 CF tube + bracket/fitting`` sandwiches,
    so the assembly view can call out each printed piece separately
    instead of the merged ``femur_link`` / ``tibia_link`` proxies.

    Kinematic chain (mirrors hexapod_walker._leg_in_body_frame):

       coxa_bracket  ->  yaw (Z axis)  ->  coxa_link  ->
       hip_pitch (Y_local axis)        ->  femur      ->
       knee_pitch (Y_local axis)       ->  tibia      ->  foot.
    """
    apothem = CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * np.pi / 3
    edge_mid = np.array([apothem * np.cos(a),
                         apothem * np.sin(a),
                         0.0])
    outboard = np.array([np.cos(a), np.sin(a), 0.0])
    z_hat    = np.array([0.0, 0.0, 1.0])

    named: list[tuple[str, trimesh.Trimesh]] = []

    # World transform from coxa-local (origin = yaw axis at the yaw
    # disc-horn top): yaw rotation about Z then translate to the edge.
    PLASTIC_HORN_H = 5.0   # mm, hobby-servo plastic horn height
    yaw_output_z = ((SERVO_BODY_H - WELL_RIM_Z)
                     + SERVO_OUTPUT_H
                     + PLASTIC_HORN_H
                     + HORN_ADAPTER_T)
    W = rotation_matrix(a, [0, 0, 1])
    W[:3, 3] = edge_mid + yaw_output_z * z_hat

    def _placed(mesh, M_local):
        m = mesh.copy()
        m.apply_transform(W @ M_local)
        return m

    # ----- Joint geometry in coxa-local (frozen kinematics) -----------
    ty = np.array(LEG_PITCH_AXIS)               # hip/knee/tibia output axis
    p  = np.deg2rad(STANCE_FEMUR_DEG)
    pt = np.deg2rad(STANCE_FEMUR_DEG + STANCE_TIBIA_DEG)
    femur_dir = rotation_matrix(p, [0, 1, 0])[:3, :3] @ np.array([1.0, 0, 0])
    tibia_dir = rotation_matrix(pt, [0, 1, 0])[:3, :3] @ np.array([1.0, 0, 0])
    H = np.array(COXA_HIP_ANCHOR)                            # hip axis pt
    K = H + FEMUR_LENGTH * femur_dir                          # knee axis pt

    # ----- Coxa (yaw-driven; carries the hip fixed side) --------------
    named.append(("coxa_link", _placed(make_coxa_link(), np.eye(4))))

    # ----- Femur: ONE printed part (Jul 2026 merge #2) ------------------
    M_femur = _joint_place(H, femur_dir, ty)
    named.append(("femur_link", _placed(make_femur_link_part(), M_femur)))

    sock_pt = np.array([_YOKE_SOCKET_X, 0.0, JOINT_SOCKET_Z, 1.0])

    # ----- Tibia: knee yoke + CF tube + foot fitting ------------------
    M_tibyoke = _joint_place(K, tibia_dir, ty)
    named.append(("tibia_knee_yoke", _placed(make_tibia_knee_yoke(), M_tibyoke)))

    tib_sock_w = (W @ M_tibyoke @ sock_pt)[:3]
    tibia_dir_w = W[:3, :3] @ tibia_dir
    # Foot "up" axis is anchored to the FIXED +Y tangential (NOT the
    # signed LEG_PITCH_AXIS) so the foot fitting stays upright regardless
    # of which way the leg cradles' open faces point.
    ty_w        = W[:3, :3] @ np.array([0.0, 1.0, 0.0])
    tube_end_w = tib_sock_w + tibia_dir_w * (TIBIA_LENGTH - FOOT_BOOT_TIP_L)
    named.append(("tibia_tube",
                  _tube_between(tib_sock_w, tube_end_w, LEG_TUBE_OD / 2.0)))

    foot_frame = _frame(tube_end_w, tibia_dir_w, np.cross(tibia_dir_w, ty_w))
    boot = make_foot_boot()
    boot.apply_transform(foot_frame)
    named.append(("foot_boot", boot))

    return named


def _leg_in_body_frame(leg_index: int) -> trimesh.Trimesh:
    """One leg in the chassis frame (standing pose) as a single mesh --
    the union of ``leg_named_parts_in_body_frame``."""
    return _union(*[m for _n, m in leg_named_parts_in_body_frame(leg_index)])


def make_assembly_preview() -> trimesh.Trimesh:
    """Build the full prototype hexapod in standing pose for visual
    checks.  Computes chassis_lift directly from the leg geometry so
    the foot pads always land on z = 0."""
    probe_leg = _leg_in_body_frame(0)
    z_min = float(probe_leg.bounds[0][2])
    chassis_lift = -z_min

    parts = []

    # Bottom chassis plate (at z = chassis_lift)
    bot = make_chassis_bottom()
    bot.apply_translation([0, 0, chassis_lift])
    parts.append(bot)

    # Top chassis plate (above the bottom by CHASSIS_GAP)
    top = make_chassis_top()
    top.apply_translation([0, 0, chassis_lift + CHASSIS_GAP
                                + CHASSIS_PLATE_T])
    parts.append(top)

    # LiPo pack: velcro-strapped to chassis_bottom's TOP face in the
    # inter-plate gap (the clip-in battery_holder is retired).
    # ``BATTERY_HOLDER_CENTRE_X`` is the single source of truth for the
    # pack's X offset; the velcro slots in chassis_bottom straddle it.
    lipo = _box((BATTERY_W, BATTERY_D, BATTERY_H),
                center=(BATTERY_HOLDER_CENTRE_X, 0,
                         chassis_lift + CHASSIS_PLATE_T / 2.0
                         + BATTERY_H / 2.0))
    parts.append(lipo)

    # As-built electronics stack (Aug 2026): posts + hex plate + raised
    # platform + PDB / controller / Wagos (no trays / carapace / buck).
    for _name, mesh, M0 in asbuilt_electronics_local_parts():
        m = mesh.copy()
        m.apply_transform(M0)
        m.apply_translation([0, 0, chassis_lift])
        parts.append(m)

    # Six legs
    for i in range(6):
        leg = _leg_in_body_frame(i)
        leg.apply_translation([0, 0, chassis_lift])
        parts.append(leg)

    preview = _union(*parts)

    # Z-up -> Y-up so default STL viewers show the walker upright
    preview.apply_transform(rotation_matrix(-np.pi / 2.0, [1, 0, 0]))
    return preview


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def stl_export_groups() -> "list[tuple[str, list[tuple[str, object]]]]":
    """Single source of truth for every STL ``main()`` writes to
    ``stl_prototype/``.

    Returns a list of ``(section_label, [(filename, builder), ...])`` where
    each ``builder`` is a zero-argument callable returning a
    ``trimesh.Trimesh`` (called lazily so importing this module is cheap).

    Both the exporter (``main()``) and the verifier's
    ``check_exported_stl_freshness`` walk THIS list, so the printable
    on-disk STLs can never silently drift from the parametric source: if
    you change a ``make_*`` factory but forget to re-run ``build_all.py``,
    the verifier rebuilds each part here and flags the stale file.
    """
    return [
        ("Body parts:", [
            # Jun 2026: chassis_bottom is a SINGLE merged plate (flat plate +
            # bearing tower + folded-in floor slab carrying the yaw cradles).
            # Aug 2026 as-built: LiPo velcro under chassis; electronics are
            # PDB + motor ctrl on chassis_top, magnet hex board + raised
            # platform (extra_stl/) — trays/carapace/imu_pad RETIRED.
            ("chassis_top.stl",        make_chassis_top),
            ("chassis_bottom.stl",     make_chassis_bottom),
            ("switch_holster.stl",     make_switch_holster),
            # Yaw-servo retainer saddle + short ground feet (print 6) --
            # anti-rotation + 34 mm foot lifting the wire exits off the
            # floor (Aug 2026 v2; replaced the deleted 38 mm stand).
            ("yaw_servo_retainer.stl", make_yaw_servo_retainer),
            # Yaw-bearing cap -- TOP half of the SPLIT bearing tower (print 6).
            ("yaw_bearing_cap.stl",    make_yaw_bearing_cap),
            # wago_mount.stl RETIRED late-Aug 2026: the corner power-Wago
            # tray walls are integrated into chassis_bottom's top face
            # (``_chassis_wago_tray_solid``) -- 6 fewer prints, no tape.
        ]),
        ("Leg parts (one of each -- print 6 sets):", [
            # Bearing-sandwich design: the femur is ONE printed part (hip
            # yoke + solid Phi 14 fused spar + knee bracket, Jul 2026 merge
            # #2); the tibia is a printed yoke + dia-8 CF tube + pressed-on
            # TPU foot boot.  Aug 2026 merge: the coxa is ONE printed part again
            # (yaw turntable hub + hip bracket welded; 5 head-access shafts
            # reach the disc-horn screws) -- coxa_yaw_hub.stl and
            # coxa_hip_bracket.stl are RETIRED.
            ("coxa_link.stl",          make_coxa_link),
            # NOTE femur_link.stl keeps the ASSEMBLED-link frame (origin =
            # hip disc-horn-top on the hip axis) so MuJoCo's visual mesh
            # loader keeps working -- it is the same single solid either way.
            ("femur_link.stl",         make_femur_link),
            ("tibia_knee_yoke.stl",    make_tibia_knee_yoke),
            # TPU boot pressed over the tibia tube end (Aug 2026 --
            # replaces tibia_foot_fitting + foot_pad + M3x16/nyloc hinge).
            ("foot_boot.stl",          make_foot_boot),
            # Clamshell clamp cap closing each sandwich-joint servo cradle
            # (hip + knee = 2 per leg, 12 per robot).
            ("servo_clamp_cap.stl",    make_servo_clamp_cap),
            # passive_horn_adapter.stl RETIRED (Jul 2026): the stock metal
            # passive horn centres directly on the rear idler boss.
        ]),
        # Assembled-link visual/sim mesh (NOT printed as a single part --
        # two printed fittings + a CF tube).  Emitted so MuJoCo's visual
        # mesh loader and any mesh consumers show the real sandwich geometry.
        # (femur_link moved UP to the printed list -- it IS one printed part
        # since the Jul 2026 merge #2.)
        ("Assembled-link visual/sim meshes (not printed as single parts):", [
            (stl_filename("tibia_link"), make_tibia_link),
        ]),
        # June 2026: the servo joints drive a 20 mm aluminum 25T DISC horn, not
        # the plastic 4-arm X-horn, so ``servo_horn`` IS the disc (a
        # ``disc_horn`` alias is written too).  ``make_servo_horn`` is kept for
        # backwards compat but is no longer placed in the robot.
        ("Servo visuals (not for printing -- MuJoCo / fit-check meshes):", [
            (stl_filename("servo_body"), make_servo_body),
            (stl_filename("servo_horn"), make_disc_horn),
            (stl_filename("disc_horn"),  make_disc_horn),
            (stl_filename("mpu6050"),    make_mpu6050_visual),
        ]),
        ("Electronics visuals (NOT FOR PRINTING -- BuildViz / inspector only):", [
            (stl_filename("uno_q"),                   make_uno_q_visual),
            # pdb RETIRED (Aug 2026): power distribution is Wago lever
            # nuts only -- see the "wago" mesh below.
            (stl_filename("motor_controller"),        make_motor_controller_visual),
            (stl_filename("breakout"),                make_breakout_visual),
            (stl_filename("screen"),                  make_screen_visual),
            (stl_filename("hex_post_standoff"),        make_hex_post_standoff_visual),
            (stl_filename("hex_post_thumb_nut"),      make_hex_post_thumb_nut_visual),
            (stl_filename("hex_post_magnet"),         make_hex_post_magnet_visual),
            (stl_filename("chassis_standoff"),        make_chassis_standoff_visual),
            (stl_filename("wago"),                    make_wago_visual),
            (stl_filename("wago3"),                   make_wago3_visual),
            (stl_filename("wago5"),                   make_wago5_visual),
            (stl_filename("antispark_switch_body"),   make_antispark_switch_body_visual),
            (stl_filename("antispark_switch_toggle"), make_antispark_switch_toggle_visual),
            (stl_filename("lipo_battery_body"),       make_lipo_battery_body_visual),
            (stl_filename("lipo_xt60"),               make_lipo_xt60_visual),
            # buck_converter RETIRED from as-built (no buck); mesh kept if
            # someone re-exports via make_buck_converter_visual manually.
        ]),
        ("Assembly preview (everything in standing pose):", [
            (stl_filename("assembly_preview"), make_assembly_preview),
        ]),
    ]


def main() -> None:
    print("Hexapod walker PROTOTYPE -- generating STLs in stl_prototype/ ...")

    parts: list[tuple[str, trimesh.Trimesh]] = []
    for section_label, builders in stl_export_groups():
        print(section_label)
        for name, build in builders:
            mesh = build()
            _save(mesh, name)
            parts.append((name, mesh))

    # ----- Final summary -----
    preview = dict(parts)[stl_filename("assembly_preview")]
    total_faces = sum(len(m.faces) for _, m in parts)
    foot_to_foot    = preview.extents[0]
    standing_height = preview.extents[1]
    print()
    print(f"OK -- {len(parts)} STL files written.")
    print(f"   Vehicle envelope (foot to foot):  {foot_to_foot/10:.1f} cm")
    print(f"   Vehicle standing height:          {standing_height/10:.1f} cm")
    print(f"   Total geometry triangle count:    {total_faces:,}")
    print()
    print("Estimated parts cost:  ~$150 - $250 in 2026 USD.")
    print("See PROTOTYPE.md for the BOM and wiring guide.")


if __name__ == "__main__":
    main()
