#!/usr/bin/env python3
"""Full rigid-bearing hexapod concept using the user's premade 56 mm C horns.

The production and rigid-hip CAD remain untouched.  This sidecar inherits the
rigid top/bottom 6805 stack and the outboard hip layout from
``cnc_chorn_overhead``.  It replaces that concept's custom CNC clamp with the
measured commodity bracket, adds two different one-piece 7 mm spacer pucks,
and adapts the bracket's six front M3 holes to the femur and tibia bodies.

Run from the repository root:

    uv run --no-project --python 3.12 --with trimesh --with numpy \
      --with manifold3d python \
      hexapod_walker/prototype_sts3215/concepts/premade_chorn_56/\
make_premade_chorn_56_variant.py
"""
from __future__ import annotations

import json
import io
import math
import os
import subprocess
import sys
import tomllib
from pathlib import Path

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix


HERE = Path(__file__).resolve().parent
PROTO_DIR = HERE.parents[1]
RIGID_DIR = PROTO_DIR / "concepts" / "rigid_hip"
OVERHEAD_DIR = PROTO_DIR / "concepts" / "cnc_chorn_overhead"
STL_DIR = HERE / "stl"

for path in (PROTO_DIR, RIGID_DIR, OVERHEAD_DIR):
    sys.path.insert(0, str(path))

import hexapod_prototype as hp  # noqa: E402
import make_cnc_chorn_variant as base  # noqa: E402
import make_rigid_hip_variant as rv  # noqa: E402


BUILD_ID = "prototype_sts3215/premade-chorn-56"
CONFIG_PATH = HERE / "hardware_config.toml"
with CONFIG_PATH.open("rb") as config_file:
    CONFIG = tomllib.load(config_file)

# This concept does not use production's dedicated 14 x 22 mm battery-trunk
# pass-through at chassis XY (48, 0). Close it across the complete lower-sheet
# stack while retaining all six Velcro strap slots, including the centre pair
# that can also carry the battery leads. The small XY overlap makes the patch
# a robust volumetric union with the existing hole perimeter.
CHASSIS_TRUNK_PATCH_OVERLAP = 0.40
CHASSIS_BOTTOM_Z0 = hp.CHASSIS_SPLIT_Z - hp.CHASSIS_BOTTOM_FLOOR_T
CHASSIS_BOTTOM_Z1 = hp.CHASSIS_PLATE_T / 2.0

# ---------------------------------------------------------------------------
# Purchased hardware.  The first four values are user measurements; the
# 25 mm blade/web width is the matching commodity long-U nominal and remains
# the one dimension that must be checked with calipers before printing 12 sets.
# ---------------------------------------------------------------------------
CHORN_OUTSIDE_SPAN = float(CONFIG["bracket"]["outside_span_mm"])
CHORN_PLATE_T = float(CONFIG["bracket"]["plate_thickness_mm"])
CHORN_AXIS_TO_FRONT_OUTER = float(
    CONFIG["bracket"]["axis_to_front_outer_mm"]
)
CHORN_BLADE_WIDTH = float(CONFIG["bracket"]["blade_and_front_width_mm"])
CHORN_CENTER_HOLE_D = float(CONFIG["bracket"]["center_hole_diameter_mm"])

AXIS_X = hp.SERVO_OUTPUT_X
FRONT_X1 = AXIS_X + CHORN_AXIS_TO_FRONT_OUTER
FRONT_X0 = FRONT_X1 - CHORN_PLATE_T
BRACKET_MID_Z = hp.JOINT_SOCKET_Z
BRACKET_Z0 = BRACKET_MID_Z - CHORN_OUTSIDE_SPAN / 2.0
BRACKET_Z1 = BRACKET_MID_Z + CHORN_OUTSIDE_SPAN / 2.0
BOT_PLATE_Z0 = BRACKET_Z0
BOT_PLATE_Z1 = BRACKET_Z0 + CHORN_PLATE_T
TOP_PLATE_Z0 = BRACKET_Z1 - CHORN_PLATE_T
TOP_PLATE_Z1 = BRACKET_Z1
CHORN_INNER_SPAN = CHORN_OUTSIDE_SPAN - 2.0 * CHORN_PLATE_T

# The user's two 7 mm pucks close almost exactly onto the real 38.04 mm
# horn-face span.  With the nominal bracket dimensions they spread the C by
# 0.24 mm total (0.12 mm per blade), which is intentional bench-fit preload.
SPACER_T = float(CONFIG["spacers"]["thickness_mm"])
SPACER_OD = float(CONFIG["spacers"]["outside_diameter_mm"])
SPACER_M3_D = float(CONFIG["spacers"]["m3_clearance_diameter_mm"])
DRIVE_HEAD_POCKET_D = float(
    CONFIG["spacers"]["driven_head_pocket_diameter_mm"]
)
DRIVE_HEAD_POCKET_DEPTH = float(
    CONFIG["spacers"]["driven_head_pocket_depth_mm"]
)
DRIVE_TOOL_ACCESS_D = float(
    CONFIG["spacers"]["driven_tool_access_diameter_mm"]
)
PASSIVE_CENTER_THROUGH_D = float(
    CONFIG["spacers"]["passive_center_through_diameter_mm"]
)
ASSEMBLED_INNER_SPAN = base.DISC_SPAN + 2.0 * SPACER_T
BRACKET_SPREAD = ASSEMBLED_INNER_SPAN - CHORN_INNER_SPAN

# Removable hip/knee servo clamp caps. The hip cap retains both original side
# screws; the overhead knee cap intentionally deleted its inboard side screw
# where the bought bracket replaces that wall, leaving one side screw plus its
# separate vertical retention bolt. This concept replaces the 18 remaining
# Phi2.5 self-tapped side pilots (2 hip + 1 knee per leg) with face-loaded
# captive M3 nuts and full-clearance screw-tip tunnels.
CLAMP_CAP = CONFIG["servo_clamp_caps"]
CLAMP_CAP_SCREW_LENGTH = float(CLAMP_CAP["screw_length_mm"])
CLAMP_CAP_M3_D = float(CLAMP_CAP["m3_clearance_diameter_mm"])
CLAMP_CAP_NUT_AF = float(CLAMP_CAP["nut_pocket_across_flats_mm"])
CLAMP_CAP_NUT_DEPTH = float(CLAMP_CAP["nut_pocket_depth_mm"])
CLAMP_CAP_NUT_T = float(CLAMP_CAP["nut_nominal_thickness_mm"])
CLAMP_CAP_TIP_CLEARANCE = float(CLAMP_CAP["tip_clearance_mm"])
CLAMP_CAP_MIN_THREAD_BEYOND_NUT = float(
    CLAMP_CAP["minimum_thread_beyond_nut_mm"]
)
CLAMP_CAP_WALL_FACE_Y = hp.WELL_D / 2.0
CLAMP_CAP_HEAD_SEAT_Y = (
    CLAMP_CAP_WALL_FACE_Y + hp.CLAMP_CAP_T - hp.CLAMP_HEAD_CB_DEPTH
)
CLAMP_CAP_SCREW_TIP_Y = CLAMP_CAP_HEAD_SEAT_Y - CLAMP_CAP_SCREW_LENGTH
CLAMP_CAP_NUT_INNER_Y = CLAMP_CAP_WALL_FACE_Y - CLAMP_CAP_NUT_DEPTH
CLAMP_CAP_THREAD_BEYOND_NUT = (
    CLAMP_CAP_NUT_INNER_Y - CLAMP_CAP_SCREW_TIP_Y
)
CLAMP_CAP_LATERAL_SHELL = (
    hp.WELL_WALL_X - CLAMP_CAP_NUT_AF
) / 2.0
assert CLAMP_CAP_NUT_T <= CLAMP_CAP_NUT_DEPTH
assert CLAMP_CAP_THREAD_BEYOND_NUT >= CLAMP_CAP_MIN_THREAD_BEYOND_NUT
assert CLAMP_CAP_LATERAL_SHELL >= 1.5

# Straight rear tower for the long outboard coxa/cap pair. Its footprint is a
# crescent copied from the lower yaw hub: the outer contour is identical and
# nothing extends beyond the existing coxa plan envelope.
TOWER = CONFIG["coxa_support_tower"]
TOWER_OUTER_R = float(TOWER["outer_radius_mm"])
TOWER_FRONT_X = float(TOWER["front_chord_x_mm"])
TOWER_BOTTOM_Z = float(TOWER["tower_bottom_z_mm"])
TOWER_TOP_Z = float(TOWER["tower_top_z_mm"])
TOWER_CAP_FLANGE_Z0 = float(TOWER["cap_flange_bottom_z_mm"])
TOWER_CAP_FLANGE_Z1 = float(TOWER["cap_flange_top_z_mm"])
TOWER_SCREW_X = float(TOWER["top_screw_x_mm"])
TOWER_SCREW_YS = [float(value) for value in TOWER["top_screw_ys_mm"]]
TOWER_YAW_DRIVER_EXTRA_R = float(
    TOWER["yaw_driver_extra_radial_clearance_mm"]
)
TOWER_M3_D = float(TOWER["m3_clearance_diameter_mm"])
TOWER_HEAD_D = float(TOWER["shcs_head_pocket_diameter_mm"])
TOWER_HEAD_DEPTH = float(TOWER["shcs_head_pocket_depth_mm"])
TOWER_NUT_AF = float(TOWER["nut_pocket_across_flats_mm"])
TOWER_NUT_DEPTH = float(TOWER["nut_pocket_depth_mm"])
TOWER_TIP_CLEARANCE_DEPTH = float(TOWER["screw_tip_clearance_depth_mm"])

COXA_REINF = CONFIG["coxa_reinforcement"]
COXA_REINF_X0 = float(COXA_REINF["arm_x0_mm"])
COXA_REINF_X1 = float(COXA_REINF["arm_x1_mm"])
COXA_REINF_CURVE_X1 = float(COXA_REINF["curve_x1_mm"])
COXA_REINF_LO_CURVE_R0 = float(
    COXA_REINF["lower_curve_inner_radius_mm"]
)
COXA_REINF_LO_CURVE_R1 = float(
    COXA_REINF["lower_curve_outer_radius_mm"]
)
COXA_REINF_LO_STRAIGHT_Y = float(
    COXA_REINF["lower_straight_center_y_mm"]
)
COXA_REINF_HI_CURVE_R0 = float(
    COXA_REINF["upper_curve_inner_radius_mm"]
)
COXA_REINF_HI_CURVE_R1 = float(
    COXA_REINF["upper_curve_outer_radius_mm"]
)
COXA_REINF_HI_STRAIGHT_Y = float(
    COXA_REINF["upper_straight_center_y_mm"]
)
COXA_REINF_CENTRE_RELIEF_R = float(
    COXA_REINF["centre_relief_radius_mm"]
)
COXA_REINF_ARM_W = float(COXA_REINF["straight_width_mm"])
COXA_REINF_LO_Z0 = float(COXA_REINF["lower_bridge_z0_mm"])
COXA_REINF_LO_Z1 = float(COXA_REINF["lower_bridge_z1_mm"])
COXA_REINF_HI_Z0 = float(COXA_REINF["upper_bridge_z0_mm"])
COXA_REINF_HI_Z1 = float(COXA_REINF["upper_bridge_z1_mm"])
COXA_REINF_ACCESS_X = float(COXA_REINF["upper_screw_access_x_mm"])
COXA_REINF_ACCESS_Z = float(COXA_REINF["upper_screw_access_z_mm"])
COXA_REINF_ACCESS_D = float(
    COXA_REINF["upper_screw_access_diameter_mm"]
)
COXA_BEARING_BUTTRESS_Z0 = float(
    COXA_REINF["upper_bearing_buttress_z0_mm"]
)
COXA_BEARING_BUTTRESS_Z1 = float(
    COXA_REINF["upper_bearing_buttress_z1_mm"]
)
COXA_BEARING_BUTTRESS_R0 = float(
    COXA_REINF["upper_bearing_buttress_inner_radius_mm"]
)
COXA_BEARING_BUTTRESS_R1 = float(
    COXA_REINF["upper_bearing_buttress_outer_radius_mm"]
)
COXA_BEARING_BUTTRESS_X1 = float(
    COXA_REINF["upper_bearing_buttress_front_x_mm"]
)
assert COXA_BEARING_BUTTRESS_Z0 < COXA_REINF_HI_Z1
assert COXA_BEARING_BUTTRESS_Z1 < TOWER_TOP_Z
assert COXA_BEARING_BUTTRESS_R0 < TOWER_OUTER_R
COXA_REINF_TAB_BLOCK_X0 = float(COXA_REINF["positive_tab_block_x0_mm"])
COXA_REINF_TAB_BLOCK_X1 = float(COXA_REINF["positive_tab_block_x1_mm"])
COXA_REINF_TAB_BLOCK_Y0 = float(COXA_REINF["positive_tab_block_y0_mm"])
COXA_REINF_TAB_BLOCK_Y1 = float(COXA_REINF["positive_tab_block_y1_mm"])
COXA_REINF_TAB_BLOCK_Z0 = float(COXA_REINF["positive_tab_block_z0_mm"])
COXA_REINF_TAB_BLOCK_Z1 = float(COXA_REINF["positive_tab_block_z1_mm"])
COXA_REINF_TAB_HOLE_X = float(COXA_REINF["positive_tab_hole_x_mm"])
COXA_REINF_TAB_HOLE_ZS = [
    float(value) for value in COXA_REINF["positive_tab_hole_zs_mm"]
]
COXA_REINF_TAB_DRIVER_D = float(
    COXA_REINF["positive_tab_driver_diameter_mm"]
)

COXA_HOLDER = CONFIG["coxa_servo_holder_centering"]
COXA_HOLDER_SERVO_CENTRE_Y = float(COXA_HOLDER["servo_center_y_mm"])
COXA_HOLDER_FOOT_Y0 = float(
    COXA_HOLDER["existing_foot_negative_y_mm"]
)
COXA_HOLDER_FOOT_OLD_Y1 = float(
    COXA_HOLDER["existing_foot_positive_y_mm"]
)
COXA_HOLDER_POSITIVE_EDGE_TRIM = float(
    COXA_HOLDER["positive_edge_centering_trim_mm"]
)
COXA_HOLDER_FOOT_Y1 = (
    2.0 * COXA_HOLDER_SERVO_CENTRE_Y - COXA_HOLDER_FOOT_Y0
    - COXA_HOLDER_POSITIVE_EDGE_TRIM
)
COXA_HOLDER_FOOT_X0, COXA_HOLDER_FOOT_X1 = (
    float(value) for value in COXA_HOLDER["foot_x_range_mm"]
)
COXA_HOLDER_FOOT_Z0, COXA_HOLDER_FOOT_Z1 = (
    float(value) for value in COXA_HOLDER["foot_z_range_mm"]
)

BEARING_CARRIER = CONFIG["hip_bearing_carrier"]
BEARING_CARRIER_R = float(BEARING_CARRIER["mount_radius_mm"])
BEARING_CARRIER_CAP_PAD_Y0 = float(
    BEARING_CARRIER["cap_pad_inner_face_y_mm"]
)
BEARING_CARRIER_SCREW_R = float(
    BEARING_CARRIER["mount_screw_radius_mm"]
)
BEARING_CARRIER_SCREW_ANGLES = [
    math.radians(float(value))
    for value in BEARING_CARRIER["mount_screw_angles_deg"]
]
BEARING_CARRIER_M3_D = float(
    BEARING_CARRIER["m3_clearance_diameter_mm"]
)
BEARING_CARRIER_CSK_D = float(
    BEARING_CARRIER["countersink_major_diameter_mm"]
)
BEARING_CARRIER_CSK_DEPTH = float(
    BEARING_CARRIER["countersink_depth_mm"]
)
BEARING_CARRIER_NUT_AF = float(
    BEARING_CARRIER["nut_pocket_across_flats_mm"]
)
BEARING_CARRIER_NUT_DEPTH = float(
    BEARING_CARRIER["nut_pocket_depth_mm"]
)
BEARING_CARRIER_TIP_CLEARANCE_DEPTH = float(
    BEARING_CARRIER["screw_tip_clearance_depth_mm"]
)
BEARING_CARRIER_TOP_GAP = float(
    BEARING_CARRIER["flange_top_gap_below_bearing_mm"]
)
BEARING_CARRIER_CENTER_X = -base.COXA_EXT
BEARING_CARRIER_CENTER_Z = rv.AXIS_Z
BEARING_CARRIER_BASE_Y = rv.CAP_FACE_Y
BEARING_CARRIER_FLANGE_TOP_Y = rv.PED_Y1 - BEARING_CARRIER_TOP_GAP
assert BEARING_CARRIER_FLANGE_TOP_Y > (
    BEARING_CARRIER_BASE_Y + BEARING_CARRIER_TIP_CLEARANCE_DEPTH + 0.3
)

COXA_HUB_SPLIT = CONFIG["coxa_yaw_hub_split"]
COXA_HUB_INTERFACE_Z = float(COXA_HUB_SPLIT["interface_z_mm"])
COXA_HUB_COLLAR_R0 = float(
    COXA_HUB_SPLIT["carrier_collar_inner_radius_mm"]
)
COXA_HUB_COLLAR_R1 = float(
    COXA_HUB_SPLIT["carrier_collar_outer_radius_mm"]
)
COXA_HUB_SOCKET_R0 = float(
    COXA_HUB_SPLIT["coxa_socket_inner_radius_mm"]
)
COXA_HUB_SOCKET_R1 = float(
    COXA_HUB_SPLIT["coxa_socket_outer_radius_mm"]
)
COXA_HUB_COLLAR_Z1 = float(COXA_HUB_SPLIT["collar_top_z_mm"])
COXA_HUB_RADIAL_CLEARANCE = float(
    COXA_HUB_SPLIT["radial_clearance_mm"]
)
COXA_HUB_SCREW_Z = float(COXA_HUB_SPLIT["radial_screw_axis_z_mm"])
COXA_HUB_SCREW_ANGLES = [
    math.radians(float(value))
    for value in COXA_HUB_SPLIT["radial_screw_angles_deg"]
]
COXA_HUB_M3_D = float(COXA_HUB_SPLIT["m3_clearance_diameter_mm"])
COXA_HUB_HEAD_D = float(COXA_HUB_SPLIT["head_access_diameter_mm"])
COXA_HUB_HEAD_SEAT_R = float(COXA_HUB_SPLIT["head_seat_radius_mm"])
COXA_HUB_HEAD_ACCESS_R1 = float(
    COXA_HUB_SPLIT["head_access_outer_radius_mm"]
)
COXA_HUB_NUT_AF = float(
    COXA_HUB_SPLIT["nut_pocket_across_flats_mm"]
)
COXA_HUB_NUT_DEPTH = float(COXA_HUB_SPLIT["nut_pocket_depth_mm"])
COXA_HUB_SCREW_LENGTH = float(COXA_HUB_SPLIT["screw_length_mm"])
assert abs(COXA_HUB_INTERFACE_Z - rv.SLAB_BOT_Z) < 1e-9
assert abs(
    COXA_HUB_SOCKET_R0 - COXA_HUB_COLLAR_R1 - COXA_HUB_RADIAL_CLEARANCE
) < 1e-9
assert abs(COXA_REINF_CENTRE_RELIEF_R - COXA_HUB_SOCKET_R0) < 1e-9, (
    "coxa centre relief must stop at the yaw socket instead of cutting "
    "a D-shaped opening into the servo-holder foot"
)

# The hip cap STL is authored in cap-local coordinates.  Derive the rigid
# cap-to-coxa relationship from the ancestor so this sidecar can add the
# rear insert bores in the common coxa frame without copying a datum.
_T_REF = base.leg_transforms(0)
HIP_CAP_TO_COXA = np.linalg.inv(_T_REF["coxa"]) @ _T_REF["hip_cap"]
COXA_TO_HIP_CAP = np.linalg.inv(HIP_CAP_TO_COXA)
KNEE_CAP_TO_FEMUR = np.linalg.inv(base.MH) @ base.M_KNEE_JP

# The inboard hip-cap clamp screw is serviced through the fixed chassis plate.
# Transform that existing access-hole centre into coxa-local coordinates so
# the crescent-envelope check explicitly preserves its vertical driver path.
_CAP_DRIVER_WORLD_XY = rv._access_hole_xy()[0]
_CAP_DRIVER_LOCAL = np.linalg.inv(_T_REF["coxa"]) @ np.array([
    _CAP_DRIVER_WORLD_XY[0], _CAP_DRIVER_WORLD_XY[1], 0.0, 1.0
])
TOWER_CAP_DRIVER_X = float(_CAP_DRIVER_LOCAL[0])
TOWER_CAP_DRIVER_Y = float(_CAP_DRIVER_LOCAL[1])

# Front web: the ordinary 4x M3 / Phi8 pattern is centred on the bracket.
# For the extra pair, the physical bracket measures 34 mm inner-edge to
# inner-edge and 40 mm outer-edge to outer-edge. Those two source measurements
# unambiguously give Ø3.0 holes on 37.0 mm centres, 9.5 mm in from the 56 mm
# bracket edges. Printed mating parts retain Ø3.4 M3 clearance holes.
FRONT_M3_D = float(CONFIG["front"]["m3_clearance_diameter_mm"])
FRONT_PATTERN_PCD = float(CONFIG["front"]["usual_pattern_pcd_mm"])
EXTRA_INNER_EDGE_GAP = float(
    CONFIG["front"]["extra_pair_inner_edge_gap_mm"]
)
EXTRA_OUTER_EDGE_SPAN = float(
    CONFIG["front"]["extra_pair_outer_edge_span_mm"]
)
EXTRA_PHYSICAL_HOLE_D = (
    EXTRA_OUTER_EDGE_SPAN - EXTRA_INNER_EDGE_GAP
) / 2.0
EXTRA_CENTRE_SPAN = (
    EXTRA_OUTER_EDGE_SPAN + EXTRA_INNER_EDGE_GAP
) / 2.0
EXTRA_EDGE_INSET = (CHORN_OUTSIDE_SPAN - EXTRA_CENTRE_SPAN) / 2.0
assert EXTRA_PHYSICAL_HOLE_D > 0.0
assert EXTRA_CENTRE_SPAN < CHORN_OUTSIDE_SPAN

# Printed front receiver, common geometry at both hip/femur and knee/tibia.
# Six through-bolts terminate in accessible captive M3 nyloc pockets.
RECEIVER_T = 8.0
RECEIVER_X0 = FRONT_X1
RECEIVER_X1 = RECEIVER_X0 + RECEIVER_T
NYLOC_AF = 5.7
NYLOC_POCKET_DEPTH = 4.0
RECEIVER_CENTER_D = 8.6

# The inherited knee holder's -X wall ends exactly at the servo cavity.  The
# old common 8 mm receiver ran 4.4 mm past this plane and into the servo body.
# Stop the femur receiver at the real cavity wall and use ordinary front-loaded
# M3 hex nuts captured by the bracket. The tibia keeps the 8 mm through-bolt
# receiver and captive nylocs because there is no servo behind that part.
FEMUR_CAVITY_W = (
    hp.SERVO_BODY_W
    + 2.0 * hp.WELL_BODY_CL
    - hp.WELL_INSIDE_X_TIGHTEN
)
FEMUR_RECEIVER_X1 = hp.FEMUR_LENGTH - FEMUR_CAVITY_W / 2.0
FEMUR_RECEIVER_T = FEMUR_RECEIVER_X1 - RECEIVER_X0
FEMUR_CENTER_D = float(
    CONFIG["front"]["femur_center_clearance_diameter_mm"]
)
FEMUR_NUT_AF = float(
    CONFIG["front"]["femur_nut_pocket_across_flats_mm"]
)
FEMUR_NUT_DEPTH = float(CONFIG["front"]["femur_nut_pocket_depth_mm"])
FEMUR_NUT_T = float(CONFIG["front"]["femur_nut_nominal_thickness_mm"])
FEMUR_FRONT_SCREW_LENGTH = float(
    CONFIG["front"]["femur_screw_length_mm"]
)
FEMUR_SCREW_TIP_CLEARANCE = float(
    CONFIG["front"]["femur_screw_tip_clearance_mm"]
)
FEMUR_MIN_THREAD_BEYOND_NUT = float(
    CONFIG["front"]["femur_minimum_thread_beyond_nut_mm"]
)
FEMUR_SCREW_PROJECTION = FEMUR_FRONT_SCREW_LENGTH - CHORN_PLATE_T
FEMUR_THREAD_BEYOND_NUT = FEMUR_SCREW_PROJECTION - FEMUR_NUT_T
FEMUR_TIP_BORE_DEPTH = (
    FEMUR_SCREW_PROJECTION + FEMUR_SCREW_TIP_CLEARANCE
)
FEMUR_POCKET_BACK_SKIN = FEMUR_RECEIVER_T - FEMUR_NUT_DEPTH
FEMUR_SERVO_SIDE_SKIN = FEMUR_RECEIVER_T - FEMUR_TIP_BORE_DEPTH
FEMUR_CENTER_TO_NUT_FLAT_WEB = (
    FRONT_PATTERN_PCD / 2.0 - FEMUR_CENTER_D / 2.0 - FEMUR_NUT_AF / 2.0
)
assert FEMUR_NUT_T <= FEMUR_NUT_DEPTH
assert FEMUR_THREAD_BEYOND_NUT >= FEMUR_MIN_THREAD_BEYOND_NUT
assert FEMUR_POCKET_BACK_SKIN >= 0.8
assert FEMUR_SERVO_SIDE_SKIN >= 0.4
assert FEMUR_CENTER_TO_NUT_FLAT_WEB >= 0.09

# The tube boss sits directly behind the normal four-hole pattern.  Radial
# U-windows expose those four captive nyloc pockets from the sides while the
# extra top/bottom pair remain naturally outside the boss.  Stop the windows
# shortly behind the 8 mm receiver so the mouth retains a complete circular
# tube-support collar.
TIB_SOCKET = CONFIG["tibia_socket"]
TIB_TUBE_ENGAGEMENT = float(TIB_SOCKET["tube_engagement_depth_mm"])
TIB_ACCESS = TIB_SOCKET
TIB_ACCESS_W = float(TIB_ACCESS["window_width_mm"])
TIB_ACCESS_X0 = RECEIVER_X1 - NYLOC_POCKET_DEPTH - 0.20
TIB_ACCESS_X1 = RECEIVER_X1 + float(
    TIB_ACCESS["window_beyond_receiver_mm"]
)
TIB_SOCKET_OUTER_R = hp.LEG_TUBE_OD / 2.0 + hp.LEG_TUBE_SOCKET_WALL
TIB_SOCKET_BORE_R = hp.LEG_TUBE_OD / 2.0 + hp.LEG_TUBE_SOCKET_CLEAR
TIB_ROOT_FLARE_R = float(TIB_SOCKET["root_flare_outer_radius_mm"])
TIB_ROOT_FLARE_LEN = float(TIB_SOCKET["root_flare_length_mm"])
TIB_ROOT_FLARE_OVERLAP = float(
    TIB_SOCKET["root_flare_receiver_overlap_mm"]
)
TIB_ROOT_RIB_R1 = float(TIB_SOCKET["root_rib_outer_radius_mm"])
TIB_ROOT_RIB_R0 = float(TIB_SOCKET["root_rib_inner_radius_mm"])
TIB_ROOT_RIB_W = float(TIB_SOCKET["root_rib_width_mm"])
TIB_ROOT_RIB_LEN = float(TIB_SOCKET["root_rib_length_mm"])
TIB_ROOT_RIB_ANGLES = [
    math.radians(float(value))
    for value in TIB_SOCKET["root_rib_angles_deg"]
]
TIB_ACCESS_RADIAL_OUTER = max(
    TIB_SOCKET_OUTER_R,
    TIB_ROOT_FLARE_R,
    TIB_ROOT_RIB_R1,
) + 1.0
assert TIB_ROOT_FLARE_R > TIB_SOCKET_OUTER_R
assert TIB_ROOT_RIB_R1 > TIB_ROOT_RIB_R0
assert TIB_ROOT_RIB_R0 < TIB_SOCKET_OUTER_R

# Moving the receiver from the custom clamp's x=58.3 datum to the bought
# bracket's x=63.5 face moves the blind tube stop by 5.2 mm.  The 30 mm socket
# then grows outward along the existing tube, shortening its unsupported span
# without moving the foot or requiring any additional shortening of the tube.
TIB_BORE_X0 = RECEIVER_X0 + 0.30
TIB_MOUTH_X = TIB_BORE_X0 + TIB_TUBE_ENGAGEMENT
TUBE_SHORTER_THAN_OVERHEAD = TIB_BORE_X0 - base.TIB_BORE_X0
TUBE_SHORTER_THAN_PRODUCTION = TIB_BORE_X0 - hp._YOKE_SOCKET_X
UNSUPPORTED_TUBE_REDUCTION = (
    TIB_TUBE_ENGAGEMENT - hp.LEG_TUBE_SOCKET_DEPTH
)
OVERHEAD_MOUTH_X = base.TIB_BORE_X0 + hp.LEG_TUBE_SOCKET_DEPTH
MOUTH_OUTBOARD_THAN_OVERHEAD = TIB_MOUTH_X - OVERHEAD_MOUTH_X

RHO_PETG = base.RHO_PETG
RHO_ALU = base.RHO_ALU


def _box(extents, center) -> trimesh.Trimesh:
    mesh = trimesh.creation.box(extents=np.asarray(extents, float))
    mesh.apply_translation(np.asarray(center, float))
    return mesh


def _cyl_z(radius: float, z0: float, z1: float,
           x: float = 0.0, y: float = 0.0, sections: int = 96) -> trimesh.Trimesh:
    mesh = trimesh.creation.cylinder(radius=radius, height=z1 - z0,
                                     sections=sections)
    mesh.apply_translation([x, y, (z0 + z1) / 2.0])
    return mesh


def _cyl_x(radius: float, x0: float, x1: float,
           y: float = 0.0, z: float = 0.0, sections: int = 96) -> trimesh.Trimesh:
    mesh = trimesh.creation.cylinder(radius=radius, height=x1 - x0,
                                     sections=sections)
    mesh.apply_transform(rotation_matrix(math.pi / 2.0, [0, 1, 0]))
    mesh.apply_translation([(x0 + x1) / 2.0, y, z])
    return mesh


def _cyl_y(radius: float, y0: float, y1: float,
           x: float = 0.0, z: float = 0.0, sections: int = 96) -> trimesh.Trimesh:
    mesh = trimesh.creation.cylinder(radius=radius, height=y1 - y0,
                                     sections=sections)
    mesh.apply_transform(rotation_matrix(math.pi / 2.0, [1, 0, 0]))
    mesh.apply_translation([x, (y0 + y1) / 2.0, z])
    return mesh


def _cone_y(radius: float, y0: float, y1: float,
            x: float = 0.0, z: float = 0.0, sections: int = 96) -> trimesh.Trimesh:
    """Cone opening at y0 and narrowing toward y1 (for a 90-deg CSK)."""
    mesh = trimesh.creation.cone(radius=radius, height=y1 - y0,
                                 sections=sections)
    mesh.apply_transform(rotation_matrix(-math.pi / 2.0, [1, 0, 0]))
    mesh.apply_translation([x, y0, z])
    return mesh


def _frustum_x(
    radius0: float,
    radius1: float,
    x0: float,
    x1: float,
    *,
    y: float = 0.0,
    z: float = 0.0,
    sections: int = 96,
) -> trimesh.Trimesh:
    """Solid circular frustum along X, with ``radius0`` at ``x0``."""
    length = x1 - x0
    assert length > 0.0 and radius0 > 0.0 and radius1 > 0.0
    profile = np.array([
        [0.0, 0.0],
        [radius0, 0.0],
        [radius1, length],
        [0.0, length],
        [0.0, 0.0],
    ])
    mesh = trimesh.creation.revolve(profile, sections=sections)
    mesh.apply_transform(rotation_matrix(math.pi / 2.0, [0, 1, 0]))
    mesh.apply_translation([x0, y, z])
    return mesh


def _hex_x(
    across_flats: float,
    x0: float,
    x1: float,
    y: float,
    z: float,
    *,
    flat_normal_angle_rad: float = 0.0,
) -> trimesh.Trimesh:
    """Hex prism on X with one flat normal in the requested YZ direction."""
    mesh = trimesh.creation.cylinder(
        radius=across_flats / math.sqrt(3.0), height=x1 - x0, sections=6
    )
    mesh.apply_transform(rotation_matrix(math.pi / 2.0, [0, 1, 0]))
    mesh.apply_transform(rotation_matrix(flat_normal_angle_rad, [1, 0, 0]))
    mesh.apply_translation([(x0 + x1) / 2.0, y, z])
    return mesh


def _hex_y(
    across_flats: float,
    y0: float,
    y1: float,
    x: float,
    z: float,
    *,
    flat_normal_angle_rad: float = 0.0,
) -> trimesh.Trimesh:
    """Hex prism on Y with one flat normal in the requested XZ direction."""
    mesh = trimesh.creation.cylinder(
        radius=across_flats / math.sqrt(3.0), height=y1 - y0, sections=6
    )
    mesh.apply_transform(rotation_matrix(math.pi / 2.0, [1, 0, 0]))
    mesh.apply_transform(rotation_matrix(
        math.pi / 6.0 - flat_normal_angle_rad, [0, 1, 0]
    ))
    mesh.apply_translation([x, (y0 + y1) / 2.0, z])
    return mesh


def _hex_z(
    across_flats: float,
    z0: float,
    z1: float,
    x: float,
    y: float,
    *,
    flat_normal_angle_rad: float = 0.0,
) -> trimesh.Trimesh:
    """Hex prism on Z with one flat normal in the requested XY direction."""
    mesh = trimesh.creation.cylinder(
        radius=across_flats / math.sqrt(3.0), height=z1 - z0, sections=6
    )
    mesh.apply_transform(rotation_matrix(
        flat_normal_angle_rad - math.pi / 6.0, [0, 0, 1]
    ))
    mesh.apply_translation([x, y, (z0 + z1) / 2.0])
    return mesh


def _clamp_cap_captive_nut_cuts(
    well_to_part: np.ndarray,
    bolt_centres: list[tuple[float, float]] | None = None,
) -> list[trimesh.Trimesh]:
    """Face-loaded M3 nut traps for one printed servo cradle.

    The pocket opens at the existing +Y clamp mating face. A standard M3 nut
    drops into the 5.8 mm-AF recess, the removable cap closes that recess, and
    its existing M3x8 screw enters along -Y. The clearance tunnel continues
    past the nut to the exact screw-tip plane, so the screw cannot bottom in
    the former Phi2.5 self-tap pilot before its threads pass through the nut.

    Hex flats are normal to local X. That keeps the 5.8 mm across-flats width
    inside the cradle's 9 mm wall, leaving 1.6 mm of printed shell on both the
    cavity and outer sides instead of putting the wider hex corners there.
    """
    cuts: list[trimesh.Trimesh] = []
    if bolt_centres is None:
        bolt_centres = hp.servo_clamp_bolt_centres()
    for bx, bz in bolt_centres:
        shaft = _cyl_y(
            CLAMP_CAP_M3_D / 2.0,
            CLAMP_CAP_SCREW_TIP_Y - CLAMP_CAP_TIP_CLEARANCE,
            CLAMP_CAP_WALL_FACE_Y + 0.30,
            x=bx,
            z=bz,
        )
        pocket = _hex_y(
            CLAMP_CAP_NUT_AF,
            CLAMP_CAP_NUT_INNER_Y,
            CLAMP_CAP_WALL_FACE_Y + 0.30,
            x=bx,
            z=bz,
            flat_normal_angle_rad=0.0,
        )
        shaft.apply_transform(well_to_part)
        pocket.apply_transform(well_to_part)
        cuts.extend((shaft, pocket))
    return cuts


def _clamp_cap_screw_recuts() -> list[trimesh.Trimesh]:
    """Restore the two original cap bores after adding adjacent solids."""
    cuts: list[trimesh.Trimesh] = []
    cap_outer_y = CLAMP_CAP_WALL_FACE_Y + hp.CLAMP_CAP_T
    for bx, bz in hp.servo_clamp_bolt_centres():
        cuts.append(_cyl_y(
            CLAMP_CAP_M3_D / 2.0,
            CLAMP_CAP_WALL_FACE_Y - 0.30,
            cap_outer_y + 0.30,
            x=bx,
            z=bz,
        ))
        cuts.append(_cyl_y(
            hp.CLAMP_HEAD_CB_OD / 2.0,
            CLAMP_CAP_HEAD_SEAT_Y,
            cap_outer_y + 0.30,
            x=bx,
            z=bz,
        ))
    return cuts


def _cyl_radial_xy(
    radius: float,
    radial0: float,
    radial1: float,
    angle_rad: float,
    *,
    z: float,
) -> trimesh.Trimesh:
    """Cylinder on an XY radial axis at a fixed Z height."""
    mesh = _cyl_x(radius, radial0, radial1, z=z)
    mesh.apply_transform(rotation_matrix(angle_rad, [0, 0, 1]))
    return mesh


def _hex_radial_xy(
    across_flats: float,
    radial0: float,
    radial1: float,
    angle_rad: float,
    *,
    z: float,
) -> trimesh.Trimesh:
    """Radial hex prism with a horizontal flat at its top and bottom."""
    mesh = trimesh.creation.cylinder(
        radius=across_flats / math.sqrt(3.0),
        height=radial1 - radial0,
        sections=6,
    )
    mesh.apply_transform(rotation_matrix(math.pi / 2.0, [0, 1, 0]))
    mesh.apply_transform(rotation_matrix(math.pi / 2.0, [1, 0, 0]))
    mesh.apply_transform(rotation_matrix(angle_rad, [0, 0, 1]))
    radial_mid = (radial0 + radial1) / 2.0
    mesh.apply_translation([
        radial_mid * math.cos(angle_rad),
        radial_mid * math.sin(angle_rad),
        z,
    ])
    return mesh


def _union(*meshes: trimesh.Trimesh) -> trimesh.Trimesh:
    out = trimesh.boolean.union(list(meshes), engine="manifold")
    out.remove_unreferenced_vertices()
    if not out.is_volume:
        raise RuntimeError("union did not produce a closed volume")
    return out


def _diff(body: trimesh.Trimesh, *cuts: trimesh.Trimesh) -> trimesh.Trimesh:
    out = trimesh.boolean.difference([body, *cuts], engine="manifold")
    out.remove_unreferenced_vertices()
    if not out.is_volume:
        raise RuntimeError("difference did not produce a closed volume")
    return out


def _intersect(*meshes: trimesh.Trimesh) -> trimesh.Trimesh:
    out = trimesh.boolean.intersection(list(meshes), engine="manifold")
    out.remove_unreferenced_vertices()
    if not out.is_volume:
        raise RuntimeError("intersection did not produce a closed volume")
    return out


def _inter_vol(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    return base._inter_vol(a, b)


def make_chassis_bottom_without_trunk(
    source: trimesh.Trimesh,
) -> trimesh.Trimesh:
    """Close only the inherited rectangular battery-trunk through-hole."""
    patch = hp._box(
        (
            hp.BATTERY_TRUNK_HOLE_X + 2.0 * CHASSIS_TRUNK_PATCH_OVERLAP,
            hp.BATTERY_TRUNK_HOLE_Y + 2.0 * CHASSIS_TRUNK_PATCH_OVERLAP,
            CHASSIS_BOTTOM_Z1 - CHASSIS_BOTTOM_Z0,
        ),
        center=(
            hp.BATTERY_TRUNK_HOLE_CENTRE[0],
            hp.BATTERY_TRUNK_HOLE_CENTRE[1],
            0.5 * (CHASSIS_BOTTOM_Z0 + CHASSIS_BOTTOM_Z1),
        ),
    )
    return _union(source, patch)


def _front_usual_m3_centres() -> list[tuple[float, float]]:
    """Return the ordinary four (y,z) centres on the closed/front web."""
    r = FRONT_PATTERN_PCD / 2.0
    return [
        (r * math.cos(angle), BRACKET_MID_Z + r * math.sin(angle))
        for angle in hp.DISC_HORN_BOLT_ANGLES_RAD
    ]


def _front_extra_m3_centres() -> list[tuple[float, float]]:
    """Return the measured outer pair of (y,z) centres."""
    return [
        (0.0, BRACKET_Z0 + EXTRA_EDGE_INSET),
        (0.0, BRACKET_Z1 - EXTRA_EDGE_INSET),
    ]


def _front_m3_centres() -> list[tuple[float, float]]:
    """Return all six (y,z) centres on the closed/front web."""
    return _front_usual_m3_centres() + _front_extra_m3_centres()


def _side_plate(z0: float, z1: float) -> trimesh.Trimesh:
    """One 2.1 mm bought-bracket blade with the normal disc pattern."""
    zc = (z0 + z1) / 2.0
    end = _cyl_z(CHORN_BLADE_WIDTH / 2.0, z0, z1, x=AXIS_X)
    run = _box(
        (FRONT_X1 - AXIS_X, CHORN_BLADE_WIDTH, z1 - z0),
        ((AXIS_X + FRONT_X1) / 2.0, 0.0, zc),
    )
    body = _union(end, run)
    cuts = [_cyl_z(CHORN_CENTER_HOLE_D / 2.0, z0 - 1.0, z1 + 1.0,
                   x=AXIS_X)]
    for x, y in hp._disc_horn_bolt_centres():
        cuts.append(_cyl_z(FRONT_M3_D / 2.0, z0 - 1.0, z1 + 1.0,
                           x=x, y=y))
    return _diff(body, *cuts)


def make_premade_chorn_reference() -> trimesh.Trimesh:
    """Installed-state reference for one purchased bracket (do not print)."""
    top = _side_plate(TOP_PLATE_Z0, TOP_PLATE_Z1)
    bot = _side_plate(BOT_PLATE_Z0, BOT_PLATE_Z1)
    web = _box(
        (CHORN_PLATE_T, CHORN_BLADE_WIDTH, CHORN_OUTSIDE_SPAN),
        ((FRONT_X0 + FRONT_X1) / 2.0, 0.0, BRACKET_MID_Z),
    )
    cuts = [
        _cyl_x(CHORN_CENTER_HOLE_D / 2.0, FRONT_X0 - 1.0, FRONT_X1 + 1.0,
               z=BRACKET_MID_Z)
    ]
    cuts += [
        _cyl_x(FRONT_M3_D / 2.0, FRONT_X0 - 1.0, FRONT_X1 + 1.0,
               y=y, z=z)
        for y, z in _front_usual_m3_centres()
    ]
    cuts += [
        _cyl_x(EXTRA_PHYSICAL_HOLE_D / 2.0,
               FRONT_X0 - 1.0, FRONT_X1 + 1.0, y=y, z=z)
        for y, z in _front_extra_m3_centres()
    ]
    return _diff(_union(top, bot, web), *cuts)


def _spacer_body(z0: float, z1: float) -> trimesh.Trimesh:
    body = _cyl_z(SPACER_OD / 2.0, z0, z1, x=AXIS_X)
    cuts = [
        _cyl_z(SPACER_M3_D / 2.0, z0 - 1.0, z1 + 1.0, x=x, y=y)
        for x, y in hp._disc_horn_bolt_centres()
    ]
    return _diff(body, *cuts)


def make_driven_spacer() -> trimesh.Trimesh:
    """One-piece 7 mm puck; blind head relief opens toward the driven horn."""
    z0 = base.DISC_TOP_FACE_Z
    z1 = z0 + SPACER_T
    body = _spacer_body(z0, z1)
    pocket = _cyl_z(DRIVE_HEAD_POCKET_D / 2.0, z0 - 0.1,
                    z0 + DRIVE_HEAD_POCKET_DEPTH, x=AXIS_X)
    tool = _cyl_z(DRIVE_TOOL_ACCESS_D / 2.0, z0 - 0.1, z1 + 0.1,
                  x=AXIS_X)
    return _diff(body, pocket, tool)


def make_passive_spacer() -> trimesh.Trimesh:
    """One-piece 7 mm puck with full centre-screw/access clearance."""
    z1 = base.DISC_BOT_FACE_Z
    z0 = z1 - SPACER_T
    body = _spacer_body(z0, z1)
    centre = _cyl_z(PASSIVE_CENTER_THROUGH_D / 2.0, z0 - 0.1, z1 + 0.1,
                    x=AXIS_X)
    return _diff(body, centre)


def _tibia_receiver_plate() -> trimesh.Trimesh:
    """Eight mm tibia receiver with six captive-nut pockets."""
    body = _box(
        (RECEIVER_T, CHORN_BLADE_WIDTH, CHORN_OUTSIDE_SPAN),
        ((RECEIVER_X0 + RECEIVER_X1) / 2.0, 0.0, BRACKET_MID_Z),
    )
    return _diff(body, *_receiver_cuts(include_center=True))


def _femur_receiver_cuts() -> list[trimesh.Trimesh]:
    """Centre clearance plus six front-loaded captive-M3-nut pockets."""
    cuts = [
        _cyl_x(
            FEMUR_CENTER_D / 2.0,
            RECEIVER_X0 - 0.5,
            FEMUR_RECEIVER_X1 + 0.5,
            z=BRACKET_MID_Z,
        )
    ]
    for y, z in _front_m3_centres():
        # Aim an inward flat at the centre hole. This preserves the maximum
        # possible web between the four PCD nut pockets and the Phi8 opening.
        flat_angle = math.atan2(z - BRACKET_MID_Z, y)
        cuts.append(
            _cyl_x(
                FRONT_M3_D / 2.0,
                RECEIVER_X0 - 0.3,
                RECEIVER_X0 + FEMUR_TIP_BORE_DEPTH,
                y=y,
                z=z,
            )
        )
        cuts.append(
            _hex_x(
                FEMUR_NUT_AF,
                RECEIVER_X0 - 0.2,
                RECEIVER_X0 + FEMUR_NUT_DEPTH,
                y=y,
                z=z,
                flat_normal_angle_rad=flat_angle,
            )
        )
    return cuts


def _femur_receiver_plate() -> trimesh.Trimesh:
    """Servo-safe receiver ending at the inherited knee-cavity wall."""
    body = _box(
        (FEMUR_RECEIVER_T, CHORN_BLADE_WIDTH, CHORN_OUTSIDE_SPAN),
        (
            (RECEIVER_X0 + FEMUR_RECEIVER_X1) / 2.0,
            0.0,
            BRACKET_MID_Z,
        ),
    )
    return _diff(body, *_femur_receiver_cuts())


def _receiver_cuts(*, include_center: bool) -> list[trimesh.Trimesh]:
    """Through-bolts and accessible outer-face nyloc pockets."""
    cuts = []
    if include_center:
        cuts.append(
        _cyl_x(RECEIVER_CENTER_D / 2.0, RECEIVER_X0 - 1.0,
               RECEIVER_X1 + 1.0, z=BRACKET_MID_Z)
        )
    for y, z in _front_m3_centres():
        cuts.append(_cyl_x(FRONT_M3_D / 2.0, RECEIVER_X0 - 1.0,
                           RECEIVER_X1 + 1.0, y=y, z=z))
        cuts.append(_hex_x(NYLOC_AF, RECEIVER_X1 - NYLOC_POCKET_DEPTH,
                           RECEIVER_X1 + 0.2, y=y, z=z))
    return cuts


def _keep_x_ge(mesh: trimesh.Trimesh, x0: float) -> trimesh.Trimesh:
    lo, hi = mesh.bounds
    keep = _box(
        (hi[0] - x0 + 4.0, hi[1] - lo[1] + 4.0, hi[2] - lo[2] + 4.0),
        ((x0 + hi[0] + 4.0) / 2.0, (lo[1] + hi[1]) / 2.0,
         (lo[2] + hi[2]) / 2.0),
    )
    out = trimesh.boolean.intersection([mesh, keep], engine="manifold")
    out.remove_unreferenced_vertices()
    if not out.is_volume:
        raise RuntimeError("trimmed inherited part is not a volume")
    return out


def make_femur_body(source: trimesh.Trimesh) -> trimesh.Trimesh:
    """Inherited knee block with receiver wall and captive clamp-cap nuts."""
    trimmed = _keep_x_ge(source, RECEIVER_X0 + 0.05)
    # The inherited body begins 0.05 mm beyond the mating datum, still well
    # inside the receiver, while its mating face stays exactly on the metal
    # web's x=63.5 face.  Its inner face stops at the existing cavity wall.
    receiver = _femur_receiver_plate()
    body = _union(trimmed, receiver)
    # The inherited wall overlaps the receiver and would otherwise refill the
    # captive-nut pockets during union, so cut the final interface once more.
    body = _diff(body, *_femur_receiver_cuts())
    knee_outboard_bolt = [
        centre for centre in hp.servo_clamp_bolt_centres()
        if centre[0] > 0.0
    ]
    return _diff(
        body,
        *_clamp_cap_captive_nut_cuts(
            KNEE_CAP_TO_FEMUR,
            knee_outboard_bolt,
        ),
    )


def make_knee_cap_premade(source: trimesh.Trimesh) -> trimesh.Trimesh:
    """Notch the removable cap around the bought web + femur receiver.

    The overhead cap predates the bought bracket.  Its +Y jaw occupied the
    same root volume as the new metal web and the receiver's upper-middle
    insert station.  Cut only that shared envelope; the rest of the jaw and
    its up-screw boss remain unchanged.
    """
    cap_to_femur = np.linalg.inv(base.MH) @ base.M_KNEE_JP
    femur_to_cap = np.linalg.inv(cap_to_femur)
    cap = source.copy()
    cap.apply_transform(cap_to_femur)
    clearance = 0.25
    # The inherited jaw starts 1.15 mm behind the bought web. Cutting exactly
    # to the normal clearance plane left that terminal root as a detached
    # 20.6 mm3 island after float32 STL tessellation. Carry the same clearance
    # envelope through the abandoned root; the retained cap begins another
    # 3.85 mm farther inboard, so no useful clamp material is removed.
    severed_root_cleanup = 1.25
    x0 = FRONT_X0 - clearance - severed_root_cleanup
    x1 = FEMUR_RECEIVER_X1 + clearance
    cut = _box(
        (
            x1 - x0,
            CHORN_BLADE_WIDTH + 2.0 * clearance,
            CHORN_OUTSIDE_SPAN + 2.0 * clearance,
        ),
        (
            (x0 + x1) / 2.0,
            0.0,
            BRACKET_MID_Z,
        ),
    )
    cap = _diff(cap, cut)
    assert cap.body_count == 1, (
        f"knee clamp cap has {cap.body_count} disconnected bodies after "
        "the bought-web clearance cut"
    )
    cap.apply_transform(femur_to_cap)
    return cap


def _tibia_ring_access_cuts() -> list[trimesh.Trimesh]:
    """Four rounded radial windows into the boss-covered nyloc pockets."""
    cuts = []
    radial_outer = TIB_ACCESS_RADIAL_OUTER
    xmid = (TIB_ACCESS_X0 + TIB_ACCESS_X1) / 2.0
    xlen = TIB_ACCESS_X1 - TIB_ACCESS_X0
    for y, z in _front_usual_m3_centres():
        # Round the closed end at the screw/nut centre to avoid a sharp stress
        # corner, then continue the slot radially through the boss OD.
        cuts.append(
            _cyl_x(
                TIB_ACCESS_W / 2.0,
                TIB_ACCESS_X0,
                TIB_ACCESS_X1,
                y=y,
                z=z,
            )
        )
        dy = y
        dz = z - BRACKET_MID_Z
        if abs(dy) > abs(dz):
            outer_y = math.copysign(radial_outer, dy)
            y0, y1 = sorted((y, outer_y))
            cuts.append(
                _box(
                    (xlen, y1 - y0, TIB_ACCESS_W),
                    (xmid, (y0 + y1) / 2.0, z),
                )
            )
        else:
            outer_z = BRACKET_MID_Z + math.copysign(radial_outer, dz)
            z0, z1 = sorted((z, outer_z))
            cuts.append(
                _box(
                    (xlen, TIB_ACCESS_W, z1 - z0),
                    (xmid, y, (z0 + z1) / 2.0),
                )
            )
    return cuts


def _tibia_root_reinforcement() -> list[trimesh.Trimesh]:
    """Tapered receiver shoulder plus diagonal ribs between nut windows."""
    x0 = RECEIVER_X1 - TIB_ROOT_FLARE_OVERLAP
    flare = _frustum_x(
        TIB_ROOT_FLARE_R,
        TIB_SOCKET_OUTER_R,
        x0,
        RECEIVER_X1 + TIB_ROOT_FLARE_LEN,
        z=BRACKET_MID_Z,
    )
    ribs = []
    rib_x0 = RECEIVER_X1 - TIB_ROOT_FLARE_OVERLAP
    rib_x1 = RECEIVER_X1 + TIB_ROOT_RIB_LEN
    radial_mid = (TIB_ROOT_RIB_R0 + TIB_ROOT_RIB_R1) / 2.0
    for angle in TIB_ROOT_RIB_ANGLES:
        rib = trimesh.creation.box(extents=(
            rib_x1 - rib_x0,
            TIB_ROOT_RIB_R1 - TIB_ROOT_RIB_R0,
            TIB_ROOT_RIB_W,
        ))
        rib.apply_transform(rotation_matrix(angle, [1, 0, 0]))
        rib.apply_translation([
            (rib_x0 + rib_x1) / 2.0,
            radial_mid * math.cos(angle),
            BRACKET_MID_Z + radial_mid * math.sin(angle),
        ])
        ribs.append(rib)
    return [flare, *ribs]


def make_tibia_socket() -> trimesh.Trimesh:
    """Six-hole receiver and reinforced Phi8 socket with four nut windows."""
    receiver = _tibia_receiver_plate()
    boss = _cyl_x(
        hp.LEG_TUBE_OD / 2.0 + hp.LEG_TUBE_SOCKET_WALL,
        RECEIVER_X0 + 0.05,
        TIB_MOUTH_X,
        z=BRACKET_MID_Z,
    )
    bore = _cyl_x(
        hp.LEG_TUBE_OD / 2.0 + hp.LEG_TUBE_SOCKET_CLEAR,
        TIB_BORE_X0,
        TIB_MOUTH_X + 0.5,
        z=BRACKET_MID_Z,
    )
    # Keep a 0.30 mm tube stop behind the metal web's Phi8 centre opening.
    # The four boss-covered nyloc pockets open radially through U-windows;
    # the two measured outer-pair pockets already lie outside the ring.
    return _diff(_union(receiver, boss, *_tibia_root_reinforcement()), bore,
                 *_receiver_cuts(include_center=False),
                 *_tibia_ring_access_cuts())


def make_front_pattern_coupon() -> trimesh.Trimesh:
    """Cheap 4 mm coupon for checking the six front holes before long prints."""
    x0, x1 = 0.0, 4.0
    body = _box((4.0, CHORN_BLADE_WIDTH, CHORN_OUTSIDE_SPAN),
                (2.0, 0.0, BRACKET_MID_Z))
    cuts = [_cyl_x(RECEIVER_CENTER_D / 2.0, x0 - 1.0, x1 + 1.0,
                   z=BRACKET_MID_Z)]
    cuts += [_cyl_x(FRONT_M3_D / 2.0, x0 - 1.0, x1 + 1.0, y=y, z=z)
             for y, z in _front_m3_centres()]
    return _diff(body, *cuts)


def _rear_contour_solid(z0: float, z1: float) -> trimesh.Trimesh:
    """Rear segment of the lower coxa's circular plan envelope."""
    body = _cyl_z(TOWER_OUTER_R, z0, z1)
    front_cut_x1 = TOWER_OUTER_R + 1.0
    front_cut = _box(
        (front_cut_x1 - TOWER_FRONT_X, 2.0 * TOWER_OUTER_R + 2.0,
         z1 - z0 + 2.0),
        ((TOWER_FRONT_X + front_cut_x1) / 2.0, 0.0, (z0 + z1) / 2.0),
    )
    rear_driver_x = -hp.DISC_HORN_BOLT_PCD / 2.0
    driver_scallop = _cyl_z(
        hp.YAW_HUB_HORN_HEAD_CB_OD / 2.0 + TOWER_YAW_DRIVER_EXTRA_R,
        z0 - 1.0,
        z1 + 1.0,
        x=rear_driver_x,
    )
    return _diff(body, front_cut, driver_scallop)


def _tower_nut_cuts() -> list[trimesh.Trimesh]:
    """Top-open captive-nut pockets and screw-tip relief in the tower."""
    cuts = []
    for y in TOWER_SCREW_YS:
        cuts.append(_cyl_z(
            TOWER_M3_D / 2.0,
            TOWER_TOP_Z - TOWER_TIP_CLEARANCE_DEPTH,
            TOWER_TOP_Z + 0.2,
            x=TOWER_SCREW_X,
            y=y,
        ))
        cuts.append(_hex_z(
            TOWER_NUT_AF,
            TOWER_TOP_Z - TOWER_NUT_DEPTH,
            TOWER_TOP_Z + 0.3,
            x=TOWER_SCREW_X,
            y=y,
            flat_normal_angle_rad=math.atan2(y, TOWER_SCREW_X),
        ))
    return cuts


def make_integral_tower_body() -> trimesh.Trimesh:
    """Extrude the lower coxa's rear circular contour straight upward."""
    body = _rear_contour_solid(TOWER_BOTTOM_Z, TOWER_TOP_Z)
    return _diff(body, *_tower_nut_cuts())


def _xy_beam(
    start: tuple[float, float],
    end: tuple[float, float],
    width: float,
    z0: float,
    z1: float,
) -> trimesh.Trimesh:
    """Rectangular horizontal beam between two XY points."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    beam = trimesh.creation.box(extents=(length, width, z1 - z0))
    beam.apply_transform(rotation_matrix(math.atan2(dy, dx), [0, 0, 1]))
    beam.apply_translation([
        (start[0] + end[0]) / 2.0,
        (start[1] + end[1]) / 2.0,
        (z0 + z1) / 2.0,
    ])
    return beam


def _contoured_arm(
    side: int,
    z0: float,
    z1: float,
    *,
    curve_r0: float,
    curve_r1: float,
    straight_y: float,
) -> trimesh.Trimesh:
    """One rib whose root wraps around, then runs tangent to, the R19 hub.

    The inside of the curved root overlaps the existing circular tower by
    1 mm. Its straight leg remains outside the servo case and strengthens the
    tower without recreating the perforated centre slab.
    """
    assert side in (-1, 1)
    annulus = _diff(
        _cyl_z(curve_r1, z0, z1),
        _cyl_z(curve_r0, z0 - 0.2, z1 + 0.2),
    )
    root_gap = 0.5
    y0, y1 = (
        (root_gap, curve_r1 + 0.5)
        if side > 0
        else (-curve_r1 - 0.5, -root_gap)
    )
    curved_root = _intersect(
        annulus,
        _box(
            (
                COXA_REINF_CURVE_X1 - COXA_REINF_X0,
                y1 - y0,
                z1 - z0 + 0.4,
            ),
            (
                (COXA_REINF_X0 + COXA_REINF_CURVE_X1) / 2.0,
                (y0 + y1) / 2.0,
                (z0 + z1) / 2.0,
            ),
        ),
    )
    straight = _box(
        (
            COXA_REINF_X1 + 0.5,
            COXA_REINF_ARM_W,
            z1 - z0,
        ),
        (
            (COXA_REINF_X1 - 0.5) / 2.0,
            side * straight_y,
            (z0 + z1) / 2.0,
        ),
    )
    return _union(curved_root, straight)


def _upper_arm_access() -> trimesh.Trimesh:
    """Driver tunnel through the remaining upper rib for the case screw."""
    return _cyl_y(
        COXA_REINF_ACCESS_D / 2.0,
        -COXA_REINF_HI_CURVE_R1 - 1.0,
        COXA_REINF_HI_CURVE_R1 + 1.0,
        x=COXA_REINF_ACCESS_X,
        z=COXA_REINF_ACCESS_Z,
    )


def _upper_bearing_root_buttresses() -> list[trimesh.Trimesh]:
    """Two rear contour webs carrying the upper bearing into the coxa.

    These stop before the servo enclosure: only the circular roots continue
    upward. They overlap the existing upper supports at their lower edge and
    the R19 tower through their full height.
    """
    annulus = _diff(
        _cyl_z(
            COXA_BEARING_BUTTRESS_R1,
            COXA_BEARING_BUTTRESS_Z0,
            COXA_BEARING_BUTTRESS_Z1,
        ),
        _cyl_z(
            COXA_BEARING_BUTTRESS_R0,
            COXA_BEARING_BUTTRESS_Z0 - 0.2,
            COXA_BEARING_BUTTRESS_Z1 + 0.2,
        ),
    )
    x0 = -COXA_BEARING_BUTTRESS_R1 - 0.5
    buttresses = []
    for side in (-1, 1):
        y0, y1 = (
            (0.5, COXA_BEARING_BUTTRESS_R1 + 0.5)
            if side > 0
            else (-COXA_BEARING_BUTTRESS_R1 - 0.5, -0.5)
        )
        buttresses.append(_intersect(
            annulus,
            _box(
                (
                    COXA_BEARING_BUTTRESS_X1 - x0,
                    y1 - y0,
                    COXA_BEARING_BUTTRESS_Z1
                    - COXA_BEARING_BUTTRESS_Z0 + 0.4,
                ),
                (
                    (x0 + COXA_BEARING_BUTTRESS_X1) / 2.0,
                    (y0 + y1) / 2.0,
                    (COXA_BEARING_BUTTRESS_Z0
                     + COXA_BEARING_BUTTRESS_Z1) / 2.0,
                ),
            ),
        ))
    return buttresses


def _positive_servo_tab_block() -> trimesh.Trimesh:
    """One solid contour-rooted block ending at the servo retention tab."""
    # A full-height circular root keeps the earlier contour requirement and
    # carries the block into the integral rear tower without a stepped seam.
    contoured = _contoured_arm(
        1,
        COXA_REINF_TAB_BLOCK_Z0,
        COXA_REINF_TAB_BLOCK_Z1,
        curve_r0=COXA_REINF_HI_CURVE_R0,
        curve_r1=COXA_REINF_HI_CURVE_R1,
        straight_y=COXA_REINF_HI_STRAIGHT_Y,
    )
    x_clip = _box(
        (
            COXA_REINF_TAB_BLOCK_X1 - (COXA_REINF_X0 - 1.0),
            2.0 * COXA_REINF_HI_CURVE_R1 + 2.0,
            COXA_REINF_TAB_BLOCK_Z1 - COXA_REINF_TAB_BLOCK_Z0 + 2.0,
        ),
        (
            (COXA_REINF_X0 - 1.0 + COXA_REINF_TAB_BLOCK_X1) / 2.0,
            0.0,
            (COXA_REINF_TAB_BLOCK_Z0 + COXA_REINF_TAB_BLOCK_Z1) / 2.0,
        ),
    )
    contoured = _intersect(contoured, x_clip)
    block = _box(
        (
            COXA_REINF_TAB_BLOCK_X1 - COXA_REINF_TAB_BLOCK_X0,
            COXA_REINF_TAB_BLOCK_Y1 - COXA_REINF_TAB_BLOCK_Y0,
            COXA_REINF_TAB_BLOCK_Z1 - COXA_REINF_TAB_BLOCK_Z0,
        ),
        (
            (COXA_REINF_TAB_BLOCK_X0 + COXA_REINF_TAB_BLOCK_X1) / 2.0,
            (COXA_REINF_TAB_BLOCK_Y0 + COXA_REINF_TAB_BLOCK_Y1) / 2.0,
            (COXA_REINF_TAB_BLOCK_Z0 + COXA_REINF_TAB_BLOCK_Z1) / 2.0,
        ),
    )
    solid = _union(contoured, block)
    driver_cuts = [
        _cyl_y(
            COXA_REINF_TAB_DRIVER_D / 2.0,
            COXA_REINF_TAB_BLOCK_Y0 - 1.0,
            COXA_REINF_TAB_BLOCK_Y1 + 1.0,
            x=COXA_REINF_TAB_HOLE_X,
            z=z,
        )
        for z in COXA_REINF_TAB_HOLE_ZS
    ]
    return _diff(solid, *driver_cuts)


def make_coxa_reinforcement_rails() -> trimesh.Trimesh:
    """Servo supports plus two rear webs below the upper bearing."""
    lower_arms = [
        _contoured_arm(
            side,
            COXA_REINF_LO_Z0,
            COXA_REINF_LO_Z1,
            curve_r0=COXA_REINF_LO_CURVE_R0,
            curve_r1=COXA_REINF_LO_CURVE_R1,
            straight_y=COXA_REINF_LO_STRAIGHT_Y,
        )
        for side in (-1,)
    ]
    upper_arms = [
        _contoured_arm(
            side,
            COXA_REINF_HI_Z0,
            COXA_REINF_HI_Z1,
            curve_r0=COXA_REINF_HI_CURVE_R0,
            curve_r1=COXA_REINF_HI_CURVE_R1,
            straight_y=COXA_REINF_HI_STRAIGHT_Y,
        )
        for side in (-1,)
    ]
    rails = _union(
        *lower_arms,
        *upper_arms,
        _positive_servo_tab_block(),
        *_upper_bearing_root_buttresses(),
    )
    return _diff(rails, _upper_arm_access())


def make_centered_servo_holder_foot() -> trimesh.Trimesh:
    """Complete the low cradle foot symmetrically about the servo centreline.

    The serviceable holder is intentionally open on +Y so the fitted horn can
    leave with the servo.  Only its low foot was asymmetric: it ended 8 mm
    early on that side, which exposed the positive-Y upper contour arm as the
    odd ledge in the top drawing.  This low extension stops at the original
    foot Z, below the servo, and retains the inherited femur-yoke sweep cut.
    """
    fill = _box(
        (
            COXA_HOLDER_FOOT_X1 - COXA_HOLDER_FOOT_X0,
            COXA_HOLDER_FOOT_Y1 - COXA_HOLDER_FOOT_OLD_Y1,
            COXA_HOLDER_FOOT_Z1 - COXA_HOLDER_FOOT_Z0,
        ),
        (
            (COXA_HOLDER_FOOT_X0 + COXA_HOLDER_FOOT_X1) / 2.0,
            (COXA_HOLDER_FOOT_OLD_Y1 + COXA_HOLDER_FOOT_Y1) / 2.0,
            (COXA_HOLDER_FOOT_Z0 + COXA_HOLDER_FOOT_Z1) / 2.0,
        ),
    )
    hip_axis_x = base.HIP_ANCHOR_OVH[0]
    hip_axis_z = base.HIP_ANCHOR_OVH[2]
    positive_yoke_sweep = _cyl_y(
        16.75,
        21.75,
        30.0,
        x=hip_axis_x,
        z=hip_axis_z,
    )
    return _diff(fill, positive_yoke_sweep)


def make_integral_tower_coxa(source: trimesh.Trimesh) -> trimesh.Trimesh:
    """Fuse the tower/arms and add the hip clamp-cap captive nuts."""
    tower = make_integral_tower_body()
    rails = make_coxa_reinforcement_rails()
    holder_foot = make_centered_servo_holder_foot()
    source = _union(source, holder_foot)
    # Remove the ancestor's perforated circular bridge above the yaw hub.
    # The relief stops at the R16.9 cartridge bore; extending it to R20 cut a
    # stray D-shaped opening into the flat servo-holder foot. The tower and
    # three side supports replace the bridge while the five horn-driver shafts
    # remain in open air rather than becoming a 3x3 hole grid.
    centre_relief = _cyl_z(
        COXA_REINF_CENTRE_RELIEF_R,
        TOWER_BOTTOM_Z,
        COXA_REINF_HI_Z0 + 0.2,
    )
    source = _diff(source, centre_relief)
    tower_overlap = _inter_vol(rails, tower)
    source_overlap = _inter_vol(rails, source)
    assert tower_overlap >= 100.0, (
        f"coxa contour arms have only {tower_overlap:.1f} mm3 tower overlap"
    )
    assert source_overlap >= 100.0, (
        f"coxa contour arms have only {source_overlap:.1f} mm3 cradle overlap"
    )
    body = _union(source.copy(), tower, rails)
    # The new upper bearing-root webs overlap the lower end of the existing
    # top-entry screw corridors. Re-cut those authored paths after the union;
    # otherwise the reinforcement would make the carrier impossible to fit.
    return _diff(
        body,
        *_tower_nut_cuts(),
        *_clamp_cap_captive_nut_cuts(HIP_CAP_TO_COXA),
    )


def split_coxa_yaw_hub(
    reinforced: trimesh.Trimesh,
) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    """Split off a deep, radially screwed lower-yaw carrier.

    An annular collar on the carrier rises 8 mm into a matching socket in the
    coxa. Three edge-access M3 screws enter radially through that socket and
    engage captive nuts in the collar. This removes the former pair of thin
    top-screw tongues while leaving the central horn-driver region open.
    """
    lo, _ = reinforced.bounds
    hub_envelope = _cyl_z(
        TOWER_OUTER_R,
        float(lo[2] - 1.0),
        COXA_HUB_INTERFACE_Z,
    )
    hub = _intersect(reinforced, hub_envelope)
    # Give the cartridge a deliberate hairline radial clearance.  Besides
    # preventing a press-fit after PETG shrinkage, this keeps the relief cut
    # away from the ancestor's coincident R19 dust-brim wall and produces a
    # robust sliced edge instead of stacked, nearly-identical facets.
    main_relief = _cyl_z(
        TOWER_OUTER_R + COXA_HUB_RADIAL_CLEARANCE,
        float(lo[2] - 1.0),
        COXA_HUB_INTERFACE_Z,
    )
    main = _diff(reinforced, main_relief)
    # Open a true cylindrical socket through the old rear tower/foot material;
    # the carrier collar occupies this volume instead of merely overlapping
    # the nominally open centre of the former bridge.
    main = _diff(
        main,
        _cyl_z(
            COXA_HUB_SOCKET_R0,
            COXA_HUB_INTERFACE_Z - 0.2,
            COXA_HUB_COLLAR_Z1 + 0.2,
        ),
    )

    socket_ring = _diff(
        _cyl_z(
            COXA_HUB_SOCKET_R1,
            COXA_HUB_INTERFACE_Z,
            COXA_HUB_COLLAR_Z1,
        ),
        _cyl_z(
            COXA_HUB_SOCKET_R0,
            COXA_HUB_INTERFACE_Z - 0.2,
            COXA_HUB_COLLAR_Z1 + 0.2,
        ),
    )
    main = _union(main, socket_ring)

    carrier_collar = _diff(
        _cyl_z(
            COXA_HUB_COLLAR_R1,
            COXA_HUB_INTERFACE_Z - 0.4,
            COXA_HUB_COLLAR_Z1,
        ),
        _cyl_z(
            COXA_HUB_COLLAR_R0,
            COXA_HUB_INTERFACE_Z - 0.6,
            COXA_HUB_COLLAR_Z1 + 0.2,
        ),
    )
    hub = _union(hub, carrier_collar)

    main_cuts = []
    hub_cuts = []
    for angle in COXA_HUB_SCREW_ANGLES:
        main_cuts.extend([
            _cyl_radial_xy(
                COXA_HUB_M3_D / 2.0,
                COXA_HUB_COLLAR_R1 - 0.2,
                COXA_HUB_HEAD_SEAT_R + 0.3,
                angle,
                z=COXA_HUB_SCREW_Z,
            ),
            _cyl_radial_xy(
                COXA_HUB_HEAD_D / 2.0,
                COXA_HUB_HEAD_SEAT_R,
                COXA_HUB_HEAD_ACCESS_R1,
                angle,
                z=COXA_HUB_SCREW_Z,
            ),
        ])
        hub_cuts.extend([
            _cyl_radial_xy(
                COXA_HUB_M3_D / 2.0,
                COXA_HUB_COLLAR_R0 - 0.3,
                COXA_HUB_COLLAR_R1 + 0.3,
                angle,
                z=COXA_HUB_SCREW_Z,
            ),
            _hex_radial_xy(
                COXA_HUB_NUT_AF,
                COXA_HUB_COLLAR_R1 - COXA_HUB_NUT_DEPTH,
                COXA_HUB_COLLAR_R1 + 0.3,
                angle,
                z=COXA_HUB_SCREW_Z,
            ),
        ])
    return _diff(main, *main_cuts), _diff(hub, *hub_cuts)


def _bearing_carrier_screw_centres() -> list[tuple[float, float]]:
    """Return (x, z) in hip-cap-local coordinates for the three screws."""
    centres = []
    for angle in BEARING_CARRIER_SCREW_ANGLES:
        # Angles are specified in coxa/world plan. Cap-local +Z maps to
        # coxa -Y, while cap-local +X maps directly to coxa +X.
        centres.append((
            BEARING_CARRIER_CENTER_X
            + BEARING_CARRIER_SCREW_R * math.cos(angle),
            BEARING_CARRIER_CENTER_Z
            - BEARING_CARRIER_SCREW_R * math.sin(angle),
        ))
    return centres


def _hip_cap_bearing_mount_cuts() -> list[trimesh.Trimesh]:
    """Through shafts and underside 90-degree head seats in the flat cap."""
    cuts = []
    for x, z in _bearing_carrier_screw_centres():
        cuts.append(_cyl_y(
            BEARING_CARRIER_M3_D / 2.0,
            BEARING_CARRIER_CAP_PAD_Y0 - 0.3,
            BEARING_CARRIER_BASE_Y + 0.3,
            x=x,
            z=z,
        ))
        cuts.append(_cone_y(
            BEARING_CARRIER_CSK_D / 2.0,
            BEARING_CARRIER_CAP_PAD_Y0 - 0.1,
            BEARING_CARRIER_CAP_PAD_Y0 + BEARING_CARRIER_CSK_DEPTH,
            x=x,
            z=z,
        ))
    return cuts


def make_hip_bearing_carrier() -> trimesh.Trimesh:
    """Flat-backed, screw-on upper 6805 pedestal in hip-cap coordinates."""
    x = BEARING_CARRIER_CENTER_X
    z = BEARING_CARRIER_CENTER_Z
    flange = _cyl_y(
        BEARING_CARRIER_R,
        BEARING_CARRIER_BASE_Y,
        BEARING_CARRIER_FLANGE_TOP_Y,
        x=x,
        z=z,
    )
    pedestal = _cyl_y(
        rv.PED_OD / 2.0,
        BEARING_CARRIER_BASE_Y,
        rv.PED_Y1,
        x=x,
        z=z,
    )
    boss = _cyl_y(
        rv.BOSS_OD / 2.0,
        rv.PED_Y1 - 1.0,
        rv.BOSS_Y1,
        x=x,
        z=z,
    )
    tip = _cyl_y(
        rv.BOSS_OD / 2.0 - rv.BOSS_TIP_STEP,
        rv.BOSS_Y1 - 0.1,
        rv.TIP_Y1,
        x=x,
        z=z,
    )
    body = _union(flange, pedestal, boss, tip)
    cuts = []
    for side in (+1.0, -1.0):
        extension = rv.PED_OD / 2.0 - rv.PULLER_NOTCH_R0 + 3.0
        cuts.append(_box(
            (extension, rv.PULLER_NOTCH_DEPTH + 0.05,
             rv.PULLER_NOTCH_W),
            (
                x + side * (rv.PULLER_NOTCH_R0 + extension / 2.0),
                rv.PED_Y1 - rv.PULLER_NOTCH_DEPTH / 2.0 + 0.025,
                z,
            ),
        ))
    # The two existing top-entry tower screws now pass through the removable
    # carrier rather than through a protruding flange on the hip cap.
    for tower_y in TOWER_SCREW_YS:
        cap_point = COXA_TO_HIP_CAP @ np.array([
            TOWER_SCREW_X, tower_y, 0.0, 1.0
        ])
        sx, sz = float(cap_point[0]), float(cap_point[2])
        cuts.append(_cyl_y(
            TOWER_M3_D / 2.0,
            BEARING_CARRIER_BASE_Y - 0.3,
            BEARING_CARRIER_FLANGE_TOP_Y + 0.5,
            x=sx,
            z=sz,
        ))
        cuts.append(_cyl_y(
            TOWER_HEAD_D / 2.0,
            BEARING_CARRIER_FLANGE_TOP_Y - TOWER_HEAD_DEPTH,
            BEARING_CARRIER_FLANGE_TOP_Y + 0.5,
            x=sx,
            z=sz,
        ))
    for sx, sz in _bearing_carrier_screw_centres():
        cuts.append(_cyl_y(
            BEARING_CARRIER_M3_D / 2.0,
            BEARING_CARRIER_BASE_Y - 0.2,
            BEARING_CARRIER_BASE_Y + BEARING_CARRIER_TIP_CLEARANCE_DEPTH,
            x=sx,
            z=sz,
        ))
        cuts.append(_hex_y(
            BEARING_CARRIER_NUT_AF,
            BEARING_CARRIER_BASE_Y - 0.3,
            BEARING_CARRIER_BASE_Y + BEARING_CARRIER_NUT_DEPTH,
            x=sx,
            z=sz,
            flat_normal_angle_rad=math.atan2(
                sz - BEARING_CARRIER_CENTER_Z,
                sx - BEARING_CARRIER_CENTER_X,
            ),
        ))
    return _diff(body, *cuts)


def make_reinforced_hip_cap(
    source: trimesh.Trimesh,
    coxa: trimesh.Trimesh,
) -> trimesh.Trimesh:
    """Make the cap flat, add its mount pad, and clear the coxa interface.

    The ancestor cap overlaps the coxa's clamp ledge by about 522 mm3.  That
    pair is supposed to close face-to-face (only the cap tongue intentionally
    preloads the servo), so carve clearance rather than allowing that pair.
    The old integral 6805 pedestal is removed at the cap-face plane; a circular
    flat pad and four countersunk through-holes receive the new cartridge.
    """
    cap = source.copy()
    bearing_cut = _cyl_y(
        BEARING_CARRIER_R + 1.0,
        BEARING_CARRIER_BASE_Y - 0.02,
        rv.TIP_Y1 + 1.0,
        x=BEARING_CARRIER_CENTER_X,
        z=BEARING_CARRIER_CENTER_Z,
    )
    cap = _diff(cap, bearing_cut)
    mount_pad = _cyl_y(
        BEARING_CARRIER_R,
        BEARING_CARRIER_CAP_PAD_Y0,
        BEARING_CARRIER_BASE_Y,
        x=BEARING_CARRIER_CENTER_X,
        z=BEARING_CARRIER_CENTER_Z,
    )
    cap = _union(cap, mount_pad)
    cap = _diff(cap, *_hip_cap_bearing_mount_cuts())
    cap.apply_transform(HIP_CAP_TO_COXA)
    cuts = []
    interference = trimesh.boolean.intersection(
        [cap, coxa], engine="manifold"
    )
    if interference is not None:
        interface_clearance = 0.20
        for component in interference.split(only_watertight=False):
            if abs(float(component.volume)) <= 0.1:
                continue
            lo, hi = component.bounds
            size = hi - lo + 2.0 * interface_clearance
            cuts.append(_box(
                tuple(float(value) for value in size),
                tuple(float(value) for value in (lo + hi) / 2.0),
            ))
    cap = _diff(cap, *cuts)
    cap.apply_transform(COXA_TO_HIP_CAP)
    # The new circular bearing pad crosses the inboard clamp screw station.
    # Recut both original clearance/counterbore paths last so that the M3x8
    # clamp hardware remains coaxial with the unchanged servo-cradle walls.
    return _diff(cap, *_clamp_cap_screw_recuts())


def _tibia_tube() -> tuple[trimesh.Trimesh, np.ndarray]:
    """Actual hollow 8 mm OD / 6 mm ID tube at the shortened length.

    The old scene-only reference was a solid cylinder. That was adequate for
    collision visualization but overstated bending stiffness in FEA. Keep the
    physical 1 mm wall so the BuildViz load-chain study sees the real section.
    """
    start_prod = (base.MH @ np.array([
        hp._YOKE_SOCKET_X, 0.0, hp.JOINT_SOCKET_Z, 1.0
    ]))[:3]
    tube_end = start_prod + np.array([hp.TIBIA_LENGTH - 8.0, 0.0, 0.0])
    start = (base.MH @ np.array([
        TIB_BORE_X0, 0.0, hp.JOINT_SOCKET_Z, 1.0
    ]))[:3]
    outer = hp._tube_between(start, tube_end, hp.LEG_TUBE_OD / 2.0)
    axis = tube_end - start
    axis /= np.linalg.norm(axis)
    inner_radius = hp.LEG_TUBE_OD / 2.0 - hp.LEG_TUBE_WALL
    inner = hp._tube_between(start - 0.5 * axis, tube_end + 0.5 * axis,
                             inner_radius)
    tube = _diff(outer, inner)
    foot_frame = hp._frame(tube_end, (1, 0, 0), (0, 0, 1))
    return tube, foot_frame


SCENE_MESH_FILES = {
    "chorn_clamp_cnc": "premade_chorn_56_DO_NOT_PRINT.stl",
    "driven_spacer": "driven_spacer_7mm.stl",
    "passive_spacer": "passive_spacer_7mm.stl",
    "femur_ovh_body": "femur_body_premade_chorn.stl",
    "tibia_ovh_socket": "tibia_socket_premade_chorn.stl",
    "front_pattern_coupon": "front_6hole_fit_coupon.stl",
    "tibia_tube_ovh": "tibia_tube_premade_chorn_DO_NOT_PRINT.stl",
    "knee_clamp_cap_ovh": "knee_clamp_cap_ovh.stl",
    "coxa_link_ovh": "coxa_link_ovh.stl",
    "coxa_yaw_hub_carrier_ovh": "coxa_yaw_hub_carrier_ovh.stl",
    "hip_clamp_cap_ovh": "hip_clamp_cap_ovh.stl",
    "hip_bearing_carrier_ovh": "hip_bearing_carrier_ovh.stl",
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


def _buildviz_clean(mesh: trimesh.Trimesh, label: str) -> trimesh.Trimesh:
    """Weld and manifold-roundtrip a final printed Boolean mesh.

    BuildViz intentionally welds vertices within 0.1 micron when checking an
    STL.  A mesh can therefore be watertight at its exact float32 coordinates
    yet expose tiny open/self-crossing edges after that realistic slicer-style
    weld.  Re-process the final triangle soup and simplify it through the
    manifold kernel so the on-disk STL passes that stronger topology gate.
    """
    processed = trimesh.load_mesh(
        file_obj=io.BytesIO(mesh.export(file_type="stl")),
        file_type="stl",
        process=True,
    )
    cleanup_tolerance = 1e-3 if label == "coxa_link_ovh" else 1e-4
    cleaned = hp._simplify_via_manifold(processed, tol=cleanup_tolerance)
    if cleaned is None:
        raise RuntimeError(
            f"manifold cleanup rejected final printable mesh {label}"
        )
    delta = abs(float(cleaned.volume) - float(mesh.volume))
    assert delta <= max(0.05, 5e-3 * abs(float(mesh.volume))), (
        f"manifold cleanup changed volume by {delta:.3f} mm3"
    )
    return cleaned


def build_meshes() -> dict[str, trimesh.Trimesh]:
    print("loading rigid-bearing / outboard-hip ancestor ...")
    meshes = base.build_meshes()
    source_femur = meshes["femur_ovh_body"]
    source_knee_cap = meshes["knee_clamp_cap_ovh"]
    source_coxa = meshes["coxa_link_ovh"]
    source_hip_cap = meshes["hip_clamp_cap_ovh"]
    source_chassis_bottom = meshes["chassis_bottom"]
    reinforced_coxa_assembly = make_integral_tower_coxa(source_coxa)
    reinforced_coxa, lower_yaw_carrier = split_coxa_yaw_hub(
        reinforced_coxa_assembly
    )
    flat_hip_cap = make_reinforced_hip_cap(source_hip_cap, reinforced_coxa)
    meshes.update({
        "chorn_clamp_cnc": make_premade_chorn_reference(),
        "driven_spacer": make_driven_spacer(),
        "passive_spacer": make_passive_spacer(),
        "femur_ovh_body": make_femur_body(source_femur),
        "knee_clamp_cap_ovh": make_knee_cap_premade(source_knee_cap),
        "tibia_ovh_socket": make_tibia_socket(),
        "front_pattern_coupon": make_front_pattern_coupon(),
        "coxa_link_ovh": reinforced_coxa,
        "coxa_yaw_hub_carrier_ovh": lower_yaw_carrier,
        "coxa_link_assembled_check": reinforced_coxa_assembly,
        "hip_clamp_cap_ovh": flat_hip_cap,
        "hip_bearing_carrier_ovh": make_hip_bearing_carrier(),
        "chassis_bottom": make_chassis_bottom_without_trunk(
            source_chassis_bottom
        ),
    })
    meshes["tibia_tube_ovh"], _ = _tibia_tube()

    for key in (
        "chorn_clamp_cnc", "driven_spacer", "passive_spacer",
        "femur_ovh_body", "knee_clamp_cap_ovh", "tibia_ovh_socket",
        "front_pattern_coupon",
        "coxa_link_ovh", "coxa_yaw_hub_carrier_ovh",
        "hip_clamp_cap_ovh", "hip_bearing_carrier_ovh",
        "tibia_tube_ovh", "chassis_bottom",
    ):
        meshes[key] = hp._heal_for_export(meshes[key])
    for key in (
        "coxa_link_ovh", "coxa_yaw_hub_carrier_ovh",
        "hip_clamp_cap_ovh", "hip_bearing_carrier_ovh", "chassis_bottom",
    ):
        meshes[key] = _buildviz_clean(meshes[key], key)

    STL_DIR.mkdir(parents=True, exist_ok=True)
    stale_support = STL_DIR / "rear_support_column.stl"
    if stale_support.exists():
        stale_support.unlink()
        print(f"removed obsolete separate part {stale_support}")
    for key, filename in SCENE_MESH_FILES.items():
        meshes[key].export(STL_DIR / filename)
    return meshes


def check_parts(meshes: dict[str, trimesh.Trimesh]) -> dict:
    """Dimension, hole, contact, and front-interface checks."""
    printable = (
        "driven_spacer", "passive_spacer", "femur_ovh_body",
        "tibia_ovh_socket", "front_pattern_coupon", "coxa_link_ovh",
        "coxa_yaw_hub_carrier_ovh", "hip_clamp_cap_ovh",
        "hip_bearing_carrier_ovh",
    )
    for key in ("chorn_clamp_cnc", *printable):
        mesh = meshes[key]
        assert mesh.is_watertight and mesh.is_volume, f"{key}: not a volume"
        assert mesh.body_count == 1, f"{key}: expected one connected body"

    bracket = meshes["chorn_clamp_cnc"]
    lo, hi = bracket.bounds
    assert abs((hi[2] - lo[2]) - CHORN_OUTSIDE_SPAN) < 0.03
    assert abs(hi[0] - FRONT_X1) < 0.03
    assert abs((hi[0] - AXIS_X) - CHORN_AXIS_TO_FRONT_OUTER) < 0.03

    # Every plate pattern and every front-web path must be open.
    pcd = hp.DISC_HORN_BOLT_PCD / 2.0
    plate_probes = []
    for z in ((TOP_PLATE_Z0 + TOP_PLATE_Z1) / 2.0,
              (BOT_PLATE_Z0 + BOT_PLATE_Z1) / 2.0):
        plate_probes.append([AXIS_X, 0.0, z])
        plate_probes += [[AXIS_X + pcd * math.cos(a), pcd * math.sin(a), z]
                         for a in hp.DISC_HORN_BOLT_ANGLES_RAD]
    assert not bracket.contains(np.asarray(plate_probes)).any(), \
        "side-plate horn pattern blocked"
    front_probes = [[(FRONT_X0 + FRONT_X1) / 2.0, 0.0, BRACKET_MID_Z]]
    front_probes += [[(FRONT_X0 + FRONT_X1) / 2.0, y, z]
                     for y, z in _front_m3_centres()]
    assert not bracket.contains(np.asarray(front_probes)).any(), \
        "front six-hole pattern blocked"

    # Spacer bores and asymmetrical centre treatment.
    drive = meshes["driven_spacer"]
    passive = meshes["passive_spacer"]
    for spacer, zmid in (
        (drive, base.DISC_TOP_FACE_Z + SPACER_T / 2.0),
        (passive, base.DISC_BOT_FACE_Z - SPACER_T / 2.0),
    ):
        assert abs(spacer.extents[2] - SPACER_T) < 0.03
        holes = [[x, y, zmid] for x, y in hp._disc_horn_bolt_centres()]
        assert not spacer.contains(np.asarray(holes)).any(), \
            "spacer M3 pattern blocked"
    # Driven puck has a blind Phi8.8 pocket (solid roof outside Phi4.2);
    # passive puck has the requested complete centre hole.
    roof_probe = np.array([[AXIS_X + 3.2, 0.0,
                            base.DISC_TOP_FACE_Z + SPACER_T - 0.4]])
    assert drive.contains(roof_probe).all(), "driven spacer blind roof missing"
    passive_line = np.array([[AXIS_X, 0.0, z] for z in np.linspace(
        base.DISC_BOT_FACE_Z - SPACER_T - 0.1,
        base.DISC_BOT_FACE_Z + 0.1, 12)])
    assert not passive.contains(passive_line).any(), \
        "passive centre hole is not complete"

    # Both receiver faces kiss the metal web without overlap.
    for key in ("femur_ovh_body", "tibia_ovh_socket"):
        part = meshes[key]
        assert _inter_vol(bracket, part) < 0.02, f"bracket overlaps {key}"

    # Tibia: six through-bolts into captive nylocs.
    tibia_line = []
    for y, z in _front_m3_centres():
        tibia_line += [
            [x, y, z]
            for x in np.linspace(FRONT_X0 - 0.2, RECEIVER_X1 + 0.2, 20)
        ]
    assert not (
        bracket.contains(np.asarray(tibia_line))
        | meshes["tibia_ovh_socket"].contains(np.asarray(tibia_line))
    ).any(), "six through-bolt paths blocked in tibia receiver"

    # Femur: six captive hex pockets open from the bracket face, with a solid
    # skin before the servo cavity. The seated metal web closes the pockets;
    # each M3x5 tip passes through its nut without entering the servo cavity.
    femur = meshes["femur_ovh_body"]
    assert abs(FEMUR_RECEIVER_T - 3.6) < 0.02
    for index, (y, z) in enumerate(_front_m3_centres()):
        open_line = np.array(
            [
                [x, y, z]
                for x in np.linspace(
                    FRONT_X0 - 0.2,
                    RECEIVER_X0 + FEMUR_SCREW_PROJECTION - 0.1,
                    24,
                )
            ]
        )
        assert not (
            bracket.contains(open_line) | femur.contains(open_line)
        ).any(), f"femur captive-nut screw path {index} blocked"
        blind_end = np.array([[FEMUR_RECEIVER_X1 - 0.2, y, z]])
        assert femur.contains(blind_end).all(), \
            f"femur nut {index} has no servo-side retaining skin"

    # The exact user dimensions intentionally encode the 0.12 mm/side spring
    # spread calculated above.  In the rigid CAD that appears as a small face
    # overlap; keep it bounded instead of pretending the nominal 7 mm pucks
    # are only 6.88 mm thick.
    drive_preload_volume = _inter_vol(bracket, drive)
    passive_preload_volume = _inter_vol(bracket, passive)
    assert 0.1 < drive_preload_volume < 50.0
    assert 0.1 < passive_preload_volume < 50.0

    masses = {
        "premade_bracket_reference_g": round(bracket.volume * RHO_ALU, 1),
        "driven_spacer_g": round(drive.volume * RHO_PETG, 1),
        "passive_spacer_g": round(passive.volume * RHO_PETG, 1),
        "femur_body_g": round(meshes["femur_ovh_body"].volume * RHO_PETG, 1),
        "tibia_socket_g": round(meshes["tibia_ovh_socket"].volume * RHO_PETG, 1),
        "flat_coxa_link_with_integral_tower_g": round(
            meshes["coxa_link_ovh"].volume * RHO_PETG, 1
        ),
        "screw_on_lower_yaw_hub_g": round(
            meshes["coxa_yaw_hub_carrier_ovh"].volume * RHO_PETG, 1
        ),
        "flat_hip_clamp_cap_g": round(
            meshes["hip_clamp_cap_ovh"].volume * RHO_PETG, 1
        ),
        "screw_on_hip_bearing_carrier_g": round(
            meshes["hip_bearing_carrier_ovh"].volume * RHO_PETG, 1
        ),
        "driven_spacer_nominal_preload_overlap_mm3": round(drive_preload_volume, 2),
        "passive_spacer_nominal_preload_overlap_mm3": round(passive_preload_volume, 2),
    }
    print(
        f"  bracket: {CHORN_OUTSIDE_SPAN:.2f} outside / "
        f"{CHORN_INNER_SPAN:.2f} inside, {CHORN_PLATE_T:.2f} plate, "
        f"{CHORN_AXIS_TO_FRONT_OUTER:.1f} axis-to-front"
    )
    print(
        f"  spacers: 2 x {SPACER_T:.2f} close {base.DISC_SPAN:.2f} horn span "
        f"=> {ASSEMBLED_INNER_SPAN:.2f}; nominal bracket spread "
        f"{BRACKET_SPREAD:.2f} mm total"
    )
    print(
        "  front receiver: normal 4xM3/Phi8 pattern + 2xM3 on "
        f"{EXTRA_CENTRE_SPAN:.1f} mm centres "
        f"({EXTRA_EDGE_INSET:.1f} mm edge inset); measured bracket holes "
        f"Ø{EXTRA_PHYSICAL_HOLE_D:.1f}, printed clearance Ø{FRONT_M3_D:.1f}"
    )
    return masses


def check_chassis_bottom_without_trunk(
    meshes: dict[str, trimesh.Trimesh],
) -> dict[str, float | int | bool]:
    """The circled battery-trunk rectangle is solid through the full plate."""
    chassis = meshes["chassis_bottom"]
    assert chassis.is_watertight and chassis.is_volume, \
        "closed-port chassis_bottom is not a volume"
    assert chassis.body_count == 1, \
        "closed-port chassis_bottom is not one connected body"

    cx, cy = hp.BATTERY_TRUNK_HOLE_CENTRE
    xs = np.linspace(
        cx - hp.BATTERY_TRUNK_HOLE_X / 2.0 + 1.0,
        cx + hp.BATTERY_TRUNK_HOLE_X / 2.0 - 1.0,
        4,
    )
    ys = np.linspace(
        cy - hp.BATTERY_TRUNK_HOLE_Y / 2.0 + 1.0,
        cy + hp.BATTERY_TRUNK_HOLE_Y / 2.0 - 1.0,
        5,
    )
    zs = np.linspace(CHASSIS_BOTTOM_Z0 + 0.5, CHASSIS_BOTTOM_Z1 - 0.5, 3)
    probes = np.array([[x, y, z] for x in xs for y in ys for z in zs])
    filled = chassis.contains(probes)
    assert filled.all(), (
        f"battery-trunk patch left {int((~filled).sum())} of "
        f"{len(filled)} interior probes open"
    )

    # The centre strap pair remains the intentional alternate lead route.
    strap_x = hp.BATTERY_HOLDER_CENTRE_X
    strap_probes = np.array([
        [strap_x, sy * hp.BATTERY_STRAP_SLOT_Y, z]
        for sy in (-1.0, 1.0)
        for z in (CHASSIS_BOTTOM_Z0 - 0.2, -2.0, CHASSIS_BOTTOM_Z1 + 0.2)
    ])
    assert not chassis.contains(strap_probes).any(), \
        "centre Velcro strap / alternate battery-lead passage was blocked"

    print(
        "  chassis bottom: former 14 x 22 mm battery-trunk port closed; "
        "centre Velcro strap passage retained"
    )
    return {
        "former_trunk_hole_closed": True,
        "former_trunk_hole_x_mm": hp.BATTERY_TRUNK_HOLE_X,
        "former_trunk_hole_y_mm": hp.BATTERY_TRUNK_HOLE_Y,
        "solid_probes": int(filled.sum()),
        "centre_strap_passage_retained": True,
    }


def check_clamp_cap_captive_nuts(
    meshes: dict[str, trimesh.Trimesh],
) -> dict:
    """Validate every face-loaded nut and its complete M3x8 path."""
    joints = (
        (
            "hip",
            meshes["coxa_link_ovh"],
            meshes["hip_clamp_cap_ovh"],
            HIP_CAP_TO_COXA,
            hp.servo_clamp_bolt_centres(),
        ),
        (
            "knee",
            meshes["femur_ovh_body"],
            meshes["knee_clamp_cap_ovh"],
            KNEE_CAP_TO_FEMUR,
            [
                centre for centre in hp.servo_clamp_bolt_centres()
                if centre[0] > 0.0
            ],
        ),
    )
    pocket_mid_y = 0.5 * (
        CLAMP_CAP_NUT_INNER_Y + CLAMP_CAP_WALL_FACE_Y
    )
    shell_probe_offset = (
        CLAMP_CAP_NUT_AF / 2.0 + CLAMP_CAP_LATERAL_SHELL / 2.0
    )

    for joint_name, cradle, cap, well_to_part, bolt_centres in joints:
        for index, (bx, bz) in enumerate(bolt_centres):
            # The whole under-head M3 shaft path is clearance through the cap,
            # nut and former self-tap pilot, ending 0.2 mm past the screw tip.
            shaft = _cyl_y(
                CLAMP_CAP_M3_D / 2.0 - 0.05,
                CLAMP_CAP_SCREW_TIP_Y - 0.10,
                CLAMP_CAP_WALL_FACE_Y + 0.05,
                x=bx,
                z=bz,
            )
            shaft.apply_transform(well_to_part)
            assert _inter_vol(cradle, shaft) < 0.02, (
                f"{joint_name} clamp screw path {index} is blocked"
            )

            pocket = _hex_y(
                CLAMP_CAP_NUT_AF - 0.10,
                CLAMP_CAP_NUT_INNER_Y + 0.05,
                CLAMP_CAP_WALL_FACE_Y + 0.05,
                x=bx,
                z=bz,
                flat_normal_angle_rad=0.0,
            )
            pocket.apply_transform(well_to_part)
            assert _inter_vol(cradle, pocket) < 0.02, (
                f"{joint_name} clamp nut pocket {index} is obstructed"
            )

            # Probe the middle of the remaining printed shell on both sides
            # of the correctly oriented hex flats. This catches a pocket that
            # breaks into either the servo cavity or the outer wall.
            local_shell_points = np.array([
                [bx - shell_probe_offset, pocket_mid_y, bz, 1.0],
                [bx + shell_probe_offset, pocket_mid_y, bz, 1.0],
            ])
            part_shell_points = (
                well_to_part @ local_shell_points.T
            ).T[:, :3]
            assert cradle.contains(part_shell_points).all(), (
                f"{joint_name} clamp nut pocket {index} breaks its side shell"
            )

            # The removable cap's existing bore must remain open from its
            # recessed head seat to the cradle mating face.
            cap_path = _cyl_y(
                CLAMP_CAP_M3_D / 2.0 - 0.05,
                CLAMP_CAP_WALL_FACE_Y - 0.05,
                CLAMP_CAP_HEAD_SEAT_Y + 0.10,
                x=bx,
                z=bz,
            )
            assert _inter_vol(cap, cap_path) < 0.02, (
                f"{joint_name} clamp cap screw bore {index} is blocked"
            )

    bearing_to_clamp_edge_clearances = []
    for carrier_x, carrier_z in _bearing_carrier_screw_centres():
        for clamp_x, clamp_z in hp.servo_clamp_bolt_centres():
            centre_distance = math.hypot(
                carrier_x - clamp_x,
                carrier_z - clamp_z,
            )
            bearing_to_clamp_edge_clearances.append(
                centre_distance
                - BEARING_CARRIER_CSK_D / 2.0
                - hp.CLAMP_HEAD_CB_OD / 2.0
            )
    min_bearing_to_clamp_edge = min(bearing_to_clamp_edge_clearances)
    assert min_bearing_to_clamp_edge >= 0.80, (
        "upper-bearing and hip-clamp screw head pockets are too close: "
        f"{min_bearing_to_clamp_edge:.2f} mm edge clearance"
    )

    result = {
        "converted_joints_per_robot": 12,
        "captive_m3_nuts_per_hip": 2,
        "captive_m3_nuts_per_knee": 1,
        "captive_m3_nuts_per_robot": 18,
        "screw": "M3x8 SHCS",
        "nut_pocket_across_flats_mm": CLAMP_CAP_NUT_AF,
        "nut_pocket_depth_mm": CLAMP_CAP_NUT_DEPTH,
        "minimum_lateral_shell_each_side_mm": round(
            CLAMP_CAP_LATERAL_SHELL, 2
        ),
        "conservative_thread_beyond_nut_mm": round(
            CLAMP_CAP_THREAD_BEYOND_NUT, 2
        ),
        "minimum_upper_bearing_to_clamp_head_edge_mm": round(
            min_bearing_to_clamp_edge, 2
        ),
        "servo_case_self_tapping_screws_changed": False,
        "knee_vertical_retention_screw_changed": False,
    }
    print(
        "  servo clamp caps: 18 self-tapped plastic threads replaced by "
        f"face-loaded M3 nuts; M3x8 reaches {CLAMP_CAP_THREAD_BEYOND_NUT:.1f} "
        f"mm beyond each pocket; bearing/clamp heads clear by "
        f"{min_bearing_to_clamp_edge:.1f} mm"
    )
    return result


def check_femur_servo_fit(meshes: dict[str, trimesh.Trimesh]) -> dict:
    """The final six-nut femur body must not occupy the servo envelope."""
    transforms = base.leg_transforms(0)
    femur = base._placed(
        meshes,
        "femur_ovh_body",
        transforms["femur"] @ base.MH,
    )
    servo = base._placed(meshes, "servo_body", transforms["knee_cap"])
    cap = base._placed(
        meshes,
        "knee_clamp_cap_ovh",
        transforms["knee_cap"],
    )
    bracket = base._placed(
        meshes,
        "chorn_clamp_cnc",
        transforms["femur"] @ base.MH,
    )
    overlap = _inter_vol(femur, servo)
    assert overlap < 0.05, (
        f"knee servo does not fit the six-hole femur receiver: "
        f"{overlap:.2f} mm3 overlap"
    )
    cap_body_overlap = _inter_vol(cap, femur)
    cap_bracket_overlap = _inter_vol(cap, bracket)
    cap_servo_press = _inter_vol(cap, servo)
    assert cap_body_overlap < 0.05, (
        f"knee cap overlaps femur receiver by {cap_body_overlap:.2f} mm3"
    )
    assert cap_bracket_overlap < 0.05, (
        f"knee cap overlaps bought bracket by {cap_bracket_overlap:.2f} mm3"
    )
    assert 650.0 < cap_servo_press < 900.0, (
        f"knee cap servo preload changed unexpectedly: "
        f"{cap_servo_press:.1f} mm3"
    )
    result = {
        "servo_receiver_overlap_mm3": round(overlap, 4),
        "cap_receiver_overlap_mm3": round(cap_body_overlap, 4),
        "cap_bracket_overlap_mm3": round(cap_bracket_overlap, 4),
        "cap_servo_intentional_press_mm3": round(cap_servo_press, 2),
        "receiver_thickness_mm": round(FEMUR_RECEIVER_T, 2),
        "retention": "6x front-loaded ordinary captive M3 hex nuts",
        "screw": f"M3x{FEMUR_FRONT_SCREW_LENGTH:g}",
        "nut_pocket_across_flats_mm": FEMUR_NUT_AF,
        "nut_pocket_depth_mm": FEMUR_NUT_DEPTH,
        "pocket_back_skin_mm": round(FEMUR_POCKET_BACK_SKIN, 2),
        "servo_side_skin_mm": round(FEMUR_SERVO_SIDE_SKIN, 2),
        "thread_beyond_nut_mm": round(FEMUR_THREAD_BEYOND_NUT, 2),
        "center_to_nut_flat_web_mm": round(
            FEMUR_CENTER_TO_NUT_FLAT_WEB, 2
        ),
    }
    print(
        "  femur servo fit: 0.00 mm3 servo/receiver and cap/interface "
        "overlap; "
        f"{FEMUR_RECEIVER_T:.1f} mm wall with 6x "
        f"{FEMUR_NUT_DEPTH:.1f} mm front-loaded M3 hex pockets; "
        f"M3x{FEMUR_FRONT_SCREW_LENGTH:g} reaches "
        f"{FEMUR_THREAD_BEYOND_NUT:.1f} mm beyond each nut"
    )
    return result


def _assembled_hip_cap_meshes(
    meshes: dict[str, trimesh.Trimesh],
) -> dict[str, trimesh.Trimesh]:
    """Compatibility view for inherited checks that expect one cap mesh."""
    assembled = dict(meshes)
    assembled["hip_clamp_cap_ovh"] = trimesh.util.concatenate([
        meshes["hip_clamp_cap_ovh"],
        meshes["hip_bearing_carrier_ovh"],
    ])
    return assembled


def _assembled_coxa_meshes(
    meshes: dict[str, trimesh.Trimesh],
) -> dict[str, trimesh.Trimesh]:
    """Compatibility view for checks that expect the yaw hub fused on."""
    assembled = dict(meshes)
    assembled["coxa_link_ovh"] = _union(
        meshes["coxa_link_ovh"],
        meshes["coxa_yaw_hub_carrier_ovh"],
    )
    return assembled


def check_split_lower_yaw_hub(
    meshes: dict[str, trimesh.Trimesh],
) -> dict:
    """Validate the deep collar/socket and three radial screw paths."""
    main = meshes["coxa_link_ovh"]
    hub = meshes["coxa_yaw_hub_carrier_ovh"]
    assert main.is_watertight and hub.is_watertight
    assert main.body_count == 1 and hub.body_count == 1
    assert abs(float(main.bounds[0, 2]) - COXA_HUB_INTERFACE_Z) < 0.02, \
        "reinforced coxa body lost its main Z=4 print face"
    assert abs(float(hub.bounds[1, 2]) - COXA_HUB_COLLAR_Z1) < 0.02, \
        "lower yaw carrier collar does not reach its full socket depth"
    interface_overlap = _inter_vol(main, hub)
    assert interface_overlap < 0.25, (
        f"split lower yaw parts overlap by {interface_overlap:.2f} mm3"
    )

    radial_clearance = COXA_HUB_SOCKET_R0 - COXA_HUB_COLLAR_R1
    assert abs(radial_clearance - COXA_HUB_RADIAL_CLEARANCE) < 1e-9
    collar_height = COXA_HUB_COLLAR_Z1 - COXA_HUB_INTERFACE_Z
    assert collar_height >= 8.0

    # Sample intact sectors between the screws to prove both annuli survived
    # the booleans and remain tied into their parent pieces.
    for angle in (0.0, math.radians(60.0), math.radians(300.0)):
        unit = np.array([math.cos(angle), math.sin(angle)])
        socket_probe = np.r_[
            unit * ((COXA_HUB_SOCKET_R0 + COXA_HUB_SOCKET_R1) / 2.0),
            COXA_HUB_COLLAR_Z1 - 0.5,
        ]
        collar_probe = np.r_[
            unit * ((COXA_HUB_COLLAR_R0 + COXA_HUB_COLLAR_R1) / 2.0),
            COXA_HUB_COLLAR_Z1 - 0.5,
        ]
        assert main.contains([socket_probe]).all(), \
            "coxa socket ring is not continuous"
        assert hub.contains([collar_probe]).all(), \
            "yaw-carrier collar is not continuous"

    radius = hp.DISC_HORN_BOLT_PCD / 2.0
    horn_stations = [(0.0, 0.0)] + [
        (radius * math.cos(angle), radius * math.sin(angle))
        for angle in hp.DISC_HORN_BOLT_ANGLES_RAD
    ]
    driver_outer_r = max(
        math.hypot(x, y) for x, y in horn_stations
    ) + hp.YAW_HUB_HORN_HEAD_CB_OD / 2.0
    assert COXA_HUB_HEAD_SEAT_R - COXA_HUB_HEAD_D / 2.0 > driver_outer_r

    for index, angle in enumerate(COXA_HUB_SCREW_ANGLES):
        main_shaft_probe = _cyl_radial_xy(
            COXA_HUB_M3_D / 2.0 - 0.12,
            COXA_HUB_COLLAR_R1 - 0.1,
            COXA_HUB_HEAD_SEAT_R + 0.15,
            angle,
            z=COXA_HUB_SCREW_Z,
        )
        head_access_probe = _cyl_radial_xy(
            COXA_HUB_HEAD_D / 2.0 - 0.12,
            COXA_HUB_HEAD_SEAT_R + 0.1,
            COXA_HUB_HEAD_ACCESS_R1 - 0.1,
            angle,
            z=COXA_HUB_SCREW_Z,
        )
        hub_shaft_probe = _cyl_radial_xy(
            COXA_HUB_M3_D / 2.0 - 0.12,
            COXA_HUB_COLLAR_R0 - 0.1,
            COXA_HUB_COLLAR_R1 + 0.1,
            angle,
            z=COXA_HUB_SCREW_Z,
        )
        nut_probe = _hex_radial_xy(
            COXA_HUB_NUT_AF - 0.15,
            COXA_HUB_COLLAR_R1 - COXA_HUB_NUT_DEPTH + 0.1,
            COXA_HUB_COLLAR_R1 + 0.1,
            angle,
            z=COXA_HUB_SCREW_Z,
        )
        assert _inter_vol(main, main_shaft_probe) < 0.02, \
            f"radial lower-hub screw shaft {index} blocked in coxa"
        assert _inter_vol(main, head_access_probe) < 0.02, \
            f"radial lower-hub screw head tunnel {index} blocked"
        assert _inter_vol(hub, hub_shaft_probe) < 0.02, \
            f"radial lower-hub screw shaft {index} blocked in carrier"
        assert _inter_vol(hub, nut_probe) < 0.02, \
            f"radial lower-hub nut pocket {index} blocked"

        radial_unit = np.array([math.cos(angle), math.sin(angle)])
        tangent_unit = np.array([-math.sin(angle), math.cos(angle)])
        back_r = (
            COXA_HUB_COLLAR_R1 - COXA_HUB_NUT_DEPTH - 0.2
        )
        back_ring_r = (
            COXA_HUB_M3_D / 2.0 + COXA_HUB_NUT_AF / 2.0
        ) / 2.0
        back_ring = []
        for cross_angle in np.linspace(
            0.0, 2.0 * math.pi, 12, endpoint=False
        ):
            xy = (
                radial_unit * back_r
                + tangent_unit * back_ring_r * math.cos(cross_angle)
            )
            back_ring.append([
                xy[0],
                xy[1],
                COXA_HUB_SCREW_Z
                + back_ring_r * math.sin(cross_angle),
            ])
        assert hub.contains(np.asarray(back_ring)).all(), \
            f"radial lower-hub nut pocket {index} has no back wall"

    nut_back_shell = (
        COXA_HUB_COLLAR_R1 - COXA_HUB_NUT_DEPTH - COXA_HUB_COLLAR_R0
    )
    nut_vertical_shell = min(
        COXA_HUB_SCREW_Z - COXA_HUB_NUT_AF / 2.0
        - (COXA_HUB_INTERFACE_Z - 0.4),
        COXA_HUB_COLLAR_Z1
        - (COXA_HUB_SCREW_Z + COXA_HUB_NUT_AF / 2.0),
    )
    head_shoulder = COXA_HUB_HEAD_SEAT_R - COXA_HUB_SOCKET_R0
    thread_beyond_nut = (
        COXA_HUB_SCREW_LENGTH
        - (COXA_HUB_HEAD_SEAT_R - COXA_HUB_COLLAR_R1)
        - COXA_HUB_NUT_DEPTH
    )
    assert nut_back_shell >= 0.85
    assert nut_vertical_shell >= 1.0
    assert head_shoulder >= 1.0
    assert thread_beyond_nut >= 1.0

    bearing = meshes["yaw_bearing_upper"].copy()
    bearing.apply_transform(base._trans([0.0, 0.0, rv.YAWBR_DROP]))
    bearing_press = _inter_vol(hub, bearing)
    assert 20.0 < bearing_press < 80.0, (
        f"lower 6805 press geometry changed: {bearing_press:.2f} mm3"
    )
    result = {
        "interface_z_mm": COXA_HUB_INTERFACE_Z,
        "main_flat_print_face_z_mm": round(float(main.bounds[0, 2]), 2),
        "carrier_collar_z_range_mm": [
            COXA_HUB_INTERFACE_Z, COXA_HUB_COLLAR_Z1
        ],
        "carrier_collar_radii_mm": [
            COXA_HUB_COLLAR_R0, COXA_HUB_COLLAR_R1
        ],
        "coxa_socket_radii_mm": [
            COXA_HUB_SOCKET_R0, COXA_HUB_SOCKET_R1
        ],
        "radial_assembly_clearance_mm": radial_clearance,
        "interface_overlap_mm3": round(interface_overlap, 4),
        "attachment": "3x radial M3x6 low-profile SHCS into captive nuts",
        "radial_screw_angles_deg": [
            round(math.degrees(angle), 1) for angle in COXA_HUB_SCREW_ANGLES
        ],
        "nut_pocket_across_flats_mm": COXA_HUB_NUT_AF,
        "nut_pocket_depth_mm": COXA_HUB_NUT_DEPTH,
        "minimum_nut_back_shell_mm": round(nut_back_shell, 2),
        "minimum_nut_vertical_shell_mm": round(nut_vertical_shell, 2),
        "head_seat_shoulder_mm": round(head_shoulder, 2),
        "thread_beyond_nut_mm": round(thread_beyond_nut, 2),
        "bearing_press_overlap_mm3": round(bearing_press, 2),
    }
    print(
        "  deep lower yaw carrier: "
        f"{collar_height:.1f} mm collar, {radial_clearance:.2f} mm radial gap, "
        "3x edge-entry M3x6 + captive nuts"
    )
    return result


def check_split_hip_bearing(
    meshes: dict[str, trimesh.Trimesh],
) -> dict:
    """Validate the flat cap and its screw-on upper-bearing cartridge."""
    cap = meshes["hip_clamp_cap_ovh"]
    carrier = meshes["hip_bearing_carrier_ovh"]
    assert cap.is_watertight and carrier.is_watertight
    assert cap.bounds[1, 1] <= BEARING_CARRIER_BASE_Y + 0.02, (
        f"hip cap is not flat-backed: max Y {cap.bounds[1, 1]:.2f}"
    )
    assert abs(carrier.bounds[0, 1] - BEARING_CARRIER_BASE_Y) < 0.02, \
        "bearing carrier does not have the intended flat mounting face"
    cap_carrier_overlap = _inter_vol(cap, carrier)
    assert cap_carrier_overlap < 0.05, (
        f"flat cap and bearing carrier overlap by {cap_carrier_overlap:.2f} mm3"
    )

    for index, (x, z) in enumerate(_bearing_carrier_screw_centres()):
        open_path = np.array([
            [x, y, z]
            for y in np.linspace(
                BEARING_CARRIER_CAP_PAD_Y0 - 0.2,
                BEARING_CARRIER_BASE_Y
                + BEARING_CARRIER_TIP_CLEARANCE_DEPTH - 0.1,
                48,
            )
        ])
        assert not (cap.contains(open_path) | carrier.contains(open_path)).any(), \
            f"bearing-carrier screw path {index} is blocked"
        back_ring_r = (
            BEARING_CARRIER_M3_D / 2.0
            + BEARING_CARRIER_NUT_AF / 2.0
        ) / 2.0
        back_ring = np.array([
            [
                x + back_ring_r * math.cos(angle),
                BEARING_CARRIER_BASE_Y + BEARING_CARRIER_NUT_DEPTH + 0.2,
                z + back_ring_r * math.sin(angle),
            ]
            for angle in np.linspace(0.0, 2.0 * math.pi, 12, endpoint=False)
        ])
        assert carrier.contains(back_ring).all(), \
            f"bearing-carrier nut pocket {index} has no closed back wall"

        radial_angle = math.atan2(
            z - BEARING_CARRIER_CENTER_Z,
            x - BEARING_CARRIER_CENTER_X,
        )
        nut_probe = _hex_y(
            BEARING_CARRIER_NUT_AF - 0.15,
            BEARING_CARRIER_BASE_Y - 0.1,
            BEARING_CARRIER_BASE_Y + BEARING_CARRIER_NUT_DEPTH - 0.1,
            x=x,
            z=z,
            flat_normal_angle_rad=radial_angle,
        )
        assert _inter_vol(carrier, nut_probe) < 0.02, \
            f"bearing-carrier nut pocket {index} is obstructed"

    nut_shell = (
        BEARING_CARRIER_R - BEARING_CARRIER_SCREW_R
        - BEARING_CARRIER_NUT_AF / 2.0
    )
    assert nut_shell >= 0.55, \
        f"bearing-carrier nut pockets leave only {nut_shell:.2f} mm shell"

    transforms = base.leg_transforms(0)
    placed_carrier = base._placed(
        meshes, "hip_bearing_carrier_ovh", transforms["hip_cap"]
    )
    bearing = base._placed(meshes, "bearing_6805", transforms["yaw_top"])
    bearing_press = _inter_vol(placed_carrier, bearing)
    assert 0.05 < bearing_press < 2.0, (
        f"upper bearing press geometry changed: {bearing_press:.2f} mm3"
    )
    for key in ("chassis_top_rigid", "top_hatch_rigid"):
        overlap = _inter_vol(placed_carrier, meshes[key])
        assert overlap < 0.05, \
            f"bearing carrier intersects {key} by {overlap:.2f} mm3"

    result = {
        "cap_flat_face_y_mm": round(float(cap.bounds[1, 1]), 2),
        "carrier_flat_face_y_mm": round(float(carrier.bounds[0, 1]), 2),
        "cap_carrier_overlap_mm3": round(cap_carrier_overlap, 4),
        "bearing_press_overlap_mm3": round(bearing_press, 3),
        "attachment": (
            f"{len(BEARING_CARRIER_SCREW_ANGLES)}x M3x8 countersunk screws "
            "into captive M3 nuts"
        ),
        "nut_pocket_across_flats_mm": BEARING_CARRIER_NUT_AF,
        "nut_pocket_depth_mm": BEARING_CARRIER_NUT_DEPTH,
        "minimum_nut_to_outer_shell_mm": round(nut_shell, 2),
        "mount_radius_mm": BEARING_CARRIER_R,
    }
    print(
        "  split upper bearing: flat hip cap + flat-backed carrier, "
        f"{len(BEARING_CARRIER_SCREW_ANGLES)}x M3x8 CSK + captive nuts, "
        f"bearing press {bearing_press:.2f} mm3"
    )
    return result


def check_integral_coxa_tower(meshes: dict[str, trimesh.Trimesh]) -> dict:
    """Check tower integration, top fasteners, service access, and clearance."""
    tower = make_integral_tower_body()
    coxa = meshes["coxa_link_ovh"]
    cap = meshes["hip_clamp_cap_ovh"].copy()
    cap.apply_transform(HIP_CAP_TO_COXA)
    carrier = meshes["hip_bearing_carrier_ovh"].copy()
    carrier.apply_transform(HIP_CAP_TO_COXA)
    rails = make_coxa_reinforcement_rails()
    assert rails.body_count == 3, (
        "expected one positive-Y tab block plus two negative-Y arms, "
        f"got {rails.body_count} reinforcement bodies"
    )
    buttress_probes = np.array([
        [-10.0, side * 18.0, 40.0]
        for side in (-1.0, 1.0)
    ])
    assert coxa.contains(buttress_probes).all(), \
        "upper bearing-root buttress is not continuous with the coxa"
    holder_foot = make_centered_servo_holder_foot()
    holder_foot_centre_y = (
        COXA_HOLDER_FOOT_Y0 + COXA_HOLDER_FOOT_Y1
    ) / 2.0
    holder_centre_error = abs(
        holder_foot_centre_y - COXA_HOLDER_SERVO_CENTRE_Y
    )
    assert holder_centre_error <= 0.21, (
        f"servo-holder foot is {holder_centre_error:.2f} mm off centre"
    )
    positive_support_outer_y = COXA_REINF_TAB_BLOCK_Y1
    assert positive_support_outer_y <= COXA_HOLDER_FOOT_Y1, (
        "positive-Y tab block still protrudes beyond the centered holder"
    )
    servo_in_coxa = meshes["servo_body"].copy()
    servo_in_coxa.apply_transform(HIP_CAP_TO_COXA)
    assert _inter_vol(holder_foot, servo_in_coxa) < 0.05, \
        "centered low holder foot enters the seated servo envelope"
    tab_block = _positive_servo_tab_block()
    assert tab_block.bounds[1, 0] <= COXA_REINF_TAB_BLOCK_X1 + 0.02, \
        "positive-Y support continues past the servo retention tab"
    for index, z in enumerate(COXA_REINF_TAB_HOLE_ZS):
        driver_line = np.array([
            [COXA_REINF_TAB_HOLE_X, y, z]
            for y in np.linspace(
                COXA_REINF_TAB_BLOCK_Y0 - 0.5,
                COXA_REINF_TAB_BLOCK_Y1 + 0.5,
                64,
            )
        ])
        assert not coxa.contains(driver_line).any(), \
            f"servo retention-tab driver path {index} is blocked"

    # The tower is one connected print with the coxa and its top horizontal
    # face sits just below the matching flange on the separate hip cap.
    assert coxa.body_count == 1, "integral tower disconnected from coxa"
    cap_coxa_overlap = _inter_vol(coxa, cap)
    assert cap_coxa_overlap < 0.05, (
        f"hip cap still intersects the coxa by {cap_coxa_overlap:.2f} mm3"
    )
    assert _inter_vol(tower, cap) < 0.05, "coxa tower overlaps hip cap"
    assert _inter_vol(tower, carrier) < 0.05, \
        "coxa tower overlaps screw-on bearing carrier"
    seat_gap = float(carrier.bounds[0, 2] - tower.bounds[1, 2])
    assert 0.0 <= seat_gap <= 0.15, f"tower/cap seat gap is {seat_gap:.2f} mm"

    for part_name, part in (("tower", tower), ("bearing carrier", carrier)):
        radial = np.hypot(part.vertices[:, 0], part.vertices[:, 1])
        assert radial.max() <= TOWER_OUTER_R + 0.02, \
            f"{part_name} exceeds lower coxa plan envelope"
    assert tower.bounds[1, 0] <= TOWER_FRONT_X + 0.02, \
        "tower extends in front of rear contour chord"

    # The pre-existing under-plate cap-driver axis is now completely behind
    # the contour-matched tower, so it remains open without a tunnel.
    cap_driver_line = np.array([
        [TOWER_CAP_DRIVER_X, TOWER_CAP_DRIVER_Y, z]
        for z in np.linspace(TOWER_BOTTOM_Z - 0.2,
                             TOWER_CAP_FLANGE_Z1 + 10.0, 64)
    ])
    assert not (coxa.contains(cap_driver_line) |
                cap.contains(cap_driver_line)).any(), \
        "contoured support blocks the existing hip-cap screwdriver path"

    # Both top screw centrelines are open through the bearing carrier and the
    # top-loaded captive-nut pockets in the integral coxa tower.
    for index, y in enumerate(TOWER_SCREW_YS):
        line = np.array([
            [TOWER_SCREW_X, y, z] for z in np.linspace(
                TOWER_CAP_FLANGE_Z1 + 10.0,
                TOWER_TOP_Z - TOWER_TIP_CLEARANCE_DEPTH + 0.1,
                64,
            )
        ])
        assert not coxa.contains(line).any(), \
            f"tower screw path {index} blocked in coxa"
        assert not cap.contains(line).any(), \
            f"tower screw path {index} blocked in cap"
        assert not carrier.contains(line).any(), \
            f"tower screw path {index} blocked in bearing carrier"

        radial_angle = math.atan2(y, TOWER_SCREW_X)
        nut_probe = _hex_z(
            TOWER_NUT_AF - 0.15,
            TOWER_TOP_Z - TOWER_NUT_DEPTH + 0.1,
            TOWER_TOP_Z + 0.1,
            x=TOWER_SCREW_X,
            y=y,
            flat_normal_angle_rad=radial_angle,
        )
        assert _inter_vol(coxa, nut_probe) < 0.02, \
            f"tower captive-nut pocket {index} is obstructed"

    tower_nut_radius = math.hypot(TOWER_SCREW_X, TOWER_SCREW_YS[0])
    tower_nut_shell = (
        TOWER_OUTER_R - tower_nut_radius - TOWER_NUT_AF / 2.0
    )
    assert tower_nut_shell >= 0.80, \
        f"tower nut pockets leave only {tower_nut_shell:.2f} mm shell"

    # Existing yaw service is a centre shaft plus four shafts on the horn PCD.
    # The rear one gets a contour scallop with a deliberate 2 mm radial margin.
    radius = hp.DISC_HORN_BOLT_PCD / 2.0
    stations = [(0.0, 0.0)] + [
        (radius * math.cos(a), radius * math.sin(a))
        for a in hp.DISC_HORN_BOLT_ANGLES_RAD
    ]
    driver_r = hp.YAW_HUB_HORN_HEAD_CB_OD / 2.0
    for x, y in stations:
        envelope = _cyl_z(driver_r, -10.0, 80.0, x=x, y=y)
        assert _inter_vol(tower, envelope) < 1e-6, \
            "integral tower blocks a yaw-horn screwdriver shaft"
        assert _inter_vol(rails, envelope) < 1e-6, \
            "coxa reinforcement blocks a yaw-horn screwdriver shaft"

    # The former lower full-width rail, nine-hole grid, and two screw tongues
    # are gone. The centre remains open inside the narrow socket/collar pair.
    centre_void = _cyl_z(
        4.0,
        COXA_REINF_LO_Z0 + 0.2,
        COXA_REINF_LO_Z1 - 0.2,
    )
    assert _inter_vol(coxa, centre_void) < 0.05, \
        "lower yaw-hub centre plate was not fully removed"

    # Preserve the upper inherited front-case screw line through both side
    # ribs.  This is the hole the previous one-sided upper rail obscured.
    access_line = np.array([
        [COXA_REINF_ACCESS_X, y, COXA_REINF_ACCESS_Z]
        for y in np.linspace(
            -COXA_REINF_HI_CURVE_R1 - 0.5,
            COXA_REINF_HI_CURVE_R1 + 0.5,
            80,
        )
    ])
    assert not coxa.contains(access_line).any(), \
        "upper contour-arm front-case screw access is blocked"

    # The integral tower sits close to the chassis side of the yaw stack;
    # verify all six copies remain outside every world-fixed shell.
    for leg in range(6):
        transforms = base.leg_transforms(leg)
        placed = tower.copy()
        placed.apply_transform(transforms["coxa"])
        for fixed_key in ("chassis_bottom", "chassis_top_rigid",
                          "top_hatch_rigid"):
            volume = _inter_vol(placed, meshes[fixed_key])
            assert volume < 1.0, \
                f"L{leg} rear column hits {fixed_key}: {volume:.1f} mm3"

    result = {
        "integrated_into_coxa": True,
        "footprint": "rear circular segment matching lower coxa",
        "outer_envelope_radius_mm": TOWER_OUTER_R,
        "front_chord_x_mm": TOWER_FRONT_X,
        "height_mm": TOWER_TOP_Z - TOWER_BOTTOM_Z,
        "cap_coxa_overlap_mm3": round(cap_coxa_overlap, 4),
        "cap_seat_gap_mm": round(seat_gap, 2),
        "top_entry_m3_screws_per_leg": len(TOWER_SCREW_YS),
        "top_captive_nuts_per_leg": len(TOWER_SCREW_YS),
        "top_nut_pocket_across_flats_mm": TOWER_NUT_AF,
        "top_nut_pocket_depth_mm": TOWER_NUT_DEPTH,
        "minimum_top_nut_to_outer_shell_mm": round(tower_nut_shell, 2),
        "yaw_driver_envelope_diameter_mm": hp.YAW_HUB_HORN_HEAD_CB_OD,
        "rear_yaw_driver_extra_radial_clearance_mm": TOWER_YAW_DRIVER_EXTRA_R,
        "reinforcement": (
            "solid positive-Y servo-tab block, two negative-Y arms, and "
            "two rear upper-bearing root buttresses"
        ),
        "reinforcement_arm_count": rails.body_count,
        "upper_arm_screw_access_diameter_mm": COXA_REINF_ACCESS_D,
        "servo_holder_foot_y_range_mm": [
            COXA_HOLDER_FOOT_Y0, COXA_HOLDER_FOOT_Y1
        ],
        "servo_holder_foot_center_y_mm": round(holder_foot_centre_y, 2),
        "servo_center_y_mm": COXA_HOLDER_SERVO_CENTRE_Y,
        "servo_holder_center_error_mm": round(holder_centre_error, 2),
        "positive_tab_block_inside_holder_outline_mm": round(
            COXA_HOLDER_FOOT_Y1 - positive_support_outer_y, 2
        ),
        "positive_tab_block_x_range_mm": [
            COXA_REINF_TAB_BLOCK_X0, COXA_REINF_TAB_BLOCK_X1
        ],
        "positive_tab_block_y_range_mm": [
            COXA_REINF_TAB_BLOCK_Y0, COXA_REINF_TAB_BLOCK_Y1
        ],
        "positive_tab_block_z_range_mm": [
            COXA_REINF_TAB_BLOCK_Z0, COXA_REINF_TAB_BLOCK_Z1
        ],
        "positive_tab_driver_diameter_mm": COXA_REINF_TAB_DRIVER_D,
        "lower_rail_z_range_mm": [COXA_REINF_LO_Z0, COXA_REINF_LO_Z1],
        "upper_rail_z_range_mm": [COXA_REINF_HI_Z0, COXA_REINF_HI_Z1],
        "upper_bearing_buttress_z_range_mm": [
            COXA_BEARING_BUTTRESS_Z0,
            COXA_BEARING_BUTTRESS_Z1,
        ],
        "upper_bearing_buttress_outer_radius_mm": (
            COXA_BEARING_BUTTRESS_R1
        ),
        "upper_bearing_buttress_front_x_mm": COXA_BEARING_BUTTRESS_X1,
        "reinforcement_volume_mm3": round(float(rails.volume), 1),
    }
    print(
        "  integral coxa tower: "
        f"rear R{TOWER_OUTER_R:.1f} contour, x <= {TOWER_FRONT_X:.1f}, "
        f"{result['height_mm']:.2f} mm straight rise, solid tab block, "
        "paired upper-bearing root buttresses, "
        f"centered holder within {holder_centre_error:.2f} mm, "
        "2x top-entry M3 + captive nuts, "
        f"{TOWER_YAW_DRIVER_EXTRA_R:.1f} mm rear-driver scallop margin"
    )
    return result


def check_tibia_screw_access(meshes: dict[str, trimesh.Trimesh]) -> dict:
    """Verify access to all six receiver nuts and the full mouth collar."""
    socket = meshes["tibia_ovh_socket"]

    engagement = TIB_MOUTH_X - TIB_BORE_X0
    assert abs(engagement - TIB_TUBE_ENGAGEMENT) < 0.01
    bore_axis = np.array(
        [
            [x, 0.0, BRACKET_MID_Z]
            for x in np.linspace(TIB_BORE_X0, TIB_MOUTH_X + 0.3, 120)
        ]
    )
    assert not socket.contains(bore_axis).any(), \
        "long tibia tube bore is obstructed"

    # The four central sites get side-loading U-windows.  Probe slightly
    # inside each authored cut so coincident boolean faces do not count as
    # interference.
    probe_width = TIB_ACCESS_W - 0.4
    probe_x0 = TIB_ACCESS_X0 + 0.2
    probe_x1 = TIB_ACCESS_X1 - 0.2
    radial_outer = TIB_ACCESS_RADIAL_OUTER - 0.5
    for index, (y, z) in enumerate(_front_usual_m3_centres()):
        probes = [
            _cyl_x(probe_width / 2.0, probe_x0, probe_x1, y=y, z=z)
        ]
        dy = y
        dz = z - BRACKET_MID_Z
        if abs(dy) > abs(dz):
            outer_y = math.copysign(radial_outer, dy)
            y0, y1 = sorted((y, outer_y))
            probes.append(
                _box(
                    (probe_x1 - probe_x0, y1 - y0, probe_width),
                    (
                        (probe_x0 + probe_x1) / 2.0,
                        (y0 + y1) / 2.0,
                        z,
                    ),
                )
            )
        else:
            outer_z = BRACKET_MID_Z + math.copysign(radial_outer, dz)
            z0, z1 = sorted((z, outer_z))
            probes.append(
                _box(
                    (probe_x1 - probe_x0, probe_width, z1 - z0),
                    (
                        (probe_x0 + probe_x1) / 2.0,
                        y,
                        (z0 + z1) / 2.0,
                    ),
                )
            )
        obstruction = sum(_inter_vol(socket, probe) for probe in probes)
        assert obstruction < 0.05, (
            f"tibia central screw window {index} blocked by "
            f"{obstruction:.2f} mm3"
        )

    # The two outer-pair pockets are beyond the boss radius and must remain
    # open directly behind the receiver.
    for index, (y, z) in enumerate(_front_extra_m3_centres()):
        pocket_probe = _hex_x(
            NYLOC_AF - 0.2,
            RECEIVER_X1 - NYLOC_POCKET_DEPTH + 0.2,
            RECEIVER_X1 + 0.8,
            y=y,
            z=z,
        )
        obstruction = _inter_vol(socket, pocket_probe)
        assert obstruction < 0.05, (
            f"tibia outer screw pocket {index} blocked by "
            f"{obstruction:.2f} mm3"
        )

    # The outer end remains a complete annulus, not four unsupported fingers.
    full_collar_len = TIB_MOUTH_X - TIB_ACCESS_X1
    assert full_collar_len >= 5.0
    collar_x = (TIB_ACCESS_X1 + TIB_MOUTH_X) / 2.0
    collar_points = []
    for radius in (TIB_SOCKET_BORE_R + 0.7, TIB_SOCKET_OUTER_R - 0.7):
        for angle in np.linspace(0.0, 2.0 * np.pi, 96, endpoint=False):
            collar_points.append(
                [
                    collar_x,
                    radius * math.cos(angle),
                    BRACKET_MID_Z + radius * math.sin(angle),
                ]
            )
    assert socket.contains(np.asarray(collar_points)).all(), \
        "tibia mouth no longer has a complete supporting collar"

    # The four diagonal sectors are the uninterrupted load paths around the
    # cardinal screw windows. Verify the tapered shoulder and its outer ribs
    # survived the final booleans at the receiver root.
    root_x = RECEIVER_X1 + 0.8
    root_points = []
    for angle in TIB_ROOT_RIB_ANGLES:
        for radius in (
            TIB_SOCKET_OUTER_R + 0.5,
            TIB_ROOT_FLARE_R - 0.6,
            TIB_ROOT_RIB_R1 - 0.6,
        ):
            root_points.append([
                root_x,
                radius * math.cos(angle),
                BRACKET_MID_Z + radius * math.sin(angle),
            ])
    assert socket.contains(np.asarray(root_points)).all(), \
        "tibia root flare/rib load path is discontinuous"

    result = {
        "accessible_screw_sites": 6,
        "radial_u_windows": 4,
        "naturally_exposed_outer_pockets": 2,
        "window_width_mm": TIB_ACCESS_W,
        "window_x_range_mm": [TIB_ACCESS_X0, TIB_ACCESS_X1],
        "tube_engagement_depth_mm": round(engagement, 2),
        "unsupported_tube_span_reduction_vs_original_mm": round(
            UNSUPPORTED_TUBE_REDUCTION, 2
        ),
        "complete_mouth_collar_length_mm": round(full_collar_len, 2),
        "root_flare_outer_diameter_mm": 2.0 * TIB_ROOT_FLARE_R,
        "root_flare_length_mm": TIB_ROOT_FLARE_LEN,
        "diagonal_root_ribs": len(TIB_ROOT_RIB_ANGLES),
        "root_rib_outer_radius_mm": TIB_ROOT_RIB_R1,
    }
    print(
        "  tibia screw access: 4 rounded radial nut windows + 2 clear outer "
        f"pockets; all 6 accessible; {engagement:.1f} mm tube engagement "
        f"({UNSUPPORTED_TUBE_REDUCTION:.1f} mm more support), "
        f"{full_collar_len:.2f} mm complete collar at mouth; "
        f"R{TIB_ROOT_FLARE_R:.1f} x {TIB_ROOT_FLARE_LEN:.1f} mm root flare "
        f"with {len(TIB_ROOT_RIB_ANGLES)} diagonal ribs"
    )
    return result


def check_horn_on_servo_extraction(
    meshes: dict[str, trimesh.Trimesh],
) -> dict:
    """Check the actual derived hip and knee holders, not a generic cradle.

    The production holder's 24 mm horn opening now continues through the +Y
    lip as a same-width slot.  Sweep a fitted 20 mm horn plus 1.5 mm radial
    running clearance from the seated position through the open clamp mouth.
    Running this against the final coxa/femur meshes catches stale ancestor
    BREP exports as well as a future boolean edit that restores the old bridge.
    """
    transforms = base.leg_transforms(0)
    stations = (
        (
            "hip",
            base._placed(meshes, "coxa_link_ovh", transforms["coxa"]),
            transforms["hip_cap"],
        ),
        (
            "knee",
            base._placed(
                meshes,
                "femur_ovh_body",
                transforms["femur"] @ base.MH,
            ),
            transforms["knee_cap"],
        ),
    )

    probe_radius = hp.DISC_HORN_OD / 2.0 + 1.5
    travel = hp.WELL_D + hp.SERVO_BODY_D
    probe_z0 = hp.WELL_RIM_Z + 0.6
    probe_z1 = hp.WELL_H - 0.6
    # Continuous swept envelope: the starting disc plus the straight portion
    # traced by its centre as it leaves in +Y.  It is inset 0.5 mm from the
    # nominal 24 mm slot walls and 0.6 mm from the lip faces, avoiding boolean
    # boundary noise while still requiring the stated 1.5 mm running margin.
    start_disc = _cyl_z(
        probe_radius,
        probe_z0,
        probe_z1,
        x=hp.SERVO_OUTPUT_X,
    )
    straight_sweep = _box(
        (2.0 * probe_radius, travel + probe_radius, probe_z1 - probe_z0),
        (
            hp.SERVO_OUTPUT_X,
            (travel + probe_radius) / 2.0,
            (probe_z0 + probe_z1) / 2.0,
        ),
    )
    local_sweep = _union(start_disc, straight_sweep)

    obstruction_by_station = {}
    for name, holder, well_transform in stations:
        probe_world = local_sweep.copy()
        probe_world.apply_transform(well_transform)
        obstruction = _inter_vol(holder, probe_world)
        obstruction_by_station[name] = round(obstruction, 4)
        assert obstruction < 0.05, (
            f"{name} holder traps fitted horn during +Y extraction: "
            f"{obstruction:.2f} mm3 of service-envelope obstruction"
        )

    result = {
        "holder_slot_width_mm": hp.HORN_SERVICE_SLOT_W,
        "fitted_horn_diameter_mm": hp.DISC_HORN_OD,
        "checked_radial_clearance_mm": 1.5,
        "extraction_direction": "+Y through open clamp mouth",
        "obstruction_volume_mm3": obstruction_by_station,
        "stations_checked": list(obstruction_by_station),
    }
    print(
        "  horn-on servo removal: hip + knee final holders clear through "
        f"the {hp.HORN_SERVICE_SLOT_W:.0f} mm +Y service slots "
        "(1.5 mm radial running clearance)"
    )
    return result


def check_joint_motion(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Bought bracket + receivers through the useful hip and knee ranges."""
    T0 = base.leg_transforms(0)
    fixed_hip = [
        base._placed(meshes, "servo_body", T0["hip_cap"]),
        base._placed(meshes, "hip_clamp_cap_ovh", T0["hip_cap"]),
        base._placed(meshes, "hip_bearing_carrier_ovh", T0["hip_cap"]),
        base._placed(meshes, "bearing_6805", T0["yaw_top"]),
        base._placed(meshes, "coxa_link_ovh", T0["coxa"]),
        base._placed(meshes, "coxa_yaw_hub_carrier_ovh", T0["coxa"]),
    ]
    first_hip_contact = None
    for angle in np.arange(30.0, -122.6, -2.5):
        T = base.leg_transforms(0, femur_deg=float(angle))
        moving = [
            base._placed(meshes, "chorn_clamp_cnc", T["femur"] @ base.MH),
            base._placed(meshes, "driven_spacer", T["femur"] @ base.MH),
            base._placed(meshes, "passive_spacer", T["femur"] @ base.MH),
            base._placed(meshes, "femur_ovh_body", T["femur"] @ base.MH),
        ]
        if any(_inter_vol(m, f) > 1.0 for m in moving for f in fixed_hip):
            first_hip_contact = float(angle)
            break
    assert first_hip_contact is None or first_hip_contact <= -45.0, \
        f"hip useful range lost; first contact {first_hip_contact}"

    knee_fixed = [
        base._placed(meshes, "femur_ovh_body", T0["femur"] @ base.MH),
        base._placed(meshes, "servo_body", T0["knee_cap"]),
        base._placed(meshes, "knee_clamp_cap_ovh", T0["knee_cap"]),
    ]
    for angle in np.arange(-30.0, 20.1, 5.0):
        T = base.leg_transforms(0, tibia_deg=float(angle))
        moving = [
            base._placed(meshes, "chorn_clamp_cnc", T["tibia"] @ base.MH),
            base._placed(meshes, "driven_spacer", T["tibia"] @ base.MH),
            base._placed(meshes, "passive_spacer", T["tibia"] @ base.MH),
            base._placed(meshes, "tibia_ovh_socket", T["tibia"] @ base.MH),
        ]
        for moving_part in moving:
            for fixed_part in knee_fixed:
                assert _inter_vol(moving_part, fixed_part) < 1.0, \
                    f"knee contact at {angle:+.1f} deg"
    print(f"  joint sweep: hip first own-stack contact {first_hip_contact}; "
          "knee -30..+20 clear")


def _mat16(matrix: np.ndarray) -> list[float]:
    return [float(value) for value in np.asarray(matrix, float).T.reshape(-1)]


def build_scene(meshes: dict[str, trimesh.Trimesh], limit: float) -> dict:
    """Adapt the scene; add 12 bearing carriers and 24 spacer pucks."""
    scene = base.build_scene(meshes, limit)
    scene["name"] = (
        "STS3215 experimental premade 56 mm C-horns + rigid top/bottom bearings"
    )
    scene["source"] = "concepts/premade_chorn_56/make_premade_chorn_56_variant.py"
    scene["designSpecUrl"] = "design_spec.yaml"
    # Use BuildViz's current BVH/penetration checker.  The inherited scene's
    # 80 mm3 voxel threshold and part-type blanket ignores hid the receiver /
    # servo collision.  This concept uses a sub-mm penetration gate and only
    # typed, instance-level allowances for named mechanical interfaces.
    scene["checksConfig"] = {
        "toleranceMm": 0.20,
        "minPenetrationMm": 0.25,
        "clearanceMm": 0.0,
        "minWallMm": 0.8,
        "minThreadEngagementMm": 2.0,
        # These two DO_NOT_PRINT reference meshes intentionally model separate
        # purchased subcomponents. Every printable mesh still defaults to one
        # connected body, so a detached print island is a hard failure.
        "expectedMeshComponents": {
            "stl:servo_body": 3,
            "stl:yaw_bearing_upper": 2,
        },
        "allowedInterferences": [],
    }
    scene["meshes"] = [
        {"id": f"stl:{key}", "name": filename, "url": f"stl/{filename}"}
        for key, filename in SCENE_MESH_FILES.items()
        if key != "front_pattern_coupon"
    ]
    for instance in scene["instances"]:
        if instance["partType"] == "chorn_clamp_cnc":
            instance["name"] = instance["name"].replace(
                "C-clamp (CNC 6061, NEW)", "premade 56 mm C-horn (NEW)"
            )
            instance["color"] = "#2f343b"
        elif instance["partType"] == "femur_ovh_body":
            instance["name"] = instance["name"].replace(
                "femur body OVH",
                "femur six-hole receiver + clamp nut traps",
            )
        elif instance["partType"] == "tibia_ovh_socket":
            instance["name"] = instance["name"].replace(
                "tibia socket OVH", "tibia six-hole receiver/socket"
            )
        elif instance["partType"] == "coxa_link_ovh":
            instance["name"] = instance["name"].replace(
                "coxa OVH outboard arm",
                "coxa OVH + yaw socket + clamp nut traps",
            )
        elif instance["partType"] == "hip_clamp_cap_ovh":
            instance["name"] = instance["name"].replace(
                "hip cap OVH + yaw pedestal",
                "flat hip clamp cap",
            )

    # The ancestor omitted parent links; add them so BuildViz FK carries the
    # knee chain with hip and the whole leg with yaw.
    joints = {joint["id"]: joint for joint in scene["joints"]}
    for leg in range(6):
        joints[f"L{leg}-hip"]["parent"] = f"L{leg}-yaw"
        joints[f"L{leg}-knee"]["parent"] = f"L{leg}-hip"

    next_id = len(scene["instances"])
    for leg in range(6):
        transforms = base.leg_transforms(leg)
        lower_carrier_id = f"{next_id:03d}-L{leg} deep lower yaw carrier"
        next_id += 1
        scene["instances"].append({
            "id": lower_carrier_id,
            "meshId": "stl:coxa_yaw_hub_carrier_ovh",
            "name": f"L{leg} deep lower yaw hub carrier + 3 radial screws (NEW)",
            "partType": "coxa_yaw_hub_carrier_ovh",
            "role": "variant",
            "leg": leg,
            "joint": None,
            "cots": False,
            "color": "#78b98a",
            "transform": _mat16(transforms["coxa"]),
        })
        joints[f"L{leg}-yaw"]["instances"].append(lower_carrier_id)
        carrier_id = f"{next_id:03d}-L{leg} screw-on upper bearing carrier"
        next_id += 1
        scene["instances"].append({
            "id": carrier_id,
            "meshId": "stl:hip_bearing_carrier_ovh",
            "name": f"L{leg} upper bearing carrier + 4 nut traps (NEW)",
            "partType": "hip_bearing_carrier_ovh",
            "role": "variant",
            "leg": leg,
            "joint": None,
            "cots": False,
            "color": "#b8d99c",
            "transform": _mat16(transforms["hip_cap"]),
        })
        joints[f"L{leg}-yaw"]["instances"].append(carrier_id)
        for joint_name, frame_name in (("hip", "femur"), ("knee", "tibia")):
            matrix = transforms[frame_name] @ base.MH
            ids = []
            for key, label in (("driven_spacer", "driven 7 mm spacer"),
                               ("passive_spacer", "passive 7 mm spacer")):
                iid = f"{next_id:03d}-L{leg} {joint_name} {label}"
                next_id += 1
                scene["instances"].append({
                    "id": iid,
                    "meshId": f"stl:{key}",
                    "name": f"L{leg} {joint_name} {label} (NEW)",
                    "partType": key,
                    "role": "variant",
                    "leg": leg,
                    "joint": None,
                    "cots": False,
                    "color": "#d6a84a" if key == "driven_spacer" else "#e6c36a",
                    "transform": _mat16(matrix),
                })
                ids.append(iid)
            joints[f"L{leg}-{joint_name}"]["instances"].extend(ids)

    def instance_id(leg: int | None, part_type: str, name_token: str) -> str:
        matches = [
            instance["id"]
            for instance in scene["instances"]
            if instance.get("leg") == leg
            and instance["partType"] == part_type
            and name_token in instance["name"]
        ]
        assert len(matches) == 1, (
            f"expected one {part_type}/{name_token} instance for L{leg}, "
            f"found {matches}"
        )
        return matches[0]

    chassis_id = instance_id(None, "chassis_bottom", "chassis_bottom")
    allowed = scene["checksConfig"]["allowedInterferences"]
    for leg in range(6):
        yaw_servo_id = instance_id(leg, "servo_body", "yaw servo")
        lower_carrier_id = instance_id(
            leg, "coxa_yaw_hub_carrier_ovh", "lower yaw hub"
        )
        lower_bearing_id = instance_id(
            leg, "yaw_bearing_upper", "yaw bearing"
        )
        hip_servo_id = instance_id(leg, "servo_body", "hip servo")
        hip_cap_id = instance_id(leg, "hip_clamp_cap_ovh", "hip clamp cap")
        carrier_id = instance_id(
            leg, "hip_bearing_carrier_ovh", "bearing carrier"
        )
        upper_bearing_id = instance_id(leg, "bearing_6805", "third 6805")
        knee_servo_id = instance_id(leg, "servo_body", "knee servo")
        knee_cap_id = instance_id(leg, "knee_clamp_cap_ovh", "knee cap")
        allowed.extend([
            {
                "kind": "press_fit",
                "instances": [chassis_id, yaw_servo_id],
                "reason": (
                    f"L{leg} yaw servo body is deliberately captured by its "
                    "chassis cradle"
                ),
                "maxPenetrationMm": 2.10,
            },
            {
                "kind": "press_fit",
                "instances": [yaw_servo_id, lower_carrier_id],
                "reason": (
                    f"L{leg} yaw output boss deliberately engages the coaxial "
                    "screw-on driven coxa hub"
                ),
                "maxPenetrationMm": 2.25,
            },
            {
                "kind": "bearing_seat",
                "instances": [lower_carrier_id, lower_bearing_id],
                "reason": (
                    f"L{leg} screw-on lower hub deliberately press-seats the "
                    "tower 6805 inner race"
                ),
                "maxPenetrationMm": 0.35,
            },
            {
                "kind": "press_fit",
                "instances": [hip_servo_id, hip_cap_id],
                "reason": (
                    f"L{leg} hip cap tongue has the documented 0.5 mm servo "
                    "body preload"
                ),
                "maxPenetrationMm": 0.75,
            },
            {
                "kind": "press_fit",
                "instances": [knee_servo_id, knee_cap_id],
                "reason": (
                    f"L{leg} knee cap tongue has the documented 0.5 mm servo "
                    "body preload"
                ),
                "maxPenetrationMm": 0.75,
            },
            {
                "kind": "bearing_seat",
                "instances": [carrier_id, upper_bearing_id],
                "reason": (
                    f"L{leg} screw-on carrier deliberately press-seats the "
                    "upper 6805 inner race"
                ),
                "maxPenetrationMm": 0.35,
            },
        ])

    scene["poses"].append({
        "id": "front-interface",
        "name": "Front six-hole interface (stance)",
        "jointValues": {},
    })
    return scene


def render_preview(meshes: dict[str, trimesh.Trimesh]) -> None:
    """Exploded joint view showing bracket, both pucks, and front receiver."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        print(f"preview skipped (optional dependency missing: {exc.name})")
        return

    parts = [
        ("chorn_clamp_cnc", 0.0, "#30343b", "premade C-horn"),
        ("driven_spacer", 4.0, "#d6a84a", "driven spacer (blind relief)"),
        ("passive_spacer", -4.0, "#e6c36a", "passive spacer (through hole)"),
        ("femur_ovh_body", 0.0, "#70a866", "six-hole femur receiver"),
    ]
    fig, ax = plt.subplots(figsize=(10.5, 6.5), dpi=140)
    try:
        for key, dz, color, label in parts:
            mesh = meshes[key].copy()
            mesh.apply_translation([0.0, 0.0, dz])
            section = mesh.section(plane_origin=[0, 0, BRACKET_MID_Z],
                                   plane_normal=[0, 1, 0])
            if section is None:
                continue
            planar, _ = section.to_2D(
                to_2D=rotation_matrix(-math.pi / 2.0, [1, 0, 0])
            )
            first = True
            for polygon in planar.polygons_full:
                ax.fill(*polygon.exterior.xy, color=color, alpha=0.68,
                        label=label if first else None)
                first = False
                for ring in polygon.interiors:
                    ax.fill(*ring.xy, color="white")
    except ModuleNotFoundError as exc:
        plt.close(fig)
        print(f"preview skipped (optional dependency missing: {exc.name})")
        return
    ax.set_aspect("equal")
    ax.set_xlabel("joint-local X [mm]")
    ax.set_ylabel("joint-local Z [mm]")
    ax.set_title("Premade 56 mm C-horn: 7 mm spacer pair + front receiver")
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(HERE / "preview.png")
    plt.close(fig)


def main() -> None:
    skip_sweep = "--skip-sweep" in sys.argv
    skip_ancestor_brep = "--skip-ancestor-brep" in sys.argv
    if not skip_ancestor_brep:
        print(
            "refreshing cnc_chorn_overhead BREP ancestors so the premade "
            "coxa/femur holders match current production service geometry ..."
        )
        subprocess.run(base.BREP_EXPORT_CMD, check=True)
    else:
        print(
            "ANCESTOR BREP REFRESH SKIPPED (--skip-ancestor-brep): "
            "reusing cnc_chorn_overhead/step/stl"
        )
    meshes = build_meshes()
    print("premade-chorn checks ...")
    masses = check_parts(meshes)
    chassis_bottom = check_chassis_bottom_without_trunk(meshes)
    clamp_cap_captive_nuts = check_clamp_cap_captive_nuts(meshes)
    femur_servo_fit = check_femur_servo_fit(meshes)
    tibia_screw_access = check_tibia_screw_access(meshes)
    serviceability = check_horn_on_servo_extraction(meshes)
    split_lower_yaw_hub = check_split_lower_yaw_hub(meshes)
    split_hip_bearing = check_split_hip_bearing(meshes)
    coxa_tower = check_integral_coxa_tower(meshes)
    check_joint_motion(meshes)
    assembled_meshes = _assembled_hip_cap_meshes(
        _assembled_coxa_meshes(meshes)
    )
    inherited_coxa_meshes = dict(meshes)
    inherited_coxa_meshes["coxa_link_ovh"] = meshes[
        "coxa_link_assembled_check"
    ]

    # Re-run the important inherited rigid-bearing and outboard-hip checks.
    rv.check_static(meshes)
    rv.check_bottom_joint(meshes)
    rv.check_chassis_variant(meshes)
    base.check_coxa_ovh(inherited_coxa_meshes)
    base.check_hip_cap_ovh(assembled_meshes)
    base.check_assembly_paths_ovh(assembled_meshes)

    if skip_sweep:
        limit = -45.0
        sweep = {"skipped": True, "placeholder_limit_deg": limit}
    else:
        print("full femur/plate sweep ...")
        limit, contact_stack, contact_by_yaw, part_by_yaw = \
            base.sweep_femur_envelope(assembled_meshes)
        sweep = {
            "limit_deg": limit,
            "contact_vs_own_stack": {
                "deg": contact_stack[0], "part": contact_stack[1]
            },
            "contact_vs_plate_by_yaw": {
                str(yaw): {"deg": contact_by_yaw[yaw],
                           "part": part_by_yaw[yaw]}
                for yaw in contact_by_yaw
            },
        }

    scene = build_scene(meshes, limit)
    (HERE / "scene.json").write_text(json.dumps(scene, indent=1) + "\n")
    report = {
        "build_id": BUILD_ID,
        "assumptions": {
            "outside_span_mm": CHORN_OUTSIDE_SPAN,
            "plate_thickness_mm": CHORN_PLATE_T,
            "axis_to_front_outer_mm": CHORN_AXIS_TO_FRONT_OUTER,
            "blade_width_mm_assumed": CHORN_BLADE_WIDTH,
            "front_extra_holes": (
                f"y=0, {EXTRA_EDGE_INSET:.1f} mm from top/bottom outside "
                f"edges; {EXTRA_CENTRE_SPAN:.1f} mm centre span derived "
                f"from {EXTRA_INNER_EDGE_GAP:.1f} mm inner-edge and "
                f"{EXTRA_OUTER_EDGE_SPAN:.1f} mm outer-edge measurements"
            ),
            "front_extra_hole_physical_diameter_mm": EXTRA_PHYSICAL_HOLE_D,
            "front_receiver_m3_clearance_diameter_mm": FRONT_M3_D,
            "brackets_installed": 12,
            "brackets_owned": 16,
            "spares": 4,
        },
        "fit": {
            "horn_face_span_mm": base.DISC_SPAN,
            "two_spacers_mm": 2.0 * SPACER_T,
            "assembled_inner_span_mm": ASSEMBLED_INNER_SPAN,
            "nominal_bracket_inner_span_mm": CHORN_INNER_SPAN,
            "nominal_total_spread_mm": BRACKET_SPREAD,
            "tube_shorter_than_overhead_mm": TUBE_SHORTER_THAN_OVERHEAD,
            "tube_shorter_than_production_mm": TUBE_SHORTER_THAN_PRODUCTION,
            "tube_engagement_depth_mm": TIB_TUBE_ENGAGEMENT,
            "unsupported_tube_span_reduction_vs_original_mm": (
                UNSUPPORTED_TUBE_REDUCTION
            ),
            "socket_mouth_outboard_vs_overhead_mm": (
                MOUTH_OUTBOARD_THAN_OVERHEAD
            ),
        },
        "chassis_bottom": chassis_bottom,
        "masses": masses,
        "clamp_cap_captive_nuts": clamp_cap_captive_nuts,
        "femur_servo_fit": femur_servo_fit,
        "split_lower_yaw_hub": split_lower_yaw_hub,
        "split_hip_bearing": split_hip_bearing,
        "tibia_screw_access": tibia_screw_access,
        "servo_serviceability": serviceability,
        "coxa_support_tower": coxa_tower,
        "sweep": sweep,
    }
    (HERE / "geometry_report.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    render_preview(meshes)
    print(f"wrote {HERE / 'scene.json'}")
    print(f"wrote {HERE / 'geometry_report.json'}")
    print(f"BuildViz instances: {len(scene['instances'])}; "
          f"premade brackets: "
          f"{sum(i['partType'] == 'chorn_clamp_cnc' for i in scene['instances'])}")


if __name__ == "__main__":
    main()
