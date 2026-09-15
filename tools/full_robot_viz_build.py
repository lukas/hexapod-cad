#!/usr/bin/env python3
"""Full-robot BuildViz scene built from the VERIFIED standing-pose parts.

History: the original scene drew the coxa / femur / tibia as plain
cylinders and dressed each joint with generic test-fit housing/yoke
meshes.  Those proxies visually clipped each other (the straight coxa
cylinder ran through the hip hardware) even though the real printed parts
clear it.

This version instead reuses the EXACT placement the verifier checks for
self-collision -- ``_verify_prototype._build_standing_leg`` (the real
``make_coxa_link`` / ``make_femur_link`` / ``make_tibia_link`` meshes) and
``_verify_prototype._place_servo_bodies`` (the three STS3215 servo
envelopes in their cradles).  That configuration passes
``check_self_collision`` (6.8 mm^3) and ``check_servo_clearance`` (<=165
mm^3), so the viewer now shows a geometry that is provably free of
unexpected pass-throughs.

The verifier builds only leg 0 (apothem direction a = pi/6).  The six legs
are rotationally symmetric about the chassis Z axis, so legs 1-5 are leg 0
rotated by i*60 deg.  Body parts (chassis plates, battery holder,
electronics tray) are placed exactly as ``make_assembly_preview`` does.

Motion is now baked into this ONE scene: the additive BuildViz ``joints[]`` +
``poses[]`` + ``animations[]`` blocks (per-joint sliders, clickable poses, a
looping tripod-walk clip) live directly in ``full_robot_viz/scene.json`` -- there
is no longer a separate ``scene_motion.json`` file or a distinct
``prototype_sts3215_motion`` build id.  Generating the motion blocks is
essentially free (~0.01 s of joint/pose math on the ALREADY-placed meshes vs the
~4 s STL rebuild), so motion is ALWAYS generated.  The only expensive motion step
is the swept self-overlap check (``buildviz sweep`` ~13 s), which is therefore
OPT-IN on push (see the Makefile ``SWEEP=1``), while the cheap manifest
validation runs every push.

Run:
    ./run.sh hexapod_walker/prototype_sts3215/tools/full_robot_viz_build.py

Then PUBLISH into the ONE machine-wide BuildViz hub (default port 5183) and view
it there -- never start a per-project dev server on 5173/etc:
    make -C hexapod_walker/prototype_sts3215 verify-buildviz   # buildviz push --bump
    open http://127.0.0.1:5183/?build=prototype_sts3215
"""

from __future__ import annotations

import json
import re
import shutil
import sys
from pathlib import Path

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix

_HERE = Path(__file__).resolve().parent

import hexapod_prototype as HP
import _verify_prototype as V
import fastener_registry as FR
import wire_harness_plan as WHP
# Joint/servo naming comes from the robot repo's executable joint contract;
# install with `uv sync --extra sim` (path dependency on ../hexapod).
from motor_setup.feetech_bus import joint_to_servo_id
from hexapod_core.joint_frame import joint_index

OUT_DIR = _HERE.parent / "full_robot_viz"
STL_DIR = OUT_DIR / "stl"
FASTENERS_DIR = _HERE.parent / "fasteners"

# Mesh URLs are RELATIVE (``stl/<file>.stl``) so this build is self-contained
# and portable across every BuildViz entry point: ``buildviz push
# --upload-assets`` ships the referenced STL bytes (it only uploads relative,
# locally-resolvable URLs) and the hub rewrites them to its content-addressed
# ``/builds/_assets/<sha>`` store; the local ``buildviz .`` debug server rewrites
# relative ``stl/...`` URLs to the served build; and ``tools/buildviz_checks.py``
# resolves them next to ``scene.json``.  (Absolute ``/builds/<id>/stl/...`` URLs
# only resolved against a hand-made static ``public/builds/<id>`` mirror, which
# this project no longer relies on -- versioned publishing now goes through the
# hub cache via ``push --bump``.)
SCENE_BUILD_ID = "prototype_sts3215"
SCENE_ASSET_BASE = "stl"


_MARKDOWN_LINK_RE = re.compile(
    r"(?P<prefix>!?\[[^]]*\]\()(?P<target>[^)]+)(?P<suffix>\))"
)


def _rebase_project_markdown_links(text: str) -> str:
    """Rebase project-relative links for a copy one directory deeper."""

    def replace(match: re.Match[str]) -> str:
        target = match.group("target")
        if (
            target.startswith(("#", "/"))
            or "://" in target
            or target.startswith("mailto:")
        ):
            return match.group(0)
        return f"{match.group('prefix')}../{target}{match.group('suffix')}"

    return _MARKDOWN_LINK_RE.sub(replace, text)

# Joint screws to render (the leg "connecting" fasteners).  Bright colours
# for the hip bolts so the coxa<->femur connection stands out.  Chassis-level
# / foot fasteners (registry joint == None) fall back to FASTENER_CHASSIS_COLOR.
FASTENER_JOINT_COLOR = {"yaw": "#bdbdbd", "hip": "#ffd000", "knee": "#9ad0ff"}
FASTENER_CHASSIS_COLOR = "#9a9a9a"          # steel hardware (standoffs/foot/deck)
DISC_HORN_COLOR = "#b8b8c0"                  # silver-anodised aluminium disc horn

# Non-printed COTS roles: every instance with one of these roles is hardware we
# BUY (servos, bearings, disc horns, fasteners) and tag ``cots: true`` so the
# viewer can distinguish them from printed bodies.  They live ONLY in this scene
# and are never registered in the printable PART_REGISTRY / trays / BOM.
COTS_ROLES = {"motor", "bearing", "horn", "fastener", "electronics"}

PALETTE = {
    # Each leg is now called out as its INDIVIDUAL printed parts (BOM-correct
    # sandwich: yoke + dia-8 CF tube + bracket/fitting) instead of the merged
    # femur_link / tibia_link proxies.
    "coxa_link": "#9467bd",
    "yaw_bearing_cap": "#6b4fa0",
    "yaw_bearing_lower": "#d4af37", "yaw_bearing_upper": "#ffd966",
    "femur_link": "#2ca02c",
    "tibia_knee_yoke": "#1f77b4",
    "tibia_tube": "#2b2b2b",                             # carbon fibre
    "foot_boot": "#3a3a3a",                              # TPU boot
    "disc_horn_yaw": DISC_HORN_COLOR, "disc_horn_hip": DISC_HORN_COLOR,
    "disc_horn_knee": DISC_HORN_COLOR,
    "passive_horn_hip": DISC_HORN_COLOR, "passive_horn_knee": DISC_HORN_COLOR,
    "yaw_servo": "#2e2e33", "hip_servo": "#3a3a40", "knee_servo": "#46464d",
    "chassis_bottom": "#79b0e1",
    "chassis_top": "#5b8fc7",
    "hex_mount_plate": "#94a3b8", "hex_raised_platform": "#64748b",
    "hex_post_standoff": "#a8a29e", "hex_post_thumb_nut": "#78716c",
    "chassis_standoff": "#b5952f",
    "hex_post_magnet": "#1e293b",
    "uno_q": "#1b7a3d", "breakout": "#0d9488",
    "motor_controller": "#b45309",
    "screen": "#2563eb", "mpu6050": "#7a5cc4",
    "wago_trunk": "#ea580c", "wago_v33": "#f97316",
    "wago_power": "#ef4444", "wago_data": "#22c55e",
    "lipo_battery": "#d62728",
    "hip_clamp_cap": "#4a90d9", "knee_clamp_cap": "#4a90d9",
    "yaw_servo_retainer": "#e377c2",
    "switch_holster": "#c46a1f",
}
ROLE = {
    "coxa_link": "frame",
    "yaw_bearing_cap": "chassis",
    "yaw_bearing_lower": "bearing", "yaw_bearing_upper": "bearing",
    "femur_link": "frame",
    "tibia_knee_yoke": "frame",
    "tibia_tube": "spar", "foot_boot": "frame",
    "disc_horn_yaw": "horn", "disc_horn_hip": "horn", "disc_horn_knee": "horn",
    "passive_horn_hip": "horn", "passive_horn_knee": "horn",
    "yaw_servo": "motor", "hip_servo": "motor", "knee_servo": "motor",
    "chassis_bottom": "chassis",
    "chassis_top": "chassis",
    "hex_mount_plate": "electronics", "hex_raised_platform": "electronics",
    "hex_post_standoff": "electronics", "hex_post_thumb_nut": "electronics",
    "chassis_standoff": "electronics",
    "hex_post_magnet": "electronics",
    "uno_q": "electronics", "breakout": "electronics",
    "motor_controller": "electronics",
    "screen": "electronics", "mpu6050": "electronics",
    "wago_trunk": "electronics", "wago_v33": "electronics",
    "wago_power": "electronics", "wago_data": "electronics",
    "lipo_battery": "electronics",
    "hip_clamp_cap": "frame", "knee_clamp_cap": "frame",
    "yaw_servo_retainer": "frame",
    "switch_holster": "electronics",
}

# Per-partType design intent / rationale.  This is the semantic source of truth
# that `tools/full_robot_viz_build.py` writes into full_robot_viz/design_spec.yaml
# so the file ALWAYS covers every partType the scene emits (BuildViz `compat`
# fails on any scene part with no entry).  Kinematics: 6 legs, COXA 25
# / FEMUR 90 / TIBIA 150 mm; only joint/segment CONSTRUCTION changes.  Joint
# architecture: YAW = cantilevered (coxa 25 mm < ~50 mm rule, no passive
# bearing); HIP + KNEE = bearing sandwich (driven Ø20 25T disc horn on one side,
# 688 passive bearing on the other).  When you add/rename a partType in PALETTE,
# add its rationale here too — a missing key makes the build non-compat.
DESCRIPTIONS = {
    "coxa_link": "ONE-PIECE printed coxa (Aug 2026 merge of the old coxa_yaw_hub + coxa_hip_bracket): the yaw turntable hub (bolts the driven Ø20 disc horn on the Ø14 / 4x M3 circle, rides the touching 6805 pair (25x37x7, Aug 2026 thick-section swap) on a Ø25.15 press-fit boss) fused to the hip servo cradle. Aug 17 2026 sink pass: the cradle rides 5 mm lower -- its foot slab fuses through the full platform band and the hip-servo well floor sits 2 mm over the platform top (+26), so the 5 vertical head-access shafts are only ~5.3 mm deep above the M3x30 horn-screw heads (any driver fits) and the part is 5 mm shorter. Aug 17 2026 scrape fix: the running gaps between the rotating link and the stationary yaw_bearing_cap widened to 1.5 mm axial (platform underside over the cap rim) and 1.0 mm radial (dust-skirt bore over the Phi 44 ring) after the printed parts scraped. The horn-screw seats are 1.25 mm deeper (bench: printed seats + screw tolerance ate ~1 mm of horn bite; corner tips now just break the disc's far face, the centre seat goes 1 mm deeper into the spline tap). Aug 16 2026 flatten: the cradle's 4x M2.5 end-face bolt holes and the DS3225-era wire-exit corridor are deleted -- a clean 4-wall box. Aug 17 2026: the cradle grew the SAME rear retention tab as the femur knee cradle (user: 'copy that same part ... so I can screw into the servo from both sides') -- a vertical 5.5 mm plate on the back-face side taking 2x M2.5x6 self-tappers into the hip servo's rear molded hole pair nearer the inboard/wire end, heads flush in Phi 5.2 x 2 mm recesses (connector-end pair stays open for the bus harness; the swinging femur arm passes ~1.5 mm over the tab's outer face).",
    "yaw_bearing_cap": "Printed cap that closes the top of each chassis yaw-bearing tower, capturing the upper yaw bearing.",
    "yaw_bearing_lower": "Lower ball bearing of the yaw-axis bearing pair in the chassis tower (COTS).",
    "yaw_bearing_upper": "Upper ball bearing of the yaw-axis bearing pair in the chassis tower (COTS).",
    "femur_link": "The WHOLE femur as ONE printed part (Jul 2026 merge #2 of the old hip yoke + knee bracket): the hip-end yoke straddles the hip servo (top arm bolts the driven disc horn, bottom arm the passive horn; 8 mm spine plate since Aug 2026, and both horn pads reach 0.5 mm/side deeper — YOKE_PAD_EXTRA_REACH, bench fit — to close the measured ~1 mm clevis-to-horn gap), a SOLID Ø18 spar with small cone flares at both ends bridges the full 90 mm hip-to-knee span, and the knee-end cradle mounts the knee servo + 688 bearing housing. Aug 2026: a rear tab under the knee servo's open back face takes 2× M2.5×6 self-tappers into the rear molded hole pair nearer the spar (the connector-end pair stays open for the bus harness; the tab reaches 4.5 mm past the servo's wire-end face — Aug 17 2026 bench remeasure: the flat back-face ledge before the ~1.8 mm case step is just under 5 mm, not the ~4 mm read off the photo, so the 1 mm-longer tab now fully encloses both screw holes instead of the old open keyhole slots — its wall riser is capped 3.5 mm above the servo base for the wire-end shell flare, and the heads sit FLUSH in Ø5.2 × 2 mm recesses like the output plate's 4 countersunk case screws: the tab is 5.5 mm thick = 3.5 shank + 2 head pocket, keeping the M2.5×6's pilot bite unchanged), and the knee well's far (+X) wall is one flat solid face: full-width buttress thickness, no DS3225-era wire channel (the knee cables leave via the back-face ports through the open back). No sockets, no slip fits, no retention pins anywhere in the femur.",
    "tibia_knee_yoke": "Printed tibia knee-end yoke driven off the knee disc horn (bearing-sandwich passive side on the opposite face); sockets the Ø8 tibia tube. Aug 2026 bench fit: both horn pads reach 0.5 mm/side deeper (YOKE_PAD_EXTRA_REACH) to close the measured ~1 mm total clevis-to-horn gap.",

    "tibia_tube": "Ø8 carbon-fibre tibia segment; with the standard boot installed the kinematic knee-centre-to-furthest-tip length is 150 mm. Retained by epoxy bond alone (Aug 2026 -- retention pin removed). CF for stiffness/weight at the longest, most-loaded segment.",
    "foot_boot": "TPU 95A standard spherical-ended boot pressed over the tibia tube end -- nominal Ø9 mm rounded tip is the ground contact (Aug 2026; replaces the hinged foot fitting + pad + M3x16 pin).",
    "disc_horn_yaw": "Driven Ø20 25T aluminium disc horn on the yaw servo output; the coxa hub bolts to it (Ø14 / 4x M3). COTS.",
    "disc_horn_hip": "Driven Ø20 25T aluminium disc horn on the hip servo output; the femur hip yoke bolts to it. COTS.",
    "disc_horn_knee": "Driven Ø20 25T aluminium disc horn on the knee servo output; the tibia knee yoke bolts to it. COTS.",
    "passive_horn_hip": "Stock passive-side metal disc horn of the hip sandwich (mirrors the driven horn so the yoke is symmetric across the servo); its centre bore rides the servo's rear idler boss directly, seating flush on the back face (Jul 2026: printed centering adapter retired).",
    "passive_horn_knee": "Stock passive-side metal disc horn of the knee sandwich; centre bore rides the rear idler boss directly (no printed adapter).",
    "yaw_servo": "FEETECH STS3215 serial-bus servo driving the hip-yaw axis (real FEETECH envelope). COTS.",
    "hip_servo": "FEETECH STS3215 serial-bus servo driving the hip-pitch axis. COTS.",
    "knee_servo": "FEETECH STS3215 serial-bus servo driving the knee axis. COTS.",
    "chassis_bottom": "Structural 200 mm flat-to-flat hex deck (single merged print) with 6 integrated STS3215 front-face-mount yaw cradles + upward yaw-bearing towers, one per leg at each hex-edge midpoint. Each STS3215 inserts from BELOW (output UP), bolts via 4x M2.5 through the cradle front plate, body hangs DOWN through a body cutout; the bolt-on yaw_servo_retainer stirrup captures it. A folded 4 mm floor makes the printed bottom one flush flat face so it prints flat, tower-up, no supports (Jun 2026 flush-bottom fix; check_flat_bottom overhang = 0.00 mm). Aug 16 2026: Ø9 full-stack seat pads restore the 4 brass-standoff seats that the inboard-shifted harness ports had clipped (each pad nibbles one port corner; connectors still pass).",
    "chassis_top": "Hex top plate that closes the chassis; carries the trunk power Wagos + motor controller and the 4 magnet posts for the hex board (per-leg power Wago pairs live below, on the chassis_bottom corner flats).",
    "hex_mount_plate": "Round Ø115 mm board (2 mm, matches the chassis_top disc) held by 4 magnets on 20 mm posts; carries Uno Q + breakout on top and the 3.3 V Wago underneath. Late-Aug 2026: 4 underside registration bosses socket the magnet tops (shear goes to plastic, magnets only carry pull); legacy 49.5 mm bolt square dropped. Review round 2: 3 stand-foot holes (M3x8 up into the stand's self-tap pilots), the Uno Q's 3-point UNO mount holes (M3x8 down into thumb nuts; the separate io board is retired), 2 Ø8 wire ports at the south rim, E/W zip-tie slots. Print-only (SVG cut file retired).",
    "hex_raised_platform": "Screen stand ('hex' is historical): round Ø115 top disc matching the plate below, on 3 blade legs at az 90/210/330 (28 mm, shortened from 72 in the late-Aug 2026 design review to cut the lever arm on the magnet-held plate). Mounts with 3x M3x8 SHCS driven up from under the plate into blind self-tap pilots in the feet. Screen on the top face, held by 4x M2 self-tappers in corner pilot holes, its 8-wire Uno pigtail entering through the 24x5 slot behind the panel's +X edge (MPU is glued on chassis_top, not under this plate).",
    "hex_post_standoff": "20 mm M3 brass standoff at CHASSIS_STANDOFF_HOLES_XY (±31.1); bottom of each magnet post stack.",
    "chassis_standoff": "32 mm M3 brass standoff between chassis_bottom and chassis_top at CHASSIS_STANDOFF_HOLES_XY (±31.1) — the inter-plate structure. COTS.",
    "hex_post_thumb_nut": "~2.5 mm M3 knurled thumb nut between standoff and magnet.",
    "hex_post_magnet": "Ø8×8 mm disc magnet that holds the hex mount plate.",
    "uno_q": "Arduino Uno Q compute board on the hex mount plate (high-level control host). COTS.",
    "breakout": "Generic shield / breakout next to the Uno Q on the hex plate. COTS.",
    "wago_trunk": "Central trunk splice pair: two 5-port Wago 221-415 side by side near the chassis_top centre (nudged +X for adapter wire clearance), wire entries facing +X toward the switch. Polarity: V+ = the SOUTH nut at (16, -16) nearest the switch, GND = the NORTH nut at (16, +16). The fused battery trunk lands here and fans out the six 16 AWG branches to the corner power Wagos (as-built Aug 2026: no PDB — power distribution is all lever nuts). COTS.",
    "wago_v33": "3.3 V rail splice: one 5-port Wago 221-415 VHB-taped flat under the round mount plate near its south rim, wire entries facing -Y. Feed = Uno Q 3V3 pin (down through the east Ø8 wire port); load = MPU VCC (straight down to the deck), 3 spare ports (Aug 2026: the screen's VCC rides its own 8-wire Uno pigtail instead). Pull the plate off its magnets to work the levers. COTS.",
    "motor_controller": "Waveshare Bus Servo Adapter (A), 42x33 mm, on the chassis_top west band. Servo/UART plugs exit its +X face, screw terminal + USB-C its -X face; both faces keep clear wire zones, and the USB-C additionally keeps a 30 mm bench-plug corridor off the -X face so a laptop cable plugs straight in (connector_clearance checks; the corridor may pass the deck rim because the robot is parked when tethered). As-built the Uno Q drives it over the D0/D1 TX/RX/GND UART pigtail (jumper A); USB-C is bench-only since a VIN-powered Uno Q gives no USB host mode / no VBUS. COTS.",
    "screen": "63×35 mm display panel centered on the raised platform top. COTS.",
    "mpu6050": "GY-521 MPU-6050 IMU glued on chassis_top just south of the central trunk Wago pair (inboard, r=43). Right-angle header row faces -X with a clear wire zone (connector_clearance check). I²C to the Uno Q above. COTS.",
    "wago_power": "5-port Wago 221-415 lever nut for a per-leg 12 V + G motor branch: ONE press-fit nut per chassis_bottom hex corner flat, wedged between the integrated tray walls (Aug 16 2026, user -- was a V+/GND pair of 3-port 221-413; WAGO_MOUNT_BAY_CLEAR is now -0.15, a 0.75 mm tighter nominal-interference press fit), wire entries facing the chassis centre. COTS.",
    "wago_data": "Wago 221-style lever nut under chassis near a yaw retainer for the shared data bus.",
    "lipo_battery": "3S 2200 mAh shorty LiPo (Zeee 75x34x26.5 mm), one of a PARALLEL pair velcro'd side by side under chassis_bottom's flat belly (block yawed 30 deg; XT60 Y-harness). Replaces the single bay pack -- lower CG and a clear inter-plate wiring bay. COTS.",
    "hip_clamp_cap": "Printed clamp cap that clamshells the hip servo cradle (bolts to the cradle wall ends), capturing the servo body. Aug 18–19 2026: a 10 mm-wide L-shaped back-face hook near the wire end — 5.5 mm wall + shelf lapping 7.4 mm over the back of the motor flush with its surface (the wire-end band steps up — no bite; plug-cable corridor open beside it, clear of the yoke pad sweep), plus a tiny corner pad at the horn end (1 mm press-fit bite, gap to the passive horn) that clamps the body against the front lip; 1 mm 45° chamfers on the outer face's vertical edges so the sweeping yoke spine can't catch. Sep 2026 service refit: the holder's Ø24 horn bore is open to the +Y clamp mouth, so after removing the two clamp bolts the servo, fitted horn and this cap lift out together; the slot removes one small front-case capture site and leaves the other three.",
    "knee_clamp_cap": "Printed clamp cap that clamshells the knee servo cradle (bolts to the cradle wall ends), capturing the servo body — same part as the hip cap. Aug 18–19 2026: 10 mm-wide L-shaped back-face hook (wall + flush shelf — the wire-end band steps up, no bite) plus a tiny horn-end corner pad (1 mm press-fit bite) that clamps the body against the front lip; plug corridor open beside the hook. Sep 2026 service refit: the holder uses the same open Ø24 horn slot as the hip holder, allowing the servo, fitted horn and attached cap to lift out as one assembly; three small front-case capture sites remain.",
    "yaw_servo_retainer": "Anti-rotation saddle under each yaw cradle (Aug 2026 flat-belly rework: the 38 mm ground stand is removed; the belly is flat except the hanging servos + saddles).",
    "switch_holster": "Printed holster for the anti-spark XT60 on/off switch, velcroed flat to chassis_top's +X edge (Aug 16 2026: the bolt-down bosses/inserts/ear are retired -- solid flat floor for the velcro patch).",
    "screw_yaw": "Fasteners around the yaw joint / servo mount (rendered set; count is the summed per-leg joint hardware).",
    "screw_hip": "Fasteners around the hip joint / bearing sandwich (rendered set).",
    "screw_knee": "Fasteners around the knee joint / bearing sandwich (rendered set).",
    "screw_chassis": "Chassis-level / foot / deck fasteners (rendered set).",
    "screw_retainer": "Yaw saddle hardware: the 4 M3 chassis anchors + 4 M2.5 rear-case capture screws per retainer — chassis-static (the saddle and servo case never rotate).",
}


def _write_design_spec(instances_json: list[dict],
                       routes: list[dict] | None = None) -> None:
    """Write full_robot_viz/design_spec.yaml keyed by every scene partType.

    BuildViz's compatibility contract (see ~/buildviz/BUILDVIZ_COMPATIBILITY.md)
    requires the design_spec's ``parts:`` mapping to cover every ``partType`` the
    scene emits.  We derive it from the instances we just placed so the spec can
    never silently drift out of coverage: qty + role + cots come from the scene,
    the rationale from DESCRIPTIONS above.  A ``wiring:`` entry per published
    route (purpose = the route label) keeps the hub's published-wire spec
    audit green the same way.
    """
    counts: dict[str, int] = {}
    role_of: dict[str, str] = {}
    cots_of: dict[str, bool] = {}
    for inst in instances_json:
        pt = inst["partType"]
        counts[pt] = counts.get(pt, 0) + 1
        role_of[pt] = inst.get("role", "")
        cots_of[pt] = bool(inst.get("cots", False))

    lines = [
        "# Hexapod STS3215 — full robot (viz).  design intent + rationale, and",
        "# BuildViz's semantic source of truth (one entry per scene partType).",
        "#",
        "# GENERATED by tools/full_robot_viz_build.py from the placed instances so",
        "# it always covers every scene partType (edit DESCRIPTIONS in that script,",
        "# not this file).  Kinematics: 6 legs, COXA 25 / FEMUR 90 / TIBIA",
        "# 150 mm.  YAW = cantilevered; HIP + KNEE = bearing sandwich (driven Ø20",
        "# 25T disc horn + 688 passive bearing).",
        "",
        "units: mm",
        "",
        "parts:",
    ]
    for pt in sorted(counts):
        label = pt.replace("_", " ")
        desc = DESCRIPTIONS.get(pt, f"{label} (see full_robot_viz_build.py).")
        lines.append(f"  {pt}:")
        lines.append(f"    label: {json.dumps(label)}")
        lines.append(f"    role: {json.dumps(role_of[pt])}")
        lines.append(f"    cots: {'true' if cots_of[pt] else 'false'}")
        lines.append(f"    qty: {counts[pt]}")
        lines.append(f"    description: {json.dumps(desc)}")
    if routes:
        lines.append("")
        lines.append("wiring:")
        for route in routes:
            lines.append(f"  {route['id']}:")
            lines.append(f"    purpose: {json.dumps(route.get('label', ''))}")
    (OUT_DIR / "design_spec.yaml").write_text("\n".join(lines) + "\n")
    print(f"Wrote {OUT_DIR/'design_spec.yaml'}  ({len(counts)} part types, "
          f"{len(routes or [])} wiring routes)")


def _trans(v) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = np.asarray(v, float)
    return T


# ---------------------------------------------------------------------------
# BuildViz "checks" sidecar (precomputed, read on scene load)
# ---------------------------------------------------------------------------
#
# BuildViz auto-loads a ``buildviz_checks.json`` sidecar next to ``scene.json``
# into its clickable Checks panel (see /Users/lbiewald/buildviz
# BUILDVIZ_INTEGRATION.md "scene.json Contract" + the ``buildviz check --emit``
# sidecar: generatedAt / build / checksConfig / summary / checks[] / highlights).
# We can't run the (concurrently-edited) buildviz CLI, so we PRECOMPUTE the
# generic geometric ``mesh_overlap`` class here -- the same printed-part
# interpenetration class the offline verifier guards with
# ``check_clamp_cap_interference`` -- and write the sidecar in BuildViz's exact
# schema so the panel surfaces it on load (the publish path is a symlink, so it
# is picked up).
#
# SceneCheck = { id, kind, status: 'pass'|'warn'|'fail', label,
#                instances?: string[], point?: [x,y,z], region?: {min,max} }
# Status colours (CHECK_STATUS_COLOR): fail #ff3b30, warn #f59e0b, pass #22c55e.

CHECK_STATUS_COLOR = {"fail": "#ff3b30", "warn": "#f59e0b", "pass": "#22c55e"}

# Overlap volume (mm^3) at/above which a non-intended printed-part pair is a
# FAIL.  A correct flush mate (clamp cap on its cradle, stacked hub/bracket)
# leaves only voxel-boundary noise well under this; the clamp-cap lip bug this
# guards against was ~560 mm^3.
CHECK_OVERLAP_MM3 = 80.0
CHECK_PITCH_MM = 2.0

# --- mass model (BuildViz get_mass_properties + mesh_mujoco) ----------------
# Real masses for BOUGHT parts (grams per instance) so the hub's mass tool
# stops weighing servos/batteries/bearings as solid PLA.  Spec'd where a
# datasheet exists; (est) = catalog-typical, refine by weighing.
KNOWN_PART_MASSES_G = {
    "yaw_servo": 61.0, "hip_servo": 61.0, "knee_servo": 61.0,  # STS3215 spec
    "yaw_bearing_upper": 10.0, "yaw_bearing_lower": 10.0,      # 6805-2RS
    "disc_horn_yaw": 4.0, "disc_horn_hip": 4.0, "disc_horn_knee": 4.0,
    "passive_horn_hip": 4.0, "passive_horn_knee": 4.0,   # metal disc horns
    "lipo_battery": 140.0,   # (est) ~68 cm3 hardcase pack — WEIGH to pin
    "tibia_tube": 4.5,       # Ø8x1 CF tube, ~125 mm cut (solid-CAD cylinder)
    "foot_boot": 3.0,        # TPU boot, thin walls + solid tip
    "uno_q": 42.0, "screen": 40.0, "breakout": 8.0,      # (est)
    "motor_controller": 22.0, "mpu6050": 1.5,            # (est)
    "wago_power": 9.3, "wago_trunk": 9.3, "wago_v33": 9.3,   # 221-415
    "wago_data": 5.0,                                        # 221-413
    "hex_post_magnet": 3.0,                              # Ø8x8 N52
    "chassis_standoff": 2.0, "hex_post_standoff": 2.0,   # M3 brass
}
# Effective densities (g/cm3) for volume-weighed parts.  Steel fasteners are
# volume-accurate CAD; printed parts get PLA 1.24 x an infill factor from the
# documented print settings (PROTOTYPE.md): plates/holster 25% gyroid 4 walls
# (~0.5 of solid), structural yokes/brackets/caps 30-40% gyroid 4 walls
# (~0.6 of solid).
PART_DENSITIES_GCM3 = {
    "screw_yaw": 7.85, "screw_hip": 7.85, "screw_knee": 7.85,
    "screw_retainer": 7.85, "screw_chassis": 7.85,
    "chassis_bottom": 0.62, "chassis_top": 0.62, "switch_holster": 0.62,
    "hex_mount_plate": 0.62, "hex_raised_platform": 0.62,
    "coxa_link": 0.744, "femur_link": 0.744, "tibia_knee_yoke": 0.744,
    "yaw_bearing_cap": 0.744, "yaw_servo_retainer": 0.744,
    "hip_clamp_cap": 0.744, "knee_clamp_cap": 0.744,
}

# Intended part-type matings (overlap by design): servo bodies seated in their
# cradles, the disc horn clamped on its servo output, the carbon tube epoxied
# into its sockets, and the bolt-together printed stacks that share a flush or
# bonded interface.  These are reported ``pass (allowed)`` rather than failing,
# exactly as BuildViz's ``checksConfig.ignoreOverlapPairs`` intends.  The
# sandwich-joint clamp cap <-> its bracket is DELIBERATELY *not* listed: it must
# mate flush (~0 mm^3), so a real interpenetration there still surfaces as FAIL.
INTENDED_OVERLAP_PAIRS = frozenset(
    frozenset(p) for p in [
        # Servo body fills its cradle / bracket.
        ("chassis_bottom", "yaw_servo"),
        ("coxa_link", "hip_servo"),
        ("femur_link", "knee_servo"),
        # Clamp cap PRESS-FIT onto its own servo body: the centre tongue
        # reaches CLAMP_TONGUE_INTERF = 0.5 mm PAST the seated body +Y face for
        # a snug press fit (user request, Jun 2026; 1.0 -> 0.5 Aug 26 2026 --
        # the cap top bowed around the screws; ~0.8 k mm^3 designed
        # interference).  NB: this is the servo<->cap press; the cap<->BRACKET
        # mate below stays strict (must be flush ~0 mm^3).
        ("hip_servo", "hip_clamp_cap"),
        ("knee_servo", "knee_clamp_cap"),
        # Disc horn seats on its servo output boss.
        ("yaw_servo", "disc_horn_yaw"),
        ("hip_servo", "disc_horn_hip"),
        ("knee_servo", "disc_horn_knee"),
        # Femur: ONE printed part since the Jul 2026 merge #2 (no pair
        # needed).  Tibia: carbon-fibre spar epoxied into the yoke /
        # fitting sockets.
        ("tibia_knee_yoke", "tibia_tube"),
        # BOTH clevises' pads press onto their disc-horn pairs:
        # YOKE_PAD_EXTRA_REACH (+0.5 mm/side, Aug 2026 bench fit -- closes the
        # measured ~1 mm total clevis gap on the tibia knee yoke AND the femur
        # hip yoke) on top of YOKE_SEAT_INTERF makes ~0.63 mm/side of DESIGNED
        # nominal interference (~150 mm^3 per horn), an intended preload fill.
        ("tibia_knee_yoke", "disc_horn_knee"),
        ("tibia_knee_yoke", "passive_horn_knee"),
        ("femur_link", "disc_horn_hip"),
        ("femur_link", "passive_horn_hip"),
        ("foot_boot", "tibia_tube"),
        # Yaw hub boss PRESSES both 6805 inner races (Aug 16 2026 bench
        # refit round 2: YAW_HUB_BOSS_OD = ID + 0.15 -- -0.05 barely touched and
        # +0.05 still printed at 25.0, the side-printed layer axis quantizing
        # to 0.2 mm layers).  ~320 mm^3 of designed nominal interference per
        # race in the faceted scene meshes.
        ("coxa_link", "yaw_bearing_lower"),
        ("coxa_link", "yaw_bearing_upper"),
        # Printed stacks sharing a flush/running interface (the one-piece
        # coxa_link's hub boss runs inside the stationary bearing cap).
        ("coxa_link", "yaw_bearing_cap"),
        # LiPo packs velcro'd flush against chassis_bottom's flat belly.
        ("chassis_bottom", "lipo_battery"),
    ]
)


def _pair_overlap(a_mesh, b_mesh, *, pitch, skip_below):
    """Voxel-sample the AABB intersection of two WORLD-space meshes.

    Returns ``(volume_mm3, centroid_xyz, region_min, region_max)`` or
    ``(0.0, None, None, None)`` when they do not interpenetrate.  Mirrors the
    generic estimator in tools/buildviz_checks.py / the verifier's
    ``_pair_overlap_volume`` so the numbers line up with the offline gate."""
    lo = np.maximum(a_mesh.bounds[0], b_mesh.bounds[0])
    hi = np.minimum(a_mesh.bounds[1], b_mesh.bounds[1])
    if np.any(hi <= lo):
        return 0.0, None, None, None
    span = hi - lo
    if float(np.prod(span)) <= skip_below:
        return 0.0, None, None, None
    n = np.maximum(2, np.ceil(span / pitch).astype(int))
    gx = np.linspace(lo[0], hi[0], int(n[0]))
    gy = np.linspace(lo[1], hi[1], int(n[1]))
    gz = np.linspace(lo[2], hi[2], int(n[2]))
    XX, YY, ZZ = np.meshgrid(gx, gy, gz, indexing="ij")
    pts = np.stack([XX.ravel(), YY.ravel(), ZZ.ravel()], axis=1)
    try:
        in_a = a_mesh.contains(pts)
    except Exception:
        return 0.0, None, None, None
    if not in_a.any():
        return 0.0, None, None, None
    pts_a = pts[in_a]
    try:
        in_b = b_mesh.contains(pts_a)
    except Exception:
        return 0.0, None, None, None
    n_overlap = int(in_b.sum())
    if n_overlap == 0:
        return 0.0, None, None, None
    voxel_vol = float(np.prod(span / n.astype(float)))
    inside = pts_a[in_b]
    centroid = [float(c) for c in inside.mean(axis=0)]
    rmin = [float(c) for c in inside.min(axis=0)]
    rmax = [float(c) for c in inside.max(axis=0)]
    return n_overlap * voxel_vol, centroid, rmin, rmax


def _connector_clearance_checks(items, chassis_lift: float):
    """Verify the deck electronics' wire-exit faces have free air.

    The motor controller (Waveshare Bus Servo Adapter A) has connectors on
    BOTH 42 mm edges (servo/UART plugs one side, screw terminal + USB-C the
    other) and the GY-521 has a right-angle header row on one long edge --
    if anything sits inside those zones the wires cannot be plugged in
    (user, Aug 2026).  Each zone is an on-deck prism off the connector face
    (constants in hexapod_prototype.py); zones must also stay inside the
    chassis_top disk so wires never dangle into the leg-sweep airspace.

    ``items`` = [(id, partType, world_mesh)] non-fastener solids; the
    meshes are in WORLD frame (standing pose), so the deck height gets
    ``chassis_lift`` added on top of the chassis-local deck z.
    """
    from trimesh.creation import box as _box_mesh

    deck0 = HP.CHASSIS_TOP_TOP_Z + chassis_lift
    checks: list[dict] = []

    mcx, mcy = HP.MOTOR_CTRL_CENTRE
    # After the 90-deg yaw the board's world X extent = D, Y extent = W.
    mhx, mhy = HP.MOTOR_CTRL_D / 2.0, HP.MOTOR_CTRL_W / 2.0
    ix, iy = HP.MPU_ASBUILT_CENTRE
    ihx, ihy = HP.IMU_PCB_D / 2.0, HP.IMU_PCB_W / 2.0

    # (label, owner partType, x-range, y-range, zone height above deck,
    #  must_stay_on_plate) -- the last flag is False only for the bench
    # USB-C plug corridor: a laptop cable is attached with the robot
    # parked/limp, so unlike permanent wiring it MAY reach past the
    # chassis_top rim into (momentarily unused) leg-sweep airspace.
    zones = [
        ("motor_controller servo/UART face (+X)", "motor_controller",
         (mcx + mhx, mcx + mhx + HP.MOTOR_CTRL_SERVO_CLEAR),
         (mcy - mhy, mcy + mhy), HP.MOTOR_CTRL_H + 4.0, True),
        ("motor_controller terminal/USB face (-X)", "motor_controller",
         (mcx - mhx - HP.MOTOR_CTRL_PWR_CLEAR, mcx - mhx),
         (mcy - mhy, mcy + mhy), HP.MOTOR_CTRL_H + 4.0, True),
        ("mpu6050 header row (-X)", "mpu6050",
         (ix - ihx - HP.MPU_WIRE_CLEAR, ix - ihx),
         (iy - ihy, iy + ihy), 12.0, True),
        ("motor_controller USB-C bench-plug corridor (-X, robot parked)",
         "motor_controller",
         (mcx - mhx - HP.MOTOR_CTRL_USB_CLEAR, mcx - mhx),
         (mcy - mhy, mcy + mhy), HP.MOTOR_CTRL_H + 4.0, False),
    ]

    for k, (label, owner, (x0, x1), (y0, y1), zh, need_on_plate) in \
            enumerate(zones):
        ext = (x1 - x0, y1 - y0, zh)
        zone = _box_mesh(extents=ext)
        # start 0.2 above the deck surface so resting on the plate never
        # counts as an obstruction
        zone.apply_translation([(x0 + x1) / 2.0, (y0 + y1) / 2.0,
                                deck0 + 0.2 + zh / 2.0])
        offenders = []
        for iid, ptype, mesh in items:
            if ptype in (owner, "chassis_top"):
                continue
            vol, _c, _rmin, _rmax = _pair_overlap(
                zone, mesh, pitch=CHECK_PITCH_MM, skip_below=40.0)
            if vol > 40.0:
                offenders.append((iid, ptype, vol))
        # containment: wires must stay over the plate, out of leg airspace
        # (waived for the bench-plug corridor -- robot parked, see zones)
        corner_r = max(np.hypot(x, y) for x in (x0, x1) for y in (y0, y1))
        on_plate = (not need_on_plate
                    or corner_r <= HP.CHASSIS_TOP_RADIUS + 0.5)
        ok = not offenders and on_plate
        detail = "clear" if ok else (
            "; ".join(f"{pt} intrudes {v:.0f} mm\u00b3"
                      for _i, pt, v in offenders)
            + ("" if on_plate else
               f" zone reaches r={corner_r:.1f} > disk {HP.CHASSIS_TOP_RADIUS}"))
        rec = {
            "id": f"connector_clearance-{k}",
            "kind": "connector_clearance",
            "status": "pass" if ok else "fail",
            "label": f"{label}: {detail}",
            "region": {"min": [x0, y0, deck0], "max": [x1, y1, deck0 + zh]},
        }
        if offenders:
            rec["instances"] = [i for i, _p, _v in offenders]
        checks.append(rec)
    return checks


def _build_checks_sidecar(tagged, instances_json, chassis_lift: float):
    """Compute the generic ``mesh_overlap`` + ``scene_meta`` checks over the
    placed scene and return the BuildViz sidecar dict (its exact schema)."""
    # tagged[idx] world mesh is aligned 1:1 with instances_json[idx].
    items = []
    for inst, (name, _leg, mesh) in zip(instances_json, tagged):
        # Fasteners occupy their holes by design (BuildViz suppresses them by
        # default); skip them from the interpenetration scan.
        if inst.get("role") == "fastener":
            continue
        items.append((inst["id"], inst["partType"], mesh))

    checks: list[dict] = []

    # scene_meta: every instance resolves to a mesh + ids are unique (the
    # builder guarantees this, so it is a green hygiene line in the panel).
    ids = [i["id"] for i in instances_json]
    dupes = sorted({x for x in ids if ids.count(x) > 1})
    checks.append({
        "id": "scene_meta-unique-ids",
        "kind": "scene_meta",
        "status": "pass" if not dupes else "fail",
        "label": (f"{len(ids)} instances, ids unique"
                  if not dupes else f"duplicate ids: {dupes}"),
    })

    # mesh_overlap: pairwise interpenetration with an AABB pre-filter.
    overlaps = []
    for i in range(len(items)):
        ida, pta, ma = items[i]
        amin, amax = ma.bounds
        for j in range(i + 1, len(items)):
            idb, ptb, mb = items[j]
            bmin, bmax = mb.bounds
            if np.any(amax < bmin) or np.any(bmax < amin):
                continue
            vol, centroid, rmin, rmax = _pair_overlap(
                ma, mb, pitch=CHECK_PITCH_MM, skip_below=CHECK_OVERLAP_MM3)
            if vol <= CHECK_OVERLAP_MM3:
                continue
            overlaps.append((vol, ida, pta, idb, ptb, centroid, rmin, rmax))

    overlaps.sort(key=lambda r: -r[0])
    for k, (vol, ida, pta, idb, ptb, centroid, rmin, rmax) in enumerate(overlaps):
        allowed = frozenset((pta, ptb)) in INTENDED_OVERLAP_PAIRS
        status = "pass" if allowed else "fail"
        suffix = " (allowed)" if allowed else ""
        rec = {
            "id": f"mesh_overlap-{k}",
            "kind": "mesh_overlap",
            "status": status,
            "label": f"{pta} \u2229 {ptb} = {vol:.0f} mm\u00b3 interpenetration{suffix}",
            "instances": [ida, idb],
        }
        if centroid is not None:
            rec["point"] = centroid
        if rmin is not None and rmax is not None:
            rec["region"] = {"min": rmin, "max": rmax}
        checks.append(rec)

    # connector_clearance: wire-exit faces of the deck electronics must
    # have free air (and stay over the plate).
    checks.extend(_connector_clearance_checks(items, chassis_lift))

    # summary + highlights, mirroring buildvizChecks.summarizeChecks /
    # checksToHighlights so the panel badge + overlays match a CLI --emit run.
    summary = {"total": len(checks), "pass": 0, "warn": 0, "fail": 0, "byKind": {}}
    for c in checks:
        summary[c["status"]] += 1
        bk = summary["byKind"].setdefault(c["kind"], {"pass": 0, "warn": 0, "fail": 0})
        bk[c["status"]] += 1

    parts: dict[str, dict] = {}
    points: list[dict] = []
    regions: list[dict] = []
    for c in checks:
        if c["status"] == "pass":
            continue
        color = CHECK_STATUS_COLOR[c["status"]]
        for iid in c.get("instances", []):
            parts.setdefault(iid, {"instanceId": iid, "color": color,
                                   "annotation": c["label"]})
        if "region" in c:
            regions.append({"min": c["region"]["min"], "max": c["region"]["max"],
                            "color": color, "annotation": c["label"]})
        if "point" in c:
            pt = {"point": c["point"], "color": color, "annotation": c["label"]}
            if "region" not in c and c.get("instances"):
                pt["instanceId"] = c["instances"][0]
            points.append(pt)

    return {
        "generatedAt": __import__("datetime").datetime.now(
            __import__("datetime").timezone.utc).isoformat(),
        "build": {"id": SCENE_BUILD_ID, "name": "Hexapod STS3215 — full robot"},
        "generatedBy": "full_robot_viz_build.py (precomputed offline)",
        "checksConfig": {
            "overlapMm3": CHECK_OVERLAP_MM3,
            "pitchMm": CHECK_PITCH_MM,
            "ignoreOverlapPairs": sorted(sorted(p) for p in INTENDED_OVERLAP_PAIRS),
        },
        "summary": summary,
        "checks": checks,
        "highlights": {"parts": list(parts.values()), "points": points,
                       "regions": regions},
    }


# ---------------------------------------------------------------------------
# LOCAL-FRAME part producers (so BuildViz reports CAD-local coordinates)
# ---------------------------------------------------------------------------
#
# BuildViz's pick read-out shows ``local XYZ`` via ``mesh.worldToLocal`` -- the
# inverse of each instance's ``transform``.  If we bake the world placement into
# the STL and ship an identity transform (the old behaviour), that read-out
# equals WORLD mm, which does NOT match any single part's design frame and made
# it painful to talk to an LLM about feature positions.
#
# Instead every producer below returns ``(name, local_mesh, M0)`` where:
#   * ``local_mesh`` is the RAW ``make_*()`` part in its OWN design frame -- the
#     identical frame as ``stl_prototype/<part>.stl`` and the ``make_*`` source
#     code -- so picking a point reports coordinates an LLM can act on directly.
#   * ``M0`` is the 4x4 that maps that local frame into the leg-0 world frame
#     (apothem a=pi/6), BEFORE the per-leg yaw rotation and standing-pose lift
#     (both applied in ``main`` and folded into the per-instance ``transform``).
# World placement is ``M0 @ local`` and reproduces EXACTLY the poses the
# verifier checks (asserted in ``main`` against ``_verify_prototype``).

# Multiple instances share one identical local STL (all six legs print the same
# coxa hub; every servo well holds the same body envelope; etc).  This map
# collapses those onto a single exported mesh; unlisted names key on themselves.
_SCENE_MESH_KEY = {
    "yaw_servo": "servo_body",
    "hip_servo": "servo_body",
    "knee_servo": "servo_body",
    "hip_clamp_cap": "servo_clamp_cap",
    "knee_clamp_cap": "servo_clamp_cap",
    "wago_trunk": "wago5",
    "wago_v33": "wago5",
    "wago_power": "wago5",
    "wago_data": "wago",
}


def _scene_mesh_key(name: str) -> str:
    return _SCENE_MESH_KEY.get(name, name)


def _leg0_local_link_parts() -> list[tuple[str, trimesh.Trimesh, np.ndarray]]:
    """``(name, local_mesh, M0)`` for every leg-0 printed link part.

    The femur/tibia are decomposed into their REAL printed sandwich parts
    (yoke + dia-8 CF tube + bracket/fitting) so the assembly calls out every
    printable part instead of the merged link proxies.  ``M0`` mirrors the
    per-link world transform ``_build_standing_leg`` uses, so ``M0 @ local``
    occupies the identical space the verifier checks for self-collision."""
    a = 0.5 * np.pi / 3.0
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    edge = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0]) \
        + HP.CHASSIS_YAW_OUTPUT_Z * np.array([0.0, 0.0, 1.0])
    p = np.deg2rad(HP.STANCE_FEMUR_DEG)
    pt = np.deg2rad(HP.STANCE_FEMUR_DEG + HP.STANCE_TIBIA_DEG)
    hip_local = np.array(HP.COXA_HIP_ANCHOR)
    knee_local = hip_local + rotation_matrix(p, [0, 1, 0])[:3, :3] \
        @ np.array([HP.FEMUR_LENGTH, 0.0, 0.0])

    # World transform of each link's local frame (mirrors _build_standing_leg).
    Rz = rotation_matrix(a, [0, 0, 1])
    T_coxa = _trans(edge) @ Rz
    T_femur = _trans(edge) @ Rz @ _trans(hip_local) @ rotation_matrix(p, [0, 1, 0])
    T_tibia = _trans(edge) @ Rz @ _trans(knee_local) @ rotation_matrix(pt, [0, 1, 0])

    xz = (1, 0, 0), HP.LEG_PITCH_AXIS
    # Femur frame inside the femur-link-local frame (== make_femur_link).
    # Jul 2026 merge #2: the whole femur is ONE printed part (femur_link).
    Mh = HP._joint_place((0.0, 0.0, 0.0), *xz)

    # Tibia sub-part frames inside the tibia-link-local frame (== make_tibia_link).
    Mk0 = HP._joint_place((0.0, 0.0, 0.0), *xz)
    ta = (Mk0 @ np.array([HP._YOKE_SOCKET_X, 0.0, HP.JOINT_SOCKET_Z, 1.0]))[:3]
    tube_end = ta + np.array([HP.TIBIA_LENGTH - 8.0, 0.0, 0.0])
    foot_frame = HP._frame(tube_end, (1, 0, 0), (0, 0, 1))
    ttube = HP._tube_between(ta, tube_end, HP.LEG_TUBE_OD / 2.0)

    return [
        # Coxa: ONE printed part (Aug 2026 merge), plus the TOUCHING 6805
        # bearing pair (visual, NOT printed) so the bearing-supported yaw
        # joint shows.
        ("coxa_link", HP.make_coxa_link_part(), T_coxa),
        ("yaw_bearing_cap", HP.make_yaw_bearing_cap(), T_coxa),
        ("yaw_bearing_lower", HP.make_yaw_bearing_lower(), T_coxa),
        ("yaw_bearing_upper", HP.make_yaw_bearing_upper(), T_coxa),
        # Femur: ONE printed part (hip yoke + solid spar + knee cradle).
        ("femur_link", HP.make_femur_link_part(), T_femur @ Mh),
        # Tibia sandwich (knee yoke + CF spar + pressed-on TPU boot,
        # Aug 2026 -- the hinged foot fitting + pad are retired).
        ("tibia_knee_yoke", HP.make_tibia_knee_yoke(), T_tibia @ Mk0),
        ("tibia_tube", ttube, T_tibia),
        ("foot_boot", HP.make_foot_boot(), T_tibia @ foot_frame),
    ]


def _servo_M0s() -> dict[str, np.ndarray]:
    """Well-frame -> leg-0 frame placement matrices for the three STS3215
    servo bodies (+ the yaw retainer frame).

    Mirrors the authoritative placement chains in
    ``_verify_prototype._place_servo_bodies`` / ``_place_yaw_retainers``;
    ``main`` asserts the resulting poses match those functions exactly, so
    any drift in the duplicated math is caught at build time.  Shared by
    ``_servo_local_parts`` (instance placement) and the wire-attach route
    builder (the servo boot / cradle exit-slot waypoints live in this same
    well frame)."""
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    a = 0.5 * np.pi / 3.0
    edge_mid = np.array([apothem * np.cos(a), apothem * np.sin(a), 0.0])
    R_a = rotation_matrix(a, [0, 0, 1])
    yaw_output_world = edge_mid + HP.CHASSIS_YAW_OUTPUT_Z * np.array([0.0, 0.0, 1.0])
    hip_local = np.array(HP.COXA_HIP_ANCHOR)
    p = np.deg2rad(HP.STANCE_FEMUR_DEG)

    M0_yaw = _trans(edge_mid) @ R_a @ _trans(
        [-HP.SERVO_OUTPUT_X, 0.0,
         HP.CHASSIS_YAW_OUTPUT_Z - HP.HORN_STACK_H - HP.WELL_RIM_Z])
    M_hip = HP._joint_place(HP.COXA_HIP_ANCHOR, (1, 0, 0), HP.LEG_PITCH_AXIS)
    M0_hip = _trans(yaw_output_world) @ R_a @ M_hip
    M_knee = HP._joint_place((HP.FEMUR_LENGTH, 0.0, 0.0), (1, 0, 0), HP.LEG_PITCH_AXIS)
    M0_knee = _trans(yaw_output_world) @ R_a @ _trans(hip_local) \
        @ rotation_matrix(p, [0, 1, 0]) @ M_knee
    M0_ret = _trans(edge_mid) @ R_a
    return {"yaw": M0_yaw, "hip": M0_hip, "knee": M0_knee, "retainer": M0_ret}


def _servo_local_parts() -> list[tuple[str, trimesh.Trimesh, np.ndarray]]:
    """``(name, local_mesh, M0)`` for the leg-0 servo bodies, sandwich clamp
    caps, and the yaw capture stirrup (placement matrices from
    ``_servo_M0s``)."""
    M0 = _servo_M0s()
    return [
        ("yaw_servo", V._load_mesh("servo_body"), M0["yaw"]),
        ("hip_servo", V._load_mesh("servo_body"), M0["hip"]),
        ("knee_servo", V._load_mesh("servo_body"), M0["knee"]),
        ("hip_clamp_cap", HP.make_servo_clamp_cap(), M0["hip"]),
        ("knee_clamp_cap", HP.make_servo_clamp_cap(), M0["knee"]),
        ("yaw_servo_retainer", HP.make_yaw_servo_retainer(), M0["retainer"]),
    ]


def _leg0_local_parts() -> list[tuple[str, trimesh.Trimesh, np.ndarray]]:
    """All leg-0 parts as ``(name, local_mesh, M0)`` (links + servos/caps/strap)."""
    return _leg0_local_link_parts() + _servo_local_parts()


def _assert_servo_placement(tol: float = 1e-4) -> None:
    """Guard: the duplicated servo/cap/strap placement math reproduces the
    authoritative ``_verify_prototype`` poses (bounds match to ``tol`` mm)."""
    ref: dict[str, trimesh.Trimesh] = {}
    ref.update(V._place_servo_bodies())
    ref.update(V._place_servo_clamp_caps())
    ref.update(V._place_yaw_retainers())
    for name, local, M0 in _servo_local_parts():
        placed = local.copy()
        placed.apply_transform(M0)
        drift = float(np.abs(np.asarray(placed.bounds)
                             - np.asarray(ref[name].bounds)).max())
        if drift > tol:
            raise AssertionError(
                f"{name}: local-frame placement drifts {drift:.3e} mm from "
                f"_verify_prototype authoritative pose")


def _leg0_parts() -> list[tuple[str, trimesh.Trimesh]]:
    """(verifier-facing) leg-0 parts as PLACED ``(name, world_mesh)`` in the
    leg-0 world frame -- a thin wrapper over ``_leg0_local_parts`` so the
    verifier and the local-coordinate scene share one source of truth."""
    out: list[tuple[str, trimesh.Trimesh]] = []
    for name, local, M0 in _leg0_local_parts():
        m = local.copy()
        m.apply_transform(M0)
        out.append((name, m))
    return out


def _axis_to_transform(axis, origin) -> np.ndarray:
    """Map mesh-local +Z onto ``axis`` with the mesh origin at ``origin``
    (fastener cache convention: +Z = shaft, origin = head face)."""
    z = np.asarray(axis, float)
    n = float(np.linalg.norm(z))
    z = z / n if n > 1e-12 else np.array([0.0, 0.0, 1.0])
    seed = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = seed - z * float(np.dot(seed, z))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, np.asarray(origin, float)
    return T


def _primitive_bolt(fi) -> trimesh.Trimesh:
    """Fallback capped-cylinder bolt for any fastener lacking a cache STL.

    Mesh-local +Z = shaft, origin = head face (matches the cache convention
    consumed by ``_axis_to_transform``): a flat head disc at z in [0, head_t]
    and a shank running down -Z for ``length_mm``."""
    from trimesh.creation import cylinder as _cyl_mesh
    dia = float(getattr(fi, "nominal_dia_mm", 3.0) or 3.0)
    length = float(getattr(fi, "length_mm", 8.0) or 8.0)
    head_d, head_t = dia * 1.8, max(0.6 * dia, 1.0)
    head = _cyl_mesh(radius=head_d / 2.0, height=head_t)
    head.apply_translation([0, 0, -head_t / 2.0])      # head top at z=0
    shank = _cyl_mesh(radius=dia / 2.0, height=length)
    shank.apply_translation([0, 0, -head_t - length / 2.0])
    bolt = trimesh.util.concatenate([head, shank])
    # Flip so +Z points along the shaft (down into the part) like the caches.
    bolt.apply_transform(rotation_matrix(np.pi, [1, 0, 0]))
    return bolt


def _fastener_instances():
    """Yield ``(name, leg, joint, color, mesh_key, local_mesh, M)`` for EVERY
    non-virtual fastener in the registry -- leg-joint screws (yaw/hip/knee) AND
    chassis-level / foot / deck hardware (registry ``joint is None``).  COTS,
    scene-only.  ``local_mesh`` is the cached bolt in its native frame (+Z =
    shaft, origin = head face) and ``M`` maps it into the per-leg WORLD frame
    (registry-supplied, pre-lift); the standing-pose lift is folded in by
    ``main``."""
    for fi in FR.build_all_fastener_instances():
        if getattr(fi, "is_virtual", False):
            continue
        cache = FASTENERS_DIR / fi.cache_stl if fi.cache_stl else None
        if cache is not None and cache.is_file():
            mesh = trimesh.load(cache, process=False)
            if isinstance(mesh, trimesh.Scene):
                mesh = trimesh.util.concatenate(list(mesh.geometry.values()))
            mesh_key = f"fastener_{cache.stem}"
        else:
            mesh = _primitive_bolt(fi)
            mesh_key = (f"fastener_prim_{getattr(fi, 'nominal_dia_mm', 3.0)}"
                        f"_{getattr(fi, 'length_mm', 8.0)}")
        M = _axis_to_transform(fi.axis_world, fi.head_world_xyz)
        # The yaw saddle's chassis anchors + rear-case capture screws hold
        # the STATIC yaw_servo_retainer / servo case (only the output spins),
        # but the registry tags them joint="yaw" -- the blanket
        # screw_yaw->yaw-link motion mapping made them swing away from the
        # stationary saddle in every pose / walk clip (mirror image of the
        # Jul 2026 "floating foot screws" bug).  Route them to their own
        # partType, which no motion link claims, so they stay put.
        if "yaw_servo_retainer" in fi.role:
            yield ("screw_retainer", fi.leg_index, "chassis",
                   FASTENER_CHASSIS_COLOR, mesh_key, mesh, M)
            continue
        joint_key = fi.joint if fi.joint in FASTENER_JOINT_COLOR else "chassis"
        color = FASTENER_JOINT_COLOR.get(fi.joint, FASTENER_CHASSIS_COLOR)
        yield f"screw_{joint_key}", fi.leg_index, joint_key, color, mesh_key, mesh, M


def _disc_horn_instances(legs: list[int]):
    """Yield ``(name, leg, joint, color, mesh_key, local_mesh, M)`` for the
    Phi20 aluminium disc horn seated on EACH servo output (yaw/hip/knee) of
    every leg.

    Authoritative pose: ``_verify_prototype._horn_world_transform`` maps the
    horn-local frame of ``make_disc_horn`` (origin = spline/servo face, +Z =
    output-shaft axis) into the per-leg world frame -- the SAME transform the
    verifier's disc-horn-fit / fastener-engagement checks use, so the horns
    land exactly on the servo output bosses the link bolts clamp to.  The
    standing-pose lift is folded in by ``main``."""
    for joint in V._HORN_JOINTS:
        for leg in legs:
            M = V._horn_world_transform(joint, leg)
            yield (f"disc_horn_{joint}", leg, joint, DISC_HORN_COLOR,
                   "disc_horn", HP.make_disc_horn(), M)


def _passive_horn_instances(legs: list[int]):
    """Yield the PASSIVE rear-boss STOCK disc horn for each sandwich joint
    (hip/knee) of every leg -- the symmetric-yoke refit.  Jul 2026 stock-horn
    refit: the horn's centre bore rides the rear idler boss directly (flush
    on the back face); the printed centering adapter is retired.  Poses come
    from the SAME verifier transform the passive-horn fastener-engagement
    check uses, so the horn lands coaxial on the servo's rear idler boss."""
    for joint in V._PASSIVE_HORN_JOINTS:
        for leg in legs:
            yield (f"passive_horn_{joint}", leg, joint, DISC_HORN_COLOR,
                   "disc_horn", HP.make_disc_horn(),
                   V._passive_horn_world_transform(joint, leg))


def _body_local_parts() -> list[tuple[str, trimesh.Trimesh, np.ndarray]]:
    """``(name, local_mesh, M0)`` for the chassis / electronics body parts.

    Each ``local_mesh`` is the raw ``make_*()`` part (matching its
    ``stl_prototype`` STL) and ``M0`` is its PRE-LIFT stack offset along Z (the
    standing-pose lift is added in ``main``).  Mirrors ``make_assembly_preview``
    so the body sits exactly as the printed stack assembles.

    Aug 2026 as-built: trays / carapace / imu_pad retired; electronics come
    from ``HP.asbuilt_electronics_local_parts``.
    """
    # The two under-belly LiPo packs come from
    # HP.asbuilt_electronics_local_parts (Aug 2026: no bay battery).
    parts: list[tuple[str, trimesh.Trimesh, np.ndarray]] = [
        ("chassis_bottom", HP.make_chassis_bottom(), np.eye(4)),
        ("chassis_top", HP.make_chassis_top(),
         _trans([0, 0, HP.CHASSIS_TOP_CENTRE_Z])),
        # Velcroed flat on the deck (Aug 16 2026: insert bosses retired).
        ("switch_holster", HP.make_switch_holster(),
         _trans([HP.SWITCH_HOLSTER_CENTRE_X, HP.SWITCH_HOLSTER_CENTRE_Y,
                 HP.CHASSIS_TOP_TOP_Z])),
    ]
    parts.extend(HP.asbuilt_electronics_local_parts())
    return parts


def _body_parts(chassis_lift: float) -> list[tuple[str, trimesh.Trimesh]]:
    """(verifier-facing) body parts as PLACED ``(name, world_mesh)`` lifted by
    ``chassis_lift`` -- a thin wrapper over ``_body_local_parts``."""
    Tlift = _trans([0.0, 0.0, chassis_lift])
    out: list[tuple[str, trimesh.Trimesh]] = []
    for name, local, M0 in _body_local_parts():
        m = local.copy()
        m.apply_transform(Tlift @ M0)
        out.append((name, m))
    return out


def _col_major(M: np.ndarray) -> list[float]:
    """Flatten a 4x4 row-major numpy matrix into the column-major 16-array
    BuildViz feeds to ``THREE.Matrix4().fromArray`` (translation at 12..14)."""
    return [float(v) for v in np.asarray(M, float).flatten("F")]


# ---------------------------------------------------------------------------
# Motion model (BuildViz ``joints[]`` + ``poses[]`` + ``animations[]``)
# ---------------------------------------------------------------------------
#
# These blocks are ALWAYS baked into the single ``scene.json`` (mirroring the
# prototype_v1 ``hexapod-prototype`` build): generating them is essentially free
# (joint/pose math on the already-placed meshes), so there is no reason to gate
# GENERATION or split them into a second build.  Only the swept self-overlap
# VALIDATION (``buildviz sweep``) is expensive and is opt-in on push.
#
# BuildViz drives motion from three OPTIONAL top-level scene keys (see
# /Users/lbiewald/buildviz BUILDVIZ_INTEGRATION.md "Motion / kinematics" and
# src/buildvizKinematics.ts).  Old viewers ignore them; a viewer that
# understands them gets per-joint sliders, an "Animate sweep" button (each DOF
# swept min->max->min), clickable named poses, and a play/scrub timeline for the
# keyframed ``animations[]`` clip, plus swept-pose overlap validation.  The
# forward-kinematics the viewer computes is EXACTLY:
#
#   L(joint, v) = T(origin) * R(axis, v-home) * T(-origin)     [revolute, deg]
#   C(j)        = C(parent) * L(j)                             [chain compose]
#   world(inst) = C(j) * inst.transform                       [home = static]
#
# so ``axis``/``origin`` are in the SCENE (world home) frame and a joint's
# ``instances`` list holds ONLY its own distal link (ancestors live on the
# parent joints).  The static scene IS the home pose (every joint value 0), so
# we can reuse the verified standing-pose ``transform``s untouched and only emit
# the joint axes/pivots + which instances each joint carries.
#
# The yaw/hip/knee axes and pivots below are derived from the SAME authoritative
# leg-0 kinematic chain ``_leg0_local_link_parts`` / ``_servo_local_parts`` use
# (apothem direction a=pi/6; the static ``T_coxa``/``T_femur``/``T_tibia``
# placement), then mapped into each leg's world frame by the per-leg yaw
# rotation + standing-pose lift -- so the rotation a slider applies lands on the
# real joint axis through the real pivot, not a centroid guess.

# partType (instance ``name``) -> the kinematic LINK it rigidly belongs to.
# An instance moves with the DEEPEST joint listing it; with the yaw->hip->knee
# parent chain, listing each part on its own link composes correctly:
#   * yaw  link = coxa turntable + the hip servo BODY bolted to it (the hip
#                 servo's body rides the coxa; its output drives the femur).
#   * hip  link = femur sandwich + hip horns + the knee servo BODY on the femur.
#   * knee link = tibia sandwich + knee horns + foot.
# The yaw servo body / its chassis retainer stay FIXED (chassis), as do all
# body/electronics parts -- they map to None and never move.
_MOTION_LINK_OF_PARTTYPE = {
    # --- yaw link (rotates the whole leg about vertical Z) ---
    "coxa_link": "yaw",
    "yaw_bearing_cap": "yaw", "yaw_bearing_lower": "yaw",
    "yaw_bearing_upper": "yaw", "disc_horn_yaw": "yaw",
    "hip_servo": "yaw", "hip_clamp_cap": "yaw", "screw_yaw": "yaw",
    # --- hip link (femur swings in its vertical plane) ---
    "femur_link": "hip",
    "disc_horn_hip": "hip", "passive_horn_hip": "hip",
    "knee_servo": "hip",
    "knee_clamp_cap": "hip", "screw_hip": "hip",
    # --- knee link (tibia + foot) ---
    "tibia_knee_yoke": "knee", "tibia_tube": "knee",
    "foot_boot": "knee",
    "disc_horn_knee": "knee", "passive_horn_knee": "knee",
    "screw_knee": "knee",
}

# Slider clamps in DEGREES of offset from the home stance (home = 0).  Kept
# inside the real STS3215 leg envelope the MuJoCo model / workspace sweep use
# (yaw +/-35; femur absolute in [-80, 30] deg about the home -25; tibia relative
# in [0, 80] deg about the home 60), so an "Animate sweep" stays in a reachable,
# physical range.
_MOTION_LIMITS = {"yaw": (-35.0, 35.0), "hip": (-45.0, 30.0), "knee": (-30.0, 20.0)}


def _leg0_joint_frames() -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """``{joint: (axis, origin)}`` in the LEG-0 world frame (pre per-leg yaw and
    pre standing-pose lift), matching the static placement chain exactly.

    * yaw  spins about world +Z through the yaw output (``edge``).
    * hip  swings about the coxa-rotated tangent ``Rz @ +Y`` through the hip
      anchor ``edge + Rz @ COXA_HIP_ANCHOR`` (the pivot of ``T_femur``).
    * knee swings about the same tangent through the knee anchor
      ``edge + Rz @ knee_local`` (the pivot of ``T_tibia``).
    """
    a = 0.5 * np.pi / 3.0
    apothem = HP.CHASSIS_FLAT_TO_FLAT / 2.0
    edge = np.array([apothem * np.cos(a), apothem * np.sin(a),
                     HP.CHASSIS_YAW_OUTPUT_Z])
    Rz = rotation_matrix(a, [0, 0, 1])[:3, :3]
    hip_local = np.array(HP.COXA_HIP_ANCHOR, float)
    p = np.deg2rad(HP.STANCE_FEMUR_DEG)
    knee_local = hip_local + rotation_matrix(p, [0, 1, 0])[:3, :3] \
        @ np.array([HP.FEMUR_LENGTH, 0.0, 0.0])
    tangent = Rz @ np.array([0.0, 1.0, 0.0])
    return {
        "yaw": (np.array([0.0, 0.0, 1.0]), edge.copy()),
        "hip": (tangent, edge + Rz @ hip_local),
        "knee": (tangent, edge + Rz @ knee_local),
    }


def _build_motion(instances_json, chassis_lift, legs):
    """Build the additive ``(joints, poses, animations)`` motion blocks for the
    placed scene.  ``joints`` carry real per-leg yaw/hip/knee axes + pivots in
    the final world frame and the exact instance ids each drives; ``poses`` are a
    handful of named whole-body configurations (a tripod-gait march + stance
    variants) expressed as degree offsets from the home stance; ``animations``
    is a single looping tripod-walk clip so the assembly can be seen WALKING."""
    frames = _leg0_joint_frames()

    # Group placed instance ids by (leg, link) using the authoritative ids the
    # builder already assigned (idx-prefixed, e.g. "045-hip_servo").
    ids_by: dict[tuple[int, str], list[str]] = {}
    for inst in instances_json:
        leg = inst.get("leg")
        if leg is None:
            continue
        part = inst["partType"]
        # Foot-hinge hardware (the M3x16 hinge bolt + nyloc nut) comes out of
        # the registry with joint=None, so it lands in the generic
        # "screw_chassis" bucket -- but it is LEG-tagged and rides the tibia's
        # foot fitting, so it must swing with the knee link.  Without this the
        # 2 bolts/leg stayed at their static stance position and floated in
        # mid-air whenever a pose or the walk clip moved the leg (the Jul 2026
        # "floating foot screws" bug; every OTHER leg-tagged screw_chassis is
        # also foot hardware, so the blanket mapping is safe).
        link = ("knee" if part == "screw_chassis"
                else _MOTION_LINK_OF_PARTTYPE.get(part))
        if link is None:
            continue
        ids_by.setdefault((leg, link), []).append(inst["id"])

    joints: list[dict] = []
    parent_of = {"yaw": None, "hip": "yaw", "knee": "hip"}
    for i in legs:
        Ri = rotation_matrix(i * np.pi / 3.0, [0, 0, 1])[:3, :3]
        lift = np.array([0.0, 0.0, chassis_lift])
        for jname in ("yaw", "hip", "knee"):
            ids = ids_by.get((i, jname))
            if not ids:
                continue
            axis0, origin0 = frames[jname]
            axis = Ri @ axis0
            origin = Ri @ origin0 + lift
            lo, hi = _MOTION_LIMITS[jname]
            joint = {
                "id": f"L{i}-{jname}",
                "type": "revolute",
                "axis": [float(v) for v in axis],
                "origin": [float(v) for v in origin],
                "instances": ids,
                "limits": {"min": lo, "max": hi},
                "home": 0,
                "label": f"L{i} {jname}",
            }
            parent = parent_of[jname]
            if parent is not None:
                joint["parent"] = f"L{i}-{parent}"
            joints.append(joint)

    poses = _build_motion_poses(legs)
    animations = _build_motion_animations(legs)
    return joints, poses, animations


def _build_motion_poses(legs):
    """Named whole-body poses (degree offsets from the home stance).

    Sign convention is tied to the static ``R(+Y, phi)`` femur/tibia placement,
    where leg-local foot height is ``-FEMUR*sin(phi_femur) - TIBIA*sin(phi_tibia)``:
    a NEGATIVE hip offset rotates the femur tip UP (lifts the foot), so the
    tripod-swing poses lift legs {0,2,4} / {1,3,5} on alternate phases.  These
    are demonstrative keyframes; the continuous "Animate sweep" drives every DOF
    directly from the joint axes above."""
    A = [i for i in legs if i % 2 == 0]
    B = [i for i in legs if i % 2 == 1]

    def fill(which, *, yaw=0.0, hip=0.0, knee=0.0):
        out: dict[str, float] = {}
        for i in which:
            out[f"L{i}-yaw"] = yaw
            out[f"L{i}-hip"] = hip
            out[f"L{i}-knee"] = knee
        return out

    return [
        {"id": "home", "name": "Home (verified stance)", "jointValues": {}},
        {"id": "tripodA", "name": "Tripod A swing (legs 0·2·4 lift)",
         "jointValues": fill(A, hip=-18.0)},
        {"id": "tripodB", "name": "Tripod B swing (legs 1·3·5 lift)",
         "jointValues": fill(B, hip=-18.0)},
        {"id": "stepA", "name": "Tripod A step forward",
         "jointValues": {**fill(A, hip=-18.0, yaw=14.0),
                         **fill(B, yaw=-10.0)}},
        {"id": "stepB", "name": "Tripod B step forward",
         "jointValues": {**fill(B, hip=-18.0, yaw=14.0),
                         **fill(A, yaw=-10.0)}},
        {"id": "crouch", "name": "Crouch (body low)",
         "jointValues": fill(legs, hip=-12.0, knee=-12.0)},
        {"id": "tall", "name": "Stand tall",
         "jointValues": fill(legs, hip=10.0, knee=18.0)},
        {"id": "splay", "name": "Yaw splay",
         "jointValues": fill(legs, yaw=28.0)},
    ]


# Tripod-walk clip tuning (degree offsets from the home stance; matches the
# _MOTION_LIMITS envelope so the clip never drives a joint out of range).
_WALK_YAW_AMP = 14.0      # +/- fore/aft stride yaw
_WALK_LIFT_DEG = 18.0     # swing-phase hip lift (femur tip up = foot up)
_WALK_FRAMES = 24         # keyframes per cycle (inclusive end closes the loop)
_WALK_SECONDS = 2.4       # clip playback length for one gait cycle


def _walk_leg_offset(phase: float) -> tuple[float, float]:
    """``(yaw, hip)`` degree offset for one leg at gait ``phase`` in [0, 1).

    Stance half (phase 0..0.5): foot planted, yaw sweeps fore->aft as the body
    drives forward (hip level).  Swing half (0.5..1): foot lifts (smooth
    ``hip = -LIFT*sin``, negative = femur tip up) and recovers aft->fore."""
    if phase < 0.5:
        s = phase / 0.5
        return _WALK_YAW_AMP - 2.0 * _WALK_YAW_AMP * s, 0.0
    s = (phase - 0.5) / 0.5
    return -_WALK_YAW_AMP + 2.0 * _WALK_YAW_AMP * s, -_WALK_LIFT_DEG * np.sin(np.pi * s)


def _build_motion_animations(legs):
    """A single looping tripod-walk clip built from the same per-joint DOFs.

    Tripod A (legs 0·2·4) and B (legs 1·3·5) run half a cycle out of phase, so
    one tripod plants + drives the body forward while the other lifts + swings.
    Keyframes are degree offsets from the home stance (knee held at home), so the
    clip plays inside the verified joint envelope.  Mirrors the prototype_v1
    ``hexapod-prototype`` build's keyframed ``animations[]`` walk clip."""
    keyframes = []
    for k in range(_WALK_FRAMES + 1):      # inclusive end closes the loop
        frac = k / _WALK_FRAMES
        values: dict[str, float] = {}
        for i in legs:
            phase = (frac + (0.0 if i % 2 == 0 else 0.5)) % 1.0
            yaw, hip = _walk_leg_offset(phase)
            values[f"L{i}-yaw"] = round(float(yaw), 3)
            values[f"L{i}-hip"] = round(float(hip), 3)
            values[f"L{i}-knee"] = 0.0
        keyframes.append({"t": round(frac * _WALK_SECONDS, 4),
                          "jointValues": values})
    return [{
        "id": "walk",
        "name": "Walk (tripod gait)",
        "loop": True,
        "duration": _WALK_SECONDS,
        "keyframes": keyframes,
    }]


# ---------------------------------------------------------------------------
# Cable/harness routes (BuildViz ``routes[]`` -> the routing_reach gate)
# ---------------------------------------------------------------------------
#
# Per-leg servo leads = the AS-BUILT topology of WIRING.md §6.2 (Aug 2026
# bench build): DATA (S+GND) enters the leg at the YAW; POWER (V+/GND)
# enters at the HIP, which is the power tee feeding yaw + knee.  Every
# servo-end waypoint lands on the REAL STS3215 dual 5264 port cluster
# (BACK / idler face, centre/−X half — see WIRING.md + HP.STS3215_PORT_*).
#
# Drawn as short leads that ride the structure (Aug 2026):
#   * TAIL (data entry, S+GND only): data Wago at the drop slot -> STRAIGHT
#     DOWN through the yaw retainer window (below the hanging body) -> yaw
#     BACK ports.
#   * yaw -> hip (S+GND+V+): yaw BACK ports -> under the case, out the
#     saddle's OPEN +X end -> up past the bearing cap between its ear
#     lugs -> hip BACK ports.
#   * POWER TEE (V+/GND): corner power-Wago branch at the drop slot ->
#     same climb corridor -> hip BACK ports (offset to ride beside the
#     data ribbon).
#   * hip -> knee (S+GND+V+): both port clusters face INBOARD on the same
#     femur-frame plane (z ≈ −3), so the lead festoons straight across the
#     femur's inboard face, sagging just past the passive-horn yoke arm
#     and coxa plate (both end at z −11) — no wrap over either clamp cap
#     (Aug 2026: the old spar-top run looped the whole way around the
#     knee servo, user-flagged as unrealistic).
# Leg-to-leg jumpers stay in ``_build_body_routes`` only.

_ROUTE_COLOR_DATA = "#eab308"   # amber: serial-bus leads (V+/GND/S or S+GND)

# Stock FEETECH 3-pin bus lead: ~2.8 mm bundle envelope.  The bend gate is set
# WELL below the physical ~3 x OD comfort bend because the drawn polylines
# corner schematically at connector entries (the molded boot's strain relief,
# the wire-exit slot lip) where the real lead is captive anyway -- the 2.5 mm
# gate only guards against accidental hairpins in the drawn path, not the free
# cable's service loops (which the +30 mm/joint slack budget of
# wire_harness_plan covers).
_BUS_LEAD_MIN_BEND_MM = 2.5

# Aug 2026 realistic conductors: each 5264 bus lead is drawn as its THREE
# real side-by-side 28 AWG conductors (flat ribbon, 5264 pin order
# GND / V+ / SIGNAL), not one fat bundle tube.  ~1.3 mm insulated OD each,
# 1.35 mm centre spacing; the ribbon plane comes from a parallel-transported
# path normal (the real ribbon twists freely -- this is visual, not a twist
# model).  Same waypoints, budget, and bend gate per conductor, so the
# routing_reach maths is unchanged -- there are just 3 thin wires per lead.
_LEAD_COND_OD_MM = 1.3
_LEAD_COND_SPACING_MM = 1.35
# (suffix, color, label tag, ribbon offset mm)
_LEAD_CONDUCTORS = (
    ("gnd", "#1f2937", "GND", -_LEAD_COND_SPACING_MM),
    ("vp",  "#ef4444", "V+",  0.0),
    ("sig", _ROUTE_COLOR_DATA, "S", +_LEAD_COND_SPACING_MM),
)
# Chassis -> yaw data entry carries S+GND ONLY (WIRING.md §6.2: V+ arrives
# via the hip tee, never through the chassis 5264 into the yaw).
_LEAD_DATA_ENTRY = (
    ("gnd", "#1f2937", "GND", -0.7),
    ("sig", _ROUTE_COLOR_DATA, "S", +0.7),
)
# Per-leg power branch continuing from the corner Wago up to the HIP tee
# (16-18 AWG silicone twisted pair).  Tight ±0.7 offsets keep its envelope
# no wider than the proven data ribbon through the yaw climb corridor
# (larger side offsets grazed the coxa skirt/plate at bends).
_LEAD_POWER_TEE = (
    ("vp", "#ef4444", "V+", -0.7),
    ("gnd", "#1f2937", "GND", +0.7),
)
_POWER_TEE_OD_MM = 2.4
# Which conductor set each intra-leg chain uses (chain name -> conductors).
_CHAIN_CONDUCTORS = {
    "tail": _LEAD_DATA_ENTRY,
    "yaw_hip": _LEAD_CONDUCTORS,
    "hip_knee": _LEAD_CONDUCTORS,
    "pwr_tee": _LEAD_POWER_TEE,
}


def _offset_polyline(points, offset_mm: float) -> list[np.ndarray]:
    """Offset a world polyline sideways by ``offset_mm`` along a
    parallel-transported normal (stable frame, no flips), for drawing the
    individual conductors of a flat ribbon side by side."""
    P = [np.asarray(p, dtype=float) for p in points]
    if abs(offset_mm) < 1e-9 or len(P) < 2:
        return P
    tangents = []
    for i in range(len(P)):
        t = P[min(i + 1, len(P) - 1)] - P[max(i - 1, 0)]
        n = np.linalg.norm(t)
        tangents.append(t / n if n > 1e-9 else np.array([1.0, 0.0, 0.0]))
    ref = (np.array([0.0, 0.0, 1.0]) if abs(tangents[0][2]) < 0.9
           else np.array([1.0, 0.0, 0.0]))
    nrm = np.cross(tangents[0], ref)
    nrm /= np.linalg.norm(nrm)
    out = []
    for p, t in zip(P, tangents):
        nrm = nrm - np.dot(nrm, t) * t
        n = np.linalg.norm(nrm)
        if n > 1e-9:
            nrm = nrm / n
        out.append(p + offset_mm * nrm)
    return out

# (Aug 2026: the per-leg _ROUTE_LEG_DOGLEGS battery-flank detours are gone --
# each power branch now lands on its leg's CORNER Wago pair right beside the
# drop slot, so no branch crosses the LiPo anymore.)


def _plan_entry(leg: int, axis: str) -> WHP.HarnessEntry:
    for entry in WHP.WIRE_HARNESS_PLAN:
        if entry["leg_idx"] == leg and entry["axis"] == axis:
            return entry
    raise KeyError((leg, axis))


def _servo_attach_paths() -> dict[str, dict[str, list[tuple[str, list[float]]]]]:
    """Leg-0 waypoint chains for the intra-leg servo wiring, one chain per
    CONDUCTOR: ``{route: {conductor_suffix: [(anchor, local_xyz), ...]}}``.

    ``anchor`` is one of ``yaw`` / ``hip`` / ``knee`` (servo well frame) or
    ``femur`` (femur_link part frame).  Spar midpoints MUST be femur-anchored:
    the hip servo body stays with the coxa, so hip-anchored "spar" points
    would float and get pinched when hip pitches.

    Attach = real BACK-face 5264 cluster (``HP.STS3215_PORT_*``).  Every
    chain wraps AROUND the servo cases (probed clear of every leg-0 solid by
    tools/_wire_probe_diag.py -- the pre-Aug-2026 paths cut straight through
    the hip + knee bodies and the v4 retainer foot poles).
    """
    M0 = _servo_M0s()
    Mf = next(M for n, _m, M in _leg0_local_link_parts() if n == "femur_link")
    inv = {ax: np.linalg.inv(M0[ax]) for ax in ("yaw", "hip", "knee")}
    inv["femur"] = np.linalg.inv(Mf)

    port_l = np.array([HP.STS3215_PORT_X_MM, HP.STS3215_PORT_Y_MM,
                       HP.STS3215_PORT_Z_MM, 1.0])
    ports = {ax: (M0[ax] @ port_l)[:3] for ax in ("yaw", "hip", "knee")}

    def axis(ax: str, basis: str) -> np.ndarray:
        i = {"x": 0, "y": 1, "z": 2}[basis]
        e = np.zeros(4); e[i] = 1.0
        return (M0[ax] @ e)[:3]

    leave_hip = ports["hip"] - 4.0 * axis("hip", "z")
    leave_knee = ports["knee"] - 4.0 * axis("knee", "z")
    leave_yaw = ports["yaw"] - 6.0 * axis("yaw", "z")

    src = np.array(_plan_entry(0, "yaw")["source_xyz_chassis"])
    drop = np.array(_plan_entry(0, "yaw")["via_chassis_drop_xyz"])

    def Wy(xyz) -> np.ndarray:
        return (M0["yaw"] @ np.array([xyz[0], xyz[1], xyz[2], 1.0]))[:3]

    def Wh(xyz) -> np.ndarray:
        return (M0["hip"] @ np.array([xyz[0], xyz[1], xyz[2], 1.0]))[:3]

    def Wk(xyz) -> np.ndarray:
        return (M0["knee"] @ np.array([xyz[0], xyz[1], xyz[2], 1.0]))[:3]

    def Wf(xyz) -> np.ndarray:
        return (Mf @ np.array([xyz[0], xyz[1], xyz[2], 1.0]))[:3]

    # Yaw hangs output-UP: body occupies roughly local z∈[-4, 34].  Ports sit
    # on the BACK at z=−3; the retainer drop window opens straight down.
    # Waypoints at local z≤−10 stay below the body (never world z=−9).
    drop_l = (inv["yaw"] @ np.array([drop[0], drop[1], drop[2], 1.0]))[:3]
    drop_down = Wy([drop_l[0], drop_l[1], -10.0])
    under_yaw = Wy([HP.STS3215_PORT_X_MM, HP.STS3215_PORT_Y_MM, -10.0])
    # yaw→hip (Aug 2026 rework): the OLD −X swing at yaw (−28, −22, −14) now
    # lands inside the v4 retainer's rear corner FOOT POLE, so the lead
    # instead slides under the case and out the saddle channel's OPEN +X end
    # (walls are on ±Y/−X only), then rises past the Ø50 bearing cap at
    # r ≈ 27 / azimuth ≈ 63° -- outside the cap rim AND between its 90°/330°
    # ear lugs -- up to the hip back ports.
    yaw_slide_open_end = Wy([5.0, 8.0, -10.0])
    yaw_exit_open_end = Wy([26.0, 12.0, -10.0])
    yaw_rise_low = Wy([24.5, 24.0, 2.0])
    yaw_rise_by_cap = Wy([25.0, 25.0, 34.0])
    # ...stay OUTSIDE the hub-platform / skirt rim (r ≈ 28) until above the
    # platform top, then duck in to the hip back ports.
    yaw_over_platform = Wy([12.5, 29.5, 64.0])

    # Power-tee approach: at OD 2.4 the direct over_platform -> leave_hip
    # crossing grazes the coxa passive-side plate corner (probe: ~0.2-0.4 mm
    # at hip (7, -30, -10)); dip INBOARD of the plate (nothing solid past
    # hip z -11) before rising to the ports.
    tee_under_plate = Wh([4.0, -33.0, -14.0])
    # ...stay below the plate (z <= -13) until past its y band (-27.7..-7.8),
    # then rise to the wire slot at x -10 (clear of the yoke arm's x 2..46).
    tee_inboard = Wh([-4.0, -8.0, -13.0])

    # hip→knee: BOTH back-port clusters sit on the femur-frame INBOARD
    # plane (z ≈ −3, y 0 -- hip ports at femur (−10,0,−3), knee ports at
    # (80,0,−3)), so the lead simply festoons across the femur's inboard
    # face.  Nothing is solid inboard of femur z −11 (the passive-horn yoke
    # arm ends at z −11, the coxa's passive-side plate at z −11 too), so
    # the run sags to z −13.5 and passes under BOTH -- probed clear.
    femur_dip_inboard = Wf((-4.0, 0.0, -13.5))
    femur_run_inboard = Wf((68.0, 0.0, -13.5))

    chains = {
        "tail": [
            ("yaw", src), ("yaw", drop), ("yaw", drop_down),
            ("yaw", under_yaw), ("yaw", leave_yaw), ("yaw", ports["yaw"]),
        ],
        "yaw_hip": [
            ("yaw", ports["yaw"]), ("yaw", leave_yaw), ("yaw", under_yaw),
            ("yaw", yaw_slide_open_end), ("yaw", yaw_exit_open_end),
            ("yaw", yaw_rise_low), ("yaw", yaw_rise_by_cap),
            ("yaw", yaw_over_platform),
            ("hip", leave_hip), ("hip", ports["hip"]),
        ],
        "hip_knee": [
            ("hip", ports["hip"]),
            ("hip", leave_hip),
            ("femur", femur_dip_inboard),
            ("femur", femur_run_inboard),
            ("knee", leave_knee),
            ("knee", ports["knee"]),
        ],
        # Power branch continuing from the drop slot (where the body-route
        # corner-Wago branch ends), up the SAME yaw climb corridor, to the
        # hip tee ports.
        "pwr_tee": [
            ("yaw", drop), ("yaw", drop_down),
            ("yaw", yaw_slide_open_end), ("yaw", yaw_exit_open_end),
            ("yaw", yaw_rise_low), ("yaw", yaw_rise_by_cap),
            ("yaw", yaw_over_platform),
            ("hip", tee_under_plate), ("hip", tee_inboard),
            ("hip", leave_hip), ("hip", ports["hip"]),
        ],
    }

    # Fan each chain out into its ribbon conductors (3 for full bus leads,
    # 2 for the S+GND data entry and the V+/GND power tee): offset the WORLD
    # polyline sideways, then express every offset waypoint back in its own
    # anchor's local frame so the conductors still ride the gait.
    paths: dict[str, dict[str, list[tuple[str, list[float]]]]] = {}
    for name, chain in chains.items():
        world = [w for _a, w in chain]
        paths[name] = {}
        for suffix, _color, _tag, off in _CHAIN_CONDUCTORS[name]:
            pts = _offset_polyline(world, off)
            paths[name][suffix] = [
                (a, [float(c) for c in (inv[a] @ np.array([*p, 1.0]))[:3]])
                for (a, _w), p in zip(chain, pts)]
    return paths


def _build_routes(chassis_lift: float, legs: list[int],
                  leg_part_ids: dict[tuple[int | None, str], str],
                  wago_data_by_leg: dict[int, str] | None = None) -> list[dict]:
    """BuildViz ``routes[]`` for the per-leg servo wiring: data-entry tail
    (S+GND), yaw->hip / hip->knee daisy segments (GND/V+/S ribbon), and the
    corner-Wago->hip power tee (V+/GND) -- see the section comment above.
    Each drawn as its individual conductors (``_CHAIN_CONDUCTORS``).
    ``leg_part_ids`` maps (leg, partType) -> scene instance id.

    Servo-end and femur-spar waypoints are instance-anchored so they ride
    the gait; chassis drop/bus nodes stay in chassis frame."""
    paths = _servo_attach_paths()
    routes: list[dict] = []

    _ANCHOR_PART = {
        "yaw": "yaw_servo", "hip": "hip_servo", "knee": "knee_servo",
        "femur": "femur_link",
    }

    def wp_anchored(leg: int, chain: list[tuple[str, list[float]]]) -> list[dict]:
        out = []
        for anchor, loc in chain:
            part = _ANCHOR_PART[anchor]
            out.append({"instanceId": leg_part_ids[(leg, part)], "local": loc})
        return out

    def ids(leg, *names):
        return [leg_part_ids[k] for k in
                (((None, n) if n == "chassis_bottom" else (leg, n))
                 for n in names) if k in leg_part_ids]

    common = dict(kind="data",
                  diameterMm=_LEAD_COND_OD_MM, radiusMm=_LEAD_COND_OD_MM / 2.0,
                  minBendRadiusMm=_BUS_LEAD_MIN_BEND_MM,
                  maxLengthMm=WHP.STOCK_PIGTAIL_MM)

    def add(leg: int, route_id: str, chain_name: str, label: str,
            instances: list, od_mm: float | None = None) -> None:
        chains = paths[chain_name]
        od = od_mm if od_mm is not None else _LEAD_COND_OD_MM
        for suffix, color, tag, _off in _CHAIN_CONDUCTORS[chain_name]:
            routes.append({
                "id": f"{route_id}-{suffix}",
                **common,
                "diameterMm": od, "radiusMm": od / 2.0,
                "color": color,
                "waypoints": wp_anchored(leg, chains[suffix]),
                "label": f"{label} [{tag}]",
                "instances": instances,
            })

    for leg in legs:
        j_yaw, j_hip, j_knee = (joint_index(leg, ax) for ax in ("yaw", "hip", "knee"))
        sid_yaw = joint_to_servo_id(j_yaw)
        sid_hip = joint_to_servo_id(j_hip)
        sid_knee = joint_to_servo_id(j_knee)

        # The tail lead's SOURCE is the leg's underside data-Wago splice, so
        # that Wago is a terminal (declared pass-through), not an obstruction.
        tail_wagos = ([wago_data_by_leg[leg]] if wago_data_by_leg
                      and leg in wago_data_by_leg else [])
        # Terminal servos are declared per route: the lead SEATS into the
        # recessed 5264 cluster, so endpoint contact with the case is
        # intended (same exemption as connector seating).
        add(leg, f"route-j{j_yaw:02d}", "tail",
            (f"L{leg} DATA entry: data Wago -> drop slot -> down retainer "
             f"window -> YAW ID {sid_yaw} BACK-face 5264 ports "
             f"(S+GND only; V+ arrives via the hip tee)"),
            ids(leg, "chassis_bottom", "yaw_servo_retainer", "yaw_servo")
            + tail_wagos)
        add(leg, f"route-j{j_hip:02d}", "yaw_hip",
            (f"L{leg} daisy: YAW ID {sid_yaw} -> HIP ID {sid_hip} "
             f"(BACK ports, out the saddle's open +X end, up past the "
             f"bearing cap)"),
            ids(leg, "chassis_bottom", "yaw_servo_retainer", "yaw_servo",
                "hip_servo"))
        add(leg, f"route-j{j_knee:02d}", "hip_knee",
            (f"L{leg} daisy: HIP ID {sid_hip} -> KNEE ID {sid_knee} "
             f"(BACK ports, festooned across the femur's inboard face, "
             f"under the passive-horn yoke arm)"),
            ids(leg, "femur_link", "hip_servo", "knee_servo"))
        add(leg, f"route-pwr-tee-l{leg}", "pwr_tee",
            (f"L{leg} POWER tee: corner power-Wago branch -> drop slot -> "
             f"up the yaw corridor -> HIP ID {sid_hip} ports (V+/GND; the "
             f"hip tees 12 V to yaw + knee -- WIRING.md §6.2)"),
            ids(leg, "chassis_bottom", "yaw_servo_retainer", "hip_servo"),
            od_mm=_POWER_TEE_OD_MM)
    return routes


# ---------------------------------------------------------------------------
# Body-harness routes (the rest of the WIRING.md §1/§6 nets)
# ---------------------------------------------------------------------------
#
# The 18 per-joint bus leads above cover only the servo DATA chains.  The
# routes below document the REST of the harness -- power trunk, per-leg power
# branches (via the trunk Wago splice pair + peripheral Wagos; as-built
# Aug 2026: NO PDB), leg-to-leg data jumpers (via underside data Wagos),
# battery→Uno Q (no buck), the data head to servo ID 1, and the IMU I2C
# pigtail -- with waypoints derived from the MODELED as-built parts (LiPo,
# switch_holster, trunk Wagos, Uno Q on hex plate, MPU under raised
# platform).  Inline fuse / adapter neighbourhoods are labelled even when
# no CAD part exists.
#
# These are DOCUMENTATION routes (which wire goes where, at what gauge), not
# reach-critical cable builds, so budgets are generous -- but every route must
# still clear the routing_reach obstruction ray test.

_ROUTE_COLOR_POWER = "#ef4444"  # bright red: 12 V V+ conductors
_ROUTE_COLOR_GND = "#1f2937"    # near-black: GND conductors / no-V+ jumpers
_ROUTE_COLOR_LOGIC = "#3b82f6"  # blue: logic (SCL / USB head)
_ROUTE_COLOR_I2C = "#22c55e"    # green: I2C SDA

# Aug 2026 realistic conductors for the body nets (same idea as
# _LEAD_CONDUCTORS): a V+/GND pair is TWO wires, the I2C pigtail is FOUR.
# (suffix, color, tag, ribbon offset mm) per net; OD passed separately.
_PAIR_POWER = (("vp", _ROUTE_COLOR_POWER, "V+", -1.3),
               ("gnd", _ROUTE_COLOR_GND, "GND", +1.3))
_PAIR_LOGIC = (("vp", _ROUTE_COLOR_POWER, "V+", -1.0),
               ("gnd", _ROUTE_COLOR_GND, "GND", +1.0))
_PAIR_JUMPER = (("sig", _ROUTE_COLOR_DATA, "S", -0.7),
                ("gnd", _ROUTE_COLOR_GND, "GND", +0.7))
# Aug 2026: the MPU's 3V3 now comes off the under-plate 3.3 V Wago (see
# route-3v3-*), so the Uno -> MPU pigtail is a TRIPLE (GND/SCL/SDA).
_TRI_I2C = (("gnd", _ROUTE_COLOR_GND, "GND", -1.1),
            ("scl", _ROUTE_COLOR_LOGIC, "SCL", 0.0),
            ("sda", _ROUTE_COLOR_I2C, "SDA", +1.1))
# Aug 2026 as-built data head: the Uno Q talks to the bus-servo adapter
# over its D0/D1 MCU UART (feetech_bridge, jumper A) -- a TX/RX/GND
# jumper triple, NOT the USB-C cable.  VIN-powered Uno Qs neither source
# 5 V from the USB-C jack nor boot the USB controller in host mode
# (arduino/linux-qcom#2), so USB is bench-only (WIRING.md intro note).
_TRI_UART = (("tx", _ROUTE_COLOR_LOGIC, "TX", -1.1),
             ("rx", _ROUTE_COLOR_I2C, "RX", 0.0),
             ("gnd", _ROUTE_COLOR_GND, "GND", +1.1))
# ST7789 SPI screen pigtail: 8 conductors bundled Uno Q -> top-plate slot.
# Tight 0.5 mm pitch (wires overlap = round bundle look, ~4.5 mm wide
# incl. wire OD) so the bundle fits the 5 mm-deep slot opening whatever
# way the parallel-transport ribbon plane twists at the crossing.
_SCREEN_8 = (("vcc", _ROUTE_COLOR_POWER, "VCC", -1.75),
             ("gnd", _ROUTE_COLOR_GND, "GND", -1.25),
             ("scl", _ROUTE_COLOR_LOGIC, "SCL", -0.75),
             ("sda", _ROUTE_COLOR_I2C, "SDA", -0.25),
             ("res", "#f59e0b", "RES", +0.25),
             ("dc", "#8b5cf6", "DC", +0.75),
             ("cs", "#06b6d4", "CS", +1.25),
             ("blk", "#ec4899", "BLK", +1.75))


def _fan_points_route(route: dict, conductors, od_mm: float,
                      overrides: dict | None = None) -> list[dict]:
    """Split a world-polyline body route into its parallel conductors
    (offset sideways via ``_offset_polyline``); same waypoint shape, budget,
    and instances per conductor -- just N thin wires instead of one tube.

    ``overrides`` maps conductor suffix -> {point_index: world_xyz} and
    pins individual conductors to their REAL terminals (the V+ vs GND
    Wago nut, a specific Uno header pin) instead of the shared fanned
    polyline.  Negative indices address from the end (-1 = endpoint)."""
    out = []
    for suffix, color, tag, off in conductors:
        r = dict(route)
        r["id"] = f"{route['id']}-{suffix}"
        r["label"] = f"{route['label']} [{tag}]"
        r["color"] = color
        r["radiusMm"] = od_mm / 2.0
        r["diameterMm"] = od_mm
        points = [[float(c) for c in p]
                  for p in _offset_polyline(route["points"], off)]
        for idx, pt in ((overrides or {}).get(suffix) or {}).items():
            points[idx] = [float(c) for c in pt]
        r["points"] = points
        out.append(r)
    return out

# Chassis-frame anchor nodes for the body harness (pre-lift z).
# _LIPO_EXIT: the under-belly pack block's +axis end (az 30, r ~ 37.5)
# where both packs' XT60 leads + the Y-harness live, below the belly.
_LIPO_EXIT = (36.0, 21.0, -19.0)
_SWITCH_SIDE = (52.0, -13.0, HP.CHASSIS_TOP_TOP_Z + 10.0)   # holster -Y wall
_SWITCH_SIDE_OUT = (48.0, -13.0, HP.CHASSIS_TOP_TOP_Z + 10.0)
_EDGE_DROP_XY = (70.0, -16.0)
# Central trunk Wago splice pair (two 5-port 221-415, V+/GND) at the
# chassis_top centre -- the battery trunk lands here (as-built Aug 2026:
# no PDB).
_WAGO_TRUNK_NODE = (HP.WAGO_TRUNK_CENTRE[0], HP.WAGO_TRUNK_CENTRE[1],
                    HP.CHASSIS_TOP_TOP_Z + HP.WAGO5_H)


def _leg_src(leg: int) -> tuple[float, float, float]:
    """Cradle wire-exit (source) of ``leg``'s servos, from the harness plan."""
    for entry in WHP.WIRE_HARNESS_PLAN:
        if entry["leg_idx"] == leg and entry["axis"] == "yaw":
            return entry["source_xyz_chassis"]
    raise KeyError(leg)


def _leg_drop(leg: int) -> tuple[float, float, float]:
    """Chassis_bottom drop slot of ``leg`` (the power-injection point)."""
    for entry in WHP.WIRE_HARNESS_PLAN:
        if entry["leg_idx"] == leg and entry["axis"] == "yaw":
            return entry["via_chassis_drop_xyz"]
    raise KeyError(leg)


def _leg_bus_post(leg: int) -> tuple[float, float, float]:
    """Bus-bar landing post of ``leg`` (destination in the harness plan)."""
    for entry in WHP.WIRE_HARNESS_PLAN:
        if entry["leg_idx"] == leg and entry["axis"] == "yaw":
            return entry["destination_xyz_chassis"]
    raise KeyError(leg)


def _build_body_routes(chassis_lift: float, legs: list[int],
                       part_ids: dict[str, str],
                       wago_data_by_leg: dict[int, str] | None = None,
                       part_ids_all: dict[str, list[str]] | None = None,
                       ) -> list[dict]:
    """BuildViz ``routes[]`` for the non-servo-lead nets of WIRING.md §1/§6.

    ``part_ids`` maps body partType -> scene instance id so each route can
    declare its termination instances (exempt from the obstruction ray test,
    exactly like connector seating).  ``wago_data_by_leg`` maps a leg to its
    underside data-Wago instance (terminal of the bus head + jumpers).
    ``part_ids_all`` maps partType -> ALL instance ids (for multi-instance
    terminals like the trunk nut pair, corner Wago pairs, and data Wagos --
    ``part_ids`` keeps only one id per type)."""
    ex, ey = _EDGE_DROP_XY
    deck0 = HP.CHASSIS_TOP_TOP_Z
    hex_top = deck0 + HP.HEX_POST_STACK_H + HP.HEX_MOUNT_PLATE_T
    uno_z = hex_top + 5.0
    raised_top = hex_top + HP.HEX_RAISED_TOTAL_H

    def pts(*chassis_pts):
        return [[float(x), float(y), float(z) + chassis_lift]
                for x, y, z in chassis_pts]

    def w(p):
        """One chassis-frame point -> world (for _fan_points_route
        overrides)."""
        return pts(p)[0]

    # --- Arduino Uno R3 header map (the Uno Q keeps the Uno form factor).
    # dx along the board's long axis from the board centre (USB-C end =
    # -X), 2.54 mm pitch, per the official Uno mechanical drawing.  The
    # DIGITAL row rides the +Y long edge, power/analog the -Y edge; the
    # visual's header strips sit at y = centre +/- 23.67, top z = pcb
    # 1.6 + 9 mm pins.
    _UNO_DIG = {"SCL": -16.51, "SDA": -13.97, "GND_D": -8.89,
                "D13": -6.35, "D12": -3.81, "D11": -1.27, "D10": 1.27,
                "D9": 3.81, "D8": 6.35, "D7": 10.16,
                "D1": 25.4, "D0": 27.94}
    _UNO_PWR = {"3V3": 0.0, "5V": 2.54, "GND_P1": 5.08, "GND_P2": 7.62,
                "VIN": 10.16}

    def uno_pin(name: str) -> tuple[float, float, float]:
        ux, uy = HP.UNO_Q_ON_HEX_CENTRE
        if name in _UNO_DIG:
            return (ux + _UNO_DIG[name], uy + 23.67, hex_top + 10.6)
        return (ux + _UNO_PWR[name], uy - 23.67, hex_top + 10.6)

    # --- Central trunk nut pair: V+ = SOUTH nut (toward the switch at
    # -Y), GND = NORTH nut.  (Two 5-port 221-415 at (16, -/+16).)
    _nut_z = HP.CHASSIS_TOP_TOP_Z + HP.WAGO5_H
    trunk_vp_nut = (HP.WAGO_TRUNK_CENTRE[0],
                    HP.WAGO_TRUNK_CENTRE[1] - HP.WAGO_TRUNK_DY / 2.0, _nut_z)
    trunk_gnd_nut = (HP.WAGO_TRUNK_CENTRE[0],
                     HP.WAGO_TRUNK_CENTRE[1] + HP.WAGO_TRUNK_DY / 2.0, _nut_z)

    def ids(*names):
        return [part_ids[n] for n in names if n in part_ids]

    def ids_all(*names):
        if not part_ids_all:
            return ids(*names)
        return [i for n in names for i in part_ids_all.get(n, [])]

    routes: list[dict] = [
        # -- power trunk V+: LiPo -> switch -> fuse -> trunk Wago ------------
        # The anti-spark switch + inline fuse interrupt V+ ONLY; battery GND
        # runs straight to the trunk GND nut (separate route below).
        # Aug 2026 under-belly packs: both XT60s + the parallel Y-harness
        # live BELOW the belly at the pack block's +axis end.  Both packs'
        # terminated XT60 leads climb through the DEDICATED 14 x 22 mm
        # battery-trunk pass-through at (48, 0) (HP.BATTERY_TRUNK_HOLE_*,
        # sized to pass an XT60 nose-first), surface in the inter-plate
        # bay, and hop the deck rim at az 0 to reach the switch / main
        # XT60 on chassis_top.
        {
            "id": "route-trunk-vp-lipo-switch",
            "points": pts(_LIPO_EXIT,
                          (HP.BATTERY_TRUNK_HOLE_CENTRE[0], 4.0, -16.0),
                          (HP.BATTERY_TRUNK_HOLE_CENTRE[0], 0.0, 6.0),
                          (63.0, -6.0, 10.0),
                          (63.0, -10.0, deck0 + 8.0),
                          _SWITCH_SIDE),
            "maxLengthMm": 220.0,
            "label": ("power trunk V+ 12-14 AWG: under-belly LiPo pair "
                      "(XT60 Y-harness) -> UP through the 14 x 22 battery "
                      "pass-through at (48, 0) -> over the deck rim at "
                      "az 0 -> anti-spark switch"),
            "color": _ROUTE_COLOR_POWER,
            "radiusMm": 1.8,
            "instances": ids("lipo_battery", "switch_holster")
            + ids_all("wago_power"),
        },
        {
            "id": "route-trunk-vp-switch-wago",
            "points": pts(_SWITCH_SIDE_OUT, (34.0, -13.0, deck0 + 14.0),
                          (20.0, -14.0, deck0 + 14.0), trunk_vp_nut),
            "maxLengthMm": 200.0,
            "label": ("power trunk V+ 12-14 AWG: switch -> 15-20 A main fuse "
                      "(inline) -> central trunk V+ nut = the SOUTH 5-port "
                      "Wago 221-415 at (16, -16), nearest the switch"),
            "color": _ROUTE_COLOR_POWER,
            "radiusMm": 1.8,
            "instances": ids("switch_holster") + ids_all("wago_trunk"),
        },
        {
            "id": "route-trunk-gnd-lipo-wago",
            "points": pts(_LIPO_EXIT,
                          (HP.BATTERY_TRUNK_HOLE_CENTRE[0], 4.0, -16.0),
                          (HP.BATTERY_TRUNK_HOLE_CENTRE[0], -4.0, 6.0),
                          (63.0, -8.0, 8.0),
                          (84.0, -6.0, 9.0),
                          (87.0, -5.0, 12.0),
                          (87.0, -14.0, deck0 + 17.0),
                          (60.0, -16.0, deck0 + 17.0),
                          (34.0, -17.0, deck0 + 17.0),
                          (10.0, -14.0, deck0 + 17.0),
                          (8.0, 2.0, deck0 + 17.0),
                          trunk_gnd_nut),
            "maxLengthMm": 360.0,
            "label": ("power trunk GND 12-14 AWG: under-belly LiPo pair -> "
                      "UP through the (48, 0) battery pass-through -> out "
                      "the az-0 bay to the corner tray, climbing PAST the "
                      "switch holster's +X face (x 85) -> back west along "
                      "the deck SOUTH of the holster (y <= -14) -> around "
                      "the WEST side of the V+ nut -> central trunk GND "
                      "nut = the NORTH 5-port Wago at (16, +16) (no "
                      "switch/fuse in the GND leg)"),
            "color": _ROUTE_COLOR_GND,
            "radiusMm": 1.8,
            "instances": ids("lipo_battery")
            + ids_all("wago_trunk", "wago_power"),
        },
    ]
    # -- battery → Uno Q (no buck): 20 AWG V+/GND pair ----------------------
    # The Uno sits ON the round Ø115 mount plate, so anything from the deck
    # below must come OVER the plate's rim (straight verticals pierce the
    # plate).  The screen stand's three legs occupy az ~82-98 / 202-218 /
    # 322-338 at r 43-50 (late-Aug 2026: was six legs); cross the rim
    # (r 57.5) in the wide free corridors between them, hopping at r ~ 59.
    routes += _fan_points_route({
        "id": "route-logic-battery-uno",
        "points": pts(_WAGO_TRUNK_NODE,
                      (29.5, 51.1, deck0 + 13.0),
                      (29.5, 51.1, hex_top + 4.0),
                      (26.5, 18.0, hex_top + 4.5),
                      # Climb OVER the board's east end (header strips top
                      # out at hex_top+10.6), then land on the -Y power
                      # header from above.
                      (24.0, -24.0, hex_top + 12.5),
                      (8.0, -35.0, hex_top + 11.0)),
        "maxLengthMm": 300.0,
        "label": ("battery tap at the trunk nuts (V+ = south, GND = north) "
                  "-> over the round plate's az-60 rim (between the "
                  "raised-platform legs, r ~ 59) -> EAST of the breakout "
                  "(x > 23) -> over the board's east end -> Uno Q power "
                  "header: V+ on VIN, GND on the adjacent GND pin (no "
                  "buck; 20 AWG; Uno Q accepts the 3S rail via its own "
                  "regulator)"),
        "instances": ids("uno_q") + ids_all("wago_trunk"),
    }, _PAIR_LOGIC, od_mm=1.8, overrides={
        "vp": {0: w(trunk_vp_nut), -1: w(uno_pin("VIN"))},
        "gnd": {0: w(trunk_gnd_nut), -1: w(uno_pin("GND_P1"))},
    })
    # -- data head 1/2: Uno Q D0/D1 -> adapter UART header (TX/RX/GND) ------
    # As-built: feetech_bridge owns the MCU UART (adapter jumper A); the
    # USB-C hookup is bench-only (no VBUS / device-mode USB when the Uno Q
    # is VIN-powered).  The adapter is on the deck UNDER the round mount
    # plate: drape over the plate's -X rim (r 57.5; hop at r ~ 60.5), run
    # SOUTH around the adapter body (y < -21), and land on its +X
    # servo/UART face from the east.
    adapter_uart_face = (HP.MOTOR_CTRL_CENTRE[0] + HP.MOTOR_CTRL_D / 2.0,
                         HP.MOTOR_CTRL_CENTRE[1])
    routes += _fan_points_route({
        "id": "route-uart-uno-adapter",
        # Starts on the D0/D1 end of the +Y digital header; hop WEST above
        # the header strips (top hex_top+10.6) and descend past the -X
        # board edge (x -34.3) before diving for the plate rim.
        "points": pts((26.0, -12.0 + 23.67, hex_top + 10.6),
                      (-10.0, 8.0, hex_top + 12.5),
                      (-40.0, -4.0, hex_top + 6.0),
                      (-60.0, -8.0, hex_top + 3.0),
                      (-60.0, -8.0, deck0 + 14.0),
                      # y ~ -24.5 threads between the (-31.1, -31.1) magnet
                      # post (Ø5) and the adapter's south face (y -21); the
                      # mid waypoint pins the rendered spline at the post so
                      # it cannot sag south into it.
                      (-48.0, -25.0, deck0 + 10.0),
                      (-31.0, -24.5, deck0 + 9.0),
                      (-14.0, -25.0, deck0 + 8.0),
                      (-4.0, -20.0, deck0 + 8.0),
                      (-4.0, -2.0, deck0 + 8.0),
                      (adapter_uart_face[0] + 1.0, adapter_uart_face[1],
                       deck0 + 8.0)),
        "maxLengthMm": 300.0,
        "label": ("Uno Q MCU UART from the digital-header pins D1(TX)/"
                  "D0(RX)/GND (feetech_bridge, 1 Mbps bus / 921600 to "
                  "Linux) -> over the round plate's -X rim -> south around "
                  "the adapter -> UART/MCU header on the adapter's +X "
                  "servo face (jumper A; USB-C is bench-only -- VIN power "
                  "gives no USB host mode / no VBUS)"),
        "instances": ids("uno_q", "motor_controller"),
    }, _TRI_UART, od_mm=1.0, overrides={
        "tx": {0: w(uno_pin("D1"))},
        "rx": {0: w(uno_pin("D0"))},
        "gnd": {0: w(uno_pin("GND_D"))},
    })
    # -- data head 2/2: adapter -> L0 data Wago (S+GND -- on the full robot
    # the adapter is DATA-ONLY; V+ never rides this pigtail, WIRING.md §1).
    # Hops OVER the switch holster (top z deck0+21.5; hex plate bottom
    # leaves a ~4 mm lane), descends into the inter-plate bay in the az-0
    # corridor (outside the top plate r 57.5, clear of the az-30 yaw
    # towers), then arcs INBOARD of the tower base (r < 52; the towers'
    # inboard face is at r ~ 54) to the L0 drop slot.
    routes += _fan_points_route({
        "id": "route-data-head",
        "points": pts((HP.MOTOR_CTRL_CENTRE[0], HP.MOTOR_CTRL_CENTRE[1],
                       deck0 + HP.MOTOR_CTRL_H),
                      (16.0, -10.0, deck0 + 19.0),
                      (32.0, -10.0, deck0 + 28.0),
                      (66.0, -10.0, deck0 + 28.0),
                      (84.0, -22.0, deck0 + 28.0),
                      (84.0, -22.0, 14.0),
                      (84.0, -27.0, 10.0),
                      (75.0, -25.0, 7.0),
                      (48.0, -4.0, 6.0),
                      (44.0, 12.0, 5.0),
                      (44.0, 25.0, 4.0),
                      _leg_drop(0), _leg_src(0)),
        "maxLengthMm": 450.0,
        "label": ("bus head: adapter servo plug -> over the switch holster "
                  "(top deck0+21.5, velcro mount; the hex-plate underside "
                  "leaves a 9 mm lane) -> down SOUTH of the az-0 corner Wago mount "
                  "(mount spans y +/-22.3, x 79-102) -> inboard of the "
                  "L0 yaw tower -> L0 drop slot -> L0 data Wago (S+GND "
                  "only; adapter is data-only on the robot)"),
        "instances": ids("motor_controller", "chassis_bottom") + (
            [wago_data_by_leg[0]] if wago_data_by_leg
            and 0 in wago_data_by_leg else []),
    }, _PAIR_JUMPER, od_mm=1.3)
    # -- IMU I2C: 3-wire 28 AWG pigtail (GND/SCL/SDA; 3V3 via the wago) -----
    routes += _fan_points_route({
        "id": "route-i2c-imu",
        # SCL/SDA/GND live at the WEST end of the +Y digital header;
        # cross the board diagonally ABOVE both header strips (top
        # hex_top+10.6) and drop past its SE corner.
        "points": pts((-14.0, -12.0 + 23.67, hex_top + 10.6),
                      (-6.0, 4.0, hex_top + 12.5),
                      (16.0, -28.0, hex_top + 12.0),
                      (24.0, -38.0, hex_top + 4.0),
                      (31.0, -50.0, hex_top + 1.2),
                      (31.0, -50.0, deck0 + 15.0),
                      (HP.MPU_ASBUILT_CENTRE[0], HP.MPU_ASBUILT_CENTRE[1],
                       deck0 + HP.IMU_PCB_T)),
        "maxLengthMm": 300.0,
        "label": ("Uno Q I2C from the digital-header pins SCL/SDA/GND -> "
                  "diagonally over the board -> over the round plate's SE "
                  "rim (r ~ 59, between the raised-platform legs) -> "
                  "GY-521 MPU glued on chassis_top beside the trunk Wagos "
                  "(28 AWG; MPU VCC comes off the 3.3 V Wago instead, "
                  "route-3v3-mpu)"),
        "instances": ids("uno_q", "mpu6050"),
    }, _TRI_I2C, od_mm=1.0, overrides={
        "gnd": {0: w(uno_pin("GND_D"))},
        "scl": {0: w(uno_pin("SCL"))},
        "sda": {0: w(uno_pin("SDA"))},
    })

    # -- 3.3 V rail: Uno Q 3V3 pin -> under-plate 5-port Wago -> loads ------
    # The 3.3 V splice nut is VHB'd flat to the UNDERSIDE of the round
    # mount plate near its south rim (HP.WAGO_V33_CENTRE = (0, -36)), wire
    # entries facing -Y.  The plate carries two dedicated Ø8 wire ports at
    # (+/-19, -44) (HP.MOUNT_PLATE_WIRE_PORT_XY): feed drops through the
    # EAST port.  (Aug 2026: the screen no longer taps this Wago -- its
    # whole 8-wire pigtail comes straight off the Uno Q, route-screen.)
    plate_under = hex_top - HP.HEX_MOUNT_PLATE_T
    v33_entry = (HP.WAGO_V33_CENTRE[0], HP.WAGO_V33_CENTRE[1] - 11.0,
                 plate_under - HP.WAGO5_H / 2.0)
    port_e, port_w = HP.MOUNT_PLATE_WIRE_PORT_XY
    routes += [
        {
            "id": "route-3v3-feed",
            "points": pts(uno_pin("3V3"),
                          (12.0, -34.0, hex_top + 2.5),
                          (port_e[0], port_e[1], hex_top + 1.5),
                          (port_e[0], port_e[1], plate_under - 2.0),
                          (10.0, -48.0, v33_entry[2]),
                          v33_entry),
            "maxLengthMm": 200.0,
            "label": ("3.3 V feed 28 AWG: Uno Q 3V3 pin -> down through the "
                      "round plate's EAST Ø8 wire port (19, -44) -> the "
                      "5-port 3.3 V Wago under the plate's south rim"),
            "color": _ROUTE_COLOR_POWER,
            "radiusMm": 0.5,
            "instances": ids("uno_q", "wago_v33", "hex_mount_plate"),
        },
        {
            "id": "route-3v3-mpu",
            "points": pts(v33_entry,
                          (2.0, -46.0, deck0 + 12.0),
                          (HP.MPU_ASBUILT_CENTRE[0],
                           HP.MPU_ASBUILT_CENTRE[1],
                           deck0 + HP.IMU_PCB_T)),
            "maxLengthMm": 150.0,
            "label": ("3.3 V branch 28 AWG: under-plate Wago -> straight "
                      "down to the GY-521 MPU VCC on chassis_top (the nut "
                      "hangs almost directly above the MPU)"),
            "color": _ROUTE_COLOR_POWER,
            "radiusMm": 0.5,
            "instances": ids("wago_v33", "mpu6050"),
        },
    ]

    # -- screen pigtail: Uno Q -> up through the top-plate wire slot --------
    # Aug 2026 (user): ALL screen wires come from the Uno Q and enter
    # through the 24x5 slot in the platform's top plate (outer edge at the
    # panel's +X edge, x 26.5..31.5, y +/-12).  The ST7789 SPI panel takes
    # an 8-wire pigtail (VCC GND SCL SDA RES DC CS BLK); the bundle is
    # zip-tied up the az-330 platform leg (foot at r 54, (47, -27)).
    # The az-330 leg blade occupies r 42..49 (LEG_T 7 radial, foot centre
    # r 45.5), so the climb rises just INBOARD of it at r ~ 36 -- the
    # ribbon fans +/-3.5 mm, leaving ~2.5 mm to the blade's inner face.
    # Only the endpoints (Uno, screen) are exempt: the polyline threads
    # the top plate's 24x5 slot opening, so the platform itself is
    # checked for real.
    # The 8 conductors start on their REAL Uno pins (overrides below):
    # power pair on the -Y power header (3V3 + GND), the six SPI/control
    # lines on the +Y digital header (SCK=D13, MOSI=D11, DC=D9, RES=D8,
    # CS=D10, BLK=D7).  Both groups converge over the board's east end
    # ABOVE the header strips (top hex_top+10.6) before the climb.
    routes += _fan_points_route({
        "id": "route-screen",
        "points": pts((10.0, -12.0, hex_top + 11.5),
                      (22.0, -16.0, hex_top + 12.5),
                      (31.0, -18.0, hex_top + 26.0),
                      (31.0, -18.0, raised_top - 16.0),
                      (30.0, -9.0, raised_top - 4.0),
                      (29.0, -3.0, raised_top - 3.0),
                      (29.0, 0.0, raised_top + 1.0),
                      (30.5, 0.0, raised_top + 2.0)),
        "maxLengthMm": 250.0,
        "label": ("screen pigtail 8x 28 AWG: Uno Q pins (3V3+GND on the "
                  "power header; D13/D11/D10/D9/D8/D7 = SCK/MOSI/CS/DC/"
                  "RES/BLK on the digital header) -> bundles over the "
                  "board's east end -> rises just inboard of the az-330 "
                  "platform leg (r 36; blade starts r 42) -> under the top "
                  "plate -> UP through the 24x5 wire slot behind the "
                  "panel's +X edge (panel held by 4x M2 self-tappers in "
                  "the top-plate pilot holes)"),
        "instances": ids("uno_q", "screen"),
    }, _SCREEN_8, od_mm=1.0, overrides={
        "vcc": {0: w(uno_pin("3V3"))},
        "gnd": {0: w(uno_pin("GND_P2"))},
        "scl": {0: w(uno_pin("D13"))},
        "sda": {0: w(uno_pin("D11"))},
        "res": {0: w(uno_pin("D8"))},
        "dc": {0: w(uno_pin("D9"))},
        "cs": {0: w(uno_pin("D10"))},
        "blk": {0: w(uno_pin("D7"))},
    })

    # -- 6x power branches: trunk Wagos -> corner Wago -> leg drop ----------
    # Aug 16 2026 (user): each corner now holds ONE press-fit 5-port Wago
    # 221-415 between the tray walls integrated into the chassis_bottom
    # top face (was a V+/GND pair of 3-port 221-413), at the chassis_bottom
    # hex CORNER FLAT at az = leg*60 deg (30 deg clockwise of the leg's yaw
    # axis), wire entries facing the chassis centre.  The branch hops off
    # the chassis_top edge, descends into the inter-plate bay outside the
    # top plate, lands on the corner nut, then runs to the leg drop slot.
    wago_r = (HP.WAGO_MOUNT_EDGE_R - HP.WAGO_MOUNT_WALL_T
              - (HP.WAGO5_D + HP.WAGO_MOUNT_BAY_CLEAR) / 2.0)
    # Nut sits directly on the plate top face (integrated tray, no floor).
    wago_top_z = HP.CHASSIS_PLATE_T / 2.0 + HP.WAGO5_H
    # V+/GND land on separate PORTS of the single 5-slot nut; the route
    # fan pins both conductors near the nut's tangential thirds (the old
    # two-nut tray used the bay centres instead).
    nut_dt = HP.WAGO5_W / 4.0
    for leg in legs:
        a = leg * np.pi / 3.0
        ca, sa = np.cos(a), np.sin(a)
        # ...and the drop slot sits at az = corner + 30 deg.
        a2 = a + np.pi / 6.0
        ca2, sa2 = np.cos(a2), np.sin(a2)
        # Leg 3's corner (az 180) is straight across the motor controller
        # from the trunk node -- hop OVER the board (top z = deck0 + 12)
        # instead of through it.  Leg 0's corner (az 0) is straight across
        # the SWITCH HOLSTER (x 48..85, y +/-11, top deck0+21.5 -- Aug 16
        # 2026 velcro mount: ear gone, boss gone, 5 mm lower): no lane
        # exists beside it on the deck, so ride the 9 mm gap between the
        # holster top and the hex-plate underside (deck0+30.5), then drop
        # past the holster's +X face (x 85) to the corner tray.
        if leg == 0:
            head = [(20.0, 4.0, deck0 + 28.5),
                    (45.0, 3.0, deck0 + 28.5),
                    (70.0, 2.0, deck0 + 28.5),
                    (87.0, 1.0, deck0 + 28.5),
                    (88.0, 0.0, deck0 + 6.0)]
        else:
            detour = ([(-10.0, 0.0, deck0 + 19.0),
                       (-48.0, 0.0, deck0 + 19.0)] if leg == 3 else [])
            # Legs 2/4: starting from the offset trunk nuts (y -/+16) the
            # straight run to the NW/SW corner cuts the adapter's east
            # corner (board top deck0+12) -- hop it at deck0+14.5.
            if leg in (2, 4):
                detour = [(-8.0, 22.0 if leg == 2 else -22.0, deck0 + 14.5)]
            head = [*detour,
                    (62.0 * ca, 62.0 * sa, deck0 + 6.0),
                    (72.0 * ca, 72.0 * sa, 20.0)]
        # From the corner tray, come back INBOARD of the yaw-bearing tower
        # base (inboard face r ~ 54, tower band az corner+18..+42) before
        # turning to the drop slot -- the direct tray -> drop diagonal cuts
        # the tower base ~3.7 mm deep.  Legs 2 and 5 have a chassis
        # standoff at exactly corner+45 deg az mirrored onto the r~47 arc
        # (az 135 / 315, r 44): swing inboard of the post there.
        dodge = ([(38.0 * np.cos(a + np.pi / 12.0),
                   38.0 * np.sin(a + np.pi / 12.0), 7.0)]
                 if leg in (2, 5) else [])
        # Pin each conductor to its REAL nut ports: trunk (V+ = south,
        # GND = north) and the corner tray's single 5-port nut
        # (V+ = a port clockwise of the corner ray, GND = a port
        # counterclockwise).  The tray via sits at points index
        # 1 + len(head).
        def corner_nut(s: float) -> tuple[float, float, float]:
            return (wago_r * ca - s * nut_dt * sa,
                    wago_r * sa + s * nut_dt * ca, wago_top_z)

        via_idx = 1 + len(head)
        routes += _fan_points_route({
            "id": f"route-pwr-L{leg}",
            "points": pts(_WAGO_TRUNK_NODE, *head,
                          (wago_r * ca, wago_r * sa, wago_top_z),
                          (49.0 * ca, 49.0 * sa, 9.0),
                          *dodge,
                          (50.0 * ca2, 50.0 * sa2, 5.0),
                          _leg_drop(leg)),
            "maxLengthMm": 320.0,
            "label": (f"L{leg} power 16-18 AWG: trunk nuts (V+ = south, "
                      f"GND = north) -> corner 5-port Wago at az "
                      f"{leg * 60} deg (single press-fit 221-415 in the "
                      f"integrated chassis tray; V+ on a clockwise port, "
                      f"GND counterclockwise) -> inboard of the yaw "
                      f"tower -> leg {leg} drop"),
            "instances": ids("chassis_bottom")
            + ids_all("wago_trunk", "wago_power"),
        }, _PAIR_POWER, od_mm=2.4, overrides={
            "vp": {0: w(trunk_vp_nut), via_idx: w(corner_nut(-1.0))},
            "gnd": {0: w(trunk_gnd_nut), via_idx: w(corner_nut(+1.0))},
        })

    # -- 5x leg-to-leg data jumpers via underside data Wagos ----------------
    # Each jumper dips UNDER the belly at the chord midpoint: the straight
    # src-to-src chord passes r ~ 44 where the inter-plate chassis
    # standoffs stand (Aug 2026; the old L2L3 bay-battery detour is gone
    # with the bay pack).  z -12 clears the belly (-6) and stays beside
    # the under-belly pack block (block corners r ~ 52 at az 74 + k*60,
    # the chords pass r ~ 46 there).
    for a, b in zip(legs, legs[1:]):
        sa_, sb_ = _leg_src(a), _leg_src(b)
        mx, my = (sa_[0] + sb_[0]) / 2.0, (sa_[1] + sb_[1]) / 2.0
        mr = float(np.hypot(mx, my))
        # Bow the midpoint OUT to r 58: the under-belly pack block reaches
        # r 52.0 at its corners, and the chassis standoffs stand at r 44.
        mid = (mx / mr * 58.0, my / mr * 58.0, -12.0)
        routes += _fan_points_route({
            "id": f"route-data-L{a}L{b}",
            "points": pts(sa_, mid, sb_),
            "maxLengthMm": 300.0,
            "label": (f"leg {a} -> leg {b} data jumper via underside data "
                      f"Wagos, dipping under the belly around the LiPo "
                      f"packs (signal+GND only, NO V+)"),
            "instances": ids("chassis_bottom") + ids_all("wago_data"),
        }, _PAIR_JUMPER, od_mm=1.3)

    return routes


def main(single_leg: bool = False, motion: bool = True) -> None:
    if STL_DIR.exists():
        shutil.rmtree(STL_DIR)
    STL_DIR.mkdir(parents=True, exist_ok=True)

    # Guard: the local-frame split reproduces the authoritative verifier poses.
    _assert_servo_placement()

    leg0 = _leg0_local_parts()        # (name, local_mesh, M0)  leg-0 frame
    body = _body_local_parts()        # (name, local_mesh, M0)  pre-lift

    # Lift so the lowest leg point sits on z = 0 (on the PLACED leg-0 parts).
    z_min = min(float(local.copy().apply_transform(M0).bounds[0][2])
                for _n, local, M0 in leg0)
    chassis_lift = -z_min
    Tlift = _trans([0.0, 0.0, chassis_lift])

    legs = [0] if single_leg else list(range(6))

    # Each entry: (name, leg, mesh_key, local_mesh, M_world, role, joint, color).
    # M_world places local_mesh into the final standing-pose WORLD frame; it is
    # shipped as the instance ``transform`` so BuildViz's pick read-out reports
    # CAD-local coordinates (worldToLocal = inverse(M_world)).
    plan: list[tuple] = []

    for name, local, M0 in body:
        plan.append((name, None, _scene_mesh_key(name), local, Tlift @ M0,
                     ROLE.get(name, "part"), None,
                     PALETTE.get(name, "#888888")))

    for i in legs:
        R = rotation_matrix(i * np.pi / 3.0, [0, 0, 1])
        for name, local, M0 in leg0:
            plan.append((name, i, _scene_mesh_key(name), local, Tlift @ R @ M0,
                         ROLE.get(name, "part"), None,
                         PALETTE.get(name, "#888888")))

    # COTS hardware placed independently in the SAME per-leg frame as the
    # verifier parts: every screw/nut/insert (registry) + the disc horns on
    # each servo output.  These are scene-only and NOT printable parts.
    for name, leg, joint, color, key, local, M in _fastener_instances():
        if single_leg and leg not in (0, None):
            continue
        plan.append((name, leg, key, local, Tlift @ M, "fastener", joint, color))
    for name, leg, joint, color, key, local, M in _disc_horn_instances(legs):
        if single_leg and leg != 0:
            continue
        plan.append((name, leg, key, local, Tlift @ M, "horn", joint, color))
    for name, leg, joint, color, key, local, M in _passive_horn_instances(legs):
        if single_leg and leg != 0:
            continue
        role = "horn" if key == "disc_horn" else "frame"
        plan.append((name, leg, key, local, Tlift @ M, role, joint, color))

    # Export each UNIQUE local mesh once (identical parts across legs share an
    # STL) and reference it by id; the per-instance transform does the placing.
    mesh_id_for: dict[str, str] = {}
    meshes_json: list[dict] = []
    instances_json: list[dict] = []
    tagged: list[tuple[str, int | None, trimesh.Trimesh]] = []
    allb: list = []
    for idx, (name, leg, key, local, M_world, role, joint, color) in enumerate(plan):
        if key not in mesh_id_for:
            fname = f"{key}.stl"
            local.export(STL_DIR / fname)
            mesh_id_for[key] = f"stl:{key}"
            meshes_json.append({"id": f"stl:{key}", "name": fname,
                                "url": f"{SCENE_ASSET_BASE}/{fname}"})
        # World mesh for the (world-space) overlap sidecar + camera framing.
        world = local.copy()
        world.apply_transform(M_world)
        allb.append(world.bounds)
        tagged.append((name, leg, world))
        instances_json.append({
            "id": f"{idx:03d}-{name}",
            "meshId": mesh_id_for[key],
            "name": f"L{leg} {name}" if leg is not None else name,
            "partType": name,
            "role": role,
            "leg": leg,
            "joint": joint,
            "cots": role in COTS_ROLES,
            "color": color,
            "transform": _col_major(M_world),
            "centroid": [float(v) for v in world.centroid],
        })

    b = np.array(allb)
    center = [float(v) for v in (b[:, 0, :].min(0) + b[:, 1, :].max(0)) / 2.0]
    scene = {
        "name": "Hexapod STS3215 — full robot (verified standing pose)",
        "source": str(OUT_DIR),
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": center,
        # Project intent as DATA (BuildViz checksConfig): the intended part-type
        # matings that overlap by design, so the viewer's live checks mark them
        # pass(allowed) instead of flagging them.
        "checksConfig": {
            "overlapMm3": CHECK_OVERLAP_MM3,
            "pitchMm": CHECK_PITCH_MM,
            "ignoreOverlapPairs": sorted(sorted(p) for p in INTENDED_OVERLAP_PAIRS),
            # real bought-part masses + infill-corrected print densities so
            # get_mass_properties reports the as-built weight, not solid PLA
            "partMassesGrams": KNOWN_PART_MASSES_G,
            "partDensitiesGCm3": PART_DENSITIES_GCM3,
        },
        "meshes": meshes_json,
        "instances": instances_json,
    }

    # ADDITIVE motion, baked into this ONE scene (there is no separate
    # scene_motion.json / prototype_sts3215_motion build any more).  Generating
    # the joints/poses/animations is essentially free (~0.01 s of joint/pose math
    # on the already-placed meshes), so we ALWAYS bake them in -- mirroring the
    # prototype_v1 hexapod-prototype build.  Old viewers ignore the extra keys and
    # render the identical static home pose.
    if motion:
        joints, poses, animations = _build_motion(
            instances_json, chassis_lift, legs)
        scene["joints"] = joints
        scene["poses"] = poses
        scene["animations"] = animations

    # ADDITIVE cable/harness routes (BuildViz routing_reach gate + tube render):
    # the per-leg servo daisy chain, servo-boot attached (see the routes
    # section comment), plus the body-harness nets of WIRING.md §1/§6 (power
    # trunk/branches, jumpers, buck feed, 5 V logic, data head, IMU I2C).
    body_part_ids = {inst["partType"]: inst["id"]
                     for inst in instances_json if inst["leg"] is None}
    body_part_ids_all: dict[str, list[str]] = {}
    for inst in instances_json:
        if inst["leg"] is None:
            body_part_ids_all.setdefault(inst["partType"], []).append(
                inst["id"])
    leg_part_ids = {(inst["leg"], inst["partType"]): inst["id"]
                    for inst in instances_json}
    # Each leg's tail lead terminates on its underside data Wago; the wagos
    # are body instances (leg=None), so match each leg's nearest one.
    wago_data_pos = [(inst["id"], np.array(inst["transform"][12:15]))
                     for inst in instances_json
                     if inst["partType"] == "wago_data"]
    wago_data_by_leg = {}
    for leg in legs:
        src = np.array([*_leg_src(leg)[:2], _leg_src(leg)[2] + chassis_lift])
        if wago_data_pos:
            wago_data_by_leg[leg] = min(
                wago_data_pos, key=lambda p: np.linalg.norm(p[1] - src))[0]
    scene["routes"] = (_build_routes(chassis_lift, legs, leg_part_ids,
                                     wago_data_by_leg)
                       + _build_body_routes(chassis_lift, legs, body_part_ids,
                                            wago_data_by_leg,
                                            body_part_ids_all))

    (OUT_DIR / "scene.json").write_text(json.dumps(scene, indent=2))
    msg = f"Wrote {OUT_DIR/'scene.json'}  ({len(instances_json)} instances, "
    if motion:
        moved = sum(len(j["instances"]) for j in scene["joints"])
        msg += (f"lift={chassis_lift:.1f}; motion: {len(scene['joints'])} joints "
                f"driving {moved} instances, {len(scene['poses'])} poses, "
                f"{len(scene['animations'])} walk clip; "
                f"{len(scene['routes'])} harness routes)")
    else:
        msg += (f"lift={chassis_lift:.1f}; motion OFF; "
                f"{len(scene['routes'])} harness routes)")
    print(msg)

    # BuildViz-compatibility contract (see ~/buildviz/BUILDVIZ_COMPATIBILITY.md):
    # write a design_spec.yaml that covers every scene partType (and a
    # ``wiring:`` entry per published route), and copy the project-authored
    # assembly guide + BOM in under the contract's expected names so
    # full_robot_viz/ is a self-contained, compat-clean build dir.
    _write_design_spec(instances_json, scene["routes"])
    project_dir = _HERE.parent
    for source_name, dest_name in (
        ("PROTOTYPE.md", "ASSEMBLY.md"),
        ("docs/BOM.md", "BOM.md"),
    ):
        source = project_dir / source_name
        if source.exists():
            destination = OUT_DIR / dest_name
            if source_name == "PROTOTYPE.md":
                destination.write_text(
                    _rebase_project_markdown_links(source.read_text())
                )
            else:
                shutil.copy2(source, destination)

    # Precompute the BuildViz checks sidecar (read on scene load).
    sidecar = _build_checks_sidecar(tagged, instances_json, chassis_lift)
    (OUT_DIR / "buildviz_checks.json").write_text(json.dumps(sidecar, indent=2))
    s = sidecar["summary"]
    print(f"Wrote {OUT_DIR/'buildviz_checks.json'}  "
          f"({s['fail']} fail / {s['warn']} warn / {s['pass']} pass "
          f"of {s['total']} checks)")


if __name__ == "__main__":
    # Motion is ALWAYS baked into scene.json (generation is free).  ``--no-motion``
    # is a debug escape hatch for a static-only scene; there is no ``--motion``
    # flag any more because motion is the default.
    main(single_leg="--single" in sys.argv,
         motion="--no-motion" not in sys.argv)
