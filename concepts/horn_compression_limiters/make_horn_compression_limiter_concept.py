"""Experimental metal compression-limiters for every disc-horn joint.

This sidecar deliberately leaves production CAD unchanged.  It takes the
three current production part builders and enlarges only the four M3
link-to-disc-horn passages at each relevant interface:

* ``femur_link`` hip moving yoke, driven and passive arms;
* ``tibia_knee_yoke`` knee moving yoke, driven and passive arms;
* ``coxa_link`` yaw drive-nub stack (the four perimeter screws only).

The enlarged bores accept the user's metal compression spacers.  Bolt preload
then closes through screw head/washer -> metal tube ->
aluminium disc horn, instead of depending on the printed pad retaining its
thickness indefinitely.  Local yoke head pads and deeper yaw head seats make
every perimeter clamp path 10.1 mm; sleeves are never cut or stacked.

The purchased spacer dimensions and the coupon-tested printed-bore allowance
live in ``spacer_config.toml``.  Changing spacer OD keeps the same diametral
allowance and shifts both the part bores and the five-hole coupon with it.

Run from the repository root with ``uv`` (or use the Makefile target):

    uv run --no-project --python 3.12 --with trimesh --with numpy \
      --with manifold3d python \
      hexapod_walker/prototype_sts3215/concepts/horn_compression_limiters/\
make_horn_compression_limiter_concept.py
"""

from __future__ import annotations

import json
import math
import tomllib
from pathlib import Path

import numpy as np
import trimesh


HERE = Path(__file__).resolve().parent
STL_DIR = HERE / "stl"

import hexapod_prototype as hp
from scripts.print_orientation import _lay_flat, _reorient_coxa_link


BUILD_ID = "prototype_sts3215/horn-compression-limiters"
CONFIG_PATH = HERE / "spacer_config.toml"


def _load_spacer_config() -> dict:
    """Load the one user-editable hardware/fit configuration file."""
    with CONFIG_PATH.open("rb") as config_file:
        return tomllib.load(config_file)


_CONFIG = _load_spacer_config()

# Purchased hardware. Change these values in spacer_config.toml, not here.
SPACER_ENABLED = bool(_CONFIG["spacer"]["enabled"])
SPACER_OD = float(_CONFIG["spacer"]["outside_diameter_mm"])
SPACER_ID = float(_CONFIG["spacer"]["inside_diameter_mm"])
SPACER_LENGTH = float(_CONFIG["spacer"]["length_mm"])

# Printer compensation is an empirical diametral allowance. A nominal Ø5.80
# CAD hole can be the correct light push fit for a Ø5.50 spacer when printed
# sideways. The 0.30 mm is total diameter difference (0.15 mm per side).
BORE_DIAMETRAL_ALLOWANCE = float(
    _CONFIG["printed_fit"]["bore_diametral_allowance_mm"]
)
DESIGN_BORE_OD = round(SPACER_OD + BORE_DIAMETRAL_ALLOWANCE, 4)
COUPON_OFFSETS = tuple(
    float(value)
    for value in _CONFIG["printed_fit"][
        "coupon_offsets_from_selected_bore_mm"
    ]
)
COUPON_BORE_ODS = tuple(
    round(DESIGN_BORE_OD + offset, 4) for offset in COUPON_OFFSETS
)

if not (SPACER_OD > SPACER_ID > 0.0):
    raise ValueError("spacer OD must be greater than its positive ID")
if SPACER_LENGTH <= 0.0:
    raise ValueError("spacer length must be positive")
if BORE_DIAMETRAL_ALLOWANCE <= 0.0:
    raise ValueError(
        "printed-bore diametral allowance must be positive; choose it with "
        "the horizontal fit coupon"
    )
if len(COUPON_BORE_ODS) != 5 or 0.0 not in COUPON_OFFSETS:
    raise ValueError("fit coupon needs five offsets including the selected bore")

# Every perimeter horn fastener is redesigned around ONE configured spacer --
# no custom tube cutting and no end-to-end stacks. Give the free printed stack
# 0.10 mm extra height so tightening first compresses the plastic lightly, then
# bottoms metal on metal. With the current 10 mm spacer, a thin M3 washer and
# M3x12 screw leave about 1.5 mm thread engagement in the 2 mm aluminum disc.
SEAT_PRELOAD = 0.10
M3_WASHER_T = 0.50
M3_WASHER_OD = 7.00
PERIMETER_SCREW_LENGTH = 12.0

# The current assembled yoke path is 4 mm arm + 5 mm nominal reach pad =
# 9 mm.  Add a local head-bearing boss at each screw rather than thickening
# the entire arm and consuming sweep clearance.
YOKE_ASSEMBLED_STACK = hp._YOKE_ARM_T + hp.YOKE_ARM_PAD
YOKE_HEAD_BOSS_H = SPACER_LENGTH + SEAT_PRELOAD - YOKE_ASSEMBLED_STACK
YOKE_HEAD_BOSS_OD = 8.0  # supports the specified 7 mm OD M3 washer
assert YOKE_HEAD_BOSS_H > 0.0

# The washer must overlap the plastic outside the selected sleeve bore
# sleeve bore so the first 0.1 mm of clamp travel still preloads the print.
# A standard 7 mm OD M3 washer provides 0.5 mm radial overlap; widen the yaw
# access shafts so it can reach the deep perimeter seats.
YAW_ACCESS_OD = M3_WASHER_OD + 0.20

# The selected 5.80 mm horizontal bore comes within 0.1 mm of the edge of the
# production Phi20 yaw drive nub at the horn-facing end. Grow just that 4 mm-tall
# experimental neck to Phi22: it still has 1 mm radial clearance in the
# production Phi24 chassis opening and restores 1.0 mm of outer bore wall.
YAW_LIMITER_NUB_OD = 22.0

# Move the four yaw perimeter head seats down to one sleeve length above the
# actual horn mating face.  The centre spline screw keeps its production seat
# and length; it is not one of the four plastic-to-horn clamp columns.
YAW_PERIMETER_HEAD_SEAT_Z = (
    hp.YAW_HUB_BOSS_BOT_Z + SPACER_LENGTH + SEAT_PRELOAD
)


def _cyl_z(radius: float, z0: float, z1: float, x: float, y: float) -> trimesh.Trimesh:
    mesh = trimesh.creation.cylinder(
        radius=radius, height=z1 - z0, sections=64
    )
    mesh.apply_translation([x, y, 0.5 * (z0 + z1)])
    return mesh


def _cyl_y(radius: float, y0: float, y1: float, x: float, z: float) -> trimesh.Trimesh:
    """Cylinder along print Y, used to reproduce the parts' horizontal bores."""
    mesh = trimesh.creation.cylinder(
        radius=radius, height=y1 - y0, sections=64
    )
    mesh.apply_transform(
        trimesh.transformations.rotation_matrix(-math.pi / 2.0, [1, 0, 0])
    )
    mesh.apply_translation([x, 0.5 * (y0 + y1), z])
    return mesh


def _annular_spacer(length: float) -> trimesh.Trimesh:
    outer = trimesh.creation.cylinder(
        radius=SPACER_OD / 2.0, height=length, sections=96
    )
    inner = trimesh.creation.cylinder(
        radius=SPACER_ID / 2.0, height=length + 2.0, sections=96
    )
    sleeve = trimesh.boolean.difference([outer, inner], engine="manifold")
    # Local placement convention for the scene: z=0 is one squared end and
    # z=length is the other, so instances can be anchored at a mating face.
    sleeve.apply_translation([0.0, 0.0, length / 2.0])
    return sleeve


def _production_assembly_mesh(part_name: str) -> trimesh.Trimesh:
    """Build current CAD in joint/assembly coordinates before print posing."""
    builders = {
        "femur_link": hp.make_femur_link_part,
        "tibia_knee_yoke": hp.make_tibia_knee_yoke,
        "coxa_link": hp.make_coxa_link_part,
    }
    mesh = builders[part_name]()
    if not mesh.is_volume:
        raise RuntimeError(f"production source must be a closed volume: {part_name}")
    return mesh


def _union(source: trimesh.Trimesh, additions: list[trimesh.Trimesh]) -> trimesh.Trimesh:
    result = trimesh.boolean.union([source, *additions], engine="manifold")
    result.remove_unreferenced_vertices()
    if not result.is_volume:
        raise RuntimeError("compression-limiter union did not produce a closed volume")
    return result


def _add_yoke_head_bosses(source: trimesh.Trimesh) -> trimesh.Trimesh:
    """Add local head pads so the fixed 10 mm sleeves can bottom."""
    top_outer = hp.JOINT_HORN_TOP_Z + hp._YOKE_ARM_T
    bottom_outer = hp.JOINT_HORN_BOT_Z - hp._YOKE_ARM_T
    weld = 0.05
    additions = []
    for x, y in hp._disc_horn_bolt_centres():
        additions.append(
            _cyl_z(
                YOKE_HEAD_BOSS_OD / 2.0,
                top_outer - weld,
                top_outer + YOKE_HEAD_BOSS_H,
                x,
                y,
            )
        )
        additions.append(
            _cyl_z(
                YOKE_HEAD_BOSS_OD / 2.0,
                bottom_outer - YOKE_HEAD_BOSS_H,
                bottom_outer + weld,
                x,
                y,
            )
        )
    return _union(source, additions)


def _add_yaw_limiter_nub(source: trimesh.Trimesh) -> trimesh.Trimesh:
    """Restore wall around enlarged yaw bores without changing the chassis."""
    shell = _cyl_z(
        YAW_LIMITER_NUB_OD / 2.0,
        hp.YAW_HUB_BOSS_BOT_Z,
        hp.YAW_HUB_BOSS_WIDE_BOT_Z,
        0.0,
        0.0,
    )
    return _union(source, [shell])


def _yoke_limiter_cuts(bore_od: float = DESIGN_BORE_OD) -> list[trimesh.Trimesh]:
    """Four axial cuts that pass through both arms of one moving yoke."""
    z0 = hp.JOINT_HORN_BOT_Z - hp._YOKE_ARM_T - 2.0
    z1 = hp.JOINT_HORN_TOP_Z + hp._YOKE_ARM_T + 2.0
    return [
        _cyl_z(bore_od / 2.0, z0, z1, x, y)
        for x, y in hp._disc_horn_bolt_centres()
    ]


def _yaw_limiter_cuts(bore_od: float = DESIGN_BORE_OD) -> list[trimesh.Trimesh]:
    """Sleeve bores from the yaw horn face to the new deep head seats."""
    r = hp.DISC_HORN_BOLT_PCD / 2.0
    z0 = hp.YAW_HUB_BOSS_BOT_Z - 1.0
    z1 = YAW_PERIMETER_HEAD_SEAT_Z + 0.05
    return [
        _cyl_z(
            bore_od / 2.0,
            z0,
            z1,
            r * math.cos(angle),
            r * math.sin(angle),
        )
        for angle in hp.DISC_HORN_BOLT_ANGLES_RAD
    ]


def _yaw_head_access_cuts() -> list[trimesh.Trimesh]:
    """Continue the existing four screwdriver shafts down to the new seats."""
    r = hp.DISC_HORN_BOLT_PCD / 2.0
    return [
        _cyl_z(
            YAW_ACCESS_OD / 2.0,
            YAW_PERIMETER_HEAD_SEAT_Z,
            80.0,
            r * math.cos(angle),
            r * math.sin(angle),
        )
        for angle in hp.DISC_HORN_BOLT_ANGLES_RAD
    ]


def _difference(source: trimesh.Trimesh, cuts: list[trimesh.Trimesh]) -> trimesh.Trimesh:
    result = trimesh.boolean.difference([source, *cuts], engine="manifold")
    result.remove_unreferenced_vertices()
    if not result.is_volume:
        raise RuntimeError("compression-limiter boolean did not produce a closed volume")
    return result


def _geometry_report(
    *,
    source: trimesh.Trimesh,
    variant: trimesh.Trimesh,
    clearance_probes: list[trimesh.Trimesh],
    expect_modified: bool = True,
) -> dict:
    """Check either enlarged spacer bores or unchanged no-spacer geometry."""
    max_probe_overlap = 0.0
    for probe in clearance_probes:
        overlap = trimesh.boolean.intersection(
            [variant, probe], engine="manifold"
        )
        if overlap is not None and not overlap.is_empty:
            max_probe_overlap = max(max_probe_overlap, abs(float(overlap.volume)))
    bounds_delta = float(np.max(np.abs(source.bounds - variant.bounds)))
    removed = float(source.volume - variant.volume)
    result = {
        "sourceIsVolume": bool(source.is_volume),
        "variantIsVolume": bool(variant.is_volume),
        "removedVolumeMm3": removed,
        "maxBoundsDeltaMm": bounds_delta,
        "clearanceProbeODmm": SPACER_OD + 0.05,
        "maxOverlapWithClearanceProbeMm3": max_probe_overlap,
        "expectedMode": "spacer" if expect_modified else "no_spacer",
    }
    common_pass = bool(
        result["sourceIsVolume"]
        and result["variantIsVolume"]
        and bounds_delta < 1e-5
    )
    if expect_modified:
        result["pass"] = bool(
            common_pass and removed > 1.0 and max_probe_overlap < 1e-5
        )
    else:
        result["pass"] = bool(common_pass and abs(removed) < 1e-5)
    if not result["pass"]:
        raise RuntimeError(f"compression-limiter geometry check failed: {result}")
    return result


def _fit_coupon() -> trimesh.Trimesh:
    """Five horizontal bores matching the parts' orientation and 10.1 mm depth."""
    body = trimesh.creation.box(extents=(81.0, 10.1, 14.0))
    body.apply_translation([0.0, 0.0, 7.0])
    cuts: list[trimesh.Trimesh] = []
    for index, bore in enumerate(COUPON_BORE_ODS):
        x = -30.0 + index * 15.0
        cuts.append(_cyl_y(bore / 2.0, -6.0, 6.0, x, 7.0))
    # A single top/front notch marks the smallest (left) end. Read the other
    # four holes left-to-right in the configured order.
    marker = trimesh.creation.box(extents=(4.0, 4.0, 4.0))
    marker.apply_translation([-38.5, -4.5, 13.0])
    cuts.append(marker)
    return _difference(body, cuts)


def _mat(tx: float = 0.0, ty: float = 0.0, tz: float = 0.0) -> list[float]:
    return [
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        float(tx), float(ty), float(tz), 1,
    ]


def _instance(
    iid: str,
    mesh_id: str,
    name: str,
    part_type: str,
    color: str,
    *,
    tx: float = 0.0,
    ty: float = 0.0,
    tz: float = 0.0,
    role: str = "concept",
    focus_group: str,
    cots: bool = False,
) -> dict:
    return {
        "id": iid,
        "meshId": mesh_id,
        "name": name,
        "partType": part_type,
        "role": role,
        "focusGroup": focus_group,
        "joint": None,
        "leg": 0,
        "cots": cots,
        "color": color,
        "transform": _mat(tx, ty, tz),
    }


def _export_mesh(file_name: str, mesh: trimesh.Trimesh) -> dict:
    path = STL_DIR / file_name
    mesh.export(path)
    return {"id": f"stl:{path.stem}", "name": file_name, "url": f"stl/{file_name}"}


def _sleeve_instances(
    *,
    prefix: str,
    mesh_id: str,
    ty: float,
    z0: float,
    focus_group: str,
) -> list[dict]:
    out = []
    for index, (x, y) in enumerate(hp._disc_horn_bolt_centres()):
        out.append(
            _instance(
                f"{prefix}-{index}",
                mesh_id,
                f"{prefix} metal limiter {index + 1}",
                "metal_compression_limiter",
                "#d9b44a",
                tx=x,
                ty=ty + y,
                tz=z0,
                role="hardware",
                focus_group=focus_group,
                cots=True,
            )
        )
    return out


def build_scene() -> dict:
    STL_DIR.mkdir(parents=True, exist_ok=True)

    femur_source = _production_assembly_mesh("femur_link")
    tibia_source = _production_assembly_mesh("tibia_knee_yoke")
    coxa_source = _production_assembly_mesh("coxa_link")
    if SPACER_ENABLED:
        femur_bossed = _add_yoke_head_bosses(femur_source)
        tibia_bossed = _add_yoke_head_bosses(tibia_source)
        femur = _difference(femur_bossed, _yoke_limiter_cuts())
        tibia = _difference(tibia_bossed, _yoke_limiter_cuts())
        coxa_prepared = _add_yaw_limiter_nub(coxa_source)
        coxa = _difference(
            coxa_prepared, [*_yaw_limiter_cuts(), *_yaw_head_access_cuts()]
        )
        geometry_report = {
            "femur_link": _geometry_report(
                source=femur_bossed,
                variant=femur,
                clearance_probes=_yoke_limiter_cuts(SPACER_OD + 0.05),
            ),
            "tibia_knee_yoke": _geometry_report(
                source=tibia_bossed,
                variant=tibia,
                clearance_probes=_yoke_limiter_cuts(SPACER_OD + 0.05),
            ),
            "coxa_link_yaw": _geometry_report(
                source=coxa_prepared,
                variant=coxa,
                clearance_probes=_yaw_limiter_cuts(SPACER_OD + 0.05),
            ),
        }
    else:
        # The explicit no-spacer option is exactly the production geometry.
        femur = femur_source.copy()
        tibia = tibia_source.copy()
        coxa = coxa_source.copy()
        geometry_report = {
            "femur_link": _geometry_report(
                source=femur_source,
                variant=femur,
                clearance_probes=[],
                expect_modified=False,
            ),
            "tibia_knee_yoke": _geometry_report(
                source=tibia_source,
                variant=tibia,
                clearance_probes=[],
                expect_modified=False,
            ),
            "coxa_link_yaw": _geometry_report(
                source=coxa_source,
                variant=coxa,
                clearance_probes=[],
                expect_modified=False,
            ),
        }
    geometry_report["allPass"] = all(
        row["pass"] for row in geometry_report.values()
    )

    mesh_defs = [
        # Printable candidates use the same canonical print poses as today's
        # production STLs.  Separate assembly-coordinate copies let the scene
        # place the reference sleeves directly in the real joint bores.
        _export_mesh(
            "femur_link_compression_limiter_test.stl", _lay_flat(femur)
        ),
        _export_mesh(
            "tibia_knee_yoke_compression_limiter_test.stl", _lay_flat(tibia)
        ),
        _export_mesh(
            "coxa_link_yaw_compression_limiter_test.stl",
            _reorient_coxa_link(coxa),
        ),
        _export_mesh("scene_femur_link_compression_limiter_test.stl", femur),
        _export_mesh(
            "scene_tibia_knee_yoke_compression_limiter_test.stl", tibia
        ),
        _export_mesh("scene_coxa_link_yaw_compression_limiter_test.stl", coxa),
    ]
    if SPACER_ENABLED:
        mesh_defs.extend(
            [
                _export_mesh(
                    "metal_spacer_reference.stl",
                    _annular_spacer(SPACER_LENGTH),
                ),
                _export_mesh(
                    "spacer_fit_coupon_horizontal.stl",
                    _fit_coupon(),
                ),
            ]
        )

    # Separate lanes keep the generic overlap check meaningful while the
    # sleeve instances remain seated in their actual bore locations.
    femur_y = 0.0
    tibia_y = 78.0
    coxa_y = -92.0
    coupon_x, coupon_y = 205.0, 78.0

    part_description = (
        f"Ø{DESIGN_BORE_OD:.2f} horizontal spacer bores"
        if SPACER_ENABLED
        else "production no-spacer geometry"
    )
    instances = [
        _instance(
            "femur-test",
            "stl:scene_femur_link_compression_limiter_test",
            f"femur_link: hip yoke with {part_description}",
            "femur_link_compression_limiter_test",
            "#4d78a8",
            ty=femur_y,
            focus_group="hip_yoke",
        ),
        _instance(
            "tibia-test",
            "stl:scene_tibia_knee_yoke_compression_limiter_test",
            f"tibia_knee_yoke: knee yoke with {part_description}",
            "tibia_knee_yoke_compression_limiter_test",
            "#5b8fc9",
            ty=tibia_y,
            focus_group="knee_yoke",
        ),
        _instance(
            "coxa-test",
            "stl:scene_coxa_link_yaw_compression_limiter_test",
            f"coxa_link: yaw horn stack with {part_description}",
            "coxa_link_yaw_compression_limiter_test",
            "#9aa0a6",
            ty=coxa_y,
            focus_group="yaw_stack",
        ),
    ]
    if SPACER_ENABLED:
        instances.append(_instance(
            "fit-coupon",
            "stl:spacer_fit_coupon_horizontal",
            (
                f"horizontal fit coupon: Ø{COUPON_BORE_ODS[0]:.2f} to "
                f"Ø{COUPON_BORE_ODS[-1]:.2f}; selected "
                f"Ø{DESIGN_BORE_OD:.2f}"
            ),
            "compression_limiter_fit_coupon",
            "#dd8452",
            tx=coupon_x,
            ty=coupon_y,
            focus_group="fit_coupon",
        ))

    # Top/driven sleeve begins on the horn.  The boss's free outer face is
    # 0.10 mm beyond the sleeve, providing the intended initial compression.
    if SPACER_ENABLED:
        top_horn_face = hp.JOINT_HORN_TOP_Z - hp.YOKE_ARM_PAD
        bottom_sleeve_z0 = hp.PASSIVE_HORN_FACE_Z - SPACER_LENGTH
        for lane, name in ((femur_y, "hip"), (tibia_y, "knee")):
            instances.extend(
                _sleeve_instances(
                    prefix=f"{name}-driven",
                    mesh_id="stl:metal_spacer_reference",
                    ty=lane,
                    z0=top_horn_face,
                    focus_group=f"{name}_yoke",
                )
            )
            instances.extend(
                _sleeve_instances(
                    prefix=f"{name}-passive",
                    mesh_id="stl:metal_spacer_reference",
                    ty=lane,
                    z0=bottom_sleeve_z0,
                    focus_group=f"{name}_yoke",
                )
            )

        yaw_r = hp.DISC_HORN_BOLT_PCD / 2.0
        for index, angle in enumerate(hp.DISC_HORN_BOLT_ANGLES_RAD):
            instances.append(
                _instance(
                    f"yaw-limiter-{index}",
                    "stl:metal_spacer_reference",
                    f"yaw {SPACER_LENGTH:g} mm metal limiter {index + 1}",
                    "metal_compression_limiter",
                    "#d9b44a",
                    tx=yaw_r * math.cos(angle),
                    ty=coxa_y + yaw_r * math.sin(angle),
                    tz=hp.YAW_HUB_BOSS_BOT_Z,
                    role="hardware",
                    focus_group="yaw_stack",
                    cots=True,
                )
            )

    nominal_sleeve_to_horn_edge_gap = (
        hp.DISC_HORN_OD / 2.0
        - hp.DISC_HORN_BOLT_PCD / 2.0
        - SPACER_OD / 2.0
    )
    center_screw_head_od = 6.0
    center_head_to_sleeve_gap = (
        hp.DISC_HORN_BOLT_PCD / 2.0
        - center_screw_head_od / 2.0
        - SPACER_OD / 2.0
    )

    return {
        "name": "STS3215 horn compression-limiter experiment",
        "source": "make_horn_compression_limiter_concept.py",
        "buildId": BUILD_ID,
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": [75.0, 0.0, 18.0],
        "meshes": mesh_defs,
        "instances": instances,
        "checksConfig": {
            "overlapMm3": 5.0,
            "pitchMm": 0.4,
            "partDensitiesGCm3": {
                "femur_link_compression_limiter_test": 0.62,
                "tibia_knee_yoke_compression_limiter_test": 0.62,
                "coxa_link_yaw_compression_limiter_test": 0.62,
                "metal_compression_limiter": 7.9,
                "compression_limiter_fit_coupon": 0.62,
            },
        },
        "analysis": {
            "status": (
                "spacer experiment enabled; production CAD is unchanged"
                if SPACER_ENABLED
                else "no-spacer option; matches production CAD"
            ),
            "hardware": {
                "configFile": CONFIG_PATH.name,
                "spacerEnabled": SPACER_ENABLED,
                "activeMode": "spacer" if SPACER_ENABLED else "no_spacer",
                "spacerODmm": SPACER_OD,
                "spacerIDmm": SPACER_ID,
                "boreDiametralAllowanceMm": BORE_DIAMETRAL_ALLOWANCE,
                "nominalBoreRadialAllowanceMm": (
                    BORE_DIAMETRAL_ALLOWANCE / 2.0
                ),
                "designBoreODmm": DESIGN_BORE_OD,
                "fitCouponBoreODsMm": list(COUPON_BORE_ODS),
                "spacerLengthMm": SPACER_LENGTH,
                "seatPreloadMm": SEAT_PRELOAD,
                "perimeterScrew": (
                    "M3x12 SHCS with 7 mm OD x approximately 0.5 mm "
                    "standard M3 washer"
                ),
                "yawAccessShaftODmm": YAW_ACCESS_OD,
                "yawLimiterNubODmm": YAW_LIMITER_NUB_OD,
                "estimatedHornThreadEngagementMm": (
                    PERIMETER_SCREW_LENGTH - SPACER_LENGTH - M3_WASHER_T
                ),
            },
            "tightGeometry": {
                "hornBoltPCDmm": hp.DISC_HORN_BOLT_PCD,
                "nominalSleeveToHornEdgeGapMm": nominal_sleeve_to_horn_edge_gap,
                "conservativeCenterScrewHeadODmm": center_screw_head_od,
                "centerScrewHeadToSleeveGapMm": center_head_to_sleeve_gap,
                "note": (
                    "The horn's raised 9 mm spline boss faces the servo, not "
                    "the printed link. A conservative 6 mm center-screw head "
                    f"still clears each {SPACER_OD:g} mm spacer by "
                    f"{center_head_to_sleeve_gap:g} mm."
                ),
            },
            "decision": (
                (
                    f"Use one configured {SPACER_LENGTH:g} mm spacer per "
                    "perimeter screw. Yokes gain local head pads; yaw moves "
                    "only its four perimeter head seats down. Test one knee "
                    "interface first."
                )
                if SPACER_ENABLED
                else (
                    "Use unmodified production horn-fastener geometry with "
                    "no compression spacers or spacer-specific bosses."
                )
            ),
            "printableCandidates": [
                "stl/femur_link_compression_limiter_test.stl",
                "stl/tibia_knee_yoke_compression_limiter_test.stl",
                "stl/coxa_link_yaw_compression_limiter_test.stl",
            ],
            "geometryReport": geometry_report,
        },
    }


def main() -> None:
    scene = build_scene()
    (HERE / "scene.json").write_text(json.dumps(scene, indent=2) + "\n")
    (HERE / "geometry_report.json").write_text(
        json.dumps(scene["analysis"]["geometryReport"], indent=2) + "\n"
    )
    print(f"Wrote {HERE / 'scene.json'}")
    print(f"Wrote {STL_DIR}")
    print(
        (
            "Limiter proposal: "
            f"OD {SPACER_OD:.2f}, ID {SPACER_ID:.2f}, "
            f"L {SPACER_LENGTH:.2f} mm; "
            f"M3x{PERIMETER_SCREW_LENGTH:.0f} + "
            f"{M3_WASHER_T:.1f} mm washer"
        )
        if SPACER_ENABLED
        else "Horn fastener option: no spacer (production geometry)"
    )


if __name__ == "__main__":
    main()
