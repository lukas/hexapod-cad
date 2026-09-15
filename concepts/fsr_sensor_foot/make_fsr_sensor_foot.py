"""Generate an angle-tolerant MakerHawk RP-C10 contact-sensing foot.

The thin-film FSR stays flat and sees only the first 0.25 mm of axial shoe
travel. The ground-facing TPU part is a spherical tread over a rigid carriage.
Three long guide pins keep that carriage translating during oblique contact;
an annular shoulder then bypasses structural impact around the sensor.

The tread extends to 54 degrees from the tibia axis. The normal planted
geometry is about 40 degrees, leaving useful angular margin.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np
import trimesh
from trimesh.boolean import difference as boolean_difference
from trimesh.boolean import intersection as boolean_intersection
from trimesh.boolean import union as boolean_union


HERE = Path(__file__).resolve().parent
STL_DIR = HERE / "stl"
VIZ_DIR = HERE / "viz"
BUILD_ID = "prototype_sts3215/fsr-sensor-foot"

# Existing robot interface and kinematic station.
TUBE_OD = 8.0
TUBE_ID = 6.0
TUBE_BORE = 8.30
TUBE_SOCKET_DEPTH = 20.0
TUBE_END_Z = 8.0
KNEE_TO_TUBE_END = 142.0
KNEE_TO_TIP = KNEE_TO_TUBE_END + TUBE_END_Z

# MakerHawk RP-C10-ST / Amazon ASIN B0CZ6L5NMM listing dimensions.
FSR_HEAD_OD = 10.0
FSR_ACTIVE_OD = 8.0
FSR_OVERALL_L = 19.0
FSR_FILM_T = 0.40
FSR_TERMINAL_PITCH = 2.54
FSR_MIN_FORCE_G = 20.0
FSR_MAX_FORCE_G = 2000.0
FSR_MAX_N = FSR_MAX_FORCE_G * 9.80665 / 1000.0

# Angle-tolerant tread and rigid carriage.
PLANTED_TIBIA_ANGLE_DEG = 40.0
DESIGN_CONTACT_ANGLE_DEG = 50.0
TREAD_EDGE_ANGLE_DEG = 54.0
TREAD_SPHERE_R = 13.40
TREAD_T = 1.00
TREAD_INNER_R = TREAD_SPHERE_R - TREAD_T
TREAD_EDGE_R = TREAD_SPHERE_R * math.sin(math.radians(TREAD_EDGE_ANGLE_DEG))
TREAD_EDGE_Z = TREAD_SPHERE_R * (
    1.0 - math.cos(math.radians(TREAD_EDGE_ANGLE_DEG))
)
CORE_SUPPORT_ANGLE_DEG = DESIGN_CONTACT_ANGLE_DEG
CORE_SUPPORT_R = TREAD_INNER_R * math.sin(
    math.radians(CORE_SUPPORT_ANGLE_DEG)
)
CORE_SUPPORT_Z = TREAD_SPHERE_R - TREAD_INNER_R * math.cos(
    math.radians(CORE_SUPPORT_ANGLE_DEG)
)

CARRIAGE_NECK_R = 8.90
CARRIAGE_BEAD_R = 9.52
CARRIAGE_BEAD_Z = CORE_SUPPORT_Z + 0.07
TPU_LOCK_POCKET_R = 9.57
TPU_LOCK_POCKET_Z = CARRIAGE_BEAD_Z
TPU_LOCK_RELIEF_R = 9.20
TPU_LOCK_RELIEF_Z = CORE_SUPPORT_Z + 0.14
TPU_LOCK_THROAT_R = 9.02
TPU_LOCK_THROAT_Z = CORE_SUPPORT_Z + 0.29
TPU_LOCK_OUTER_R = 9.50
TPU_LOCK_OUTER_Z = CORE_SUPPORT_Z + 0.11
TPU_LOCK_RADIAL_UNDERCUT = CARRIAGE_BEAD_R - TPU_LOCK_THROAT_R
CARRIAGE_FLANGE_R = 9.80
CARRIAGE_FLANGE_Z0 = 6.05
CARRIAGE_TOP_Z = 6.30

GUIDE_COUNT = 3
GUIDE_CENTER_R = 7.50
GUIDE_PIN_R = 0.90
GUIDE_BORE_R = 1.25
GUIDE_TOP_Z = 13.00
GUIDE_BORE_TOP_Z = 13.50
GUIDE_TOWER_R = 1.70
GUIDE_ANGLES_DEG = (30.0, 150.0, 270.0)

# Housing, sensor gap, and overload bypass.
LOWER_COLLAR_R = 10.85
LOWER_COLLAR_BOTTOM_Z = 5.80
HOUSING_R = 11.50
HOUSING_MAIN_BOTTOM_Z = 6.55
HOUSING_TOP_Z = TUBE_END_Z
SLEEVE_OD = 13.0
SLEEVE_TOP_Z = TUBE_END_Z + TUBE_SOCKET_DEPTH + 0.50
ZIP_GROOVE_Z0 = 24.0
ZIP_GROOVE_Z1 = 26.5
ZIP_GROOVE_OD = 11.8
CLAMP_SLIT_W = 0.80

LOWER_SNAP_OPENING_R = 9.60
FLANGE_CAVITY_R = 10.10
HOUSING_STOP_Z = 6.55
STOP_TRAVEL = HOUSING_STOP_Z - CARRIAGE_TOP_Z

SPREADER_OD = 7.50
SPREADER_T = 0.25
SPREADER_Z0 = CARRIAGE_TOP_Z
SPREADER_Z1 = SPREADER_Z0 + SPREADER_T
SPRING_OUTER_R = 6.15
SPRING_RING_INNER_R = 5.50
SPRING_CENTER_R = 3.60
SPRING_BEAM_W = 1.00
SPRING_Z0 = 5.30
SPRING_Z1 = CARRIAGE_TOP_Z
SPRING_CLEARANCE_FLOOR_Z = 4.70
SENSOR_Z0 = 6.65
SENSOR_Z1 = SENSOR_Z0 + FSR_FILM_T
INITIAL_SENSOR_GAP = SENSOR_Z0 - SPREADER_Z1

SENSOR_POCKET_OD = 10.50
SENSOR_POCKET_Z0 = HOUSING_STOP_Z
SENSOR_POCKET_Z1 = 7.10
TAIL_FILM_W = 5.80
TAIL_BAY_W = 8.00
TAIL_BAY_END_Y = 17.50
TAIL_LOBE_W = 10.00

ROBOT_MASS_KG = 1.30
STATIC_TRIPOD_AVG_N = ROBOT_MASS_KG * 9.80665 / 3.0
RECORDED_WALKING_PEAK_N = 44.6966


def _mat(tx: float = 0.0, ty: float = 0.0, tz: float = 0.0) -> list[float]:
    return [
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        float(tx), float(ty), float(tz), 1,
    ]


def _rot_x_mat(
    angle_deg: float,
    tx: float = 0.0,
    ty: float = 0.0,
    tz: float = 0.0,
) -> list[float]:
    angle = math.radians(angle_deg)
    c = math.cos(angle)
    s = math.sin(angle)
    return [
        1, 0, 0, 0,
        0, c, s, 0,
        0, -s, c, 0,
        float(tx), float(ty), float(tz), 1,
    ]


def _box(
    extents: tuple[float, float, float],
    center: tuple[float, float, float],
) -> trimesh.Trimesh:
    mesh = trimesh.creation.box(extents=extents)
    mesh.apply_translation(center)
    return mesh


def _beam_xy(
    start: tuple[float, float],
    end: tuple[float, float],
    width: float,
    z0: float,
    z1: float,
) -> trimesh.Trimesh:
    """Rectangular XY beam between two points, used for TPU flexure spokes."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    mesh = trimesh.creation.box(extents=(length, width, z1 - z0))
    rotation = trimesh.transformations.rotation_matrix(
        math.atan2(dy, dx), (0.0, 0.0, 1.0)
    )
    mesh.apply_transform(rotation)
    mesh.apply_translation(
        ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0, (z0 + z1) / 2.0)
    )
    return mesh


def _cyl_z(
    radius: float,
    z0: float,
    z1: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
    sections: int = 128,
) -> trimesh.Trimesh:
    mesh = trimesh.creation.cylinder(
        radius=radius,
        height=z1 - z0,
        sections=sections,
    )
    mesh.apply_translation([center_x, center_y, (z0 + z1) / 2.0])
    return mesh


def _annular_cylinder_z(
    *,
    outer_r: float,
    inner_r: float,
    z0: float,
    z1: float,
    sections: int = 128,
) -> trimesh.Trimesh:
    """Closed annular cylinder without relying on a boolean."""
    vertices: list[tuple[float, float, float]] = []
    for z in (z0, z1):
        for radius in (outer_r, inner_r):
            for index in range(sections):
                angle = 2.0 * math.pi * index / sections
                vertices.append(
                    (radius * math.cos(angle), radius * math.sin(angle), z)
                )

    def at(layer: int, ring: int, index: int) -> int:
        return layer * 2 * sections + ring * sections + (index % sections)

    faces: list[tuple[int, int, int]] = []
    for index in range(sections):
        nxt = index + 1
        faces.extend(
            [
                (at(0, 0, index), at(1, 0, nxt), at(0, 0, nxt)),
                (at(0, 0, index), at(1, 0, index), at(1, 0, nxt)),
                (at(0, 1, index), at(0, 1, nxt), at(1, 1, nxt)),
                (at(0, 1, index), at(1, 1, nxt), at(1, 1, index)),
                (at(0, 0, index), at(0, 0, nxt), at(0, 1, nxt)),
                (at(0, 0, index), at(0, 1, nxt), at(0, 1, index)),
                (at(1, 0, index), at(1, 1, nxt), at(1, 0, nxt)),
                (at(1, 0, index), at(1, 1, index), at(1, 1, nxt)),
            ]
        )

    mesh = trimesh.Trimesh(
        vertices=np.asarray(vertices, dtype=float),
        faces=np.asarray(faces, dtype=int),
        process=True,
    )
    if mesh.volume < 0.0:
        mesh.invert()
    return mesh


def _solid_union(parts: Iterable[trimesh.Trimesh]) -> trimesh.Trimesh:
    mesh = boolean_union(list(parts), engine="manifold")
    mesh.remove_unreferenced_vertices()
    return mesh


def _solid_difference(
    body: trimesh.Trimesh,
    cuts: Iterable[trimesh.Trimesh],
) -> trimesh.Trimesh:
    mesh = boolean_difference([body, *list(cuts)], engine="manifold")
    mesh.remove_unreferenced_vertices()
    return mesh


def _solid_intersection(parts: Iterable[trimesh.Trimesh]) -> trimesh.Trimesh:
    mesh = boolean_intersection(list(parts), engine="manifold")
    mesh.remove_unreferenced_vertices()
    return mesh


def _guide_xy() -> list[tuple[float, float]]:
    return [
        (
            GUIDE_CENTER_R * math.cos(math.radians(angle)),
            GUIDE_CENTER_R * math.sin(math.radians(angle)),
        )
        for angle in GUIDE_ANGLES_DEG
    ]


def make_housing() -> trimesh.Trimesh:
    """Rigid socket, three guides, flat FSR roof, and annular hard stop."""
    guide_towers = [
        _cyl_z(
            GUIDE_TOWER_R,
            CARRIAGE_TOP_Z - 0.15,
            GUIDE_BORE_TOP_Z,
            center_x=x,
            center_y=y,
        )
        for x, y in _guide_xy()
    ]
    outer = _solid_union(
        [
            _cyl_z(LOWER_COLLAR_R, LOWER_COLLAR_BOTTOM_Z, HOUSING_TOP_Z),
            _cyl_z(HOUSING_R, HOUSING_MAIN_BOTTOM_Z, HOUSING_TOP_Z),
            _cyl_z(SLEEVE_OD / 2.0, HOUSING_TOP_Z - 0.15, SLEEVE_TOP_Z),
            *guide_towers,
            _box(
                (TAIL_LOBE_W, TAIL_BAY_END_Y - 9.5, 3.0),
                (0.0, (9.5 + TAIL_BAY_END_Y) / 2.0, 7.25),
            ),
        ]
    )

    cuts: list[trimesh.Trimesh] = [
        _cyl_z(TUBE_BORE / 2.0, TUBE_END_Z, SLEEVE_TOP_Z + 1.0),
        _box(
            (CLAMP_SLIT_W, 5.2, SLEEVE_TOP_Z - 9.0),
            (0.0, -5.5, (9.0 + SLEEVE_TOP_Z) / 2.0),
        ),
        _annular_cylinder_z(
            outer_r=7.0,
            inner_r=ZIP_GROOVE_OD / 2.0,
            z0=ZIP_GROOVE_Z0,
            z1=ZIP_GROOVE_Z1,
        ),
        # Split lower collar: the 19.2 mm opening expands over the 19.6 mm
        # carriage flange, then retains it while the leg is airborne.
        _cyl_z(
            LOWER_SNAP_OPENING_R,
            LOWER_COLLAR_BOTTOM_Z - 0.2,
            CARRIAGE_FLANGE_Z0,
        ),
        _cyl_z(FLANGE_CAVITY_R, CARRIAGE_FLANGE_Z0, HOUSING_STOP_Z),
        _box(
            (2.2, 1.0, HOUSING_STOP_Z - LOWER_COLLAR_BOTTOM_Z + 0.4),
            (LOWER_COLLAR_R - 1.0, 0.0, (LOWER_COLLAR_BOTTOM_Z + HOUSING_STOP_Z) / 2.0),
        ),
        _cyl_z(
            SENSOR_POCKET_OD / 2.0,
            SENSOR_POCKET_Z0,
            SENSOR_POCKET_Z1,
        ),
        _box(
            (TAIL_FILM_W, TAIL_BAY_END_Y, SENSOR_POCKET_Z1 - SENSOR_POCKET_Z0),
            (0.0, TAIL_BAY_END_Y / 2.0, (SENSOR_POCKET_Z0 + SENSOR_POCKET_Z1) / 2.0),
        ),
        _box(
            (TAIL_BAY_W, TAIL_BAY_END_Y - 7.0, 2.20),
            (0.0, (7.0 + TAIL_BAY_END_Y) / 2.0, 6.65),
        ),
    ]
    cuts.extend(
        _cyl_z(
            GUIDE_BORE_R,
            CARRIAGE_TOP_Z - 0.25,
            GUIDE_BORE_TOP_Z + 0.25,
            center_x=x,
            center_y=y,
        )
        for x, y in _guide_xy()
    )
    return _solid_difference(outer, cuts)


def _sphere_profile(radius: float, angle_deg: float, samples: int = 36) -> np.ndarray:
    angles = np.linspace(0.0, math.radians(angle_deg), samples)
    radial = radius * np.sin(angles)
    z = TREAD_SPHERE_R - radius * np.cos(angles)
    return np.column_stack([radial, z])


def make_tpu_tread() -> trimesh.Trimesh:
    """Spherical TPU grip with a captive return lip behind the carriage rim."""
    outer = _sphere_profile(TREAD_SPHERE_R, TREAD_EDGE_ANGLE_DEG)
    inner = _sphere_profile(TREAD_INNER_R, CORE_SUPPORT_ANGLE_DEG)[::-1]
    # The return lip is fully inside the housing's 9.60 mm moving opening.
    # Its 9.02 mm throat stretches over the 9.52 mm rigid carriage rim, then
    # relaxes into the recessed neck.  The seated profiles have clearance and
    # therefore do not depend on modeled interference or adhesive retention.
    lock_lip = np.array(
        [
            (TPU_LOCK_OUTER_R, TPU_LOCK_OUTER_Z),
            (TPU_LOCK_OUTER_R, TPU_LOCK_THROAT_Z),
            (TPU_LOCK_THROAT_R, TPU_LOCK_THROAT_Z),
            (TPU_LOCK_RELIEF_R, TPU_LOCK_RELIEF_Z),
            (TPU_LOCK_POCKET_R, TPU_LOCK_POCKET_Z),
        ]
    )
    profile = np.vstack([outer, lock_lip, inner])
    mesh = trimesh.creation.revolve(profile, sections=128)
    mesh.remove_unreferenced_vertices()
    return mesh


def make_guided_carriage() -> trimesh.Trimesh:
    """Rigid supported shoe core, snap flange, stop shoulder, and guide pins."""
    inner_support = _sphere_profile(TREAD_INNER_R, CORE_SUPPORT_ANGLE_DEG)
    profile = np.vstack(
        [
            inner_support,
            [
                (CARRIAGE_BEAD_R, CARRIAGE_BEAD_Z),
                (CARRIAGE_NECK_R, TPU_LOCK_RELIEF_Z),
                (CARRIAGE_NECK_R, CARRIAGE_FLANGE_Z0),
                (CARRIAGE_FLANGE_R, CARRIAGE_FLANGE_Z0),
                (CARRIAGE_FLANGE_R, CARRIAGE_TOP_Z),
                (0.0, CARRIAGE_TOP_Z),
            ],
        ]
    )
    core = trimesh.creation.revolve(profile, sections=128)
    pins = [
        _cyl_z(
            GUIDE_PIN_R,
            CARRIAGE_TOP_Z - 0.10,
            GUIDE_TOP_Z,
            center_x=x,
            center_y=y,
            sections=64,
        )
        for x, y in _guide_xy()
    ]
    body = _solid_union([core, *pins])
    # The spring's outer ring sits on the z=5.30 shelf.  Its three spokes and
    # center can flex downward into the deeper 0.60 mm relief after FSR contact.
    return _solid_difference(
        body,
        [
            _cyl_z(SPRING_OUTER_R + 0.10, SPRING_Z0, CARRIAGE_TOP_Z + 0.10),
            _cyl_z(
                SPRING_RING_INNER_R,
                SPRING_CLEARANCE_FLOOR_Z,
                SPRING_Z0 + 0.02,
            ),
        ],
    )


def make_tpu_sensing_spring() -> trimesh.Trimesh:
    """Flat three-spoke TPU spring supporting the rigid FSR spreader."""
    outer_ring = _annular_cylinder_z(
        outer_r=SPRING_OUTER_R,
        inner_r=SPRING_RING_INNER_R,
        z0=SPRING_Z0,
        z1=SPRING_Z1,
    )
    center = _cyl_z(SPRING_CENTER_R, SPRING_Z0, SPRING_Z1)
    beams: list[trimesh.Trimesh] = []
    for angle_deg in (0.0, 120.0, 240.0):
        angle = math.radians(angle_deg)
        start = (3.20 * math.cos(angle), 3.20 * math.sin(angle))
        end = (5.80 * math.cos(angle), 5.80 * math.sin(angle))
        beams.append(_beam_xy(start, end, SPRING_BEAM_W, SPRING_Z0, SPRING_Z1))
    return _solid_union([outer_ring, center, *beams])


def make_force_spreader() -> trimesh.Trimesh:
    return _cyl_z(SPREADER_OD / 2.0, SPREADER_Z0, SPREADER_Z1)


def make_sensor_reference() -> trimesh.Trimesh:
    film = _solid_union(
        [
            _cyl_z(FSR_HEAD_OD / 2.0, SENSOR_Z0, SENSOR_Z1),
            _box(
                (5.0, 11.5, FSR_FILM_T),
                (0.0, 8.25, (SENSOR_Z0 + SENSOR_Z1) / 2.0),
            ),
        ]
    )
    terminals = [
        _box(
            (0.82, 4.0, 0.62),
            (x, 12.0, (SENSOR_Z0 + SENSOR_Z1) / 2.0),
        )
        for x in (-FSR_TERMINAL_PITCH / 2.0, FSR_TERMINAL_PITCH / 2.0)
    ]
    return _solid_union([film, *terminals])


def make_tube_reference() -> trimesh.Trimesh:
    return _annular_cylinder_z(
        outer_r=TUBE_OD / 2.0,
        inner_r=TUBE_ID / 2.0,
        z0=TUBE_END_Z,
        z1=38.0,
    )


def make_ground_plate() -> trimesh.Trimesh:
    return _box((36.0, 36.0, 1.2), (0.0, 0.0, 0.0))


def make_load_arrow() -> trimesh.Trimesh:
    shaft = _cyl_z(0.55, 0.0, 13.1, sections=48)
    head = trimesh.creation.cone(radius=1.8, height=4.0, sections=64)
    head.apply_translation((0.0, 0.0, 13.0))
    return _solid_union([shaft, head])


def make_cutaway_housing(housing: trimesh.Trimesh) -> trimesh.Trimesh:
    cutter = _box((31.0, 50.0, 45.0), (-15.5, 0.0, 15.0))
    return _solid_intersection([housing, cutter])


def _assert_mesh(name: str, mesh: trimesh.Trimesh) -> None:
    if not mesh.is_watertight:
        raise RuntimeError(f"{name} is not watertight")
    if not mesh.is_winding_consistent:
        raise RuntimeError(f"{name} has inconsistent winding")
    if mesh.volume <= 0.0:
        raise RuntimeError(f"{name} has non-positive volume")


def _export_meshes() -> tuple[list[dict], dict[str, trimesh.Trimesh]]:
    STL_DIR.mkdir(parents=True, exist_ok=True)
    VIZ_DIR.mkdir(parents=True, exist_ok=True)

    housing = make_housing()
    carriage = make_guided_carriage()
    tread = make_tpu_tread()
    spring = make_tpu_sensing_spring()
    spreader = make_force_spreader()
    sensor = make_sensor_reference()
    tube = make_tube_reference()
    ground = make_ground_plate()
    arrow = make_load_arrow()

    meshes = {
        "fsr_foot_housing": housing,
        "fsr_guided_carriage": carriage,
        "fsr_tpu_tread": tread,
        "fsr_tpu_sensing_spring": spring,
        "fsr_force_spreader": spreader,
        "makerhawk_rpc10_sensor": sensor,
        "carbon_tube_reference": tube,
        "ground_plate": ground,
        "load_arrow": arrow,
    }
    for name, mesh in meshes.items():
        _assert_mesh(name, mesh)

    destinations = {
        "fsr_foot_housing": STL_DIR / "fsr_foot_housing.stl",
        "fsr_guided_carriage": STL_DIR / "fsr_guided_carriage.stl",
        "fsr_tpu_tread": STL_DIR / "fsr_tpu_tread.stl",
        "fsr_tpu_sensing_spring": STL_DIR / "fsr_tpu_sensing_spring.stl",
        "fsr_force_spreader": STL_DIR / "fsr_force_spreader.stl",
        "makerhawk_rpc10_sensor": VIZ_DIR / "makerhawk_rpc10_sensor.stl",
        "carbon_tube_reference": VIZ_DIR / "carbon_tube_reference.stl",
        "ground_plate": VIZ_DIR / "ground_plate.stl",
        "load_arrow": VIZ_DIR / "load_arrow.stl",
    }
    mesh_defs: list[dict] = []
    for name, path in destinations.items():
        meshes[name].export(path)
        mesh_defs.append(
            {
                "id": f"mesh:{name}",
                "name": path.name,
                "url": str(path.relative_to(HERE)),
            }
        )
    return mesh_defs, meshes


def _instance(
    iid: str,
    mesh: str,
    name: str,
    part_type: str,
    color: str,
    *,
    focus_group: str,
    role: str,
    tx: float = 0.0,
    ty: float = 0.0,
    tz: float = 0.0,
    transform: list[float] | None = None,
    cots: bool = False,
) -> dict:
    return {
        "id": iid,
        "meshId": f"mesh:{mesh}",
        "name": name,
        "partType": part_type,
        "role": role,
        "focusGroup": focus_group,
        "joint": "foot",
        "leg": 0,
        "cots": cots,
        "color": color,
        "transform": transform if transform is not None else _mat(tx, ty, tz),
    }


def _ground_height_at_radius(angle_deg: float, radius: float) -> float:
    angle = math.radians(angle_deg)
    return (
        TREAD_SPHERE_R
        - TREAD_SPHERE_R / math.cos(angle)
        + radius * math.tan(angle)
    )


def _geometry_report(meshes: dict[str, trimesh.Trimesh]) -> dict:
    printables = [
        "fsr_foot_housing",
        "fsr_guided_carriage",
        "fsr_tpu_tread",
        "fsr_tpu_sensing_spring",
        "fsr_force_spreader",
    ]
    lower_clearance = LOWER_COLLAR_BOTTOM_Z - _ground_height_at_radius(
        DESIGN_CONTACT_ANGLE_DEG, LOWER_COLLAR_R
    )
    main_clearance = HOUSING_MAIN_BOTTOM_Z - _ground_height_at_radius(
        DESIGN_CONTACT_ANGLE_DEG, HOUSING_R
    )
    return {
        "buildId": BUILD_ID,
        "kinematics": {
            "existingTubeLengthMm": KNEE_TO_TUBE_END,
            "tubeEndToGroundTipMm": TUBE_END_Z,
            "preservedKneeToGroundTipMm": KNEE_TO_TIP,
        },
        "sensor": {
            "model": "MakerHawk RP-C10-ST",
            "asin": "B0CZ6L5NMM",
            "headODMm": FSR_HEAD_OD,
            "activeODMm": FSR_ACTIVE_OD,
            "overallLengthMm": FSR_OVERALL_L,
            "filmThicknessMm": FSR_FILM_T,
            "terminalPitchMm": FSR_TERMINAL_PITCH,
            "listedForceRangeG": [FSR_MIN_FORCE_G, FSR_MAX_FORCE_G],
            "listedForceCeilingN": FSR_MAX_N,
        },
        "angledContact": {
            "robotPlantedTibiaFromGroundNormalDeg": PLANTED_TIBIA_ANGLE_DEG,
            "designContactAngleDeg": DESIGN_CONTACT_ANGLE_DEG,
            "sphericalTreadEdgeAngleDeg": TREAD_EDGE_ANGLE_DEG,
            "operatingMarginBeyondPlantedDeg": (
                DESIGN_CONTACT_ANGLE_DEG - PLANTED_TIBIA_ANGLE_DEG
            ),
            "axialForceFractionAtPlantedAngle": math.cos(
                math.radians(PLANTED_TIBIA_ANGLE_DEG)
            ),
            "lowerCollarGroundClearanceAtDesignAngleMm": lower_clearance,
            "mainHousingGroundClearanceAtDesignAngleMm": main_clearance,
            "tailOrientation": (
                "Point the sensor tail toward the chassis/uphill side. "
                "The spherical tread is azimuth-independent; the flat tail lobe is not."
            ),
        },
        "mechanism": {
            "treadSphereRadiusMm": TREAD_SPHERE_R,
            "treadThicknessMm": TREAD_T,
            "treadOuterDiameterAtEdgeMm": 2.0 * TREAD_EDGE_R,
            "treadLockCarriageRimDiameterMm": 2.0 * CARRIAGE_BEAD_R,
            "treadLockTpuThroatDiameterMm": 2.0 * TPU_LOCK_THROAT_R,
            "treadLockRadialUndercutMm": TPU_LOCK_RADIAL_UNDERCUT,
            "treadLockOuterClearanceInHousingMm": (
                LOWER_SNAP_OPENING_R - TPU_LOCK_OUTER_R
            ),
            "guideCount": GUIDE_COUNT,
            "guidePinDiameterMm": 2.0 * GUIDE_PIN_R,
            "guideBoreDiameterMm": 2.0 * GUIDE_BORE_R,
            "guideDiametralClearanceMm": 2.0 * (GUIDE_BORE_R - GUIDE_PIN_R),
            "engagedGuideLengthMm": GUIDE_TOP_Z - CARRIAGE_TOP_Z,
            "firstSensorContactGapMm": INITIAL_SENSOR_GAP,
            "positiveStopTravelMm": STOP_TRAVEL,
            "postContactSpringDeflectionToStopMm": STOP_TRAVEL - INITIAL_SENSOR_GAP,
            "sensingSpringThicknessMm": SPRING_Z1 - SPRING_Z0,
            "sensingSpringDownwardReliefMm": SPRING_Z0 - SPRING_CLEARANCE_FLOOR_Z,
            "retention": (
                "The TPU tread's 18.04 mm return lip snaps behind the carriage's "
                "19.04 mm rim without adhesive. The rigid carriage flange separately "
                "snaps through the split lower housing collar and retains the moving "
                "shoe only while airborne."
            ),
        },
        "loadProtection": {
            "staticTripodAveragePerFootN": STATIC_TRIPOD_AVG_N,
            "recordedWalkingPeakSingleFootN": RECORDED_WALKING_PEAK_N,
            "peakToListedSensorCeilingRatio": RECORDED_WALKING_PEAK_N / FSR_MAX_N,
            "decision": (
                "The guided carriage first loads the flat FSR through the 7.5 mm "
                "spreader, then its broad flange reaches an annular rigid stop. "
                "The stop and guide towers carry structural and lateral load."
            ),
        },
        "printables": {
            name: {
                "watertight": bool(meshes[name].is_watertight),
                "windingConsistent": bool(meshes[name].is_winding_consistent),
                "volumeMm3": float(meshes[name].volume),
                "boundsMm": meshes[name].bounds.tolist(),
            }
            for name in printables
        },
    }


def build_scene() -> dict:
    mesh_defs, meshes = _export_meshes()
    focus_group = "angle_tolerant_sensor_foot"
    instances = [
        _instance(
            "foot-housing",
            "fsr_foot_housing",
            "rigid sensor housing and three guide towers",
            "fsr_foot_housing",
            "#4f78b8",
            focus_group=focus_group,
            role="structure",
        ),
        _instance(
            "foot-sensor",
            "makerhawk_rpc10_sensor",
            "flat MakerHawk RP-C10 sensor",
            "makerhawk_rpc10_fsr",
            "#d9a441",
            focus_group=focus_group,
            role="sensor",
            cots=True,
        ),
        _instance(
            "foot-spring",
            "fsr_tpu_sensing_spring",
            "three-spoke TPU sensing and return spring",
            "fsr_tpu_sensing_spring",
            "#8ed7c8",
            focus_group=focus_group,
            role="sensor_compliance",
        ),
        _instance(
            "foot-spreader",
            "fsr_force_spreader",
            "7.5 mm smooth force spreader",
            "fsr_force_spreader",
            "#e6d9ae",
            focus_group=focus_group,
            role="load_spreader",
        ),
        _instance(
            "foot-carriage",
            "fsr_guided_carriage",
            "rigid translating shoe carriage with three guides",
            "fsr_guided_carriage",
            "#dd7b4a",
            focus_group=focus_group,
            role="guided_load_path",
        ),
        _instance(
            "foot-tread",
            "fsr_tpu_tread",
            "54 degree spherical TPU tread with captive snap lip",
            "fsr_tpu_tread",
            "#45b9a6",
            focus_group=focus_group,
            role="ground_contact",
        ),
        _instance(
            "foot-tube",
            "carbon_tube_reference",
            "existing Ø8 x Ø6 carbon-fibre tibia reference",
            "carbon_tube_reference",
            "#303840",
            focus_group=focus_group,
            role="robot_interface",
            cots=True,
        ),
    ]

    # The ground normal points +Y/+Z in foot coordinates, putting contact on
    # -Y. The sensor tail exits +Y, deliberately toward the chassis/uphill.
    angle = math.radians(PLANTED_TIBIA_ANGLE_DEG)
    normal_y = math.sin(angle)
    normal_z = math.cos(angle)
    contact_y = -TREAD_SPHERE_R * math.sin(angle)
    contact_z = TREAD_SPHERE_R * (1.0 - math.cos(angle))
    plate_t = 1.2
    plate_transform = _rot_x_mat(
        -PLANTED_TIBIA_ANGLE_DEG,
        ty=contact_y - normal_y * plate_t / 2.0,
        tz=contact_z - normal_z * plate_t / 2.0,
    )
    arrow_transform = _rot_x_mat(
        -PLANTED_TIBIA_ANGLE_DEG,
        # Keep the diagram arrow beside the mechanism so it explains the
        # reaction direction without becoming a fake collision in checks.
        tx=15.0,
        ty=contact_y,
        tz=contact_z,
    )
    instances.extend(
        [
            _instance(
                "foot-ground",
                "ground_plate",
                "ground tangent at the robot's 40 degree planted tibia angle",
                "inspection_fixture",
                "#747b86",
                focus_group=focus_group,
                role="ground_reference",
                transform=plate_transform,
                cots=True,
            ),
            _instance(
                "foot-load-arrow",
                "load_arrow",
                "ground reaction direction at 40 degrees",
                "load_vector",
                "#d95757",
                focus_group=focus_group,
                role="load_direction",
                transform=arrow_transform,
                cots=True,
            ),
        ]
    )

    report = _geometry_report(meshes)
    moving_parts = {
        "explode-tread": "foot-tread",
        "explode-carriage": "foot-carriage",
        "explode-spring": "foot-spring",
        "explode-spreader": "foot-spreader",
        "explode-sensor": "foot-sensor",
        "explode-housing": "foot-housing",
        "explode-tube": "foot-tube",
    }
    limits = {
        "explode-tread": (-13.0, 0.25),
        "explode-carriage": (-4.0, 0.25),
        "explode-spring": (0.0, 1.0),
        "explode-spreader": (0.0, 5.0),
        "explode-sensor": (0.0, 11.0),
        "explode-housing": (0.0, 23.0),
        "explode-tube": (0.0, 48.0),
    }
    joints = [
        {
            "id": joint_id,
            "type": "prismatic",
            "axis": [0.0, 0.0, 1.0],
            "origin": [0.0, 0.0, 0.0],
            "instances": [instance_id],
            "limits": {"min": limits[joint_id][0], "max": limits[joint_id][1]},
            "home": 0.0,
            "label": joint_id.replace("explode-", "") + " spacing / sensing stroke",
        }
        for joint_id, instance_id in moving_parts.items()
    ]
    joints.append(
        {
            "id": "explode-fixture",
            "type": "prismatic",
            "axis": [0.0, 0.0, 1.0],
            "origin": [0.0, 0.0, 0.0],
            "instances": ["foot-ground", "foot-load-arrow"],
            "limits": {"min": -30.0, "max": 0.0},
            "home": 0.0,
            "label": "ground/load reference spacing",
        }
    )
    poses = [
        {
            "id": "assembled-40deg",
            "name": "Assembled on 40° ground",
            "jointValues": {joint_id: 0.0 for joint_id in moving_parts},
        },
        {
            "id": "sensor-contact",
            "name": "0.10 mm: spreader reaches FSR",
            "jointValues": {
                "explode-tread": 0.10,
                "explode-carriage": 0.10,
                "explode-spring": 0.10,
                "explode-spreader": 0.10,
            },
        },
        {
            "id": "overload-stop",
            "name": "0.25 mm: carriage reaches hard stop",
            "jointValues": {
                "explode-tread": 0.25,
                "explode-carriage": 0.25,
                # The spring's center and the spreader remain against the FSR
                # while the carriage uses the remaining travel to reach stop.
                "explode-spring": 0.10,
                "explode-spreader": 0.10,
            },
        },
        {
            "id": "exploded",
            "name": "Exploded assembly",
            "jointValues": {
                "explode-tread": -13.0,
                "explode-carriage": -4.0,
                "explode-spring": 1.0,
                "explode-spreader": 5.0,
                "explode-sensor": 11.0,
                "explode-housing": 23.0,
                "explode-tube": 48.0,
                "explode-fixture": -30.0,
            },
        },
    ]
    animations = [
        {
            "id": "contact-stroke",
            "name": "Contact approach → hard stop",
            "loop": True,
            "duration": 3.0,
            "keyframes": [
                {"t": 0.0, "jointValues": {}},
                {
                    "t": 1.0,
                    "jointValues": {
                        "explode-tread": 0.10,
                        "explode-carriage": 0.10,
                        "explode-spring": 0.10,
                        "explode-spreader": 0.10,
                    },
                },
                {
                    "t": 2.0,
                    "jointValues": {
                        "explode-tread": 0.25,
                        "explode-carriage": 0.25,
                        "explode-spring": 0.10,
                        "explode-spreader": 0.10,
                    },
                },
                {"t": 3.0, "jointValues": {}},
            ],
        }
    ]
    return {
        "name": "STS3215 angle-tolerant guided RP-C10 sensor foot",
        "source": "make_fsr_sensor_foot.py",
        "buildId": BUILD_ID,
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": [0.0, 0.0, 19.0],
        "meshes": mesh_defs,
        "instances": instances,
        "joints": joints,
        "poses": poses,
        "animations": animations,
        "checksConfig": {
            "overlapMm3": 0.50,
            "pitchMm": 0.20,
            "partDensitiesGCm3": {
                "fsr_foot_housing": 1.27,
                "fsr_guided_carriage": 1.27,
                "fsr_tpu_tread": 1.20,
                "fsr_tpu_sensing_spring": 1.20,
                "fsr_force_spreader": 1.27,
                "makerhawk_rpc10_fsr": 1.40,
                "carbon_tube_reference": 1.55,
                "inspection_fixture": 0.05,
                "load_vector": 0.05,
            },
        },
        "analysis": {
            "purpose": (
                "Detect touchdown at the normal 40 degree tibia angle without "
                "letting side load fold the FSR or wedge a flexible flange."
            ),
            "views": {
                "assembled-40deg": "Connected home assembly on tangent ground; tail exits uphill.",
                "sensor-contact": "Carriage, TPU spring, and spreader translated 0.10 mm to first FSR contact.",
                "overload-stop": "Carriage reaches 0.25 mm while the spring center remains against the FSR.",
                "exploded": "One-click pose separating tread, carriage, TPU spring, spreader, sensor, housing, and tube.",
            },
            "printableCandidates": [
                "stl/fsr_foot_housing.stl",
                "stl/fsr_guided_carriage.stl",
                "stl/fsr_tpu_tread.stl",
                "stl/fsr_tpu_sensing_spring.stl",
                "stl/fsr_force_spreader.stl",
            ],
            "geometryReport": report,
        },
    }


def main() -> None:
    scene = build_scene()
    (HERE / "scene.json").write_text(json.dumps(scene, indent=2) + "\n")
    (HERE / "geometry_report.json").write_text(
        json.dumps(scene["analysis"]["geometryReport"], indent=2) + "\n"
    )
    print(f"Wrote {HERE / 'scene.json'}")
    print(f"Wrote {HERE / 'geometry_report.json'}")
    print(f"Wrote printable meshes to {STL_DIR}")


if __name__ == "__main__":
    main()
