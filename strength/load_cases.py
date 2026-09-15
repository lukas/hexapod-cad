"""Per-part loading scenarios for the hexapod's printed parts.

Every load case below derives its numbers from the EXISTING design
constants in ``hexapod_prototype`` (link lengths, hip pad radius, foot
pad geometry, chassis layout) and from a SINGLE assembly-mass estimate
that we compute by summing trimesh volume * material density over every
printed part plus a fixed budget for the non-printed bits (servos,
electronics, battery, fasteners).

Nothing in here is hand-tuned to make a particular case "pass" -- the
goal is to give the user an honest peak-stress and safety-factor number
for the geometry they currently have.

All loads / lengths land in SI on the way OUT of the helpers so the FEA
deck writer and the beam-bending checker can consume them directly:

* Lengths     : metres
* Forces      : Newtons
* Moments     : Newton-metres
* Coordinates : metres (relative to the part's own STL origin, NOT the
                chassis frame -- each load case names the part and the
                CalculiX deck applies the load in the part's local
                frame so the STL doesn't need to be moved into a world
                pose first)

Internal-helper functions are mm-native so they read like the rest of
``hexapod_prototype.py`` -- they just convert to SI on the way out.
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field
from typing import Iterable

import numpy as np

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.dirname(THIS_DIR)
if PROTO_DIR not in sys.path:
    sys.path.insert(0, PROTO_DIR)

import hexapod_prototype as hp  # noqa: E402
from strength.materials import Material, get as _get_material  # noqa: E402


# ---------------------------------------------------------------------------
# Constants pulled out of hexapod_prototype for clarity
# ---------------------------------------------------------------------------

# Gravity (m / s^2).  Standard sea-level.
G_ACCEL = 9.81

# Dynamic / impact factor on a foot-strike.  "Worst-case the leg lands
# with the full reaction load * IMPACT_G_FACTOR" -- 2 g is the standard
# walking-robot conservative bound (Raibert 1986 hopping-robot work;
# Boston Dynamics BigDog landing-impact paper).  The user asked for this
# explicitly in the task brief.
IMPACT_G_FACTOR = 2.0

# DS3225 stall torque headline number is 25 kg-cm at 6.8 V.  In SI:
# 25 kgf-cm = 25 * 9.81e-3 N-m/cm * 1 cm = 2.45 N-m.  We round to the
# 2.5 N-m figure the user named in the brief and that the existing
# ``check_fastener_engagement`` / ``check_hipyaw_bolt_engagement``
# checks in ``_verify_prototype.py`` use to size bolt torque.
DS3225_STALL_TORQUE = 2.5  # N-m

# Non-printed mass budget (kg) -- everything that ISN'T in the
# printed-part list.  Sourced from docs/BOM.md:
#   18 x DS3225 servo bodies ~ 60 g each            = 1.08 kg
#   1 x 3S 2200 mAh LiPo ~ 185 g (Turnigy spec)    = 0.185 kg
#   1 x Raspberry Pi 4 / 5 + microSD + heatsink     = 0.050 kg
#   1 x Arduino Mega 2560 R3 (clone)                = 0.037 kg
#   2 x PCA9685 boards                              = 0.020 kg
#   2 x 5V 5A BECs + harness                        = 0.030 kg
#   1 x anti-spark switch + XT60 pigtails           = 0.020 kg
#   1 x MPU-6050 + dupont                           = 0.005 kg
#   262 x assorted M2 / M2.5 / M3 fasteners +
#         brass standoffs + heat-set inserts ~      = 0.100 kg
# Total non-printed mass:                           ~ 1.527 kg
NON_PRINTED_MASS_BUDGET = 1.527

# Per-component breakdown so the report can quote the individual lines.
NON_PRINTED_MASS_DETAIL = {
    "18 x DS3225 servos":                 18 * 0.060,
    "3S 2200 mAh LiPo + leads":           0.185,
    "Raspberry Pi 4/5 + SD":              0.050,
    "Arduino Mega 2560":                  0.037,
    "2 x PCA9685":                        0.020,
    "2 x BEC + harness":                  0.030,
    "Anti-spark switch + XT60 pigtails":  0.020,
    "MPU-6050 + dupont":                  0.005,
    "Fasteners + standoffs + inserts":    0.100,
    "Wiring + heatshrink + ziptie misc":  0.080,
}

# Battery hangs off chassis_bottom (two shorty packs velcro'd UNDER the
# flat belly, Aug 2026 -- was one 0.300 kg bay pack on the top face),
# not on chassis_top.  Pull it out for the chassis_top load case.
# 2 x Zeee 3S 2200 mAh shorty at ~137 g each.
BATTERY_MASS_ON_BOTTOM = 0.274


# Printed parts handled by the strength pipeline.  Subset of the
# ``_MESH_BUILDERS`` registry in ``_verify_prototype.py`` -- we leave
# the small visual-only meshes (servo_body, servo_horn, IMU pad, etc.)
# out because they're either not load-bearing or their failure mode is
# bond-line shear rather than bulk yield.
# Aug 2026: foot_pad dropped -- the TPU foot_boot is elastomeric (its
# "failure" mode is compression set, not bulk yield) and it is already
# included in the tibia_link assembled mesh for mass purposes.
PRINTED_PARTS_FOR_STRENGTH = (
    "tibia_link",
    "femur_link",
    "coxa_link",
    "chassis_bottom",
    "chassis_top",
)

# Per-leg copies of the printed parts.  6 legs x 3 leg-side parts +
# 1 TPU foot boot each.
PER_LEG_PARTS = ("coxa_link", "femur_link", "tibia_link", "foot_boot")
NUM_LEGS = 6

# Tibia / femur orientation in their STL frames.  Per the
# ``LINK_THICKNESS`` block at line ~ 1463 of hexapod_prototype.py the
# link frames are:
#     +X = spar long axis
#     +Y = joint-axis direction (link thickness)
#     +Z = perpendicular to spar in the leg's motion plane
# so the foot tip on the tibia STL sits at link-local +X = TIBIA_LENGTH,
# and the femur knee end sits at link-local +X = FEMUR_LENGTH.  Both
# links have their hip / knee pad centred at link-local x = 0 with the
# 4-bolt disc-horn pattern on DISC_HORN_BOLT_PCD = 14 mm (June 2026 disc-horn
# switch; DISC_HORN_BOLT_PCD is the retained legacy name for the disc PCD).



# ---------------------------------------------------------------------------
# Mass model
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class MassReport:
    """Itemised mass estimate for the assembled prototype."""
    printed_mass_per_part: dict[str, float]    # kg, single copy
    printed_count_per_part: dict[str, int]
    non_printed_detail: dict[str, float]       # kg, per-line
    material_short_name: str                   # which Material was assumed

    @property
    def printed_total(self) -> float:
        return sum(
            m * self.printed_count_per_part[name]
            for name, m in self.printed_mass_per_part.items()
        )

    @property
    def non_printed_total(self) -> float:
        return sum(self.non_printed_detail.values())

    @property
    def total(self) -> float:
        return self.printed_total + self.non_printed_total


def _printed_count(part: str) -> int:
    """How many copies of ``part`` exist in the assembled robot."""
    if part in PER_LEG_PARTS:
        return NUM_LEGS
    return 1


def _printed_part_mass_kg(stl_path: str, material: Material) -> float:
    """Return the mass (kg) of one copy of the STL at ``stl_path``.

    Volume comes from trimesh; printed parts are sliced at the default
    print profile (25 % infill, 4 walls), so we multiply by an
    effective-density factor of ~ 0.40 to compare apples-to-apples
    against the dry-mass spec.  100 %-infill mass would over-estimate
    by 2-3x.
    """
    import trimesh
    mesh = trimesh.load(stl_path, force="mesh")
    volume_mm3 = float(mesh.volume) if mesh.volume > 0 else 0.0
    # mm^3 -> m^3
    volume_m3 = volume_mm3 * 1.0e-9
    # Effective infill density factor for 25 % gyroid + 4 walls.  Empirical
    # measurement from a Bambu X1C calibration cube (Generic PETG, the
    # exact profile in PROTOTYPE.md §3.1).  100 % infill would have
    # effective_factor = 1.0.
    effective_density = material.density * 0.40
    return volume_m3 * effective_density


def _printed_part_volume_m3(stl_path: str) -> float:
    import trimesh
    mesh = trimesh.load(stl_path, force="mesh")
    volume_mm3 = float(mesh.volume) if mesh.volume > 0 else 0.0
    return volume_mm3 * 1.0e-9


def build_mass_report(material: Material,
                      stl_dir: str | None = None,
                      ) -> MassReport:
    """Compute the assembled mass from STL volumes + the fixed non-printed budget."""
    if stl_dir is None:
        stl_dir = hp.STL_DIR
    masses: dict[str, float] = {}
    counts: dict[str, int] = {}
    # Use the full structural-printed set, not just PRINTED_PARTS_FOR_STRENGTH
    # -- battery_holder / electronics_tray / bec_cradle / switch_holster /
    # imu_pad all contribute mass even though we won't FEA them.
    all_printed = (
        "chassis_top", "chassis_bottom",
        "battery_holder", "electronics_tray",
        "bec_cradle", "switch_holster", "imu_pad",
        "coxa_link", "femur_link", "tibia_link", "foot_boot",
    )
    for part in all_printed:
        stl_path = os.path.join(stl_dir, f"{part}.stl")
        if not os.path.exists(stl_path):
            # Part not built yet; skip rather than crash.
            continue
        masses[part] = _printed_part_mass_kg(stl_path, material)
        counts[part] = _printed_count(part)
    return MassReport(
        printed_mass_per_part=masses,
        printed_count_per_part=counts,
        non_printed_detail=dict(NON_PRINTED_MASS_DETAIL),
        material_short_name=material.short_name,
    )


# ---------------------------------------------------------------------------
# Load cases
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class PointLoad:
    """A force vector applied at a point on the part (SI)."""
    location_m: tuple[float, float, float]   # part-local x, y, z (m)
    force_N: tuple[float, float, float]      # F_x, F_y, F_z (N)
    label: str = ""


@dataclass(frozen=True)
class FaceTraction:
    """A uniform pressure applied to a face of the part.

    The face is selected geometrically (by axis-aligned plane near a
    given coordinate) inside ``run_calculix._select_face``.
    """
    axis: str                  # "x" / "y" / "z"
    sign: int                  # +1 / -1; positive face means largest value
    pressure_Pa: float         # signed; positive = compressive into the face
    label: str = ""


@dataclass(frozen=True)
class ClampedRegion:
    """A region of the part that is fully fixed (u = v = w = 0).

    The region is a cylinder around a bolt-pattern centre or the full
    +/-z face of a small pad.  Resolved by the FEA deck writer at
    nodal-set construction time.
    """
    kind: str                  # "bolt_circle" | "face" | "edge_ring"
    centre_m: tuple[float, float, float]
    radius_m: float            # for bolt_circle / edge_ring
    axis: str = "z"            # for bolt_circle / face
    sign: int | None = None    # for "face" only; +1 / -1
    label: str = ""


@dataclass(frozen=True)
class LoadCase:
    """A single named load scenario on a single named printed part."""
    name: str                       # "tibia_link_foot_strike", etc.
    part: str                       # STL part name (matches stl_prototype/)
    description: str
    loads: tuple[PointLoad, ...] = field(default_factory=tuple)
    tractions: tuple[FaceTraction, ...] = field(default_factory=tuple)
    clamps: tuple[ClampedRegion, ...] = field(default_factory=tuple)
    # Beam-bending parameters (None for parts where the beam check
    # doesn't apply -- the chassis plates and the foot pad).
    beam_axis: str | None = None    # "x" / "y" / "z" -- spar long axis
    beam_length_m: float | None = None
    beam_tip_force_N: float | None = None   # transverse force at the tip
    beam_root_clamp_m: float | None = None  # x-position of the fixed root


# Convenience: robot total weight reaction per leg in tripod stance
# (3 legs supporting at any one time).  Used by tibia / femur / foot pad.


def per_leg_static_load_N(total_mass_kg: float) -> float:
    """Static reaction at one foot pad, tripod stance (3 legs).

    weight is m * g; divided over 3 legs gives the per-leg static
    reaction.  Multiply by IMPACT_G_FACTOR (2) for the worst-case
    foot-strike spike documented in the task brief.
    """
    return total_mass_kg * G_ACCEL / 3.0


def per_leg_impact_load_N(total_mass_kg: float) -> float:
    """Foot-strike impact load = static * IMPACT_G_FACTOR."""
    return per_leg_static_load_N(total_mass_kg) * IMPACT_G_FACTOR


# ---------------------------------------------------------------------------
# Helpers for picking off positions from the STL frame
# ---------------------------------------------------------------------------


def _mm_to_m(v: float) -> float:
    return v * 1.0e-3


def _disc_horn_bolt_circle_radius_m() -> float:
    return _mm_to_m(hp.DISC_HORN_BOLT_PCD / 2.0)


# ---------------------------------------------------------------------------
# Per-part load case definitions
# ---------------------------------------------------------------------------


def tibia_link_case(total_mass_kg: float) -> LoadCase:
    """Foot tip loaded vertically; reacted at the knee 4-bolt pattern.

    The tibia STL has the knee bolt circle at link-local (x = 0,
    y = 0, z = 0) with the 4 disc-horn bolts on DISC_HORN_BOLT_PCD =
    14 mm.  The foot tip (TPU boot face, Aug 2026) is at link-local
    x = TIBIA_LENGTH exactly.
    """
    F = per_leg_impact_load_N(total_mass_kg)   # downward foot load (N)
    L_m = _mm_to_m(hp.TIBIA_LENGTH)
    return LoadCase(
        name="tibia_link_foot_strike",
        part="tibia_link",
        description=(
            f"Foot tip loaded -Z with {F:.1f} N (= robot_weight/3 * "
            f"{IMPACT_G_FACTOR:g}g impact); reacted at the knee disc-horn "
            f"bolt circle (PCD {hp.DISC_HORN_BOLT_PCD} mm)."
        ),
        loads=(
            PointLoad(
                location_m=(L_m, 0.0, 0.0),
                force_N=(0.0, 0.0, -F),
                label="foot tip -Z impact",
            ),
        ),
        clamps=(
            ClampedRegion(
                kind="bolt_circle",
                centre_m=(0.0, 0.0, 0.0),
                radius_m=_disc_horn_bolt_circle_radius_m(),
                axis="y",
                label="knee 4-bolt disc-horn pattern",
            ),
        ),
        beam_axis="x",
        beam_length_m=L_m,
        beam_tip_force_N=F,
        beam_root_clamp_m=0.0,
    )


def femur_link_case(total_mass_kg: float) -> LoadCase:
    """Hip cantilever with the foot-tip-equivalent vertical tip load.

    The femur STL has the hip pad at link-local (0, 0, 0); the knee
    pad sits at link-local (FEMUR_LENGTH, 0, 0).  In the assembled
    leg the knee bolts attach the tibia, so the load that arrives at
    the femur knee end is the same tibia foot-tip force resolved at
    x = FEMUR_LENGTH.  Reacted at the hip 4-bolt disc-horn pattern at
    the hip pad centre.

    The pitch angle (STANCE_FEMUR_DEG ~ -25 deg) means a vertical
    foot load decomposes into ~ cos(25 deg) of bending force at the
    knee + ~ sin(25 deg) of compression along the spar.  We use the
    full vertical magnitude as the bending force (worst-case "leg
    near horizontal") and ignore the spar-axis compression -- the
    spar in compression is orders of magnitude away from buckling at
    this scale.
    """
    F = per_leg_impact_load_N(total_mass_kg)
    L_m = _mm_to_m(hp.FEMUR_LENGTH)
    # Moment about the hip pad: F * L  (vertical force at the knee).
    M = F * L_m
    return LoadCase(
        name="femur_link_hip_cantilever",
        part="femur_link",
        description=(
            f"Knee-end loaded -Z with {F:.1f} N (foot-tip impact load "
            f"transferred through the tibia); reacted at the hip disc-horn "
            f"bolt circle.  Resulting moment at the hip = {M:.2f} N-m "
            f"(arm = {hp.FEMUR_LENGTH:.0f} mm)."
        ),
        loads=(
            PointLoad(
                location_m=(L_m, 0.0, 0.0),
                force_N=(0.0, 0.0, -F),
                label="knee -Z impact",
            ),
        ),
        clamps=(
            ClampedRegion(
                kind="bolt_circle",
                centre_m=(0.0, 0.0, 0.0),
                radius_m=_disc_horn_bolt_circle_radius_m(),
                axis="y",
                label="hip 4-bolt disc-horn pattern",
            ),
        ),
        beam_axis="x",
        beam_length_m=L_m,
        beam_tip_force_N=F,
        beam_root_clamp_m=0.0,
    )


def coxa_link_case(total_mass_kg: float) -> LoadCase:
    """Yaw stall torque applied at the hip-pitch pad; reacted at the
    yaw disc-horn 4-bolt pattern.

    The coxa link sits on the yaw servo's disc horn.  In stall the yaw
    servo delivers ~ 2.5 N-m which the coxa link arm has to carry
    out to the hip-pitch servo pad at link-local +X = COXA_LENGTH.
    We model that as a pure horizontal force F_h applied at the
    hip-pitch pad and a +X arm of COXA_LENGTH, giving
    F_h = T / COXA_LENGTH.  Direction is +Y (the yaw axis is Z, so a
    pure yaw torque applied to a +X-pointing arm rotates the arm in
    the X-Y plane, i.e. lateral force at the +X tip).

    Reacted at the yaw 4-bolt disc-horn pattern at the inboard end of
    the coxa link (link-local origin), which is bolted DOWN onto the
    yaw servo's disc horn.
    """
    L_m = _mm_to_m(hp.COXA_LENGTH)
    F_h = DS3225_STALL_TORQUE / L_m
    return LoadCase(
        name="coxa_link_yaw_stall",
        part="coxa_link",
        description=(
            f"DS3225 stall torque {DS3225_STALL_TORQUE:.2f} N-m applied "
            f"about the yaw axis at the hip-pitch pad ({hp.COXA_LENGTH:.0f} "
            f"mm out).  Equivalent lateral force = {F_h:.1f} N in +Y at "
            f"x = {hp.COXA_LENGTH:.0f} mm.  Reacted at the yaw disc-horn "
            f"bolt circle (PCD {hp.DISC_HORN_BOLT_PCD} mm)."
        ),
        loads=(
            PointLoad(
                location_m=(L_m, 0.0, 0.0),
                force_N=(0.0, F_h, 0.0),
                label="hip-pitch pad +Y stall force",
            ),
        ),
        clamps=(
            ClampedRegion(
                kind="bolt_circle",
                centre_m=(0.0, 0.0, 0.0),
                radius_m=_disc_horn_bolt_circle_radius_m(),
                axis="z",
                label="yaw 4-bolt disc-horn pattern",
            ),
        ),
        beam_axis="x",
        beam_length_m=L_m,
        beam_tip_force_N=F_h,
        beam_root_clamp_m=0.0,
    )


def _battery_holder_chassis_xy_m() -> tuple[float, float]:
    # Aug 2026: the under-belly pack block is centred on the origin
    # (BATTERY_UNDER_CENTRE), no longer on the old strap-slot datum.
    return (_mm_to_m(hp.BATTERY_UNDER_CENTRE[0]),
            _mm_to_m(hp.BATTERY_UNDER_CENTRE[1]))


def _electronics_tray_chassis_xy_m() -> tuple[float, float]:
    return (_mm_to_m(hp.ELEC_TRAY_CENTRE_X), _mm_to_m(hp.ELEC_TRAY_CENTRE_Y))


def _yaw_cradle_chassis_xy_m(leg_index: int) -> tuple[float, float]:
    """Centre of one yaw cradle on chassis_bottom (mm chassis-frame -> m).

    Per ``_chassis_yaw_cradle_to_well_local`` in _verify_prototype.py,
    the 6 cradles sit at the midpoints of the 6 hex edges, i.e.
    radius = CHASSIS_FLAT_TO_FLAT / 2 from the chassis centre at
    angles (i + 0.5) * 60 deg.
    """
    apothem = hp.CHASSIS_FLAT_TO_FLAT / 2.0
    a = (leg_index + 0.5) * math.pi / 3.0
    return (_mm_to_m(apothem * math.cos(a)), _mm_to_m(apothem * math.sin(a)))


def _chassis_standoff_holes_m() -> tuple[tuple[float, float], ...]:
    """4 mounting holes on the 35-mm-radius / 45-deg square pattern."""
    return tuple(
        (_mm_to_m(x), _mm_to_m(y))
        for x, y in hp.ELEC_CHASSIS_MOUNT_HOLES_XY
    )


def chassis_bottom_case(mass: MassReport) -> LoadCase:
    """Distributed dead load on chassis_bottom; supported at the 4 standoffs."""
    # Items that hang off chassis_bottom in the assembled robot:
    items = []
    # Two under-belly shorty LiPos (velcro'd; no printed holder).
    holder_m = mass.printed_mass_per_part.get("battery_holder", 0.0)
    bat_total = holder_m + BATTERY_MASS_ON_BOTTOM
    items.append((
        _battery_holder_chassis_xy_m(),
        bat_total * G_ACCEL,
        f"2x under-belly 3S shorty LiPo ({bat_total*1000:.0f} g)",
    ))
    # Electronics tray + boards.  Tray + Mega + Pi + 2 x PCA9685 + BECs.
    tray_m = mass.printed_mass_per_part.get("electronics_tray", 0.0)
    boards_m = (
        mass.non_printed_detail.get("Raspberry Pi 4/5 + SD", 0.0)
        + mass.non_printed_detail.get("Arduino Mega 2560", 0.0)
        + mass.non_printed_detail.get("2 x PCA9685", 0.0)
        + mass.non_printed_detail.get("2 x BEC + harness", 0.0)
    )
    tray_total = tray_m + boards_m
    items.append((
        _electronics_tray_chassis_xy_m(),
        tray_total * G_ACCEL,
        f"electronics_tray + boards ({tray_total*1000:.0f} g)",
    ))
    # 6 yaw cradles -- each holds one DS3225 servo + the coxa_link
    # arm bolted to it.  Per yaw-cradle mass = 1 servo + 1 coxa_link.
    coxa_m = mass.printed_mass_per_part.get("coxa_link", 0.0)
    # 1 servo ~ 0.060 kg.  But each cradle is the seat for the YAW
    # servo only; the hip-pitch and knee servos hang off coxa_link
    # and femur_link respectively, NOT chassis_bottom -- those are
    # leg dead loads applied at the foot pad, not on the chassis.
    yaw_servo_m = 0.060
    per_cradle_load = (coxa_m + yaw_servo_m) * G_ACCEL
    for leg in range(NUM_LEGS):
        items.append((
            _yaw_cradle_chassis_xy_m(leg),
            per_cradle_load,
            f"yaw cradle L{leg} (coxa_link + yaw servo)",
        ))

    # Plus the 6 LEGs hanging off the cradles.  Each leg's mass is
    # delivered as a downward dead load at its yaw cradle XY.  Per
    # leg below the yaw output = femur + tibia + foot pad + 2 servos.
    femur_m = mass.printed_mass_per_part.get("femur_link", 0.0)
    tibia_m = mass.printed_mass_per_part.get("tibia_link", 0.0)
    foot_m = mass.printed_mass_per_part.get("foot_boot", 0.0)
    per_leg_dead_load = (
        (femur_m + tibia_m + foot_m + 2 * 0.060) * G_ACCEL
    )
    # We add the per-leg dead load to the existing cradle entries.
    for i_leg_entry in range(len(items) - NUM_LEGS, len(items)):
        xy, load, label = items[i_leg_entry]
        items[i_leg_entry] = (xy, load + per_leg_dead_load,
                              label + " + leg below cradle")

    point_loads = tuple(
        PointLoad(
            location_m=(xy[0], xy[1], 0.0),
            force_N=(0.0, 0.0, -force),
            label=label,
        )
        for xy, force, label in items
    )
    # Supports: 4 chassis-standoff holes on the 35-mm-radius pattern.
    # Each is a small bolt hole; we model as a small bolt-circle clamp
    # with radius 4 mm around each XY (M3 clearance hole).
    clamps = tuple(
        ClampedRegion(
            kind="bolt_circle",
            centre_m=(x, y, 0.0),
            radius_m=_mm_to_m(4.0),
            axis="z",
            label=f"chassis-top standoff bolt {i}",
        )
        for i, (x, y) in enumerate(_chassis_standoff_holes_m())
    )
    total_F = sum(p.force_N[2] for p in point_loads)
    return LoadCase(
        name="chassis_bottom_dead_load",
        part="chassis_bottom",
        description=(
            f"Distributed dead load = battery + tray + 6 cradle/leg "
            f"masses = {-total_F:.1f} N total; supported at 4 brass "
            f"M3 standoffs on the 35-mm-radius / 45-deg square pattern."
        ),
        loads=point_loads,
        clamps=clamps,
    )


def chassis_top_case(mass: MassReport) -> LoadCase:
    """Same dead load minus the battery."""
    case = chassis_bottom_case(mass)
    # Strip the battery-holder entry (first item).
    new_loads = tuple(L for L in case.loads if "battery_holder" not in L.label)
    # And NEGATE the cradle/leg masses to zero -- chassis_top doesn't
    # carry the legs.  We keep ONLY the electronics_tray + switch_holster
    # + IMU pad block.  Easier to rebuild from scratch.
    items = []
    tray_m = mass.printed_mass_per_part.get("electronics_tray", 0.0)
    boards_m = (
        mass.non_printed_detail.get("Raspberry Pi 4/5 + SD", 0.0)
        + mass.non_printed_detail.get("Arduino Mega 2560", 0.0)
        + mass.non_printed_detail.get("2 x PCA9685", 0.0)
        + mass.non_printed_detail.get("2 x BEC + harness", 0.0)
    )
    tray_total = tray_m + boards_m
    items.append((
        (0.0, 0.0),
        tray_total * G_ACCEL,
        f"electronics_tray + boards ({tray_total*1000:.0f} g) "
        "(via standoffs onto chassis_top)",
    ))
    # Switch holster + IMU pad sit DIRECTLY on chassis_top, NOT via
    # the standoffs.  Use their chassis-XY positions.
    sw_m = mass.printed_mass_per_part.get("switch_holster", 0.0)
    items.append((
        (_mm_to_m(hp.SWITCH_HOLSTER_CENTRE_X),
         _mm_to_m(hp.SWITCH_HOLSTER_CENTRE_Y)),
        (sw_m + mass.non_printed_detail.get(
            "Anti-spark switch + XT60 pigtails", 0.0)) * G_ACCEL,
        "switch_holster + anti-spark switch",
    ))
    imu_m = mass.printed_mass_per_part.get("imu_pad", 0.0)
    items.append((
        (_mm_to_m(hp.IMU_PAD_CENTRE_X), _mm_to_m(hp.IMU_PAD_CENTRE_Y)),
        (imu_m + mass.non_printed_detail.get("MPU-6050 + dupont", 0.0)) * G_ACCEL,
        "imu_pad + MPU-6050",
    ))
    point_loads = tuple(
        PointLoad(
            location_m=(xy[0], xy[1], 0.0),
            force_N=(0.0, 0.0, -force),
            label=label,
        )
        for xy, force, label in items
    )
    clamps = tuple(
        ClampedRegion(
            kind="bolt_circle",
            centre_m=(x, y, 0.0),
            radius_m=_mm_to_m(4.0),
            axis="z",
            label=f"chassis-top standoff bolt {i}",
        )
        for i, (x, y) in enumerate(_chassis_standoff_holes_m())
    )
    total_F = sum(p.force_N[2] for p in point_loads)
    return LoadCase(
        name="chassis_top_dead_load",
        part="chassis_top",
        description=(
            f"Dead load on chassis_top = electronics_tray + switch + IMU "
            f"= {-total_F:.1f} N (excludes battery, which sits on "
            f"chassis_bottom).  Supported at the same 4 brass standoffs."
        ),
        loads=point_loads,
        clamps=clamps,
    )


# ``foot_pad_case`` RETIRED (Aug 2026): the TPU foot_boot replaced the
# hinged pad; an elastomer bulk-yield FEA case is not meaningful.


# ---------------------------------------------------------------------------
# Public dispatcher
# ---------------------------------------------------------------------------


_CASE_BUILDERS = {
    "tibia_link":     tibia_link_case,
    "femur_link":     femur_link_case,
    "coxa_link":      coxa_link_case,
}

_CASE_BUILDERS_MASS_REPORT = {
    "chassis_bottom": chassis_bottom_case,
    "chassis_top":    chassis_top_case,
}


def build_all_cases(mass: MassReport) -> dict[str, LoadCase]:
    """Build every load case for ``PRINTED_PARTS_FOR_STRENGTH`` from one mass report."""
    cases: dict[str, LoadCase] = {}
    for part in PRINTED_PARTS_FOR_STRENGTH:
        if part in _CASE_BUILDERS:
            cases[part] = _CASE_BUILDERS[part](mass.total)
        elif part in _CASE_BUILDERS_MASS_REPORT:
            cases[part] = _CASE_BUILDERS_MASS_REPORT[part](mass)
        else:
            raise KeyError(f"No load case for printed part {part!r}")
    return cases


__all__ = [
    "PointLoad", "FaceTraction", "ClampedRegion", "LoadCase",
    "MassReport",
    "PRINTED_PARTS_FOR_STRENGTH",
    "G_ACCEL", "IMPACT_G_FACTOR", "DS3225_STALL_TORQUE",
    "NON_PRINTED_MASS_BUDGET", "NON_PRINTED_MASS_DETAIL",
    "build_mass_report", "build_all_cases",
    "per_leg_static_load_N", "per_leg_impact_load_N",
]
