"""Shared color + role registry for the hexapod-walker prototype.

Both the PyVista build inspector (``inspect_build.py``) and the MuJoCo
viewer (``mujoco_prototype.py``) import from this module so a part type
is rendered with the *same* color in either tool.  One color per part
*type*, NOT per leg -- e.g. all six ``coxa_link`` instances share the
same green, regardless of which leg they live on.

Design choices
--------------
* The palette is a hand-tuned variant of matplotlib's ``tab10`` /
  ``tab20`` ordering.  Each printable part type gets one of the
  visually distinct hues; the two chassis plates use two related blues
  so they're obviously a pair but still individually identifiable.
* Servo proxies (``servo_body``, ``servo_horn``) are deliberately muted
  near-grays.  The printed parts are the "interesting" geometry an
  assembler looks at; the servos are hardware-store commodities and
  shouldn't compete for attention.
* The palette is fixed.  We do NOT randomize per run -- a part-type
  hue is meant to become muscle memory.

This module imports *only* the standard library so it can be imported
without pulling in trimesh / pyvista / mujoco / numpy.  That keeps the
import graph cheap for both visualization tools and any future CLI
that just wants the color table.
"""

from __future__ import annotations

# Hand-tuned palette (mostly matplotlib tab10 in RGB[0..1] form).
# Keep these ordered roughly chassis -> per-leg printed parts -> servo
# proxies so a quick visual scan of the legend matches the assembly
# order.
PART_COLORS: dict[str, tuple[float, float, float]] = {
    # Chassis "A" (top plate) - tab:blue
    "chassis_top":         (0.122, 0.467, 0.706),
    # Chassis "B" (bottom plate) - lighter sibling of chassis_top
    "chassis_bottom":      (0.475, 0.690, 0.882),
    # As-built hex board + raised platform
    "hex_mount_plate":     (0.580, 0.639, 0.722),
    "hex_raised_platform": (0.392, 0.455, 0.545),
    "hex_post_standoff":   (0.659, 0.635, 0.620),
    "hex_post_thumb_nut":  (0.471, 0.443, 0.424),
    "hex_post_magnet":     (0.118, 0.161, 0.231),
    # Arduino Uno Q board visual - muted green PCB
    "uno_q":               (0.106, 0.478, 0.239),
    "breakout":            (0.051, 0.580, 0.533),
    "pdb":                 (0.918, 0.345, 0.047),
    "motor_controller":    (0.706, 0.325, 0.035),
    "screen":              (0.145, 0.388, 0.922),
    "mpu6050":             (0.478, 0.361, 0.769),
    "wago_trunk":          (0.918, 0.345, 0.047),
    "wago_power":          (0.937, 0.267, 0.267),
    "wago_data":           (0.133, 0.773, 0.369),
    # Legacy alias: the corner power-Wago tray was a separate print
    # until late Aug 2026; its walls are now part of chassis_bottom.
    "wago_mount":          (0.961, 0.620, 0.043),
    # Sandwich-joint servo clamp cap - tab:gray-blue
    "servo_clamp_cap":     (0.290, 0.565, 0.851),
    # Coxa link - tab:green
    "coxa_link":           (0.173, 0.627, 0.173),
    # Femur link - tab:red
    "femur_link":          (0.839, 0.153, 0.157),
    # Tibia link - tab:pink
    "tibia_link":          (0.890, 0.467, 0.761),
    # Foot boot - tab:brown (TPU boot over the tibia tube end, Aug 2026;
    # replaces the hinged foot_pad + tibia_foot_fitting).
    "foot_boot":           (0.549, 0.337, 0.294),
    # Legacy alias kept so old scenes / labels still resolve a color.
    "foot_pad":            (0.549, 0.337, 0.294),
    # Servo horn adapter - tab:cyan
    "servo_horn_adapter":  (0.090, 0.745, 0.812),
    # Servo body - muted dark gray (commodity hardware)
    "servo_body":          (0.180, 0.180, 0.200),
    # Servo horn (plastic, ships with the servo) - muted mid gray
    "servo_horn":          (0.500, 0.500, 0.520),
    # ---- Fasteners (May 2026 -- see fastener_registry.py) -------------
    # M3 cap screws: dark steel (black-oxide finish).
    "M3x14 SHCS":          (0.30, 0.32, 0.36),
    "M3x8 SHCS":           (0.30, 0.32, 0.36),
    "M3x32 SHCS":          (0.30, 0.32, 0.36),
    # Cradle M3 x 8 SHCS that thread into a heat-set insert (same
    # physical bolt as ``M3x8 SHCS`` -- same P/N 91290A113 -- with a
    # distinct spec string so the inspector / BOM can tell cradle
    # bolts apart from any future "M3 x 8 into plastic" use).  Same
    # dark-steel hue.
    "M3x8 SHCS into heat-set insert": (0.30, 0.32, 0.36),
    # Link-to-disc-horn M3 x 6 cap screws: bluer steel, slightly
    # lighter than the M3 family (June 2026 disc-horn switch -- see
    # DISC_HORN_BOLT_PCD / DISC_HORN_* docstrings).
    "M3x6 SHCS":           (0.28, 0.34, 0.42),
    # M3 pan-head hinge pin: same dark steel family but a hint lighter
    # so the foot hinge bolt visually distinguishes from the cap screws.
    "M3x16 pan-head":      (0.36, 0.38, 0.40),
    # M2.5 spline screws: bluer steel so they stand out from the M3
    # family (and from the matte nuts).
    "M2.5x8 spline screw": (0.28, 0.32, 0.40),
    # M3 nyloc nuts: matte mid-gray.  The nylon insert ring is
    # conveyed by the mesh geometry itself (slightly inset cylinder
    # extruded above the steel hex body), not by per-vertex colors.
    "M3 nyloc nut":        (0.55, 0.55, 0.55),
    # M3 heat-set insert (McMaster 94459A130): brass / bronze hue so
    # the insert reads visually distinct from every steel fastener
    # when the build is exploded.  Matches the "brass-bolt" palette
    # family the spec calls for.
    "M3 heat-set insert":  (0.72, 0.45, 0.20),
}


# Aliases so callers (and the spec) can refer to the two chassis plates
# as "chassis_plate_a" / "chassis_plate_b" interchangeably with the
# concrete ``chassis_top`` / ``chassis_bottom`` STL names.
PART_COLORS["chassis_plate_a"] = PART_COLORS["chassis_top"]
PART_COLORS["chassis_plate_b"] = PART_COLORS["chassis_bottom"]


# Part types that don't carry a per-leg index in their label (they
# exist once per robot).
_CHASSIS_LEVEL = frozenset({
    "chassis_top", "chassis_bottom",
    "chassis_plate_a", "chassis_plate_b",
    "hex_mount_plate", "hex_raised_platform",
    "hex_post_standoff", "hex_post_thumb_nut", "hex_post_magnet",
    "uno_q", "breakout", "pdb", "motor_controller",
    "screen", "mpu6050", "wago_trunk", "wago_power", "wago_data",
    "wago_mount",
})


def is_chassis_level(part_type: str) -> bool:
    """True if this part type exists once per robot (no leg index)."""
    return part_type in _CHASSIS_LEVEL


_JOINT_ROLE = {
    "yaw":  "hip-yaw",
    "hip":  "hip-pitch",
    "knee": "knee",
}


_FASTENER_PART_TYPES = frozenset({
    "M3x14 SHCS",
    "M3x8 SHCS",
    "M3x8 SHCS into heat-set insert",
    "M3x32 SHCS",
    "M3x16 pan-head",
    "M3x6 SHCS",
    "M2.5x8 spline screw",
    "M3 nyloc nut",
    "M3 heat-set insert",
})


def is_fastener(part_type: str) -> bool:
    """True if this part type is a fastener (SHCS, nyloc nut, etc.).

    Used by the build inspector's "fasteners" master toggle to flip
    visibility on every fastener actor in one pass.
    """
    return part_type in _FASTENER_PART_TYPES


def instance_role(
    part_type: str,
    leg_index: int | None,
    joint: str | None,
    *,
    fastener_role: str | None = None,
) -> str:
    """Human-readable role label for one *instance* of a part type.

    Examples (matching the spec)::

        instance_role("coxa_link",    0,    None)  -> "hip-yaw -> hip-pitch"
        instance_role("femur_link",   3,    None)  -> "hip-pitch -> knee"
        instance_role("servo_body",   1,    "yaw") -> "hip-yaw servo"
        instance_role("chassis_top",  None, None)  -> "chassis"

    ``leg_index`` and ``joint`` are both optional so chassis-level
    parts can just pass ``None`` for both.
    """
    # Fasteners carry their own role string (built in the registry); it
    # is far more informative than a leg-index-stamped joint label.
    if part_type in _FASTENER_PART_TYPES:
        if fastener_role:
            return fastener_role
        # Fall back to a generic per-leg / per-joint label so the role
        # never goes empty if a caller forgot to forward the role.
        if joint:
            return f"{_JOINT_ROLE.get(joint, joint)} fastener"
        return "fastener"
    del leg_index  # the role string does not embed the leg index
    if part_type in _CHASSIS_LEVEL:
        return "chassis"
    if part_type == "coxa_link":
        return "hip-yaw -> hip-pitch"
    if part_type == "femur_link":
        return "hip-pitch -> knee"
    if part_type == "tibia_link":
        return "knee -> foot"
    if part_type in ("foot_pad", "foot_boot"):
        return "foot"
    if part_type == "servo_horn_adapter":
        suffix = _JOINT_ROLE.get(joint or "", "joint")
        return f"{suffix} output"
    if part_type == "servo_body":
        suffix = _JOINT_ROLE.get(joint or "", "joint")
        return f"{suffix} servo"
    if part_type == "servo_horn":
        suffix = _JOINT_ROLE.get(joint or "", "joint")
        return f"{suffix} horn"
    return ""


def instance_label(
    part_type: str,
    leg_index: int | None,
    joint: str | None = None,
    *,
    fastener_role: str | None = None,
) -> str:
    """Compose the full floating-label string used by the build inspector.

    Format:
        ``<part_type> L<idx>  <role>``     for per-leg parts
        ``<part_type>  <role>``            for chassis-level parts (no L<idx>)
    """
    role = instance_role(part_type, leg_index, joint, fastener_role=fastener_role)
    if part_type in _FASTENER_PART_TYPES:
        return f"{part_type}  {role}".rstrip()
    if is_chassis_level(part_type) or leg_index is None:
        return f"{part_type}  {role}".rstrip()
    return f"{part_type} L{leg_index}  {role}".rstrip()
