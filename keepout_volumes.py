"""Named registry of keep-out / clearance volumes for the hexapod prototype.

The CAD geometry is driven by the constants in ``hexapod_prototype.py``.
The runtime validator (``_verify_prototype.py``) builds matching keep-out
volumes ad-hoc inside its various ``check_*`` helpers (horn-sweep
cylinders, pad-sweep cylinders, servo-body envelopes, wire-exit corridors).
This module re-exposes those same volumes as a flat dictionary of named
``trimesh`` primitives so OTHER tools (``scripts/render_views.py``,
``scripts/validate_geometry.py``) can render or probe them without having
to know how to construct each one.

Conventions
-----------

Every entry returns a watertight ``trimesh.Trimesh`` expressed in the
PART-LOCAL frame indicated by the entry name's prefix (``yaw_``,
``hip_pitch_``, ``knee_``, ``femur_``, ``tibia_``).  The naming intentionally
matches the keep-out volume names in ``design_spec.yaml`` so the validator
can cross-reference one against the other.

Usage
-----

    >>> from keepout_volumes import KEEP_OUT_VOLUMES
    >>> mesh = KEEP_OUT_VOLUMES["yaw_horn_sweep"]()    # call the factory

Each value is a CALLABLE returning a fresh mesh so callers can mutate the
returned mesh (e.g. apply a transform) without affecting others.

This module is a *thin* wrapper: prefer importing values from
``hexapod_prototype`` directly when the geometry depends on the SOURCE OF
TRUTH constants.  Anything you change here that should also change in the
parametric source is a bug.
"""

from __future__ import annotations

import os
import sys
from typing import Callable

import numpy as np
import trimesh
from trimesh.creation import box as _box_mesh
from trimesh.creation import cylinder as _cyl_mesh
from trimesh.transformations import rotation_matrix

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import hexapod_prototype as hp


# ---------------------------------------------------------------------------
# Primitive helpers (axis-aligned in the part-local frame)
# ---------------------------------------------------------------------------

def _cyl_axis(radius: float, height: float, *, axis: str = "z",
              centre=(0.0, 0.0, 0.0),
              sections: int = 48) -> trimesh.Trimesh:
    """Cylinder of given radius/height along the named axis, centred on
    ``centre`` in the part-local frame.
    """
    m = _cyl_mesh(radius=radius, height=height, sections=sections)
    if axis == "x":
        m.apply_transform(rotation_matrix(np.pi / 2.0, [0, 1, 0]))
    elif axis == "y":
        m.apply_transform(rotation_matrix(-np.pi / 2.0, [1, 0, 0]))
    elif axis == "z":
        pass
    else:
        raise ValueError(f"axis must be one of x|y|z, got {axis!r}")
    m.apply_translation(centre)
    return m


def _box(extents, centre=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    m = _box_mesh(extents=extents)
    m.apply_translation(centre)
    return m


# ---------------------------------------------------------------------------
# Factories
# ---------------------------------------------------------------------------
# Each factory takes no args and returns a fresh mesh.  Geometry tracks
# the SOURCE OF TRUTH constants in hexapod_prototype.py.


def _yaw_horn_sweep() -> trimesh.Trimesh:
    """Cylinder swept by the disc horn above the coxa_bracket flange
    (bracket-local frame).

    June 2026 disc-horn switch: the printed servo_horn_adapter has been
    retired; the link's pad now bolts directly onto the aluminum disc
    horn, so the swept stack collapses from
        SERVO_OUTPUT_H + PLASTIC_HORN_H + HORN_ADAPTER_T = 14 mm
    down to
        SERVO_OUTPUT_H + HORN_STACK_H = 10 mm
    where ``HORN_STACK_H == PLASTIC_HORN_H == 5 mm``.  Centred on the
    yaw axis above the flange top; anything solid inside this cylinder
    collides with the rotating horn assembly.
    """
    radius = hp.PLASTIC_HORN_X_TIP_R + 0.6
    height = hp.SERVO_OUTPUT_H + hp.HORN_STACK_H
    return _cyl_axis(
        radius=radius,
        height=height,
        axis="z",
        centre=(0.0, 0.0, hp.BRACKET_FLANGE_T + height / 2.0),
    )


def _yaw_servo_body_pocket() -> trimesh.Trimesh:
    """Yaw-servo body cavity that hangs below the coxa_bracket flange
    (bracket-local frame).
    """
    extents = (
        hp.SERVO_BODY_W + 2.0 * hp.WELL_BODY_CL,
        hp.SERVO_BODY_D + 2.0 * hp.WELL_BODY_CL,
        hp.SERVO_BODY_H + hp.WELL_TAB_FLOAT,
    )
    centre = (
        -hp.SERVO_OUTPUT_X,
        0.0,
        -hp.WELL_RIM_Z + extents[2] / 2.0,
    )
    return _box(extents, centre=centre)


def _yaw_wire_exit_corridor() -> trimesh.Trimesh:
    """Cuboid envelope of the L-shaped wire-exit corridor at the yaw
    well's +X bottom-outboard corner (bracket-local frame).
    """
    slot_x_min = -hp.SERVO_OUTPUT_X + hp.SERVO_BODY_W / 2.0 - hp.WIRE_SLOT_X_INBOARD
    slot_x_max = -hp.SERVO_OUTPUT_X + hp.WELL_W / 2.0 + hp.WIRE_SLOT_X_PAST_WALL
    z_top = hp.WIRE_SLOT_DEPTH - hp.WELL_RIM_Z
    z_bot = -hp.WELL_RIM_Z - hp.WELL_FLOOR_T - hp.WIRE_SLOT_Z_BELOW_FLOOR
    return _box(
        (slot_x_max - slot_x_min, hp.WIRE_SLOT_W, z_top - z_bot),
        centre=(
            0.5 * (slot_x_min + slot_x_max),
            0.0,
            0.5 * (z_top + z_bot),
        ),
    )


def _hip_pad_sweep() -> trimesh.Trimesh:
    """Cylinder swept by the femur's hip pad as it rotates about the
    hip-pitch axis (coxa_link-local frame).

    Anything solid inside this cylinder would collide with the femur's
    hip pad during normal pitch motion.
    """
    return _cyl_axis(
        radius=hp.HIP_PAD_R,
        height=2.0 * (hp.HORN_STACK_H + hp.LINK_THICKNESS + 2.0),
        axis="y",
        centre=(hp.COXA_LENGTH, hp.COXA_HIP_ANCHOR_Y, hp.COXA_HIP_DROP),
    )


def _hip_pitch_servo_body_pocket() -> trimesh.Trimesh:
    """Hip-pitch-servo body cavity in the coxa_link's outboard well
    (coxa_link-local frame).
    """
    extents = (
        hp.SERVO_BODY_W + 2.0 * hp.WELL_BODY_CL,
        hp.SERVO_BODY_H + hp.WELL_TAB_FLOAT,
        hp.SERVO_BODY_D + 2.0 * hp.WELL_BODY_CL,
    )
    centre = (
        hp.COXA_LENGTH - hp.SERVO_OUTPUT_X,
        -(hp.SERVO_BODY_H / 2.0 + hp.SERVO_OUTPUT_H) + hp.COXA_HIP_ANCHOR_Y,
        hp.COXA_HIP_DROP,
    )
    return _box(extents, centre=centre)


def _knee_servo_body_pocket() -> trimesh.Trimesh:
    """Knee servo body cavity in the femur's outboard well
    (NEW May 2026 collinear-pad femur-local frame).

    The well sits HORN_STACK_H deeper in NEW femur-Y than pre-refactor
    (the femur's NEW origin is the pad mating face, HORN_STACK_H
    above the joint axis along link +Y).  World coordinates of the
    pocket are unchanged -- the femur translates +HORN_STACK_H in
    coxa-Y to compensate.
    """
    extents = (
        hp.SERVO_BODY_W + 2.0 * hp.WELL_BODY_CL,
        hp.SERVO_BODY_H + hp.WELL_TAB_FLOAT,
        hp.SERVO_BODY_D + 2.0 * hp.WELL_BODY_CL,
    )
    centre = (
        hp.FEMUR_LENGTH - hp.SERVO_OUTPUT_X,
        -(hp.SERVO_BODY_H / 2.0 + hp.SERVO_OUTPUT_H) - hp.HORN_STACK_H,
        0.0,
    )
    return _box(extents, centre=centre)


def _femur_horn_stack_void() -> trimesh.Trimesh:
    """Cylindrical clearance void below the femur's hip pad mating face.

    Holds the aluminum disc horn when the femur is bolted directly to
    the hip-pitch servo (femur-local frame).  The void is still sized
    as a CONSERVATIVE keep-out to the now-retired plastic 4-arm
    X-horn's larger swept envelope.

    May 2026 collinear-pad refactor: the link's NEW local origin is
    the pad MATING FACE (= disc-horn-top plane), so the horn envelope
    is now at NEW y in [-HORN_STACK_H, 0] = [-5, 0] -- the same
    physical volume as before but expressed in the new origin.  Pre-
    refactor (origin = joint axis) the void sat at y in [-0.05,
    +5.05]; the new local Y origin shift of +HORN_STACK_H carries
    the void down to y in [-5.05, +0.05].  The link's material starts
    at NEW y = 0 (pad mating face) so the void is OUTSIDE the link
    body by construction -- no boolean diff is needed any more, this
    helper now just documents the (retired) X-horn envelope.
    """
    void_y_lo = -hp.HORN_STACK_H - 0.05
    void_y_hi = 0.05
    return _cyl_axis(
        radius=hp.HORN_STACK_VOID_R,
        height=void_y_hi - void_y_lo,
        axis="y",
        centre=(0.0, (void_y_lo + void_y_hi) / 2.0, 0.0),
    )


def _tibia_horn_stack_void() -> trimesh.Trimesh:
    """Same shape as the femur's, but in tibia-local coords."""
    void_y_lo = -hp.HORN_STACK_H - 0.05
    void_y_hi = 0.05
    return _cyl_axis(
        radius=hp.HORN_STACK_VOID_R,
        height=void_y_hi - void_y_lo,
        axis="y",
        centre=(0.0, (void_y_lo + void_y_hi) / 2.0, 0.0),
    )


# Aug 2026: ``_link_hub_horn_sweep`` DELETED.  It reserved the retired
# X-horn's swing volume below coxa-local z = 0, from the era when the coxa
# was a flat pad seated on top of a protruding horn.  Since the Jun 2026
# flush-horn depth fix the hub deliberately reaches down to
# YAW_HUB_BOSS_BOT_Z = -9 (drive nub through the mount-plate bore onto the
# recessed disc) and is BOLTED to the horn -- hub and horn rotate together,
# so the keep-out could only ever flag the design working as intended.


# ---------------------------------------------------------------------------
# Public registry
# ---------------------------------------------------------------------------

KEEP_OUT_VOLUMES: dict[str, Callable[[], trimesh.Trimesh]] = {
    # chassis_bottom yaw cradle-local (May 2026 Design F: previously
    # coxa_bracket-local; the yaw cradle moved into chassis_bottom).
    "yaw_horn_sweep":           _yaw_horn_sweep,
    "yaw_servo_body":           _yaw_servo_body_pocket,
    "yaw_wire_exit_corridor":   _yaw_wire_exit_corridor,
    # coxa_link-local (Aug 2026: link_hub_horn_sweep deleted -- see above)
    "hip_pad_sweep":            _hip_pad_sweep,
    "hip_pitch_servo_body":     _hip_pitch_servo_body_pocket,
    # femur-local
    "femur_horn_stack_void":    _femur_horn_stack_void,
    "knee_servo_body":          _knee_servo_body_pocket,
    # tibia-local
    "tibia_horn_stack_void":    _tibia_horn_stack_void,
}


# Map part-name (matching design_spec.yaml's `parts:` keys) to the list
# of keep-out volume keys defined in this module that apply to that
# part's LOCAL frame.  Used by render_views.py and validate_geometry.py.
PART_KEEPOUTS: dict[str, list[str]] = {
    "coxa_link": [
        "hip_pad_sweep",
        "hip_pitch_servo_body",
    ],
    "femur_link": [
        "femur_horn_stack_void",
        "knee_servo_body",
    ],
    "tibia_link": [
        "tibia_horn_stack_void",
    ],
}


def part_keepouts(part_name: str) -> dict[str, trimesh.Trimesh]:
    """Return ``{name: mesh}`` for every keep-out volume that applies to
    ``part_name``'s local frame.  Empty dict if the part has none.
    """
    return {
        key: KEEP_OUT_VOLUMES[key]()
        for key in PART_KEEPOUTS.get(part_name, [])
    }
