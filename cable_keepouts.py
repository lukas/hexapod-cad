"""Cable-clearance keep-out volumes for the stacked electronics decks.

Deck redesign (Jun 2026): the in-gap electronics_tray (Raspberry Pi +
USB-to-TTL bus adapter) is retired.  The brain is an Arduino Uno Q
(on-board Linux SoC + MCU) on the LOWER stacked deck that drives the
STS3215 serial bus DIRECTLY -- it replaces both the Pi and the bus
adapter.  A XINGYHENG 12V->5V buck converter rides the UPPER deck.

For each modelled connector on the Uno Q (USB-C power/data + the two
shield pin-header strips that carry the servo-bus + power harness) and
the buck converter (12 V-in / 5 V-out screw terminals), this module
exposes a ``trimesh.Trimesh`` box that covers the airspace the cable
plug + strain relief occupies once the cable is plugged in.
``check_cable_clearance`` in ``_verify_prototype.py`` asserts each
keep-out has < 50 mm^3 of overlap with any printed part, modelled
electronics body, or fastener mesh -- a future CAD edit that grew
material into a cable's plug-in airspace will FAIL the verifier
instead of being discovered the first time the cables are routed.

Frame
-----

All keep-out volumes are returned in the **CHASSIS frame** (the same
pre-chassis-lift world frame the fastener registry uses).  Z = 0 at
the chassis_bottom plate's CENTRE plane.  The decks bolt onto 4
standoff columns rising ABOVE chassis_top, whose top face is at
chassis-z = CHASSIS_GAP + 1.5 * CHASSIS_PLATE_T.  Both decks are
centred on the chassis Z axis at deck-local (0, 0).

Sources
-------

Connector positions match the ``make_uno_q_visual`` /
``make_buck_converter_visual`` proxy meshes in hexapod_prototype.py.
Where a connector position is uncertain we conservatively place it at
the centre of the appropriate board edge -- the > 50 mm^3 verifier
tolerance absorbs the 1-3 mm of slop this introduces.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np
import trimesh
from trimesh.creation import box as _box_mesh


import hexapod_prototype as hp


# ---------------------------------------------------------------------------
# Public dataclass
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CableKeepout:
    """One cable clearance airspace volume in the chassis frame.

    Fields
    ------
    part_name : str
        Board the connector lives on, e.g. ``"Mega2560"``.
    connector_name : str
        Per-board connector label, e.g. ``"USB-B"``.
    mesh : trimesh.Trimesh
        Watertight box covering the plug + cable strain relief
        airspace, in the chassis design frame (the same frame the
        fastener_registry uses; see ``cable_keepouts`` module
        docstring).
    """
    part_name: str
    connector_name: str
    mesh: trimesh.Trimesh


# ---------------------------------------------------------------------------
# Constants tied to the stacked decks' chassis-frame placement
# ---------------------------------------------------------------------------
#
# The decks bolt onto 4 standoff columns above chassis_top, whose top
# face sits at chassis-z = CHASSIS_GAP + 1.5 * CHASSIS_PLATE_T (deck
# z0).  Each board's PCB BOTTOM face lands at:
#     tray base + DECK_TRAY_T + DECK_STANDOFF_BOSS_H
# matching the deck placement used by build_prototype_assembly,
# inspect_build and full_robot_viz_build.

_DECK_Z0 = hp.CHASSIS_GAP + 1.5 * hp.CHASSIS_PLATE_T          # chassis_top top
_UNO_BOARD_BASE_Z = (_DECK_Z0 + hp.DECK_LEVEL_1_STANDOFF_H
                     + hp.DECK_TRAY_T + hp.DECK_STANDOFF_BOSS_H)   # PCB bottom
_BUCK_BOARD_BASE_Z = (_DECK_Z0 + hp.DECK_LEVEL_1_STANDOFF_H
                      + hp.DECK_LEVEL_2_STANDOFF_H
                      + hp.DECK_TRAY_T + hp.DECK_STANDOFF_BOSS_H)  # PCB bottom

_PCB_T = 1.6   # FR-4 thickness for both proxy boards (see make_*_visual)

# Both decks are centred on the chassis Z axis at deck-local (0, 0).
_UNO_HALF_W = hp.UNO_Q_PCB_W / 2.0
_UNO_HALF_D = hp.UNO_Q_PCB_D / 2.0
_BUCK_HALF_W = hp.BUCK_PCB_W / 2.0
_BUCK_HALF_D = hp.BUCK_PCB_D / 2.0


# ---------------------------------------------------------------------------
# Box primitive
# ---------------------------------------------------------------------------

def _aabb_box(x_lo: float, x_hi: float,
              y_lo: float, y_hi: float,
              z_lo: float, z_hi: float) -> trimesh.Trimesh:
    """Axis-aligned bounding box from min/max corner coordinates."""
    extents = (x_hi - x_lo, y_hi - y_lo, z_hi - z_lo)
    m = _box_mesh(extents=extents)
    m.apply_translation([
        (x_lo + x_hi) / 2.0,
        (y_lo + y_hi) / 2.0,
        (z_lo + z_hi) / 2.0,
    ])
    return m


# ---------------------------------------------------------------------------
# Arduino Uno Q connectors (lower deck)
# ---------------------------------------------------------------------------
#
# The Uno Q board is centred on the lower deck at deck-local (0, 0),
# long axis (UNO_Q_PCB_W = 68.58 mm) along chassis +X, short axis
# (UNO_Q_PCB_D = 53.34 mm) along chassis +Y.  ``make_uno_q_visual``
# puts the USB-C jack on the -X short edge and the two shield pin-
# header strips along the +/- Y long edges.
#
# The Uno Q sits on the LOWER deck; the buck tray hangs above it (its
# bottom face at chassis-z = DECK_Z0 + DECK_LEVEL_1 + DECK_LEVEL_2 =
# +76 mm).  So Uno Q cables exit HORIZONTALLY out past the deck edges
# rather than straight up into the buck tray:
#   * USB-C power/data  -- exits -X off the short edge.
#   * servo-bus + power harness on the shield headers -- exits +Y off
#     the long edge (the half-duplex TTL bus daisy-chains out to the
#     18 STS3215 servos from here).

def _uno_usb_c() -> trimesh.Trimesh:
    # USB-C jack centred at board-local (-_UNO_HALF_W + 4, +14) per
    # make_uno_q_visual.  Plug + strain relief exit -X off the short
    # edge.  12 mm wide (Y) x 7 mm tall plug face + ~26 mm plug body.
    cy = 14.0
    plug_face_x = -_UNO_HALF_W
    return _aabb_box(
        x_lo=plug_face_x - 26.0,  x_hi=plug_face_x + 4.0,
        y_lo=cy - 12.0 / 2.0,     y_hi=cy + 12.0 / 2.0,
        z_lo=_UNO_BOARD_BASE_Z,   z_hi=_UNO_BOARD_BASE_Z + 7.0,
    )


def _uno_bus_header() -> trimesh.Trimesh:
    # Shield pin-header strip on the +Y long edge (servo-bus + power
    # harness).  Connectors plug onto the header and the bundle exits
    # +Y off the long edge.  Slab spanning most of the board length in
    # X, rising over the ~9 mm header height, extending +Y for the
    # plug bodies + immediate strain relief.
    plug_face_y = _UNO_HALF_D - 3.0   # header strip centreline
    return _aabb_box(
        x_lo=-25.0,               x_hi=25.0,
        y_lo=plug_face_y,         y_hi=_UNO_HALF_D + 16.0,
        z_lo=_UNO_BOARD_BASE_Z + 2.0, z_hi=_UNO_BOARD_BASE_Z + 11.0,
    )


# ---------------------------------------------------------------------------
# XINGYHENG buck converter terminals (upper deck)
# ---------------------------------------------------------------------------
#
# The buck board is centred on the upper deck at deck-local (0, 0).
# ``make_buck_converter_visual`` places the two screw-terminal blocks
# at board-local (+/-(BUCK_PCB_W/2 - 6), -14).  The 12 V-in lead (from
# the LiPo / anti-spark switch) lands on one terminal and the 5 V-out
# logic-rail lead leaves the other.  Nothing sits above the upper
# deck, so both leads exit straight UP into clear air.

def _buck_terminal(sx: int) -> trimesh.Trimesh:
    """sx = -1 -> -X terminal (5 V out); sx = +1 -> +X terminal (12 V in)."""
    cx = sx * (_BUCK_HALF_W - 6.0)
    cy = -14.0
    term_top_z = _BUCK_BOARD_BASE_Z + _PCB_T + 9.0   # screw-terminal top
    return _aabb_box(
        x_lo=cx - 10.0 / 2.0,     x_hi=cx + 10.0 / 2.0,
        y_lo=cy - 9.0 / 2.0,      y_hi=cy + 9.0 / 2.0,
        z_lo=term_top_z,          z_hi=term_top_z + 14.0,
    )


# ---------------------------------------------------------------------------
# Public registry
# ---------------------------------------------------------------------------

_CABLE_KEEPOUTS_TABLE: tuple[tuple[str, str, Callable[[], trimesh.Trimesh]], ...] = (
    ("UnoQ",         "USB-C power",  _uno_usb_c),
    ("UnoQ",         "servo-bus hdr", _uno_bus_header),
    ("Buck",         "12V in",       lambda: _buck_terminal(+1)),
    ("Buck",         "5V out",       lambda: _buck_terminal(-1)),
)


def build_cable_keepouts() -> list[CableKeepout]:
    """Return one ``CableKeepout`` per modelled board connector.

    Each volume is a fresh ``trimesh.Trimesh`` (callers can mutate).
    """
    return [
        CableKeepout(part_name=part_name,
                     connector_name=connector_name,
                     mesh=factory())
        for (part_name, connector_name, factory) in _CABLE_KEEPOUTS_TABLE
    ]


def _self_test_summary() -> str:
    out = ["Cable-clearance keepout self-test:"]
    total_vol = 0.0
    for ko in build_cable_keepouts():
        v = float(ko.mesh.volume)
        b = ko.mesh.bounds
        out.append(
            f"  {ko.part_name:<14s} {ko.connector_name:<14s}  "
            f"vol = {v:8.1f} mm^3   "
            f"bounds x [{b[0,0]:+6.1f},{b[1,0]:+6.1f}]   "
            f"y [{b[0,1]:+6.1f},{b[1,1]:+6.1f}]   "
            f"z [{b[0,2]:+6.1f},{b[1,2]:+6.1f}]"
        )
        total_vol += v
    out.append(f"  ---")
    out.append(f"  {len(_CABLE_KEEPOUTS_TABLE)} keepouts, total volume "
               f"{total_vol/1000.0:.1f} cm^3")
    return "\n".join(out)


if __name__ == "__main__":
    print(_self_test_summary())
