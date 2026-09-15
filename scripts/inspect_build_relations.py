"""Sub-assembly relationship helpers for the PyVista build inspector.

The build inspector's "focus on sub-assembly" mode (``inspect_build.py``
keybinding ``F`` and double-click) needs to know, for any clicked
printable part, which OTHER inspector entities (servos, disc horns,
fasteners) physically belong to the same sub-assembly so it can hide
the rest of the robot and present the picked part unobstructed.

This module is the SINGLE SOURCE OF TRUTH for that "what belongs to
this part" mapping.  It is kept separate from ``fastener_registry``
(which describes individual fasteners) and ``inspect_build`` (which
loads STLs and drives the plotter) so the rule table is easy to
audit and re-tune in one place.

Design
------
The mapping is intentionally **role-string driven** rather than
geometric.  Every fastener already carries a human-readable ``role``
(``"coxa_link L0 hip cradle +X top SHCS"``, ``"foot L3 hinge pin
pan-head"``, ...) and every printable / servo Instance has a
``(part_type, leg_index, joint)`` triple.  We pick a small set of
substring patterns per focus part type and accept any fastener whose
``role`` contains AT LEAST ONE of those patterns.  The patterns are
specified in :func:`build_subassembly_for` itself rather than as a
data table so the conventions stay close to the code that uses them.

What we deliberately DO NOT do:

* No geometric containment / ray casting.  If a fastener's role does
  not match a pattern, it just stays out of the sub-assembly and
  becomes invisible in focus mode -- which is the safe default; the
  user can always exit focus mode (``Esc`` / ``I``) to see the
  hidden hardware.
* No per-leg per-joint special-case logic for chassis-level parts.
  The chassis plates pull in every fastener whose role contains the
  literal string ``"chassis"`` and trust the registry's naming.

The structural part of the sub-assembly (the servo dropped into a
cradle and the disc horn the part rotates with) is enumerated from the
inspector's ``all_instances`` list by matching ``(part_type,
leg_index, joint)`` exactly, NOT by role strings -- so a missing
horn / servo Instance loudly drops out instead of silently matching
the wrong leg.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

# Type aliases.  We intentionally do NOT import the concrete classes
# here so this module remains a leaf in the import graph (it would
# otherwise pull in pyvista via inspect_build).  Callers pass in their
# Instance / FastenerInstance objects and we operate via the duck-typed
# attributes documented in the spec (``part_type``, ``leg_index``,
# ``joint``, ``fastener_role``, ``role``).
Instance = Any           # inspect_build.Instance
FastenerInstance = Any   # fastener_registry.FastenerInstance


@dataclass(frozen=True)
class SubAssembly:
    """The set of inspector instances that visually belong to one printable part.

    Members:
        focus: the printable part itself (a single Instance).
        servos_inside: list of servo_body / servo_horn Instances that
            sit INSIDE this part's cradle (i.e. the cradle hosts the
            servo).
        horns_below: list of servo_horn Instances that this part's
            mating PAD bolts onto (the horn rotates WITH the part).
        fasteners: list of FastenerInstance entries whose head or
            threads pass through this part.
    """

    focus: Instance
    servos_inside: list[Instance]
    horns_below: list[Instance]
    fasteners: list[FastenerInstance]

    def all_members(self) -> list:
        """Flat list of every Instance / FastenerInstance in the sub-assembly.

        The focus part is always first; the rest are in the order
        stored on the dataclass (servos -> horns -> fasteners), which
        ``build_subassembly_for`` already sorts deterministically.
        """
        return [self.focus, *self.servos_inside, *self.horns_below, *self.fasteners]


# ---------------------------------------------------------------------------
# Pattern tables (one row per focus part type)
# ---------------------------------------------------------------------------


def _fastener_role_patterns(part_type: str, leg_index: int | None) -> list[str]:
    """Substring patterns that select fasteners belonging to this part.

    A fastener is included in the sub-assembly if its ``role`` string
    contains AT LEAST ONE of the returned substrings.  Empty list ->
    no fasteners (the part stands alone, e.g. battery_holder).
    """
    leg_tag = f"L{leg_index}" if leg_index is not None else None
    if part_type == "coxa_bracket" and leg_tag:
        return [f"coxa_bracket {leg_tag}", f"{leg_tag} yaw cradle"]
    if part_type == "coxa_link" and leg_tag:
        return [
            f"coxa_link {leg_tag}",
            f"{leg_tag} hip cradle",
            f"{leg_tag} yaw horn",
        ]
    if part_type == "femur_link" and leg_tag:
        return [
            f"femur_link {leg_tag}",
            f"{leg_tag} knee cradle",
            f"{leg_tag} hip horn",
        ]
    if part_type == "tibia_link" and leg_tag:
        return [f"tibia_link {leg_tag}", f"{leg_tag} knee horn"]
    # foot_pad relation RETIRED (Aug 2026): the TPU foot_boot has no
    # fasteners (pressed onto the tube).
    if part_type in (
        "chassis_top", "chassis_bottom",
        "chassis_plate_a", "chassis_plate_b",
    ):
        # Both chassis plates own every chassis-side fastener.  The
        # bracket-to-chassis bolts use the role ``"coxa_bracket L<i>
        # chassis bolt ..."`` so a ``"chassis"`` substring is enough.
        return ["chassis"]
    return []


def _structural_member_filter(
    part_type: str, leg_index: int | None,
) -> tuple[list[tuple[str, str]], list[tuple[str, str]]]:
    """Return (servos_inside_filters, horns_below_filters).

    Each filter is a ``(part_type, joint)`` tuple matched on the
    inspector's Instance list together with ``leg_index``.  The
    spec table:

    ====================  =====================  ======================
    focus part_type       servos_inside          horns_below
    ====================  =====================  ======================
    coxa_bracket          servo_body / yaw       (none)
    coxa_link             servo_body / hip       servo_horn / yaw
    femur_link            servo_body / knee      servo_horn / hip
    tibia_link            (none)                 servo_horn / knee
    others (chassis, ...) (none)                 (none)
    ====================  =====================  ======================
    """
    if leg_index is None:
        return [], []
    if part_type == "coxa_bracket":
        return [("servo_body", "yaw")], []
    if part_type == "coxa_link":
        return [("servo_body", "hip")], [("servo_horn", "yaw")]
    if part_type == "femur_link":
        return [("servo_body", "knee")], [("servo_horn", "hip")]
    if part_type == "tibia_link":
        return [], [("servo_horn", "knee")]
    return [], []


# ---------------------------------------------------------------------------
# Public builder
# ---------------------------------------------------------------------------


def build_subassembly_for(
    focus: Instance,
    all_instances: list[Instance],
    all_fasteners: list[FastenerInstance],
) -> SubAssembly:
    """Build the :class:`SubAssembly` rooted at ``focus``.

    ``all_instances`` is the inspector's full ``Instance`` list
    (printed + servo + per-fastener Instance wrappers).  We only walk
    the non-fastener entries for ``servos_inside`` / ``horns_below``;
    the ``fastener_role`` Instance entries are not consulted here
    because the raw ``FastenerInstance`` list is the source of truth
    for hardware membership.

    ``all_fasteners`` is the raw ``FastenerInstance`` list from
    ``fastener_registry.build_all_fastener_instances()``.

    Determinism: every list inside the returned ``SubAssembly`` is
    sorted -- structural members by ``(part_type, joint, leg_index)``,
    fasteners by ``(spec, role)`` -- so the focus-mode visibility
    computation is stable run-to-run.
    """
    pt = focus.part_type
    li = focus.leg_index

    servo_filters, horn_filters = _structural_member_filter(pt, li)
    role_patterns = _fastener_role_patterns(pt, li)

    def _match_structural(filters: list[tuple[str, str]]) -> list[Instance]:
        if not filters:
            return []
        out: list[Instance] = []
        for inst in all_instances:
            # Skip fastener-wrapper Instances; they have stl_dir set
            # to the fasteners/ directory and never belong to the
            # structural-members list (they live in .fasteners).
            if getattr(inst, "fastener_role", None):
                continue
            for want_pt, want_joint in filters:
                if (
                    inst.part_type == want_pt
                    and inst.leg_index == li
                    and inst.joint == want_joint
                ):
                    out.append(inst)
                    break
        return out

    servos_inside = _match_structural(servo_filters)
    horns_below = _match_structural(horn_filters)

    fasteners: list[FastenerInstance] = []
    if role_patterns:
        for fi in all_fasteners:
            role = getattr(fi, "role", "") or ""
            for pat in role_patterns:
                if pat in role:
                    fasteners.append(fi)
                    break

    servos_inside.sort(
        key=lambda i: (i.part_type, i.joint or "", i.leg_index or -1),
    )
    horns_below.sort(
        key=lambda i: (i.part_type, i.joint or "", i.leg_index or -1),
    )
    fasteners.sort(
        key=lambda f: (getattr(f, "spec", ""), getattr(f, "role", "")),
    )

    return SubAssembly(
        focus=focus,
        servos_inside=servos_inside,
        horns_below=horns_below,
        fasteners=fasteners,
    )


# ---------------------------------------------------------------------------
# Self-test (run via ``python inspect_build_relations.py``)
# ---------------------------------------------------------------------------


def _self_test_summary() -> str:
    """Print a tiny audit of which fastener roles each printed-part
    focus matches, so a CI grep can spot regressions in the role
    naming conventions.

    Importing inspect_build is heavy, so this self-test only loads
    the FastenerInstance list and forms ``Instance``-like
    ``SimpleNamespace`` stand-ins for the printed parts.  The
    inspector itself never hits this code path.
    """
    import os
    import sys
    from types import SimpleNamespace

    here = os.path.dirname(os.path.abspath(__file__))
    proto = os.path.dirname(here)
    for _p in (proto, here):
        if _p not in sys.path:
            sys.path.insert(0, _p)

    import fastener_registry  # noqa: E402

    all_fasteners = fastener_registry.build_all_fastener_instances()
    cases: list[SimpleNamespace] = [
        SimpleNamespace(part_type="coxa_bracket", leg_index=0, joint=None,
                        fastener_role=None),
        SimpleNamespace(part_type="coxa_link",    leg_index=0, joint=None,
                        fastener_role=None),
        SimpleNamespace(part_type="femur_link",   leg_index=3, joint=None,
                        fastener_role=None),
        SimpleNamespace(part_type="tibia_link",   leg_index=5, joint=None,
                        fastener_role=None),
        SimpleNamespace(part_type="chassis_top",  leg_index=None, joint=None,
                        fastener_role=None),
    ]
    out = ["SubAssembly self-test (per focus -> matched fastener count):"]
    for focus in cases:
        sub = build_subassembly_for(focus, [], all_fasteners)
        leg = "--" if focus.leg_index is None else f"L{focus.leg_index}"
        out.append(
            f"  {focus.part_type:14s} {leg:>4s}  "
            f"{len(sub.fasteners):3d} fasteners"
        )
    return "\n".join(out)


if __name__ == "__main__":
    print(_self_test_summary())
