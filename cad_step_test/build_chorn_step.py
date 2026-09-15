#!/usr/bin/env python3
"""Export STEP-first parts for the optional stock C-horn variant.

This is an additive sidecar for ``tools/make_chorn_variant.py``. It imports the
same assumed/measured C-horn constants, rebuilds the printable adapters as
OpenCascade BREP solids, then derives STL as the final slicer format.
"""

from __future__ import annotations

import argparse
import json
import math
import sys

import numpy as np
import trimesh
from build123d import (
    BuildPart,
    BuildSketch,
    Plane,
    Pos,
    RegularPolygon,
    extrude,
)

from step_common import (
    OUT_DIR,
    PROTO_DIR,
    THIS_DIR,
    StepPart,
    export_all,
    write_bundle,
)

sys.path.insert(0, str(THIS_DIR))
sys.path.insert(0, str(PROTO_DIR))
sys.path.insert(0, str(PROTO_DIR / "tools"))

import build_step_first_test as step  # noqa: E402
import hexapod_prototype as hp  # noqa: E402
import make_chorn_variant as ch  # noqa: E402


def _web_hole_centres() -> list[tuple[float, float]]:
    zc = hp.JOINT_SOCKET_Z
    return [
        (sy * ch.CHORN_WEB_HOLE_DY, zc + sz * ch.CHORN_WEB_HOLE_DZ)
        for sy in (+1.0, -1.0)
        for sz in (+1.0, -1.0)
    ]


def _hex_prism_x(radius: float,
                 length: float,
                 center: tuple[float, float, float]) -> object:
    """Six-sided prism extruded along X, used for M3 nyloc pockets."""
    with BuildPart() as pocket:
        with BuildSketch(Plane.YZ):
            RegularPolygon(radius=radius, side_count=6)
        extrude(amount=length)
    return Pos(center[0] - length / 2.0, center[1], center[2]) * pocket.part


def _rounded_plate(z0: float, z1: float) -> object:
    """One stock C-horn side plate: rounded disc end plus straight web run."""
    t = z1 - z0
    zc = 0.5 * (z0 + z1)
    end = step._cyl_z(ch.CHORN_LEG_R0, t, (ch.AXIS_X, 0.0, zc))
    run = step._box(
        (ch.CHORN_WEB_X0 + ch.CHORN_WEB_T - ch.AXIS_X, ch.CHORN_WIDTH, t),
        (
            0.5 * (ch.AXIS_X + ch.CHORN_WEB_X0 + ch.CHORN_WEB_T),
            0.0,
            zc,
        ),
    )
    cuts = [
        step._cyl_z(
            ch.CHORN_PLATE_CENTER_HOLE_D / 2.0,
            4.0 * t,
            (ch.AXIS_X, 0.0, zc),
        )
    ]
    for hx, hy in step._disc_horn_bolt_centres():
        cuts.append(
            step._cyl_z(hp.DISC_HORN_BOLT_OD / 2.0, 4.0 * t, (hx, hy, zc))
        )
    return step._diff(step._union(end, run), *cuts)


def make_chorn_reference() -> object:
    """Assumed stock aluminum C horn reference, not a printable part."""
    top = _rounded_plate(ch.Z_TOP_PLATE0, ch.Z_TOP_PLATE1)
    bot = _rounded_plate(ch.Z_BOT_PLATE0, ch.Z_BOT_PLATE1)
    web = step._box(
        (ch.CHORN_WEB_T, ch.CHORN_WIDTH, ch.Z_TOP_PLATE1 - ch.Z_BOT_PLATE0),
        (
            ch.CHORN_WEB_X0 + ch.CHORN_WEB_T / 2.0,
            0.0,
            0.5 * (ch.Z_BOT_PLATE0 + ch.Z_TOP_PLATE1),
        ),
    )
    cuts = [
        step._cyl_x(
            ch.WEB_BOLT_OD / 2.0,
            4.0 * ch.CHORN_WEB_T,
            (ch.CHORN_WEB_X0 + ch.CHORN_WEB_T / 2.0, wy, wz),
        )
        for wy, wz in _web_hole_centres()
    ]
    return step._diff(step._union(top, bot, web), *cuts)


def make_spacers() -> object:
    """Eight C-horn standoff spacers in joint-local placement."""
    parts = []
    for hx, hy in step._disc_horn_bolt_centres():
        for z0, t in (
            (ch.DISC_TOP_FACE_Z, ch.SPACER_TOP_T),
            (ch.DISC_BOT_FACE_Z - ch.SPACER_BOT_T, ch.SPACER_BOT_T),
        ):
            zc = z0 + t / 2.0
            parts.append(
                step._diff(
                    step._cyl_z(ch.SPACER_OD / 2.0, t, (hx, hy, zc)),
                    step._cyl_z(ch.WEB_BOLT_OD / 2.0, 4.0 * t, (hx, hy, zc)),
                )
            )
    return step._union(*parts)


def _web_flange(*, keepout_cuts: tuple[object, ...] = ()) -> object:
    """Printed adapter flange with four clearance holes and hex nut pockets."""
    body = step._box(
        (ch.FLANGE_T, ch.CHORN_WIDTH, ch.Z_TOP_PLATE1 - ch.Z_BOT_PLATE0),
        (
            0.5 * (ch.FLANGE_X0 + ch.FLANGE_X1),
            0.0,
            0.5 * (ch.Z_BOT_PLATE0 + ch.Z_TOP_PLATE1),
        ),
    )
    cuts = list(keepout_cuts)
    for wy, wz in _web_hole_centres():
        cuts.append(
            step._cyl_x(
                ch.WEB_BOLT_OD / 2.0,
                4.0 * ch.FLANGE_T,
                (0.5 * (ch.FLANGE_X0 + ch.FLANGE_X1), wy, wz),
            )
        )
        cuts.append(
            _hex_prism_x(
                ch.NUT_POCKET_AF / math.sqrt(3.0),
                ch.NUT_POCKET_DEPTH,
                (ch.FLANGE_X1 - ch.NUT_POCKET_DEPTH / 2.0, wy, wz),
            )
        )
    return step._diff(body, *cuts)


def _spar_with_cones(x0: float, x1_wall: float) -> object:
    """Solid 18 mm femur spar with the same cone flare idea as production."""
    bite = hp._FEMUR_SPAR_WALL_BITE
    length = (x1_wall - x0) + bite
    spar = step._cyl_x(
        hp.FEMUR_SPAR_OD / 2.0,
        length,
        (x0 + length / 2.0, 0.0, hp.JOINT_SOCKET_Z),
    )
    cone_h = (x1_wall - x0) + bite
    gussets = [
        step._cone_x_from_base(
            hp.FEMUR_GUSSET_R_KNEE,
            cone_h,
            base_x=base_x,
            y=0.0,
            z=hp.JOINT_SOCKET_Z,
            direction=direction,
        )
        for base_x, direction in ((x0 - bite, +1), (x1_wall + bite, -1))
    ]
    return step._union(spar, *gussets)


def make_femur_chorn_body() -> object:
    """Printed femur adapter/body for the bought C-horn variant."""
    wall_face = hp._YOKE_SOCKET_X + hp.FEMUR_SPAR_LEN
    flange = _web_flange()
    spar = _spar_with_cones(ch.FLANGE_X0, wall_face)
    knee = step.Pos(hp.FEMUR_LENGTH, 0.0, 0.0) * step._femur_knee_fixed_solid()
    return step._union(flange, spar, knee)


def make_tibia_chorn_socket() -> object:
    """Printed tibia adapter with flange plus unchanged tube-socket mouth."""
    mouth_x = hp._YOKE_SOCKET_X + 20.0
    boss_len = mouth_x - ch.FLANGE_X0
    boss = step._cyl_x(
        hp.LEG_TUBE_OD / 2.0 + hp.LEG_TUBE_SOCKET_WALL,
        boss_len,
        (ch.FLANGE_X0 + boss_len / 2.0, 0.0, hp.JOINT_SOCKET_Z),
    )
    flare = step._cone_x_from_base(
        hp.FEMUR_GUSSET_R_KNEE,
        boss_len + 1.0,
        base_x=ch.FLANGE_X0 - 1.0,
        y=0.0,
        z=hp.JOINT_SOCKET_Z,
        direction=+1,
    )
    bore_depth = hp.LEG_TUBE_SOCKET_DEPTH
    bore = step._cyl_x(
        hp.LEG_TUBE_OD / 2.0 + hp.LEG_TUBE_SOCKET_CLEAR,
        bore_depth + 0.5,
        (mouth_x - (bore_depth + 0.5) / 2.0 + 0.5, 0.0, hp.JOINT_SOCKET_Z),
    )
    return step._diff(step._union(_web_flange(), boss, flare), bore)


def chorn_part_specs() -> list[StepPart]:
    base = PROTO_DIR / "extra_stl" / "chorn"
    return [
        StepPart(
            "chorn_reference_DO_NOT_PRINT",
            make_chorn_reference,
            base / "chorn_reference_DO_NOT_PRINT.stl",
            "Assumed bought C-horn geometry for fit/reference only.",
            printable=False,
        ),
        StepPart(
            "spacers",
            make_spacers,
            base / "spacers.stl",
            "Eight per-joint M3 standoff spacers derived from the horn span.",
        ),
        StepPart(
            "femur_chorn_body",
            make_femur_chorn_body,
            base / "femur_chorn_body.stl",
            "Printed femur body that bolts to the bought C-horn web.",
        ),
        StepPart(
            "tibia_chorn_socket",
            make_tibia_chorn_socket,
            base / "tibia_chorn_socket.stl",
            "Printed tibia adapter/socket that bolts to the bought C-horn web.",
        ),
    ]


def _static_checks() -> list[str]:
    problems = []
    if ch.SPACER_TOP_T < 3.6:
        problems.append(
            f"top spacer {ch.SPACER_TOP_T:.2f} mm is below the 3.6 mm "
            "clearance target"
        )
    if ch.SPACER_BOT_T < 0.0:
        problems.append(
            f"C-horn span {ch.CHORN_SPAN:.2f} mm is too small; needs at least "
            f"{ch.DISC_TO_DISC_SPAN + ch.SPACER_TOP_T:.2f} mm"
        )
    min_web_x = ch.AXIS_X + ch.CAP_SWEEP_R + 0.4
    if ch.CHORN_WEB_X0 < min_web_x:
        problems.append(
            f"web inner face x={ch.CHORN_WEB_X0:.2f} mm enters cap sweep; "
            f"needs >= {min_web_x:.2f} mm"
        )
    return problems


def _clearance_checks_from_stl(rows: list[dict]) -> list[str]:
    meshes = []
    for row in rows:
        mesh = trimesh.load(OUT_DIR / row["stl"], process=True)
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(
                [g for g in mesh.geometry.values() if len(g.faces) > 0]
            )
        if len(mesh.faces) > 0:
            meshes.append(mesh)
    if not meshes:
        return ["no derived STL geometry available for C-horn clearance checks"]

    moving = trimesh.util.concatenate(meshes)
    v = moving.vertices
    r = np.sqrt((v[:, 0] - ch.AXIS_X) ** 2 + v[:, 1] ** 2)
    in_cap = (v[:, 2] > ch.CAP_SWEEP_Z[0] - 0.25) & (v[:, 2] < ch.CAP_SWEEP_Z[1])
    on_disc = r <= hp.DISC_HORN_OD / 2.0 + 2.1
    bad = in_cap & ~on_disc & (r < ch.CAP_SWEEP_R + 0.4)

    problems = []
    if np.any(bad):
        i = int(np.argmin(np.where(bad, r, np.inf)))
        problems.append(
            f"cap-sweep clearance: {int(bad.sum())} STL vertices inside "
            f"r {ch.CAP_SWEEP_R + 0.4:.1f} mm; worst r={r[i]:.2f}, z={v[i, 2]:.2f}"
        )
    in_deck = (v[:, 2] > ch.DECK_SWEEP_Z[0]) & (v[:, 2] < ch.DECK_SWEEP_Z[1])
    bad_deck = in_deck & (r > ch.DECK_SWEEP_R[0]) & (r < ch.DECK_SWEEP_R[1])
    if np.any(bad_deck):
        problems.append(
            f"coxa-deck band: {int(bad_deck.sum())} STL vertices sweep deck annulus "
            f"r {ch.DECK_SWEEP_R} at z {ch.DECK_SWEEP_Z}"
        )
    return problems


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()

    exported = export_all(chorn_part_specs())
    problems = _static_checks() + _clearance_checks_from_stl(exported)
    manifest = {
        "units": "mm",
        "source": (
            "build123d/OpenCascade BREP, C-horn constants imported from "
            "tools/make_chorn_variant.py"
        ),
        "source_constants": {
            "chorn_plate_t": ch.CHORN_PLATE_T,
            "chorn_span": ch.CHORN_SPAN,
            "chorn_web_x0": ch.CHORN_WEB_X0,
            "spacer_top_t": ch.SPACER_TOP_T,
            "spacer_bot_t": ch.SPACER_BOT_T,
            "disc_to_disc_span": ch.DISC_TO_DISC_SPAN,
        },
        "exported_parts": exported,
        "checks": {
            "passed": not problems,
            "problems": problems,
        },
        "files": [rel for row in exported for rel in (row["step"], row["stl"])],
    }
    manifest_path = OUT_DIR / "chorn_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    bundle = write_bundle(manifest, "chorn_step_first_bundle.zip",
                          "chorn_manifest.json")
    print(f"wrote {manifest_path.relative_to(THIS_DIR)}")
    print(f"wrote {bundle.relative_to(THIS_DIR)}")
    if problems:
        print("C-horn STEP checks failed:")
        for problem in problems:
            print(f"  - {problem}")
        raise SystemExit(1)
    print("C-horn STEP sidecar complete; existing mesh generator was not modified.")


if __name__ == "__main__":
    main()
