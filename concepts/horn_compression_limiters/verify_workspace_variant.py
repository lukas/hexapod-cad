"""Coarse ROM collision gate for the configured compression-limiter variant.

The production verifier builds production parts by name, so this small adapter
replaces only its three leg templates with the sidecar variants, then reuses
the real chassis, clamp-cap, kinematics, overlap tolerances, and workspace
checker.  It does not alter production CAD or move hardware.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import _verify_prototype as verify  # noqa: E402
import hexapod_prototype as hp  # noqa: E402
import make_horn_compression_limiter_concept as concept  # noqa: E402


def _variant_parts():
    if concept.SPACER_ENABLED:
        femur_part = concept._difference(
            concept._add_yoke_head_bosses(hp.make_femur_link_part()),
            concept._yoke_limiter_cuts(),
        )
        tibia_yoke = concept._difference(
            concept._add_yoke_head_bosses(hp.make_tibia_knee_yoke()),
            concept._yoke_limiter_cuts(),
        )
        coxa = concept._difference(
            concept._add_yaw_limiter_nub(hp.make_coxa_link_part()),
            [*concept._yaw_limiter_cuts(), *concept._yaw_head_access_cuts()],
        )
    else:
        femur_part = hp.make_femur_link_part()
        tibia_yoke = hp.make_tibia_knee_yoke()
        coxa = hp.make_coxa_link_part()

    def make_femur():
        mesh = femur_part.copy()
        mesh.apply_transform(
            hp._joint_place(
                (0.0, 0.0, 0.0), (1, 0, 0), hp.LEG_PITCH_AXIS
            )
        )
        return mesh

    def make_tibia():
        transform = hp._joint_place(
            (0.0, 0.0, 0.0), (1, 0, 0), hp.LEG_PITCH_AXIS
        )
        yoke = tibia_yoke.copy()
        yoke.apply_transform(transform)
        socket = (
            transform
            @ np.array([hp._YOKE_SOCKET_X, 0.0, hp.JOINT_SOCKET_Z, 1.0])
        )[:3]
        tube_end = socket + np.array(
            [hp.TIBIA_LENGTH - hp.FOOT_BOOT_TIP_L, 0.0, 0.0]
        )
        boot = hp.make_foot_boot()
        boot.apply_transform(
            hp._frame(tube_end, (1, 0, 0), (0, 0, 1))
        )
        return hp._union(
            yoke,
            hp._tube_between(socket, tube_end, hp.LEG_TUBE_OD / 2.0),
            boot,
        )

    return coxa, make_femur, make_tibia


def main() -> int:
    coxa, make_femur, make_tibia = _variant_parts()
    verify._MESH_BUILDERS["coxa_link"] = lambda: coxa.copy()
    verify._MESH_BUILDERS["femur_link"] = make_femur
    verify._MESH_BUILDERS["tibia_link"] = make_tibia
    verify._MESH_CACHE.clear()
    verify._WS_WORKER_STATE.clear()

    samples = {"yaw": 3, "femur": 5, "knee": 5}
    passed = verify.check_workspace_self_collision(
        n_yaw=samples["yaw"],
        n_femur=samples["femur"],
        n_knee=samples["knee"],
        verbose=False,
        pool=None,
    )
    report = {
        "variant": (
            (
                f"{concept.SPACER_LENGTH:g} mm long x "
                f"{concept.SPACER_OD:g} mm OD horn compression spacers"
            )
            if concept.SPACER_ENABLED
            else "no compression spacer; production horn geometry"
        ),
        "samples": samples,
        "posesChecked": samples["yaw"] * samples["femur"] * samples["knee"] + 1,
        "pass": bool(passed),
    }
    (HERE / "workspace_report.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    print(json.dumps(report, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
