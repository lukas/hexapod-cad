#!/usr/bin/env python3
"""BREP source for the experimental two-piece CNC C-clamp.

The existing ``cnc_chorn_overhead`` robot is inherited unchanged except for
its one-piece aluminum clamp.  That clamp becomes:

* ``chorn_drive_frame_cnc`` -- driven-side horn plate plus the full web;
* ``chorn_passive_plate_cnc`` -- passive-side horn plate plus a short keyed
  web foot.

The two horn plates can be clocked and bolted to their discs one face at a
time.  A broad tongue locates the passive plate in the drive-frame web and
  two low-head M2.5 screws, reached from the passive side, clamp the split
  together.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROTO_DIR = HERE.parent.parent
BASE_DIR = HERE.parent / "cnc_chorn_overhead"
CAD_STEP_DIR = PROTO_DIR / "cad_step_test"
for path in (CAD_STEP_DIR, PROTO_DIR, BASE_DIR, HERE):
    sys.path.insert(0, str(path))

from step_common import StepPart, export_all, write_bundle  # noqa: E402

import build_step_first_test as step  # noqa: E402
import build_cnc_chorn_step as base_step  # noqa: E402
import hexapod_prototype as hp  # noqa: E402
import make_cnc_chorn_variant as cv  # noqa: E402
import make_two_piece_chorn_variant as tp  # noqa: E402

STEP_OUT_DIR = HERE / "step"


def _box_span(x0: float, x1: float, y0: float, y1: float,
              z0: float, z1: float) -> object:
    return step._box(
        (x1 - x0, y1 - y0, z1 - z0),
        ((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0),
    )


def _disc_cuts(z0: float, z1: float, mating_face_z: float,
               mating_side: int) -> list[object]:
    """Bolt, centre and collar-recess cuts for one horn plate.

    ``mating_side`` is +1 when material lies above the horn face (driven
    plate) and -1 when it lies below the horn face (passive plate).
    """
    cuts = []
    for hx, hy in step._disc_horn_bolt_centres():
        cuts.append(base_step._cyl_z(
            hp.DISC_HORN_BOLT_OD / 2.0, z0 - 1.0, z1 + 1.0,
            x=hx, y=hy,
        ))
    cuts.append(base_step._cyl_z(
        hp.HORN_CENTRE_OD / 2.0, z0 - 1.0, z1 + 1.0, x=cv.AXIS_X,
    ))
    rr = hp.DISC_HORN_COLLAR_OD / 2.0 + 0.25
    rd = hp.DISC_HORN_COLLAR_DEPTH + 1.0
    if mating_side > 0:
        cuts.append(base_step._cyl_z(
            rr, mating_face_z - 0.5, mating_face_z + rd, x=cv.AXIS_X,
        ))
    else:
        cuts.append(base_step._cyl_z(
            rr, mating_face_z - rd, mating_face_z + 0.5, x=cv.AXIS_X,
        ))
    return cuts


def _drive_web_holes() -> list[object]:
    """The unchanged printed-link web joint, cut into the drive frame."""
    cuts = []
    for y, z in cv.WEB_A_YZ:
        x0 = tp.JOIN_X0 - 1.0 if z < tp.JOIN_BOSS_Z1 else cv.WEB_X0 - 1.0
        cuts.append(base_step._cyl_x(
            cv.WEB_TAP_D / 2.0, x0, cv.WEB_X1 + 1.0, y=y, z=z,
        ))
    for y, z in cv.WEB_B_YZ:
        cuts.append(base_step._cyl_x(
            cv.WEB_B_D / 2.0, cv.WEB_X0 - 1.0, cv.WEB_X1 + 1.0,
            y=y, z=z,
        ))
        r_csk = cv.WEB_CSK_D / 2.0 + 0.1
        cuts.append(step._cone_x_from_base(
            r_csk, r_csk, base_x=cv.WEB_X0 - 0.1,
            y=y, z=z, direction=1,
        ))
    return cuts


def make_chorn_drive_frame_cnc() -> object:
    """Driven horn plate + web, the first half installed on a joint."""
    main_web = _box_span(
        cv.WEB_X0, cv.WEB_X1, cv.WEB_Y_BOT, cv.BLADE_W_UP,
        tp.JOIN_PLANE_Z, cv.ARM_TOP_Z1,
    )
    # Extra width exists only at the lower split, providing real edge
    # distance around the two vertical M2.5 join threads and tongue pocket.
    join_boss = _box_span(
        tp.JOIN_X0, cv.WEB_X1, cv.WEB_Y_BOT, cv.BLADE_W_UP,
        tp.JOIN_PLANE_Z, tp.JOIN_BOSS_Z1,
    )
    body = step._union(
        base_step._blade(cv.ARM_TOP_Z0),
        base_step._cyl_z(
            cv.PAD_OD / 2.0, cv.DISC_TOP_FACE_Z, cv.ARM_TOP_Z0,
            x=cv.AXIS_X,
        ),
        main_web,
        join_boss,
        base_step._gusset(cv.ARM_TOP_Z0, -1),
    )
    cuts = _disc_cuts(
        cv.DISC_TOP_FACE_Z, cv.ARM_TOP_Z1,
        cv.DISC_TOP_FACE_Z, +1,
    )
    cuts += _drive_web_holes()
    # Female tongue pocket.  It opens at the split plane and has 0.15 mm
    # clearance per wall around the passive plate's broad locating tongue.
    cuts.append(_box_span(
        tp.TONGUE_X0 - tp.TONGUE_CLEAR,
        tp.TONGUE_X1 + tp.TONGUE_CLEAR,
        tp.TONGUE_Y0 - tp.TONGUE_CLEAR,
        tp.TONGUE_Y1 + tp.TONGUE_CLEAR,
        tp.JOIN_PLANE_Z - 0.2,
        tp.JOIN_PLANE_Z + tp.TONGUE_H + 0.2,
    ))
    # Two M2.5-6H vertical threads.  They are separated from the tongue so
    # their clamp preload does not bear on a thin pocket wall.
    for y in tp.JOIN_SCREW_YS:
        cuts.append(base_step._cyl_z(
            tp.JOIN_TAP_D / 2.0,
            tp.JOIN_PLANE_Z - 0.5,
            tp.JOIN_THREAD_Z1,
            x=tp.JOIN_SCREW_X, y=y,
        ))
    return step._diff(body, *cuts)


def make_chorn_passive_plate_cnc() -> object:
    """Passive horn plate + keyed web foot, installed from the idler side."""
    web_foot = _box_span(
        tp.JOIN_X0, tp.PASSIVE_FOOT_X1, cv.WEB_Y_BOT, cv.BLADE_W_UP,
        cv.ARM_BOT_Z0, tp.JOIN_PLANE_Z,
    )
    tongue = _box_span(
        tp.TONGUE_X0, tp.TONGUE_X1, tp.TONGUE_Y0, tp.TONGUE_Y1,
        tp.JOIN_PLANE_Z, tp.JOIN_PLANE_Z + tp.TONGUE_H,
    )
    body = step._union(
        base_step._blade(cv.ARM_BOT_Z0),
        base_step._cyl_z(
            cv.PAD_OD / 2.0, cv.ARM_BOT_Z1, cv.DISC_BOT_FACE_Z,
            x=cv.AXIS_X,
        ),
        web_foot,
        tongue,
    )
    cuts = _disc_cuts(
        cv.ARM_BOT_Z0, cv.DISC_BOT_FACE_Z,
        cv.DISC_BOT_FACE_Z, -1,
    )
    for y in tp.JOIN_SCREW_YS:
        cuts.append(base_step._cyl_z(
            tp.JOIN_CLEAR_D / 2.0,
            cv.ARM_BOT_Z0 - 1.0,
            tp.JOIN_PLANE_Z + 0.6,
            x=tp.JOIN_SCREW_X, y=y,
        ))
        cuts.append(base_step._cyl_z(
            tp.JOIN_HEAD_CB_D / 2.0,
            cv.ARM_BOT_Z0 - 1.0,
            cv.ARM_BOT_Z0 + tp.JOIN_HEAD_CB_DEPTH,
            x=tp.JOIN_SCREW_X, y=y,
        ))
    return step._diff(body, *cuts)


def part_specs() -> list[StepPart]:
    return [
        StepPart(
            "chorn_drive_frame_cnc",
            make_chorn_drive_frame_cnc,
            None,
            "CNC 6061 driven-side horn plate plus keyed web frame (12x).",
            printable=False,
        ),
        StepPart(
            "chorn_passive_plate_cnc",
            make_chorn_passive_plate_cnc,
            None,
            "CNC 6061 removable passive horn plate with locating tongue (12x).",
            printable=False,
        ),
    ]


def main() -> None:
    argparse.ArgumentParser(description=__doc__).parse_args()
    exported = export_all(
        part_specs(), out_dir=STEP_OUT_DIR,
        step_dir=STEP_OUT_DIR, stl_dir=STEP_OUT_DIR / "stl",
    )
    problems = []
    import trimesh
    for row in exported:
        mesh = hp._heal_for_export(
            trimesh.load(STEP_OUT_DIR / row["stl"], process=True)
        )
        if not mesh.is_volume:
            problems.append(f"{row['name']}: STL does not heal to a volume")
    manifest = {
        "units": "mm",
        "source": "build_two_piece_chorn_step.py (build123d/OpenCascade)",
        "inherits": "concepts/cnc_chorn_overhead",
        "exported_parts": exported,
        "checks": {"passed": not problems, "problems": problems},
        "files": [rel for row in exported for rel in (row["step"], row["stl"])],
    }
    manifest_path = STEP_OUT_DIR / "two_piece_chorn_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    bundle = write_bundle(
        manifest, "two_piece_chorn_step_bundle.zip",
        "two_piece_chorn_manifest.json", out_dir=STEP_OUT_DIR,
    )
    print(f"wrote {manifest_path.relative_to(PROTO_DIR)}")
    print(f"wrote {bundle.relative_to(PROTO_DIR)}")
    if problems:
        raise SystemExit("\n".join(problems))
    print("two-piece chorn BREP export complete")


if __name__ == "__main__":
    main()
