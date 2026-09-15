#!/usr/bin/env python3
"""Build a standalone BuildViz scene for the STS3215 bearing-sandwich joint.

This is a *visualization* assembly (not a printed part) that shows how the
chassis-yaw joint goes together with the dual-sided "sandwich" support the
design review recommended:

    fixed printed HOUSING  (sts3215_testfit cradle)
      front M2 self-tap plate  ->  bolts to the SERVO case
      driven DISC HORN on the spline (output side)
      partial back housing press-fits a 688 BALL BEARING (passive side)
    moving YOKE straddles the servo
      top arm  bolts to the disc horn      (driven)
      bottom arm stub rides the 688 bore   (passive)
      Phi 8 CARBON TUBE socketed outboard  (leg segment)

Every mesh is authored in the SAME world frame as ``sts3215_testfit.py``:
    origin = centre of the servo BACK (idler) face
    +X     = body long axis (output shaft offset toward +X, at OUTPUT_X)
    +Z     = output-shaft direction (out of the FRONT face)
so each scene instance uses the identity transform and BuildViz draws the
parts already meshed together.

Run:
    python tools/joint_viz_build.py
    # then publish into the ONE machine-wide hub (port 5183) -- never a
    # per-project dev server on 5174/etc:
    npx buildviz register joint_viz --build-id prototype_sts3215_joint_viz
    open http://127.0.0.1:5183/?build=prototype_sts3215_joint_viz
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent

import hexapod_prototype as HP
import sts3215_testfit as TF  # noqa: E402  (servo + cradle + yoke geometry)


OUT_DIR = _HERE.parent / "joint_viz"
STL_DIR = OUT_DIR / "stl"


REF_SERVO_STL = _HERE.parent / "reference_soarm" / "STS3215_03a.stl"


def _servo_body() -> "HP.trimesh.Trimesh":
    """The REAL FEETECH STS3215 (official SO-ARM STEP export).

    The reference mesh's native frame already matches ours: output spline
    axis at (x=12.5, y=0) on +Z, back (idler) face at the -Z extreme.  We
    only shift +Z so the back face lands on z=0 (origin = back-face centre),
    which seats the front/output face against the housing plate and puts the
    spline up through the bore into the disc horn -- no rotation needed."""
    m = HP.trimesh.load(REF_SERVO_STL, process=False)
    if isinstance(m, HP.trimesh.Scene):
        m = HP.trimesh.util.concatenate(list(m.geometry.values()))
    m.apply_translation([0.0, 0.0, -float(m.bounds[0][2])])  # back face -> z=0
    return m


def _disc_horn() -> "HP.trimesh.Trimesh":
    """Driven aluminium disc horn on the output spline (Phi 20 x 5)."""
    front_top = TF.BODY_H + TF.PLATE_T              # 39
    disc = HP._cyl(TF.DISC_HORN_OD / 2.0, TF.DISC_HORN_H)
    disc.apply_translation([TF.OUTPUT_X, 0.0, front_top + TF.DISC_HORN_H / 2.0])
    # Spline bore + 4x M3 link bolt holes (so the bolt circle reads).
    cuts = [HP._cyl(TF.SPLINE_OD / 2.0 + 0.2, TF.DISC_HORN_H * 3)]
    cuts[0].apply_translation([TF.OUTPUT_X, 0.0, front_top + TF.DISC_HORN_H / 2.0])
    for (hx, hy) in TF.horn_bolt_centres():
        h = HP._cyl(TF.DISC_HORN_BOLT_OD / 2.0, TF.DISC_HORN_H * 3)
        h.apply_translation([hx, hy, front_top + TF.DISC_HORN_H / 2.0])
        cuts.append(h)
    return HP._diff(disc, *cuts)


def _bearing() -> "HP.trimesh.Trimesh":
    """688-2RS ball bearing (Phi 16 OD x Phi 8 bore x 5 W) in the back pocket.

    Pocket opens on the housing outer face (z = -BACK_PLATE_T) and is
    BEARING_W deep, so the bearing sits at z in [-BACK_PLATE_T, -BACK_PLATE_T
    + BEARING_W]."""
    z0 = -TF.BACK_PLATE_T
    z_mid = z0 + TF.BEARING_W / 2.0
    outer = HP._cyl(TF.BEARING_OD / 2.0, TF.BEARING_W)
    outer.apply_translation([TF.OUTPUT_X, 0.0, z_mid])
    bore = HP._cyl(TF.BEARING_BORE / 2.0, TF.BEARING_W * 3)
    bore.apply_translation([TF.OUTPUT_X, 0.0, z_mid])
    return HP._diff(outer, bore)


def _carbon_tube() -> "HP.trimesh.Trimesh":
    """Phi 8 carbon-fibre leg segment socketed into the yoke and extending +X."""
    # Yoke socket sits at sock_z; tube runs along X out of the spine.
    bot_z0 = (-TF.BACK_PLATE_T - 1.0) - 4.0          # mirrors make_testfit_yoke
    top_z1 = (TF.BODY_H + TF.PLATE_T) + TF.DISC_HORN_H + 4.0
    sock_z = 0.5 * (bot_z0 + top_z1)
    spine_x1 = 32.0
    x0 = spine_x1 - 4.0                               # a little inside the socket
    length = 120.0
    tube = HP._cyl(TF.LEG_TUBE_OD / 2.0, length)
    tube.apply_transform(HP.rotation_matrix(np.pi / 2.0, [0, 1, 0]))  # Z -> X
    tube.apply_translation([x0 + length / 2.0, 0.0, sock_z])
    return tube


# (mesh_id, filename, builder, color, role, focus group)
# Bought (COTS) reference meshes get the loud _DO_NOT_PRINT suffix; only the
# printed testfit parts (housing, yoke) keep a plain printable filename.
PARTS = [
    ("servo_body", "servo_body_DO_NOT_PRINT.stl", _servo_body, "#2e2e33",
     "STS3215 servo (real FEETECH geometry, fixed)", "joint"),
    ("housing", "housing.stl", lambda: TF.make_testfit_cradle(back_bearing=True),
     "#ff7f0e", "printed housing: front M2 plate + back bearing pocket", "joint"),
    ("disc_horn", "disc_horn_DO_NOT_PRINT.stl", _disc_horn, "#b6b6bd",
     "driven Phi20 disc horn (output side)", "joint"),
    ("bearing_688", "bearing_688_DO_NOT_PRINT.stl", _bearing, "#5b6b7a",
     "688-2RS ball bearing (passive side)", "joint"),
    ("yoke", "yoke.stl", TF.make_testfit_yoke, "#2ca02c",
     "moving yoke: top->horn, bottom stub->bearing", "joint"),
    ("carbon_tube", "carbon_tube_DO_NOT_PRINT.stl", _carbon_tube, "#1a1a1a",
     "Phi 8 carbon-fibre leg segment", "joint"),
]


def _identity_colmajor() -> list[float]:
    return [1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0]


def main() -> None:
    STL_DIR.mkdir(parents=True, exist_ok=True)

    meshes = []
    instances = []
    all_pts = []
    for idx, (pid, fname, builder, color, role, group) in enumerate(PARTS):
        mesh = builder()
        mesh.export(STL_DIR / fname)
        all_pts.append(mesh.bounds)
        cen = [float(v) for v in mesh.centroid]
        meshes.append({
            "id": f"stl:{pid}",
            "name": fname,
            "url": f"stl/{fname}",
        })
        instances.append({
            "id": f"{idx:03d}-{pid}",
            "meshId": f"stl:{pid}",
            "name": f"{pid}  {role}",
            "partType": pid,
            "role": role,
            "leg": None,
            "joint": "yaw",
            "color": color,
            "transform": _identity_colmajor(),
            "centroid": cen,
            "focusGroup": group,
        })
        b = mesh.bounds
        print(f"  {pid:12s} watertight={mesh.is_watertight!s:5s} "
              f"tris={len(mesh.faces):6,d}  "
              f"X {b[0][0]:7.1f}..{b[1][0]:6.1f}  "
              f"Z {b[0][2]:7.1f}..{b[1][2]:6.1f}")

    bounds = np.array(all_pts)  # (n, 2, 3)
    lo = bounds[:, 0, :].min(axis=0)
    hi = bounds[:, 1, :].max(axis=0)
    center = [float(v) for v in (lo + hi) / 2.0]

    scene = {
        "name": "STS3215 yaw-joint bearing sandwich",
        "source": str(OUT_DIR),
        "designSpecUrl": "design_spec.yaml",
        "units": "mm",
        "center": center,
        "meshes": meshes,
        "instances": instances,
    }
    (OUT_DIR / "scene.json").write_text(json.dumps(scene, indent=2))
    print(f"\nWrote {OUT_DIR/'scene.json'} ({len(instances)} instances)")
    rel = OUT_DIR.relative_to(Path.cwd()) if OUT_DIR.is_relative_to(Path.cwd()) else OUT_DIR
    print(f"View:  npx buildviz register {rel} --build-id prototype_sts3215_joint_viz")
    print("       open http://127.0.0.1:5183/?build=prototype_sts3215_joint_viz")


if __name__ == "__main__":
    main()
