"""Render 4 standard views (front / side / top / iso) of every part in
``stl_prototype/``, plus an overlay of the keep-out volumes from
``keepout_volumes.py`` so holes / corridors / clearances are visually
obvious in the report.

Run from the repo root:

    ./run.sh hexapod_walker/prototype/scripts/render_views.py

Or:

    make -C hexapod_walker/prototype render

Outputs land in ``hexapod_walker/prototype/artifacts/views/<part>/``:

    front.png   side.png   top.png   iso.png

Rendering backend
-----------------

This script tries the following backends in order and falls back if a
backend is missing or fails to render:

    1. ``pyrender`` (requires system OpenGL).
    2. ``pyvista``  (off-screen VTK; already in repo requirements).
    3. ``matplotlib`` trisurf (slow, but pure-Python; always works).

The first backend that succeeds for a given part is used.  Generated
file paths are printed (absolute) so they can be pasted into chat /
issue trackers.
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass

import numpy as np
import trimesh

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.dirname(THIS_DIR)

STL_DIR = os.path.join(PROTO_DIR, "stl_prototype")
VIEWS_DIR = os.path.join(PROTO_DIR, "artifacts", "views")

# Standard view directions (camera looks from this position toward the
# origin).  Order matters: the loop below renders them in this order.
VIEWS: dict[str, tuple[float, float, float]] = {
    "front": (0.0, -1.0, 0.0),     # camera at -Y, look toward +Y
    "side":  (1.0, 0.0, 0.0),      # camera at +X, look toward -X
    "top":   (0.0, 0.0, 1.0),      # camera at +Z, look toward -Z
    "iso":   (1.0, -1.0, 0.7),     # 3/4 isometric
}


@dataclass
class _PartRender:
    part: str
    stl_path: str
    output_paths: list[str]
    backend: str


# ---------------------------------------------------------------------------
# Camera + scene setup helpers
# ---------------------------------------------------------------------------

def _compute_camera(part_mesh: trimesh.Trimesh,
                    overlay_meshes: list[trimesh.Trimesh],
                    direction: tuple[float, float, float]) -> tuple[
                        np.ndarray, np.ndarray, float]:
    """Return ``(camera_pos, target, span)`` for a part+overlay scene.

    ``span`` is the bounding-sphere radius used to set FOV / matplotlib
    axis limits.
    """
    all_meshes = [part_mesh] + overlay_meshes
    bounds = np.vstack([m.bounds for m in all_meshes])
    bb_min = bounds[::2].min(axis=0)
    bb_max = bounds[1::2].max(axis=0)
    centre = (bb_min + bb_max) / 2.0
    span = float(np.linalg.norm(bb_max - bb_min)) / 2.0 + 5.0
    direction = np.asarray(direction, dtype=float)
    direction = direction / np.linalg.norm(direction)
    cam_pos = centre + direction * span * 2.5
    return cam_pos, centre, span


# ---------------------------------------------------------------------------
# PyVista backend (preferred; already in repo's requirements)
# ---------------------------------------------------------------------------

def _render_pyvista(part_mesh: trimesh.Trimesh,
                    overlays: dict[str, trimesh.Trimesh],
                    out_path: str,
                    direction: tuple[float, float, float],
                    size=(900, 720)) -> None:
    import pyvista as pv

    pl = pv.Plotter(off_screen=True, window_size=size,
                    lighting="three lights")
    pl.set_background("#f1f2f4")
    pv_part = pv.wrap(part_mesh)
    pl.add_mesh(pv_part, color="#c5cbd1",
                opacity=1.0, smooth_shading=False,
                show_edges=False,
                specular=0.2, specular_power=20.0)

    palette = [
        "#e67e22", "#27ae60", "#2980b9",
        "#8e44ad", "#c0392b", "#16a085",
        "#d35400", "#7f8c8d", "#34495e",
    ]
    for i, (name, ov) in enumerate(sorted(overlays.items())):
        pv_ov = pv.wrap(ov)
        pl.add_mesh(pv_ov,
                    color=palette[i % len(palette)],
                    opacity=0.35,
                    show_edges=False)

    cam_pos, target, span = _compute_camera(part_mesh,
                                              list(overlays.values()),
                                              direction)
    # Use Z up for iso / top, Y up for top-down would invert the part
    up = (0.0, 0.0, 1.0)
    if direction[2] > 0.9:
        # Looking straight down — Z is the view axis, choose Y as up
        up = (0.0, 1.0, 0.0)
    pl.camera_position = [tuple(cam_pos), tuple(target), up]
    pl.camera.parallel_projection = True
    pl.camera.parallel_scale = max(span, 5.0)
    pl.screenshot(out_path)
    pl.close()


# ---------------------------------------------------------------------------
# Pyrender backend (optional, tries first if available)
# ---------------------------------------------------------------------------

def _render_pyrender(part_mesh, overlays, out_path, direction,
                     size=(900, 720)) -> None:
    import pyrender  # noqa: WPS433
    scene = pyrender.Scene(bg_color=[0.94, 0.94, 0.96, 1.0],
                            ambient_light=[0.3, 0.3, 0.3])
    pm = pyrender.Mesh.from_trimesh(part_mesh, smooth=False)
    scene.add(pm)
    for ov in overlays.values():
        ov_mesh = ov.copy()
        ov_mesh.visual.face_colors = [220, 110, 50, 90]
        scene.add(pyrender.Mesh.from_trimesh(ov_mesh, smooth=False))

    cam_pos, target, span = _compute_camera(part_mesh,
                                             list(overlays.values()),
                                             direction)
    camera = pyrender.OrthographicCamera(xmag=span, ymag=span)
    fwd = target - cam_pos
    fwd = fwd / np.linalg.norm(fwd)
    up = np.array([0.0, 0.0, 1.0]) if abs(fwd[2]) < 0.95 \
        else np.array([0.0, 1.0, 0.0])
    right = np.cross(fwd, up)
    right = right / np.linalg.norm(right)
    up = np.cross(right, fwd)
    cam_pose = np.eye(4)
    cam_pose[:3, 0] = right
    cam_pose[:3, 1] = up
    cam_pose[:3, 2] = -fwd
    cam_pose[:3, 3] = cam_pos
    scene.add(camera, pose=cam_pose)
    light = pyrender.DirectionalLight(color=np.ones(3), intensity=3.0)
    scene.add(light, pose=cam_pose)

    renderer = pyrender.OffscreenRenderer(*size)
    color, _ = renderer.render(scene)
    renderer.delete()
    import imageio
    imageio.imwrite(out_path, color)


# ---------------------------------------------------------------------------
# Matplotlib fallback backend (always works)
# ---------------------------------------------------------------------------

def _render_matplotlib(part_mesh, overlays, out_path, direction,
                       size=(8, 6.4)) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=size, dpi=120)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("#f1f2f4")

    part_collection = Poly3DCollection(
        part_mesh.vertices[part_mesh.faces],
        facecolor="#c5cbd1", edgecolor="#4a4a4a",
        linewidths=0.1, alpha=1.0,
    )
    ax.add_collection3d(part_collection)

    palette = ["#e67e22", "#27ae60", "#2980b9",
               "#8e44ad", "#c0392b", "#16a085"]
    for i, (name, ov) in enumerate(sorted(overlays.items())):
        col = palette[i % len(palette)]
        ov_collection = Poly3DCollection(
            ov.vertices[ov.faces],
            facecolor=col, edgecolor="none", alpha=0.30,
        )
        ax.add_collection3d(ov_collection)

    cam_pos, target, span = _compute_camera(part_mesh,
                                             list(overlays.values()),
                                             direction)
    bb_min = target - span
    bb_max = target + span
    ax.set_xlim(bb_min[0], bb_max[0])
    ax.set_ylim(bb_min[1], bb_max[1])
    ax.set_zlim(bb_min[2], bb_max[2])
    ax.set_box_aspect((1, 1, 1))

    # Set view direction by computing elevation + azimuth from
    # ``direction``.
    d = np.asarray(direction, dtype=float)
    elev = float(np.degrees(np.arcsin(d[2] / max(np.linalg.norm(d), 1e-9))))
    azim = float(np.degrees(np.arctan2(d[1], d[0])))
    ax.view_init(elev=elev, azim=azim + 180.0)
    ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Backend dispatch
# ---------------------------------------------------------------------------

_BACKENDS = [
    ("pyrender",    _render_pyrender),
    ("pyvista",     _render_pyvista),
    ("matplotlib",  _render_matplotlib),
]


def _render_one(part_mesh, overlays, out_path, direction) -> str:
    last_err: Exception | None = None
    for name, fn in _BACKENDS:
        try:
            fn(part_mesh, overlays, out_path, direction)
            return name
        except ImportError as exc:
            last_err = exc
            continue
        except Exception as exc:
            last_err = exc
            continue
    raise RuntimeError(
        f"all backends failed for {out_path}: {last_err}"
    )


# ---------------------------------------------------------------------------
# Per-part driver
# ---------------------------------------------------------------------------

def _load_part_overlays(part_name: str) -> dict[str, trimesh.Trimesh]:
    try:
        from keepout_volumes import part_keepouts
    except Exception:
        return {}
    return part_keepouts(part_name)


def _load_stl(path: str) -> trimesh.Trimesh:
    mesh = trimesh.load(path)
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(
            [g for g in mesh.geometry.values() if len(g.faces) > 0]
        )
    return mesh


def render_part(part_name: str, *,
                stl_dir: str = STL_DIR,
                out_root: str = VIEWS_DIR) -> _PartRender:
    stl_path = os.path.join(stl_dir, f"{part_name}.stl")
    out_dir = os.path.join(out_root, part_name)
    os.makedirs(out_dir, exist_ok=True)

    mesh = _load_stl(stl_path)
    overlays = _load_part_overlays(part_name)

    paths: list[str] = []
    backend_used = ""
    for view_name, direction in VIEWS.items():
        out_path = os.path.join(out_dir, f"{view_name}.png")
        backend_used = _render_one(mesh, overlays, out_path, direction)
        paths.append(out_path)

    return _PartRender(part=part_name, stl_path=stl_path,
                        output_paths=paths, backend=backend_used)


def render_all(*, stl_dir: str = STL_DIR,
               out_root: str = VIEWS_DIR) -> list[_PartRender]:
    if not os.path.isdir(stl_dir):
        raise FileNotFoundError(
            f"STL directory does not exist: {stl_dir}. "
            f"Run `make -C hexapod_walker/prototype build` first."
        )
    parts = sorted(
        os.path.splitext(p)[0]
        for p in os.listdir(stl_dir)
        if p.endswith(".stl") and not p.startswith("_")
        and p not in ("assembly_preview_DO_NOT_PRINT.stl",)
    )
    results = []
    for name in parts:
        try:
            res = render_part(name, stl_dir=stl_dir, out_root=out_root)
            results.append(res)
            print(f"  rendered {name} via {res.backend}: "
                  f"{', '.join(res.output_paths)}")
        except Exception as exc:
            print(f"  FAILED to render {name}: {exc}")
    return results


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.
                                      RawDescriptionHelpFormatter)
    parser.add_argument(
        "--part",
        action="append",
        default=None,
        help=("Render only this part (filename WITHOUT the .stl "
              "extension).  Repeatable.  Default: every part in "
              "stl_prototype/."),
    )
    args = parser.parse_args(argv)

    if args.part:
        results = []
        for name in args.part:
            try:
                res = render_part(name)
                print(f"  rendered {name} via {res.backend}: "
                      f"{', '.join(res.output_paths)}")
                results.append(res)
            except Exception as exc:
                print(f"  FAILED to render {name}: {exc}")
    else:
        results = render_all()

    print()
    print(f"Wrote {sum(len(r.output_paths) for r in results)} view(s) "
          f"to {VIEWS_DIR}")
    return 0 if results else 1


if __name__ == "__main__":
    sys.exit(main())
