"""Parametric flat semicircular sensor backing, with three rear glue posts.

Run with uv run --no-project --with trimesh --with manifold3d --with matplotlib
python make_foot.py --sensor-od 20 --cylinder-id 10
"""
import argparse
import json
from pathlib import Path

import numpy as np
import trimesh as tm
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = Path(__file__).resolve().parent


def cylinder(r, height, xyz):
    mesh = tm.creation.cylinder(radius=r, height=height, sections=128)
    mesh.apply_translation(xyz)
    return mesh


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sensor-od', type=float, required=True)
    parser.add_argument('--cylinder-id', type=float, default=10)
    args = parser.parse_args()
    radius = args.sensor_od / 2 + 1
    thickness, post_height, post_r, glue_gap = 2., 5., .95, .2
    post_center_r = args.cylinder_id / 2 - glue_gap - post_r
    assert post_center_r > post_r
    disk = cylinder(radius, thickness, [0, 0, thickness / 2])
    halfspace = tm.creation.box([radius * 3, radius * 2, thickness + 2])
    halfspace.apply_translation([0, -radius, thickness / 2])
    base = tm.boolean.intersection([disk, halfspace], engine='manifold')
    posts = []
    for angle in (210, 270, 330):
        a = np.deg2rad(angle)
        x, y = post_center_r * np.cos(a), post_center_r * np.sin(a)
        assert np.hypot(x, y) + post_r < radius
        assert y + post_r < 0
        posts.append(cylinder(post_r, post_height + .2,
                              [x, y, thickness + post_height / 2 - .1]))
    part = tm.boolean.union([base, *posts], engine='manifold')
    assert part.is_watertight and part.is_winding_consistent and part.volume > 0
    assert len(part.split()) == 1
    part.export(HERE / 'sensor_semicircle_foot.stl')
    scene = tm.Scene(part)
    scene.units = 'mm'
    scene.export(str(HERE / 'sensor_semicircle_foot.3mf'))
    report = dict(sensor_outside_diameter_mm=args.sensor_od,
                  sensor_inner_diameter_mm=8.5, tab_width_mm=6.5,
                  backing_radius_mm=radius, backing_thickness_mm=thickness,
                  cylinder_inside_diameter_mm=args.cylinder_id,
                  glue_post_diameter_mm=post_r * 2,
                  glue_post_height_mm=post_height,
                  nominal_radial_glue_clearance_mm=glue_gap,
                  bounds_mm=part.bounds.tolist(), watertight=bool(part.is_watertight),
                  connected_components=len(part.split()), volume_mm3=float(part.volume))
    (HERE / 'dimensions.json').write_text(json.dumps(report, indent=2) + '\n')
    fig = plt.figure(figsize=(11, 5), facecolor='#fafafa')
    ax = fig.add_subplot(121, projection='3d')
    ax.add_collection3d(Poly3DCollection(part.triangles, facecolor='#e5a443',
                                       edgecolor='none', linewidth=0))
    ax.set(xlim=(-radius, radius), ylim=(-radius, 2), zlim=(0, 8))
    ax.set_box_aspect((radius*2, radius+2, 8))
    ax.view_init(28, -55)
    ax.set_title('Rear: three glue-in posts')
    ax.set_xlabel('mm'); ax.set_ylabel('mm'); ax.set_zlabel('mm')
    ax = fig.add_subplot(122)
    a = np.linspace(np.pi, 2*np.pi, 180)
    ax.fill(np.r_[radius*np.cos(a), -radius],
            np.r_[radius*np.sin(a), 0], color='#e5a443', label='2 mm backing')
    ax.add_patch(plt.Circle((0,0),args.sensor_od/2, fill=False,
                           color='#227f8b', linewidth=2, label='Sensor outline'))
    ax.add_patch(plt.Circle((0,0),4.25, fill=False, color='#227f8b',linestyle=':'))
    ax.plot([-3.25,-3.25,3.25,3.25], [args.sensor_od/2,args.sensor_od/2+3,
            args.sensor_od/2+3,args.sensor_od/2], color='#227f8b')
    ax.set_aspect('equal'); ax.set_title('Front: sensor adhered to lower half')
    ax.set_xlim(-radius-2,radius+2); ax.set_ylim(-radius-2,args.sensor_od/2+5)
    ax.set_xlabel('mm'); ax.set_ylabel('mm'); ax.legend(loc='lower right',fontsize=8)
    fig.suptitle(f'Semicircle sensor backing · sensor Ø{args.sensor_od:g} · cylinder bore Ø{args.cylinder_id:g} mm')
    fig.tight_layout()
    fig.savefig(HERE / 'preview.png', dpi=170)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
