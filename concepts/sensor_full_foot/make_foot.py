"""Full sensor backing with glue posts separated from sensor and wire tab."""
from pathlib import Path
import argparse
import json
import numpy as np
import trimesh as tm

HERE = Path(__file__).resolve().parent


def cyl(r, z0, z1, x=0, y=0):
    m = tm.creation.cylinder(radius=r, height=z1-z0, sections=128)
    m.apply_translation([x, y, (z0+z1)/2])
    return m


def box(size, center):
    m = tm.creation.box(size)
    m.apply_translation(center)
    return m


def union(parts):
    return tm.boolean.union(parts, engine='manifold')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--layout', choices=['rear','surround'], required=True)
    args = p.parse_args()
    surround = args.layout == 'surround'
    bore = 16. if surround else 10.
    outer = bore + 3
    base_r = outer/2
    post_r, gap, thickness = 1., .2, 2.
    center_r = bore/2-1.
    sphere = tm.creation.uv_sphere(radius=base_r, count=[64,128])
    lower_half = tm.boolean.intersection([
        sphere, box([outer+2,outer+2,base_r+1],[0,0,-(base_r+1)/2])
    ], engine='manifold')
    base = union([cyl(base_r, -.02, thickness), lower_half])
    posts = [cyl(post_r, -2.8, 2.4, center_r*np.cos(a), center_r*np.sin(a))
             for a in np.deg2rad([0,180,270])]
    socket_r = post_r+gap
    centers = [(center_r*np.cos(a),center_r*np.sin(a))
               for a in np.deg2rad([0,180,270])]
    sockets = [cyl(socket_r,-3.1,2.1,x,y) for x,y in centers]
    foot = tm.boolean.difference([base,*sockets],engine='manifold')
    sensor_z = 2. if surround else -.2
    # Tab is an illustrative 6.5 mm width; length and film thickness unmeasured.
    film = union([cyl(5,sensor_z,sensor_z+.2),
                  box([6.5,5,.2],[0,6,sensor_z+.1])])
    cylinder_height = 20.
    sleeve = tm.boolean.difference([cyl(outer/2,2.2,2.2+cylinder_height),cyl(bore/2,2,3+cylinder_height),
                                    box([7.1,outer,2],[0,outer/2,2.8])],engine='manifold')
    # Sloped pin supports grow gradually inward from the sleeve wall when
    # printed with the unnotched rim down. No floating horizontal boss roofs.
    boss_r = 1.5
    ramps = []
    for x,y in centers:
        bottom = cyl(boss_r,2.2,2.4,x,y)
        top = cyl(.5,8.6,8.8,x*(bore/2+.5)/center_r,y*(bore/2+.5)/center_r)
        ramps.append(tm.convex.convex_hull(np.vstack([bottom.vertices,top.vertices])))
    sleeve = union([sleeve,*ramps,*posts])
    socket_outer_wall = np.sqrt(base_r**2-3.1**2)-center_r-socket_r
    assert socket_outer_wall > .7
    assert center_r-socket_r-5 >= .7 if surround else True
    if surround:
        sensor_post_clearance = center_r-post_r-5
        assert sensor_post_clearance >= .5
    else:
        sensor_post_clearance = 1.8
    for post in posts:
        overlap = tm.boolean.intersection([post,film],engine='manifold')
        assert abs(overlap.volume) < 1e-8
    overlap = tm.boolean.intersection([sleeve,foot],engine='manifold')
    assert abs(overlap.volume) < 1e-8
    assert abs(tm.boolean.intersection([sleeve,film],engine='manifold').volume)<1e-8
    # Check the complete straight insertion path, including partially engaged pins.
    for lift in (.5,1.,2.,3.,4.,5.):
        moving = sleeve.copy(); moving.apply_translation([0,0,lift])
        assert abs(tm.boolean.intersection([moving,foot],engine='manifold').volume)<1e-8
    for x,y in centers:
        probe = cyl(post_r,-2.8,1.99,x,y)
        assert abs(tm.boolean.intersection([probe,foot],engine='manifold').volume)<1e-8
    meshes = {'sensor_full_foot':foot, 'sensor_reference':film, 'mating_cylinder':sleeve}
    for name,m in meshes.items():
        assert m.is_watertight and m.is_winding_consistent and m.volume>0
        assert len(m.split())==1
        m.export(HERE/f'{name}.stl')
    foot_print = foot.copy()
    foot_print.apply_transform(tm.transformations.rotation_matrix(np.pi,[1,0,0]))
    foot_print.apply_translation([0,0,-foot_print.bounds[0,2]])
    foot_print.export(HERE/'sensor_full_foot_print.stl')
    printable = tm.Scene(foot_print); printable.units='mm'
    printable.export(str(HERE/'sensor_full_foot.3mf'))
    sleeve_print = sleeve.copy()
    sleeve_print.apply_transform(tm.transformations.rotation_matrix(np.pi,[1,0,0]))
    sleeve_print.apply_translation([0,0,-sleeve_print.bounds[0,2]])
    sleeve_print.export(HERE/'mating_cylinder_print.stl')
    printable = tm.Scene(sleeve_print); printable.units='mm'
    printable.export(str(HERE/'mating_cylinder.3mf'))
    identity = np.eye(4).T.flatten().tolist()
    transforms = {'sensor_reference':identity}
    for name,height in [('sensor_full_foot',2.),('mating_cylinder',22.2)]:
        matrix = tm.transformations.rotation_matrix(np.pi,[1,0,0])
        matrix[2,3] = height
        transforms[name] = matrix.T.flatten().tolist()
    scene = dict(name='Rounded sensor foot and matching cylinder',units='mm',
                 buildId='prototype_sts3215/sensor-full-foot',center=[0,0,6.35],
                 meshes=[dict(id=k,url=f'{k}.stl' if k=='sensor_reference' else f'{k}_print.stl') for k in meshes],
                 instances=[dict(id=k,meshId=k,partType=k,name=n,color=c,
                                 cots=k=='sensor_reference',transform=transforms[k])
                            for k,n,c in [('sensor_full_foot','Rounded bottom with three glue sockets','#e9a13d'),
                                          ('sensor_reference','10 mm sensor; illustrative wire tab','#249796'),
                                          ('mating_cylinder','20 mm cylinder with downward pins and wire exit','#97aac3')]])
    (HERE/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    workflow = dict(schema=1,parts={
        'sensor_full_foot':dict(kind='printed',material='PLA',notes='Print flat sensor seat down, dome up. Three 2.4 mm blind sockets require only short bridges; no external supports expected. Physical fit untested.'),
        'sensor_reference':dict(kind='purchased',notes='User-measured 10 mm OD, 8.5 mm inner diameter, 6.5 mm tab. Film thickness and tab length illustrative.'),
        'mating_cylinder':dict(kind='printed',material='PLA',notes='Print unnotched rim on bed, pins up. Sloped internal supports avoid horizontal boss overhangs. Use the matching v5 foot.')},
        instructions=[dict(id='assembly',title='Assembly',text='Print both v5 parts. The cylinder pins enter sockets in the rounded bottom. Dry-fit, apply a small amount of glue, and slide together. Keep adhesive off sensor and wire tab.')])
    (HERE/'workflow.json').write_text(json.dumps(workflow,indent=2)+'\n')
    report=dict(layout=args.layout,sensor_od_mm=10,sensor_inner_diameter_mm=8.5,
                tab_width_mm=6.5,backing_diameter_mm=base_r*2,backing_thickness_mm=2,
                rounded_bottom_radius_mm=base_r,rounded_bottom_depth_mm=base_r,
                cylinder_height_mm=cylinder_height,cylinder_wall_mm=1.5,
                cylinder_bore_mm=bore,cylinder_od_mm=outer,post_diameter_mm=2,
                post_height_mm=5,radial_glue_gap_mm=gap,
                post_center_radius_mm=center_r,socket_diameter_mm=socket_r*2,
                socket_depth_mm=5.1,post_engagement_mm=4.8,
                socket_end_clearance_mm=.3,socket_min_outer_wall_mm=socket_outer_wall,
                socket_location='rounded bottom',pin_location='cylinder',
                sensor_to_socket_clearance_mm=center_r-socket_r-5,
                sensor_to_post_clearance_mm=sensor_post_clearance,
                watertight=True,foot_connected_solids=1,
                sensor_post_intersection_mm3=0,cylinder_foot_intersection_mm3=0)
    (HERE/'dimensions.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
