"""Ground-contact prototype: TPU return collar, guided motion, pad and hard stop."""
from pathlib import Path
import json
import numpy as np
import trimesh as tm
HERE=Path(__file__).resolve().parent
FOOT_R,SEAT_Z,FILM_T,TRAVEL=9.5,2.,.2,.6
BODY_Z,TOP_Z,PAD_BOTTOM,PAD_TOP,ROD_STOP_Z=2.6,22.6,2.4,3.4,5.4

def cyl(r,z0,z1,x=0,y=0):
 m=tm.creation.cylinder(radius=r,height=z1-z0,sections=128);m.apply_translation([x,y,(z0+z1)/2]);return m

def box(size,center):
 m=tm.creation.box(size);m.apply_translation(center);return m

def union(parts): return tm.boolean.union(parts,engine='manifold')
def diff(parts): return tm.boolean.difference(parts,engine='manifold')
def overlap(a,b):
 m=tm.boolean.intersection([a,b],engine='manifold');return 0. if len(m.faces)==0 else abs(float(m.volume))
def moved(m,z):
 m=m.copy();m.apply_translation([0,0,z]);return m

def main():
 centers=[(7*np.cos(a),7*np.sin(a)) for a in np.deg2rad([0,180,270])]
 hemi=tm.boolean.intersection([tm.creation.uv_sphere(radius=FOOT_R,count=[64,128]),box([21,21,11],[0,0,-5.5])],engine='manifold')
 blank=union([hemi,cyl(FOOT_R,-.02,SEAT_Z)])
 groove=diff([cyl(10,.3,1.3),cyl(8.85,.2,1.4)])
 foot=diff([blank,groove,*[cyl(1.2,-3.1,2.1,x,y) for x,y in centers]])
 film=union([cyl(5,SEAT_Z,SEAT_Z+FILM_T),box([6.5,5,FILM_T],[0,6,SEAT_Z+FILM_T/2])])
 upper_groove=diff([cyl(8.2,4.6,5.6),cyl(7.55,4.5,5.7)])
 # Closed web supports the carbon rod; outer rim is the positive travel stop.
 body=diff([cyl(8,BODY_Z,TOP_Z),cyl(4.1,ROD_STOP_Z,TOP_Z+.1),cyl(5.3,BODY_Z-.1,PAD_TOP),box([7.1,16.2,2],[0,11.9,3]),upper_groove])
 body=union([body,*[cyl(1,-2.4,BODY_Z+.2,x,y) for x,y in centers]])
 pad=cyl(3.25,PAD_BOTTOM,PAD_TOP)
 assert overlap(cyl(3.25,PAD_TOP+.01,PAD_TOP+.1),body) > .99*np.pi*3.25**2*.09,'pressure pad backing incomplete'
 # Flexible snap cuffs in rigid grooves; bowed wall supplies return compliance.
 profile=np.array([[9.9,.4],[9.9,1.2],[10.4,1.6],[11.4,2.6],[10.4,3.6],[8.65,4.7],[8.65,5.5],[7.65,5.5],[7.65,4.8],[8.15,4.6],[9.8,3.0],[10.5,2.6],[9.8,2.2],[9.65,1.3],[8.95,1.1],[8.95,.4]])
 collar=tm.creation.revolve(np.vstack([profile,profile[0]]),sections=128)
 if collar.volume<0: collar.invert()
 collar=diff([collar,box([7.1,20,2.2],[0,10,2.6])])
 meshes={'sensor_full_foot':foot,'mating_cylinder':body,'return_collar_tpu':collar,'contact_pad_tpu':pad,'sensor_reference':film}
 for name,m in meshes.items():
  assert m.is_watertight and m.is_winding_consistent and m.volume>0,name
  assert len(m.split())==1,name
  m.export(HERE/f'{name}.stl')
  if name!='sensor_reference': assert overlap(m,film)<1e-7,(name,'sensor')
 for a,b in [(body,foot),(collar,foot),(collar,body)]: assert overlap(a,b)<1e-7,('rest overlap',overlap(a,b))
 for travel in np.linspace(0,TRAVEL,7):
  assert overlap(moved(body,-travel),foot)<1e-6,('travel foot',travel)
  assert overlap(moved(body,-travel),film)<1e-6,('travel sensor',travel)
 assert overlap(moved(body,-TRAVEL-.1),foot)>.1,'missing stop'
 assert -2.4-TRAVEL > -3.1,'pin bottoms before stop'
 rod=cyl(4,ROD_STOP_Z+.01,TOP_Z+2)
 assert overlap(body,rod)<1e-7
 assert overlap(body,moved(rod,-.2))>.1,'missing rod stop'
 tail=box([6.5,20,.2],[0,10,2.15])
 assert overlap(tail,collar)<1e-7,'collar wire clearance'
 assert overlap(tail,moved(body,-TRAVEL))<1e-7,'body wire clearance'
 gap=PAD_BOTTOM-(SEAT_Z+FILM_T)
 assert 0<gap<TRAVEL
 report=dict(cylinder_od_mm=16,cylinder_bore_mm=8.2,cylinder_height_mm=20,nominal_rod_od_mm=8,rod_socket_depth_mm=TOP_Z-ROD_STOP_Z,foot_od_mm=19,return_collar_od_mm=22.8,max_rigid_travel_mm=TRAVEL,pad_diameter_mm=6.5,pad_thickness_mm=PAD_TOP-PAD_BOTTOM,assumed_sensor_plus_adhesive_thickness_mm=FILM_T,nominal_initial_pad_gap_mm=gap,nominal_pad_compression_at_stop_mm=TRAVEL-gap,guide_diametral_clearance_mm=.4,pin_end_gap_at_stop_mm=.1,sensor_to_guide_clearance_mm=.8,watertight=True,rigid_travel_checks_passed=True,rod_stop_check_passed=True,trigger_force='unmeasured; bench calibration required',flexible_deformation='not simulated; collar and pad shown unloaded')
 (HERE/'dimensions.json').write_text(json.dumps(report,indent=2)+'\n')
 colors={'sensor_full_foot':'#e9a13d','mating_cylinder':'#97aac3','return_collar_tpu':'#945dc7','contact_pad_tpu':'#dd617c','sensor_reference':'#249796'}
 names={'sensor_full_foot':'Rounded moving foot with guide bores','mating_cylinder':'Rod socket, pressure web, guide pins and stop','return_collar_tpu':'TPU snap-on return collar (unloaded)','contact_pad_tpu':'TPU pressure pad, 6.5 mm diameter','sensor_reference':'10 mm film sensor reference'}
 mesh_defs=[];instances=[]
 for name,m in meshes.items():
  matrix=np.eye(4)
  if name=='sensor_reference': url=f'{name}.stl'
  else:
   printable=m.copy()
   if name in ('sensor_full_foot','mating_cylinder'):
    matrix=tm.transformations.rotation_matrix(np.pi,[1,0,0]);printable.apply_transform(matrix)
   shift=-printable.bounds[0,2];printable.apply_translation([0,0,shift]);matrix[2,3]=shift
   matrix=np.linalg.inv(matrix)
   url=f'{name}_print.stl';printable.export(HERE/url)
   single=tm.Scene(printable);single.units='mm';single.export(str(HERE/f'{name}.3mf'))
   # Round-trip print-to-assembly transform must preserve every vertex.
   check=printable.copy();check.apply_transform(matrix);assert np.allclose(check.vertices,m.vertices,atol=1e-6)
  mesh_defs.append(dict(id=name,url=url))
  instances.append(dict(id=name,meshId=name,partType=name,name=names[name],color=colors[name],cots=name=='sensor_reference',transform=matrix.T.flatten().tolist()))
 scene=dict(name='Ground-contact foot with TPU return and travel stop',buildId='prototype_sts3215/sensor-full-foot',units='mm',center=[0,0,6.5],meshes=mesh_defs,instances=instances)
 (HERE/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
 parts={name:dict(kind='purchased' if name=='sensor_reference' else 'printed',material='TPU' if name.endswith('_tpu') else 'PETG',label=names[name]) for name in meshes}
 parts['sensor_reference'].pop('material')
 workflow=dict(schema=1,parts=parts,instructions=[dict(id='assembly',title='Contact-sensing assembly',text='Use all v9 parts. Stick sensor to foot without covering active area or vent. Attach TPU pad to cylinder web. Guides must slide freely; do not glue them. Stretch collar cuffs into both grooves with wire window aligned. Rod seats against closed web. Measure film plus adhesive and set pad gap with thin shims. Bench-test release, touch threshold, tilted contact and hard stop.')])
 (HERE/'workflow.json').write_text(json.dumps(workflow,indent=2)+'\n')
 print(json.dumps(report,indent=2))

if __name__=='__main__': main()
