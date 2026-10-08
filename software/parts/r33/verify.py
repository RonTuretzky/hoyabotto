"""R3.3 geometry checks; no hardware or physical-fit claim."""
from pathlib import Path
import subprocess,json,hashlib,shutil
import numpy as np
import trimesh
from shapely.geometry import Polygon,box
from shapely.ops import unary_union
R=Path(__file__).parent;M=R;O=R/'verification';P=R/'print-folders';O.mkdir(exist_ok=True)
meshes={};report={'revision':'R3.3','status':'CAD only; R3.3 physical fit, print and grasp untested','parts':{},'collisions':{}}
for n in ['reservoir','basket','retainer','plate','assembly']:
 p=M/(n+'.stl');run=subprocess.run(['/opt/homebrew/bin/openscad','--backend','Manifold','--export-format','binstl','-D',f'part="{n}"','-o',str(p),str(M/'planter.scad')],capture_output=True,text=True);assert run.returncode==0,run.stderr
 (O/(n+'.log')).write_text(run.stdout+run.stderr);m=trimesh.load_mesh(p);meshes[n]=m
 bed=m.triangles[:,:,2].max(axis=1)<.025;bad=(m.face_normals[:,2]<-np.sqrt(.5)-.003)&~bed
 d={'dimensions_mm':m.extents.round(3).tolist(),'bounds_mm':m.bounds.round(3).tolist(),'watertight':bool(m.is_watertight),'components':len(m.split()),'volume_mm3':float(m.volume),'downward_area_beyond_45_deg_mm2':float(m.area_faces[bad].sum()),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()};report['parts'][n]=d
 assert m.is_watertight and m.is_winding_consistent
 if n in ['reservoir','basket','retainer']:assert d['components']==1 and d['downward_area_beyond_45_deg_mm2']<.1,(n,d)
 print(n,d,flush=True)
def placed(m,z):
 q=m.copy();q.apply_translation([0,0,z]);return q
def solid(bounds):
 b=np.array(bounds);q=trimesh.creation.box(extents=b[1]-b[0]);q.apply_translation(b.mean(axis=0));return q
def intersection(name,a,b,allowed=.01):
 c=trimesh.boolean.intersection([a,b],engine='manifold');v=abs(float(c.volume)) if len(c.faces) else 0;report['collisions'][name]=round(v,6);assert v<=allowed,(name,v);return c
basket=placed(meshes['basket'],27);ring=placed(meshes['retainer'],34.4)
for dz in np.linspace(0,60,61):
 intersection(f'basket_lower_{dz}',placed(meshes['basket'],27+dz),meshes['reservoir'])
 c=intersection(f'retainer_lower_{dz}',placed(meshes['retainer'],34.4+dz),basket,.01)
corridor=solid([[-45,-34.9,4],[45,-21.1,32]])
for n,m in [('reservoir',meshes['reservoir']),('basket',basket),('ring',ring)]:intersection('cloth_route_'+n,corridor,m)
for side in [-1,1]:
 for phase in ['opening','retreat','lowering']:
  lo,hi=(3,12) if phase=='opening' else (8,12) if phase=='retreat' else (3,7)
  yr=[28.8+lo,28.8+hi] if side==1 else [28.8-hi,28.8-lo]
  jaw=solid([[-6,yr[0],58.4],[6,yr[1],82.4+(0 if phase=='opening' else 60)]])
  for n,m in [('basket',basket),('reservoir',meshes['reservoir']),('ring',ring)]:intersection(f'{phase}_jaw{side}_{n}',jaw,m)
for name,b in [('palm',[[-10,14.8,81.4],[10,42.8,149.4]]),('wrist',[[-10,19.8,88.9],[10,37.8,173.9]])]:
 for n,m in [('basket',basket),('ring',ring)]:intersection(name+'_'+n,solid(b),m)
foot=unary_union([Polygon(t[:,:2]) for t in meshes['retainer'].triangles if Polygon(t[:,:2]).area>1e-8]);covered=foot.intersection(box(-44,-21,44,21)).area;assert abs(covered-64)<.02
plate=meshes['plate'];margin=min(*plate.bounds[0,:2],*(220-plate.bounds[1,:2]));assert margin>=9.99 and len(plate.split())==2
report.update(assembly={'basket_floor_z_mm':27,'basket_rim_z_mm':45,'reservoir_rim_z_mm':45,'opening_mm':[134,88],'retainer_z_mm':34.4,'cloth_thickness_assumption_mm':2,'grow_pad_thickness_assumption_mm':3,'cloth_bottom_z_mm':29.4},retainer={'handle_mm':[16,6,34],'jaw_tip_clearance_above_rear_rim_mm':13.4,'pad_coverage_mm2':covered,'retention':'Gravity seated; no latch. Print retainer solid. Mass is an estimate, not proof of mat retention.','estimated_solid_mass_g_at_1_2_g_cm3':round(meshes['retainer'].volume*.0012,2),'solid_density_assumption_g_cm3':1.2},plate={'minimum_edge_margin_mm':margin},physical_feedback={'revision':'R3.2','user_report':'Print finished; basket perched too high on three rests; retainer too light and lacks grip','response':'Wider reservoir; 8 mm lower seating; raised handle; heavier solid gravity ring; reservoir and ring revised; existing R3.2 basket can be reused'})
(O/'geometry.json').write_text(json.dumps(report,indent=2)+'\n')
for folder,n in [('01-reservoir','reservoir'),('02-grow-basket','basket'),('03-retainer','retainer'),('04-replacement-layout','plate')]:
 d=P/folder;d.mkdir(parents=True,exist_ok=True);shutil.copy2(M/(n+'.stl'),d/(n+'_R3.3.stl'))
 (d/'READ-ME.txt').write_text('R3.3 matched prototype. Units mm, 100% scale, upright. Slice afresh. Use the new reservoir and ring; R3.2 basket is unchanged and reusable. Mat retention, fit, leakage and robot grasp untested.\n'+('Replacement plate: new reservoir and retainer only. Reuse your printed R3.2 basket. Alternative to printing 01 and 03 separately.\n' if n=='plate' else 'Center this part on the build plate.\n'))
(P/'manifest.json').write_text(json.dumps({'revision':'R3.3','files':{str(p.relative_to(P)):hashlib.sha256(p.read_bytes()).hexdigest() for p in P.rglob('*.stl')}},indent=2)+'\n')
print('Geometry passed; gravity-seated parts have no CAD intersections.')
