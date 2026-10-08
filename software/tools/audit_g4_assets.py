"""Compare pusher collision blocks against exact G4 STL by Boolean difference."""
import argparse
import json
from pathlib import Path
import numpy as np
import trimesh
from planter.g4_sim import BLOCKS,sha


def audit(cad):
    path=cad/'feeder_print_PROTOTYPE.stl'
    mesh=trimesh.load_mesh(path);mesh.apply_scale(.001)
    blocks=[]
    for _,center,half in BLOCKS:
        block=trimesh.creation.box(np.array(half)*2);block.apply_translation(center);blocks.append(block)
    collision=trimesh.boolean.union(blocks,engine='manifold')
    missing=trimesh.boolean.difference([mesh,collision],engine='manifold')
    extra=trimesh.boolean.difference([collision,mesh],engine='manifold')
    report=dict(cad=str(path),sha256=sha(path),cad_volume_mm3=mesh.volume*1e9,
                collision_volume_mm3=collision.volume*1e9,missing_volume_mm3=abs(missing.volume)*1e9,
                extra_volume_mm3=abs(extra.volume)*1e9,centroid_mm=(mesh.center_mass*1000).tolist(),
                bounds_mm=(mesh.bounds*1000).tolist(),physical_fit_validated=False)
    report['passed']=bool(report['missing_volume_mm3']<.005 and report['extra_volume_mm3']<.005)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--cad',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    if a.out.exists():p.error('Preserve prior audit; choose a new file')
    r=audit(a.cad);a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(r,indent=2)+'\n')
    print(json.dumps(r));raise SystemExit(0 if r['passed'] else 1)
