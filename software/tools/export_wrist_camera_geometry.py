"""Re-export upstream camera triangles into SO101 gripper coordinates. Offline only.

Usage: python tools/export_wrist_camera_geometry.py --model /path/xlerobot.xml
The checked-in registration is from the fixed-jaw and servo mesh ICP (1.7 mm RMS).
This preserves the upstream 1 mm left/right mount difference.
"""
import argparse
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np

OUT = Path(__file__).resolve().parents[1] / 'carton/assets/wrist-camera'

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--model', type=Path, required=True)
    a = ap.parse_args()
    reg = json.loads((OUT/'registration.json').read_text())
    R, t = np.array(reg['rotation']), np.array(reg['translation_m'])
    m = mujoco.MjModel.from_xml_path(str(a.model))
    d = mujoco.MjData(m); mujoco.mj_forward(m,d)
    manifest = {'source_model': str(a.model), 'source_sha256': hashlib.sha256(a.model.read_bytes()).hexdigest(),
                'registration': reg, 'physical_mount_verified': False,
                'collision': 'one convex hull per upstream part', 'mass_calibrated': False, 'parts': {}}
    # Upstream body labels are opposite to the fold sim's left/right convention.
    for side, jaw, camera in [('left','Fixed_Jaw_2','Left_Arm_Camera'), ('right','Fixed_Jaw','Right_Arm_Camera')]:
        for part in (1,2):
            gid = next(g for g in range(m.ngeom) if m.geom_bodyid[g]==m.body(camera).id and
                       m.mesh(m.geom_dataid[g]).name==f'XLeRobot_camera{part}')
            mid = m.geom_dataid[gid]
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]]
            faces = m.mesh_face[m.mesh_faceadr[mid]:m.mesh_faceadr[mid]+m.mesh_facenum[mid]]
            world = np.einsum("ij,kj->ik", v, d.geom_xmat[gid].reshape(3,3)) + d.geom_xpos[gid]
            local = np.einsum("ij,jk->ik", world-d.body(jaw).xpos, d.body(jaw).xmat.reshape(3,3))
            v = np.einsum("ij,kj->ik", local, R)+t
            assert np.isfinite(v).all()
            dest = OUT/f'{side}-{part}.obj'
            dest.write_text('# XLeRobot camera CAD in SO101 gripper_link metres\n'+
                            ''.join('v %.10g %.10g %.10g\n'%tuple(p) for p in v)+
                            ''.join('f %d %d %d\n'%tuple(f+1) for f in faces))
            manifest['parts'][dest.name]={'bounds_m':[v.min(0).tolist(),v.max(0).tolist()],
                                         'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),
                                         'upstream_body':camera, 'vertices':len(v), 'triangles':len(faces)}
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({k:v['bounds_m'] for k,v in manifest['parts'].items()},indent=2))

if __name__=='__main__': main()
