"""Build and render three offline DCM desk candidate stations. Never trains."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image
from carton.folding_refit_preview import (FoldingStation,FoldingSimulation,BASE_PLANE_Z,
    configure_scene,render,oak_render,label,clearance,reach_screen,camera_detail_render)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--simulation-root',type=Path,required=True)
    p.add_argument('--upstream',type=Path,required=True)
    p.add_argument('--camera-metadata',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--box-inset',type=float,default=.01)
    a=p.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    meta=json.loads(a.camera_metadata.read_text())['result']['cameras']['oak']
    K=np.array(meta['intrinsics']);report=dict(training_started=False,physical_registration_verified=False,
        hardware_commands=0,table=dict(model='DCM-F5040H',width_m=.5,depth_m=.48,height_m=.7,
        label_capacity_kg=5,source='user-confirmed dimensions'),
        assumptions=dict(base_mount_height_m=BASE_PLANE_Z,base_spacing_m=.22,box_inset_m=a.box_inset,
        head_pose='CAD head chain with historical box-relative 35.145 deg tilt, -5.132 deg pan; uncalibrated',
        wrist_camera_mass='not calibrated; existing gripper inertial retained',
        table_thickness_m=.032,desk_legs='schematic',cloth_cover='not modeled'),
        projection=dict(K=K.tolist(),image_size=[640,360],distortion='ideal rectified pinhole',
        source_metadata=str(a.camera_metadata),source_sha256=hashlib.sha256(a.camera_metadata.read_bytes()).hexdigest()),cases=[])
    for gap in (.15,.18,.20):
        name=f'setback-{round(gap*1000)}';dest=out/name
        st=FoldingStation(BASE_PLANE_Z-.7,gap,a.box_inset,base_spacing=.22,table_size=(.5,.48))
        targets={s:[sign*.16,st.base_y+.12,st.base_height+.36] for s,sign in [('left',-1),('right',1)]}
        sim=FoldingSimulation(a.simulation_root,dest,station=st,initial_arm_targets=targets)
        cart,spec=configure_scene(sim,a.upstream,dest)
        initial=clearance(sim)
        for cam in ('overall','profile','plan','work'):
            label(render(sim,cam),f'DCM desk | {cam} | base-line setback {gap*1000:.0f} mm',
                  'SIMULATION CANDIDATE | 500 x 480 x 700 mm desk | mounting and carton registration unverified').save(dest/f'{cam}.jpg',quality=92)
        Image.fromarray(oak_render(sim,K)).save(dest/'head.jpg',quality=95)
        for side in ('left','right'):
            Image.fromarray(render(sim,side+'_wrist',640,480)).save(dest/f'{side}-wrist.jpg',quality=94)
        label(camera_detail_render(sim),'Right wrist | CAD shell + two convex collision hulls',
              'Registered mesh, ~1.7 mm CAD-fit RMS; physical mount and mass remain uncalibrated').save(dest/'camera-detail.jpg',quality=94)
        probes=reach_screen(sim)
        row=dict(name=name,station=st.report(),base_line_setback_mm=gap*1000,
                 shoulder_pan_setback_mm=(gap-.0388353)*1000,initial=initial,
                 initial_qpos=sim.data.qpos.tolist(),initial_ctrl=sim.data.ctrl.tolist(),
                 footprint=st.carton_footprint(),head_spec=spec,cart_geoms=cart,probes=probes,
                 static_cart_table_check='no penetration; checked independently of contact filter',
                 sparse_pass_count=sum(x['passes'] for x in probes),probe_count=len(probes))
        report['cases'].append(row)
        (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print(json.dumps({k:row[k] for k in ('name','initial','sparse_pass_count','probe_count')}),flush=True)
    print(out/'report.json')

if __name__=='__main__':main()
