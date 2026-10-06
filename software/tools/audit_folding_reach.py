"""Offline reach sensitivity, using the folding controller's contact geometry.

These are selected hypothetical layouts, not measurements or a confidence
interval fitted to photos. No simulated camera observations or physical motion.
Numerical IK misses reject this controller; only the conservative chain-length
bound proves a point unreachable independent of the chosen wrist orientation.
Passing sparse reach probes does not validate collision-free motion or folding.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from carton.folding_controller import short_flap_points, long_flap_point
from carton.folding_sim import FoldingSimulation
from carton.folding_station import FoldingStation


def probes():
    for theta in (0., math.pi/2):
        for side, point in short_flap_points(theta).items():
            yield f'short_{side}_{round(math.degrees(theta))}',side,point,'down'
    for name,side,sign,x in [('far','right',1,.10),('near','left',-1,0.)]:
        for theta in (0., math.pi/2):
            point,orientation=long_flap_point(theta,sign,x)
            yield f'{name}_{round(math.degrees(theta))}',side,point,orientation


def chain_bound(sim,side):
    model=sim.model
    site=model.site(side+'_tip').id
    body=int(model.site_bodyid[site])
    root=int(model.jnt_bodyid[model.joint(side+'_shoulder_pan').id])
    radius=float(np.linalg.norm(model.site_pos[site]))
    while body!=root:
        if body==0:raise ValueError('Fingertip is not a descendant of shoulder pan')
        radius+=float(np.linalg.norm(model.body_pos[body]))
        body=int(model.body_parentid[body])
    # The imported model's joint is at the shoulder body origin. Without that
    # invariant the simple sum would need to include the joint's local offset.
    if np.linalg.norm(model.jnt_pos[model.joint(side+'_shoulder_pan').id])>1e-8:
        raise ValueError('Nonzero shoulder joint offset is unsupported')
    return sim.data.xpos[root].copy(),radius


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--simulation-root',required=True);p.add_argument('--out',required=True)
    args=p.parse_args();out=Path(args.out).resolve()
    if out.exists():raise ValueError('Preserve previous evidence; use a new output directory')
    out.mkdir(parents=True)
    layouts=[('historical-reference',FoldingStation.historical_reference())]
    # Isolate separation first at the old height; then inspect lower mounts.
    for height in (.26,.12,.06):
        for gap in (.05,.15,.25):
            layouts.append((f'height-{height:.2f}_gap-{gap:.2f}_inset-0.05',FoldingStation(height,gap,.05)))
    rows=[]
    for name,station in layouts:
        sim=FoldingSimulation(Path(args.simulation_root),out/name,station=station,width=960,height=540,stiffness=.008)
        points=[]
        for label,side,point,orientation in probes():
            target=np.asarray(point)+[0,0,.001]  # Declared carton bottom height.
            origin,radius=chain_bound(sim,side)
            _,error=sim.ik(side,target,orientation)
            distance=float(np.linalg.norm(target-origin))
            points.append({'probe':label,'side':side,'target_m':target.tolist(),
                           'ik_position_error_mm':error*1000,'passes_controller_8mm_tolerance':error<=.008,
                           'distance_from_shoulder_pan_mm':distance*1000,'conservative_chain_radius_mm':radius*1000,
                           'outside_conservative_chain_bound':distance>radius+.008})
        Image.fromarray(sim.render('side')).save(out/name/'layout.png')
        sim.renderer.close()
        row={'case':name,'station':station.report(),'probes':points,
             'all_sampled_contacts_within_ik_tolerance':all(v['passes_controller_8mm_tolerance'] for v in points),
             'any_contact_outside_chain_bound':any(v['outside_conservative_chain_bound'] for v in points)}
        rows.append(row)
        print(json.dumps({'case':name,'passes_sparse_reach_screen':row['all_sampled_contacts_within_ik_tolerance'],
                          'far_start_ik_miss_mm':next(v['ik_position_error_mm'] for v in points if v['probe']=='far_0'),
                          'outside_chain_bound':row['any_contact_outside_chain_bound']}),flush=True)
        report={'physical_registration_verified':False,'hardware_commands':0,'contact_simulation_run':False,
                'scope':__doc__,'cases':rows,'source_arm_sha256':hashlib.sha256((Path(args.simulation_root)/'scene-assets/arm-import.xml').read_bytes()).hexdigest(),
                'code_sha256':{str(f.relative_to(Path(__file__).resolve().parents[1])):hashlib.sha256(f.read_bytes()).hexdigest() for f in [Path(__file__).resolve(),*[Path(__file__).resolve().parents[1]/'carton'/s for s in ('folding_station.py','folding_sim.py','folding_controller.py')]]}}
        (out/'reach.json').write_text(json.dumps(report,indent=2,allow_nan=False))


if __name__=='__main__':main()
