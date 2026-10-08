"""Passive material test starting with preset closed flaps; NOT a robot fold.

The carton remains unbolted and the arms stay parked without contacting it.
This isolates whether selected crease properties let the flaps stay closed.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import mujoco

from carton.folding_sim import FoldingSimulation, FLAPS, L, W
from carton.folding_material import CartonMaterial
from carton.folding_station import FoldingStation


def run(source,out,stiffness,seconds=5.,contents_mass=.96,video=True):
    out=Path(out).resolve()
    if out.exists():raise ValueError('Preserve prior material tests; choose a new output directory')
    if not math.isfinite(seconds) or seconds<=0:raise ValueError('Positive finite observation duration required')
    yaw=math.radians(30);dy=L/2*math.sin(yaw)+W/2*math.cos(yaw)-W/2
    material=CartonMaterial(contents_mass_kg=contents_mass,hinge_stiffness=stiffness)
    station=FoldingStation(.06,.15,.01,table_marker_xy=(-.5,.55),backup_table_marker_xy=(.45,.70))
    sim=FoldingSimulation(Path(source),out,station=station,material=material,offset=(0.,dy),yaw=yaw,width=960,height=540)
    # Declared test initial condition. No claim that the robot achieved it.
    for f in FLAPS:sim.data.qpos[sim.model.joint(f+'_hinge').qposadr[0]]=math.pi/2
    sim.data.qvel[:]=0;mujoco.mj_forward(sim.model,sim.data)
    sim.capture('MATERIAL TEST: preset closed flaps; robot did not fold them')
    event=sim.move({},seconds,'MATERIAL TEST: parked arms, passive springback',capture=video)
    sim.capture('MATERIAL TEST: end of hands-off observation')
    no_contact=not event['contact_pairs']
    retained=no_contact and all(85<=lo<=hi<=95 for lo,hi in event['flap_angle_extrema_degrees'].values())
    result={'simulation_only':True,'robot_folding_success':None,'test':'passive springback from explicitly preset closed initial condition',
            'no_robot_flap_contact':no_contact,'remains_closed':retained,'material':material.report(),
            'initial_flap_degrees':{f:90. for f in FLAPS},'final_flap_degrees':sim.truth_angles(),
            'physics':sim.save('springback'),'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (out/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps({k:result[k] for k in ('test','remains_closed','no_robot_flap_contact','final_flap_degrees')},indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--simulation-root',required=True);p.add_argument('--out',required=True)
    p.add_argument('--stiffness',type=float,default=.04);p.add_argument('--seconds',type=float,default=5.)
    p.add_argument('--contents-mass',type=float,default=.96);p.add_argument('--no-video',action='store_true')
    a=p.parse_args();run(a.simulation_root,a.out,a.stiffness,a.seconds,a.contents_mass,not a.no_video)
