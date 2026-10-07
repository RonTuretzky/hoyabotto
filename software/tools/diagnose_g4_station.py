"""Run a dual-arm, free-part G4 guide placement diagnostic; never hardware.

Declared staging coordinates drive the first mechanical diagnostic. This is
explicitly privileged station setup, not yet a calibrated visual controller.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict
import gzip
import json
from pathlib import Path
import sys
import traceback

import numpy as np
from planter.g4_station import StationConfig,StationSimulation,build_station,guide_targets,sha,unpack_contact_lists
from planter.g4_assembly_score import score_assembly_episode
from planter.g4_collision_assets import load_bundle


def read_rows(path):
    try:
        import orjson
        loads=orjson.loads
    except ImportError:loads=json.loads
    with gzip.open(path,'rb') as handle:
        for line in handle:yield unpack_contact_lists(loads(line))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--simulation-root',type=Path,required=True);p.add_argument('--cad',type=Path,required=True)
    p.add_argument('--collision-manifest',type=Path,required=True);p.add_argument('--roller-manifest',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--close-angle',type=float,default=-.12)
    p.add_argument('--timestep',type=float,default=.002);p.add_argument('--noslip',type=int,default=0)
    p.add_argument('--contact-timeconstant',type=float,default=.004)
    p.add_argument('--base-y',type=float,default=-.300)
    p.add_argument('--roller-handle-support',action='store_true')
    p.add_argument('--width',type=int,default=1280);p.add_argument('--grip-height',type=float,default=.043)
    p.add_argument('--approach-inset',type=float,default=0.);p.add_argument('--grip-inset',type=float,default=0.)
    p.add_argument('--gentle-seat',action='store_true',help='Stop4mm above seat, settle0.4s, then40 commanded0.1mm steps over2s; same final target')
    p.add_argument('--setdown-correction',type=float,default=-.0005);p.add_argument('--lift-height',type=float,default=.060)
    p.add_argument('--no-render',action='store_true');p.add_argument('--stop-after',choices=['settle','acquire','lift','complete'],default='complete')
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    c=StationConfig(base_y_m=a.base_y,timestep_s=a.timestep,contact_timeconstant_s=a.contact_timeconstant,noslip_iterations=a.noslip,width=a.width,roller_handle_support=a.roller_handle_support)
    bundle=load_bundle(a.collision_manifest);roller=json.loads(a.roller_manifest.read_text())
    manifest=dict(argv=sys.argv,config=asdict(c),controller='Declared station coordinates only: privileged mechanical diagnostic, not deployable visual control',
                  hardware_commands=0,physical_success=False,full_planter_success=False,
                  trace_encoding="gzipJSONL with explicit lossless float64 contact arrays; decode via read_rows before independent scoring",
                  collision_manifest=str(a.collision_manifest),collision_manifest_sha256=sha(a.collision_manifest),
                  gentle_seat_parameters=None if not a.gentle_seat else dict(clearance_m=.004,settle_s=.4,final_step_m=.0001,final_duration_s=2.),
                  roller_manifest=str(a.roller_manifest),roller_manifest_sha256=sha(a.roller_manifest),
                  source_hashes={str(Path(m.__file__).resolve()):sha(m.__file__) for m in list(sys.modules.values()) if getattr(m,'__file__',None) and str(m.__file__).endswith('.py') and any(x in str(m.__file__) for x in ['/planter/','/tools/diagnose_g4_station.py'])})
    (a.out/'invocation.json').write_text(json.dumps(manifest,indent=2)+'\n')
    sim=None;stopped=None;complete=False
    try:
        model,_=build_station(a.simulation_root,a.cad,bundle,roller,a.out/'scene',c)
        sim=StationSimulation(model,a.out,config=c,render=not a.no_render)
        sim.move('initial_settle',seconds=.20)
        if a.stop_after!='settle':
            pickup=np.asarray(c.guide_pickup_m);seat=np.asarray(c.assembly_origin_m)
            def target(origin,inset=None):return guide_targets(origin,height_m=a.grip_height,inset_m=a.grip_inset if inset is None else inset)
            sim.move('guide_approach_above',target(pickup+[0,0,.065],a.approach_inset),jaw=.55,seconds=1.5)
            sim.move('guide_approach',target(pickup,a.approach_inset),jaw=.55,seconds=1.5)
            if a.approach_inset!=a.grip_inset:sim.move('guide_approach',target(pickup),jaw=.55,seconds=.50,cartesian_step=.00025)
            sim.move('guide_acquire',jaw=a.close_angle,seconds=1.0)
            if a.stop_after not in ('acquire',):
                sim.move('guide_transfer',target(pickup+[0,0,a.lift_height]),seconds=1.3)
                if a.stop_after!='lift':
                    sim.move('guide_transfer',target(seat+[0,0,a.lift_height]),seconds=1.8)
                    sim.move('guide_hold',seconds=.50)
                    if a.gentle_seat:
                        sim.move('guide_place',target(seat+[0,0,a.setdown_correction+.004]),seconds=1.8)
                        sim.move('guide_place',seconds=.40)
                        for step in range(1,41):
                            sim.move('guide_place',target(seat+[0,0,a.setdown_correction+.004*(1-step/40)]),seconds=.05,cartesian_step=0.)
                    else:sim.move('guide_place',target(seat+[0,0,a.setdown_correction]),seconds=1.8)
                    sim.move('guide_release',jaw=.55,seconds=1.0)
                    if a.approach_inset!=a.grip_inset:sim.move('guide_withdraw',target(seat+[0,0,a.setdown_correction],a.approach_inset),jaw=.55,seconds=.50,cartesian_step=.00025)
                    sim.move('guide_withdraw',target(seat+[0,0,.07],a.approach_inset),jaw=.55,seconds=1.3)
                    sim.move('guide_released_hold',seconds=.70);complete=True
    except Exception as error:
        stopped=str(error);(a.out/'error.txt').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
    finally:
        if sim is not None:sim.save(final_label='STOPPED: '+stopped if stopped else ('sequence ended; independent audit pending' if complete else 'PARTIAL diagnostic; complete task not executed'))
    result=dict(completed_command_sequence=complete,stop_reason=stopped,physical_success=False,full_planter_success=False,
                controller_scope='Privileged declared-station contact diagnostic; no visual-controller claim',primitive_success=False)
    if sim is not None:
        result['runtime']=dict(steps=sim.step_index,maximum_contact_force_n=sim.max_force,maximum_penetration_mm=sim.max_penetration*1000,maximum_contacts=sim.max_contacts)
        print(json.dumps(dict(physics_complete=True,**result['runtime'],stop_reason=stopped)),flush=True)
        paths={name:bundle['parts'][name]['visual_stl'] for name in ['trough','holder','guide','carrier']}
        paths.update(roller_handle=roller['parts']['handle']['source'],roller_wheel=roller['parts']['wheel']['source'])
        audit=score_assembly_episode(read_rows(a.out/'physics.jsonl.gz'),model=sim.model,spec={'source_paths':paths})
        result['independent_audit']=audit;result['primitive_success']=audit['primitive_results']['guide_transfer']['passed']
    (a.out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='independent_audit'}),flush=True)


if __name__=='__main__':main()
