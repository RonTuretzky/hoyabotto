"""Reproduce the positive and negative RGB-D folding cases without hardware."""
import argparse
import json
from pathlib import Path
from simulate_bimanual_folding import run

CASES=[
    ('nominal',{},True),
    ('shift-a',dict(dx=.007,dy=-.004,yaw=2.,seed=17),True),
    ('shift-b',dict(dx=-.006,dy=.004,yaw=-2.,seed=29),True),
    ('sensor-stress',dict(noise=.0015,dropout=.45,seed=37),True),
    ('stiffness-012',dict(stiffness=.012),True),
    ('missing-tags',dict(fault='missing_tags'),False),
    ('missing-depth',dict(fault='missing_depth'),False),
    ('stuck-far',dict(fault='stuck_far_flap'),False),
    ('disabled-right',dict(fault='right_arm_disabled'),False),
    ('no-actions',dict(fault='no_actions'),False),
    ('bad-gripper-calibration',dict(fault='bad_gripper_calibration'),False),
    ('strong-springback-known-limit',dict(stiffness=.018),False),
]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--simulation-root',required=True);p.add_argument('--out',required=True)
    args=p.parse_args();root=Path(args.out).resolve()
    if root.exists():raise ValueError('Refusing to overwrite an existing evaluation')
    root.mkdir(parents=True)
    base=dict(simulation_root=args.simulation_root,base_height=.26,setback=.04,stiffness=.008,seed=1,noise=.0008,dropout=.25,dx=0,dy=0,yaw=0,fault=None,no_video=True,width=640,height=360)
    rows=[]
    for name,changes,expected in CASES:
        config={**base,**changes,'out':str(root/name),'no_video':name!='nominal'}
        r=run(argparse.Namespace(**config))
        rows.append({'case':name,'expected_success':expected,'success':r['success'],'matches_expectation':r['success']==expected,
                     'error':r['controller'].get('error'),'evaluation':r['independent_evaluation'],
                     'max_robot_collision_penetration_mm':r['physics']['stats']['max_bad_penetration_mm'],
                     'max_gripper_tag_fk_error_mm':max([x['encoder_fk_error_mm'] for x in r['arm_tag_checks']] or [0]),
                     'result':str(root/name/'result.json')})
        (root/'matrix.json').write_text(json.dumps({'cases':rows,'all_expected_results':all(x['matches_expectation'] for x in rows)},indent=2))
    if not all(x['matches_expectation'] for x in rows):raise SystemExit('Unexpected simulation result; inspect matrix.json')

if __name__=='__main__':main()
