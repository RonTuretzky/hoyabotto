"""G4 passive-wheel mechanism diagnostics, not robot folding or paper training.

The rolling bench constrains the handle on passive X/Z rails and sets initial
linear velocity once at reset. Only contact can spin its wheel. The spin bench
sets initial wheel velocity once at reset. Neither bench actuates a joint.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as E

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from planter.g4_roller import add_roller, prepare_roller, sha


def run_case(manifest, out, *, mode='rolling', timestep=.0005, friction=.6,
             bearing_loss=1e-5, initial_speed=.08, initial_clearance=.0004, video=True):
    out.mkdir(parents=True, exist_ok=False)
    root = E.Element('mujoco', model='G4_roller_mechanism_diagnostic')
    E.SubElement(root, 'compiler', angle='radian', autolimits='true')
    option = E.SubElement(root, 'option', timestep=str(timestep), integrator='implicitfast',
                         gravity='0 0 -9.81' if mode == 'rolling' else '0 0 0',
                         iterations='100', tolerance='1e-10', cone='elliptic')
    E.SubElement(option, 'flag', multiccd='disable')
    visual = E.SubElement(root, 'visual')
    E.SubElement(visual, 'global', offwidth='800', offheight='600')
    E.SubElement(visual, 'headlight', ambient='.6 .6 .6', diffuse='.7 .7 .7')
    world = E.SubElement(root, 'worldbody')
    E.SubElement(world, 'light', pos='.1 -.3 .4', dir='0 0 -1')
    E.SubElement(world, 'geom', name='ground_visual', type='box', size='.25 .15 .005',
                 pos='.06 0 -.03', contype='0', conaffinity='0', rgba='.75 .70 .60 1')
    if mode == 'rolling':
        # 46 mm track is narrower than the fork. It is a rigid contact coupon,
        # not paper, and the fork cannot provide hidden support on the coupon.
        E.SubElement(world, 'geom', name='coupon', type='box', size='.18 .023 .005',
                     pos='.07 0 -.005', friction=f'{friction} .0001 .00001',
                     priority='1', solref='.004 1', condim='4', rgba='.9 .85 .65 1')
    meta = add_roller(root, manifest, position_m=(0, 0, initial_clearance if mode == 'rolling' else .02),
                      free=False, frictionloss=bearing_loss)
    handle = root.find("./worldbody/body[@name='roller']")
    if mode == 'rolling':
        E.SubElement(handle, 'joint', name='bench_x', type='slide', axis='1 0 0', damping='0')
        E.SubElement(handle, 'joint', name='bench_z', type='slide', axis='0 0 1', damping='0')
    E.indent(root); E.ElementTree(root).write(out/'scene.xml', encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(out/'scene.xml')); data = mujoco.MjData(model)
    hinge = model.joint(meta['wheel_joint']); hdof = hinge.dofadr[0]; hq = hinge.qposadr[0]
    if mode == 'rolling':
        data.qvel[model.joint('bench_x').dofadr[0]] = initial_speed
    else:
        data.qvel[hdof] = initial_speed
    mujoco.mj_forward(model, data)
    initial_contacts = [float(c.dist) for c in data.contact if c.dist < -1e-6]
    if initial_contacts:
        raise ValueError(f'Initial penetration: {initial_contacts}')
    assert model.nu == 0 and model.neq == 0 and model.nmocap == 0
    camera = mujoco.MjvCamera(); camera.lookat[:] = [.045, 0, .025]
    camera.distance = .25; camera.azimuth = 135; camera.elevation = -25
    renderer = mujoco.Renderer(model, height=600, width=800) if video else None
    render_opt = mujoco.MjvOption(); render_opt.geomgroup[3] = 0
    frames=[]; rows=[]; peak_pen=0.; peak_force=0.; unexpected=[]
    frame_every=max(1,round(.05/timestep)); duration=1.5
    with (out/'physics.jsonl').open('w') as log:
        for step in range(round(duration/timestep)+1):
            # Coherent post-step kinematics, contact and state for independent audit.
            if step:
                mujoco.mj_step(model,data); mujoco.mj_forward(model,data)
            contacts=[]
            for ci,c in enumerate(data.contact):
                force=np.zeros(6);mujoco.mj_contactForce(model,data,ci,force)
                names=[model.geom(int(g)).name for g in c.geom]
                contacts.append(dict(geoms=names, distance_m=float(c.dist),
                                     position_m=c.pos.tolist(), frame=c.frame.tolist(),
                                     wrench_contact=force.tolist()))
                peak_pen=max(peak_pen,-float(c.dist))
                peak_force=max(peak_force,float(force[0]))
                if force[0]>.005 and not any(n.startswith('roller_wheel_') for n in names):
                    unexpected.append(dict(time_s=float(data.time),geoms=names,normal_N=float(force[0])))
            row=dict(time_s=float(data.time),qpos=data.qpos.tolist(),qvel=data.qvel.tolist(),
                     wheel_angle_rad=float(data.qpos[hq]),wheel_speed_rad_s=float(data.qvel[hdof]),
                     handle_position_m=data.body('roller').xpos.tolist(),contacts=contacts,
                     qfrc_applied=data.qfrc_applied.tolist(),xfrc_applied=data.xfrc_applied.tolist())
            rows.append(row);log.write(json.dumps(row)+'\n')
            if renderer and (step%frame_every==0 or step==round(duration/timestep)):
                renderer.update_scene(data,camera=camera,scene_option=render_opt)
                frame=Image.fromarray(renderer.render());draw=ImageDraw.Draw(frame)
                draw.rectangle([0,0,800,49],fill='white')
                draw.text((10,8),f'G4 {mode}: passive-wheel mechanism bench | {data.time:.3f}s',fill='black')
                draw.text((10,27),'DIAGNOSTIC RAILS / INITIAL VELOCITY — NO ROBOT OR PAPER',fill='black')
                frames.append(frame)
    if renderer: renderer.close()
    if frames:
        frames[0].save(out/'timeline.gif',save_all=True,append_images=frames[1:],duration=50,loop=0)
        frames[0].save(out/'before.png');frames[-1].save(out/'after.png')
    dx=rows[-1]['handle_position_m'][0]-rows[0]['handle_position_m'][0]
    angle=rows[-1]['wheel_angle_rad']-rows[0]['wheel_angle_rad']
    moving=[r for r in rows if .2 <= r['time_s'] <= 1.]
    # Wheel rotating about +Y rolls toward +X. Geometry radius is 8 mm.
    ratios=[abs(r['qvel'][0]-.008*r['wheel_speed_rad_s']) for r in moving] if mode=='rolling' else []
    passed=(not unexpected and peak_pen<.0001 and abs(dx)>.02 and angle>2 and
            max(ratios,default=100)<.005) if mode=='rolling' else (
            0<abs(rows[-1]['wheel_speed_rad_s'])<abs(initial_speed) and not unexpected)
    result=dict(scope='passive_roller_mechanism_only',robot_folding_success=False,
                simulated_mechanism_pass=bool(passed),mode=mode,timestep_s=timestep,
                coupon_friction=friction,bearing_frictionloss_Nm=bearing_loss,
                initial_speed=initial_speed,initial_clearance_m=initial_clearance,
                initial_velocity_is_reset_only=True,
                full_planter_success=False,hardware_commands=0,
                observations=len(rows),displacement_m=dx,wheel_rotation_rad=angle,
                final_wheel_speed_rad_s=rows[-1]['wheel_speed_rad_s'],
                max_rolling_velocity_error_m_s=max(ratios,default=None),
                peak_point_normal_N=peak_force,max_penetration_m=peak_pen,
                unexpected_loaded_contacts=unexpected,model_actuators=int(model.nu),
                model_mocap=int(model.nmocap),model_equalities=int(model.neq),
                assets_manifest=manifest,source_sha256=sha(__file__),
                artifacts={p.name:sha(p) for p in out.iterdir() if p.is_file()})
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cad',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--no-video',action='store_true')
    parser.add_argument('--initial-clearance',type=float,default=.0004)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    if not np.isfinite(args.initial_clearance) or args.initial_clearance<=0:
        parser.error('Initial clearance must be finite and positive')
    manifest=prepare_roller(args.cad,args.out/'assets')
    cases=[dict(name='rolling'),dict(name='half_timestep',timestep=.00025),
           dict(name='zero_friction',friction=0.),dict(name='locked_bearing',bearing_loss=.02),
           dict(name='disabled_motion',initial_speed=0.),dict(name='spin_decay',mode='spin',initial_speed=5.)]
    rows=[]
    for case in cases:
        name=case.pop('name');result=run_case(manifest,args.out/name,video=not args.no_video,
                                            initial_clearance=args.initial_clearance,**case)
        rows.append(dict(name=name,result=str(args.out/name/'result.json'),
                         simulated_mechanism_pass=result['simulated_mechanism_pass'],
                         displacement_m=result['displacement_m'],wheel_rotation_rad=result['wheel_rotation_rad'],
                         max_penetration_m=result['max_penetration_m'],
                         max_rolling_velocity_error_m_s=result['max_rolling_velocity_error_m_s']))
        print(json.dumps(rows[-1]),flush=True)
    expected={r['name']:r['simulated_mechanism_pass'] for r in rows}
    (args.out/'result.json').write_text(json.dumps(dict(scope='mechanism_diagnostic',cases=rows,
        expected_positive=['rolling','half_timestep','spin_decay'],
        expected_negative=['zero_friction','locked_bearing','disabled_motion'],
        discriminates_all=all(expected[n] for n in ['rolling','half_timestep','spin_decay']) and
                           all(not expected[n] for n in ['zero_friction','locked_bearing','disabled_motion']),
        full_planter_success=False,robot_folding_success=False),indent=2)+'\n')


if __name__=='__main__':main()
