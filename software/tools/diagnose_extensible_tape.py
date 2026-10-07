"""Free-strip mechanics only: balanced tensile/three-point instrument loads.

No carton, plate, prebond, actuator, equality, pinned root, or hardware exists.
Zero gravity isolates the constitutive benchmark; this is not carton physics.
Three-point bending uses +Fz at both ends and -Fz at both middle segments, not
unbalanced opposite-end forces. Net applied force/torque are audited per step.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from carton.folding_extensible_tape import ExtensibleTapeSpec, add_extensible_tape


def make_model(spec, timestep):
    if timestep not in (.000025, .000010):
        raise ValueError('This benchmark is declared only for 25 or 10 microseconds')
    root = ET.Element('mujoco', model='Free extensible-strip mechanical coupon only')
    option = ET.SubElement(root, 'option', timestep=str(timestep), integrator='discrete',
                           solver='Newton', iterations='120', tolerance='1e-10',
                           cone='elliptic', gravity='0 0 0')
    ET.SubElement(option, 'flag', diagexact='enable')
    ET.SubElement(root, 'size', memory='32M')
    ET.SubElement(root, 'worldbody')
    add_extensible_tape(root, spec, start_m=[-spec.tape.length_m/2, 0., 0.])
    xml = ET.tostring(root, encoding='unicode')
    return mujoco.MjModel.from_xml_string(xml), xml


def smooth(u):
    u = min(1., max(0., u))
    return u*u*(3-2*u)


def loads_at(t, mode, tension=.6, transverse=.06):
    if mode == 'axial':
        scale = smooth(t/.05) if t < .05 else (1. if t < .15 else 1.-smooth((t-.15)/.05))
        return tension*scale, 0.
    if mode != 'three_point':
        raise ValueError('Unknown free-strip load protocol')
    scale = smooth(t/.05) if t < .4 else 1.-smooth((t-.4)/.05)
    bend = smooth((t-.15)/.05) if t < .35 else 1.-smooth((t-.35)/.05)
    return tension*scale, transverse*bend


def run_case(out, *, timestep, mode, tension=.6, transverse=.06, spec=None):
    out = Path(out)
    if out.exists():
        raise ValueError('Use a new evidence directory')
    for name, value in [('tension', tension), ('transverse', transverse)]:
        if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
            raise ValueError('Positive finite '+name+' load required')
    spec = spec or ExtensibleTapeSpec()
    model, xml = make_model(spec, timestep)
    if model.nu or model.neq:
        raise ValueError('Mechanical coupon must have no actuators/equalities')
    data, measure = mujoco.MjData(model), mujoco.MjData(model)
    body_ids = np.array([model.body(f'tape_{i}').id for i in range(spec.tape.segments)])
    axial = np.array([int(model.joint(f'tape_axial_{i}').qposadr[0]) for i in range(1, spec.tape.segments)])
    bend = np.array([int(model.joint(f'tape_bend_{i}').qposadr[0]) for i in range(1, spec.tape.segments)])
    twist = np.array([int(model.joint(f'tape_twist_{i}').qposadr[0]) for i in range(1, spec.tape.segments)])
    duration = .30 if mode == 'axial' else .60
    n = round(duration/timestep)
    every = round(.001/timestep)
    samples = []
    max_force = max_torque = max_strain = max_contact = 0.
    max_com_drift = max_span_rotation = peak_absolute_work = 0.
    work = 0.
    error = None
    started = time.monotonic()
    masses = model.body_mass[body_ids]
    mids = [spec.tape.segments//2-1, spec.tape.segments//2]
    if spec.tape.segments % 2:
        raise ValueError('Symmetric three-point instrument requires an even segment count')

    def positions(target):
        mujoco.mj_kinematics(model, target)
        mujoco.mj_comPos(model, target)
        return target.xipos[body_ids].copy()

    def observe():
        measure.qpos[:] = data.qpos
        xyz = positions(measure)
        center = np.average(xyz, axis=0, weights=masses)
        axial_energy = .5*spec.axial_joint_stiffness_N_m*float(np.sum(data.qpos[axial]**2))
        angular_energy = .5*spec.tape.bend_stiffness_Nm*float(np.sum(data.qpos[bend]**2))+.5*spec.tape.twist_stiffness_Nm*float(np.sum(data.qpos[twist]**2))
        return xyz, dict(time_s=float(data.time), contour_extension_m=float(np.sum(data.qpos[axial])),
                         end_com_span_m=float(np.linalg.norm(xyz[-1]-xyz[0])),
                         center_sag_m=float((xyz[0,2]+xyz[-1,2])/2-np.mean(xyz[mids,2])),
                         center_of_mass_m=center.tolist(), axial_spring_energy_J=axial_energy,
                         angular_spring_energy_J=angular_energy,
                         max_absolute_axial_strain=float(np.max(np.abs(data.qpos[axial]))/spec.axial_cell_length_m))

    initial_xyz, first = observe()
    initial_com = np.array(first['center_of_mass_m'])
    initial_span = initial_xyz[-1]-initial_xyz[0]
    initial_span /= np.linalg.norm(initial_span)
    first.update(tension_N=0., end_transverse_N=0., applied_work_J=0.)
    samples.append(first)
    for step in range(n):
        t = float(data.time)
        tension_now, transverse_now = loads_at(t, mode, tension, transverse)
        xyz = positions(data)
        forces = np.zeros((spec.tape.segments, 3))
        forces[0,0], forces[-1,0] = -tension_now, tension_now
        forces[0,2] += transverse_now
        forces[-1,2] += transverse_now
        forces[mids,2] -= transverse_now
        center = np.average(xyz, axis=0, weights=masses)
        net_force = float(np.linalg.norm(forces.sum(axis=0)))
        net_torque = float(np.linalg.norm(np.sum(np.cross(xyz-center, forces),axis=0)))
        max_force, max_torque = max(max_force,net_force), max(max_torque,net_torque)
        data.xfrc_applied[:] = 0.
        data.qfrc_applied[:] = 0.
        data.xfrc_applied[body_ids,:3] = forces
        mujoco.mj_step(model, data)
        max_contact = max(max_contact, int(data.ncon))
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or any(w.number for w in data.warning):
            error = 'Nonfinite state or MuJoCo warning'; break
        after, sample = observe()
        work += float(np.sum(forces*(after-xyz)))
        peak_absolute_work=max(peak_absolute_work,abs(work))
        max_com_drift=max(max_com_drift,float(np.linalg.norm(np.array(sample['center_of_mass_m'])-initial_com)))
        span=after[-1]-after[0]
        span/=np.linalg.norm(span)
        max_span_rotation=max(max_span_rotation,float(math.atan2(np.linalg.norm(np.cross(span,initial_span)),span@initial_span)))
        max_strain = max(max_strain,sample['max_absolute_axial_strain'])
        if data.ncon:
            error = 'Unexpected self-contact: free-strip constitutive benchmark no longer isolated'; break
        if net_force > 1e-9 or net_torque > 1e-7:
            error = 'Unbalanced instrument loads: cannot interpret as a static mechanical coupon'; break
        if max_strain > .01:
            error = 'Strain exceeds declared small-strain diagnostic domain (1 percent); no clipping applied'; break
        if (step+1)%every==0 or step==n-1:
            sample.update(tension_N=tension_now,end_transverse_N=transverse_now,applied_work_J=work)
            samples.append(sample)
    expected = spec.extension_m(tension)
    plateau = [s for s in samples if .10 <= s['time_s'] <= .15]
    mean = float(np.mean([s['contour_extension_m'] for s in plateau])) if plateau else None
    report = dict(component_test_only=True, full_task_complete=False, hardware_commands=False,
                  adhesion_or_peel_strength_test=False, initialized_bonds=False, carton_state_present=False,
                  material=spec.report(),mode=mode,timestep_s=timestep,mujoco_version=mujoco.__version__,
                  integrator='discrete',solver='Newton',diagexact=True,gravity=[0.,0.,0.],
                  actuator_count=model.nu,equality_count=model.neq,free_root=True,
                  applied_instrument='Balanced +/- axial loads at end segment COMs; +vertical load at both ends and equal total negative load split over two middle COMs. No prescribed position or torque compensation.',
                  tension_N=tension,per_end_transverse_N=transverse if mode=='three_point' else 0.,
                  expected_axial_extension_m=expected,axial_plateau_mean_extension_m=mean,
                  axial_plateau_relative_error=None if mean is None else abs(mean-expected)/expected,
                  peak_net_applied_force_N=max_force,peak_net_applied_torque_Nm=max_torque,
                  peak_center_of_mass_drift_m=max_com_drift,peak_end_span_rotation_radians=max_span_rotation,
                  peak_absolute_instrument_work_J=peak_absolute_work,final_instrument_work_J=work,
                  energy_passivity_validated=False,
                  energy_note='Work uses each applied force times its segment COM displacement. The small final signed residual and integration drift are reported, not a proof of passivity or exact energy conservation.',
                  max_absolute_axial_strain=max_strain,max_actual_contact_count=max_contact,
                  duration_s=float(data.time),elapsed_s=time.monotonic()-started,error=error,samples=samples,
                  limitation='Free-strip assumed-material mechanics only; no adhesive bond, gripper pickup, pressure, release, carton retention, or loaded robot contact audit is tested.')
    sources=[Path(__file__).resolve(),Path(__file__).resolve().parents[1]/'carton/folding_extensible_tape.py',Path(__file__).resolve().parents[1]/'carton/folding_tape.py']
    report['source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    out.mkdir(parents=True)
    (out/'sources').mkdir()
    for p in sources:(out/'sources'/p.name).write_bytes(p.read_bytes())
    (out/'scene.xml').write_text(xml)
    (out/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report


def compare(coarse, fine):
    if coarse['mode'] != fine['mode']:
        raise ValueError('Compare matched load protocols')
    time_fine=np.array([s['time_s'] for s in fine['samples']])
    time_coarse=np.array([s['time_s'] for s in coarse['samples']])
    metrics={}
    for field in ('contour_extension_m','center_sag_m','applied_work_J'):
        a=np.array([s[field] for s in coarse['samples']])
        b=np.interp(time_coarse,time_fine,[s[field] for s in fine['samples']])
        delta=a-b
        metrics[field]=dict(rms_difference=float(np.sqrt(np.mean(delta**2))),max_difference=float(np.max(np.abs(delta))))
    return dict(mode=coarse['mode'],coarse_timestep_s=coarse['timestep_s'],fine_timestep_s=fine['timestep_s'],
                complete=coarse['error'] is None and fine['error'] is None,
                comparison=metrics,scope='Numerical sensitivity of this free-strip instrument protocol only.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',required=True)
    args=parser.parse_args()
    root=Path(args.out)
    if root.exists():raise ValueError('Use a new output directory')
    reports={}
    for mode in ('axial','three_point'):
        for dt in (.000025,.000010):
            report=run_case(root/f'{mode}-{round(dt*1e6)}us',timestep=dt,mode=mode)
            reports[(mode,dt)]=report
            print(json.dumps({k:report[k] for k in ('mode','timestep_s','error','axial_plateau_relative_error','duration_s','elapsed_s')}),flush=True)
    summary=dict(component_test_only=True,full_task_complete=False,
                 comparisons=[compare(reports[(m,.000025)],reports[(m,.000010)]) for m in ('axial','three_point')])
    (root/'comparison.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
