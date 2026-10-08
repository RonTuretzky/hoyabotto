"""Material coupon only: an instrument pulls a free tape end from a free plate.

No robot, dispenser pickup, folding or completed carton is represented here.
The upward test force is explicit and logged. Adhesion itself is passive.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as E

import mujoco
import numpy as np
from PIL import Image,ImageDraw

from carton.folding_tape import TapeSpec,add_tape


class ContactTimeline:
    """Step-resolution separation evidence, keeping later recontacts explicit."""
    def __init__(self):
        self.first_adhesive_contact_s = None
        self.last_adhesive_contact_s = None
        self.last_tensile_contact_s = None
        self.open_separation = None
        self.separations = []
        self.recontacts = 0

    def update(self, time, adhesive_contacts, tension):
        if adhesive_contacts:
            if self.first_adhesive_contact_s is None:
                self.first_adhesive_contact_s = time
            self.last_adhesive_contact_s = time
            if self.open_separation is not None:
                self.separations.append((self.open_separation, time))
                self.open_separation = None
                self.recontacts += 1
        elif self.first_adhesive_contact_s is not None and self.open_separation is None:
            self.open_separation = time
        if tension > 1e-9:
            self.last_tensile_contact_s = time

    def report(self, time):
        intervals = self.separations + ([(self.open_separation, time)]
                                       if self.open_separation is not None else [])
        sustained = [start for start, end in intervals if end - start >= .050 - 1e-10]
        return {'first_adhesive_contact_s': self.first_adhesive_contact_s,
                'last_adhesive_contact_s': self.last_adhesive_contact_s,
                'last_tensile_contact_s': self.last_tensile_contact_s,
                'first_separation_at_least_50ms_s': min(sustained) if sustained else None,
                'recontacts_after_complete_separation': self.recontacts,
                'complete_separation_intervals_s': intervals,
                'measurement': 'Detected adhesive/sample contacts at every solver step; no geometric bond is inferred when absent.'}


def run(args):
    out=Path(args.out).resolve()
    if out.exists():raise ValueError('Use a new output directory')
    if not np.isfinite(args.dt) or not .000001<=args.dt<=.002:raise ValueError('Invalid test timestep')
    if not np.isfinite(args.seconds) or args.seconds<=0:raise ValueError('Positive finite test duration required')
    if not np.isfinite(args.lift_distance) or args.lift_distance<=0:raise ValueError('Positive finite lift distance required')
    mode=getattr(args,'mode','peel');hold_force=getattr(args,'hold_force',.010)
    if mode not in ('peel','hold'):raise ValueError('Select peel or hold coupon instrument mode')
    if not np.isfinite(hold_force) or not 0<hold_force<=.15:raise ValueError('Hold load must be finite in (0, 0.15] N')
    source_paths=(Path(__file__).resolve(),Path(__file__).resolve().parents[1]/'carton/folding_tape.py')
    sources={str(path):path.read_bytes() for path in source_paths}
    spec=TapeSpec(adhesion_per_contact_N=args.adhesion)
    root=E.Element('mujoco',model='Passive tape coupon; no folding claim')
    option=E.SubElement(root,'option',timestep=str(args.dt),integrator=args.integrator,solver=args.solver,
                 iterations='120',tolerance='1e-10',cone='elliptic',gravity='0 0 -9.81')
    if getattr(args,'exact_impedance',False):E.SubElement(option,'flag',diagexact='enable')
    E.SubElement(root,'size',memory='32M')
    E.SubElement(E.SubElement(root,'visual'),'global',offwidth='960',offheight='540')
    world=E.SubElement(root,'worldbody')
    E.SubElement(world,'light',pos='0 -.2 .5',dir='0 0 -1')
    E.SubElement(world,'geom',name='table',type='plane',size='.2 .2 .01',rgba='.8 .82 .85 1',friction='.35 .002 .0001')
    plate=E.SubElement(world,'body',name='cardboard',pos='0 0 .003')
    E.SubElement(plate,'freejoint',name='cardboard_free')
    E.SubElement(plate,'geom',name='cardboard_contact',type='box',size='.060 .035 .003',mass='.050',rgba='.65 .4 .20 1',friction='.35 .002 .0001')
    start=np.array([-.04,0,.006+spec.thickness_m/2+.00008])
    body_name=add_tape(root,spec,start_m=start,flipped=args.flipped)
    pos=np.array([-.16,-.20,.17]);look=np.array([0,0,.015]);back=pos-look;back/=np.linalg.norm(back)
    right=np.cross([0,0,1],back);right/=np.linalg.norm(right);up=np.cross(back,right)
    E.SubElement(world,'camera',name='coupon',pos=' '.join(map(str,pos)),xyaxes=' '.join(map(str,np.r_[right,up])),fovy='38')
    out.mkdir(parents=True);E.indent(root);E.ElementTree(root).write(out/'scene.xml',encoding='unicode')
    (out/'sources').mkdir()
    for path,content in sources.items():(out/'sources'/Path(path).name).write_bytes(content)
    model=mujoco.MjModel.from_xml_path(str(out/'scene.xml'));data=mujoco.MjData(model)
    if model.nu or model.neq:raise ValueError('Coupon must not contain actuators or equality constraints')
    bid=model.body(body_name).id;vadr=model.joint('tape_free').dofadr[0]
    mujoco.mj_forward(model,data)
    plate_origin=data.body('cardboard').xpos.copy()
    frames=[];samples=[];max_translation=0.;max_tension=0.;max_tape_penetration=0.;error=None
    renderer=None if args.no_video else mujoco.Renderer(model,width=960,height=540)
    every=max(1,round(.05/args.dt));n=round(args.seconds/args.dt)
    max_force=0.;max_adhesive_contacts=0;last_time=0.;warning_time=None
    max_wrong_face_tension=0.
    timeline=ContactTimeline();instrument_work=0.;previous_end_z=float(data.body(bid).xpos[2])
    cardboard_geom=model.geom('cardboard_contact').id
    tape_geoms={model.geom(f'tape_{face}_{index}').id for index in range(spec.segments) for face in ('backing','adhesive')}
    adhesive_geoms={model.geom(f'tape_adhesive_{index}').id for index in range(spec.segments)}
    for step in range(n+1):
        t=float(data.time);goal=start[2]+min(args.lift_distance,.020*max(0,t-.2))
        force=0. if t<.2 else (hold_force if mode=='hold' else float(np.clip(10*(goal-data.body(bid).xpos[2])-.03*data.qvel[vadr+2],0,.15)))
        data.qfrc_applied[:]=0
        mujoco.mj_applyFT(model,data,np.array([0,0,force]),np.zeros(3),data.body(bid).xpos,bid,data.qfrc_applied)
        if step:mujoco.mj_step(model,data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or any(w.number for w in data.warning):
            error='Nonfinite state or MuJoCo warning';warning_time=t;break
        last_time=float(data.time);max_force=max(max_force,force)
        contacts=0;adhesive_contacts=0;tension=0.
        for ci,contact in enumerate(data.contact):
            a,b=contact.geom1,contact.geom2
            if cardboard_geom not in (a,b) or not (a in tape_geoms or b in tape_geoms):continue
            contacts+=1
            if a in adhesive_geoms or b in adhesive_geoms:adhesive_contacts+=1
            f=np.zeros(6);mujoco.mj_contactForce(model,data,ci,f)
            tension+=max(0.,-float(f[0]));max_tension=max(max_tension,max(0.,-float(f[0])))
            for gid,sign in ((contact.geom1,1),(contact.geom2,-1)):
                if gid in adhesive_geoms:
                    sticky_normal=-data.geom_xmat[gid].reshape(3,3)[:,2]
                    outward_normal=sign*contact.frame[:3]
                    if np.dot(sticky_normal,outward_normal)<-.5:
                        max_wrong_face_tension=max(max_wrong_face_tension,-float(f[0]))
            max_tape_penetration=max(max_tape_penetration,-float(contact.dist)*1000)
        # mj_step leaves its contact solution at the beginning of the step;
        # use that time, not the already-advanced qpos clock, for contact events.
        timeline.update(t,adhesive_contacts,tension)
        end_z=float(data.body(bid).xpos[2]);instrument_work+=force*(end_z-previous_end_z);previous_end_z=end_z
        movement=float(np.linalg.norm(data.body('cardboard').xpos-plate_origin)*1000)
        max_translation=max(max_translation,movement)
        max_adhesive_contacts=max(max_adhesive_contacts,adhesive_contacts)
        if step%every==0:
            sample={'time':float(data.time),'instrument_force_N':force,'end_z_mm':float(data.body(bid).xpos[2]*1000),
                    'plate_translation_mm':movement,'contacts':contacts,'adhesive_contacts':adhesive_contacts,'total_tensile_contact_force_N':tension}
            sample['contact_solution_time_s']=t
            samples.append(sample)
            if renderer:
                renderer.update_scene(data,camera='coupon');im=Image.fromarray(renderer.render()).convert('RGB');draw=ImageDraw.Draw(im)
                draw.rectangle((0,0,960,52),fill='white');draw.text((12,6),'MATERIAL COUPON ONLY - no robot folding or tape-placement success',fill='black')
                draw.text((12,27),f't={data.time:.2f}s  pull={force:.3f} N  adhesive contacts={adhesive_contacts}  flipped={args.flipped}',fill='black');frames.append(im)
        if max_tape_penetration>spec.backing_thickness_m*1000/2:
            error='Tape penetration exceeds half the backing thickness; one-sided adhesion is unverified';break
        if max_wrong_face_tension>1e-9:
            error='Adhesion detected on the covered face of an adhesive segment';break
    if renderer:renderer.close()
    if frames:
        frames[0].save(out/'coupon.gif',save_all=True,append_images=frames[1:],duration=50,loop=0)
        frames[-1].save(out/'coupon-after.png')
    report={'component_test_only':True,'full_task_complete':False,'hardware_commands':False,
            'configuration':vars(args),'material':spec.report(),'mujoco_version':mujoco.__version__,
            'error':error,'duration_s':last_time,'warning_time_s':warning_time,'max_plate_translation_mm':max_translation,
            'max_single_contact_tension_N':max_tension,'max_tape_penetration_mm':max_tape_penetration,
            'max_instrument_force_N':max_force,'max_adhesive_contacts':max_adhesive_contacts,
            'max_wrong_face_tension_N':max_wrong_face_tension,
            'contact_timeline':timeline.report(last_time),'instrument_work_J':instrument_work,
            'actuators':model.nu,'equality_constraints':model.neq,
            'explicit_test_force':('Vertical compliant peel at first tape segment, capped at 0.15 N; not robot execution.' if mode=='peel'
                                   else f'Constant {hold_force:g} N upward coupon load after 0.2s settling; not robot execution.'),
            'numerical_options_change_material_parameters':False,'folding_integration_enabled':False,
            'samples':samples,'sources':{p:hashlib.sha256(content).hexdigest() for p,content in sources.items()}}
    (out/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps({k:report[k] for k in ('error','duration_s','max_plate_translation_mm','max_single_contact_tension_N','max_tape_penetration_mm')},indent=2))
    print('Final sample',samples[-1])
    return report


def parser():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',required=True);parser.add_argument('--adhesion',type=float,default=.020)
    parser.add_argument('--flipped',action='store_true');parser.add_argument('--dt',type=float,default=.00005)
    parser.add_argument('--seconds',type=float,default=6.7)
    parser.add_argument('--lift-distance',type=float,default=.120)
    parser.add_argument('--no-video',action='store_true')
    parser.add_argument('--integrator',choices=['implicitfast','discrete'],default='implicitfast')
    parser.add_argument('--solver',choices=['Newton','CG'],default='Newton')
    parser.add_argument('--exact-impedance',action='store_true',help='Use exact constraint inertia diagonal; does not change tape material parameters')
    parser.add_argument('--mode',choices=['peel','hold'],default='peel')
    parser.add_argument('--hold-force',type=float,default=.010,help='Explicit upward load in N for --mode hold')
    return parser


if __name__=='__main__':
    run(parser().parse_args())
