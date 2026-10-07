"""Independent pusher handling measurements in the complete rigid G4 station.

The public evaluator replays every physics step. Its mechanical result remains
separate from original-source contact qualification and full assembly. Recorded
commands never count as grip, motion, support, release, or paper insertion.
"""
from dataclasses import asdict, dataclass
from itertools import groupby
import math
import mujoco
import numpy as np

from planter.g4_assembly_score import (AssemblyThresholds, _contact_force_on,
    _runs, audit_model, validate_trace)
from planter.g4_contact_codec import decode_contacts


@dataclass(frozen=True)
class PusherThresholds:
    """Provisional simulation assumptions; no measured hardware force limit."""
    loaded_n: float = .005
    acquisition_s: float = .100
    retained_fraction: float = .95
    unloaded_gap_s: float = .010
    rigid_slip_m: float = .005
    hold_s: float = 2.
    hold_clearance_m: float = .020
    setdown_s: float = .040
    supported_clearance_m: float = .001
    released_hold_s: float = 1.
    released_drift_m: float = .002
    released_speed_m_s: float = .002
    released_angular_speed_rad_s: float = .02


PHASES = ('approach_above','lower_outside','approach_handle','close','lift',
          'hold','lower','release','disengage','withdraw','released_hold')


def pusher_vertices(model):
    """Actual compiled original visual mesh, transformed into the body frame."""
    gid=model.geom('pusher_visual').id
    if model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH or model.geom_bodyid[gid] != model.body('pusher').id:
        raise ValueError('Pusher source visual mesh missing or attached to wrong body')
    mid=model.geom_dataid[gid]; start=model.mesh_vertadr[mid]; count=model.mesh_vertnum[mid]
    rotation=np.empty(9);mujoco.mju_quat2Mat(rotation,model.geom_quat[gid])
    return np.einsum('ij,kj->ik',model.mesh_vert[start:start+count].astype(float),rotation.reshape(3,3))+model.geom_pos[gid]


def score_pusher_motion(model, states, *, thresholds=PusherThresholds()):
    """Measure coherent states; use evaluate_station_pusher for a replayed verdict."""
    t=thresholds; failures=[]
    result=dict(passed=False,requires_native_trace_audit=True,thresholds=asdict(t),
        full_assembly_success=False,physical_success=False)
    if not states:return dict(result,failure_reasons=['empty_trace'])
    try:
        if model.nflex:raise ValueError('unreviewed_flex_contact_schema')
        bid=model.body('pusher').id;grip=model.body('right_gripper_link').id
        moving=model.body('right_moving_jaw_so101_v1_link').id
        rest=model.geom('transferred_pusher_rest').id
        if model.geom_type[rest]!=mujoco.mjtGeom.mjGEOM_BOX or model.geom_bodyid[rest]!=0:
            raise ValueError('required_static_box_rest_missing')
        rest_R=np.empty(9);mujoco.mju_quat2Mat(rest_R,model.geom_quat[rest]);rest_R=rest_R.reshape(3,3)
        if not np.allclose(rest_R[:,2],[0,0,1],atol=1e-10):raise ValueError('rest_top_not_horizontal')
        vertices=pusher_vertices(model);dt=float(model.opt.timestep)
        if not 0<dt<=t.unloaded_gap_s:raise ValueError('invalid_sampling_timestep')
        joint=model.joint('pusher_free');veladr=int(joint.dofadr[0])
        checked=[]
        for i,s in enumerate(states):
            if not np.isfinite(s['time_s']) or (i and abs(s['time_s']-states[i-1]['time_s']-dt)>1e-9):
                raise ValueError('missing_or_irregular_samples')
            pose=np.asarray(s['xpos']);rot=np.asarray(s['xmat']);qvel=np.asarray(s['qvel'])
            if (pose.shape!=(model.nbody,3) or rot.shape!=(model.nbody,3,3) or qvel.shape!=(model.nv,)
                    or not np.isfinite(np.r_[pose.ravel(),rot.ravel(),qvel]).all()):
                raise ValueError('nonfinite_or_malformed_measurement')
            world=np.einsum('ij,kj->ik',vertices,rot[bid])+pose[bid]
            local=np.einsum('ij,jk->ik',world-model.geom_pos[rest],rest_R)
            clearance=float(local[:,2].min()-model.geom_size[rest,2])
            com=rest_R.T@(pose[bid]+rot[bid]@model.body_ipos[bid]-model.geom_pos[rest])
            margin=float(np.min(model.geom_size[rest,:2]-np.abs(com[:2])))
            jaw_forces={grip:np.zeros(3),moving:np.zeros(3)};jaw_count={grip:0,moving:0}
            upward=0.;robot_support=0.;other_support=0.;unintended=0.
            nonjaw_support={}
            for c in s['contacts']:
                force,normal,other=_contact_force_on(model,c,{bid})
                if force is None:continue
                load=float(np.linalg.norm(force));other_body=int(model.geom_bodyid[other])
                if other==rest:upward+=float(force[2])
                elif load>t.loaded_n:other_support+=load
                if model.body(other_body).name.startswith(('left_','right_')):robot_support+=load
                if other_body in jaw_forces and c['wrench'][0]>t.loaded_n:
                    jaw_forces[other_body]+=normal*c['wrench'][0];jaw_count[other_body]+=1
            opposed=bool(all(jaw_count.values()) and np.dot(jaw_forces[grip],jaw_forces[moving])<0)
            for stream in ('contacts','step_contacts'):
                stream_load=0.;stream_support={}
                for c in s[stream]:
                    load=float(np.linalg.norm(c['wrench'][:3]))
                    force,normal,other=_contact_force_on(model,c,{bid})
                    if force is not None and int(model.geom_bodyid[other]) not in (grip,moving):
                        name=model.geom(other).name
                        stream_support[name]=stream_support.get(name,0.)+load
                    if load<=t.loaded_n:continue
                    bodies=[int(model.geom_bodyid[c[k]]) for k in ('geom1','geom2')]
                    names=[model.body(b).name for b in bodies]
                    sides=[next((a for a in ('left','right') if name.startswith(a+'_')),None) for name in names]
                    intended=(bid in bodies and any(b in (grip,moving) for b in bodies))
                    if any(sides) and not intended and not (sides[0] and sides[0]==sides[1]):stream_load+=load
                unintended=max(unintended,stream_load)
                # Max across streams preserves their distinct sample times;
                # summing them would count the same interaction twice.
                for name,load in stream_support.items():
                    nonjaw_support[name]=max(nonjaw_support.get(name,0.),load)
            checked.append(dict(time=s['time_s'],phase=s['phase'],world=world,
                relative=np.einsum('ij,jk->ik',world-pose[grip],rot[grip]),loaded=opposed,
                support=upward>t.loaded_n and abs(clearance)<=t.supported_clearance_m and margin>=0,
                clearance=clearance,margin=margin,upward=upward,robot_support=robot_support,
                other_support=other_support,unintended=unintended,
                nonjaw_support=nonjaw_support,
                speed=float(np.linalg.norm(qvel[veladr:veladr+3])),angular=float(np.linalg.norm(qvel[veladr+3:veladr+6]))))
    except (KeyError,ValueError,IndexError,TypeError) as error:
        return dict(result,failure_reasons=['invalid_pusher_evidence:'+str(error)])
    phases=[p for p,_ in groupby(s['phase'] for s in checked)]
    while phases and phases[0] in ('initial','pusher_initial_settle','pusher_settle'):phases.pop(0)
    expected=['pusher_'+p for p in PHASES]
    if phases not in (expected,[p for p in expected if p!='pusher_disengage']):failures.append('pusher_phase_order_or_completion')
    indices={p:[i for i,s in enumerate(checked) if s['phase']=='pusher_'+p] for p in PHASES}
    result['observed_phases']=phases
    if any(not indices[p] for p in ('close','lift','hold','lower','release','withdraw','released_hold')):
        return dict(result,failure_reasons=sorted(set(failures+['required_pusher_phases_missing'])))
    initial=[s for s in checked if s['phase'] in ('initial','pusher_initial_settle','pusher_settle')]
    if any(s['loaded'] for s in initial):failures.append('pusher_initialized_in_loaded_grasp')
    window=indices['close'][-math.ceil(t.acquisition_s/dt-1e-9):]
    mask=[checked[i]['loaded'] for i in window]
    if len(window)*dt<t.acquisition_s-1e-9 or np.mean(mask)<t.retained_fraction or _runs([not v for v in mask],dt)>t.unloaded_gap_s+1e-9:
        failures.append('grip_not_established_before_lift')
    setdown=None;streak=0
    for i in indices['lower']:
        streak=streak+1 if checked[i]['support'] else 0
        if streak*dt>=t.setdown_s-1e-9:setdown=i-streak+1;break
    if setdown is None:failures.append('supported_setdown_not_observed_before_release')
    end=indices['release'][0] if setdown is None else setdown
    carried=list(range(window[0],end+1));mask=[checked[i]['loaded'] for i in carried]
    rest_name=model.geom(rest).name
    lift_off=next((i for i in carried if i>=indices['lift'][0]
        and checked[i]['clearance']>t.supported_clearance_m
        and checked[i]['nonjaw_support'].get(rest_name,0.)<=t.loaded_n),None)
    if lift_off is None:failures.append('unsupported_lift_off_not_observed')
    forbidden=[]
    for i in carried:
        # Before actual lift-off and from sustained setdown onwards, the
        # declared rest is expected. Other fixtures never establish a grip.
        rest_allowed=(lift_off is None or i<lift_off or (setdown is not None and i>=setdown))
        load=sum(v for name,v in checked[i]['nonjaw_support'].items() if name!=rest_name or not rest_allowed)
        forbidden.append(load)
    if max(forbidden,default=0.)>t.loaded_n:failures.append('nonjaw_support_during_carry')
    gap=_runs([not v for v in mask],dt);fraction=float(np.mean(mask))
    if fraction<t.retained_fraction or gap>t.unloaded_gap_s+1e-9:failures.append('grasp_lost_before_setdown')
    anchor=next((i for i in window if checked[i]['loaded']),window[-1])
    drift=max(float(np.linalg.norm(checked[i]['relative']-checked[anchor]['relative'],axis=1).max()) for i in carried if i>=anchor)
    if drift>t.rigid_slip_m:failures.append('whole_pusher_grasp_drift')
    hold=indices['hold'];released=indices['released_hold']
    if len(hold)*dt<t.hold_s-1e-9:failures.append('carried_hold_too_short')
    if not all(checked[i]['loaded'] and checked[i]['clearance']>=t.hold_clearance_m for i in hold):failures.append('lift_or_hold_retention_failed')
    if len(released)*dt<t.released_hold_s-1e-9:failures.append('released_hold_too_short')
    if not all(checked[i]['support'] and checked[i]['other_support']<=t.loaded_n and checked[i]['robot_support']<=t.loaded_n for i in released):
        failures.append('release_not_free_and_supported')
    free_drift=max(float(np.linalg.norm(checked[i]['world']-checked[released[0]]['world'],axis=1).max()) for i in released)
    max_speed=max(checked[i]['speed'] for i in released);max_angular=max(checked[i]['angular'] for i in released)
    if free_drift>t.released_drift_m or max_speed>t.released_speed_m_s or max_angular>t.released_angular_speed_rad_s:failures.append('released_pusher_not_settled')
    unintended=max(s['unintended'] for s in checked)
    if unintended>t.loaded_n:failures.append('unintended_loaded_arm_contact')
    return dict(result,passed=not failures,failure_reasons=sorted(set(failures)),
        acquisition_loaded_fraction=float(np.mean([checked[i]['loaded'] for i in window])),
        retained_fraction=fraction,longest_unloaded_gap_s=gap,maximum_rigid_grasp_drift_m=drift,
        minimum_hold_clearance_m=min(checked[i]['clearance'] for i in hold),
        measured_lift_off_time_s=None if lift_off is None else checked[lift_off]['time'],
        supported_setdown_time_s=None if setdown is None else checked[setdown]['time'],
        maximum_forbidden_carry_support_n=max(forbidden,default=0.),
        forbidden_carry_support_duration_s=sum(v>t.loaded_n for v in forbidden)*dt,
        forbidden_carry_support_impulse_n_s=sum(v for v in forbidden if v>t.loaded_n)*dt,
        nonjaw_support_inventory={name:dict(peak_resultant_n=max(s['nonjaw_support'].get(name,0.) for s in checked),
            loaded_duration_s=sum(s['nonjaw_support'].get(name,0.)>t.loaded_n for s in checked)*dt,
            loaded_impulse_n_s=sum(s['nonjaw_support'].get(name,0.) for s in checked if s['nonjaw_support'].get(name,0.)>t.loaded_n)*dt)
            for name in sorted({name for s in checked for name in s['nonjaw_support']})},
        released_hold_s=len(released)*dt,released_rigid_drift_m=free_drift,
        released_max_speed_m_s=max_speed,released_max_angular_speed_rad_s=max_angular,
        maximum_unintended_arm_resultant_n=unintended)


def evaluate_station_pusher(model, rows):
    """Replay raw rows and retain the tool/rest contacts absent from guide scoring."""
    data=mujoco.MjData(model);states=[];pusher=model.body('pusher').id
    relevant={g for g,b in enumerate(model.geom_bodyid) if b==pusher or model.body(b).name.startswith(('left_','right_'))}
    def observed():
        for raw in rows:
            row=decode_contacts(raw) if 'contact_codec' in raw else raw
            mujoco.mj_setState(model,data,np.asarray(row['integration_state']),mujoco.mjtState.mjSTATE_INTEGRATION)
            mujoco.mj_kinematics(model,data)
            states.append(dict(phase=row['phase'],time_s=row['time_s'],xpos=data.xpos.copy(),
                xmat=data.xmat.copy().reshape(-1,3,3),qpos=data.qpos.copy(),qvel=data.qvel.copy(),
                **{k:[c for c in row[k] if c['geom1'] in relevant or c['geom2'] in relevant] for k in ('contacts','step_contacts')}))
            yield row
    trace=validate_trace(model,observed());trace.pop('states');model_audit=audit_model(model)
    motion=score_pusher_motion(model,states)
    return dict(scorer='g4_station_pusher_v2',trace_audit=trace,model_audit=model_audit,motion_audit=motion,
        mechanical_measurements_passed=trace['passed'] and model_audit['passed'] and motion['passed'],
        source_contact_qualification='Separate original-source loaded-surface audit required',
        policy_export_eligible=False,full_assembly_success=False,physical_success=False,states=states)
