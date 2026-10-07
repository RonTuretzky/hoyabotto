"""Controlled single-overlap adhesion coupon, never free-carton retention.

An unbonded strip settles onto one fixed substrate by gravity. The 180 mm
candidate and a 50 mm local-interface coupon both retain the 5 mm cells.
An external instrument then pulls its free tail. The substrate is an explicit
laboratory fixture: its reaction is measured, not hidden as carton support.
No strip qpos is assigned during execution, and no bond/equality is initialized.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import gzip
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from carton.folding_extensible_tape import ExtensibleTapeSpec, add_extensible_tape


SETTLE_S = .2
RAMP_S = .1
END_S = .4
OVERLAP_M = .0405


class ReturnedContactTimeline:
    """Bind observations to calls, without asserting a discrete solver stage.

    A change between successive returned arrays spans both calls. Repeated
    empty arrays do not prove continuous absence between their observations.
    """
    def __init__(self):
        self.first_contact = self.last_contact = self.last_tension = None
        self.previous_step = None
        self.open_absence = None
        self.closed_absences = []

    def update(self, start, end, contacts, tension):
        if (not math.isfinite(start) or not math.isfinite(end) or start < 0 or end <= start
                or self.previous_step is not None and start != self.previous_step[1]):
            raise ValueError('Returned contacts require finite, ordered, contiguous step intervals')
        interval = [start, end]
        if contacts:
            if self.first_contact is None:
                self.first_contact = interval
            self.last_contact = interval
            if self.open_absence is not None:
                self.open_absence['recontact_observation_step_s'] = interval
                self.closed_absences.append(self.open_absence)
                self.open_absence = None
        elif self.first_contact is not None:
            if self.open_absence is None:
                self.open_absence = dict(first_empty_observation_step_s=interval,
                                         previous_contact_observation_step_s=self.last_contact)
            self.open_absence['last_empty_observation_step_s'] = interval
        if tension > 1e-9:
            self.last_tension = interval
        self.previous_step = interval

    @staticmethod
    def describe_absence(absence):
        first = absence['first_empty_observation_step_s']
        last = absence['last_empty_observation_step_s']
        previous = absence['previous_contact_observation_step_s']
        return dict(**absence,
                    contact_to_empty_observation_transition_bracket_s=[previous[0], first[1]],
                    minimum_span_between_empty_observations_s=max(0., last[0]-first[1]))

    def has_empty_observation_span(self, seconds):
        return (self.open_absence is not None and
                self.describe_absence(self.open_absence)['minimum_span_between_empty_observations_s'] >= seconds-1e-10)

    def report(self):
        absences = [self.describe_absence(a) for a in self.closed_absences]
        if self.open_absence is not None:
            absences.append(self.describe_absence(self.open_absence))
        sustained = [a for a in absences if a['minimum_span_between_empty_observations_s'] >= .05-1e-10]
        return dict(first_contact_observation_step_s=self.first_contact,
                    last_contact_observation_step_s=self.last_contact,
                    last_tensile_observation_step_s=self.last_tension,
                    first_persistent_empty_observation_transition_bracket_s=(sustained[0]['contact_to_empty_observation_transition_bracket_s'] if sustained else None),
                    empty_observation_sequences=absences,
                    recontact_observation_count=len(self.closed_absences),
                    measurement='Raw contact arrays returned immediately after each mj_step, bound to that call interval. No instantaneous discrete-solver stage or continuous inter-observation absence is certified.')


def load_at(t, shear=.6, opening=.06):
    u = min(1., max(0., (t-SETTLE_S)/RAMP_S))
    return np.array([shear, 0., opening]) * u*u*(3.-2.*u)


def make_model(timestep, *, adhesion=True, strip_length=.180):
    if type(timestep) not in (int, float) or timestep not in (.000025, .000010):
        raise ValueError('Only the declared 25 and 10 microsecond cases are supported')
    if type(adhesion) is not bool:
        raise ValueError('Adhesion switch is a boolean negative control only')
    if type(strip_length) not in (int, float) or strip_length not in (.180, .050):
        raise ValueError('Use the 180 mm candidate or explicit 50 mm local-interface coupon')
    spec = ExtensibleTapeSpec()
    spec = replace(spec, tape=replace(spec.tape, length_m=strip_length, segments=round(strip_length/.005)))
    if not adhesion:
        spec = replace(spec, tape=replace(spec.tape, adhesion_per_contact_N=0.))
    root = ET.Element('mujoco', model='Fixed-substrate single-overlap adhesion coupon only')
    option = ET.SubElement(root, 'option', timestep=str(timestep), integrator='discrete',
                           solver='Newton', iterations='120', tolerance='1e-10',
                           cone='elliptic', gravity='0 0 -9.81')
    ET.SubElement(option, 'flag', diagexact='enable')
    ET.SubElement(root, 'size', memory='32M')
    world = ET.SubElement(root, 'worldbody')
    # The top ends at x=-49.5 mm; the strip starts at -90 mm. No other
    # support, table, pad, or kinematic constraint can hold the free tail.
    ET.SubElement(world, 'geom', name='coupon_substrate', type='box',
                  pos='-.0995 0 .003', size='.05 .035 .003',
                  friction='.35 .002 .0001', rgba='.65 .4 .2 1')
    add_extensible_tape(root, spec, start_m=[-.09, 0., .006+spec.tape.thickness_m/2+.00008])
    xml = ET.tostring(root, encoding='unicode')
    model = mujoco.MjModel.from_xml_string(xml)
    if model.nu or model.neq:
        raise ValueError('Coupon must contain no actuator/equality')
    return model, xml, spec


def contact_evidence(model, data, substrate, adhesives):
    """Read arrays returned immediately after mj_step, never replayed qpos.

    This coupon does not certify which instantaneous discrete-solver state
    the returned arrays represent.
    """
    records = []
    force = np.zeros(3)
    moment = np.zeros(3)
    adhesive_count = 0
    tension = penetration = wrong_face = 0.
    unexpected = []
    for ci, contact in enumerate(data.contact):
        a, b = int(contact.geom1), int(contact.geom2)
        local = np.zeros(6)
        mujoco.mj_contactForce(model, data, ci, local)
        # MuJoCo force acts on geom2 from geom1; contact frame axes are rows.
        world = np.asarray(contact.frame).reshape(3, 3).T @ local[:3]
        if substrate not in (a, b):
            unexpected.append([model.geom(a).name, model.geom(b).name])
            continue
        tape_geom = b if a == substrate else a
        on_tape = world if a == substrate else -world
        force += on_tape
        moment += np.cross(contact.pos, on_tape)
        adhesive = tape_geom in adhesives
        adhesive_count += int(adhesive)
        if adhesive:
            tension += max(0., -float(local[0]))
            sticky_normal = -data.geom_xmat[tape_geom].reshape(3, 3)[:, 2]
            outward_normal = (1. if tape_geom == a else -1.) * contact.frame[:3]
            if np.dot(sticky_normal, outward_normal) < -.5:
                wrong_face = max(wrong_face, -float(local[0]))
        penetration = max(penetration, -float(contact.dist))
        records.append(dict(geom=model.geom(tape_geom).name, distance_m=float(contact.dist),
                            position_m=contact.pos.tolist(), force_on_tape_N=on_tape.tolist(),
                            normal_force_N=float(local[0]), adhesive=adhesive))
    return dict(contacts=records, adhesive_contacts=adhesive_count,
                tensile_contact_sum_N=tension, interface_force_on_tape_N=force.tolist(),
                interface_moment_about_origin_Nm=moment.tolist(), penetration_m=penetration,
                wrong_face_tension_N=max(0., wrong_face), unexpected_contacts=unexpected)


def run_case(out, *, timestep, adhesion=True, shear=.6, opening=.06, strip_length=.180):
    out = Path(out)
    if out.exists():
        raise ValueError('Use a new evidence directory')
    for name, value in [('shear', shear), ('opening', opening)]:
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError('Positive finite '+name+' load required')
    model, xml, spec = make_model(timestep, adhesion=adhesion, strip_length=strip_length)
    data, measure = mujoco.MjData(model), mujoco.MjData(model)
    body_ids = np.array([model.body(f'tape_{i}').id for i in range(spec.tape.segments)])
    axial = np.array([int(model.joint(f'tape_axial_{i}').qposadr[0]) for i in range(1, spec.tape.segments)])
    substrate = model.geom('coupon_substrate').id
    adhesives = {model.geom(f'tape_adhesive_{i}').id for i in range(spec.tape.segments)}
    masses = model.body_mass[body_ids]
    mass = float(masses.sum())
    gravity_force = mass * model.opt.gravity
    started = time.monotonic()
    sources = [Path(__file__).resolve(), Path(__file__).resolve().parents[1]/'carton/folding_extensible_tape.py',
               Path(__file__).resolve().parents[1]/'carton/folding_tape.py']
    frozen = {str(path): path.read_bytes() for path in sources}
    out.mkdir(parents=True)
    (out/'sources').mkdir()
    for path, content in frozen.items():
        (out/'sources'/Path(path).name).write_bytes(content)
    (out/'scene.xml').write_text(xml)

    def observe():
        # This separate data object refreshes positions/velocities only. It is
        # never used for force inference and cannot affect the evolving state.
        measure.qpos[:] = data.qpos
        measure.qvel[:] = data.qvel
        mujoco.mj_kinematics(model, measure)
        mujoco.mj_comPos(model, measure)
        mujoco.mj_comVel(model, measure)
        mujoco.mj_subtreeVel(model, measure)
        xyz = measure.xipos[body_ids].copy()
        center = np.average(xyz, axis=0, weights=masses)
        return dict(xyz=xyz, center=center,
                    momentum=mass*measure.subtree_linvel[body_ids[0]].copy(),
                    extension=float(np.sum(data.qpos[axial])),
                    max_strain=float(np.max(np.abs(data.qpos[axial]))/spec.axial_cell_length_m))

    mujoco.mj_forward(model, data)
    initial_contacts = int(data.ncon)
    before = initial = observe()
    timeline = ReturnedContactTimeline()
    samples = []
    work = 0.
    impulse = np.zeros(3)
    max_penetration = max_wrong = max_strain = max_impulse_residual = 0.
    max_contact_count = 0
    error = outcome = None
    settled = None
    steps = 0
    stream_path = out/'contact-steps.jsonl.gz'
    every = round(.001/timestep)
    with gzip.open(stream_path, 'wt') as stream:
        for step in range(round(END_S/timestep)):
            t = float(data.time)
            applied = load_at(t, shear, opening)
            data.xfrc_applied[:] = 0.
            data.qfrc_applied[:] = 0.
            data.xfrc_applied[body_ids[-1], :3] = applied
            # xfrc_applied is an actual force at this segment's COM. No servo,
            # position target, reset, artificial latch, or end clamp exists.
            mujoco.mj_step(model, data)
            steps += 1
            evidence = contact_evidence(model, data, substrate, adhesives)
            timeline.update(t, float(data.time), evidence['adhesive_contacts'], evidence['tensile_contact_sum_N'])
            after = observe()
            interface = np.array(evidence['interface_force_on_tape_N'])
            net_force = applied+gravity_force+interface
            net_moment = (np.cross(before['xyz'][-1], applied)
                          + np.cross(before['center'], gravity_force)
                          + np.array(evidence['interface_moment_about_origin_Nm']))
            impulse += net_force*timestep
            impulse_residual = after['momentum']-initial['momentum']-impulse
            max_impulse_residual = max(max_impulse_residual, float(np.linalg.norm(impulse_residual)))
            work += float(applied @ (after['xyz'][-1]-before['xyz'][-1]))
            max_penetration = max(max_penetration, evidence['penetration_m'])
            max_wrong = max(max_wrong, evidence['wrong_face_tension_N'])
            max_strain = max(max_strain, after['max_strain'])
            max_contact_count = max(max_contact_count, evidence['adhesive_contacts'])
            record = dict(step=step, step_started_s=t, step_ended_s=float(data.time),
                          timestep_s=timestep, instrument_force_N=applied.tolist(),
                          instrument_body=model.body(int(body_ids[-1])).name, instrument_point_at_step_start_m=before['xyz'][-1].tolist(),
                          gravity_force_N=gravity_force.tolist(), gravity_point_at_step_start_m=before['center'].tolist(),
                          returned_contact_plus_applied_force_N=net_force.tolist(), mixed_stage_moment_diagnostic_Nm=net_moment.tolist(),
                          returned_force_rectangular_sum_momentum_difference_Ns=impulse_residual.tolist(),
                          tail_com_m=after['xyz'][-1].tolist(), root_com_m=after['xyz'][0].tolist(),
                          contour_extension_m=after['extension'], instrument_work_J=work,
                          **evidence)
            stream.write(json.dumps(record, separators=(',', ':'), allow_nan=False)+'\n')
            if step % every == 0 or step == round(END_S/timestep)-1:
                samples.append({k: v for k, v in record.items() if k != 'contacts'})
            if t < SETTLE_S <= data.time+1e-12:
                settled = {k: v for k, v in record.items() if k != 'contacts'}
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or any(w.number for w in data.warning):
                error = 'Nonfinite state or MuJoCo warning'
            elif max_penetration > spec.tape.backing_thickness_m/2:
                error = 'Penetration exceeds unchanged half-backing-thickness coupon gate'
            elif max_wrong > 1e-9:
                error = 'Covered-face adhesion exceeds unchanged coupon gate'
            elif evidence['unexpected_contacts']:
                error = 'Unexpected strip self-contact; single-interface inference refused'
            elif max_strain > .01:
                error = 'Axial strain exceeds existing 1 percent constitutive diagnostic domain'
            before = after
            if error:
                outcome = 'invalid_numerical_or_contact_evidence'
                break
            if t >= SETTLE_S and settled and not settled['adhesive_contacts']:
                outcome = 'failed_to_form_overlap_before_loading'
                break
            if t >= SETTLE_S and timeline.has_empty_observation_span(.05):
                outcome = 'no_returned_contacts_over_at_least_50ms_observation_span'
                break
    if outcome is None:
        outcome = 'retained_contact_to_end_not_a_strength_certificate'
    if steps and (not samples or samples[-1]['step'] != record['step']):
        samples.append({k: v for k, v in record.items() if k != 'contacts'})
    report = dict(component_test_only=True, full_task_complete=False, hardware_commands=False,
                  robot_or_carton_present=False, free_box_retention_validated=False,
                  material_strength_validated=False, initialized_bonds=False,
                  tape_prescribed_motion=False, fixed_substrate_fixture=True,
                  actuator_count=model.nu, equality_count=model.neq, initial_contact_count=initial_contacts,
                  material=spec.report(), overlap_m=OVERLAP_M, free_tail_length_m=strip_length-OVERLAP_M,
                  strip_length_m=strip_length, local_overlap_only=strip_length == .050,
                  model='Single fixed flat substrate; free tape settles from 80 um gap in real gravity. One tail COM receives explicit force; no other support.',
                  load_protocol=dict(settle_s=SETTLE_S, ramp_s=RAMP_S, hold_s=END_S-SETTLE_S-RAMP_S,
                                     target_shear_N=shear, target_opening_N=opening,
                                     note='One overlap loaded once. Tail bending, gravity and acceleration are measured; this is not exact two-short-flap kinematics.'),
                  timestep_s=timestep, integrator='discrete', solver='Newton', diagexact=True,
                  mujoco_version=mujoco.__version__, adhesion_enabled=adhesion,
                  error=error, outcome=outcome, duration_s=float(data.time), actual_steps=steps,
                  returned_contact_timeline=timeline.report(), settled_sample=settled,
                  final_sample=samples[-1] if samples else None, samples=samples,
                  max_adhesive_contacts=max_contact_count, max_penetration_m=max_penetration,
                  max_wrong_face_tension_N=max_wrong, max_absolute_axial_strain=max_strain,
                  max_returned_force_rectangular_sum_momentum_difference_Ns=max_impulse_residual, instrument_work_J=work,
                  discrete_contact_sampling_semantics_verified=False,
                  force_diagnostic_note='Contact forces/positions are raw returned arrays. Force and moment sums combine them with explicit inputs/start-of-step application points; rectangular force sums versus momentum are descriptive diagnostics, not certified simultaneous balance or impulse residuals.',
                  energy_passivity_validated=False,
                  energy_note='Prior free-strip signed-work residuals are unresolved. No positive strength inference is made; gross bond loss and timestep sensitivity are recorded independently.',
                  actual_contact_step_evidence=dict(path=stream_path.name, sha256=hashlib.sha256(stream_path.read_bytes()).hexdigest(),
                                                    format='gzip_jsonl', rows=steps, expected_start_time_s=0., expected_end_time_s=float(data.time),
                                                    timing_schema='returned-contact-step-interval-v1',
                                                    note='Raw contact arrays returned immediately after mj_step, bound only to [step_started_s, step_ended_s]. Their instantaneous discrete-solver stage is not verified. No qpos-only force replay. Coupon-only evidence, not the folding contact scorer.'),
                  source_sha256={p: hashlib.sha256(content).hexdigest() for p, content in frozen.items()},
                  elapsed_s=time.monotonic()-started)
    (out/'result.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--dt', type=float, choices=[.000025, .000010], default=.000025)
    parser.add_argument('--no-adhesion', action='store_true')
    parser.add_argument('--strip-length', type=float, choices=[.180, .050], default=.180)
    args = parser.parse_args()
    report = run_case(args.out, timestep=args.dt, adhesion=not args.no_adhesion, strip_length=args.strip_length)
    print(json.dumps({k: report[k] for k in ('outcome', 'error', 'duration_s', 'elapsed_s', 'max_penetration_m')}, indent=2))


if __name__ == '__main__':
    main()
