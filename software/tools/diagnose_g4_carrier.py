#!/usr/bin/env python3
"""Reproduce audited G4 empty-carrier gravity drops, never hardware motion."""
import argparse
import json
from pathlib import Path
import shutil
import sys
import traceback
import importlib.metadata
from dataclasses import asdict, replace

from planter.g4_carrier import DropCase, THRESHOLDS, cases, release_search_cases, release_refine_cases, guided_control_cases, guided_confirmation_cases, prepare_geometry, run_case, sha


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cad',type=Path,required=True)
    p.add_argument('--holder',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--no-render',action='store_true')
    p.add_argument('--single-contact',action='store_true',help='Explicit native single-contact-per-convex-pair comparison; same geometry/material/gates')
    p.add_argument('--holder-cover',choices=['partition','continuous'],default='partition',help='Explicit source-plane continuous holder wall cover comparison')
    p.add_argument('--case',action='append',help='Run named case(s); default all frozen cases')
    p.add_argument('--suite',choices=['full_height','full_height_matched','release_search','release_refine','guided_controls','guided_confirmation'],default='full_height')
    p.add_argument('--release-height-mm',type=float,default=None)
    p.add_argument('--contact-timeconstant-s',type=float,default=None)
    args=p.parse_args()
    if args.suite in ('guided_controls','guided_confirmation') and (args.release_height_mm is None or args.contact_timeconstant_s is None):
        p.error('Guided suites require explicit frozen release height and contact time constant')
    if args.suite=='full_height_matched' and args.contact_timeconstant_s is None:
        p.error('Matched full-height release requires explicit contact time constant')
    pool=(release_search_cases() if args.suite=='release_search' else
          release_refine_cases() if args.suite=='release_refine' else
          guided_control_cases(args.release_height_mm,args.contact_timeconstant_s) if args.suite=='guided_controls' else
          guided_confirmation_cases(args.release_height_mm,args.contact_timeconstant_s) if args.suite=='guided_confirmation' else
          [DropCase('full_height_matched',contact_timeconstant_s=args.contact_timeconstant_s)] if args.suite=='full_height_matched' else cases())
    if args.single_contact:pool=[replace(c,multiccd=False) for c in pool]
    selected=[c for c in pool if not args.case or c.name in args.case]
    if not selected:p.error('No matching cases')
    args.out.mkdir(parents=True,exist_ok=False)
    src=args.out/'sources';src.mkdir()
    source_files=[Path(__file__).resolve(),Path(__file__).resolve().parents[1]/'planter/g4_carrier.py',Path(__file__).resolve().parents[1]/'tests/test_g4_carrier.py',args.cad/'shared_tools.scad',args.cad/'README.txt',args.cad/'geometry-check.json',args.holder,args.cad/'carrier_PROTOTYPE.stl',args.cad/'guide_PROTOTYPE.stl']
    hashes={}
    for f in source_files:
        hashes[str(f)]=sha(f);shutil.copy2(f,src/f.name)
    manifest=dict(argv=sys.argv,source_hashes=hashes,suite=args.suite,holder_cover=args.holder_cover,contact_pipeline='native_single' if args.single_contact else 'native_multi',predeclared_cases=[asdict(c) for c in selected],
                  release_scope=('Full-height capture' if args.suite.startswith('full_height') else 'Positive-clearance release already constrained by guide; preceding robot positioning is unmodeled'),versions={n:importlib.metadata.version(n) for n in ['mujoco','numpy','scipy','trimesh','manifold3d','Pillow']},
                  thresholds=THRESHOLDS,assumptions=['2.5g carrier mass; uniform source-solid inertia.',
                  'Rigid fixed guide and holder, perfectly aligned nominal CAD assembly coordinates.',
                  'Coulomb sliding friction .4 nominal, 0 and 2 controls; not measured material properties.',
                  'Contact time constants are unmeasured numerical compliance hypotheses; no material-fit or physical force claim.',
                  'No robot, no grasp/release mechanics, no paper, trough, water or perception.',
                  'Only static source x=[-61,-29]mm corridor has collisions; full guide/holder visuals are decorative outside it.',
                  '0.1mm penetration and 2N force aborts are simulation diagnostics, not validated hardware safety thresholds.'])
    (args.out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    result=dict(status='incomplete',evidence_boundary='Passive gravity contact diagnostic with privileged reset/evaluator; not a trained policy or robot/physical success.',
                geometry_audit=str(args.out/'collision/geometry-audit.json'),trials=[],**manifest)
    try:
        files,audit=prepare_geometry(args.cad,args.holder,args.out/'collision',holder_cover=args.holder_cover)

        for case in selected:
            trial=run_case(args.cad,args.holder,files,case,args.out/case.name,render=not args.no_render)
            result['trials'].append(trial)
            print(json.dumps(dict(case=case.name,status=trial['status'],reason=trial['reason'],metrics=trial['metrics'])),flush=True)
            (args.out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        result['status']='diagnostic_complete'
        result['simulated_passes']=sum(t['status']=='pass' for t in result['trials'])
        result['policy_exported']=False
    except Exception as e:
        result['status']='blocked';result['error']=str(e)
        (args.out/'error.txt').write_text(traceback.format_exc())
        print(traceback.format_exc(),file=sys.stderr)
    (args.out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    return 0 if result['status']=='diagnostic_complete' else 1


if __name__=='__main__':raise SystemExit(main())
