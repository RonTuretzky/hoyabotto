"""Offline bare-gripper reach and policy-shape audit; no robot connections."""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path

import yaml

from carton.bimanual import fold_reach_screen, molmo_checkpoint_contract
from carton.geometry import Box, Stance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,default=Path('profiles/carton-v0.yaml'))
    parser.add_argument('--norm-stats',type=Path)
    parser.add_argument('--norm-tag')
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if args.out.exists():parser.error('Use a new report path to preserve prior evidence')
    raw=yaml.safe_load(args.profile.read_text())['carton']
    box=Box(**{name:raw[name+'_m'] for name in ('length','width','height','flap')},mass_kg=raw['mass_kg'])
    layout=raw['stance']
    stance=Stance(setback=layout['setback_m'],spacing=layout['spacing_m'],height=layout['height_m'],paddle=0)
    report=fold_reach_screen(box,stance)
    report['source_profile']=str(args.profile.resolve())
    report['setback_comparison']=[{'setback_m':v,
        'worst_reach_margin_m':min(p['reach_margin_m'] for p in fold_reach_screen(box,replace(stance,setback=v))['phases'])}
        for v in (0,.02,.03,.04,.06)]
    if args.norm_stats:
        if not args.norm_tag:parser.error('--norm-tag required with --norm-stats')
        report['checkpoint_contract']=molmo_checkpoint_contract(json.loads(args.norm_stats.read_text()),args.norm_tag)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='phases'},indent=2))


if __name__=='__main__':main()
