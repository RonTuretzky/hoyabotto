#!/usr/bin/env python3
"""Build and audit full source-bound G4 collision assets; no hardware access."""
import argparse
import json
from pathlib import Path
import sys
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planter.g4_collision_assets import build_bundle, load_bundle, run_resting_diagnostic


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cad', type=Path, required=True)
    parser.add_argument('--trough', type=Path, required=True)
    parser.add_argument('--holder', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--holder-cover', choices=['continuous', 'angular', 'interface'], default='angular')
    parser.add_argument('--resting-duration', type=float, default=0., help='Optional privileged nominal resting check in seconds')
    args = parser.parse_args()
    try:
        result = build_bundle(args.cad, args.trough, args.holder, args.out,
                              holder_cover=args.holder_cover, progress=lambda s: print(s, flush=True))
        if result['status'] != 'complete':
            print(json.dumps([p for p in result['functional_probes'] if not p['passed']], indent=2))
            return 1
        load_bundle(args.out/'manifest.json')
        if args.resting_duration > 0:
            run_resting_diagnostic(args.out/'manifest.json', args.out/'resting', duration_s=args.resting_duration)
        print(f"Full bundle verified: {args.out/'manifest.json'}", flush=True)
        return 0
    except Exception:
        if args.out.exists():
            (args.out/'error.txt').write_text(traceback.format_exc())
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
