"""Record fold demonstrations on a measured station: tools/record_fold_demos.py with the arm-base height,
base line to table edge, base spacing and the policy cameras taken from a measurement JSON. Simulation only.

Wraps the unchanged recorder: each trial process first calls carton.folding_station_measured.install(), so
the scripted controller (tools/diagnose_short_flap_brace.py, unchanged) builds the measured station and
the restaged `overhead`/`front` cameras. `--base-height`/`--base-to-table-edge` are set from the file.
The controller's own station camera, markers and contact rules are unchanged; its stage tuning was done at
120 mm base height, so check the success rate of a small batch before recording hundreds.

    PYTHONPATH=. python tools/record_measured_fold_demos.py --measurement station.json -- \
        --simulation-root .../gemma-xlerobot --out .../fold-demos/measured-01 --episodes 20 --workers 4
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import record_fold_demos as recorder  # noqa: E402

from carton.folding_station_measured import load_measurement  # noqa: E402

PRELUDE = '''
from carton.folding_station_measured import install as _install_measured
_install_measured({path!r})
'''


def measured_base_args(base_args, station):
    args = list(base_args)
    for flag, key in (('--base-height', 'base_height_above_table_m'),
                      ('--base-to-table-edge', 'base_line_to_table_edge_m')):
        value = f'{float(station[key]):.6g}'
        if flag in args:
            args[args.index(flag) + 1] = value
        else:
            args += [flag, value]
    return args


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if '--' in argv:
        split = argv.index('--')
        own, rest = argv[:split], argv[split + 1:]
    else:
        own, rest = argv, []
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--measurement', type=Path, required=True)
    ap.add_argument('--allow-unmeasured', action='store_true')
    args, unknown = ap.parse_known_args(own)
    rest = unknown + rest
    measurement = load_measurement(args.measurement)
    if not (measurement.get('measured') or measurement.get('model_derived')) and not args.allow_unmeasured:
        raise SystemExit('Measurement file is marked "measured": false; pass --allow-unmeasured for a check run')
    path = str(args.measurement.resolve())
    if measurement.get('station') is not None:
        recorder.BASE_ARGS = measured_base_args(recorder.BASE_ARGS, measurement['station'])
    # Each trial process must end with os._exit after writing its outputs: the controller's renderer is only
    # closed by FoldingSimulation.save(), which a stopped trial never reaches, and finalizing it at
    # interpreter shutdown segfaults (macOS crash dialogs). install() also closes renderers at exit.
    if 'os._exit(0)' not in recorder.TRIAL:
        raise SystemExit('tools/record_fold_demos.py TRIAL must end with os._exit(0) (commit 153f664)')
    recorder.TRIAL = PRELUDE.format(path=path) + recorder.TRIAL
    recorder.main(rest)
    out = Path(rest[rest.index('--out') + 1])
    (out / 'measurement.json').write_text(json.dumps({'file': path, 'measurement': measurement}, indent=1))


if __name__ == '__main__':
    main()
