"""Offline alternative flap order: attempt the near major before either short.

Same empty free carton, passive resistance, station, CAD and motion gates.
Success here means only a held near panel, never a complete folding cycle.
"""
import argparse
import hashlib
import json
from pathlib import Path

from carton.folding_sim import FoldingSimulation
from carton.folding_station import FoldingStation
from carton.folding_material import CartonMaterial
from carton.folding_solver import FoldingSolver
from carton.folding_controller import FoldingController
from carton.folding_cascade import press_near_over_short
from carton.folding_markers import BOX_MARKERS
from tools.simulate_bimanual_folding import PixelPort


def run(args):
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    sources = [Path(__file__), *sorted(Path('carton').glob('folding*.py')),
               Path('tools/simulate_bimanual_folding.py')]
    (out/'sources').mkdir()
    hashes = {}
    for path in sources:
        data = path.read_bytes()
        (out/'sources'/path.name).write_bytes(data)
        hashes[str(path)] = hashlib.sha256(data).hexdigest()
    BOX_MARKERS[24] = ('box_tag_floor', [.08, 0, .0038], [1,0,0,0,1,0])
    BOX_MARKERS[25] = ('box_tag_floor_center', [0, 0, .0038], [1,0,0,0,1,0])
    sim = FoldingSimulation(args.simulation_root, out,
        station=FoldingStation(.06, .15, .01, table_marker_xy=(-.5,.55), backup_table_marker_xy=(.45,.70)),
        material=CartonMaterial(), solver=FoldingSolver.friction(), width=1280, height=720,
        offset=(0,0), yaw=0, initial_right_roll=1.5,
        initial_arm_targets={'left':[-.20,-.18,.30], 'right':[.20,-.18,.30]},
        initial_flaps={'short_left':.1,'short_right':.1,'long_far':-.1,'long_near':-.1})
    port = PixelPort(sim, seed=args.seed, record=args.video, camera='station')
    controller = FoldingController(port, rear_cart=True)
    result = dict(simulation_only=True, full_task_complete=False, hardware_commands=False,
                  source_sha256=hashes, configuration={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()})
    try:
        sim.capture('Initial free carton: long-flap-first experiment')
        sim.move({}, .4, 'Settle', capture=False)
        result['near_held'] = press_near_over_short(sim, controller, capture=args.video,
            majors_first=True, along=args.along, radius=args.radius, tilt=args.tilt,
            pre_out=.02, pre_up=0.)
    except ValueError as error:
        result['error'] = str(error)
    result.update(angles=sim.truth_angles(), motion=dict(sim.motion_stats),
                  readings=port.readings, time=float(sim.data.time),
                  checks=getattr(controller,'contact_progress_checks', []))
    sim.capture('Partial near hold' if result.get('near_held') else 'STOP: major-first diagnostic')
    result['physics'] = sim.save('folding')
    (out/'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ('readings','source_sha256','checks','near_held','physics')}))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--simulation-root', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--along', type=float, default=0.)
    p.add_argument('--radius', type=float, default=.105)
    p.add_argument('--tilt', type=float, default=-.65)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--video', action='store_true')
    run(p.parse_args())
