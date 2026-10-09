import copy
import hashlib
import json
import pytest
from tools.refit_launch_preflight import CHECKS, digest, load_station, require_qualification, require_storage_headroom
from tools.train_refit_parallel import parse_args, station_arguments, robot_conditions


def test_parallel_cli_forwards_geometry_and_teacher_without_opaque_station_flags():
    args = parse_args(['--model-repo', 'local/model', '--dataset-repo', 'local/dataset',
        '--gpus', '8', '--base-spacing', '.3104', '--base-height', '-.02', '--base-to-table-edge', '.18', '--carton-inset', '0',
        '--prepare-near-degrees', '-25', '--left-press-along', '-.115', '--near-release-lift', '.07',
        '--full-grip-orientation', '--radius', '.135',
        '--arm-cap-ticks-s', '80', '--jaw-cap-ticks-s', '36', '--fps', '4',
        '--sample-dt', '.05', '--camera-lag', 'front=.2', 'left_wrist=.4', 'right_wrist=.4'])
    flags = station_arguments(args)
    for flag, value in [('--base-spacing', '0.3104'), ('--base-height', '-0.02'),
                         ('--base-to-table-edge', '0.18'), ('--carton-inset', '0.0'), ('--prepare-near-degrees', '-25.0'),
                         ('--radius', '0.135'), ('--left-press-along', '-0.115'), ('--near-release-lift', '0.07')]:
        assert flags.count(flag) == 1
        assert flags[flags.index(flag)+1] == value
    assert '--full-grip-orientation' in flags
    assert (args.arm_cap_ticks_s, args.jaw_cap_ticks_s, args.fps, args.sample_dt) == (80, 36, 4, .05)
    conditions = robot_conditions(args)
    assert conditions['station'] == flags
    assert conditions['camera_lag'] == ['front=.2', 'left_wrist=.4', 'right_wrist=.4']


def station():
    return dict(measured=True, measurement_source='test fixture only', frame='forward_so101_base_link',
                station_unchanged_since_measurement=True, desk_height_m=.7, carton_inset_m=.01,
                base_spacing_m=.3104, base_height_above_desk_m=.0291, base_to_table_edge_m=.18)


@pytest.mark.parametrize('change', [{'measured': False}, {'measurement_source': ''}, {'frame': 'pan_axis'},
    {'station_unchanged_since_measurement': False}, {'desk_height_m': .8}, {'carton_inset_m': -.01},
    {'base_spacing_m': float('nan')}, {'base_height_above_desk_m': None}, {'base_to_table_edge_m': float('inf')}])
def test_station_rejects_missing_or_ambiguous_measurements(tmp_path, change):
    p = tmp_path/'station.json'
    p.write_text(json.dumps(dict(station(), **change)))
    with pytest.raises(ValueError):
        load_station(p)


def test_qualification_requires_exact_source_station_arguments_and_intact_evidence(tmp_path):
    manifest = {'files': {'software/teacher.py': 'test-source-hash'}}
    args = ['--base-spacing', '.3104']
    log = tmp_path/'test-evidence.json'; log.write_text('{"passed": true}')
    report = dict(schema=3, scope='simulation_training_only', physical_execution_ready=False,
                  verify_placement_before_robot_execution=True, station_sha256=digest(station()), pipeline_args=args,
                  source_files=manifest['files'], **{k: True for k in CHECKS},
                  evidence={k: dict(path=log.name, sha256=hashlib.sha256(log.read_bytes()).hexdigest()) for k in CHECKS})
    require_qualification(report, station(), args, manifest, tmp_path)
    for key, value, match in [('schema', 1, 'legacy'), ('station_sha256', 'wrong', 'station'),
                             ('pipeline_args', [], 'arguments'), ('source_files', {}, 'manifest'),
                             ('capped_teacher_folds', False, 'capped_teacher')]:
        bad = copy.deepcopy(report); bad[key] = value
        with pytest.raises(ValueError, match=match):
            require_qualification(bad, station(), args, manifest, tmp_path)
    log.write_text('changed after qualification')
    with pytest.raises(ValueError, match='evidence'):
        require_qualification(report, station(), args, manifest, tmp_path)


@pytest.mark.parametrize('flags', [['--base-height', 'nan'], ['--base-spacing', '.9'],
    ['--arm-cap-ticks-s', '0'], ['--sample-dt', '-.1'], ['--left-press-along', '-.14'],
    ['--near-release-lift', '.01'], ['--near-release-lift', 'nan']])
def test_invalid_pipeline_parameters_fail_before_hub_calls(flags):
    with pytest.raises(SystemExit):
        parse_args(['--model-repo', 'local/model', '--dataset-repo', 'local/dataset', *flags])


def test_storage_gate_rejects_stale_quota_or_insufficient_space():
    observation = dict(source='test fixture', checked_at=1000, available_bytes=100)
    require_storage_headroom(observation, 80, 1100)
    for changed, required, now in [(observation, 101, 1100), (observation, 80, 2000),
        (observation, 80, 900), (observation, None, 1100),
        (dict(observation, available_bytes=float('nan')), 80, 1100)]:
        with pytest.raises(ValueError):
            require_storage_headroom(changed, required, now)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), .01, .11])
def test_near_flap_withdrawal_rejects_invalid_distance_before_motion(value):
    from carton.folding_retention import open_near_for_transfer
    with pytest.raises(ValueError, match='release lift'):
        open_near_for_transfer(None, None, release_lift=value)


@pytest.mark.parametrize('inset', [0., .01, .025])
def test_measured_inset_matches_rotated_carton_footprint(tmp_path, inset):
    import math
    from carton.folding_station import FoldingStation
    from carton.geometry import Box
    measured = dict(station(), carton_inset_m=inset)
    p = tmp_path/'station.json'; p.write_text(json.dumps(measured))
    assert load_station(p)['carton_inset_m'] == inset
    st = FoldingStation(.05, .2488353, inset, base_spacing=.273, table_size=(.5,.48))
    b = Box()
    for yaw in (-.02, 0., .02):
        y = st.table_edge_y + inset + b.length/2*abs(math.sin(yaw)) + b.width/2*abs(math.cos(yaw))
        footprint = st.carton_footprint((0., y), yaw)
        assert footprint['fully_on_table']
        assert min(v[1] for v in footprint['corners_xy_m']) - st.table_edge_y == pytest.approx(inset)


def test_target_placement_requires_explicit_owner_choice_and_future_verification(tmp_path):
    value = dict(station(), placement_mode='owner_will_match_target',
        station_unchanged_since_measurement=False, physical_placement_verified=False,
        target_placement_authorization='Owner will place carton at specified target',
        verify_placement_before_robot_execution=True)
    p = tmp_path/'station.json'; p.write_text(json.dumps(value))
    assert load_station(p)['placement_mode'] == 'owner_will_match_target'
    for change in ({'target_placement_authorization': ''}, {'physical_placement_verified': True},
                   {'verify_placement_before_robot_execution': False}):
        p.write_text(json.dumps(dict(value, **change)))
        with pytest.raises(ValueError, match='Target placement'):
            load_station(p)
