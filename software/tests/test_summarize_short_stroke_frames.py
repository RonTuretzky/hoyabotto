"""The fold-frame summary reads recorded commands only and keeps partial results partial."""
import json

import numpy as np

from tools.summarize_short_stroke_frames import summarize_run, summarize


def write_run(tmp_path, *, stroke_rows, probe=True):
    run = tmp_path/'run'
    run.mkdir()
    commands = [dict(stage='normal_approach', seq=1, event_time=10., sources={}, robot_substeps={})]
    for index, (target, actual) in enumerate(stroke_rows, 2):
        commands.append(dict(stage='stroke', seq=index, event_time=10.+index,
            sources={side: dict(world_from_box=np.eye(4).tolist(), measured_degrees=-15.+index)
                     for side in ('left', 'right')},
            robot_substeps={side: dict(sensor_target_world=target, actual_point_world=actual)
                            for side in ('left', 'right')}))
    result = dict(simulation_only=True, full_task_complete=False)
    if probe:
        result['partial_short_probe'] = dict(contact_policy='tangent_deadband_v4', fault='stalled',
            stage='bounded simultaneous short-fold probe', contact_commands=commands,
            motion=dict(max_horizontal_translation_mm=.4))
    (run/'result.json').write_text(json.dumps(result))
    return run


def test_summary_projects_recorded_gaps_onto_the_current_fold_frame(tmp_path):
    # Right panel at -13 degrees: fold ~ (-cos13, 0, sin13), hinge = y.
    rows = [([0., .002, 0.], [0., 0., 0.]), ([-.001, 0., 0.], [0., 0., 0.])]
    summary = summarize_run(write_run(tmp_path, stroke_rows=rows))
    right = summary['sides']['right']
    assert right['hinge_gap']['max_mm'] == 2.
    assert right['fold_gap']['min_mm'] == 0.
    assert right['fold_gap']['max_mm'] > .97
    assert right['measured_degrees_first'] == -13. and right['measured_degrees_last'] == -12.
    assert summary['stroke_commands'] == 2 and summary['approach_commands'] == 1
    assert summary['fault'] == 'stalled' and not summary['bounded_target_verified']
    assert not summary['full_task_complete']
    assert summary['stroke_contacts'] is None


def test_missing_probe_or_empty_stroke_is_reported_not_invented(tmp_path):
    absent = summarize_run(write_run(tmp_path, stroke_rows=[], probe=False))
    assert absent['partial_short_probe'] == 'absent'
    empty = summarize([write_run(tmp_path/'other', stroke_rows=[])]) if (tmp_path/'other').mkdir() is None else None
    assert empty[0]['stroke_commands'] == 0
    assert empty[0]['sides']['left']['fold_gap'] is None
