"""Offline checks tying paid refit qualification to the exact station and bundle.

No Hub calls or robot adapters. A previous run's passing booleans alone do not
qualify new geometry, camera placement, teacher settings or source files.
"""
from __future__ import annotations
import hashlib
import json
import math
from pathlib import Path

CHECKS = ('target_scene_camera_contract_passed', 'tests_passed', 'capped_teacher_folds', 'replay_under_robot_limits_passed',
          'dataset_conversion_passed', 'local_training_smoke_passed', 'frozen_bundle_smoke_passed')


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def load_station(path):
    path = Path(path)
    if not path.is_file():
        raise ValueError(f'Missing measured station: {path}; need mounting height and desk setback')
    station = json.loads(path.read_text())
    if station.get('measured') is not True or not station.get('measurement_source'):
        raise ValueError('Station needs measured=true and a measurement_source; model assumptions do not qualify')
    if station.get('frame') != 'forward_so101_base_link':
        raise ValueError('Station frame must be forward_so101_base_link; convert pan-axis setback to base-origin setback')
    mode = station.get('placement_mode', 'measured_current')
    if mode == 'owner_will_match_target':
        if (not station.get('target_placement_authorization')
                or station.get('physical_placement_verified') is not False
                or station.get('verify_placement_before_robot_execution') is not True):
            raise ValueError('Target placement needs owner authorization and a pending physical execution gate')
    elif mode != 'measured_current' or station.get('station_unchanged_since_measurement') is not True:
        raise ValueError('Confirm unchanged measured placement or explicitly authorize a target placement')
    if station.get('desk_height_m') != .7:
        raise ValueError('Current refit requires a 700 mm desk; reconcile scene before launch')
    for key, low, high in [('base_spacing_m', .1, .5), ('base_height_above_desk_m', -.05, .3),
                           ('base_to_table_edge_m', 0, .5), ('carton_inset_m', 0, .1)]:
        value = station.get(key)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not low <= value <= high):
            raise ValueError(f'Invalid measured station {key}; expected {low}..{high} metres')
    return station


def require_qualification(report, station, pipeline_args, manifest, evidence_root):
    if report.get('schema') != 3:
        raise ValueError('Need fresh schema-3 qualification; legacy validation cannot qualify target placement')
    if (report.get('scope') != 'simulation_training_only'
            or report.get('physical_execution_ready') is not False
            or report.get('verify_placement_before_robot_execution') is not True):
        raise ValueError('Training qualification must retain the physical placement and calibration gate')
    if report.get('station_sha256') != digest(station):
        raise ValueError('Qualification station mismatch')
    if report.get('pipeline_args') != pipeline_args:
        raise ValueError('Qualification pipeline arguments mismatch')
    if not manifest.get('files') or report.get('source_files') != manifest['files']:
        raise ValueError('Qualification source/asset manifest mismatch')
    for check in CHECKS:
        if report.get(check) is not True:
            raise ValueError(f'Qualification failed: {check}')
        evidence = report.get('evidence', {}).get(check, {})
        path = Path(evidence_root) / evidence.get('path', '')
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != evidence.get('sha256'):
            raise ValueError(f'Missing or changed qualification evidence: {check}')
    return report


def require_storage_headroom(observation, required_bytes, now):
    """A source-bound projection needs a recent account-wide storage observation."""
    if not observation.get('source'):
        raise ValueError('Storage observation needs an account quota source')
    checked = observation.get('checked_at')
    available = observation.get('available_bytes')
    for value in (checked, available, required_bytes, now):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('Storage observation and projection must be finite numbers')
    if not 0 <= now - checked <= 900:
        raise ValueError('Storage observation must be refreshed within 15 minutes before launch')
    if required_bytes <= 0 or available < required_bytes:
        raise ValueError('Insufficient private storage headroom for projected run artifacts')
