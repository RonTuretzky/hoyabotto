"""Read-only: relay round trip and clock offset between this (chat) Mac and the robot API.

Calls only robot_get_execution (no motor, camera or state change) through the chat pilot's
authenticated client, then projects whether the AprilTag registration mover fits the link:
owner rows must be 0..0.75 s old and camera frames 0..1.0 s old on THIS Mac's clock, and a run
must finish inside its time budget. Those limits are not adjustable here; if the projection fails,
synchronize both clocks (sudo sntp -sS time.apple.com on each Mac) or improve the link.

  cd software && PYTHONPATH=. python tools/measure_robot_link.py --pilot-root "$PILOT" [--count 20]

The robot clock at each call is the API's own reading: status time + status_age_s (computed by the
API on the robot Mac when it served the call). Offset = robot clock - midpoint of this Mac's send and
receive times; its error is at most half the round trip, so the tool reports offset +/- rtt/2.
"""
from __future__ import annotations

import argparse
import importlib
import json
import statistics
import sys
import time
from pathlib import Path


def measure(robot, count=20, clock=time.time, pause=0.05, sleep=time.sleep):
    samples = []
    for i in range(count):
        sent = clock()
        payload = robot.call('robot_get_execution', {})
        received = clock()
        result = payload.get('result') if isinstance(payload, dict) and payload.get('ok') is True else None
        if not isinstance(result, dict) or type(result.get('time')) not in (int, float) or type(result.get('status_age_s')) not in (int, float):
            raise RuntimeError(f'robot_get_execution unavailable or without time/status_age_s: {str(payload)[:300]}')
        robot_now = result['time'] + result['status_age_s']
        rows = result.get('rows') or {}
        stamps = [r['captured_at'] for r in rows.values() if type(r.get('captured_at')) in (int, float)]
        samples.append({'round_trip_s': received-sent, 'offset_s': robot_now-(sent+received)/2,
                        'owner_status_age_seen_s': received-result['time'],
                        'oldest_row_age_seen_s': received-min(stamps) if stamps else None,
                        'owner_publish_age_s': result['status_age_s']})
        if i+1 < count:
            sleep(pause)
    return samples


def summarize(samples, limits=None, camera_latency_s=0.05):
    from carton.servo.tag_calibration import registration_timing
    rtt = [s['round_trip_s'] for s in samples]
    offset = [s['offset_s'] for s in samples]
    seen = [s['owner_status_age_seen_s'] for s in samples]
    rows = [s['oldest_row_age_seen_s'] for s in samples if s['oldest_row_age_seen_s'] is not None]
    median_rtt, median_offset = statistics.median(rtt), statistics.median(offset)
    publish = statistics.median(s['owner_publish_age_s'] for s in samples)
    projection = registration_timing(median_rtt, limits, clock_offset_s=median_offset,
                                     camera_latency_s=camera_latency_s, owner_publish_age_s=max(publish, .1))
    worst = registration_timing(max(rtt), limits, clock_offset_s=median_offset,
                                camera_latency_s=camera_latency_s, owner_publish_age_s=max(publish, .1))
    out = {'calls': len(samples), 'motor_writes': 0,
           'round_trip_s': {'median': median_rtt, 'min': min(rtt), 'max': max(rtt)},
           'clock_offset_robot_minus_chat_s': {'median': median_offset, 'uncertainty_s': min(rtt)/2,
                                               'spread_s': max(offset)-min(offset)},
           'median_rtt_plus_abs_offset_s': median_rtt+abs(median_offset),
           'owner_status_age_seen_by_mover_s': {'min': min(seen), 'max': max(seen)},
           'oldest_row_age_seen_by_mover_s': {'max': max(rows)} if rows else None,
           'registration_projection': projection, 'registration_projection_at_max_rtt': worst}
    problems = []
    limit = projection['status_age_limit_s']
    if min(seen) < 0:
        problems.append('This Mac\'s clock is behind the robot: owner timestamps look like the future and the mover refuses them')
    if max(seen) > limit or (rows and max(rows) > limit):
        problems.append(f'Owner rows reach this Mac older than {limit} s')
    for name, check in (('time budget', 'fits_time_budget'), ('frame age', 'fits_frame_age'), ('status age', 'fits_status_age')):
        if not projection[check]:
            problems.append(f'Projected registration exceeds its {name} at the median round trip')
    out['problems'] = problems
    out['verdict'] = 'LINK_FITS_REGISTRATION' if not problems else 'FIX_LINK_OR_CLOCKS_FIRST'
    if problems:
        out['remedies'] = ['sudo sntp -sS time.apple.com on both Macs, then measure again',
                           'or run the calibration CLI on a host with a shorter path to the robot API',
                           'a longer run may set limits.max_seconds (<=300) in tag-calibration.json; freshness limits stay as they are']
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--pilot-root', type=Path, required=True)
    p.add_argument('--count', type=int, default=20)
    p.add_argument('--camera-latency-s', type=float, default=0.05,
                   help='OAK capture-to-serve latency assumed for the frame-age projection')
    args = p.parse_args()
    if not 3 <= args.count <= 200:
        p.error('--count must be 3..200')
    sys.path.insert(0, str(args.pilot_root.resolve()))
    robot = importlib.import_module('chat_server').Robot(args.pilot_root/'.private/robot.json')
    report = summarize(measure(robot, args.count), camera_latency_s=args.camera_latency_s)
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report['verdict'] == 'LINK_FITS_REGISTRATION' else 1


if __name__ == '__main__':
    raise SystemExit(main())
