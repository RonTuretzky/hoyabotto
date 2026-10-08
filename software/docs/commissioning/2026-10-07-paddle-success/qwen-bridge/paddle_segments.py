"""Split model joint targets into pickup-profile segments; pure, no hardware access."""
# The pickup executor allows 341 ticks from the measured position. A longer move is split into 280-tick
# pieces so each later piece stays within 341 of wherever the previous one settled (up to 57 ticks off).
PADDLE_MAX_SEGMENT_TICKS = 341
PADDLE_SEGMENT_TICKS = 280


def paddle_target_segments(targets, state):
    """Split model targets into pickup-profile segments that move the requested joints together.

    A move whose joints all travel <=341 ticks is one segment (as in the pilot); a longer one becomes
    ceil(travel/280) segments in which every joint moves proportionally, so all segments name the same joints. A closing gripper runs after the other joints, alone, as the profile requires.
    Joints already within two ticks are dropped."""
    rows = state.get('rows', {})
    travel = {}
    for name, target in targets.items():
        start = rows.get(name, {}).get('Present_Position')
        if type(start) is not int:
            raise ValueError('Pickup start encoder unavailable for ' + name + '; refresh robot_get_state')
        if abs(target - start) > 2:
            travel[name] = (start, target)
    closing = {n: v for n, v in travel.items() if n.endswith('gripper') and v[1] < v[0]}
    moving = {n: v for n, v in travel.items() if n not in closing}
    segments = []
    for group in (moving, closing):
        if not group:
            continue
        spans = {n: 1 if abs(t - s) <= PADDLE_MAX_SEGMENT_TICKS else -(-abs(t - s) // PADDLE_SEGMENT_TICKS)
                 for n, (s, t) in group.items()}
        count = max(spans.values())
        # Every joint moves a share of its travel in every segment, so all segments name the same joints (the
        # owner refuses a path whose waypoints differ) and the arm moves as one; no leg exceeds 280 ticks.
        for i in range(1, count + 1):
            segments.append({n: s + round((t - s) * i / count) for n, (s, t) in group.items()})
    return segments


def expand_path(points, start):
    """Fill each waypoint to the full joint set (carrying unchanged joints forward from start) and split
    any leg longer than 341 ticks into <=280-tick pieces, so the executor can run the whole path continuously."""
    names = sorted(set(start) & {n for p in points for n in p})
    previous = {n: start[n] for n in names}
    out = []
    for point in points:
        target = {**previous, **{n: point[n] for n in point if n in previous}}
        pieces = max(1, max(0 if abs(target[n] - previous[n]) <= PADDLE_MAX_SEGMENT_TICKS else -(-abs(target[n] - previous[n]) // PADDLE_SEGMENT_TICKS) for n in names))
        for i in range(1, pieces + 1):
            out.append({n: previous[n] + round((target[n] - previous[n]) * i / pieces) for n in names})
        previous = target
    return out
