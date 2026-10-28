"""Analyze captured odometry, planning transactions and timed-plant evidence.

Usage: python3 analyze.py trace.json [trace.json ...] [--assert-smooth]
Assertions are replay acceptance criteria, not a hard real-time guarantee.
"""
import collections
import json
import math
from pathlib import Path
import statistics
import sys


def stats(values):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return None
    return dict(count=len(values), mean=statistics.mean(values),
                p95=values[int((len(values)-1)*.95)], maximum=values[-1])


def analyze(filename):
    trace = json.loads(Path(filename).read_text())
    odom = trace['odom']
    moves = [(b['t']-a['t'], math.hypot(b['x']-a['x'], b['y']-a['y']), b)
             for a, b in zip(odom, odom[1:])
             if math.hypot(b['x']-a['x'], b['y']-a['y']) > 1e-6]
    moving_gaps = [b[2]['t']-a[2]['t'] for a, b in zip(moves, moves[1:])]
    goal = trace.get('goal')  # historical baseline did not capture route metadata
    distance = lambda p: math.hypot(p['x']-goal[0], p['y']-goal[1])
    transactions = [t['data'] for t in trace['transactions']]
    execution = [e['data'] for e in trace.get('execution', [])]
    return dict(
        start=trace.get('start'), goal=goal,
        movement_intervals_seconds=stats([m[0] for m in moves]),
        between_moving_updates_seconds=stats(moving_gaps),
        movement_gaps_over_150ms=sum(g > .15 for g in moving_gaps),
        displacement_metres=stats([m[1] for m in moves]),
        critical_transaction_seconds=stats([t[18] for t in transactions]),
        total_callback_seconds=stats([t[19] for t in transactions]),
        execution_validation_seconds=stats([e[7] for e in execution]),
        actual_advance_seconds=stats([e[4] for e in execution]),
        stops=dict(collections.Counter(s['text'] for s in trace['stops'])),
        astar_publications=len(trace['astar']),
        astar_intervals_seconds=stats([b['t']-a['t'] for a, b in
                                      zip(trace['astar'], trace['astar'][1:])]),
        preview_publications=len(trace.get('preview', [])),
        nonempty_pedestrian_frames=sum(bool(p['positions']) for p in trace.get('pedestrians', [])),
        minimum_goal_distance_metres=min(map(distance, odom)) if goal else None,
        final_goal_distance_metres=distance(odom[-1]) if goal else None, final_pose=odom[-1],
        execution_frames=len(execution),
    )


if __name__ == '__main__':
    results = {Path(p).name: analyze(p) for p in sys.argv[1:] if not p.startswith('--')}
    print(json.dumps(results, indent=2, allow_nan=False))
    if '--assert-smooth' in sys.argv:
        for result in results.values():
            assert result['movement_gaps_over_150ms'] == 0
            assert result['displacement_metres']['maximum'] < .08
            assert result['final_goal_distance_metres'] is not None
            assert result['final_goal_distance_metres'] < .2
            assert result['execution_frames'] > 100
