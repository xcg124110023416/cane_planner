# Mixed scenario corridor debug notes

Date: 2026-06-15

This note records the bag-driven debugging pass for planner `3` in the
`mixed` pedestrian scenario. The immediate symptom was repeated
`CORRIDOR_INFEASIBLE` stop advice even when a route should exist.

## Diagnostic workflow

Two helper scripts were added under `plan_manage/scripts`:

```bash
rosrun plan_manage record_corridor_debug.py LABEL --duration 70
rosrun plan_manage analyze_corridor_debug.py /home/xcg/ws/records/<record_dir>
```

The analyzer writes:

```text
corridor_debug_samples.csv
corridor_debug_summary.txt
```

The summary joins corridor labels, MPPI debug metrics, stop reasons, odometry,
A* path length, MPC path length, and rosout stop logs.

## Root-cause chain

Initial failing bags showed two distinct static corridor failure modes:

1. A distant narrow point near the end of the local corridor made the full
   4 m corridor infeasible.
2. After truncation was introduced, later bags showed the cane could already be
   inside or immediately beside a narrow static band. In that case the narrow
   point was at `sw_s=0.00` or very close to the corridor start, so stopping
   prevented MPPI from moving out of the narrow band.

A representative failing sample before the final fix:

```text
reason=CORRIDOR_INFEASIBLE
corridor[w=0.120 st=1 dyn=0 off=0.00]
narrow[s=0.00 x=-1.63 y=-2.09]
debug[valid=0.770 corridor_reject=0.0 static_reject=46.0 dynamic_reject=0.0]
```

This is important because the MPPI sample validity was high. The failure was
not "no local motion exists"; it was the supervisory corridor stop being too
strict at the corridor start.

## Main implementation changes

### A* global guide clearance

A* now has stricter static clearance controls:

- `astar/min_clearance`
- `astar/traversable_radius`

The new traversable-radius check rejects an A* node if a small disk around the
node is not traversable. This keeps the global guide away from narrow static
bands that would later make DWC and MPPI fail.

Current launch defaults:

```xml
<arg name="astar_min_clearance" default="0.35"/>
<arg name="astar_traversable_radius" default="0.25"/>
<param name="astar/w_clearance" value="3.0" type="double"/>
<param name="astar/clearance_sigma" value="1.2" type="double"/>
```

### Dynamic walking corridor static handling

DWC now supports:

- a single path-following corridor by default;
- static centerline optimization;
- static width shrinking;
- static prefix truncation for distant narrow points;
- start grace for narrow regions at the current cane position.

Current relevant launch defaults:

```xml
<param name="dwc/min_half_width" value="0.25" type="double"/>
<param name="dwc/static_truncate_enable" value="true" type="bool"/>
<param name="dwc/static_min_feasible_length" value="0.8" type="double"/>
<param name="dwc/static_truncate_backoff" value="0.2" type="double"/>
<param name="dwc/static_start_grace_length" value="0.4" type="double"/>
```

The start grace only applies to the prefix near the current cane position. It
does not make the full corridor ignore static obstacles. Narrow points farther
than the grace distance still make the corridor infeasible or trigger static
truncation.

### MPPI hard reject split

Dynamic risk-field hard reject and dynamic geometry hard reject were separated:

- `mpc/dynamic_hard_reject_enable`
- `mpc/dynamic_collision_hard_reject_enable`

The current mixed-scenario tuning keeps geometry collision hard reject, while
the risk field is handled mostly as cost.

## Regression bags

After the final DWC start-grace change, three mixed-scenario bags were analyzed.

| Label | Stop reasons | Static infeasible | Width ok | MPC path |
| --- | --- | --- | --- | --- |
| `mixed_start_grace04_test` | `OK: 35` | `st=1: 0 / 35` | `35 / 35` | `13.698 m` |
| `mixed_regression_01` | `OK: 38` | `st=1: 0 / 38` | `38 / 38` | `14.901 m` |
| `mixed_regression_02` | `OK: 36` | `st=1: 0 / 36` | `36 / 36` | `14.087 m` |

All three bags had:

```text
rosout_stop_logs: 0
First non-OK stop contexts: none
First ROS stop logs: none
```

`dyn>0` remains common in the labels, which is expected in the mixed pedestrian
scenario. Dynamic occupancy should influence risk, yield, and cost. It should
not by itself make the static walking corridor infeasible.

## Verification commands

The code was verified with:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin build path_searching plan_manage --no-deps -DCMAKE_BUILD_TYPE=Release
catkin test path_searching --no-deps --catkin-make-args run_tests
catkin_test_results build/path_searching
```

Observed result:

```text
path_searching and plan_manage build succeeded
20 tests, 0 errors, 0 failures, 0 skipped
```

## Remaining caution

This is a regression result for the current lightweight simulation mixed
scenario. It should not be treated as a proof that all maps, goals, or hardware
cases are solved. If a new stuck case appears, record a bag first and compare:

- stop reason distribution;
- `st`, `dyn`, `tr`, and width counts;
- `sw_s` location for the first non-OK context;
- MPPI `valid`, `static_reject`, `dynamic_reject`, and `corridor_reject`;
- A* path length vs. MPC path length.
