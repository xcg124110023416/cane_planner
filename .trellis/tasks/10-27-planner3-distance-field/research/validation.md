# Implementation evidence

Actual recovery regression was RED before the change:
`ROS_MASTER_URI=http://127.0.0.1:11339 .../test_astar_edges
 --gtest_filter=AstarRecovery.SoftDistanceQueriesReuseSelectedGrid`:
hard=463, soft=159, grids=622; assertion grids<=hard+8 failed.
After change: hard=463, soft=159, grids=464; elapsed 0.558 ms vs 2.907 ms
in this small synthetic case. The counter assertion is deterministic; timing is
informational, not a machine-dependent pass threshold.

Production plan_env/path_searching/plan_manage build passed without warnings.
Affected suites: A* + field 21, static MPPI 23, FIRI 17, timed SAT 7 and
manager 30 = 98 enabled tests passed. Disabled opt-in benchmarks not run.
Original whole-cell edge, polygon, goal, sigma/grid-budget and heuristic
regressions retained, with map/source invalidation and interpolation tests added.

Original office-map/remembered-pedestrian route replay on owned master 11339:
new first A* 27.991 ms vs baseline 208.330 ms. Recovery A* 15 attempts all
complete: mean 40.612 ms, range 38.236–42.695 ms; inclusive soft 14.014 ms,
hard 20.284 ms; mean expanded nodes 5805 (baseline 3546 before timeout).
All candidate routes were subsequently rejected by same-hull full-footprint FIRI
with DYNAMIC_REFERENCE_BLOCKED / recovery=REBUILD_FAILED, rather than TIMEOUT.
Mean FIRI rebuild 0.750 ms. No accepted route replacement was forced. 42 motion
and 15 STOP decisions; final (3.718676,-4.844827), inside goal radius; zero
NO_PROGRESS. Baseline had 42 motion / 7 STOP decisions and 7 timeouts at 200 ms.
New searches run faster and thus can retry on more of the unchanged 10 Hz cycles.
Both runs resume after the pedestrian disappears; no successful bypass claim.

This is a same-configuration closed-loop comparison, not identical frozen
search inputs: observation timing, interpolated objective and path may differ.
The newly visible remaining limitation is COM-only recovery searching too close
to a pedestrian for the whole human-cane corridor certificate; changing recovery
geometry/strategy is outside this optimization. Existing manager SUCCESS vs
REBUILD_FAILED/EXPIRED/NO_PATH behavior and hard safety rules passed regression.

Recorder used `/tmp/cane_distance_field_replay.py reported`; logs/result prefix
`/tmp/cane-distance-field-`. Old `/tmp/cane-pedestrian-analysis-*` left intact.
Owned test launch stopped after replay; user master 11311 never restarted.
