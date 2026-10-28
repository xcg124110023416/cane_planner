# Planner 3 stuttering diagnosis, 2026-10-28

The synchronous full global A* blocks the 100ms FSM. This is reproduced without
RViz and without safety STOP. Changing the .2s scheduling period previously
improved route publication cadence, but did not establish uniform plant updates.
No production code, launch defaults or safety contracts changed in this diagnosis.

## Reproduction and feedback loop

The latest user launch was `sim_kin_replan.launch planner:=3
use_pedestrians:=true pedestrian_scenario:=dwcg_crossing`. Its last route was from
(-6.846, 4.728, yaw=-.157) to (4.139, -4.681). Sanitized rosout showed early
Reanchored intervals near240ms, falling to100ms later, and successful full-search
durations198.5/180.9/131.7ms. No STOP was logged for that route.

An unattended owned-master11339 replay uses the same office, start and goal;
RViz and hardware controller are omitted. The saved editor pedestrian is reproduced
via a private config: start(2.713644,4.219024), yaw=-3.124397, speed2m/s,
duration2s, disappear, start_on_goal. The user-owned11311 master was not changed.
Capture monotonic receipt times and source header stamps on `/sim_odom`, plus
the existing33-field `/mpc/transaction_diagnostics`, metrics, route and STOP topics.

The executable feedback loop is saved as `analyze.py` plus the captured topic
traces. From this directory:

```bash
python3 analyze.py baseline.json --check
# RED: STUTTER: 15 moving updates exceed150ms, max242.9ms
python3 analyze.py sparse-global.json --check
# GREEN: 0 gaps exceed150ms, max103.9ms
```

The150ms threshold measures irregular position updates relative to the100ms FSM,
not GUI FPS. Ignore idle/arrival repeats and initial goal setup. The baseline
captured45 executions and44 movement intervals; all STOP reasons were OK.
Further minimization removing pedestrians still reproduces14 long intervals.
These are measured replays of one scene, not a general deadline guarantee.

## Hypotheses and one-variable probes

Ranked hypotheses were shared before probes: synchronous full-search blocking;
FIRI/MPPI load; visualization/preview blocking. The independent contrasts retain
every other default, the same map and start/goal.

| Replay | Moving gaps >150ms | Active-cycle p95 | Observation |
|---|---:|---:|---|
| Default .2s | 15/44 | 241.4ms | No STOP; A* repeatedly reaches200ms timeout |
| Only global period5s | 0/44 | 23.7ms | One initial timeout; following motion stable near100ms |
| Only preview disabled | 14/44 | 238.2ms | Search still dominates; one separate timed execution STOP |
| Only pedestrians disabled | 14/44 | 233.8ms | No STOP; problem survives without dynamic SAT or callbacks |

Sparse-search mode is a diagnostic contrast, not a proposed default or a safety
replacement. The disabled-preview run has a478ms maximum movement gap including
one TIMED_EXECUTION_COLLISION; do not attribute that entire gap to computation.
The baseline/no-pedestrian runs independently demonstrate computation stutter.
None of these experiments measures RViz rendering load; they show it is not
necessary to produce this symptom.

## Attribution

Default replay mean FIRI1.97ms, MPPI5.43ms (max14.23ms), full search107.84ms
per attempted search, maximum200.00ms. The first nine attempts hit timeout.
Early hard human-cane translation/turn certificates cost about150ms per search;
full static-grid snapshot about16ms, soft cost evaluation about11ms. Search
expands about25,000 states and calls transition hard checks about50,000 times in
those early attempts. Timing categories are not an exhaustive partition.

`updateAndPublishConvexCorridorLocked` invokes `refreshGlobalRouteLocked` inline,
before FIRI/MPPI; `refreshGlobalRouteLocked` allows .2s search. Pose-heading A*
checks the whole physical human-cane shape, occupied cells and <=5degree turn
samples. These costs exceed the100ms FSM even when the search times out and the
previous reference is retained. Because the .2s refresh deadline has elapsed by
the end of a long transaction, the next cycle often starts another search.
Do not treat this as the cost of an ordinary point-grid A* or claim ESDF scalar
queries are the dominant cost.

Observed accepted-observation callback lock waits reach90ms, whereas planner
entry-lock wait is negligible in the baseline. This is a consequence of the long
transaction, not evidence that callbacks originally caused the stall. Measured
post-critical work averages2.26ms; disabling preview preserves the long cycles.

The lightweight plant compounds the visible effect: `mpcSimStep` calls
`LFPC::updateOneStep` then publishes its final pose once. Consecutive measured
poses move0.36-0.40m with gaps ranging from about9ms to243ms. It does not execute
the internal step samples according to wall time. This existing timing limitation
was not changed here. Removing global stalls alone cannot prove continuous motion
or resolve LFPC/model-time versus simulation-time semantics.

## Follow-up boundary

Preserve the requested periodic full-global route, full footprint, original
source/receipt freshness, timed MPPI and execution checks. Optimize the measured
geometry/snapshot work and investigate decoupling search from the control cycle.
An asynchronous design needs a reviewed snapshot/result-acceptance contract:
current pose/map/source validation and normal FIRI/MPPI/execution authorization
before replacement. Do not publish pending routes or silently reuse obsolete
pedestrian geometry. A separate plant timing design is needed before changing
LFPC integration or introducing sample-by-sample execution.

Evidence: `summary.json`, four raw topic traces, and `analyze.py`. No temporary
debug logging was added. Owned replay processes were cleaned up after each run.
