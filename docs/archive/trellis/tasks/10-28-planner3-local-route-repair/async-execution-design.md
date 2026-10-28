# Asynchronous global search and timed lightweight execution

User approved both phases after the measured stuttering diagnosis (2026-10-28).
Continue this task; preserve all existing uncommitted full-global work.

## Requirements

Keep periodic full-map A* to the exact final goal. Slow or timed-out global search
must not hold up the local FSM or accepted-observation callbacks. Retained routes
still require fresh FIRI, timed MPPI and execution checks. Keep whole human-cane
geometry, .5s original source/receipt freshness, STOP heartbeat, goal reset and
mode guards. Lightweight motion must execute LFPC model time, rather than jump
whole steps on each FSM invocation. Preserve gait discretization and control
changes at support boundaries. No speed optimization, new pedestrian model or
unvalidated route display. No user-node restart or commit/push in this task.

## Phase 1: immutable global search

Split A* request preparation (selected grid snapshot and copied configuration)
from pure static request evaluation. The worker captures only values and a
deadline, no manager, map, A* object or mutex. Preserve the synchronous wrapper
for algorithm callers/tests. Cache oriented shape support intervals inside each
query to reduce repeated static-cell SAT arithmetic; no certificate survives a
grid change.

Manager owns one future, never waits in a planning callback and never replaces a
running future. Fixed .2s scheduling coalesces missed attempts. Reset increments a
goal revision; old jobs are drained and discarded without blocking. Results carry
request pose/goal/revision/source times. Require original request freshness and
current source validity, then reanchor the computed full route to the actual pose.
FIRI verifies that connector against current static data; MPPI and final checks
use ONE current accepted observation under the existing transaction lock. A search
observation is guidance, not execution authorization; generation changes alone do
not starve adoption. Only final authorization permits route replacement/publication.

## Phase 2: time-driven plant

Add exact LFPC controlled-step sampling from an immutable boundary state, preserving
the existing truncated step duration and endpoint/next-support state. Production
lightweight mode enables a20ms execution timer; other modes retain their existing
plant adapters. Plan/authorize a step on a copy, queue it without moving the real
plant, then sample the actual LFPC state by elapsed time. Optimize new steering at
support boundaries; during a support phase the100ms FSM keeps global scheduling
and safety checks active. The execution timer can request the next boundary plan
without waiting an extra100ms FSM slot.

Before each advance, under the observation lock, validate original current source
freshness and rebuild current pedestrian geometry, check full static-cell geometry
and timed remaining-step samples with current source age. Also check the proposed
advance at the actual check epoch. Extra <=20ms samples strengthen the existing
sampled contract; no continuous swept-body or physical braking claim is added.
Never jump multiple queued steps after a delayed callback; bound one advance and
reanchor model time after delays/STOP. Invalid execution freezes the true LFPC
phase/pose and publishes STOP; resume only after a fresh remaining-step check.
Goal/start reset cancels queued execution and invalidates search jobs. The published
pose and velocity come from the executed plant, not an interpolated display proxy.

## Verification order

1. Failing manager concurrency regression: delayed worker, continued local updates
   and prompt new observations; one in-flight job; old-goal/expired result rejection.
2. Immutable A* query matches synchronous wrapper after original map mutation;
   retain exact-goal/full-envelope/static-edge suites.
3. Build phase1 and rerun measured office/pedestrian timing probe before phase2.
4. LFPC sample endpoint equivalence, monotone phase, no early support switch.
5. Real manager timed execution: no movement on enqueue, elapsed-time pose,
   no catch-up jump, source expiry/new blocker freezes phase, clear resumes,
   goal reset and non-lightweight mode guards, true arrival/idle heartbeat.
6. Build/tests and office/pedestrian replay with timestamped odometry and transaction
   stages. Inspect gaps, displacement/elapsed time, STOP/resume, exact-goal arrival,
   current-source authorization and accepted route/preview publication.
7. Update runtime specs and user docs to the final verified behavior. Archive measured
   evidence; preserve user-owned11311 master and clean only owned11339 jobs.

## Completion record

- [x] Pure immutable global query, single worker and completed-result mailbox;
  no local wait, full-map search and current-pose FIRI/MPPI acceptance retained.
- [x] Query-local static/pedestrian support caches checked against independent SAT.
- [x] Phase1 replay: no >150ms movement intervals; maximum119.6ms, still whole steps.
- [x] Exact LFPC sampler and time-driven lightweight plant, per-advance current-frame
  checks, STOP/resume, boundary-only steering/adoption and current timed displays.
- [x] Start reset cancels execution, clears prior velocity and publishes requested yaw;
  real callback regression observed RED before fix.
- [x] Production/test catkin build without warnings;127 enabled tests pass
  (51 manager,26 A*,2 LFPC,24 MPPI,17 corridor,7 timed geometry).
- [x] Final latest route and still-present pedestrian route both reach actual goal;
  mean movement update20ms, maxima55.7/43.4ms, no >150ms intervals, no STOP.
- [x] Specs, README and user usage/validation docs updated; original baseline and
  final GREEN replay assertions, raw traces and logs retained in
  research/async-execution/. No user11311 process was restarted; no commit/push.

The20ms plant checks are discrete, not a continuous swept/braking certificate.
Midphase blockers freeze/resume the admitted phase; there is no midphase steering
optimization. Search cadence is distinct from boundary route publication: final
latest-route path gap reaches1.906s while locally validated motion continues.
See docs/PLANNER3_ASYNC_EXECUTION_VALIDATION.md for measurements and limits.
