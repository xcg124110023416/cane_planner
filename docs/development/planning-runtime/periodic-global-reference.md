# Periodic full global reference (planner 3)

Approved 2026-10-28. Supersedes the local-repair runtime in
[time-triggered-local-repair.md](time-triggered-local-repair.md).

## Scheduling and search

After the existing initial-route activation, check manager/global_replan_period
(default .2s, finite .1..5s) every active planning transaction. The first active
cycle refreshes immediately. Keep a fixed next-deadline ROS phase instead of adding the period to every
actual late start. On each due attempt (including failures), advance that deadline
past now once; skip missed slots without catch-up bursts. Goal reset clears both
last-start and next-deadline times; clock rollback reanchors the schedule, without
bypassing original source/receipt freshness. This is best-effort cadence on the
existing 100ms FSM, not a hard 5Hz deadline. One background job is allowed at a
time; busy slots are skipped. Lightweight route adoption occurs only at support
boundaries, so path publication is not a 5Hz guarantee.

Astar::snapshotGlobalReference copies the selected grid and configuration under
the manager transaction. Astar::evaluateGlobalReference evaluates this immutable
value in a background worker, without a live manager, map, Astar object or mutex.
The synchronous searchGlobalReference wrapper remains for algorithm callers.
Search covers the full selected map bounds, from the request's actual
CoM to exact end_pt. It does not inspect an old blocked interval, crop around a
previous route, preserve a suffix, require a collision witness or await MPPI
failure. The old repairLocalReference runtime/API is removed. Legacy searchRecovery
is retained for historical tests; planner 3 does not call it.

Build accepted pedestrian geometry ONCE before search, under dynObsMutex. Translate
that exact hull by original velocity times source age at search start, only as a
query-scoped current-occupancy overlay. Do not persist it in the map. Continue to
use the original hull/velocity for timed MPPI and execution-age SAT. Empty fresh
frames and disabled source perform static global searches; missing/invalid/stale
input cannot trigger a search or execute.

Search is over 2D lattice positions, from exact actual CoM to exact goal.
The reference is a point polyline, not an oriented future human-cane sweep.
Check its straight segments against whole occupied cells and the convex current
pedestrian overlay via separating support intervals. Current hull containment is
FIRI's root constraint; future pedestrian safety belongs to timed MPPI/execution.
Routing margin remains existing clearance plus one search bin. Use the Euclidean
heuristic, finite allocations and cooperative deadline; no completeness/optimality
claim.

One selected static-grid snapshot, one search-scoped comfort field and cached
point/segment certificates.
Occupancy prefix sums may skip an AABB only when every covered
cell is free. Preserve full SAT/cell checking otherwise. Comfort distance only
changes cost. Deadline <=.2s and, when source enabled, half remaining original
header/receipt freshness. Grid/field work is bounded but not preemptible; check
deadline after it and inside expansion, geometry, prefix and comfort loops.

## Worker lifecycle and pending candidate transaction

Poll the single future without waiting. Keep at most one completed result in the
mailbox while a lightweight support phase executes. A goal/start reset increments
the revision and clears that mailbox; it never replaces or waits for a running
future. Old-revision, wrong-enable-mode, clock-invalid and originally expired
results cannot become candidates. A new observation generation alone does not
invalidate spatial guidance: the result is never execution authorization. Reattach
its full path to the nearest segment from the CURRENT actual CoM, then certify
that connector using current current-frame FIRI. The worker has no manager capture, so
its bounded shutdown completion cannot access destroyed runtime state.

Under one nonrecursive accepted-observation lock: capture current frame, build
geometry, poll/schedule global work, build current-frame FIRI on candidate (or retained)
active-length reference, seed copied LFPC, timed MPPI, original-frame authorization,
execution-epoch revalidation and route commit. Lightweight mode queues the first
support phase without moving the plant; other adapters retain immediate integration.
A new candidate resets optimizer warm start, never actual LFPC state. Its target
and reference are temporary; only successful final execution authorization replaces
global_waypoints_/index/lookahead. Publish accepted A* afterward. Search failure
may use the retained route only after fresh FIRI/MPPI/all execution checks.
Candidate corridor or MPPI rejection retries the retained route once in the same
observation transaction, rebuilding geometry and resetting/seeding MPPI from the
actual LFPC state. Do not schedule another search or reuse old corridor/controls.
Report RETAINED_AFTER_REBUILD_FAILED or RETAINED_AFTER_MPPI_REJECTED on successful
fallback; rejected candidates never replace the authoritative route. Invalid or
expired observations still withhold execution. Keep STOP heartbeat, current-root
containment, timed pedestrian envelopes, COM/foot certified-corridor-union checks and selected-map revision validation and exact goal.

FIRI and preview always separate static cells and accepted current pedestrian
hulls. Timed collision enable does not remove spatial pedestrian exclusion.
Nonempty frames report CURRENT_FRAME; empty/disabled frames remain STATIC_BASE.
New global candidates require complete active-length FIRI. For the retained
reference, adaptive FIRI may return a connected independently certified prefix
when a farther geometric obstruction prevents full coverage. The full MPPI
prediction and execution checks must fit that fresh prefix; otherwise STOP.
Never substitute a static-only corridor or accept invalid/numerical/expired work
as a prefix. A geometric output-certificate failure after one or more certified
regions may retain only that prior prefix after re-certification; a first-region
or failed re-certification still stops. `complete`, `covered_length`, `suffix_reason` diagnostics distinguish
this from full coverage; the truncation marker is orange. Preview still requires
complete coverage and has no STOP authority. Preview defaults to .2s (.1..5s),
with fixed next deadlines initialized from the global search phase, not the
variable completion time after MPPI. Both global and preview eligibility use one
scheduling epoch captured after acquiring the accepted-observation mutex: using
wall time before search for one and after MPPI for the other can cross a deadline
within a non-refresh cycle and launch a doomed preview. This epoch is ONLY for
scheduling; source age, CV extrapolation, deadlines, authorization, static preview
creation stamps and execution checks still use their original real times.
Keep its independent parameter, pure async
value-copy job, source/receipt expiry and route/goal revision rejection. This
avoids launching due previews on intervening non-refresh cycles merely because
previous search duration moved the relative launch phase. No stale job acceptance
or continuous/guaranteed display rate is implied.

Spatial ablation retains current-frame pedestrian FIRI exclusion and periodic
search. Source/receipt max age stays .5s.
All samples, yaw/time arrays, weighted validation and selected replay remain.
No speed/wait actions, expanded pedestrian count, LFPC discretization changes or time A*.
Lightweight execution now follows elapsed model time; see the timed collision contract.

## Outputs and regressions

Existing A* path, preview, human-cane and timed displays remain. Text debug declares
route_scope=FULL_GLOBAL, period and global_replan status/attempts/seconds. READY is
pending, SUCCESS means execution-authorized commit. Failure statuses distinguish
NO_PATH, TIMEOUT, REBUILD_FAILED, MPPI_REJECTED, EXPIRED and EXECUTION_REJECTED.
NOT_NEEDED means no attempt this transaction; SEARCHING means a job is pending,
SUPERSEDED means its goal/source mode changed. Keep metric array positions. In the
33-field transaction array, 14 and 22..28 describe a completed global worker
result; they are NOT time spent searching in this local callback. Field24 is
request snapshot cost on the manager thread; 15 measures candidate FIRI, 16 one
MPPI pass. Fields18/19 measure actual callback cost. Queuing a lightweight step
keeps field21=1; actual motion has its own execution diagnostics.

Regress proactive refresh while MPPI already succeeds, preview publication despite changing search durations,
configurable .5s boundary, default .2s timing, .1s jitter
phase preservation, skipped-slot handling, newly acquired
static occupancy, temporary current pedestrian overlay, exact nonlattice goal and
full-map detour outside former local bounds. Keep no-path/fresh-clear resumption,
source/search/rebuild/MPPI/execution failure with no commit, queued callbacks, current-root
hull/FIRI, actual arrival and STOP/idle heartbeat. Replay the real office
start/goal under an owned private master, plus a still-present pedestrian bypass.
Record actual timings and intermittent STOP; do not infer continuous safety.
Regress delayed search with continued plant/observation updates, immutable map
snapshots, single in-flight work, revision/expiry rejection and exact point/segment checks
against an independent whole-cell/pedestrian oracle.
