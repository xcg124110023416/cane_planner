# Planner-3 current-frame pedestrian corridor contract

## Scope

Planner 3 with private `use_pedestrians=true`. Default current-frame dynamic spatial FIRI plus timed SAT and periodic full global reference refresh are described in [periodic-global-reference.md](periodic-global-reference.md). False preserves static planning. Planner 4 retains its legacy timed-corridor/obstacle flow. This is one ConvexCorridor/FIRI generator; the additive LFPC timed SAT checker is documented in [lfpc-timed-collision.md](./lfpc-timed-collision.md), independently of the legacy timed walking corridor. The robust-speed timed spec is not activated by this feature.

## Source validity and transforms

- `/onboard_detector/dynamic_obstacles_info` uses its existing header and full position/velocity/size arrays. First version supports zero or one pedestrian; more than one is explicitly invalid, never silently truncated.
- Enabled missing, mismatched arrays, unsupported/nonfinite values, zero/future/stale stamps, empty frame, or unavailable observation-time TF produces `DYNAMIC_DATA_INVALID`. Invalid input replaces previous validity. Fresh empty is valid; timeout is never synthesized as empty.
- Default maximum source age is 0.5 s, configurable finite `(0,2]`. Check both observation and receipt age before geometry, after geometry, and after MPPI before integration. Future frames fail on receipt and cannot become valid merely by waiting. ROS clock rollback fails closed.
- Geometry captures a per-plan immutable `PedestrianFrameToken` (generation, enabled state, original stamp and receipt) under `dynObsMutex_`. Each accepted enabled frame increments generation, including same-stamp/invalid messages; disabled messages do not change cache/generation. Goal reset and any future mutable enable transition use this mutex and advance generation. Final authorization retains generation/enable and captured-age checks as invariants; expired snapshot gives `DYNAMIC_DATA_INVALID` / `PLANNED_FRAME_EXPIRED`. No retry, empty-frame exception or widened timeout.
- One continuous nonrecursive transaction holds `dynObsMutex_` from observation binding through geometry, MPPI, final authorization and step admission/output. Lightweight mode queues a support phase without moving; each 20ms advance is another complete current-observation transaction through its own final check and LFPC update. Other adapters retain immediate integration. `updateAndPublishConvexCorridorLocked` requires caller ownership. Planner3 visualization has no ROS-duration sleep. This supersedes the earlier geometry/MPPI lock gap and requirement to reject every concurrent frame, which caused systematic 10 Hz starvation. A callback waiting during a transaction is accepted after commit, including a blocker; the next execution tick checks it before moving. Only already-accepted observations are checked by a commit. Original snapshot expiry during computation still prevents admission/advance. This is an explicit accepted-observation ordering, not a guarantee about unprocessed incoming data or future collision safety.
- Callback receipt is captured at callback entry BEFORE mutex wait, not when the cache is finally updated; header stamp is retained. Original receipt/header age validation prevents buffering from renewing old observations. Existing subscriber queue and cooperative FIRI/SFM limits remain; MPPI has finite configured sample/rollout counts, not a hard time bound. Long computation can yield stale STOP and callback delay; no unconditional latency/fairness or hard-real-time guarantee follows.
- Geometry clear/iteration and snapshot reset retain the same mutex. Goal callbacks own the mutex across goal state updates and call `resetCorridorForGoalLocked`; already-held paths call only Locked helpers, avoiding recursive lock deadlock. Regression-only barriers cover actual MPPI and callback entry before lock acquisition. Queued callbacks are joined only after transaction completion; repeated traffic tests must show LFPC progress as well as freshness/invalidity STOP. No runtime test flags.
- Planning frame is `world`, consistent with existing maps/LFPC/markers. Transform centres with full TF and velocity with rotation only, at the original header stamp. Conservatively enclose a rotated complete 3D source box using `abs(R)*full_size`; retain original header and source values separately. Unknown map/world identity is not allowed. The lightweight simulator explicitly emits configured world coordinates in planner-3 launches.
- Source-enable propagates through algorithm, lightweight, hardware and Gazebo entrypoints. Gazebo enable is the OR of lightweight pedestrians, truth bridge and world pedestrians; world-only without observation source therefore fails closed. Truth source takes precedence over lightweight source to avoid two contradictory publishers. Gazebo `ModelStates` is world data; bridge rejects arbitrary frame relabeling and retains callback receipt stamp on timer republishes. A source stall expires rather than becoming perpetually fresh. More than one Gazebo pedestrian intentionally fails the first-version cap.

## Pure polygon module

`PedestrianPolygon` uses observed velocity as desired velocity, hence zero initial drive. It omits target attraction, stochastic terms, other-pedestrian forces and intent inference. Force is an acceleration-like m/s^2 engineering approximation: bounded exponential nearest static-boundary repulsion plus LFPC centre-surrogate repulsion. System repulsion is amplified inside the pedestrian's +/-110-degree FOV. System heading is supplied/validated but the centre surrogate is isotropic; no full human/cane footprint is represented.

Nearest whole blocked-bin face/boundary and finite grid exterior come from the exact selected corridor grid; no raster-density force summation, raw-ESDF alternate classifier or pedestrian rasterization. Only nearest distance inside the finite influence range contributes. A cropped-domain boundary is conservatively treated as blocked, like FIRI, and may produce an artificial boundary force near its edge. Grid scan is bounded and has a cooperative per-observation budget.

Defaults and supported ranges (all finite):

| Parameter `pedestrian/…` | Default | Range / units |
|---|---:|---|
| safety_radius | 0.4 | [0.01,2] m; independent social radius |
| body_margin | 0.03 | [0,0.5] m; hard rectangle padding |
| wall_gain, system_gain | 1 | [0,20] m/s^2 |
| decay_length | 0.5 | [0.01,5] m |
| fov_amplification | 2 | [1,10] dimensionless |
| alpha | 0.3 | [0,5] s^2; force-to-distance, not prediction duration |
| max_force | 4 | (0,20] m/s^2 |
| max_extension | 1.5 | [0,5] m beyond safety radius |
| influence_distance | 3 | [0.01,10] m |
| max_seconds | 0.02 | (0,0.1] cooperative seconds |

Source full dimensions retain [0.01,4] m, speed <=10 m/s and coordinate bounds.
Current geometry is body rectangle padded independently by body_margin, combined
with a raw social triangle. Moving heading +/-110-degree Ds points and force point
are no longer masked by a full safety-radius disk. Zero force uses moving heading;
stationary observations lack heading and retain padded symmetric body. See the
[2026-10-27 geometry contract](directional-footprint-corridor.md), which supersedes
the earlier full pedestrian octagon and COM-only corridor conventions.

## FIRI and failure evidence

Default production wiring changed on 2026-10-28: see
[periodic-global-reference.md](periodic-global-reference.md). FIRI separates static cells and current pedestrian hulls; timed SAT handles
future pedestrian occupancy independently. Current-frame polygon exclusion
remains mandatory in production.

`buildStatic(path,grid,dynamic_polygons={})` shares the existing separator template with four-corner static arrays. Complete strictly convex CCW polygons (3–32 vertices, diameter <=20 m) use the same stationary/support-switch/endpoint-active directions, one common separating plane and unmodified metric/MVIE certificates. Empty dynamic input retains the selected static geometry/sampling mode. Precheck all original local reference edges before inflation, then use the configured coverage-driven or legacy edge sampling with the same original-edge coverage, joins and global budgets. Whole human-cane seed vertices participate when enabled. Any reference obstruction with clearance gives `DYNAMIC_REFERENCE_BLOCKED`. Current-frame exclusion is mandatory even when timed SAT is enabled. Periodic full global A* provides detour candidates before local feasibility checks; a blocked retained reference may use a freshly certified connected prefix if the full MPPI horizon remains feasible; new candidates require complete active-length coverage. Candidate corridor/MPPI rejection retries the retained route once with current geometry. See the periodic global reference contract for scheduling and transaction.

`Result::has_dynamic_input` prevents `CorridorFailureCapture::saveOnce` from saving a static-only file even for an otherwise eligible numeric failure. It returns SKIPPED with `DYNAMIC_INPUT_NOT_CAPTURED`, without consuming the static capture latch. Manager also suppresses such capture and logs the limitation. Existing v1/v2 snapshots replay point/edge mode; v3 also records footprint/sampling/root yaw; exact dynamic runtime replay is **not implemented**. Dynamic regressions preserve complete polygons as source fixtures instead.

## Obstacle-triggered route recovery

### Search-local static comfort distance field (2026-10-27)

Planner-3 initial/recovery A* now derives its soft wall-distance cost from
`CorridorDistanceField` in plan_env (C++ namespace `cane_planner`), built from the SAME
selected `getStaticCorridorGrid` classifier as the hard whole-cell certificate.
Do not substitute raw 3D ESDF distances or omit unknown/model/map-boundary rules.
The field is unsigned: blocked cells and domain exterior have zero comfort
distance; only free-space distance is needed for this cost. Mark complete closed
cells on a half-native-bin lattice and apply a separable exact squared Euclidean
transform. Distances at those samples are exact up to roundoff. Bilinear lookup
between samples approximates whole-cell distance with absolute error bounded by
native_resolution*sqrt(2)/4 (about 3.54 cm at 0.1 m); this is NOT a hard certificate.
Retain complete-edge midpoint integration, nonnegative metre costs, the existing
sigma/weight meaning, and the Euclidean lower-bound heuristic.

Initial A* and legacy searchRecovery lazily build native-spaced tiles with cores up to 64 bins and halos of
ceil((sigma+clearance)/native)+1 bins. A cropped side cannot affect the truncated
cost inside its core; real map boundaries still contribute. Reduce core size to
respect existing finite snapshot budgets and reject unsupported configurations
before expanding. Bound tile work and check the recovery deadline before and
after construction. Clear all fields at every search entry/reset, parameter reload,
clearance change and collision-source replacement. Cache lifetime is one search,
not one pedestrian frame or one process; this avoids stale local-map/backend data
without introducing a revision contract for every map producer. No dynamic polygon
is rasterized into this field. Hard edge/polygon checks, current-root-hull FIRI and
source-age/execution authorization remain authoritative and unchanged.

Regressions must exercise actual recovery call counters (map extractions scale
with tiles, not soft edges), exact lattice distances vs independent whole-cell
geometry, interpolation bounds, true/cropped boundaries, map changes between
searches, invalid/replaced sources, selected unknown/ESDF/local classifications,
exact goals and manager expiry/rebuild/STOP cases. Faster COM search does not
guarantee a footprint-valid recovery route: inspect REBUILD_FAILED separately from
TIMEOUT and never weaken FIRI to make a benchmark pass.

`Astar::searchRecovery(start,goal,polygon,steady_deadline)` reuses the existing search and static certificates. The immutable query polygon is validated as complete strictly convex CCW geometry (3–32 vertices, finite bounded coordinates); continuous segment/polygon clipping rejects closed intersections and segment-edge distances enforce the same FIRI clearance. Empty overlay does not affect the static objective/edge decisions. No overlay persists beyond the call, including timeout/failure. Dynamic geometry is hard-only, never a new comfort penalty. Recovery output must end at the exact actual final goal; partial horizon success is returned as `INCOMPLETE_PATH` with empty output, never committed.

Default planner 3 now refreshes the full global reference periodically, before
MPPI and independent of its success. The legacy searchRecovery API above remains
covered for tests; production uses point-reference searchGlobalReference from
actual CoM to exact final goal with a temporary current pedestrian overlay.
See [periodic-global-reference.md](periodic-global-reference.md) for full-map
point search, scheduling, candidate validation and authorized commit. Accepted
observation locking, generation/stamp/receipt checks, .5s freshness and frozen
STOP remain. Preview never triggers search. Search failure may retain the old
route only after all current-cycle FIRI/MPPI/execution checks pass.


## Output and safety limits

Body/social outlines share `/mpc/convex_corridor` MarkerArray with stable namespaces `mpc_pedestrian_body` and `mpc_pedestrian_social_hull`, id 0, world frame and 0.3 s lifetime. DELETE on invalid/empty/reset and expired FSM data prevents persistent ghosts; corridor publication DELETEALL precedes current geometry. No tracking-ID persistence is assumed. Existing corridor/A*/MPPI visuals remain.

Metric positions 0–25 retain their indices. Field 5 now counts timed SAT rejects; field 19 indicates spatial-only mode (zero when timed SAT is active). Fields 23=source enabled, 24=active corridor feasible, 25=FailureReason enum. Timed fields 26–29 are documented in the timed checker contract. Legacy MPPI validity/cost fields describe the evaluated rollout; if data expires after MPPI, field 24 is false and STOP reason is authoritative. Stop/debug text distinguishes `DYNAMIC_REFERENCE_BLOCKED`, `DYNAMIC_DATA_INVALID`, static invalidity and no feasible rollout.

The timed checker adds hard feasibility checks without changing motion costs or the LFPC STOP serializer. Spatial generation protects the actual current human-cane envelope in one region; predicted COM samples and support feet need corridor-union containment; the manager invalidates/rebuilds spatial geometry when its selected map changes, while predicted full envelopes are checked only against pedestrians at matching times. Invalid or expired data withholds integration and preserves the lightweight STOP heartbeat. FIRI always excludes current pedestrian hulls, even with timed SAT enabled. Translate accepted source hulls by velocity times source age to each generation epoch; keep the original source hull for future timed SAT. A* supplies spatial detours around current occupancy. A blocked retained reference may use its freshly certified connected prefix while asynchronous search/validation is pending, only if the full MPPI horizon and execution checks fit. Otherwise stop; never substitute a static-only corridor. New candidates require complete active-length coverage before commit. Geometry/MPPI candidate rejection retries the retained route once; invalid or expired source data still stops. Neither mode provides physical braking. Navigation evidence requires runtime checks beyond geometry/unit tests.

## Initial planner-3 goal activation transaction

GEN_NEW_TRAJ and REPLAN_TRAJ call one locking activatePlanner3Route helper covering A* search, authoritative route materialization, subgoal/LFPC initialization and MPC_STEP activation. Inner callAstarPlan/generateGlobalWaypoints do not reacquire this nonrecursive mutex. Planner 4 retains its prior branch. Failed planner-3 search retries on the next existing FSM cycle, not a 0.5 s Gazebo sleep while locked.

A queued goal B is not accepted until A activation commits. Once accepted, goal reset invalidates initial_route_active_ and clears corridor/warm start; a stale MPC_STEP dispatch cannot advance LFPC. The real FSM must search/materialize B before reactivation. Both goal callbacks retain their full goal-state mutex ownership. The inactive guard defaults false; tests which directly invoke a step explicitly model an already activated route. No claim is made that received-but-unprocessed B was incorporated in A.

## Transaction diagnostic recording (planner3_transaction_v1)

One `/mpc/transaction_diagnostics` Float64MultiArray is published at each planner-3 step return, including inactive/invalid/STOP exits. Existing `/mpc/debug_metrics` indices are unchanged. Layout label includes `source_frame=<original header frame>` or DISABLED; transform output remains world. All time/duration values are seconds. Steady absolute time is local process/machine monotonic time, not ROS epoch or a cross-machine clock. NaN means skipped/unavailable; per-cycle storage is reset before use. No per-node log spam or eligibility changes.

| Index | Meaning |
|---|---|
| 0 | schema version 1 |
| 1,2 | transaction callback entry ROS epoch / steady seconds |
| 3,4 | prior transaction entry interval ROS / steady; first cycle NaN |
| 5,6 | pedestrian enabled / accepted token generation |
| 7,8 | original source header stamp / callback-entry receipt, before lock wait |
| 9,10 | source stamp / receipt age at transaction entry (negative diagnoses future/queued ordering) |
| 11 | accepted source callback lock wait, steady seconds |
| 12 | planner entry to transaction mutex acquisition, steady seconds |
| 13 | current active FIRI solve only |
| 14 | completed periodic global A* worker time, not blocking time in this callback; snapshot preparation is separate |
| 15 | candidate FIRI solve when a global candidate exists; also included in 13 |
| 16 | reference proposal / single MPPI call |
| 17 | authorization/check and invalid-output cleanup |
| 18 | entry through critical transaction output, before unlock/visualization |
| 19,20 | total step return elapsed steady / ROS delta, includes visualization; planner3 has no ROS-duration sleep |
| 21 | no immediate LFPC integration =1, integrated =0; lightweight enqueue/active phase and goal/inactive returns are1; actual motion is in execution_diagnostics |
| 22,23 | global search inclusive hard-edge / soft-cost query seconds |
| 24 | global search grid extraction seconds; separate from 22/23 |
| 25,26,27,28 | hard calls / soft calls / grid calls / expanded nodes |
| 29,30 | accepted observation count / frame validation state before current computation |
| 31 | current corridor FailureReason enum, NaN if no corridor evaluated |
| 32 | step admission/preparation duration; other adapters also integrate and collect the path here |

Index 15 is a candidate-specific view of 13, not additional FIRI time; geometry wrapper also includes map extraction/SFM and is not identical to FIRI. Indices 18/19 are inclusive totals, not sums of disjoint stages; 13–17/32 omit other callback work. Source receipt is never refreshed by waiting. Count/state 29/30 may describe malformed acceptance (state=0); use validity and failure reason, never count alone. Data age/expiry logic remains authoritative and unchanged. The step diagnostic does not cover initial global A* activation outside mpcSimStep; a long initial search can appear as an inter-step interval, not an identified cause.

Existing lightweight recorder includes raw DynamicObstacles, /clock even if absent in wall-time simulation, goals/initialpose, rosout, source/sim/hardware odoms, accepted A* path, corridor, trajectory/sample and debug/STOP topics. Command after reviewed integration: `rosrun plan_manage record_lightweight_mpc_eval.sh recovery_profile`. Recording is user-run; no recording was executed by implementation. A simultaneous all-topic two-second gap remains of unknown cause; ROS vs steady evidence helps investigate, not automatically attribute it to A*.

The earlier profiling-only implementation was superseded by search-scoped comfort fields and global point/segment geometry caches. Initial A* tiles have query lifetime; periodic global search copies one selected-grid snapshot then evaluates it in a pure background job, caching exact pose/sweep certificates only inside that job. No cache survives a search or assumes a producer revision contract. Search profiling describes completed periodic global jobs, not this local callback's blocking time; initial legacy static search does not start profiling clocks. Timer overhead is nonzero.

## Optional pinned pedsim source

See plan_manage/config/pedsim/DEPLOYMENT.txt for fixed upstream revision, optional pre-integration robot freshness patch, source selection, body dimensions, same-world PCD projection and restart-on-loss contract. This does not change planner geometry/control or default deterministic source. World walls are not local SDF classifications. No low-obstacle/full-footprint/future collision guarantee.

## Owned single-entry pedsim round

start_pedsim_navigation.sh loads primary plus dedicated home dependency with --extend, checks guard manifest and unused private master port, exports same PCD and composes existing navigation/source. Opt-in bridge auto_start releases only once after navigation goal and fresh post-goal plant odom plus actual PAUSED_READY. Never release before goal or clear a latch. Status topic reports WAIT_GOAL/WAIT_PLANT/WAIT_INPUT/WAIT_PAIRED_OUTPUT/RUNNING/FAULT_RESTART_ROUND. Idle heartbeat remains manager authority. Full reset is stopping only the owned roslaunch parent then relaunching the entry. Existing deterministic/manual default remains unchanged. See pedsim/USAGE.txt.

Ownership correction: start wrapper delegates to pedsim_round.py supervisor. Per-port flock spans preparation through cleanup; explicit child rosmaster PID and random token are proved before navigation, monitored afterward. Nonowned/preexisting/racing masters are refused without killing them. record subcommand shares environment/port, requires held-lock/PID/token identity, forwards validated label via argv and owns only its recorder process group. SIGINT/TERM cleanup targets owned groups only; persistent lock files are never ownership evidence or deleted. Use start --port and record --port --label, no manual environment setup.

## Current-CoM local reference binding

Each new planner-3 local corridor starts at actual current LFPC CoM and connects to the projection on the authoritative forward A* polyline. The configured planning_length includes this connector and truncates the final original edge exactly; it is independent of lookahead and preview length. Only a projection less than 1e-9 m from CoM is replaced by CoM to avoid a degenerate FIRI edge; no previously predicted trajectory supplies reference geometry. The connector participates in the same selected static-grid point-reference FIRI certificates as every other edge, with the actual current hull additionally protected at the root. Blockage fails closed with existing static/dynamic reasons; periodic full global A* refresh does not require MPPI failure or a collision witness. No old-cell fallback or clearance relaxation.

A feasible new result must contain the entire configured envelope at actual CoM/torso yaw in one region under the controller's 1e-6 containment tolerance before publication/controller replacement, and a recovered result before authoritative route replacement. A failed root postcondition clears partial geometry and reports OUTPUT_CERTIFICATE / OUTPUT_CERTIFICATE stage at the actual root, not optimizer NO_FEASIBLE. Accepted-observation locking, freshness, final rollout checks and STOP remain unchanged. Geometric root coverage alone does not certify support foot, full future LFPC motion, or physical braking.

## LFPC optimizer and arrival

The intermediate five-angle beam implementation (after 4fff814, through 80f8ab2)
is superseded by continuous steering MPPI. Its beam_width and beam_evidence
contracts are historical; use [lfpc-mppi.md](./lfpc-mppi.md) for current sampling,
weighted-update validation, fallback, bounds and evidence. Current-frame DWCG/FIRI,
accepted-observation transactions and timed SAT remain mandatory by default.

Manager passes actual end_pt and global_wp_arrival_radius separately from the
local lookahead goal. Only a fully validated incoming step inside the strict
final radius permits terminal idle continuation. Local arrival only suppresses
the goal-distance term; every later walking step still incurs steering and
variation cost and all safety checks. Independent replay reconstructs the full
LFPC rollout/objective and requires exact finite equality before execution.
The next manager cycle detects actual arrival before integration; WAIT_TARGET
maintains the existing authoritative idle heartbeat. No enlarged arrival radius
or unsafe-step exemption.
