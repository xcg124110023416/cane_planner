# Planner-3 LFPC time-synchronized collision contract

## Geometry and timing

`TimedPedestrianCollision` in path_searching is pure domain geometry, independent
of ROS. Bind the exact accepted social hull and world velocity already used for
the current accepted geometry transaction. Default FIRI is static-only; see
[periodic-global-reference.md](periodic-global-reference.md). No additional SFM solve, FIRI inflation, intent inference or
polygon rasterization. CV occupancy at future relative time tau is
`P0 + velocity * (execution_epoch - source_stamp + tau)`.

The robot defaults to the rigid full human-cane envelope shared with FIRI and
visualization (human_cane parameters). Rotate it by LFPC torso yaw at each queried
pose; cache SAT normals/support intervals for that yaw. Same-yaw LFPC samples and
execution queries reuse intervals. Contact is collision with 1e-9 m tolerance.
Explicit human_cane/enabled=false retains the fixed circumscribed COM octagon for
historical comparisons. Invalid geometry/time fails closed. The optional timed
ablation does not bypass source freshness, spatial footprint checks or FIRI.
See [directional-footprint-corridor.md](directional-footprint-corridor.md).

LFPC records actual integration phases alongside COM samples. Copy/reset/prepare
preserve or clear both arrays together. Keep existing `int(t_sup/delta_t)` behavior
(floating .35/.07 can produce four samples). Do not assign nominal t_sup to the
last sample or encode time in Z. Controller edges consider newly generated samples,
subtract the incoming phase, and accumulate actual elapsed time through ancestry.
Re-roll the selected sequence with identical point/time checks. Root time zero is
also checked. Full-footprint mode adds <=5-degree intermediate turn poses at each
step root, at that root model time. Keep path/yaw/time arrays together through replay
and actual execution-age recheck. Legacy point/circle mode retains original samples.

## Transaction and execution

Manager binds hulls/velocities/source age under the existing nonrecursive accepted
observation lock, after geometry and before MPPI optimization. Freshness, original stamp,
receipt and generation authorization remain mandatory. Immediately after that
check, revalidate the root and ALL selected samples using the actual execution
epoch, before admitting a step or changing the plant. Cost is bounded by selected
path length, not the search tree. A collision invalidates the plan and publishes
`TIMED_EXECUTION_COLLISION`; LFPC state remains frozen, existing STOP odometry
continues. No retry after execution rejection, deadline widening, old-prediction fallback or moving output. Periodic full global A* may supply a pending reference before MPPI; it still needs this final execution check.
One bounded collision witness retains COM, future offset and source age at check.

Production lightweight mode (planner3 && simulation && !gazebo) queues the first
step from an immutable boundary LFPC copy. A 20ms timer samples the exact LIPM
solution at elapsed ROS time, preserving int(t_sup/delta_t) duration and switching
support only once at completion. One advance is capped at 40ms; delayed callbacks
and STOP reanchor the clock instead of catching up by jumping multiple steps.
The timer and FSM serialize through fsmMutex_, then dynObsMutex_; pedestrian
callbacks take only dynObsMutex_. At a completed phase the timer may request the
next plan immediately, avoiding an extra 100ms FSM idle wait. Global scheduling
continues during the phase, but controls/reference adoption wait for its boundary.

Before EACH advance, bind a fresh accepted observation and rebuild its geometry.
Check the original admitted corridor, latest selected whole occupied cells and
support foot. Check current/proposed poses, nominal LFPC samples, additional 20ms
samples and root yaw sweep. Reauthorize that original current token immediately
before advance. Dynamic query time is source_age + max(0,sample_phase-target_phase).
Thus current/proposed poses use the current epoch; remaining poses use their
remaining phase offset. New blockers or stale input freeze actual pose/phase,
publish zero-twist heartbeat and STOP, and prevent clock catch-up. Fresh clear
observations may resume the checked remainder; no midphase steering optimization
or physical braking is claimed. Other plant adapters retain prior behavior.

Static full-footprint FIRI remains mandatory. Newly vacated pedestrian space can
be used only when the matching-time checks pass. Waiting/speed actions are not added. This certifies only sampled configured rigid-envelope poses
under CV; no continuous swept LIPM, acceleration/turning uncertainty or physical
braking claim. Lightweight motion now follows model time within callback jitter;
stalls/STOP slow or freeze that motion rather than skipping unchecked phases.

## Outputs and regressions

`/mpc/timed_best_path` is nav_msgs/Path in world. Header stamp is execution epoch;
pose stamps are epoch + actual future offsets, positions keep spatial Z. It is
empty on STOP, fresh empty/disabled input, reset or arrival.
`/mpc/timed_collision` is a dedicated world MarkerArray with DELETEALL before each
frame, bounded lifetimes and stable timed_robot/timed_pedestrian/timed_labels
namespaces. At most ten matched safe pairs with +seconds text; cyan-to-magenta
encodes increasing time. STOP shows one red Rejected witness, never a stale safe
trajectory. Invalid/empty/reset/arrival clears outputs. RViz display name is
MPC_TimedCollision. Recorders include both topics.

During lightweight execution this display is replaced by the checked REMAINING
first support phase, with current authorization epoch and remaining offsets;
markers subsample every third pose and expire after .15s. It must not keep old
whole-horizon timestamps. STOP clears the path and can show a current rejected
execution footprint. MPC_HumanCane shows actual pose in this mode; best_traj
remains the last MPPI prediction, not current execution authorization.

`/mpc/execution_diagnostics` Float64MultiArray v1 has eight fields:
version, ROS epoch, executed phase, effective duration, ACTUAL advance (zero on
STOP), stopped, current accepted generation, validation seconds. The 33-field
planning diagnostic keeps21=1 when a step is queued without immediate integration.

Debug indices 0–25 retain positions; 5=timed rejected edge/query count,
19=spatial-only mode. Append 26=timed active, 27=point checks (search/replay/final),
28=execution rechecked, 29=execution safe. Distance/CPA metrics remain unavailable;
SAT axis projection gaps must not be reported as Euclidean clearance. Final plan
validity is false after execution rejection. MPPI optimization evidence describes
sampling/weighted-update replay; execution safety is independently indicated by
28/29 at admission, then execution_diagnostics and STOP while the lightweight
plant advances. See [lfpc-mppi.md](./lfpc-mppi.md) for the optimizer contract.

Tests cover SAT contact/containment/rotated separation/invalid/empty/CV;
controller sample timing/copy/reset, crossing beyond first step, source age,
latency rejection and ablation; manager captured snapshot and source-age binding,
matched future marker/path stamps and empty cleanup, execution-time STOP before
integration with frozen odometry. Add exact LFPC endpoint equivalence, phase and
support switching; enqueue without motion, timed advance/catch-up bound, new-frame
STOP/resume, source expiry, goal reset, idle heartbeat, mode guards, delayed global
worker progress and current execution display time/geometry. Existing transaction/
arrival/static tests remain.
Opt-in same-budget benchmark (10 steps, 256 samples, 2 iterations): test_static_mpc with
`--gtest_filter=TimedMpcRuntime.DISABLED_BenchmarkSameMppiWork --gtest_also_run_disabled_tests`.
