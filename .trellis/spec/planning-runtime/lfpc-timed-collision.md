# Planner-3 LFPC time-synchronized collision contract

## Geometry and timing

`TimedPedestrianCollision` in path_searching is pure domain geometry, independent
of ROS. Bind the exact accepted social hull and world velocity already used for
the current accepted geometry transaction. Default FIRI is static-only; see
[time-triggered-local-repair.md](time-triggered-local-repair.md). No additional SFM solve, FIRI inflation, intent inference or
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
epoch, before any LFPC control change or integration. Cost is bounded by selected
path length, not the search tree. A collision invalidates the plan and publishes
`TIMED_EXECUTION_COLLISION`; LFPC state remains frozen, existing STOP odometry
continues. No retry after execution rejection, deadline widening, old-prediction fallback or moving output. A failed initial MPPI may perform one local repair/retry before this final execution check.
One bounded collision witness retains COM, future offset and source age at check.

Static full-footprint FIRI remains mandatory. Newly vacated pedestrian space can
be used only when the matching-time checks pass. Waiting/speed actions are not added. This certifies only sampled configured rigid-envelope poses
under CV; no continuous swept LIPM, acceleration/turning uncertainty or physical
braking claim. Lightweight simulation still jumps whole LFPC steps at the FSM
cadence rather than continuously executing model time. Do not claim wall-time
trajectory tracking synchronization on the strength of these predictions.

## Outputs and regressions

`/mpc/timed_best_path` is nav_msgs/Path in world. Header stamp is execution epoch;
pose stamps are epoch + actual future offsets, positions keep spatial Z. It is
empty on STOP, fresh empty/disabled input, reset or arrival.
`/mpc/timed_collision` is a dedicated world MarkerArray with DELETEALL before each
frame, .6-second lifetimes and stable timed_robot/timed_pedestrian/timed_labels
namespaces. At most ten matched safe pairs with +seconds text; cyan-to-magenta
encodes increasing time. STOP shows one red Rejected witness, never a stale safe
trajectory. Invalid/empty/reset/arrival clears outputs. RViz display name is
MPC_TimedCollision. Recorders include both topics.

Debug indices 0–25 retain positions; 5=timed rejected edge/query count,
19=spatial-only mode. Append 26=timed active, 27=point checks (search/replay/final),
28=execution rechecked, 29=execution safe. Distance/CPA metrics remain unavailable;
SAT axis projection gaps must not be reported as Euclidean clearance. Final plan
validity is false after execution rejection. MPPI optimization evidence describes
sampling/weighted-update replay; execution safety is independently indicated by
28/29 and STOP. See [lfpc-mppi.md](./lfpc-mppi.md) for the optimizer contract.

Tests cover SAT contact/containment/rotated separation/invalid/empty/CV;
controller sample timing/copy/reset, crossing beyond first step, source age,
latency rejection and ablation; manager captured snapshot and source-age binding,
matched future marker/path stamps and empty cleanup, execution-time STOP before
integration with frozen odometry. Existing transaction/arrival/static tests remain.
Opt-in same-budget benchmark (10 steps, 256 samples, 2 iterations): test_static_mpc with
`--gtest_filter=TimedMpcRuntime.DISABLED_BenchmarkSameMppiWork --gtest_also_run_disabled_tests`.
