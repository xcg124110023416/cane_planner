# Planner-3 LFPC MPPI contract

## Scope and objective

`MpcController` replaces the intermediate five-angle beam controller with
MPPI-style continuous steering optimization. Planner 4 remains separate.
Optimize only the N relative heading increments `api`; validated nominal al/aw
remain fixed. Keep the existing objective and effective +/-30 degree bound:

```text
J = sum(w_steer * abs(api[n]))
  + sum(n > 0 ? w_dapi * abs(api[n] - api[n-1]) : 0)
  + (local_arrived ? 0 : w_goal / N * terminal_goal_distance)
```

Local arrival only suppresses distance cost. It never truncates later walking
costs or checks. Only a fully safe incoming step inside the actual final-goal
radius can terminate a rollout; the manager supplies that goal separately.
Retain the exact LFPC discretization and model state copying. Do not restore
old local-goal early termination, velocity optimization or planner-4 risk costs.

## Sampling and update

Launch defaults: N=10, K=256, iterations=2, temperature=.4, sigma_api=.35 rad,
max_api=pi/6, seed=0. Direct C++ default N remains 6. Supported N=1..32,
K=1..4096, iterations=1..16; finite positive temperature, finite nonnegative
sigma, nonnegative seed and validated fixed gait/weights are mandatory.
Configuration failure returns invalid zero command and clears warm start.

The .35 rad launch/default spread was selected after the mixed_turn_upper
lightweight route repeatedly stopped with .15 rad cold-start exploration.
Both that cornering route and mixed_stress reached their final goals with the
same 256x2 budget at .35 rad. Keep a default-budget cold-start 90-degree corridor
regression; do not repair sampling failure by shortening the checked horizon or
relaxing geometry. This evidence is scenario-specific, not completeness.

First proposal is the prior selected sequence shifted one action, with zero
tail. Without a retained sequence the manager seeds steering from a copied LFPC
following the current reference, advancing its projection along the polyline;
standalone unseeded calls use zeros. Seeding preserves a retained warm start and
never changes the actual LFPC. Revalidate every proposal on the current
state, corridor and observation. Seed the RNG once at initialization; reset
does not silently reseed it. Identical seed and identical calls/inputs reproduce
the samples on the same toolchain; live sensor timing may differ between runs.

Each iteration reserves slot 0 for the proposal and slot 1 for this cycle's
best verified candidate, or zero steering if none. This straight anchor retains
a simple alternative when the geometric seed turns poorly; it uses the same
budget and receives the same checks. Remaining slots are paired
positive/negative Gaussian perturbations around the proposal, clipped to bounds.
K<3 has no random exploration. Samples are full control sequences, not branches
of a quantized tree. Unsafe/nonfinite rollouts have infinite cost and zero weight.

```text
rho = min(finite J)
w[k] = exp(-(J[k]-rho)/temperature) / sum(exp(-(J-rho)/temperature))
proposal[n] = sum(w[k] * sampled_api[k][n])
```

All-invalid batches have all-zero weights; never assign uniform weights to
unsafe sequences. This finite sampled, clipped update with hard rejection is
an MPPI-style optimizer, not a certificate of strict local/global optimality.
No penalty or safety guarantee is inferred from the Gaussian distribution.

## Feasibility, selection and execution

One production rollout evaluator checks the support foot and every generated COM
sample for containment in the certified spatial corridor union. They may occupy
different regions. Do not query the static map per rollout point: FIRI owns that
certificate, and the manager checks its selected-map revision before admission.
Future human-cane envelopes need not fit a spatial region and are not checked
against static cells. Their purpose in rollouts is timed pedestrian SAT at the
actual future time and torso yaw. Current-pose hull containment and current pedestrian exclusion are certified by
FIRI generation before MPPI, regardless of timed collision enable. Intermediate step-root turn poses
are retained with their yaw/time for execution revalidation. Root corridor and
timed checks remain mandatory. See [directional-footprint-corridor.md](./directional-footprint-corridor.md).
A sequence must safely complete the horizon or reach the actual final goal
through a fully checked step.

The weighted mean must pass the same full rollout evaluator: a mean of feasible
controls can collide with static obstacles, cross the gap in a nonconvex union,
or violate time-dependent collision constraints. Never execute it unchecked.
If valid, it becomes the next proposal. Retain the lowest-cost initial feasible
proposal / feasible weighted iterate as the optimized candidate. Separately
retain the best verified sample or weighted iterate from this planning cycle.
On weighted failure, use that current-cycle candidate as the next proposal.
For a newly refreshed global reference the manager resets the old warm start and calls
seedReference again. The geometric proposal receives the same full rollout checks;
it never certifies safety or mutates the actual model.

If at least one weighted update is valid, select the optimized candidate;
otherwise select the current-cycle best verified fallback. This preserves
weighted optimization as the primary result rather than merely selecting a
random minimum. No feasible candidate means STOP. A final independent full
rollout from the original LFPC state must reproduce finite cost, step count and
terminal status exactly before returning its first command.

Existing manager accepted-observation transaction, source freshness, final
execution-epoch timed revalidation, frozen-LFPC STOP and stopped odometry remain
unchanged. Failed planning or manager STOP clears warm start. A retained sequence
is never an unvalidated trajectory from an older observation. The final source
check can still reject an apparently successful optimizer result. Lightweight
mode queues the first support phase after authorization and executes exact LFPC
samples on a 20ms timer, rechecking current geometry and remaining-step times before
each actual advance. Reoptimize steering at support boundaries, not each timer tick.

## Diagnostics and visualization

`/mpc/sample_trajectories` shows a bounded uniform subset of actual last-iteration
rollouts without drawing extra random values. Existing selected trajectory,
time-matched hull pairs and timed path outputs remain. Historical beam_evidence
is replaced by optimization_evidence (Float64MultiArray, version 1):

| Index | Meaning |
|---|---|
| 0 | schema version |
| 1,2,3 | horizon, samples per iteration, completed iterations |
| 4,5 | evaluated sampled sequences, valid sampled sequences |
| 6,7 | weighted rollout checks, weighted rejections |
| 8 | last iteration effective sample size, `1/sum(w*w)`; zero if all invalid |
| 9,10,11 | selected steps, actual-final terminal, independent replay valid |
| 12,13 | best verified cost across all candidates, selected executed cost |
| 14.. | I retained-best costs, followed by I weighted costs |

Nonfinite/unavailable costs serialize as -1. Retained-best cost is nonincreasing
by construction; weighted costs need not be monotonic. Evidence is published
before final execution authorization and does not certify that integration
occurred. Debug fields 28/29 indicate admission-time revalidation; STOP and the
lightweight execution_diagnostics describe current execution decisions.

Existing debug indices are retained. Field 1 is valid/evaluated sampled ratio,
7 is sampled count, 16/17 are valid sampled counts, 15 is -1 unavailable, 20 is
weighted-check performed, 21 is fallback because every weighted update failed.
Field 6 retains the legacy static-reject slot but now counts invalid LFPC model
outputs only; COM/foot union failures use field 12 (convex corridor rejects).
Field 19 remains spatial-only mode (zero when timed collision checking is active).
Fields 2/22 differ when a sampled fallback is cheaper than the selected weighted
solution. Counts include proposal/retained anchors in every iteration, so they
are not independent Monte Carlo trials or a collision probability. Rejection
counters additionally include weighted/replay/final checks.

## Regression and limits

Test actual production sampling, stable weights/all-invalid behavior, collision
of a mean across a nonconvex union, support-foot rejection/fallback, deterministic
seed, warm-start shifting, continuous cost reduction and exact sequence replay.
Keep local-arrival suffix cost/full-horizon checks, true final termination,
timed crossing/latency and real manager transaction/STOP/arrival tests.

Measure the full-footprint overhead with the opt-in same-budget benchmark in
docs/PLANNER3_CORRIDOR_USAGE.md. The earlier center/circle benchmark remains in
[lfpc-timed-collision.md](./lfpc-timed-collision.md); historical lightweight launch
evidence is recorded in docs/LFPC_MPPI_VALIDATION.md.
Finite sampling can miss feasible routes. Fixed gait does not add waiting/speed
actions. Current-frame DWCG excludes currently occupied pedestrian space even if timed SAT
predicts it will later be vacated; timed SAT remains an additional future check.
Periodic full global A* supplies candidate references before MPPI; see
[periodic-global-reference.md](periodic-global-reference.md). Sampled CV
collision checks and time-driven lightweight execution retain the timed-checker limitations;
no continuous-motion, physical-braking or real-hardware tracking claim.
