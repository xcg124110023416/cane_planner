# MPC Stage 1 Experiment Tables

This file summarizes the lightweight simulation ablations used to characterize
the current MPPI dynamic-risk baseline. Stage 1 is intentionally limited to
explaining the present controller behavior. It is not a parameter-search track
for a final social-navigation policy.

## Main Ablation: Mixed Pedestrian Scene

The `mixed` scenario includes pedestrians that create head-on and same-direction
interaction pressure. B0 keeps pedestrians in the scene but disables dynamic
intervention in the MPC decision.

| Scene | Group | DRF cost | Pedestrians | Final goal error (m) | Path efficiency | Deviation mean/max (m) | Min collision clearance (m) | Takeaway |
|---|---|---|---|---:|---:|---:|---:|---|
| mixed | A | off | no | 0.116 | 0.821 | 0.232 / 0.665 | NA | Static navigation reaches the goal. |
| mixed | B0 | off | yes | 0.032 | 0.837 | 0.147 / 0.289 | 0.447 | Pedestrians are present, but MPC mostly tracks the global path. |
| mixed | C | on | yes | 0.064 | 0.810 | 0.187 / 0.562 | 0.127 | DRF gives a small, controlled response but can pass close to the overtaking pedestrian. |

Stage 1 interpretation: DRF-only is the clean Stage 1 candidate. It keeps the
navigation behavior close to the global path while introducing a dynamic
pedestrian response. This table should be the main mixed-scene ablation in the
paper.

## Internal Rejected Variants

These runs are kept as internal design evidence, not as main-paper baselines.
They should not appear in the method or main experimental table unless needed
to answer why the final system does not include additional predictive terms.

| Scene | Group | Variant | Final goal error (m) | Path efficiency | Deviation mean/max (m) | Min collision clearance (m) | Internal takeaway |
|---|---|---|---:|---:|---:|---:|---|
| mixed | D | DRF + CPA | 0.139 | 0.676 | 1.155 / 3.502 | 0.727 | CPA strongly increases global-path deviation. |
| mixed | D1 | DRF + CPA=0.4 | 0.098 | 0.756 | 0.313 / 1.260 | 0.282 | Lower CPA weight still increases deviation without clear safety gain. |

Internal interpretation: CPA/TTC-style risk was tested during development but
is not part of the final Stage 1 system. Do not present it as a method
contribution or a required comparison framework.

## Crossing Risk-Shape Sweep

The `crossing` scenario isolates a single crossing pedestrian. The purpose is
to observe whether the continuous risk field can reliably produce a crossing
interaction strategy such as pass-behind.

The crossing clearance values below use the conservative analysis assumption
`robot_radius=0.35m`, `ped_radius=0.25m`.

| Scene | Group | `w_risk` | `risk_sigma_y` | Closest pass mode | Path efficiency | Deviation mean/max (m) | Min collision clearance (m) | Takeaway |
|---|---|---:|---:|---|---:|---:|---:|---|
| crossing | B0 | 0.0 | NA | PASS_AHEAD / straight | 0.996 | 0.082 / 0.272 | 0.337 | Best path tracking, but no dynamic-interaction intent. |
| crossing | C20_y025 | 2.0 | 0.25 | PASS_AHEAD | 0.995 | 0.071 / 0.186 | 0.063 | Risk field is too narrow; the robot passes too close. |
| crossing | C20_y030 | 2.0 | 0.30 | PASS_AHEAD | 0.997 | 0.074 / 0.202 | 0.319 | Safer clearance, but still behaves like straight-path passing. |
| crossing | C20_y035 | 2.0 | 0.35 | NEAR / PASS_BEHIND | 0.900 | 0.532 / 1.454 | 0.035 | Starts to switch behind the pedestrian, but too close. |
| crossing | C20_y038 | 2.0 | 0.38 | unstable | 0.993 | 0.102 / 0.332 | 0.394 | A single run can look good, but repeated runs are not stable. |
| crossing | C20_y040 | 2.0 | 0.40 | PASS_BEHIND | 0.893 | 0.580 / 1.502 | 0.067 | Clear pass-behind behavior, but with larger path deviation. |

Stage 1 interpretation: the DRF cost can change interaction behavior,
but the pass-ahead/pass-behind decision is not a stable semantic choice under
the continuous risk cost alone.

## Crossing K Sanity Check

This check tests whether increasing MPPI samples stabilizes the crossing mode.
It uses `w_risk=2.0` and `risk_sigma_y=0.38`. The check is deliberately small:
it is evidence for whether to keep tuning K, not a main contribution.

| Scene | K | Runs | PASS_BEHIND | PASS_AHEAD/SIDE | NEAR | COLLISION | Min clearance range (m) | Deviation mean range (m) | Takeaway |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| crossing y=0.38 | 200 | 3 | 1 | 0 | 2 | 0 | 0.029 to 0.105 | 0.569 to 0.612 | Interaction mode remains unstable; the valid rerun replaced a timing-outlier run. |
| crossing y=0.38 | 300 | 3 | 0 | 0 | 1 | 2 | -0.120 to 0.026 | 0.503 to 0.702 | More samples did not improve stability or closest clearance. |

Stage 1 interpretation: increasing MPPI samples from 200 to 300 does not solve
the multi-modal crossing behavior. Under the conservative radius assumption,
some repeats are classified as unsafe/near-collision even when the RViz view
does not show an obvious visual collision. This supports moving future work
toward a higher-level interaction intent layer instead of continuing K or
risk-shape tuning.

## Stage 1 Conclusion

The lightweight simulation results support the following claim:

> The Dynamic Risk Field (DRF) cost can alter pedestrian interaction behavior, but
> pure MPPI risk costs do not reliably choose a stable pass-ahead or pass-behind
> mode in crossing interactions. Increasing samples from 200 to 300 did not
> stabilize the crossing behavior, motivating a higher-level intent layer in the
> next stage.

## Generated Figures

Generated trajectory figures are stored under `/home/xcg/ws/records/stage1_figures`:

- `stage1_mixed_ablation.png`: mixed-scene A/B0/C/D1 trajectory comparison.
- `stage1_crossing_risk_shape.png`: crossing risk-shape sweep.
- `stage1_crossing_k_sanity.png`: K=200 and K=300 repeat overview.
- `stage1_crossing_k200_repeats.png`: K=200 repeat runs only.
- `stage1_crossing_k300_repeats.png`: K=300 repeat runs only.
