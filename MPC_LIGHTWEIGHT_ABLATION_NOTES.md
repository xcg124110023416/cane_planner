# MPC Lightweight Ablation Notes

## Two-stage direction

Stage 1: finish the current MPPI baseline and make its behavior explainable.

- Use fixed start/goal scenarios for repeatable lightweight simulation.
- Keep A/B/C/D-style ablations focused on which dynamic layer changes behavior.
- Treat B0 as an algorithmic baseline: pedestrians are present, but dynamic intervention is disabled.
- Treat C as the current Stage 1 candidate safety behavior: Dynamic Risk Field (DRF) soft cost only.
- Treat CPA, adaptive risk, stop/yield, and hard reject as internal variants only. They are not part of the Stage 1 final method unless explicitly reintroduced later.
- Do not try to tune the current risk field until it is perfect; use experiments to document its useful range and failure modes.

Stage 2: add a higher-level dynamic intent layer, then let MPPI track that intent.

- Detect whether a pedestrian will cross or occupy the global path corridor.
- Decide an interaction intent such as pass-behind, pass-ahead, yield, or continue.
- Feed MPPI a local target or constraint consistent with that intent.
- Avoid relying on MPPI risk weights alone to discover social interaction behavior.

## RAL thesis direction

The paper should not be framed as a perfect social-navigation MPC. The safer and
cleaner story is:

> A lightweight dynamic-obstacle avoidance system for smart-cane navigation,
> integrating MPPI local planning with a Dynamic Risk Field pedestrian cost, and
> validating its behavior through simulation ablations and real-world system
> trials.

Key constraints and motivation:

- This is a smart-cane system, not a fully actuated mobile robot.
- The human provides forward motion; the system mainly shapes heading/local path.
- The planner should be lightweight, real-time, explainable, and compatible with
  real odometry, mapping, and pedestrian tracking.
- The goal is not to solve general crowd navigation in Stage 1.

The expected paper content should be organized around:

1. Problem definition for smart-cane dynamic obstacle avoidance.
2. ROS system architecture: odometry/SLAM, map/collision layer, pedestrian
   tracking, global path, MPPI local planner, controller interface.
3. DRF-only MPPI baseline: global waypoint tracking plus pedestrian dynamic
   risk soft cost.
4. Lightweight simulation ablations: static, pedestrian-without-DRF, DRF-only,
   crossing risk-shape, and K sanity check.
5. Real-world minimum demo: static navigation and one-pedestrian dynamic
   interaction with bag/video/trajectory evidence.

## Method scope decisions

Use the name **Dynamic Risk Field (DRF)** instead of "basic risk". In Stage 1,
DRF means:

- `mpc_w_risk > 0`
- `mpc_enable_cpa=false`
- `mpc_enable_adaptive_risk=false`
- `mpc_enable_yield=false`
- `mpc_enable_stop_advice=false`
- `mpc_enable_stop_enforce=false`
- `mpc_enable_dynamic_hard_reject=false`

This is a soft pedestrian risk-field cost in MPPI rollouts. It is not a hard
constraint, not an intent selector, and not guaranteed to choose pass-behind.

CPA/TTC should not be presented as a method contribution. It should be omitted
from the main method, system diagram, and main experimental table. Keep the data
only as internal design evidence. Mention it only if there is a specific need to
explain why the final system does not include a CPA/TTC term:

> We tested an additional CPA/TTC-style risk term, but it increased path
> deviation without consistently improving closest pedestrian clearance, so it
> was not used in the final configuration.

Similarly, increasing MPPI samples from 200 to 300 is not a method contribution.
It is only a sanity check showing that the crossing-mode instability is not
resolved by more samples.

## Next priority: real-world minimum demo

After Stage 1 simulation is documented, stop expanding simulation parameter
search. The next priority is to prepare a minimum real-world demo.

Minimum real-world checklist:

- Confirm real odometry topic is available.
- Confirm map/cloud input is available.
- Confirm dynamic pedestrian tracking topic is available.
- Confirm MPC receives a goal and publishes debug metrics/waypoints/best
  trajectory.
- Record rosbag with odom, pedestrian tracking, MPC debug, waypoints, best
  trajectory, and relevant TF/map topics.
- Capture synchronized video or at least a clear external video.

Minimum real-world success criteria:

- The platform/cane can navigate from start toward goal.
- Pedestrian observations enter the MPC pipeline.
- DRF-only MPC produces a visible response to a moving pedestrian.
- The recorded bag can be analyzed with the same metric family used in
  lightweight simulation: path efficiency, deviation, closest encounter,
  collision clearance, pass mode, and planning time.

## Current findings

- Fixed mixed scenario start/goal is stored as `mixed` in `plan_manage/config/lightweight_fixed_goals.yaml`.
- Fixed crossing scenario start/goal is stored as `crossing` in the same file.
- Pedestrian scenarios are stored in `plan_manage/config/pedestrian_scenarios.yaml`.
- For mixed/head-on+overtake tests, internal CPA variants increased deviation and did not provide stable actual-clearance gains. Do not include CPA in the main paper unless needed as a rejected design note.
- For crossing tests, B0 tracks the global path best but ignores the pedestrian interaction semantics.
- For crossing tests, DRF with `mpc_w_risk=2.0` can switch behavior toward pass-behind, but current `risk_sigma_y=0.4` causes large path deviation.
- `analyze_mpc_eval.py` now reports closest robot-pedestrian encounter, collision clearance, and pass mode.
- Crossing tests show multi-modal MPPI behavior: the same risk shape can alternate between pass-ahead and pass-behind, so K changes are only a sanity check, not the main RAL contribution.
- Crossing K sanity check at `mpc_w_risk=2.0`, `mpc_risk_sigma_y=0.38`:
  - Recomputed with `robot_radius=0.35` for a conservative safety margin.
  - `K=200` repeated 3 times: one clear pass-behind candidate became a near encounter under the conservative radius, plus two near encounters; collision clearance min values were `0.105`, `0.029`, `0.033` m. A timing-outlier run was replaced by `K200_r3b`.
  - `K=300` repeated 3 times: one near encounter and two collisions under the conservative radius; collision clearance min values were `0.026`, `-0.120`, `-0.004` m.
  - Increasing K did not stabilize the interaction mode and did not improve closest clearance in this scenario. Stop K tuning for Stage 1.

## Near-term tests

Stage 1 minimum loop:

- Keep CPA/adaptive/yield/stop/hard-reject disabled unless explicitly testing those layers.
- Use B0 as the no-dynamic-intervention baseline.
- Use C as the DRF-only candidate.
- K sanity check is complete; use the result to justify Stage 2 intent-layer work instead of continuing parameter search.
