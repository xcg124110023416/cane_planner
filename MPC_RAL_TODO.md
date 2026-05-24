# MPC Dynamic Avoidance TODO for Guide-Cane Navigation

This document records the next technical steps for improving the current
guide-cane MPC dynamic obstacle avoidance system toward a publishable RAL-level
system contribution.

## Current Baseline

The current simulation stack is:

- Gazebo dynamic corridor world with pedestrian models.
- FAST-LIO-SAM map, FAST-LIO-Localization relocalization.
- Static global A* path from `corridor_dynamic_175.pcd`.
- Local SDF/ESDF from `/corrected_current_pcd`.
- Gazebo pedestrian truth bridge publishing:
  - `/onboard_detector/dynamic_obstacles_info`
  - `/onboard_detector/dynamic_bboxes`
  - `/gazebo_pedestrian_truth/visualization`
- CERLAB-style dynamic free region clearing before local map inflation.
- MPC/MPPI local planner in `path_searching`.

Important current guide-cane assumption:

- The human provides forward walking motion.
- The cane provides left/right steering guidance.
- MPC should mainly optimize steering direction, not behave like a fully
  autonomous mobile robot with active braking and lateral drive.

Current relevant parameters:

- `mpc/fix_step_params = true`
- `mpc/nominal_al = user_step_length`
- `manager/lookahead_dist = 2.2`
- `mpc/w_risk = 2.0`
- `mpc/risk_tau = 0.8`
- `mpc/risk_halo_scale = 2.5`

## Target Contribution

The target should not be framed as a brand-new generic MPC algorithm. A more
defensible contribution is:

> A human-driven guide-cane navigation framework that performs dynamic
> pedestrian-aware local steering under underactuated shared-control constraints,
> combining static map following, dynamic obstacle clearing, and predictive
> pedestrian risk-aware MPPI.

Expected contribution outputs:

1. A formal guide-cane shared-control motion model.
2. A pedestrian-aware MPPI local planner that uses time-to-conflict risk, not
   only tuned Gaussian risk.
3. A complete dynamic obstacle map-cleaning and planning pipeline.
4. Quantitative simulation and, if available, real-world validation.

## Priority 1: Add CPA/TTC Dynamic Conflict Risk

### Purpose

The current Gaussian risk field can avoid pedestrians, but it often reacts late
for perpendicular or high-crossing-angle pedestrian motion. A time-related
conflict term should make the planner react earlier when the predicted robot
trajectory and pedestrian trajectory will intersect soon.

### Technical Idea

For each predicted robot rollout point `p_r(t)` and pedestrian prediction
`p_o(t)`, compute a conflict risk based on:

- CPA: closest point of approach.
- TTC: time to collision or time to closest approach.
- Relative velocity and crossing angle.

Example cost shape:

```text
d_min = min_t ||p_r(t) - p_o(t)||
t_cpa = argmin_t ||p_r(t) - p_o(t)||
risk_cpa = exp(-d_min^2 / sigma_d^2) * exp(-t_cpa / tau)
```

### Expected Code Output

- Extend `path_searching/include/path_searching/dynamic_risk_field.h`.
- Extend `path_searching/src/dynamic_risk_field.cpp`.
- Use the new risk term in `path_searching/src/mpc_controller.cpp`.
- Add launch parameters in
  `plan_manage/launch/include/algorithm.launch`, for example:
  - `mpc/ttc_enable`
  - `mpc/ttc_sigma_d`
  - `mpc/ttc_tau`
  - `mpc/ttc_weight`
  - `mpc/ttc_hard_threshold`

### Expected Evaluation Output

- Compare perpendicular crossing before/after.
- Measure earliest avoidance time.
- Measure minimum pedestrian distance.
- Measure stop count and success rate.

## Priority 2: Use Pedestrian Size and Safety Radius in MPC

### Purpose

The current dynamic obstacle callback stores obstacle size, but MPC currently
plans with only position and velocity. This makes the dynamic obstacle boundary
less physically meaningful and forces avoidance behavior to depend too much on
Gaussian sigma tuning.

### Technical Idea

Use an effective radius:

```text
effective_radius = pedestrian_radius + cane_radius + safety_margin
```

Then use this radius in:

- hard dynamic collision checks;
- Gaussian/CPA risk normalization;
- visualization of dynamic safety regions.

### Expected Code Output

- Pass `dynObsSize_` from `PlannerManager::mpcSimStep()` into
  `MpcController::plan()`.
- Extend `MpcController::plan()` and `rolloutBatch()` signatures.
- Use per-obstacle radius in dynamic hard checks.
- Keep backward-compatible default behavior if size is unavailable.

### Expected Evaluation Output

- Show that avoidance clearance corresponds to pedestrian/cane geometry.
- Reduce parameter sensitivity of `risk_sigma_x` and `risk_sigma_y`.

## Priority 3: Publish MPC Debug and Evaluation Metrics

### Purpose

For tuning and paper experiments, RViz-only observation is not enough. The
system should expose numeric metrics that explain why MPC chose a control.

### Metrics to Publish or Log

- Current lookahead target.
- Selected control: `al`, `aw`, `api`.
- Best trajectory total cost.
- Cost breakdown:
  - goal cost;
  - dynamic risk cost;
  - static collision/proximity cost;
  - steering cost;
  - steering smoothness cost;
  - TTC/CPA cost.
- Minimum predicted pedestrian distance.
- Minimum predicted TTC/CPA time.
- Whether a hard dynamic constraint was triggered.
- MPC runtime in milliseconds.

### Expected Code Output

- Add a debug message or lightweight ROS topics under `/mpc/debug/*`.
- Alternatively add a CSV logger controlled by a launch parameter:
  - `mpc/debug_enable`
  - `mpc/log_csv`
  - `mpc/log_path`

### Expected Evaluation Output

- Easier parameter tuning.
- Quantitative plots for paper figures.
- Evidence that the proposed conflict risk improves behavior.

## Priority 4: Adaptive Lookahead and Adaptive Risk Weight

### Purpose

Fixed lookahead and fixed risk weight are simple but can be suboptimal:

- too short: late avoidance and local dithering;
- too long: over-committed tracking and poor narrow-space behavior;
- fixed risk: either too conservative in free space or too late near crossing
  pedestrians.

### Technical Idea

Adaptive lookahead:

```text
lookahead = clamp(k_v * v + k_clearance * clearance, min_lookahead, max_lookahead)
```

Adaptive risk weight:

```text
w_risk_eff = w_risk * f(min_ttc, min_distance)
```

### Expected Code Output

- Add optional adaptive mode, disabled by default until validated.
- Keep current fixed values as fallback.
- Add launch parameters:
  - `manager/adaptive_lookahead`
  - `manager/lookahead_min`
  - `manager/lookahead_max`
  - `mpc/adaptive_risk_weight`

### Expected Evaluation Output

- Compare fixed vs adaptive in:
  - empty corridor;
  - crossing pedestrian;
  - multi-pedestrian corridor.

## Priority 5: Formalize Guide-Cane Shared-Control Model

### Purpose

This is important for publication. The novelty is not generic MPC, but using
MPC under guide-cane constraints:

- forward walking motion is human-supplied;
- the cane gives only steering guidance;
- active braking and lateral drive are unavailable or limited;
- guidance must be smooth and comfortable.

### Expected Documentation Output

Add a technical note or paper draft section describing:

- state definition;
- external human walking input;
- control input;
- constraints;
- cost function;
- why this differs from mobile robot navigation.

Suggested file:

- `docs/guide_cane_mpc_model.md`

## Priority 6: Experiment Suite

### Required Scenarios

1. Empty corridor path following.
2. Head-on pedestrian.
3. Perpendicular crossing pedestrian.
4. Diagonal crossing pedestrian.
5. Same-direction slower pedestrian.
6. Multiple pedestrians in corridor.

### Required Baselines

At minimum:

1. Static-only local ESDF avoidance.
2. Current Gaussian-risk MPPI.
3. Gaussian + CPA/TTC-risk MPPI.

Optional:

1. DWA-like local planner.
2. TEB-like local planner.
3. Velocity-obstacle style baseline.

### Required Metrics

- Success rate.
- Collision rate.
- Minimum pedestrian distance.
- Minimum TTC/CPA.
- Navigation time.
- Path deviation from global A* path.
- Number of stops.
- Mean/max steering angle.
- Mean/max steering rate.
- Left-right oscillation count.
- MPC mean/max runtime.
- CPU usage.

## Priority 7: Real or Semi-Real Validation

### Purpose

RAL-level systems work is much stronger with real or semi-real validation.

### Minimum Useful Validation

- Real corridor.
- Real or pushed cane platform.
- One or two pedestrians.
- Localization and local map active.
- Record trajectory, pedestrian distance, and steering command.

### Human Subject Note

Blind-user experiments may require ethics approval. Early validation can use:

- sighted participants;
- blindfolded participants if approved;
- trained operator pushing the cane.

## Suggested Implementation Order

1. Add CPA/TTC risk term.
2. Pass and use pedestrian size/safety radius in MPC.
3. Add MPC debug metrics.
4. Run and tune perpendicular crossing scenario.
5. Add adaptive lookahead/risk only if fixed parameters remain insufficient.
6. Build experiment scripts and collect baseline results.
7. Write the guide-cane shared-control model document.

## Notes for Future Codex Sessions

- Build with `catkin build`, not `catkin_make`.
- Do not commit frequently; commit only when the user asks.
- Preserve the current Gazebo truth bridge and CERLAB-style dynamic clearing.
- Keep `/onboard_detector/dynamic_obstacles_info` as the planner-facing dynamic
  obstacle interface so LV-DOT or another detector can be connected later.
- The current launch flow is:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch plan_manage gazebo_localization_mpc.launch start_teleop:=true rviz:=true
```

Then, in a second terminal:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch fast_lio_localization_qn sim_corridor_175_fastlio.launch rviz:=false
```

