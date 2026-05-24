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
- `mpc/risk_halo_scale = 2.0`

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

Status: implemented in the first prototype. The current implementation adds an
optional CPA/TTC cost term in `DynamicRiskField` and applies it inside MPC
rollout using the predicted CoM point velocity and pedestrian velocity.

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

Status: implemented in the first prototype. `PlannerManager` now passes cached
dynamic obstacle sizes into `MpcController`, and MPC uses an effective dynamic
safety radius for hard collision rejection and CPA/TTC clearance calculation.

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

Status: implemented in the first lightweight prototype. The planner publishes
`/mpc/debug_metrics` as `std_msgs/Float64MultiArray` with runtime, valid sample
ratio, best cost, minimum dynamic clearance, minimum CPA time, rejection counts,
sample count, plan-valid flag, and best-trajectory minimum dynamic
clearance/CPA time. The original fields are preserved and the best-trajectory
fields are appended as `data[10]` and `data[11]`.

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
- Minimum predicted pedestrian distance of the selected best trajectory.
- Minimum predicted TTC/CPA time of the selected best trajectory.
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
- Clearer distinction between bad sampled rollouts and the selected trajectory.

## Priority 3B: Event-Style MPC Monitor

Status: implemented as a lightweight first version. The previous validation
workflow relied on raw `rostopic echo`
for `/mpc/stop_reason`, `/mpc/stop_advice`, and `/mpc/debug_metrics`. These
topics print continuously and are hard to inspect during simulation. The new
monitor tool only prints meaningful state changes and compact summaries.

### Purpose

Make validation usable without watching several scrolling terminals. The monitor
should report stop/yield transitions, reason changes, and selected debug values
only when they matter.

### Expected Tool Behavior

Example output:

```text
[32.9s] OK -> YIELD_TO_CROSSING_PEDESTRIAN valid=0.77 clearance=-0.08 ttc=0.00
[35.1s] YIELD_TO_CROSSING_PEDESTRIAN -> OK valid=0.95 clearance=0.72 ttc=1.40
```

The implemented monitor defaults to a quiet validation mode: it prints only
STOP entry, STOP release, long-active warnings, and the final summary. Per-frame
reason changes can be enabled when debugging with:

```bash
rosrun plan_manage mpc_event_monitor.py _print_reason_updates:=true
```

### Expected Code Output

Implemented script:

- `plan_manage/scripts/mpc_event_monitor.py`

Suggested subscriptions:

- `/mpc/stop_reason`
- `/mpc/stop_advice`
- `/mpc/debug_metrics`

Optional later extensions:

- write a CSV event log;
- count stop/yield events per run;
- report stop duration and release time;
- warn if stop/yield remains active longer than a configurable threshold.

## Priority 3C: Rosbag Recording Script for MPC Evaluation

Status: implemented as a first lightweight recorder and analyzer. The recorder
stores the core numeric, state, trajectory, and pedestrian-truth topics needed
to replay a run without storing large point clouds by default. The analyzer
prints and saves a compact experiment summary from the recorded bag.

### Purpose

Move from manual terminal observation to repeatable experiment data collection.
Each run should leave a bag file plus metadata so later scripts can compute
stop count, stop duration, minimum pedestrian distance, path progress, and MPC
runtime statistics.

### Implemented Tool

- `plan_manage/scripts/record_mpc_eval.sh`
- `plan_manage/scripts/analyze_mpc_eval.py`

Usage:

```bash
rosrun plan_manage record_mpc_eval.sh crossing_test_01
```

Analyze a completed run:

```bash
rosrun plan_manage analyze_mpc_eval.py /home/xcg/ws/records/YYYYMMDD_HHMMSS_crossing_test_01
```

The analyzer writes `summary.txt` next to the bag by default.

Default output directory:

```text
/home/xcg/ws/records/YYYYMMDD_HHMMSS_crossing_test_01/
```

Override output root:

```bash
MPC_EVAL_RECORD_DIR=/tmp/mpc_records rosrun plan_manage record_mpc_eval.sh crossing_test_01
```

### Recorded Topics

- `/clock`
- `/mpc/debug_metrics`
- `/mpc/stop_advice`
- `/mpc/stop_reason`
- `/mpc/path`
- `/mpc/best_traj`
- `/mpc/current_waypoint`
- `/mpc/waypoints`
- `/localization_odom`
- `/cmd_vel_footprint`
- `/onboard_detector/dynamic_obstacles_info`
- `/onboard_detector/dynamic_bboxes`
- `/gazebo_pedestrian_truth/visualization`

Large point-cloud topics are intentionally excluded in the first version to
keep bags small and focused on evaluation metrics.

### First Analyzer Outputs

- run duration;
- stop count and total stop duration;
- stop reason counts;
- MPC runtime statistics;
- valid sample ratio statistics;
- plan-valid ratio;
- actual path length, straight-line progress, path efficiency, speed;
- final goal error and deviation from the global waypoint polyline;
- best/global dynamic clearance and CPA/TTC statistics;
- approximate robot-pedestrian center distance and geometry clearance from
  `/localization_odom` and `/onboard_detector/dynamic_obstacles_info`;
- per-stop event list.

### Multi-Run Summary Table

Implemented:

- `plan_manage/scripts/summarize_mpc_eval.py`

Usage:

```bash
rosrun plan_manage summarize_mpc_eval.py /home/xcg/ws/records
```

This scans all `summary.txt` files under the records directory and writes:

```text
/home/xcg/ws/records/summary_table.csv
```

The CSV is intended for baseline/ablation comparison and includes run name,
stop count, stop duration, minimum actual clearance, MPC runtime, path length,
path efficiency, final goal error, and key best/global dynamic-risk metrics.

## Priority 4: Adaptive Lookahead and Adaptive Risk Weight

Status: partially implemented. The first prototype enables adaptive dynamic-risk
weighting only; adaptive lookahead is intentionally deferred until the risk
behavior is validated. The soft dynamic risk weight scales up near low dynamic
clearance or short CPA/TTC, while hard safety checks remain unchanged.

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

## Priority 4B: Stop/Wait Advice for Unavoidable Dynamic Conflict

Status: implemented in the first prototype. The planner publishes
`/mpc/stop_advice` and `/mpc/stop_reason`; by default the Gazebo command is
also forced to zero when an imminent dynamic conflict is detected. This models
the real guide-cane behavior where the system can tell the user to stop/wait
through voice feedback, even though it cannot provide active braking force.

### Purpose

Some pedestrian conflicts cannot be solved by steering alone, especially when a
pedestrian suddenly changes direction or cuts in too close. The guide cane
should not pretend that a safe steering-only solution exists; it should issue a
clear stop/wait instruction.

### Current Trigger Signals

- no valid MPC trajectory while dynamic obstacles exist;
- low valid-sample ratio together with predicted dynamic clearance below the
  stop threshold;
- low valid-sample ratio together with short CPA/TTC.

## Priority 4C: Pedestrian Yielding / Right-of-Way Policy

Status: implemented in the first prototype. The planner can stop/wait for a
near front crossing pedestrian when continuing to move would likely occupy the
pedestrian's path. This is a behavior-layer rule above MPC collision avoidance.
Current launch tuning makes yielding trigger earlier and reduces the far-field
dynamic halo so parallel pedestrians do not push the cane too hard toward walls.

### Purpose

Avoid the guide cane slowly cutting into or blocking a pedestrian's route when
the safer and more socially natural action is to stop and let the pedestrian
cross first.

### Current Trigger Signals

- pedestrian is in front of the cane within a configurable distance;
- pedestrian is laterally close to the robot path corridor;
- pedestrian has enough lateral crossing speed and is moving toward the path
  centerline;
- pedestrian arrival time at the path overlaps the robot's estimated arrival
  time at the crossing region.

## Priority 4D: Stop/Yield Hysteresis State Machine

Status: implemented in the first prototype. The planner now keeps a latched
stop/yield state after a dynamic conflict is detected. It enters stop quickly,
then releases only after a minimum hold time and a continuous clear interval.
Crossing-pedestrian release uses a slightly expanded front/lateral/time window
so the system does not alternate between stop and go near a threshold.

### Purpose

Frame-by-frame stop decisions can flicker when pedestrian tracking, MPC
sampling, or localization hovers around a threshold. For a guide cane, this is
especially undesirable because a voice stop/wait prompt should be stable enough
for a human user to follow.

### Current Parameters

- `mpc/stop_hold_time`
- `mpc/stop_release_clear_time`
- `mpc/yield_release_front_margin`
- `mpc/yield_release_lateral_margin`
- `mpc/yield_release_time_margin`

### Expected Evaluation Output

- Check that crossing-pedestrian stop does not flicker.
- Check that the robot resumes motion after the pedestrian leaves.
- Check that same-direction or parallel pedestrians do not cause unnecessary
  long stops.

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

Status: baseline launch toggles are implemented for the Gazebo localization MPC
flow. The default launch keeps the full method enabled, while ablation runs can
disable individual modules from the command line.

### Implemented Baseline Toggles

Available on `gazebo_localization_mpc.launch`, forwarded through
`gazebo_mpc.launch` into `include/algorithm.launch`:

- `mpc_enable_cpa`: enable/disable CPA/TTC conflict risk.
- `mpc_enable_adaptive_risk`: enable/disable adaptive dynamic-risk weight.
- `mpc_enable_yield`: enable/disable pedestrian yielding/right-of-way rule.
- `mpc_enable_stop_enforce`: enable/disable actually forcing zero command when
  stop advice is active.

Example runs:

```bash
# Full method
roslaunch plan_manage gazebo_localization_mpc.launch start_teleop:=true rviz:=true

# No CPA/TTC risk
roslaunch plan_manage gazebo_localization_mpc.launch start_teleop:=true rviz:=true mpc_enable_cpa:=false

# Fixed risk weight
roslaunch plan_manage gazebo_localization_mpc.launch start_teleop:=true rviz:=true mpc_enable_adaptive_risk:=false

# No yielding rule
roslaunch plan_manage gazebo_localization_mpc.launch start_teleop:=true rviz:=true mpc_enable_yield:=false

# Stop advice is still published, but Gazebo command is not forced to zero
roslaunch plan_manage gazebo_localization_mpc.launch start_teleop:=true rviz:=true mpc_enable_stop_enforce:=false
```

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
5. Add an event-style MPC monitor for practical validation.
6. Add stop/yield hysteresis to make stop prompts stable.
7. Build experiment scripts and collect baseline results.
8. Write the guide-cane shared-control model document.

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
