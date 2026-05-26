# Stage 2 Intent-aware DRF-MPPI Implementation Plan

> **For agentic workers:** Implement task-by-task with review checkpoints. Do not commit automatically; this project prefers manual commit decisions after validation.

**Goal:** Build the E group: DRF-only MPPI plus a lightweight crossing intent layer with `CONTINUE`, `PASS_BEHIND`, and later `YIELD`.

**Architecture:** Keep MPPI as the optimizer. Add an intent layer in `PlannerManager` that computes a mode and optional local target before calling `MpcController::plan()`. First implement E0 and E1; add E2 only after E1 is stable.

**Tech Stack:** ROS Noetic, C++14, catkin_tools, existing `PlannerManager`, existing `MpcController`, existing lightweight simulation and bag analyzer.

---

## File Structure

- Modify `/home/xcg/ws/src/cane_planner/plan_manage/include/plan_manager.h`
  - Add intent state enum, parameters, latched state, debug publishers, and helper declarations.
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`
  - Load intent parameters, compute intent state, publish debug info, and pass mode-conditioned target to MPPI.
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/launch/include/algorithm.launch`
  - Expose Stage 2 intent parameters.
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/launch/sim_kin_replan.launch`
  - Expose E group launch toggles.
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/scripts/analyze_mpc_eval.py`
  - Parse `/mpc/intent_state` and `/mpc/intent_debug`.
- Optional modify `/home/xcg/ws/src/cane_planner/plan_manage/scripts/record_lightweight_mpc_eval.sh`
  - Ensure intent topics are recorded.

## Task 1: Add E0 Intent Wrapper

**Files:**
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/include/plan_manager.h`
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/launch/include/algorithm.launch`
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/launch/sim_kin_replan.launch`

- [ ] Add params:

```text
mpc_enable_intent:=false
mpc_intent_enable_pass_behind:=false
mpc_intent_enable_yield:=false
```

- [ ] Add enum:

```cpp
enum MpcIntentState {
  INTENT_CONTINUE = 0,
  INTENT_PASS_BEHIND = 1,
  INTENT_YIELD = 2
};
```

- [ ] Add publishers:

```text
/mpc/intent_state
/mpc/intent_debug
/mpc/intent_target
```

- [ ] Add helper behavior:

```text
When mpc_enable_intent=false: no behavior change.
When mpc_enable_intent=true and pass/yield disabled: publish CONTINUE and keep original mpc_sim_goal_.
```

- [ ] Build:

```shell
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && catkin build plan_manage -DCMAKE_BUILD_TYPE=Release
```

- [ ] E0 smoke test:

```shell
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch plan_manage sim_kin_replan.launch planner:=3 use_pedestrians:=true pedestrian_scenario:=crossing mpc_w_risk:=2.0 mpc_enable_cpa:=false mpc_enable_adaptive_risk:=false mpc_enable_yield:=false mpc_enable_stop_advice:=false mpc_enable_stop_enforce:=false mpc_enable_dynamic_hard_reject:=false mpc_enable_intent:=true mpc_intent_enable_pass_behind:=false mpc_intent_enable_yield:=false
```

Expected: `/mpc/intent_state` publishes `CONTINUE`; trajectory should match C qualitatively.

## Task 2: Implement Crossing Candidate Detection

**Files:**
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/include/plan_manager.h`
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`

- [ ] Add parameters:

```text
mpc_intent_front_min:=0.3
mpc_intent_front_max:=4.0
mpc_intent_corridor_width:=0.7
mpc_intent_cross_speed:=0.15
mpc_intent_time_gap:=0.8
mpc_intent_min_robot_speed:=0.15
```

- [ ] Compute path-aligned frame:

```cpp
Eigen::Vector2d path_forward(mpc_sim_goal_(0) - current_com(0),
                             mpc_sim_goal_(1) - current_com(1));
if (path_forward.norm() < 1e-3) {
  path_forward = Eigen::Vector2d(std::cos(start_state_(2)), std::sin(start_state_(2)));
} else {
  path_forward.normalize();
}
Eigen::Vector2d path_left(-path_forward.y(), path_forward.x());
```

- [ ] For each pedestrian, compute:

```cpp
front, lateral, v_front, v_lateral, t_ped_to_path, t_robot_to_cross, time_gap
```

- [ ] Select the most relevant crossing candidate:

```text
Prefer smallest abs(time_gap) among valid candidates.
```

- [ ] Publish candidate fields in `/mpc/intent_debug`.

- [ ] Build and smoke test with E0 params.

Expected: intent state remains `CONTINUE`, but debug shows candidate detection values when the crossing pedestrian enters the corridor.

## Task 3: Implement PASS_BEHIND Target Feasibility

**Files:**
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/include/plan_manager.h`
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`

- [ ] Add parameters:

```text
mpc_intent_behind_dist:=0.8
mpc_intent_forward_bias:=0.6
mpc_intent_target_front_min:=0.4
mpc_intent_target_front_max:=3.5
mpc_intent_target_lateral_max:=1.8
mpc_intent_max_path_deviation:=1.5
mpc_intent_target_ped_clearance:=0.7
```

- [ ] Generate target:

```cpp
ped_dir = normalized(pedestrian_velocity)
predicted_ped = pedestrian_position + pedestrian_velocity * t_ped_to_path
raw_behind = predicted_ped - ped_dir * behind_dist
intent_target = raw_behind + path_forward * forward_bias
```

- [ ] Feasibility checks:

```text
target_front > target_front_min
target_front < target_front_max
abs(target_lateral) < target_lateral_max
distance_to_predicted_ped > target_ped_clearance
distance_to_global_path < max_path_deviation
collision_->evaluateCoarseEDT(target, -1.0) is free enough, or equivalent existing collision query
```

- [ ] Publish `/mpc/intent_target` as a marker.

- [ ] Build.

Expected: target marker appears only during feasible crossing conflicts.

## Task 4: Implement E1 Mode Selection and Hysteresis

**Files:**
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/include/plan_manager.h`
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`

- [ ] Add parameters:

```text
mpc_intent_pass_behind_hold_time:=1.2
mpc_intent_pass_behind_clear_time:=0.5
mpc_intent_yield_hold_time:=0.8
mpc_intent_yield_clear_time:=0.7
```

- [ ] Raw selection:

```text
No crossing candidate -> CONTINUE
Candidate and robot clearly first with safe time gap -> CONTINUE
Candidate and pedestrian clears first and target feasible -> PASS_BEHIND
Candidate and near-simultaneous or target infeasible -> YIELD only if mpc_intent_enable_yield=true; otherwise CONTINUE
```

- [ ] Hysteresis:

```text
PASS_BEHIND holds for pass_behind_hold_time.
YIELD holds until conflict clear for yield_clear_time.
CONTINUE only after clear condition is stable.
```

- [ ] Apply target:

```cpp
Eigen::Vector3d plan_goal = mpc_sim_goal_;
if (mpc_intent_state_ == INTENT_PASS_BEHIND && intent_target_valid_) {
  plan_goal << intent_target_(0), intent_target_(1), 0.0;
}
control = mpc_controller_->plan(lfpc_model_, plan_goal, obs_pos, obs_vel, obs_size);
```

- [ ] E1 launch test:

```shell
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch plan_manage sim_kin_replan.launch planner:=3 use_pedestrians:=true pedestrian_scenario:=crossing mpc_w_risk:=2.0 mpc_risk_sigma_y:=0.38 mpc_enable_cpa:=false mpc_enable_adaptive_risk:=false mpc_enable_yield:=false mpc_enable_stop_advice:=false mpc_enable_stop_enforce:=false mpc_enable_dynamic_hard_reject:=false mpc_enable_intent:=true mpc_intent_enable_pass_behind:=true mpc_intent_enable_yield:=false
```

Expected: mode enters `PASS_BEHIND` during crossing conflict and MPPI uses intent target without forced stop.

## Task 5: Add Analyzer Support

**Files:**
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/scripts/analyze_mpc_eval.py`
- Optional modify `/home/xcg/ws/src/cane_planner/plan_manage/scripts/record_lightweight_mpc_eval.sh`

- [ ] Record:

```text
/mpc/intent_state
/mpc/intent_debug
/mpc/intent_target
```

- [ ] Analyze:

```text
intent_states
intent_transition_count
PASS_BEHIND dwell time
YIELD dwell time
intent_target_valid_ratio
```

- [ ] Re-analyze one E1 bag:

```shell
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && rosrun plan_manage analyze_mpc_eval.py $(ls -td /home/xcg/ws/records/*light_crossing_E1* | head -1) --odom-topic /sim_odom --robot-radius 0.35
```

Expected: summary includes intent state counts, transitions, and dwell times.

## Task 6: Add E2 Yield Advice Only After E1

**Files:**
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`
- Modify launch files if needed.

- [ ] Enable `YIELD` only when:

```text
crossing conflict exists
behind target is infeasible or conflict is too close
mpc_intent_enable_yield=true
```

- [ ] Publish stop/yield advice without physical braking by default:

```text
mpc_enable_stop_advice=true
mpc_enable_stop_enforce=false
```

- [ ] Enforce stop only for a dedicated safety test, not the main E1 comparison.

- [ ] Check Go/No-Go stop constraint:

```text
E stop duration <= 30% task duration unless C has collision/near-collision.
```

## Verification Commands

Build:

```shell
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && catkin build plan_manage -DCMAKE_BUILD_TYPE=Release
```

Run C baseline:

```shell
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch plan_manage sim_kin_replan.launch planner:=3 use_pedestrians:=true pedestrian_scenario:=crossing mpc_w_risk:=2.0 mpc_risk_sigma_y:=0.38 mpc_enable_cpa:=false mpc_enable_adaptive_risk:=false mpc_enable_yield:=false mpc_enable_stop_advice:=false mpc_enable_stop_enforce:=false mpc_enable_dynamic_hard_reject:=false
```

Run E1:

```shell
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch plan_manage sim_kin_replan.launch planner:=3 use_pedestrians:=true pedestrian_scenario:=crossing mpc_w_risk:=2.0 mpc_risk_sigma_y:=0.38 mpc_enable_cpa:=false mpc_enable_adaptive_risk:=false mpc_enable_yield:=false mpc_enable_stop_advice:=false mpc_enable_stop_enforce:=false mpc_enable_dynamic_hard_reject:=false mpc_enable_intent:=true mpc_intent_enable_pass_behind:=true mpc_intent_enable_yield:=false
```

Record E1:

```shell
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && rosrun plan_manage record_lightweight_mpc_eval.sh light_crossing_E1_intent_fixed
```

Analyze E1:

```shell
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && rosrun plan_manage analyze_mpc_eval.py $(ls -td /home/xcg/ws/records/*light_crossing_E1_intent_fixed | head -1) --odom-topic /sim_odom --robot-radius 0.35
```

## Self-review

- Covers E0, E1, and E2.
- Keeps YIELD last to avoid "safe by stopping".
- Keeps MPPI unchanged except for a mode-conditioned target input from `PlannerManager`.
- Adds metrics needed by `RAL_DIRECTION_NOTES.md`.
- Does not require commits during implementation.
