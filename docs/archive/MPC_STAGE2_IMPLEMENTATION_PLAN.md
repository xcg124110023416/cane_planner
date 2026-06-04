# Stage 2 Implementation Plan: Spatiotemporal Yield Supervisor

## Current Direction

Stage 2 is now the lightweight interaction layer on top of DRF-MPPI:

```text
DRF-MPPI + spatiotemporal yield supervisor
```

The goal is not to steer the cane through a special interaction target. The goal is to prevent the guide cane from rushing through a pedestrian-priority crossing conflict. When the robot's nominal future path occupancy and a pedestrian's predicted path occupancy overlap in time and space, `PlannerManager` publishes stop advice. After the conflict clears, DRF-MPPI continues with the original goal and its normal dynamic risk field.

Current active sets:

```text
scene in {none, crossing}
mode  in {CONTINUE, YIELD}
```

The current Stage 2 mainline does not include `PASS_BEHIND`, `PASS_AHEAD`, `PASS_SIDE`, interaction target redirection, or MPPI interaction social-cost terms.

## Active Runtime Flow

```text
dynamic obstacle observation
    -> path-aligned robot frame
    -> per-pedestrian crossing/debug timing
    -> spatiotemporal conflict check
    -> scene/mode/debug topics
    -> /mpc/stop_advice and /mpc/stop_reason
    -> optional stop enforcement
    -> original DRF-MPPI plan with unchanged mpc_sim_goal_
```

The active stop reason is:

```text
INTERACTION_YIELD_CONFLICT
```

`MpcController::plan()` keeps its original call shape:

```cpp
mpc_controller_->plan(lfpc_model_, mpc_sim_goal_, obs_pos, obs_vel, obs_size)
```

No interaction context is passed into `MpcController`.

## Main Files

- `/home/xcg/ws/src/cane_planner/plan_manage/include/plan_manager.h`
- `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`
- `/home/xcg/ws/src/cane_planner/plan_manage/launch/include/algorithm.launch`
- `/home/xcg/ws/src/cane_planner/plan_manage/launch/sim_kin_replan.launch`
- `/home/xcg/ws/src/cane_planner/plan_manage/scripts/analyze_mpc_eval.py`
- `/home/xcg/ws/src/cane_planner/plan_manage/scripts/analyze_stage2_batch.py`
- `/home/xcg/ws/src/cane_planner/STAGE2_EXPERIMENT_COMMANDS.md`

## Parameters To Keep

These parameters define the current Stage 2 behavior and should remain visible in launch files:

```text
mpc_enable_interaction
mpc_interaction_enable_yield
mpc_interaction_st_horizon
mpc_interaction_yield_trigger_time
mpc_interaction_robot_radius
mpc_interaction_yield_safety_margin
mpc_interaction_front_min
mpc_interaction_front_max
mpc_interaction_corridor_width
mpc_interaction_cross_speed
mpc_interaction_time_gap
mpc_interaction_min_robot_speed
mpc_interaction_cpa_horizon
mpc_interaction_cpa_dist
mpc_interaction_use_cpa_check
mpc_interaction_stop_release_clear_time
mpc_interaction_post_yield_grace_time
```

Removed items should stay out of the active mainline: the old switch that could disable the ST supervisor, threshold-based scene/mode FSM parameters, pass-behind mode toggles, interaction target parameters, and MPPI interaction social-cost parameters.

## Interaction Debug Layout

Current `/mpc/interaction_debug` uses the compact 35-field layout:

```text
0  interaction_enable
1  scene
2  mode
3  candidate_valid
4  obs_idx
5  front
6  lateral
7  v_front
8  v_lateral
9  t_ped_to_path
10 t_robot_to_cross
11 time_gap
12 t_cpa
13 d_cpa
14 cpa_conflict
15 robot_speed_used
16 path_forward_x
17 path_forward_y
18 r_crossing
19 crossing_confirm_count
20 crossing_clear_count
21 risk_front
22 signed_t_ped_to_path
23 ped_before_path
24 ped_at_or_after_path
25 yield_required
26 st_conflict
27 st_t_conflict
28 st_d_conflict
29 st_safety_radius
30 st_robot_s
31 st_ped_s
32 st_path_occupied
33 st_path_t_enter
34 st_path_t_exit
```

Some names are historical, such as `r_crossing`, `crossing_confirm_count`, and `risk_front`. They are still useful for current bag analysis and topic compatibility, but the active mode decision is the spatiotemporal conflict supervisor, not the removed threshold FSM.

## Experiment Groups

The current comparison groups are:

```text
B0: no dynamic intervention
C: DRF-only
E0b: Stage 2 debug topics enabled, interaction yield disabled
E1: DRF-MPPI + spatiotemporal yield supervisor
```

The main experimental claim should be scoped to E1:

```text
Spatiotemporal yield supervision prevents rushing in crossing conflicts while avoiding the large detours seen in DRF-only crossing runs.
```

Do not claim that Stage 2 implements active pass-behind guidance.

## Validation Commands

Build:

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && catkin build plan_manage -DCMAKE_BUILD_TYPE=Release
```

Run the current E1 crossing scenario:

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch plan_manage sim_kin_replan.launch planner:=3 use_pedestrians:=true pedestrian_scenario:=crossing mpc_w_risk:=2.0 mpc_risk_sigma_y:=0.38 mpc_enable_cpa:=false mpc_enable_stop_advice:=true mpc_enable_stop_enforce:=true mpc_enable_interaction:=true mpc_interaction_enable_yield:=true mpc_interaction_st_horizon:=4.0 mpc_interaction_yield_trigger_time:=2.5 mpc_interaction_robot_radius:=0.25 mpc_interaction_yield_safety_margin:=0.20
```

Record:

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && mkdir -p /home/xcg/ws/records/stage2_current_E1 && rosbag record --duration=45 -O /home/xcg/ws/records/stage2_current_E1/E1_run01.bag /clock /sim_odom /simulation_generator/odom /onboard_detector/dynamic_obstacles_info /mpc/interaction_scene /mpc/interaction_mode /mpc/interaction_debug /mpc/debug_metrics /mpc/stop_advice /mpc/stop_reason /mpc/path /mpc/best_traj /mpc/current_waypoint /mpc/waypoints /cmd_vel_footprint
```

Publish the fixed crossing goal:

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && rosrun plan_manage publish_lightweight_fixed_goal.py _scenario:=crossing _goal_publish_duration:=0.0
```

Analyze one bag:

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && rosrun plan_manage analyze_mpc_eval.py /home/xcg/ws/records/stage2_current_E1/E1_run01.bag --odom-topic /sim_odom --robot-radius 0.25
```

## Completion Criteria

- `catkin build plan_manage -DCMAKE_BUILD_TYPE=Release` succeeds.
- E0b produces interaction debug topics but no interaction stop.
- E1 produces `CONTINUE -> YIELD -> CONTINUE` when a true crossing conflict exists.
- E1 stop reason is `INTERACTION_YIELD_CONFLICT`.
- `mpc_sim_goal_` is not redirected by interaction logic.
- `MpcController` has no interaction social-cost context.
- Analysis scripts parse the current compact debug layout and do not carry old bag compatibility branches.
