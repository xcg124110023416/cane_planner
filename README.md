# Cane Planner

`cane_planner` is the navigation, planning, and control package set for a ROS1
smart-cane navigation system. The current branch focuses on lightweight dynamic
obstacle avoidance for visually impaired navigation: a global static planner
provides waypoints, while a local MPPI controller follows those waypoints and
reacts to pedestrians through a Dynamic Risk Field (DRF). A Stage 2
spatiotemporal yield supervisor can additionally stop the cane before it rushes
through a pedestrian-priority crossing conflict.

This README describes the current project state. Older prototype notes and
paper-era descriptions are kept separately under `docs/archive/` when available.

![Smart cane system overview](figure/current_system_overview.png)

Figure: current smart-cane software and data-flow overview, including sensing,
mapping, localization, dynamic obstacle tracking, navigation planning, human
interaction, and low-level guidance.

README figures:

```text
figure/current_system_overview.png   Full software and data-flow overview
figure/hardware_current.png          Current smart-cane hardware platform
figure/runtime_rviz.png              RViz view of the MPC simulation
figure/stage2_crossing.gif           Crossing scenario animation
figure/mixed_scenario.gif            Mixed pedestrian scenario animation
figure/fast_lio_relocalization.png   FAST-LIO / localization map example
```

## Current Method

The current local navigation stack is:

```text
global A* path
    -> waypoint/sub-goal selection
    -> DRF-MPPI local planner
    -> optional spatiotemporal yield supervisor
    -> L1 / steering control output
```

Key design assumptions:

- The platform is a smart cane, not a fully actuated mobile robot.
- The human supplies forward walking motion; the system mainly shapes the local
  heading and walking path.
- The local planner uses MPPI rollouts over an LFPC/LIPM-style walking model.
- Pedestrian influence is modeled with a Dynamic Risk Field soft cost in MPPI.
- Stage 2 does not redirect the MPC target or add social interaction cost inside
  `MpcController`; it supervises crossing conflicts in `PlannerManager` and
  publishes stop advice when needed.

The current Stage 2 mode set is intentionally small:

```text
scene in {none, crossing}
mode  in {CONTINUE, YIELD}
```

It does not currently implement active `PASS_BEHIND`, `PASS_AHEAD`, `PASS_SIDE`,
or interaction-target redirection.

![Stage 2 crossing yield behavior](figure/stage2_crossing.gif)

Figure: crossing-pedestrian simulation used to inspect DRF-MPPI behavior and
Stage 2 yield timing.

## Repository Layout

```text
cane_planner/
  plan_manage/       PlannerManager FSM, launch files, experiment scripts
  path_searching/    A*, KinodynamicAstar, MPPI MPC, LFPC, DynamicRiskField
  plan_env/          SDF map, collision checking, object prediction interface
  plan_grid_env/     Alternative grid environment representation
  bspline/           Non-uniform B-spline trajectory representation
  bspline_opt/       NLopt-based trajectory smoothing and feasibility costs
  plan_ctrl/         L1 look-ahead controller and serial motor interface
  plan_ctrl2/        ros_control hardware interface for GKF platform
  omniGKF_ctrl/      Omnidirectional wheel driver and custom messages
  Utils/             Map generator, waypoint generator, IMU and motor utilities
  LFPC/              Python LFPC/LIPM reference prototype, not catkin-built
```

In the full workspace, `cane_planner` is typically used together with:

- `FAST-LIO-SAM-QN` or `FAST-LIO-Localization-QN` for LiDAR-inertial odometry,
  localization, and registered point clouds.
- `LV-DOT/onboard_detector` for dynamic obstacle detection and tracking.

The hardware data flow is:

```text
FAST-LIO / localization odometry + registered cloud
        -> plan_env SDF/collision layer
LV-DOT dynamic obstacles
        -> PlannerManager dynamic obstacle callback
        -> path_searching MPPI / KinodynamicAstar
        -> plan_ctrl / steering interface
```

The lightweight simulation replaces real SLAM and tracking with a simulation
generator and `pedestrian_sim.py`.

## Hardware Platform

The target platform is a smart cane with onboard sensing, computation, power,
and a steering or wheel module for guidance. The exact hardware configuration
can vary by experiment, but the planning stack expects odometry or localization,
map/cloud input for the SDF collision layer, and dynamic obstacle observations
when pedestrian-aware planning is evaluated.

![Current smart-cane hardware platform](figure/hardware_current.png)

Figure: current smart-cane hardware platform, including the tactile guidance
mechanism, Livox Mid-360 LiDAR, RealSense D455 depth camera, NVIDIA Orin NX
compute unit, battery, IMU, low-level controller, motor, and omnidirectional
wheel.

## Build

Build from the workspace root, not from this package directory:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash

# Required by FAST-LIO-SAM-QN in the full workspace.
catkin build nano_gicp -DCMAKE_BUILD_TYPE=Release
catkin build quatro -DCMAKE_BUILD_TYPE=Release -DQUATRO_TBB=ON -DQUATRO_DEBUG=OFF

# Build the full workspace.
catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

This workspace uses `catkin_tools` (`catkin build`), not `catkin_make`.

For planner-only development, a narrower build is often enough:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
catkin build plan_manage path_searching -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Quick Start: Lightweight Simulation

Run the MPC planner with simulated pedestrians:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch plan_manage sim_kin_replan.launch planner:=3
```

Planner selection:

```text
planner:=1  A*
planner:=2  KinodynamicAstar
planner:=3  MPPI MPC
```

Use RViz `2D Nav Goal` to send a goal, or publish one of the fixed lightweight
scenarios:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun plan_manage publish_lightweight_fixed_goal.py _scenario:=crossing _goal_publish_duration:=0.0
```

The `_goal_publish_duration:=0.0` setting publishes the goal once. This is
important for timing-sensitive pedestrian-triggered experiments because repeated
goal messages can retrigger `pedestrian_sim.py`.

![RViz MPC runtime view](figure/runtime_rviz.png)

Figure: RViz view of the MPC runtime, including the local map, dynamic obstacle
markers, current waypoint, and planned trajectory.

## Stage 2 Crossing Experiment

The lightweight experiment workflow is:

```text
launch simulation
    -> publish fixed start/goal
    -> record rosbag topics
    -> analyze bag metrics
    -> compare B0 / C / E0b / E1
```

The current Stage 2 comparison groups are:

```text
B0   pedestrians present, no dynamic intervention
C    DRF-only MPPI
E0b  Stage 2 debug topics enabled, yield disabled
E1   DRF-MPPI + spatiotemporal yield supervisor
```

![Mixed dynamic-obstacle scenario](figure/mixed_scenario.gif)

Figure: mixed lightweight simulation with multiple pedestrian interactions. This
scenario is useful for checking that dynamic-obstacle handling does not only
work in the single crossing case.

Current E1 launch command:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch plan_manage sim_kin_replan.launch \
  planner:=3 \
  use_pedestrians:=true \
  pedestrian_scenario:=crossing \
  mpc_w_risk:=2.0 \
  mpc_risk_sigma_y:=0.38 \
  mpc_enable_cpa:=false \
  mpc_enable_stop_advice:=true \
  mpc_enable_stop_enforce:=true \
  mpc_enable_interaction:=true \
  mpc_interaction_enable_yield:=true \
  mpc_interaction_st_horizon:=4.0 \
  mpc_interaction_yield_trigger_time:=2.5 \
  mpc_interaction_robot_radius:=0.25 \
  mpc_interaction_yield_safety_margin:=0.20
```

Record the main analysis topics:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
mkdir -p /home/xcg/ws/records/stage2_current_E1
rosbag record --duration=45 \
  -O /home/xcg/ws/records/stage2_current_E1/E1_run01.bag \
  /clock \
  /sim_odom \
  /simulation_generator/odom \
  /onboard_detector/dynamic_obstacles_info \
  /mpc/interaction_scene \
  /mpc/interaction_mode \
  /mpc/interaction_debug \
  /mpc/debug_metrics \
  /mpc/stop_advice \
  /mpc/stop_reason \
  /mpc/path \
  /mpc/best_traj \
  /mpc/current_waypoint \
  /mpc/waypoints \
  /cmd_vel_footprint
```

Analyze one bag:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun plan_manage analyze_mpc_eval.py \
  /home/xcg/ws/records/stage2_current_E1/E1_run01.bag \
  --odom-topic /sim_odom \
  --robot-radius 0.25
```

## Runtime Topics

Important MPC and Stage 2 topics:

```text
/mpc/path                    nav_msgs/Path
/mpc/best_traj               visualization_msgs/Marker
/mpc/current_waypoint        visualization_msgs/Marker
/mpc/waypoints               visualization_msgs/Marker
/mpc/debug_metrics           std_msgs/Float64MultiArray
/mpc/risk_field              sensor_msgs/PointCloud2
/mpc/risk_halo               sensor_msgs/PointCloud2
/mpc/interaction_scene       std_msgs/String
/mpc/interaction_mode        std_msgs/String
/mpc/interaction_debug       std_msgs/Float64MultiArray
/mpc/stop_advice             std_msgs/Bool
/mpc/stop_reason             std_msgs/String
/cmd_vel_footprint           geometry_msgs/Twist, Gazebo mode
```

`/mpc/interaction_debug` currently uses a compact 35-field layout documented in
`MPC_STAGE2_IMPLEMENTATION_PLAN.md`.

## Configuration Files

Core launch and configuration files:

```text
plan_manage/launch/sim_kin_replan.launch
plan_manage/launch/kin_replan.launch
plan_manage/launch/dynamic_detect.launch
plan_manage/launch/include/algorithm.launch
plan_manage/config/lightweight_fixed_goals.yaml
plan_manage/config/pedestrian_scenarios.yaml
```

Current MPC and Stage 2 implementation files:

```text
plan_manage/include/plan_manager.h
plan_manage/src/planner_manager.cpp
path_searching/include/path_searching/mpc_controller.h
path_searching/src/mpc_controller.cpp
path_searching/include/path_searching/dynamic_risk_field.h
path_searching/src/dynamic_risk_field.cpp
```

Useful experiment scripts:

```text
plan_manage/scripts/pedestrian_sim.py
plan_manage/scripts/publish_lightweight_fixed_goal.py
plan_manage/scripts/record_lightweight_mpc_eval.sh
plan_manage/scripts/analyze_mpc_eval.py
plan_manage/scripts/analyze_stage2_batch.py
plan_manage/scripts/plot_mpc_eval_trajectories.py
```

## Real Hardware Run

For hardware runs, `kin_replan.launch` expects external odometry, map/cloud
input, and dynamic obstacle tracking:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch plan_manage kin_replan.launch
```

Typical companion processes in the full workspace:

```bash
# SLAM / localization
roslaunch fast_lio_sam_qn run.launch lidar:=ouster

# Dynamic obstacle detection and tracking
roslaunch onboard_detector run_detector.launch
```

![FAST-LIO localization example](figure/fast_lio_relocalization.png)

Figure: example localization or relocalization map output used by the real-world
navigation stack.

Exact topics depend on the sensor setup and launch remaps. Confirm that the
planner receives odometry, map/cloud input, and
`/onboard_detector/dynamic_obstacles_info` before evaluating dynamic behavior.

## Project Notes

- DRF-only MPPI is a soft pedestrian-risk method. It is not a hard safety
  guarantee and should not be described as active social navigation.
- CPA/TTC variants and larger MPPI sample counts are treated as internal design
  evidence, not the current main contribution.
- The Stage 2 claim should stay scoped: the yield supervisor prevents rushing
  into detected crossing conflicts and then allows the original DRF-MPPI planner
  to continue after the conflict clears.
- Do not over-interpret one visually good run. Use repeated bags and
  distribution-level metrics for experimental claims.

## Related Documents

```text
MPC_LIGHTWEIGHT_ABLATION_NOTES.md
MPC_STAGE1_EXPERIMENT_TABLES.md
MPC_STAGE2_IMPLEMENTATION_PLAN.md
STAGE2_EXPERIMENT_COMMANDS.md
RAL_DIRECTION_NOTES.md
docs/archive/
```
