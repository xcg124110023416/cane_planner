# Cane Planner

`cane_planner` is the navigation, planning, and control package set for a ROS1
smart-cane navigation system. The current branch focuses on lightweight dynamic
obstacle avoidance for visually impaired navigation: a global static planner
provides waypoints, while a local MPPI controller follows those waypoints and
reacts to pedestrians through a Dynamic Risk Field (DRF). A
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

Optional figures to add later:

```text
figure/gazebo_relocalization_runtime.png   Gazebo + FAST-LIO localization + MPC in RViz
figure/gazebo_world_pedestrians.png        Gazebo corridor world with walking pedestrians
figure/localization_topic_flow.png         Gazebo, FAST-LIO, localization, and planner topic flow
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
- The spatiotemporal yield supervisor does not redirect the MPC target or add
  social interaction cost inside `MpcController`; it supervises crossing
  conflicts in `PlannerManager` and publishes stop advice when needed.

The current interaction mode set is intentionally small:

```text
scene in {none, crossing}
mode  in {CONTINUE, YIELD}
```

It does not currently implement active `PASS_BEHIND`, `PASS_AHEAD`, `PASS_SIDE`,
or interaction-target redirection.

![Crossing yield behavior](figure/stage2_crossing.gif)

Figure: crossing-pedestrian simulation demonstrating DRF-MPPI behavior and
yield timing.

## Repository Layout

```text
cane_planner/
  plan_manage/       PlannerManager FSM, launch files, simulation utilities
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
can vary by platform revision, but the planning stack expects odometry or
localization, map/cloud input for the SDF collision layer, and dynamic obstacle
observations when pedestrian-aware planning is enabled.

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
important for pedestrian-triggered simulations because repeated goal messages
can retrigger `pedestrian_sim.py`.

![RViz MPC runtime view](figure/runtime_rviz.png)

Figure: RViz view of the MPC runtime, including the local map, dynamic obstacle
markers, current waypoint, and planned trajectory.

## Dynamic Pedestrian Handling

![Mixed dynamic-obstacle scenario](figure/mixed_scenario.gif)

Figure: mixed lightweight simulation with multiple pedestrian interactions.

The current pedestrian-aware planner combines:

```text
static global path
    -> local MPPI tracking
    -> Dynamic Risk Field cost around moving pedestrians
    -> crossing-conflict supervisor for stop/yield advice
```

The main behavior exposed by the current branch is the integrated navigation
behavior:

- Follow global waypoints with an MPPI local planner.
- Use dynamic obstacle observations to shape local rollouts away from pedestrians.
- Detect crossing conflicts in front of the cane.
- Publish stop/yield advice when the pedestrian conflict should take priority.
- Resume the original waypoint-following behavior after the conflict clears.

Run the lightweight pedestrian simulation:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch plan_manage sim_kin_replan.launch \
  planner:=3 \
  use_pedestrians:=true \
  pedestrian_scenario:=crossing
```

## Gazebo Simulation With Relocalization

The Gazebo branch runs the same planning stack against a simulated smart-cane
model, Gazebo LiDAR/IMU data, FAST-LIO odometry, map-based relocalization, and
Gazebo pedestrian ground truth.

The validated runtime data flow is:

```text
Gazebo cane model
    -> /velodyne_points + /imu
    -> FAST-LIO frontend
    -> /Odometry + /cloud_registered
    -> FAST-LIO-Localization-QN
    -> /localization_odom + /corrected_current_pcd
    -> cane_planner MPC stack
    -> /cmd_vel_footprint
```

Start the Gazebo/planner side:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch plan_manage gazebo_localization_mpc.launch \
  gui:=true \
  rviz:=true \
  start_teleop:=false
```

Start the FAST-LIO and relocalization side in another terminal:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch fast_lio_localization_qn sim_corridor_175_fastlio.launch rviz:=false
```

Use RViz `2D Nav Goal` to send a navigation target. The planner side expects the
localization package to publish:

```text
/localization_odom
/corrected_current_pcd
```

The localization side receives FAST-LIO output:

```text
/Odometry
/cloud_registered
```

## Runtime Topics

Important MPC and pedestrian-interaction topics:

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
plan_manage/launch/gazebo_localization_mpc.launch
plan_manage/launch/gazebo_mpc.launch
plan_manage/launch/kin_replan.launch
plan_manage/launch/dynamic_detect.launch
plan_manage/launch/include/algorithm.launch
plan_manage/config/lightweight_fixed_goals.yaml
plan_manage/config/pedestrian_scenarios.yaml
```

Current MPC and pedestrian-interaction implementation files:

```text
plan_manage/include/plan_manager.h
plan_manage/src/planner_manager.cpp
path_searching/include/path_searching/mpc_controller.h
path_searching/src/mpc_controller.cpp
path_searching/include/path_searching/dynamic_risk_field.h
path_searching/src/dynamic_risk_field.cpp
```

Useful development utilities:

```text
plan_manage/scripts/pedestrian_sim.py
plan_manage/scripts/publish_lightweight_fixed_goal.py
plan_manage/scripts/gazebo_pedestrian_truth_bridge.py
plan_manage/scripts/registered_map_accumulator.py
plan_manage/scripts/teleop_cmd.py
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

## Scope Notes

- DRF-only MPPI is a soft pedestrian-risk method. It is not a hard safety
  guarantee and should not be described as active social navigation.
- The yield supervisor should stay scoped: it prevents rushing
  into detected crossing conflicts and then allows the original DRF-MPPI planner
  to continue after the conflict clears.

## Related Documents

```text
RAL_DIRECTION_NOTES.md
MPC_STAGE2_IMPLEMENTATION_PLAN.md
docs/archive/
```
