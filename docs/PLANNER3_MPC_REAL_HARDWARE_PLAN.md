# planner == 3 real-hardware integration plan

## Current judgment

`planner == 3` already has the main MPC planning capability in software. It can use:

- A* global path guidance;
- SDF/ESDF static collision checking;
- LV-DOT dynamic obstacle position, velocity, and size;
- MPPI-MPC rollout with the LFPC model;
- dynamic risk field and stop-advice logic.

However, it should not yet be treated as a complete real-hardware closed-loop controller. The missing part is not the MPC planner itself, but the interface from MPC output to the real L1 serial motor controller.

The target real-hardware control route should be:

```text
MPC local path
-> L1 lookahead point selection
-> eta angle computation
-> smoothing / rate limiting / deadband / stop handling
-> serial motor command
```

This keeps MPC responsible for obstacle-aware local planning, while the lower controller converts the planned local path into a smooth and comfortable steering command for the guide-cane hardware.

## Active document layout

The detailed adaptive lookahead and comfort-control idea is kept in:

```text
docs/MPC_L1_ADAPTIVE_LOOKAHEAD_PLAN.md
```

This file focuses on the minimum real-hardware integration work needed to make `planner == 3` usable on the cane.

## Minimum changes

### 1. Let `kin_replan.launch` support `planner := 3`

The current real-hardware launch still defaults to:

```xml
<arg name="planner" value="2"/>
```

This means the real launch still enters `KinodynamicAstar` by default.

Recommended change:

```xml
<arg name="planner" default="3"/>
```

or keep a launch argument so the mode can be selected at runtime.

LV-DOT should also be launched for real dynamic-obstacle input. Without LV-DOT, the MPC dynamic obstacle list will be empty and only static-map avoidance will be active.

### 2. Split MPC control and visualization topics

Do not use one accumulated `/mpc/path` topic for both control and RViz inspection.

Recommended topic split:

```text
/mpc/local_path    nav_msgs/Path                 short rolling local path for L1 tracking
/mpc/history_path  nav_msgs/Path                 accumulated MPC path for RViz
/mpc/best_traj     visualization_msgs/Marker     current best predicted rollout for RViz
```

The control path should be short, clean, and forward-looking. It should be generated from the current MPC best path or current local waypoint window.

The history path should preserve the full route for visualization and debugging.

### 3. Add `planner == 3` support in `L1_controller_v2`

The current L1 controller only handles:

```cpp
planner == 1 -> /astar/path
planner == 2 -> /planning_vis/trajectory
```

It should add:

```cpp
planner == 3 -> /mpc/local_path
```

The existing `eta` calculation can remain. The change is mainly the source and quality of the path.

### 4. Use adaptive lookahead for the MPC local path

The current front-point selection only finds a point in front of the cane and farther than a mostly fixed `Lfw`.

For real hardware, `Lfw` should become adaptive:

```text
Lfw_dynamic = f(user_speed, path_curvature, obstacle_risk, lateral_error, steering_stability)
```

Qualitative rule:

- higher speed: longer lookahead;
- sharper local path curvature: shorter lookahead;
- closer dynamic obstacle or higher risk: shorter lookahead;
- larger lateral tracking error: shorter lookahead;
- frequent steering oscillation: slightly longer lookahead.

The first engineering version can be rule-based. Later versions can upgrade to fuzzy control or a small optimization problem.

### 5. Add a comfort and safety layer before serial output

Do not send raw `eta` or raw MPC steering output directly to the motor.

The final serial command should pass through:

```text
angle saturation
-> angle-rate limiting
-> low-pass filtering
-> deadband
-> stop override
```

This is important for a guide-cane system. The steering command should be smooth and predictable, not just collision-avoiding in simulation.

### 6. Subscribe to `/mpc/stop_advice`

MPC already publishes:

```text
/mpc/stop_advice
/mpc/stop_reason
```

The real L1 controller should subscribe to `/mpc/stop_advice`.

When stop advice is true:

- send a zero/stop command through serial;
- freeze or clear the current path tracking command;
- resume only after stop advice is false and a fresh local path is available.

This prevents the motor controller from following an old path while MPC is asking the system to yield or stop.

### 7. Use real odometry as the only hardware pose source

In the current MPC stepping logic, the non-Gazebo branch still contains simulation-style pose advancement:

```cpp
if (!gazebo_sim_) {
    odom_pos_ = new_com;
}
publishSimOdom();
```

This is not suitable for real hardware.

For real hardware:

- FAST-LIO `/Odometry` should be the source of truth;
- MPC should re-anchor from real odometry every cycle;
- LFPC/MPC internal state should not overwrite `odom_pos_`;
- simulated odometry publishing should be disabled or kept clearly separate from real hardware mode.

### 8. Check TF and frame consistency

The real chain involves at least:

```text
world / camera_init / body / cane_base
```

The following must be consistent:

- `/Odometry` can be transformed into `world`;
- `/cloud_registered` and SDF/ESDF map use the same world frame;
- LV-DOT dynamic obstacle `position`, `velocity`, and `size` are in the frame expected by MPC;
- `cane_base` yaw matches the physical steering direction;
- L1 positive/negative steering convention matches the motor controller.

Frame mismatch can make RViz look reasonable while the hardware turns in the wrong direction.

### 9. Tune real-hardware parameters only after the interface works

Do not tune MPC parameters before the control interface is correct.

After the closed loop runs, tune:

```text
mpc/num_samples
mpc/horizon_steps
manager/lookahead_dist
mpc/nominal_al
mpc/risk_sigma_y
mpc/w_risk
mpc/dynamic_hard_reject_enable
mpc/stop_advice_enable
mpc/stop_advice_enforce
L1 adaptive lookahead limits
angle-rate limit
angle deadband
low-pass filter coefficient
```

The first hardware test should use a low-speed, open-space scene before adding dynamic pedestrians.

## Recommended implementation order

1. Move MPC control output to `/mpc/local_path` and keep visualization topics separate.
2. Add `planner == 3` path subscription in `L1_controller_v2`.
3. Add stop-advice subscription and serial stop override.
4. Add angle saturation, rate limiting, low-pass filtering, and deadband.
5. Add adaptive lookahead using a simple rule-based `Lfw_dynamic`.
6. Remove or gate simulation odometry advancement in real hardware mode.
7. Verify TF and LV-DOT obstacle frame consistency.
8. Test with static obstacles.
9. Test with LV-DOT dynamic obstacles.
10. Tune MPC and L1 comfort parameters.

## Longer-term upgrade

After the rule-based version works, the adaptive lookahead module can be upgraded in two directions:

```text
rule-based adaptive lookahead
-> fuzzy-control lookahead adjustment
-> optimization-based lookahead selection
```

Fuzzy control is attractive for this project because it remains interpretable and can express human-comfort rules such as "slow speed + narrow corridor + high risk -> short lookahead".

Optimization-based lookahead can be considered later if enough real walking data is available.

## Summary

The most suitable real-hardware route is not raw MPC angle output. It is:

```text
MPC rolling local path
-> adaptive L1 lookahead
-> eta
-> comfort and safety filter
-> serial motor command
```

This keeps the MPC dynamic-obstacle avoidance ability while making the final motor command more stable for real guide-cane use.
