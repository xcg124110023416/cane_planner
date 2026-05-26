# Stage 2 Design: Intent-aware DRF-MPPI

## Goal

Build the E group for Stage 2:

```text
E = DRF-only MPPI + lightweight interaction intent layer
```

The intent layer should make crossing interaction behavior explicit without replacing MPPI. It converts pedestrian-path conflict semantics into a local target bias or yield advice.

## Scope

This stage only targets single-pedestrian crossing interactions in the lightweight simulation. It does not attempt crowd navigation, learned human intent prediction, or broad scenario matrices.

## Method Overview

```text
path-aligned local frame
    -> crossing candidate detection
    -> timing analysis
    -> behind-target feasibility
    -> hysteresis
    -> mode-conditioned MPPI target
```

The selected mode is held through hysteresis and converted into MPPI input:

- `CONTINUE`: use the original local waypoint.
- `PASS_BEHIND`: replace the local waypoint with a feasible behind-pedestrian local target.
- `YIELD`: publish stop/yield advice; in real cane use this means user guidance, not physical braking.

MPPI still performs rollout optimization and retains the DRF soft risk cost.

## Local Frame

Use the current path direction, not only robot yaw, to define the local frame:

```text
path_forward = normalized(current_local_waypoint - robot_position)
path_left = rotate90(path_forward)
```

For each pedestrian:

```text
rel = pedestrian_position - robot_position
front = rel dot path_forward
lateral = rel dot path_left
v_front = pedestrian_velocity dot path_forward
v_lateral = pedestrian_velocity dot path_left
```

## Crossing Candidate

A pedestrian is a crossing candidate when:

- `front` is inside a forward range;
- `abs(lateral)` is near the path corridor;
- `abs(v_lateral)` is above a crossing-speed threshold;
- the pedestrian is approaching or traversing the path corridor.

Timing:

```text
t_ped_to_path = abs(lateral) / max(abs(v_lateral), eps)
t_robot_to_cross = front / max(robot_speed, min_robot_speed)
time_gap = t_robot_to_cross - t_ped_to_path
```

## Mode Selection

For the selected crossing candidate:

- No candidate: `CONTINUE`.
- Pedestrian is predicted to clear the path first and behind target is feasible: `PASS_BEHIND`.
- Nearly simultaneous arrival, conflict too close, or behind target infeasible: `YIELD`.
- Robot clearly arrives first and predicted clearance is safe: `CONTINUE`, without explicitly declaring `PASS_AHEAD`.

## Behind Target

Let:

```text
ped_dir = normalized(pedestrian_velocity)
predicted_ped = pedestrian_position + pedestrian_velocity * t_ped_to_path
raw_behind = predicted_ped - ped_dir * behind_dist
intent_target = raw_behind + path_forward * forward_bias
```

Minimum feasibility checks:

- target is in front of the robot;
- target is inside local/FOV range;
- target is not in static collision;
- target keeps sufficient distance from the predicted pedestrian position;
- target deviation from the global path is below a configured maximum;
- target does not force the robot to chase backward.

If the target fails, do not force `PASS_BEHIND`.

## Hysteresis

Use mode hold and clear times:

- entering `PASS_BEHIND` holds for at least `pass_behind_hold_time`;
- entering `YIELD` holds until the conflict has been clear for `yield_clear_time`;
- `CONTINUE` is restored only after the active mode's clear condition is stable.

This prevents frame-to-frame mode switching.

## Development Variants

- `E0`: intent wrapper only; should match C behavior.
- `E1`: `PASS_BEHIND` local target + feasibility + hysteresis. This is the Stage 2 core.
- `E2`: add `YIELD` advice after E1 works, with stop-duration constraints.

## Required Metrics

Record and analyze:

- intent state counts;
- intent transition count;
- `PASS_BEHIND` dwell time;
- `YIELD` dwell time;
- intent target valid ratio;
- closest robot-pedestrian clearance;
- pass mode;
- path efficiency;
- global path deviation;
- stop duration;
- MPPI plan time.

## Experiment Groups

Use the crossing timing sweep:

- `early`;
- `simultaneous`;
- `late`.

Compare:

- `B0`: no DRF, no dynamic intervention;
- `C`: DRF-only MPPI;
- `E`: Intent-aware DRF-MPPI.

Each timing should be repeated at least 3 times during development and preferably 5 times before paper tables.
