# MPC-L1 adaptive lookahead plan

## Background

The current real-hardware control route should not send raw MPC steering output directly to the motor. For a guide-cane system, the control signal must be smooth, predictable, and comfortable for the user. A better real-hardware route is:

```text
MPC local path
-> L1 lookahead point selection
-> eta angle computation
-> smoothing / rate limiting / stop handling
-> serial motor command
```

In this structure, MPC is responsible for local obstacle-aware planning, while the lower controller converts the planned local path into a stable steering command.

## Current limitation

The current L1 controller selects a forward point mostly by checking whether a path point is in front of the cane and farther than a fixed `Lfw`.

This is simple and usable, but it does not adapt to:

- user walking speed;
- path curvature;
- local dynamic-obstacle risk;
- lateral tracking error;
- steering command oscillation;
- different indoor corridor widths or clutter levels.

As a result, a fixed lookahead distance may be too short in straight, open space, causing unnecessary steering sensitivity, or too long in narrow/dynamic scenes, causing the controller to miss local avoidance details.

## Proposed baseline scheme

Keep the current `eta` logic, but replace the fixed lookahead point selection with an adaptive lookahead distance:

```text
Lfw_dynamic = f(user_speed, path_curvature, obstacle_risk, lateral_error, steering_stability)
```

The controller still computes:

```text
eta = atan2(local_y, local_x)
```

The difference is that the selected target point becomes smarter.

Recommended qualitative rules:

- Higher user speed: increase lookahead distance.
- Higher path curvature: decrease lookahead distance.
- Higher dynamic-obstacle risk: decrease lookahead distance.
- Larger lateral path error: decrease lookahead distance.
- Frequent steering oscillation: increase lookahead distance slightly.
- Open straight corridor: allow a longer lookahead distance.
- Narrow or cluttered region: use a shorter lookahead distance.

A conservative first range for guide-cane hardware can be:

```text
Lfw_min  = 0.35 m
Lfw_base = 0.50 m
Lfw_max  = 1.20 m
```

These values should be treated as initial engineering guesses, not final tuned parameters.

## Integration with MPC

MPC should publish a short rolling local path, for example:

```text
/mpc/local_path
```

This path should be used by the L1 controller for real-time tracking. It should not contain the full accumulated history.

For visualization, separate topics should be kept:

```text
/mpc/best_traj      current best predicted rollout
/mpc/history_path   accumulated MPC path for RViz inspection
/mpc/local_path     short control path for L1 tracking
```

This separation keeps the control input clean while still allowing the full MPC behavior to be inspected in RViz.

## Safety and comfort layer

The computed `eta` should not be sent directly to the serial motor command. It should pass through a comfort layer:

```text
eta
-> angle saturation
-> angle-rate limiting
-> low-pass filtering
-> deadband
-> stop override
-> serial command
```

The goal is not only collision avoidance, but also comfortable guidance. The cane should not twitch or rapidly alternate steering directions when the MPC risk field or perception result changes slightly.

Useful rules:

- Limit the maximum steering angle.
- Limit the maximum angle change per control cycle.
- Ignore very small angle changes with a deadband.
- Apply low-pass filtering before sending commands.
- If `/mpc/stop_advice` is true, override tracking and send a stop/zero command.

## Future upgrade directions

### 1. Rule-based adaptive lookahead

This should be the first version. It is easy to debug and explain.

Example idea:

```text
Lfw_dynamic =
  Lfw_base
  + speed_gain * user_speed
  - curvature_gain * path_curvature
  - risk_gain * obstacle_risk
  - error_gain * lateral_error
```

Then clamp it into `[Lfw_min, Lfw_max]`.

### 2. Fuzzy-control lookahead adjustment

After real data is available, the rule-based version can be upgraded into a fuzzy controller.

Inputs can include:

- walking speed: low / medium / high;
- path curvature: small / medium / large;
- obstacle risk: safe / cautious / dangerous;
- tracking error: small / medium / large.

Output:

- lookahead distance: short / normal / long.

This is attractive because it is interpretable and suitable for a human-assistive device.

### 3. Optimization-based lookahead selection

A later version can formulate lookahead selection as a small optimization problem.

Possible objective:

```text
minimize:
  tracking_error
  + steering_smoothness_cost
  + obstacle_risk_cost
  + user_comfort_cost
```

Decision variable:

```text
Lfw_dynamic
```

This could be solved at a low rate or combined with MPC debug metrics. It is more complex than fuzzy control, so it should be considered only after the rule-based version is validated.

### 4. Learning-assisted tuning

If enough real walking data is collected, the gains or fuzzy rules can be tuned from data. The model should remain conservative and interpretable because the system is assistive and safety-critical.

## Recommended development path

1. Keep the current `eta` calculation.
2. Add `/mpc/local_path` as the control path.
3. Add adaptive `Lfw_dynamic` in the L1 controller.
4. Add smoothing, rate limiting, deadband, and stop override before serial output.
5. Validate first in static scenes, then with LV-DOT dynamic obstacles.
6. Record user-speed, curvature, risk, `Lfw_dynamic`, `eta_raw`, and `eta_cmd`.
7. Upgrade to fuzzy control only after enough real or replayed data is available.

## Summary

The proposed direction is to keep the current L1 geometric control idea, but make the lookahead point adaptive and add a safety-comfort layer before serial output. This preserves the practical simplicity of the existing controller while making it more suitable for a real guide-cane system with MPC-based dynamic obstacle avoidance.
