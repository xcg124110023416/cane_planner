# Stage 2 Implementation Plan: Scene-conditioned Social-cost DRF-MPPI

> **For agentic workers:** Implement task-by-task with review checkpoints. Do not commit automatically; this project prefers manual commit decisions after validation.

## Goal

Build the E group:

```text
E = DRF-MPPI + lightweight interaction decision layer + scene/mode-conditioned social cost
```

The interaction layer must not directly command the cane to turn or stop. It should identify the interaction scene, select and maintain an interaction mode, then modulate MPPI inputs through scene/mode-conditioned social-cost parameters, optional local target bias, and optional yield prompt.

Current minimal mode set:

```text
mode ∈ {CONTINUE, PASS_BEHIND, YIELD}
```

Conceptual scene set:

```text
scene ∈ {none, static, oncoming, crossing}
```

Current implementation and experiments focus on `crossing`; `static` and `oncoming` are kept as objective-mapping extensions, not Stage 2 main experiments.

Do not add `PASS_AHEAD` or `PASS_SIDE` in the current Stage 2 implementation.

## Current Progress Checklist

Snapshot date: 2026-05-28.

Completed and build-verified:

- [x] Gap 0: cleaned Stage 2 baseline and kept `MpcController::plan(lfpc_model_, mpc_sim_goal_, obs_pos, obs_vel, obs_size)` unchanged.
- [x] Gap 1: added interaction scene/mode/debug topics and signed crossing timing debug.
- [x] Gap 2: implemented YIELD-first crossing FSM with `CONTINUE`, `YIELD`, and `PASS_BEHIND`.
- [x] Gap 3: interaction `YIELD` publishes `/mpc/stop_advice=true` with `INTERACTION_YIELD_CROSSING`; old yield reasons are isolated when interaction is enabled.
- [x] Gap 4 foundation: added `InteractionContext` and PASS_BEHIND-only social-cost hook without goal redirect.
- [x] Gap 4d: `crossing -> none` exits naturally while still allowing `ped_at_or_after_path` to be observed.
- [x] Gap 4e: `PASS_BEHIND` releases interaction stop and is held briefly enough for MPPI to receive the mode.

Gap 4 findings:

- [x] Gap 4f showed that `J_behind_corridor` behaves like attraction to a rear target and can cause excessive post-yield detour.
- [x] Gap 4h showed the large post-yield detour remains even with social cost and adaptive risk disabled; the issue was not primarily the new PASS_BEHIND social cost.
- [x] Gap 4i showed that resetting MPPI warm-start during enforced stop reduces one source of release-time pollution but does not by itself remove run-to-run instability.
- [x] Gap 4j delayed release with `interaction_pass_behind_clear_time`; repeated sim checks no longer showed the previous large post-yield rear detour.
- [x] Keep `crossing -> none -> CONTINUE` as a valid exit when the pedestrian clears naturally before `PASS_BEHIND` becomes ready.
- [x] Treat `J_behind_corridor` as optional/experimental until repeated-run ablation proves it helps beyond YIELD-first timing.
- [x] Do not present Gap 4j as solved `YIELD -> PASS_BEHIND` behavior. It validates the safer path `YIELD -> crossing clears -> CONTINUE`; the large-detour behavior of an active `PASS_BEHIND` mode remains unresolved.
- [ ] If `PASS_BEHIND` must be kept as a publishable Stage 2 mode, isolate it as a separate subproblem: force a case where crossing remains active after the pedestrian has clearly passed, then redesign the cost so it gates front-passing risk without acting like a strong rear-target attraction.
- [ ] Gap 4k diagnostic matrix:
  - `4k0`: force crossing to remain active (`interaction_cross_exit_threshold=0.0`) with social cost off. If this already produces a large detour, the issue is YIELD release + DRF/MPPI recovery rather than the new social cost.
  - `4k0` result: crossing remained active, but `PASS_BEHIND` did not trigger because the candidate became invalid before `signed_t_ped_to_path` reached the `0.7s` clear-time threshold. This is not yet a valid PASS_BEHIND isolation sample.
  - `4k0b`: keep social cost off, extend candidate latch, and temporarily reduce `interaction_pass_behind_clear_time` to force an active PASS_BEHIND sample without adding cost terms.
  - `4k0b` result: active `YIELD -> PASS_BEHIND -> CONTINUE` occurred with social cost off. The executed path showed a moderate rear detour (about 0.69 m max lateral deviation), not the previous 1.5 m class large detour. This suggests active PASS_BEHIND mode alone is not sufficient to explain the large rear arc.
  - `4k1`: same forced PASS_BEHIND case with `J_front_pass` only. This should discourage rushing in front without pulling to a rear target.
  - `4k1` result: active `YIELD -> PASS_BEHIND -> CONTINUE` occurred with `J_front_pass` only. Executed max lateral deviation was about 0.70 m, comparable to 4k0b, so `J_front_pass` did not appear to create the large rear arc.
  - `4k2`: same forced PASS_BEHIND case with `J_behind_corridor` enabled. If only this case grows a large rear arc, deprecate or replace this term.
  - `4k2` result: active PASS_BEHIND with `J_front_pass + J_behind_corridor` increased executed max lateral deviation to about 0.97 m. This is worse than 4k0b/4k1 but did not reproduce the earlier 1.5 m class detour at weight 1.0. Treat `J_behind_corridor` as a rear-target attraction that amplifies detour and keep it disabled by default.
  - Gap 4 closeout result: normal conservative config (`cross_exit_threshold=0.3`, social cost off, `pass_behind_clear_time=0.7`) produced `CONTINUE -> YIELD -> CONTINUE`, with `crossing -> none` before PASS_BEHIND readiness and about 0.55 m executed max lateral deviation.

Remaining Stage 2 tasks:

- [x] Finalize Gap 4 by freezing the conservative default: interaction YIELD-first enabled, social cost off by default, no goal redirect, and `interaction_pass_behind_clear_time` enabled.
- [x] Decide whether the paper/experiment claim needs active `PASS_BEHIND`. If not, proceed to Gap 5 with the honest claim "YIELD-first crossing interaction avoids rushing; PASS_BEHIND social cost remains experimental."
- [x] Add analyzer support for repeated-run ablation summaries (`analyze_stage2_batch.py`) with dwell times, transitions, clearance, deviation, efficiency, and stop/yield duration.
- [ ] Gap 5: repeated-run evaluation and ablation (`B0`, `C`, `E0`, `E1`) with dwell times, transitions, clearance, deviation, efficiency, and stop/yield duration.
  - `B0` 5-run result: dynamic intervention off produced near-straight trajectories with mean max deviation about 0.16 m and mean path efficiency about 1.00, but mean closest collision clearance dropped to about 0.32 m (minimum about 0.13 m). This is the risky/no-intervention lower bound and can include PASS_AHEAD/rushing behavior.
  - `C` initial 5-run result: DRF-only with interaction/yield disabled produced no interaction mode transitions, no enforced interaction stop, mean max deviation about 2.59 m, mean closest collision clearance about 1.11 m, and mean path efficiency about 0.79. This C batch is safe in clearance but inefficient/large-detour in the current crossing setup.
  - Initial `E0` run exposed a logic pollution bug: `mpc_interaction_enable=true` and `mpc_interaction_enable_yield=false` still published `INTERACTION_YIELD_CROSSING` stop advice through `debug.yield_required`. Fixed by requiring `mpc_interaction_enable_yield_` before interaction yield stop advice can activate. Re-record E0 as `E0b`.
  - `E0b` 5-run result: interaction scene/debug published without mode transitions or interaction stop advice. Mean max deviation about 2.89 m, mean closest collision clearance about 1.08 m, mean path efficiency about 0.76, and mean crossing dwell about 1.06 s. This is a valid debug-only control after the pollution fix.
  - `E1` 5-run result: conservative YIELD-first interaction produced stable `CONTINUE -> YIELD -> CONTINUE` in all runs, no PASS_BEHIND entries, mean max deviation about 0.63 m, mean closest collision clearance about 1.67 m, mean stop duration about 3.02 s, and mean path efficiency about 0.98. This supports the current Stage 2 claim: YIELD-first crossing interaction avoids rushing and reduces large detours, while PASS_BEHIND social cost remains experimental.
- [ ] Keep `scene` limited to `{none, crossing}` and mode limited to `{CONTINUE, YIELD, PASS_BEHIND}` until crossing is stable.

## Architecture

```text
pedestrian observation and prediction
    -> scene recognition
    -> risk assessment + mode scoring / raw mode selection
    -> temporal consistency / hysteresis
    -> joint (scene, mode) objective modulation
    -> MpcController::plan()
```

MPPI remains the optimizer. `PlannerManager` computes the active scene and mode, then passes joint scene/mode-conditioned information to `MpcController`.

## File Structure

Modify these files as needed:

- `/home/xcg/ws/src/cane_planner/plan_manage/include/plan_manager.h`
  - Add scene/mode enums, crossing candidate struct, counters, mode memory, parameters, debug publishers, and helper declarations.
- `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`
  - Load parameters, compute scene/crossing candidate, update mode memory, publish debug info, and pass joint scene/mode-conditioned goal/cost options to MPPI.
- `/home/xcg/ws/src/cane_planner/path_searching/include/path_searching/mpc_controller.h`
  - Add optional social-cost configuration only if target bias alone is insufficient.
- `/home/xcg/ws/src/cane_planner/path_searching/src/mpc_controller.cpp`
  - Add joint scene/mode-conditioned social-cost terms only after E1 target-bias version is verified.
- `/home/xcg/ws/src/cane_planner/plan_manage/launch/include/algorithm.launch`
  - Expose Stage 2 parameters.
- `/home/xcg/ws/src/cane_planner/plan_manage/launch/sim_kin_replan.launch`
  - Expose E group launch toggles.
- `/home/xcg/ws/src/cane_planner/plan_manage/scripts/analyze_mpc_eval.py`
  - Parse mode/debug topics and compute stability metrics.
- Optional: `/home/xcg/ws/src/cane_planner/plan_manage/scripts/record_lightweight_mpc_eval.sh`
  - Ensure Stage 2 topics are recorded.

---

## Task 0: Fix Timing Source Before Evaluating E1

**Reason:** Crossing timing is invalid if robot speed uses a default simulation constant instead of the actual LFPC/user walking speed.

**Files:**

- Modify `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/include/plan_manager.h` if additional helper state is needed.

- [ ] Ensure `t_robot_to_cross` uses the actual forward speed estimate consistent with the running LFPC/MPPI setup.

Preferred source:

```text
robot_speed ≈ user_step_length / t_sup
```

or a filtered odometry speed if that is more consistent with the simulation.

- [ ] Do not use `sim_speed_` unless it is confirmed to equal the active walking speed.
- [ ] Publish the speed used for timing in `/mpc/intent_debug` or equivalent debug output.
- [ ] Re-check early / simultaneous / late crossing logs after the fix.

Expected:

```text
t_robot_to_cross should match the observed crossing timing in RViz/bag analysis.
```

---

## Task 1: E0 Interaction Wrapper

**Goal:** Add the Stage 2 framework without changing C baseline behavior.

**Files:**

- Modify `/home/xcg/ws/src/cane_planner/plan_manage/include/plan_manager.h`
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/launch/include/algorithm.launch`
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/launch/sim_kin_replan.launch`

- [ ] Add launch parameters:

```text
mpc_enable_interaction:=false
mpc_interaction_enable_pass_behind:=false
mpc_interaction_enable_yield:=false
mpc_interaction_enable_social_cost:=false
mpc_interaction_enable_switch_penalty:=false
```

If existing names already use `mpc_enable_intent`, keep names consistent for minimal code churn, but document that the method is now an interaction decision layer rather than human intent prediction.

- [ ] Keep enum minimal:

```cpp
enum MpcInteractionMode {
  INTERACTION_CONTINUE = 0,
  INTERACTION_PASS_BEHIND = 1,
  INTERACTION_YIELD = 2
};
```

If existing code already has `MpcIntentState`, either keep it temporarily or rename only if the churn is small.

- [ ] Add publishers:

```text
/mpc/interaction_mode
/mpc/interaction_debug
/mpc/interaction_target
```

If existing topics are `/mpc/intent_state`, `/mpc/intent_debug`, `/mpc/intent_target`, keep them for now and update labels later to avoid breaking analysis scripts.

- [ ] E0 behavior:

```text
interaction disabled -> exact original C behavior
interaction enabled but pass/yield/social-cost disabled -> CONTINUE only, original local goal, original MPPI cost
```

- [ ] Build:

```shell
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && catkin build plan_manage -DCMAKE_BUILD_TYPE=Release
```

Expected:

```text
E0 publishes CONTINUE and remains qualitatively equivalent to C baseline.
```

---

## Task 2: Crossing Candidate Assessment

**Goal:** Detect a crossing interaction candidate in the path-aligned local frame.

**Files:**

- Modify `/home/xcg/ws/src/cane_planner/plan_manage/include/plan_manager.h`
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`

- [ ] Add parameters:

```text
mpc_interaction_front_min:=0.3
mpc_interaction_front_max:=4.0
mpc_interaction_corridor_width:=0.7
mpc_interaction_cross_speed:=0.15
mpc_interaction_time_gap:=0.8
mpc_interaction_min_robot_speed:=0.15
mpc_interaction_cpa_horizon:=3.0
mpc_interaction_cpa_dist:=0.8
mpc_interaction_use_cpa_check:=true
```

Existing `mpc_intent_*` parameter names can be reused if already implemented.

- [ ] Compute path-aligned frame:

```cpp
Eigen::Vector2d path_forward(goal_x - robot_x, goal_y - robot_y);
if (path_forward.norm() < 1e-3) {
  path_forward = Eigen::Vector2d(std::cos(theta), std::sin(theta));
} else {
  path_forward.normalize();
}
Eigen::Vector2d path_left(-path_forward.y(), path_forward.x());
```

- [ ] For each pedestrian, compute:

```text
front
lateral
v_front
v_lateral
t_ped_to_path
t_robot_to_cross
time_gap
t_cpa
d_cpa
```

CPA auxiliary check:

```text
r = p_h - p_u
v_rel = v_h - v_u
t_cpa = - (r dot v_rel) / ||v_rel||^2
d_cpa = ||r + v_rel * t_cpa||

cpa_conflict = 0 < t_cpa < cpa_horizon and d_cpa < cpa_dist
```

Use CPA as an auxiliary conflict signal, not as a replacement for DRF or crossing geometry.

- [ ] Select candidate:

```text
valid crossing candidate = forward + near path corridor + sufficient lateral velocity + predicted path/cpa conflict
selected candidate = valid candidate with smallest abs(time_gap) or smallest d_cpa
```

- [ ] Apply pedestrian-priority crossing-order policy:

```text
Do not select PASS_AHEAD.
If conflict is significant and behind space is not feasible yet -> YIELD if enabled.
If pedestrian has priority and behind space is feasible -> PASS_BEHIND.
If conflict is cleared or outside prediction window -> CONTINUE.
```

- [ ] Publish debug fields:

```text
front, lateral, v_lateral, t_ped_to_path, t_robot_to_cross, time_gap, t_cpa, d_cpa, cpa_conflict, robot_speed_used
```

Expected:

```text
Mode can remain CONTINUE, but debug should show sensible crossing timing values.
```

---

## Task 3: Risk-based Scene Confidence Accumulation

**Goal:** Use continuous risk scores plus dual-threshold hysteresis before activating crossing-related modes.

**Files:**

- Modify `/home/xcg/ws/src/cane_planner/plan_manage/include/plan_manager.h`
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`

- [ ] Add parameters:

```text
mpc_interaction_cross_enter_threshold:=0.7
mpc_interaction_cross_exit_threshold:=0.3
mpc_interaction_cross_confirm_frames:=3
mpc_interaction_cross_clear_frames:=3
```

- [ ] Compute minimal crossing risk scores:

```text
R_crossing
R_collision
R_front_pass
R_behind_free
```

For current Stage 2, only `R_crossing` is required for the scene FSM; the others can be published/debugged and used by mode selection.

- [ ] Add counters:

```text
crossing_confirm_count
crossing_clear_count
```

- [ ] Update scene FSM logic:

```text
if R_crossing > cross_enter_threshold:
    crossing_confirm_count += 1
    crossing_clear_count = 0
elif R_crossing < cross_exit_threshold:
    crossing_clear_count += 1
    crossing_confirm_count = 0
else:
    keep current counters/scene, or decay counters conservatively

enter crossing if crossing_confirm_count >= cross_confirm_frames
exit crossing if crossing_clear_count >= cross_clear_frames
```

- [ ] Publish risk scores and counters in debug output:

```text
R_crossing, R_collision, R_front_pass, R_behind_free,
crossing_confirm_count, crossing_clear_count
```

Expected:

```text
Scene should not flip between none/crossing near threshold boundaries.
```

---

## Task 4: PASS_BEHIND Target and Feasibility

**Goal:** Generate a behind-pedestrian local target or behind bias for crossing mode.

**Files:**

- Modify `/home/xcg/ws/src/cane_planner/plan_manage/include/plan_manager.h`
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`

- [ ] Add parameters:

```text
mpc_interaction_behind_dist:=0.8
mpc_interaction_forward_bias:=0.6
mpc_interaction_target_front_min:=0.4
mpc_interaction_target_front_max:=3.5
mpc_interaction_target_lateral_max:=1.8
mpc_interaction_max_path_deviation:=1.5
mpc_interaction_target_ped_clearance:=0.7
```

- [ ] Generate target:

```cpp
ped_dir = normalized(pedestrian_velocity)
predicted_ped = pedestrian_position + pedestrian_velocity * t_ped_to_path
raw_behind = predicted_ped - ped_dir * behind_dist
interaction_target = raw_behind + path_forward * forward_bias
```

- [ ] Feasibility checks:

```text
target_front > target_front_min
target_front < target_front_max
abs(target_lateral) < target_lateral_max
distance_to_predicted_ped > target_ped_clearance
distance_to_global_path < max_path_deviation
static collision query says target is free enough
```

- [ ] Do not force PASS_BEHIND if target is infeasible.
- [ ] Publish target marker and target validity.

Expected:

```text
Target marker appears only for feasible crossing conflicts.
```

---

## Task 5: Mode Selection and Temporal Consistency

**Goal:** Select and maintain `{CONTINUE, PASS_BEHIND, YIELD}` without pass-mode jitter.

**Files:**

- Modify `/home/xcg/ws/src/cane_planner/plan_manage/include/plan_manager.h`
- Modify `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`

- [ ] Add parameters:

```text
mpc_interaction_switch_margin:=0.8
mpc_interaction_switch_penalty:=1.0
mpc_interaction_min_debounce_time:=0.3
mpc_interaction_mode_confirm_frames:=2
mpc_interaction_mode_clear_frames:=2
mpc_interaction_pass_behind_clear_time:=0.5
mpc_interaction_yield_clear_time:=0.7
```

- [ ] Raw mode selection:

```text
No active crossing scene -> CONTINUE
Active crossing + R_collision high + R_behind_free low -> YIELD if enabled
Active crossing + pedestrian-priority order favors letting the pedestrian pass first + R_behind_free high -> PASS_BEHIND
Crossing risk below exit threshold or outside prediction window -> CONTINUE
Robot clearly arrives first with safe margin -> CONTINUE, but do not declare PASS_AHEAD
```

Current Stage 2 explicitly does not select `PASS_AHEAD` as a guidance mode.

- [ ] Add temporal consistency:

```text
Require crossing scene confirmation before entering crossing-related modes.
Require candidate mode conditions to hold for mode_confirm_frames before switching.
Evaluate whether the current mode remains feasible every frame.
Switch to a new mode only if it beats the current mode by switch_margin.
Switch immediately if the current mode becomes infeasible or unsafe.
Return to CONTINUE only after clear condition is stable for mode_clear_frames.
Use min_debounce_time only as a short single-frame flip guard, not as a fixed PASS_BEHIND execution duration.
```

- [ ] Optional soft switch penalty:

```text
J_switch = w_switch * I(candidate_mode != previous_mode)
```

Use this for mode scoring if the implementation has comparable costs. Otherwise implement the equivalent margin rule directly:

```text
switch if score(new_mode) + switch_margin < score(current_mode)
```

Do not make fixed-duration `PASS_BEHIND` holding the main stability mechanism; any debounce window should only suppress instantaneous single-frame flips.

Expected:

```text
Mode changes should be rare and explainable; pass-mode switch count should decrease relative to DRF-only.
```

---

## Task 6: E1 Apply Mode-conditioned Bias

**Goal:** Make PASS_BEHIND influence MPPI without turning the interaction layer into a direct controller.

**Files:**

- Modify `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`
- Optional later: modify `MpcController` files for explicit social-cost terms.

- [ ] Minimal E1 target-bias implementation:

```cpp
Eigen::Vector3d plan_goal = mpc_sim_goal_;
if (active_mode == INTERACTION_PASS_BEHIND && interaction_target_valid) {
  plan_goal << interaction_target.x(), interaction_target.y(), 0.0;
}
control = mpc_controller_->plan(lfpc_model_, plan_goal, obs_pos, obs_vel, obs_size);
```

- [ ] Keep original common costs enabled:

```text
J_goal
J_static_obstacle / ESDF proximity
J_pedestrian_collision / DRF safety risk
J_social_basic / personal-space risk
J_lateral_control
J_motion_smooth / control delta smoothness
```

These already correspond to existing MPPI terms such as goal tracking, static proximity, dynamic risk, steering magnitude, and dapi smoothing.

- [ ] Do not enable forced stop in E1.
- [ ] If target bias alone is not stable, add explicit mode-conditioned social-cost terms in `MpcController`.

Prioritize only the three critical terms first:

```text
J_front_pass_high:
  high cost for entering pedestrian front-passing zone;
  implements no-pass-ahead guidance.

J_behind_corridor:
  PASS_BEHIND-only cost that encourages the predicted trajectory to pass through the pedestrian's rear safe corridor.

J_guidance_smooth:
  penalize abrupt changes or sign flips in lateral guidance/steering command.
```

Mode interpretation:

```text
CONTINUE:
  use baseline goal tracking + static obstacle + DRF/personal-space + smoothness costs

YIELD:
  increase front-passing zone and conflict-zone cost
  reduce aggressive lateral guidance through control/guidance smoothness
  publish wait/yield prompt; do not physically brake by default

PASS_BEHIND:
  increase front-passing zone cost
  add behind-corridor / behind-bias cost
  retain lateral-control and guidance-smoothness penalties
```

Do not implement `J_wait_or_slow` as direct forward-speed optimization unless the system has a reliable user-speed control interface. In this project it should mean prompt output and stable/low-aggression guidance.

Expected:

```text
E1 should bias MPPI toward a stable pass-behind solution without direct rule control.
```

---

## Task 7: Analyzer Support

**Files:**

- Modify `/home/xcg/ws/src/cane_planner/plan_manage/scripts/analyze_mpc_eval.py`
- Optional modify `/home/xcg/ws/src/cane_planner/plan_manage/scripts/record_lightweight_mpc_eval.sh`

- [ ] Record topics:

```text
/mpc/interaction_mode or /mpc/intent_state
/mpc/interaction_debug or /mpc/intent_debug
/mpc/interaction_target or /mpc/intent_target
```

- [ ] Analyze:

```text
mode_counts
mode_transition_count
PASS_BEHIND dwell time
YIELD dwell time
crossing_confirm_count
crossing_clear_count
behind_target_valid_ratio
pass-mode switch count
minimum clearance
path efficiency
stop/yield duration
planning time
```

Expected:

```text
E vs C comparison should include both safety and mode-stability metrics.
```

---

## Task 8: E2 Yield Prompt Only After E1

**Goal:** Add YIELD as prompt-oriented behavior, not as the main way to win experiments.

**Files:**

- Modify `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`
- Modify launch files if needed.

- [ ] Enable `YIELD` only when:

```text
confirmed crossing conflict exists
behind target is infeasible or conflict is too close
mpc_interaction_enable_yield=true
```

- [ ] Default behavior:

```text
mpc_enable_stop_advice=true
mpc_enable_stop_enforce=false
```

- [ ] Physical stop enforcement is allowed only in a dedicated safety test, not in the main E1 comparison.
- [ ] Enforce Stage 2 stop/yield constraint:

```text
E stop/yield duration <= 30% task duration unless C has collision/near-collision.
```

Expected:

```text
YIELD improves safety in hard conflicts without turning the method into “停着赢”.
```

---

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
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch plan_manage sim_kin_replan.launch planner:=3 use_pedestrians:=true pedestrian_scenario:=crossing mpc_w_risk:=2.0 mpc_risk_sigma_y:=0.38 mpc_enable_cpa:=false mpc_enable_adaptive_risk:=false mpc_enable_yield:=false mpc_enable_stop_advice:=false mpc_enable_stop_enforce:=false mpc_enable_dynamic_hard_reject:=false mpc_enable_interaction:=true mpc_interaction_enable_pass_behind:=true mpc_interaction_enable_yield:=false
```

If the current code still uses `mpc_enable_intent` names, use the existing names until parameters are renamed consistently.

Record E1:

```shell
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && rosrun plan_manage record_lightweight_mpc_eval.sh light_crossing_E1_interaction
```

Analyze E1:

```shell
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && rosrun plan_manage analyze_mpc_eval.py $(ls -td /home/xcg/ws/records/*light_crossing_E1_interaction | head -1) --odom-topic /sim_odom --robot-radius 0.35
```

---

## Self-review

- Keeps Stage 2 focused on single-pedestrian crossing.
- Keeps mode set limited to `CONTINUE`, `PASS_BEHIND`, and `YIELD`.
- Does not introduce `PASS_AHEAD` or `PASS_SIDE` into the current implementation.
- Reframes the method as interaction decision plus scene/mode-conditioned social cost, not direct rule control.
- Includes temporal consistency: consecutive-frame confirmation, switch margin, stable clear condition, and optional short debounce.
- Keeps YIELD last to avoid “safe by stopping”.
- Adds metrics needed to compare C and E on safety, stability, efficiency, and runtime.
- Does not require commits during implementation.
