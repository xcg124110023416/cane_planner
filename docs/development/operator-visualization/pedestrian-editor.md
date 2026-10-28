# Deterministic pedestrian editor

`plan_manage` exports `PedestrianPanel` (RViz Panel) and `PlacePedestrian` (RViz Tool) via `rviz_plugins.xml`. Both are included in `config/replan.rviz`. The tool inherits RViz's `GoalTool`; link **librviz_default_plugin**, in addition to librviz, or pluginlib will fail at load time with undefined GoalTool symbols even when compilation succeeds. Use shortcut `P`; leave robot `2D Nav Goal` and `/move_base_simple/goal` intact.

## Contracts

- `/pedestrian_sim/initial_pose`: `geometry_msgs/PoseStamped`, position plus walking yaw. Transform into the simulator frame with TF; never merely relabel a foreign frame.
- `/pedestrian_sim/control`: `plan_manage/ControlPedestrian`, acknowledged apply/start/pause/reset/clear/load/export commands. Apply/load validate all settings before replacing the person and reset to the start. Load sends YAML contents (not a backend-host path); export returns YAML of authoritative applied state.
- `/pedestrian_sim/state`: latched `plan_manage/PedestrianState` snapshots, refreshed at 10 Hz. The simulator owns state; marker geometry and panel values are not the source of truth.
- The panel pumps a private ROS callback queue on the Qt thread. Periodic state snapshots must not overwrite pending form edits. Explicit apply/load force a form refresh even when loaded values equal the previous applied state. Save exports applied state, not unsent form values.
- Chinese labels require CJK font coverage (`fonts-wqy-microhei` runtime dependency). Missing glyph boxes are a font issue even when the UTF-8 strings are correct. Verify with `QFontMetrics.inFontUcs4`; restart existing RViz processes after installing fonts.
- `/pedestrian_sim/visualization` includes the body, heading arrow and route preview. Preview points never enter `/onboard_detector/dynamic_obstacles_info`.

## Motion and compatibility

`scripts/cane_pedestrian` contains the ROS-independent motion model and is exported using `catkin_python_setup()`, so both devel relay scripts and install-space nodes can import it. Without remembered settings, existing trigger/loop/patrol YAMLs retain their legacy behavior until an operator explicitly places/applies/starts/resets/loads a single person. Multi-person scenes cannot be edited or silently truncated.

Controlled mode uses speed in [0, 10] m/s and **one-way** duration T in [0.1, 3600] seconds. End policies: stop/disappear after T, return and stop after 2T, or repeat with period 2T. Evaluate position from elapsed motion time, clamping terminal endpoints and retaining overshoot across turns. Pausing freezes motion time; resuming must preserve travel direction. Placing, applying and resetting leave a visible stationary person waiting at the start. A navigation goal restarts only people configured with `start_on_goal`.

Pause must continue fresh zero-velocity observations; clear/disappear must continue fresh empty observations. Stopping publication would trigger planner 3's stale-data STOP. Editor callbacks and the simulator loop share a lock so partial configuration never leaks into observations.

## Loading and remembering settings

`scene_io.py` owns the YAML schema, validation, and atomic cache writes. The panel provides file dialogs; both file export and automatic memory use the backend's serialization. Saved YAML contains one named scene with one `mode: controlled` entry: frame_id, start, yaw (radians), speed, duration, size, end_behavior, start_on_goal, placed. Older panel exports without frame_id/placed default to the current simulator frame/placed=true. Explicit foreign frames are rejected, never relabeled; planner 3 uses `world`. Malformed/multi-person files leave live state and the last cache untouched.

By default the simulator remembers successfully applied configuration, including the cleared state, under `${ROS_HOME:-~/.ros}/cane_planner/pedestrian_editor/`. The filename hashes scenario name, resolved scenario-config path and frame. Context changes therefore select separate memory. Restore only for an originally zero/one-person scene; start at the configured origin waiting, never at a transient running pose/time. A missing or corrupt cache falls back to the launch scene. Cache writes use a temporary file plus atomic replace; IO failures are logged and reported in PedestrianState without rolling back valid live edits. No disk writes on periodic observation ticks.

`pedestrian_remember_last:=false` in `sim_kin_replan.launch` disables both cache reads and writes (`~remember_last`). `~editor_state_file` overrides the cache path for controlled testing. To force a particular startup YAML, combine `pedestrian_scenario_config` and `pedestrian_scenario:=custom_pedestrian` with memory disabled. The normal user flow is panel loading or automatic restore.

## Verification

- `test/test_pedestrian_motion.py`: terminal endpoints, return/repeat timing, pause/resume, stationary heading, legacy behavior, YAML compatibility and context isolation.
- `test/pedestrian_editor.test`: actual simulator, TF placement, separate goal topic, triggers, rejected settings, fresh paused/empty observations.
- `test/pedestrian_panel.test`: pluginlib loading of tool/panel and Qt button-to-service controls, pending edits, YAML export and loading after clear. Uses Qt offscreen; full RViz rendering is a separate smoke check.
- `test/pedestrian_persistence.test`: actually restarts the simulator to verify applied/cleared state restore, malformed load rejection, memory opt-out and corrupt-cache fallback. Existing non-persistence rostests set remember_last=false; persistence tests override editor_state_file into a temporary directory. Never read or modify a developer's actual remembered configuration in tests.

Build with `catkin build plan_manage --no-deps`; build test executables with `--catkin-make-args tests`. Run `catkin test plan_manage` (`catkin test` does not support `--no-deps`). Use a separate ROS master for smoke checks and leave any user-owned nodes untouched.

The pre-existing `test_sim_odom_stop` gtest also needs a reachable ROS master. Start a dedicated test roscore and set ROS_MASTER_URI before running the package suite; otherwise that gtest waits for registration indefinitely. The new rostests launch their own isolated masters.
