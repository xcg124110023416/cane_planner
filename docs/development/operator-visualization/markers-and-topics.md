# Markers and Topic Contracts

## Standard message shapes

Prefer standard ROS visualization messages already used by the stack:

- planner output: `nav_msgs/Path` (`/astar/path`, `/mpc/path`, optionally `/kin_astar/path`)
- point clouds: `sensor_msgs/PointCloud2` for map and sensor displays
- goals: `geometry_msgs/PoseStamped` on `/move_base_simple/goal`
- obstacles and debug primitives: `visualization_msgs/Marker` or `MarkerArray`

Dynamic obstacles are represented as cube markers. `plan_env/src/map_ros.cpp` accepts `MarkerArray`, filters `ADD` + `CUBE` markers with positive dimensions, and converts pose/scale into obstacle boxes. `plan_env/src/obj_generator.cpp` publishes simulation cube markers on `/dynamic/obj`.

## Marker construction

Follow the existing controller pattern in `plan_ctrl/src/L1_controller_v2.cpp`: initialize marker frame and namespace once, set clear type/color/scale values, clear and repopulate transient point/line data each cycle, then publish. Use stable topic names and namespaces so RViz can retain display configuration.

Planner visualization is published by `plan_manage/src/planner_manager.cpp` for paths, samples, trajectories, and foot markers. If a marker topic changes, update its RViz display and any scripts that consume it.

## Topic/frame checklist

Before changing an operator-facing message:

1. Find the publisher and subscriber with `grep`.
2. Check the launch remap and parameter source.
3. Check `plan_manage/config/replan.rviz` and any scripts.
4. Verify the frame is connected to `world`/`map`/robot TF at runtime.
5. Exercise an actual launch scenario, not only static YAML syntax.

## Avoid

- Do not use a custom message where `Path`, `Marker(Array)`, `PoseStamped`, or `PointCloud2` already expresses the contract.
- Do not publish markers with zero dimensions, an invalid frame, or an unbounded lifetime when the consumer expects transient updates.
- Do not treat RViz markers as persistent application state; they are visualization output derived from runtime ROS state.

## LFPC time-matched prediction display

Planner 3 publishes `/mpc/timed_collision` MarkerArray and `/mpc/timed_best_path`
Path. RViz MPC_TimedCollision shows up to ten same-color robot envelope/pedestrian
hull pairs, cyan to magenta with +seconds text. Red Rejected on STOP denotes one
rejected sample; it is not an executed trajectory. Path pose headers carry actual
future times; marker headers stay at the prediction epoch so TF is not queried in
the future. All are in world. Dedicated DELETEALL and .6-second marker lifetimes
prevent leftover IDs. STOP/empty/invalid/reset/arrival publishes an empty timed
path. During lightweight execution this topic is replaced by the currently
checked remainder of the first support phase, usually at most .28s under the
default discretization. The plant advances that phase with elapsed model time.
Geometry/timing contract: planning-runtime/lfpc-timed-collision.md.

`/mpc/timed_plan` is a separate MarkerArray, enabled as MPC_TimedPlan in both
default RViz configurations. It retains up to ten pairs from the latest verified
full MPPI horizon, with `Plan +seconds` labels and the original planning epoch.
Execution updates must not erase this display. This is a planning snapshot, not
current execution authorization; do not re-stamp it as a new live prediction.
New plans replace it; STOP/invalid/empty/reset/arrival clear it, and its markers
expire after .6s even if no further messages arrive. A fresh empty execution
frame also clears it immediately. All three evaluation/debug recorders capture
the topic. The short checked remainder remains visible in MPC_TimedCollision.

## LFPC MPPI outputs

`/mpc/sample_trajectories` displays uniformly selected last-iteration sampled
rollouts (default 20, cap 50), cyan for feasible and orange for rejected prefixes.
It does not draw new random samples or display an unsafe weighted trajectory as
the executed path. `/mpc/best_traj` remains the selected verified rollout.
`/mpc/optimization_evidence` Float64MultiArray replaces historical beam_evidence;
its versioned layout label defines counts, ESS, weighted rechecks, replay and
per-iteration costs. The retained-best curve is nonincreasing by construction,
not a certificate of local optimality. Execution-epoch authorization still uses
STOP and debug indices 28/29. See planning-runtime/lfpc-mppi.md.

## Planner-3 directional geometry and preview (2026-10-27)

`/mpc/human_cane` draws production geometry at the current pose in yellow and up
to eight checked prediction poses in cyan, including when no pedestrians exist.
Timed robot markers rotate the identical envelope using saved sample yaw; timed
Path orientations carry that yaw, and step-root rotation samples may share times.
`/mpc/convex_corridor` includes physical body, raw social triangle, used collision
hull, force/FOV and active failure point. Do not emit an empty social LINE_STRIP
for a stationary observation. `/mpc/corridor_preview` is a separate pale outline
with an explicit Preview only / Preview blocked label and observation age. Its
async result must pass source/receipt freshness and route/goal revision before
publication; TTL never outlives the bound source. No motion authority. RViz names
are MPC_ActiveCorridor, MPC_HumanCane and MPC_CorridorPreview.

Planner force diagnostics share `/mpc/convex_corridor`: cyan map repulsion,
orange robot repulsion, blue peer repulsion, magenta total and green observed velocity. Their
namespaces are `mpc_pedestrian_map_force`, `mpc_pedestrian_robot_force`,
`mpc_pedestrian_peer_force`, `mpc_pedestrian_force`, and `mpc_pedestrian_velocity`. Per-frame person ids 0..N-1 keep all people visible; empty/invalid/reset clears ids 0..4. The per-person text marks
these forces as planner estimates; pedsim uses its own motion model. Force
arrows scale by 0.3 s² and velocity by 0.8 s. Zero vectors omit arrows. All
components and the text use the existing 0.3 s lifetime and are deleted with
pedestrian geometry on invalid/empty/reset inputs. Artificial map crop edges
no longer generate planner social force; occupied map bins still contribute.
