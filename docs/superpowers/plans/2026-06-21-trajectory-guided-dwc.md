# Trajectory-Guided Dynamic Walking Corridor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a receding-horizon trajectory-corridor-trajectory loop where the previous MPPI human-feasible trajectory generates the next time-indexed DWC.

**Architecture:** Add a small trajectory/corridor data layer in `path_searching`, connect it through `PlannerManager`, and extend MPPI diagnostics before changing hard runtime behavior. A* remains the bootstrap/global guide; previous MPPI best trajectory becomes the normal DWC reference.

**Tech Stack:** ROS1 Noetic, C++14, Eigen, catkin_tools, existing `path_searching` and `plan_manage` packages.

---

## File Map

- Create `path_searching/include/path_searching/timed_trajectory.h`
  - Owns lightweight data structs for timed local trajectories and source labels.
- Create `path_searching/include/path_searching/timed_walking_corridor.h`
  - Owns lightweight data structs for time-indexed corridor segments.
- Modify `path_searching/include/path_searching/dynamic_walking_corridor.h`
  - Add an API that accepts `TimedTrajectory`.
- Modify `path_searching/src/dynamic_walking_corridor.cpp`
  - Implement timed trajectory to corridor conversion using existing polyline DWC machinery.
- Modify `path_searching/include/path_searching/mpc_controller.h`
  - Add timed corridor setters and debug fields.
- Modify `path_searching/src/mpc_controller.cpp`
  - Penalize or reject rollout points against time-matched corridor segments.
- Modify `plan_manage/include/plan_manager.h`
  - Store previous MPPI timed trajectory and nominal source state.
- Modify `plan_manage/src/planner_manager.cpp`
  - Build nominal trajectory from previous MPPI path or A* bootstrap.
- Modify `plan_manage/scripts/record_corridor_debug.py`
  - Record any new debug topics.
- Modify `plan_manage/scripts/analyze_corridor_debug.py`
  - Summarize nominal trajectory source and timed corridor feasibility.
- Add or modify tests under `path_searching/test/`.

---

## Task 1: Freeze Current Baseline and Add Compile-Time Data Types

**Files:**
- Create: `path_searching/include/path_searching/timed_trajectory.h`
- Create: `path_searching/include/path_searching/timed_walking_corridor.h`
- Modify: `path_searching/CMakeLists.txt` only if tests require new files.

- [ ] **Step 1: Create `TimedTrajectory` structs**

Add:

```cpp
#ifndef _TIMED_TRAJECTORY_H_
#define _TIMED_TRAJECTORY_H_

#include <Eigen/Eigen>
#include <vector>

namespace cane_planner
{

enum class TimedTrajectorySource
{
    EMPTY = 0,
    ASTAR_BOOTSTRAP = 1,
    PREVIOUS_MPPI = 2
};

struct TimedTrajectoryPoint
{
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    double yaw = 0.0;
    double t_from_now = 0.0;
};

struct TimedTrajectory
{
    TimedTrajectorySource source = TimedTrajectorySource::EMPTY;
    std::vector<TimedTrajectoryPoint> points;

    bool valid() const { return points.size() >= 2; }
};

} // namespace cane_planner

#endif // _TIMED_TRAJECTORY_H_
```

- [ ] **Step 2: Create `TimedWalkingCorridor` structs**

Add:

```cpp
#ifndef _TIMED_WALKING_CORRIDOR_H_
#define _TIMED_WALKING_CORRIDOR_H_

#include <Eigen/Eigen>
#include <vector>

namespace cane_planner
{

struct TimedWalkingCorridorSegment
{
    double t_start = 0.0;
    double t_end = 0.0;
    Eigen::Vector2d start = Eigen::Vector2d::Zero();
    Eigen::Vector2d end = Eigen::Vector2d::Zero();
    Eigen::Vector2d forward = Eigen::Vector2d::UnitX();
    Eigen::Vector2d left = Eigen::Vector2d::UnitY();
    double half_width = 0.0;
    bool feasible = true;
    bool blocked_static = false;
    bool blocked_dynamic = false;
};

struct TimedWalkingCorridor
{
    std::vector<TimedWalkingCorridorSegment> segments;

    bool valid() const { return !segments.empty(); }
};

} // namespace cane_planner

#endif // _TIMED_WALKING_CORRIDOR_H_
```

- [ ] **Step 3: Build**

Run:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin build path_searching --no-deps -DCMAKE_BUILD_TYPE=Release
```

Expected:

```text
Summary: All 1 packages succeeded
```

- [ ] **Step 4: Commit**

```bash
cd /home/xcg/ws/src/cane_planner
git add path_searching/include/path_searching/timed_trajectory.h \
        path_searching/include/path_searching/timed_walking_corridor.h
git commit -m "Add timed trajectory corridor data types"
```

---

## Task 2: Build Nominal Trajectory in PlannerManager

**Files:**
- Modify: `plan_manage/include/plan_manager.h`
- Modify: `plan_manage/src/planner_manager.cpp`
- Test manually through debug logs first.

- [ ] **Step 1: Add members**

Add PlannerManager members:

```cpp
cane_planner::TimedTrajectory last_mppi_timed_trajectory_;
cane_planner::TimedTrajectory current_nominal_timed_trajectory_;
double timed_nominal_dt_ = 0.2;
double timed_nominal_max_age_ = 1.0;
ros::Time last_mppi_trajectory_time_;
```

- [ ] **Step 2: Convert MPPI best path to timed trajectory**

After `mpc_controller_->plan(...)`, read:

```cpp
std::vector<Eigen::Vector3d> best_path = mpc_controller_->getBestPath();
```

Convert each point:

```cpp
TimedTrajectoryPoint p;
p.position = best_path[i].head<2>();
p.yaw = (i + 1 < best_path.size())
    ? std::atan2(best_path[i + 1].y() - best_path[i].y(),
                 best_path[i + 1].x() - best_path[i].x())
    : last_theta_;
p.t_from_now = i * timed_nominal_dt_;
```

Save as `PREVIOUS_MPPI` only when at least two points exist.

- [ ] **Step 3: Build A* bootstrap nominal**

When no valid previous MPPI trajectory exists, build a timed trajectory from the local prefix of `global_path_dense_`.

Use the same point shape and assign:

```cpp
source = TimedTrajectorySource::ASTAR_BOOTSTRAP
```

Time is assigned by approximate walking speed:

```cpp
t_from_now += segment_length / std::max(0.2, mpc_nominal_al_);
```

- [ ] **Step 4: Add log-only verification**

Print throttled logs:

```text
[TrajectoryNominal] source=PREVIOUS_MPPI points=20 length=...
[TrajectoryNominal] source=ASTAR_BOOTSTRAP points=...
```

- [ ] **Step 5: Build**

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin build plan_manage --no-deps -DCMAKE_BUILD_TYPE=Release
```

- [ ] **Step 6: Commit**

```bash
cd /home/xcg/ws/src/cane_planner
git add plan_manage/include/plan_manager.h plan_manage/src/planner_manager.cpp
git commit -m "Track timed nominal trajectory for MPPI"
```

---

## Task 3: Generate DWC from Timed Nominal Trajectory

**Files:**
- Modify: `path_searching/include/path_searching/dynamic_walking_corridor.h`
- Modify: `path_searching/src/dynamic_walking_corridor.cpp`
- Add: `path_searching/test/test_timed_walking_corridor.cpp`
- Modify: `path_searching/CMakeLists.txt`

- [ ] **Step 1: Add API**

Add:

```cpp
TimedWalkingCorridor planTimed(
    const TimedTrajectory &nominal,
    const std::vector<Eigen::Vector3d> &obs_pos,
    const std::vector<Eigen::Vector3d> &obs_vel,
    const std::vector<Eigen::Vector3d> &obs_size = {}) const;
```

- [ ] **Step 2: First implementation uses existing reference-path DWC**

Convert `TimedTrajectory` points to `std::vector<Eigen::Vector2d>`, call existing:

```cpp
auto result = plan(reference_path, obs_pos, obs_vel, obs_size);
```

Then split selected centerline into timed segments using nearest nominal times.

- [ ] **Step 3: Add unit test**

Test:

```cpp
TEST(TimedWalkingCorridor, BuildsSegmentsWithIncreasingTime)
```

Expected:

```text
segments are non-empty
t_end >= t_start
segment start/end follow nominal direction
```

- [ ] **Step 4: Build and test**

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin build path_searching --no-deps -DCMAKE_BUILD_TYPE=Release
catkin test path_searching --no-deps --catkin-make-args run_tests
catkin_test_results build/path_searching
```

- [ ] **Step 5: Commit**

```bash
git add path_searching/include/path_searching/dynamic_walking_corridor.h \
        path_searching/src/dynamic_walking_corridor.cpp \
        path_searching/test/test_timed_walking_corridor.cpp \
        path_searching/CMakeLists.txt
git commit -m "Generate walking corridor from timed nominal trajectory"
```

---

## Task 4: Connect Timed DWC to MPPI as Soft Constraint

**Files:**
- Modify: `path_searching/include/path_searching/mpc_controller.h`
- Modify: `path_searching/src/mpc_controller.cpp`

- [ ] **Step 1: Add config**

Add:

```cpp
bool timed_corridor_enable = false;
bool timed_corridor_hard_reject_enable = false;
double w_timed_corridor = 20.0;
double timed_corridor_hard_margin = 0.30;
```

- [ ] **Step 2: Add setter**

Add:

```cpp
void setTimedWalkingCorridor(const TimedWalkingCorridor &corridor);
void clearTimedWalkingCorridor();
```

- [ ] **Step 3: Add debug counters**

Add:

```cpp
int timed_corridor_reject_count = 0;
double timed_corridor_inside_ratio = 0.0;
double max_timed_corridor_violation = 0.0;
int timed_corridor_segments = 0;
```

- [ ] **Step 4: Apply soft penalty in rollout**

For each rollout point at step `i`, use the corridor segment whose `[t_start, t_end]` contains rollout time.

If outside:

```cpp
cost += cfg_.w_timed_corridor * violation * violation;
```

If hard reject is enabled and violation exceeds margin:

```cpp
cost = infinity;
```

- [ ] **Step 5: Build**

```bash
catkin build path_searching --no-deps -DCMAKE_BUILD_TYPE=Release
```

- [ ] **Step 6: Commit**

```bash
git add path_searching/include/path_searching/mpc_controller.h \
        path_searching/src/mpc_controller.cpp
git commit -m "Constrain MPPI with timed walking corridor"
```

---

## Task 5: PlannerManager Runtime Integration

**Files:**
- Modify: `plan_manage/include/plan_manager.h`
- Modify: `plan_manage/src/planner_manager.cpp`
- Modify: `plan_manage/launch/include/algorithm.launch`
- Modify: `plan_manage/launch/sim_kin_replan.launch`

- [ ] **Step 1: Add launch params**

Add:

```xml
<arg name="timed_dwc_enable" default="true"/>
<arg name="timed_dwc_use_previous_mppi" default="true"/>
```

- [ ] **Step 2: Build nominal trajectory each MPC_STEP**

Use:

```text
previous MPPI if valid
else A* bootstrap
```

- [ ] **Step 3: Generate timed DWC**

Call:

```cpp
auto timed_corridor = dynamic_walking_corridor_->planTimed(
    current_nominal_timed_trajectory_,
    obs_pos,
    obs_vel,
    obs_size);
```

- [ ] **Step 4: Pass timed DWC to MPPI**

Call:

```cpp
mpc_controller_->setTimedWalkingCorridor(timed_corridor);
```

If invalid:

```cpp
mpc_controller_->clearTimedWalkingCorridor();
```

- [ ] **Step 5: Keep old DWC visualization for transition**

Do not delete `/mpc/walking_corridors` yet. Keep it for comparison until timed DWC debug is validated.

- [ ] **Step 6: Build**

```bash
catkin build path_searching plan_manage --no-deps -DCMAKE_BUILD_TYPE=Release
```

- [ ] **Step 7: Commit**

```bash
git add plan_manage/include/plan_manager.h \
        plan_manage/src/planner_manager.cpp \
        plan_manage/launch/include/algorithm.launch \
        plan_manage/launch/sim_kin_replan.launch
git commit -m "Use previous MPPI trajectory for timed DWC"
```

---

## Task 6: Debug Topics and Analyzer

**Files:**
- Modify: `plan_manage/src/planner_manager.cpp`
- Modify: `plan_manage/include/plan_manager.h`
- Modify: `plan_manage/scripts/record_corridor_debug.py`
- Modify: `plan_manage/scripts/analyze_corridor_debug.py`

- [ ] **Step 1: Publish nominal trajectory**

Topic:

```text
/mpc/nominal_timed_trajectory
```

Use `visualization_msgs/MarkerArray` or `nav_msgs/Path` plus text marker for source.

- [ ] **Step 2: Publish timed corridor debug**

Topic:

```text
/mpc/timed_walking_corridor
```

Each segment marker text includes:

```text
t=[0.20,0.40] w=0.35 feasible=1 dyn=0 st=0
```

- [ ] **Step 3: Record topics**

Add both topics to `record_corridor_debug.py`.

- [ ] **Step 4: Analyzer summary**

Add summary:

```text
Timed nominal:
  source PREVIOUS_MPPI / ASTAR_BOOTSTRAP counts
  length mean/min/max

Timed walking corridor:
  segments mean/min/max
  time span mean/min/max
  infeasible segments count

MPPI timed corridor:
  inside ratio
  reject count
```

- [ ] **Step 5: Commit**

```bash
git add plan_manage/src/planner_manager.cpp \
        plan_manage/include/plan_manager.h \
        plan_manage/scripts/record_corridor_debug.py \
        plan_manage/scripts/analyze_corridor_debug.py
git commit -m "Record timed DWC diagnostics"
```

---

## Task 7: STOP Semantics Cleanup

**Files:**
- Modify: `plan_manage/src/planner_manager.cpp`
- Modify: `plan_manage/scripts/analyze_corridor_debug.py`

- [ ] **Step 1: Keep debug reasons, change main decision**

Main GO/STOP should depend on:

```text
timed corridor exists
MPPI valid_sample_ratio above threshold
best trajectory satisfies timed corridor constraint
```

- [ ] **Step 2: Rename paper-facing reason**

Use:

```text
NO_FEASIBLE_TIMED_TRAJECTORY
```

for the main method-level STOP.

- [ ] **Step 3: Keep old reasons as debug fields**

Old reasons remain in logs, but analyzer should show them as secondary diagnostics.

- [ ] **Step 4: Commit**

```bash
git add plan_manage/src/planner_manager.cpp \
        plan_manage/scripts/analyze_corridor_debug.py
git commit -m "Use feasibility based stop for timed DWC"
```

---

## Task 8: Mixed Scenario Regression

**Files:**
- Modify only test config if needed:
  - `plan_manage/config/lightweight_fixed_goals.yaml`
  - or add a new scenario label such as `mixed_research`.

- [ ] **Step 1: Confirm start/goal manually**

Run:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch plan_manage sim_kin_replan.launch planner:=3 pedestrian_scenario:=mixed
```

Use RViz to choose a start/goal whose A* path crosses the intended mixed pedestrian area.

- [ ] **Step 2: Save confirmed start/goal**

Add a separate scenario in `lightweight_fixed_goals.yaml`:

```yaml
mixed_research:
  description: "Validated mixed route crossing pedestrian interaction area."
```

- [ ] **Step 3: Run three automated bags**

```bash
for i in 01 02 03; do
  rosrun plan_manage run_corridor_regression.py mixed_research_$i --scenario mixed_research --duration 60
done
```

- [ ] **Step 4: Acceptance checks**

Each summary should show:

```text
timed nominal source includes PREVIOUS_MPPI after bootstrap
timed corridor segments > 0
rosout_stop_logs not dominated by false corridor infeasibility
mpc_path length comparable to astar path
```

- [ ] **Step 5: Commit**

```bash
git add plan_manage/config/lightweight_fixed_goals.yaml
git commit -m "Add validated mixed research scenario"
```

---

## Execution Order

Implement tasks in order. Do not start Task 4 before Task 3 has tests. Do not change STOP semantics before timed corridor debug data exists. Do not tune mixed scenario behavior before the start/goal route is confirmed.

## Rollback Rule

If a task introduces worse runtime behavior, revert only that task commit and keep earlier tasks. Do not add fallback gates to mask the failure.
