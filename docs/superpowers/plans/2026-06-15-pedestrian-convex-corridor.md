# Pedestrian Convex Corridor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Build a literature-style pedestrian-aware convex corridor pipeline on top of the current mixed-scenario DWC baseline.

**Architecture:** Keep the current A* + DWC + MPPI system as the working baseline. Add a separate convex corridor module that converts the local path, static SDF/map, and predicted pedestrians into half-space corridor segments. First validate corridor generation and visualization from bags, then enforce the corridor in MPPI, and finally add an optional continuous trajectory optimizer inside the corridor.

**Tech Stack:** ROS1 Noetic, C++14, Eigen, existing `path_searching`, `plan_manage`, `plan_env`, gtest, rosbag diagnostics.

---

## Current baseline

The current committed baseline is:

```text
28933f8 Stabilize mixed-scenario walking corridor
```

It solves the immediate mixed-scene stuck issue with:

- A* traversable-radius global guide filtering.
- Dynamic Walking Corridor static centerline optimization.
- Static width shrinking and truncation.
- Start grace near the current cane pose.
- MPPI soft/hard reject split.
- Bag recording and analysis scripts.

The next stage should not replace this baseline immediately. The convex corridor should be developed as a parallel module, toggled by launch parameters, and compared against the current DWC behavior.

## Definition of done

This stage is complete only when all of the following are true:

- The planner publishes a single selected convex corridor as half-space segments.
- Static obstacles are represented by convex half-space constraints in each segment.
- Pedestrian predictions can carve time-indexed forbidden regions or yield constraints from the corridor.
- MPPI can score or reject rollout points against the convex corridor.
- A bag summary reports corridor feasibility, segment count, min corridor width, static reject count, dynamic reject count, and optimizer status.
- Mixed-scene regression bags show no repeated `CORRIDOR_INFEASIBLE` loops.
- Existing DWC tests still pass.

## File structure

Create:

- `/home/xcg/ws/src/cane_planner/path_searching/include/path_searching/convex_corridor.h`  
  Data structures and API for convex corridor segments.

- `/home/xcg/ws/src/cane_planner/path_searching/src/convex_corridor.cpp`  
  Static and pedestrian-aware corridor construction.

- `/home/xcg/ws/src/cane_planner/path_searching/test/test_convex_corridor.cpp`  
  Unit tests for half-space geometry, static obstacle clipping, and pedestrian time clipping.

- `/home/xcg/ws/src/cane_planner/docs/CONVEX_CORRIDOR_METHOD.md`  
  Method notes, assumptions, parameters, and bag evidence.

Modify:

- `/home/xcg/ws/src/cane_planner/path_searching/CMakeLists.txt`  
  Build the new corridor library/test.

- `/home/xcg/ws/src/cane_planner/path_searching/include/path_searching/mpc_controller.h`  
  Add optional convex corridor input and debug counters.

- `/home/xcg/ws/src/cane_planner/path_searching/src/mpc_controller.cpp`  
  Evaluate rollout points against half-space constraints.

- `/home/xcg/ws/src/cane_planner/plan_manage/include/plan_manager.h`  
  Own the convex corridor planner and publishers.

- `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`  
  Build local convex corridors from the same reference path used by DWC, publish markers, pass constraints to MPPI.

- `/home/xcg/ws/src/cane_planner/plan_manage/launch/include/algorithm.launch`  
  Add feature flags and default parameters.

- `/home/xcg/ws/src/cane_planner/plan_manage/scripts/analyze_corridor_debug.py`  
  Parse convex corridor debug labels and optimizer status.

---

### Task 1: Add Convex Corridor Data Model

**Files:**
- Create: `/home/xcg/ws/src/cane_planner/path_searching/include/path_searching/convex_corridor.h`
- Create: `/home/xcg/ws/src/cane_planner/path_searching/src/convex_corridor.cpp`
- Modify: `/home/xcg/ws/src/cane_planner/path_searching/CMakeLists.txt`
- Test: `/home/xcg/ws/src/cane_planner/path_searching/test/test_convex_corridor.cpp`

- [x] **Step 1: Add the header**

Create `convex_corridor.h` with these core types:

```cpp
#ifndef PATH_SEARCHING_CONVEX_CORRIDOR_H
#define PATH_SEARCHING_CONVEX_CORRIDOR_H

#include <Eigen/Eigen>
#include <vector>

namespace cane_planner
{

class ConvexCorridor
{
public:
    struct Halfspace
    {
        Eigen::Vector2d normal = Eigen::Vector2d::Zero();
        double offset = 0.0;  // normal.dot(p) <= offset
    };

    struct Segment
    {
        double s0 = 0.0;
        double s1 = 0.0;
        double t0 = 0.0;
        double t1 = 0.0;
        Eigen::Vector2d center = Eigen::Vector2d::Zero();
        std::vector<Halfspace> halfspaces;
        bool static_feasible = true;
        bool dynamic_feasible = true;
    };

    struct Config
    {
        bool enable = false;
        double segment_length = 0.8;
        double half_width = 0.45;
        double min_half_width = 0.25;
        double static_sample_ds = 0.2;
        double static_sample_dl = 0.1;
        double pedestrian_radius = 0.35;
        double pedestrian_time_margin = 0.4;
        double start_grace_length = 0.4;
    };

    struct Result
    {
        std::vector<Segment> segments;
        bool feasible = false;
        double min_width = 0.0;
        int static_block_count = 0;
        int dynamic_block_count = 0;
    };

    static bool contains(const Segment &segment, const Eigen::Vector2d &point, double tol = 1e-6);
    static double violation(const Segment &segment, const Eigen::Vector2d &point);
};

}  // namespace cane_planner

#endif
```

- [x] **Step 2: Add geometry implementation**

Create `convex_corridor.cpp`:

```cpp
#include <path_searching/convex_corridor.h>

#include <algorithm>

namespace cane_planner
{

bool ConvexCorridor::contains(const Segment &segment, const Eigen::Vector2d &point, double tol)
{
    for (const auto &h : segment.halfspaces)
    {
        if (h.normal.dot(point) - h.offset > tol)
            return false;
    }
    return true;
}

double ConvexCorridor::violation(const Segment &segment, const Eigen::Vector2d &point)
{
    double max_v = 0.0;
    for (const auto &h : segment.halfspaces)
        max_v = std::max(max_v, h.normal.dot(point) - h.offset);
    return std::max(0.0, max_v);
}

}  // namespace cane_planner
```

- [x] **Step 3: Add minimal tests**

Create `test_convex_corridor.cpp`:

```cpp
#include <gtest/gtest.h>

#include <path_searching/convex_corridor.h>

using cane_planner::ConvexCorridor;

TEST(ConvexCorridor, ContainsPointInsideRectangle)
{
    ConvexCorridor::Segment seg;
    seg.halfspaces = {
        {Eigen::Vector2d(1.0, 0.0), 1.0},
        {Eigen::Vector2d(-1.0, 0.0), 1.0},
        {Eigen::Vector2d(0.0, 1.0), 0.5},
        {Eigen::Vector2d(0.0, -1.0), 0.5},
    };

    EXPECT_TRUE(ConvexCorridor::contains(seg, Eigen::Vector2d(0.0, 0.0)));
    EXPECT_TRUE(ConvexCorridor::contains(seg, Eigen::Vector2d(0.8, 0.3)));
    EXPECT_FALSE(ConvexCorridor::contains(seg, Eigen::Vector2d(1.2, 0.0)));
    EXPECT_NEAR(0.2, ConvexCorridor::violation(seg, Eigen::Vector2d(1.2, 0.0)), 1e-9);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

- [x] **Step 4: Wire CMake**

Modify `path_searching/CMakeLists.txt` to build:

```cmake
add_library(convex_corridor
  src/convex_corridor.cpp
)
target_link_libraries(convex_corridor ${catkin_LIBRARIES})

catkin_add_gtest(test_convex_corridor test/test_convex_corridor.cpp)
if(TARGET test_convex_corridor)
  target_link_libraries(test_convex_corridor convex_corridor ${catkin_LIBRARIES})
endif()
```

- [x] **Step 5: Verify**

Run:

```bash
cd /home/xcg/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin build path_searching --no-deps -DCMAKE_BUILD_TYPE=Release
catkin test path_searching --no-deps --catkin-make-args run_tests
catkin_test_results build/path_searching
```

Expected:

```text
0 errors, 0 failures
```

- [x] **Step 6: Commit**

```bash
cd /home/xcg/ws/src/cane_planner
git add path_searching/include/path_searching/convex_corridor.h \
        path_searching/src/convex_corridor.cpp \
        path_searching/test/test_convex_corridor.cpp \
        path_searching/CMakeLists.txt
git commit -m "Add convex corridor geometry model"
```

---

### Task 2: Generate Static Convex Segments From Local Path

**Files:**
- Modify: `/home/xcg/ws/src/cane_planner/path_searching/include/path_searching/convex_corridor.h`
- Modify: `/home/xcg/ws/src/cane_planner/path_searching/src/convex_corridor.cpp`
- Test: `/home/xcg/ws/src/cane_planner/path_searching/test/test_convex_corridor.cpp`

- [x] **Step 1: Add static corridor API**

Add to `ConvexCorridor`:

```cpp
using TraversableFn = std::function<bool(double, double)>;

Result buildStatic(const std::vector<Eigen::Vector2d> &reference_path,
                   const TraversableFn &is_traversable) const;

void setConfig(const Config &cfg) { cfg_ = cfg; }
const Config &getConfig() const { return cfg_; }

private:
Config cfg_;
```

Also add `#include <functional>`.

- [x] **Step 2: Implement rectangular segment generation**

In `convex_corridor.cpp`, implement path segmentation. Each segment starts as a local rectangle:

```text
s direction: segment tangent
l direction: left normal
constraints:
  +forward dot p <= +forward dot end
  -forward dot p <= -forward dot start
  +left dot p <= +left dot center + width
  -left dot p <= -left dot center + width
```

Then shrink width by sampling left/right until `is_traversable` fails.

Acceptance rule:

```text
segment.static_feasible = width >= cfg_.min_half_width
ignore width failure for s < cfg_.start_grace_length
```

- [x] **Step 3: Add tests**

Add a test where the reference path is straight and a fake traversability band is `abs(y) <= 0.35`:

```cpp
TEST(ConvexCorridor, BuildsStaticSegmentsInsideTraversableBand)
{
    ConvexCorridor cc;
    ConvexCorridor::Config cfg;
    cfg.segment_length = 1.0;
    cfg.half_width = 0.5;
    cfg.min_half_width = 0.25;
    cfg.static_sample_dl = 0.05;
    cc.setConfig(cfg);

    std::vector<Eigen::Vector2d> path = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(3.0, 0.0),
    };
    auto traversable = [](double, double y) { return std::abs(y) <= 0.35; };

    auto result = cc.buildStatic(path, traversable);

    ASSERT_TRUE(result.feasible);
    ASSERT_GE(result.segments.size(), 3u);
    EXPECT_GE(result.min_width, cfg.min_half_width);
    for (const auto &seg : result.segments)
        EXPECT_TRUE(ConvexCorridor::contains(seg, seg.center));
}
```

- [x] **Step 4: Verify and commit**

Run the same `catkin build` and `catkin test` commands from Task 1.

Commit:

```bash
git add path_searching/include/path_searching/convex_corridor.h \
        path_searching/src/convex_corridor.cpp \
        path_searching/test/test_convex_corridor.cpp
git commit -m "Generate static convex corridor segments"
```

---

### Task 3: Add Pedestrian Time-Aware Corridor Clipping

**Files:**
- Modify: `/home/xcg/ws/src/cane_planner/path_searching/include/path_searching/convex_corridor.h`
- Modify: `/home/xcg/ws/src/cane_planner/path_searching/src/convex_corridor.cpp`
- Test: `/home/xcg/ws/src/cane_planner/path_searching/test/test_convex_corridor.cpp`

- [x] **Step 1: Add prediction inputs**

Add:

```cpp
struct PedestrianPrediction
{
    Eigen::Vector2d p0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d v = Eigen::Vector2d::Zero();
    double radius = 0.35;
};

Result buildSpatioTemporal(const std::vector<Eigen::Vector2d> &reference_path,
                            const TraversableFn &is_traversable,
                            const std::vector<PedestrianPrediction> &pedestrians) const;
```

- [x] **Step 2: Implement conservative dynamic clipping**

For each segment with time interval `[t0, t1]`, predict pedestrian center at segment midpoint time:

```cpp
const double tm = 0.5 * (segment.t0 + segment.t1);
const Eigen::Vector2d ped = pred.p0 + pred.v * tm;
```

If the pedestrian overlaps the segment, add one separating half-space that keeps the corridor on the side opposite the pedestrian:

```cpp
Eigen::Vector2d n = (segment.center - ped);
if (n.norm() < 1e-6)
    n = Eigen::Vector2d::UnitY();
n.normalize();
Halfspace h;
h.normal = -n;
h.offset = -n.dot(ped + n * (pred.radius + cfg_.pedestrian_radius));
```

This is conservative and simple. It is not yet the final optimizer; it gives the planner a convex dynamic exclusion boundary to debug.

- [x] **Step 3: Add tests**

Add a test with one pedestrian centered above the path and verify the segment center remains inside but a point near the pedestrian violates the segment:

```cpp
TEST(ConvexCorridor, ClipsSegmentAwayFromPredictedPedestrian)
{
    ConvexCorridor cc;
    ConvexCorridor::Config cfg;
    cfg.segment_length = 1.0;
    cfg.half_width = 0.6;
    cfg.min_half_width = 0.2;
    cfg.pedestrian_radius = 0.3;
    cc.setConfig(cfg);

    std::vector<Eigen::Vector2d> path = {
        Eigen::Vector2d(0.0, 0.0),
        Eigen::Vector2d(2.0, 0.0),
    };
    auto traversable = [](double, double) { return true; };
    std::vector<ConvexCorridor::PedestrianPrediction> peds = {
        {Eigen::Vector2d(1.0, 0.35), Eigen::Vector2d::Zero(), 0.3},
    };

    auto result = cc.buildSpatioTemporal(path, traversable, peds);

    ASSERT_TRUE(result.feasible);
    ASSERT_FALSE(result.segments.empty());
    EXPECT_GT(result.dynamic_block_count, 0);
}
```

- [x] **Step 4: Verify and commit**

Run build/tests, then:

```bash
git add path_searching/include/path_searching/convex_corridor.h \
        path_searching/src/convex_corridor.cpp \
        path_searching/test/test_convex_corridor.cpp
git commit -m "Add pedestrian-aware convex corridor clipping"
```

---

### Task 4: Publish Convex Corridor in PlannerManager

**Files:**
- Modify: `/home/xcg/ws/src/cane_planner/plan_manage/include/plan_manager.h`
- Modify: `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`
- Modify: `/home/xcg/ws/src/cane_planner/plan_manage/launch/include/algorithm.launch`

- [x] **Step 1: Add launch flags**

Add:

```xml
<arg name="convex_corridor_enable" default="false"/>
<arg name="convex_corridor_segment_length" default="0.8"/>
<arg name="convex_corridor_half_width" default="0.45"/>
<arg name="convex_corridor_min_half_width" default="0.25"/>
```

Under `kin_replan_node`, add:

```xml
<param name="convex_corridor/enable" value="$(arg convex_corridor_enable)" type="bool"/>
<param name="convex_corridor/segment_length" value="$(arg convex_corridor_segment_length)" type="double"/>
<param name="convex_corridor/half_width" value="$(arg convex_corridor_half_width)" type="double"/>
<param name="convex_corridor/min_half_width" value="$(arg convex_corridor_min_half_width)" type="double"/>
```

- [x] **Step 2: Add PlannerManager ownership**

Add members:

```cpp
std::shared_ptr<cane_planner::ConvexCorridor> convex_corridor_;
ros::Publisher convex_corridor_pub_;
ros::Publisher convex_corridor_debug_pub_;
```

- [x] **Step 3: Initialize module**

In planner initialization:

```cpp
bool convex_enable = false;
nh.param("convex_corridor/enable", convex_enable, false);
if (convex_enable)
{
    convex_corridor_.reset(new cane_planner::ConvexCorridor);
    cane_planner::ConvexCorridor::Config cfg;
    nh.param("convex_corridor/segment_length", cfg.segment_length, 0.8);
    nh.param("convex_corridor/half_width", cfg.half_width, 0.45);
    nh.param("convex_corridor/min_half_width", cfg.min_half_width, 0.25);
    convex_corridor_->setConfig(cfg);
}
convex_corridor_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/mpc/convex_corridor", 10);
convex_corridor_debug_pub_ = nh.advertise<std_msgs::String>("/mpc/convex_corridor_debug", 10);
```

- [x] **Step 4: Publish markers**

Represent each segment as a line strip polygon and publish text:

```text
seg=<i> hs=<N> st=<0/1> dyn=<0/1> w=<min_width>
```

- [x] **Step 5: Manual verification**

Run:

```bash
roslaunch plan_manage sim_kin_replan.launch planner:=3 convex_corridor_enable:=true
```

Expected:

- `/mpc/convex_corridor` appears in `rostopic list`.
- RViz shows corridor segments tracking the A* local path.
- `/mpc/convex_corridor_debug` publishes segment counts and feasibility.

- [x] **Step 6: Commit**

```bash
git add plan_manage/include/plan_manager.h \
        plan_manage/src/planner_manager.cpp \
        plan_manage/launch/include/algorithm.launch
git commit -m "Publish convex corridor diagnostics"
```

---

### Task 5: Enforce Convex Corridor in MPPI

**Files:**
- Modify: `/home/xcg/ws/src/cane_planner/path_searching/include/path_searching/mpc_controller.h`
- Modify: `/home/xcg/ws/src/cane_planner/path_searching/src/mpc_controller.cpp`
- Modify: `/home/xcg/ws/src/cane_planner/plan_manage/src/planner_manager.cpp`
- Modify: `/home/xcg/ws/src/cane_planner/plan_manage/scripts/analyze_corridor_debug.py`

- [x] **Step 1: Add controller API**

Add to `MpcController`:

```cpp
void setConvexCorridor(const std::vector<ConvexCorridor::Segment> &segments);
void clearConvexCorridor();
```

Add config:

```cpp
bool convex_corridor_enable = false;
bool convex_corridor_hard_reject_enable = false;
double w_convex_corridor = 30.0;
```

- [x] **Step 2: Add rollout scoring**

For each rollout point, find the segment by predicted path progress or nearest segment center. Compute:

```cpp
const double v = ConvexCorridor::violation(segment, point);
cost += cfg_.w_convex_corridor * v * v;
if (cfg_.convex_corridor_hard_reject_enable && v > 0.05)
    convex_corridor_rejected = true;
```

Add debug counters:

```cpp
int convex_corridor_reject_count = 0;
double max_convex_corridor_violation = 0.0;
```

- [x] **Step 3: Pass corridor from PlannerManager**

After building a feasible convex corridor:

```cpp
mpc_controller_->setConvexCorridor(convex_result.segments);
```

If no feasible corridor exists:

```cpp
mpc_controller_->clearConvexCorridor();
```

- [x] **Step 4: Extend analyzer**

Add columns:

```text
convex_reject
convex_max_violation
convex_segments
```

Expected summary line:

```text
Convex corridor:
  segments mean=<...> min=<...> max=<...>
  reject mean=<...>
  max violation mean=<...>
```

- [x] **Step 5: Verify with soft enforcement**

Run:

```bash
roslaunch plan_manage sim_kin_replan.launch planner:=3 convex_corridor_enable:=true
rosrun plan_manage record_corridor_debug.py mixed_convex_soft_test --duration 70
rosrun plan_manage analyze_corridor_debug.py /home/xcg/ws/records/$(ls -t /home/xcg/ws/records | grep mixed_convex_soft_test | head -1)
```

Expected:

- No repeated `CORRIDOR_INFEASIBLE`.
- `convex_max_violation` remains bounded.
- MPPI valid ratio does not collapse to zero.

- [x] **Step 6: Commit**

```bash
git add path_searching/include/path_searching/mpc_controller.h \
        path_searching/src/mpc_controller.cpp \
        plan_manage/src/planner_manager.cpp \
        plan_manage/scripts/analyze_corridor_debug.py
git commit -m "Constrain MPPI with convex corridor cost"
```

---

### Task 6: Add Optional Corridor-Inside Trajectory Optimizer

**Files:**
- Create: `/home/xcg/ws/src/cane_planner/path_searching/include/path_searching/convex_corridor_optimizer.h`
- Create: `/home/xcg/ws/src/cane_planner/path_searching/src/convex_corridor_optimizer.cpp`
- Test: `/home/xcg/ws/src/cane_planner/path_searching/test/test_convex_corridor_optimizer.cpp`
- Modify: `/home/xcg/ws/src/cane_planner/path_searching/CMakeLists.txt`

- [x] **Step 1: Add optimizer API**

Use a small Eigen-based projected optimizer first, not a new dependency. The first version optimizes 2D waypoints inside halfspaces:

```cpp
class ConvexCorridorOptimizer
{
public:
    struct Config
    {
        int max_iter = 50;
        double step_size = 0.1;
        double smooth_weight = 1.0;
        double reference_weight = 1.0;
        double violation_weight = 100.0;
    };

    struct Result
    {
        std::vector<Eigen::Vector2d> path;
        bool success = false;
        double max_violation = 0.0;
        int iterations = 0;
    };

    Result optimize(const std::vector<Eigen::Vector2d> &reference,
                    const std::vector<ConvexCorridor::Segment> &segments) const;
};
```

- [x] **Step 2: Implement projection**

For each point and segment, project violated halfspaces:

```cpp
const double v = h.normal.dot(p) - h.offset;
if (v > 0.0)
    p -= v * h.normal / std::max(1e-6, h.normal.squaredNorm());
```

Optimize smoothness with Laplacian updates, then project back into constraints.

- [x] **Step 3: Add tests**

Test that a zig-zag reference path is smoothed and remains inside a rectangular corridor.

- [x] **Step 4: Verify and commit**

Run build/tests.

Commit:

```bash
git add path_searching/include/path_searching/convex_corridor_optimizer.h \
        path_searching/src/convex_corridor_optimizer.cpp \
        path_searching/test/test_convex_corridor_optimizer.cpp \
        path_searching/CMakeLists.txt
git commit -m "Optimize local path inside convex corridor"
```

---

### Task 7: Bag Regression and Method Documentation

**Files:**
- Modify: `/home/xcg/ws/src/cane_planner/docs/CONVEX_CORRIDOR_METHOD.md`
- Modify: `/home/xcg/ws/src/cane_planner/docs/MIXED_CORRIDOR_DEBUG_NOTES.md`

- [x] **Step 1: Run three soft-corridor bags**

```bash
rosrun plan_manage record_corridor_debug.py mixed_convex_soft_01 --duration 70
rosrun plan_manage record_corridor_debug.py mixed_convex_soft_02 --duration 70
rosrun plan_manage record_corridor_debug.py mixed_convex_soft_03 --duration 70
```

- [x] **Step 2: Analyze**

```bash
for label in mixed_convex_soft_01 mixed_convex_soft_02 mixed_convex_soft_03; do
  rosrun plan_manage analyze_corridor_debug.py /home/xcg/ws/records/$(ls -t /home/xcg/ws/records | grep "$label" | head -1)
  cat /home/xcg/ws/records/$(ls -t /home/xcg/ws/records | grep "$label" | head -1)/corridor_debug_summary.txt
done
```

Acceptance:

```text
rosout_stop_logs: 0 or only short justified interaction-yield stops
st=1 count: 0 or explained by true static blockage
MPPI valid ratio remains nonzero
convex corridor max violation remains bounded
```

- [x] **Step 3: Document method**

Write:

```text
A* guide path -> static convex segment generation -> pedestrian time clipping
-> MPPI convex corridor cost/hard reject -> optional projected path optimizer
```

Include limitations:

- First dynamic clipping is conservative.
- It is not yet a full QP with formal optimality proof.
- Hardware validation remains separate.

- [x] **Step 4: Final verification and commit**

Run:

```bash
catkin build path_searching plan_manage --no-deps -DCMAKE_BUILD_TYPE=Release
catkin test path_searching --no-deps --catkin-make-args run_tests
catkin_test_results build/path_searching
```

Commit:

```bash
git add docs/CONVEX_CORRIDOR_METHOD.md docs/MIXED_CORRIDOR_DEBUG_NOTES.md
git commit -m "Document convex corridor validation results"
```

---

## Execution recommendation

Do not implement all tasks in one large commit. The safest order is:

```text
Task 1 -> Task 2 -> bag/RViz sanity check -> Task 3 -> Task 4 -> Task 5
```

Only start Task 6 after Task 5 produces stable bag results. If Task 5 already gives good MPPI behavior, Task 6 can be presented as an optional method upgrade rather than a required fix.

## Self-review

- Spec coverage: The plan covers static convex corridor generation, pedestrian-aware clipping, MPPI enforcement, optional trajectory optimization, diagnostics, and regression documentation.
- Placeholder scan: No task depends on an undefined "later" implementation. Each task has files, concrete APIs, commands, and acceptance criteria.
- Type consistency: `ConvexCorridor::Segment`, `Halfspace`, `Config`, `Result`, and `PedestrianPrediction` are introduced before later tasks reference them.
