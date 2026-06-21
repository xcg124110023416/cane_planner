# Trajectory-Guided Dynamic Walking Corridor Design

## Goal

Build the planner around a closed-loop structure:

```text
previous human-feasible trajectory
  -> time-indexed dynamic walking corridor
  -> constrained LIPM-LFPC-MPPI
  -> new human-feasible trajectory
```

The goal is to remove the current split where DWC follows an A* geometric path while MPPI searches human-motion trajectories. A* remains as global guidance and bootstrap only; the local safety corridor should be generated from the previous MPPI best trajectory whenever such a trajectory exists.

## Current Baseline

Current local branch:

```text
mpc-lfpc-prototype
70986ef Add corridor regression runner
ba0fa5a Summarize corridor stop gate diagnostics
```

Current runtime baseline:

```text
A* global path
  -> DWC along global/path reference
  -> MPPI rollout using LIPM+LFPC
  -> stop gate
```

Important existing code:

- `path_searching/include/path_searching/dynamic_walking_corridor.h`
- `path_searching/src/dynamic_walking_corridor.cpp`
- `path_searching/include/path_searching/mpc_controller.h`
- `path_searching/src/mpc_controller.cpp`
- `plan_manage/include/plan_manager.h`
- `plan_manage/src/planner_manager.cpp`
- `plan_manage/scripts/record_corridor_debug.py`
- `plan_manage/scripts/analyze_corridor_debug.py`
- `plan_manage/scripts/run_corridor_regression.py`

## Research Claim

The method should be described as:

> A trajectory-guided dynamic walking corridor replanning method. The previous LIPM-LFPC-MPPI human-feasible trajectory is used as the nominal trajectory. A time-indexed local walking corridor is generated around this nominal trajectory using static obstacles and predicted pedestrian positions. The next MPPI cycle samples human-feasible trajectories constrained by this corridor. The system repeats this trajectory-corridor-trajectory loop in receding horizon.

This makes DWC and MPPI reference the same object: a human-executable trajectory.

## Architecture

### Frame 0

At the first planning frame, no previous MPPI trajectory exists. The system uses A* only to create an initial nominal trajectory.

```text
A* path -> initial nominal trajectory -> initial DWC -> MPPI best trajectory
```

### Frame k

At each later frame:

```text
previous MPPI best trajectory
  -> prune already executed prefix
  -> nominal trajectory
  -> time-indexed DWC
  -> MPPI constrained rollout
  -> new MPPI best trajectory
```

If the previous MPPI trajectory is stale, too short, or invalid, the system may fall back to the A* local reference only as a bootstrap source, not as the normal DWC backbone.

## Data Model

### TimedTrajectory

Represents a human-feasible local trajectory.

Each point contains:

```text
x
y
yaw
t_from_now
```

The trajectory source must be explicit:

```text
ASTAR_BOOTSTRAP
PREVIOUS_MPPI
FALLBACK_EMPTY
```

### TimedWalkingCorridor

Represents corridor segments aligned with the nominal trajectory.

Each segment contains:

```text
start_time
end_time
centerline or segment center
forward direction
half_width
length
static feasibility status
dynamic feasibility status
```

The first version may keep the existing rectangular/polyline DWC geometry, but it must be indexed by trajectory time.

### Feasibility

MPPI should expose feasibility in terms of sampled human trajectories:

```text
valid_sample_ratio
inside_timed_corridor_ratio
best_trajectory_valid
```

STOP should mean:

```text
no human-feasible trajectory exists under the current time-indexed corridor
```

Debug stop reasons may remain for analysis, but they are not the research method.

## DWC Generation Rule

Input:

```text
nominal TimedTrajectory tau*
static obstacle query
dynamic pedestrian predictions
```

For each nominal segment:

```text
xi -> xi+1
```

Generate a local corridor segment around that trajectory segment. Static obstacles define spatial shrink/truncation. Dynamic pedestrians are evaluated at the segment time interval, not only at the current time.

## MPPI Usage Rule

For each sampled MPPI trajectory:

```text
tau = {x0, x1, ..., xN}
```

Each rollout point is matched to the corridor segment at the same prediction time. A rollout point outside the segment corridor is penalized or rejected depending on configuration.

The first implementation should use a soft penalty plus debug counters. Hard rejection can be enabled only after the timed corridor behavior is verified.

## Safety and Stop Semantics

The main method should not be a pile of rules. The clean semantics are:

```text
feasible trajectory exists -> GO
no feasible trajectory exists -> STOP
```

Engineering stop reasons are allowed for diagnosis:

```text
NO_TIMED_CORRIDOR
NO_VALID_MPPI_TRAJECTORY
STATIC_CORRIDOR_BLOCKED
DYNAMIC_CORRIDOR_BLOCKED
```

But the paper-level description should focus on feasibility under a time-indexed corridor constraint.

## Verification Strategy

Each stage must be independently verifiable from bag data.

Required metrics:

```text
nominal_source
nominal_points
nominal_length
timed_corridor_segments
timed_corridor_time_span
candidate_inside_timed_corridor_ratio
valid_sample_ratio
best_path_length
stop_reason
```

Mixed-scene validation is valid only if the A* path and nominal trajectory actually pass through the intended pedestrian interaction region.

## Scope Guard

Do not continue from the older optimizer/fallback/prefix branch as the research mainline. That branch can remain as reference material, but this design should be implemented as a cleaner trajectory-guided DWC loop.

Do not add new stop gates unless they directly report feasibility of the timed-corridor-constrained MPPI problem.
