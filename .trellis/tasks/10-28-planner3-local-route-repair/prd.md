# Planner 3 time-triggered local route repair

## Goal

Let planner 3 continue through future-vacated space and find a checked local
detour when a real timed pedestrian conflict defeats its first MPPI attempt.

Approved by the user on 2026-10-28 after discussion of time-synchronized collision
checks, local A* detours, FIRI reconstruction and final MPPI/execution checks.
The initial approval covered planning and implementation. The user later
explicitly authorized the local commit and remote push.

## Requirements

- A pedestrian crossing the spatial reference must not alone prevent optimization.
- Use matching LFPC sample times and CV pedestrian predictions to decide whether
  dynamic avoidance is necessary. Always check selected/executed trajectories.
- When timed MPPI cannot find a feasible candidate, repair the affected reference
  section with local A*, retaining the unaffected route and exact final goal.
- Recovery must account for the complete configured human-cane envelope, including
  reference heading changes; point-only clearance is insufficient.
- Rebuild the active corridor and validate the resulting MPPI trajectory before
  committing/executing a repair. Failed/expired candidates cannot replace the route.
- Preserve selected static-map semantics, physical dimensions, accepted-observation
  locking, original source/receipt freshness of 0.5 s, and execution revalidation.
- Keep visualization honest about static space versus future dynamic safety.
- No speed/wait optimization, multi-pedestrian expansion, full SFM or time-expanded
  A* in this task. Do not disturb the user's ROS master/nodes.

## Acceptance

- A currently crossing pedestrian that clears before arrival allows planning and
  does not trigger A* merely because its current hull intersects the reference.
- A real matching-time conflict is rejected; blocked/no-path cases remain STOP.
- A reproducible stationary blocker with room around it produces a footprint-valid
  local detour, preserves its suffix, rebuilds FIRI and permits safe MPPI motion.
- Source expiration, queued replacements, malformed data, root/static blockage and
  execution latency regressions remain covered.
- Build with catkin build; run affected unit/manager tests and an isolated-master
  office-scene replay. Report actual bypass separately from eventual arrival.
