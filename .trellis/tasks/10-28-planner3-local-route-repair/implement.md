# Implementation and validation

- [x] Add a manager regression for a crossing that clears before arrival; run it
      RED against production orchestration before changing runtime behavior.
- [x] Implement footprint-aware, bounded local A* detours and geometric regressions
      for suffix retention, body/cane clearance, heading changes and closed paths.
- [x] Separate static FIRI from timed pedestrian validation; integrate one optional
      local repair, FIRI rebuild and timed MPPI retry with transactional commit.
- [x] Update affected manager tests and retain observation/execution regressions.
- [x] Synchronize runtime contracts, README and corridor visualization descriptions.
- [x] Build production targets, run affected suites and replay the original office
      scene on an owned private master; record safety/arrival/recovery outcomes.
- [x] Inspect final diff, remove temporary instrumentation and record limitations.

Use existing catkin workspace /home/xcg/ws and isolated ROS_MASTER_URI on 11339.
Never restart or stop user-owned processes on 11311. Missing Trellis helper skills
were searched under .agents/.codex and global skills; follow workflow manually.

Validation: [persistent results](../../../docs/PLANNER3_LOCAL_REPAIR_VALIDATION.md).
All 106 enabled tests pass. Production builds pass without compiler warnings.
Final office replay arrived after six NO_PATH/STOP cycles; persistent stationary
blocker tests separately verify direct MPPI and accepted local-repair bypass.
Implementation is reviewable; the user explicitly approved the local commit and
remote push after reviewing the result and discussing its scope.
