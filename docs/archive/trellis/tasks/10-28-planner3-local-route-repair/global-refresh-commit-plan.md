# Local commit proposal

1. `Replace planner 3 local repair with periodic global A*`

One coherent change: full global footprint A*, validated reference commit, launch
configuration, timing/preview corrections, tests/specs/docs and measured evidence.
The final default is .2s for both global and preview scheduling, using a fixed
phase and one per-transaction scheduling epoch. Existing safety clocks remain.

Files (including deletion of the previous local search):

- `.trellis/spec/planning-runtime/current-frame-pedestrian-corridor.md`
- `.trellis/spec/planning-runtime/directional-footprint-corridor.md`
- `.trellis/spec/planning-runtime/index.md`
- `.trellis/spec/planning-runtime/lfpc-mppi.md`
- `.trellis/spec/planning-runtime/lfpc-timed-collision.md`
- `.trellis/spec/planning-runtime/periodic-global-reference.md`
- `.trellis/spec/planning-runtime/time-triggered-local-repair.md`
- `.trellis/tasks/10-28-planner3-local-route-repair/global-refresh-commit-plan.md`
- `.trellis/tasks/10-28-planner3-local-route-repair/global-refresh-design.md`
- `.trellis/tasks/10-28-planner3-local-route-repair/global-refresh-implement.md`
- `.trellis/tasks/10-28-planner3-local-route-repair/global-refresh-prd.md`
- `.trellis/tasks/10-28-planner3-local-route-repair/refresh-cadence-design.md`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/astar-map-diagnosis/diagnosis.md`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/astar-map-diagnosis/failure-tl2s4Q.snapshot`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/astar-map-diagnosis/summary.json`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/global-refresh/astar-final.log`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/global-refresh/corridor-final.log`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/global-refresh/manager-final.log`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/global-refresh/mpc-final.log`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/global-refresh/office.json`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/global-refresh/pedestrian.json`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/global-refresh/pedestrian.yaml`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/global-refresh/red.log`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/global-refresh/summary.json`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/global-refresh/timed-final.log`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/refresh-cadence/final-office.json`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/refresh-cadence/final-pedestrian.json`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/refresh-cadence/manager-final.log`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/refresh-cadence/old-preview-500ms.json`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/refresh-cadence/old-scheduler-200ms.json`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/refresh-cadence/phase-red.log`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/refresh-cadence/preview-period-only-200ms.json`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/refresh-cadence/preview-phase-red.log`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/refresh-cadence/preview-rate-red.log`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/refresh-cadence/summary.json`
- `.trellis/tasks/10-28-planner3-local-route-repair/research/refresh-cadence/user-run-summary.json`
- `.trellis/tasks/10-28-planner3-local-route-repair/task.json`
- `README.md`
- `docs/PLANNER3_CORRIDOR_USAGE.md`
- `docs/PLANNER3_GLOBAL_REFRESH_VALIDATION.md`
- `docs/PLANNER3_LOCAL_REPAIR_VALIDATION.md`
- `path_searching/CMakeLists.txt`
- `path_searching/include/path_searching/astar.h`
- `path_searching/src/astar_global_reference.cpp`
- `path_searching/src/astar_local_recovery.cpp`
- `path_searching/test/test_astar_edges.cpp`
- `plan_manage/include/plan_manager.h`
- `plan_manage/launch/gazebo_localization_mpc.launch`
- `plan_manage/launch/gazebo_mpc.launch`
- `plan_manage/launch/include/algorithm.launch`
- `plan_manage/launch/kin_replan.launch`
- `plan_manage/launch/sim_kin_replan.launch`
- `plan_manage/src/planner_manager.cpp`
- `plan_manage/test/test_sim_odom_stop.cpp`

Unrecognized dirty files: none. All work belongs to this task, including the
preceding map diagnosis. Prior proposal remains pending; user reported slow
updates, which have now been corrected and verified. No new commit or push has
been executed.
