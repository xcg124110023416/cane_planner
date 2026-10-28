# 本地提交计划

1. `Add time-triggered local route repair for planner 3`

   本次 planner 3 运行改动、几何与 manager 回归、使用说明、运行规范和验证记录。

   文件清单：

   - `.trellis/spec/planning-runtime/current-frame-pedestrian-corridor.md`
   - `.trellis/spec/planning-runtime/directional-footprint-corridor.md`
   - `.trellis/spec/planning-runtime/index.md`
   - `.trellis/spec/planning-runtime/lfpc-mppi.md`
   - `.trellis/spec/planning-runtime/lfpc-timed-collision.md`
   - `README.md`
   - `docs/PLANNER3_CORRIDOR_USAGE.md`
   - `path_searching/CMakeLists.txt`
   - `path_searching/include/path_searching/astar.h`
   - `path_searching/include/path_searching/mpc_controller.h`
   - `path_searching/src/mpc_controller.cpp`
   - `path_searching/test/test_astar_edges.cpp`
   - `path_searching/test/test_static_mpc.cpp`
   - `plan_manage/include/plan_manager.h`
   - `plan_manage/src/planner_manager.cpp`
   - `plan_manage/test/test_sim_odom_stop.cpp`
   - `.trellis/spec/planning-runtime/time-triggered-local-repair.md`
   - `.trellis/tasks/10-28-planner3-local-route-repair/check.jsonl`
   - `.trellis/tasks/10-28-planner3-local-route-repair/design.md`
   - `.trellis/tasks/10-28-planner3-local-route-repair/implement.jsonl`
   - `.trellis/tasks/10-28-planner3-local-route-repair/implement.md`
   - `.trellis/tasks/10-28-planner3-local-route-repair/prd.md`
   - `.trellis/tasks/10-28-planner3-local-route-repair/research/commit-plan.md`
   - `.trellis/tasks/10-28-planner3-local-route-repair/research/office-replay-summary.json`
   - `.trellis/tasks/10-28-planner3-local-route-repair/task.json`
   - `docs/PLANNER3_LOCAL_REPAIR_VALIDATION.md`
   - `path_searching/src/astar_local_recovery.cpp`

未识别的修改：无。全部为本任务实现与验证产物。

用户已明确确认提交本地并上传云端。按上述单次提交执行，再推送 origin/mpc-lfpc-prototype；不修改历史提交。
