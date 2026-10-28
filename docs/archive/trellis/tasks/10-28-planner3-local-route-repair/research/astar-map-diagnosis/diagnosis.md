# A* 路径进入后来更新的静态障碍：2026-10-28

## 结论

最新一次 14:22 的运行存在两个衔接问题：初次全局 A* 使用局部 SDF，远处
尚未融合的区域按乐观规则视为自由；机器人前进后融合出真实障碍，但旧的
全局参考未重规划。静态走廊正确拒绝了这条参考，随后 STOP 分支又每次清零
卡住计数，使已有的 30 次卡住重规划条件无法积累。

该问题关闭行人也能复现，与行人 CV 外推无关。本轮未修改运行代码。

## 最新记录与离线复现

原日志：ROS run `b660ca22-d281-11f1-a81c-f92175a34ec4` 中
`roslaunch-Lenovo-xcg-42755.log` / `rosout.log`。
起点约 `(-5.695, 4.479)`，终点 `(7.558, 2.972)`。

- 初次 `[Astar sdf]` 报终点可通行、距离 5 m，随后生成 134 个参考点。
- 后续静态走廊在约 `(4.475, 3.579)` 报 `REFERENCE_OCCUPIED`，
  全程保持 `INVALID_STATIC_CORRIDOR`。
- 实际位置 `(4.475, 3.579)` 对应栅格本身仍自由；FIRI 拒绝的是前向保护
  种子/完整外形无法与障碍分离，不能把 failure_position 当作占用格中心。
- 保存快照的原参考中心线与占用格有 17 对相交，首次位于
  x=`5.7…5.8`、y=`3.5…3.6` 的格子。中心线也确实进入了障碍。

原始静态快照及校验值已保留，运行生产回放程序即可重现相同位置/原因：

```bash
rosrun path_searching convex_corridor_replay \
  .trellis/tasks/10-28-planner3-local-route-repair/research/astar-map-diagnosis/failure-tl2s4Q.snapshot
```

`recorded` 与 `replayed` 均为 `feasible=0 reason=REFERENCE_OCCUPIED`。
将 `replayed feasible=1` 作为通过断言时，该回放已实际运行并得到 RED。

## 隔离验证

使用独占 master 11339、单独 ROS_HOME、原生产节点及办公室 PCD，去除
RViz/硬件控制节点，并关闭行人。未启停用户的 11311 节点，也未写行人场景。
诊断程序结束后，已清理自己启动的 launch/master。

- 同一起终点仍生成 134 点 A*，全过程只发布一次路线。
- A* 发布时，上述目标格中膨胀占用体素数为零。
- 随机器人移动，该格出现 8 个占用体素；紧接着 FIRI 报
  `REFERENCE_OCCUPIED`。此时质心约 `(1.544, 3.906)`。
- 记录到 19 个 OK 和 49 个 `INVALID_STATIC_CORRIDOR` 决策，未触发重搜。
- 原 PCD 在附近存在点 `(5.9225, 3.5949, 0.2)`；距初始地图中心约
  11.65 m，距停下时中心约 4.40 m，符合 5 m 融合范围。
  原始点不必落在膨胀格内，不应把原始点计数与膨胀体素计数混淆。

异步地图更新使隔离运行比用户运行更早停车，但初始参考和触发机制一致。
持久数值证据见 `summary.json`。

## 代码与参数证据

- `use_static_global_map=false`、`use_static_global_esdf=false`；
  `astar.cpp` 因此选择局部 SDF 后端，搜索范围仍为整个地图。
- `max_ray_length=5.0`、`local_update_range_x/y=5.5`、`optimistic=true`。
  `SDFMap::inputPointCloud` 截断超出射线范围的点，远处障碍不直接融合为
  占用；距离场在乐观模式下以膨胀占用作为零距离源，未知空间没有硬排除。
- `CollisionDetection::getStaticCorridorGrid` 与 A* 使用同一选定后端的
  可通行分类。这次的关键差异是地图更新时刻；完整人杖条件还会提前拒绝
  接近障碍的参考，但不是中心线穿障的根本原因。
- `mpcSimStep` 只在基础走廊可行、MPPI 失败且有时间碰撞记录时局部修路。
  静态 `REFERENCE_OCCUPIED` 不走这条分支。
- 同一函数的卡住检测阈值为 30，但 STOP 分支将 `mpc_stuck_steps_` 置零，
  连续静态失败不能触发这个重规划条件。

## 修正方向

对于已有完整 PCD 的仿真，应使全局 A* 使用完整静态地图，并核对其高度、
膨胀和已知区域规则与走廊的静态判据一致。另需独立处理“静态参考因新地图
失效”：根据新地图重搜并验证候选，通过后替换参考；失败时仍停止执行。
这两点不同于单纯调高 A* 更新频率，或只处理行人触发的局部修路。
