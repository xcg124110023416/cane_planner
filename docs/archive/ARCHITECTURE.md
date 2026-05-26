# MPC 导盲杖 — 全局+局部规划架构

## 当前状态
伪仿真：PCD 地图全量加载，起终点手动指定，MPC 直接看全局地图。

## 推荐架构

```
全局地图 (已知/预建)               局部感知 (LiDAR/深度相机)
      │                                    │
  A* 全局规划 (~1Hz)                  局部代价地图 (costmap)
      │                                    │
  生成 waypoints ──────────→ MPC 局部规划 (~10Hz)
                                     │
                                  [al, aw, api] → L1控制器 → 电机
```

- **A***：在全局地图上跑，发布间隔 ~1m 的路径点
- **MPC**：在局部代价地图里滚动优化，逐个 waypoint 推进
- MPC horizon 只需覆盖下一个 waypoint，不用看整个地图
- 局部感知到的动态/新障碍物在局部代价图里体现
- A* 保证全局可通行性，MPC 处理局部避障和平滑

## 分步实施

### 第一步（优先）：现有仿真加全局路径层
- PCD 全量加载作为「全局地图」
- 加 A* 全局规划节点，发布 /global_path (waypoints)
- MPC 改为追踪当前 waypoint，到达后切下一个
- 限制 MPC 可感知范围（模拟传感器 FOV，取机器人周围 5m 的 ESDF）
- 改动量小，快速验证全局+局部协作逻辑

### 第二步：Gazebo + SLAM
- Gazebo 搭带障碍物场景，导盲杖 URDF + 传感器插件
- gmapping 或 cartographer 做 SLAM 建图
- 全局规划用 SLAM 地图，MPC 用局部 costmap
- 此时传感器噪声、里程计漂移、实时性等真实问题才会暴露

## 依赖链

```
plan_env (地图+碰撞+障碍物预测)
    ↓
path_searching (A* 全局 + MPC 局部)
    ↓
bspline → bspline_opt (后端轨迹优化，可选平滑层)
    ↓
plan_ctrl (L1 前瞻控制器 → GKF 电机)
    ↓
plan_manage (FSM 统一调度)
```
