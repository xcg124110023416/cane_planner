# mpc_prototype — MPC + LFPC + LIPM 导盲杖动态避障

## 项目目标

用 MPC (MPPI) 替换原有 C++ 的 A* 搜索，实现欠驱动导盲杖的动态障碍物避障。系统只控制航向角 θ，人提供前进速度（欠驱动）。Python 原型已验证通过，下一步 C++ 移植到 ROS1 Noetic。

## 核心约束

- 设备只控制 steering heading θ，通过全向轮实现
- 人提供 forward velocity（系统不可控、不可刹停）
- 因此是无刹车能力的欠驱动系统

## 项目结构

```
mpc_prototype/
├── CLAUDE.md                         # 本文件，项目启动文档
├── path_searching/                   # 原始 C++ ROS 包（A* + LFPC + LIPM）
│   ├── src/lfpc.cpp                  # LFPC + LIPM 动力学（C++ 参考实现）
│   ├── src/kinodynamic_astar.cpp     # A* 搜索 + 动态风险场（待替换）
│   ├── src/dynamic_risk_field.cpp    # 各向异性高斯风险场
│   └── include/                      # 头文件
├── mpc_cane/                         # Python MPC 原型（当前验证平台）
│   ├── config.py                     # 所有参数（LFPCConfig, MPCConfig, RiskFieldConfig）
│   ├── lfpc.py                       # LFPC + LIPM 动力学（精确平移 lfpc.cpp）
│   ├── dynamic_risk_field.py         # 各向异性高斯风险场
│   ├── mpc_controller.py             # MPPI MPC 控制器（替换 kinodynamic_astar.cpp）
│   ├── simulation.py                 # 仿真环境（步进、障碍物、指标）
│   └── main.py                       # 入口：3 场景 + 对比 + 可视化
└── requirements.txt                  # numpy, scipy, matplotlib
```

## MPC 架构（三要素）

### 系统模型
- **LFPC + LIPM**：控制输入 [al, aw, api] → 落脚点计算 → LIPM 闭式解推演 CoM 轨迹
- 转移函数：`x(t) = x₀·cosh(t/tc) + tc·v₀·sinh(t/tc)`
- 步内 5 个子步（0.07s × 5 = 0.35s 支撑相）

### 约束
- 控制边界：al ∈ [0.05, 0.4], aw ∈ [0, 0.15], api ∈ [-0.5236, 0.5236]
- 硬安全约束：risk > 8.5 → cost = INF（丢弃轨迹）
- 欠驱动约束：vx, vy 来自观测（里程计），非 MPC 控制

### 代价函数
```
总代价 = w_move × ||Δpos|| + w_steer × |api| + w_risk × Σ risk(q, obs) + w_goal × ||终点-目标||
```
权重：w_move=1.0, w_steer=0.5, w_risk=2.0, w_goal=10.0

### 优化方法：MPPI
- K=100 个采样序列，N=6 步 horizon
- 无梯度（LFPC 不可微），加权平均 + 暖启动 + 滚动时域

## 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| t_sup | 0.35s | 支撑相时间 |
| delta_t | 0.07s | 离散步长 |
| h | 1.0m | 腿长/质心高度 |
| tc | 0.316 | LIPM 时间常数 √(h/g) |
| b | ≈0.3 | 捕获点稳定系数 |
| tau (风险场) | 0.5s | 障碍物预测前推时间 |
| sigma_x | 1.2m | 沿速度方向风险宽度 |
| sigma_y | 0.5m | 侧向风险宽度 |
| cutoff_dist | 3.0m | 风险场截止距离 |

## 已验证场景

| 场景 | 结果 | 关键指标 |
|------|------|---------|
| 1. 无障碍直行 | ✅ | 15 步到 5m, 24ms avg |
| 2. 交叉障碍物 | ✅ | MPC 主动减速+绕行, 最小距离 1.81m |
| 3. 迎面行人 | ✅ | 唯一存活控制器 (Greedy/Reactive 均碰撞) |

## 已确认的 Bug 修复

1. **障碍物传播 bug**：MPC rollout 中每步都用同一障碍物位置 → 修复为 `obs_pos + n*dt*vel`
2. **风险场位置 bug**：`_add_risk_heatmap` 用障碍物最终位置 → 修复为轨迹中最近遭遇点

## 后续：C++ 移植到 ROS1 Noetic

### 目标工作空间：cane_planner/

```
cane_planner/
├── plan_env/          # 环境感知（SDFMap, collision_detection, obj_predictor）
├── bspline/           # B样条基础
├── bspline_opt/       # B样条轨迹优化（后端）
├── path_searching/    # ★ 路径搜索（A*, KinodynamicAstar, LFPC → 加 MPC）
├── plan_ctrl/         # L1 前瞻控制器 + GKF 串口（执行层）
├── plan_manage/       # ★ 任务调度（FSM, 入口, algorithm.launch 参数）
├── omniGKF_ctrl/      # 全向轮驱动
├── Utils/             # 工具包（map_generator, traj_analyze）
└── LFPC/              # Python LIPM 原型（参考）
```

### 依赖链

```
plan_env (地图+碰撞+障碍物预测)
    ↓
path_searching (前端规划) ← 这里用 MPC 替换 KinodynamicAstar
    ↓
bspline → bspline_opt (后端轨迹优化，可能保留作为平滑层)
    ↓
plan_ctrl (L1 前瞻控制器 → GKF 电机)
    ↓
plan_manage (FSM 统一调度，algorithm.launch 加载所有参数)
```

### MPC 集成清单（需改动 4 个位置）

**① 新增文件** (2 个):
- `path_searching/include/path_searching/mpc_controller.h`
- `path_searching/src/mpc_controller.cpp`

**② 修改文件** (4 个):
- `path_searching/CMakeLists.txt` — 加 `mpc_controller.cpp` 编译
- `path_searching/include/path_searching/lfpc.h` — 加 `copyState()`
- `plan_manage/src/planner_manager.cpp` — 替换 `KinodynamicAstar::search` 为 `MpcController::plan`
- `plan_manage/launch/include/algorithm.launch` — 加 MPC 参数 (`mpc/K, mpc/N, mpc/w_risk, ...`)

**③ 现有可用资产**（不需改动）:
- `plan_env/include/plan_env/obj_predictor.h` — 已有 `linear_obj_model` 提取 `[x,y,vx,vy]`
- `plan_env/include/plan_env/collision_detection.h` — 2D 可通行性检测
- `plan_manage/launch/dynamic_detect.launch` — 已有动态障碍物检测

### 关键集成细节

**障碍物数据流**：
```
obj_predictor (ROS topic) → PlannerManager::dynamicObstacles
    → mpc_controller->setDynamicObstacles(pos, vel, size)
    → MPPI rollout 中逐步推进: obs_pos + n*dt*vel
```

**状态数据流**：
```
FAST-LIO (里程计) → PlannerManager::odom
    → LFPC::reset(odom_vx, odom_vy, theta)
    → MpcController::plan(lfpc_state, goal, obstacles)
```

**FSM 切入时机**（在 PlannerManager 的 GEN_NEW_TRAJ 状态）：
```cpp
// 原: kinodynamic_astar_->search(start_pos, start_state, end_pos);
// 改:
auto [al, aw, api] = mpc_->plan(lfpc_model_, goal, dyn_obstacles);
lfpc_model_->setCtrlParams(al, aw, api);
lfpc_model_->updateOneStep();
// 发布步态控制 [al, aw, api] 给 plan_ctrl → GKF 电机
```
- ROS param server + catkin 构建系统

### 需要新写
- `mpc_controller.h/cpp` — MPPI 类（~400 行 C++）
- `LFPC::copyState()` — 轻量状态复制（替代 Python clone）
- 替换 `KinodynamicAstar::search` 调用点

### 预估性能：<5ms/plan (C++ vs Python 16ms)

## 真实系统部署需考虑

1. **状态估计**：激光雷达里程计 → EKF 估计 CoM 状态；支撑腿用 vy 符号过零检测
2. **人速观测**：里程计测 vx/vy 注入 MPC rollout step 0（非控制输入，是观测值）
3. **鲁棒性**：速度窗口替代点估计、缩短 horizon、discount 远期代价
4. **安全层**：独立超声/雷达检测 → 直接 override；死锁 → 语音提示
5. **时序同步**：多传感器时间戳对齐 + 外推

## 参考论文

- Na et al. 2024 IEEE TNSRE: Smart Cane WPG (LIPM + LFPC + A*), 静态环境
- Yuan, Na et al. (NUST): 全驱导盲机器人 NMPC (unicycle, full-drive, 轨迹跟踪)

## 沟通偏好

用简单易懂的方式解释改动——先说整体思路（什么时机做什么事），再用直观比喻。不要上来就贴代码或罗列技术细节。
