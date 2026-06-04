# Stage 2 Design: Scene-conditioned Social-cost DRF-MPPI

## 1. Goal

Stage 2 的目标是构建 E 组方法：

```text
E = DRF-MPPI + lightweight interaction decision layer + scene/mode-conditioned social cost
```

当前方法不再表述为“规则层直接决定动作”，也不表述为“单纯增加一个 J_social”。更合理的定位是：

> 交互决策层负责判断当前横穿交互语义并保持模式稳定；该模式不直接生成控制命令，而是调制 MPPI 看到的社会代价场和局部规划偏置。

换句话说：

```text
DRF / J_social: 表达哪里风险高、哪里社会空间不合适
interaction decision: 决定当前采用哪种交互语义
hysteresis / memory: 防止反复改变交互模式
MPPI: 在动力学约束下求可执行局部运动
```

## 2. Scope

Stage 2 只关注单行人 crossing 场景。

当前不扩展到一般人群导航，也不把迎面会车、静止行人、多行人避让作为主实验矩阵。迎面会车中的 passing-side memory 可以作为未来扩展，但不进入当前最小方法。

当前最小 mode 集合保持为：

```text
mode ∈ {CONTINUE, YIELD, PASS_BEHIND}
```

不引入 `PASS_AHEAD` 作为显式主策略，也不引入 `PASS_SIDE` 作为当前 Stage 2 mode。

## 3. Method Overview

整体链路为：

```text
pedestrian observation and prediction
    -> scene recognition
    -> risk assessment + interaction mode state machine
    -> temporal consistency / hysteresis
    -> joint scene-mode objective modulation
    -> DRF-MPPI optimization
    -> steering guidance + optional yield prompt
```

`scene` 和 `mode` 是两个不同层次：

```text
scene: 当前属于哪类行人交互场景
mode: 在该场景下采用哪种通行/让行语义
```

当前论文和实现聚焦 crossing，但方法概念上可以写成：

```text
scene ∈ {none, static, oncoming, crossing}
mode  ∈ {CONTINUE, YIELD, PASS_BEHIND}
```

二者联合决定 MPC 目标函数参数：

```text
(scene, mode) -> MPC objective
```

示例：

```text
scene = static,   mode = CONTINUE:
  正常绕行静止行人，保持舒适距离。

scene = oncoming, mode = CONTINUE:
  继续通行，但通过 J_social 和 side preference 提前稳定选边。
  该场景不进入当前 Stage 2 主实验。

scene = crossing, mode = YIELD:
  横穿冲突风险高，提示用户减速/等待。

scene = crossing, mode = PASS_BEHIND:
  行人优先通过后，引导用户从行人后方通过。

scene = crossing, mode = CONTINUE:
  虽然存在横穿行人，但时间间隙足够大或冲突不显著，保持原有引导。
```

核心思想：

```text
scene / mode 不直接输出 turn left / turn right / stop
scene / mode 联合输出 MPPI cost landscape 的参数和局部偏置
```

因此，最终控制仍然由 MPPI 在 LFPC/LIPM 约束下优化得到。

## 4. Cost Structure

Stage 2 的代价可以概括为：

```text
J(scene, mode) = J_goal
               + J_static_obstacle
               + J_pedestrian_collision
               + J_social(scene, mode, memory)
               + J_lateral_control
               + J_motion_smooth
               + J_guidance_smooth
```

其中 `scene` 决定社会空间和交互风险的基本形状，`mode` 决定当前通行语义下需要强化的行为偏好。例如，`crossing + PASS_BEHIND` 会增强 front-passing penalty 和 behind corridor preference；`crossing + YIELD` 会增强 conflict/front-zone penalty 并输出 yield prompt。

这里需要符合导盲杖系统约束：系统主要控制横向引导量、转向角或轮子偏转控制，不完整控制用户前向速度。预测状态可以抽象为：

```text
x_k: 沿前进方向的位置
y_k: 横向偏移
theta_k: 行走方向角
u_k: 横向引导量 / 转向角 / 轮子偏转控制
```

因此，涉及“等待/减速”的项不能简单解释为 MPC 直接优化前向速度，而应解释为 yield prompt、降低抢行倾向和保持导盲杖引导稳定。

## 5. Common Costs

以下代价在所有 mode 中都需要，只是权重可能随 mode 变化。

### J_goal

目标跟踪代价，使用户整体仍朝原路径或局部目标前进，避免为了避让行人偏离过大：

```text
J_goal = Σ ||p_k - p_ref,k||²_Q
```

其中 `p_ref` 可以是全局路径、局部中心线或期望前进方向。

### J_static_obstacle

静态障碍物代价，用于避开墙、桌椅、柱子等环境障碍物。若已有 ESDF，可写成：

```text
J_static_obstacle = Σ φ(d_obs(p_k))
φ(d) = max(0, d_static_safe - d)^2
```

### J_pedestrian_collision

行人碰撞安全代价。它不同于社会空间代价：

```text
J_pedestrian_collision: 硬安全 / 最小安全距离
J_social: 舒适距离 / 社会合规 / 交互偏好
```

可写成：

```text
J_pedestrian_collision = Σ max(0, d_ped_safe - ||p_k - p_h,k||)^2
```

其中 `p_h,k` 是行人预测位置。

### J_social_basic

基础社会空间代价，即使没有碰撞风险，也尽量避免贴近行人。可以使用沿行人运动方向的各向异性高斯：

```text
J_social_basic = Σ exp(-0.5 * [
    (d_parallel / sigma_parallel)^2
  + (d_lateral  / sigma_lateral)^2
])
```

对于移动行人，行人前方 `sigma` 可以更大，后方较小，侧向中等。

### J_lateral_control

横向控制幅度代价，限制导盲杖给出过猛的左右引导：

```text
J_lateral_control = Σ ||u_k||²
```

### J_motion_smooth

运动平滑代价，减少急转或预测轨迹突变：

```text
J_motion_smooth = Σ ||u_k - u_{k-1}||²
```

也可以用轨迹二阶差分表示：

```text
J_motion_smooth = Σ ||p_{k+1} - 2p_k + p_{k-1}||²
```

### J_guidance_smooth

引导提示平滑代价，更偏人机交互层面。如果导盲杖输出左右引导：

```text
g_k ∈ [-1, 1]
-1: left guidance
 0: neutral / no strong guidance
 1: right guidance
```

则：

```text
J_guidance_smooth = Σ ||g_k - g_{k-1}||²
```

可选加入方向切换惩罚：

```text
J_guidance_switch = I(sign(g_k) != sign(g_{k-1}))
```

它的作用是防止导盲杖左一下右一下，让用户困惑。

## 6. Mode-conditioned Costs

模式代价只在 `YIELD` 或 `PASS_BEHIND` 中强化特定行为。

### J_front_pass_high

这是 crossing 场景中的关键导盲约束：不引导盲人用户从横穿行人前方抢行。

定义行人运动方向前方的 front-passing zone。若用户预测轨迹进入该区域，则给出高代价：

```text
J_front_pass_high = Σ W_front * I(p_k in front_zone_h,k)
```

也可以用连续形式：

```text
J_front_pass_high = Σ W_front * exp(-distance_to_front_zone² / sigma_front²)
```

当前阶段建议优先用高代价，而不是硬约束，避免 MPPI 在误检或局部狭窄情况下无可行解。

### J_wait_or_slow

该名字容易误解。由于导盲杖不可靠控制用户前向速度，它不应被解释为直接优化速度的代价。

在 `YIELD` 模式中，它可以有两种实现：

```text
1. 横向引导稳定：J_wait_or_slow = Σ ||u_k||²
2. 提示策略：risk high -> output wait/yield cue; risk low -> exit YIELD
```

直观含义：当前不适合绕行或抢行，系统应提示用户减速/等待，并避免给出剧烈横向引导。

### J_behind_corridor

`PASS_BEHIND` 模式的核心代价，鼓励轨迹从行人后方安全走廊通过，而不是仅仅避开行人身体。

令：

```text
e_h = normalized(pedestrian_velocity)
n_h = rotate90(e_h)
s_k = (p_k - p_h,k) dot e_h
l_k = (p_k - p_h,k) dot n_h
```

其中 `s_k < 0` 表示用户预测点位于行人后方。后方走廊可定义为：

```text
s_k ≈ -d_behind
|l_k| within passable corridor
```

对应代价：

```text
J_behind_corridor = Σ [
    (s_k + d_behind)^2
  + penalty(|l_k| too large)
]
```

该项应与 `J_front_pass_high` 同时使用，使轨迹倾向于从行人后方通过，而不是从前方抢行。

### Mode Summary

```text
CONTINUE:
  J_goal
  + J_static_obstacle
  + J_pedestrian_collision
  + J_social_basic
  + J_lateral_control
  + J_motion_smooth
  + J_guidance_smooth

YIELD:
  J_goal
  + J_static_obstacle
  + J_pedestrian_collision
  + J_front_pass_high
  + J_wait_or_slow
  + J_guidance_smooth

PASS_BEHIND:
  J_goal
  + J_static_obstacle
  + J_pedestrian_collision
  + J_front_pass_high
  + J_behind_corridor
  + J_lateral_control
  + J_motion_smooth
  + J_guidance_smooth
```

最关键的三项是：

```text
J_front_pass_high: 不从行人前方抢行
J_behind_corridor: 引导从行人后方通过
J_guidance_smooth: 保证导盲提示稳定
```

这些 scene/mode-conditioned costs 是对 MPPI cost landscape 的调制，不是直接控制命令。

## 7. Joint Scene-mode Objective Mapping

`scene` 和 `mode` 联合设置 MPC objective。当前 Stage 2 主实现只需要完整支持 crossing；static/oncoming 可以作为概念扩展或后续工作。

| scene | mode | MPC objective modulation |
|---|---|---|
| none | CONTINUE | baseline goal/static/pedestrian risk/smoothness |
| static | CONTINUE | stronger personal-space cost around stationary pedestrian |
| oncoming | CONTINUE | stronger front/personal cost + optional side preference memory |
| crossing | CONTINUE | baseline cost; no pass-ahead mode is declared |
| crossing | YIELD | front-passing/conflict cost high + yield prompt + low-aggression guidance |
| crossing | PASS_BEHIND | front-passing cost high + behind corridor/target bias + guidance smoothness |

在当前论文边界内，`oncoming` 不进入主实验矩阵；如果出现该场景，只作为未来可扩展说明，不新增 `PASS_SIDE` mode。

## 8. Social Cost Parameterization

其中 `J_social` 不是一个固定势场，而是被当前交互场景、交互模式和时间记忆调制：

```text
J_social =
    w_personal     J_personal
  + w_future_path  J_future_path
  + w_front        J_front
  + w_conflict     J_conflict
  + w_behind_bias  J_behind_bias
```

各项含义：

- `J_personal`：行人个人空间代价，避免贴近身体；
- `J_future_path`：行人未来轨迹带代价，避免切入行人预测路径；
- `J_front`：行人前方区域代价，抑制抢行；
- `J_conflict`：机器人和行人到达冲突区时间接近时的惩罚；
- `J_behind_bias`：在 `PASS_BEHIND` 模式下鼓励从行人后方通过。

这比 DRF-only 更强，因为 DRF 主要回答“哪里危险”，而 mode-conditioned social cost 进一步回答“在当前交互语义下，哪里不合适走”。

## 9. Interaction Modes

### CONTINUE

使用原始 DRF-MPPI 目标和基础社会空间代价。

适用情况：

- 没有有效 crossing candidate；
- 行人不会进入机器人路径走廊；
- 机器人明显先到达且冲突风险低；
- crossing 置信度不足。

输出：

```text
mode = CONTINUE
social cost = baseline DRF + personal space
plan goal = original local goal
```

### PASS_BEHIND

用于横穿行人先到达或即将占据冲突区，且后方通过区域可行的情况。

它不直接命令机器人“必须绕后”，而是让 MPPI 看到如下代价场：

```text
行人前方区域代价升高
行人未来路径代价升高
行人后方通道相对更可取
必要时提供 behind local target / corridor bias
```

输出：

```text
mode = PASS_BEHIND
social cost = front/future/conflict high + behind bias
optional plan goal = feasible behind-pedestrian local target
```

### YIELD

用于冲突强、近乎同时到达、或者 behind target 不可行的情况。

真实导盲杖没有物理刹车能力，所以 `YIELD` 不应被表述为系统直接刹停，而应表述为：

```text
yield / stop prompt
高冲突区代价
降低继续抢占冲突区的倾向
```

输出：

```text
mode = YIELD
social cost = conflict/front high
yield_prompt = true
stop_enforce = false by default
```

## 10. Crossing Scene Handling Policy

横穿场景采用 **pedestrian-priority conservative crossing order + MPC lateral guidance** 策略。

核心原则：

```text
横穿行人默认优先；
导盲杖不引导盲人用户从行人前方抢行；
当行人后方空间安全时，从行人后方通过；
否则输出 YIELD / stop prompt，等待冲突解除。
```

这借鉴 crossing-order 思路，但针对导盲杖场景做了收敛：

```text
mode ∈ {CONTINUE, YIELD, PASS_BEHIND}
PASS_AHEAD is not selected as a guidance mode.
```

### 6.1 Crossing Conflict Definition

在 path-aligned local frame 中：

```text
x: 用户前进方向
 y: 左右横向方向
```

用户和行人的短时预测可写成：

```text
p_u(t) = p_u(0) + v_u t
p_h(t) = p_h(0) + v_h t
```

当满足以下条件时，认为存在 crossing interaction：

- 行人在用户前方或侧前方；
- 行人速度方向与用户前进方向夹角较大，具有明显横穿分量；
- 行人预测轨迹会穿过用户前方通道；
- 未来 `T_horizon` 内两者距离可能低于交互阈值。

除 `front/lateral/time_gap` 判断外，可以使用 CPA 作为辅助冲突判据：

```text
r = p_h - p_u
v_rel = v_h - v_u

t_cpa = - (r dot v_rel) / ||v_rel||^2
d_cpa = ||r + v_rel * t_cpa||
```

如果：

```text
0 < t_cpa < T_horizon
and d_cpa < d_interaction
```

则 crossing conflict 更可信。

CPA/TTC 不作为主方法贡献，也不单独替代 DRF；它只作为轻量冲突判据，帮助判断是否进入 crossing interaction。

### 6.2 Mode Policy in Crossing

#### CONTINUE

适用情况：

- 没有显著 crossing conflict；
- `d_cpa > d_safe`；
- `t_cpa` 不在预测窗口内；
- 行人已经远离用户路径或离开前方通道；
- crossing 置信度不足。

行为：

```text
保持原 DRF-MPPI 目标；
只使用基础 personal/social risk；
不额外绕行或 yield。
```

#### YIELD

适用情况：

- 行人即将横穿用户前方；
- 用户和行人到达冲突区的时间间隙不足；
- `d_cpa < d_safe` 且 `|t_user_conflict - t_human_conflict| < tau_gap`；
- `PASS_BEHIND` 后方通道暂时不可行。

行为：

```text
输出 yield / stop prompt；
提高 pedestrian front-passing zone 和 conflict zone 的代价；
默认不强制物理刹停。
```

由于导盲杖不能可靠控制用户前向速度，`YIELD` 应被解释为用户提示和局部引导稳定策略，而不是系统直接制动。

#### PASS_BEHIND

适用情况：

- 行人已经接近、占据或开始通过用户前方路径；
- 行人后方存在安全通道；
- behind target / behind corridor 可行；
- 静态障碍物和 ESDF 检查安全；
- 该模式相对 `CONTINUE` / `YIELD` 有足够 score margin。

行为：

```text
提高 pedestrian front-passing zone 代价；
提高 pedestrian future-path / conflict zone 代价；
提供 behind local target 或 behind corridor bias；
MPPI 在该代价场下求解横向引导。
```

### 6.3 No Pass-ahead Constraint

当前方法明确不选择 `PASS_AHEAD` 作为导盲杖引导模式。

原因：盲人用户对横穿行人的意图、速度变化和对方让行意愿感知不足。从行人前方抢行虽然可能在几何上更短，但不符合保守导盲交互策略。

可以在代价中体现为：

```text
J_front_pass = w_front * I(user trajectory enters pedestrian front-passing zone)
```

或在更保守实现中作为约束：

```text
trajectory should not enter pedestrian front-passing zone
```

当前阶段建议先用高代价而不是硬约束，避免 MPPI 在狭窄或误检场景下无可行解。

### 6.4 Crossing Mode Transitions

推荐模式转移：

```text
CONTINUE -> YIELD:
  confirmed crossing conflict
  and time gap is insufficient
  and PASS_BEHIND is not feasible yet

CONTINUE -> PASS_BEHIND:
  confirmed crossing conflict
  and pedestrian-priority order favors passing behind
  and behind target/corridor is feasible

YIELD -> PASS_BEHIND:
  pedestrian has passed or is passing the user path
  and behind space becomes feasible

YIELD -> CONTINUE:
  pedestrian leaves the user path
  and crossing conflict is stably cleared

PASS_BEHIND -> CONTINUE:
  user has passed behind the pedestrian
  and social/conflict risk is stably cleared
```

模式切换不依赖固定的 `PASS_BEHIND` 执行时长。进入 crossing 需要连续帧确认；切换 mode 需要满足 score margin；退出需要 clear condition 稳定。短 `min_debounce_time` 只作为单帧翻转兜底。

## 11. Crossing Candidate Assessment

使用 path-aligned local frame，而不是只依赖机器人 yaw：

```text
path_forward = normalized(current_local_goal - robot_position)
path_left = rotate90(path_forward)
```

对每个行人计算：

```text
rel = pedestrian_position - robot_position
front = rel dot path_forward
lateral = rel dot path_left
v_front = pedestrian_velocity dot path_forward
v_lateral = pedestrian_velocity dot path_left
```

crossing candidate 的基本条件：

- `front` 位于机器人前方有效范围；
- `abs(lateral)` 接近路径走廊；
- `abs(v_lateral)` 大于横穿速度阈值；
- 行人正在接近或穿越路径走廊。

时序估计：

```text
t_ped_to_path = abs(lateral) / max(abs(v_lateral), eps)
t_robot_to_cross = front / max(robot_speed, min_robot_speed)
time_gap = t_robot_to_cross - t_ped_to_path
```

注意：`robot_speed` 应使用与 LFPC/仿真一致的实际前进速度估计，不应误用默认仿真速度常数，否则会导致 crossing timing 判断失真。

## 12. Scene Confidence Accumulation

为了避免单帧检测噪声导致模式误触发，crossing 不应由单帧 raw 判断直接激活。

建议使用连续帧确认：

```text
if raw_scene == CROSSING:
    crossing_count += 1
else:
    crossing_count = 0

if crossing_count >= N_cross:
    active_scene = CROSSING
```

典型参数：

```text
N_cross = 2 ~ 5 frames
```

该机制可以解释为 scene confidence accumulation。

## 13. Risk-based Scene and Mode FSM

Scene 和 mode 不应由单帧硬规则直接切换。Stage 2 使用 risk-based finite state machine with hysteresis：

```text
perception / prediction
    -> continuous risk scores
    -> scene confidence
    -> scene FSM
    -> mode FSM
    -> (scene, mode) -> MPC objective
```

当前最小实现只需要支持：

```text
scene ∈ {none, crossing}
mode ∈ {CONTINUE, YIELD, PASS_BEHIND}
```

`static` 和 `oncoming` 可以保留为 objective mapping 的概念扩展，但不进入当前 Stage 2 主实现。

### 12.1 Risk Scores

当前 crossing 验证所需的最小风险评分：

```text
R_crossing: 是否存在横穿交互
R_collision: 未来近碰/碰撞风险
R_front_pass: 从行人前方抢行风险
R_behind_free: 行人后方通道可行性
```

其中 `R_crossing` 可以由以下因素组成：

- 行人速度方向与用户前进方向近似垂直；
- 行人预测轨迹穿过用户前方通道；
- `t_cpa` 位于预测窗口内；
- `d_cpa` 较小；
- `time_gap` 表明双方接近同一冲突区。

概念扩展风险包括：

```text
R_static
R_oncoming
```

但它们不要求在当前 crossing-focused 实现中完成。

### 12.2 Scene Hysteresis

使用双阈值和连续帧确认避免 scene 抖动：

```text
enter crossing:
  R_crossing > R_cross_enter
  for N_enter consecutive frames

exit crossing:
  R_crossing < R_cross_exit
  for N_exit consecutive frames
```

典型关系：

```text
R_cross_enter > R_cross_exit
```

例如：

```text
R_cross_enter = 0.7
R_cross_exit  = 0.3
```

中间区域保持当前 scene，不每帧重新切换。

### 12.3 Mode Transitions

mode transition 同样基于风险条件、连续帧确认和 switch margin，而不是固定执行某个 mode 若干秒。

推荐转移：

```text
CONTINUE -> YIELD:
  R_crossing high
  R_collision high or time gap small
  R_behind_free low
  condition holds for N frames

CONTINUE -> PASS_BEHIND:
  R_crossing high
  pedestrian-priority order favors rear passing
  R_behind_free high
  condition holds for N frames

YIELD -> PASS_BEHIND:
  pedestrian has crossed or is crossing the user path
  R_behind_free high
  condition holds for N frames

YIELD -> CONTINUE:
  R_crossing low or conflict cleared
  condition holds for N_exit frames

PASS_BEHIND -> CONTINUE:
  user has passed behind pedestrian
  social/conflict risk is low
  condition holds for N_exit frames
```

切换条件可写成：

```text
switch to new_mode only if:
    score(new_mode) + switch_margin < score(current_mode)
    or current_mode is infeasible / unsafe
    or clear condition is stable
```

### 12.4 Switch Penalties

上层 mode 切换可使用：

```text
J_mode_switch = w_mode * I(mode_k != mode_{k-1})
```

这可以实现为 mode score 中的软惩罚，也可以等价地实现为 switch margin。

底层导盲提示应额外考虑方向切换惩罚：

```text
J_guidance_switch = w_g * I(sign(g_k) != sign(g_{k-1}))
```

它的作用是防止导盲杖左右引导方向频繁翻转。

### 12.5 Debounce Window

可以保留很短的 `min_debounce_time`，例如 `0.3 ~ 0.5s`，作为单帧翻转兜底。

但它不应被解释为：

```text
进入 PASS_BEHIND 后必须强制绕后执行固定时长
```

而应解释为：

```text
刚切换 mode 后，忽略单帧噪声导致的立即反向切换；
若当前 mode 变得不可行或不安全，仍允许提前切换。
```

## 14. Mode Scoring and Switch Penalty

每一帧可以对候选 mode 进行轻量评分：

```text
mode_k = argmin_m [
    J_mpc_estimate(m)
  + J_social_estimate(m)
  + J_feasibility(m)
  + J_switch(m, mode_{k-1})
]
```

其中：

```text
J_switch = w_switch * I(m != mode_{k-1})
```

`J_switch` 的作用是软滞回：只有当新 mode 明显更优时才切换，避免因为代价微小变化或 MPPI 采样随机性导致 pass-mode 抖动。

但 `J_switch` 不能单独决定交互策略。它只能提高时间稳定性，不能替代 mode feasibility 和 crossing timing 判断。

## 15. Temporal Consistency and Mode Memory

Stage 2 需要显式时间一致性机制，但不应把“进入某个 mode 后必须硬保持若干秒”作为主逻辑。

推荐机制是：

```text
continuous-frame confirmation
+ mode confidence / switch margin
+ clear condition
+ optional short debounce window
```

推荐规则：

- 只有连续 `N_cross` 帧满足 crossing 条件，才允许进入 crossing-related mode；
- 每一帧重新评估当前 mode 是否仍然合理；
- 新 mode 只有在 score 明显优于当前 mode 时才允许切换；
- 如果当前 mode 变得不可行或不安全，允许立即切换；
- `CONTINUE` 只有在 clear condition 连续满足后恢复；
- 很短的 `min_debounce_time` 只作为防止单帧瞬时翻转的兜底，不作为强制执行 `PASS_BEHIND` 的主要依据。

可以把切换条件写成：

```text
switch to new_mode only if:
    score(new_mode) + switch_margin < score(current_mode)
    or current_mode is infeasible / unsafe
    or crossing_clear is stable
```

这部分可以在论文里称为：

```text
temporal consistency mechanism
```

它的目标不是替代 MPC，也不是把 `PASS_BEHIND` 锁死固定时长，而是防止导盲杖左右抖动或 pass-ahead/pass-behind 因为微小代价差反复切换。

## 16. Behind Target / Behind Bias

`PASS_BEHIND` 可以通过两种方式影响 MPPI：

1. behind local target；
2. behind-biased social cost。

最小实现可以先保留 behind local target：

```text
ped_dir = normalized(pedestrian_velocity)
predicted_ped = pedestrian_position + pedestrian_velocity * t_ped_to_path
raw_behind = predicted_ped - ped_dir * behind_dist
intent_target = raw_behind + path_forward * forward_bias
```

可行性检查：

- target 位于机器人前方；
- target 在局部规划范围内；
- target 不在静态障碍物内；
- target 与预测行人位置保持足够距离；
- target 偏离全局路径不超过阈值；
- target 不要求机器人向后追赶。

如果 behind target 不可行，不应强制 `PASS_BEHIND`。

后续如果 behind target 仍不稳定，可以再加入 `J_behind_bias` 或 crossing corridor penalty，而不是一开始扩大复杂度。

## 17. Development Variants

### E0: Wrapper

只搭建 interaction decision 框架，不改变原 DRF-MPPI 行为。

要求：

```text
mode = CONTINUE
plan goal = original local goal
cost = original DRF-MPPI cost
```

E0 应该与 C baseline 行为基本一致。

### E1: PASS_BEHIND + Temporal Consistency

Stage 2 的核心版本。

包括：

- crossing candidate detection；
- crossing confidence accumulation；
- `PASS_BEHIND` mode selection；
- behind target feasibility；
- mode confidence / switch margin；
- optional short debounce window；
- mode-conditioned social cost 或 target bias。

E1 的目标是减少 DRF-only 中 pass-ahead/pass-behind 不稳定，而不是靠停止取胜。

### E2: YIELD Prompt

只有在 E1 稳定后加入。

包括：

- `YIELD` mode；
- yield / stop prompt；
- stop duration 约束；
- 默认不启用物理 stop enforce。

E2 不能变成“停着赢”的策略。

## 18. Required Metrics

必须记录：

- success rate；
- collision count；
- near-collision count；
- minimum clearance；
- pass mode；
- pass-mode switch count；
- active mode sequence；
- mode transition count；
- `PASS_BEHIND` dwell time；
- `YIELD` dwell time；
- crossing confidence count；
- behind target valid ratio；
- global path deviation；
- path efficiency；
- stop / yield duration；
- MPPI planning time。

最关键的是：

```text
E 是否比 C 减少 near-collision / collision
E 是否比 C 减少 pass-mode switch
E 是否保持可接受路径效率和实时性
```

## 19. Experiment Groups

Stage 2 仍然使用 crossing timing sweep：

```text
early
simultaneous
late
```

对比：

```text
B0: no DRF, no dynamic interaction intervention
C: DRF-only MPPI
E: scene/mode-conditioned social-cost DRF-MPPI
```

每个 timing 至少 3 次，条件允许时 5 次。

实验目标不是证明 E 在所有指标上都最优，而是证明：

```text
相比 DRF-only，E 的交互模式更稳定、更可解释，并且安全性不差或更好。
```

## 20. Paper Framing

推荐名称：

```text
Interaction-aware DRF-MPPI with Scene-conditioned Social Cost
```

或者：

```text
Scene-conditioned Social-cost DRF-MPPI for Smart-cane Pedestrian Interaction
```

避免过度使用 `intent prediction` 这类表述。本文不是学习复杂人类意图，而是在单行人 crossing 场景中进行轻量、可解释、实时的交互模式选择与社会代价调制。

可以用一句话概括：

> DRF tells where risk is. The interaction layer decides which crossing mode to maintain. Scene-conditioned social cost tells MPPI which regions are socially inappropriate under that mode. MPPI solves the feasible local motion.
