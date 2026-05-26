# RAL 方向决策：面向导盲杖行人交互的 Intent-aware DRF-MPPI

## 1. 当前结论

DRF-only MPPI 是 Stage 1 的连续风险基线和局限性量化工具，不是最终 RAL 论文的完整方法贡献。

当前更合理的主线是：

> DRF 负责表达动态行人风险，Intent-aware 层负责表达“继续前进 / 让行 / 绕后通过”等交互语义，并将该语义转化为 MPPI 可执行的局部规划偏置。

论文边界需要保持收敛：本文不试图解决一般人群导航问题，也不声称学习或预测复杂人类意图。本文只关注导盲杖在单行人 crossing 场景下的轻量、可解释、实时交互决策。

## 2. 为什么 DRF-only 不够

基于 Stage 1 结果，目前可以得到以下判断：

- DRF 能改变动态行人场景下的轨迹选择；
- DRF-only 在某些参数和时序条件下可以产生 pass-behind 行为；
- 但 pass-ahead / pass-behind 是隐式涌现的结果，不是显式交互决策；
- 在当前 crossing 场景和 DRF-only cost 下，增大 MPPI 采样数 K 未能稳定降低近碰风险或消除 pass-mode 不稳定；
- CPA/TTC 已经作为内部变体测试过，但不进入主方法；
- 连续风险势场能够表达“哪里危险”，但无法显式表达“让行 / 绕后 / 继续前进”等交互语义。

因此 Stage 1 的价值不是证明 DRF-only 足够强，而是证明：

> DRF-only MPPI 是一个合理但不充分的连续风险基线。

## 3. 拟定 RAL 贡献

拟定三个贡献：

1. 智能导盲杖行人交互问题建模；
2. DRF-MPPI 基线系统与局限性量化；
3. 面向 crossing 行人交互的轻量 Intent-aware 层。

其中第三点是最终方法的核心。它不应被表述为简单增加一个绕后权重，而应表述为：

> 通过冲突预测、交互意图仲裁和状态保持机制，将行人交互语义转化为 MPPI 的局部规划偏置。

## 4. Stage 2 最小方法

Stage 2 先只保留三个状态：

- `CONTINUE`：继续跟踪全局路径；
- `YIELD`：让行意图，可转化为停止建议或减速提示；
- `PASS_BEHIND`：绕到行人后方通过。

暂时不把 `PASS_AHEAD` 作为主要策略。对导盲杖辅助场景而言，`PASS_BEHIND` 和 `YIELD` 更保守，也更容易解释和防守。

在真实导盲杖上，`YIELD` 对应语音、触觉或方向引导层面的停止建议，而不是系统物理刹车。

## 5. Intent 如何影响 MPPI

Intent 不直接控制机器人，也不直接生成最终控制量。MPPI 仍然负责局部轨迹采样与优化。

Intent-aware 层只向 MPPI 提供偏置，例如：

- 局部子目标偏置；
- 绕后目标点；
- pass-behind cost；
- yield / stop advice；
- 可选的冲突通道惩罚。

Stage 2 最小闭环优先实现：

1. `YIELD`：输出 stop/yield advice；
2. `PASS_BEHIND`：生成绕后局部子目标；
3. `CONTINUE`：保持原 DRF-MPPI 目标。

实际代码实现顺序应反过来：

1. `CONTINUE baseline wrapper`：先搭 intent state machine 框架，默认不改变现有 DRF-MPPI 行为；
2. `PASS_BEHIND local target`：实现 Stage 2 核心能力，先证明能够稳定绕后；
3. `YIELD stop/yield advice`：最后加入让行建议，并受 stop duration 约束，防止变成“停着赢”的策略。

`pass-behind cost` 和 `corridor penalty` 作为可选增强，只有在局部子目标不足以稳定行为时再加入。

目标是让 MPPI 执行一个稳定的交互模式，而不是每次都从连续风险场中隐式地重新选择 pass-ahead 或 pass-behind。

可以概括为：

> DRF tells where risk is. Intent tells what interaction mode to execute. MPPI solves the local motion.

## 6. 最小实验设计

Stage 2 先只做 crossing timing sweep，不扩展大规模场景矩阵。

三种时序：

- `early`：行人先到达冲突点；
- `simultaneous`：机器人和行人几乎同时到达冲突点；
- `late`：机器人先接近冲突点。

对比方法：

- `B0`：无 DRF，无动态交互干预；
- `C`：DRF-only MPPI；
- `E`：Intent-aware DRF-MPPI。

每个 timing 至少重复 3 次，使用不同随机种子或 MPPI 采样序列；条件允许时优先重复 5 次。否则不能支撑 pass-mode stability 的结论。

实验目标不是证明 E 在所有指标上都最优，而是证明 E 相比 C 具有更稳定、更可解释的交互行为。

## 7. 必要指标

必须记录以下指标：

- 成功率；
- 碰撞次数；
- 近碰撞次数；
- 最小 clearance；
- pass mode；
- pass-mode switch；
- intent 状态切换次数；
- `YIELD` / `PASS_BEHIND` 状态驻留时间；
- 路径偏离；
- 路径效率；
- 停止时长；
- 规划时间。

其中最关键的是：

- 是否减少 near-collision / collision；
- 是否减少 pass-mode 不稳定；
- 是否保持可接受的路径效率和实时性。

## 8. 最小实物演示

实物阶段不做大规模实验矩阵，先做一个代表性的横穿行人 demo，证明完整系统链路能跑通。

该 demo 要验证：

```text
odometry / map
    -> pedestrian observation
    -> DRF / intent-aware MPPI
    -> smart-cane steering response
    -> rosbag + video + trajectory analysis
```

实物 demo 的作用不是替代仿真定量实验，而是证明方法可以接入真实导盲杖系统。

## 9. 当前不要做什么

当前阶段不要做以下事情：

- 不继续扫 K；
- 不把 CPA/TTC 写进主方法；
- 不做复杂 speed / noise / narrow-corridor 大矩阵；
- 不把规则层写成直接控制器；
- 不声称当前 Stage 1 已经是 intent-aware 方法；
- 不把 Stage 2 简化表述为“加一个绕后权重”。

## 10. Go / No-Go 标准

只有当 Intent-aware DRF-MPPI 相比 DRF-only 在安全性或模式稳定性上有明确改善，并且不显著恶化实时性与路径效率，才进入论文主方法。

重点观察：

- near-collision / collision 是否减少；
- pass-mode switch 是否减少；
- `PASS_BEHIND` 或 `YIELD` 行为是否更稳定；
- 路径效率是否仍在可接受范围；
- 规划时间是否仍满足实时要求。

暂定 Go 标准：

- crossing timing sweep 中，Intent-aware 方法的 collision + near-collision 次数低于 DRF-only；
- pass-mode switch 次数低于 DRF-only；
- 平均规划时间不超过 DRF-only 的 1.2 倍；
- 路径效率下降不超过 15%；
- E 的 stop duration 不超过总任务时间的 30%，除非该场景中 DRF-only 出现 collision / near-collision；如果 E 增加停止时长，必须对应明显的 near-collision / collision 降低。

上述阈值是 Stage 2 开发阶段的临时门槛，后续应根据真实实验数据调整；它们的作用是避免只凭 RViz 观感判断方法是否有效。

如果 Intent-aware 方法只是在个别参数下改善轨迹，但不能稳定改善交互行为，则不应作为最终 RAL 主贡献。
