# 社会力行人仿真实施进度

更新日期：2026-10-29。**阶段 1–4 已完成，空场地多人社会力仿真与 RViz 编辑已可测试。**

## 目标与约定

使用 pedsim 社会力驱动行人，在与 planner 3 一致的环境中验证 A*、动态空间走廊与 LFPC MPPI。以官方 `blank_world.xml` 的规则场地开始，后续增加长走廊和交叉场景。规划端保留匀速预测作为基线；仿真端内部目标和真实受力仅用于诊断。

本文件是未完成工作的唯一进度记录。继续社会力仿真工作前先读本文件；每阶段结束或中断时更新完成项、验证证据和下一步。研究依据见 [可行性调研](PEDSIM_SOCIAL_FORCE_FEASIBILITY.md)。

## 阶段与完成条件

| 阶段 | 状态 | 完成条件 |
| --- | --- | --- |
| 1. 新地图与单行人仿真 | 已完成 | 同一场景生成 pedsim 墙与规划地图；提供单行人对穿/横穿/超越入口；机器人实际位置反馈给 pedsim；私有 ROS master 下完成地图、观测和导航验证 |
| 2. 社会力与行人凸包校正 | 已完成 | 修复人工裁剪边界产生假墙力；区分速度和各项受力；验证墙/机器人作用方向与几何结果；保留 CV 时间碰撞检查 |
| 3. 多人完整规划链路 | 已完成 | 桥接、输入校验、所有行人凸包、全局 A*、FIRI、预测碰撞和显示都支持多人；先验证 2–5 人 |
| 4. RViz 场景编辑 | 已完成，可测试 | 选中、增删行人；配置起点/目标/期望速度/往返；保存、加载并恢复场景；参数实际传给社会力仿真器 |

## 当前边界

- planner 3 与 pedsim 桥接支持每帧零至五人，预设生成 1–5 人，空场地面板支持 0–5 人；新鲜空帧合法，超人数或非法帧会停车。
- 空场地使用 pedsim 专用多人面板，可在线编辑、暂停、保存/加载和恢复配置。旧确定性单人面板独立；办公室场景仍通过 YAML 与重启自有轮次应用。
- 旧办公室入口继续可用；新场景以同一份几何生成两侧地图，坐标、墙厚、范围和过滤高度一起对齐。
- 社会力下设置的是期望速度，实际速度由交互决定。
- 测试使用自有私有 ROS master，仅清理本次启动的进程。
- Trellis 已退役，本记录不引入其任务、审批、钩子或日志流程。

## 当前工作与下一步

1. 用户已开始体验 blank 多人编辑与规划；首次反馈的“重新设置起点后第二次无法执行”已复现并修复，见末节。Qt 面板截图已检查中文与布局，继续按实际操作核对交互体验。
2. 后续地图扩展顺序：同源长走廊，再做交叉场景；仍由同一几何生成仿真墙和规划地图。当前四个阶段没有尚未接通的模块。
3. 已为标准 pedsim 启动器接入单调时钟 `/clock`，隔离用户运行 `197da7d6` 中 ROS 时间异常推进造成的误过期。真实导航故障注入已验证：旧启动方式在系统时间前跳后失败，新方式经过前跳/回退后仍完成两次到达；真实输入中断仍锁定，时钟进程退出会结束本轮。还验证了同 tick 位姿发布与 `/clock` 消息乱序的兼容，见末节。此前不同类型的输入故障仍须保留各自结论，不能宣称所有间歇问题已消失。
4. 研究评估下一步可在可编辑场景上固定配置、重复运行并统计等待/绕行/碰撞与耗时；仿真目标和真实社会力继续只用于诊断，规划预测仍以 CV 为基线。
5. 已提供 `--pedestrian-mode independent`，用于行人不主动避让人杖的独立避障实验；默认 `interactive` 已按后续要求改为仅在 220° 视野内对人杖响应。真实双模式受力、视野边界、编辑与五人导航已验证，见末节；没有改变规划端凸包/预测算法。

## 已完成的阶段 1

- `export_pedsim_blank.py` 读取官方 `blank_world.xml` 的四面墙，平移到 `[-15,15]²`，生成同源 `world.xml`、`world.pcd` 和 `world.json`；墙厚 0.10 m，墙采样高度 0.2–2.0 m，地面 z=-0.1 m，点云共 162120 点。
- 启动器新增 `--world blank|office`，默认新场地；新场地规划静态地图与仿真点云共用生成的 PCD。普通轻量仿真的默认地图仍为办公室。
- `blank_scenarios.yaml` 提供对穿、横穿、超越的单行人配置；终端打印建议机器人起终点，由 RViz 操作。原行人面板暂不控制 pedsim。
- 机器人实际 LFPC `/sim_odom`、原始时间戳 TF 和已有成对观测桥接继续使用；本阶段没有改规划端社会力算法或多人限制。

运行入口：

```bash
bash "$(rospack find plan_manage)/scripts/start_pedsim_navigation.sh" \
  start --world blank --scenario crossing
```

RViz 起点 `(-5,0)`、朝向 `+X`，终点 `(5,0)`；超越场景终点 `(6,0)`。场景操作见 [USAGE.txt](../plan_manage/config/pedsim/USAGE.txt)。

## 阶段 1 验证记录（2026-10-29）

- `catkin build plan_manage map_generator --no-deps -DCMAKE_BUILD_TYPE=Release`：通过；已有 PCL/弃用依赖和未使用变量警告。
- `python3 plan_manage/test/test_pedsim_blank.py`：2 项通过，覆盖 XML/PCD 墙几何一致、三场景共用确定性地图、地面过滤及拒绝错误模板。
- 加载 ROS、主工作区和 pedsim overlay 后运行 `python3 plan_manage/test/test_pedsim_interface.py`：原有 6 项通过。
- 启动参数解析通过；新场地的规划静态地图与仿真点云均指向本轮 `world.pcd`。
- 自有私有 master 上逐个运行三个无人导航轮次，测试主动发布上述机器人起终点；每轮桥接进入 `RUNNING`，无 `FAULT_RESTART_ROUND`，各轮均出现 `Reached final goal`。启动等待首帧时有预期的 `DYNAMIC_DATA_INVALID` STOP。

| 场景 | 私有端口 | 有效行人观测帧 | 最大发布轨迹点数 | 结果 |
| --- | --- | --- | --- | --- |
| 横穿 | 48383 | 77 | 379 | 到达终点，社会作用力非零 |
| 对穿 | 45517 | 77 | 378 | 到达终点；行人从约 `(4,0)` 移动至 `(1.71,-1.34)`，发生侧向响应 |
| 超越 | 48063 | 85 | 421 | 到达终点，社会作用力非零 |

原始测试日志与 JSON 位于测试机器 `/tmp/cane-pedsim-stage1-{crossing,headon,overtake}.{log,json}`。临时文件可能被系统清理，上表保留本阶段结果。三轮均未启动 RViz，视觉呈现待操作时核对；这些测试覆盖接入和导航流程，尚未覆盖多人或统计避障性能。已检查本轮 ROS 子进程全部清理。

## 已完成的阶段 2（2026-10-29）

- 人工裁剪边缘和栅格外部不再产生社会力。供给地图中的不可通行栅格仍参与地图分量，FIRI 的保守边界约束独立保留。
- `PedestrianPolygon::build()` 输出地图与机器人分量，共同限幅后相加等于合力。几何方向仍由合力决定，CV 预测继续使用观测速度；本阶段未将仿真内部目标/真实受力输入规划器。
- `/mpc/convex_corridor` 增加青色地图、橙色机器人、紫红色合力、绿色速度箭头和英文图例。零向量不画箭头；空帧、无效/过期数据和目标重置会清理新增标记。
- 地图分量仍使用选定可通行栅格的近似，包含膨胀和未知区规则，且看不到供给裁剪范围外的障碍；不是 pedsim 墙力真值。完整运动社会力在仿真端，多人邻居项待阶段 3。

验证：生产包 `path_searching`、`plan_manage` 编译通过；14 项多边形/动态走廊测试和 5 项规划执行检查通过。新增回归覆盖空裁剪域内/外不产生力、域外真实障碍斥力方向，以及合力限幅后的分量相加。

私有 master `47875` 运行对穿导航，86 帧有效行人观测，机器人到达约 `(5.147,-0.001)`；检查了 49 帧实际发布的合力/分量箭头、图例、world 坐标与有效期，未出现桥接故障。原始日志和摘要为测试机器 `/tmp/cane-pedsim-stage2-headon.{log,json}`，测试进程均已清理；RViz 视觉效果由操作者核对。运行入口和机器人起终点与阶段 1 相同。

- 本轮尚未提交或推送。此前本地提交 `bf9e15d` 的上传状态以 Git 为准。

## 已完成的阶段 3（2026-10-29）

- 输入与桥接统一为零至五人；pedsim 按原始 ID 排序，拒绝重复 ID，保留原始时间戳、完整成对证据和机器人新鲜度保护。规划器不依赖跨帧身份关联。
- 每人独立构造凸包，并引入同帧其他行人的指数斥力；地图、机器人、邻居分量共同限幅，相加等于合力。新增蓝色 `mpc_pedestrian_peer_force`，编号及清理覆盖全部五人。
- 全局 A* 的冻结查询携带全部多边形，每条边逐一排除；不合并成大凸包，保留行人间通道。FIRI 逐个排除所有当前凸包，MPPI 与执行端继续对全部行人做 CV 时间同步 SAT；执行预测可视化补齐所有人。
- blank 配置新增 `crowd_2`、`crowd_5`，每人有独立起点、目标和期望速度。完整上游补丁、manifest 与本机独立依赖构建同步更新；`pedestrian_speeds` 数组按 XML 创建顺序应用，保留旧单人参数兼容，拒绝参数混用、长度错误和非法数值。
- 上游完整补丁从固定 revision 的原始三个文件重新生成，核对 baseline/patched SHA256 并验证可重新应用。修改依赖源码/构建期间没有重启用户节点。

验证：

- `catkin build path_searching plan_manage --no-deps --no-status -DCMAKE_BUILD_TYPE=Release` 通过；测试目标构建通过；独立 pedsim 工作区 `catkin build pedsim_simulator` 通过。
- `test_pedestrian_polygon` 16 项、`test_astar_edges` 27 项、`test_timed_pedestrian_collision` 8 项通过。新增用例证明第二/第五人可触发全局绕行、五人间隙不被大凸包封闭、第五人可阻断走廊和触发未来 SAT 相交。
- 私有 master 下 10 项 manager 检查通过，包含五人完整接收/走廊排除/标记清理、第五人阻挡已排队步态且空帧恢复、原事务时效与单人运动回归。修正测试中步态采样参数的泄漏，未改生产时效或模型参数。
- Python 几何配置 3 项、桥接/配对 6 项通过。真实 pedsim 启动验证拒绝速度数量不匹配、零速度、数组与旧参数混用、非数值数组；五人运行的各人实际速度均不超过各自配置上限。

私有 master 的真实导航（无 RViz）均进入目标 0.2m 半径；列出的耗时仅为本机该轮观测，并非实时性能保证：

| 场景 | 端口 | 观测帧 | 全人数当前凸包帧 / 预测帧 | 执行复检中位 / P95 / 最大 |
| --- | --- | --- | --- | --- |
| 两人 `crowd_2` | 47629 | 85 | 135 / 440 | 0.285 / 0.633 / 0.963 ms |
| 五人 `crowd_5` | 45417 | 85 | 137 / 406 | 0.293 / 0.617 / 1.026 ms |
| 单人 `headon`（最后一轮） | 45961 | 85 | 131 / 419 | 0.269 / 0.543 / 0.750 ms |

五人轮次 MPPI 调用中位 38.542ms，P95 57.653ms，最大 69.504ms；这些是含原有采样优化的整体耗时，不能全部算作新增多人开销。场景中的行人也会主动避让机器人；到达测试不意味着每次均由机器人产生显著侧向绕行，不代表连续运动安全或统计避障成功率。

单人兼容性复测曾在时间戳约 `1793264676.4` 触发 `FAULT_RESTART_ROUND`；日志相邻规划输出有约 0.9s 间隔，但没有足够证据定位其具体失败分支，不能直接归因于规划耗时。随后带临时定位信息的相同场景连续四轮通过，未重现；临时定位代码已移除，原始 freshness/配对规则不变。无人脚本已保留故障时 guard 状态和最大消息间隔输出，便于后续定位。

可重复运行（需加载 ROS、主工作区与 pedsim overlay）：

```bash
python3 plan_manage/test/pedsim_navigation_smoke.py crowd_5
```

脚本使用空闲私有 master、主动发送配置中的机器人起终点，检查全人数观测/力分量/预测显示/实际速度与导航进展，结束只清理自有轮次。摘要与日志为测试机器 `/tmp/cane-pedsim-stage3-{crowd_2,crowd_5,headon}.{json,log}`；单人重复轮次保存在 `headon-repeat-{1,2,3}.{txt,log}`。临时文件可能被清理，上表保留主要结果。所有测试进程结束后保留原有用户 11311 master；没有进行 RViz 目测验证。

手动体验：

```bash
bash "$(rospack find plan_manage)/scripts/start_pedsim_navigation.sh" \
  start --world blank --scenario crowd_5
```

RViz 使用 `2D Pose Estimate` 设置 `(-5,0)` 朝 +X，再以 `2D Nav Goal` 设置 `(5,0)`；替换 `crowd_2` 可测试两人。当前行人参数仍在 YAML 修改，下一阶段实现 pedsim 专用的多人交互编辑。本轮未提交或推送。

## 已完成的阶段 4（2026-10-29）

- 新增真实 pedsim `EditScene` 服务，支持全量替换、开始/继续、暂停和查询。先校验全部配置，再替换普通行人；保留机器人、墙体及输入保护历史。0–5 人，速度 0–3 m/s，起点至少相距 0.55 m，终点停止/返回一次/持续往返均由真实社会力积分驱动。
- 专用 `SocialPedestrianPanel` 和起点 P、终点 K、选择 U 工具支持多人编辑；参考时间只计算目标，实际到达时间由社会力决定。地图更改、应用、增删、加载和复位使全部行人复位等待；暂停冻结行人并继续真实静止观测，不使用全局 pause。
- 配置采用 `cane_pedsim_scene_v1` YAML，可保存/加载，并默认按稳定场景上下文自动恢复；空场景可记忆。`--fresh-scene` 完全跳过自动记忆读写，`--editor-state-file` 指定独立缓存。测试均使用临时缓存或关闭记忆，没有读写用户缓存。
- 编辑回执与观测携带场景版本。面板直接从成功回执取得版本，解决快速连续操作被旧版本拒绝的问题；状态心跳不覆盖未应用字段。桥接保留精确 odom/TF 配对，丢弃旧版本晚到帧；未就绪编辑器阻止首轮释放。外部改场景、真实依赖丢失和上游保护锁定均不能被编辑/新目标清除。
- 上游补丁现包含六个文件，新增服务及其构建依赖也纳入 manifest。已从固定 revision 原始文件重新应用补丁并逐一核对全部 hash；本机依赖已重建。编辑预览/目标不会进入规划观测。

验证证据：

- 独立 pedsim 的 `catkin build pedsim_simulator`、主工作区 `catkin build plan_manage --no-deps` 和可选 Qt 测试目标构建均通过；最终 `git diff --check` 通过。
- `test_social_scene.py` 4 项通过，覆盖所有预设的序列化/反序列化、空场景、坐标系/人数/ID/重叠/非有限数/坏 YAML 等拒绝规则。
- `test_pedsim_interface.py` 15 项通过，保留原有配对/时效测试，并覆盖新版本两路乱序与旧帧晚到、持续零人/静止批次、编辑器就绪门槛、真实丢失锁定、失去快照后仍拒绝外部版本及非法回执不改变配置。额外修复较新完整批次已转发后，旧的未配对证据仍留在队列并误报过期的问题；回归先失败后通过，不能据此推断此前所有间歇故障都来自该分支。
- 私有 master `47637` 上 `pedsim_scene_smoke.py` 最终完整通过（前一完整验证轮次 `46779` 也通过）：真实 Qt/pluginlib 放置/应用/暂停/文件保存加载，地图起终点/选人、非法和旧版本请求、实际 stop/return/repeat、暂停超过 0.5s 仍新鲜、增删/清空人数传播、空场景重启恢复、关闭记忆、坏缓存回退，以及真实停止机器人输入后禁止恢复。获得 160 帧观测，正常运行最大发布里程计间隔约 50.5 ms、观测时间间隔约 102.1 ms；末尾 TF_STALE 是故意断开输入的预期结果。
- 面板截图 `/tmp/cane_social_pedestrian_panel.png` 已实际查看，中文及滚动布局正常；完整 RViz/OGRE 中的鼠标交互没有无人目测验证。
- 私有 master `46609` 上 `pedsim_navigation_smoke.py crowd_5 --edit-scene` 通过：机器人设置起点后加载编辑场景，五人起终点整体 Y 偏移 0.2 m、各人速度改为预设的 85%，确认编辑不复位机器人；随后真实导航到约 `(5.136,0)`。87 帧观测、138 帧全人数当前凸包和 416 帧预测显示；无故障锁定。执行复检 P50/P95/最大约 0.289/0.538/0.915 ms，MPPI 整体约 45.206/55.247/55.423 ms；仅为该轮观测。
- 调试中曾遇到非预期输入时效故障：一轮上游 TF 年龄达到约 1.307 s，另一轮桥接配对心跳间隔约 0.977 s。定位轮次里场景替换仅约 8–56 µs，完整流程随后通过，未证明编辑服务造成超时，也未证明间歇性故障已修复。临时定位代码均已移除，生产桥接保留首次故障原因日志。
- 测试进程已清理，仅保留用户原有 11311 master（PID 83849）。本轮未提交或推送。

主要日志在测试机器 `/tmp/cane-pedsim-stage4-{editor-verified.log,edited-navigation.txt,edited-crowd_5.json}`；最终编辑测试目录 `/tmp/cane-pedsim-stage4-test.b9nj4h59`，临时文件可能被系统清理。

体验命令：

```bash
bash "$(rospack find plan_manage)/scripts/start_pedsim_navigation.sh" \
  start --world blank --scenario crowd_5
```

首次机器人起点 `(-5,0)`、朝向 +X，终点 `(5,0)`。面板可直接增删和加载；需忽略以前编辑时加 `--fresh-scene`。详细操作与数据格式见 [社会力场景编辑说明](development/operator-visualization/social-scene-editor.md)。后续扩展同源长走廊和交叉地图时从“当前工作与下一步”继续。

## 用户反馈修复：同一轮第二次设置机器人起终点无法执行（2026-10-29）

已修复。用户运行 `5336219b-0a24-4694-bd28-7a29716a07a2` 的日志显示：

- `17:59:27.623` 第一次到达终点 `(5.81,-4.70)`。
- `18:00:54.292` 重新设置机器人起点，`18:00:54.854` pedsim 报 `LATCHED_TF_STALE`，所用 TF 年龄约 0.561 s。
- `18:00:56.700` 才收到第二个目标；此时桥接已经锁定，规划器持续 `DYNAMIC_DATA_INVALID`。故障发生在第二次目标之前，不能归因为这次目标的走廊或 MPPI 计算。

根因是 `startCallback` 清除 `mpc_reached_goal_`，但 WAIT_TARGET 的静止位姿发布又依赖这个“已到达”标志。重新放置后只发布一帧新起点；等待操作员选目标时没有后续心跳，真实输入保护按规则锁定。执行定时器在 WAIT_TARGET 不推进或发布轨迹，不能补上此心跳。

修改：WAIT_TARGET 中，只要 planner 3 轻量仿真已有初始化的机器人/LFPC 状态且没有目标，就持续发布其真实当前位置、朝向和零速度。与“已到达”标志解耦，不推进步态；未初始化、Gazebo 和其他 planner 的模式保护保留。没有改 pedsim 的 0.5s 阈值或清除故障锁定。

验证：

- `pedsim_navigation_smoke.py crowd_5 --repeat-goals` 修复前复现：第一次实际到达后重设起点，约 0.55 s 后出现 `FAULT_RESTART_ROUND`，断言 `robot pose reset interrupted the live pedsim input heartbeat` 失败。测试依据 planner 的实际到达日志，避免把“已进入目标半径、但步态还没结束”误当完整到达。
- `catkin build plan_manage --no-deps --no-status --make-args kin_replan_node test_sim_odom_stop` 通过。
- 私有 master 下五项 manager 回归通过：新增重设起点后等待 0.8 s 的连续静止心跳检查；原有重设朝向与清零速度、实际到达后静止及新目标、最后安全步到达、未初始化/错误模式不发布等检查保留通过。新增用例同时确认 LFPC 内部状态不推进。
- 修复后的同一命令在私有端口 `47909` 上通过：第一目标到达 `(5.15,0)`，重设起点后等待 2 s，确认持续静止位姿与零速度，再发第二目标，到达 `(5.15,0.5)`；共 201 帧有效观测，状态始终运行，无故障锁定。
- `git diff --check` 通过，自有测试进程已清理，用户 11311 master 保留；没有提交或推送。

日志快照 `/tmp/cane-repeat-goal-user-log/`，失败复现 `/tmp/cane-pedsim-repeat-goals-before.{txt,log}`，成功摘要 `/tmp/cane-pedsim-repeat-goals-crowd_5.json`，回归日志 `/tmp/cane-repeat-goal-gtest.log`。这些临时文件可能被系统清理。本次修复无需改动上游补丁/manifest；重启本轮启动命令即可加载新 planner 二进制，之后可在同一轮连续测试。

## 本地提交节点（2026-10-29）

按用户要求，阶段 1–4 的社会力仿真接入、多人规划、RViz 编辑与连续测试心跳修复一并纳入 `mpc-lfpc-prototype` 的本地提交，本次未推送云端。前面章节中的“未提交”描述各阶段验证结束时的历史状态；当前提交范围以本节为准。后续仍按“当前工作与下一步”推进。

## 独立避障模式（2026-10-29）

已完成。新增启动参数 `--pedestrian-mode independent|interactive`，默认保留 `interactive`。独立模式只在仿真端社会力计算中跳过 ROBOT 邻居，保留目标驱动、墙体及行人间交互。仍保留真实机器人、LFPC 位姿反馈、原时效保护和成对观测；不是关闭全部社会力。该参数在本轮启动时选择，每次积分前应用，编辑替换、加载或复位不会丢失；场景 YAML 和记忆不覆盖模式。

规划端社会力凸包、当前空间走廊、实际速度 CV 预测与时间同步碰撞检查不变。橙色机器人力箭头仍属于规划估计，不是 pedsim 真值；仿真端单行人机器人交互力可从原始 `simulated_agents` 验证。上游完整补丁/manifest 增至八文件，新增 libpedsim 的头文件和实现；已从原固定 revision 校验原始文件并重新应用全部补丁核对 hash，依赖工作区已重建。

运行：

```bash
bash "$(rospack find plan_manage)/scripts/start_pedsim_navigation.sh" \
  start --world blank --scenario crowd_5 --pedestrian-mode independent
```

沿用 RViz 的机器人起点 `(-5,0)` 朝 +X、终点 `(5,0)` 和原场景编辑操作。改用 `--pedestrian-mode interactive` 恢复双向交互；当前轮结束后重新启动命令生效。启动器还修复了 `--help` 被 catkin 环境脚本截获的问题。

验证：

- `catkin build pedsim_simulator --no-status -j4 -p2`：五个依赖包全部构建成功；主规划 C++ 未改动。
- `pedsim_robot_interaction_smoke.py`：在私有端口 `48589`、`47299` 分别运行真实交互/独立模式，均通过。单行人启动与编辑替换后的机器人社会力，交互模式约 `0.447/0.444`，独立模式均为 `0`；独立模式仍有约 `0.214` 的邻居社会力、`0.014` 的墙体力和 `1.280` 的目标力。主动断开真实机器人输入仍按原规则锁定，场景 start 不能清除锁定。
- `pedsim_navigation_smoke.py crowd_5 --edit-scene --pedestrian-mode independent`：私有端口 `46989` 通过，真实编辑五人后完成一次到达，最终约 `(4.911,-0.058)`，87 帧有效观测、137 帧全员当前凸包、426 帧全员预测显示，无输入锁定。MPPI P50/P95/最大约 `35.410/55.312/56.319 ms`，仅代表本轮观测。
- `pedsim_navigation_smoke.py headon --pedestrian-mode independent`：更新后的启动脚本完整通过，机器人完成一次到达，最终约 `(4.920,0.039)`；单行人社会力始终为 `0`，沿原目标直线行走。87 帧观测，无输入锁定，摘要为 `/tmp/cane-pedsim-stage3-independent-headon.json`。
- 启动器帮助与 Bash/Python 语法检查通过。真实受力测试日志 `/tmp/cane-pedsim-robot-interaction-test.log`；五人导航摘要 `/tmp/cane-pedsim-stage4-edited-independent-crowd_5.json`。临时证据可能被系统清理。
- `git diff --check` 通过；自有测试进程已清理，用户原 11311 master 保留。

用户已实测确认该模式正常，本次按要求将模式开关、上游补丁、测试和说明一并纳入本地提交，未推送云端。下一步仍按“当前工作与下一步”进行地图扩展和可重复实验统计；本开关不是规划算法改动，也不代表所有场景避障成功。

## 交互模式的 220° 视野限制（2026-10-29）

已完成，可测试。用户希望人杖位于行人视野外时不引起主动避让。原 PDF 第 3 页 IV-A 说明 220° 视野内增强斥力，第 4 页式 (5)、(6) 与图 3 使用朝向 ±110°；没有明确规定视野外为零。本次采用论文角度，并按用户明确要求增加视野外关闭机器人交互的规则，不能称为对论文仿真细节的完整复现。

- `interactive` 只对当前朝向左右各 110° 内（包含边界）的人杖计算原机器人社会力，视野外为零。`independent` 继续在所有方向忽略人杖；目标、墙体和行人之间的交互未改，不增加墙体视线遮挡。
- 为仿真行人维护朝向：速度超过 `1e-4 m/s` 时沿实际速度更新，停止/暂停/到达后保留最后朝向；编辑复位使用 `start.theta`，旧 XML 未提供朝向时由首次期望方向初始化。原始 AgentStates 与编辑状态报告同一朝向，避免停车后错误显示为 +X 或跳回出生方向。
- 编辑器对选中者发布青色 `social_fov` 扇区和 `social_facing` 当前朝向箭头，标签显示 `FOV 220 deg`；RViz 配置默认启用它们。扇区半径 1 m 只示意角度，不是作用距离。独立模式不画扇区；原起点箭头保留。
- 上游补丁/manifest 仍为八文件，已从固定 revision 的纯净基线重新应用并核对部署 hash，pedsim 工作区已重建。主规划 C++、估计社会力凸包、CV 时间碰撞检查与输入保护未改；规划端橙色力箭头仍不是 pedsim 真值。

验证：

- 新增背后响应检查在旧版本实际失败，日志 `/tmp/cane-pedsim-fov-before.log` 报 `pedestrian reacts to the robot behind its field of view`。
- `catkin build pedsim_simulator --no-status -j4 -p2`：全部五个依赖包成功，耗时约 13 s，既有警告保留。
- `pedsim_robot_interaction_smoke.py`：私有端口 `46703`（交互）、`45015`（独立）通过。交互模式单人前方力约 `0.444`，背后为 `0`；在相同 1.5 m 距离的零速受力对照中，±109.9° 和 ±110° 均约 `0.424`，±110.1° 及 180° 均为 `0`；独立模式全部为 `0`。显式 π 朝向与实际目标相反的零速行人保持该朝向，未被目标方向覆盖。墙、目标、邻居力仍存在，真实输入断开仍锁定。日志 `/tmp/cane-pedsim-fov-interaction-test.log`。
- `pedsim_scene_smoke.py`：私有端口 `47557` 通过真实 Qt/pluginlib、地图编辑、保存/加载/记忆、暂停心跳、stop/return/repeat 及输入锁定检查；新增检查确认发布扇区与同帧实际位置/朝向一致，边界为 ±110°，返回后停止仍保留返程朝向。日志 `/tmp/cane-pedsim-fov-editor-test.log`；完整 RViz 窗口的视觉体验仍需操作者核对。
- `pedsim_navigation_smoke.py crowd_5 --edit-scene --pedestrian-mode interactive`：私有端口 `48601` 通过，编辑五人后实际完成一次到达，最终约 `(5.148,0)`；88 帧观测，139 帧全员当前凸包、417 帧全员预测显示，无输入锁定。MPPI P50/P95/最大约 `50.461/56.235/58.696 ms`，仅为该轮观测。日志 `/tmp/cane-pedsim-fov-navigation-test.log`，摘要 `/tmp/cane-pedsim-fov-navigation.json`。

Bash/Python 语法、启动器帮助与 `git diff --check` 通过。测试使用独立缓存并仅清理自有轮次，保留用户原 11311 master；临时证据可能被系统清理。下一步由用户按原入口加 `--pedestrian-mode interactive` 体验，地图扩展与统计实验仍按“当前工作与下一步”推进。本次未提交、未推送。

## 最新用户反馈：第三次导航配对心跳超时（2026-10-29）

**直接失败原因已确认，底层中断来源尚未定位，不能宣称修复。** 用户运行 `f70da8c5-0377-4659-8cca-39d45342bcca`：

- `20:04:01.328`、`20:04:13.842`：前两次导航实际到达终点。
- `20:04:18.065`：第三个目标 `(5.466,0.226)`，起点 `(-3.558,-1.474)`；初始 A* 在 `18.193–18.198` 完成，约 5 ms。
- `18.207`：走廊出现 `OUTPUT_CERTIFICATE` 和短暂 `INVALID_STATIC_CORRIDOR`；`18.224` 已得到 7.77 m 的认证前缀。前两次成功导航也有瞬时走廊失败，不能把它当作持续锁定的根因。
- `19.578`：规划器报告 `DYNAMIC_DATA_INVALID`；`19.638`：桥接首次故障为 `paired simulation heartbeat expired`，随后进入 `FAULT_RESTART_ROUND`。之后新起点/目标不能清除此锁定；需重启该轮仿真。该日志不足以区分仿真未发布、消息传输/处理延迟或配对失败，也不足以将问题归因于新视野规则。

复现尝试使用默认 `crowd_5`、`interactive`、独立私有 master 与关闭场景记忆，按用户三组位置、朝向、选目标等待时间依次运行：首次三次通过，再连续九次通过，最后启用完整 RViz 三次通过，共 15 次实际到达，没有触发锁定。九次轮中原始行人/输入证据最大接收间隔约 0.176 s，桥接输出约 0.275 s；RViz 轮分别约 0.151/0.167 s，均低于 0.5 s。未能获得可重复的失败样本，因此没有针对猜测修改算法或保护逻辑。

本次仅补充桥接首次故障证据日志：配对与里程计的 ROS 时间年龄、两路原始消息时间戳年龄/单调时钟接收年龄、watchdog 调用间隔、待配对队列大小、场景版本及上游 guard。原始戳、配对、0.5 s 时效和故障锁定规则不变；日志帮助下一次区分源头断流与收到后未配对，不代表已经解决间歇故障。

验证：`test_pedsim_interface.py` 原有 15 项通过，包含乱序、版本切换、空帧、实际回调丢失锁定与禁止恢复；Python 语法和 `git diff --check` 通过。仅 Python 日志与此记录有新增修改，无需重建 C++。自有测试进程已清理，用户原 11311 master 保留。

原用户日志快照为 `/tmp/cane-pedsim-user-latest-fault.ljlfibmz/`；三组连续导航复现脚本为 `/tmp/cane_pedsim_latest_fault_repro.py`，带 RViz 版本为 `/tmp/cane_pedsim_latest_fault_rviz_repro.py`。接收时间线为 `/tmp/cane-pedsim-latest-fault-{timeline-green,nine-timeline,rviz-timeline}.json`，接口测试日志为 `/tmp/cane-pedsim-latest-fault-interface-test.log`。临时证据可能被清理。

下一步：若再次出现，优先读取同一轮的 `pedsim fault evidence` 与上游 guard，结合消息接收年龄和待配对队列定位；保留真实失败样本后再做定向修复。当前没有证据支持扩大视野、削弱输入检查或自动清除锁定。本次未提交、未推送。

## 本次提交范围（2026-10-29）

按用户要求，将 220° 行人视野限制、朝向保持、RViz 视野显示、上游可重放补丁/manifest、相关验证与首次超时诊断日志一并归档到 `mpc-lfpc-prototype`，并同步到 `origin/mpc-lfpc-prototype`。此前各节“未提交、未推送”描述验证结束时的历史状态；实际提交及远端同步状态以 Git 为准。本次不包含间歇心跳超时的根因修复，该问题继续按上一节跟踪。

## 新证据：ROS 时间与单调时钟推进不一致（2026-10-29）

用户在提交 `5f3b0b5` 后再次触发，运行 `197da7d6-4c54-4fc3-bbe9-f86ca92d179f`，场景为 blank/crowd_5、interactive。本轮已结束，原日志已在私有测试之前快照至 `/tmp/cane-pedsim-heartbeat-fault-8ckjbxln/`。

- 起点 `(4.836,3.410)`、朝向 `-2.472`，目标 `(-5.072,-2.470)`、朝向 `-2.431`。
- 初始 A* 约 8 ms，此后周期 A* 约 7–12 ms 正常接受；`20:39:27.720` 仍有步态进展。
- `20:39:28.835` 桥接首次故障为 `robot odometry heartbeat expired`。故障证据：`pair_age_ros=1.158`、`odom_age_ros=1.116`、`input_stamp_age_ros=1.176`、`agents_stamp_age_ros=1.159`，而 `input_rx_age_wall=0.076`、`agents_rx_age_wall=0.076`、`watchdog_gap_wall=0.100`，两路待配对队列均为零。
- `20:39:28.859` 上游进一步进入 `LATCHED_TF_STALE`，TF 年龄约 1.140 s。规划器持续 `DYNAMIC_DATA_INVALID` 是故障后的结果。

**定位结论：本轮出现 ROS 时间与单调时钟的推进差异，不能按普通长回调阻塞或单纯消息断流解释。** 若前一次 watchdog 已经运行且尚未报错，配对年龄应不超过 0.5 s；仅隔 0.10 s 单调时间后年龄为 1.158 s，意味着期间 ROS 时间比单调时间至少多推进约 0.56 s。结合两路仅 0.076 s 前收到并完成配对，特征接近约 1.08 s 的额外时间推进。这说明异常发生在时间基准上；现有日志不能进一步区分 WSL/宿主暂停、系统校时或其他时间源异常，不应直接宣称某个系统服务已被证明是根因。

验证与边界：

- 按相同起终点与等待时间启用完整 RViz 连续导航三次，全部实际到达，无故障。本轮新增约 2167 次系统时间、ROS 时间、单调时钟对照采样，最大相邻推进差约 30 微秒，没有出现用户轮次的秒级差异；未能自然重现环境时间异常。
- `/tmp/cane_pedsim_clock_evidence_replay.py` 通过真实 Bridge 回调做受控对照：正常 0.076 s 无故障；ROS 额外前跳 1.082 s 时复现相同 `robot odometry heartbeat expired`、配对年龄 1.158 s、odom 年龄 1.116 s、接收年龄 0.076 s、watchdog 0.10 s、空配对队列；普通实际停顿 1.158 s 时接收年龄和 watchdog 间隔也随之超过 1 s，与用户证据不同。该重放验证报错机制，不等于复现了 WSL 系统事件本身。
- 上游 `createMsgHeader()` 每批使用 `ros::Time::now()`，桥接保留原戳；本轮没有发现积压待配对消息。没有修改算法、保护阈值、原始戳或系统时间配置。

后续修复方向：在自有轻量仿真中考虑统一由单调时钟驱动 `/clock`，所有仿真节点使用同一 ROS 仿真时间，隔离操作系统校时/宿主暂停造成的时间不连续；需要覆盖启动就绪、持续导航、真实输入丢失仍锁定及自有进程清理。不要仅在桥接里改用接收时间或重打时间戳，否则上游 TF 和规划端时效仍可能失败。此方向尚未实现，当前轮需重新启动才能恢复。此前不同故障样本仍保留各自结论。

本轮仅更新排查记录，未改生产代码、未提交或推送。测试使用独立缓存和私有端口 `46443`，自有 ROS 进程已结束，保留用户原 11311 master。原始对照输出 `/tmp/cane-pedsim-clock-evidence-replay.log`，导航时间线 `/tmp/cane-pedsim-clock-fault-rviz-timeline.json`，完整启动日志 `/tmp/cane-pedsim-clock-fault-rviz-crowd_5.log`；临时证据可能被清理。

## 修复：自有 pedsim 轮次统一使用单调仿真时钟（2026-10-29）

**已实现并验证，可按原命令重新启动测试。** 针对上节时间基准异常，不修改操作系统时钟或原 0.5 s 输入时效规则。

- `pedsim_round.py` 在验证新私有 master 的 PID/token 后先设置 `/use_sim_time=true`，启动 `pedsim_monotonic_clock.py` 并等首次发布确认，再启动 rosout 与规划、pedsim、桥接和 RViz。时钟只在启动时取一次可读的系统 epoch，之后按 `time.monotonic_ns()` 以 1 倍速推进，循环采用独立的 1 ms sleep；不使用依赖自身 `/clock` 的 ROS Rate。
- 时钟进程纳入自有进程监控与清理；退出即结束本轮，不退回系统时间、不复用外部 master。`run_owned` 仍允许独立测试调用者不提供时钟；标准 blank/office 启动器默认提供。直接手动 launch 需在启动前自行统一时间配置，Gazebo 和实机不接入此时钟。
- 离散仿真 tick 下，FSM 与执行回调曾在同一个时间戳重复发布不同里程计消息，真实测试触发 `conflicting duplicate batch`。planner-3 轻量仿真现在每个相同仿真戳只发布一个权威位姿，后续 tick 的移动/STOP/等待心跳保留，不推进 LFPC。对应到达测试明确在下一 tick 检查独立的停止发布。
- 真实连续导航还复现了 `/clock` 比观测晚到导致 `invalid/stale agent batch`。桥接仅在仿真时间模式、消息比本地时钟超前不超过 50 ms 时，最多等待 50 ms 单调时间，让本地时钟回调赶上后执行原校验；消息戳不改，持续未来戳、冻结时钟、真实回退和过期仍锁定。新增三项接口回归分别验证正常乱序、永远未赶上的未来戳与真实回退。
- 新增 `rosgraph_msgs` 运行依赖、脚本安装入口及运行说明。本轮未改 pedsim 上游源/补丁/manifest，无需重建该独立依赖。

验证证据：

1. `pedsim_navigation_smoke.py crowd_5 --repeat-goals --clock-jumps` 是本次实际 ROS 链路回归。测试只给自有子进程注入 ROS 系统时间读取偏移：C++ 拦截系统时间模式的 `ros::Time::now()`，Python 修改壁钟读取，不改内核时钟或底层线程等待。运行期依次设为 +1.082 s、-0.8 s、恢复 0，独立探针确认偏移确实生效。测试辅助编译需要 C++ 编译器与 rostime 的 pkg-config。
2. 旧启动器反事实对照在私有端口 `47149` 注入第一次前跳后稳定失败，报告 `pedsim bridge fault`；同一测试的修复后轮次端口 `45139` 通过全部三次偏移并完成两次真实到达，第二次包含复位起点后等待 2 s 选目标。
3. 成功导航后主动 SIGSTOP 经父进程归属校验的 planner，桥接按原规则进入 `FAULT_RESTART_ROUND`，上游 TF 失效锁定，桥接没有空帧回退；恢复进程后锁定未被清除。随后终止自有时钟，启动器以失败状态结束并清理本轮 master/节点。末尾故障是有意断输入测试，不能算作正常导航误报。
4. 按用户最新日志中的起终点与朝向启用完整 RViz 连续三次导航，私有端口 `45067`，全部实际到达，无故障锁定。首轮等待第一帧时有原有的预期 `DYNAMIC_DATA_INVALID`，后续没有持续输入故障。
5. `catkin build plan_manage --no-deps --no-status -j4 -p1 --make-args kin_replan_node test_sim_odom_stop` 通过；测试目标更新后单独构建也通过。六项 manager 回归通过，覆盖实际停止/到达、最后安全步、重设起点后的静止心跳、模式和初始化保护。Python 配对/接口回归增至 18 项，全部通过。

本轮 MPPI P50/P95/最大约 `52.105/58.586/66.136 ms`，仅代表这一轮，不能作为实时性能保证。早期使用 libc 全局时间拦截的测试会影响信号量的绝对等待期限，已改为只拦截 ROS/Python 的系统时间读取，并以旧启动器对照重新验证；该早期测试不作为最终验证依据。

证据位于 `/tmp/cane-pedsim-clock-{before.txt,legacy-control.txt,after.txt,build.log,test-build.log,interface.txt,manager-tests.log}`、`/tmp/cane-pedsim-repeat-goals-clock-jumps-crowd_5.json` 与 `/tmp/cane-pedsim-monotonic-clock-rviz.txt`。原用户快照保留。临时证据可能被清理。

用户可沿用 `start_pedsim_navigation.sh start --world blank --scenario crowd_5 --pedestrian-mode interactive`，结束已锁定旧轮后重新启动，无需额外参数。自有测试进程全部清理，用户原 11311 master 保留。下一步按用户实测继续观察新一轮日志，后续同源长走廊/交叉地图和统计实验仍按本文件阶段计划进行。

按用户要求，本次单调仿真时钟修复、回归测试及说明一并纳入 `mpc-lfpc-prototype` 的本地提交，未推送云端。前文“未提交”描述排查阶段的历史状态；实际提交状态以 Git 为准。

## 共享规划器修复：远处参考失效不再冻结安全当前步（2026-10-29）

用户在原确定性行人入口复现持续停止。原因位于两种入口共用的 planner 3：
地图变化后，执行端沿远处全局参考重建走廊，远段失败使旧步不能完成，也无法
进入下一次 MPPI。现改为完整认证当前步的实际剩余质心轨迹，保留当前人杖、
行人分离、质心与落脚点包含、时效和时间同步检查；安全完成原步后在边界接入
新 A* 与 MPPI，真实当前步受阻仍冻结原相位，不引入步中转向或重置。

原用户起终点与行人配置在完整 RViz 下已实际到达；pedsim 五人交互模式在私有
端口 `47611` 连续两次实际到达，包含重设起点后的静止等待，无输入锁定。
pedsim 源码、补丁、manifest、编辑器及交互/独立模式均未改。细节和回归证据见
[逐段执行验证](PLANNER3_ASYNC_EXECUTION_VALIDATION.md#2026-10-29远处参考失效冻结当前步的修复)。
下一步由用户沿原命令实测，地图扩展和统计实验仍按前文阶段计划推进。
本阶段修复、回归测试及说明按用户要求提交到 `mpc-lfpc-prototype`，并同步至
`origin/mpc-lfpc-prototype`；实际提交和远端同步状态以 Git 为准。
