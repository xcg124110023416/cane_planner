# ROS1 办公室社会力行人仿真可行性

核对日期：2026-10-29。范围为上游 README、动力学源码、消息和场景接口；本次调研未启动或修改 ROS 节点，也未修改规划代码。上游引用固定到当日读取的提交，不能据此声称已经完成闭环仿真验收。

本文保留调研依据。后续实施状态、已验证入口及剩余阶段见 [实施进度](PEDSIM_IMPLEMENTATION_PROGRESS.md)。

## 结论

**优先继续使用 `srl-freiburg/pedsim_ros`，先完成单行人与 planner 3 的闭环。** 它已有 ROS1 接口，支持同场景中的墙、多人和外部控制的机器人，Gazebo 不是必需依赖。本仓库已有配套补丁和单人桥接；更换仿真器不能自动解除 planner 3 的单人数限制。需要轻量、可在线编辑的多人动力学时，`robotics-upo/lightsfm` 适合嵌入自有 ROS1 节点；`robotics-upo/hunav_sim` 的机器人交互更丰富，但当前是 ROS2，接入 Noetic 的成本最高。[P0][P1][P4][L0][H0]

社会力模型本来包含**目标吸引/期望速度恢复项、墙排斥项、人与人交互项**；群组模型还可包含凝聚吸引项。观察到行人朝目标走，或合力短时朝向机器人，不能直接推出“排斥力符号反了”。应分别检查每一项、坐标系和障碍输入。[P1][L1]

## pedsim_ros：能力与接口边界

核对版本为 `master@de3772630ae174b278da00bcf26e5cf6c2db5ab2`。它使用 catkin、roscpp、tf 和 dynamic_reconfigure；官方 README 仍列旧 ROS 版本及 Melodic，**没有声明 Noetic 已通过测试**。本机已有匹配补丁的源码和构建产物，应与官方支持范围分开记录；本轮没有运行真实导航闭环。[P0][P5]

| 需求 | 上游已具备的能力与接入要求 |
| --- | --- |
| 单个或多个行人 | XML `<agent x="…" y="…" n="…" dx="…" dy="…" type="0">` 定义人数和出生分布；可写多个 agent 块。精确放置一人使用 `n="1" dx="0" dy="0"`。有个实现细节：`publishAgents()` 在总 agent 数少于 2 时不发布，因此本需求应同时放置一个 `type="2"` 机器人 agent。[P2][P4] |
| 对墙响应 | `<obstacle x1="…" y1="…" x2="…" y2="…"/>` 定义线段；`obstacleForce()` 找最近线段上的最近点，按 `pedestrian - closestPoint` 向外施力。只取最近障碍，不是完整刚体碰撞引擎。[P1][P2] |
| 对人响应 | 依据相对位置和速度计算减速及侧向避让；核心查询半径为 10 m。支持群组附加行为。[P0][P1] |
| 对机器人响应 | XML `type="2"` 为机器人；`robot_mode=1`（TELEOPERATION）从 `frame_id → robot_base_frame_id` 的 TF 更新真实位置和速度，机器人不由社会力驱动。它作为邻居参与行人社会力；机器人距离项会除以默认缩放因子 5，扩大其影响。`SOCIAL_DRIVE` 则让机器人也由模型移动，不适合替代现有 LFPC 执行器。[P1][P3][P4][P10] |
| 速度与目标 | `<waypoint id="…" x="…" y="…" r="…"/>` 和 `<addwaypoint id="…"/>` 定义循环目标序列。普通行人初始最大速度由均值 1.34、标准差 0.26 m/s 的正态分布生成；C++ 有 `setVmax()`，但原版 XML parser 没有读取每人速度属性，dynamic_reconfigure 也没有每人速度/目标接口。指定固定速度需要本仓库部署补丁或另做接口。[P1][P2][P3][P6] |
| 运行时配置与增删 | 可动态调力权重、障碍衰减尺度、仿真参数和暂停状态；XML `<source>` 配合定时器产生人，SINK waypoint 可删除到达的人。**这不等于任意在线点选增删/改目标**：`pedsim_srvs` 虽定义 Set/GetAgentState，当前 simulator 实际只注册 pause/unpause 服务，没有绑定这些服务。[P2][P4][P6][P7][P8] |
| RViz/Gazebo | 核心是二维状态仿真；输出 `simulated_agents`（含 id、pose、twist、forces）、`simulated_walls` 等消息，RViz 可用于显示，Gazebo 是可选扩展。直接启动核心节点可不启动示例中的机器人控制器、visualizer、RViz；核心自身仍依赖 Qt5Widgets，并创建 QApplication，无显示环境需要处理 Qt 平台配置。[P0][P4][P5][P9] |

### 同一静态办公室地图怎样对齐

上游核心从 XML 读取线段障碍；本次核对的初始化与循环没有订阅 occupancy map 的路径，不能假设启动了 `/map` 就会自动感知办公室墙体。[P2][P4] 应从**当前规划器采用的同一份静态地图**导出墙和家具轮廓，转成 XML 线段，正确使用地图 origin、resolution 和坐标变换。机器人 TF、行人、目标及线段必须处在同一全局坐标系中。

行人目标需要布置在可通行的房间、走廊与门口。核心目标力直接指向当前 waypoint，它并未调用本项目 A* 为行人做穿房间寻路；应提供经过门口的中间目标，避免把“朝墙后的目标走、在墙边停住”误判为排斥错误。[P1][P2][P3]

诊断消息也需按源码解释：上游 `AgentForce.desired_force` 实际填入 `getDesiredDirection()`，不是完整目标加速度；不能仅按消息字段名称重建合力。[P4]

## 两个替代方案

| 方案 | 已核实能力 | 对当前 Noetic 项目的代价 |
| --- | --- | --- |
| [lightsfm](https://github.com/robotics-upo/lightsfm)，`master@b30327c` | 不依赖 ROS 的 C++ header-only 库。具目标力、墙力、人际力、群组力；`Agent` 可配置速度、半径、目标列表、循环目标和力参数，提供单人/多人 `computeForces()`、`updatePosition(dt)`。障碍由调用者填点集或实现 `Map::getNearestObstacle()`；机器人可作为外部更新的另一个 agent 参与作用。[L0][L1][L2] | 无须 Gazebo/RViz/ROS2，适合自建小型 ROS1 动力学节点。需要自己做 ROS 消息、定时积分、地图查询、机器人状态输入、可视化及运行时服务。在线增删/改速度/改目标可以通过管理 Agent 容器实现，但库没有现成 ROS 服务或交互界面。[L0][L1] |
| [HuNavSim](https://github.com/robotics-upo/hunav_sim)，当前默认 `v2.0@35c2d81` | 官方声明测试于 ROS2 Humble；核心基于 lightsfm，可经 wrapper 接不同仿真器。YAML 配置人数、速度、半径、目标与行为，RViz2 面板配置；支持 regular、impassive、surprised、curious、scared、threatening 等机器人反应。`compute_agents` 请求携带机器人和全部行人，`Agent.closest_obs` 接收障碍点；README 明确 `map` 配置当前未使用。[H0][H1][H2] | ROS2/rclcpp/ament、行为树依赖，不能直接作为 catkin/Noetic 包。若保留 ROS1 planner，需要 ROS1/ROS2 通信适配、自定义消息/服务转换和仿真 wrapper；纯二维场景也要自己供障碍。Gazebo 是可选 wrapper，不是核心强制依赖。[H0][H1][H3] |

HuNavSim 有 `reset_agents` 服务，但当前实现只是更新内部状态；更新路径没有同步删除不再出现的 id，目标覆盖代码也被注释。不能将该服务或 RViz2 配置面板直接等同于“仿真中任意热增删并修改目标”，还需验证/扩展 wrapper、行为树和内部容器生命周期。[H2][H3]

## 本仓库的当前边界与已发现问题

- 现有入口、补丁依赖和轮次控制见 [README 的可选 pedsim 场景](../README.md#可选-pedsim-场景) 及 [部署说明](../plan_manage/config/pedsim/DEPLOYMENT.txt)。应继续遵守 manifest 对应的补丁版本；普通上游不是现有部署的直接替代品。
- pedsim 能模拟多人，而当前 [桥接器](../plan_manage/scripts/pedsim_single_bridge.py)、[规划管理器](../plan_manage/src/planner_manager.cpp)、[走廊生成器](../path_searching/src/convex_corridor.cpp) 约束为零或一人；[传给全局 A* 的障碍](../plan_manage/src/planner_manager.cpp#L1435) 还只取第一个行人多边形。多人闭环需同步扩展输入验证、所有行人多边形、时间预测与碰撞检查，不能只删掉桥接器人数检查。
- [PedestrianPolygon](../path_searching/src/pedestrian_polygon.cpp) 的本地力估计用于生成规划中的行人形状，未驱动 [pedestrian_sim.py](../plan_manage/scripts/pedestrian_sim.py) 的行人动力学；需区分“形状朝向异常”与“真实仿真人受到吸引”。pedsim 的真值合力适合仿真诊断，不应直接喂给规划器冒充仅由观测估计的力。
- 本轮直接调用生产 C++ 模型复现了一个**局部裁剪域边界被当墙**的问题：物理世界空地图、`system_gain=0`，人 `(0,-1.2)`、机器人 `(0,0)`、人速度 `(0,-0.4)`；裁剪域为 `[-1,1]²` 时估计力为 `(0,+1)`，朝机器人；扩大到 `[-10,10]²` 后力为零。原因是 [域外 clamped 分支](../path_searching/src/pedestrian_polygon.cpp#L78) 使用指向裁剪域内的向量作为墙法向。它确实能产生表观吸引，但缺少当时运行帧，不能证明它就是某张截图的唯一原因。已有独立机器人/墙排斥用例通过，也不能排除这个输入边界问题。

推荐顺序：先修正并验证规划端裁剪边界语义，再用同一地图的 pedsim 单人场景验证“机器人停留/移动时行人会响应”；之后扩展多人完整链路。单人验收至少记录地图叠合、行人速度/目标、机器人 TF、新鲜观测和力分量，覆盖对穿、横穿、超越及墙边场景。是否成功应由实际轨迹和避碰结果判断，不能仅看社会力箭头。

上述裁剪域假力复现对应修正前版本；后续阶段 2 已移除该边界项，并增加力分量与观测速度显示，验证证据见 [实施进度](PEDSIM_IMPLEMENTATION_PROGRESS.md)。

## 一手资料

GitHub API 本次受限返回 403；raw 文件和 git refs 可访问，以上源码已直接读取。以下链接固定到读取的提交。

[P0]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/README.md
[P1]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/3rdparty/libpedsim/src/ped_agent.cpp
[P2]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/src/scenarioreader.cpp
[P3]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/src/element/agent.cpp
[P4]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/src/simulator.cpp
[P5]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/CMakeLists.txt
[P6]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/config/PedsimSimulator.cfg
[P7]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_srvs/CMakeLists.txt
[P8]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/src/scene.cpp#L543
[P9]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/src/simulator_node.cpp#L36
[P10]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/include/pedsim_simulator/utilities.h#L38
[L0]: https://github.com/robotics-upo/lightsfm/blob/b30327cca189af2fb90443a5d0040cceb46d7195/README.md
[L1]: https://github.com/robotics-upo/lightsfm/blob/b30327cca189af2fb90443a5d0040cceb46d7195/include/sfm.hpp
[L2]: https://github.com/robotics-upo/lightsfm/blob/b30327cca189af2fb90443a5d0040cceb46d7195/include/map.hpp
[H0]: https://github.com/robotics-upo/hunav_sim/blob/35c2d81fc0e8166a3d124558fd718dd03e8a914c/README.md
[H1]: https://github.com/robotics-upo/hunav_sim/blob/35c2d81fc0e8166a3d124558fd718dd03e8a914c/hunav_msgs/msg/Agent.msg
[H2]: https://github.com/robotics-upo/hunav_sim/blob/35c2d81fc0e8166a3d124558fd718dd03e8a914c/hunav_agent_manager/src/agent_manager.cpp
[H3]: https://github.com/robotics-upo/hunav_sim/blob/35c2d81fc0e8166a3d124558fd718dd03e8a914c/hunav_agent_manager/src/bt_node.cpp

## 改用 pedsim 自带场景（续查）

**可以不用办公室 PCD，改用上游场景的几何。** 但 pedsim 自带的是 XML 墙线段、waypoint 和 agent 配置，不是能直接交给当前 planner 的 PCD 或 ROS occupancy 地图；需要把同一组墙线段转换成规划器的地图输入。[S0][P2]

本机 `pedsim_simulator/scenarios/` 的 19 个 XML 均与上游固定提交一致，实际文件为：[S0]

- 根目录：`airport.xml`、`airport2.xml`、`airport_activities.xml`、`airport_activities2.xml`、`blank_world.xml`、`corridor.xml`、`scene1.xml`、`scene2.xml`、`scene3.xml`、`scene4.xml`、`singleagent.xml`、`small_airport.xml`、`social_contexts.xml`。
- `contexts/`：`anti_flow.xml`、`flow.xml`、`queueing.xml`、`shopping.xml`、`standing.xml`。
- `srl_experiments/`：`nav_test1.xml`。

优先考虑以下三个基础场景。尺寸按 XML 墙端点计算，人数为原版配置，**不是当前 planner 已支持的人数**。

| 场景 | 几何、人数、机器人配置 | 适合怎样使用 |
| --- | --- | --- |
| `blank_world.xml` | `[0,30] × [0,30]`，四面围墙；0 行人、1 个 `type=2` 机器人，出生点 `(14,6)`，目标在围墙内。[S1] | **最适合先做单人。** 添加一人和两个目标即可分别布置对穿、横穿、超越，墙和路径关系容易核查；增加人也无需改变地图。 |
| `scene4.xml` | `[0,40] × [0,10]` 的长方形通道；四组各 15 人，共 60 人，按上下两侧 waypoint 循环行走；另有机器人 `(1.03,6.97)`。[S2] | 走廊双向流、对穿和超越。当前先删减为一人；多人链路完成后再恢复人群。原版宽 10 m，没有窄门。 |
| `scene3.xml` | 十字形通道，墙包围盒 `x=0..110, y=5..115`；121 行人、1 个机器人 `(5,60)`，包含横向与纵向流。[S3] | 横穿及多人压力测试。尺寸和人数都较大，需缩放几何或扩大规划地图，不能原样放入当前 40 m 地图。 |

瓶颈建议在 `blank_world` 或 `scene4` 中加两段墙，明确设置门宽，形成可解释的单人/多人实验。官方 `airport_activities2.xml` 虽有开口和排队配置，但原版墙范围为 `60 × 30 m`、75 行人加 1 机器人，顶边开口 `x=5..15` 达 10 m，仍需调整才是人尺度窄门实验。[S4]

另外，文件名不能代表可直接验收：`singleagent.xml` 确有 1 行人加 1 机器人，但人是 `type=3`，机器人目标 `(22,27)` 落在其 `20 × 15 m` 封闭围墙外；`corridor.xml` 是 `30 × 15 m` 的 source/sink 场景，每批生成 8 人，默认生成周期 5 s，机器人目标同样在围墙外。[S5][S6][P4] 上游 `sources_and_sinks.launch` 选的是后者；`simple_pedestrians.launch` 默认选 `social_contexts.xml`，并非单人场景。[S7][S8]

上游并非完全没有点云转换：`pedsim_sensors` 的墙传感器将 `simulated_walls` 转成 `sensor_msgs/PointCloud`，但带 XY 各 ±0.5 m、Z 为 0..1 m 的随机采样，global 输出也受视野限制，且不是 `PointCloud2`。[S9] 为避免规划器与行人对墙的理解不一致，本实验宜从 XML 确定性栅格化，明确墙厚，再生成 PCD 或 `PointCloud2`，不直接把该近似传感器当完整静态地图。

切换需要同时对齐 [仿真地图输入](../plan_manage/launch/include/simulation.launch)、[可选静态地图与地图范围](../plan_manage/launch/sim_kin_replan.launch)、[SDF 坐标范围](../plan_env/src/sdf_map.cpp#L40) 和 [pedsim scene_file](../plan_manage/launch/pedsim_single_source.launch)。后续阶段 1 已为 `blank_world` 增加共享几何导出和启动参数：地图、墙、人和目标统一采用居中坐标，适配默认 `40 × 40 m` 范围；完整步骤见 [USAGE.txt](../plan_manage/config/pedsim/USAGE.txt)。较大官方场景仍需另行适配地图尺寸和人数。

[S0]: https://github.com/srl-freiburg/pedsim_ros/tree/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/scenarios
[S1]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/scenarios/blank_world.xml
[S2]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/scenarios/scene4.xml
[S3]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/scenarios/scene3.xml
[S4]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/scenarios/airport_activities2.xml
[S5]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/scenarios/singleagent.xml
[S6]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/scenarios/corridor.xml
[S7]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/launch/sources_and_sinks.launch
[S8]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_simulator/launch/simple_pedestrians.launch
[S9]: https://github.com/srl-freiburg/pedsim_ros/blob/de3772630ae174b278da00bcf26e5cf6c2db5ab2/pedsim_sensors/src/pedsim_sensors/obstacle_point_cloud.cpp
