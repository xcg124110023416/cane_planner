# pedsim 社会力场景编辑

`start_pedsim_navigation.sh start --world blank` 启用 `SocialPedestrianPanel`、三种地图工具和真实 pedsim 编辑服务。旧确定性单人面板与办公室入口独立。最多五人，空场景合法；配置的起终点在 `world` 下，需离场地墙至少 0.5 m，行人起点间距至少 0.55 m。

## 操作与运动

- “新增”后以“行人起点”（P）点击并拖拽设置起点、朝向；默认目标在朝向前方 3 m。选中已有行人时，该工具移动起点并旋转等长路线。
- “行人终点”（K）单独设置目标。“选择行人”（U）点击当前行人 1 m 内或通过下拉框选中；黄色标记是选中对象。
- 期望速度为 0–3 m/s，社会力决定实际速度与轨迹。速度 0 表示原地静止。“参考单程时间”仅通过速度、朝向计算终点，仍需点击“应用并全部复位”。
- 到目标 0.2 m 内停止、返回一次或持续往返；返程也由社会力运动。暂停保留当前位置，继续从当前位置运行；已结束者需复位后重走。
- 地图起终点变更、应用、增删、清空、加载或复位会使**全部行人复位等待**。机器人和保护状态保留。勾选并应用“新机器人目标时复位并启动全部行人”后，新导航目标触发一次行人复位启动。

暂停只冻结行人，仿真继续发布零速度真观测，空场景继续发布新鲜空帧。不能调用 pedsim 全局暂停代替此按钮，否则会失去观测心跳。

同一轮内可重复设置机器人起点和终点：机器人到达后用 `2D Pose Estimate` 重新放置，再用 `2D Nav Goal` 发送目标。等待选择目标时，已初始化的 LFPC 状态持续发布真实静止位姿；这不推进步态，也不依赖“已到达”标志。真实输入中断的 0.5s 保护仍保留。

## 保存、加载与自动恢复

面板保存已应用的配置；未应用表单、瞬时位置、仿真时间不写入。加载采用全量校验后替换，复位等待。

默认在 `${ROS_HOME:-~/.ros}/cane_planner/social_editor/<hash>.yaml` 自动记忆，按场地、预设名称、稳定源配置上下文区分；与旧单人面板缓存隔离。首次默认启动不覆盖缓存。有效编辑、加载、清空后原子写盘，写失败显示但不撤销已成功的仿真操作。坏缓存警告后回到默认配置。

`--fresh-scene` 跳过自动记忆的读取和写入；`--editor-state-file PATH` 指定独立缓存。记忆恢复配置后复位等待，不恢复中途状态。空场景同样保存并恢复。当前只支持规则 blank 场地。

```yaml
format: cane_pedsim_scene_v1
frame_id: world
world: blank
start_on_goal: true
pedestrians:
  - id: 1
    start: [0.0, -3.0, 1.5707963267948966] # x, y, yaw（弧度）
    goal: [0.0, 6.0]
    desired_speed: 0.45
    end_behavior: return # stop | return | repeat
```

允许 `pedestrians: []`；文件限制 64 KiB，拒绝重复 ID、非有限数、不匹配的坐标系、越界端点、重叠起点和超人数。

## ROS 接口与状态

| 接口 | 含义 |
| --- | --- |
| `/pedsim_scene/start_pose`, `/pedsim_scene/goal_pose` | `PoseStamped`，仅接受 world 平面位姿 |
| `/pedsim_scene/select_point` | `PointStamped`，world 下选人 |
| `/pedsim_scene/control` | `ControlSocialScene`，select/apply/delete/clear/reset/start/pause/load/export |
| `/pedsim_scene/state` | `SocialSceneState`，配置、上游实际位置/阶段、版本、就绪状态及记忆错误 |
| `/pedsim_scene/ready` | 上游完成配置确认后就绪；首轮释放仿真的条件之一 |
| `/pedsim_scene/visualization` | 路线、起点方向、目标、当前身体、编号/速度/阶段；仅供编辑显示 |
| `/pedsim_simulator/edit_scene` | 上游 `EditScene`，编辑节点专用，不应绕过编辑器直接改场景 |

控制请求携带 `expected_revision`，成功回执返回新版本。周期状态不会覆盖未应用表单；地图操作、选择切换和加载会刷新选中配置。上游保留机器人和保护历史，只替换普通行人和其目标。场景版本变更后桥接丢弃旧批次，仍用原始机器人时间、位置和朝向做精确配对。外部修改版本、真实依赖超时或保护锁定需要结束整轮并重新启动，编辑不能清除故障。

规划器只接收桥接输出的位置、速度和固定尺寸；编辑预览、目标、仿真阶段及上游真实社会力均不进入规划输入。规划端继续使用当前空间走廊和 CV 时间同步检查。

## 可重复验证

先加载 ROS、主工作区和固定 pedsim overlay，再构建可选 Qt 测试：

```bash
catkin build plan_manage --no-deps --make-args test_social_pedestrian_panel
python3 plan_manage/test/test_social_scene.py
python3 plan_manage/test/test_pedsim_interface.py
python3 plan_manage/test/pedsim_scene_smoke.py
python3 plan_manage/test/pedsim_navigation_smoke.py crowd_5 --edit-scene
python3 plan_manage/test/pedsim_navigation_smoke.py crowd_5 --repeat-goals
```

真实仿真测试使用空闲私有 master、独立临时缓存并只清理自有进程。Qt 测试通过真实服务验证放置、应用、暂停和文件保存/加载，并输出面板截图；交互拖拽和完整 RViz 操作仍需实际体验。
