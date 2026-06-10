# FAST-LIO-Localization-QN 话题输出与导航对接

## 概述

定位包基于 FAST-LIO + Quatro + Nano-GICP 实现地图匹配定位，输出纠正后的里程计、位姿、TF 和点云。

---

## 输出话题一览

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/corrected_odometry` | nav_msgs/Odometry | 同雷达帧率 | **纠正后的里程计**，定位核心输出 |
| `/pose_stamped` | geometry_msgs/PoseStamped | 同雷达帧率 | 当前实时位姿（position + orientation） |
| `/tf` (map → body) | TF | 同雷达帧率 | 坐标变换，导航系统必须 |
| `/corrected_current_pcd` | sensor_msgs/PointCloud2 | 同雷达帧率 | 当前帧点云（已变换到 map 坐标系） |
| `/saved_map` | sensor_msgs/PointCloud2 | 启动时一次 | 加载的静态地图点云 |
| `/corrected_path` | nav_msgs/Path | 匹配时更新 | 纠正后的轨迹 |
| `/ori_path` | nav_msgs/Path | 同帧率 | 原始里程计轨迹（未纠正） |
| `/map_match` | visualization_msgs/Marker | 匹配时更新 | 匹配连线可视化 |

---

## 导航对接

### 1. 里程计（必须）

```xml
<!-- 在 move_base 或导航 launch 中 -->
<param name="odom_frame" value="odom"/>
<remap from="odom" to="/corrected_odometry"/>
```

`/corrected_odometry` 输出格式：
```
header:
  frame_id: "map"
  stamp: 当前时间
child_frame_id: "body"
pose:
  pose:
    position: {x, y, z}
    orientation: {x, y, z, w}
```

### 2. TF（必须）

定位包自动发布：
```
map → body
```

如果你的导航系统期望 `map → odom → base_link` 的三级 TF 链，需要额外写一个节点将 `corrected_odometry` 拆分为：
```
map → odom    (来自定位包的纠正)
odom → base_link  (来自原始里程计)
```

或者直接将 `base_frame` 设为 `body`，让导航系统直接使用 `map → body`。

### 3. 点云避障（可选）

`/corrected_current_pcd` 已在 map 坐标系下，可直接用于 costmap：

```xml
<remap from="obstacles" to="/corrected_current_pcd"/>
```

---

## 关键参数（config.yaml）

```yaml
basic:
  map_frame: "map"                    # TF 和所有话题的 frame_id
  map_match_hz: 2.0                   # 匹配频率，2Hz 足够
  saved_map: "/path/to/result.bag"    # 建图生成的地图文件
  visualize_voxel_size: 0.2           # 可视化点云体素大小

keyframe:
  keyframe_threshold: 1.0             # 关键帧间距 (m)

match:
  match_detection_radius: 20.0        # 匹配搜索半径 (m)
  quatro_nano_gicp_voxel_resolution: 0.2

nano_gicp:
  icp_score_threshold: 1.0            # Score < 此值才接受匹配
```

---

## 匹配状态判断

通过终端日志或订阅 `/map_match` 话题判断：

| 日志 | 含义 |
|------|------|
| `Map matching accepted. Score: xxx` | 匹配成功，Score < 1.0 |
| `Loop closure rejected. Score: xxx` | 匹配失败，Score >= 1.0 |
| 无匹配输出 | 搜索半径内无候选关键帧 |

---

## 使用流程

```
1. 建图: roslaunch fast_lio_sam_qn run.launch lidar:=livox rviz:=false
2. 填入地图路径到 config.yaml 的 saved_map
3. 定位: roslaunch fast_lio_localization_qn run.launch lidar:=livox rviz:=false
4. 导航: 订阅 /corrected_odometry + /tf + /corrected_current_pcd
```
