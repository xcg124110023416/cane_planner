# Stage 2 Crossing Experiment Commands

本文档用于重复测试 Stage 2 crossing 实验。每组实验建议开 3 个终端：

1. 启动仿真
2. 录制 rosbag
3. 发布固定目标

每次录包时，只需要改 `run01` 为 `run02`、`run03` 等即可。

## Common Goal Command

每一组都先启动仿真，再启动录包，最后运行下面这条固定目标命令：

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && rosrun plan_manage publish_lightweight_fixed_goal.py _scenario:=crossing _goal_publish_duration:=0.0
```

## C: DRF-only

含义：只用当前 MPC/DRF 动态风险场，不启用 Stage 2 interaction。用于观察没有交互决策时是否绕行、是否抢行、路径偏差和 clearance 如何。

注意：这里必须显式关闭 stop advice/enforce 和 CPA。否则它不是“纯 DRF-only”，而会变成“DRF + 安全停顿 + CPA”，容易出现停顿和过大的绕行。

启动仿真：

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch plan_manage sim_kin_replan.launch planner:=3 use_pedestrians:=true pedestrian_scenario:=crossing mpc_w_risk:=2.0 mpc_risk_sigma_y:=0.38 mpc_enable_cpa:=false mpc_enable_stop_advice:=false mpc_enable_stop_enforce:=false mpc_enable_interaction:=false
```

录包：

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && mkdir -p /home/xcg/ws/records/stage2_retest_C && rosbag record --duration=45 -O /home/xcg/ws/records/stage2_retest_C/C_run01.bag /clock /sim_odom /simulation_generator/odom /onboard_detector/dynamic_obstacles_info /mpc/interaction_scene /mpc/interaction_mode /mpc/interaction_debug /mpc/debug_metrics /mpc/stop_advice /mpc/stop_reason /mpc/path /mpc/best_traj /mpc/current_waypoint /mpc/waypoints /cmd_vel_footprint
```

## E0b: Stage 2 Debug-only

含义：启用 Stage 2 scene/debug topic，但不允许 interaction YIELD，不允许 PASS_BEHIND，不启用 social cost。用于确认 interaction 观测层是否污染原始行为。

这组应该和 C 组共享同一套 DRF base，只多打开 interaction debug wrapper。

启动仿真：

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch plan_manage sim_kin_replan.launch planner:=3 use_pedestrians:=true pedestrian_scenario:=crossing mpc_w_risk:=2.0 mpc_risk_sigma_y:=0.38 mpc_enable_cpa:=false mpc_enable_stop_advice:=false mpc_enable_stop_enforce:=false mpc_enable_interaction:=true mpc_interaction_enable_yield:=false mpc_interaction_enable_pass_behind:=false mpc_interaction_enable_social_cost:=false
```

录包：

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && mkdir -p /home/xcg/ws/records/stage2_retest_E0b && rosbag record --duration=45 -O /home/xcg/ws/records/stage2_retest_E0b/E0b_run01.bag /clock /sim_odom /simulation_generator/odom /onboard_detector/dynamic_obstacles_info /mpc/interaction_scene /mpc/interaction_mode /mpc/interaction_debug /mpc/debug_metrics /mpc/stop_advice /mpc/stop_reason /mpc/path /mpc/best_traj /mpc/current_waypoint /mpc/waypoints /cmd_vel_footprint
```

## E1: Spatiotemporal Yield Stage 2

含义：当前 Stage 2 主线。启用时空冲突让行监督层，用机器人未来路径占用和行人预测占用的时间重叠决定是否 YIELD；冲突解除后通常回到 CONTINUE。当前默认不启用 PASS_BEHIND social cost，避免重新引入大绕后问题。

这组保留 interaction stop advice/enforce，因为 YIELD 的输出就是 `/mpc/stop_advice=true`。但仍关闭 CPA，避免把 Stage 2 效果和额外时间冲突代价混在一起。

启动仿真：

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch plan_manage sim_kin_replan.launch planner:=3 use_pedestrians:=true pedestrian_scenario:=crossing mpc_w_risk:=2.0 mpc_risk_sigma_y:=0.38 mpc_enable_cpa:=false mpc_enable_stop_advice:=true mpc_enable_stop_enforce:=true mpc_enable_interaction:=true mpc_interaction_enable_yield:=true mpc_interaction_use_spatiotemporal_yield:=true mpc_interaction_st_horizon:=4.0 mpc_interaction_yield_trigger_time:=2.5 mpc_interaction_robot_radius:=0.25 mpc_interaction_yield_safety_margin:=0.20 mpc_interaction_enable_pass_behind:=true mpc_interaction_enable_social_cost:=false mpc_interaction_enable_behind_corridor:=false mpc_interaction_w_front_pass:=0.0 mpc_interaction_w_behind_corridor:=0.0
```

录包：

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && mkdir -p /home/xcg/ws/records/stage2_retest_E1 && rosbag record --duration=45 -O /home/xcg/ws/records/stage2_retest_E1/E1_run01.bag /clock /sim_odom /simulation_generator/odom /onboard_detector/dynamic_obstacles_info /mpc/interaction_scene /mpc/interaction_mode /mpc/interaction_debug /mpc/debug_metrics /mpc/stop_advice /mpc/stop_reason /mpc/path /mpc/best_traj /mpc/current_waypoint /mpc/waypoints /cmd_vel_footprint
```

## Optional: B0 No Dynamic Intervention

含义：风险场、CPA、stop advice、hard reject、Stage 2 interaction 全部关闭。用于观察无动态干预时的低安全下界。

启动仿真：

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch plan_manage sim_kin_replan.launch planner:=3 use_pedestrians:=true pedestrian_scenario:=crossing mpc_w_risk:=0.0 mpc_enable_cpa:=false mpc_enable_dynamic_hard_reject:=false mpc_enable_stop_advice:=false mpc_enable_stop_enforce:=false mpc_enable_interaction:=false
```

录包：

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && mkdir -p /home/xcg/ws/records/stage2_retest_B0 && rosbag record --duration=45 -O /home/xcg/ws/records/stage2_retest_B0/B0_run01.bag /clock /sim_odom /simulation_generator/odom /onboard_detector/dynamic_obstacles_info /mpc/interaction_scene /mpc/interaction_mode /mpc/interaction_debug /mpc/debug_metrics /mpc/stop_advice /mpc/stop_reason /mpc/path /mpc/best_traj /mpc/current_waypoint /mpc/waypoints /cmd_vel_footprint
```

## Analyze One Bag

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && rosrun plan_manage analyze_mpc_eval.py /home/xcg/ws/records/stage2_retest_E1/E1_run01.bag --odom-topic /sim_odom --robot-radius 0.25
```

## Analyze Multiple Groups

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && rosrun plan_manage analyze_stage2_batch.py /home/xcg/ws/records/stage2_retest_C /home/xcg/ws/records/stage2_retest_E0b /home/xcg/ws/records/stage2_retest_E1 --odom-topic /sim_odom --robot-radius 0.25 --out-dir /home/xcg/ws/records/stage2_retest_analysis
```

如果也录了 B0：

```bash
cd /home/xcg/ws && source /opt/ros/noetic/setup.bash && source devel/setup.bash && rosrun plan_manage analyze_stage2_batch.py /home/xcg/ws/records/stage2_retest_B0 /home/xcg/ws/records/stage2_retest_C /home/xcg/ws/records/stage2_retest_E0b /home/xcg/ws/records/stage2_retest_E1 --odom-topic /sim_odom --robot-radius 0.25 --out-dir /home/xcg/ws/records/stage2_retest_B0_C_E0b_E1_analysis
```

## Expected Differences

- `C`: DRF-only。通常不会发布 Stage 2 interaction mode transition，可能出现较大绕行。
- `E0b`: 发布 interaction scene/debug，但不应该触发 interaction stop。
- `E1`: 期望出现 `CONTINUE -> YIELD -> CONTINUE`，`stop_reason` 为 `INTERACTION_YIELD_CONFLICT`，当前主线不依赖 PASS_BEHIND social cost。
