#!/usr/bin/env python3

import argparse
import glob
import math
import os
from collections import Counter

import rosbag


DEBUG_TOPIC = "/mpc/debug_metrics"
STOP_ADVICE_TOPIC = "/mpc/stop_advice"
STOP_REASON_TOPIC = "/mpc/stop_reason"
INTERACTION_SCENE_TOPIC = "/mpc/interaction_scene"
INTERACTION_MODE_TOPIC = "/mpc/interaction_mode"
INTERACTION_DEBUG_TOPIC = "/mpc/interaction_debug"
DEFAULT_ODOM_TOPIC = "/localization_odom"
LIGHTWEIGHT_ODOM_TOPIC = "/sim_odom"
CMD_TOPIC = "/cmd_vel_footprint"
DYN_OBS_TOPIC = "/onboard_detector/dynamic_obstacles_info"
CLOCK_TOPIC = "/clock"
WAYPOINTS_TOPIC = "/mpc/waypoints"
BEST_TRAJ_TOPIC = "/mpc/best_traj"

INTERACTION_DEBUG_FIELD_COUNT = 35
INT_R_CROSSING_IDX = 18
INT_RISK_FRONT_IDX = 21
INT_SIGNED_T_PED_TO_PATH_IDX = 22
INT_PED_BEFORE_PATH_IDX = 23
INT_PED_AT_OR_AFTER_PATH_IDX = 24
INT_YIELD_REQUIRED_IDX = 25
INT_ST_CONFLICT_IDX = 26
INT_ST_T_CONFLICT_IDX = 27
INT_ST_D_CONFLICT_IDX = 28
INT_ST_SAFETY_RADIUS_IDX = 29
INT_ST_ROBOT_S_IDX = 30
INT_ST_PED_S_IDX = 31
INT_ST_PATH_OCCUPIED_IDX = 32
INT_ST_PATH_T_ENTER_IDX = 33
INT_ST_PATH_T_EXIT_IDX = 34


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def point_to_segment_distance(px, py, ax, ay, bx, by):
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    seg_len_sq = vx * vx + vy * vy
    if seg_len_sq <= 1e-9:
        return math.hypot(px - ax, py - ay)
    ratio = max(0.0, min(1.0, (wx * vx + wy * vy) / seg_len_sq))
    proj_x = ax + ratio * vx
    proj_y = ay + ratio * vy
    return math.hypot(px - proj_x, py - proj_y)


def polyline_length(points):
    if len(points) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(points)):
        total += math.hypot(points[i][0] - points[i - 1][0],
                            points[i][1] - points[i - 1][1])
    return total


def distance_to_polyline(px, py, points):
    if not points:
        return float("nan")
    if len(points) == 1:
        return math.hypot(px - points[0][0], py - points[0][1])
    return min(
        point_to_segment_distance(px, py,
                                  points[i][0], points[i][1],
                                  points[i + 1][0], points[i + 1][1])
        for i in range(len(points) - 1)
    )


class RunningStats:
    def __init__(self):
        self.values = []

    def add(self, value):
        if value is None or not math.isfinite(value):
            return
        self.values.append(value)

    def count(self):
        return len(self.values)

    def mean(self):
        return sum(self.values) / len(self.values) if self.values else float("nan")

    def min(self):
        return min(self.values) if self.values else float("nan")

    def max(self):
        return max(self.values) if self.values else float("nan")

    def line(self, name, unit=""):
        suffix = " {}".format(unit) if unit else ""
        if not self.values:
            return "{}: count=0".format(name)
        return "{}: count={} mean={:.3f} min={:.3f} max={:.3f}{}".format(
            name, len(self.values), self.mean(), self.min(), self.max(), suffix)


def metric_or_none(data, index):
    if len(data) <= index:
        return None
    value = data[index]
    # PlannerManager publishes -1.0 for unavailable metrics. Small negative
    # clearances are meaningful and should be kept.
    if value <= -0.999:
        return None
    return value


def resolve_bag_path(path):
    if os.path.isdir(path):
        bags = sorted(glob.glob(os.path.join(path, "*.bag")))
        if not bags:
            raise RuntimeError("No .bag files found in {}".format(path))
        return bags[0]
    return path


def fmt(value, unit=""):
    if value is None or not math.isfinite(value):
        return "NA"
    return "{:.3f}{}".format(value, unit)


def pedestrian_relative_coordinates(encounter):
    vx = encounter.get("ped_vx", float("nan"))
    vy = encounter.get("ped_vy", float("nan"))
    speed = math.hypot(vx, vy)
    if not math.isfinite(speed) or speed < 1e-3:
        return float("nan"), float("nan")

    rx = encounter["robot_x"] - encounter["ped_x"]
    ry = encounter["robot_y"] - encounter["ped_y"]
    ux = vx / speed
    uy = vy / speed
    along = rx * ux + ry * uy
    cross = rx * (-uy) + ry * ux
    return along, cross


def classify_encounter_pass_mode(encounter, near_clearance=0.15, along_deadband=0.20):
    collision_clearance = encounter.get("collision_clearance", float("nan"))
    if math.isfinite(collision_clearance):
        if collision_clearance <= 0.0:
            return "COLLISION"
        if collision_clearance <= near_clearance:
            return "NEAR_COLLISION"

    along, _ = pedestrian_relative_coordinates(encounter)
    if math.isfinite(along):
        if along < -along_deadband:
            return "PASS_BEHIND"
        if along > along_deadband:
            return "PASS_AHEAD"
        return "SIDE_PASS"

    return "UNKNOWN"


def analyze_bag(bag_path, odom_topic=DEFAULT_ODOM_TOPIC, robot_radius=0.25):
    metrics_count = 0
    plan_valid_count = 0
    total_plan_count = 0

    plan_time = RunningStats()
    valid_ratio = RunningStats()
    best_clearance = RunningStats()
    best_ttc = RunningStats()
    global_clearance = RunningStats()
    global_ttc = RunningStats()
    actual_center_dist = RunningStats()
    actual_geom_clearance = RunningStats()
    actual_collision_clearance = RunningStats()
    closest_encounter = None

    stop_events = []
    reason_counter = Counter()
    stop_active = False
    stop_start = None
    current_reason = "OK"
    current_advice = False
    active_reason = "OK"

    # Interaction (Stage 2)
    interaction_scene_counter = Counter()
    interaction_mode_counter = Counter()
    interaction_scene_transitions = []
    interaction_mode_transitions = []
    current_interaction_scene = None
    current_interaction_mode = None
    interaction_scene_last_time = None
    interaction_mode_last_time = None
    interaction_scene_dwell = {}
    interaction_mode_dwell = {}
    interaction_debug_count = 0
    int_r_crossing = RunningStats()
    int_risk_front = RunningStats()
    int_time_gap = RunningStats()
    int_t_cpa = RunningStats()
    int_d_cpa = RunningStats()
    int_signed_t_ped_to_path = RunningStats()
    int_st_t_conflict = RunningStats()
    int_st_d_conflict = RunningStats()
    int_st_safety_radius = RunningStats()
    int_st_robot_s = RunningStats()
    int_st_ped_s = RunningStats()
    int_st_path_t_enter = RunningStats()
    int_st_path_t_exit = RunningStats()
    int_cpa_conflict_count = 0
    int_ped_before_count = 0
    int_ped_at_or_after_count = 0
    int_yield_required_count = 0
    int_st_conflict_count = 0
    int_st_path_occupied_count = 0

    first_time = None
    last_time = None
    first_clock = None
    last_clock = None
    latest_odom_xy = None
    first_odom_xy = None
    last_odom_xy = None
    prev_odom_xy = None
    prev_odom_time = None
    odom_points = []
    actual_path_length = 0.0
    speed = RunningStats()
    skipped_speed_samples = 0
    actual_forward_step = RunningStats()
    actual_backward_step_count = 0
    actual_step_count = 0
    prev_odom_yaw = None
    prev_odom_yaw_rate = None
    total_abs_heading_change = 0.0
    actual_abs_heading_step = RunningStats()
    odom_yaw_rate_abs = RunningStats()
    odom_yaw_accel_abs = RunningStats()
    final_waypoints = []
    latest_odom_yaw = None
    cmd_count = 0
    cmd_linear_x = RunningStats()
    cmd_angular_z = RunningStats()
    cmd_zero_count = 0
    cmd_negative_count = 0
    best_traj_count = 0
    best_traj_backward_count = 0
    best_traj_min_front = RunningStats()
    best_traj_end_front = RunningStats()
    best_traj_backward_fraction = RunningStats()
    best_traj_waypoint_deviation = RunningStats()
    best_traj_max_waypoint_deviation = RunningStats()
    best_traj_end_waypoint_deviation = RunningStats()
    best_traj_far_from_waypoints_count = 0

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, stamp in bag.read_messages():
            t = stamp.to_sec()
            if first_time is None:
                first_time = t
            last_time = t

            if topic == CLOCK_TOPIC:
                clock_t = msg.clock.to_sec()
                if first_clock is None:
                    first_clock = clock_t
                last_clock = clock_t
                continue

            if topic == DEBUG_TOPIC:
                data = list(msg.data)
                metrics_count += 1
                if len(data) > 0:
                    plan_time.add(data[0])
                if len(data) > 1:
                    valid_ratio.add(data[1])
                if len(data) > 3:
                    value = metric_or_none(data, 3)
                    if value is not None:
                        global_clearance.add(value)
                if len(data) > 4:
                    value = metric_or_none(data, 4)
                    if value is not None:
                        global_ttc.add(value)
                if len(data) > 8:
                    total_plan_count += 1
                    if data[8] > 0.5:
                        plan_valid_count += 1
                if len(data) > 9:
                    value = metric_or_none(data, 9)
                    if value is not None:
                        best_clearance.add(value)
                if len(data) > 10:
                    value = metric_or_none(data, 10)
                    if value is not None:
                        best_ttc.add(value)

            elif topic == odom_topic:
                odom_xy = (
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                )
                latest_odom_xy = odom_xy
                latest_odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
                last_odom_xy = odom_xy
                odom_points.append(odom_xy)
                if first_odom_xy is None:
                    first_odom_xy = odom_xy

                if prev_odom_xy is not None:
                    step_dist = math.hypot(odom_xy[0] - prev_odom_xy[0],
                                           odom_xy[1] - prev_odom_xy[1])
                    actual_path_length += step_dist
                    if prev_odom_yaw is not None:
                        yaw_step = angle_diff(latest_odom_yaw, prev_odom_yaw)
                        total_abs_heading_change += abs(yaw_step)
                        actual_abs_heading_step.add(abs(yaw_step))
                        forward_step = ((odom_xy[0] - prev_odom_xy[0]) * math.cos(prev_odom_yaw) +
                                        (odom_xy[1] - prev_odom_xy[1]) * math.sin(prev_odom_yaw))
                        actual_forward_step.add(forward_step)
                        actual_step_count += 1
                        if forward_step < -0.02:
                            actual_backward_step_count += 1
                    dt = t - prev_odom_time if prev_odom_time is not None else 0.0
                    if dt >= 0.02:
                        inst_speed = step_dist / dt
                        if inst_speed <= 3.0:
                            speed.add(inst_speed)
                        else:
                            skipped_speed_samples += 1
                        if prev_odom_yaw is not None:
                            yaw_rate = angle_diff(latest_odom_yaw, prev_odom_yaw) / dt
                            odom_yaw_rate_abs.add(abs(yaw_rate))
                            if prev_odom_yaw_rate is not None:
                                odom_yaw_accel_abs.add(abs(yaw_rate - prev_odom_yaw_rate) / dt)
                            prev_odom_yaw_rate = yaw_rate
                    else:
                        skipped_speed_samples += 1
                prev_odom_xy = odom_xy
                prev_odom_time = t
                prev_odom_yaw = latest_odom_yaw

            elif topic == CMD_TOPIC:
                cmd_count += 1
                cmd_linear_x.add(msg.linear.x)
                cmd_angular_z.add(msg.angular.z)
                if abs(msg.linear.x) < 1e-3 and abs(msg.angular.z) < 1e-3:
                    cmd_zero_count += 1
                if msg.linear.x < -1e-3:
                    cmd_negative_count += 1

            elif topic == WAYPOINTS_TOPIC:
                final_waypoints = [(pt.x, pt.y) for pt in msg.points]

            elif topic == BEST_TRAJ_TOPIC and latest_odom_xy is not None and latest_odom_yaw is not None:
                points = [(pt.x, pt.y) for pt in msg.points]
                if len(points) >= 2:
                    best_traj_count += 1
                    rx, ry = latest_odom_xy
                    fx = math.cos(latest_odom_yaw)
                    fy = math.sin(latest_odom_yaw)
                    fronts = [((px - rx) * fx + (py - ry) * fy) for px, py in points]
                    min_front = min(fronts)
                    end_front = fronts[-1]
                    backward_points = sum(1 for value in fronts if value < -0.10)
                    backward_fraction = backward_points / float(len(fronts))
                    best_traj_min_front.add(min_front)
                    best_traj_end_front.add(end_front)
                    best_traj_backward_fraction.add(backward_fraction)
                    if min_front < -0.20 or end_front < -0.05:
                        best_traj_backward_count += 1
                    if final_waypoints:
                        deviations = [
                            distance_to_polyline(px, py, final_waypoints)
                            for px, py in points
                        ]
                        mean_dev = sum(deviations) / float(len(deviations))
                        max_dev = max(deviations)
                        end_dev = deviations[-1]
                        best_traj_waypoint_deviation.add(mean_dev)
                        best_traj_max_waypoint_deviation.add(max_dev)
                        best_traj_end_waypoint_deviation.add(end_dev)
                        if max_dev > 1.0 or end_dev > 0.8:
                            best_traj_far_from_waypoints_count += 1

            elif topic == DYN_OBS_TOPIC and latest_odom_xy is not None:
                rx, ry = latest_odom_xy
                for idx, pos in enumerate(msg.position):
                    dx = pos.x - rx
                    dy = pos.y - ry
                    center_dist = math.hypot(dx, dy)
                    actual_center_dist.add(center_dist)
                    ped_radius = 0.0
                    if idx < len(msg.size):
                        ped_radius = 0.5 * max(msg.size[idx].x, msg.size[idx].y)
                    geom_clearance = center_dist - ped_radius
                    collision_clearance = geom_clearance - robot_radius
                    actual_geom_clearance.add(geom_clearance)
                    actual_collision_clearance.add(collision_clearance)

                    yaw = latest_odom_yaw if latest_odom_yaw is not None else 0.0
                    front = dx * math.cos(yaw) + dy * math.sin(yaw)
                    lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
                    ped_speed = float("nan")
                    vx = float("nan")
                    vy = float("nan")
                    if idx < len(msg.velocity):
                        vx = msg.velocity[idx].x
                        vy = msg.velocity[idx].y
                        ped_speed = math.hypot(vx, vy)
                    if closest_encounter is None or collision_clearance < closest_encounter["collision_clearance"]:
                        closest_encounter = {
                            "time": t,
                            "ped_id": idx,
                            "robot_x": rx,
                            "robot_y": ry,
                            "robot_yaw": yaw,
                            "ped_x": pos.x,
                            "ped_y": pos.y,
                            "ped_vx": vx,
                            "ped_vy": vy,
                            "ped_speed": ped_speed,
                            "center_dist": center_dist,
                            "ped_radius": ped_radius,
                            "robot_radius": robot_radius,
                            "geom_clearance": geom_clearance,
                            "collision_clearance": collision_clearance,
                            "rel_front": front,
                            "rel_lateral": lateral,
                        }

            elif topic == STOP_REASON_TOPIC:
                current_reason = msg.data if msg.data else "OK"
                if stop_active and current_reason != "OK" and active_reason == "STOP_ADVICE":
                    active_reason = current_reason

            elif topic == STOP_ADVICE_TOPIC:
                current_advice = bool(msg.data)

            elif topic == INTERACTION_SCENE_TOPIC:
                new_scene = msg.data if msg.data else "none"
                interaction_scene_counter[new_scene] += 1
                if current_interaction_scene is not None and new_scene != current_interaction_scene:
                    interaction_scene_transitions.append((t, current_interaction_scene, new_scene))
                if current_interaction_scene is not None and interaction_scene_last_time is not None:
                    dt = max(0.0, t - interaction_scene_last_time)
                    interaction_scene_dwell[current_interaction_scene] = interaction_scene_dwell.get(current_interaction_scene, 0.0) + dt
                current_interaction_scene = new_scene
                interaction_scene_last_time = t

            elif topic == INTERACTION_MODE_TOPIC:
                new_mode = msg.data if msg.data else "CONTINUE"
                interaction_mode_counter[new_mode] += 1
                if current_interaction_mode is not None and new_mode != current_interaction_mode:
                    interaction_mode_transitions.append((t, current_interaction_mode, new_mode))
                if current_interaction_mode is not None and interaction_mode_last_time is not None:
                    dt = max(0.0, t - interaction_mode_last_time)
                    interaction_mode_dwell[current_interaction_mode] = interaction_mode_dwell.get(current_interaction_mode, 0.0) + dt
                current_interaction_mode = new_mode
                interaction_mode_last_time = t

            elif topic == INTERACTION_DEBUG_TOPIC:
                data = list(msg.data)
                if len(data) != INTERACTION_DEBUG_FIELD_COUNT:
                    continue
                interaction_debug_count += 1
                if len(data) > INT_R_CROSSING_IDX:
                    int_r_crossing.add(data[INT_R_CROSSING_IDX])
                if len(data) > INT_RISK_FRONT_IDX:
                    int_risk_front.add(data[INT_RISK_FRONT_IDX])
                if len(data) > 11:
                    int_time_gap.add(data[11])
                if len(data) > 12:
                    int_t_cpa.add(data[12])
                if len(data) > 13:
                    int_d_cpa.add(data[13])
                if len(data) > 14 and data[14] > 0.5:
                    int_cpa_conflict_count += 1
                if len(data) > INT_SIGNED_T_PED_TO_PATH_IDX:
                    int_signed_t_ped_to_path.add(data[INT_SIGNED_T_PED_TO_PATH_IDX])
                if len(data) > INT_PED_BEFORE_PATH_IDX and data[INT_PED_BEFORE_PATH_IDX] > 0.5:
                    int_ped_before_count += 1
                if len(data) > INT_PED_AT_OR_AFTER_PATH_IDX and data[INT_PED_AT_OR_AFTER_PATH_IDX] > 0.5:
                    int_ped_at_or_after_count += 1
                if len(data) > INT_YIELD_REQUIRED_IDX and data[INT_YIELD_REQUIRED_IDX] > 0.5:
                    int_yield_required_count += 1
                if len(data) > INT_ST_CONFLICT_IDX and data[INT_ST_CONFLICT_IDX] > 0.5:
                    int_st_conflict_count += 1
                if len(data) > INT_ST_T_CONFLICT_IDX:
                    int_st_t_conflict.add(data[INT_ST_T_CONFLICT_IDX])
                if len(data) > INT_ST_D_CONFLICT_IDX:
                    int_st_d_conflict.add(data[INT_ST_D_CONFLICT_IDX])
                if len(data) > INT_ST_SAFETY_RADIUS_IDX:
                    int_st_safety_radius.add(data[INT_ST_SAFETY_RADIUS_IDX])
                if len(data) > INT_ST_ROBOT_S_IDX:
                    int_st_robot_s.add(data[INT_ST_ROBOT_S_IDX])
                if len(data) > INT_ST_PED_S_IDX:
                    int_st_ped_s.add(data[INT_ST_PED_S_IDX])
                if len(data) > INT_ST_PATH_OCCUPIED_IDX and data[INT_ST_PATH_OCCUPIED_IDX] > 0.5:
                    int_st_path_occupied_count += 1
                if len(data) > INT_ST_PATH_T_ENTER_IDX:
                    int_st_path_t_enter.add(data[INT_ST_PATH_T_ENTER_IDX])
                if len(data) > INT_ST_PATH_T_EXIT_IDX:
                    int_st_path_t_exit.add(data[INT_ST_PATH_T_EXIT_IDX])

            if topic in (STOP_REASON_TOPIC, STOP_ADVICE_TOPIC):
                should_stop = current_advice or current_reason != "OK"
                display_reason = current_reason if current_reason != "OK" else "STOP_ADVICE"

                if should_stop and not stop_active:
                    stop_active = True
                    stop_start = t
                    active_reason = display_reason
                elif not should_stop and stop_active:
                    duration = max(0.0, t - stop_start) if stop_start is not None else 0.0
                    stop_events.append((stop_start, t, duration, active_reason))
                    reason_counter[active_reason] += 1
                    stop_active = False
                    stop_start = None
                    active_reason = "OK"

    if stop_active and stop_start is not None and last_time is not None:
        duration = max(0.0, last_time - stop_start)
        stop_events.append((stop_start, last_time, duration, active_reason))
        reason_counter[active_reason] += 1

    total_stop_duration = sum(event[2] for event in stop_events)
    run_duration = None
    if first_clock is not None and last_clock is not None:
        run_duration = max(0.0, last_clock - first_clock)
    elif first_time is not None and last_time is not None:
        run_duration = max(0.0, last_time - first_time)

    straight_distance = float("nan")
    if first_odom_xy is not None and last_odom_xy is not None:
        straight_distance = math.hypot(last_odom_xy[0] - first_odom_xy[0],
                                       last_odom_xy[1] - first_odom_xy[1])
    path_efficiency = float("nan")
    if actual_path_length > 1e-6 and math.isfinite(straight_distance):
        path_efficiency = straight_distance / actual_path_length

    global_path_length = polyline_length(final_waypoints)
    final_goal_error = float("nan")
    if last_odom_xy is not None and final_waypoints:
        goal_xy = final_waypoints[-1]
        final_goal_error = math.hypot(last_odom_xy[0] - goal_xy[0],
                                      last_odom_xy[1] - goal_xy[1])

    deviation = RunningStats()
    if final_waypoints:
        for px, py in odom_points:
            deviation.add(distance_to_polyline(px, py, final_waypoints))

    # Finalize interaction dwell times for still-active scene/mode
    if current_interaction_scene is not None and interaction_scene_last_time is not None and last_time is not None:
        dt = max(0.0, last_time - interaction_scene_last_time)
        interaction_scene_dwell[current_interaction_scene] = interaction_scene_dwell.get(current_interaction_scene, 0.0) + dt
    if current_interaction_mode is not None and interaction_mode_last_time is not None and last_time is not None:
        dt = max(0.0, last_time - interaction_mode_last_time)
        interaction_mode_dwell[current_interaction_mode] = interaction_mode_dwell.get(current_interaction_mode, 0.0) + dt

    lines = []
    lines.append("MPC Evaluation Summary")
    lines.append("bag: {}".format(bag_path))
    lines.append("odom_topic: {}".format(odom_topic))
    lines.append("duration: {}".format(fmt(run_duration, " s")))
    lines.append("metrics_messages: {}".format(metrics_count))
    lines.append("stop_count: {}".format(len(stop_events)))
    lines.append("total_stop_duration: {}".format(fmt(total_stop_duration, " s")))
    if run_duration and run_duration > 0.0:
        lines.append("stop_time_ratio: {:.3f}".format(total_stop_duration / run_duration))
    lines.append("stop_reasons: {}".format(
        ", ".join("{}={}".format(k, v) for k, v in sorted(reason_counter.items())) or "none"))
    lines.append("")
    lines.append(plan_time.line("mpc_plan_time", "ms"))
    lines.append(valid_ratio.line("valid_sample_ratio"))
    if total_plan_count > 0:
        lines.append("plan_valid_ratio: {:.3f} ({}/{})".format(
            plan_valid_count / float(total_plan_count), plan_valid_count, total_plan_count))
    lines.append("")
    lines.append("actual_path_length: {}".format(fmt(actual_path_length, " m")))
    lines.append("straight_start_to_end_distance: {}".format(fmt(straight_distance, " m")))
    lines.append("path_efficiency: {}".format(fmt(path_efficiency)))
    lines.append("global_waypoint_path_length: {}".format(fmt(global_path_length, " m")))
    lines.append("final_goal_error: {}".format(fmt(final_goal_error, " m")))
    lines.append(speed.line("odom_speed", "m/s"))
    lines.append("odom_speed_filtered_samples: {}".format(skipped_speed_samples))
    lines.append(actual_forward_step.line("actual_forward_step", "m"))
    lines.append("actual_backward_step_count: {}".format(actual_backward_step_count))
    if actual_step_count > 0:
        lines.append("actual_backward_step_ratio: {:.3f}".format(
            actual_backward_step_count / float(actual_step_count)))
    lines.append("heading_change_total: {}".format(fmt(total_abs_heading_change, " rad")))
    lines.append(actual_abs_heading_step.line("abs_heading_step", "rad"))
    lines.append(odom_yaw_rate_abs.line("odom_abs_yaw_rate", "rad/s"))
    lines.append(odom_yaw_accel_abs.line("odom_abs_yaw_accel", "rad/s^2"))
    lines.append("cmd_vel_messages: {}".format(cmd_count))
    lines.append(cmd_linear_x.line("cmd_linear_x", "m/s"))
    lines.append(cmd_angular_z.line("cmd_angular_z", "rad/s"))
    lines.append("cmd_zero_count: {}".format(cmd_zero_count))
    lines.append("cmd_negative_count: {}".format(cmd_negative_count))
    if cmd_count > 0:
        lines.append("cmd_zero_ratio: {:.3f}".format(cmd_zero_count / float(cmd_count)))
    lines.append(deviation.line("deviation_from_global_waypoints", "m"))
    lines.append("")
    lines.append(best_clearance.line("best_min_dynamic_clearance", "m"))
    lines.append(best_ttc.line("best_min_cpa_time", "s"))
    lines.append(global_clearance.line("global_min_dynamic_clearance", "m"))
    lines.append(global_ttc.line("global_min_cpa_time", "s"))
    lines.append("")
    lines.append(actual_center_dist.line("actual_robot_ped_center_distance", "m"))
    lines.append(actual_geom_clearance.line("actual_robot_ped_geom_clearance", "m"))
    lines.append(actual_collision_clearance.line("actual_robot_ped_collision_clearance", "m"))
    if closest_encounter is not None:
        base = first_time if first_time is not None else closest_encounter["time"]
        collision_clearance = closest_encounter["collision_clearance"]
        if collision_clearance <= 0.0:
            collision_status = "COLLISION"
        elif collision_clearance <= 0.15:
            collision_status = "NEAR"
        else:
            collision_status = "CLEAR"
        pass_mode = classify_encounter_pass_mode(closest_encounter)
        rel_along_ped, rel_cross_ped = pedestrian_relative_coordinates(closest_encounter)
        lines.append(
            "closest_robot_ped_encounter: time={:.2f}s ped_id={} center={:.3f}m geom_clearance={:.3f}m collision_clearance={:.3f}m status={} pass_mode={} robot=({:.3f},{:.3f}) ped=({:.3f},{:.3f}) rel_front={:.3f}m rel_lateral={:.3f}m rel_along_ped={:.3f}m rel_cross_ped={:.3f}m ped_vel=({:.3f},{:.3f})m/s".format(
                closest_encounter["time"] - base,
                closest_encounter["ped_id"],
                closest_encounter["center_dist"],
                closest_encounter["geom_clearance"],
                collision_clearance,
                collision_status,
                pass_mode,
                closest_encounter["robot_x"],
                closest_encounter["robot_y"],
                closest_encounter["ped_x"],
                closest_encounter["ped_y"],
                closest_encounter["rel_front"],
                closest_encounter["rel_lateral"],
                rel_along_ped,
                rel_cross_ped,
                closest_encounter["ped_vx"],
                closest_encounter["ped_vy"],
            )
        )
        lines.append(
            "closest_robot_ped_assumptions: robot_radius={:.3f}m, ped_radius={:.3f}m, collision_clearance=center_dist-ped_radius-robot_radius".format(
                closest_encounter["robot_radius"],
                closest_encounter["ped_radius"],
            )
        )
    lines.append("best_traj_messages: {}".format(best_traj_count))
    lines.append(best_traj_min_front.line("best_traj_min_front", "m"))
    lines.append(best_traj_end_front.line("best_traj_end_front", "m"))
    lines.append(best_traj_backward_fraction.line("best_traj_backward_fraction"))
    lines.append("best_traj_backward_count: {}".format(best_traj_backward_count))
    if best_traj_count > 0:
        lines.append("best_traj_backward_ratio: {:.3f}".format(
            best_traj_backward_count / float(best_traj_count)))
    lines.append(best_traj_waypoint_deviation.line("best_traj_waypoint_deviation", "m"))
    lines.append(best_traj_max_waypoint_deviation.line("best_traj_max_waypoint_deviation", "m"))
    lines.append(best_traj_end_waypoint_deviation.line("best_traj_end_waypoint_deviation", "m"))
    lines.append("best_traj_far_from_waypoints_count: {}".format(best_traj_far_from_waypoints_count))
    if best_traj_count > 0:
        lines.append("best_traj_far_from_waypoints_ratio: {:.3f}".format(
            best_traj_far_from_waypoints_count / float(best_traj_count)))
    lines.append("")
    lines.append("stop_events:")
    if not stop_events:
        lines.append("  none")
    else:
        base = first_time if first_time is not None else 0.0
        for idx, (start, end, duration, reason) in enumerate(stop_events, 1):
            lines.append("  {}. start={:.2f}s end={:.2f}s duration={:.2f}s reason={}".format(
                idx, start - base, end - base, duration, reason))

    # Interaction (Stage 2)
    lines.append("")
    lines.append("--- Interaction (Stage 2) ---")
    lines.append("interaction_debug_messages: {}".format(interaction_debug_count))
    lines.append("interaction_scenes: {}".format(
        ", ".join("{}={}".format(k, v) for k, v in sorted(interaction_scene_counter.items())) or "none"))
    lines.append("interaction_modes: {}".format(
        ", ".join("{}={}".format(k, v) for k, v in sorted(interaction_mode_counter.items())) or "none"))
    lines.append("interaction_scene_transitions: {}".format(len(interaction_scene_transitions)))
    lines.append("interaction_mode_transitions: {}".format(len(interaction_mode_transitions)))
    if interaction_mode_transitions:
        # Count switches away from CONTINUE (actual interaction decisions)
        yield_switch_count = sum(1 for _, old, new in interaction_mode_transitions if new == "YIELD")
        lines.append("yield_entry_count: {}".format(yield_switch_count))
    lines.append("interaction_scene_dwell:")
    if interaction_scene_dwell:
        for scene_name in sorted(interaction_scene_dwell.keys()):
            dt = interaction_scene_dwell[scene_name]
            lines.append("  {}: {:.2f}s".format(scene_name, dt))
    else:
        lines.append("  none")
    lines.append("interaction_mode_dwell:")
    if interaction_mode_dwell:
        for mode_name in sorted(interaction_mode_dwell.keys()):
            dt = interaction_mode_dwell[mode_name]
            lines.append("  {}: {:.2f}s".format(mode_name, dt))
    else:
        lines.append("  none")
    lines.append("")
    lines.append("interaction_risk_scores:")
    lines.append("  " + int_r_crossing.line("R_crossing"))
    lines.append("  " + int_risk_front.line("risk_front"))
    lines.append("  " + int_time_gap.line("time_gap", "s"))
    lines.append("  " + int_signed_t_ped_to_path.line("signed_t_ped_to_path", "s"))
    lines.append("  " + int_t_cpa.line("t_cpa", "s"))
    lines.append("  " + int_d_cpa.line("d_cpa", "m"))
    lines.append("  cpa_conflict_count: {} / {}".format(int_cpa_conflict_count, interaction_debug_count))
    lines.append("  ped_before_path_count: {} / {}".format(int_ped_before_count, interaction_debug_count))
    lines.append("  ped_at_or_after_path_count: {} / {}".format(int_ped_at_or_after_count, interaction_debug_count))
    lines.append("  yield_required_count: {} / {}".format(int_yield_required_count, interaction_debug_count))
    lines.append("  st_conflict_count: {} / {}".format(int_st_conflict_count, interaction_debug_count))
    lines.append("  " + int_st_t_conflict.line("st_t_conflict", "s"))
    lines.append("  " + int_st_d_conflict.line("st_d_conflict", "m"))
    lines.append("  " + int_st_safety_radius.line("st_safety_radius", "m"))
    lines.append("  " + int_st_robot_s.line("st_robot_s", "m"))
    lines.append("  " + int_st_ped_s.line("st_ped_s", "m"))
    lines.append("  st_path_occupied_count: {} / {}".format(int_st_path_occupied_count, interaction_debug_count))
    lines.append("  " + int_st_path_t_enter.line("st_path_t_enter", "s"))
    lines.append("  " + int_st_path_t_exit.line("st_path_t_exit", "s"))
    lines.append("")
    lines.append("interaction_mode_transitions:")
    if not interaction_mode_transitions:
        lines.append("  none")
    else:
        base = first_time if first_time is not None else 0.0
        for idx, (bt, old_state, new_state) in enumerate(interaction_mode_transitions, 1):
            lines.append("  {}. time={:.2f}s {} -> {}".format(
                idx, bt - base, old_state, new_state))
    lines.append("interaction_scene_transitions:")
    if not interaction_scene_transitions:
        lines.append("  none")
    else:
        base = first_time if first_time is not None else 0.0
        for idx, (bt, old_state, new_state) in enumerate(interaction_scene_transitions, 1):
            lines.append("  {}. time={:.2f}s {} -> {}".format(
                idx, bt - base, old_state, new_state))

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Analyze a recorded MPC evaluation rosbag.")
    parser.add_argument("bag_or_dir", help="Path to mpc_eval.bag or a record directory containing a .bag")
    parser.add_argument(
        "--lightweight",
        action="store_true",
        help="Use lightweight simulation topics, especially /simulation_generator/odom")
    parser.add_argument(
        "--odom-topic",
        default=None,
        help="Odometry topic to use for actual path metrics")
    parser.add_argument(
        "--robot-radius",
        type=float,
        default=0.25,
        help="Robot radius used for collision-clearance estimates against pedestrian boxes")
    parser.add_argument("--no-write", action="store_true", help="Only print summary; do not write summary.txt")
    args = parser.parse_args()

    bag_path = resolve_bag_path(args.bag_or_dir)
    odom_topic = args.odom_topic
    if odom_topic is None:
        odom_topic = LIGHTWEIGHT_ODOM_TOPIC if args.lightweight else DEFAULT_ODOM_TOPIC
    summary = analyze_bag(bag_path, odom_topic=odom_topic, robot_radius=args.robot_radius)
    print(summary)

    if not args.no_write:
        out_path = os.path.join(os.path.dirname(bag_path), "summary.txt")
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(summary)
            f.write("\n")
        print("\nWrote summary: {}".format(out_path))


if __name__ == "__main__":
    main()
