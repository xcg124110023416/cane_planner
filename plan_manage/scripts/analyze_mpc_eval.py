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
ODOM_TOPIC = "/localization_odom"
DYN_OBS_TOPIC = "/onboard_detector/dynamic_obstacles_info"
CLOCK_TOPIC = "/clock"
WAYPOINTS_TOPIC = "/mpc/waypoints"


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


def analyze_bag(bag_path):
    metrics_count = 0
    plan_valid_count = 0
    total_plan_count = 0

    plan_time = RunningStats()
    valid_ratio = RunningStats()
    risk_scale = RunningStats()
    best_clearance = RunningStats()
    best_ttc = RunningStats()
    global_clearance = RunningStats()
    global_ttc = RunningStats()
    actual_center_dist = RunningStats()
    actual_geom_clearance = RunningStats()

    stop_events = []
    reason_counter = Counter()
    stop_active = False
    stop_start = None
    current_reason = "OK"
    current_advice = False
    active_reason = "OK"

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
    final_waypoints = []

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
                    risk_scale.add(data[9])
                if len(data) > 10:
                    value = metric_or_none(data, 10)
                    if value is not None:
                        best_clearance.add(value)
                if len(data) > 11:
                    value = metric_or_none(data, 11)
                    if value is not None:
                        best_ttc.add(value)

            elif topic == ODOM_TOPIC:
                odom_xy = (
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                )
                latest_odom_xy = odom_xy
                last_odom_xy = odom_xy
                odom_points.append(odom_xy)
                if first_odom_xy is None:
                    first_odom_xy = odom_xy

                if prev_odom_xy is not None:
                    step_dist = math.hypot(odom_xy[0] - prev_odom_xy[0],
                                           odom_xy[1] - prev_odom_xy[1])
                    actual_path_length += step_dist
                    dt = t - prev_odom_time if prev_odom_time is not None else 0.0
                    if dt >= 0.02:
                        inst_speed = step_dist / dt
                        if inst_speed <= 3.0:
                            speed.add(inst_speed)
                        else:
                            skipped_speed_samples += 1
                    else:
                        skipped_speed_samples += 1
                prev_odom_xy = odom_xy
                prev_odom_time = t

            elif topic == WAYPOINTS_TOPIC:
                final_waypoints = [(pt.x, pt.y) for pt in msg.points]

            elif topic == DYN_OBS_TOPIC and latest_odom_xy is not None:
                rx, ry = latest_odom_xy
                for idx, pos in enumerate(msg.position):
                    dx = pos.x - rx
                    dy = pos.y - ry
                    center_dist = math.hypot(dx, dy)
                    actual_center_dist.add(center_dist)
                    radius = 0.0
                    if idx < len(msg.size):
                        radius = 0.5 * max(msg.size[idx].x, msg.size[idx].y)
                    actual_geom_clearance.add(center_dist - radius)

            elif topic == STOP_REASON_TOPIC:
                current_reason = msg.data if msg.data else "OK"
                if stop_active and current_reason != "OK" and active_reason == "STOP_ADVICE":
                    active_reason = current_reason

            elif topic == STOP_ADVICE_TOPIC:
                current_advice = bool(msg.data)

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

    lines = []
    lines.append("MPC Evaluation Summary")
    lines.append("bag: {}".format(bag_path))
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
    lines.append(risk_scale.line("risk_weight_scale"))
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
    lines.append(deviation.line("deviation_from_global_waypoints", "m"))
    lines.append("")
    lines.append(best_clearance.line("best_min_dynamic_clearance", "m"))
    lines.append(best_ttc.line("best_min_cpa_time", "s"))
    lines.append(global_clearance.line("global_min_dynamic_clearance", "m"))
    lines.append(global_ttc.line("global_min_cpa_time", "s"))
    lines.append("")
    lines.append(actual_center_dist.line("actual_robot_ped_center_distance", "m"))
    lines.append(actual_geom_clearance.line("actual_robot_ped_geom_clearance", "m"))
    lines.append("")
    lines.append("stop_events:")
    if not stop_events:
        lines.append("  none")
    else:
        base = first_time if first_time is not None else 0.0
        for idx, (start, end, duration, reason) in enumerate(stop_events, 1):
            lines.append("  {}. start={:.2f}s end={:.2f}s duration={:.2f}s reason={}".format(
                idx, start - base, end - base, duration, reason))

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Analyze a recorded MPC evaluation rosbag.")
    parser.add_argument("bag_or_dir", help="Path to mpc_eval.bag or a record directory containing a .bag")
    parser.add_argument("--no-write", action="store_true", help="Only print summary; do not write summary.txt")
    args = parser.parse_args()

    bag_path = resolve_bag_path(args.bag_or_dir)
    summary = analyze_bag(bag_path)
    print(summary)

    if not args.no_write:
        out_path = os.path.join(os.path.dirname(bag_path), "summary.txt")
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(summary)
            f.write("\n")
        print("\nWrote summary: {}".format(out_path))


if __name__ == "__main__":
    main()
