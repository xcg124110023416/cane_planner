#!/usr/bin/env python3

import argparse
import csv
import glob
import math
import os
from collections import Counter, defaultdict

import rosbag


INTERACTION_SCENE_TOPIC = "/mpc/interaction_scene"
INTERACTION_MODE_TOPIC = "/mpc/interaction_mode"
INTERACTION_DEBUG_TOPIC = "/mpc/interaction_debug"
STOP_ADVICE_TOPIC = "/mpc/stop_advice"
STOP_REASON_TOPIC = "/mpc/stop_reason"
DYN_OBS_TOPIC = "/onboard_detector/dynamic_obstacles_info"
WAYPOINTS_TOPIC = "/mpc/waypoints"
BEST_TRAJ_TOPIC = "/mpc/best_traj"
DEBUG_TOPIC = "/mpc/debug_metrics"


def fmt(value):
    if value is None or not math.isfinite(value):
        return "NA"
    return "{:.3f}".format(value)


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


def discover_bags(paths):
    bags = []
    for path in paths:
        if os.path.isdir(path):
            bags.extend(sorted(glob.glob(os.path.join(path, "**", "*.bag"), recursive=True)))
        else:
            bags.append(path)
    return [bag for bag in bags if os.path.isfile(bag)]


def infer_group(bag_path):
    parts = os.path.normpath(bag_path).split(os.sep)
    for part in reversed(parts[:-1]):
        if part:
            return part
    return "unknown"


def update_dwell(dwell, current_value, last_time, now):
    if current_value is not None and last_time is not None:
        dwell[current_value] += max(0.0, now - last_time)


def analyze_bag(bag_path, odom_topic, robot_radius):
    first_time = None
    last_time = None
    odom_points = []
    last_odom = None
    path_length = 0.0
    waypoints = []
    latest_odom = None
    closest_collision_clearance = float("nan")
    closest_center_dist = float("nan")
    best_min_dynamic_clearance = float("nan")
    best_traj_max_deviation = float("nan")

    current_scene = None
    current_mode = None
    scene_last_time = None
    mode_last_time = None
    scene_dwell = defaultdict(float)
    mode_dwell = defaultdict(float)
    scene_transitions = []
    mode_transitions = []
    mode_counts = Counter()
    scene_counts = Counter()

    current_stop_advice = False
    current_stop_reason = "OK"
    stop_active = False
    stop_start = None
    stop_events = []
    stop_reason_counts = Counter()

    signed_t_values = []
    yield_required_count = 0
    pass_ready_count = 0
    debug_count = 0

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, stamp in bag.read_messages():
            t = stamp.to_sec()
            if first_time is None:
                first_time = t
            last_time = t

            if topic == odom_topic:
                xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
                latest_odom = xy
                odom_points.append(xy)
                if last_odom is not None:
                    path_length += math.hypot(xy[0] - last_odom[0], xy[1] - last_odom[1])
                last_odom = xy

            elif topic == WAYPOINTS_TOPIC:
                waypoints = [(pt.x, pt.y) for pt in msg.points]

            elif topic == DYN_OBS_TOPIC and latest_odom is not None:
                rx, ry = latest_odom
                for idx, pos in enumerate(msg.position):
                    center_dist = math.hypot(pos.x - rx, pos.y - ry)
                    ped_radius = 0.0
                    if idx < len(msg.size):
                        ped_radius = 0.5 * max(msg.size[idx].x, msg.size[idx].y)
                    collision_clearance = center_dist - ped_radius - robot_radius
                    if (not math.isfinite(closest_collision_clearance) or
                            collision_clearance < closest_collision_clearance):
                        closest_collision_clearance = collision_clearance
                        closest_center_dist = center_dist

            elif topic == BEST_TRAJ_TOPIC and waypoints:
                points = [(pt.x, pt.y) for pt in msg.points]
                if points:
                    max_dev = max(distance_to_polyline(px, py, waypoints) for px, py in points)
                    if not math.isfinite(best_traj_max_deviation) or max_dev > best_traj_max_deviation:
                        best_traj_max_deviation = max_dev

            elif topic == DEBUG_TOPIC:
                data = list(msg.data)
                if len(data) > 10 and data[10] > -0.999:
                    value = data[10]
                    if not math.isfinite(best_min_dynamic_clearance) or value < best_min_dynamic_clearance:
                        best_min_dynamic_clearance = value

            elif topic == INTERACTION_SCENE_TOPIC:
                new_scene = msg.data if msg.data else "none"
                scene_counts[new_scene] += 1
                if current_scene is not None and new_scene != current_scene:
                    scene_transitions.append("{}->{}".format(current_scene, new_scene))
                update_dwell(scene_dwell, current_scene, scene_last_time, t)
                current_scene = new_scene
                scene_last_time = t

            elif topic == INTERACTION_MODE_TOPIC:
                new_mode = msg.data if msg.data else "CONTINUE"
                mode_counts[new_mode] += 1
                if current_mode is not None and new_mode != current_mode:
                    mode_transitions.append("{}->{}".format(current_mode, new_mode))
                update_dwell(mode_dwell, current_mode, mode_last_time, t)
                current_mode = new_mode
                mode_last_time = t

            elif topic == INTERACTION_DEBUG_TOPIC:
                data = list(msg.data)
                debug_count += 1
                if len(data) > 30 and math.isfinite(data[30]):
                    signed_t_values.append(data[30])
                if len(data) > 33 and data[33] > 0.5:
                    yield_required_count += 1
                if len(data) > 34 and data[34] > 0.5:
                    pass_ready_count += 1

            elif topic == STOP_ADVICE_TOPIC:
                current_stop_advice = bool(msg.data)

            elif topic == STOP_REASON_TOPIC:
                current_stop_reason = msg.data if msg.data else "OK"
                if stop_active and current_stop_reason != "OK" and active_reason == "STOP_ADVICE":
                    active_reason = current_stop_reason

            if topic in (STOP_ADVICE_TOPIC, STOP_REASON_TOPIC):
                should_stop = current_stop_advice or current_stop_reason != "OK"
                reason = current_stop_reason if current_stop_reason != "OK" else "STOP_ADVICE"
                if should_stop and not stop_active:
                    stop_active = True
                    stop_start = t
                    active_reason = reason
                elif not should_stop and stop_active:
                    duration = max(0.0, t - stop_start) if stop_start is not None else 0.0
                    stop_events.append(duration)
                    stop_reason_counts[active_reason] += 1
                    stop_active = False
                    stop_start = None

    if stop_active and stop_start is not None and last_time is not None:
        duration = max(0.0, last_time - stop_start)
        stop_events.append(duration)
        stop_reason_counts[active_reason] += 1

    update_dwell(scene_dwell, current_scene, scene_last_time, last_time)
    update_dwell(mode_dwell, current_mode, mode_last_time, last_time)

    duration = max(0.0, last_time - first_time) if first_time is not None and last_time is not None else float("nan")
    actual_max_deviation = float("nan")
    actual_mean_deviation = float("nan")
    if waypoints and odom_points:
        deviations = [distance_to_polyline(px, py, waypoints) for px, py in odom_points]
        actual_max_deviation = max(deviations)
        actual_mean_deviation = sum(deviations) / float(len(deviations))

    straight = float("nan")
    efficiency = float("nan")
    if len(odom_points) >= 2:
        straight = math.hypot(odom_points[-1][0] - odom_points[0][0],
                              odom_points[-1][1] - odom_points[0][1])
        if path_length > 1e-6:
            efficiency = straight / path_length

    def dwell(name):
        return scene_dwell.get(name, 0.0), mode_dwell.get(name, 0.0)

    _, continue_dwell = dwell("CONTINUE")
    _, yield_dwell = dwell("YIELD")
    _, pass_behind_dwell = dwell("PASS_BEHIND")
    crossing_dwell, _ = dwell("crossing")
    none_dwell, _ = dwell("none")

    signed_min = min(signed_t_values) if signed_t_values else float("nan")
    signed_max = max(signed_t_values) if signed_t_values else float("nan")

    return {
        "group": infer_group(bag_path),
        "bag": bag_path,
        "duration_s": duration,
        "path_length_m": path_length,
        "straight_distance_m": straight,
        "path_efficiency": efficiency,
        "actual_mean_deviation_m": actual_mean_deviation,
        "actual_max_deviation_m": actual_max_deviation,
        "best_traj_max_deviation_m": best_traj_max_deviation,
        "closest_center_dist_m": closest_center_dist,
        "closest_collision_clearance_m": closest_collision_clearance,
        "best_min_dynamic_clearance_m": best_min_dynamic_clearance,
        "stop_count": len(stop_events),
        "stop_duration_s": sum(stop_events),
        "yield_entry_count": sum(1 for tr in mode_transitions if tr.endswith("->YIELD")),
        "pass_behind_entry_count": sum(1 for tr in mode_transitions if tr.endswith("->PASS_BEHIND")),
        "crossing_dwell_s": crossing_dwell,
        "none_dwell_s": none_dwell,
        "continue_dwell_s": continue_dwell,
        "yield_dwell_s": yield_dwell,
        "pass_behind_dwell_s": pass_behind_dwell,
        "mode_transitions": ";".join(mode_transitions) or "none",
        "scene_transitions": ";".join(scene_transitions) or "none",
        "signed_t_min_s": signed_min,
        "signed_t_max_s": signed_max,
        "yield_required_count": yield_required_count,
        "pass_ready_count": pass_ready_count,
        "interaction_debug_count": debug_count,
        "stop_reasons": ";".join("{}={}".format(k, v) for k, v in sorted(stop_reason_counts.items())) or "none",
    }


def mean(values):
    finite = [value for value in values if math.isfinite(value)]
    return sum(finite) / float(len(finite)) if finite else float("nan")


def write_markdown(rows, out_path):
    groups = defaultdict(list)
    for row in rows:
        groups[row["group"]].append(row)

    key_metrics = [
        "actual_max_deviation_m",
        "closest_collision_clearance_m",
        "stop_duration_s",
        "yield_entry_count",
        "pass_behind_entry_count",
        "path_efficiency",
    ]
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# Stage 2 Batch Summary\n\n")
        f.write("| group | n | max_dev_mean | clearance_mean | stop_s_mean | yield_entries_mean | pass_behind_entries_mean | efficiency_mean |\n")
        f.write("|---|---:|---:|---:|---:|---:|---:|---:|\n")
        for group in sorted(groups.keys()):
            items = groups[group]
            values = {metric: mean([item[metric] for item in items]) for metric in key_metrics}
            f.write("| {} | {} | {} | {} | {} | {} | {} | {} |\n".format(
                group,
                len(items),
                fmt(values["actual_max_deviation_m"]),
                fmt(values["closest_collision_clearance_m"]),
                fmt(values["stop_duration_s"]),
                fmt(values["yield_entry_count"]),
                fmt(values["pass_behind_entry_count"]),
                fmt(values["path_efficiency"]),
            ))
        f.write("\n## Runs\n\n")
        for row in rows:
            f.write("- `{}`: group={} max_dev={}m clearance={}m stop={}s modes={} scenes={}\n".format(
                os.path.basename(row["bag"]),
                row["group"],
                fmt(row["actual_max_deviation_m"]),
                fmt(row["closest_collision_clearance_m"]),
                fmt(row["stop_duration_s"]),
                row["mode_transitions"],
                row["scene_transitions"],
            ))


def main():
    parser = argparse.ArgumentParser(description="Aggregate Stage 2 crossing evaluation bags.")
    parser.add_argument("paths", nargs="+", help="Bag files or directories containing bags")
    parser.add_argument("--odom-topic", default="/sim_odom")
    parser.add_argument("--robot-radius", type=float, default=0.25)
    parser.add_argument("--out-dir", default=None)
    args = parser.parse_args()

    bags = discover_bags(args.paths)
    if not bags:
        raise RuntimeError("No .bag files found")

    rows = [analyze_bag(bag, args.odom_topic, args.robot_radius) for bag in bags]
    out_dir = args.out_dir or os.path.dirname(os.path.commonpath(bags))
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "stage2_batch_summary.csv")
    md_path = os.path.join(out_dir, "stage2_batch_summary.md")

    fieldnames = list(rows[0].keys())
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    write_markdown(rows, md_path)

    print("Analyzed {} bags".format(len(rows)))
    print("CSV: {}".format(csv_path))
    print("Markdown: {}".format(md_path))


if __name__ == "__main__":
    main()
