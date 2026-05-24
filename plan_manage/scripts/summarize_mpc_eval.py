#!/usr/bin/env python3

import argparse
import csv
import glob
import os
import re


FIELDS = [
    "run_name",
    "summary_path",
    "bag",
    "duration_s",
    "stop_count",
    "total_stop_duration_s",
    "stop_time_ratio",
    "stop_reasons",
    "mpc_plan_time_mean_ms",
    "mpc_plan_time_max_ms",
    "valid_sample_ratio_mean",
    "plan_valid_ratio",
    "actual_path_length_m",
    "path_efficiency",
    "global_waypoint_path_length_m",
    "final_goal_error_m",
    "odom_speed_mean_mps",
    "odom_speed_max_mps",
    "deviation_mean_m",
    "deviation_max_m",
    "best_clearance_min_m",
    "best_ttc_min_s",
    "global_clearance_min_m",
    "global_ttc_min_s",
    "actual_geom_clearance_min_m",
]


def parse_number(text):
    match = re.search(r"[-+]?\d+(?:\.\d+)?", text)
    return match.group(0) if match else ""


def parse_stat_line(line):
    result = {}
    for key in ("count", "mean", "min", "max"):
        match = re.search(r"\b{}=([-+]?\d+(?:\.\d+)?)".format(key), line)
        if match:
            result[key] = match.group(1)
    return result


def parse_summary(path):
    row = {field: "" for field in FIELDS}
    row["run_name"] = os.path.basename(os.path.dirname(path))
    row["summary_path"] = path

    with open(path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            key = key.strip()
            value = value.strip()

            if key == "bag":
                row["bag"] = value
            elif key == "duration":
                row["duration_s"] = parse_number(value)
            elif key == "stop_count":
                row["stop_count"] = parse_number(value)
            elif key == "total_stop_duration":
                row["total_stop_duration_s"] = parse_number(value)
            elif key == "stop_time_ratio":
                row["stop_time_ratio"] = parse_number(value)
            elif key == "stop_reasons":
                row["stop_reasons"] = value
            elif key == "plan_valid_ratio":
                row["plan_valid_ratio"] = parse_number(value)
            elif key == "actual_path_length":
                row["actual_path_length_m"] = parse_number(value)
            elif key == "path_efficiency":
                row["path_efficiency"] = parse_number(value)
            elif key == "global_waypoint_path_length":
                row["global_waypoint_path_length_m"] = parse_number(value)
            elif key == "final_goal_error":
                row["final_goal_error_m"] = parse_number(value)
            elif key == "mpc_plan_time":
                stats = parse_stat_line(value)
                row["mpc_plan_time_mean_ms"] = stats.get("mean", "")
                row["mpc_plan_time_max_ms"] = stats.get("max", "")
            elif key == "valid_sample_ratio":
                stats = parse_stat_line(value)
                row["valid_sample_ratio_mean"] = stats.get("mean", "")
            elif key == "odom_speed":
                stats = parse_stat_line(value)
                row["odom_speed_mean_mps"] = stats.get("mean", "")
                row["odom_speed_max_mps"] = stats.get("max", "")
            elif key == "deviation_from_global_waypoints":
                stats = parse_stat_line(value)
                row["deviation_mean_m"] = stats.get("mean", "")
                row["deviation_max_m"] = stats.get("max", "")
            elif key == "best_min_dynamic_clearance":
                stats = parse_stat_line(value)
                row["best_clearance_min_m"] = stats.get("min", "")
            elif key == "best_min_cpa_time":
                stats = parse_stat_line(value)
                row["best_ttc_min_s"] = stats.get("min", "")
            elif key == "global_min_dynamic_clearance":
                stats = parse_stat_line(value)
                row["global_clearance_min_m"] = stats.get("min", "")
            elif key == "global_min_cpa_time":
                stats = parse_stat_line(value)
                row["global_ttc_min_s"] = stats.get("min", "")
            elif key == "actual_robot_ped_geom_clearance":
                stats = parse_stat_line(value)
                row["actual_geom_clearance_min_m"] = stats.get("min", "")

    return row


def find_summaries(root):
    if os.path.isfile(root):
        return [root]
    return sorted(glob.glob(os.path.join(root, "**", "summary.txt"), recursive=True))


def main():
    parser = argparse.ArgumentParser(description="Collect MPC evaluation summary.txt files into a CSV table.")
    parser.add_argument("root", help="Records root directory or a single summary.txt")
    parser.add_argument("-o", "--output", default="", help="Output CSV path. Default: <root>/summary_table.csv")
    args = parser.parse_args()

    summaries = find_summaries(args.root)
    if not summaries:
        raise RuntimeError("No summary.txt files found under {}".format(args.root))

    output = args.output
    if not output:
        output = os.path.join(args.root if os.path.isdir(args.root) else os.path.dirname(args.root),
                              "summary_table.csv")

    rows = [parse_summary(path) for path in summaries]
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    print("Wrote {} rows to {}".format(len(rows), output))
    for row in rows:
        print("{}: stops={} stop_time={}s min_clearance={}m path_eff={}".format(
            row["run_name"],
            row["stop_count"],
            row["total_stop_duration_s"],
            row["actual_geom_clearance_min_m"],
            row["path_efficiency"]))


if __name__ == "__main__":
    main()
