#!/usr/bin/env python3

import argparse
import glob
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag


DEFAULT_ODOM_TOPIC = "/sim_odom"
DYN_OBS_TOPIC = "/onboard_detector/dynamic_obstacles_info"
WAYPOINTS_TOPIC = "/mpc/waypoints"


def resolve_bag_path(path):
    if os.path.isdir(path):
        bags = sorted(glob.glob(os.path.join(path, "*.bag")))
        if not bags:
            raise RuntimeError("No .bag files found in {}".format(path))
        return bags[0]
    return path


def parse_case(spec):
    if "=" not in spec:
        raise ValueError("case must be LABEL=PATH, got {}".format(spec))
    label, path = spec.split("=", 1)
    label = label.strip()
    path = path.strip()
    if not label or not path:
        raise ValueError("case must be LABEL=PATH, got {}".format(spec))
    return label, path


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_case(label, path, odom_topic, robot_radius):
    bag_path = resolve_bag_path(path)
    odom = []
    waypoints = []
    ped_tracks = {}
    latest_odom = None
    closest = None

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, stamp in bag.read_messages(
                topics=[odom_topic, WAYPOINTS_TOPIC, DYN_OBS_TOPIC]):
            if topic == odom_topic:
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                yaw = yaw_from_quaternion(msg.pose.pose.orientation)
                latest_odom = (x, y, yaw, stamp.to_sec())
                odom.append((x, y))

            elif topic == WAYPOINTS_TOPIC:
                pts = []
                if hasattr(msg, "poses"):
                    for pose in msg.poses:
                        pts.append((pose.pose.position.x, pose.pose.position.y))
                elif hasattr(msg, "points"):
                    for point in msg.points:
                        pts.append((point.x, point.y))
                if pts:
                    waypoints = pts

            elif topic == DYN_OBS_TOPIC and latest_odom is not None:
                rx, ry, _, _ = latest_odom
                for idx, pos in enumerate(msg.position):
                    ped_tracks.setdefault(idx, []).append((pos.x, pos.y))
                    ped_radius = 0.0
                    if idx < len(msg.size):
                        ped_radius = 0.5 * max(msg.size[idx].x, msg.size[idx].y)
                    center = math.hypot(pos.x - rx, pos.y - ry)
                    clearance = center - ped_radius - robot_radius
                    if closest is None or clearance < closest["clearance"]:
                        closest = {
                            "robot": (rx, ry),
                            "ped": (pos.x, pos.y),
                            "clearance": clearance,
                            "ped_id": idx,
                        }

    return {
        "label": label,
        "bag": bag_path,
        "odom": odom,
        "waypoints": waypoints,
        "ped_tracks": ped_tracks,
        "closest": closest,
    }


def expand_limits(ax, all_points):
    if not all_points:
        return
    xs = [p[0] for p in all_points]
    ys = [p[1] for p in all_points]
    margin = 0.8
    ax.set_xlim(min(xs) - margin, max(xs) + margin)
    ax.set_ylim(min(ys) - margin, max(ys) + margin)


def plot_cases(cases, output, title, show_pedestrians):
    fig, ax = plt.subplots(figsize=(7.2, 6.0), dpi=180)
    colors = plt.cm.tab10.colors
    all_points = []

    if cases and cases[0]["waypoints"]:
        wx = [p[0] for p in cases[0]["waypoints"]]
        wy = [p[1] for p in cases[0]["waypoints"]]
        ax.plot(wx, wy, color="0.25", linewidth=2.0, linestyle="--",
                label="Global waypoints")
        all_points.extend(cases[0]["waypoints"])

    for i, case in enumerate(cases):
        color = colors[i % len(colors)]
        if case["odom"]:
            xs = [p[0] for p in case["odom"]]
            ys = [p[1] for p in case["odom"]]
            ax.plot(xs, ys, color=color, linewidth=2.2, label=case["label"])
            ax.scatter(xs[0], ys[0], color=color, marker="o", s=20)
            ax.scatter(xs[-1], ys[-1], color=color, marker="x", s=32)
            all_points.extend(case["odom"])

        if case["closest"] is not None:
            rx, ry = case["closest"]["robot"]
            px, py = case["closest"]["ped"]
            ax.plot([rx, px], [ry, py], color=color, linewidth=1.0, alpha=0.6)
            ax.scatter([rx], [ry], color=color, marker="D", s=24)
            ax.scatter([px], [py], color=color, marker="s", s=22,
                       facecolors="none")
            ax.annotate("{:.2f}m".format(case["closest"]["clearance"]),
                        xy=(rx, ry), xytext=(4, 4),
                        textcoords="offset points", fontsize=7, color=color)
            all_points.extend([case["closest"]["robot"], case["closest"]["ped"]])

    if show_pedestrians and cases:
        for idx, pts in cases[0]["ped_tracks"].items():
            if len(pts) < 2:
                continue
            px = [p[0] for p in pts]
            py = [p[1] for p in pts]
            ax.plot(px, py, color="0.1", linewidth=1.0, linestyle=":",
                    alpha=0.55, label="Ped {}".format(idx))
            all_points.extend(pts)

    ax.set_title(title)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle=":", linewidth=0.6, alpha=0.7)
    expand_limits(ax, all_points)
    ax.legend(loc="best", fontsize=7, framealpha=0.9)
    fig.tight_layout()
    os.makedirs(os.path.dirname(output), exist_ok=True)
    fig.savefig(output)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description="Plot MPC evaluation trajectories from rosbag records.")
    parser.add_argument("--case", action="append", required=True,
                        help="Case as LABEL=record_dir_or_bag. Repeatable.")
    parser.add_argument("--output", required=True, help="Output PNG path")
    parser.add_argument("--title", default="MPC evaluation trajectories")
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--robot-radius", type=float, default=0.35)
    parser.add_argument("--no-pedestrians", action="store_true",
                        help="Do not draw pedestrian tracks")
    args = parser.parse_args()

    cases = []
    for spec in args.case:
        label, path = parse_case(spec)
        cases.append(load_case(label, path, args.odom_topic, args.robot_radius))

    plot_cases(cases, args.output, args.title, not args.no_pedestrians)
    print("Wrote figure: {}".format(args.output))


if __name__ == "__main__":
    main()
