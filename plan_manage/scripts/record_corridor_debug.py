#!/usr/bin/env python3

import argparse
import datetime
import os
import sys


TOPICS = [
    "/clock",
    "/rosout",
    "/rosout_agg",
    "/mpc/walking_corridors",
    "/mpc/convex_corridor",
    "/mpc/convex_corridor_debug",
    "/mpc/debug_metrics",
    "/mpc/stop_advice",
    "/mpc/stop_reason",
    "/mpc/path",
    "/mpc/best_traj",
    "/mpc/current_waypoint",
    "/mpc/waypoints",
    "/astar/path",
    "/simulation_generator/odom",
    "/sim_odom",
    "/onboard_detector/dynamic_obstacles_info",
    "/mpc/interaction_scene",
    "/mpc/interaction_mode",
    "/mpc/interaction_debug",
    "/pedestrian_sim/visualization",
]


def main():
    parser = argparse.ArgumentParser(
        description="Record a lightweight bag for walking-corridor stuck diagnosis."
    )
    parser.add_argument("label", nargs="?", default="corridor_debug")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Optional rosbag recording duration in seconds.")
    parser.add_argument("--root-dir", default=os.environ.get("MPC_EVAL_RECORD_DIR", "/home/xcg/ws/records"))
    args = parser.parse_args()

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(args.root_dir, "{}_{}".format(stamp, args.label))
    prefix = os.path.join(out_dir, "corridor_debug")
    os.makedirs(out_dir, exist_ok=True)

    metadata_path = os.path.join(out_dir, "topics.txt")
    with open(metadata_path, "w", encoding="utf-8") as f:
        f.write("label: {}\n".format(args.label))
        f.write("created_at: {}\n".format(stamp))
        f.write("bag_prefix: {}\n".format(prefix))
        f.write("duration: {}\n".format(args.duration if args.duration > 0 else "until Ctrl+C"))
        f.write("topics:\n")
        for topic in TOPICS:
            f.write("  {}\n".format(topic))

    print("[record_corridor_debug] Writing bag to: {}_*.bag".format(prefix))
    print("[record_corridor_debug] Metadata: {}".format(metadata_path))
    if args.duration > 0:
        print("[record_corridor_debug] Duration: {:.1f}s".format(args.duration))
    else:
        print("[record_corridor_debug] Press Ctrl+C to stop recording.")
    sys.stdout.flush()

    argv = ["rosbag", "record", "-O", prefix]
    if args.duration > 0:
        argv.append("--duration={:.3f}".format(args.duration))
    argv += TOPICS
    os.execvp("rosbag", argv)


if __name__ == "__main__":
    main()
