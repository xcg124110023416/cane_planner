#!/usr/bin/env python3

import datetime
import os
import sys


TOPICS = [
    "/clock",
    "/mpc/debug_metrics",
    "/mpc/stop_advice",
    "/mpc/stop_reason",
    "/mpc/path",
    "/mpc/best_traj",
    "/mpc/current_waypoint",
    "/mpc/waypoints",
    "/mpc/walking_corridors",
    "/localization_odom",
    "/cmd_vel_footprint",
    "/onboard_detector/dynamic_obstacles_info",
    "/onboard_detector/dynamic_bboxes",
    "/gazebo_pedestrian_truth/visualization",
]


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else "run"
    root_dir = os.environ.get("MPC_EVAL_RECORD_DIR", "/home/xcg/ws/records")
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(root_dir, "{}_{}".format(stamp, label))
    prefix = os.path.join(out_dir, "mpc_eval")

    os.makedirs(out_dir, exist_ok=True)

    metadata_path = os.path.join(out_dir, "topics.txt")
    with open(metadata_path, "w", encoding="utf-8") as f:
        f.write("label: {}\n".format(label))
        f.write("created_at: {}\n".format(stamp))
        f.write("bag_prefix: {}\n".format(prefix))
        f.write("topics:\n")
        for topic in TOPICS:
            f.write("  {}\n".format(topic))

    print("[record_mpc_eval] Writing bag to: {}_*.bag".format(prefix))
    print("[record_mpc_eval] Metadata: {}".format(metadata_path))
    print("[record_mpc_eval] Press Ctrl+C to stop recording.")
    sys.stdout.flush()

    argv = ["rosbag", "record", "-O", prefix] + TOPICS
    os.execvp("rosbag", argv)


if __name__ == "__main__":
    main()
