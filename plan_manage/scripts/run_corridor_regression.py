#!/usr/bin/env python3

import argparse
import glob
import os
import signal
import subprocess
import sys
import time


DEFAULT_RECORD_ROOT = "/home/xcg/ws/records"


def run_cmd(cmd, check=True):
    print("[run_corridor_regression] $ {}".format(" ".join(cmd)))
    sys.stdout.flush()
    return subprocess.run(cmd, check=check)


def start_process(cmd, name):
    print("[run_corridor_regression] starting {}: {}".format(name, " ".join(cmd)))
    sys.stdout.flush()
    return subprocess.Popen(cmd, preexec_fn=os.setsid)


def stop_process(proc, name, grace_sec=8.0):
    if proc is None or proc.poll() is not None:
        return
    print("[run_corridor_regression] stopping {}".format(name))
    sys.stdout.flush()
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    deadline = time.time() + grace_sec
    while time.time() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(0.2)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass


def latest_record_dir(root_dir, label):
    pattern = os.path.join(root_dir, "*_{}".format(label))
    matches = [p for p in glob.glob(pattern) if os.path.isdir(p)]
    if not matches:
        return None
    return max(matches, key=os.path.getmtime)


def parse_launch_arg(raw):
    if ":=" not in raw:
        raise argparse.ArgumentTypeError(
            "launch args must use ROS syntax name:=value, got '{}'".format(raw)
        )
    return raw


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Run the lightweight corridor regression flow: launch simulation, "
            "record a debug bag, publish the fixed mixed-scenario goal, then analyze."
        )
    )
    parser.add_argument("label", nargs="?", default="mixed_regression")
    parser.add_argument("--duration", type=float, default=45.0,
                        help="Rosbag recording duration in seconds.")
    parser.add_argument("--scenario", default="mixed",
                        help="Scenario name used by sim_kin_replan and fixed-goal publisher.")
    parser.add_argument("--planner", default="3",
                        help="Planner arg for sim_kin_replan.launch.")
    parser.add_argument("--record-root", default=os.environ.get("MPC_EVAL_RECORD_DIR", DEFAULT_RECORD_ROOT))
    parser.add_argument("--launch-wait", type=float, default=6.0,
                        help="Seconds to wait after roslaunch before recording.")
    parser.add_argument("--record-warmup", type=float, default=1.0,
                        help="Seconds to record before publishing start/goal.")
    parser.add_argument("--no-launch", action="store_true",
                        help="Use an already-running simulation instead of starting roslaunch.")
    parser.add_argument("--no-goal", action="store_true",
                        help="Do not publish the fixed start/goal.")
    parser.add_argument("--no-analyze", action="store_true",
                        help="Record only; skip analyze_corridor_debug.py.")
    parser.add_argument("--keep-launch", action="store_true",
                        help="Leave roslaunch running after recording.")
    parser.add_argument("--launch-arg", action="append", default=[], type=parse_launch_arg,
                        help="Extra sim_kin_replan.launch arg, e.g. name:=value. May repeat.")
    args = parser.parse_args()

    launch_proc = None
    record_proc = None
    try:
        if not args.no_launch:
            launch_cmd = [
                "roslaunch", "plan_manage", "sim_kin_replan.launch",
                "planner:={}".format(args.planner),
                "pedestrian_scenario:={}".format(args.scenario),
            ] + args.launch_arg
            launch_proc = start_process(launch_cmd, "simulation")
            time.sleep(max(0.0, args.launch_wait))

        record_cmd = [
            "rosrun", "plan_manage", "record_corridor_debug.py",
            args.label,
            "--duration", "{:.3f}".format(args.duration),
            "--root-dir", args.record_root,
        ]
        record_proc = start_process(record_cmd, "bag recorder")
        time.sleep(max(0.0, args.record_warmup))

        if not args.no_goal:
            run_cmd([
                "rosrun", "plan_manage", "publish_lightweight_fixed_goal.py",
                "_scenario:={}".format(args.scenario),
            ])

        print("[run_corridor_regression] waiting for recorder to finish")
        sys.stdout.flush()
        rc = record_proc.wait()
        if rc != 0:
            raise RuntimeError("bag recorder exited with code {}".format(rc))

    except KeyboardInterrupt:
        print("\n[run_corridor_regression] interrupted")
        return 130
    finally:
        stop_process(record_proc, "bag recorder", grace_sec=2.0)
        if launch_proc is not None and not args.keep_launch:
            stop_process(launch_proc, "simulation")

    out_dir = latest_record_dir(args.record_root, args.label)
    if out_dir is None:
        print("[run_corridor_regression] no record directory found for label {}".format(args.label), file=sys.stderr)
        return 2

    print("[run_corridor_regression] record directory: {}".format(out_dir))
    if not args.no_analyze:
        run_cmd(["rosrun", "plan_manage", "analyze_corridor_debug.py", out_dir])
        summary = os.path.join(out_dir, "corridor_debug_summary.txt")
        if os.path.exists(summary):
            print("[run_corridor_regression] summary: {}".format(summary))

    return 0


if __name__ == "__main__":
    sys.exit(main())
