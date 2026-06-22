#!/usr/bin/env python3

import argparse
import glob
import os
import re
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


def parse_summary(summary_path):
    metrics = {
        "rosout_stop_logs": None,
        "non_ok_stop_count": 0,
        "mpc_path_length": None,
        "astar_path_length": None,
        "static_label_count": None,
        "corridor_label_count": None,
        "dynamic_label_count": None,
    }
    if not os.path.exists(summary_path):
        raise RuntimeError("summary not found: {}".format(summary_path))

    in_stop_reasons = False
    with open(summary_path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.rstrip()
            stripped = line.strip()
            if line.startswith("rosout_stop_logs:"):
                metrics["rosout_stop_logs"] = int(line.split(":", 1)[1].strip())
                continue

            match = re.search(r"mpc_path_points_last:\s+\d+\s+length=([-+0-9.]+)m", line)
            if match:
                metrics["mpc_path_length"] = float(match.group(1))
                continue
            match = re.search(r"astar_path_points_last:\s+\d+\s+length=([-+0-9.]+)m", line)
            if match:
                metrics["astar_path_length"] = float(match.group(1))
                continue
            match = re.search(r"st=1 count:\s+(\d+)\s+/\s+(\d+)", line)
            if match:
                metrics["static_label_count"] = int(match.group(1))
                metrics["corridor_label_count"] = int(match.group(2))
                continue
            match = re.search(r"dyn>0 count:\s+(\d+)\s+/\s+(\d+)", line)
            if match:
                metrics["dynamic_label_count"] = int(match.group(1))
                if metrics["corridor_label_count"] is None:
                    metrics["corridor_label_count"] = int(match.group(2))
                continue

            if stripped == "Stop reasons:":
                in_stop_reasons = True
                continue
            if in_stop_reasons:
                if not line.startswith("  "):
                    in_stop_reasons = False
                    continue
                if stripped == "none":
                    continue
                if ":" not in stripped:
                    continue
                reason, count_text = stripped.rsplit(":", 1)
                count = int(count_text.strip())
                if reason != "OK":
                    metrics["non_ok_stop_count"] += count

    return metrics


def validate_summary(summary_path, args):
    metrics = parse_summary(summary_path)
    failures = []

    if args.max_stop_logs is not None:
        value = metrics.get("rosout_stop_logs")
        if value is None or value > args.max_stop_logs:
            failures.append("rosout_stop_logs={} > {}".format(value, args.max_stop_logs))

    if args.max_non_ok_stops is not None:
        value = metrics.get("non_ok_stop_count")
        if value is None or value > args.max_non_ok_stops:
            failures.append("non_OK_stop_count={} > {}".format(value, args.max_non_ok_stops))

    if args.min_mpc_path_length is not None:
        value = metrics.get("mpc_path_length")
        if value is None or value < args.min_mpc_path_length:
            failures.append("mpc_path_length={} < {:.3f}".format(value, args.min_mpc_path_length))

    if args.min_path_length_ratio is not None:
        mpc_len = metrics.get("mpc_path_length")
        astar_len = metrics.get("astar_path_length")
        ratio = (mpc_len / astar_len) if astar_len and astar_len > 1e-6 and mpc_len is not None else None
        if ratio is None or ratio < args.min_path_length_ratio:
            failures.append("mpc/astar_length_ratio={} < {:.3f}".format(ratio, args.min_path_length_ratio))

    if args.max_path_length_ratio is not None:
        mpc_len = metrics.get("mpc_path_length")
        astar_len = metrics.get("astar_path_length")
        ratio = (mpc_len / astar_len) if astar_len and astar_len > 1e-6 and mpc_len is not None else None
        if ratio is None or ratio > args.max_path_length_ratio:
            failures.append("mpc/astar_length_ratio={} > {:.3f}".format(ratio, args.max_path_length_ratio))

    if args.max_static_label_ratio is not None:
        st = metrics.get("static_label_count")
        total = metrics.get("corridor_label_count")
        ratio = (float(st) / float(total)) if total else None
        if ratio is None or ratio > args.max_static_label_ratio:
            failures.append("static_label_ratio={} > {:.3f}".format(ratio, args.max_static_label_ratio))

    if args.min_dynamic_label_ratio is not None:
        dyn = metrics.get("dynamic_label_count")
        total = metrics.get("corridor_label_count")
        ratio = (float(dyn) / float(total)) if total else None
        if ratio is None or ratio < args.min_dynamic_label_ratio:
            failures.append("dynamic_label_ratio={} < {:.3f}".format(ratio, args.min_dynamic_label_ratio))

    print("[run_corridor_regression] parsed metrics: {}".format(metrics))
    if failures:
        print("[run_corridor_regression] FAILED checks:")
        for failure in failures:
            print("  - {}".format(failure))
        return False
    print("[run_corridor_regression] PASSED checks")
    return True


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
    parser.add_argument("--start-publish-duration", type=float, default=0.5,
                        help="Seconds to keep publishing the fixed start pose.")
    parser.add_argument("--goal-publish-duration", type=float, default=1.0,
                        help="Seconds to keep publishing the fixed goal pose.")
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
    parser.add_argument("--max-stop-logs", type=int, default=None,
                        help="Fail if rosout_stop_logs is larger than this value.")
    parser.add_argument("--max-non-ok-stops", type=int, default=None,
                        help="Fail if non-OK stop reason samples exceed this value.")
    parser.add_argument("--min-mpc-path-length", type=float, default=None,
                        help="Fail if latest MPC path length is shorter than this many meters.")
    parser.add_argument("--min-path-length-ratio", type=float, default=None,
                        help="Fail if latest MPC path length / A* path length is below this ratio.")
    parser.add_argument("--max-path-length-ratio", type=float, default=None,
                        help="Fail if latest MPC path length / A* path length is above this ratio.")
    parser.add_argument("--max-static-label-ratio", type=float, default=None,
                        help="Fail if st=1 corridor label ratio exceeds this value.")
    parser.add_argument("--min-dynamic-label-ratio", type=float, default=None,
                        help="Fail if dyn>0 corridor label ratio is below this value.")
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
                "_start_publish_duration:={:.3f}".format(args.start_publish_duration),
                "_goal_publish_duration:={:.3f}".format(args.goal_publish_duration),
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
            checks_requested = any([
                args.max_stop_logs is not None,
                args.max_non_ok_stops is not None,
                args.min_mpc_path_length is not None,
                args.min_path_length_ratio is not None,
                args.max_path_length_ratio is not None,
                args.max_static_label_ratio is not None,
                args.min_dynamic_label_ratio is not None,
            ])
            if checks_requested and not validate_summary(summary, args):
                return 3

    return 0


if __name__ == "__main__":
    sys.exit(main())
