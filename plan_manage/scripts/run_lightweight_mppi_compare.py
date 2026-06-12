#!/usr/bin/env python3

import argparse
import datetime
import glob
import os
import signal
import subprocess
import sys
import time


WORKSPACE = "/home/xcg/ws"
DEFAULT_RECORD_ROOT = os.path.join(WORKSPACE, "records")


def timestamp():
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def run_name(planner, scenario, trial):
    return "planner{}_{}_trial{:02d}".format(planner, scenario, trial)


def popen_group(argv, env=None, stdout=None, stderr=None):
    return subprocess.Popen(
        argv,
        env=env,
        stdout=stdout,
        stderr=stderr,
        preexec_fn=os.setsid,
    )


def stop_process(proc, name, timeout=8.0):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    deadline = time.time() + timeout
    while time.time() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(0.1)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.time() + 3.0
    while time.time() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(0.1)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass
    print("[compare] force killed {}".format(name))


def wait_for_exit(proc, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        code = proc.poll()
        if code is not None:
            return code
        time.sleep(0.1)
    return None


def sleep_with_progress(duration, label, interval=5.0):
    if duration <= 0.0:
        return
    start = time.time()
    next_report = start
    end = start + duration
    while True:
        now = time.time()
        if now >= end:
            break
        if now >= next_report:
            elapsed = now - start
            remaining = max(0.0, end - now)
            print("[compare] {} recording {:.0f}/{:.0f}s, remaining {:.0f}s".format(
                label,
                elapsed,
                duration,
                remaining,
            ))
            sys.stdout.flush()
            next_report = now + interval
        time.sleep(min(0.2, max(0.0, end - now)))


def newest_record_dir(root, before_dirs):
    dirs = [
        path for path in glob.glob(os.path.join(root, "*"))
        if os.path.isdir(path) and path not in before_dirs
    ]
    if not dirs:
        return None
    return max(dirs, key=os.path.getmtime)


def find_bag(record_dir):
    bags = sorted(glob.glob(os.path.join(record_dir, "*.bag")))
    return bags[0] if bags else None


def write_run_metadata(path, args, planner, trial):
    with open(path, "w", encoding="utf-8") as f:
        f.write("planner: {}\n".format(planner))
        f.write("scenario: {}\n".format(args.scenario))
        f.write("trial: {}\n".format(trial))
        f.write("duration: {}\n".format(args.duration))
        f.write("startup_wait: {}\n".format(args.startup_wait))
        f.write("goal_start_time: {}\n".format(args.goal_start_time))
        f.write("goal_time: {}\n".format(args.goal_time))
        f.write("analyze_odom_topic: {}\n".format(args.odom_topic))
        f.write("goal_odom_topic: {}\n".format(args.goal_odom_topic))
        f.write("mpc_num_samples: {}\n".format(args.mpc_num_samples))
        f.write("mpc_w_risk: {}\n".format(args.mpc_w_risk))
        f.write("extra_launch_args: {}\n".format(" ".join(args.launch_arg)))


def tail_text(path, max_lines=40):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            lines = f.readlines()
    except OSError as exc:
        return "could not read {}: {}".format(path, exc)
    return "".join(lines[-max_lines:]).rstrip()


def analyze_record(record_dir, odom_topic, robot_radius):
    cmd = [
        "rosrun",
        "plan_manage",
        "analyze_mpc_eval.py",
        record_dir,
        "--lightweight",
        "--odom-topic",
        odom_topic,
        "--robot-radius",
        str(robot_radius),
    ]
    subprocess.check_call(cmd)


def summarize_records(root):
    output = os.path.join(root, "summary_table.csv")
    cmd = [
        "rosrun",
        "plan_manage",
        "summarize_mpc_eval.py",
        root,
        "-o",
        output,
    ]
    subprocess.check_call(cmd)
    return output


def run_one(args, planner, trial, root):
    label = run_name(planner, args.scenario, trial)
    print("\n[compare] === {} ===".format(label))
    sys.stdout.flush()

    os.makedirs(root, exist_ok=True)
    existing_dirs = set(
        path for path in glob.glob(os.path.join(root, "*")) if os.path.isdir(path)
    )

    launch_cmd = [
        "roslaunch",
        "plan_manage",
        "sim_kin_replan.launch",
        "planner:={}".format(planner),
        "pedestrian_scenario:={}".format(args.scenario),
        "mpc_num_samples:={}".format(args.mpc_num_samples),
        "mpc_w_risk:={}".format(args.mpc_w_risk),
    ] + args.launch_arg

    env = os.environ.copy()
    env["MPC_EVAL_RECORD_DIR"] = root

    launch_proc = None
    record_proc = None
    goal_proc = None
    launch_log = os.path.join(root, "{}_roslaunch.log".format(label))
    goal_log = os.path.join(root, "{}_goal.log".format(label))
    record_log = os.path.join(root, "{}_record.log".format(label))

    try:
        with open(launch_log, "w", encoding="utf-8") as launch_out:
            launch_proc = popen_group(
                launch_cmd,
                env=env,
                stdout=launch_out,
                stderr=subprocess.STDOUT,
            )
        time.sleep(args.startup_wait)
        if launch_proc.poll() is not None:
            raise RuntimeError("roslaunch exited early, see {}".format(launch_log))

        with open(record_log, "w", encoding="utf-8") as record_out:
            record_proc = popen_group(
                ["rosrun", "plan_manage", "record_lightweight_mpc_eval.sh", label],
                env=env,
                stdout=record_out,
                stderr=subprocess.STDOUT,
            )
        time.sleep(args.record_warmup)

        with open(goal_log, "w", encoding="utf-8") as goal_out:
            goal_proc = popen_group(
                [
                    "rosrun",
                    "plan_manage",
                    "publish_lightweight_fixed_goal.py",
                    "_scenario:={}".format(args.scenario),
                    "_start_time:={}".format(args.goal_start_time),
                    "_goal_time:={}".format(args.goal_time),
                    "_start_publish_duration:={}".format(args.start_publish_duration),
                    "_goal_publish_duration:={}".format(args.goal_publish_duration),
                    "_odom_topic:={}".format(args.goal_odom_topic),
                ],
                env=env,
                stdout=goal_out,
                stderr=subprocess.STDOUT,
            )

        goal_code = wait_for_exit(goal_proc, args.goal_timeout)
        if goal_code not in (0, None):
            raise RuntimeError(
                "fixed goal publisher failed, see {}\n--- goal log tail ---\n{}".format(
                    goal_log,
                    tail_text(goal_log),
                )
            )

        sleep_with_progress(args.duration, label)
    finally:
        stop_process(record_proc, "rosbag record")
        stop_process(goal_proc, "fixed goal publisher")
        stop_process(launch_proc, "roslaunch")

    record_dir = newest_record_dir(root, existing_dirs)
    if record_dir is None:
        raise RuntimeError("no record directory created for {}".format(label))
    bag = find_bag(record_dir)
    if bag is None:
        raise RuntimeError("no bag found in {}".format(record_dir))

    write_run_metadata(os.path.join(record_dir, "compare_run.txt"), args, planner, trial)
    analyze_record(record_dir, args.odom_topic, args.robot_radius)
    print("[compare] done {} -> {}".format(label, record_dir))
    return record_dir


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Run lightweight mixed-scenario MPPI comparisons. "
            "It records planner=3/4 bags, analyzes each run, and writes summary_table.csv."
        )
    )
    parser.add_argument("--scenario", default="mixed", help="Pedestrian/fixed-goal scenario name")
    parser.add_argument("--planners", nargs="+", type=int, default=[3, 4], help="Planner ids to compare")
    parser.add_argument("--trials", type=int, default=3, help="Trials per planner")
    parser.add_argument("--duration", type=float, default=45.0, help="Seconds to record after goal publisher exits")
    parser.add_argument("--startup-wait", type=float, default=6.0, help="Seconds to wait after roslaunch starts")
    parser.add_argument("--record-warmup", type=float, default=1.0, help="Seconds to record before publishing start/goal")
    parser.add_argument("--goal-start-time", type=float, default=1.0, help="publish_lightweight_fixed_goal start_time")
    parser.add_argument("--goal-time", type=float, default=3.0, help="publish_lightweight_fixed_goal goal_time")
    parser.add_argument("--goal-timeout", type=float, default=20.0, help="Seconds to wait for goal publisher to finish")
    parser.add_argument("--start-publish-duration", type=float, default=0.5, help="Repeat start publishing duration")
    parser.add_argument("--goal-publish-duration", type=float, default=0.5, help="Repeat goal publishing duration")
    parser.add_argument("--record-root", default=DEFAULT_RECORD_ROOT, help="Root directory for comparison records")
    parser.add_argument("--odom-topic", default="/sim_odom", help="Odometry topic used by analyzer")
    parser.add_argument(
        "--goal-odom-topic",
        default="/simulation_generator/odom",
        help="Odometry topic used only by publish_lightweight_fixed_goal.py while waiting for the fixed start",
    )
    parser.add_argument("--robot-radius", type=float, default=0.25, help="Robot radius for clearance metrics")
    parser.add_argument("--mpc-num-samples", type=int, default=200, help="MPPI samples passed to launch")
    parser.add_argument("--mpc-w-risk", type=float, default=2.0, help="Dynamic risk weight passed to launch")
    parser.add_argument(
        "--launch-arg",
        action="append",
        default=[],
        help="Extra roslaunch arg, e.g. mpc_enable_cpa:=true. Can be repeated.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    root = os.path.join(
        args.record_root,
        "{}_{}_planner{}_compare".format(
            timestamp(),
            args.scenario,
            "_".join(str(p) for p in args.planners),
        ),
    )
    os.makedirs(root, exist_ok=True)
    print("[compare] writing records under {}".format(root))
    print("[compare] metrics to inspect: success/final_goal_error, duration, path length, speed, stop count/time, min clearance, TTC, path deviation, heading change, yaw-rate smoothness, planning time")

    completed = []
    interrupted = False
    try:
        for trial in range(1, args.trials + 1):
            for planner in args.planners:
                completed.append(run_one(args, planner, trial, root))
    except KeyboardInterrupt:
        interrupted = True
        print("\n[compare] interrupted by user")
    finally:
        print("[compare] completed {} runs".format(len(completed)))

    if interrupted:
        return

    summary_csv = summarize_records(root)
    print("\n[compare] summary table: {}".format(summary_csv))
    print("[compare] compare planner=3 vs planner=4 mainly on:")
    print("  final_goal_error_m, duration_s, actual_path_length_m, odom_speed_mean_mps")
    print("  stop_count, total_stop_duration_s, actual_geom_clearance_min_m")
    print("  best_ttc_min_s, deviation_mean_m, heading_change_total_rad")
    print("  odom_abs_yaw_rate_mean_radps, odom_abs_yaw_accel_mean_radps2")


if __name__ == "__main__":
    main()
