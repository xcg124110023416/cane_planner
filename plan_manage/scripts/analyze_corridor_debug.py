#!/usr/bin/env python3

import argparse
import csv
import glob
import math
import os
import re
from collections import Counter


CORRIDOR_TOPIC = "/mpc/walking_corridors"
DEBUG_TOPIC = "/mpc/debug_metrics"
STOP_ADVICE_TOPIC = "/mpc/stop_advice"
STOP_REASON_TOPIC = "/mpc/stop_reason"
ODOM_TOPICS = ["/sim_odom", "/simulation_generator/odom", "/localization_odom"]
ASTAR_TOPIC = "/astar/path"
MPC_PATH_TOPIC = "/mpc/path"
WAYPOINTS_TOPIC = "/mpc/waypoints"
ROSOUT_TOPICS = ["/rosout", "/rosout_agg"]

LABEL_RE = re.compile(
    r"(?:selected\s+)?off=(?P<off>[-+0-9.]+)\s+"
    r"cost=(?P<cost>[-+0-9.]+)\s+"
    r"w=(?P<w>[-+0-9.]+)\s+"
    r"st=(?P<st>[01])\s+"
    r"dyn=(?P<dyn>[-+0-9]+)"
    r"(?:\s+tr=(?P<tr>[01]))?"
    r"(?:\s+sw_s=(?P<sw_s>[-+0-9.]+)\s+sw=\((?P<sw_x>[-+0-9.]+),(?P<sw_y>[-+0-9.]+)\))?"
)


def resolve_bag_path(path):
    if os.path.isdir(path):
        bags = sorted(glob.glob(os.path.join(path, "*.bag")))
        if not bags:
            raise RuntimeError("No .bag files found in {}".format(path))
        return bags[0]
    return path


def rel_time(t, t0):
    return (t - t0).to_sec() if t0 is not None else 0.0


def path_points(msg):
    if hasattr(msg, "poses"):
        return [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
    if hasattr(msg, "points"):
        return [(p.x, p.y) for p in msg.points]
    if hasattr(msg, "pose"):
        return [(msg.pose.position.x, msg.pose.position.y)]
    return []


def path_length(points):
    total = 0.0
    for i in range(1, len(points)):
        total += math.hypot(points[i][0] - points[i - 1][0],
                            points[i][1] - points[i - 1][1])
    return total


def nearest_by_time(samples, t_rel, max_dt=0.35):
    if not samples:
        return None
    best = min(samples, key=lambda s: abs(s["t"] - t_rel))
    if abs(best["t"] - t_rel) > max_dt:
        return None
    return best


def parse_corridor_labels(msg, t_rel):
    samples = []
    for marker in msg.markers:
        text = getattr(marker, "text", "")
        if not text:
            continue
        match = LABEL_RE.search(text)
        if not match:
            continue
        groups = match.groupdict()
        samples.append({
            "t": t_rel,
            "selected": 1 if text.startswith("selected") else 0,
            "off": float(groups["off"]),
            "cost": float(groups["cost"]),
            "w": float(groups["w"]),
            "st": int(groups["st"]),
            "dyn": int(groups["dyn"]),
            "tr": int(groups["tr"]) if groups.get("tr") else 0,
            "sw_s": float(groups["sw_s"]) if groups.get("sw_s") else float("nan"),
            "sw_x": float(groups["sw_x"]) if groups.get("sw_x") else float("nan"),
            "sw_y": float(groups["sw_y"]) if groups.get("sw_y") else float("nan"),
            "x": marker.pose.position.x,
            "y": marker.pose.position.y,
            "text": text,
        })
    return samples


def summarize_boolean_runs(samples, key):
    runs = []
    active_start = None
    last_t = None
    for sample in samples:
        value = bool(sample.get(key, 0))
        if value and active_start is None:
            active_start = sample["t"]
        if not value and active_start is not None:
            runs.append((active_start, last_t if last_t is not None else sample["t"]))
            active_start = None
        last_t = sample["t"]
    if active_start is not None:
        runs.append((active_start, last_t if last_t is not None else active_start))
    if not runs:
        return None
    return max(runs, key=lambda r: r[1] - r[0])


def analyze_bag(bag_path):
    import rosbag

    corridor_samples = []
    debug_samples = []
    stop_reason_samples = []
    stop_advice_samples = []
    rosout_stop_samples = []
    odom_samples = []
    latest_astar = []
    latest_mpc_path = []
    latest_waypoints = []
    t0 = None

    topics = [
        CORRIDOR_TOPIC, DEBUG_TOPIC, STOP_ADVICE_TOPIC, STOP_REASON_TOPIC,
        ASTAR_TOPIC, MPC_PATH_TOPIC, WAYPOINTS_TOPIC,
    ] + ODOM_TOPICS + ROSOUT_TOPICS

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            if t0 is None:
                t0 = t
            tr = rel_time(t, t0)

            if topic == CORRIDOR_TOPIC:
                corridor_samples.extend(parse_corridor_labels(msg, tr))
            elif topic == DEBUG_TOPIC:
                data = list(msg.data)
                debug_samples.append({
                    "t": tr,
                    "valid_ratio": data[1] if len(data) > 1 else float("nan"),
                    "plan_valid": data[8] if len(data) > 8 else float("nan"),
                    "best_clearance": data[9] if len(data) > 9 else float("nan"),
                    "corridor_reject_count": data[11] if len(data) > 11 else float("nan"),
                    "convex_reject_count": data[12] if len(data) > 12 else float("nan"),
                    "convex_max_violation": data[13] if len(data) > 13 else float("nan"),
                    "convex_segments": data[14] if len(data) > 14 else float("nan"),
                    "dynamic_reject_count": data[5] if len(data) > 5 else float("nan"),
                    "static_reject_count": data[6] if len(data) > 6 else float("nan"),
                })
            elif topic == STOP_REASON_TOPIC:
                stop_reason_samples.append({"t": tr, "reason": msg.data})
            elif topic == STOP_ADVICE_TOPIC:
                stop_advice_samples.append({"t": tr, "active": bool(msg.data)})
            elif topic in ROSOUT_TOPICS:
                text = getattr(msg, "msg", "")
                if "STOP advice enforced" in text or "CORRIDOR_INFEASIBLE" in text:
                    rosout_stop_samples.append({
                        "t": tr,
                        "level": getattr(msg, "level", 0),
                        "name": getattr(msg, "name", ""),
                        "msg": text,
                    })
            elif topic in ODOM_TOPICS:
                odom_samples.append({
                    "t": tr,
                    "topic": topic,
                    "x": msg.pose.pose.position.x,
                    "y": msg.pose.pose.position.y,
                })
            elif topic == ASTAR_TOPIC:
                latest_astar = path_points(msg)
            elif topic == MPC_PATH_TOPIC:
                latest_mpc_path = path_points(msg)
            elif topic == WAYPOINTS_TOPIC:
                latest_waypoints = path_points(msg)

    return {
        "bag_path": bag_path,
        "corridor": corridor_samples,
        "debug": debug_samples,
        "stop_reason": stop_reason_samples,
        "stop_advice": stop_advice_samples,
        "rosout_stop": rosout_stop_samples,
        "odom": odom_samples,
        "astar": latest_astar,
        "mpc_path": latest_mpc_path,
        "waypoints": latest_waypoints,
    }


def write_outputs(result, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "corridor_debug_samples.csv")
    summary_path = os.path.join(out_dir, "corridor_debug_summary.txt")

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        fieldnames = [
            "t", "selected", "off", "cost", "w", "st", "dyn", "tr",
            "sw_s", "sw_x", "sw_y", "x", "y", "text"
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in result["corridor"]:
            writer.writerow(row)

    corridor = result["corridor"]
    debug = result["debug"]
    reasons = result["stop_reason"]
    rosout_stop = result["rosout_stop"]
    stop_reasons = Counter(r["reason"] for r in reasons)
    st_count = sum(1 for s in corridor if s["st"])
    dyn_count = sum(1 for s in corridor if s["dyn"] > 0)
    trunc_count = sum(1 for s in corridor if s.get("tr", 0))
    feasible_like_count = sum(1 for s in corridor if not s["st"] and s["w"] >= 0.25)
    widths = [s["w"] for s in corridor if math.isfinite(s["w"])]
    longest_st = summarize_boolean_runs(corridor, "st")
    convex_rejects = [d["convex_reject_count"] for d in debug
                      if math.isfinite(d.get("convex_reject_count", float("nan")))]
    convex_violations = [d["convex_max_violation"] for d in debug
                         if math.isfinite(d.get("convex_max_violation", float("nan")))]
    convex_segments = [d["convex_segments"] for d in debug
                       if math.isfinite(d.get("convex_segments", float("nan")))]

    stop_with_context = []
    for reason in reasons:
        if reason["reason"] == "OK":
            continue
        c = nearest_by_time(corridor, reason["t"])
        d = nearest_by_time(debug, reason["t"])
        stop_with_context.append((reason, c, d))

    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("Corridor Debug Summary\n")
        f.write("======================\n")
        f.write("bag: {}\n".format(result["bag_path"]))
        f.write("corridor_label_samples: {}\n".format(len(corridor)))
        f.write("debug_samples: {}\n".format(len(debug)))
        f.write("rosout_stop_logs: {}\n".format(len(rosout_stop)))
        f.write("odom_samples: {}\n".format(len(result["odom"])))
        f.write("astar_path_points_last: {} length={:.3f}m\n".format(
            len(result["astar"]), path_length(result["astar"])))
        f.write("mpc_path_points_last: {} length={:.3f}m\n".format(
            len(result["mpc_path"]), path_length(result["mpc_path"])))
        f.write("waypoints_last: {} length={:.3f}m\n".format(
            len(result["waypoints"]), path_length(result["waypoints"])))
        f.write("\nStop reasons:\n")
        if stop_reasons:
            for reason, count in stop_reasons.most_common():
                f.write("  {}: {}\n".format(reason, count))
        else:
            f.write("  none\n")

        f.write("\nCorridor labels:\n")
        f.write("  st=1 count: {} / {}\n".format(st_count, len(corridor)))
        f.write("  dyn>0 count: {} / {}\n".format(dyn_count, len(corridor)))
        f.write("  tr=1 count: {} / {}\n".format(trunc_count, len(corridor)))
        f.write("  w>=0.25 and st=0 count: {} / {}\n".format(feasible_like_count, len(corridor)))
        if widths:
            f.write("  width: mean={:.3f} min={:.3f} max={:.3f}\n".format(
                sum(widths) / len(widths), min(widths), max(widths)))
        if longest_st:
            f.write("  longest st=1 run: {:.2f}s -> {:.2f}s duration={:.2f}s\n".format(
                longest_st[0], longest_st[1], longest_st[1] - longest_st[0]))

        f.write("\nConvex corridor:\n")
        if convex_segments:
            f.write("  segments: mean={:.2f} min={:.0f} max={:.0f}\n".format(
                sum(convex_segments) / len(convex_segments),
                min(convex_segments), max(convex_segments)))
        else:
            f.write("  segments: none\n")
        if convex_rejects:
            f.write("  reject: mean={:.2f} max={:.0f}\n".format(
                sum(convex_rejects) / len(convex_rejects), max(convex_rejects)))
        if convex_violations:
            f.write("  max violation: mean={:.3f} max={:.3f}\n".format(
                sum(convex_violations) / len(convex_violations),
                max(convex_violations)))

        f.write("\nFirst non-OK stop contexts:\n")
        if not stop_with_context:
            f.write("  none\n")
        for reason, c, d in stop_with_context[:20]:
            f.write("  t={:.2f}s reason={}".format(reason["t"], reason["reason"]))
            if c:
                f.write(" corridor[w={:.3f} st={} dyn={} off={:.2f} x={:.2f} y={:.2f}]".format(
                    c["w"], c["st"], c["dyn"], c["off"], c["x"], c["y"]))
                if math.isfinite(c.get("sw_s", float("nan"))):
                    f.write(" narrow[s={:.2f} x={:.2f} y={:.2f}]".format(
                        c["sw_s"], c["sw_x"], c["sw_y"]))
            else:
                f.write(" corridor[NA]")
            if d:
                f.write(" debug[valid={:.3f} corridor_reject={} convex_reject={} convex_v={:.3f} static_reject={} dynamic_reject={}]".format(
                    d["valid_ratio"], d["corridor_reject_count"],
                    d.get("convex_reject_count", float("nan")),
                    d.get("convex_max_violation", float("nan")),
                    d["static_reject_count"], d["dynamic_reject_count"]))
            else:
                f.write(" debug[NA]")
            f.write("\n")

        f.write("\nFirst ROS stop logs:\n")
        if not rosout_stop:
            f.write("  none\n")
        for row in rosout_stop[:20]:
            c = nearest_by_time(corridor, row["t"])
            d = nearest_by_time(debug, row["t"])
            f.write("  t={:.2f}s {}".format(row["t"], row["msg"]))
            if c:
                f.write(" corridor[w={:.3f} st={} dyn={} off={:.2f}]".format(
                    c["w"], c["st"], c["dyn"], c["off"]))
                if math.isfinite(c.get("sw_s", float("nan"))):
                    f.write(" narrow[s={:.2f} x={:.2f} y={:.2f}]".format(
                        c["sw_s"], c["sw_x"], c["sw_y"]))
            else:
                f.write(" corridor[NA]")
            if d:
                f.write(" debug[valid={:.3f} corridor_reject={} convex_reject={} convex_v={:.3f} static_reject={} dynamic_reject={}]".format(
                    d["valid_ratio"], d["corridor_reject_count"],
                    d.get("convex_reject_count", float("nan")),
                    d.get("convex_max_violation", float("nan")),
                    d["static_reject_count"], d["dynamic_reject_count"]))
            else:
                f.write(" debug[NA]")
            f.write("\n")

        f.write("\nHeuristic diagnosis:\n")
        if corridor and st_count / max(1, len(corridor)) > 0.5:
            f.write("  Dominant issue: static corridor infeasibility (st=1 for most labels).\n")
        elif corridor and dyn_count / max(1, len(corridor)) > 0.5:
            f.write("  Dominant issue: dynamic corridor occupancy (dyn>0 for most labels).\n")
        elif debug and max((d.get("corridor_reject_count", 0.0) for d in debug), default=0.0) > 0:
            f.write("  Dominant issue may be MPPI corridor hard rejection despite feasible labels.\n")
        elif debug and max((d.get("convex_reject_count", 0.0) for d in debug), default=0.0) > 0:
            f.write("  Dominant issue may be MPPI convex-corridor hard rejection.\n")
        else:
            f.write("  Dominant issue unclear from recorded topics; inspect CSV timeline.\n")

    return csv_path, summary_path


def main():
    parser = argparse.ArgumentParser(description="Analyze walking-corridor debug bags.")
    parser.add_argument("bag_or_dir")
    parser.add_argument("--out-dir", default="")
    args = parser.parse_args()

    bag_path = resolve_bag_path(args.bag_or_dir)
    out_dir = args.out_dir or (args.bag_or_dir if os.path.isdir(args.bag_or_dir)
                               else os.path.dirname(os.path.abspath(bag_path)))
    result = analyze_bag(bag_path)
    csv_path, summary_path = write_outputs(result, out_dir)
    print("Wrote CSV: {}".format(csv_path))
    print("Wrote summary: {}".format(summary_path))


if __name__ == "__main__":
    main()
