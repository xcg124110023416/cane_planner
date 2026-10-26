"""Shared YAML format for panel import/export and remembered editor settings."""
import hashlib
import math
import os
from pathlib import Path
import tempfile

import yaml

from . import Pedestrian


def from_config(config, frame_id):
    """Validate the entire candidate before the simulator replaces live state."""
    try:
        if config["mode"] != "controlled":
            raise ValueError("Load a scene saved by the pedestrian panel (mode: controlled)")
        if config.get("frame_id", frame_id) != frame_id:
            raise ValueError("Scene coordinate frame does not match the simulator: " + frame_id)
        speed, yaw = float(config["speed"]), float(config["yaw"])
        if not math.isfinite(speed) or not 0 <= speed <= Pedestrian.MAX_SPEED:
            raise ValueError("Speed must be between 0 and 10 m/s")
        on_goal = config["start_on_goal"]
        placed = config.get("placed", True)
        if not isinstance(on_goal, bool) or not isinstance(placed, bool):
            raise ValueError("start_on_goal and placed must be YAML booleans")
        pedestrian = Pedestrian(
            0, config["start"], [speed * math.cos(yaw), speed * math.sin(yaw), 0.0],
            config["size"], config["duration"], mode="controlled", yaw=yaw,
            end_behavior=config["end_behavior"], start_on_goal=on_goal)
        return pedestrian, placed
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError("Incomplete or invalid pedestrian scene: " + str(exc)) from exc


def loads_scene(text, frame_id):
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ValueError("Invalid YAML: " + str(exc)) from exc
    if not isinstance(data, dict) or len(data) != 1:
        raise ValueError("The file must contain exactly one named pedestrian scene")
    people = next(iter(data.values()))
    if not isinstance(people, list) or len(people) != 1 or not isinstance(people[0], dict):
        raise ValueError("The scene must contain exactly one pedestrian")
    return from_config(people[0], frame_id)


def dumps_scene(pedestrian, frame_id, placed=True):
    return yaml.safe_dump({"custom_pedestrian": [{
        "mode": "controlled",
        "frame_id": frame_id,
        "start": pedestrian.start.tolist(),
        "yaw": pedestrian.yaw,
        "speed": pedestrian.speed,
        "duration": pedestrian.period,
        "size": pedestrian.size.tolist(),
        "end_behavior": pedestrian.end_behavior,
        "start_on_goal": pedestrian.start_on_goal,
        "placed": placed,
    }]}, sort_keys=False, allow_unicode=True)


def default_state_path(ros_home, scenario, config_path, frame_id):
    context = repr((scenario, str(Path(config_path).expanduser().resolve()), frame_id))
    key = hashlib.sha256(context.encode("utf-8")).hexdigest()[:20]
    return Path(ros_home) / "cane_planner" / "pedestrian_editor" / (key + ".yaml")


def write_scene(path, text):
    """Replace the remembered file atomically; a failed write keeps the last copy."""
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=".pedestrian-", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(text)
        os.replace(str(temporary), str(path))
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
