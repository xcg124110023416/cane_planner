#!/usr/bin/env python3
"""
Simple pedestrian simulator for testing MPC dynamic obstacle avoidance.
Publishes onboard_detector::DynamicObstacles and visualization markers.
"""
import math
from pathlib import Path
import threading

import rospy
import tf
import numpy as np
import rospkg
import yaml
from geometry_msgs.msg import Vector3, Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from onboard_detector.msg import DynamicObstacles
from plan_manage.msg import PedestrianState
from plan_manage.srv import ControlPedestrian, ControlPedestrianResponse
from cane_pedestrian import Pedestrian
from cane_pedestrian.scene_io import (
    default_state_path, dumps_scene, from_config, loads_scene, write_scene,
)


class PedestrianSim:
    def __init__(self):
        rospy.init_node("pedestrian_sim")
        # These configured coordinates belong to this frame; no TF relabeling.
        self.frame_id = rospy.get_param("~frame_id", "map")
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("frame_id must be a nonempty coordinate frame")

        # Publishers
        self.obs_pub = rospy.Publisher(
            "/onboard_detector/dynamic_obstacles_info", DynamicObstacles, queue_size=10)
        self.viz_pub = rospy.Publisher(
            "/pedestrian_sim/visualization", MarkerArray, queue_size=10)

        default_pedestrians = [
            {"start": [-3.0, 3.0, 0.0], "vel": [0.8, 0.0, 0.0], "size": [0.5, 0.5, 1.7], "period": 5.0},
            {"start": [3.0, -2.0, 0.0], "vel": [-0.6, 0.3, 0.0], "size": [0.5, 0.5, 1.7], "period": 4.0},
        ]
        ped_configs = rospy.get_param("~pedestrians", None)
        if ped_configs is None:
            ped_configs = self.load_scenario(default_pedestrians)

        self.pedestrians = []
        initial_editor = None
        for i, cfg in enumerate(ped_configs):
            if cfg.get("mode") == "controlled":
                p, placed = from_config(cfg, self.frame_id)
                initial_editor = p
                if placed:
                    self.pedestrians.append(p)
            else:
                self.pedestrians.append(Pedestrian(
                    pid=i, start=cfg["start"], vel=cfg["vel"], size=cfg["size"],
                    period=cfg.get("period", 5.0), mode=cfg.get("mode", "trigger")))

        self.lock = threading.RLock()
        self.last_step = rospy.Time.now()
        self.editable = len(ped_configs) <= 1
        self.editor = (self.pedestrians[0] if len(self.pedestrians) == 1 else
                       initial_editor or Pedestrian(
                           0, [0, 0, 0], [0.4, 0, 0], [0.5, 0.5, 1.7], 5.0,
                           mode="controlled", end_behavior="return"))
        self.update_settings()
        self.remember_last = rospy.get_param("~remember_last", True)
        self.persistence_error = ""
        self.state_file = Path(rospy.get_param("~editor_state_file", str(default_state_path(
            rospkg.get_ros_home(), rospy.get_param("~scenario", "mixed"),
            rospy.get_param("~scenario_config", self.default_scenario_config()), self.frame_id)))).expanduser()
        if self.remember_last and self.editable:
            try:
                p, placed = loads_scene(self.state_file.read_text(encoding="utf-8"), self.frame_id)
                self.replace_editor(p, placed)
                rospy.loginfo("PedestrianSim: restored previous editor settings from %s", self.state_file)
            except FileNotFoundError:
                pass
            except (OSError, UnicodeError, ValueError) as exc:
                rospy.logwarn("PedestrianSim: cannot restore editor settings: %s", exc)
        self.tf_listener = tf.TransformListener()
        self.state_pub = rospy.Publisher(
            "/pedestrian_sim/state", PedestrianState, queue_size=1, latch=True)
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=1)
        self.pose_sub = rospy.Subscriber(
            "/pedestrian_sim/initial_pose", PoseStamped, self.pose_cb, queue_size=1)
        self.control_srv = rospy.Service(
            "/pedestrian_sim/control", ControlPedestrian, self.control_cb)
        self.rate = rospy.Rate(10)  # 10 Hz
        rospy.loginfo("PedestrianSim: %d pedestrians, 10Hz", len(self.pedestrians))

    def load_scenario(self, default_pedestrians):
        scenario = rospy.get_param("~scenario", "mixed")
        config_path = rospy.get_param("~scenario_config", self.default_scenario_config())
        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f) or {}
        except Exception as exc:
            rospy.logwarn(
                "PedestrianSim: failed to read scenario config %s: %s; using defaults",
                config_path,
                exc,
            )
            return default_pedestrians
        if scenario not in config:
            rospy.logwarn(
                "PedestrianSim: scenario '%s' not found in %s; available=%s; using defaults",
                scenario,
                config_path,
                sorted(config.keys()),
            )
            return default_pedestrians
        rospy.loginfo("PedestrianSim: scenario=%s config=%s", scenario, config_path)
        return config[scenario]

    def default_scenario_config(self):
        package_path = rospkg.RosPack().get_path("plan_manage")
        return package_path + "/config/pedestrian_scenarios.yaml"

    def update_settings(self):
        self.settings = dict(
            speed=self.editor.speed, duration=self.editor.period,
            end_behavior=self.editor.end_behavior, start_on_goal=self.editor.start_on_goal)

    def replace_editor(self, pedestrian, placed=True):
        self.editor = pedestrian
        self.pedestrians = [pedestrian] if placed else []
        self.update_settings()

    def remember_editor(self):
        if not self.remember_last:
            return
        try:
            write_scene(self.state_file, dumps_scene(self.editor, self.frame_id, bool(self.pedestrians)))
            self.persistence_error = ""
        except OSError as exc:
            self.persistence_error = str(exc)
            rospy.logwarn("PedestrianSim: cannot remember editor settings: %s", exc)

    def advance(self, now):
        dt = max(0.0, (now - self.last_step).to_sec())
        self.last_step = now
        for p in self.pedestrians:
            p.step(dt)

    def goal_cb(self, msg):
        """Legacy scenes retrigger; the editor can choose manual-only start."""
        with self.lock:
            self.advance(rospy.Time.now())
            for p in self.pedestrians:
                if p.mode != "controlled" or p.start_on_goal:
                    p.trigger()

    def make_controlled(self, start, yaw, settings):
        speed = settings["speed"]
        if not math.isfinite(speed) or not 0 <= speed <= Pedestrian.MAX_SPEED:
            raise ValueError("Speed must be between 0 and 10 m/s")
        return Pedestrian(
            0, start, [speed * math.cos(yaw), speed * math.sin(yaw), 0.0],
            self.editor.size, settings["duration"], mode="controlled", yaw=yaw,
            end_behavior=settings["end_behavior"], start_on_goal=settings["start_on_goal"])

    def pose_cb(self, msg):
        try:
            if not msg.header.frame_id:
                raise ValueError("Placement requires a coordinate frame")
            if msg.header.frame_id != self.frame_id:
                msg = self.tf_listener.transformPose(self.frame_id, msg)
            q = msg.pose.orientation
            quat = np.array([q.x, q.y, q.z, q.w])
            if not np.all(np.isfinite(quat)) or np.linalg.norm(quat) < 1e-8:
                raise ValueError("Placement requires a valid orientation")
            yaw = tf.transformations.euler_from_quaternion(quat / np.linalg.norm(quat))[2]
            pos = msg.pose.position
            with self.lock:
                if not self.editable:
                    raise ValueError("The editor requires a zero- or one-pedestrian scenario")
                p = self.make_controlled([pos.x, pos.y, pos.z], yaw, self.settings)
                self.advance(rospy.Time.now())
                self.replace_editor(p)
                self.remember_editor()
                self.publish_state()
        except (ValueError, tf.Exception) as exc:
            rospy.logwarn("Pedestrian placement rejected: %s", exc)

    def control_cb(self, req):
        with self.lock:
            if not self.editable:
                return ControlPedestrianResponse(False, "Only single-pedestrian scenes can be edited", "")
            # Parse before touching live state; failed loads retain the current motion.
            if req.command == "export":
                return ControlPedestrianResponse(
                    True, "OK", dumps_scene(self.editor, self.frame_id, bool(self.pedestrians)))
            if req.command == "load":
                try:
                    loaded, placed = loads_scene(req.scenario_yaml, self.frame_id)
                except ValueError as exc:
                    return ControlPedestrianResponse(False, str(exc), "")
                self.advance(rospy.Time.now())
                self.replace_editor(loaded, placed)
            elif req.command == "clear":
                self.advance(rospy.Time.now())
                self.pedestrians = []
            elif not self.pedestrians:
                return ControlPedestrianResponse(False, "Place a pedestrian first", "")
            elif req.command == "apply":
                settings = dict(speed=req.speed, duration=req.duration,
                                end_behavior=req.end_behavior, start_on_goal=req.start_on_goal)
                try:
                    p = self.make_controlled(self.editor.start, self.editor.yaw, settings)
                except ValueError as exc:
                    return ControlPedestrianResponse(False, str(exc), "")
                self.advance(rospy.Time.now())
                self.replace_editor(p)
            elif req.command in ("start", "pause", "reset"):
                if req.command == "pause" and self.editor.mode != "controlled":
                    return ControlPedestrianResponse(False, "Place or apply the pedestrian before pausing", "")
                self.advance(rospy.Time.now())
                if self.editor.mode != "controlled":
                    self.replace_editor(self.make_controlled(
                        self.editor.start, self.editor.yaw, self.settings))
                if req.command == "start":
                    self.editor.resume_or_start()
                elif req.command == "pause":
                    self.editor.pause()
                else:
                    self.editor.reset()
            else:
                return ControlPedestrianResponse(False, "Unknown control command", "")
            # Store configuration only, never elapsed time or a transient moving pose.
            self.remember_editor()
            self.publish_state()
            return ControlPedestrianResponse(True, "OK", "")

    def publish_state(self):
        msg = PedestrianState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.editable = self.editable
        msg.placed = bool(self.pedestrians)
        p = self.editor
        msg.start_pose.position = Point(*p.start)
        q = tf.transformations.quaternion_from_euler(0, 0, p.yaw)
        msg.start_pose.orientation.x, msg.start_pose.orientation.y = q[0], q[1]
        msg.start_pose.orientation.z, msg.start_pose.orientation.w = q[2], q[3]
        msg.size = Vector3(*p.size)
        msg.speed = self.settings["speed"]
        msg.duration = self.settings["duration"]
        msg.end_behavior = self.settings["end_behavior"]
        msg.start_on_goal = self.settings["start_on_goal"]
        msg.phase = ((p.phase if p.mode == "controlled" else "legacy_" + p.mode)
                     if msg.placed else "cleared")
        msg.visible = msg.placed and p.visible
        msg.remember_last = self.remember_last
        msg.persistence_error = self.persistence_error
        self.state_pub.publish(msg)

    def publish_obstacles(self):
        msg = DynamicObstacles()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        # Paused/waiting controlled pedestrians remain fresh stationary obstacles.
        active = [p for p in self.pedestrians if p.visible]
        msg.num = len(active)
        for p in active:
            msg.position.append(Vector3(p.pos[0], p.pos[1], p.pos[2]))
            msg.velocity.append(Vector3(p.vel[0], p.vel[1], p.vel[2]))
            msg.size.append(Vector3(p.size[0], p.size[1], p.size[2]))
        self.obs_pub.publish(msg)

    def publish_viz(self):
        arr = MarkerArray()
        del_mk = Marker()
        del_mk.header.frame_id = self.frame_id
        del_mk.header.stamp = rospy.Time.now()
        del_mk.action = Marker.DELETEALL
        arr.markers.append(del_mk)

        for p in self.pedestrians:
            if p.mode == "controlled":
                self.add_preview(arr, p)
            if not p.visible:
                continue

            # Bounding box
            mk = Marker()
            mk.header.frame_id = self.frame_id
            mk.header.stamp = rospy.Time.now()
            mk.ns = "ped_{}".format(p.pid)
            mk.id = 0
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.pose.position.x = p.pos[0]
            mk.pose.position.y = p.pos[1]
            mk.pose.position.z = p.pos[2] + p.size[2] / 2.0
            mk.pose.orientation.w = 1.0
            mk.scale.x = p.size[0]
            mk.scale.y = p.size[1]
            mk.scale.z = p.size[2]
            if p.mode == "trigger":
                mk.color.r, mk.color.g, mk.color.b = 1.0, 0.2, 0.2  # red
            elif p.mode == "loop":
                mk.color.r, mk.color.g, mk.color.b = 0.2, 0.4, 1.0
            elif p.pid == 0:
                mk.color.r, mk.color.g, mk.color.b = 1.0, 0.3, 0.0
            else:
                mk.color.r, mk.color.g, mk.color.b = 1.0, 0.6, 0.0
            mk.color.a = 0.7
            arr.markers.append(mk)

            # Velocity arrow
            mk2 = Marker()
            mk2.header.frame_id = self.frame_id
            mk2.header.stamp = rospy.Time.now()
            mk2.ns = "ped_{}_vel".format(p.pid)
            mk2.id = 0
            mk2.type = Marker.ARROW
            mk2.action = Marker.ADD
            mk2.pose.position.x = p.pos[0]
            mk2.pose.position.y = p.pos[1]
            mk2.pose.position.z = p.pos[2] + p.size[2] / 2.0
            vel_norm = np.linalg.norm(p.vel[:2])
            yaw = np.arctan2(p.vel[1], p.vel[0]) if vel_norm > 0.01 else p.yaw
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            mk2.pose.orientation.x = q[0]
            mk2.pose.orientation.y = q[1]
            mk2.pose.orientation.z = q[2]
            mk2.pose.orientation.w = q[3]
            mk2.scale.x = vel_norm * 0.5 + 0.3
            mk2.scale.y = 0.08
            mk2.scale.z = 0.08
            mk2.color.a = 0.9
            mk2.color.r = 1.0
            mk2.color.g = 0.2
            mk2.color.b = 0.2
            arr.markers.append(mk2)

        self.viz_pub.publish(arr)

    def add_preview(self, arr, p):
        """The preview never enters the dynamic-obstacle observation stream."""
        end = p.start + p.vel_mag * p.period
        path = Marker()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time.now()
        path.ns = "pedestrian_preview"
        path.id = 0
        path.type = Marker.ARROW
        path.pose.orientation.w = 1.0
        path.points = [Point(p.start[0], p.start[1], p.start[2] + 0.1),
                       Point(end[0], end[1], end[2] + 0.1)]
        path.scale.x, path.scale.y, path.scale.z = 0.04, 0.12, 0.18
        path.color.r, path.color.g, path.color.b, path.color.a = 0.1, 0.9, 0.8, 0.7
        arr.markers.append(path)
        for i, pos in enumerate((p.start, end)):
            dot = Marker()
            dot.header = path.header
            dot.ns = path.ns
            dot.id = i + 1
            dot.type = Marker.SPHERE
            dot.pose.position = Point(pos[0], pos[1], pos[2] + 0.1)
            dot.pose.orientation.w = 1.0
            dot.scale.x = dot.scale.y = dot.scale.z = 0.15
            dot.color = path.color
            arr.markers.append(dot)

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                self.advance(rospy.Time.now())
                self.publish_obstacles()
                self.publish_viz()
                self.publish_state()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        PedestrianSim().run()
    except rospy.ROSInterruptException:
        pass
