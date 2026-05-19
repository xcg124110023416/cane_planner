#!/usr/bin/env python3
"""
Simple pedestrian simulator for testing MPC dynamic obstacle avoidance.
Publishes onboard_detector::DynamicObstacles and visualization markers.
"""
import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray
from onboard_detector.msg import DynamicObstacles


class Pedestrian:
    def __init__(self, pid, start, vel, size, period, mode="patrol"):
        self.pid = pid
        self.start = np.array(start, dtype=float)
        self.vel_mag = np.array(vel, dtype=float)
        self.pos = self.start.copy()
        self.vel = self.vel_mag.copy()
        self.size = np.array(size, dtype=float)
        self.period = float(period)
        self.mode = mode               # "patrol" (back-forth) or "loop" (one-way reset)
        self.elapsed = 0.0
        self.trail = []
        self.max_trail = 30

    def step(self, dt):
        self.elapsed += dt

        if self.mode == "loop":
            # Walk straight, reset to start after period
            self.vel = self.vel_mag.copy()
            self.pos = self.start + self.vel_mag * self.elapsed
            if self.elapsed >= self.period:
                self.elapsed = 0.0
                self.pos = self.start.copy()
        else:
            # patrol: back and forth, start = center
            if self.elapsed >= self.period:
                self.elapsed -= self.period
            t = self.elapsed
            if t < self.period / 4.0:
                self.vel = self.vel_mag
            elif t < self.period * 3.0 / 4.0:
                self.vel = -self.vel_mag
            else:
                self.vel = self.vel_mag
            self.pos = self.pos + self.vel * dt

        self.trail.append(self.pos.copy())
        if len(self.trail) > self.max_trail:
            self.trail.pop(0)


class PedestrianSim:
    def __init__(self):
        rospy.init_node("pedestrian_sim")

        # Publishers
        self.obs_pub = rospy.Publisher(
            "/onboard_detector/dynamic_obstacles_info", DynamicObstacles, queue_size=10)
        self.viz_pub = rospy.Publisher(
            "/pedestrian_sim/visualization", MarkerArray, queue_size=10)

        # Define pedestrians from rosparam
        self.pedestrians = []
        ped_configs = rospy.get_param("~pedestrians", [
            {"start": [-3.0, 3.0, 0.0], "vel": [0.8, 0.0, 0.0], "size": [0.5, 0.5, 1.7], "period": 5.0},
            {"start": [3.0, -2.0, 0.0], "vel": [-0.6, 0.3, 0.0], "size": [0.5, 0.5, 1.7], "period": 4.0},
        ])

        for i, cfg in enumerate(ped_configs):
            period = cfg.get("period", 5.0)
            mode = cfg.get("mode", "patrol")
            self.pedestrians.append(Pedestrian(
                pid=i, start=cfg["start"], vel=cfg["vel"],
                size=cfg["size"], period=period, mode=mode))

        self.rate = rospy.Rate(10)  # 10 Hz
        rospy.loginfo("PedestrianSim: %d pedestrians, 10Hz", len(self.pedestrians))

    def publish_obstacles(self):
        msg = DynamicObstacles()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.num = len(self.pedestrians)
        for p in self.pedestrians:
            msg.position.append(Vector3(p.pos[0], p.pos[1], p.pos[2]))
            msg.velocity.append(Vector3(p.vel[0], p.vel[1], p.vel[2]))
            msg.size.append(Vector3(p.size[0], p.size[1], p.size[2]))
        self.obs_pub.publish(msg)

    def publish_viz(self):
        arr = MarkerArray()
        # Delete old markers
        del_mk = Marker()
        del_mk.header.frame_id = "world"
        del_mk.header.stamp = rospy.Time.now()
        del_mk.action = Marker.DELETEALL
        arr.markers.append(del_mk)

        for p in self.pedestrians:
            # Bounding box
            mk = Marker()
            mk.header.frame_id = "world"
            mk.header.stamp = rospy.Time.now()
            mk.ns = "ped_{}".format(p.pid)
            mk.id = 0
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.pose.position.x = p.pos[0]
            mk.pose.position.y = p.pos[1]
            mk.pose.position.z = p.size[2] / 2.0
            mk.pose.orientation.w = 1.0
            mk.scale.x = p.size[0]
            mk.scale.y = p.size[1]
            mk.scale.z = p.size[2]
            # Color: patrol=orange, loop=blue
            if p.mode == "loop":
                mk.color.r, mk.color.g, mk.color.b = 0.2, 0.4, 1.0
            elif p.pid == 0:
                mk.color.r, mk.color.g, mk.color.b = 1.0, 0.3, 0.0
            else:
                mk.color.r, mk.color.g, mk.color.b = 1.0, 0.6, 0.0
            mk.color.a = 0.7
            arr.markers.append(mk)

            # Velocity arrow
            mk2 = Marker()
            mk2.header.frame_id = "world"
            mk2.header.stamp = rospy.Time.now()
            mk2.ns = "ped_{}_vel".format(p.pid)
            mk2.id = 0
            mk2.type = Marker.ARROW
            mk2.action = Marker.ADD
            mk2.pose.position.x = p.pos[0]
            mk2.pose.position.y = p.pos[1]
            mk2.pose.position.z = p.size[2] / 2.0
            vel_norm = np.linalg.norm(p.vel[:2])
            if vel_norm > 0.01:
                yaw = np.arctan2(p.vel[1], p.vel[0])
                from tf.transformations import quaternion_from_euler
                q = quaternion_from_euler(0, 0, yaw)
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

    def run(self):
        dt = 0.1
        while not rospy.is_shutdown():
            for p in self.pedestrians:
                p.step(dt)
            self.publish_obstacles()
            self.publish_viz()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        PedestrianSim().run()
    except rospy.ROSInterruptException:
        pass
