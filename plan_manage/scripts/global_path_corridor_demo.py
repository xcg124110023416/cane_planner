#!/usr/bin/env python3
import math
import threading

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class GlobalPathCorridorDemo:
    def __init__(self):
        self.path_topic = rospy.get_param("~path_topic", "/astar/path")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/simulation_generator/global_cloud")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.use_cloud = rospy.get_param("~use_cloud", True)

        self.segment_length = max(0.2, rospy.get_param("~segment_length", 1.8))
        self.overlap = max(0.0, rospy.get_param("~overlap", 0.7))
        self.half_width = max(0.05, rospy.get_param("~half_width", 0.9))
        self.min_half_width = max(0.01, rospy.get_param("~min_half_width", 0.25))
        self.width_step = max(0.01, rospy.get_param("~width_step", 0.05))
        self.obstacle_margin = max(0.0, rospy.get_param("~obstacle_margin", 0.08))
        self.min_z = rospy.get_param("~cloud_min_z", 0.10)
        self.max_z = rospy.get_param("~cloud_max_z", 2.60)
        self.max_cloud_points = int(rospy.get_param("~max_cloud_points", 60000))
        self.publish_rate = max(0.2, rospy.get_param("~publish_rate", 3.0))

        self.lock = threading.Lock()
        self.path = []
        self.cloud_xy = []
        self.last_debug = "waiting for /astar/path"

        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1, latch=True)
        self.debug_pub = rospy.Publisher("~debug", String, queue_size=1, latch=True)
        rospy.Subscriber(self.path_topic, Path, self.path_cb, queue_size=1)
        if self.use_cloud:
            rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish)

    def path_cb(self, msg):
        path = []
        for pose in msg.poses:
            p = pose.pose.position
            if not path or math.hypot(p.x - path[-1][0], p.y - path[-1][1]) > 1e-3:
                path.append((p.x, p.y))
        with self.lock:
            self.path = path

    def cloud_cb(self, msg):
        pts = []
        count = 0
        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if self.min_z <= z <= self.max_z:
                pts.append((x, y))
                count += 1
                if count >= self.max_cloud_points:
                    break
        with self.lock:
            self.cloud_xy = pts

    @staticmethod
    def arc_lengths(path):
        arcs = [0.0]
        for i in range(1, len(path)):
            arcs.append(arcs[-1] + math.hypot(path[i][0] - path[i - 1][0],
                                             path[i][1] - path[i - 1][1]))
        return arcs

    @staticmethod
    def interp(path, arcs, s):
        if s <= 0.0:
            return path[0]
        if s >= arcs[-1]:
            return path[-1]
        lo = 0
        hi = len(arcs) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if arcs[mid] <= s:
                lo = mid
            else:
                hi = mid
        ds = arcs[lo + 1] - arcs[lo]
        if ds < 1e-9:
            return path[lo]
        u = (s - arcs[lo]) / ds
        return (path[lo][0] + u * (path[lo + 1][0] - path[lo][0]),
                path[lo][1] + u * (path[lo + 1][1] - path[lo][1]))

    @staticmethod
    def rect_polygon(a, b, half_width):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            return []
        lx = -dy / norm
        ly = dx / norm
        return [
            (a[0] + lx * half_width, a[1] + ly * half_width),
            (b[0] + lx * half_width, b[1] + ly * half_width),
            (b[0] - lx * half_width, b[1] - ly * half_width),
            (a[0] - lx * half_width, a[1] - ly * half_width),
        ]

    def segment_clear(self, a, b, half_width, cloud_xy):
        if not self.use_cloud or not cloud_xy:
            return True
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < 1e-9:
            return True
        fx = dx / length
        fy = dy / length
        lx = -fy
        ly = fx
        margin = self.obstacle_margin
        min_x = min(a[0], b[0]) - half_width - margin
        max_x = max(a[0], b[0]) + half_width + margin
        min_y = min(a[1], b[1]) - half_width - margin
        max_y = max(a[1], b[1]) + half_width + margin
        for px, py in cloud_xy:
            if px < min_x or px > max_x or py < min_y or py > max_y:
                continue
            rx = px - a[0]
            ry = py - a[1]
            s = rx * fx + ry * fy
            l = rx * lx + ry * ly
            if -margin <= s <= length + margin and abs(l) <= half_width + margin:
                return False
        return True

    def build_corridor(self, path, cloud_xy):
        if len(path) < 2:
            return []
        arcs = self.arc_lengths(path)
        total = arcs[-1]
        if total < 1e-6:
            return []
        stride = max(0.1, self.segment_length - self.overlap)
        cells = []
        s0 = 0.0
        while s0 < total - 1e-6:
            s1 = min(total, s0 + self.segment_length)
            a = self.interp(path, arcs, s0)
            b = self.interp(path, arcs, s1)
            width = self.half_width
            while width > self.min_half_width and not self.segment_clear(a, b, width, cloud_xy):
                width = max(self.min_half_width, width - self.width_step)
            polygon = self.rect_polygon(a, b, width)
            feasible = self.segment_clear(a, b, width, cloud_xy)
            cells.append((s0, s1, width, feasible, polygon))
            s0 += stride
        return cells

    def make_clear_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "global_path_corridor_demo"
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker

    def publish(self, _event):
        with self.lock:
            path = list(self.path)
            cloud_xy = list(self.cloud_xy)

        markers = MarkerArray()
        markers.markers.append(self.make_clear_marker())
        cells = self.build_corridor(path, cloud_xy)
        now = rospy.Time.now()

        for idx, (_s0, _s1, width, feasible, polygon) in enumerate(cells):
            if len(polygon) < 3:
                continue
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now
            marker.ns = "global_path_corridor_demo_cells"
            marker.id = idx
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.045
            marker.color.a = 0.95
            marker.color.r = 0.0 if feasible else 1.0
            marker.color.g = 0.85 if feasible else 0.15
            marker.color.b = 1.0 if feasible else 0.15
            for x, y in polygon + [polygon[0]]:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.12
                marker.points.append(p)
            markers.markers.append(marker)

        center = Marker()
        center.header.frame_id = self.frame_id
        center.header.stamp = now
        center.ns = "global_path_corridor_demo_path"
        center.id = 100000
        center.type = Marker.LINE_STRIP
        center.action = Marker.ADD
        center.pose.orientation.w = 1.0
        center.scale.x = 0.05
        center.color.a = 0.9
        center.color.r = 1.0
        center.color.g = 0.95
        center.color.b = 0.0
        for x, y in path:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.18
            center.points.append(p)
        if len(center.points) >= 2:
            markers.markers.append(center)

        feasible_count = sum(1 for c in cells if c[3])
        mean_width = sum(c[2] for c in cells) / len(cells) if cells else 0.0
        self.last_debug = (
            "global_path_corridor_demo "
            "path_points={} cells={} feasible={} mean_half_width={:.2f} "
            "cloud_points={}"
        ).format(len(path), len(cells), feasible_count, mean_width, len(cloud_xy))
        self.marker_pub.publish(markers)
        self.debug_pub.publish(String(data=self.last_debug))


if __name__ == "__main__":
    rospy.init_node("global_path_corridor_demo")
    GlobalPathCorridorDemo()
    rospy.spin()
