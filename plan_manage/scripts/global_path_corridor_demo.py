#!/usr/bin/env python3
import math
import threading

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from onboard_detector.msg import DynamicObstacles
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class GlobalPathCorridorDemo:
    def __init__(self):
        self.path_topic = rospy.get_param("~path_topic", "/astar/path")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/simulation_generator/global_cloud")
        self.dynamic_topic = rospy.get_param("~dynamic_topic", "/onboard_detector/dynamic_obstacles_info")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.use_cloud = rospy.get_param("~use_cloud", True)
        self.use_dynamic = rospy.get_param("~use_dynamic", True)

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
        self.robot_speed = max(0.05, rospy.get_param("~robot_speed", 1.0))
        self.dynamic_radius = max(0.0, rospy.get_param("~dynamic_radius", 0.35))
        self.dynamic_margin = max(0.0, rospy.get_param("~dynamic_margin", 0.25))
        self.dynamic_time_margin = rospy.get_param("~dynamic_time_margin", 0.0)
        self.min_polygon_area = max(0.01, rospy.get_param("~min_polygon_area", 0.08))

        self.lock = threading.Lock()
        self.path = []
        self.cloud_xy = []
        self.dynamic_obstacles = []
        self.last_debug = "waiting for /astar/path"

        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1, latch=True)
        self.debug_pub = rospy.Publisher("~debug", String, queue_size=1, latch=True)
        rospy.Subscriber(self.path_topic, Path, self.path_cb, queue_size=1)
        if self.use_cloud:
            rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)
        if self.use_dynamic:
            rospy.Subscriber(self.dynamic_topic, DynamicObstacles, self.dynamic_cb, queue_size=1)

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

    def dynamic_cb(self, msg):
        obstacles = []
        count = min(msg.num, len(msg.position))
        for i in range(count):
            pos = msg.position[i]
            vel = msg.velocity[i] if i < len(msg.velocity) else None
            size = msg.size[i] if i < len(msg.size) else None
            vx = vel.x if vel is not None else 0.0
            vy = vel.y if vel is not None else 0.0
            radius = self.dynamic_radius
            if size is not None:
                radius = max(radius, 0.5 * max(size.x, size.y))
            obstacles.append((pos.x, pos.y, vx, vy, radius + self.dynamic_margin))
        with self.lock:
            self.dynamic_obstacles = obstacles

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

    @staticmethod
    def polygon_area(poly):
        if len(poly) < 3:
            return 0.0
        area = 0.0
        for i, p in enumerate(poly):
            q = poly[(i + 1) % len(poly)]
            area += p[0] * q[1] - q[0] * p[1]
        return abs(0.5 * area)

    @staticmethod
    def clip_polygon_keep_greater(poly, normal, threshold):
        if len(poly) < 3:
            return []

        def value(p):
            return p[0] * normal[0] + p[1] * normal[1] - threshold

        clipped = []
        for i, cur in enumerate(poly):
            prev = poly[i - 1]
            cur_v = value(cur)
            prev_v = value(prev)
            cur_inside = cur_v >= -1e-9
            prev_inside = prev_v >= -1e-9
            if cur_inside != prev_inside:
                denom = prev_v - cur_v
                if abs(denom) > 1e-9:
                    u = prev_v / denom
                    clipped.append((prev[0] + u * (cur[0] - prev[0]),
                                    prev[1] + u * (cur[1] - prev[1])))
            if cur_inside:
                clipped.append(cur)
        return clipped

    @staticmethod
    def point_in_segment_box(a, b, half_width, point, radius):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < 1e-9:
            return False
        fx = dx / length
        fy = dy / length
        lx = -fy
        ly = fx
        rx = point[0] - a[0]
        ry = point[1] - a[1]
        s = rx * fx + ry * fy
        l = rx * lx + ry * ly
        return -radius <= s <= length + radius and abs(l) <= half_width + radius

    def dynamic_clip_positions(self, obstacle, cell_time):
        ox0, oy0, vx, vy, radius = obstacle
        positions = [(ox0, oy0, radius)]
        if cell_time > 1e-3 and (abs(vx) > 1e-6 or abs(vy) > 1e-6):
            positions.append((ox0 + vx * cell_time, oy0 + vy * cell_time, radius))
        return positions

    def apply_dynamic_clips(self, polygon, a, b, half_width, s_mid, dyn_obs):
        if not self.use_dynamic or not dyn_obs or len(polygon) < 3:
            return polygon, 0
        cell_time = max(0.0, s_mid / self.robot_speed + self.dynamic_time_margin)
        cx = 0.5 * (a[0] + b[0])
        cy = 0.5 * (a[1] + b[1])
        clipped_count = 0
        for obstacle in dyn_obs:
            for ox, oy, radius in self.dynamic_clip_positions(obstacle, cell_time):
                if not self.point_in_segment_box(a, b, half_width, (ox, oy), radius):
                    continue
                nx = cx - ox
                ny = cy - oy
                norm = math.hypot(nx, ny)
                if norm < 1e-6:
                    dx = b[0] - a[0]
                    dy = b[1] - a[1]
                    seg_norm = math.hypot(dx, dy)
                    if seg_norm < 1e-6:
                        continue
                    nx = -dy / seg_norm
                    ny = dx / seg_norm
                else:
                    nx /= norm
                    ny /= norm
                threshold = nx * ox + ny * oy + radius
                new_polygon = self.clip_polygon_keep_greater(polygon, (nx, ny), threshold)
                if new_polygon:
                    polygon = new_polygon
                else:
                    polygon = []
                clipped_count += 1
                if len(polygon) < 3:
                    break
            if len(polygon) < 3:
                break
        return polygon, clipped_count

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

    def build_corridor(self, path, cloud_xy, dyn_obs):
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
            polygon, clipped_count = self.apply_dynamic_clips(
                polygon, a, b, width, 0.5 * (s0 + s1), dyn_obs)
            feasible = (self.segment_clear(a, b, width, cloud_xy) and
                        self.polygon_area(polygon) >= self.min_polygon_area)
            cells.append((s0, s1, width, feasible, polygon, clipped_count))
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
            dyn_obs = list(self.dynamic_obstacles)

        markers = MarkerArray()
        markers.markers.append(self.make_clear_marker())
        cells = self.build_corridor(path, cloud_xy, dyn_obs)
        now = rospy.Time.now()

        for idx, (_s0, _s1, width, feasible, polygon, clipped_count) in enumerate(cells):
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
            if feasible and clipped_count > 0:
                marker.color.r = 0.75
                marker.color.g = 0.25
                marker.color.b = 1.0
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

        for idx, (ox0, oy0, vx, vy, radius) in enumerate(dyn_obs):
            dyn = Marker()
            dyn.header.frame_id = self.frame_id
            dyn.header.stamp = now
            dyn.ns = "global_path_corridor_demo_dynamic_obstacles"
            dyn.id = 200000 + idx
            dyn.type = Marker.CYLINDER
            dyn.action = Marker.ADD
            dyn.pose.position.x = ox0
            dyn.pose.position.y = oy0
            dyn.pose.position.z = 0.08
            dyn.pose.orientation.w = 1.0
            dyn.scale.x = 2.0 * radius
            dyn.scale.y = 2.0 * radius
            dyn.scale.z = 0.04
            dyn.color.a = 0.35
            dyn.color.r = 1.0
            dyn.color.g = 0.0
            dyn.color.b = 1.0
            markers.markers.append(dyn)

        feasible_count = sum(1 for c in cells if c[3])
        dynamic_clip_count = sum(c[5] for c in cells)
        mean_width = sum(c[2] for c in cells) / len(cells) if cells else 0.0
        self.last_debug = (
            "global_path_corridor_demo "
            "path_points={} cells={} feasible={} mean_half_width={:.2f} "
            "cloud_points={} dynamic_obstacles={} dynamic_clips={}"
        ).format(len(path), len(cells), feasible_count, mean_width,
                 len(cloud_xy), len(dyn_obs), dynamic_clip_count)
        self.marker_pub.publish(markers)
        self.debug_pub.publish(String(data=self.last_debug))


if __name__ == "__main__":
    rospy.init_node("global_path_corridor_demo")
    GlobalPathCorridorDemo()
    rospy.spin()
