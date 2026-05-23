#!/usr/bin/env python3
"""Accumulate /cloud_registered into a persistent PointCloud2 map."""
import math
from collections import OrderedDict

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class RegisteredMapAccumulator:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/cloud_registered")
        self.output_topic = rospy.get_param("~output_topic", "/Laser_map")
        self.frame_id = rospy.get_param("~frame_id", "camera_init")
        self.voxel_size = float(rospy.get_param("~voxel_size", 0.08))
        self.publish_every_n = max(1, int(rospy.get_param("~publish_every_n", 3)))
        self.max_points = int(rospy.get_param("~max_points", 180000))

        self.points = OrderedDict()
        self.scan_count = 0
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1, latch=True)
        rospy.Subscriber(self.input_topic, PointCloud2, self.cloud_callback, queue_size=3)

    def voxel_key(self, x, y, z):
        inv = 1.0 / self.voxel_size
        return (int(math.floor(x * inv)), int(math.floor(y * inv)), int(math.floor(z * inv)))

    def cloud_callback(self, msg):
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            key = self.voxel_key(x, y, z)
            if key not in self.points:
                self.points[key] = (x, y, z)
                if len(self.points) > self.max_points:
                    self.points.popitem(last=False)

        self.scan_count += 1
        if self.scan_count % self.publish_every_n != 0:
            return

        header = msg.header
        header.frame_id = self.frame_id
        cloud = pc2.create_cloud_xyz32(header, list(self.points.values()))
        self.pub.publish(cloud)


if __name__ == "__main__":
    rospy.init_node("registered_map_accumulator")
    RegisteredMapAccumulator()
    rospy.spin()
