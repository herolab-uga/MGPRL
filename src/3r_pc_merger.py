#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import struct

def to_msg(open3d_cloud, frame_id="map"):
    header = std_msgs.msg.Header(frame_id=frame_id, stamp=rospy.Time.now())
    points = np.asarray(open3d_cloud.points)
    if open3d_cloud.has_colors():
        colors = np.asarray(open3d_cloud.colors)
        # Assuming colors are [0, 1] float, convert to 0-255 uint8
        colors = (colors * 255).astype(np.uint8)
        # Pack RGB into a single float
        rgb = np.array([struct.unpack('f', struct.pack('i', r << 16 | g << 8 | b))[0] for r, g, b in colors])
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgb', 12, PointField.FLOAT32, 1)]
        cloud_data = np.c_[points, rgb].astype(np.float32)
    else:
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]
        cloud_data = points.astype(np.float32)

    cloud_msg = pc2.create_cloud(header, fields, cloud_data)
    return cloud_msg

def from_msg(ros_cloud):
    assert isinstance(ros_cloud, PointCloud2)
    open3d_cloud = o3d.geometry.PointCloud()
    gen = pc2.read_points(ros_cloud, field_names=("x", "y", "z", "rgb"), skip_nans=True)
    points = []
    colors = []
    for p in gen:
        points.append(p[:3])
        if len(p) > 3:  # Has color information
            # Convert float32 to int so that bitwise operations are possible
            rgb = struct.unpack('I', struct.pack('f', p[3]))[0]
            # Extract individual channel colors and convert to [0, 1] range
            colors.append(((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF))
    open3d_cloud.points = o3d.utility.Vector3dVector(np.array(points))
    if colors:
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(colors) / 255.0)
    return open3d_cloud


class PointCloudMerger:
    def __init__(self):
        rospy.init_node('point_cloud_merger')
        # Subscribers for each robot's point cloud map
        self.pc1_sub = rospy.Subscriber('/rtabmap_0/cloud_map', PointCloud2, self.pc1_callback)
        self.pc2_sub = rospy.Subscriber('/rtabmap_1/cloud_map', PointCloud2, self.pc2_callback)
        self.pc3_sub = rospy.Subscriber('/rtabmap_2/cloud_map', PointCloud2, self.pc3_callback)  # New subscriber for the third robot
        self.merged_pc_pub = rospy.Publisher('/total_cloud_map', PointCloud2, queue_size=10)

        self.pc1 = None
        self.pc2 = None
        self.pc3 = None  # New attribute for the third point cloud

    def pc1_callback(self, msg):
        try:
            self.pc1 = from_msg(msg)
            self.try_merge_and_publish()
        except Exception as e:
            rospy.logerr("An error occurred in pc1_callback: {}".format(e))

    def pc2_callback(self, msg):
        try:
            self.pc2 = from_msg(msg)
            self.try_merge_and_publish()
        except Exception as e:
            rospy.logerr("An error occurred in pc2_callback: {}".format(e))

    def pc3_callback(self, msg):
        # Similar to the pc1_callback and pc2_callback methods
        try:
            self.pc3 = from_msg(msg)
            self.try_merge_and_publish()
        except Exception as e:
            rospy.logerr("An error occurred in pc3_callback: {}".format(e))

    def try_merge_and_publish(self):
        # Wait for all three point clouds before attempting to merge
        if self.pc1 is not None and self.pc2 is not None and self.pc3 is not None:
            try:
                # Merge the point clouds. This method will need adjustment.
                merged_cloud = self.align_and_merge(self.pc1, self.pc2, self.pc3)
                merged_msg = to_msg(merged_cloud, frame_id="map")
                self.merged_pc_pub.publish(merged_msg)
                # Reset the point clouds to None after publishing
                self.pc1 = None
                self.pc2 = None
                self.pc3 = None
            except Exception as e:
                rospy.logerr("An error occurred during merging and publishing: {}".format(e))

    def align_and_merge(self, pc1, pc2, pc3):
        # Extended method to merge three point clouds
        try:
            # Initial merge between pc1 and pc2 with ICP
            threshold = 0.02  # Distance threshold for ICP
            trans_init = np.eye(4)  # Identity matrix, no initial alignment
            reg_p2p = o3d.pipelines.registration.registration_icp(
                pc1, pc2, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
            pc1.transform(reg_p2p.transformation)
            partial_merge = pc1 + pc2

            # Merge the result with pc3
            reg_p2p = o3d.pipelines.registration.registration_icp(
                partial_merge, pc3, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
            partial_merge.transform(reg_p2p.transformation)
            final_merge = partial_merge + pc3
            return final_merge
        except Exception as e:
            rospy.logerr("An error occurred in align_and_merge: {}".format(e))
            return None

if __name__ == '__main__':
    try:
        pc_merger = PointCloudMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass