#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion

class ProximitySlamController:
    def __init__(self, robot_namespaces, rtabmap_namespaces, proximity_threshold):
        self.robot_positions = {}
        self.robot_orientations = {}  # Store orientation data separately
        self.proximity_threshold = proximity_threshold
        self.odom_subscribers = []
        self.pause_services = {}
        self.resume_services = {}
        self.slam_paused = {}  # Track whether SLAM is paused for each robot

        for robot_namespace, rtabmap_namespace in zip(robot_namespaces, rtabmap_namespaces):
            odom_topic = f"/{robot_namespace}/odom"
            self.odom_subscribers.append(rospy.Subscriber(odom_topic, Odometry, self.odom_callback, robot_namespace))

            pause_service_name = f"/{rtabmap_namespace}/pause"
            resume_service_name = f"/{rtabmap_namespace}/resume"
            self.pause_services[robot_namespace] = rospy.ServiceProxy(pause_service_name, Empty)
            self.resume_services[robot_namespace] = rospy.ServiceProxy(resume_service_name, Empty)
            self.slam_paused[robot_namespace] = False

    def odom_callback(self, msg, robot_namespace):
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.robot_positions[robot_namespace] = np.array([position.x, position.y])
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.robot_orientations[robot_namespace] = yaw
        self.check_proximity_and_control_slam()

    def check_proximity_and_control_slam(self):
        robot_names = list(self.robot_positions.keys())
        if len(robot_names) < 2:
            return

        # Iterate over pairs of robots
        for i, robot_name in enumerate(robot_names):
            for j, other_robot_name in enumerate(robot_names):
                if i == j:
                    continue  # Skip comparing the robot to itself
                
                if self.is_within_fov(robot_name, other_robot_name) and self.calculate_distance(robot_name, other_robot_name) < self.proximity_threshold:
                    self.pause_slam(robot_name)
                else:
                    self.resume_slam(robot_name)

    def calculate_distance(self, robot_name, other_robot_name):
        pos1 = self.robot_positions[robot_name]
        pos2 = self.robot_positions[other_robot_name]
        return np.linalg.norm(pos1 - pos2)


    def is_within_fov(self, observer_name, target_name):
        observer_pos = self.robot_positions[observer_name]
        target_pos = self.robot_positions[target_name]
        observer_yaw = self.robot_orientations[observer_name]
        robot_radius = max(0.281, 0.306, 0.141) / 2
        direction_to_target = target_pos - observer_pos
        distance_to_target = np.linalg.norm(direction_to_target)

        if distance_to_target - robot_radius > self.proximity_threshold:
            return False

        angle_to_target = np.arctan2(direction_to_target[1], direction_to_target[0])
        angle_to_target_normalized = (angle_to_target + np.pi) % (2 * np.pi) - np.pi
        observer_yaw_normalized = (observer_yaw + np.pi) % (2 * np.pi) - np.pi
        angle_difference = (angle_to_target_normalized - observer_yaw_normalized + np.pi) % (2 * np.pi) - np.pi
        angular_size_of_robot = np.arctan(robot_radius / distance_to_target)
        effective_half_fov_rad = np.radians(100 / 2) - angular_size_of_robot
        within_fov = abs(angle_difference+40) <= effective_half_fov_rad
        #if within_fov:
            #rospy.loginfo(f"{target_name} is within the adjusted FoV of {observer_name}.")
        #else:
            #rospy.loginfo(f"{target_name} is NOT within the adjusted FoV of {observer_name}.")

        return within_fov

    
    def pause_slam(self, robot_namespace):
        try:
            if not self.slam_paused[robot_namespace]:  # Only pause if not already paused
                self.pause_services[robot_namespace]()
                rospy.loginfo(f"SLAM paused for {robot_namespace}")
                self.slam_paused[robot_namespace] = True
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to pause SLAM for {robot_namespace}: {e}")

    def resume_slam(self, robot_namespace):
        try:
            if self.slam_paused[robot_namespace]:  # Only resume if it was paused
                self.resume_services[robot_namespace]()
                rospy.loginfo(f"SLAM resumed for {robot_namespace}")
                self.slam_paused[robot_namespace] = False
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to resume SLAM for {robot_namespace}: {e}")

if __name__ == "__main__":
    rospy.init_node('proximity_slam_controller')

    robot_namespaces = ['tb3_0', 'tb3_1', 'tb3_2']
    rtabmap_namespaces = ['rtabmap_0', 'rtabmap_1', 'rtabmap_2']
    proximity_threshold = 10 # Define the threshold distance in meters

    controller = ProximitySlamController(robot_namespaces, rtabmap_namespaces, proximity_threshold)
    rospy.spin()