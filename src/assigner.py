#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import numpy as np
from tf.transformations import quaternion_from_euler
from multi_explore.msg import Frontier
from frontier_finder import FrontierFinder
from nav_msgs.msg import Odometry
from math import sqrt

class Robot:
    def __init__(self, name):
        self.name = name
        self.action_client = actionlib.SimpleActionClient(f"{name}/move_base", MoveBaseAction)
        self.action_client.wait_for_server(rospy.Duration(30))
        rospy.loginfo(f"Robot {name} connected to move_base action server")
        self.last_goal_pose = None
        self.last_robot_pose = None
        self.distance_threshold = 1.0  # Threshold distance between goal and robot
        self.time_threshold = rospy.Duration(5)  # Time threshold for being close to the goal
        self.time_at_goal = None

    def send_goal(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        q = quaternion_from_euler(0, 0, theta)  # Convert theta to quaternion
        goal.target_pose.pose.orientation = Quaternion(*q)
        goal.target_pose.pose.position = Point(x, y, 0)
        self.action_client.send_goal(goal)
        self.last_goal_pose = goal.target_pose.pose
        rospy.loginfo(f"Goal sent to {self.name}: x={x}, y={y}, theta={theta}")
        self.time_at_goal = rospy.Time.now()

    def check_goal_reached(self, current_pose):
        if self.last_goal_pose is None:
            return False
        dx = self.last_goal_pose.position.x - current_pose.position.x
        dy = self.last_goal_pose.position.y - current_pose.position.y
        distance = sqrt(dx*dx + dy*dy)
        return distance < self.distance_threshold

    def check_stuck(self, current_pose):
        if self.last_robot_pose is None:
            return False
        dx = self.last_robot_pose.position.x - current_pose.position.x
        dy = self.last_robot_pose.position.y - current_pose.position.y
        distance = sqrt(dx*dx + dy*dy)
        if distance < self.distance_threshold:
            if (rospy.Time.now() - self.time_at_goal) > self.time_threshold:
                return True
        return False

    def update_last_pose(self, current_pose):
        self.last_robot_pose = current_pose


class FrontierAssigner:
    def __init__(self, robot_namespaces):
        self.robots = [Robot(name) for name in robot_namespaces]
        self.frontier_finder = FrontierFinder()
        rospy.loginfo("FrontierAssigner initialized with FrontierFinder instance.")

    def run_frontier_assignment(self):
        self.frontier_finder.spin()  # Process and publish frontiers for RViz
        self.assign_frontiers()

    def assign_frontiers(self):
        if len(self.frontier_finder.frontiers.frontiers) < len(self.robots):
            rospy.loginfo("Not enough frontiers for all robots.")
            return

        for robot in self.robots:
            if self.frontier_finder.frontiers.frontiers:
                frontier = self.frontier_finder.frontiers.frontiers.pop(0)
                robot.send_goal(frontier.pose.x, frontier.pose.y, frontier.pose.theta)

    def update_robot_poses(self, robot_poses):
        for robot, pose in zip(self.robots, robot_poses):
            robot.update_last_pose(pose)


if __name__ == "__main__":
    rospy.init_node('frontier_assigner')
    robot_namespaces = rospy.get_param('~robot_namespaces').split(',')
    assigner = FrontierAssigner(robot_namespaces)

    rate = rospy.Rate(0.5)  # Adjust the rate as needed
    while not rospy.is_shutdown():
        assigner.run_frontier_assignment()

        # Update robot poses
        robot_poses = [rospy.wait_for_message(f'/{name}/odom', Odometry).pose.pose for name in robot_namespaces]
        assigner.update_robot_poses(robot_poses)

        # Check for robots being stuck and randomly reassign them
        for robot in assigner.robots:
            current_pose = robot_poses[assigner.robots.index(robot)]
            if robot.check_stuck(current_pose):
                rospy.loginfo(f"Robot {robot.name} seems stuck. Reassigning...")
                nearest_frontiers = []  # Implement logic to find nearest frontiers
                frontier = np.random.choice(nearest_frontiers)
                robot.send_goal(frontier.pose.x, frontier.pose.y, frontier.pose.theta)

        rate.sleep()