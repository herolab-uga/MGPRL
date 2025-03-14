#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
import os
import std_msgs.msg
from std_msgs.msg import String


class ProximitySlamController:
    def __init__(self, robot_namespaces, rtabmap_namespaces, proximity_threshold):
        self.robot_positions = {}
        self.robot_orientations = {}
        self.proximity_threshold = proximity_threshold
        self.odom_subscribers = []
        self.pause_services = {}
        self.resume_services = {}
        self.slam_paused = {}
        
        self.bridge = CvBridge()
        self.image_publishers = {}
        self.image_subscribers = []
        self.depth_image_subscribers = []
        self.image_data = {}
        self.depth_image_data = {}
        self.msg_header = std_msgs.msg.Header()
        self.depth_msg_header = std_msgs.msg.Header()
        
        image_width = 1920
        image_height = 1080
        horizontal_fov = 1.3439  # radians

        # Calculate focal lengths
        fx = 0.5 * image_width / np.tan(0.5 * horizontal_fov)
        fy = fx  # Assuming square pixels
        cx, cy = image_width / 2, image_height / 2

        self.camera_params = {}
        for robot_namespace in robot_namespaces:
            self.camera_params[robot_namespace] = {
                'camera_matrix': np.array([
                    [fx, 0, cx],
                    [0, fy, cy],
                    [0, 0, 1]
                ]),
                'rotation_vector': np.array([0.83606397, -0.48346672, 0.81249374]),
                'translation_vector': np.array([[-0.76478351], [0.91356075], [-0.05974413]])
            }


        for robot_namespace, rtabmap_namespace in zip(robot_namespaces, rtabmap_namespaces):
            # Odometry
            odom_topic = f"/{robot_namespace}/odom"
            self.odom_subscribers.append(rospy.Subscriber(odom_topic, Odometry, self.odom_callback, robot_namespace))

            # SLAM services
            pause_service_name = f"/{rtabmap_namespace}/pause"
            resume_service_name = f"/{rtabmap_namespace}/resume"
            self.pause_services[robot_namespace] = rospy.ServiceProxy(pause_service_name, Empty)
            self.resume_services[robot_namespace] = rospy.ServiceProxy(resume_service_name, Empty)
            self.slam_paused[robot_namespace] = False

            # RGB and Depth image topics
            rgb_topic = f"/{robot_namespace}/camera/rgb/image_raw"
            depth_topic = f"/{robot_namespace}/camera/depth/image_raw"
            self.image_subscribers.append(rospy.Subscriber(rgb_topic, Image, self.camera_callback, robot_namespace))
            self.depth_image_subscribers.append(rospy.Subscriber(depth_topic, Image, self.depth_camera_callback, robot_namespace))

    def odom_callback(self, msg, robot_namespace):
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.robot_positions[robot_namespace] = np.array([position.x, position.y])
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.robot_orientations[robot_namespace] = yaw
        self.check_proximity_and_control_slam()

    def depth_camera_callback(self, data, robot_namespace):
        #rospy.loginfo(f"Received depth image for {robot_namespace}")
        try:
            self.depth_image_data[robot_namespace] = data
            cv_depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")
        except CvBridgeError as e:
            rospy.logerr(f"Error converting ROS Depth Image message to OpenCV image: {str(e)}")

    def camera_callback(self, data, robot_namespace):
        #rospy.loginfo(f"Received image for {robot_namespace}")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_data[robot_namespace] = cv_image
            self.msg_header = data.header
        except CvBridgeError as e:
            rospy.logerr(f"Error converting ROS Image message to OpenCV image: {str(e)}")

    def check_proximity_and_control_slam(self):
        robot_names = list(self.robot_positions.keys())
        if len(robot_names) < 2:
            return  # Not enough robots to compare proximity

        for observer_name in robot_names:
            robots_in_fov = []
            for target_name in robot_names:
                if observer_name == target_name:
                    continue  # Skip self

                if self.is_within_fov(observer_name, target_name) and self.calculate_distance(observer_name, target_name) < self.proximity_threshold:
                    robots_in_fov.append(target_name)
                    self.masker(observer_name, observer_name, target_name)
                else:
                    # Directly republish the depth image stored in the dictionary
                    if target_name in self.depth_image_data:
                        depth_msg = self.depth_image_data[target_name]
                        pub_topic = f"/{target_name}/masker/masked_image_raw"
                        if target_name not in self.image_publishers:
                            self.image_publishers[target_name] = rospy.Publisher(pub_topic, Image, queue_size=10)
                        self.image_publishers[target_name].publish(depth_msg)

                    

    def save_image(self, observer_name, target_name):
        if observer_name in self.image_data and observer_name in self.depth_image_data:
            rgb_image = self.image_data[observer_name]
            depth_image = self.depth_image_data[observer_name]
            observer_pos = self.robot_positions[observer_name]
            target_pos = self.robot_positions[target_name]
            observer_yaw = self.robot_orientations[observer_name]
            target_yaw = self.robot_orientations[target_name]

            base_directory = "/home/krishna/catkin_ws/src/MEAL/multi_explore/src"
            rgb_directory = f"{base_directory}/RGB"
            depth_directory = f"{base_directory}/Depth"
            if not os.path.exists(rgb_directory):
                os.makedirs(rgb_directory)
            if not os.path.exists(depth_directory):
                os.makedirs(depth_directory)

            filename = f"{observer_name}_pos_{observer_pos[0]:.2f}_{observer_pos[1]:.2f}_yaw_{observer_yaw:.2f}_vs_{target_name}_pos_{target_pos[0]:.2f}_{target_pos[1]:.2f}_yaw_{target_yaw:.2f}.png"
            rgb_filename = f"{rgb_directory}/{filename}"
            depth_filename = f"{depth_directory}/{filename}"

            cv2.imwrite(rgb_filename, rgb_image)
            cv2.imwrite(depth_filename, depth_image)
            #rospy.loginfo(f"RGB and Depth images saved: {rgb_filename}, {depth_filename}")

            # Clear the saved image data to free up memory
            del self.image_data[observer_name]
            del self.depth_image_data[observer_name]
        else:
            rospy.logwarn(f"No complete image data available for {observer_name} to save.")


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
        within_fov = abs(angle_difference) <= effective_half_fov_rad
        #if within_fov:
        #    rospy.loginfo(f"{target_name} is within the adjusted FoV of {observer_name}.")
        #else:
        #    rospy.loginfo(f"{target_name} is NOT within the adjusted FoV of {observer_name}.")
        return within_fov

    def pause_slam(self, robot_namespace):
        try:
            if not self.slam_paused[robot_namespace]:  # Only pause if not already paused
                self.pause_services[robot_namespace]()
                #rospy.loginfo(f"SLAM paused for {robot_namespace}")
                self.slam_paused[robot_namespace] = True
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to pause SLAM for {robot_namespace}: {e}")

    def resume_slam(self, robot_namespace):
        try:
            if self.slam_paused[robot_namespace]:  # Only resume if it was paused
                self.resume_services[robot_namespace]()
                #rospy.loginfo(f"SLAM resumed for {robot_namespace}")
                self.slam_paused[robot_namespace] = False
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to resume SLAM for {robot_namespace}: {e}")

    def transform_to_observer_frame(self, target_pos, observer_pos, observer_yaw, observer_height):
        relative_pos_2d = target_pos - observer_pos
        relative_pos_3d = np.array([relative_pos_2d[0], relative_pos_2d[1], 0])
        observer_3d_pos = np.array([0, observer_height, 0])  
        distance_xy = np.linalg.norm(relative_pos_2d)
        relative_pos_3d[2] = distance_xy  
        relative_pos_3d[1] = observer_height 
        rotation_matrix = np.array([
            [np.cos(-observer_yaw), -np.sin(-observer_yaw), 0],
            [np.sin(-observer_yaw), np.cos(-observer_yaw), 0],
            [0, 0, 1]  
        ])
        transformed_pos = rotation_matrix @ relative_pos_3d  
        return transformed_pos
    
    def world_to_image_coordinates(X, Y, Z, camera_matrix, rotation_vector, translation_vector):
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        extrinsic_matrix = np.hstack((rotation_matrix, translation_vector))
        projection_matrix = camera_matrix.dot(np.vstack((extrinsic_matrix, [0, 0, 0, 1]))[:3, :])  # Ensure it is a 3x4 matrix
        world_coordinates = np.array([X, Y, Z, 1])
        image_coordinates = projection_matrix.dot(world_coordinates)
        image_coordinates /= image_coordinates[-1]
        return int(image_coordinates[0]), int(image_coordinates[1])
    
    def masker(self, robot_namespace, observer_name, target_name):
        observer_pos = self.robot_positions[observer_name]
        target_pos = self.robot_positions[target_name]
        observer_yaw = self.robot_orientations[observer_name]
        params = self.camera_params[robot_namespace]
        camera_matrix = params['camera_matrix']
        rotation_vector = params['rotation_vector']
        translation_vector = params['translation_vector']
        
        relative_pos = self.transform_to_observer_frame(target_pos, observer_pos, observer_yaw, 0.2)
        u, v = self.world_to_image_coordinates(relative_pos[0], relative_pos[1], relative_pos[2], camera_matrix, rotation_vector, translation_vector)
        mask_size = 700  # Size of the square mask
        depth_image = self.depth_image_data[robot_namespace]
        mask = np.zeros_like(depth_image)
        start_x = max(v - mask_size // 2, 0)
        end_x = min(v + mask_size // 2, depth_image.shape[0])
        start_y = max(u - mask_size // 2, 0)
        end_y = min(u + mask_size // 2, depth_image.shape[1])
        mask[start_x:end_x, start_y:end_y] = 1
        masked_depth_image = np.where(mask == 1, depth_image, 0)
        try:
            masked_depth_msg = self.bridge.cv2_to_imgmsg(masked_depth_image, "32FC1")
            masked_depth_msg.header = self.depth_image_data[robot_namespace].header
            if robot_namespace in self.image_publishers:
                self.image_publishers[robot_namespace].publish(masked_depth_msg)
            else:
                pub_topic = f"/{robot_namespace}/masker/masked_image_raw"
                self.image_publishers[robot_namespace] = rospy.Publisher(pub_topic, Image, queue_size=10)
                self.image_publishers[robot_namespace].publish(masked_depth_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Error converting OpenCV image to ROS message: {str(e)}")

if __name__ == "__main__":
    rospy.init_node('proximity_slam_controller')

    robot_namespaces = ['tb3_0', 'tb3_1']
    rtabmap_namespaces = ['rtabmap_0', 'rtabmap_1']
    proximity_threshold = 5.0  # Define the threshold distance in meters

    controller = ProximitySlamController(robot_namespaces, rtabmap_namespaces, proximity_threshold)
    rospy.spin()

