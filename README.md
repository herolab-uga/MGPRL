# MGPRL

This repository contains the ROS package that implements the works from the algorithm "MGPRL: Distributed Multi-Gaussian Processes for Wi-Fi-based Multi-Robot Relative Localization in Large Indoor Environments". The following figure shows the conceptual overview of MGPRL:

![](multimedia/Proposed_Methodology.pdf)


### Experimental Demonstration and Video
https://github.com/user-attachments/assets/4e24ec7f-a444-4c3b-a3e0-17c09267e1a9

## Requirements & Environment Setup
The package has been tested on ROS Noetic for both simulated and hardware Turtlebot3 robot (waffle) & Turtlebot2 (2e and 2i). The following requirements are needed before installing the package:

1. **ROS Installation**: You should have installed a ROS distribution (Noetic) on Ubuntu 20.04.
2. **Workspace Creation**: Make sure you have created a ROS workspace. You can follow the ROS [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) to do this.
3. **Install the `rtabmap`  package with the following command:
    ```bash
    sudo apt-get install ros-noetic-rtabmap-ros
    ```
4. **Install ROS Navigation Stack**: You can install the ROS navigation stack using the following command:
    ```bash
    sudo apt-get install ros-noetic-navigation
    ```
6. **Python Modules Installation**: Make sure python version is 3.8+. Install the following Python modules using `pip`:
    ```bash
    pip install numpy pandas matplotlib open3d opencv-python scikit-learn
    ```
7. **Turtlebot3 Installation**:
    ```bash
    cd ~/catkin_ws/src/
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
    source ~/.bashrc
    sudo apt-get install ros-noetic-turtlebot3*
    ```
8. **Exploration Framework Installation**: We utilize [SPACE](https://github.com/herolab-uga/SPACE-MAP) for exploration in ROS simulations.
    ```bash
    cd ~/catkin_ws/src/
    git clone https://github.com/herolab-uga/SPACE-MAP.git
    ```

8. **AWS Gazebo Worlds Installation**:
    ```bash
    cd ~/catkin_ws/src/
    https://github.com/aws-robotics/aws-robomaker-small-house-world.git
    https://github.com/aws-robotics/aws-robomaker-bookstore-world.git
    ```

9. **Move_Base Navigation**:
    ```bash
    sudo apt-get install ros-$ROS_noetic-move-base
    ```
10. ** Python Libraries Installation**:
    ```bash
    pip install --upgrade pip
    # Install required Python libraries
    pip install opencv-python
    pip install matplotlib
    pip install scikit-learn
    pip install scipy
    pip install numpy
    pip install pandas
    pip install GPy
    ```
10. **Environment Initialization**:
    ```bash
    cd ~/catkin_ws/src/
    git clone https://github.com/herolab-uga/SPACE-MAP.git
    catkin_make
    source devel/setup.bash
    ```

## ROS Nodes
- **Gazebo Simulation**: Launches simulation environments (e.g., AWS House, AWS Bookstore) using launch files.
- **Robot State Publisher**: Publishes robot states from URDF models at high frequency (e.g., 100 Hz).
- **Model Spawner**: Places TurtleBot3 models in Gazebo at specified coordinates.
- **RTAB-Map SLAM**: Processes RGB-D inputs for SLAM, operating at specified frequencies and saving mapping data.
- **RGBD Synchronization**: Synchronizes RGB and depth image streams for SLAM processing.
- **RTAB-Map Visualization**: Visualizes SLAM outputs and odometry for debugging and analysis.
- **Map Merger**: Merges grid maps from multiple robots even with unknown initial positions.
- **Move Base**: Manages local and global path planning and navigation.
- **Frontier Assigner**: Allocates exploration tasks to robots based on predefined criteria.
- **RSSI Simulation Node**: `RSSI_Simulation_House.py` – simulates RSSI fields with multiple fixed Access Points.
- **RSSI Robot Nodes**: 
  - `rssi_robot_1.py`
  - `rssi_robot_2.py`
  - `rssi_robot_3.py`  
  These nodes collect RSSI measurements (using the robot’s ground-truth pose), perform multi-output Gaussian Process regression to predict RSSI fields, detect global and local maxima with uncertainty-based weighting, compute convex hulls of detected AP positions, and publish their AP estimates (in their own frame) while subscribing to similar data from other robots for relative localization.

## ROS Topics
- **Odometry & Control**
  - `${robot_namespace}/odom`: Provides odometry data.
  - `${robot_namespace}/cmd_vel`: Receives velocity command inputs.
- **Mapping & Perception**
  - `${robot_namespace}/cloud_map`: Outputs individual robot maps (e.g., from RTAB-Map).
  - `${robot_namespace}/scan`: Publishes laser scan data for navigation.
  - `${robot_namespace}/camera/rgb/image_raw` & `${robot_namespace}/camera/depth/image_raw`: Publish RGB and depth image streams.
  - `${robot_namespace}/camera/rgb/camera_info`: Provides metadata for the RGB camera.
  - `${robot_namespace}/move_base`: Each robot’s Move Base server for local/global planning.
- **Map Merging & Frontier Detection**
  - `/total_map`: Publishes the merged 2D grid map.
  - `/frontier_marker`: Publishes 2D/3D spatial frontier markers.
  - `/{target_name}/masker/masked_image_raw`: Publishes masked images for feature extraction and mapping.
- **RSSI & Relative Localization**
  - `/robot1_pose`, `/robot2_pose`, `/robot3_pose`: Publish each robot’s ground-truth pose.
  - `/robot1_ap_data`, `/robot2_ap_data`, `/robot3_ap_data`: Each robot publishes its detected AP positions (global and candidate positions) with uncertainty weights in its own frame.
  - `/robot1_relative`, `/robot2_relative`, `/robot3_relative`: Each robot publishes computed relative transform data (alignment results) for other robots.

## Launch
Run one of the example launch (includes rviz) files included in this ROS package. For instance, 
```bash
    roslaunch <package_name> house.launch/ bookstore.launch
```
## Contributions

- **Sai Krishna Ghanta** - PhD Candidate
- **Dr. Ramviyas Parasuraman** - Lab Director

### [Heterogeneous Robotics Lab](https://hero.uga.edu/)
School of Computing, University of Georgia.


![](Images/Lab.png)

For further information, please contact Dr. Ramviyas Parasuraman at [ramviyas@uga.edu](mailto:ramviyas@uga.edu). 

