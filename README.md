# Mazen_4dw

Mazen_4dw is a ROS 2 package designed for ROS 2 Foxy and later distributions. It simulates and controls a 4-wheel-drive (4WD) robot equipped with various sensors, such as a 2D LiDAR, camera, and other modules for perception and navigation.

The package supports Gazebo simulation and RViz visualization, along with YOLOv8-based object detection, making it suitable for research and development in autonomous robotics, SLAM, and AI-based perception.

-- Features
 4WD robot model with realistic physics in Gazebo

 Integrated camera for image processing and computer vision tasks

 2D LiDAR support for mapping, obstacle avoidance, and SLAM

 Object recognition using YOLOv8 (Ultralytics)

 RViz support for visualizing robot states and sensor data

 Modular launch files for easy simulation and testing

-- How to Set Up
Follow these steps to set up and run the project:


# Create a new ROS 2 workspace
mkdir -p ~/mazen_ws/src
cd ~/mazen_ws/src

# Clone the project repository
git clone <project_link>

# Build the workspace
cd ~/mazen_ws
colcon build --symlink-install

# Source the setup file
source install/setup.bash
-- Running the Simulation
Use the following launch commands to run the robot in simulation and test recognition:

- Gazebo + RViz Simulation
ros2 launch mazen_4wd gazebo_model.launch.py


- YOLOv8 Object Detection
ros2 launch recognition launch_yolov8.launch.py

*Notes
Make sure all dependencies (e.g., YOLOv8, camera drivers, etc.) are correctly installed.

Tested on Ubuntu 20.04 with ROS 2 Foxy. Later ROS 2 versions like Humble and Iron should also work with minor adjustments.


