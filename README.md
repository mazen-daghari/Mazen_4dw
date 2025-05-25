
## About

Mazen_4dw is a ROS 2 package designed for ROS 2 Foxy and later distributions. It simulates and controls a 4-wheel-drive (4WD) robot equipped with various sensors, such as a 2D LiDAR, camera, and other modules for perception and navigation.

The package supports Gazebo simulation and RViz visualization, along with YOLOv8-based object detection, making it suitable for research and development in autonomous robotics, SLAM, and AI-based perception.


![Logo](https://github.com/mazen-daghari/Mazen_4dw/blob/77e424a2dfffa17c5dbb93baea33e0d8d2a60c1e/logo.png)


## Features
![1](https://github.com/mazen-daghari/mazeN/blob/b3823cf7ffdf21d697620cc41f57276ea91076f6/moving_robot.gif)
## Features

- 4WD robot model with realistic physics in Gazebo

* Integrated camera for image processing and computer vision tasks

* 2D LiDAR support for mapping, obstacle avoidance, and SLAM

* Object recognition using YOLOv8 (Ultralytics)

* RViz support for visualizing robot states and sensor data

* Modular launch files for easy simulation and testing



## Installation

Create project directory

```bash
  mkdir -p ~/mazen_ws/src cd ~/mazen_ws/src
```

Clone the project

```bash
  git clone https://github.com/mazen-daghari/Mazen_4dw.git
```

Build project

```bash
 colcon build --symlink-install
```

Source project

```bash
  source install/setup.bash 
```

Launch Gazebo simulation

```bash
  ros2 launch mazen_4wd gazebo_model.launch.py
```

Launch Yolo v8 

```bash
  ros2 launch recognition launch_yolov8.launch.py
```

## Roadmap

- Create robot urdf
- Add sensors 
- Add extended kalman filer
- Add yolo v8 model to simulation
- Add teleop twist keyboard script 


## Acknowledgements

 - [Ultralytics](https://github.com/ultralytics/ultralytics)
 



## Notes


- Make sure all dependencies (e.g., YOLOv8, camera drivers, etc.) are correctly installed.

- Tested on Ubuntu 20.04 with ROS 2 Foxy. Later ROS 2 versions like Humble and Iron should also work with minor adjustments.

- models like person suv stop_sign bus etc.. must be copyed to .gazebo/models (directory may be hidden check hidden file option to see it ) first time only 

- for any further help contact me on dagmazen@gmail.com or via linkedin 

## Authors

- [@mazen-daghari](https://www.github.com/mazen-daghari)


## License

[MIT](https://choosealicense.com/licenses/mit/)

