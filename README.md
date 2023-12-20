---
title: Drone Morphing ROS interface
updated: 2023-12-20 01:07:48Z
created: 2023-12-20 00:58:10Z
latitude: 28.66189760
longitude: 77.22739580
altitude: 0.0000
---

The package consists of two directories:
- **src:** This contains the code for ROS nodes and necessary processing.
- **scripts:** This contains the code for transforms and other setup related to the drone hardware.

For implementing this realsense-ros package([Link](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)) is required to be built in the system.

Run the following commands to setup this package in your system:

```shell
cd ~/Documents/
mkdir -p drone_morphing_ros/src
cd drone_morphing_ros/src/
git clone https://github.com/LShivaRudra/drone_morphing_ros
cd ../
catkin build
```

To execute the package:

Terminal 1:
```shell
cd <path_to_realsense-ros workspace>
source devel/setup.bash
roslaunch realsense2_camera demo_pointcloud.launch
```

Terminal 2:
```shell
cd ~/Documents/drone_morphing_ros/
source devel/setup.bash
rosrun drone_morphing_ros image_listener.py
```
