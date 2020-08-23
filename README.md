# ROS Navigation and Localisation Study

This repository contains the setup for studying localisation and navigation with a vehicle emulator.

## Usage
```
$ source /opt/ros/melodic/setup.bash
$ source .../ros-study-01/devel/setup.bash
$ roslaunch ros_study test_navi_loca.launch &
```

You can use `rviz` to set navigation goals and `rqt_reconfigure` to set parameters of the navigation.

## Components

It uses the [navigation stack]() and the [`robot_localization`]() package. The robot is emulated by a simple vehicle emulator.

### Vehicle Emulator

A simple emulator with GPS, compass and odometry.

#### Subscribed Topics

* `/cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))
  Receive velocity commands

#### Published Topics

* `/fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))
  GPS position with error.
* `/imu` ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))
  Vehicle orientation only. No angular velocity and acceleration components.
* `/twist` ([geometry_msgs/TwistWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html))
  Linear and angular velocity. The covariance matrix is all zero.
* `/path` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))
  Path for debugging

#### Node Graph

<p align="center">
  <img src="docs/rqt_graph_01.png">
</p>
