Globally_Aligned_Events
=======================

The source code is released under the **MIT License**.
The code works best on Ubuntu 16.04

# Prerequisites
## ROS
The code is based on [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu).

## OpenCV
We use [OpenCV](http://opencv.org) to visualize and manipulate images.

## Eigen3
We use [Eigen3](http://eigen.tuxfamily.org) for matrix operations.

## rpg_dvs_ros (optional: for event cameras)
To publish the event messeages from an event camera[rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros/).


# Installation
 
0. Create a catkin workspace (for first-time user of ROS):
```
$ cd
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
$ catkin_init_workspace
```

1. Clone this repository:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/Haram-kim/Globally_Aligned_Events.git
```

2. Build the packages:
```
$ catkin build event_publisher
$ catkin build event_bundler
$ catkin build rotation_estimator
```

3. Set up the environment:
```
. devel/setup.bash
```
