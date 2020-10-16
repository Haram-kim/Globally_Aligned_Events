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
To publish the event messeages from an event camera, please use the [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros/) package.


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

# Launch files

If you use data set
```
roslaunch event_publisher event_publisher.launch
```


# Data set format

## File structure
```
/dataset_name
  /image
  /events.txt
  /images
  /imu.txt
```

## /image
```
/image
  /{timestamp}.png
  /{timestamp}.png
  ...
```
## `events.txt` format
```
# events
# timestampe x y polarity
{timestamp} {x} {y} {polarity}
{timestamp} {x} {y} {polarity}
...
```
## `images.txt` format
```
# grey images
# timestamp filename
{timestamp} image/{timestamp}.png
{timestamp} image/{timestamp}.png
...
```
## `imu.txt` format
```
# imu
# acceleration gyroscope
# timestamp ax ay az gx gy gz
{timestamp} {ax} {ay} {az} {gx} {gy} {gz}
{timestamp} {ax} {ay} {az} {gx} {gy} {gz}
...
```

