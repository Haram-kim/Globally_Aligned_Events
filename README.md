Globally_Aligned_Events
=======================

<img src="https://larr.snu.ac.kr/haramkim/SNU_LARR.png" alt="SNU LARR logo" width = "600">

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
$ catkin config --init --mkdirs --extend /opt/ros/kinetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
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

## Option 1: data set 

When you use data set (check out the format described in below),

set data set directory in ` ~/catkin_ws/src/event_publisher/launch/event_publisher.launch`.
```
<param name="filename" type="string" value="{Set this value}" />
```

Launch the execute file.
```
roslaunch rotation_estimator rotation_estimation.launch
```

## Option 2: event cameras

When you use an event camera, install the [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros/) package.

Launch the execute file.
```
roslaunch rotation_estimator rotation_estimation_davis.launch
```

# Configuration settings

## Event publisher

In ` ~/catkin_ws/src/event_publisher/config/config.yaml`,

`width` and `height`: the camera parameters of the data set.

`Event.delta_time` and `Event.delta_time`: the events number of an event array message.

## Rotation estimator

In ` ~/catkin_ws/src/rotation_estimator/config/config.yaml`,

`sampling_rate`: rate of systemmatic samplling for events.

`map_sampling_rate`: rate of systemmatic samplling for globally aligned events.

`event_image_threshold`: event image theshold T_rho.

`rotation_estimation`: on/off (1/0) the proposed method.

`run_index`:  0- runs in real-time, x- runs for the x temporal windows instead of real-time operation.


# Data set format

## File structure
```
/dataset_name
  /image
  /events.txt
  /images.txt
  /imu.txt
  /groundtruth.txt
```

### /image : timestamp (sec)
```
/image
  /{timestamp}.png
  /{timestamp}.png
  ...
```
### `events.txt` format : polarity(0/1)
```
# events
# timestamp x y polarity
{timestamp} {x} {y} {polarity}
{timestamp} {x} {y} {polarity}
...
```
### `images.txt` format
```
# grey images
# timestamp filename
{timestamp} image/{timestamp}.png
{timestamp} image/{timestamp}.png
...
```
### `imu.txt` format
```
# imu
# acceleration gyroscope
# timestamp ax ay az gx gy gz
{timestamp} {ax} {ay} {az} {gx} {gy} {gz}
{timestamp} {ax} {ay} {az} {gx} {gy} {gz}
...
```
### `groundtruth.txt` format
```
# groundtruth
# position quaternion
# timestamp x y z qx qy qz qw
{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}
{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}
...
```

