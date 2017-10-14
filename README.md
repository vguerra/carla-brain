# Self Driving Car Engineer Nanodegree -- Capstone project

[![Codacy Badge](https://api.codacy.com/project/badge/Grade/79ddbbe506054e43859247d9fd0b11b5)](https://www.codacy.com/app/Kairos-Automotive/carla-brain?utm_source=github.com&utm_medium=referral&utm_content=Kairos-Automotive/carla-brain&utm_campaign=badger) 
[![Build Status](https://travis-ci.org/Kairos-Automotive/carla-brain.svg?branch=master)](https://travis-ci.org/Kairos-Automotive/carla-brain) 
[![Docker Status](https://dockerbuildbadges.quelltext.eu/status.svg?organization=kairosautomotive&repository=carla-brain)](https://hub.docker.com/r/kairosautomotive/carla-brain/) 

This is the project repo for the final project of the Udacity
Self-Driving Car Nanodegree: Programming a Real Self-Driving Car.

## Team Members

This repository is maintained by the following:
- [Alexey Simonov](https://github.com/asimonov)
- [Victor Guerra](https://github.com/vguerra)
- [Ralf Beier](https://github.com/avrabe)
- [Peter Luptak](https://github.com/dabavil)
- [Gustavo Espindola](https://github.com/yhoazk)


## Workflow

Please see [wiki](https://github.com/Kairos-Automotive/carla-brain/wiki) for complete description.


## Installation

We use ROS Kinetic on Ubuntu 16.04

### Native Install

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus.
 [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).

The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow [these instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu) to install ROS
* Install [Dataspeed DBW SDK](https://bitbucket.org/DataspeedInc/dbw_mkz_ros).
  On a workstation that already has ROS installed use this option:
  [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)

### Docker Install
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t kairosautomotive/carla-brain:latest
```

or pull the latest docker container from dockerhub
```bash
docker pull kairosautomotive/carla-brain:latest
```
The prefered way for using containers is to use the prebuild.

Run the docker file
```bash
docker run -p 127.0.0.1:4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it kairosautomotive/carla-brain
```

### Simulator

Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2)


## Build and Run in Simulator

1. Clone the project repository
```bash
git clone https://github.com/Kairos-Automotive/carla-brain.git
```

2. Install python dependencies
```bash
cd carla-brain
pip install -r requirements.txt
```

3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
If the connection of the simulator is not working as expected it 
is possible to lauch with the workaround described in the [discussion](https://discussions.udacity.com/t/car-freezes-in-simulator-solved/363942/12?u=victor_guerra_986699).

```bash
roslaunch launch/styx.launch monkey_patch:=true
```

To use the visualizer use below commands to configure the environment:
```bash
xhost +SI:localuser:root
docker run -p 127.0.0.1:4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --rm -it kairosautomotive/carla-brain:latest
source devel/setup.sh
roslaunch launch/styx.launch &
rosrun waypoint_updater show_waypoints.py
```
4. Run the simulator

## Visualize the drive
### Mac with X11 and docker
#### Setup
Install and start X11 (XQuartz).
```bash
brew cask install xquartz
open -a Xquartz
```

Set the preferences as described in [docker for mac and gui](https://fredrikaverpil.github.io/2016/07/31/docker-for-mac-and-gui-applications/).
Restart Xquartz.

#### Setup the environment and start docker 
```bash
ip=$(ifconfig en0 | grep inet | awk ‘$1==“inet” {print $2}’)
xhost + $ip
docker run -p 127.0.0.1:4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$ip:0 --rm -it kairosautomotive/carla-brain:latest
source devel/setup.sh
roslaunch launch/styx.launch &
rosrun waypoint_updater show_waypoints.py
```
 
### Linux with X11 and docker
To use the visualizer use below commands to configure the environment:
```bash
xhost +SI:localuser:root
docker run -p 127.0.0.1:4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --rm -it kairosautomotive/carla-brain:latest
source devel/setup.sh
roslaunch launch/styx.launch &
rosrun waypoint_updater show_waypoints.py
```

## Unit tests

To run the unit tests call
```bash
source devel/setup.sh
catkin_make run_tests
```

## Test with ROS bags from real car

1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing)
that was recorded on the Udacity self-driving car (a bag demonstraing the correct
predictions in autonomous mode can be found
[here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))

2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```

3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```

4. Launch your project in site mode
```bash
cd carla-brain/ros
roslaunch launch/site.launch
```

5. Confirm that traffic light detection works on real life images

