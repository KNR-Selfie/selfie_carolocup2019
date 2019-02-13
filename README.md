# Selfie @ Carolo-Cup 2019

This repository contains the [catkin workspace](http://wiki.ros.org/catkin/workspaces) for Selfie Autonomous Car at [Carolo-Cup 2019](https://wiki.ifr.ing.tu-bs.de/carolocup/en/event-history/2019/dates) competition.


## Build instructions

The project is targetting [ROS Kinetic Kame](http://wiki.ros.org/kinetic) distribution. Before proceeding, make sure you have it [installed](http://wiki.ros.org/kinetic/Installation) on your development machine.

First, clone the repository to a convenient location using:

```bash
git clone --recurse-submodules https://github.com/KNR-Selfie/selfie_carolocup2019
```

Navigate to the main directory with:

```bash
cd selfie_carolocup2019
```

Lastly, the following set of commands will in turn download all external dependencies, build the packages in `src` directory and include them in your environment.

```bash
./resolve-dependencies.sh
catkin_make
source ./devel/setup.bash
```

## Running

```bash
roslaunch selfie initialization.launch
```
Remember to put an appropriate `homography_export.yaml` file generated from [this program](https://github.com/tum-phoenix/drive_ros_camera_homography) in your `ROS_HOME` directory.
