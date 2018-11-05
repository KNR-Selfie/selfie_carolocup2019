# Selfie @ Carolo-Cup 2019

This repository contains the [catkin workspace](http://wiki.ros.org/catkin/workspaces) for Selfie Autonomous Car at [Carolo-Cup 2019](https://wiki.ifr.ing.tu-bs.de/carolocup/en/event-history/2019/dates) competition.


## Build instructions

The project is targetting [ROS Kinetic Kame](http://wiki.ros.org/kinetic) distribution. Before proceeding, make sure you have it [installed](http://wiki.ros.org/kinetic/Installation) on your development machine.

First, clone the repository to a convenient location using:

```bash
git clone https://github.com/KNR-Selfie/selfie_carolocup2019
```

Navigate to the main directory with:

```bash
cd selfie_carolocup2019
```

Lastly, the following set of commands will in turn download all external dependencies, build the packages in `src` directory and include them in your environment.

```bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source ./devel/setup.bash
```


