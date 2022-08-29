# Armer Panda
[![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://github.com/qcr/armer/workflows/Build/badge.svg?branch=master)](https://github.com/qcr/armer/actions?query=workflow%3ABuild)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/qcr/armer.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/qcr/armer/context:python)
[![Coverage](https://codecov.io/gh/qcr/armer/branch/master/graph/badge.svg)](https://codecov.io/gh/qcr/armer)



*To be used with the [Armer Driver](https://github.com/qcr/armer)*

This package launches the drivers of the Franka Panda for use with the [Armer Driver](https://github.com/qcr/armer).

It interfaces with the [Franka drivers](https://github.com/frankaemika/franka_ros) so they must be installed as well.

To enable the Panda's extra functionality, the Armer Driver ROSRobot class has been extended by [PandaROSRobot.py](https://github.com/qcr/armer_panda/blob/main/armer_panda/robots/PandaROSRobot.py). 

## Installation

### Preinstallation step: Install Franka drivers
1. Install Franka drivers for Noetic from apt
```
sudo apt install ros-noetic-franka-ros
```

### Armer Panda installation
1. Clone this repository into the armer_ws/src folder.

```
cd ~/armer_ws
```
```sh
git clone https://github.com/qcr/armer_panda.git src/armer_panda
```
3. Install the required dependencies.
```sh
rosdep install --from-paths src --ignore-src -r -y 
```
4. Build the packages.
```sh
catkin_make 
```

## Usage
```sh
roslaunch armer_panda robot_bringup.launch 
```
 By default this will launch to control a physical Panda arm. To run a Swift simulation the sim parameter can be set to true. For example:

```sh
roslaunch armer_ur robot_bringup.launch sim:=true 
```
