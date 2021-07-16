# Armer Panda

This package launches the drivers of the Franka Panda for use with the [Armer Driver](https://github.com/qcr/armer).

It interfaces with the [Franka drivers](https://github.com/frankaemika/franka_ros) so they must be installed.


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
git clone https://github.com/qcr/armer_panda.git src
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
