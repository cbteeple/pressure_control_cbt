# Ctrl-P 2.0: ROS Driver
A module to do pressure control in ROS with a Ctrl-P pressure control system.

## Dependencies
### Hardware
- A pressure control system running my [Ctrl-P firmware](https://github.com/cbteeple/pressure_controller)
- A desktop computer running Linux (currently tested only in [Ubuntu 18.04](https://ubuntu.com/download/desktop))

### Software
- [ROS Melodic](http://wiki.ros.org/melodic/Installation)
- The [rqt_multiplot](http://wiki.ros.org/rqt_multiplot) package for nice plot layouts and custom axes
- Before installing python dependencies, [Cython HID Library](https://github.com/trezor/cython-hidapi) from Trezor has a few extra requirements on Ubuntu:
	``` bash
	sudo apt-get install python-dev libusb-1.0-0-dev libudev-dev
	sudo pip install --upgrade setuptools
	```
- Various python libraries:
	- All python dependencies are managed in the reqirements file. `pip install -r requirements.txt`


## Installation
1. Clone this package to the `src` folder of your catkin workspace.
2. Install cython-hid dependencies (see above).
3. In the root folder of this package, run `pip install -r requirements.txt` to install python dependencies.
4. In the root folder of your catkin workspace, run `catkin_make` to enable the custom python modules in this package to work.


## Usage
This driver is split into two ROS packages:
- **pressure_trajectory_ros** is where trajectories are set up, built, and stored for execution
- **pressure_control_ros** is the main driver that actually interfaces with the pressure control hardware

[See the Documentation](https://ctrl-p.cbteeple.com/ros-driver)

## About Ctrl-P
The Ctrl-P project is a full-stack pneumatic control system featuring smooth control of pressure at a high bandwidth.

Ctrl-P has three parts:
- [Arduino-Based Firmware](https://github.com/cbteeple/pressure_controller)
- [Python Control Interface](https://github.com/cbteeple/pressure_control_interface)
- [ROS Driver](https://github.com/cbteeple/pressure_control_cbt)
