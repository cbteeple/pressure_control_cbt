# Ctrl-P: ROS Driver
A module to do pressure control in ROS with a Ctrl-P pressure control system.

## Versions
You can always keep up to date by using the version in the master branch. For previous stable versions, checkout the [releases](https://github.com/cbteeple/pressure_control_cbt/releases).

## Dependencies
### Hardware
- A pressure control system running my [Ctrl-P firmware](https://github.com/cbteeple/pressure_controller)
- A desktop computer running Linux (currently tested only in [Ubuntu 18.04](https://ubuntu.com/download/desktop))

### Software
- [ROS Melodic](http://wiki.ros.org/melodic/Installation)
- The [rqt_multiplot](http://wiki.ros.org/rqt_multiplot) package for nice plot layouts and custom axes
- [Cython HID Library](https://github.com/trezor/cython-hidapi) from Trezor:
- The [Python Control Interface](https://github.com/cbteeple/pressure_control_interface) package for this project
- Various python libraries:
	- All python dependencies are managed in the reqirements file. `pip install -r requirements.txt`


## Installation
1. Clone the [Python Control Interface](https://github.com/cbteeple/pressure_control_interface) package to the `src` folder of your catkin workspace.
2. Clone this package to the `src` folder of your catkin workspace.
3. Install cython-hid dependencies (as specified in the Cython HID Library)
	``` bash
	sudo apt-get install python-dev libusb-1.0-0-dev libudev-dev
	sudo pip install --upgrade setuptools
	```
4. In the root folder of this package, run `pip install -r requirements.txt` to install python dependencies.
5. In the root folder of your catkin workspace, run `catkin_make` to enable the custom python modules in this package to work.


## Usage
This driver is split into two ROS packages:
- **pressure_trajectory_ros** is where trajectories are set up, built, and stored for execution
- **pressure_control_ros** is the main driver that actually interfaces with the pressure control hardware

[See the Documentation](https://ctrl-p.cbteeple.com/ros-driver)

## About Ctrl-P
The Ctrl-P project is a full-stack pneumatic control system featuring smooth control of pressure at a high bandwidth.

Ctrl-P has three parts:
- [Arduino-Based Firmware](https://github.com/cbteeple/pressure_controller): Contains the low-level control
- [Python Control Interface](https://github.com/cbteeple/pressure_control_interface): Controls pressure via serial comms
- [ROS Driver](https://github.com/cbteeple/pressure_control_cbt): Controls pressure via serial or RawUSB

Related Packages:
- [Pressure Controller Skills](https://github.com/cbteeple/pressure_controller_skills): Build complex parametric skills using straightforward definition files.
