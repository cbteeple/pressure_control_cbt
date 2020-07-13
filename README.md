# Ctrl-P 2.0: ROS Driver
A module to do pressure control in ROS with a Ctrl-P pressure control system.

## Dependencies
### Hardware
- A pressure control system running my [Ctrl-P firmware](https://github.com/cbteeple/pressure_controller)
- A desktop computer running Linux (currently tested only in [Ubuntu 18.04](https://ubuntu.com/download/desktop))

### Software
- [ROS Melodic](http://wiki.ros.org/melodic/Installation)
- The [rqt_multiplot](http://wiki.ros.org/rqt_multiplot) package for nice plot layouts and custom axes
- [Cython HID Library](https://github.com/trezor/cython-hidapi) from Trezor
- Various python libraries:
	- [scipy](https://www.scipy.org/) (`pip install scipy`)
	- [numpy](https://www.numpy.org/) (`pip install numpy`)
	- [numbers](https://docs.python.org/2/library/numbers.html) (`pip install numbers`)
	- [matplotlib](https://matplotlib.org/) (`pip install matplotlib`)
	- [pynput](https://pypi.org/project/pynput/) (`pip install pynput`)
	- [yaml](https://pyyaml.org/wiki/PyYAMLDocumentation) (`pip install pyyaml`)
	- [colorama](https://pypi.org/project/colorama) (`pip install colorama`)
	- [termcolor](https://pypi.org/project/termcolor) (`pip install termcolor`)

## Installation
1. Add this package to your `workspace/src` folder.
2. Run `catkin_make` to enable the custom python modules in this package to work


## Usage
This driver is split into two ROS packages:
- **pressure_trajectory_ros** is where trajectories are set up, built, and stored for execution
- **pressure_control_ros** is the main driver that actually interfaces with the pressure control hardware

[See the Documentation](https://cbteeple.github.io/pressure_controller_docs/ros-driver)

## About Ctrl-P
The Ctrl-P project is a full-stack pneumatic control system featuring smooth control of pressure at a high bandwidth.

Ctrl-P has three parts:
- [Arduino-Based Firmware](https://github.com/cbteeple/pressure_controller)
- [Python Control Interface](https://github.com/cbteeple/pressure_control_interface)
- [ROS Driver](https://github.com/cbteeple/pressure_control_cbt)
