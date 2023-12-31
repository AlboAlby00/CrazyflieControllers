# CrazyflyControllers

- Tested with Ubuntu 20.04.
- Uses ros2 galactic

## Installation

1. Install ros2 galactic, see [galactic_installation_documentation](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
2. Clone this repository, inclusive recursive submodules 

```s
$ git clone git@github.com:AlboAlby00/CrazyflieControllers.git
$ git submodule update --init --recursive
```

3. Install the dependencies listed in [apt_dependencies](./apt_dependencies.txt)
4. Install pip requirements

## Start commands for controllers

- launch the pid attitude controller:  `ros2 launch crazyflie_controllers attitude_pid_controller.launch.py`
- launch the pid position controller:  `ros2 launch crazyflie_controllers position_pid_controller.launch.py` (it launches both attitude and position controller)
- launch just drone with still-standing proppellers: `ros2 launch crazyflie_ros2_driver crazyflie_ros2_driver.launch.py`

## Start commands for apriltag detection in apartment
- `ros2 launch crazyflie_detectors sim_target_publisher.launch.py`

## Crazyflie 2.1


### motor numbers and spinning orientation

#### m1 - clockwise
![m1](https://github.com/AlboAlby00/CrazyflieControllers/assets/23526716/6d818960-ba34-4e41-8b84-6226d07ec23c)

### Dependencies for Visual Odometry with ORBSLAM3

- build and install the original Orbslam3 and its dependency:
  https://github.com/UZ-SLAMLab/ORB_SLAM3
- clone the ROS2 Orbslam3 wrapper on your ros2 workspace and follow the instructions on the Readme
  https://github.com/AlboAlby00/ORB_SLAM3_ROS2

## Usage

Build via 

```s
$ colcon build
```

This will create an `install` folder. Make sure to source the install:

```s
$ source ./install/setup.bash
```

Launch via 

```s
ros2 launch <crazyflie_<package_name>> <launch_script_name>
```


### Visualization with plotjuggler

To start plotjuggler, run

```s
ros2 run plotjuggler plotjuggler
```

For explanations on how to use plotjuggler, see [plotjuggler.io](https://plotjuggler.io/)

### Control topics with rqt

To start rqt, run

```s
rqt
```

With rqt, values to topics can be published.

## More documentation

- [crazyflie_analysis](./docs/crazyflie_notes.md)
