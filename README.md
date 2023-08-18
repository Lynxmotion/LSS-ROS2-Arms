# LSS ROS2 Arms Package

The LSS-ROS2-Arms repository contains common packages that are used by both the physical and simulated LSS Arms (4DoF and 5DoF versions).

## Table of Contents

- [Prerequisites](#prerequisites)
- [Package installation](#package-installation)
- [Description package](#description-package)
- [MoveIt package](#moveit-package)
- [Follow goal example](#lss-ignition-moveit-example)
- [Author](#author)
- [Resources](#resources)

## Prerequisites

1. [ROS 2 Galactic](https://docs.ros.org/en/galactic/Installation.html)
2. ROS 2 dev tools:
    - [colcon-common-extensions](https://pypi.org/project/colcon-common-extensions/)
    - [rosdep](https://pypi.org/project/rosdep/): Used to install dependencies when building from sources
    - [vcs](https://pypi.org/project/vcstool/): Automates cloning of git repositories declared on a YAML file.
3. Ignition Gazebo
```
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ignition-edifice
```

## Package installation

```
mkdir -p ~/LSS-ROS2-Arms/src
cd ~/LSS-ROS2-Arms/src
git clone https://github.com/Lynxmotion/LSS-ROS2-Arms.git
```

### Install dependencies

```
vcs import < required.repos
cd ~/LSS-ROS2-Arms
rosdep install --from-path src -yi --rosdistro galactic
```

### Build instructions

```
source /opt/ros/galactic/setup.bash
export IGNITION_VERSION=edifice
colcon build --symlink-install
```

### Initialization instructions

You will have to use this in every new session in which you wish to use these packages:

```
source install/setup.bash
```

## Description package

The lss_arm_description package contains the URDF description & SDF model of the robot and the mesh files for each component.

It contains scripts that convert xacro into the required URDF and SDF files.

To generate the **required** SDF file used for the simulation run:
```
bash src/lss_arm_description/scripts/xacro2sdf.bash
```

The script defaults to the 4DoF version. To generate the SDF of the 5DoF model run:
```
bash src/lss_arm_description/scripts/xacro2sdf.bash -d 5
```

Similarly, if you want to generate the URDF files (not required) you can run:
```
bash src/lss_arm_description/scripts/xacro2urdf.bash
```
or
```
bash src/lss_arm_description/scripts/xacro2urdf.bash -d 5
```
All the available launch files have the following configuration options:

**dof**: Which LSS Arm version to use
- options: 4, 5
- default: 4

**View Model in Rviz**

```
ros2 launch lss_arm_description view.launch.py dof:=4
```

**View in Ignition Gazebo**

```
ros2 launch lss_arm_description view_ign.launch.py dof:=4
```

### MoveIt package

The lss_arm_moveit package contains all the configuration and launch files for using the LSS Arm with the MoveIt2 Motion Planning Framework.

It offers different controller plugins for the manipulator ('fake', 'ign' and 'real')

If you want to generate the SRDF files (not required) you can run:
```
bash src/lss_arm_moveit/scripts/xacro2srdf.bash
```
or
```
bash src/lss_arm_description/scripts/xacro2urdf.bash -d 5
```

The following launch files are available:

**Fake controllers (Rviz)**

```
ros2 launch lss_arm_moveit fake_arm_control.launch.py dof:=4
```

**Simulated controllers (Rviz + Ignition Gazebo Simulation)**

```
ros2 launch lss_arm_moveit ign_arm_control.launch.py dof:=4
```

**Real controller (RViz + Real Robot)**

Before controlling the real robot first follow these steps:

1. Update the firmware on the servos using [LSS Config](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-configuration-software/)

2. Follow the [initial setup](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-software/lss-flowarm/?#HInitialSetup) and make sure the IDs are configured correctly

3. Calibrate the arm using [FlowArm](https://www.robotshop.com/products/lynxmotion-lss-lss-flowarm-app-download)

4. Set the *Baud rate* to 921600 and the *Gyre Direction* to CCW (-1) for all the servos

To control the arm:

1. Run the launch file

```
ros2 launch lss_arm_moveit real_arm_control.launch.py dof:=4
```

* Note: If the servos light up *Blue* they have been configured correctly if not try running:
```
sudo chmod 766 /dev/ttyUSB0
```

2. To activate the servos open another terminal and run:
```
ros2 topic pub --once /effort_controller/commands std_msgs/msg/Float64MultiArray "data:
- 6.8
- 6.8
- 6.8
- 6.8
- 6.8"
```
* Note: For the 5DoF version add an extra - 6.8

3. Now you are able to plan the trajectories using MoveIt2 and executing them with real hardware

To make the servos go Limp use:
```
ros2 topic pub --once /effort_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0
- 0
- 0
- 0
- 0"
```
* Note: For the 5DoF version add an extra - 0

### LSS Ignition MoveIt Example

The lss_ign_moveit_example package contains an example of a C++ implementation to follow a target. This is simulated in Gazebo Ignition, the target (box) can be moved in the world and the Arm will move to the desired position. This example is only available for the 4 DoF version.

**Follow Goal Demo (Simulation)**

```
ros2 launch lss_ign_moveit_example ex_cpp_follow_target.launch.py
```

* Note: The arm only has 4 DoF so it is not always possible to reach the desired position + orientation goal. This implementation adjusts the goal orientation so it is always parallel to the base of the robot, this allows it to plan a trajectory "ignoring" the orientation.

## Author

- [Geraldine Barreto](http://github.com/geraldinebc)

## Resources

Read more about the LSS Robotic Arm in the [Wiki](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-v2-arms/).

Purchase the LSS arm on [RobotShop](https://www.robotshop.com/collections/lynxmotion-smart-servos-articulated-arm).

Official Lynxmotion Smart Servo (LSS) Hardware Interface available [here](https://github.com/Lynxmotion/LSS-ROS2-Control).

If you want more details about the LSS protocol, go [here](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/).

Have any questions? Ask them on the RobotShop [Community](https://community.robotshop.com/forum/c/lynxmotion/electronics-software/27).
