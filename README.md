# LSS ROS2 Arms Package

The LSS-ROS2-Arms repository contains common packages that are used by both the physical and simulated LSS Arm.

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

To generathe the SDF file used for the simulation run:
```
bash src/lss_arm_description/scripts/xacro2sdf.bash
```

Similarly, if you want to check the generated URDF file you can run:
```
bash src/lss_arm_description/scripts/xacro2urdf.bash
```

The robot model can be viewed with:

**View Model in Rviz**

```
ros2 launch lss_arm_description view.launch.py
```

**View in Ignition Gazebo**

```
ros2 launch lss_arm_description view_ign.launch.py
```

### MoveIt package

The lss_arm_moveit package contains all the configuration and launch files for using the LSS Arm with the MoveIt2 Motion Planning Framework.

It offers different controller plugins for the manipulator ('fake', 'ign' and 'real')

If you want to check the generated SRDF file you can run:
```
bash src/lss_arm_moveit/scripts/xacro2srdf.bash
```

The following launch files are available:

**Fake controllers (Rviz)**

```
ros2 launch lss_arm_moveit fake_arm_control.launch.py
```

**Simulated controllers (Rviz + Ignition Gazebo Simulation)**

```
ros2 launch lss_arm_moveit ign_arm_control.launch.py
```

**Real controller (RViz + Real Robot)**

```
ros2 launch lss_arm_moveit real_arm_control.launch.py
```

### LSS Ignition MoveIt Example

The lss_ign_moveit_example package contains an example of a C++ implementation to follow a target. This is simulated in Gazebo Ignition, the target (box) can be moved in the world and the Arm will move to the desired position.

**Follow Goal Demo (Simulation)**

```
ros2 launch lss_ign_moveit_example ex_cpp_follow_target.launch.py
```

* Note: The default arm only has 4 DoF so it is not always possible to reach the desired position+orientation goal. This implementation adjusts the goal orientation so it is always parallel to the base of the robot, this allows it to plan a trajectory "ignoring" the orientation.

## Author

- [Geraldine Barreto](http://github.com/geraldinebc)

## Resources

Read more about the LSS Robotic Arm in the [wiki](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-v2-arms/).

Purchase the LSS arm on [RobotShop](https://www.robotshop.com/en/lynxmotion-smart-servos-articulated-arm.html).

Official Lynxmotion Smart Servo (LSS) Hardware Interface available [here](https://github.com/Lynxmotion/LSS-ROS2-Control).

If you want more details about the LSS protocol, go [here](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/).

Have any questions? Ask them on the Robotshop [Community](https://www.robotshop.com/community/).