# LSS ROS2 Arms Package

The LSS-ROS2-Arms repository contains common packages that are used by both the physical and simulated LSS Arms (4DoF and 5DoF versions).

## Table of Contents

- [Prerequisites](#prerequisites)
- [Package installation](#package-installation)
- [Description package](#description-package)
- [MoveIt2 package](#moveit2-package)
- [Follow goal example](#lss-ignition-moveit-example)
- [Author](#author)
- [Resources](#resources)

## Prerequisites

1. [Ubuntu 20.04.6 (Focal Fossa)](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
2. [ROS2 Galactic (Desktop)](https://docs.ros.org/en/galactic/Installation.html)
3. ROS2 dev tools:

```
sudo pip install colcon-common-extensions
sudo pip install vcstool
sudo apt install ros-galactic-backward-ros
sudo apt install python3-rosdep2
rosdep update --include-eol-distros
```
4. [Ignition Edifice](https://gazebosim.org/docs/edifice/install)

## Package installation

```
git clone https://github.com/Lynxmotion/LSS-ROS2-Arms.git
mkdir -p src
mv LSS-ROS2-Arms/* src
mv src LSS-ROS2-Arms
```

### Install dependencies

```
cd LSS-ROS2-Arms
rosdep install --from-path src -yi --rosdistro galactic
cd src
vcs import < required.repos
cd ..
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

The lss_arm_description package contains the [URDF](http://wiki.ros.org/urdf/XML/model) description & [SDF](https://classic.gazebosim.org/tutorials?tut=build_model) model of the robot and the mesh files for each component.

It contains scripts that convert [xacro](https://github.com/ros/xacro) into the required URDF and SDF files.

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
<p align="center">
  <table align="center" border="0">
    <tr>
      <td align="center">
        <img src="https://github.com/Lynxmotion/LSS-ROS2-Arms/blob/master/images/rviz_view_4dof.png" height="210px"/>
        <br>4DoF
      </td>
      <td align="center">
        <img src="https://github.com/Lynxmotion/LSS-ROS2-Arms/blob/master/images/rviz_view_5dof.png" height="210px"/>
        <br>5DoF
      </td>
    </tr>
  </table>
</p>

<p align="center">
  <img src="https://github.com/Lynxmotion/LSS-ROS2-Arms/blob/master/images/description_5dof.gif" width="600px"/>
</p>

**View in Ignition Gazebo**

```
ros2 launch lss_arm_description view_ign.launch.py dof:=5
```
<p align="center">
  <img src="https://github.com/Lynxmotion/LSS-ROS2-Arms/blob/master/images/sim_ign_5dof.png" width="450px"/>
</p>

### MoveIt2 package

The lss_arm_moveit package contains all the configuration and launch files for using the LSS Arm with the MoveIt2 Motion Planning Framework.

It offers different controller plugins for the manipulator ('fake', 'ign' and 'real')

If you want to generate the [SRDF](https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/urdf_srdf/urdf_srdf_tutorial.html#srdf) files (not required) you can run:
```
bash src/lss_arm_moveit/scripts/xacro2srdf.bash
```
or
```
bash src/lss_arm_description/scripts/xacro2urdf.bash -d 5
```

The following launch files are available:

**Fake controller (Rviz)**

MoveIt allows the use of a fake trajectory controller to be used for visualization purposes. The fake controllers are not designed to interact with a physics engine, so the robot’s motion is not affected by gravity or other physical forces.

```
ros2 launch lss_arm_moveit fake_arm_control.launch.py dof:=4
```
<p align="center">
  <table align="center" border="0">
    <tr>
      <td align="center">
        <img src="https://github.com/Lynxmotion/LSS-ROS2-Arms/blob/master/images/moveit_4dof.png" height="230px"/>
        <br>4DoF
      </td>
      <td align="center">
        <img src="https://github.com/Lynxmotion/LSS-ROS2-Arms/blob/master/images/moveit_5dof.gif" height="230px"/>
        <br>5DoF
      </td>
    </tr>
  </table>
</p>

**Simulated controller (Rviz Interface + Ignition Gazebo Simulation)**

Unlike the "fake controller" the "simulated controller" integrates the robot controllers with Gazebo, allowing a realistic simulation of the robot's motion. This is useful for testing and validating the robot’s behavior in a simulated environment before deploying it in the real world.

```
ros2 launch lss_arm_moveit ign_arm_control.launch.py dof:=4
```
<p align="center">
  <img src="https://github.com/Lynxmotion/LSS-ROS2-Arms/blob/master/images/moveit_ign_4dof.png" width="650px"/>
</p>

```
ros2 launch lss_arm_moveit ign_arm_control.launch.py dof:=5
```
<p align="center">
  <img src="https://github.com/Lynxmotion/LSS-ROS2-Arms/blob/master/images/ign_obstacle_5dof.gif" width="600px"/>
  <br>Obstacle avoidance example
</p>

**Real controller (RViz Interface + Physical Robot)**

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

<p align="center">
  <img src="https://github.com/geraldinebc/tb4_lss_tests/blob/main/images/4dof_real_control.gif" width="600px"/>
</p>

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

3. Now you are able to plan the trajectories using MoveIt2 and execute them with real hardware

<p align="center">
  <img src="https://github.com/geraldinebc/tb4_lss_tests/blob/main/images/5dof_real_control.gif" width="600px"/>
</p>

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

The lss_ign_moveit_example package launches a simulation of the 4DoF LSS Arm in Gazebo Ignition where you can interact with a box in the virtual environment. This example includes a C++ implementation that makes the arm track the target (box) whenever you change its location.

**Follow Goal Demo (Simulation)**

```
ros2 launch lss_ign_moveit_example ex_cpp_follow_target.launch.py
```
<p align="center">
  <img src="https://github.com/Lynxmotion/LSS-ROS2-Arms/blob/master/images/follow_goal_4dof.gif" width="600px"/>
</p>

* Note: The arm only has 4/5 DoF so it is not always possible to reach the desired pose (position + orientation) goal. This implementation uses a position only target when the pose is not reachable.

## Author

- [Geraldine Barreto](http://github.com/geraldinebc)

## Resources

Read more about the LSS Robotic Arm in the [Wiki](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-v2-arms/).

Purchase the LSS arm on [RobotShop](https://www.robotshop.com/collections/lynxmotion-smart-servos-articulated-arm).

Official Lynxmotion Smart Servo (LSS) Hardware Interface available [here](https://github.com/Lynxmotion/LSS-ROS2-Control).

If you want more details about the LSS protocol, go [here](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/lynxmotion-smart-servo/lss-communication-protocol/).

Have any questions? Ask them on the RobotShop [Community](https://community.robotshop.com/forum/c/lynxmotion/electronics-software/27).
