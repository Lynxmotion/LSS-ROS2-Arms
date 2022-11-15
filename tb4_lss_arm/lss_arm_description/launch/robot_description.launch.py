# @author Geraldine Barreto (geraldinebc94@gmail.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.descriptions import ParameterValue

from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('arm_model', default_value='tb4_lss_arm',
                          choices=['lss_arm', 'tb4_lss_arm'],
                          description='Turtlebot4 + LSS Arm model'),
    DeclareLaunchArgument('file', default_value='tb4_lss_arm.urdf.xacro',
                          choices=['lss_arm.urdf.xacro', 'tb4_lss_arm.urdf.xacro'],
                          description='Turtlebot4 + LSS Arm File'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                        choices=['true', 'false'],
                        description='use_sim_time')
]


def generate_launch_description():
    pkg_lss_arm_description = get_package_share_directory('lss_arm_description')
    xacro_file = PathJoinSubstitution([pkg_lss_arm_description,
                                       'urdf',
                                       LaunchConfiguration('arm_model'),
                                       LaunchConfiguration('file')])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': ParameterValue(Command(['xacro', ' ', xacro_file, ' ', 'gazebo:=ignition']), value_type=str)},
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    
    # Add nodes to LaunchDescription
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    return ld
