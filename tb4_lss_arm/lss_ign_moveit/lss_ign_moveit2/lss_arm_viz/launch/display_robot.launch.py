from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'),
    DeclareLaunchArgument(
        'description',
        default_value='false',
        description='Launch lss_arm_description'
    ),
    DeclareLaunchArgument(
        'arm_model',
        default_value='tb4_lss_arm',
        choices=['lss_arm', 'tb4_lss_arm'],
        description='Turtlebot4 + LSS Arm model'
    )
]


def generate_launch_description():

    pkg_lss_arm_viz = get_package_share_directory('lss_arm_viz')
    pkg_lss_arm_description = get_package_share_directory('lss_arm_description')

    rviz2_config = PathJoinSubstitution(
        [pkg_lss_arm_viz, 'rviz', 'robot.rviz'])
    description_launch = PathJoinSubstitution(
        [pkg_lss_arm_description, 'launch', 'robot_description.launch.py']
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen')

    # Delay launch of robot description to allow Rviz2 to load first.
    # Prevents visual bugs in the model.
    robot_description = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([description_launch]),
                launch_arguments=[('arm_model', LaunchConfiguration('arm_model'))],
                condition=IfCondition(LaunchConfiguration('description'))
            )]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_description)
    ld.add_action(rviz2)
    return ld
