from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='lite',
                          choices=['standard', 'lite'],
                          description='TB4 + LSS Arm Model')
]


def generate_launch_description():

    # Directories
    # pkg_turtlebot4_ignition_bringup = get_package_share_directory('turtlebot4_ignition_bringup')
    pkg_tb4_lss_ignition_bringup = get_package_share_directory('tb4_lss_ignition_bringup')

    # # Parameters
    # param_file_cmd = DeclareLaunchArgument(
    #     'param_file',
    #     default_value=PathJoinSubstitution(
    #         [pkg_turtlebot4_ignition_bringup, 'config', 'turtlebot4_node.yaml']),
    #     description='Turtlebot4 Robot param file'
    # )

    # turtlebot4_node_yaml_file = LaunchConfiguration('param_file')

    # # Turtlebot4 node
    # turtlebot4_node = Node(
    #     package='turtlebot4_node',
    #     name='turtlebot4_node',
    #     executable='turtlebot4_node',
    #     parameters=[turtlebot4_node_yaml_file,
    #                 {'model': LaunchConfiguration('model')}],
    #     output='screen',
    # )

    # # Turtlebot4 Ignition Hmi node
    # turtlebot4_ignition_hmi_node = Node(
    #     package='turtlebot4_ignition_toolbox',
    #     name='turtlebot4_ignition_hmi_node',
    #     executable='turtlebot4_ignition_hmi_node',
    #     output='screen',
    #     condition=LaunchConfigurationEquals('model', 'standard')
    # )

    # # Define LaunchDescription variable
    # ld = LaunchDescription(ARGUMENTS)
    # ld.add_action(param_file_cmd)
    # ld.add_action(turtlebot4_node)
    # ld.add_action(turtlebot4_ignition_hmi_node)
    return ld
