from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('robot_name', default_value='turtlebot4',
                          description='Robot name'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='World name'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('arm_model', default_value='tb4_lss_arm',
                          choices=['lss_arm', 'tb4_lss_arm'],
                          description='Turtlebot4 + LSS Arm Model'),
]


def generate_launch_description():
    # leds = [
    #     'power',
    #     'motors',
    #     'comms',
    #     'wifi',
    #     'battery',
    #     'user1',
    #     'user2'
    # ]

    # cliff_sensors = [
    #     'cliff_front_left',
    #     'cliff_front_right',
    #     'cliff_side_left',
    #     'cliff_side_right',
    # ]

    # ir_intensity_sensors = [
    #     'ir_intensity_front_center_left',
    #     'ir_intensity_front_center_right',
    #     'ir_intensity_front_left',
    #     'ir_intensity_front_right',
    #     'ir_intensity_left',
    #     'ir_intensity_right',
    #     'ir_intensity_side_left',
    # ]

    namespace = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_tb4_lss_ignition_bringup = get_package_share_directory(
        'tb4_lss_ignition_bringup')
    
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')

    turtlebot4_ros_ign_bridge_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ros_ign_bridge.launch.py'])
   
    turtlebot4_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot4_ros_ign_bridge_launch]),
        launch_arguments=[
            ('robot_name', 'turtlebot4'),
            ('world', LaunchConfiguration('world'))
        ]
    )

    # # Buttons message bridge
    # buttons_msg_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
    #                           namespace=namespace,
    #                           name='buttons_msg_bridge',
    #                           output='screen',
    #                           parameters=[{
    #                                'use_sim_time': use_sim_time
    #                           }],
    #                           arguments=[
    #                               ['/create3/buttons' +
    #                                '@std_msgs/msg/Int32' +
    #                                '[ignition.msgs.Int32']
    #                           ])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(turtlebot4_bridge)
    # ld.add_action(buttons_msg_bridge)
    return ld
