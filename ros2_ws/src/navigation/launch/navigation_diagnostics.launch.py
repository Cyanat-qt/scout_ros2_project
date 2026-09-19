from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_to_odom_x', default_value='0.0'),
        DeclareLaunchArgument('map_to_odom_y', default_value='0.0'),
        DeclareLaunchArgument('map_to_odom_yaw', default_value='0.0'),
        Node(
            package='navigation',
            executable='navigation_diagnostics.py',
            name='navigation_diagnostics',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_to_odom_x': LaunchConfiguration('map_to_odom_x'),
                'map_to_odom_y': LaunchConfiguration('map_to_odom_y'),
                'map_to_odom_yaw': LaunchConfiguration('map_to_odom_yaw'),
            }],
        ),
    ])
