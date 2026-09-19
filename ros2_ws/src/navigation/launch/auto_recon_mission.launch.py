from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument(
            'route_ids',
            default_value='101,102,103',
            description='Comma-separated recognition route IDs to run in sequence.',
        ),
        Node(
            package='navigation',
            executable='route_sequence_runner.py',
            name='route_sequence_runner',
            output='screen',
            parameters=[{
                'route_ids': LaunchConfiguration('route_ids'),
                'service_prefix': '/go_recognition_',
                'route_status_topic': '/recognition_route_status',
                'service_wait_timeout_sec': 2.0,
                'start_delay_sec': 1.0,
            }],
        ),
    ])
