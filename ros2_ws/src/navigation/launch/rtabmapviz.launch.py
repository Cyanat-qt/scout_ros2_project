from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': 'base_footprint',
                'use_sim_time': use_sim_time,
                'subscribe_rgb': True,
                'subscribe_scan': True,
            }],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('scan', '/scan'),
            ],
        ),
    ])
