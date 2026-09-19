import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory('navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(navigation_share, 'rviz', 'rtabmap.rviz'),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_rtabmap_navigation',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
