import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('mecanum_robot_sim')
    description_share = get_package_share_directory('mecanum_robot')
    urdf_path = os.path.join(sim_share, 'urdf', 'mecanum_robot_sim.urdf')
    rviz_config_path = os.path.join(description_share, 'rviz', 'config.rviz')

    with open(urdf_path, 'r', encoding='utf-8') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='mecanum_robot_urdf_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'ignore_timestamp': True,
            }],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='mecanum_robot_joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_mecanum_robot_urdf',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
    ])
