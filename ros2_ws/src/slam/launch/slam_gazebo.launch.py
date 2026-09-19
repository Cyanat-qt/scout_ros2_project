import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory('slam')
    sim_share = get_package_share_directory('mecanum_robot_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    teleop = LaunchConfiguration('teleop')
    rviz = LaunchConfiguration('rviz')
    world_name = LaunchConfiguration('world_name')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'keyboard_gazebo.launch.py')
        ),
        launch_arguments={
            'gui': gui,
            'teleop': teleop,
            'world_name': world_name,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    odom_tf = Node(
        package='slam',
        executable='odom_tf_broadcaster.py',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': '/odom',
            'child_frame_id': 'base_footprint',
        }],
    )

    scan_frame = Node(
        package='slam',
        executable='scan_frame_republisher.py',
        name='scan_frame_republisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_scan_topic': '/scan_raw',
            'output_scan_topic': '/scan',
            'odom_topic': '/odom',
            'frame_id': 'lidar_link',
            'publish_rate': 3.0,
            'startup_delay_sec': 1.0,
        }],
    )

    scan_cone = Node(
        package='slam',
        executable='scan_cone_visualizer.py',
        name='scan_cone_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_topic': '/scan',
            'marker_topic': '/scan_cone',
            'max_visual_range': 6.0,
        }],
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(slam_share, 'config', 'slam_toolbox.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(slam_share, 'rviz', 'slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_SHM', '0'),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('world_name', default_value='raicom_intelligent_recon'),
        gazebo_launch,
        odom_tf,
        TimerAction(period=5.0, actions=[scan_frame, scan_cone]),
        TimerAction(period=6.0, actions=[slam_toolbox]),
        TimerAction(period=8.0, actions=[rviz_node]),
    ])
