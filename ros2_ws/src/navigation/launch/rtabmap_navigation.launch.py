import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory('navigation')
    sim_share = get_package_share_directory('mecanum_robot_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    start_sim = LaunchConfiguration('start_sim')
    teleop = LaunchConfiguration('teleop')
    world_name = LaunchConfiguration('world_name')
    controller = LaunchConfiguration('controller')
    use_teb = LaunchConfiguration('use_teb')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'keyboard_gazebo.launch.py')
        ),
        launch_arguments={
            'gui': gui,
            'teleop': teleop,
            'world_name': world_name,
        }.items(),
        condition=IfCondition(start_sim),
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
            'frame_id': 'lidar_link',
            'publish_rate': 5.0,
        }],
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'include', 'rtabmap_include.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'include', 'navigation_base_include.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'controller': controller,
            'use_teb': use_teb,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_rtabmap_navigation',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('start_sim', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('world_name', default_value='raicom_intelligent_recon'),
        DeclareLaunchArgument('controller', default_value='rpp'),
        DeclareLaunchArgument('use_teb', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(navigation_share, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(navigation_share, 'rviz', 'rtabmap.rviz'),
        ),
        gazebo_launch,
        odom_tf,
        scan_frame,
        TimerAction(period=4.0, actions=[rtabmap_launch]),
        TimerAction(period=10.0, actions=[navigation_launch]),
        TimerAction(period=12.0, actions=[rviz_node]),
    ])
