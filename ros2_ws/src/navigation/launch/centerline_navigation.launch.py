import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
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
    teleop = LaunchConfiguration('teleop')
    start_sim = LaunchConfiguration('start_sim')
    world_name = LaunchConfiguration('world_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    map_to_odom_x = LaunchConfiguration('map_to_odom_x')
    map_to_odom_y = LaunchConfiguration('map_to_odom_y')
    map_to_odom_yaw = LaunchConfiguration('map_to_odom_yaw')
    track_file = LaunchConfiguration('track_file')
    auto_start = LaunchConfiguration('auto_start')
    controller_start_delay = LaunchConfiguration('controller_start_delay')
    delayed_start = LaunchConfiguration('delayed_start')
    motion_start_delay = LaunchConfiguration('motion_start_delay')
    enable_yolo_detector = LaunchConfiguration('enable_yolo_detector')
    yolo_url = LaunchConfiguration('yolo_url')
    yolo_conf = LaunchConfiguration('yolo_conf')
    yolo_timeout = LaunchConfiguration('yolo_timeout')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_start_delay = LaunchConfiguration('rviz_start_delay')
    map_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'keyboard_gazebo.launch.py')
        ),
        launch_arguments={
            'gui': gui,
            'teleop': teleop,
            'world_name': world_name,
            'use_sim_time': use_sim_time,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'spawn_yaw': spawn_yaw,
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
            'odom_topic': '/odom',
            'frame_id': 'lidar_link',
            'publish_rate': 5.0,
            'startup_delay_sec': 1.0,
        }],
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        output='screen',
        arguments=[
            '--x', map_to_odom_x,
            '--y', map_to_odom_y,
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', map_to_odom_yaw,
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file,
        }],
    )

    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_centerline_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server'],
        }],
    )

    centerline_controller = Node(
        package='navigation',
        executable='orthogonal_centerline_controller.py',
        name='orthogonal_centerline_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'track_file': track_file,
            'cmd_vel_topic': '/cmd_vel',
            'auto_start': auto_start,
        }],
    )

    yolo_detector_node = Node(
        package='navigation',
        executable='yolo_camera_detector.py',
        name='yolo_camera_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': '/camera/image_raw',
            'active_on_start': False,
            'yolo_url': yolo_url,
            'conf_threshold': yolo_conf,
            'frame_stride': 1,
            'print_period_sec': 1.0,
            'timeout_sec': yolo_timeout,
            'debug_image_width': 480,
        }],
        condition=IfCondition(enable_yolo_detector),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_centerline_navigation',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    delayed_start_command = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/centerline_track/start', 'std_srvs/srv/Trigger', '{}'],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('start_sim', default_value='true'),
        DeclareLaunchArgument('world_name', default_value='raicom_intelligent_recon'),
        DeclareLaunchArgument('spawn_x', default_value='0.2695579528808594'),
        DeclareLaunchArgument('spawn_y', default_value='3.7155404090881348'),
        DeclareLaunchArgument('spawn_z', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='-1.5708'),
        DeclareLaunchArgument('map_to_odom_x', default_value='0.0'),
        DeclareLaunchArgument('map_to_odom_y', default_value='0.0'),
        DeclareLaunchArgument('map_to_odom_yaw', default_value='0.0'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('controller_start_delay', default_value='10.0'),
        DeclareLaunchArgument('delayed_start', default_value='false'),
        DeclareLaunchArgument('motion_start_delay', default_value='10.0'),
        DeclareLaunchArgument('enable_yolo_detector', default_value='true'),
        DeclareLaunchArgument('yolo_url', default_value='http://localhost:8765/detect'),
        DeclareLaunchArgument('yolo_conf', default_value='0.25'),
        DeclareLaunchArgument('yolo_timeout', default_value='5.0'),
        DeclareLaunchArgument(
            'track_file',
            default_value=os.path.join(navigation_share, 'config', 'centerline_track.yaml'),
        ),
        DeclareLaunchArgument('map', default_value='/ws/map01/compitation.yaml'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(navigation_share, 'rviz', 'centerline_light.rviz'),
        ),
        DeclareLaunchArgument('rviz_start_delay', default_value='12.0'),
        gazebo_launch,
        odom_tf,
        map_to_odom_tf,
        TimerAction(period=4.0, actions=[map_server, map_lifecycle]),
        TimerAction(period=8.0, actions=[scan_frame]),
        TimerAction(period=controller_start_delay, actions=[yolo_detector_node, centerline_controller]),
        TimerAction(period=motion_start_delay, actions=[delayed_start_command], condition=IfCondition(delayed_start)),
        TimerAction(period=rviz_start_delay, actions=[rviz_node]),
    ])
