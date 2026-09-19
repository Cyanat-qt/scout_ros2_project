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
    navigation_share = get_package_share_directory('navigation')
    sim_share = get_package_share_directory('mecanum_robot_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    recognition_routes_file = LaunchConfiguration('recognition_routes_file')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    route_commander = LaunchConfiguration('route_commander')
    nav2_route_commander = LaunchConfiguration('nav2_route_commander')
    start_sim = LaunchConfiguration('start_sim')
    teleop = LaunchConfiguration('teleop')
    world_name = LaunchConfiguration('world_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    map_to_odom_x = LaunchConfiguration('map_to_odom_x')
    map_to_odom_y = LaunchConfiguration('map_to_odom_y')
    map_to_odom_yaw = LaunchConfiguration('map_to_odom_yaw')
    controller = LaunchConfiguration('controller')
    use_teb = LaunchConfiguration('use_teb')
    enable_amcl = LaunchConfiguration('enable_amcl')
    nav_linear_speed = LaunchConfiguration('nav_linear_speed')
    nav_angular_speed = LaunchConfiguration('nav_angular_speed')
    nav_reverse_speed = LaunchConfiguration('nav_reverse_speed')
    enable_yolo_detector = LaunchConfiguration('enable_yolo_detector')

    default_map = '/home/ubuntu/raicom/maps/my_map.yaml'

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

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'include', 'bringup_include.launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'enable_amcl': enable_amcl,
            'controller': controller,
            'use_teb': use_teb,
            'nav_linear_speed': nav_linear_speed,
            'nav_angular_speed': nav_angular_speed,
            'nav_reverse_speed': nav_reverse_speed,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_navigation',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    route_commander_node = Node(
        package='navigation',
        executable='recognition_line_commander.py',
        name='recognition_line_commander',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'routes_file': recognition_routes_file,
            'route_id_topic': '/recognition_route_id',
            'route_status_topic': '/recognition_route_status',
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odom',
            'scan_topic': '/scan_raw',
            'path_topic': '/recognition_line_path',
            'map_to_odom_x': map_to_odom_x,
            'map_to_odom_y': map_to_odom_y,
            'map_to_odom_yaw': map_to_odom_yaw,
            'max_linear_speed': nav_linear_speed,
            'max_angular_speed': nav_angular_speed,
            'service_prefix': '/go_recognition_',
            'cancel_service': '/cancel_recognition_route',
            'min_linear_speed': 0.12,
            'linear_gain': 1.25,
            'heading_gain': 3.2,
            'cross_track_gain': 1.25,
            'max_linear_accel': 1.7,
            'max_linear_decel': 2.8,
            'max_angular_accel': 4.5,
            'slowdown_distance': 0.48,
            'corner_slowdown_speed': 0.34,
            'pre_rotate_angle': 0.65,
            'obstacle_stop_distance': 0.14,
            'obstacle_slow_distance': 0.28,
            'side_keep_distance': 0.16,
            'blocked_timeout_sec': 0.60,
            'scan_timeout_sec': 0.8,
            'use_graph_planner': True,
        }],
        condition=IfCondition(route_commander),
    )

    nav2_route_commander_node = Node(
        package='navigation',
        executable='recognition_route_commander.py',
        name='recognition_route_commander',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'routes_file': recognition_routes_file,
            'map_yaml': map_file,
            'route_id_topic': '/recognition_route_id',
            'route_status_topic': '/recognition_route_status',
            'nav_action_name': '/navigate_through_poses',
            'pose_nav_action_name': '/navigate_to_pose',
            'strict_follow_waypoints': False,
            'service_prefix': '/go_recognition_',
            'cancel_service': '/cancel_recognition_route',
            'cancel_on_new_goal': True,
            'retry_on_abort': True,
            'max_retries': 2,
            'retry_delay_sec': 1.0,
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odom',
            'map_to_odom_x': map_to_odom_x,
            'map_to_odom_y': map_to_odom_y,
            'map_to_odom_yaw': map_to_odom_yaw,
            'heading_align_yaw_tolerance': 0.12,
            'alignment_timeout_sec': 6.0,
            'alignment_angular_gain': 1.4,
            'alignment_min_angular_speed': 0.12,
            'alignment_max_angular_speed': nav_angular_speed,
        }],
        condition=IfCondition(nav2_route_commander),
    )

    yolo_detector_node = Node(
        package='navigation',
        executable='yolo_camera_detector.py',
        name='yolo_camera_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': '/camera/image_raw',
            'yolo_url': LaunchConfiguration('yolo_url'),
            'conf_threshold': LaunchConfiguration('yolo_conf'),
        }],
        condition=IfCondition(enable_yolo_detector),
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('route_commander', default_value='false'),
        DeclareLaunchArgument('nav2_route_commander', default_value='true'),
        DeclareLaunchArgument('start_sim', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('world_name', default_value='raicom_intelligent_recon'),
        DeclareLaunchArgument('spawn_x', default_value='0.2695579528808594'),
        DeclareLaunchArgument('spawn_y', default_value='3.7155404090881348'),
        DeclareLaunchArgument('spawn_z', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='-1.5708'),
        DeclareLaunchArgument('map_to_odom_x', default_value='0.0'),
        DeclareLaunchArgument('map_to_odom_y', default_value='0.0'),
        DeclareLaunchArgument('map_to_odom_yaw', default_value='0.0'),
        DeclareLaunchArgument('controller', default_value='rpp'),
        DeclareLaunchArgument('use_teb', default_value='false'),
        DeclareLaunchArgument('enable_amcl', default_value='false'),
        DeclareLaunchArgument('enable_yolo_detector', default_value='true'),
        DeclareLaunchArgument(
            'yolo_url',
            default_value='http://localhost:8765/detect',
            description='YOLO HTTP service URL (host machine)',
        ),
        DeclareLaunchArgument('yolo_conf', default_value='0.15', description='Detection confidence threshold'),
        DeclareLaunchArgument(
            'nav_linear_speed',
            default_value='0.35',
            description='Navigation forward speed in m/s.',
        ),
        DeclareLaunchArgument(
            'nav_angular_speed',
            default_value='0.80',
            description='Navigation turning speed in rad/s.',
        ),
        DeclareLaunchArgument(
            'nav_reverse_speed',
            default_value='0.10',
            description='Navigation reverse speed limit in m/s.',
        ),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(navigation_share, 'config', 'nav2_params.yaml'),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(navigation_share, 'rviz', 'navigation.rviz'),
        ),
        DeclareLaunchArgument(
            'recognition_routes_file',
            default_value=os.path.join(navigation_share, 'config', 'recognition_routes.yaml'),
        ),
        gazebo_launch,
        odom_tf,
        map_to_odom_tf,
        TimerAction(period=8.0, actions=[scan_frame]),
        TimerAction(period=4.0, actions=[nav2_bringup]),
        TimerAction(period=10.0, actions=[route_commander_node, nav2_route_commander_node]),
        TimerAction(period=10.5, actions=[yolo_detector_node]),
        TimerAction(period=12.0, actions=[rviz_node]),
    ])
