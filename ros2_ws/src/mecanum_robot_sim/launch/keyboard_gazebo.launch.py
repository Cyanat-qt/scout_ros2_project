import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def append_resource_path(env_name, *paths):
    existing_path = os.environ.get(env_name, '')
    resource_path = os.pathsep.join(path for path in paths if path)
    if existing_path:
        resource_path = resource_path + os.pathsep + existing_path
    return SetEnvironmentVariable(env_name, resource_path)


def find_first_package(package_names):
    for package_name in package_names:
        try:
            return package_name, get_package_share_directory(package_name)
        except PackageNotFoundError:
            pass
    return None, None


def gazebo_actions(world_path, gui):
    # Launch Gazebo directly so ros2 launch owns the actual server process.
    # The ros_gz_sim wrapper can leave an ign gazebo child process alive after Ctrl+C.
    return [
        ExecuteProcess(
            cmd=['ign', 'gazebo', '--force-version', '6', '-r', '-v', '4', world_path],
            output='screen',
            condition=IfCondition(gui),
        ),
        ExecuteProcess(
            cmd=['ign', 'gazebo', '--force-version', '6', '-s', '-r', '-v', '4', world_path],
            output='screen',
            condition=UnlessCondition(gui),
        ),
    ]


def generate_launch_description():
    description_share = get_package_share_directory('mecanum_robot')
    sim_share = get_package_share_directory('mecanum_robot_sim')
    repo_root = os.path.abspath(
        os.path.join(description_share, '..', '..', '..', '..', '..')
    )
    arena_generator = os.path.join(repo_root, 'scripts', 'generate_recon_arena.py')
    create_package, _ = find_first_package(('ros_gz_sim', 'ros_ign_gazebo'))
    bridge_package, _ = find_first_package(('ros_gz_bridge', 'ros_ign_bridge'))

    if create_package is None or bridge_package is None:
        missing_packages = []
        if create_package is None:
            missing_packages.append('ros_gz_sim')
        if bridge_package is None:
            missing_packages.append('ros_gz_bridge')
        missing_text = ', '.join(missing_packages)
        install_text = 'sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge'
        return LaunchDescription([
            LogInfo(msg=f'ERROR: missing Ignition ROS 2 package(s): {missing_text}.'),
            LogInfo(msg=f'Install them with: {install_text}'),
            LogInfo(msg='Then rebuild/source and run this launch file again.'),
            EmitEvent(event=Shutdown(reason='Ignition ROS 2 integration package is missing')),
        ])

    if bridge_package == 'ros_gz_bridge':
        gazebo_msg_prefix = 'gz.msgs'
    else:
        gazebo_msg_prefix = 'ignition.msgs'

    try:
        get_package_share_directory(bridge_package)
    except PackageNotFoundError:
        return LaunchDescription([
            LogInfo(msg='ERROR: Ignition bridge is not installed, so /cmd_vel cannot reach Gazebo.'),
            LogInfo(msg='Install it with: sudo apt install ros-humble-ros-ign-bridge'),
            LogInfo(msg='Then rebuild/source and run this launch file again.'),
            EmitEvent(event=Shutdown(reason='Ignition bridge package is missing')),
        ])

    world_path = PathJoinSubstitution([
        description_share,
        'worlds',
        LaunchConfiguration('world_file'),
    ])
    urdf_path = os.path.join(sim_share, 'urdf', 'mecanum_robot_sim.urdf')

    with open(urdf_path, 'r', encoding='utf-8') as infp:
        robot_description = infp.read()

    # Gazebo is more reliable with absolute file URIs than package:// mesh URIs.
    robot_description = robot_description.replace(
        'package://mecanum_robot/',
        'file://' + description_share + '/',
    )

    model_path = os.path.join(description_share, 'models')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    actions = [
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Ignition Gazebo with GUI.',
        ),
        DeclareLaunchArgument(
            'teleop',
            default_value='true',
            description='Start keyboard teleoperation in this terminal.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock for robot TF.',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='raicom_intelligent_recon',
            description='Gazebo world name to spawn the robot into.',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='raicom_intelligent_recon.world',
            description='Gazebo world file under mecanum_robot/worlds.',
        ),
        DeclareLaunchArgument(
            'generate_arena',
            default_value='true',
            description='Regenerate the RAICOM arena SDF before launching Gazebo.',
        ),
        DeclareLaunchArgument(
            'spawn_x',
            default_value='0.25',
            description='Initial robot X position in the arena.',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='3.10',
            description='Initial robot Y position in the arena.',
        ),
        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.0',
            description='Initial robot Z position in the arena.',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='-1.5708',
            description='Initial robot yaw angle in radians.',
        ),
        append_resource_path('IGN_GAZEBO_RESOURCE_PATH', model_path, description_share),
        append_resource_path('GZ_SIM_RESOURCE_PATH', model_path, description_share),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('IGN_PARTITION', 'raicom_intelligent_recon'),
        SetEnvironmentVariable('GZ_PARTITION', 'raicom_intelligent_recon'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_SHM', '0'),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        ExecuteProcess(
            cmd=['python3', arena_generator],
            output='screen',
            condition=IfCondition(LaunchConfiguration('generate_arena')),
        ),
        *gazebo_actions(world_path, gui),
        Node(
            package='mecanum_robot_sim',
            executable='clock_republisher.py',
            name='clock_republisher',
            output='screen',
            parameters=[{
                'input_topic': '/clock_raw',
                'output_topic': '/clock',
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='mecanum_robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
                'ignore_timestamp': True,
            }],
            remappings=[
                ('robot_description', '/mecanum_robot/robot_description'),
                ('/robot_description', '/mecanum_robot/robot_description'),
                ('joint_states', '/joint_states_clean'),
                ('/joint_states', '/joint_states_clean'),
            ],
        ),
        Node(
            package='mecanum_robot_sim',
            executable='joint_state_republisher.py',
            name='joint_state_republisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/joint_states_raw',
                'output_topic': '/joint_states_clean',
            }],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package=create_package,
                    executable='create',
                    name='spawn_mecanum_robot',
                    output='screen',
                    arguments=[
                        '-world', LaunchConfiguration('world_name'),
                        '-file', urdf_path,
                        '-name', 'mecanum_robot',
                        '-x', LaunchConfiguration('spawn_x'),
                        '-y', LaunchConfiguration('spawn_y'),
                        '-z', LaunchConfiguration('spawn_z'),
                        '-Y', LaunchConfiguration('spawn_yaw'),
                    ],
                ),
            ],
        ),
        Node(
            package='mecanum_robot_sim',
            executable='collision_safety_node.py',
            name='collision_safety_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'cmd_vel_in': '/cmd_vel',
                'cmd_vel_out': '/cmd_vel_safe',
                'scan_topic': '/scan_raw',
                'front_angle_deg': 35.0,
                'side_sector_start_deg': 55.0,
                'side_sector_end_deg': 125.0,
                'stop_distance': 0.095,
                'release_distance': 0.16,
                'side_stop_distance': 0.10,
                'side_release_distance': 0.15,
                'side_emergency_distance': 0.05,
                'side_emergency_release_distance': 0.08,
                'corridor_front_clear_distance': 0.22,
                'corridor_balance_distance': 0.45,
                'corridor_balance_deadband': 0.02,
                'corridor_centering_gain': 1.2,
                'max_centering_angular_bias': 0.10,
                'creep_speed': 0.18,
                'side_creep_speed': 0.10,
                'turning_drive_speed': 0.06,
                'max_angular_speed': 1.8,
                'backup_speed': -0.14,
                'backup_duration': 0.22,
                'unstick_turn_speed': 0.90,
                'wall_escape_turn_speed': 0.45,
                'unstick_toggle_period': 0.60,
            }],
        ),
        Node(
            package=bridge_package,
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            output='screen',
            arguments=[
                f'/model/mecanum_robot/cmd_vel@geometry_msgs/msg/Twist]{gazebo_msg_prefix}.Twist',
                f'/odom@nav_msgs/msg/Odometry[{gazebo_msg_prefix}.Odometry',
                f'/scan_raw@sensor_msgs/msg/LaserScan[{gazebo_msg_prefix}.LaserScan',
                f'/camera@sensor_msgs/msg/Image[{gazebo_msg_prefix}.Image',
                f'/camera_info@sensor_msgs/msg/CameraInfo[{gazebo_msg_prefix}.CameraInfo',
                f'/joint_states@sensor_msgs/msg/JointState[{gazebo_msg_prefix}.Model',
                f'/world/raicom_intelligent_recon/clock@rosgraph_msgs/msg/Clock[{gazebo_msg_prefix}.Clock',
            ],
            remappings=[
                ('/model/mecanum_robot/cmd_vel', '/cmd_vel_safe'),
                ('/camera', '/camera/image_raw'),
                ('/camera_info', '/camera/camera_info'),
                ('/joint_states', '/joint_states_raw'),
                ('/world/raicom_intelligent_recon/clock', '/clock_raw'),
            ],
        ),
        Node(
            package='mecanum_robot_sim',
            executable='keyboard_teleop.py',
            name='keyboard_teleop',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('teleop')),
        ),
    ]

    return LaunchDescription(actions)
