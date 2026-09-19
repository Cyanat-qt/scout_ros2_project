import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _resolve(context, value):
    if isinstance(value, list):
        return ''.join(_resolve(context, item) for item in value)
    if hasattr(value, 'perform'):
        return value.perform(context)
    return str(value)


def _typed_value(value):
    text = str(value)
    lowered = text.lower()
    if lowered in ('true', 'false'):
        return lowered == 'true'
    try:
        return float(text) if '.' in text else int(text)
    except ValueError:
        return text


def _rewrite_leaf(data, key_name, value):
    if isinstance(data, dict):
        for key, item in data.items():
            if key == key_name:
                data[key] = value
            else:
                _rewrite_leaf(item, key_name, value)
    elif isinstance(data, list):
        for item in data:
            _rewrite_leaf(item, key_name, value)


def _set_path_if_exists(data, path, value):
    keys = path.split('.')
    current = data
    for key in keys[:-1]:
        if isinstance(current, list):
            try:
                current = current[int(key)]
            except (ValueError, IndexError):
                return
        elif isinstance(current, dict) and key in current:
            current = current[key]
        else:
            return

    final_key = keys[-1]
    if isinstance(current, list):
        try:
            current[int(final_key)] = value
        except (ValueError, IndexError):
            return
    elif isinstance(current, dict) and final_key in current:
        current[final_key] = value


def _rewrite_yaml_file(context, source_file, namespace, leaf_rewrites, path_rewrites):
    source_path = _resolve(context, source_file)
    with open(source_path, 'r', encoding='utf-8') as infp:
        data = yaml.safe_load(infp)

    for key, value in leaf_rewrites.items():
        _rewrite_leaf(data, key, _typed_value(_resolve(context, value)))
    for path, value in path_rewrites.items():
        _set_path_if_exists(data, path, _typed_value(_resolve(context, value)))

    namespace_value = _resolve(context, namespace)
    if namespace_value:
        data = {namespace_value: data}

    rewritten = tempfile.NamedTemporaryFile(mode='w', delete=False)
    yaml.safe_dump(data, rewritten)
    rewritten.close()
    return rewritten.name


def launch_setup(context, *args, **kwargs):
    navigation_share = get_package_share_directory('navigation')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    nav_linear_speed = LaunchConfiguration('nav_linear_speed')
    nav_angular_speed = LaunchConfiguration('nav_angular_speed')
    nav_reverse_speed = LaunchConfiguration('nav_reverse_speed')

    controller = LaunchConfiguration('controller').perform(context).lower()
    if _as_bool(LaunchConfiguration('use_teb').perform(context)):
        controller = 'teb'

    if controller == 'teb':
        controller_source = os.path.join(navigation_share, 'config', 'nav2_controller_teb.yaml')
    elif controller == 'mppi':
        controller_source = os.path.join(navigation_share, 'config', 'nav2_controller_mppi.yaml')
    elif controller == 'dwb':
        controller_source = os.path.join(navigation_share, 'config', 'nav2_controller_dwb.yaml')
    else:
        controller_source = params_file

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    nav_params = _rewrite_yaml_file(
        context,
        params_file,
        namespace,
        {
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        },
        {
            'velocity_smoother.ros__parameters.max_velocity.0': nav_linear_speed,
            'velocity_smoother.ros__parameters.max_velocity.2': nav_angular_speed,
            'velocity_smoother.ros__parameters.min_velocity.0': ['-', nav_reverse_speed],
            'velocity_smoother.ros__parameters.min_velocity.2': ['-', nav_angular_speed],
        },
    )

    controller_params = _rewrite_yaml_file(
        context,
        controller_source,
        namespace,
        {'use_sim_time': use_sim_time},
        {
            'controller_server.ros__parameters.FollowPath.desired_linear_vel': nav_linear_speed,
            'controller_server.ros__parameters.FollowPath.rotate_to_heading_angular_vel': nav_angular_speed,
            'controller_server.ros__parameters.FollowPath.max_vel_x': nav_linear_speed,
            'controller_server.ros__parameters.FollowPath.max_speed_xy': nav_linear_speed,
            'controller_server.ros__parameters.FollowPath.max_vel_theta': nav_angular_speed,
            'controller_server.ros__parameters.FollowPath.min_vel_x': ['-', nav_reverse_speed],
            'controller_server.ros__parameters.FollowPath.max_vel_x_backwards': nav_reverse_speed,
        },
    )

    return [
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav_params, controller_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }],
        ),
    ]


def generate_launch_description():
    navigation_share = get_package_share_directory('navigation')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_namespace', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('controller', default_value='rpp'),
        DeclareLaunchArgument('use_teb', default_value='false'),
        DeclareLaunchArgument('nav_linear_speed', default_value='0.45'),
        DeclareLaunchArgument('nav_angular_speed', default_value='0.65'),
        DeclareLaunchArgument('nav_reverse_speed', default_value='0.10'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(navigation_share, 'config', 'nav2_params.yaml'),
        ),
        OpaqueFunction(function=launch_setup),
    ])
