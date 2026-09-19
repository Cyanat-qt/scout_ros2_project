import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    enable_amcl = LaunchConfiguration('enable_amcl')

    lifecycle_nodes = ['map_server']
    if _as_bool(enable_amcl.perform(context)):
        lifecycle_nodes.append('amcl')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml_file,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            condition=IfCondition(enable_amcl),
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
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
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('enable_amcl', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(navigation_share, 'config', 'nav2_params.yaml'),
        ),
        OpaqueFunction(function=launch_setup),
    ])
