import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    navigation_share = get_package_share_directory('navigation')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    enable_amcl = LaunchConfiguration('enable_amcl')
    controller = LaunchConfiguration('controller')
    use_teb = LaunchConfiguration('use_teb')
    nav_linear_speed = LaunchConfiguration('nav_linear_speed')
    nav_angular_speed = LaunchConfiguration('nav_angular_speed')
    nav_reverse_speed = LaunchConfiguration('nav_reverse_speed')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'include', 'localization_include.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'map': map_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'enable_amcl': enable_amcl,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'include', 'navigation_base_include.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'controller': controller,
            'use_teb': use_teb,
            'nav_linear_speed': nav_linear_speed,
            'nav_angular_speed': nav_angular_speed,
            'nav_reverse_speed': nav_reverse_speed,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_namespace', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('enable_amcl', default_value='false'),
        DeclareLaunchArgument('controller', default_value='rpp'),
        DeclareLaunchArgument('use_teb', default_value='false'),
        DeclareLaunchArgument('nav_linear_speed', default_value='0.45'),
        DeclareLaunchArgument('nav_angular_speed', default_value='0.65'),
        DeclareLaunchArgument('nav_reverse_speed', default_value='0.10'),
        DeclareLaunchArgument(
            'map',
            default_value='/home/ubuntu/raicom/maps/my_map.yaml',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(navigation_share, 'config', 'nav2_params.yaml'),
        ),
        localization_launch,
        TimerAction(period=6.0, actions=[navigation_launch]),
    ])
