import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def append_resource_path(env_name, *paths):
    existing_path = os.environ.get(env_name, '')
    resource_path = os.pathsep.join(path for path in paths if path)
    if existing_path:
        resource_path = resource_path + os.pathsep + existing_path
    return SetEnvironmentVariable(env_name, resource_path)


def gazebo_actions(world_path, gui):
    launch_candidates = (
        ('ros_ign_gazebo', 'ign_gazebo.launch.py', 'ign_args'),
        ('ros_gz_sim', 'gz_sim.launch.py', 'gz_args'),
    )

    for package_name, launch_file, args_name in launch_candidates:
        try:
            gazebo_share = get_package_share_directory(package_name)
        except PackageNotFoundError:
            continue

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, 'launch', launch_file)
                ),
                launch_arguments={
                    args_name: f'-r -v 4 "{world_path}"',
                }.items(),
                condition=IfCondition(gui),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, 'launch', launch_file)
                ),
                launch_arguments={
                    args_name: f'-s -r -v 4 "{world_path}"',
                }.items(),
                condition=UnlessCondition(gui),
            ),
        ]

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
    pkg_share = get_package_share_directory('mecanum_robot')

    world_path = os.path.join(pkg_share, 'worlds', 'raicom_intelligent_recon.world')
    model_path = os.path.join(pkg_share, 'models')

    gui = LaunchConfiguration('gui')
    actions = [
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Ignition Gazebo with GUI.',
        ),
        append_resource_path('IGN_GAZEBO_RESOURCE_PATH', model_path, pkg_share),
        append_resource_path('GZ_SIM_RESOURCE_PATH', model_path, pkg_share),
    ]
    actions.extend(gazebo_actions(world_path, gui))

    return LaunchDescription(actions)
