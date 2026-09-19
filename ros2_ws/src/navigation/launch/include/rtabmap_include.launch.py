from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')

    parameters = {
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_rgb': True,
        'subscribe_scan': True,
        'use_action_for_goal': True,
        'qos_scan': qos,
        'qos_image': qos,
        'queue_size': 30,
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'Grid/RangeMin': '0.2',
        'Optimizer/GravitySigma': '0',
    }

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('odom', '/odom'),
        ('scan', '/scan'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('qos', default_value='2'),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[parameters],
            remappings=remappings,
        ),
    ])
