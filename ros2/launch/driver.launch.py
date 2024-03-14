import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_prefix('cms50dplus_ros2_driver'),
        '..',
        '..',
        'src',
        'cms50dplus_ros2_driver',
        'ros2',
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='cms50dplus_ros2_driver',
            executable='driver_node',
            name='driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[config]
        )
    ])
