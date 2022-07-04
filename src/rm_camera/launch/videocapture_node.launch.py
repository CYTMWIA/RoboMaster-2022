import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rm_camera'), 'params', 'videocapture_node.yaml')

    videocapture_node = Node(
        package='rm_camera',
        executable='videocapture_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([videocapture_node])