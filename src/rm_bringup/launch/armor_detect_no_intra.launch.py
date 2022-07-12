import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

import yaml


def generate_launch_description():
    params_path = os.path.join(
        get_package_share_directory('rm_bringup'), 'params', 'armor_detect.yaml')
    with open(params_path, "r") as f:
        params = yaml.safe_load(f)

    armor_detector_node_params = params["/armor_detector_node"]["ros__parameters"]
    armor_detector_node_params.update({
        "icon_model_path": os.path.join(
            get_package_share_directory('autoaim_detector'), 'assets', 'fc.onnx')
    })
    armor_detector_node = Node(
        package='autoaim_detector',
        executable="armor_detector_node",
        name='armor_detector_node',
        parameters=[armor_detector_node_params],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'debug']
    )

    camera_node = Node(
        package='rm_camera',
        executable="videocapture_node",
        name='camera_node',
        parameters=[{
            "use_sensor_data_qos": True,
            "capture_target": "/home/rm/Videos/2021-0802-1157-小组赛-四川大学vs东莞理工-RD0.mp4"
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([camera_node, armor_detector_node])
