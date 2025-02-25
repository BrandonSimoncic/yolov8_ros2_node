# Copyright (c) [year]. All rights reserved.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Navigation executable for localization
    stereo_vision_node = Node(
        package='yolov8_ros2',
        executable='yolov8_node',
        parameters=[
            os.path.join(get_package_share_directory('yolov8_node'), 'config', 'config.yaml')
        ],
        remappings=[],
        output='screen'
    )
    ld.add_action(stereo_vision_node)

    return ld