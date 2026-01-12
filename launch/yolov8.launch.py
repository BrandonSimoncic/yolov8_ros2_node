# Copyright (c) [year]. All rights reserved.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # YOLOv8 object detection node
    yolov8_node = Node(
        package='yolov8_ros',
        executable='yolov8_ros',
        remappings=[],
        output='screen'
    )
    ld.add_action(yolov8_node)

    return ld