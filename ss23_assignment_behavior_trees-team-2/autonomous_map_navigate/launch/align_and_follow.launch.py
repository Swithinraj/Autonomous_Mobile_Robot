#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    align_and_follow_node = Node(
        package='autonomous_map_navigate',
        executable='align_and_follow',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        align_and_follow_node
    ])
