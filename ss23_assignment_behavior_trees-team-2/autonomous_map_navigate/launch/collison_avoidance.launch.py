#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    collison_avoidance_node = Node(
        package='autonomous_map_navigate',
        executable='collison_avoidance',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        collison_avoidance_node
    ])
