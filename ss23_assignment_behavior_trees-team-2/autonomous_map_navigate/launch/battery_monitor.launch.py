#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    battery_monitor_node = Node(
        package='autonomous_map_navigate',
        executable='battery_monitor',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        battery_monitor_node
    ])
