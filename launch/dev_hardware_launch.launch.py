#!/usr/bin/env python3
"""Launch joystick and teleop nodes on dev machine."""

import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        # Joystick node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # Teleop node (twist from joystick)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen'
        ),
    ])
