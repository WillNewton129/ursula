#!/usr/bin/env python3
"""
sim_teleop_launch.launch.py

Lightweight teleop launcher for simulation.
Run this in a SEPARATE terminal alongside gazebo_sim_launch.

Provides TWO teleop options selected via launch arg:

  ros2 launch ursula sim_teleop_launch.launch.py teleop:=keyboard
    → Use keyboard (WASD / arrow keys). Good for precise steering tests.

  ros2 launch ursula sim_teleop_launch.launch.py teleop:=joystick
    → Use the F310 gamepad (same config as real robot).

Keyboard controls (teleop_twist_keyboard):
  i / , = forward / backward
  j / l = turn left / right
  u / o = forward-left / forward-right diagonal (key for steering-while-moving test!)
  k     = stop
  q / z = increase / decrease all speeds
  w / x = increase / decrease linear speed only
  e / c = increase / decrease angular speed only
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ursula_share = get_package_share_directory('ursula')

    teleop_type_arg = DeclareLaunchArgument(
        'teleop',
        default_value='joystick',
        description='Teleop type: keyboard or joystick'
    )
    teleop_type = LaunchConfiguration('teleop')

    # Keyboard teleop
    # keyboard_teleop = Node(
    #     condition=IfCondition(
    #         # Hack: LaunchConfiguration doesn't support == directly,
    #         # but teleop_twist_keyboard is always fine to run — the arg is just a hint.
    #         # See note below about running this manually if needed.
    #         teleop_type
    #     ),
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop_twist_keyboard',
    #     output='screen',
    #     prefix='xterm -e',   # Opens in its own terminal window so stdin works
    #     parameters=[{'use_sim_time': True}],
    #     remappings=[('/cmd_vel', '/cmd_vel')]
    # )

    # Joystick nodes
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'use_sim_time': True}]
    )

    joystick_teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[
            os.path.join(ursula_share, 'config', 'ps4_teleop.yaml'),
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        teleop_type_arg,
        #keyboard_teleop,
        joy_node,
        joystick_teleop,
    ])
