#!/usr/bin/env python3
"""
nav2_launch.launch.py
Launch the Nav2 autonomous navigation stack for URSULA.

In Humble, nav2_recoveries was renamed nav2_behaviors.
Run AFTER gazebo_sim_launch.launch.py (or jetson_hardware_launch + slam_launch).

Usage (sim):
  ros2 launch ursula nav2_launch.launch.py
  ros2 launch ursula nav2_launch.launch.py use_sim_time:=true

Usage (real robot, run on Jetson):
  ros2 launch ursula nav2_launch.launch.py use_sim_time:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ursula_share = get_package_share_directory('ursula')
    nav2_params = os.path.join(ursula_share, 'config', 'nav2_params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        use_sim_time_arg,

        # ── Controller server ─────────────────────────────────────────────
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}]
        ),

        # ── Planner server ────────────────────────────────────────────────
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}]
        ),

        # ── Behavior server (was nav2_recoveries in Foxy/Galactic) ────────
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}]
        ),

        # ── BT Navigator ──────────────────────────────────────────────────
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}]
        ),

        # ── Lifecycle manager ─────────────────────────────────────────────
        # Manages startup/shutdown order for all Nav2 nodes.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ]}
            ]
        ),
    ])
