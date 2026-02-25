#!/usr/bin/env python3
"""Launch serial bridge, lidar, and odometry on Jetson."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ursula_share = os.path.join(
        os.environ.get('AMENT_PREFIX_PATH').split(':')[0],
        'share', 'ursula'
    )

    return LaunchDescription([
        # Serial bridge to Arduino
        Node(
            package='ursula',
            executable='serial_command_bridge.py',
            name='serial_command_bridge',
            output='screen',
            parameters=[os.path.join(ursula_share, 'config', 'serial_bridge_params.yaml')]
        ),

        # RPLIDAR driver
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_composition',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser'}]  # Adjust serial port
        ),

        # RF2O odometry node
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry_node',
            output='screen',
            parameters=[{'scan_topic': '/scan', 'odom_frame': 'odom', 'base_frame': 'base_link'}]
        ),
    ])
