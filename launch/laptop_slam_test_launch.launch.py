#!/usr/bin/env python3
"""
laptop_slam_test_launch.launch.py

Testing real lidar SLAM on dev laptop only, no Jetson or Arduino needed.
Runs: RPLidar + robot_state_publisher + rf2o + slam_toolbox + RViz

Usage:
  Terminal 1: ros2 launch ursula laptop_slam_test_launch.launch.py
  Terminal 2: ros2 launch ursula dev_hardware_launch.launch.py  (for controller)
"""

import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ursula_share = get_package_share_directory('ursula')
    urdf_file = os.path.join(ursula_share, 'urdf', 'ursula.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([

        # ----------------------------------------------------------------
        # RPLidar A3
        # ----------------------------------------------------------------
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'lidar_link',
                'serial_baudrate': 256000,
                'angle_compensate': True,
            }]
        ),

        # ----------------------------------------------------------------
        # Robot state publisher - publishes TF tree from URDF
        # ----------------------------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),

        # Publishes zero joint states so robot_state_publisher can
        # compute correct wheel positions from URDF
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        #         # ----------------------------------------------------------------
        # # Static transforms for wheel links
        # # No joint states available without Arduino, wheels are fixed for visualisation
        # # ----------------------------------------------------------------
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='front_left_wheel_tf',
        #     arguments=['0', '0', '0', '0', '0', '0',
        #             'chassis_link', 'front_left_wheel_link'],
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='front_right_wheel_tf',
        #     arguments=['0', '0', '0', '0', '0', '0',
        #             'chassis_link', 'front_right_wheel_link'],
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='rear_left_wheel_tf',
        #     arguments=['0', '0', '0', '0', '0', '0',
        #             'chassis_link', 'rear_left_wheel_link'],
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='rear_right_wheel_tf',
        #     arguments=['0', '0', '0', '0', '0', '0',
        #             'chassis_link', 'rear_right_wheel_link'],
        # ),

        # ----------------------------------------------------------------
        # RF2O laser odometry
        # Generates odom->base_link transform from lidar scan matching
        # ----------------------------------------------------------------
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry_node',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom',
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 10.0,
                'use_sim_time': False
            }]
        ),

        # ----------------------------------------------------------------
        # SLAM Toolbox
        # Generates map->odom transform from lidar scan matching
        # ----------------------------------------------------------------
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(ursula_share, 'config', 'slam_toolbox_params.yaml'),
                {'use_sim_time': False}
            ]
        ),

        # ----------------------------------------------------------------
        # RViz
        # ----------------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(ursula_share, 'rviz', 'ursula.rviz')],
            parameters=[{'use_sim_time': False}]
        ),
    ])
