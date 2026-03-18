#!/usr/bin/env python3
"""
jetson_hardware_launch.launch.py

Full onboard stack for URSULA running on the Jetson.
Runs: RPLidar + robot_state_publisher + joint_state_publisher + rf2o
      + serial bridge + twist_mux + slam_toolbox + Nav2

Usage (from dev laptop over SSH):
  ssh uoljetson@<jetson-ip>
  ros2 launch ursula jetson_hardware_launch.launch.py

Optional args:
  ros2 launch ursula jetson_hardware_launch.launch.py slam:=false
    (disable SLAM if running in localisation mode with a saved map)

  ros2 launch ursula jetson_hardware_launch.launch.py nav2:=false
    (disable Nav2 if you only want to drive manually)
"""

import os
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ursula_share = get_package_share_directory('ursula')
    urdf_file = os.path.join(ursula_share, 'urdf', 'ursula.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    # ----------------------------------------------------------------
    # Launch arguments
    # ----------------------------------------------------------------
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Run slam_toolbox for mapping. Set false for localisation with saved map.'
    )
    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Launch Nav2 autonomous navigation stack.'
    )

    use_slam = LaunchConfiguration('slam')
    use_nav2 = LaunchConfiguration('nav2')

    # ----------------------------------------------------------------
    # RPLidar A3
    # ----------------------------------------------------------------
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'lidar_link',
            'serial_baudrate': 256000,
            'angle_compensate': True,
        }]
    )

    # ----------------------------------------------------------------
    # Robot state publisher - publishes TF tree from URDF
    # ----------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
            'publish_frequency': 10.0,
        }]
    )

    # ----------------------------------------------------------------
    # Joint state publisher
    # Publishes zero joint states so wheel positions are correct in RViz
    # ----------------------------------------------------------------
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # ----------------------------------------------------------------
    # Serial bridge to Arduino
    # Handles cmd_vel -> PWM and estop from joystick
    # ----------------------------------------------------------------
    serial_bridge = Node(
        package='ursula',
        executable='serial_command_bridge.py',
        name='serial_command_bridge',
        output='screen',
        parameters=[os.path.join(ursula_share, 'config', 'serial_bridge_params.yaml')]
    )

    # ----------------------------------------------------------------
    # RF2O laser odometry
    # Generates odom->base_link transform from lidar scan matching
    # ----------------------------------------------------------------
    rf2o = Node(
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
    )

    # ----------------------------------------------------------------
    # Twist mux - prioritises joystick over Nav2 autonomous commands
    # ----------------------------------------------------------------
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[os.path.join(ursula_share, 'config', 'twist_mux.yaml')],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    # ----------------------------------------------------------------
    # SLAM Toolbox
    # Generates map->odom transform and builds occupancy map
    # ----------------------------------------------------------------
    slam_toolbox = Node(
        condition=IfCondition(use_slam),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(ursula_share, 'config', 'slam_toolbox_params.yaml'),
            {'use_sim_time': False}
        ]
    )

    # ----------------------------------------------------------------
    # Nav2 - delayed by 5 seconds to allow SLAM to initialise first
    # and establish the map->odom transform before Nav2 starts up.
    # Disable with nav2:=false if you only want manual teleop.
    # ----------------------------------------------------------------
    nav2 = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ursula_share, 'launch', 'nav2_launch.launch.py')
                ),
                launch_arguments={'use_sim_time': 'false'}.items()
            )
        ]
    )

    return LaunchDescription([
        slam_arg,
        nav2_arg,
        rplidar,
        robot_state_publisher,
        joint_state_publisher,
        serial_bridge,
        rf2o,
        twist_mux,
        slam_toolbox,
        nav2,
    ])
