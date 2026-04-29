#!/usr/bin/env python3
"""
jetson_hardware_launch.launch.py

Full onboard stack for URSULA running on the Jetson.
Runs: RPLidar + robot_state_publisher + joint_state_publisher + rf2o
      + serial bridge + twist_mux + slam_toolbox + Nav2 + foxglove-bridge

Usage (from dev laptop over SSH):
  ssh uoljetson@<jetson-ip>
  ros2 launch ursula jetson_hardware_launch.launch.py

Optional args:
  ros2 launch ursula jetson_hardware_launch.launch.py slam_mode:=mapping
    (default - build a new map from scratch)

  ros2 launch ursula jetson_hardware_launch.launch.py slam_mode:=localization
    (use a previously saved map - requires /home/uoljetson/maps/ursula_map.yaml)

  ros2 launch ursula jetson_hardware_launch.launch.py nav2:=false
    (disable Nav2 if you only want to drive manually)

  ros2 launch ursula jetson_hardware_launch.launch.py foxglove:=false
    (disable foxglove-bridge if not needed)
"""

import os
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
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
    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='mapping',
        description=(
            'SLAM mode: "mapping" starts a new map from scratch, '
            '"localization" loads an existing map from '
            '/home/uoljetson/maps/ursula_map'
        )
    )
    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Launch Nav2 autonomous navigation stack.'
    )
    foxglove_arg = DeclareLaunchArgument(
        'foxglove',
        default_value='true',
        description='Start foxglove-bridge WebSocket server on port 8765.'
    )

    slam_mode  = LaunchConfiguration('slam_mode')
    use_nav2   = LaunchConfiguration('nav2')
    use_foxglove = LaunchConfiguration('foxglove')

    # ----------------------------------------------------------------
    # RPLidar A3
    # ----------------------------------------------------------------
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port':      '/dev/ttyUSB0',
            'frame_id':         'lidar_link',
            'serial_baudrate':  256000,
            'angle_compensate': True,
        }]
    )

    # ----------------------------------------------------------------
    # Robot state publisher
    # ----------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time':      False,
            'publish_frequency': 10.0,
        }]
    )

    # ----------------------------------------------------------------
    # Joint state publisher
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
    # ----------------------------------------------------------------
    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'rf2o_laser_odometry_node:=ERROR'],
        parameters=[{
            'laser_scan_topic':  '/scan',
            'odom_topic':        '/odom',
            'publish_tf':        True,
            'base_frame_id':     'base_link',
            'odom_frame_id':     'odom',
            'init_pose_from_topic': '',
            'freq':              15.0,
            'use_sim_time':      False
        }]
    )

    # ----------------------------------------------------------------
    # Twist mux
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
    # SLAM Toolbox — MAPPING mode
    # Uses slam_toolbox_params.yaml (the existing mapping config).
    # ----------------------------------------------------------------
    slam_toolbox_mapping = Node(
        condition=LaunchConfigurationEquals('slam_mode', 'mapping'),
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
    # SLAM Toolbox — LOCALIZATION mode
    # Uses slam_toolbox_localization.yaml which points at saved map.
    # Run:  ros2 launch ursula jetson_hardware_launch.launch.py slam_mode:=localization
    # ----------------------------------------------------------------
    slam_toolbox_localization = Node(
        condition=LaunchConfigurationEquals('slam_mode', 'localization'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(ursula_share, 'config', 'slam_toolbox_localization.yaml'),
            {'use_sim_time': False}
        ]
    )

    # ----------------------------------------------------------------
    # Foxglove bridge
    # Connect from Foxglove Studio: ws://<jetson-ip>:8765
    # Over Tailscale:               ws://ursula-jetson:8765
    # ----------------------------------------------------------------
    foxglove_bridge = Node(
        condition=IfCondition(use_foxglove),
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port':               8765,
            'address':            '0.0.0.0',
            'topic_whitelist':    ['.*'],
            'send_buffer_limit':  10000000,
            'use_sim_time':       False,
            'max_qos_depth':      5,
        }]
    )

    # ----------------------------------------------------------------
    # URSULA Manager — map save/load services, status publishing
    # ----------------------------------------------------------------
    ursula_manager = Node(
        package='ursula',
        executable='ursula_manager.py',
        name='ursula_manager',
        output='screen',
        parameters=[{
            'map_save_dir': '/home/uoljetson/maps',
            'use_sim_time': False,
        }]
    )

    # ----------------------------------------------------------------
    # Nav2 — delayed 5 s to allow SLAM to establish map→odom TF first.
    # Nav2 velocity output is remapped inside nav2_launch.launch.py:
    #   controller_server publishes to /cmd_vel_nav
    #   twist_mux picks that up at priority 10
    # ----------------------------------------------------------------
    nav2 = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ursula_share, 'launch', 'nav2_launch.launch.py')
                ),
                # NOTE: remappings= is NOT supported on IncludeLaunchDescription.
                # The /cmd_vel → /cmd_vel_nav remapping is handled inside
                # nav2_launch.launch.py directly on the controller_server node.
                launch_arguments={'use_sim_time': 'false'}.items(),
            )
        ]
    )

    return LaunchDescription([
        slam_mode_arg,
        nav2_arg,
        foxglove_arg,
        rplidar,
        robot_state_publisher,
        joint_state_publisher,
        serial_bridge,
        rf2o,
        twist_mux,
        slam_toolbox_mapping,
        slam_toolbox_localization,
        foxglove_bridge,
        ursula_manager,
        nav2,
    ])
