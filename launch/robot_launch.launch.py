#!/usr/bin/env python3
"""
jetson_full_launch.launch.py

Full onboard stack for URSULA — hardware + camera + object detection.
Extends jetson_hardware_launch with:
  - OAK-D camera (depthai_ros_driver)
  - YOLOv4 spatial object detection (depthai_examples)
  - Detection mapper (ursula/scripts/detection_mapper.py)
    → publishes labelled cylinder markers to /ursula/detection_markers

Usage:
  ros2 launch ursula jetson_full_launch.launch.py

Optional args (same as jetson_hardware_launch plus camera controls):
  slam_mode:=mapping        (default) build a new map from scratch
  slam_mode:=localization   load saved map from /home/uoljetson/maps/ursula_map
  nav2:=true|false          enable/disable Nav2 (default true)
  foxglove:=true|false      enable/disable Foxglove bridge (default true)
  camera:=true|false        enable/disable OAK-D camera (default true)
  detection:=true|false     enable/disable YOLO detection node (default true)

Prerequisites on Jetson:
  sudo apt install ros-humble-depthai-ros
  The YOLO blob must exist at the path in detection.launch.py
  (see launch/detection.launch.py resourceBaseFolder and nnName)

Notes:
  - Camera launch is included via IncludeLaunchDescription referencing
    launch/camera.launch.py, which wraps the depthai_ros_driver composable
    node container.
  - Detection uses depthai_examples tracker_yolov4_spatial_node — this
    runs inference on the OAK-D's VPU, not the Jetson CPU.
  - detection_mapper.py subscribes to /oak/nn/detections and the TF tree
    to place labelled markers on the map in Foxglove's 3D panel.
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
    urdf_file    = os.path.join(ursula_share, 'urdf', 'ursula.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    # ----------------------------------------------------------------
    # Launch arguments
    # ----------------------------------------------------------------
    slam_mode_arg = DeclareLaunchArgument(
        'slam_mode',
        default_value='mapping',
        description=(
            'SLAM mode: "mapping" starts a new map, '
            '"localization" loads /home/uoljetson/maps/ursula_map'
        )
    )
    nav2_arg = DeclareLaunchArgument(
        'nav2', default_value='true',
        description='Launch Nav2 autonomous navigation stack.'
    )
    foxglove_arg = DeclareLaunchArgument(
        'foxglove', default_value='true',
        description='Start Foxglove bridge WebSocket server on port 8765.'
    )
    camera_arg = DeclareLaunchArgument(
        'camera', default_value='true',
        description='Launch OAK-D camera driver.'
    )
    detection_arg = DeclareLaunchArgument(
        'detection', default_value='true',
        description='Launch YOLOv4 spatial detection node.'
    )

    slam_mode    = LaunchConfiguration('slam_mode')
    use_nav2     = LaunchConfiguration('nav2')
    use_foxglove = LaunchConfiguration('foxglove')
    use_camera   = LaunchConfiguration('camera')
    use_detection = LaunchConfiguration('detection')

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
        parameters=[{
            'laser_scan_topic':     '/scan',
            'odom_topic':           '/odom',
            'publish_tf':           True,
            'base_frame_id':        'base_link',
            'odom_frame_id':        'odom',
            'init_pose_from_topic': '',
            'freq':                 15.0,
            'use_sim_time':         False,
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
    # ----------------------------------------------------------------
    foxglove_bridge = Node(
        condition=IfCondition(use_foxglove),
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port':              8765,
            'address':           '0.0.0.0',
            'topic_whitelist':   ['.*'],
            'send_buffer_limit': 10000000,
            'use_sim_time':      False,
            'max_qos_depth':     5,
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
    # OAK-D Camera — depthai_ros_driver composable node container
    # Launched via the existing camera.launch.py which handles all the
    # depthai_descriptions TF and composable node setup.
    # Delayed 3 s to allow RSP to publish robot_description first.
    # ----------------------------------------------------------------
    camera = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ursula_share, 'launch', 'camera.launch.py')
                ),
                launch_arguments={
                    'name':         'oak',
                    'namespace':    '',
                    # Point the camera TF parent at our URDF frame
                    'parent_frame': 'oak_d_camera_link',
                    'publish_tf_from_calibration': 'false',
                    'camera_model': 'OAK-D-PRO',
                    'use_rviz':     'false',
                    # Enable depth + pointcloud for RTAB-Map compatibility later
                    'pointcloud.enable': 'false',
                    'rectify_rgb':  'true',
                }.items(),
            )
        ]
    )

    # ----------------------------------------------------------------
    # YOLOv4 spatial detection
    # Runs on OAK-D VPU — requires the blob file at the path below.
    # Delayed 8 s to allow camera pipeline to establish first.
    # ----------------------------------------------------------------
    detection = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ursula_share, 'launch', 'detection.launch.py')
                ),
            )
        ]
    )

    # ----------------------------------------------------------------
    # Detection mapper
    # Subscribes to /oak/nn/detections + TF, publishes RViz/Foxglove markers.
    # Delayed 10 s to allow detection pipeline to start.
    # ----------------------------------------------------------------
    detection_mapper = TimerAction(
        period=10.0,
        actions=[
            Node(
                condition=IfCondition(use_detection),
                package='ursula',
                executable='detection_mapper.py',
                name='detection_mapper',
                output='screen',
                parameters=[{'use_sim_time': False}]
            )
        ]
    )

    # ----------------------------------------------------------------
    # Nav2 — delayed 5 s to allow SLAM to establish map→odom TF first
    # ----------------------------------------------------------------
    nav2 = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ursula_share, 'launch', 'nav2_launch.launch.py')
                ),
                launch_arguments={'use_sim_time': 'false'}.items(),
            )
        ]
    )

    return LaunchDescription([
        slam_mode_arg,
        nav2_arg,
        foxglove_arg,
        camera_arg,
        detection_arg,
        # Core hardware — start immediately
        rplidar,
        robot_state_publisher,
        joint_state_publisher,
        serial_bridge,
        rf2o,
        twist_mux,
        # SLAM
        slam_toolbox_mapping,
        slam_toolbox_localization,
        # Monitoring / management
        foxglove_bridge,
        ursula_manager,
        # Camera stack — delayed 3 s
        camera,
        # Detection — delayed 8 s (needs camera pipeline)
        detection,
        detection_mapper,
        # Nav2 — delayed 5 s (needs SLAM TF)
        nav2,
    ])