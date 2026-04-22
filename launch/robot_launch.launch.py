#!/usr/bin/env python3
"""
robot_launch.launch.py

Full onboard stack for URSULA — hardware + camera + object detection.

CHANGES FROM PREVIOUS VERSION:
  1. camera arg now actually gates the camera TimerAction (was always running).
  2. detection IncludeLaunchDescription now has condition=IfCondition(use_detection)
     so YOLO does not start when detection:=false.
  3. detection.launch.py no longer references my_robot_bringup/detection_visualiser
     — that package is not in this repo. Visualisation is handled by detection_mapper.py.
  4. Added base_to_oakd_tf static TF (oak_d_camera_link -> oak-d-base-frame)
     so depthai_ros_driver calibration TF resolves correctly.
  5. Nav2 delay increased from 5s to 10s — gives rf2o + SLAM time to establish
     map->odom before Nav2 starts trying to use it.
  6. slam_toolbox_localization condition fixed — was checking 'slam_mode' string
     which works, kept as-is (LaunchConfigurationEquals is correct).

Usage:
  # Mapping mode (default) — builds a new map
  ros2 launch ursula robot_launch.launch.py

  # Localization mode — navigates on a previously saved map
  ros2 launch ursula robot_launch.launch.py slam_mode:=localization

  # Manual teleop only — disables Nav2 and detection for initial testing
  ros2 launch ursula robot_launch.launch.py nav2:=false detection:=false camera:=false

Optional args:
  slam_mode:=mapping|localization   (default: mapping)
  nav2:=true|false                  (default: true)
  foxglove:=true|false              (default: true)
  camera:=true|false                (default: true)
  detection:=true|false             (default: true)

Test sequence (do in this order):
  Step 1: nav2:=false detection:=false camera:=false  — verify motors, lidar, SLAM
  Step 2: nav2:=false detection:=false                — add camera, verify topics
  Step 3: nav2:=false                                  — add detection, verify markers
  Step 4: full launch                                  — Nav2 autonomous navigation
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
        description='"mapping" starts fresh, "localization" loads saved map'
    )
    nav2_arg = DeclareLaunchArgument(
        'nav2', default_value='true',
        description='Launch Nav2 autonomous navigation stack'
    )
    foxglove_arg = DeclareLaunchArgument(
        'foxglove', default_value='true',
        description='Start Foxglove bridge WebSocket server on port 8765'
    )
    camera_arg = DeclareLaunchArgument(
        'camera', default_value='true',
        description='Launch OAK-D camera driver'
    )
    detection_arg = DeclareLaunchArgument(
        'detection', default_value='true',
        description='Launch YOLOv4 spatial detection + detection_mapper'
    )

    slam_mode     = LaunchConfiguration('slam_mode')
    use_nav2      = LaunchConfiguration('nav2')
    use_foxglove  = LaunchConfiguration('foxglove')
    use_camera    = LaunchConfiguration('camera')
    use_detection = LaunchConfiguration('detection')

    # ----------------------------------------------------------------
    # RPLidar A3
    # /dev/ttyUSB0 — confirm with: ls -la /dev/ttyUSB* before launching
    # If absent, check: sudo dmesg | tail -20
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
    # Robot state publisher — publishes TF tree from URDF
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
    # Joint state publisher — publishes zero states for suspension/
    # front wheel joints so robot_state_publisher can compute TF.
    # Without this, RViz/Foxglove will show TF lookup failures for
    # those links.
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
    # /dev/ttyACM0 — confirm with: ls -la /dev/ttyACM*
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
    # Generates odom->base_link TF from lidar scan matching.
    # Requires lidar to be publishing /scan before it starts.
    # freq: 15.0 — matches RPLIDAR A3 scan rate
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
    # Twist mux — prioritises joystick over Nav2
    # /cmd_vel_joy  priority 100 (from dev laptop via ROS network)
    # /cmd_vel_nav  priority 10  (from Nav2 controller_server)
    # Output: /cmd_vel -> serial_bridge -> Arduino
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
    # SLAM Toolbox — MAPPING mode (default)
    # Builds map from scratch. Publishes map->odom TF.
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
    # Loads /home/uoljetson/maps/ursula_map.yaml and localises within it.
    # Switch to this after a successful mapping run + map save.
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
    # Connect from Foxglove Studio on any device: ws://<jetson-ip>:8765
    # Over Tailscale: ws://100.x.x.x:8765 or ws://ursula-jetson:8765
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
    # URSULA Manager — map save/load services + /ursula/status publisher
    # Call from anywhere: ros2 service call /ursula/save_map std_srvs/srv/Trigger {}
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
    # Static TF: oak_d_camera_link -> oak-d-base-frame
    # Required by depthai_ros_driver for camera calibration TF lookups.
    # The URDF defines oak_d_camera_link; depthai publishes oak-d-base-frame.
    # This bridge tells TF they are coincident.
    # Only published when camera is enabled.
    # ----------------------------------------------------------------
    base_to_oakd_tf = Node(
        condition=IfCondition(use_camera),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_oakd_tf',
        arguments=['0', '0', '0', '0', '0', '0',
                   'oak_d_camera_link', 'oak-d-base-frame']
    )

    # ----------------------------------------------------------------
    # OAK-D Camera — depthai_ros_driver composable node container
    # Delayed 5s to allow robot_state_publisher to latch robot_description
    # before depthai tries to look up TF frames.
    #
    # FIX: now correctly gated by use_camera condition.
    # ----------------------------------------------------------------
    camera = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                condition=IfCondition(use_camera),   # FIX: was always running
                launch_description_source=PythonLaunchDescriptionSource(
                    os.path.join(ursula_share, 'launch', 'camera.launch.py')
                ),
                launch_arguments={
                    'name':         'oak',
                    'namespace':    '',
                    'parent_frame': 'oak_d_camera_link',
                    'publish_tf_from_calibration': 'false',
                    'camera_model': 'OAK-D-PRO',
                    'use_rviz':     'false',
                    'pointcloud.enable': 'false',
                    'rectify_rgb':  'true',
                }.items(),
            )
        ]
    )

    # ----------------------------------------------------------------
    # YOLOv4 spatial detection
    # Delayed 10s — needs camera pipeline fully started.
    #
    # FIX: IncludeLaunchDescription now has condition=IfCondition(use_detection)
    # Previously the YOLO node always started regardless of detection:=false.
    #
    # detection.launch.py runs tracker_yolov4_spatial_node from depthai_examples.
    # Verify the blob path inside detection.launch.py before first run:
    #   ls /home/uoljetson/oak_camera_ws/src/depthai-ros/depthai_examples/resources/
    # ----------------------------------------------------------------
    detection = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                condition=IfCondition(use_detection),   # FIX: was always running
                launch_description_source=PythonLaunchDescriptionSource(
                    os.path.join(ursula_share, 'launch', 'detection.launch.py')
                ),
            )
        ]
    )

    # ----------------------------------------------------------------
    # Detection mapper
    # Subscribes to tracker output + TF, publishes cylinder markers to
    # /ursula/detection_markers for Foxglove 3D panel.
    # Delayed 12s — needs detection pipeline to be publishing first.
    # ----------------------------------------------------------------
    detection_mapper = TimerAction(
        period=12.0,
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
    # Nav2 — delayed 10s to allow SLAM to establish map->odom TF.
    # FIX: was 5s — too short for rf2o cold start + SLAM initialisation.
    # Nav2 velocity output (/cmd_vel_nav) is picked up by twist_mux at
    # priority 10 — joystick always overrides.
    # ----------------------------------------------------------------
    nav2 = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                condition=IfCondition(use_nav2),
                launch_description_source=PythonLaunchDescriptionSource(
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

        # SLAM (one of these fires depending on slam_mode)
        slam_toolbox_mapping,
        slam_toolbox_localization,

        # Monitoring / management
        foxglove_bridge,
        ursula_manager,

        # Camera TF bridge (instant, gated by use_camera)
        base_to_oakd_tf,

        # Camera driver — delayed 5s
        camera,

        # Detection pipeline — delayed 10s + 12s
        detection,
        detection_mapper,

        # Nav2 — delayed 10s
        nav2,
    ])