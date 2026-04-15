#!/usr/bin/env python3
"""
jetson_rtabmap_launch.launch.py

Full onboard stack using RTAB-Map for 3-D mapping instead of slam_toolbox.
RTAB-Map fuses the OAK-D RGB-D data with lidar scan matching to produce:
  - A 3-D occupancy voxel map
  - A 2-D projected occupancy grid (for Nav2 costmaps)
  - Loop-closure-corrected odometry
  - A persistent database (~/.ros/rtabmap.db) that survives reboots

Usage:
  ros2 launch ursula jetson_rtabmap_launch.launch.py

Optional args:
  mode:=mapping       (default) build a new 3-D map
  mode:=localization  localise against the last saved database
  nav2:=true|false    enable/disable Nav2 (default true)
  foxglove:=true|false enable/disable Foxglove bridge (default true)
  detection:=true|false enable/disable YOLOv4 detection (default true)
  reset_db:=true      DELETE the existing rtabmap.db and start fresh
                      (use this when you want a completely new map)

How it works:
  - rf2o provides wheel-less odometry from lidar scan matching (odom frame)
  - RTAB-Map rgbd_odometry refines this with visual odometry from the OAK-D
  - RTAB-Map rtabmap node fuses lidar + RGB-D into a 3-D map with loop closure
  - The 2-D projection is published to /map for Nav2
  - In localization mode, the existing database is loaded and no new nodes
    are added to the map — the robot just tracks its position within it

Switching between mapping and localization:
  After a good mapping run, simply relaunch with mode:=localization.
  The database at ~/.ros/rtabmap.db is loaded automatically.
  To start a completely new map: mode:=mapping reset_db:=true

Foxglove visualisation:
  Add a 3D panel and subscribe to:
    /rtabmap/cloud_map     — accumulated 3-D point cloud
    /rtabmap/grid_map      — 2-D occupancy grid
    /ursula/detection_markers — YOLO detection markers
    /scan                  — live lidar
    /odom                  — odometry arrow

Prerequisites on Jetson:
  sudo apt install ros-humble-rtabmap-ros
  sudo apt install ros-humble-depthai-ros

Topic mapping:
  This launch remaps OAK-D topics to match what RTAB-Map expects:
    /oak/rgb/image_raw        → rgb/image
    /oak/rgb/camera_info      → rgb/camera_info
    /oak/stereo/image_raw     → depth/image
    /oak/stereo/camera_info   → depth/camera_info  (NOT remapped — see note)

  NOTE: The OAK-D stereo topic does not publish camera_info by default in
  some depthai_ros versions. If RTAB-Map complains about missing camera_info,
  enable it in your OAK-D params config or use approx_sync:=true (already set).
"""

import os
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ursula_share  = get_package_share_directory('ursula')
    urdf_file     = os.path.join(ursula_share, 'urdf', 'ursula.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    # ----------------------------------------------------------------
    # Launch arguments
    # ----------------------------------------------------------------
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description=(
            '"mapping" — build a new 3-D map (adds nodes to rtabmap.db). '
            '"localization" — localise against existing rtabmap.db without modifying it.'
        )
    )
    nav2_arg = DeclareLaunchArgument(
        'nav2', default_value='true',
        description='Launch Nav2 stack.'
    )
    foxglove_arg = DeclareLaunchArgument(
        'foxglove', default_value='true',
        description='Start Foxglove bridge on port 8765.'
    )
    detection_arg = DeclareLaunchArgument(
        'detection', default_value='true',
        description='Launch YOLOv4 spatial detection + detection_mapper.'
    )
    reset_db_arg = DeclareLaunchArgument(
        'reset_db', default_value='false',
        description='Set true to delete ~/.ros/rtabmap.db before launching.'
    )

    mode         = LaunchConfiguration('mode')
    use_nav2     = LaunchConfiguration('nav2')
    use_foxglove = LaunchConfiguration('foxglove')
    use_detection = LaunchConfiguration('detection')
    reset_db     = LaunchConfiguration('reset_db')

    # ----------------------------------------------------------------
    # Optional: delete rtabmap.db before starting a fresh map
    # Only runs if reset_db:=true
    # ----------------------------------------------------------------
    delete_db = ExecuteProcess(
        condition=IfCondition(reset_db),
        cmd=['rm', '-f', os.path.expanduser('~/.ros/rtabmap.db')],
        output='screen'
    )

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
    # Serial bridge
    # ----------------------------------------------------------------
    serial_bridge = Node(
        package='ursula',
        executable='serial_command_bridge.py',
        name='serial_command_bridge',
        output='screen',
        parameters=[os.path.join(ursula_share, 'config', 'serial_bridge_params.yaml')]
    )

    # ----------------------------------------------------------------
    # RF2O laser odometry — provides initial odom→base_link TF.
    # RTAB-Map will use this as a prior and refine it with visual odom.
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
    # URSULA Manager
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
    # OAK-D camera — delayed 3 s for RSP to publish robot_description
    # ----------------------------------------------------------------
    camera = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ursula_share, 'launch', 'camera.launch.py')
                ),
                launch_arguments={
                    'name':                        'oak',
                    'namespace':                   '',
                    'parent_frame':                'oak_d_camera_link',
                    'publish_tf_from_calibration': 'false',
                    'camera_model':                'OAK-D-PRO',
                    'use_rviz':                    'false',
                    # Enable pointcloud topic — RTAB-Map can use it directly
                    'pointcloud.enable':           'false',  # RTAB-Map prefers depth image
                    'rectify_rgb':                 'true',
                }.items(),
            )
        ]
    )

    # ----------------------------------------------------------------
    # RTAB-Map — delayed 8 s to allow camera + RF2O to start
    #
    # Two node setup:
    #   1. rgbd_odometry  — visual odometry refinement (optional but helpful)
    #   2. rtabmap        — 3-D mapping / loop closure / localisation
    #
    # Topic remappings match the OAK-D driver's default output topics.
    # ----------------------------------------------------------------

    rtabmap_common_params = {
        'frame_id':                 'base_link',
        'odom_frame_id':            'odom',
        'approx_sync':              True,
        'approx_sync_max_interval': 0.05,
        'queue_size':               10,
        'qos':                      1,
        'use_sim_time':             False,
    }

    rtabmap_remappings = [
        ('rgb/image',       '/oak/rgb/image_raw'),
        ('rgb/camera_info', '/oak/rgb/camera_info'),
        ('depth/image',     '/oak/stereo/image_raw'),
    ]

    # Visual odometry node — refines rf2o with RGB-D matching.
    # Publishes to /visual_odom. RTAB-Map uses this as its odom input.
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            **rtabmap_common_params,
            'publish_null_when_lost': False,
            'guess_frame_id':         'base_link',
        }],
        remappings=rtabmap_remappings + [('odom', '/visual_odom')]
    )

    # RTAB-Map core — 3-D mapping, loop closure, occupancy grid projection.
    # mode:=localization sets Mem/IncrementalMemory=false so the database
    # is read-only and the robot only localises within the existing map.
    rtabmap_mapping = Node(
        condition=LaunchConfigurationEquals('mode', 'mapping'),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            **rtabmap_common_params,
            'map_frame_id':           'map',
            'subscribe_depth':        True,
            'subscribe_rgb':          True,
            'subscribe_scan':         True,
            'database_path':          '~/.ros/rtabmap.db',
            # Incremental mapping — add new nodes as the robot moves
            'Mem/IncrementalMemory':  'true',
            'Mem/InitWithSavedMap':   'true',
            # Loop closure tuning
            'Vis/MinInliers':         '10',
            'Reg/Strategy':           '1',          # 1 = ICP (uses lidar scan)
            'Icp/CorrespondenceRatio': '0.2',
            'RGBD/OptimizeMaxError':  '3.0',
            'RGBD/AngularUpdate':     '0.1',
            'RGBD/LinearUpdate':      '0.1',
            # 2-D occupancy grid projection (for Nav2)
            'Grid/FromDepth':         'false',      # Use lidar for the 2-D grid
            'Grid/3D':                'false',
            'Mem/OccupancyGrid':      'true',
        }],
        remappings=rtabmap_remappings + [
            ('odom',  '/visual_odom'),
            ('scan',  '/scan'),
            ('map',   '/rtabmap/grid_map'),
        ]
    )

    rtabmap_localization = Node(
        condition=LaunchConfigurationEquals('mode', 'localization'),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            **rtabmap_common_params,
            'map_frame_id':           'map',
            'subscribe_depth':        True,
            'subscribe_rgb':          True,
            'subscribe_scan':         True,
            'database_path':          '~/.ros/rtabmap.db',
            # Read-only — do not add new nodes
            'Mem/IncrementalMemory':  'false',
            'Mem/InitWithSavedMap':   'true',
            'Vis/MinInliers':         '10',
            'Reg/Strategy':           '1',
            'Icp/CorrespondenceRatio': '0.2',
            'RGBD/OptimizeMaxError':  '3.0',
            'Grid/FromDepth':         'false',
            'Grid/3D':                'false',
            'Mem/OccupancyGrid':      'true',
        }],
        remappings=rtabmap_remappings + [
            ('odom',  '/visual_odom'),
            ('scan',  '/scan'),
            ('map',   '/rtabmap/grid_map'),
        ]
    )

    # Static TF: RTAB-Map needs base_link → oak-d-base-frame for camera calibration.
    # This is supplementary to the URDF joints already in robot_state_publisher.
    base_to_oakd_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_oakd_tf',
        arguments=['0', '0', '0', '0', '0', '0',
                   'oak_d_camera_link', 'oak-d-base-frame']
    )

    # ----------------------------------------------------------------
    # Detection — delayed 10 s (needs camera pipeline)
    # ----------------------------------------------------------------
    detection = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ursula_share, 'launch', 'detection.launch.py')
                ),
            ),
            Node(
                condition=IfCondition(use_detection),
                package='ursula',
                executable='detection_mapper.py',
                name='detection_mapper',
                output='screen',
                parameters=[{'use_sim_time': False}]
            ),
        ]
    )

    # ----------------------------------------------------------------
    # Nav2 — delayed 10 s to allow RTAB-Map to establish map→odom TF
    # Nav2 subscribes to /rtabmap/grid_map which is remapped to /map
    # inside the rtabmap node above.
    # ----------------------------------------------------------------
    nav2 = TimerAction(
        period=10.0,
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
        mode_arg,
        nav2_arg,
        foxglove_arg,
        detection_arg,
        reset_db_arg,
        # Optional: wipe DB before starting
        delete_db,
        # Core hardware
        rplidar,
        robot_state_publisher,
        joint_state_publisher,
        serial_bridge,
        rf2o,
        twist_mux,
        # Monitoring
        foxglove_bridge,
        ursula_manager,
        # Camera — delayed 3 s
        camera,
        base_to_oakd_tf,
        # RTAB-Map — delayed 8 s
        TimerAction(period=8.0, actions=[
            rgbd_odometry,
            rtabmap_mapping,
            rtabmap_localization,
        ]),
        # Detection — delayed 10 s
        detection,
        # Nav2 — delayed 10 s
        nav2,
    ])