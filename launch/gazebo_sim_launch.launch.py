#!/usr/bin/env python3
"""
gazebo_sim_launch.launch.py

Launches the full URSULA simulation stack:
  - Gazebo Classic with the maze world
  - robot_state_publisher (URDF → TF)
  - joint_state_publisher (for wheel TF in RViz)
  - Spawn the robot into Gazebo
  - slam_toolbox
  - RViz2

Usage:
  ros2 launch ursula gazebo_sim_launch.launch.py

Optional args:
  ros2 launch ursula gazebo_sim_launch.launch.py use_rviz:=false
  ros2 launch ursula gazebo_sim_launch.launch.py slam:=false   (if you just want to drive)
  ros2 launch ursula gazebo_sim_launch.launch.py x_spawn:=2.0 y_spawn:=-4.0
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    ursula_share = get_package_share_directory('ursula')

    # ------------------------------------------------------------------ #
    # Launch arguments                                                     #
    # ------------------------------------------------------------------ #
    use_rviz_arg    = DeclareLaunchArgument('use_rviz', default_value='true')
    use_slam_arg    = DeclareLaunchArgument('slam',     default_value='true')
    x_spawn_arg     = DeclareLaunchArgument('x_spawn',  default_value='0.0')
    y_spawn_arg     = DeclareLaunchArgument('y_spawn',  default_value='-4.0')
    yaw_spawn_arg   = DeclareLaunchArgument('yaw_spawn',default_value='0.0')

    use_rviz  = LaunchConfiguration('use_rviz')
    use_slam  = LaunchConfiguration('slam')
    x_spawn   = LaunchConfiguration('x_spawn')
    y_spawn   = LaunchConfiguration('y_spawn')
    yaw_spawn = LaunchConfiguration('yaw_spawn')

    # ------------------------------------------------------------------ #
    # Process URDF / xacro                                                #
    # ------------------------------------------------------------------ #
    urdf_file = os.path.join(ursula_share, 'urdf', 'ursula.urdf.xacro')
    robot_description_raw = xacro.process_file(urdf_file).toxml()

    # ------------------------------------------------------------------ #
    # Gazebo                                                               #
    # ------------------------------------------------------------------ #
    world_file = os.path.join(ursula_share, 'worlds', 'ursula_maze.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items(),
    )

    # ------------------------------------------------------------------ #
    # Robot State Publisher                                                #
    # ------------------------------------------------------------------ #
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': True}]
    )

    # ------------------------------------------------------------------ #
    # Joint State Publisher                                                #
    # Publishes zero positions for fixed joints so RViz doesn't complain  #
    # ------------------------------------------------------------------ #
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'use_sim_time': True}]
    # )

    # ------------------------------------------------------------------ #
    # Spawn robot in Gazebo                                                #
    # Small delay gives Gazebo time to finish loading the world            #
    # ------------------------------------------------------------------ #
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_ursula',
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'ursula',
                    '-x', x_spawn,
                    '-y', y_spawn,
                    '-z', '0.12',   # Slight lift so wheels land cleanly
                    '-Y', yaw_spawn,
                ]
            )
        ]
    )

    # ------------------------------------------------------------------ #
    # SLAM Toolbox                                                         #
    # ------------------------------------------------------------------ #
    slam_toolbox = Node(
        condition=IfCondition(use_slam),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(ursula_share, 'config', 'slam_toolbox_params.yaml'),
            {'use_sim_time': True}
        ]
    )

    # ------------------------------------------------------------------ #
    # RViz2                                                                #
    # ------------------------------------------------------------------ #
    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(ursula_share, 'rviz', 'ursula.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        use_rviz_arg,
        use_slam_arg,
        x_spawn_arg,
        y_spawn_arg,
        yaw_spawn_arg,
        gazebo,
        robot_state_publisher,
        #joint_state_publisher,
        spawn_entity,
        slam_toolbox,
        rviz,
    ])
