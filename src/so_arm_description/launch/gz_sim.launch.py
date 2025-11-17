#!/usr/bin/env python3
"""
Launch Gazebo (gz-sim) with SO-ARM robot using ROS2 integration
For ROS2 Humble - uses ros-gz packages
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_so_arm = get_package_share_directory('so_arm_urdf')

    # Path to URDF
    urdf_file = os.path.join(pkg_so_arm, 'urdf', 'so_arm_urdf.urdf')
    mesh_dir = os.path.join(pkg_so_arm, 'meshes')

    # Read URDF and fix mesh paths for Gazebo
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Replace package:// with file:// for Gazebo
    robot_desc_fixed = robot_desc.replace(
        'package://so_arm_urdf/meshes/',
        f'file://{mesh_dir}/'
    )

    # Save fixed URDF to temp location
    import tempfile
    temp_urdf = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf')
    temp_urdf.write(robot_desc_fixed)
    temp_urdf.close()

    # Launch Gazebo with ROS integration
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # Spawn robot - delay to let Gazebo start
    spawn_entity = TimerAction(
        period=8.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', temp_urdf.name,
                '-name', 'so_arm',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.0'  # On the ground
            ],
            output='screen'
        )]
    )

    # Bridge for clock
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Joint State Publisher GUI for manual control
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge_clock,
        joint_state_publisher_gui,
    ])
