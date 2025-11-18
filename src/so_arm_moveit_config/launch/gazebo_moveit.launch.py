#!/usr/bin/env python3
"""
Combined launch file for Gazebo + MoveIt
Launches:
1. Gazebo with robot and ros2_control
2. MoveIt move_group for motion planning
3. RViz with MoveIt plugin
4. Automatically fixes use_sim_time parameter for move_group

Usage: ros2 launch so_arm_moveit_config gazebo_moveit.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    pkg_so_arm_urdf = get_package_share_directory('so_arm_urdf')
    pkg_moveit_config = get_package_share_directory('so_arm_moveit_config')

    # Path to global use_sim_time params file
    use_sim_time_params = os.path.join(pkg_moveit_config, 'config', 'use_sim_time_global.yaml')

    # Launch Gazebo with robot and ros2_control
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_so_arm_urdf, 'launch', 'gz_sim.launch.py')
        )
    )

    # Launch MoveIt move_group (delayed to let Gazebo start)
    move_group = TimerAction(
        period=12.0,  # Wait for Gazebo and controllers to be ready
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_moveit_config, 'launch', 'move_group.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'ros2_control_params_file': use_sim_time_params
            }.items()
        )]
    )

    # Launch RViz with MoveIt plugin (delayed to let move_group start)
    rviz = TimerAction(
        period=15.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_moveit_config, 'launch', 'moveit_rviz.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items()
        )]
    )

    # Automatically fix use_sim_time parameter after move_group starts
    # This is needed because generate_move_group_launch() doesn't properly
    # propagate the use_sim_time parameter in ROS2 Jazzy
    fix_use_sim_time = TimerAction(
        period=18.0,  # Wait for move_group to fully initialize (12s + 6s buffer)
        actions=[
            LogInfo(msg="Applying use_sim_time parameter fix..."),
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'ros2 param set /move_group use_sim_time true && '
                    'ros2 param set /controller_manager use_sim_time true && '
                    'ros2 param set /arm_controller use_sim_time true && '
                    'ros2 param set /joint_state_broadcaster use_sim_time true && '
                    'ros2 param set /robot_state_publisher use_sim_time true && '
                    'echo "✓ use_sim_time parameters applied successfully"'
                ],
                output='screen',
                shell=False
            )
        ]
    )

    return LaunchDescription([
        # Set ROS_USE_SIM_TIME environment variable for all nodes
        SetEnvironmentVariable('ROS_USE_SIM_TIME', '1'),
        # Set ROS parameter server default
        SetEnvironmentVariable('ROS_PARAMETER_FILE', use_sim_time_params),
        gazebo,
        move_group,
        rviz,
        fix_use_sim_time,  # Automatic parameter fix
    ])
