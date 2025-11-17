import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Load MoveIt config
    moveit_config = MoveItConfigsBuilder("so_arm_urdf", package_name="so_arm_moveit_config").to_moveit_configs()

    # Generate demo launch with use_sim_time parameter injected
    demo_launch = generate_demo_launch(moveit_config)

    # Add use_sim_time parameter to all Node actions
    for entity in demo_launch.entities:
        if isinstance(entity, Node):
            # Get existing parameters or create empty list
            existing_params = getattr(entity, 'parameters', None)
            if existing_params is None:
                entity.parameters = [{"use_sim_time": use_sim_time}]
            else:
                entity.parameters.append({"use_sim_time": use_sim_time})

    return LaunchDescription(declared_arguments + demo_launch.entities)
