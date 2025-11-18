import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def launch_setup(context, *args, **kwargs):
    # Get use_sim_time value
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time_bool = use_sim_time.lower() in ['true', '1', 'yes']

    # Load MoveIt config with joint limits
    moveit_config = (
        MoveItConfigsBuilder("so_arm_urdf", package_name="so_arm_moveit_config")
        .robot_description(file_path="config/so_arm_urdf.urdf")
        .robot_description_semantic(file_path="config/so_arm_urdf.srdf")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Generate demo launch
    demo_launch = generate_demo_launch(moveit_config)

    # Add use_sim_time parameter to all Node actions
    for entity in demo_launch.entities:
        if isinstance(entity, Node):
            # Get existing parameters or create empty list
            existing_params = getattr(entity, 'parameters', None)
            if existing_params is None:
                entity.parameters = [{"use_sim_time": use_sim_time_bool}]
            else:
                # Convert to list if it's not already
                if not isinstance(existing_params, list):
                    existing_params = [existing_params]
                # Add use_sim_time to parameters
                existing_params.append({"use_sim_time": use_sim_time_bool})
                entity.parameters = existing_params

    return demo_launch.entities


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time (set to true when using Gazebo)",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
