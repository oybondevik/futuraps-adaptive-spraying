from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # TODO: set these to your MoveIt config
    robot_name = "platform_ur10"
    moveit_config_pkg = "futuraps_platform_ur10_moveit_config"

    moveit_config = MoveItConfigsBuilder(
        robot_name, package_name=moveit_config_pkg
    ).to_moveit_configs()

    declared_args = [
        DeclareLaunchArgument("planning_group", default_value="ur10_arm"),
        DeclareLaunchArgument("target_frame", default_value="platform_link"),
        DeclareLaunchArgument("x", default_value="0.4"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.55"),
        DeclareLaunchArgument("roll", default_value="0.0"),
        DeclareLaunchArgument("pitch", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("planning_time", default_value="5.0"),
        DeclareLaunchArgument("planning_attempts", default_value="5"),
        DeclareLaunchArgument("retries", default_value="2"),
        DeclareLaunchArgument("execute", default_value="true"),
        DeclareLaunchArgument("position_only", default_value="false"),
        DeclareLaunchArgument("pipeline_id", default_value="ompl"),
        DeclareLaunchArgument("planner_id", default_value=""),
        DeclareLaunchArgument("max_velocity_scaling", default_value="0.2"),
        DeclareLaunchArgument("max_acceleration_scaling", default_value="0.2"),
        DeclareLaunchArgument("wait_for_state_timeout_s", default_value="3.0"),
    ]

    node = Node(
        package="futuraps_task_planner",
        executable="pose_plan_execute",
        name="pose_plan_execute",
        output="screen",
        remappings=[
            ("joint_states", "/joint_states"),
        ],
        parameters=[
            # MoveIt model params (critical)
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            # Runtime params
            {
                "planning_group": LaunchConfiguration("planning_group"),
                "target_frame": LaunchConfiguration("target_frame"),
                "x": LaunchConfiguration("x"),
                "y": LaunchConfiguration("y"),
                "z": LaunchConfiguration("z"),
                "roll": LaunchConfiguration("roll"),
                "pitch": LaunchConfiguration("pitch"),
                "yaw": LaunchConfiguration("yaw"),
                "planning_time": LaunchConfiguration("planning_time"),
                "planning_attempts": LaunchConfiguration("planning_attempts"),
                "retries": LaunchConfiguration("retries"),
                "execute": LaunchConfiguration("execute"),
                "position_only": LaunchConfiguration("position_only"),
                "pipeline_id": LaunchConfiguration("pipeline_id"),
                "planner_id": LaunchConfiguration("planner_id"),
                "max_velocity_scaling": LaunchConfiguration("max_velocity_scaling"),
                "max_acceleration_scaling": LaunchConfiguration(
                    "max_acceleration_scaling"
                ),
                "wait_for_state_timeout_s": LaunchConfiguration(
                    "wait_for_state_timeout_s"
                ),
            },
        ],
    )

    return LaunchDescription(declared_args + [node])
