from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    robot_name = "platform_ur10"
    moveit_config_pkg = "futuraps_platform_ur10_moveit_config"

    moveit_config = MoveItConfigsBuilder(
        robot_name, package_name=moveit_config_pkg
    ).to_moveit_configs()

    config_file = PathJoinSubstitution(
        [FindPackageShare("futuraps_task_planner"), "config", "horizontal_spray.yaml"]
    )

    declared_args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("planning_group", default_value="ur10_arm"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="platform_link"),
        DeclareLaunchArgument("target_frame", default_value="platform_link"),
        DeclareLaunchArgument("boom_length", default_value="0.5"),
        DeclareLaunchArgument("standoff_m", default_value="0.4"),
        DeclareLaunchArgument("eef_step", default_value="0.01"),
        DeclareLaunchArgument("jump_threshold", default_value="0.0"),
        DeclareLaunchArgument("avoid_collisions", default_value="true"),
        DeclareLaunchArgument("max_velocity_scaling", default_value="0.2"),
        DeclareLaunchArgument("max_acceleration_scaling", default_value="0.2"),
        DeclareLaunchArgument("cartesian_velocity_scaling", default_value="0.15"),
        DeclareLaunchArgument("cartesian_acceleration_scaling", default_value="0.15"),
        DeclareLaunchArgument("cell_size", default_value="0.3"),
        DeclareLaunchArgument("rows", default_value="6"),
        DeclareLaunchArgument("cols", default_value="5"),
        DeclareLaunchArgument("x0", default_value="-0.75"),
        DeclareLaunchArgument("z0", default_value="0.20"),
        DeclareLaunchArgument("y_left_max", default_value="2.0"),
        DeclareLaunchArgument("y_right_max", default_value="2.0"),
        DeclareLaunchArgument("side", default_value="0"),
        DeclareLaunchArgument("front_percentile", default_value="0.01"),
        DeclareLaunchArgument("min_points_per_cell", default_value="20"),
        DeclareLaunchArgument("normal_min_x", default_value="-0.5"),
        DeclareLaunchArgument("normal_max_x", default_value="0.5"),
        DeclareLaunchArgument("normal_min_y", default_value="-2.0"),
        DeclareLaunchArgument("normal_max_y", default_value="2.0"),
        DeclareLaunchArgument("normal_z_overlap", default_value="0.05"),
    ]

    node = Node(
        package="futuraps_task_planner",
        executable="horizontal_spray",
        name="horizontal_spray",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            config_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "planning_group": LaunchConfiguration("planning_group"),
                "map_frame": LaunchConfiguration("map_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "target_frame": LaunchConfiguration("target_frame"),
                "boom_length": LaunchConfiguration("boom_length"),
                "standoff_m": LaunchConfiguration("standoff_m"),
                "eef_step": LaunchConfiguration("eef_step"),
                "jump_threshold": LaunchConfiguration("jump_threshold"),
                "avoid_collisions": LaunchConfiguration("avoid_collisions"),
                "max_velocity_scaling": LaunchConfiguration("max_velocity_scaling"),
                "max_acceleration_scaling": LaunchConfiguration(
                    "max_acceleration_scaling"
                ),
                "cartesian_velocity_scaling": LaunchConfiguration(
                    "cartesian_velocity_scaling"
                ),
                "cartesian_acceleration_scaling": LaunchConfiguration(
                    "cartesian_acceleration_scaling"
                ),
                "cell_size": LaunchConfiguration("cell_size"),
                "rows": LaunchConfiguration("rows"),
                "cols": LaunchConfiguration("cols"),
                "x0": LaunchConfiguration("x0"),
                "z0": LaunchConfiguration("z0"),
                "y_left_max": LaunchConfiguration("y_left_max"),
                "y_right_max": LaunchConfiguration("y_right_max"),
                "side": LaunchConfiguration("side"),
                "front_percentile": LaunchConfiguration("front_percentile"),
                "min_points_per_cell": LaunchConfiguration("min_points_per_cell"),
                "normal_min_x": LaunchConfiguration("normal_min_x"),
                "normal_max_x": LaunchConfiguration("normal_max_x"),
                "normal_min_y": LaunchConfiguration("normal_min_y"),
                "normal_max_y": LaunchConfiguration("normal_max_y"),
                "normal_z_overlap": LaunchConfiguration("normal_z_overlap"),
            },
        ],
    )

    return LaunchDescription(declared_args + [node])
