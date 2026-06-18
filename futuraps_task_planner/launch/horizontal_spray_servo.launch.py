from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import os
import yaml


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    robot_name = "platform_ur10"
    moveit_config_pkg = "futuraps_platform_ur10_moveit_config"

    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=moveit_config_pkg)
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )

    task_planner_config_file = PathJoinSubstitution(
        [
            FindPackageShare("futuraps_task_planner"),
            "config",
            "horizontal_spray_servo.yaml",
        ]
    )

    servo_config_file = os.path.join(
        get_package_share_directory(moveit_config_pkg),
        "config",
        "servo.yaml",
    )
    sensor_config_file = os.path.join(
        get_package_share_directory(moveit_config_pkg),
        "config",
        "sensors_3d.yaml",
    )
    sensor_config = load_yaml(sensor_config_file)

    declared_args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("platform_motion_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="platform_link"),
        DeclareLaunchArgument("target_frame", default_value="platform_link"),
        DeclareLaunchArgument("boom_length", default_value="0.5"),
        DeclareLaunchArgument("standoff_m", default_value="0.4"),
        DeclareLaunchArgument("use_platform_pid", default_value="true"),
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
        DeclareLaunchArgument("perception_timeout_s", default_value="0.5"),
    ]

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            sensor_config,
            servo_config_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"moveit_servo.move_group_name": "ur10_arm"},
            {"moveit_servo.planning_frame": "platform_link"},
            {"moveit_servo.ee_frame_name": "tool0"},
            {"moveit_servo.robot_link_command_frame": "platform_link"},
            {"moveit_servo.command_out_topic": "/forward_position_controller/commands"},
            {"moveit_servo.command_out_type": "std_msgs/Float64MultiArray"},
            {"moveit_servo.publish_joint_positions": True},
            {"moveit_servo.publish_joint_velocities": False},
            {"moveit_servo.publish_joint_accelerations": False},
        ],
    )

    planner_node = Node(
        package="futuraps_task_planner",
        executable="horizontal_spray_servo",
        name="horizontal_spray_servo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            task_planner_config_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_frame": LaunchConfiguration("map_frame"),
                "platform_motion_frame": LaunchConfiguration("platform_motion_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "target_frame": LaunchConfiguration("target_frame"),
                "boom_length": LaunchConfiguration("boom_length"),
                "standoff_m": LaunchConfiguration("standoff_m"),
                "use_platform_pid": LaunchConfiguration("use_platform_pid"),
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
                "perception_timeout_s": LaunchConfiguration("perception_timeout_s"),
            },
        ],
    )

    return LaunchDescription(declared_args + [servo_node, planner_node])
