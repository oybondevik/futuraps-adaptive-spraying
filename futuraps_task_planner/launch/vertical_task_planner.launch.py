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
        robot_name,
        package_name=moveit_config_pkg,
    ).to_moveit_configs()

    config_file = PathJoinSubstitution(
        [
            FindPackageShare("futuraps_task_planner"),
            "config",
            "vertical_pose_planner.yaml",
        ]
    )

    declared_args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # General planner settings
        DeclareLaunchArgument("planning_group", default_value="ur10_arm"),
        DeclareLaunchArgument("target_frame", default_value="platform_link"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="platform_link"),
        DeclareLaunchArgument("execute", default_value="true"),
        # MoveIt planning
        DeclareLaunchArgument("planning_time", default_value="1.5"),
        DeclareLaunchArgument("planning_attempts", default_value="3"),
        DeclareLaunchArgument("pipeline_id", default_value="ompl"),
        DeclareLaunchArgument("planner_id", default_value=""),
        DeclareLaunchArgument("max_velocity_scaling", default_value="0.15"),
        DeclareLaunchArgument("max_acceleration_scaling", default_value="0.15"),
        # Timing
        DeclareLaunchArgument("plan_period_s", default_value="0.5"),
        DeclareLaunchArgument("perception_dt_s", default_value="0.1"),
        DeclareLaunchArgument("wait_for_state_timeout_s", default_value="3.0"),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument("max_joint_state_age_s", default_value="1.0"),
        # IK
        DeclareLaunchArgument("ik_timeout_s", default_value="0.05"),
        DeclareLaunchArgument("ik_attempts", default_value="4"),
        # Perception services
        DeclareLaunchArgument("closest_srv", default_value="/get_closest_grid"),
        DeclareLaunchArgument("normal_srv", default_value="/get_global_normal"),
        DeclareLaunchArgument("perception_timeout_s", default_value="0.3"),
        # Closest-grid params
        DeclareLaunchArgument("cell_size", default_value="0.3"),
        DeclareLaunchArgument("rows", default_value="5"),
        DeclareLaunchArgument("cols", default_value="1"),
        DeclareLaunchArgument("x0", default_value="-0.15"),
        DeclareLaunchArgument("z0", default_value="0.2"),
        DeclareLaunchArgument("y_left_max", default_value="2.0"),
        DeclareLaunchArgument("y_right_max", default_value="2.0"),
        DeclareLaunchArgument("side", default_value="0"),
        DeclareLaunchArgument("front_percentile", default_value="0.01"),
        DeclareLaunchArgument("min_points_per_cell", default_value="20"),
        # Normal ROI
        DeclareLaunchArgument("normal_min_x", default_value="-0.5"),
        DeclareLaunchArgument("normal_max_x", default_value="0.5"),
        DeclareLaunchArgument("normal_min_y", default_value="-2.0"),
        DeclareLaunchArgument("normal_max_y", default_value="2.0"),
        DeclareLaunchArgument("normal_min_z", default_value="0.2"),
        DeclareLaunchArgument("normal_max_z", default_value="2.0"),
        # Target pose behavior
        DeclareLaunchArgument("standoff_m", default_value="0.20"),
        DeclareLaunchArgument("z_target", default_value="1.0"),
        DeclareLaunchArgument("filter_tau_s", default_value="0.20"),
        DeclareLaunchArgument("tool_axis_forward", default_value="2"),
        DeclareLaunchArgument("tool_roll_orientation", default_value="-1.57"),
        DeclareLaunchArgument("point_tool_into_surface", default_value="true"),
        # Replan / cooldown behavior
        DeclareLaunchArgument("replan_pos_threshold_m", default_value="0.03"),
        DeclareLaunchArgument("replan_angle_threshold_rad", default_value="0.20"),
        DeclareLaunchArgument("failed_plan_cooldown_s", default_value="1.0"),
        DeclareLaunchArgument("successful_plan_cooldown_s", default_value="0.25"),
        # Optional look-ahead
        DeclareLaunchArgument("platform_speed_mps", default_value="0.0"),
        DeclareLaunchArgument("lookahead_time_s", default_value="0.0"),
    ]

    node = Node(
        package="futuraps_task_planner",
        executable="vertical_pose_planner",
        name="vertical_pose_planner",
        output="screen",
        remappings=[
            ("joint_states", "/joint_states"),
        ],
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
                "target_frame": LaunchConfiguration("target_frame"),
                "map_frame": LaunchConfiguration("map_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "execute": LaunchConfiguration("execute"),
                "planning_time": LaunchConfiguration("planning_time"),
                "planning_attempts": LaunchConfiguration("planning_attempts"),
                "pipeline_id": LaunchConfiguration("pipeline_id"),
                "planner_id": LaunchConfiguration("planner_id"),
                "max_velocity_scaling": LaunchConfiguration("max_velocity_scaling"),
                "max_acceleration_scaling": LaunchConfiguration(
                    "max_acceleration_scaling"
                ),
                "plan_period_s": LaunchConfiguration("plan_period_s"),
                "perception_dt_s": LaunchConfiguration("perception_dt_s"),
                "wait_for_state_timeout_s": LaunchConfiguration(
                    "wait_for_state_timeout_s"
                ),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "max_joint_state_age_s": LaunchConfiguration("max_joint_state_age_s"),
                "ik_timeout_s": LaunchConfiguration("ik_timeout_s"),
                "ik_attempts": LaunchConfiguration("ik_attempts"),
                "closest_srv": LaunchConfiguration("closest_srv"),
                "normal_srv": LaunchConfiguration("normal_srv"),
                "perception_timeout_s": LaunchConfiguration("perception_timeout_s"),
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
                "normal_min_z": LaunchConfiguration("normal_min_z"),
                "normal_max_z": LaunchConfiguration("normal_max_z"),
                "standoff_m": LaunchConfiguration("standoff_m"),
                "z_target": LaunchConfiguration("z_target"),
                "filter_tau_s": LaunchConfiguration("filter_tau_s"),
                "tool_axis_forward": LaunchConfiguration("tool_axis_forward"),
                "tool_roll_orientation": LaunchConfiguration("tool_roll_orientation"),
                "point_tool_into_surface": LaunchConfiguration(
                    "point_tool_into_surface"
                ),
                "replan_pos_threshold_m": LaunchConfiguration("replan_pos_threshold_m"),
                "replan_angle_threshold_rad": LaunchConfiguration(
                    "replan_angle_threshold_rad"
                ),
                "failed_plan_cooldown_s": LaunchConfiguration("failed_plan_cooldown_s"),
                "successful_plan_cooldown_s": LaunchConfiguration(
                    "successful_plan_cooldown_s"
                ),
                "platform_speed_mps": LaunchConfiguration("platform_speed_mps"),
                "lookahead_time_s": LaunchConfiguration("lookahead_time_s"),
            },
        ],
    )

    return LaunchDescription(declared_args + [node])
