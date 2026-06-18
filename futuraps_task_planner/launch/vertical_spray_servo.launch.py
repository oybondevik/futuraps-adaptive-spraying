from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    auto_start_pid = LaunchConfiguration("auto_start_pid")
    servo_command_out_topic = LaunchConfiguration("servo_command_out_topic")
    servo_command_out_type = LaunchConfiguration("servo_command_out_type")

    declared_args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("auto_start_pid", default_value="true"),
        DeclareLaunchArgument(
            "servo_command_out_topic",
            default_value="/forward_position_controller/commands",
            description="MoveIt Servo output topic.",
        ),
        DeclareLaunchArgument(
            "servo_command_out_type",
            default_value="std_msgs/Float64MultiArray",
            description="MoveIt Servo output message type.",
        ),
    ]

    moveit_config = MoveItConfigsBuilder(
        "platform_ur10", package_name="futuraps_platform_ur10_moveit_config"
    ).to_moveit_configs()

    servo_yaml = PathJoinSubstitution(
        [
            FindPackageShare("futuraps_platform_ur10_moveit_config"),
            "config",
            "servo.yaml",
        ]
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            servo_yaml,
            {"use_sim_time": use_sim_time},
            {"moveit_servo.move_group_name": "ur10_arm"},
            {"moveit_servo.planning_frame": "platform_link"},
            {"moveit_servo.ee_frame_name": "tool0"},
            {"moveit_servo.robot_link_command_frame": "platform_link"},
            {
                "moveit_servo.command_out_topic": ParameterValue(
                    servo_command_out_topic, value_type=str
                )
            },
            {
                "moveit_servo.command_out_type": ParameterValue(
                    servo_command_out_type, value_type=str
                )
            },
            {"moveit_servo.publish_joint_positions": True},
            {"moveit_servo.publish_joint_velocities": False},
            {"moveit_servo.publish_joint_accelerations": False},
        ],
    )

    controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("futuraps_task_planner"),
            "config",
            "vertical_spray_servo.yaml",
        ]
    )

    vertical_spray_servo = Node(
        package="futuraps_task_planner",
        executable="vertical_spray_servo",
        name="vertical_spray_servo",
        output="screen",
        parameters=[
            controller_yaml,
            {
                "use_sim_time": use_sim_time,
                "auto_start_pid": auto_start_pid,
            },
        ],
    )

    return LaunchDescription(declared_args + [servo_node, vertical_spray_servo])
