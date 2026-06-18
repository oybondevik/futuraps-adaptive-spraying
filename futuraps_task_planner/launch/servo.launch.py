from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declared_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
    ]

    moveit_config = (
        MoveItConfigsBuilder(
            "ur10",
            package_name="futuraps_platform_ur10_moveit_config",
        )
        .robot_description(file_path="config/ur.urdf.xacro")
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "moveit_servo": {
                    # Frames
                    "planning_frame": "base_link",
                    "ee_frame_name": "tool0",
                    "robot_link_command_frame": "base_link",

                    # Input topics
                    "cartesian_command_in_topic": "/servo_node/delta_twist_cmds",
                    "joint_command_in_topic": "/servo_node/delta_joint_cmds",

                    # Output to your active controller
                    "command_out_topic": "/forward_position_controller/commands",
                    "command_out_type": "std_msgs/Float64MultiArray",

                    # Publish joint positions for forward_position_controller
                    "publish_joint_positions": True,
                    "publish_joint_velocities": False,
                    "publish_joint_accelerations": False,

                    # Servo behavior
                    "publish_period": 0.01,
                    "incoming_command_timeout": 0.5,
                    "num_outgoing_halt_msgs_to_publish": 4,

                    # Scaling
                    "linear_scale": 0.4,
                    "rotational_scale": 0.4,
                    "joint_scale": 0.4,

                    # Safety limits
                    "lower_singularity_threshold": 17.0,
                    "hard_stop_singularity_threshold": 30.0,
                    "leaving_singularity_threshold_multiplier": 2.0,
                    "joint_limit_margin": 0.1,

                    # Debug: disable collision checking first
                    "check_collisions": False,
                    "check_octomap_collisions": False,

                    # Collision params still included for completeness
                    "collision_check_rate": 10.0,
                    "self_collision_proximity_threshold": 0.01,
                    "scene_collision_proximity_threshold": 0.02,
                },
            },
        ],
    )

    return LaunchDescription(declared_args + [servo_node])
