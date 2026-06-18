from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_exg = LaunchConfiguration("use_exg")
    crop_input_topic = LaunchConfiguration("crop_input_topic")
    crop_input_qos = LaunchConfiguration("crop_input_qos")
    crop_frame = LaunchConfiguration("crop_frame")
    local_map_frame = LaunchConfiguration("local_map_frame")

    pkg = FindPackageShare("futuraps_perception")

    crop_box_filter_cfg = PathJoinSubstitution(
        [pkg, "config", "pointcloud_filters", "crop_box_filter.yaml"]
    )
    local_filter_cfg = PathJoinSubstitution(
        [pkg, "config", "pointcloud_filters", "local_map_filter.yaml"]
    )

    nodes = [
        # ------- Crop Box -------
        Node(
            package="futuraps_perception",
            executable="crop_box_filter_node",
            name="crop_box_filter",
            output="screen",
            parameters=[
                crop_box_filter_cfg,
                {
                    "use_sim_time": use_sim_time,
                    "input_qos": crop_input_qos,
                    "output_frame": crop_frame,
                    "publish_frame": crop_frame,
                },
            ],
            remappings=[
                ("input", crop_input_topic),
                ("output", "/crop_box_filtered_cloud"),
            ],
        ),
        # ------- Local Map Filter -------
        Node(
            package="futuraps_perception",
            executable="local_map_filter_node",
            name="local_map_filter",
            output="screen",
            parameters=[
                local_filter_cfg,
                {
                    "use_sim_time": use_sim_time,
                    "use_exg": use_exg,
                    "target_frame": local_map_frame,
                },
            ],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("crop_input_topic", default_value="/cloud_obstacles"),
            DeclareLaunchArgument("crop_input_qos", default_value="transient_local"),
            DeclareLaunchArgument("crop_frame", default_value="platform_link"),
            DeclareLaunchArgument("local_map_frame", default_value="platform_link"),
            *nodes,
        ]
    )
