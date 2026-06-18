from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg = FindPackageShare("futuraps_perception")
    parameter_extractor_cfg = PathJoinSubstitution(
        [pkg, "config", "parameter_extractor", "parameter_extractor.yaml"]
    )
    canopy_density_cfg = PathJoinSubstitution(
        [pkg, "config", "canopy_density", "canopy_density.yaml"]
    )

    nodes = [
        # -------- Clolsest Point Extractor Server --------
        Node(
            package="futuraps_perception",
            executable="closest_grid_server",
            name="closest_grid_server",
            output="screen",
            parameters=[parameter_extractor_cfg, {"use_sim_time": use_sim_time}],
        ),
        # -------- Canopy Density Estimator Server --------
        Node(
            package="futuraps_perception",
            executable="canopy_density_server",
            name="canopy_density_server",
            output="screen",
            parameters=[canopy_density_cfg, {"use_sim_time": use_sim_time}],
        ),
        # Cloud bounds server
        Node(
            package="futuraps_perception",
            executable="cloud_bounds_server",
            name="cloud_bounds_server",
            output="screen",
            parameters=[
                parameter_extractor_cfg,
                {
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
        # Top canopy point server
        Node(
            package="futuraps_perception",
            executable="top_canopy_point_server",
            name="top_canopy_point_server",
            output="screen",
            parameters=[
                parameter_extractor_cfg,
                {"use_sim_time": use_sim_time},
            ],
        ),
        # -------- Surface Normal Estimator Node --------
        Node(
            package="futuraps_perception",
            executable="surface_normal_server",
            name="surface_normal_server",
            output="screen",
            parameters=[
                parameter_extractor_cfg,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ]

    return LaunchDescription(
        [
            *nodes,
        ]
    )
