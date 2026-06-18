from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg = FindPackageShare("futuraps_perception")

    canopy_density_cfg = PathJoinSubstitution(
        [pkg, "config", "canopy_density", "canopy_density.yaml"]
    )
    visualization_cfg = PathJoinSubstitution(
        [pkg, "config", "visualization", "visualization.yaml"]
    )

    nodes = [
        # ------- Closest Point Visualizer -------
        Node(
            package="futuraps_perception",
            executable="closest_point_visualizer",
            name="closest_point_visualizer",
            output="screen",
            parameters=[visualization_cfg, {"use_sim_time": use_sim_time}],
        ),
        # ------- Canopy Density Visualizer -------
        Node(
            package="futuraps_perception",
            executable="canopy_density_visualizer",
            name="canopy_density_visualizer",
            output="screen",
            parameters=[canopy_density_cfg, {"use_sim_time": use_sim_time}],
        ),
        # -------- Surface Normal Visualizer -------
        Node(
            package="futuraps_perception",
            executable="surface_normal_visualizer",
            name="surface_normal_visualizer",
            output="screen",
            parameters=[visualization_cfg, {"use_sim_time": use_sim_time}],
        ),
        # ------- RViz2 -------
        SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1"),
        SetEnvironmentVariable(name="QT_XCB_GL_INTEGRATION", value="none"),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [
                        pkg,
                        "rviz",
                        "futuraps.rviz",
                    ]
                ),
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(
        [
            *nodes,
        ]
    )
