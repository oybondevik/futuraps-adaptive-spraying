from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    _use_sim_time = LaunchConfiguration("use_sim_time")
    _rgbd_mode = LaunchConfiguration("rgbd_mode")
    _run_vo = LaunchConfiguration("run_vo")
    _frame_id = LaunchConfiguration("rtabmap_frame_id")
    _odom_frame_id = LaunchConfiguration("rtabmap_odom_frame_id")
    _map_frame_id = LaunchConfiguration("rtabmap_map_frame_id")
    _rgb = LaunchConfiguration("rgb_topic")
    _depth = LaunchConfiguration("depth_topic")
    _info = LaunchConfiguration("camera_info_topic")
    _rgbd = LaunchConfiguration("rgbd_topic")
    _gps = LaunchConfiguration("gps_topic")
    _odom = LaunchConfiguration("odom_topic")
    _imu = LaunchConfiguration("imu_topic")

    pkg = FindPackageShare("futuraps_perception")
    vo_cfg = PathJoinSubstitution([pkg, "config", "rtabmap", "rtabmap_vo.yaml"])
    slam_cfg = PathJoinSubstitution([pkg, "config", "rtabmap", "rtabmap_slam.yaml"])

    vo_node = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rtabmap_odom",
        namespace="vo",
        output="screen",
        condition=IfCondition(_run_vo),
        parameters=[
            vo_cfg,
            {"use_sim_time": _use_sim_time},
            {"subscribe_rgbd": _rgbd_mode},
            {"frame_id": _frame_id},
            {"odom_frame_id": _odom_frame_id},
        ],
        remappings=[
            ("rgbd/image", _rgbd),
            ("rgb/image", _rgb),
            ("depth/image", _depth),
            ("rgb/camera_info", _info),
            ("odom", _odom),
        ],
    )

    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            slam_cfg,
            {"use_sim_time": _use_sim_time},
            {"delete_db_on_start": True},
            {"subscribe_rgbd": _rgbd_mode},
            {"frame_id": _frame_id},
            {"odom_frame_id": _odom_frame_id},
            {"map_frame_id": _map_frame_id},
        ],
        remappings=[
            ("rgbd/image", _rgbd),
            ("rgb/image", _rgb),
            ("depth/image", _depth),
            ("rgb/camera_info", _info),
            ("odom", _odom),
            ("gps/fix", _gps),
            ("imu", _imu),
        ],
    )

    return LaunchDescription([vo_node, rtabmap])
