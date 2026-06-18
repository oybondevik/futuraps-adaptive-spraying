import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def _get(d, path, default=None):
    """Read nested dict values safely: _get(cfg, ['cam','x'], 0.0)."""
    cur = d
    for p in path:
        if not isinstance(cur, dict) or p not in cur:
            return default
        cur = cur[p]
    return cur


def _as_bool_str(v, default=False):
    """Return 'true'/'false' string for launch defaults."""
    if v is None:
        v = default
    return "true" if bool(v) else "false"


def generate_launch_description():
    # For substitutions used in IncludeLaunchDescription paths
    pkg = FindPackageShare("futuraps_perception")

    # ---- Load launch-defaults YAML (real filesystem path) ----
    pkg_share_fs = get_package_share_directory("futuraps_perception")
    cfg_path = os.path.join(pkg_share_fs, "config", "perception.yaml")

    cfg = {}
    if os.path.exists(cfg_path):
        with open(cfg_path, "r") as f:
            cfg = yaml.safe_load(f) or {}
    else:
        print(f"[futuraps_perception.launch.py] WARNING: config not found: {cfg_path}")

    # ---- Public args ----
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value=_as_bool_str(_get(cfg, ["use_sim_time"], True)),
    )
    show_visualization = DeclareLaunchArgument(
        "show_visualization",
        default_value=_as_bool_str(_get(cfg, ["show_visualization"], True)),
    )
    use_exg = DeclareLaunchArgument(
        "use_exg",
        default_value=_as_bool_str(_get(cfg, ["use_exg"], False)),
    )
    crop_input_topic = DeclareLaunchArgument(
        "crop_input_topic",
        default_value=str(
            _get(cfg, ["point_cloud_filter", "crop_input_topic"], "/cloud_obstacles")
        ),
    )
    crop_input_qos = DeclareLaunchArgument(
        "crop_input_qos",
        default_value=str(
            _get(cfg, ["point_cloud_filter", "crop_input_qos"], "transient_local")
        ),
    )
    crop_frame = DeclareLaunchArgument(
        "crop_frame",
        default_value=str(
            _get(cfg, ["point_cloud_filter", "crop_frame"], "platform_link")
        ),
    )
    local_map_frame = DeclareLaunchArgument(
        "local_map_frame",
        default_value=str(
            _get(cfg, ["point_cloud_filter", "local_map_frame"], "platform_link")
        ),
    )

    launch_rtabmap_arg = DeclareLaunchArgument("launch_rtabmap", default_value="true")

    # ---- Static TF: parent -> perception_base ----
    publish_perception_base_tf = DeclareLaunchArgument(
        "publish_perception_base_tf",
        default_value=_as_bool_str(_get(cfg, ["publish_perception_base_tf"], True)),
    )
    perception_base_parent = DeclareLaunchArgument(
        "perception_base_parent",
        default_value=str(_get(cfg, ["perception_base_parent"], "platform_link")),
    )
    perception_base_x = DeclareLaunchArgument(
        "perception_base_x",
        default_value=str(_get(cfg, ["perception_base", "x"], 0.0)),
    )
    perception_base_y = DeclareLaunchArgument(
        "perception_base_y",
        default_value=str(_get(cfg, ["perception_base", "y"], 0.0)),
    )
    perception_base_z = DeclareLaunchArgument(
        "perception_base_z",
        default_value=str(_get(cfg, ["perception_base", "z"], 0.0)),
    )
    perception_base_roll = DeclareLaunchArgument(
        "perception_base_roll",
        default_value=str(_get(cfg, ["perception_base", "roll"], 0.0)),
    )
    perception_base_pitch = DeclareLaunchArgument(
        "perception_base_pitch",
        default_value=str(_get(cfg, ["perception_base", "pitch"], 0.0)),
    )
    perception_base_yaw = DeclareLaunchArgument(
        "perception_base_yaw",
        default_value=str(_get(cfg, ["perception_base", "yaw"], 0.0)),
    )

    # ---- Optional static TF: parent -> camera ----
    publish_cam_tf = DeclareLaunchArgument(
        "publish_cam_tf",
        default_value=_as_bool_str(_get(cfg, ["publish_cam_tf"], True)),
    )
    cam_parent_frame = DeclareLaunchArgument(
        "cam_parent_frame",
        default_value=str(_get(cfg, ["cam", "parent_frame"], "base_link")),
    )
    cam_child_frame = DeclareLaunchArgument(
        "cam_child_frame",
        default_value=str(_get(cfg, ["cam", "child_frame"], "cam2_link")),
    )
    cam_x = DeclareLaunchArgument(
        "cam_x",
        default_value=str(_get(cfg, ["cam", "x"], 0.55)),
    )
    cam_y = DeclareLaunchArgument(
        "cam_y",
        default_value=str(_get(cfg, ["cam", "y"], 0.0)),
    )
    cam_z = DeclareLaunchArgument(
        "cam_z",
        default_value=str(_get(cfg, ["cam", "z"], 0.80)),
    )
    cam_roll = DeclareLaunchArgument(
        "cam_roll",
        default_value=str(_get(cfg, ["cam", "roll"], 1.5708)),
    )
    cam_pitch = DeclareLaunchArgument(
        "cam_pitch",
        default_value=str(_get(cfg, ["cam", "pitch"], 0.0)),
    )
    cam_yaw = DeclareLaunchArgument(
        "cam_yaw",
        default_value=str(_get(cfg, ["cam", "yaw"], 1.5708)),
    )

    # ---- RTAB-Map input selection + topics ----
    rgbd_mode = DeclareLaunchArgument(
        "rgbd_mode",
        default_value=_as_bool_str(_get(cfg, ["rtabmap", "rgbd_mode"], False)),
    )
    run_vo = DeclareLaunchArgument(
        "run_vo",
        default_value=_as_bool_str(_get(cfg, ["rtabmap", "run_vo"], False)),
    )

    rgb_topic = DeclareLaunchArgument(
        "rgb_topic",
        default_value=str(_get(cfg, ["rtabmap", "rgb_topic"], "/cam2/color/image_raw")),
    )
    depth_topic = DeclareLaunchArgument(
        "depth_topic",
        default_value=str(
            _get(
                cfg,
                ["rtabmap", "depth_topic"],
                "/cam2/aligned_depth_to_color/image_raw",
            )
        ),
    )
    camera_info_topic = DeclareLaunchArgument(
        "camera_info_topic",
        default_value=str(
            _get(cfg, ["rtabmap", "camera_info_topic"], "/cam2/color/camera_info")
        ),
    )
    rgbd_topic = DeclareLaunchArgument(
        "rgbd_topic",
        default_value=str(_get(cfg, ["rtabmap", "rgbd_topic"], "/cam2/rgbd_image_raw")),
    )
    gps_topic = DeclareLaunchArgument(
        "gps_topic",
        default_value=str(_get(cfg, ["rtabmap", "gps_topic"], "/emlid/fix")),
    )
    odom_topic = DeclareLaunchArgument(
        "odom_topic",
        default_value=str(_get(cfg, ["rtabmap", "odom_topic"], "/odom")),
    )
    imu_topic = DeclareLaunchArgument(
        "imu_topic",
        default_value=str(_get(cfg, ["rtabmap", "imu_topic"], "/imu/data")),
    )
    rtabmap_frame_id = DeclareLaunchArgument(
        "rtabmap_frame_id",
        default_value=str(_get(cfg, ["rtabmap", "frame_id"], "perception_base")),
    )
    rtabmap_odom_frame_id = DeclareLaunchArgument(
        "rtabmap_odom_frame_id",
        default_value=str(_get(cfg, ["rtabmap", "odom_frame_id"], "odom")),
    )
    rtabmap_map_frame_id = DeclareLaunchArgument(
        "rtabmap_map_frame_id",
        default_value=str(_get(cfg, ["rtabmap", "map_frame_id"], "map")),
    )

    # ---- Static tf for placing perception_base ----
    static_perception_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_perception_base_tf",
        # x y z roll pitch yaw parent child
        arguments=[
            LaunchConfiguration("perception_base_x"),
            LaunchConfiguration("perception_base_y"),
            LaunchConfiguration("perception_base_z"),
            LaunchConfiguration("perception_base_roll"),
            LaunchConfiguration("perception_base_pitch"),
            LaunchConfiguration("perception_base_yaw"),
            LaunchConfiguration("perception_base_parent"),
            "perception_base",
        ],
        # Avoid two-parent TF conflict when VO publishes odom->perception_base.
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("publish_perception_base_tf"),
                    "' == 'true' and ('",
                    LaunchConfiguration("run_vo"),
                    "' != 'true' or '",
                    LaunchConfiguration("rtabmap_frame_id"),
                    "' != 'perception_base')",
                ]
            )
        ),
        output="screen",
    )

    # ---- Optional static TF from parent -> camera ----
    static_cam_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_cam_tf",
        # x y z roll pitch yaw parent child
        arguments=[
            LaunchConfiguration("cam_x"),
            LaunchConfiguration("cam_y"),
            LaunchConfiguration("cam_z"),
            LaunchConfiguration("cam_roll"),
            LaunchConfiguration("cam_pitch"),
            LaunchConfiguration("cam_yaw"),
            LaunchConfiguration("cam_parent_frame"),
            LaunchConfiguration("cam_child_frame"),
        ],
        condition=IfCondition(LaunchConfiguration("publish_cam_tf")),
        output="screen",
    )

    # ---- Includes ----
    include_rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "includes", "rtabmap_local.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "rgbd_mode": LaunchConfiguration("rgbd_mode"),
            "run_vo": LaunchConfiguration("run_vo"),
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "rgbd_topic": LaunchConfiguration("rgbd_topic"),
            "gps_topic": LaunchConfiguration("gps_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "rtabmap_frame_id": LaunchConfiguration("rtabmap_frame_id"),
            "rtabmap_odom_frame_id": LaunchConfiguration("rtabmap_odom_frame_id"),
            "rtabmap_map_frame_id": LaunchConfiguration("rtabmap_map_frame_id"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_rtabmap")),
    )

    include_point_cloud_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg, "launch", "includes", "point_cloud_filter.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_exg": LaunchConfiguration("use_exg"),
            "crop_input_topic": LaunchConfiguration("crop_input_topic"),
            "crop_input_qos": LaunchConfiguration("crop_input_qos"),
            "crop_frame": LaunchConfiguration("crop_frame"),
            "local_map_frame": LaunchConfiguration("local_map_frame"),
        }.items(),
    )

    include_parameter_extractor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg, "launch", "includes", "parameter_extractor.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )

    include_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "includes", "visualization.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("show_visualization")),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )

    return LaunchDescription(
        [
            # Public args
            use_sim_time,
            show_visualization,
            use_exg,
            crop_input_topic,
            crop_input_qos,
            crop_frame,
            local_map_frame,
            launch_rtabmap_arg,
            rgbd_mode,
            run_vo,
            rgb_topic,
            depth_topic,
            camera_info_topic,
            rgbd_topic,
            gps_topic,
            odom_topic,
            imu_topic,
            rtabmap_frame_id,
            rtabmap_odom_frame_id,
            rtabmap_map_frame_id,
            # Must have TF args
            publish_perception_base_tf,
            perception_base_x,
            perception_base_y,
            perception_base_z,
            perception_base_roll,
            perception_base_pitch,
            perception_base_yaw,
            perception_base_parent,
            # Optional TF args
            publish_cam_tf,
            cam_parent_frame,
            cam_child_frame,
            cam_x,
            cam_y,
            cam_z,
            cam_roll,
            cam_pitch,
            cam_yaw,
            # Nodes
            static_cam_tf,
            static_perception_base_tf,
            # Includes
            include_rtabmap,
            include_point_cloud_filter,
            include_parameter_extractor,
            include_visualization,
        ]
    )
