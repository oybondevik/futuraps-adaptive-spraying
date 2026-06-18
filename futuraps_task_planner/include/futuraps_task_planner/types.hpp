#pragma once

#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Vector3.h>

namespace futuraps_task_planner
{

struct PerceptionResult{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    std::vector<bool> found;
    std::vector<tf2::Vector3> normals;
    std::vector<bool> normal_valid;

    bool top_point_found{false};
    double top_x{0.0};
    double top_y{0.0};
    double top_z{0.0};
    double top_detected_max_z{0.0};
    uint32_t top_count{0};

    bool valid {false};
};

struct PerceptionConfig
{
  std::string closest_srv{"/get_closest_grid"};
  std::string normal_srv{"/get_global_normal"};
  std::string target_frame{"platform_link"};

  double cell_size{0.3};
  int rows{5};
  int normal_rows{5};
  int cols{1};
  double x0{-0.15};
  double z0{0.2};
  double y_left_max{2.0};
  double y_right_max{2.0};
  int side{0};
  double front_percentile{0.01};
  int min_points_per_cell{20};

  double normal_min_x{-0.5};
  double normal_max_x{0.5};
  double normal_min_y{-2.0};
  double normal_max_y{2.0};
  double normal_z_overlap{0.05};

  double timeout_s{0.5};

  std::string top_point_srv{"/get_top_canopy_point"};

  double top_min_x{-0.5};
  double top_max_x{0.5};
  double top_min_y{-2.0};
  double top_max_y{2.0};
  double top_min_z{0.0};
  double top_max_z{2.5};
  double top_band_height{0.10};
  double top_front_percentile{0.01};
  int top_min_points{5};
};

struct HorizontalPathConfig
{
  int rows{5};
  int cols{1};
  double standoff_m{0.4};
  int tool_axis_forward{2};
  double tool_roll_orientation{3.1415};
  bool point_tool_into_surface{true};

  double cell_size{0.3};
  double z0{0.0};

  double row_z_blend{0.35};          // 0.0 => pure row center, 1.0 => pure perception z
  double max_row_delta_y{0.25};      // reject large lateral jumps between neighboring rows
  double max_row_delta_z{0.40};      // reject large vertical jumps between neighboring rows
  double max_abs_y{1.20};            // reject waypoints too far from platform frame origin
  double max_z{1.60};                // cap top-row height to avoid unreachable poses

  bool add_top_waypoint{true};
  double top_waypoint_margin{0.03};
  double min_top_extension{0.03};
  double max_top_extension{0.40};

  // "perception_normals" keeps the legacy fixed-up orientation.
  // "perception_normals_path_tangent" uses normals for spray direction and path tangent for roll/yaw alignment.
  std::string orientation_mode{"perception_normals"};
  bool use_perception_yaw{false};
  double predefined_tilt_up_deg{12.0};
  double predefined_tilt_down_deg{12.0};
  double max_inward_angle_deg{25.0};
  double sample_spacing{0.02};

  // Smooth sharp local y-corners after reachability/platform repair.
  // Only position.y is modified; x, z, and orientation are preserved.
  bool local_y_smoothing_enabled{false};
  double local_y_smoothing_min_sharpness{0.025};
  int local_y_smoothing_radius{8};

  // Remove low, far-out endpoint waypoints after reachability/platform repair.
  bool prune_low_z_outward_waypoints_enabled{false};
  double prune_low_z_outward_max_z{0.35};
  double prune_low_z_outward_min_abs_y{1.0};
  int prune_low_z_outward_min_remaining{5};
};

struct PathFollowerConfig
{
  double pos_gain{1.5};
  double ori_gain{3.0};
  double max_linear_speed{1.5};
  double max_angular_speed{1.5};
  double pos_tolerance{0.05};
  double ori_tolerance{0.30};
  int lookahead_points{3};
  double v_min{0.25};
  double v_max{0.40};
  std::string path_frame{"platform_link"};
  std::string command_frame{"base_link"};
  std::string ee_frame{"tool0"};
};

struct ArmMotionConfig
{
  std::string planning_group{"ur10_arm"};
  double eef_step{0.01};
  double jump_threshold{0.0};
  bool avoid_collisions{true};
  double max_velocity_scaling{0.05};
  double max_acceleration_scaling{0.05};

  double cartesian_velocity_scaling{0.2};
  double cartesian_acceleration_scaling{0.2};

  std::vector<double> home_joint_values{
    -1.57, -1.80, -2.00, 0.80, 1.57, 0.0
  };
  double home_joint_tolerance{0.02};
};

struct PlatformMovingConfig
{
  std::string map_frame{"map"};
  std::string base_frame{"platform_link"};
  bool use_pid{true};
};



} // namespace futuraps_task_planner
