#pragma once
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace futuraps {

class LocalMapFilterNode : public rclcpp::Node {
public:
  explicit LocalMapFilterNode(const rclcpp::NodeOptions &options);

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter>& params);

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // IO
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_out_;
  // Optional debug pubs
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_after_height_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_after_ror_;

  // Topics/frames
  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  bool transform_to_target_{true};

  // Height filter (pot removal)
  bool enable_height_filter_{true};
  double bottom_cut_m_{0.20};
  double top_cut_m_{10.0};

  // Outlier cleanup
  bool enable_radius_outlier_{true};
  double radius_{0.06};
  int min_neighbors_{8};

  // Voxel downsample
  bool enable_voxel_{false};
  double voxel_leaf_{0.01};

  // ExG (optional)
  bool use_exg_{false};
  bool normalize_rgb_{true};
  double exg_threshold_{0.05};
  bool drop_low_intensity_{true};
  double min_intensity_{0.05}; // [0,1] after normalization

  // Debug
  bool publish_debug_topics_{false};

  // QoS
  rclcpp::QoS qos_{rclcpp::SystemDefaultsQoS()};

  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

} // namespace futuraps
