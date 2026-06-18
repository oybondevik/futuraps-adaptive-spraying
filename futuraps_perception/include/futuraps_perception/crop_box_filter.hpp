#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "visualization_msgs/msg/marker.hpp"

namespace futuraps
{

class CropBoxFilterNode : public rclcpp::Node
{
public:
  explicit CropBoxFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Dynamic params
  rcl_interfaces::msg::SetParametersResult onParamSet(const std::vector<rclcpp::Parameter> & params);

  // Main callback
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Helper
  void publishMarker(); 

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // IO
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Params
  std::string output_frame_;
  double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
  double leaf_size_;
  bool use_latest_tf_;
  double tf_timeout_sec_;

  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

} // namespace futuraps
