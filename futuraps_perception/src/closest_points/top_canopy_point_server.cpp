#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "futuraps_perception/srv/get_top_canopy_point.hpp"

using PointT = pcl::PointXYZ;
using GetTopCanopyPoint = futuraps_perception::srv::GetTopCanopyPoint;

class TopCanopyPointServer : public rclcpp::Node
{
public:
  TopCanopyPointServer()
  : Node("top_canopy_point_server"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/local_map_filtered");
    default_frame_ = declare_parameter<std::string>("frame_id", "perception_base");

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg)
      {
        last_cloud_ = msg;
      });

    srv_ = create_service<GetTopCanopyPoint>(
      "get_top_canopy_point",
      std::bind(
        &TopCanopyPointServer::onRequest,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "top_canopy_point_server up. Sub '%s', srv 'get_top_canopy_point'",
      cloud_topic_.c_str());
  }

private:
  struct Candidate
  {
    float x;
    float y;
    float z;
  };

  void onRequest(
    const std::shared_ptr<GetTopCanopyPoint::Request> req,
    std::shared_ptr<GetTopCanopyPoint::Response> res)
  {
    res->found = false;
    res->x = 0.0f;
    res->y = 0.0f;
    res->z = 0.0f;
    res->count = 0;
    res->detected_max_z = -std::numeric_limits<float>::infinity();

    if (!last_cloud_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No point cloud received yet");
      return;
    }

    const std::string target_frame =
      req->frame_id.empty() ? default_frame_ : req->frame_id;

    pcl::PointCloud<PointT> cloud_target;

    if (last_cloud_->header.frame_id == target_frame) {
      pcl::fromROSMsg(*last_cloud_, cloud_target);
    } else {
      try {
        const auto tf = tf_buffer_.lookupTransform(
          target_frame,
          last_cloud_->header.frame_id,
          tf2::TimePointZero);

        const Eigen::Isometry3d T = tf2::transformToEigen(tf);

        pcl::PointCloud<PointT> cloud_in;
        pcl::fromROSMsg(*last_cloud_, cloud_in);

        cloud_target.reserve(cloud_in.size());

        for (const auto& p : cloud_in.points) {
          if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
          }

          const Eigen::Vector3d v(p.x, p.y, p.z);
          const Eigen::Vector3d w = T * v;

          cloud_target.emplace_back(
            PointT{
              static_cast<float>(w.x()),
              static_cast<float>(w.y()),
              static_cast<float>(w.z())});
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(
          get_logger(),
          "TF to '%s' unavailable: %s",
          target_frame.c_str(),
          e.what());
        return;
      }
    }

    if (cloud_target.empty()) {
      RCLCPP_WARN(get_logger(), "TopCanopyPoint: transformed cloud is empty");
      return;
    }

    const float min_x = req->min_x;
    const float max_x = req->max_x;
    const float min_y = req->min_y;
    const float max_y = req->max_y;
    const float min_z = req->min_z;
    const float max_z_limit = req->max_z;

    float detected_max_z = -std::numeric_limits<float>::infinity();

    for (const auto& p : cloud_target.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }

      if (p.x < min_x || p.x > max_x ||
          p.y < min_y || p.y > max_y ||
          p.z < min_z || p.z > max_z_limit)
      {
        continue;
      }

      detected_max_z = std::max(detected_max_z, p.z);
    }

    if (!std::isfinite(detected_max_z)) {
      RCLCPP_WARN(get_logger(), "TopCanopyPoint: no finite points inside search bounds");
      return;
    }

    const float top_band_height =
      std::max(0.01f, req->top_band_height);

    const float z_band_min = detected_max_z - top_band_height;

    std::vector<Candidate> candidates;
    candidates.reserve(cloud_target.size());

    for (const auto& p : cloud_target.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }

      if (p.x < min_x || p.x > max_x ||
          p.y < min_y || p.y > max_y ||
          p.z < min_z || p.z > max_z_limit)
      {
        continue;
      }

      if (p.z < z_band_min) {
        continue;
      }

      candidates.push_back(Candidate{p.x, p.y, p.z});
    }

    res->detected_max_z = detected_max_z;
    res->count = static_cast<uint32_t>(candidates.size());

    if (candidates.size() < req->min_points) {
      RCLCPP_WARN(
        get_logger(),
        "TopCanopyPoint: too few top-band points: %zu < %u",
        candidates.size(),
        req->min_points);
      return;
    }

    std::sort(
      candidates.begin(),
      candidates.end(),
      [](const Candidate& a, const Candidate& b)
      {
        return a.y < b.y;
      });

    double p = std::clamp(static_cast<double>(req->front_percentile), 0.0, 0.5);

    // If you are spraying from the positive-y side and want closest/front-most
    // points, you may need the opposite percentile. For now this picks low-y.
    const size_t idx = std::min(
      candidates.size() - 1,
      static_cast<size_t>(std::floor(p * static_cast<double>(candidates.size() - 1))));

    const Candidate selected = candidates[idx];

    // Use selected y/x for lateral placement, but detected_max_z for height.
    res->found = true;
    res->x = selected.x;
    res->y = selected.y;
    res->z = detected_max_z;
    res->count = static_cast<uint32_t>(candidates.size());

    RCLCPP_INFO(
      get_logger(),
      "TopCanopyPoint: found top point x=%.3f y=%.3f z=%.3f, detected_max_z=%.3f, count=%u",
      res->x,
      res->y,
      res->z,
      res->detected_max_z,
      res->count);
  }

  std::string cloud_topic_;
  std::string default_frame_;

  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Service<GetTopCanopyPoint>::SharedPtr srv_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopCanopyPointServer>());
  rclcpp::shutdown();
  return 0;
}