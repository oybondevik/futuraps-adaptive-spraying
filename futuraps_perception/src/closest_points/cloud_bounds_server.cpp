#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <limits>
#include "futuraps_perception/srv/get_cloud_bounds.hpp"

using futuraps_perception::srv::GetCloudBounds;

class CloudBoundsServer : public rclcpp::Node {
public:
  CloudBoundsServer() : Node("cloud_bounds_server")
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/local_map_filtered");
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg){ latest_ = msg; });

    srv_ = create_service<GetCloudBounds>(
      "get_cloud_bounds",
      [this](const std::shared_ptr<GetCloudBounds::Request> req,
             std::shared_ptr<GetCloudBounds::Response> resp)
      {
        on_request(req, resp);
      });

    RCLCPP_INFO(get_logger(), "cloud_bounds_server up. Sub '%s', srv 'get_cloud_bounds'",
                cloud_topic_.c_str());
  }

private:
  void on_request(const std::shared_ptr<GetCloudBounds::Request> req,
                  std::shared_ptr<GetCloudBounds::Response> resp)
  {
    (void)req;
    resp->min_x = resp->min_y = resp->min_z =  std::numeric_limits<float>::infinity();
    resp->max_x = resp->max_y = resp->max_z = -std::numeric_limits<float>::infinity();
    resp->count = 0;

    if (!latest_) return;

    if (!req->frame_id.empty() && req->frame_id != latest_->header.frame_id) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Requested frame '%s' but cloud is in '%s' (no TF).",
        req->frame_id.c_str(), latest_->header.frame_id.c_str());
    }

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*latest_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*latest_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*latest_, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const float x = *it_x, y = *it_y, z = *it_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      resp->count++;
      resp->min_x = std::min(resp->min_x, x);
      resp->min_y = std::min(resp->min_y, y);
      resp->min_z = std::min(resp->min_z, z);
      resp->max_x = std::max(resp->max_x, x);
      resp->max_y = std::max(resp->max_y, y);
      resp->max_z = std::max(resp->max_z, z);
    }
  }

  std::string cloud_topic_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Service<GetCloudBounds>::SharedPtr srv_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudBoundsServer>());
  rclcpp::shutdown();
  return 0;
}
