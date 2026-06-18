#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Dense>
#include <optional>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>

#include "futuraps_perception/srv/get_global_normal.hpp"

class GlobalNormalServer : public rclcpp::Node {
public:
  GlobalNormalServer()
  : rclcpp::Node("surface_normal_server")
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/local_map_filtered");
    frame_id_    = declare_parameter<std::string>("frame_id",    "base_link");
    min_points_  = declare_parameter<int>("min_points", 50); // require at least N points to trust

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&GlobalNormalServer::cloud_callback, this, std::placeholders::_1)
    );

    srv_ = create_service<futuraps_perception::srv::GetGlobalNormal>(
      "get_global_normal",
      std::bind(
        &GlobalNormalServer::on_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

    RCLCPP_INFO(get_logger(),
      "surface_normal_server up. Subscribing to '%s', answering on 'get_global_normal', assuming frame '%s'",
      cloud_topic_.c_str(), frame_id_.c_str());
  }

private:
  // ---------- Callbacks ----------

  void cloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    latest_cloud_ = msg;
  }

  void on_request(
    const futuraps_perception::srv::GetGlobalNormal::Request::SharedPtr req,
    futuraps_perception::srv::GetGlobalNormal::Response::SharedPtr      resp)
  {
    resp->nx = resp->ny = resp->nz = std::numeric_limits<float>::quiet_NaN();
    resp->cx = resp->cy = resp->cz = 0.0f;
    resp->confidence = 0.0f;

    if (!latest_cloud_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "No point cloud received yet.");
      return;
    }

    if (req->frame_id != latest_cloud_->header.frame_id) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Requested frame '%s' but cloud is in '%s'. "
        "No TF transform implemented; returning in cloud frame.",
        req->frame_id.c_str(),
        latest_cloud_->header.frame_id.c_str()
      );
    }

    // Extract XYZ
    // Pull bounds
    const bool use_bounds = (req->min_x <= req->max_x) &&
                            (req->min_y <= req->max_y) &&
                            (req->min_z <= req->max_z);

    std::vector<Eigen::Vector3f> pts;
    pts.reserve(latest_cloud_->width * latest_cloud_->height);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*latest_cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*latest_cloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*latest_cloud_, "z");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const float x = *it_x, y = *it_y, z = *it_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

      if (use_bounds) {
        if (x < req->min_x || x > req->max_x) continue;
        if (y < req->min_y || y > req->max_y) continue;
        if (z < req->min_z || z > req->max_z) continue;
      }
      pts.emplace_back(x, y, z);
    }


    if ((int)pts.size() < min_points_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Not enough valid points for normal estimation (%zu < %d).",
        pts.size(), min_points_);
      return;
    }

    // Compute centroid μ
    Eigen::Vector3f mu(0.f, 0.f, 0.f);
    for (const auto &p : pts) {
      mu += p;
    }
    mu /= static_cast<float>(pts.size());

    // Compute covariance Σ = mean( (p-μ)(p-μ)^T )
    Eigen::Matrix3f Sigma = Eigen::Matrix3f::Zero();
    for (const auto &p : pts) {
      Eigen::Vector3f q = p - mu;
      Sigma += q * q.transpose();
    }
    Sigma /= static_cast<float>(pts.size());

    // Eigen-decomposition (symmetric)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(Sigma);
    if (es.info() != Eigen::Success) {
      RCLCPP_WARN(get_logger(), "Eigen decomposition failed.");
      return;
    }


    Eigen::Vector3f n = es.eigenvectors().col(0);
    if (!std::isfinite(n[0]) || !std::isfinite(n[1]) || !std::isfinite(n[2])) {
      RCLCPP_WARN(get_logger(), "Computed invalid normal (NaN).");
      return;
    }
    n.normalize();

    // Orient normal to point roughly towards XZ plane (should have neg y component)
    {
      if (n.y() > 0.0f) {
        n = -n;
      }
    }

    // Simple planarity/confidence metric:
    // l0 = smallest, l1 = middle. If l0<<l1, it's very planar.
    Eigen::Vector3f evals = es.eigenvalues(); // ascending
    const float l0 = evals(0);
    const float l1 = evals(1);
    float planarity = 1.0f - (l0 / (l1 + 1e-6f));
    if (!std::isfinite(planarity)) planarity = 0.0f;
    planarity = std::clamp(planarity, 0.0f, 1.0f);

    // Fill response
    resp->nx = n.x();
    resp->ny = n.y();
    resp->nz = n.z();
    resp->cx = mu.x();
    resp->cy = mu.y();
    resp->cz = mu.z();
    resp->confidence = planarity;
  }

  // ---------- Members ----------
  std::string cloud_topic_;
  std::string frame_id_;
  int min_points_;

  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Service<futuraps_perception::srv::GetGlobalNormal>::SharedPtr srv_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalNormalServer>());
  rclcpp::shutdown();
  return 0;
}
