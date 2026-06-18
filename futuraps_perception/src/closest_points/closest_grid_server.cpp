#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "futuraps_perception/srv/get_closest_grid.hpp"
#include "futuraps_perception/closest_grid.hpp"

using PointT = pcl::PointXYZ;
using GetSrv = futuraps_perception::srv::GetClosestGrid;

class ClosestGridServer : public rclcpp::Node {
public:
  ClosestGridServer() : Node("closest_grid_server"),
    tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {

    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/plants_only");
    base_link_   = declare_parameter<std::string>("base_link", "base_link");

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ClosestGridServer::onCloud, this, std::placeholders::_1));

    srv_ = create_service<GetSrv>(
      "get_closest_grid",
      std::bind(&ClosestGridServer::onReq, this,
                std::placeholders::_1, std::placeholders::_2));
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { last_cloud_ = msg; }

  void fillEmptyResponse(const std::shared_ptr<GetSrv::Request> req,
                         std::shared_ptr<GetSrv::Response> res) {
    const size_t n = static_cast<size_t>(req->rows) * static_cast<size_t>(req->cols);
    res->cloud_stamp = this->now();
    res->cloud_frame_id = base_link_;
    res->found.assign(n, false);
    res->x.assign(n, 0.0f);
    res->y.assign(n, 0.0f);
    res->z.assign(n, 0.0f);
    res->dist.assign(n, 0.0f);
    res->count.assign(n, 0);
    res->confidence.assign(n, 0.0f);
  }

  void onReq(const std::shared_ptr<GetSrv::Request> req,
             std::shared_ptr<GetSrv::Response> res) {
    if (!last_cloud_) {
      fillEmptyResponse(req, res);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "ClosestGrid has no input cloud yet on '%s'", cloud_topic_.c_str());
      return;
    }

    // Ensure cloud in base_link
    sensor_msgs::msg::PointCloud2 cloud_bl;
    if (last_cloud_->header.frame_id == base_link_) {
      cloud_bl = *last_cloud_;
    } else {
      try {
        auto tf = tf_buffer_.lookupTransform(
          base_link_, last_cloud_->header.frame_id, rclcpp::Time(last_cloud_->header.stamp));
        Eigen::Isometry3d T = tf2::transformToEigen(tf);
        pcl::PointCloud<PointT> pc_in; pcl::fromROSMsg(*last_cloud_, pc_in);
        pcl::PointCloud<PointT> pc_out; pc_out.reserve(pc_in.size());
        for (const auto& p : pc_in.points) {
          Eigen::Vector3d v(p.x,p.y,p.z), w = T * v;
          pc_out.emplace_back(PointT{float(w.x()), float(w.y()), float(w.z())});
        }
        pcl::toROSMsg(pc_out, cloud_bl);
        cloud_bl.header.frame_id = base_link_;
        cloud_bl.header.stamp = last_cloud_->header.stamp;
      } catch (const std::exception& e) {
        try {
          auto tf = tf_buffer_.lookupTransform(
            base_link_, last_cloud_->header.frame_id, tf2::TimePointZero);
          Eigen::Isometry3d T = tf2::transformToEigen(tf);
          pcl::PointCloud<PointT> pc_in; pcl::fromROSMsg(*last_cloud_, pc_in);
          pcl::PointCloud<PointT> pc_out; pc_out.reserve(pc_in.size());
          for (const auto& p : pc_in.points) {
            Eigen::Vector3d v(p.x,p.y,p.z), w = T * v;
            pc_out.emplace_back(PointT{float(w.x()), float(w.y()), float(w.z())});
          }
          pcl::toROSMsg(pc_out, cloud_bl);
          cloud_bl.header.frame_id = base_link_;
          cloud_bl.header.stamp = last_cloud_->header.stamp;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "TF to base_link unavailable at cloud stamp; using latest TF fallback: %s",
            e.what());
        } catch (const std::exception& latest_e) {
          RCLCPP_WARN(get_logger(), "Latest TF fallback to base_link unavailable: %s", latest_e.what());
          fillEmptyResponse(req, res);
          return;
        }
      }
    }

    futuraps::GridReq Rq;
    Rq.s = req->cell_size;
    Rq.rows = req->rows; Rq.cols = req->cols;
    Rq.x0 = req->x0; Rq.z0 = req->z0;
    Rq.yL = req->y_left_max; Rq.yR = req->y_right_max;
    Rq.side = req->side;
    Rq.p_front = req->front_percentile;
    Rq.min_pts = req->min_points_per_cell;

    pcl::PointCloud<PointT> pc; pcl::fromROSMsg(cloud_bl, pc);
    auto out = futuraps::computeClosestGrid(pc, Rq);

    res->cloud_stamp = cloud_bl.header.stamp;
    res->cloud_frame_id = cloud_bl.header.frame_id;

    const size_t N = out.size();
    size_t found_count = 0;
    res->found.resize(N); res->x.resize(N); res->y.resize(N); res->z.resize(N);
    res->dist.resize(N); res->count.resize(N); res->confidence.resize(N);
    for (size_t k=0; k<N; ++k) {
      res->found[k] = out[k].found;
      if (out[k].found) ++found_count;
      res->x[k] = out[k].x; res->y[k] = out[k].y; res->z[k] = out[k].z;
      res->dist[k] = out[k].dist; res->count[k] = out[k].count; res->confidence[k] = out[k].confidence;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "ClosestGrid cloud='%s' frame='%s' points=%zu grid=%ux%u x=[%.2f,%.2f] z=[%.2f,%.2f] y=[-%.2f,%.2f] min_pts=%u found_cells=%zu",
      cloud_topic_.c_str(), cloud_bl.header.frame_id.c_str(), pc.size(),
      req->cols, req->rows,
      req->x0, req->x0 + req->cols * req->cell_size,
      req->z0, req->z0 + req->rows * req->cell_size,
      req->y_right_max, req->y_left_max,
      req->min_points_per_cell, found_count);
  }

  // params & members
  std::string cloud_topic_, base_link_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Service<GetSrv>::SharedPtr srv_;
  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosestGridServer>());
  rclcpp::shutdown();
  return 0;
}
