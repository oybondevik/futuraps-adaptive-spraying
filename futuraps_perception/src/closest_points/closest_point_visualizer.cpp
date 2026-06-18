#include <limits>
#include <memory>
#include <vector>
#include <chrono>
#include <optional>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <futuraps_perception/srv/get_closest_grid.hpp>

using futuraps_perception::srv::GetClosestGrid;
using std::chrono::milliseconds;

class GridVizNode : public rclcpp::Node {
public:
  GridVizNode() : rclcpp::Node("grid_viz_node")
  {
    // ---- Generic params ----
    frame_id_    = declare_parameter<std::string>("frame_id", "base_link");
    cell_size_   = declare_parameter<double>("cell_size", 0.20);
    centered_    = declare_parameter<bool>("centered_grid", true);
    show_points_ = declare_parameter<bool>("show_centers", true);

    fallback_y_offset_ = declare_parameter<double>("fallback_y_offset", 0.0);
    last_y_row_  = fallback_y_offset_;

    // ---- Two config modes ----
    // (A) Direct grid mode
    origin_x_ = declare_parameter<double>("origin_x", 0.0);
    origin_z_ = declare_parameter<double>("origin_z", 0.20);
    cols_     = declare_parameter<int>("cols", 20);
    rows_     = declare_parameter<int>("rows", 8);

    // (B) Derive grid from crop-box XZ
    derive_from_crop_box_ = declare_parameter<bool>("derive_from_crop_box", false);
    min_x_ = declare_parameter<double>("min_x", -3.0);
    max_x_ = declare_parameter<double>("max_x",  2.0);
    min_z_ = declare_parameter<double>("min_z", -0.2);
    max_z_ = declare_parameter<double>("max_z",  2.0);

    // ---- Service behaviour ----
    call_service_for_closest_ = declare_parameter<bool>("call_service_for_closest", true);
    closest_srv_name_         = declare_parameter<std::string>("closest_srv", "/get_closest_grid");

    y_left_max_   = declare_parameter<double>("y_left_max",  1.0);
    y_right_max_  = declare_parameter<double>("y_right_max", 1.0);
    side_         = declare_parameter<int>("side", 0);            // 0 both, 1 left, -1 right
    front_pct_    = declare_parameter<double>("front_percentile", 0.10);
    min_pts_      = declare_parameter<int>("min_points_per_cell", 20);

    // Compute from crop-box if requested
    if (derive_from_crop_box_) {
      const double width_x  = std::max(0.0, max_x_ - min_x_);
      const double height_z = std::max(0.0, max_z_ - min_z_);
      cols_     = static_cast<int>(std::ceil(width_x  / cell_size_));
      rows_     = static_cast<int>(std::ceil(height_z / cell_size_));
      origin_x_ = min_x_;     // lower-left
      origin_z_ = min_z_;
      centered_ = false;      // LL origin is explicit now
    }

    pub_   = create_publisher<visualization_msgs::msg::MarkerArray>("grid_viz", 10);
    timer_ = create_wall_timer(milliseconds(500), std::bind(&GridVizNode::tick, this));

    if (call_service_for_closest_) {
      client_ = create_client<GetClosestGrid>(closest_srv_name_);
    }
  }

private:
  void maybe_send_request(double x0, double z0) {
    if (!call_service_for_closest_ || !client_) return;
    if (pending_future_.has_value()) return;
    if (!client_->wait_for_service(milliseconds(10))) return;

    auto req = std::make_shared<GetClosestGrid::Request>();
    req->cell_size            = static_cast<float>(cell_size_);
    req->rows                 = static_cast<uint32_t>(rows_);
    req->cols                 = static_cast<uint32_t>(cols_);
    req->x0                   = static_cast<float>(x0);  // lower-left origin
    req->z0                   = static_cast<float>(z0);
    req->y_left_max           = static_cast<float>(y_left_max_);
    req->y_right_max          = static_cast<float>(y_right_max_);
    req->side                 = static_cast<int8_t>(side_);
    req->front_percentile     = static_cast<float>(front_pct_);
    req->min_points_per_cell  = static_cast<uint32_t>(min_pts_);

    pending_future_ = client_->async_send_request(req);
  }

  void maybe_consume_response(visualization_msgs::msg::MarkerArray &arr, int &best_idx_out) {
    best_idx_out = -1;
    if (!pending_future_.has_value()) return;

    auto &fut = pending_future_.value();
    if (fut.wait_for(milliseconds(0)) != std::future_status::ready) return;

    auto resp = fut.get();      // consume
    pending_future_.reset();

    const size_t n = static_cast<size_t>(rows_) * static_cast<size_t>(cols_);
    if (resp->found.size() != n || resp->dist.size() != n ||
        resp->x.size() != n || resp->y.size() != n || resp->z.size() != n) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ClosestGrid response wrong size.");
      have_resp_ = false;
      return;
    }

    // --- cache everything for drawing later in tick() ---
    last_found_.assign(resp->found.begin(), resp->found.end());
    last_x_.assign(resp->x.begin(), resp->x.end());
    last_y_.assign(resp->y.begin(), resp->y.end());
    last_z_.assign(resp->z.begin(), resp->z.end());
    last_dist_.assign(resp->dist.begin(), resp->dist.end());
    have_resp_ = true;

    // --- compute the single best cell (unchanged behavior) ---
    float best_dist = std::numeric_limits<float>::infinity();
    int best_idx = -1;
    for (size_t i = 0; i < n; ++i) {
      if (last_found_[i] && last_dist_[i] < best_dist) {
        best_dist = last_dist_[i];
        best_idx  = static_cast<int>(i);
      }
    }

    if (best_idx >= 0) {
      const double raw_y = static_cast<double>(last_y_[best_idx]);
      const double alpha = 0.3; // smoothing
      last_y_row_ = alpha * raw_y + (1.0 - alpha) * last_y_row_;
      best_idx_out = best_idx;

      // Optional: draw the true 3D point as a sphere if available
      visualization_msgs::msg::Marker pt;
      pt.header.frame_id = frame_id_;
      pt.header.stamp = rclcpp::Time(0,0,RCL_ROS_TIME);
      pt.ns = "closest_point";
      pt.id = 3;
      pt.type = visualization_msgs::msg::Marker::SPHERE;
      pt.action = visualization_msgs::msg::Marker::ADD;

      pt.pose.position.x = last_x_[best_idx];
      pt.pose.position.y = last_y_[best_idx];
      pt.pose.position.z = last_z_[best_idx];
      pt.pose.orientation.w = 1.0;

      pt.scale.x = 0.05; pt.scale.y = 0.05; pt.scale.z = 0.05;
      pt.color.r = 0.1f; pt.color.g = 1.0f; pt.color.b = 0.4f; pt.color.a = 0.9f;
      arr.markers.push_back(pt);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "best_idx=%d y_row=%.3f best_dist=%.3f", best_idx, last_y_row_, best_dist);
    }
  }

  void tick() {
    visualization_msgs::msg::MarkerArray arr;

    const double width  = cols_ * cell_size_;
    const double height = rows_ * cell_size_;
    const double x0 = centered_ ? (origin_x_ - 0.5 * width)  : origin_x_;
    const double z0 = centered_ ? (origin_z_ - 0.5 * height) : origin_z_;

    // Non-blocking service logic
    maybe_send_request(x0, z0);
    int best_idx = -1;
    maybe_consume_response(arr, best_idx);

    // Draw grid on XZ plane at y = last_y_row_
    const double y_row = last_y_row_;

    // Grid lines
    visualization_msgs::msg::Marker lines;
    lines.header.frame_id = frame_id_;
    lines.header.stamp    = rclcpp::Time(0,0,RCL_ROS_TIME);
    lines.ns   = "grid_plane_xz";
    lines.id   = 0;
    lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines.action = visualization_msgs::msg::Marker::ADD;
    lines.scale.x = 0.01;
    lines.color.r = 0.9f; lines.color.g = 0.9f; lines.color.b = 0.9f; lines.color.a = 1.0f;

    // Vertical lines (vary z)
    for (int c = 0; c <= cols_; ++c) {
      geometry_msgs::msg::Point p0, p1;
      p0.x = x0 + c * cell_size_; p0.y = y_row; p0.z = z0;
      p1.x = x0 + c * cell_size_; p1.y = y_row; p1.z = z0 + height;
      lines.points.push_back(p0); lines.points.push_back(p1);
    }
    // Horizontal lines (vary x)
    for (int r = 0; r <= rows_; ++r) {
      geometry_msgs::msg::Point p0, p1;
      p0.x = x0; p0.y = y_row; p0.z = z0 + r * cell_size_;
      p1.x = x0 + width; p1.y = y_row; p1.z = z0 + r * cell_size_;
      lines.points.push_back(p0); lines.points.push_back(p1);
    }
    arr.markers.push_back(lines);

    // Cell centers (optional)
    if (show_points_) {
      visualization_msgs::msg::Marker pts;
      pts.header.frame_id = frame_id_;
      pts.header.stamp    = rclcpp::Time(0,0,RCL_ROS_TIME);
      pts.ns = "grid_centers_xz";
      pts.id = 1;
      pts.type = visualization_msgs::msg::Marker::POINTS;
      pts.action = visualization_msgs::msg::Marker::ADD;
      pts.scale.x = 0.04; pts.scale.y = 0.04;
      pts.color.r = 0.2f; pts.color.g = 0.6f; pts.color.b = 1.0f; pts.color.a = 0.9f;

      for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
          geometry_msgs::msg::Point pc;
          pc.x = x0 + (c + 0.5) * cell_size_;
          pc.y = y_row;
          pc.z = z0 + (r + 0.5) * cell_size_;
          pts.points.push_back(pc);
        }
      }
      arr.markers.push_back(pts);
    }

    // --- draw closest point for each cell ---
    if (have_resp_) {
      const size_t n = static_cast<size_t>(rows_) * static_cast<size_t>(cols_);

      // Compute a max distance for color scaling (ignore inf)
      double max_d = 0.0;
      for (size_t i = 0; i < n; ++i) {
        if (last_found_[i]) max_d = std::max<double>(max_d, last_dist_[i]);
      }
      if (max_d <= 1e-6) max_d = 1.0;

      // POINTS marker: one point per cell hit, color by distance (green=near -> red=far)
      visualization_msgs::msg::Marker pts_hit;
      pts_hit.header.frame_id = frame_id_;
      pts_hit.header.stamp    = rclcpp::Time(0,0,RCL_ROS_TIME);
      pts_hit.ns   = "closest_points_all";
      pts_hit.id   = 4;
      pts_hit.type = visualization_msgs::msg::Marker::POINTS;
      pts_hit.action = visualization_msgs::msg::Marker::ADD;
      pts_hit.scale.x = 0.04; pts_hit.scale.y = 0.04;
      pts_hit.colors.reserve(n);
      pts_hit.points.reserve(n);

      // Optional: LINE_LIST rays from cell center (at y_row) to the hit point
      visualization_msgs::msg::Marker rays;
      rays.header.frame_id = frame_id_;
      rays.header.stamp    = rclcpp::Time(0,0,RCL_ROS_TIME);
      rays.ns   = "closest_rays";
      rays.id   = 5;
      rays.type = visualization_msgs::msg::Marker::LINE_LIST;
      rays.action = visualization_msgs::msg::Marker::ADD;
      rays.scale.x = 0.005;

      auto push_color = [&](double t){ // t in [0,1] -> green->red
        std_msgs::msg::ColorRGBA c;
        c.a = 0.95f;
        c.r = static_cast<float>(t);
        c.g = static_cast<float>(1.0 - t);
        c.b = 0.2f;
        return c;
      };

      for (int r = 0; r < rows_; ++r) {
        for (int c = 0; c < cols_; ++c) {
          const size_t i = static_cast<size_t>(r)*static_cast<size_t>(cols_) + static_cast<size_t>(c);
          if (!last_found_[i]) continue;

          // add hit point
          geometry_msgs::msg::Point p;
          p.x = last_x_[i]; p.y = last_y_[i]; p.z = last_z_[i];
          pts_hit.points.push_back(p);

          const double t = std::min(1.0, std::max(0.0, static_cast<double>(last_dist_[i]) / max_d));
          pts_hit.colors.push_back(push_color(t));

          // ray from cell center (on the XZ grid plane at y_row) to the hit point
          geometry_msgs::msg::Point c0;
          c0.x = x0 + (c + 0.5)*cell_size_;
          c0.y = y_row;
          c0.z = z0 + (r + 0.5)*cell_size_;

          rays.points.push_back(c0);
          rays.points.push_back(p);

          auto ray_col = push_color(t);
          rays.colors.push_back(ray_col);
          rays.colors.push_back(ray_col);
        }
      }

      // default colors if empty
      if (pts_hit.points.empty()) {
        pts_hit.color.r = 0.8f; pts_hit.color.g = 0.8f; pts_hit.color.b = 0.8f; pts_hit.color.a = 0.6f;
      }
      if (rays.points.empty()) {
        rays.color.r = 0.6f; rays.color.g = 0.6f; rays.color.b = 0.6f; rays.color.a = 0.6f;
      }

      arr.markers.push_back(pts_hit);
      arr.markers.push_back(rays);
    }

    // Highlight best cell
    if (best_idx >= 0) {
      const int r = best_idx / cols_;
      const int c = best_idx % cols_;

      visualization_msgs::msg::Marker cube;
      cube.header.frame_id = frame_id_;
      cube.header.stamp    = rclcpp::Time(0,0,RCL_ROS_TIME);
      cube.ns = "closest_cell_xz";
      cube.id = 2;
      cube.type = visualization_msgs::msg::Marker::CUBE;
      cube.action = visualization_msgs::msg::Marker::ADD;

      cube.pose.position.x = x0 + (c + 0.5) * cell_size_;
      cube.pose.position.y = y_row;
      cube.pose.position.z = z0 + (r + 0.5) * cell_size_;
      cube.pose.orientation.w = 1.0;

      cube.scale.x = cell_size_;
      cube.scale.y = 0.01;
      cube.scale.z = cell_size_;

      cube.color.r = 1.0f; cube.color.g = 0.3f; cube.color.b = 0.2f; cube.color.a = 0.6f;
      arr.markers.push_back(cube);
    }

    pub_->publish(arr);
  }

  // ---- params / state ----
  std::string frame_id_, closest_srv_name_;
  double origin_x_, origin_z_, cell_size_;
  int rows_, cols_;
  bool centered_, show_points_;
  bool call_service_for_closest_;
  double fallback_y_offset_, last_y_row_;

  // crop-box for derivation
  bool derive_from_crop_box_;
  double min_x_, max_x_, min_z_, max_z_;

  double y_left_max_, y_right_max_, front_pct_;
  int side_, min_pts_;

  // --- cached per-cell data from last service response ---
  std::vector<uint8_t> last_found_;
  std::vector<float>   last_x_, last_y_, last_z_, last_dist_;
  bool have_resp_ = false;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<GetClosestGrid>::SharedPtr client_;
  std::optional<rclcpp::Client<GetClosestGrid>::SharedFuture> pending_future_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridVizNode>());
  rclcpp::shutdown();
  return 0;
}
