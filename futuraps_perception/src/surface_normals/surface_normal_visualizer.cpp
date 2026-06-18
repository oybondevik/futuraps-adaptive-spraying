#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <unordered_map>
#include <vector>
#include <string>
#include <chrono>
#include <future>
#include <algorithm>
#include <cmath>

#include "futuraps_perception/srv/get_global_normal.hpp"
#include "futuraps_perception/srv/get_cloud_bounds.hpp"

using futuraps_perception::srv::GetGlobalNormal;
using futuraps_perception::srv::GetCloudBounds;

class NormalXZGridViz : public rclcpp::Node {
public:
  NormalXZGridViz() : Node("surface_normal_visualizer")
  {
    // ---- Params ----
    service_name_     = declare_parameter<std::string>("service_name", "/get_global_normal");
    bounds_service_   = declare_parameter<std::string>("bounds_service", "/get_cloud_bounds");
    frame_id_         = declare_parameter<std::string>("frame_id", "base_link");
    marker_topic_     = declare_parameter<std::string>("marker_topic", "/normal_grid_markers");

    publish_rate_hz_  = declare_parameter<double>("publish_rate_hz", 5.0);
    max_inflight_     = declare_parameter<int>("max_inflight_requests", 32);

    // Grid dims (used for both bounds-fit and fallback)
    dim_x_            = declare_parameter<int>("dim_x", 2);
    dim_z_            = declare_parameter<int>("dim_z", 2);

    // Fallback XZ box (used only when bounds unavailable)
    fb_x_min_         = declare_parameter<double>("fb_x_min", -3.0);
    fb_x_max_         = declare_parameter<double>("fb_x_max",  1.0);
    fb_z_min_         = declare_parameter<double>("fb_z_min", -0.2);
    fb_z_max_         = declare_parameter<double>("fb_z_max",  2.0);

    // Marker aesthetics
    shaft_diam_       = declare_parameter<double>("shaft_diameter", 0.02);
    head_diam_        = declare_parameter<double>("head_diameter",  0.05);
    head_len_         = declare_parameter<double>("head_length",    0.08);
    base_len_m_       = declare_parameter<double>("base_length_m",  0.35);
    extra_len_m_      = declare_parameter<double>("extra_length_m", 0.45);

    // Where to anchor arrows: "cell_center" (even spacing) or "centroid"
    anchor_mode_      = declare_parameter<std::string>("anchor_mode", "cell_center");

    // ---- ROS I/O ----
    client_normals_ = create_client<GetGlobalNormal>(service_name_);
    client_bounds_  = create_client<GetCloudBounds>(bounds_service_);
    pub_            = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

    // Try live bounds first; else compute fallback evenly
    if (!refresh_bounds_and_set_grid()) {
      RCLCPP_WARN(get_logger(), "Bounds unavailable; using fallback XZ box.");
      set_fallback_grid_from_params();  // computes origin_x_, origin_z_, cell_x_, cell_z_
    }
    build_cells();

    // Timer
    auto period = std::chrono::duration<double>(1.0 / std::max(0.5, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&NormalXZGridViz::tick, this));

    RCLCPP_INFO(get_logger(),
      "normal_xz_grid_viz up. normals: '%s', bounds: '%s', frame '%s' | dim %d x %d (x,z), cell %.2fm x %.2fm, anchor=%s, Y=[ALL]",
      service_name_.c_str(), bounds_service_.c_str(), frame_id_.c_str(),
      dim_x_, dim_z_, cell_x_, cell_z_, anchor_mode_.c_str());
  }

private:
  struct Cell {
    int ix, iz;
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    int id;
  };
  struct Inflight {
    rclcpp::Client<GetGlobalNormal>::SharedFuture fut;
    int cell_id;
    rclcpp::Time sent;
  };

  // --- Grid derivation helpers ---

  bool refresh_bounds_and_set_grid()
  {
    if (!client_bounds_->wait_for_service(std::chrono::milliseconds(2000))) {
      return false;
    }
    auto req = std::make_shared<GetCloudBounds::Request>();
    req->frame_id = frame_id_;
    auto fut = client_bounds_->async_send_request(req);
    if (fut.wait_for(std::chrono::milliseconds(3000)) != std::future_status::ready) {
      return false;
    }
    auto resp = fut.get();
    if (!resp || resp->count == 0 ||
        !std::isfinite(resp->min_x) || !std::isfinite(resp->max_x) ||
        !std::isfinite(resp->min_z) || !std::isfinite(resp->max_z)) {
      return false;
    }

    const double min_x = resp->min_x;
    const double max_x = resp->max_x;
    const double min_z = resp->min_z;
    const double max_z = resp->max_z;

    const double span_x = max_x - min_x;
    const double span_z = max_z - min_z;
    if (span_x <= 1e-6 || span_z <= 1e-6) {
      return false;
    }

    origin_x_ = min_x;
    origin_z_ = min_z;

    const int gx = std::max(1, dim_x_);
    const int gz = std::max(1, dim_z_);

    cell_x_ = span_x / static_cast<double>(gx);
    cell_z_ = span_z / static_cast<double>(gz);

    RCLCPP_INFO(get_logger(),
      "Bounds fit: X[%.2f, %.2f] Z[%.2f, %.2f] -> dim %d×%d, cell %.3f×%.3f",
      min_x, max_x, min_z, max_z, gx, gz, cell_x_, cell_z_);
    return true;
  }

  bool set_fallback_grid_from_params()
  {
    const double span_x = fb_x_max_ - fb_x_min_;
    const double span_z = fb_z_max_ - fb_z_min_;
    const int gx = std::max(1, dim_x_);
    const int gz = std::max(1, dim_z_);

    if (span_x <= 1e-6 || span_z <= 1e-6) {
      RCLCPP_WARN(get_logger(), "Fallback spans degenerate; check fb_x/z_* params.");
      return false;
    }

    origin_x_ = fb_x_min_;
    origin_z_ = fb_z_min_;
    cell_x_   = span_x / static_cast<double>(gx);
    cell_z_   = span_z / static_cast<double>(gz);

    RCLCPP_INFO(get_logger(),
      "Fallback grid: X[%.2f, %.2f], Z[%.2f, %.2f] -> dim %d×%d, cell %.3f×%.3f",
      fb_x_min_, fb_x_max_, fb_z_min_, fb_z_max_, gx, gz, cell_x_, cell_z_);
    return true;
  }

  void build_cells()
  {
    cells_.clear();
    cells_by_id_.clear();
    cells_.reserve(dim_x_ * dim_z_);
    int id = 0;

    // Use ALL Y (cloud already filtered upstream)
    constexpr double kYMin = -1e9;
    constexpr double kYMax = +1e9;

    for (int ix = 0; ix < dim_x_; ++ix) {
      for (int iz = 0; iz < dim_z_; ++iz) {
        const double x0 = origin_x_ + ix * cell_x_;
        const double z0 = origin_z_ + iz * cell_z_;

        Cell c;
        c.ix = ix; c.iz = iz;
        c.min_x = x0;              c.max_x = x0 + cell_x_;
        c.min_y = kYMin;           c.max_y = kYMax; // ALL Y
        c.min_z = z0;              c.max_z = z0 + cell_z_;
        c.id = id++;

        cells_by_id_[c.id] = c;
        cells_.push_back(c);
      }
    }
    next_cell_index_ = 0;
    sweep_index_ = 0;
  }

  // --- Main loop ---

  void tick()
  {
    // 1) Collect finished
    process_ready_responses();

    // 2) Non-blocking service availability for normals
    if (!client_normals_->wait_for_service(std::chrono::milliseconds(5))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for service %s ...", service_name_.c_str());
      return;
    }

    // 3) Dispatch new up to limit
    while ((int)inflight_.size() < max_inflight_ && next_cell_index_ < (int)cells_.size()) {
      send_request_for_cell(cells_[next_cell_index_]);
      ++next_cell_index_;
    }

    // 4) End of sweep: publish and refresh bounds optionally
    if (next_cell_index_ >= (int)cells_.size() && inflight_.empty()) {
      publish_marker_array();

      if (refresh_bounds_and_set_grid()) {
        build_cells();
      }
      next_cell_index_ = 0;
      ++sweep_index_;
    }
  }

  void send_request_for_cell(const Cell& c)
  {
    auto req = std::make_shared<GetGlobalNormal::Request>();
    req->frame_id = frame_id_;
    req->min_x = (float)c.min_x; req->max_x = (float)c.max_x;
    req->min_y = (float)c.min_y; req->max_y = (float)c.max_y; // ALL Y
    req->min_z = (float)c.min_z; req->max_z = (float)c.max_z;

    Inflight infl;
    infl.fut  = client_normals_->async_send_request(req).share();
    infl.cell_id = c.id;
    infl.sent = now();
    inflight_.push_back(std::move(infl));
  }

  void process_ready_responses()
  {
    std::vector<size_t> done;
    done.reserve(inflight_.size());
    for (size_t i = 0; i < inflight_.size(); ++i) {
      auto &f = inflight_[i].fut;
      if (f.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        auto resp = f.get();
        if (resp) handle_cell_response(inflight_[i].cell_id, *resp);
        done.push_back(i);
      } else {
        const auto age_ms = (now() - inflight_[i].sent).nanoseconds() / 1'000'000;
        if (age_ms > 2000) {
          RCLCPP_WARN(get_logger(), "Normal request timed out; dropping cell id %d", inflight_[i].cell_id);
          done.push_back(i);
        }
      }
    }
    for (int k = (int)done.size() - 1; k >= 0; --k) {
      inflight_.erase(inflight_.begin() + done[k]);
    }
  }

  void handle_cell_response(int cell_id, const GetGlobalNormal::Response& resp)
  {
    if (!std::isfinite(resp.nx) || !std::isfinite(resp.ny) || !std::isfinite(resp.nz)) {
      latest_markers_.erase(cell_id);
      return;
    }

    auto it = cells_by_id_.find(cell_id);
    if (it == cells_by_id_.end()) return;
    const Cell& c = it->second;

    // Base point: cell center (even spacing) or actual centroid
    geometry_msgs::msg::Point p0;
    if (anchor_mode_ == "cell_center") {
      p0.x = 0.5 * (c.min_x + c.max_x);
      p0.y = 0.5 * (c.min_y + c.max_y);
      p0.z = 0.5 * (c.min_z + c.max_z);
    } else { // "centroid"
      p0.x = resp.cx; p0.y = resp.cy; p0.z = resp.cz;
    }

    const double t   = std::clamp<double>(resp.confidence, 0.0, 1.0);
    const double len = base_len_m_ + extra_len_m_ * t;

    geometry_msgs::msg::Point p1;
    p1.x = p0.x + resp.nx * len;
    p1.y = p0.y + resp.ny * len;
    p1.z = p0.z + resp.nz * len;

    visualization_msgs::msg::Marker m;
    m.header.stamp = now();
    m.header.frame_id = frame_id_;
    m.ns = "normal_xz_grid";
    m.id = cell_id;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.points = {p0, p1};

    m.scale.x = shaft_diam_;
    m.scale.y = head_diam_;
    m.scale.z = head_len_;

    m.color.a = 1.0f;
    m.color.r = 1.0f; //static_cast<float>(t);
    m.color.g = 0.0f; //static_cast<float>(1.0 - 0.5*t);
    m.color.b = 1.0f; //0.0f;

    m.lifetime = rclcpp::Duration::from_seconds(0.0);
    latest_markers_[cell_id] = std::move(m);
  }

  void publish_marker_array()
  {
    visualization_msgs::msg::MarkerArray arr;

    // Add current
    arr.markers.reserve(latest_markers_.size());
    for (auto &kv : latest_markers_) arr.markers.push_back(kv.second);

    // Delete any missing since last sweep
    if (!sent_ids_last_sweep_.empty()) {
      for (int id : sent_ids_last_sweep_) {
        if (latest_markers_.find(id) == latest_markers_.end()) {
          visualization_msgs::msg::Marker del;
          del.header.stamp = now();
          del.header.frame_id = frame_id_;
          del.ns = "normal_xz_grid";
          del.id = id;
          del.action = visualization_msgs::msg::Marker::DELETE;
          arr.markers.push_back(std::move(del));
        }
      }
    }

    sent_ids_last_sweep_.clear();
    sent_ids_last_sweep_.reserve(latest_markers_.size());
    for (auto &kv : latest_markers_) sent_ids_last_sweep_.push_back(kv.first);

    pub_->publish(arr);
  }

  // ---- Params ----
  std::string service_name_;
  std::string bounds_service_;
  std::string frame_id_;
  std::string marker_topic_;
  double publish_rate_hz_;
  int    max_inflight_;
  int    dim_x_;
  int    dim_z_;
  double fb_x_min_, fb_x_max_, fb_z_min_, fb_z_max_;
  std::string anchor_mode_;

  // Grid (XZ)
  double origin_x_{0.0};
  double origin_z_{0.0};
  double cell_x_{1.0};
  double cell_z_{1.0};

  // Marker style
  double shaft_diam_;
  double head_diam_;
  double head_len_;
  double base_len_m_;
  double extra_len_m_;

  // ---- State ----
  std::vector<Cell> cells_;
  std::unordered_map<int, Cell> cells_by_id_;
  size_t next_cell_index_{0};
  size_t sweep_index_{0};
  std::vector<Inflight> inflight_;
  std::unordered_map<int, visualization_msgs::msg::Marker> latest_markers_;
  std::vector<int> sent_ids_last_sweep_;

  // ---- ROS ----
  rclcpp::Client<GetGlobalNormal>::SharedPtr client_normals_;
  rclcpp::Client<GetCloudBounds>::SharedPtr  client_bounds_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NormalXZGridViz>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
