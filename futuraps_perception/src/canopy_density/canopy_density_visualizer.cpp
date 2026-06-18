#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <futuraps_perception/msg/canopy_density_grid.hpp>
#include <futuraps_perception/srv/get_canopy_density_grid.hpp>

#include <cmath>
#include <chrono>
#include <string>

using futuraps_perception::msg::CanopyDensityGrid;
using futuraps_perception::srv::GetCanopyDensityGrid;
using namespace std::chrono_literals;

class CanopyDensityViz : public rclcpp::Node {
public:
  CanopyDensityViz() : rclcpp::Node("canopy_density_visualizer") {
    // --- Visualization parameters ---
    alpha_         = declare_parameter<double>("alpha", 0.85);
    height_offset_ = declare_parameter<double>("height_offset", 0.01);
    v_min_         = declare_parameter<double>("v_min",  0.0);  // clip low
    v_max_         = declare_parameter<double>("v_max",  1.0);  // clip high
    gain_          = declare_parameter<double>("gain",   1.5);  // >1 = more punch
    bias_          = declare_parameter<double>("bias",   0.0);  // shift before scaling
    gamma_         = declare_parameter<double>("gamma",  0.7);  // <1 brightens
    auto_stretch_  = declare_parameter<bool>("auto_stretch", false);

    // --- Service + grid config ---
    srv_name_ = declare_parameter<std::string>("srv_name",
                                               "/get_canopy_density_grid");
    frame_id_ = declare_parameter<std::string>("frame_id", "perception_base");

    min_x_     = declare_parameter<double>("min_x", -3.0);
    max_x_     = declare_parameter<double>("max_x",  2.0);
    min_z_     = declare_parameter<double>("min_z", -0.2);
    max_z_     = declare_parameter<double>("max_z",  2.0);
    cell_size_ = declare_parameter<double>("cell_size", 0.20);

    min_y_           = declare_parameter<double>("min_y", -2.0);
    max_y_           = declare_parameter<double>("max_y",  2.0);
    vy_              = declare_parameter<double>("vy", 0.10);
    k_min_points_    = static_cast<uint16_t>(declare_parameter<int>("k_min_points", 3));
    y_min_canopy_    = declare_parameter<double>("y_min_canopy", 0.0);
    use_column_density_ = declare_parameter<bool>("use_column_density", true);
    apply_ema_       = declare_parameter<bool>("apply_ema", true);
    ema_alpha_       = declare_parameter<double>("ema_alpha", 0.2);
    time_window_     = declare_parameter<double>("time_window", 0.75);

    call_period_     = declare_parameter<double>("call_period", 0.5); // seconds

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/canopy_density/markers", rclcpp::QoS(1));
    client_ = create_client<GetCanopyDensityGrid>(srv_name_);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(call_period_),
      std::bind(&CanopyDensityViz::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "CanopyDensityViz calling %s with fixed grid [%f,%f]x[%f,%f] in %s",
      srv_name_.c_str(), min_x_, max_x_, min_z_, max_z_, frame_id_.c_str());
  }

private:
  // --- Periodic service call ---
  void tick() {
    if (!client_->wait_for_service(10ms)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Service %s not available yet", srv_name_.c_str());
      return;
    }

    auto req = std::make_shared<GetCanopyDensityGrid::Request>();
    req->min_x   = min_x_;
    req->max_x   = max_x_;
    req->min_z   = min_z_;
    req->max_z   = max_z_;
    req->cell_size = cell_size_;

    req->min_y   = min_y_;
    req->max_y   = max_y_;
    req->vy      = vy_;
    req->k_min_points    = k_min_points_;
    req->y_min_canopy    = y_min_canopy_;
    req->use_column_density = use_column_density_;
    req->apply_ema       = apply_ema_;
    req->alpha           = ema_alpha_;
    req->frame_id        = frame_id_;
    req->time_window     = time_window_;

    auto future = client_->async_send_request(
      req,
      std::bind(&CanopyDensityViz::on_response, this, std::placeholders::_1));
    (void)future;
  }

  void on_response(rclcpp::Client<GetCanopyDensityGrid>::SharedFuture future) {
    auto res = future.get();
    auto grid_msg = std::make_shared<CanopyDensityGrid>(res->grid);
    cb(grid_msg);  // reuse the original visualization callback
  }

  void cb(const CanopyDensityGrid::SharedPtr msg) {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;

    float a_min = 1.0f, a_max = 0.0f;
    if (auto_stretch_) {
      for (uint32_t i = 0; i < msg->width; ++i) {
        for (uint32_t j = 0; j < msg->height; ++j) {
          const size_t idx = static_cast<size_t>(i) * msg->height + j;
          float d = msg->data[idx];
          if (std::isnan(d)) continue;
          a_min = std::min(a_min, d);
          a_max = std::max(a_max, d);
        }
      }
      if (a_max <= a_min) { a_min = 0.0f; a_max = 1.0f; }
    }
    const float clip_min  = auto_stretch_ ? a_min : static_cast<float>(v_min_);
    const float clip_max  = auto_stretch_ ? a_max : static_cast<float>(v_max_);
    const float clip_span = std::max(1e-6f, clip_max - clip_min);

    for (uint32_t i = 0; i < msg->width; ++i) {
      for (uint32_t j = 0; j < msg->height; ++j) {
        const size_t idx = static_cast<size_t>(i) * msg->height + j;
        float d = msg->data[idx];
        if (std::isnan(d)) continue;

        visualization_msgs::msg::Marker m;
        m.header = msg->header;
        m.header.stamp = rclcpp::Time(0);
        m.frame_locked = true;
        m.ns = "canopy_density";
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;

        const double x = msg->origin_x + (i + 0.5) * msg->cell_size;
        const double z = msg->origin_z + (j + 0.5) * msg->cell_size;

        m.pose.position.x = x;
        m.pose.position.y = height_offset_;
        m.pose.position.z = z;
        m.pose.orientation.w = 1.0;

        m.scale.x = msg->cell_size;
        m.scale.y = 0.005;
        m.scale.z = msg->cell_size;

        // normalize + boost
        float v = (d - clip_min) / clip_span;     // 0..1
        v = std::clamp(v - static_cast<float>(bias_), 0.0f, 1.0f);
        v = std::pow(v, static_cast<float>(gamma_));
        v = std::clamp(v * static_cast<float>(gain_), 0.0f, 1.0f);

        // map to color (blue->red)
        m.color.a = static_cast<float>(alpha_);
        m.color.r = v;
        m.color.g = 0.0f;
        m.color.b = 1.0f - v;

        arr.markers.push_back(std::move(m));
      }
    }
    pub_->publish(arr);
  }

  // --- Members ---
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Client<GetCanopyDensityGrid>::SharedPtr                    client_;
  rclcpp::TimerBase::SharedPtr                                       timer_;

  // Service / grid config
  std::string srv_name_;
  std::string frame_id_;
  double min_x_, max_x_, min_z_, max_z_;
  double cell_size_;
  double min_y_, max_y_, vy_;
  uint16_t k_min_points_;
  double y_min_canopy_;
  bool use_column_density_;
  bool apply_ema_;
  double ema_alpha_;
  double time_window_;
  double call_period_;

  // Visualization parameters
  double alpha_{0.85};
  double height_offset_{0.01};
  double v_min_{0.0}, v_max_{1.0}, gain_{1.5}, bias_{0.0}, gamma_{0.7};
  bool auto_stretch_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanopyDensityViz>());
  rclcpp::shutdown();
  return 0;
}
