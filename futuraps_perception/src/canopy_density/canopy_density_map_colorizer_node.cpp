#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <futuraps_perception/msg/canopy_density_grid.hpp>
#include <futuraps_perception/grid_spec.hpp>
#include <cmath>
#include <limits>
#include <mutex>

using futuraps_perception::msg::CanopyDensityGrid;
using futuraps::GridSpec;

class LocalMapColorizer : public rclcpp::Node {
public:
  LocalMapColorizer() : rclcpp::Node("local_map_colorizer") {
    cloud_sub_topic_ = declare_parameter<std::string>("cloud_in", "/local_map/points");
    cloud_pub_topic_ = declare_parameter<std::string>("cloud_out","/local_map/colored");
    grid_topic_      = declare_parameter<std::string>("grid_topic","/canopy_density/grid");
    frame_id_        = declare_parameter<std::string>("grid_frame","base_link");

    // Color tuning
    alpha_  = declare_parameter<double>("alpha", 1.0);
    gain_   = declare_parameter<double>("gain", 2.0);
    bias_   = declare_parameter<double>("bias", 0.0);
    gamma_  = declare_parameter<double>("gamma", 0.7);
    v_min_  = declare_parameter<double>("v_min", 0.0);
    v_max_  = declare_parameter<double>("v_max", 1.0);
    autostretch_ = declare_parameter<bool>("auto_stretch", false);

    grid_sub_ = create_subscription<CanopyDensityGrid>(
      grid_topic_, rclcpp::QoS(1),
      std::bind(&LocalMapColorizer::grid_cb, this, std::placeholders::_1));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_sub_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalMapColorizer::cloud_cb, this, std::placeholders::_1));

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_pub_topic_, rclcpp::QoS(1));
  }

private:
  void grid_cb(const CanopyDensityGrid::SharedPtr msg) {
    std::scoped_lock lk(m_);
    grid_ = *msg; // copy; small (metadata + vectors)
    // optional auto-stretch precompute
    if (autostretch_) {
      float mn =  1.f, mx = 0.f;
      for (float d : grid_.data) if (!std::isnan(d)) { if (d<mn) mn=d; if (d>mx) mx=d; }
      if (mx > mn) { v_min_dyn_ = mn; v_max_dyn_ = mx; } else { v_min_dyn_ = 0.f; v_max_dyn_ = 1.f; }
    }
  }

  void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // need a grid
    CanopyDensityGrid grid;
    {
      std::scoped_lock lk(m_);
      if (grid_.data.empty()) return;
      grid = grid_;
    }

    if (!grid.header.frame_id.empty() && grid.header.frame_id != msg->header.frame_id) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Colorizer: cloud frame '%s' != grid frame '%s' (skipping)",
                           msg->header.frame_id.c_str(), grid.header.frame_id.c_str());
      return;
    }

    sensor_msgs::msg::PointCloud2 out = *msg;

    bool has_rgb = false;
    for (auto &f : out.fields) if (f.name == "rgb") has_rgb = true;
    if (!has_rgb) {
      sensor_msgs::PointCloud2Modifier mod(out);
      mod.setPointCloud2FieldsByString(2, "xyz", "rgb");
    }

    GridSpec g{};
    g.min_x = grid.origin_x;
    g.min_z = grid.origin_z;
    g.cell_size = grid.cell_size;
    g.width  = grid.width;
    g.height = grid.height;

    auto flat_index = [&](int i, int j)->size_t { return static_cast<size_t>(i)*g.height + j; };

    // Color transform parameters
    const float vmin = autostretch_ ? v_min_dyn_ : static_cast<float>(v_min_);
    const float vmax = autostretch_ ? v_max_dyn_ : static_cast<float>(v_max_);
    const float span = std::max(1e-6f, vmax - vmin);
    const float gain = static_cast<float>(gain_);
    const float bias = static_cast<float>(bias_);
    const float gamma = static_cast<float>(gamma_);

    // Iterate points and write RGB
    sensor_msgs::PointCloud2Iterator<float> x_it(out, "x");
    sensor_msgs::PointCloud2Iterator<float> y_it(out, "y");
    sensor_msgs::PointCloud2Iterator<float> z_it(out, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> r_it(out, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> g_it(out, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> b_it(out, "b");

    for (; x_it != x_it.end(); ++x_it, ++y_it, ++z_it, ++r_it, ++g_it, ++b_it) {
      const float x = *x_it, z = *z_it;

      // map to cell
      const int i = static_cast<int>(std::floor((x - g.min_x) / g.cell_size));
      const int j = static_cast<int>(std::floor((z - g.min_z) / g.cell_size));
      float d = std::numeric_limits<float>::quiet_NaN();
      if (i >= 0 && j >= 0 && i < static_cast<int>(g.width) && j < static_cast<int>(g.height)) {
        d = grid.data[flat_index(i,j)];
      }

      // no data? fade to gray
      float R=128, G=128, B=128;
      if (!std::isnan(d)) {
        // normalize & boost
        float v = (d - vmin) / span;
        v = std::clamp(v - bias, 0.0f, 1.0f);
        v = std::pow(v, gamma);
        v = std::clamp(v * gain, 0.0f, 1.0f);

        // blue→red colormap
        R = 255.0f * v;
        G = 0.0f;
        B = 255.0f * (1.0f - v);
      }

      *r_it = static_cast<uint8_t>(R);
      *g_it = static_cast<uint8_t>(G);
      *b_it = static_cast<uint8_t>(B);
    }

    cloud_pub_->publish(out);
  }

  // params
  std::string cloud_sub_topic_, cloud_pub_topic_, grid_topic_, frame_id_;
  double alpha_{1.0}, gain_{2.0}, bias_{0.0}, gamma_{0.7}, v_min_{0.0}, v_max_{1.0};
  bool autostretch_{false};

  // state
  std::mutex m_;
  CanopyDensityGrid grid_;
  float v_min_dyn_{0.f}, v_max_dyn_{1.f};

  // io
  rclcpp::Subscription<CanopyDensityGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalMapColorizer>());
  rclcpp::shutdown();
  return 0;
}
