#include <deque>
#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <futuraps_perception/msg/canopy_density_grid.hpp>
#include <futuraps_perception/srv/get_canopy_density_grid.hpp>
#include <futuraps_perception/grid_spec.hpp>

using futuraps_perception::msg::CanopyDensityGrid;
using futuraps_perception::srv::GetCanopyDensityGrid;
using futuraps::GridSpec;

namespace {

struct TimedCloud {
  rclcpp::Time stamp;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
};

inline bool nearly_equal(double a, double b, double eps = 1e-6) {
  return std::abs(a - b) < eps;
}

struct PrevSignature {
  double min_x{}, max_x{}, min_z{}, max_z{}, cell_size{};
  bool   use_column_density{};
  std::string frame_id;

  bool operator==(const PrevSignature &o) const {
    return nearly_equal(min_x, o.min_x) &&
           nearly_equal(max_x, o.max_x) &&
           nearly_equal(min_z, o.min_z) &&
           nearly_equal(max_z, o.max_z) &&
           nearly_equal(cell_size, o.cell_size) &&
           use_column_density == o.use_column_density &&
           frame_id == o.frame_id;
  }
};

}  // namespace

class CanopyDensityServer : public rclcpp::Node {
public:
  CanopyDensityServer()
  : rclcpp::Node("canopy_density_server"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    // ---- Parameters (I/O) ----
    cloud_topic_    = declare_parameter<std::string>("cloud_topic", "/local_map/points");
    default_frame_  = declare_parameter<std::string>("default_frame", "base_link");
    max_buffer_sec_ = declare_parameter<double>("max_buffer_sec", 2.0);

    // ---- EMA default behavior ----
    apply_ema_default_ = declare_parameter<bool>("apply_ema_default", false);

    // ---- Mode selection: "column" (default) or "ortho" ----
    mode_ = declare_parameter<std::string>("mode", "ortho");

    // ---- Ortho mode parameter ----
    ortho_pixel_size_ = declare_parameter<double>("ortho_pixel_size", 0.01); // meters/pixel on x–z plane

    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CanopyDensityServer::cloud_cb, this, std::placeholders::_1));

    // Pure service server (no periodic publisher)
    srv_ = create_service<GetCanopyDensityGrid>(
      "/get_canopy_density_grid",
      std::bind(&CanopyDensityServer::srv_cb, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
      "canopy_density_server started as pure service server on /get_canopy_density_grid");
  }

private:
  // ----------------- Cloud buffering -----------------
  void cloud_cb(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    buffer_.push_back({msg->header.stamp, msg});
    prune_buffer(now());
  }

  void prune_buffer(const rclcpp::Time &tnow) {
    while (!buffer_.empty()) {
      if ((tnow - buffer_.front().stamp).seconds() > max_buffer_sec_) {
        buffer_.pop_front();
      } else {
        break;
      }
    }
  }

  // ----------------- Helper: select clouds in time window -----------------
  std::vector<sensor_msgs::msg::PointCloud2::SharedPtr>
  select_clouds(double time_window) {
    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> selected;
    if (buffer_.empty()) return selected;

    const rclcpp::Time tnow = now();

    if (time_window <= 0.0) {
      selected.push_back(buffer_.back().cloud);
      return selected;
    }

    for (auto it = buffer_.rbegin(); it != buffer_.rend(); ++it) {
      if ((tnow - it->stamp).seconds() <= time_window) {
        selected.push_back(it->cloud);
      } else {
        break;
      }
    }
    return selected;
  }

  // ----------------- EMA for both modes -----------------
  CanopyDensityGrid maybe_apply_ema(
    CanopyDensityGrid &out,
    const GridSpec &g,
    const GetCanopyDensityGrid::Request &req)
  {
    if (!req.apply_ema) {
      prev_data_.clear();
      prev_sig_ = {};
      return out;
    }

    PrevSignature sig;
    sig.min_x = g.min_x;
    sig.max_x = g.max_x;
    sig.min_z = g.min_z;
    sig.max_z = g.max_z;
    sig.cell_size = g.cell_size;
    sig.use_column_density = req.use_column_density;
    sig.frame_id = out.header.frame_id;

    const auto N = out.data.size();
    if (prev_data_.size() != N || !(sig == prev_sig_)) {
      prev_data_ = out.data;
      prev_sig_  = sig;
      return out;
    }

    const float alpha = std::clamp(static_cast<float>(req.alpha), 0.0f, 1.0f);

    for (size_t idx = 0; idx < N; ++idx) {
      const float cur  = out.data[idx];
      const float prev = prev_data_[idx];
      float fused;
      if (std::isnan(prev)) fused = cur;
      else if (std::isnan(cur)) fused = prev; // keep previous if no new info
      else fused = alpha * cur + (1.0f - alpha) * prev;
      out.data[idx] = fused;
    }
    prev_data_ = out.data; // persist for next call
    return out;
  }

  // ----------------- Service callback -----------------
  void srv_cb(
    const std::shared_ptr<GetCanopyDensityGrid::Request> req,
    std::shared_ptr<GetCanopyDensityGrid::Response> res)
  {
    if (mode_ == "ortho") {
      auto local_req = *req;
      if (!local_req.apply_ema) {
          // leave as false
      } else if (!apply_ema_default_) {
          // force-disable unless parameter is explicitly enabled
          local_req.apply_ema = false;
      }

      res->grid = compute_grid_ortho(local_req);
    } else {
      if (!req->use_column_density) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "use_column_density=false requested, but node runs in 'column' mode. Proceeding with column mode.");
      }
      auto local_req = *req;
      if (!local_req.apply_ema) {
          // leave as false
      } else if (!apply_ema_default_) {
          // force-disable unless parameter is explicitly enabled
          local_req.apply_ema = false;
      }

      res->grid = compute_grid_column(local_req);
    }
  }

  // ----------------- COLUMN MODE (same as before) -----------------
  CanopyDensityGrid compute_grid_column(const GetCanopyDensityGrid::Request &req) {
    CanopyDensityGrid out;
    out.header.frame_id = req.frame_id.empty() ? default_frame_ : req.frame_id;
    out.header.stamp    = now();

    // 1) Configure grid
    GridSpec g{};
    g.min_x = req.min_x; g.max_x = req.max_x;
    g.min_z = req.min_z; g.max_z = req.max_z;
    g.cell_size = req.cell_size;
    g.min_y = req.min_y; g.max_y = req.max_y;
    g.origin_x = g.min_x;
    g.origin_z = g.min_z;

    const double span_x = std::max(0.0, g.max_x - g.min_x);
    const double span_z = std::max(0.0, g.max_z - g.min_z);
    g.width  = static_cast<unsigned>(std::ceil(span_x / g.cell_size));
    g.height = static_cast<unsigned>(std::ceil(span_z / g.cell_size));

    out.cell_size = static_cast<float>(g.cell_size);
    out.origin_x  = static_cast<float>(g.origin_x);
    out.origin_z  = static_cast<float>(g.origin_z);
    out.min_y     = static_cast<float>(g.min_y);
    out.max_y     = static_cast<float>(g.max_y);
    out.width     = g.width;
    out.height    = g.height;

    const size_t Ncells = static_cast<size_t>(g.width) * static_cast<size_t>(g.height);
    out.data.assign(Ncells, std::numeric_limits<float>::quiet_NaN());
    out.point_counts.assign(Ncells, 0);
    out.occupied_voxels.assign(Ncells, 0);

    const double vy = std::max(1e-3, (double)req.vy);
    const uint16_t Ny = static_cast<uint16_t>(
      std::max(0, (int)std::ceil((g.max_y - g.min_y) / vy)));
    out.num_vertical_voxels = Ny;

    if (g.width == 0 || g.height == 0 || Ny == 0) {
      return maybe_apply_ema(out, g, req);
    }

    // 2) Pick clouds in time window
    auto selected = select_clouds(req.time_window);
    if (selected.empty()) {
      return maybe_apply_ema(out, g, req);
    }

    // 3) Collect into x–z grid, track vertical occupancy
    std::vector<std::unordered_set<int>> occ_sets(Ncells);

    for (auto &cptr : selected) {
      sensor_msgs::msg::PointCloud2 cloud_plane;
      try {
        if (cptr->header.frame_id == out.header.frame_id) {
          cloud_plane = *cptr;
        } else {
          geometry_msgs::msg::TransformStamped T =
            tf_buffer_->lookupTransform(
              out.header.frame_id, cptr->header.frame_id, tf2::TimePointZero);
          tf2::doTransform(*cptr, cloud_plane, T);
        }
      } catch (const std::exception &e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TF to canopy frame failed: %s", e.what());
        continue;
      }

      sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud_plane, "x");
      sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud_plane, "y");
      sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud_plane, "z");

      for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
        const double x = *it_x;
        const double y = *it_y;
        const double z = *it_z;

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        if (y < g.min_y || y > g.max_y) continue;

        const int ix = static_cast<int>(std::floor((x - g.min_x) / g.cell_size));
        const int iz = static_cast<int>(std::floor((z - g.min_z) / g.cell_size));
        if (ix < 0 || iz < 0 || ix >= (int)g.width || iz >= (int)g.height) continue;

        const size_t idx = static_cast<size_t>(ix) * g.height + static_cast<size_t>(iz);
        out.point_counts[idx]++;

        if (req.use_column_density) {
          const int iv = static_cast<int>(std::floor((y - g.min_y) / vy));
          if (iv >= 0 && iv < (int)Ny) {
            occ_sets[idx].insert(iv);
          }
        } else {
          if (y >= req.y_min_canopy) {
            occ_sets[idx].insert(0);
          }
        }
      }
    }

    for (size_t idx = 0; idx < Ncells; ++idx) {
      const uint32_t pc  = out.point_counts[idx];
      const uint16_t occ = static_cast<uint16_t>(occ_sets[idx].size());
      out.occupied_voxels[idx] = occ;

      if (pc == 0 || occ == 0) {
        // No points (or no occupied voxels) => zero density, not NaN
        out.data[idx] = 0.0f;
        continue;
      }

      if (req.use_column_density && Ny > 0) {
        out.data[idx] = static_cast<float>(occ) / static_cast<float>(Ny);
      } else {
        out.data[idx] = 1.0f;
      }
    }


    return maybe_apply_ema(out, g, req);
  }

  // ----------------- ORTHO MODE -----------------
  CanopyDensityGrid compute_grid_ortho(const GetCanopyDensityGrid::Request &req) {
    CanopyDensityGrid out;
    out.header.frame_id = req.frame_id.empty() ? default_frame_ : req.frame_id;
    out.header.stamp    = now();

    // Grid on x–z in out.header.frame_id
    GridSpec g{};
    g.min_x = req.min_x; g.max_x = req.max_x;
    g.min_z = req.min_z; g.max_z = req.max_z;
    g.cell_size = req.cell_size;
    g.min_y = req.min_y; g.max_y = req.max_y;
    g.origin_x = g.min_x;
    g.origin_z = g.min_z;

    const double span_x = std::max(0.0, g.max_x - g.min_x);
    const double span_z = std::max(0.0, g.max_z - g.min_z);
    g.width  = static_cast<unsigned>(std::ceil(span_x / g.cell_size));
    g.height = static_cast<unsigned>(std::ceil(span_z / g.cell_size));

    out.cell_size = static_cast<float>(g.cell_size);
    out.origin_x  = static_cast<float>(g.origin_x);
    out.origin_z  = static_cast<float>(g.origin_z);
    out.min_y     = static_cast<float>(g.min_y);
    out.max_y     = static_cast<float>(g.max_y);
    out.width     = g.width;
    out.height    = g.height;

    const size_t Ncells = static_cast<size_t>(g.width) * static_cast<size_t>(g.height);
    out.data.assign(Ncells, std::numeric_limits<float>::quiet_NaN());
    out.point_counts.assign(Ncells, 0);
    out.occupied_voxels.assign(Ncells, 0);
    out.num_vertical_voxels = 0;

    auto latest = buffer_.empty() ? nullptr : buffer_.back().cloud;
    if (!latest) return maybe_apply_ema(out, g, req);

    // Transform to plane frame (usually base_link)
    sensor_msgs::msg::PointCloud2 cloud_plane;
    try {
      if (latest->header.frame_id == out.header.frame_id) {
        cloud_plane = *latest;
      } else {
        geometry_msgs::msg::TransformStamped T =
          tf_buffer_->lookupTransform(
            out.header.frame_id, latest->header.frame_id, tf2::TimePointZero);
        tf2::doTransform(*latest, cloud_plane, T);
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF to ortho plane failed: %s", e.what());
      return maybe_apply_ema(out, g, req);
    }

    pcl::PointCloud<pcl::PointXYZ> pcl_pts;
    pcl::fromROSMsg(cloud_plane, pcl_pts);

    // Per-cell raster on x–z plane
    const double pix = std::max(1e-6, ortho_pixel_size_);
    const int px_per_cell   = std::max(1, (int)std::ceil(g.cell_size / pix));
    const int denom_pixels  = px_per_cell * px_per_cell;

    std::vector<std::unordered_set<int>> cell_pixels(Ncells);

    for (const auto &p : pcl_pts.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (p.y < g.min_y || p.y > g.max_y) continue;

      const int ix = (int)std::floor((p.x - g.min_x) / g.cell_size);
      const int iz = (int)std::floor((p.z - g.min_z) / g.cell_size);
      if (ix < 0 || iz < 0 || ix >= (int)g.width || iz >= (int)g.height) continue;

      const size_t cell_idx = (size_t)ix * g.height + (size_t)iz;

      const double local_x = p.x - (g.min_x + ix * g.cell_size);
      const double local_z = p.z - (g.min_z + iz * g.cell_size);

      const int px = (int)std::floor(local_x / pix);
      const int pz = (int)std::floor(local_z / pix);
      if (px < 0 || pz < 0 || px >= px_per_cell || pz >= px_per_cell) continue;

      const int pix_idx = px * px_per_cell + pz;
      cell_pixels[cell_idx].insert(pix_idx);
    }

    for (size_t idx = 0; idx < Ncells; ++idx) {
      const int occ = (int)cell_pixels[idx].size();
      if (occ == 0) {
        // No covered pixels => zero density
        out.data[idx] = 0.0f;
        continue;
      }
      out.data[idx] = static_cast<float>(occ) / static_cast<float>(denom_pixels);
    }


    return maybe_apply_ema(out, g, req);
  }

  // ----------------- Members -----------------
  std::string cloud_topic_, default_frame_;
  double max_buffer_sec_{2.0};

  bool apply_ema_default_{false};

  std::string mode_;
  double ortho_pixel_size_{0.01};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Service<GetCanopyDensityGrid>::SharedPtr              srv_;

  std::deque<TimedCloud> buffer_;

  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // EMA
  std::vector<float> prev_data_;
  PrevSignature      prev_sig_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanopyDensityServer>());
  rclcpp::shutdown();
  return 0;
}
