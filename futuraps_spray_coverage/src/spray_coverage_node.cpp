// Copyright 2026 Oystein Bondevik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>
#include <unordered_map>
#include <cstdint>
#include <stdexcept>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <cctype>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

#include "futuraps_spray_coverage/spray_model.hpp"

namespace
{

constexpr double kVectorEps = 1e-9;
constexpr double kDoseTargetEps = 1e-9;
constexpr double kMaxDoseColorRatio = 2.0;
constexpr int kMinMarkerSegments = 6;
constexpr int kMarkerSpokes = 8;

}  // namespace

class SprayCoverageNode : public rclcpp::Node
{
public:
  SprayCoverageNode()
  : Node("spray_coverage_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    spray_enabled_(false),
    have_cloud_(false)
  {
    world_frame_ = this->declare_parameter<std::string>("world_frame", "map");
    coverage_frame_ = this->declare_parameter<std::string>("coverage_frame", "odom");
    nozzle_frame_ = this->declare_parameter<std::string>("nozzle_frame", "nozzle_link");
    cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/cloud");
    accumulate_live_cloud_ = this->declare_parameter<bool>("accumulate_live_cloud", true);
    live_cloud_topic_ = this->declare_parameter<std::string>("live_cloud_topic", "/cloud");
    cloud_qos_reliability_ =
      this->declare_parameter<std::string>("cloud_qos_reliability", "reliable");
    cloud_qos_depth_ = this->declare_parameter<int>("cloud_qos_depth", 10);
    spray_topic_ = this->declare_parameter<std::string>("spray_topic", "/spray/enabled");
    dose_cloud_topic_ = this->declare_parameter<std::string>(
      "dose_cloud_topic", "/spray_coverage/dose_cloud");
    coverage_topic_ = this->declare_parameter<std::string>(
      "coverage_topic", "/spray_coverage/coverage");
    spray_marker_topic_ = this->declare_parameter<std::string>(
      "spray_marker_topic", "/spray_coverage/spray_marker");
    update_rate_hz_ = this->declare_parameter<double>("update_rate_hz", 20.0);

    peak_intensity_ = this->declare_parameter<double>("peak_intensity", 1.0);
    fan_angle_deg_ = this->declare_parameter<double>("fan_angle_deg", 80.0);
    narrow_angle_deg_ = this->declare_parameter<double>("narrow_angle_deg", 35.0);
    use_distance_attenuation_ = this->declare_parameter<bool>("use_distance_attenuation", true);
    reference_distance_ = this->declare_parameter<double>("reference_distance", 0.5);
    min_distance_ = this->declare_parameter<double>("min_distance", 0.1);
    attenuation_exponent_ = this->declare_parameter<double>("attenuation_exponent", 2.0);
    max_attenuation_ = this->declare_parameter<double>("max_attenuation", 10.0);
    min_attenuation_ = this->declare_parameter<double>("min_attenuation", 0.01);

    max_range_ = this->declare_parameter<double>("max_range", 1.0);
    leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.03);
    max_point_age_ = this->declare_parameter<double>("max_point_age", 30.0);
    use_recent_cloud_for_occlusion_ =
      this->declare_parameter<bool>("use_recent_cloud_for_occlusion", true);
    target_dose_ = this->declare_parameter<double>("target_dose", 1.0);
    dose_tol_ratio_ = this->declare_parameter<double>("dose_tolerance_ratio", 0.2);

    // Marker visualization params
    marker_line_width_ = this->declare_parameter<double>("marker_line_width", 0.01);
    marker_num_rings_ = this->declare_parameter<int>("marker_num_rings", 10);
    marker_num_segments_ = this->declare_parameter<int>("marker_num_segments", 24);
    marker_show_axis_ = this->declare_parameter<bool>("marker_show_axis", true);

    // Simple occlusion parameters
    occlusion_enabled_ = this->declare_parameter<bool>("occlusion_enabled", true);
    occlusion_cell_size_ = this->declare_parameter<double>("occlusion_cell_size", 0.03);
    occlusion_depth_tolerance_ = this->declare_parameter<double>("occlusion_depth_tolerance", 0.02);

    save_dose_on_shutdown_ = this->declare_parameter<bool>("save_dose_on_shutdown", false);
    dose_output_directory_ = this->declare_parameter<std::string>(
      "dose_output_directory", "/tmp/spray_coverage_results");
    dose_output_filename_ = this->declare_parameter<std::string>(
      "dose_output_filename", "spray_dose_points.csv");

    // Multi-nozzle parameters
    nozzle_offsets_x_ = this->declare_parameter<std::vector<double>>(
      "nozzle_offsets_x", std::vector<double>{0.0});
    nozzle_offsets_y_ = this->declare_parameter<std::vector<double>>(
      "nozzle_offsets_y", std::vector<double>{0.0});
    nozzle_offsets_z_ = this->declare_parameter<std::vector<double>>(
      "nozzle_offsets_z", std::vector<double>{0.0});

    nozzle_axes_x_ = this->declare_parameter<std::vector<double>>(
      "nozzle_axes_x", std::vector<double>{0.0});
    nozzle_axes_y_ = this->declare_parameter<std::vector<double>>(
      "nozzle_axes_y", std::vector<double>{0.0});
    nozzle_axes_z_ = this->declare_parameter<std::vector<double>>(
      "nozzle_axes_z", std::vector<double>{1.0});

    nozzle_wide_axes_x_ = this->declare_parameter<std::vector<double>>(
      "nozzle_wide_axes_x", std::vector<double>{1.0});
    nozzle_wide_axes_y_ = this->declare_parameter<std::vector<double>>(
      "nozzle_wide_axes_y", std::vector<double>{0.0});
    nozzle_wide_axes_z_ = this->declare_parameter<std::vector<double>>(
      "nozzle_wide_axes_z", std::vector<double>{0.0});

    if (!loadNozzlesFromParameters()) {
      throw std::runtime_error("Failed to load nozzle parameters.");
    }

    if (leaf_size_ <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "voxel_leaf_size must be positive; using 0.03 instead of %.6f",
        leaf_size_);
      leaf_size_ = 0.03;
    }

    if (max_point_age_ < 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "max_point_age must be non-negative; using 0.0 instead of %.6f",
        max_point_age_);
      max_point_age_ = 0.0;
    }

    if (cloud_qos_depth_ <= 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "cloud_qos_depth must be positive; using 10 instead of %d",
        cloud_qos_depth_);
      cloud_qos_depth_ = 10;
    }

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      accumulate_live_cloud_ ? live_cloud_topic_ : cloud_topic_,
      makeCloudQos(),
      std::bind(&SprayCoverageNode::cloudCallback, this, std::placeholders::_1));

    spray_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      spray_topic_,
      10,
      std::bind(&SprayCoverageNode::sprayCallback, this, std::placeholders::_1));

    dose_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(dose_cloud_topic_, 10);
    coverage_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(coverage_topic_, 10);
    spray_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      spray_marker_topic_, 10);

    auto period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&SprayCoverageNode::updateLoop, this));

    last_update_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "SprayCoverageNode started.");
    RCLCPP_INFO(
      this->get_logger(),
      "Parameters: world_frame=%s coverage_frame=%s nozzle_frame=%s cloud_topic=%s "
      "live_cloud_topic=%s "
      "accumulate_live_cloud=%s cloud_qos_reliability=%s cloud_qos_depth=%d spray_topic=%s "
      "dose_cloud_topic=%s coverage_topic=%s spray_marker_topic=%s update_rate=%.2fHz "
      "peak_intensity=%.3f fan_angle=%.1f narrow_angle=%.1f max_range=%.3f "
      "voxel_leaf=%.3f target_dose=%.3f tol=%.3f "
      "max_point_age=%.3f use_recent_cloud_for_occlusion=%s "
      "occlusion_enabled=%s occlusion_cell_size=%.3f occlusion_depth_tolerance=%.3f nozzles=%zu",
      world_frame_.c_str(),
      coverage_frame_.c_str(),
      nozzle_frame_.c_str(),
      cloud_topic_.c_str(),
      live_cloud_topic_.c_str(),
      accumulate_live_cloud_ ? "true" : "false",
      cloud_qos_reliability_.c_str(),
      cloud_qos_depth_,
      spray_topic_.c_str(),
      dose_cloud_topic_.c_str(),
      coverage_topic_.c_str(),
      spray_marker_topic_.c_str(),
      update_rate_hz_,
      peak_intensity_, fan_angle_deg_, narrow_angle_deg_, max_range_, leaf_size_,
      target_dose_, dose_tol_ratio_,
      max_point_age_,
      use_recent_cloud_for_occlusion_ ? "true" : "false",
      occlusion_enabled_ ? "true" : "false",
      occlusion_cell_size_,
      occlusion_depth_tolerance_,
      nozzles_.size());

    for (std::size_t i = 0; i < nozzles_.size(); ++i) {
      const auto & n = nozzles_[i];
      RCLCPP_INFO(
        this->get_logger(),
        "Nozzle %zu: offset=(%.3f, %.3f, %.3f), axis=(%.3f, %.3f, %.3f), "
        "wide_axis=(%.3f, %.3f, %.3f)",
        i,
        n.offset_x, n.offset_y, n.offset_z,
        n.axis_x, n.axis_y, n.axis_z,
        n.wide_axis_x, n.wide_axis_y, n.wide_axis_z);
    }
  }

  ~SprayCoverageNode() override
  {
    saveDoseCsvOnShutdown();
  }

private:
  struct CanopyPoint
  {
    float x;
    float y;
    float z;
    float dose;
    double last_seen;
    std::uint32_t observation_count;
  };

  struct VoxelKey
  {
    std::int32_t x;
    std::int32_t y;
    std::int32_t z;

    bool operator==(const VoxelKey & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & key) const
    {
      std::size_t h = static_cast<std::size_t>(static_cast<std::uint32_t>(key.x));
      h ^= static_cast<std::size_t>(static_cast<std::uint32_t>(key.y)) +
        0x9e3779b9 + (h << 6) + (h >> 2);
      h ^= static_cast<std::size_t>(static_cast<std::uint32_t>(key.z)) +
        0x9e3779b9 + (h << 6) + (h >> 2);
      return h;
    }
  };

  struct NozzleConfig
  {
    double offset_x;
    double offset_y;
    double offset_z;
    double axis_x;
    double axis_y;
    double axis_z;
    double wide_axis_x;
    double wide_axis_y;
    double wide_axis_z;
  };

  struct NozzleWorldState
  {
    double x;
    double y;
    double z;

    double dir_x;
    double dir_y;
    double dir_z;

    double wide_x;
    double wide_y;
    double wide_z;
  };

  static std::int64_t makeBinKey(std::int32_t ix, std::int32_t iy)
  {
    return (static_cast<std::int64_t>(ix) << 32) |
           static_cast<std::uint32_t>(iy);
  }

  VoxelKey makeVoxelKey(float x, float y, float z) const
  {
    return {
      static_cast<std::int32_t>(std::floor(static_cast<double>(x) / leaf_size_)),
      static_cast<std::int32_t>(std::floor(static_cast<double>(y) / leaf_size_)),
      static_cast<std::int32_t>(std::floor(static_cast<double>(z) / leaf_size_))};
  }

  const std::string & activeFrame() const
  {
    return accumulate_live_cloud_ ? coverage_frame_ : world_frame_;
  }

  rclcpp::QoS makeCloudQos() const
  {
    rclcpp::QoS qos(static_cast<std::size_t>(cloud_qos_depth_));
    qos.durability_volatile();

    std::string reliability = cloud_qos_reliability_;
    std::transform(
      reliability.begin(),
      reliability.end(),
      reliability.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});

    if (reliability == "best_effort" || reliability == "besteffort") {
      qos.best_effort();
    } else if (reliability == "reliable") {
      qos.reliable();
    } else if (reliability == "system_default" || reliability == "system") {
      qos.reliability(rclcpp::ReliabilityPolicy::SystemDefault);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Unknown cloud_qos_reliability '%s'; using reliable.",
        cloud_qos_reliability_.c_str());
      qos.reliable();
    }

    return qos;
  }

  futuraps_spray_coverage::SprayModelConfig sprayModelConfig() const
  {
    futuraps_spray_coverage::SprayModelConfig config;
    config.peak_intensity = peak_intensity_;
    config.fan_angle_deg = fan_angle_deg_;
    config.narrow_angle_deg = narrow_angle_deg_;
    config.max_range = max_range_;
    config.use_distance_attenuation = use_distance_attenuation_;
    config.reference_distance = reference_distance_;
    config.min_distance = min_distance_;
    config.attenuation_exponent = attenuation_exponent_;
    config.max_attenuation = max_attenuation_;
    config.min_attenuation = min_attenuation_;
    return config;
  }

  futuraps_spray_coverage::SprayFrame makeSprayFrame(
    const NozzleWorldState & nozzle,
    const tf2::Vector3 & dir,
    const tf2::Vector3 & thin,
    const tf2::Vector3 & wide) const
  {
    futuraps_spray_coverage::SprayFrame frame;
    frame.origin = {nozzle.x, nozzle.y, nozzle.z};
    frame.direction = {dir.x(), dir.y(), dir.z()};
    frame.narrow = {thin.x(), thin.y(), thin.z()};
    frame.wide = {wide.x(), wide.y(), wide.z()};
    return frame;
  }

  futuraps_spray_coverage::Vector3 makeSprayPoint(const CanopyPoint & pt) const
  {
    return {
      static_cast<double>(pt.x),
      static_cast<double>(pt.y),
      static_cast<double>(pt.z)};
  }

  bool loadNozzlesFromParameters()
  {
    const std::size_t n = nozzle_offsets_x_.size();

    if (n == 0) {
      RCLCPP_ERROR(this->get_logger(), "No nozzle offsets provided.");
      return false;
    }

    auto same_size = [n](const std::vector<double> & v) {
        return v.size() == n;
      };

    if (!same_size(nozzle_offsets_y_) ||
      !same_size(nozzle_offsets_z_) ||
      !same_size(nozzle_axes_x_) ||
      !same_size(nozzle_axes_y_) ||
      !same_size(nozzle_axes_z_) ||
      !same_size(nozzle_wide_axes_x_) ||
      !same_size(nozzle_wide_axes_y_) ||
      !same_size(nozzle_wide_axes_z_))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Nozzle offset, axis, and wide-axis arrays must all have the same length.");
      return false;
    }

    nozzles_.clear();
    nozzles_.reserve(n);

    for (std::size_t i = 0; i < n; ++i) {
      const double ax = nozzle_axes_x_[i];
      const double ay = nozzle_axes_y_[i];
      const double az = nozzle_axes_z_[i];
      const double norm = std::sqrt(ax * ax + ay * ay + az * az);

      if (norm < kVectorEps) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Nozzle %zu has zero-length axis.", i);
        return false;
      }

      const double wx = nozzle_wide_axes_x_[i];
      const double wy = nozzle_wide_axes_y_[i];
      const double wz = nozzle_wide_axes_z_[i];
      const double wide_norm = std::sqrt(wx * wx + wy * wy + wz * wz);

      if (wide_norm < kVectorEps) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Nozzle %zu has zero-length wide axis.", i);
        return false;
      }

      NozzleConfig nozzle;
      nozzle.offset_x = nozzle_offsets_x_[i];
      nozzle.offset_y = nozzle_offsets_y_[i];
      nozzle.offset_z = nozzle_offsets_z_[i];
      nozzle.axis_x = ax / norm;
      nozzle.axis_y = ay / norm;
      nozzle.axis_z = az / norm;
      nozzle.wide_axis_x = wx / wide_norm;
      nozzle.wide_axis_y = wy / wide_norm;
      nozzle.wide_axis_z = wz / wide_norm;

      nozzles_.push_back(nozzle);
    }

    return true;
  }

  std::vector<NozzleWorldState> computeNozzleWorldStates(
    const geometry_msgs::msg::TransformStamped & tf_msg) const
  {
    std::vector<NozzleWorldState> world_nozzles;
    world_nozzles.reserve(nozzles_.size());

    tf2::Quaternion q;
    tf2::fromMsg(tf_msg.transform.rotation, q);
    tf2::Matrix3x3 R(q);

    const tf2::Vector3 t(
      tf_msg.transform.translation.x,
      tf_msg.transform.translation.y,
      tf_msg.transform.translation.z);

    for (const auto & nozzle : nozzles_) {
      tf2::Vector3 local_offset(nozzle.offset_x, nozzle.offset_y, nozzle.offset_z);
      tf2::Vector3 local_axis(nozzle.axis_x, nozzle.axis_y, nozzle.axis_z);
      tf2::Vector3 local_wide_axis(
        nozzle.wide_axis_x,
        nozzle.wide_axis_y,
        nozzle.wide_axis_z);

      tf2::Vector3 world_pos = t + R * local_offset;
      tf2::Vector3 world_axis = R * local_axis;
      tf2::Vector3 world_wide = R * local_wide_axis;

      if (world_axis.length() < kVectorEps) {
        continue;
      }
      world_axis.normalize();

      // Project wide direction onto plane perpendicular to spray direction
      world_wide = world_wide - world_wide.dot(world_axis) * world_axis;

      if (world_wide.length() < kVectorEps) {
        continue;
      }
      world_wide.normalize();

      NozzleWorldState s;
      s.x = world_pos.x();
      s.y = world_pos.y();
      s.z = world_pos.z();

      s.dir_x = world_axis.x();
      s.dir_y = world_axis.y();
      s.dir_z = world_axis.z();

      s.wide_x = world_wide.x();
      s.wide_y = world_wide.y();
      s.wide_z = world_wide.z();

      world_nozzles.push_back(s);
    }

    return world_nozzles;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      3000,
      "cloudCallback triggered. Incoming cloud frame='%s' mode=%s",
      msg->header.frame_id.c_str(),
      accumulate_live_cloud_ ? "live" : "static");

    if (!accumulate_live_cloud_ && have_cloud_) {
      RCLCPP_DEBUG(this->get_logger(), "Cloud already stored, ignoring new cloud.");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *raw_cloud);

    if (raw_cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty cloud.");
      return;
    }

    if (accumulate_live_cloud_) {
      insertLiveCloud(*raw_cloud, msg->header);
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(raw_cloud);
    vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    vg.filter(*filtered_cloud);

    canopy_points_.clear();
    canopy_points_.reserve(filtered_cloud->size());

    std::size_t invalid_points = 0;
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    for (const auto & p : filtered_cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        ++invalid_points;
        continue;
      }
      canopy_points_.push_back({p.x, p.y, p.z, 0.0f, stamp, 1U});
    }

    have_cloud_ = !canopy_points_.empty();

    RCLCPP_INFO(
      this->get_logger(),
      "Stored canopy snapshot: raw=%zu, downsampled=%zu, valid=%zu, invalid=%zu",
      raw_cloud->size(),
      filtered_cloud->size(),
      canopy_points_.size(),
      invalid_points);

    if (!have_cloud_) {
      RCLCPP_WARN(this->get_logger(), "After filtering, no valid canopy points remained.");
      return;
    }

    publishDoseCloud();
  }

  void insertLiveCloud(
    const pcl::PointCloud<pcl::PointXYZ> & raw_cloud,
    const std_msgs::msg::Header & header)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(
        coverage_frame_,
        header.frame_id,
        rclcpp::Time(header.stamp));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Skipping live cloud: no TF from '%s' to coverage_frame '%s' at cloud stamp: %s",
        header.frame_id.c_str(),
        coverage_frame_.c_str(),
        ex.what());
      return;
    }

    tf2::Quaternion q;
    tf2::fromMsg(tf_msg.transform.rotation, q);
    tf2::Matrix3x3 R(q);
    const tf2::Vector3 t(
      tf_msg.transform.translation.x,
      tf_msg.transform.translation.y,
      tf_msg.transform.translation.z);

    const double stamp = rclcpp::Time(header.stamp).seconds();
    std::size_t inserted = 0;
    std::size_t updated = 0;
    std::size_t invalid = 0;

    for (const auto & p : raw_cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        ++invalid;
        continue;
      }

      const tf2::Vector3 local(p.x, p.y, p.z);
      const tf2::Vector3 transformed = t + R * local;
      const float x = static_cast<float>(transformed.x());
      const float y = static_cast<float>(transformed.y());
      const float z = static_cast<float>(transformed.z());

      const VoxelKey key = makeVoxelKey(x, y, z);
      auto it = voxel_index_.find(key);
      if (it == voxel_index_.end()) {
        voxel_index_.emplace(key, canopy_points_.size());
        canopy_points_.push_back({x, y, z, 0.0f, stamp, 1U});
        ++inserted;
        continue;
      }

      CanopyPoint & pt = canopy_points_[it->second];
      const double count = static_cast<double>(std::max<std::uint32_t>(pt.observation_count, 1U));
      const double alpha = 1.0 / std::min(count + 1.0, 50.0);
      pt.x = static_cast<float>(
        (1.0 - alpha) * static_cast<double>(pt.x) + alpha * static_cast<double>(x));
      pt.y = static_cast<float>(
        (1.0 - alpha) * static_cast<double>(pt.y) + alpha * static_cast<double>(y));
      pt.z = static_cast<float>(
        (1.0 - alpha) * static_cast<double>(pt.z) + alpha * static_cast<double>(z));
      pt.last_seen = stamp;
      if (pt.observation_count < std::numeric_limits<std::uint32_t>::max()) {
        ++pt.observation_count;
      }
      ++updated;
    }

    have_cloud_ = !canopy_points_.empty();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Merged live cloud into %s: raw=%zu inserted=%zu updated=%zu invalid=%zu voxels=%zu",
      coverage_frame_.c_str(),
      raw_cloud.size(),
      inserted,
      updated,
      invalid,
      canopy_points_.size());

    if (have_cloud_) {
      publishDoseCloud();
    }
  }

  void saveDoseCsvOnShutdown()
  {
    if (!save_dose_on_shutdown_) {
      return;
    }

    if (canopy_points_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No canopy point cloud loaded; not saving spray dose CSV.");
      return;
    }

    std::error_code ec;
    const std::filesystem::path output_dir(dose_output_directory_);
    std::filesystem::create_directories(output_dir, ec);
    if (ec) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to create spray dose output directory '%s': %s",
        output_dir.string().c_str(),
        ec.message().c_str());
      return;
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &now_time);
#else
    localtime_r(&now_time, &tm);
#endif

    std::ostringstream timestamp;
    timestamp << std::put_time(&tm, "%Y%m%d_%H%M%S");

    const std::filesystem::path filename(dose_output_filename_);
    const std::filesystem::path output_path =
      output_dir / (timestamp.str() + "_" + filename.filename().string());

    std::ofstream file(output_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to open spray dose CSV for writing: %s",
        output_path.string().c_str());
      return;
    }

    file << std::setprecision(std::numeric_limits<float>::max_digits10);
    file << "x,y,z,dose\n";
    for (const auto & pt : canopy_points_) {
      file << pt.x << "," << pt.y << "," << pt.z << "," << pt.dose << "\n";
    }

    if (!file.good()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed while writing spray dose CSV: %s",
        output_path.string().c_str());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Saved spray dose CSV: %s",
      output_path.string().c_str());
  }

  void sprayCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    spray_enabled_ = msg->data;
    RCLCPP_INFO(
      this->get_logger(), "Received spray enable topic: spray_enabled=%s",
      spray_enabled_ ? "true" : "false");
  }

  void updateLoop()
  {
    const rclcpp::Time now = this->now();
    const double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "updateLoop alive. dt=%.4f have_cloud=%s spray_enabled=%s canopy_points=%zu",
      dt,
      have_cloud_ ? "true" : "false",
      spray_enabled_ ? "true" : "false",
      canopy_points_.size());

    if (!have_cloud_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Waiting for first canopy cloud on topic '%s' ...",
        (accumulate_live_cloud_ ? live_cloud_topic_ : cloud_topic_).c_str());
      return;
    }

    publishDoseCloud();
    publishCoverageMetric();

    if (!spray_enabled_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        3000,
        "Spray disabled. Waiting for topic '%s' to become true.",
        spray_topic_.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(activeFrame(), nozzle_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Could not get nozzle transform from '%s' to '%s': %s",
        activeFrame().c_str(),
        nozzle_frame_.c_str(),
        ex.what());
      return;
    }

    const auto world_nozzles = computeNozzleWorldStates(tf_msg);
    if (world_nozzles.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "No valid world nozzles could be computed.");
      return;
    }

    publishSprayMarkers(world_nozzles);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "TF OK. Boom pose=(%.3f, %.3f, %.3f), nozzles=%zu, dt=%.4f",
      tf_msg.transform.translation.x,
      tf_msg.transform.translation.y,
      tf_msg.transform.translation.z,
      world_nozzles.size(),
      dt);

    const std::size_t affected_points = applySprayToCanopy(world_nozzles, dt);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Spray update applied. affected_points=%zu / %zu",
      affected_points,
      canopy_points_.size());

    publishDoseCloud();
    publishCoverageMetric();
    printDoseStats();
  }

  void publishSprayMarkers(const std::vector<NozzleWorldState> & world_nozzles)
  {
    for (std::size_t nozzle_idx = 0; nozzle_idx < world_nozzles.size(); ++nozzle_idx) {
      const auto & nozzle = world_nozzles[nozzle_idx];

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = activeFrame();
      marker.header.stamp = this->now();
      marker.ns = "spray_model";
      marker.id = static_cast<int>(nozzle_idx);
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = marker_line_width_;

      marker.color.r = 1.0;
      marker.color.g = 0.5;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      tf2::Vector3 dir(nozzle.dir_x, nozzle.dir_y, nozzle.dir_z);
      if (dir.length() < kVectorEps) {
        continue;
      }
      dir.normalize();

      tf2::Vector3 wide(nozzle.wide_x, nozzle.wide_y, nozzle.wide_z);
      if (wide.length() < kVectorEps) {
        continue;
      }
      wide.normalize();

      tf2::Vector3 thin = dir.cross(wide);
      if (thin.length() < kVectorEps) {
        continue;
      }
      thin.normalize();

      const auto model_config = sprayModelConfig();

      if (marker_show_axis_) {
        geometry_msgs::msg::Point p0, p1;
        p0.x = nozzle.x;
        p0.y = nozzle.y;
        p0.z = nozzle.z;

        p1.x = nozzle.x + max_range_ * dir.x();
        p1.y = nozzle.y + max_range_ * dir.y();
        p1.z = nozzle.z + max_range_ * dir.z();

        marker.points.push_back(p0);
        marker.points.push_back(p1);
      }

      const int num_rings = std::max(1, marker_num_rings_);
      const int num_segments = std::max(kMinMarkerSegments, marker_num_segments_);

      for (int i = 1; i <= num_rings; ++i) {
        const double s = max_range_ * (static_cast<double>(i) / static_cast<double>(num_rings));

        const double wide_semi_axis =
          futuraps_spray_coverage::wideSemiAxisAtDistance(s, model_config);
        const double narrow_semi_axis =
          futuraps_spray_coverage::narrowSemiAxisAtDistance(s, model_config);

        std::vector<geometry_msgs::msg::Point> ring;
        ring.reserve(num_segments);

        for (int j = 0; j < num_segments; ++j) {
          const double theta =
            2.0 * M_PI * static_cast<double>(j) / static_cast<double>(num_segments);

          tf2::Vector3 offset =
            s * dir +
            narrow_semi_axis * std::cos(theta) * thin +
            wide_semi_axis * std::sin(theta) * wide;

          geometry_msgs::msg::Point p;
          p.x = nozzle.x + offset.x();
          p.y = nozzle.y + offset.y();
          p.z = nozzle.z + offset.z();
          ring.push_back(p);
        }

        for (int j = 0; j < num_segments; ++j) {
          marker.points.push_back(ring[j]);
          marker.points.push_back(ring[(j + 1) % num_segments]);
        }

        if (i == 1) {
          for (int j = 0; j < kMarkerSpokes; ++j) {
            const int idx = (j * num_segments) / kMarkerSpokes;

            geometry_msgs::msg::Point nozzle_origin;
            nozzle_origin.x = nozzle.x;
            nozzle_origin.y = nozzle.y;
            nozzle_origin.z = nozzle.z;

            marker.points.push_back(nozzle_origin);
            marker.points.push_back(ring[idx]);
          }
        }
      }

      spray_marker_pub_->publish(marker);
    }
  }

  std::size_t applySprayToCanopy(
    const std::vector<NozzleWorldState> & world_nozzles,
    double dt)
  {
    if (dt <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "Non-positive dt=%.6f, skipping spray update.", dt);
      return 0;
    }

    std::size_t affected_points_total = 0;
    std::size_t occluded_points_total = 0;
    const auto model_config = sprayModelConfig();
    const double now_seconds = this->now().seconds();

    for (const auto & nozzle : world_nozzles) {
      tf2::Vector3 dir(nozzle.dir_x, nozzle.dir_y, nozzle.dir_z);
      if (dir.length() < kVectorEps) {
        continue;
      }
      dir.normalize();

      tf2::Vector3 wide(nozzle.wide_x, nozzle.wide_y, nozzle.wide_z);
      if (wide.length() < kVectorEps) {
        continue;
      }
      wide.normalize();

      tf2::Vector3 thin = dir.cross(wide);
      if (thin.length() < kVectorEps) {
        continue;
      }
      thin.normalize();

      const auto spray_frame =
        makeSprayFrame(nozzle, dir, thin, wide);

      std::unordered_map<std::int64_t, double> front_depth_by_bin;
      if (occlusion_enabled_) {
        front_depth_by_bin.reserve(canopy_points_.size() / 4 + 1);

        for (const auto & pt : canopy_points_) {
          if (!pointUsableForOcclusion(pt, now_seconds)) {
            continue;
          }

          const auto projection =
            futuraps_spray_coverage::projectPointToSpray(
            makeSprayPoint(pt),
            spray_frame,
            model_config);

          if (!projection.inside_footprint) {
            continue;
          }

          const std::int32_t bin_u =
            static_cast<std::int32_t>(std::floor(projection.p_narrow / occlusion_cell_size_));
          const std::int32_t bin_v =
            static_cast<std::int32_t>(std::floor(projection.p_wide / occlusion_cell_size_));
          const std::int64_t key = makeBinKey(bin_u, bin_v);

          auto it = front_depth_by_bin.find(key);
          if (it == front_depth_by_bin.end()) {
            front_depth_by_bin.emplace(key, projection.s);
          } else if (projection.s < it->second) {
            it->second = projection.s;
          }
        }
      }

      for (auto & pt : canopy_points_) {
        const auto projection =
          futuraps_spray_coverage::projectPointToSpray(
          makeSprayPoint(pt),
          spray_frame,
          model_config);

        if (!projection.inside_footprint) {
          continue;
        }

        if (occlusion_enabled_) {
          const std::int32_t bin_u =
            static_cast<std::int32_t>(std::floor(projection.p_narrow / occlusion_cell_size_));
          const std::int32_t bin_v =
            static_cast<std::int32_t>(std::floor(projection.p_wide / occlusion_cell_size_));
          const std::int64_t key = makeBinKey(bin_u, bin_v);

          auto it = front_depth_by_bin.find(key);
          if (it != front_depth_by_bin.end()) {
            const double front_s = it->second;
            if (projection.s > front_s + occlusion_depth_tolerance_) {
              ++occluded_points_total;
              continue;
            }
          }
        }

        double intensity =
          futuraps_spray_coverage::computeIntensity(
          projection,
          model_config);

        if (intensity <= 0.0) {
          continue;
        }

        pt.dose += static_cast<float>(intensity * dt);
        ++affected_points_total;
      }
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Occlusion: enabled=%s, occluded_points=%zu",
      occlusion_enabled_ ? "true" : "false",
      occluded_points_total);

    return affected_points_total;
  }

  bool pointUsableForOcclusion(const CanopyPoint & pt, double now_seconds) const
  {
    if (!accumulate_live_cloud_ || !use_recent_cloud_for_occlusion_) {
      return true;
    }

    return (now_seconds - pt.last_seen) <= max_point_age_;
  }

  geometry_msgs::msg::Vector3 computeCoverageFractions() const
  {
    geometry_msgs::msg::Vector3 fractions;
    fractions.x = 0.0;  // undersprayed
    fractions.y = 0.0;  // well-sprayed
    fractions.z = 0.0;  // oversprayed

    if (canopy_points_.empty()) {
      return fractions;
    }

    const double dmin = target_dose_ * (1.0 - dose_tol_ratio_);
    const double dmax = target_dose_ * (1.0 + dose_tol_ratio_);

    std::size_t under = 0;
    std::size_t good = 0;
    std::size_t over = 0;

    for (const auto & pt : canopy_points_) {
      if (pt.dose < dmin) {
        ++under;
      } else if (pt.dose > dmax) {
        ++over;
      } else {
        ++good;
      }
    }

    const double total = static_cast<double>(canopy_points_.size());
    fractions.x = static_cast<double>(under) / total;
    fractions.y = static_cast<double>(good) / total;
    fractions.z = static_cast<double>(over) / total;

    return fractions;
  }

  void publishCoverageMetric()
  {
    geometry_msgs::msg::Vector3 msg = computeCoverageFractions();
    coverage_pub_->publish(msg);

    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Published coverage fractions: under=%.4f good=%.4f over=%.4f",
      msg.x, msg.y, msg.z);
  }

  void publishDoseCloud()
  {
    if (canopy_points_.empty()) {
      return;
    }

    float min_dose = std::numeric_limits<float>::max();
    float max_dose = std::numeric_limits<float>::lowest();

    for (const auto & pt : canopy_points_) {
      min_dose = std::min(min_dose, pt.dose);
      max_dose = std::max(max_dose, pt.dose);
    }

    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    color_cloud.header.frame_id = activeFrame();
    color_cloud.points.reserve(canopy_points_.size());

    for (const auto & pt : canopy_points_) {
      pcl::PointXYZRGB p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;

      uint8_t r, g, b;
      colorFromDoseContinuous(pt.dose, target_dose_, r, g, b);

      p.r = r;
      p.g = g;
      p.b = b;

      color_cloud.points.push_back(p);
    }

    color_cloud.width = static_cast<uint32_t>(color_cloud.points.size());
    color_cloud.height = 1;
    color_cloud.is_dense = true;

    sensor_msgs::msg::PointCloud2 ros_msg;
    pcl::toROSMsg(color_cloud, ros_msg);
    ros_msg.header.stamp = this->now();
    ros_msg.header.frame_id = activeFrame();
    dose_cloud_pub_->publish(ros_msg);

    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Published dose cloud with %zu points. dose[min=%.4f max=%.4f]",
      canopy_points_.size(),
      min_dose,
      max_dose);
  }

  void printDoseStats()
  {
    if (canopy_points_.empty()) {
      return;
    }

    double min_dose = std::numeric_limits<double>::max();
    double max_dose = std::numeric_limits<double>::lowest();
    double sum_dose = 0.0;

    for (const auto & pt : canopy_points_) {
      min_dose = std::min(min_dose, static_cast<double>(pt.dose));
      max_dose = std::max(max_dose, static_cast<double>(pt.dose));
      sum_dose += static_cast<double>(pt.dose);
    }

    const double mean_dose = sum_dose / static_cast<double>(canopy_points_.size());
    const auto fractions = computeCoverageFractions();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Dose stats: min=%.4f max=%.4f mean=%.4f | under=%.4f good=%.4f over=%.4f",
      min_dose,
      max_dose,
      mean_dose,
      fractions.x,
      fractions.y,
      fractions.z);
  }

  static void colorFromDoseContinuous(
    double dose,
    double target,
    uint8_t & r,
    uint8_t & g,
    uint8_t & b)
  {
    const double normalized = dose / std::max(target, kDoseTargetEps);

    // Scale around target:
    // 0.0 -> blue
    // 1.0 -> green
    // 2.0 or higher -> red
    const double x = std::clamp(normalized, 0.0, kMaxDoseColorRatio);

    if (x <= 1.0) {
      // Blue -> Green
      const double t = x;

      r = 0;
      g = static_cast<uint8_t>(255.0 * t);
      b = static_cast<uint8_t>(255.0 * (1.0 - t));
    } else {
      // Green -> Red
      const double t = x - 1.0;

      r = static_cast<uint8_t>(255.0 * t);
      g = static_cast<uint8_t>(255.0 * (1.0 - t));
      b = 0;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spray_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dose_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr coverage_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spray_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<CanopyPoint> canopy_points_;
  std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> voxel_index_;
  std::vector<NozzleConfig> nozzles_;

  bool spray_enabled_;
  bool have_cloud_;

  rclcpp::Time last_update_time_;

  std::string world_frame_;
  std::string coverage_frame_;
  std::string nozzle_frame_;
  std::string cloud_topic_;
  bool accumulate_live_cloud_;
  std::string live_cloud_topic_;
  std::string cloud_qos_reliability_;
  int cloud_qos_depth_;
  std::string spray_topic_;
  std::string dose_cloud_topic_;
  std::string coverage_topic_;
  std::string spray_marker_topic_;

  double update_rate_hz_;
  double peak_intensity_;
  double fan_angle_deg_;
  double narrow_angle_deg_;
  bool use_distance_attenuation_;
  double reference_distance_;
  double min_distance_;
  double attenuation_exponent_;
  double max_attenuation_;
  double min_attenuation_;
  double max_range_;
  double leaf_size_;
  double max_point_age_;
  bool use_recent_cloud_for_occlusion_;
  double target_dose_;
  double dose_tol_ratio_;

  double marker_line_width_;
  int marker_num_rings_;
  int marker_num_segments_;
  bool marker_show_axis_;

  bool occlusion_enabled_;
  double occlusion_cell_size_;
  double occlusion_depth_tolerance_;

  bool save_dose_on_shutdown_;
  std::string dose_output_directory_;
  std::string dose_output_filename_;

  std::vector<double> nozzle_offsets_x_;
  std::vector<double> nozzle_offsets_y_;
  std::vector<double> nozzle_offsets_z_;
  std::vector<double> nozzle_axes_x_;
  std::vector<double> nozzle_axes_y_;
  std::vector<double> nozzle_axes_z_;
  std::vector<double> nozzle_wide_axes_x_;
  std::vector<double> nozzle_wide_axes_y_;
  std::vector<double> nozzle_wide_axes_z_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SprayCoverageNode>());
  rclcpp::shutdown();
  return 0;
}
