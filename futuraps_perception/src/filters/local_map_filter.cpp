#include "futuraps_perception/local_map_filter.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <chrono> 

namespace futuraps {

using pclRGB = pcl::PointXYZRGB;

static inline float clamp01(float v) { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }

LocalMapFilterNode::LocalMapFilterNode(const rclcpp::NodeOptions &options)
: Node("local_map_filter", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // IO
  input_topic_  = this->declare_parameter<std::string>("input_topic",  "/local_map");
  output_topic_ = this->declare_parameter<std::string>("output_topic", "/local_map_for_extractor");

  // Frames
  target_frame_        = this->declare_parameter<std::string>("target_frame", "base_link");
  transform_to_target_ = this->declare_parameter<bool>("transform_to_target", true);

  // Height filter
  enable_height_filter_ = this->declare_parameter<bool>("enable_height_filter", true);
  bottom_cut_m_         = this->declare_parameter<double>("bottom_cut_m", 0.20);
  top_cut_m_            = this->declare_parameter<double>("top_cut_m",  3.00);

  // Outlier cleanup
  enable_radius_outlier_ = this->declare_parameter<bool>("enable_radius_outlier", true);
  radius_                = this->declare_parameter<double>("radius", 0.06);
  min_neighbors_         = this->declare_parameter<int>("min_neighbors", 8);

  // Voxel
  enable_voxel_ = this->declare_parameter<bool>("enable_voxel_downsample", false);
  voxel_leaf_   = this->declare_parameter<double>("voxel_leaf", 0.01);

  // ExG
  use_exg_            = this->declare_parameter<bool>("use_exg", false);
  normalize_rgb_      = this->declare_parameter<bool>("normalize_rgb", true);
  exg_threshold_      = this->declare_parameter<double>("exg_threshold", 0.05);
  drop_low_intensity_ = this->declare_parameter<bool>("drop_low_intensity", true);
  min_intensity_      = this->declare_parameter<double>("min_intensity", 0.05);

  // Debug
  publish_debug_topics_ = this->declare_parameter<bool>("publish_debug_topics", false);

  // Param callback
  param_cb_ = this->add_on_set_parameters_callback(
    std::bind(&LocalMapFilterNode::onParamChange, this, std::placeholders::_1)
  );

  // Sub/Pub
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, qos_, std::bind(&LocalMapFilterNode::cloudCallback, this, std::placeholders::_1)
  );
  pub_out_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos_);

  if (publish_debug_topics_) {
    pub_debug_after_height_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug_after_height", qos_);
    pub_debug_after_ror_    = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug_after_ror", qos_);
  }

  RCLCPP_INFO(get_logger(), "LocalMapFilterNode ready. Sub: %s  Pub: %s",
              input_topic_.c_str(), output_topic_.c_str());
}

rcl_interfaces::msg::SetParametersResult
LocalMapFilterNode::onParamChange(const std::vector<rclcpp::Parameter>& params)
{
  for (const auto &p : params) {
    const auto &n = p.get_name();
    if      (n == "bottom_cut_m")            bottom_cut_m_ = p.as_double();
    else if (n == "top_cut_m")               top_cut_m_ = p.as_double();
    else if (n == "enable_height_filter")    enable_height_filter_ = p.as_bool();
    else if (n == "enable_radius_outlier")   enable_radius_outlier_ = p.as_bool();
    else if (n == "radius")                  radius_ = p.as_double();
    else if (n == "min_neighbors")           min_neighbors_ = p.as_int();
    else if (n == "enable_voxel_downsample") enable_voxel_ = p.as_bool();
    else if (n == "voxel_leaf")              voxel_leaf_ = p.as_double();
    else if (n == "use_exg")                 use_exg_ = p.as_bool();
    else if (n == "normalize_rgb")           normalize_rgb_ = p.as_bool();
    else if (n == "exg_threshold")           exg_threshold_ = p.as_double();
    else if (n == "drop_low_intensity")      drop_low_intensity_ = p.as_bool();
    else if (n == "min_intensity")           min_intensity_ = p.as_double();
    else if (n == "transform_to_target")     transform_to_target_ = p.as_bool();
    else if (n == "target_frame")            target_frame_ = p.as_string();
  }
  rcl_interfaces::msg::SetParametersResult res; res.successful = true; return res;
}

void LocalMapFilterNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg) return;

  // --- Start timing ---
  auto t_start = std::chrono::steady_clock::now();

  // 0) Transform to target frame (so Z is robot-up)
  sensor_msgs::msg::PointCloud2 cloud_tf = *msg;
  if (transform_to_target_ && msg->header.frame_id != target_frame_) {
    try {
      auto tf = tf_buffer_.lookupTransform(
        target_frame_, msg->header.frame_id, rclcpp::Time(msg->header.stamp));
      tf2::doTransform(*msg, cloud_tf, tf);
      cloud_tf.header.stamp = msg->header.stamp;
      cloud_tf.header.frame_id = target_frame_;
    } catch (const tf2::TransformException &ex) {
      try {
        auto tf = tf_buffer_.lookupTransform(
          target_frame_, msg->header.frame_id, tf2::TimePointZero);
        tf2::doTransform(*msg, cloud_tf, tf);
        cloud_tf.header.stamp = msg->header.stamp;
        cloud_tf.header.frame_id = target_frame_;
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                             "TF %s->%s failed at cloud stamp; using latest TF fallback: %s",
                             msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      } catch (const tf2::TransformException &latest_ex) {
        auto t_end = std::chrono::steady_clock::now();
        double dt_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

        std::size_t in_points =
          static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);

        // Log failure timing as well (with 0 output points)
        RCLCPP_INFO(
          get_logger(),
          "PROC_TIME,node=local_map_filter,dt_ms=%.3f,in_pts=%zu,out_pts=%zu",
          dt_ms, in_points, static_cast<std::size_t>(0));

        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                             "Latest TF fallback %s->%s failed: %s",
                             msg->header.frame_id.c_str(), target_frame_.c_str(), latest_ex.what());
        return;
      }
    }
  }

  // Count input points (after TF but equivalent)
  std::size_t in_points =
    static_cast<std::size_t>(cloud_tf.width) * static_cast<std::size_t>(cloud_tf.height);

  // Convert to PCL
  pcl::PointCloud<pclRGB>::Ptr cloud(new pcl::PointCloud<pclRGB>());
  pcl::fromROSMsg(cloud_tf, *cloud);

  // 1) Height filter
  if (enable_height_filter_) {
    pcl::PassThrough<pclRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(bottom_cut_m_, top_cut_m_);
    pcl::PointCloud<pclRGB>::Ptr tmp(new pcl::PointCloud<pclRGB>());
    pass.filter(*tmp);
    cloud.swap(tmp);
  }

  if (publish_debug_topics_) {
    sensor_msgs::msg::PointCloud2 dbg;
    pcl::toROSMsg(*cloud, dbg);
    dbg.header.stamp = cloud_tf.header.stamp;
    dbg.header.frame_id = transform_to_target_ ? target_frame_ : cloud_tf.header.frame_id;
    pub_debug_after_height_->publish(dbg);
  }

  // 2) Radius outlier removal
  if (enable_radius_outlier_) {
    pcl::RadiusOutlierRemoval<pclRGB> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(radius_);
    ror.setMinNeighborsInRadius(min_neighbors_);
    pcl::PointCloud<pclRGB>::Ptr tmp(new pcl::PointCloud<pclRGB>());
    ror.filter(*tmp);
    cloud.swap(tmp);
  }

  if (publish_debug_topics_) {
    sensor_msgs::msg::PointCloud2 dbg;
    pcl::toROSMsg(*cloud, dbg);
    dbg.header.stamp = cloud_tf.header.stamp;
    dbg.header.frame_id = transform_to_target_ ? target_frame_ : cloud_tf.header.frame_id;
    pub_debug_after_ror_->publish(dbg);
  }

  // 3) Voxel
  if (enable_voxel_) {
    pcl::VoxelGrid<pclRGB> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
    pcl::PointCloud<pclRGB>::Ptr tmp(new pcl::PointCloud<pclRGB>());
    vox.filter(*tmp);
    cloud.swap(tmp);
  }

  // 4) Optional ExG masking
  if (use_exg_) {
    pcl::PointCloud<pclRGB>::Ptr masked(new pcl::PointCloud<pclRGB>());
    masked->reserve(cloud->size());
    for (const auto &p : cloud->points) {
      // Normalize RGB to [0,1] if requested
      float r = normalize_rgb_ ? (p.r / 255.f) : static_cast<float>(p.r);
      float g = normalize_rgb_ ? (p.g / 255.f) : static_cast<float>(p.g);
      float b = normalize_rgb_ ? (p.b / 255.f) : static_cast<float>(p.b);

      r = clamp01(r); g = clamp01(g); b = clamp01(b);

      float intensity = (r + g + b) / (normalize_rgb_ ? 3.f : 765.f);
      if (drop_low_intensity_ && intensity < static_cast<float>(min_intensity_)) continue;

      // Simple ExG = 2g - r - b (scaled)
      float exg = (2.0f * g) - r - b;

      if (exg >= static_cast<float>(exg_threshold_)) {
        masked->push_back(p);
      }
    }
    masked->width = static_cast<uint32_t>(masked->size());
    masked->height = 1;
    masked->is_dense = false;
    cloud.swap(masked);
  }

  // Publish final output
  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(*cloud, out);
  out.header.stamp = cloud_tf.header.stamp;
  out.header.frame_id = transform_to_target_ ? target_frame_ : cloud_tf.header.frame_id;
  pub_out_->publish(out);

  // --- End timing + log ---
  std::size_t out_points = static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height);
  auto t_end = std::chrono::steady_clock::now();
  double dt_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  RCLCPP_INFO(
    get_logger(),
    "PROC_TIME,node=local_map_filter,dt_ms=%.3f,in_pts=%zu,out_pts=%zu",
    dt_ms, in_points, out_points);
}


} // namespace futuraps

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(futuraps::LocalMapFilterNode)
