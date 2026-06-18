#include <memory>
#include <string>
#include <chrono>
#include <algorithm>

#include "futuraps_perception/crop_box_filter.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

namespace futuraps
{

CropBoxFilterNode::CropBoxFilterNode(const rclcpp::NodeOptions & options)
: Node("local_cloud_filter", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Parameters
  output_frame_   = this->declare_parameter<std::string>("output_frame", "base_link"); // crop frame
  this->declare_parameter<std::string>("publish_frame", "map");                         // final publish frame

  min_x_ = this->declare_parameter<double>("min_x", -4.0);
  max_x_ = this->declare_parameter<double>("max_x",  1.0);
  min_y_ = this->declare_parameter<double>("min_y", -2.0);
  max_y_ = this->declare_parameter<double>("max_y",  2.0);
  min_z_ = this->declare_parameter<double>("min_z", -0.2);
  max_z_ = this->declare_parameter<double>("max_z",  2.0);

  leaf_size_ = this->declare_parameter<double>("leaf_size", 0.03); // <= 0 disables voxel

  this->declare_parameter<bool>("use_latest_tf", false);
  this->declare_parameter<double>("tf_timeout_sec", 0.2);

  this->declare_parameter<bool>("publish_marker", true);
  this->declare_parameter<double>("marker_alpha", 0.15);
  const std::string input_qos = this->declare_parameter<std::string>("input_qos", "transient_local");

  rclcpp::QoS input_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  if (input_qos == "sensor_data") {
    input_qos_profile = rclcpp::SensorDataQoS();
  } else if (input_qos != "transient_local") {
    RCLCPP_WARN(
      get_logger(),
      "Unknown input_qos '%s'; using transient_local. Valid values: transient_local, sensor_data",
      input_qos.c_str());
  }

  using std::placeholders::_1;
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", input_qos_profile,
    std::bind(&CropBoxFilterNode::cloudCallback, this, _1));

  // Publisher QoS (Reliable + Transient Local so RViz sees last cloud)
  rclcpp::QoS out_qos(rclcpp::KeepLast(1));
  out_qos.reliable();
  out_qos.transient_local();
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", out_qos);

  // Marker publisher (latched)
  rclcpp::QoS marker_qos(1);
  marker_qos.reliable();
  marker_qos.transient_local();
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("local_box_marker", marker_qos);

  // Allow live param tweaks
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CropBoxFilterNode::onParamSet, this, std::placeholders::_1));

  // Publish the initial marker
  publishMarker();

  RCLCPP_INFO(get_logger(),
    "LocalCloudFilter ready. input_qos='%s', crop frame='%s' -> publish frame='%s', box[x:(%.2f,%.2f) y:(%.2f,%.2f) z:(%.2f,%.2f)] leaf=%.3f",
    input_qos.c_str(),
    output_frame_.c_str(),
    this->get_parameter("publish_frame").as_string().c_str(),
    min_x_, max_x_, min_y_, max_y_, min_z_, max_z_, leaf_size_);
}

rcl_interfaces::msg::SetParametersResult
CropBoxFilterNode::onParamSet(const std::vector<rclcpp::Parameter> & params)
{
  for (const auto & p : params)
  {
    const auto & n = p.get_name();
    if      (n == "output_frame")  output_frame_ = p.as_string();
    else if (n == "min_x")         min_x_ = p.as_double();
    else if (n == "max_x")         max_x_ = p.as_double();
    else if (n == "min_y")         min_y_ = p.as_double();
    else if (n == "max_y")         max_y_ = p.as_double();
    else if (n == "min_z")         min_z_ = p.as_double();
    else if (n == "max_z")         max_z_ = p.as_double();
    else if (n == "leaf_size")     leaf_size_ = p.as_double();
    // use_latest_tf, tf_timeout_sec, publish_frame, publish_marker, marker_alpha are read on demand
  }

  publishMarker();

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void CropBoxFilterNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // --- Start timing ---
  auto t_start = std::chrono::steady_clock::now();

  // Read dynamic params
  const bool   use_latest_tf  = this->get_parameter("use_latest_tf").as_bool();
  const double tf_timeout_sec = this->get_parameter("tf_timeout_sec").as_double();
  const std::string publish_frame = this->get_parameter("publish_frame").as_string();

  // 1) Transform input cloud to crop frame (output_frame_, e.g., base_link)
  sensor_msgs::msg::PointCloud2 cloud_in = *msg;
  sensor_msgs::msg::PointCloud2 cloud_tf = cloud_in;

  // Count input points for logging
  const std::size_t in_points =
    static_cast<std::size_t>(cloud_in.width) * static_cast<std::size_t>(cloud_in.height);

  if (!output_frame_.empty() && cloud_in.header.frame_id != output_frame_)
  {
    try {
      rclcpp::Time stamp = use_latest_tf
        ? rclcpp::Time(0, 0, get_clock()->get_clock_type())  // latest TF
        : rclcpp::Time(cloud_in.header.stamp);

      auto tf = tf_buffer_.lookupTransform(
        output_frame_, cloud_in.header.frame_id, stamp,
        rclcpp::Duration::from_seconds(tf_timeout_sec));

      tf2::doTransform(cloud_in, cloud_tf, tf);
      cloud_tf.header.stamp = cloud_in.header.stamp;
      cloud_tf.header.frame_id = output_frame_;
    }
    catch (const std::exception & e) {
      if (!use_latest_tf) {
        try {
          auto tf = tf_buffer_.lookupTransform(
            output_frame_, cloud_in.header.frame_id,
            rclcpp::Time(0, 0, get_clock()->get_clock_type()),
            rclcpp::Duration::from_seconds(tf_timeout_sec));
          tf2::doTransform(cloud_in, cloud_tf, tf);
          cloud_tf.header.stamp = cloud_in.header.stamp;
          cloud_tf.header.frame_id = output_frame_;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "TF '%s' <- '%s' failed at cloud stamp; using latest TF fallback: %s",
            output_frame_.c_str(), cloud_in.header.frame_id.c_str(), e.what());
        } catch (const std::exception & latest_e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Latest TF fallback '%s' <- '%s' also failed: %s",
            output_frame_.c_str(), cloud_in.header.frame_id.c_str(), latest_e.what());
          return;
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "TF '%s' <- '%s' failed: %s. Skipping cloud.",
          output_frame_.c_str(), cloud_in.header.frame_id.c_str(), e.what());
        return;
      }
    }
  }

  // 2) Convert to PCL and Crop (axis-aligned box in output_frame_)
  pcl::PCLPointCloud2::Ptr pcl_in (new pcl::PCLPointCloud2());
  pcl_conversions::toPCL(cloud_tf, *pcl_in);

  pcl::PCLPointCloud2::Ptr pcl_cropped (new pcl::PCLPointCloud2());
  {
    pcl::CropBox<pcl::PCLPointCloud2> crop;
    crop.setInputCloud(pcl_in);
    Eigen::Vector4f min(min_x_, min_y_, min_z_, 1.0f);
    Eigen::Vector4f max(max_x_, max_y_, max_z_, 1.0f);
    crop.setMin(min);
    crop.setMax(max);
    crop.setNegative(false);
    crop.filter(*pcl_cropped);
  }

  // 3) Optional VoxelGrid
  pcl::PCLPointCloud2::Ptr pcl_out = pcl_cropped;
  pcl::PCLPointCloud2::Ptr pcl_vox (new pcl::PCLPointCloud2());
  if (leaf_size_ > 0.0) {
    pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
    vox.setInputCloud(pcl_cropped);
    vox.setLeafSize(static_cast<float>(leaf_size_),
                    static_cast<float>(leaf_size_),
                    static_cast<float>(leaf_size_));
    vox.setDownsampleAllData(true);
    vox.filter(*pcl_vox);
    pcl_out = pcl_vox;
  }

  // 4) Back to ROS 2, transform to publish_frame (e.g., map), and publish
  sensor_msgs::msg::PointCloud2 out;
  pcl_conversions::fromPCL(*pcl_out, out);

  if (out.width == 0 || out.height == 0 || out.data.empty()) {
    // Still log timing, but with 0 output points
    auto t_end = std::chrono::steady_clock::now();
    double dt_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    std::size_t out_points = 0;

    RCLCPP_INFO(
      get_logger(),
      "PROC_TIME,node=crop_box,dt_ms=%.3f,in_pts=%zu,out_pts=%zu",
      dt_ms, in_points, out_points);

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                          "Crop yielded 0 points -> not publishing");
    return;
  }

  const std::size_t out_points =
    static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height);

  // Out is currently in output_frame_. Transform to publish_frame if needed.
  if (!publish_frame.empty() && publish_frame != output_frame_) {
    try {
      rclcpp::Time publish_stamp = use_latest_tf
        ? rclcpp::Time(0, 0, get_clock()->get_clock_type())
        : rclcpp::Time(cloud_in.header.stamp);

      auto tf_pub = tf_buffer_.lookupTransform(
        publish_frame, output_frame_, publish_stamp,
        rclcpp::Duration::from_seconds(tf_timeout_sec));
      sensor_msgs::msg::PointCloud2 out_tf;
      tf2::doTransform(out, out_tf, tf_pub);
      out = std::move(out_tf);
    } catch (const std::exception& e) {
      if (!use_latest_tf) {
        try {
          auto tf_pub = tf_buffer_.lookupTransform(
            publish_frame, output_frame_,
            rclcpp::Time(0, 0, get_clock()->get_clock_type()),
            rclcpp::Duration::from_seconds(tf_timeout_sec));
          sensor_msgs::msg::PointCloud2 out_tf;
          tf2::doTransform(out, out_tf, tf_pub);
          out = std::move(out_tf);
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Final publish transform %s <- %s failed at cloud stamp; using latest TF fallback: %s",
            publish_frame.c_str(), output_frame_.c_str(), e.what());
        } catch (const std::exception& latest_e) {
          auto t_end = std::chrono::steady_clock::now();
          double dt_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

          RCLCPP_INFO(
            get_logger(),
            "PROC_TIME,node=crop_box,dt_ms=%.3f,in_pts=%zu,out_pts=%zu",
            dt_ms, in_points, out_points);

          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Final publish latest TF fallback %s <- %s also failed: %s",
            publish_frame.c_str(), output_frame_.c_str(), latest_e.what());
          return;
        }
      } else {
        auto t_end = std::chrono::steady_clock::now();
        double dt_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

        RCLCPP_INFO(
          get_logger(),
          "PROC_TIME,node=crop_box,dt_ms=%.3f,in_pts=%zu,out_pts=%zu",
          dt_ms, in_points, out_points);

        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Final publish transform %s <- %s failed: %s",
          publish_frame.c_str(), output_frame_.c_str(), e.what());
        return;
      }
    }
  }

  out.header.stamp    = cloud_in.header.stamp;
  out.header.frame_id = publish_frame.empty() ? output_frame_ : publish_frame;
  pub_->publish(out);

  // Update the marker
  publishMarker();

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Published local cloud (w=%u h=%u) in frame %s",
    out.width, out.height, out.header.frame_id.c_str());

  // --- End timing + log ---
  auto t_end = std::chrono::steady_clock::now();
  double dt_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  RCLCPP_INFO(
    get_logger(),
    "PROC_TIME,node=crop_box,dt_ms=%.3f,in_pts=%zu,out_pts=%zu",
    dt_ms, in_points, out_points);
}


void CropBoxFilterNode::publishMarker()
{
  bool publish_marker = this->get_parameter("publish_marker").as_bool();
  if (!publish_marker || !marker_pub_) return;

  const double alpha = this->get_parameter("marker_alpha").as_double();

  visualization_msgs::msg::Marker m;
  m.header.frame_id = output_frame_;  // keep marker anchored to crop frame (e.g., base_link)
  m.header.stamp    = rclcpp::Time(0, 0, this->get_clock()->get_clock_type()); 

  m.ns   = "local_box";
  m.id   = 0;
  m.type = visualization_msgs::msg::Marker::CUBE;
  m.action = visualization_msgs::msg::Marker::ADD;

  // Center pose (axis-aligned in output_frame_)
  m.pose.position.x = 0.5 * (min_x_ + max_x_);
  m.pose.position.y = 0.5 * (min_y_ + max_y_);
  m.pose.position.z = 0.5 * (min_z_ + max_z_);
  m.pose.orientation.w = 1.0;

  // Dimensions
  m.scale.x = std::max(0.0, max_x_ - min_x_);
  m.scale.y = std::max(0.0, max_y_ - min_y_);
  m.scale.z = std::max(0.0, max_z_ - min_z_);

  // Color
  m.color.r = 0.1f; m.color.g = 0.8f; m.color.b = 0.3f; m.color.a = static_cast<float>(alpha);

  m.frame_locked = true;               // follow the frame with the latest TF
  m.lifetime = rclcpp::Duration(0,0);  // forever

  marker_pub_->publish(m);
}

} // namespace futuraps

// Register for composition
RCLCPP_COMPONENTS_REGISTER_NODE(futuraps::CropBoxFilterNode)
