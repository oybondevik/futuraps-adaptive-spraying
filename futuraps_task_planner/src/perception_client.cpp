#include "futuraps_task_planner/perception_client.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>

namespace futuraps_task_planner
{

PerceptionClient::PerceptionClient(rclcpp::Node* node)
: node_(node)
{
}

void PerceptionClient::configure(const PerceptionConfig& config)
{
  config_ = config;
  createClientsIfNeeded();
}

void PerceptionClient::createClientsIfNeeded()
{
  if (!closest_grid_client_) {
    closest_grid_client_ =
      node_->create_client<futuraps_perception::srv::GetClosestGrid>(config_.closest_srv);
  }

  if (!global_normal_client_) {
    global_normal_client_ =
      node_->create_client<futuraps_perception::srv::GetGlobalNormal>(config_.normal_srv);
  }

  if (!top_point_client_) {
    top_point_client_ =
      node_->create_client<futuraps_perception::srv::GetTopCanopyPoint>(
        config_.top_point_srv);
  }
}

bool PerceptionClient::sendRequest()
{
  createClientsIfNeeded();

  if (!closest_grid_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "ClosestGrid service not ready: %s", config_.closest_srv.c_str());
    return false;
  }

  if (!global_normal_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "GlobalNormal service not ready: %s", config_.normal_srv.c_str());
    return false;
  }

  if (!top_point_client_->service_is_ready()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "TopCanopyPoint service not ready: %s",
      config_.top_point_srv.c_str());
    return false;
  }

  if (closest_pending_ || normal_pending_ || top_point_pending_) {
    RCLCPP_WARN(node_->get_logger(), "Perception request already pending");
    return false;
  }

  auto closest_req =
    std::make_shared<futuraps_perception::srv::GetClosestGrid::Request>();
  closest_req->cell_size = config_.cell_size;
  closest_req->rows = config_.rows;
  closest_req->cols = config_.cols;
  closest_req->x0 = config_.x0;
  closest_req->z0 = config_.z0;
  closest_req->y_left_max = config_.y_left_max;
  closest_req->y_right_max = config_.y_right_max;
  closest_req->side = config_.side;
  closest_req->front_percentile = config_.front_percentile;
  closest_req->min_points_per_cell = config_.min_points_per_cell;

  closest_future_ = closest_grid_client_->async_send_request(closest_req).future.share();

  normal_futures_.clear();
  normal_futures_.reserve(static_cast<size_t>(std::max(0, config_.rows)));

  const double total_z_span =
    static_cast<double>(config_.rows) * config_.cell_size;

  const double normal_cell_size =
    total_z_span / static_cast<double>(config_.normal_rows);

  for (int i = 0; i < config_.normal_rows; ++i) {
    auto normal_req =
      std::make_shared<futuraps_perception::srv::GetGlobalNormal::Request>();

    const double row_min_z =
      config_.z0 + static_cast<double>(i) * normal_cell_size - config_.normal_z_overlap;

    const double row_max_z =
      config_.z0 + static_cast<double>(i + 1) * normal_cell_size + config_.normal_z_overlap;

    normal_req->frame_id = config_.target_frame;
    normal_req->min_x = config_.normal_min_x;
    normal_req->max_x = config_.normal_max_x;
    normal_req->min_y = config_.normal_min_y;
    normal_req->max_y = config_.normal_max_y;
    normal_req->min_z = row_min_z;
    normal_req->max_z = row_max_z;

    normal_futures_.push_back(
      global_normal_client_->async_send_request(normal_req).future.share());
  }

  auto top_req =
    std::make_shared<futuraps_perception::srv::GetTopCanopyPoint::Request>();

  top_req->frame_id = config_.target_frame;
  top_req->min_x = config_.top_min_x;
  top_req->max_x = config_.top_max_x;
  top_req->min_y = config_.top_min_y;
  top_req->max_y = config_.top_max_y;
  top_req->min_z = config_.top_min_z;
  top_req->max_z = config_.top_max_z;
  top_req->top_band_height = config_.top_band_height;
  top_req->front_percentile = config_.top_front_percentile;
  top_req->min_points = static_cast<uint32_t>(std::max(1, config_.top_min_points));

  top_point_future_ =
    top_point_client_->async_send_request(top_req).future.share();

  closest_pending_ = true;
  normal_pending_ = true;
  top_point_pending_ = true;
  request_start_time_ = node_->now();
  clearResult();

  RCLCPP_INFO(node_->get_logger(), "Sent perception requests (%d row normals)", config_.rows);
  return true;
}

bool PerceptionClient::checkReady()
{
  if (!closest_pending_ && !normal_pending_ && !top_point_pending_) {
    return latest_result_.valid;
  }

  if ((node_->now() - request_start_time_).seconds() > config_.timeout_s) {
    RCLCPP_WARN(node_->get_logger(), "Perception request timed out");
    closest_pending_ = false;
    normal_pending_ = false;
    top_point_pending_ = false;
    clearResult();
    return false;
  }

  bool closest_ready =
    closest_pending_ &&
    closest_future_.valid() &&
    closest_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;

  bool normal_ready = normal_pending_;
  for (const auto & fut : normal_futures_) {
    if (!fut.valid() ||
        fut.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      normal_ready = false;
      break;
    }
  }

  bool top_point_ready =
    top_point_pending_ &&
    top_point_future_.valid() &&
    top_point_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;

  if (!closest_ready || !normal_ready || !top_point_ready) {
    return false;
  }

  auto closest_res = closest_future_.get();
  auto top_point_res = top_point_future_.get();  

  closest_pending_ = false;
  normal_pending_ = false;
  top_point_pending_ = false;

  if (!closest_res) {
    RCLCPP_WARN(node_->get_logger(), "ClosestGrid response pointer was null");
    clearResult();
    return false;
  }

  if (closest_res->x.empty() || closest_res->y.empty() || closest_res->z.empty()) {
    RCLCPP_WARN(node_->get_logger(), "ClosestGrid returned empty arrays");
    clearResult();
    return false;
  }

  if (closest_res->x.size() != closest_res->y.size() ||
      closest_res->x.size() != closest_res->z.size() ||
      closest_res->x.size() != closest_res->found.size()) {
    RCLCPP_WARN(node_->get_logger(),
                "ClosestGrid size mismatch: x=%zu y=%zu z=%zu found=%zu",
                closest_res->x.size(), closest_res->y.size(),
                closest_res->z.size(), closest_res->found.size());
    clearResult();
    return false;
  }

  latest_result_.x = closest_res->x;
  latest_result_.y = closest_res->y;
  latest_result_.z = closest_res->z;
  latest_result_.found = closest_res->found;
    latest_result_.normals.clear();
  latest_result_.normal_valid.clear();
  latest_result_.normals.reserve(normal_futures_.size());
  latest_result_.normal_valid.reserve(normal_futures_.size());

  int valid_normals = 0;

  for (size_t i = 0; i < normal_futures_.size(); ++i) {
    auto normal_res = normal_futures_[i].get();

    if (!normal_res) {
      RCLCPP_WARN(node_->get_logger(), "Normal response %zu pointer was null", i);
      latest_result_.normals.emplace_back(0.0, -1.0, 0.0);
      latest_result_.normal_valid.push_back(false);
      continue;
    }

    if (!std::isfinite(normal_res->nx) ||
        !std::isfinite(normal_res->ny) ||
        !std::isfinite(normal_res->nz))
    {
      RCLCPP_WARN(node_->get_logger(), "Normal %zu contains non-finite values", i);
      latest_result_.normals.emplace_back(0.0, -1.0, 0.0);
      latest_result_.normal_valid.push_back(false);
      continue;
    }

    tf2::Vector3 n(normal_res->nx, normal_res->ny, normal_res->nz);
    if (n.length2() < 1e-10) {
      RCLCPP_WARN(node_->get_logger(), "Normal %zu too small", i);
      latest_result_.normals.emplace_back(0.0, -1.0, 0.0);
      latest_result_.normal_valid.push_back(false);
      continue;
    }

    n.normalize();
    latest_result_.normals.push_back(n);
    latest_result_.normal_valid.push_back(true);
    ++valid_normals;
  }

  if (valid_normals == 0) {
    RCLCPP_WARN(node_->get_logger(), "All row normals were invalid");
    clearResult();
    return false;
  }

  if (top_point_res &&
      top_point_res->found &&
      top_point_res->count > 0 &&
      std::isfinite(top_point_res->x) &&
      std::isfinite(top_point_res->y) &&
      std::isfinite(top_point_res->z))
  {
    latest_result_.top_point_found = true;
    latest_result_.top_x = top_point_res->x;
    latest_result_.top_y = top_point_res->y;
    latest_result_.top_z = top_point_res->z;
    latest_result_.top_detected_max_z = top_point_res->detected_max_z;
    latest_result_.top_count = top_point_res->count;

    RCLCPP_INFO(
      node_->get_logger(),
      "Top canopy point: x=%.3f y=%.3f z=%.3f count=%u",
      latest_result_.top_x,
      latest_result_.top_y,
      latest_result_.top_z,
      latest_result_.top_count);
  } else {
    latest_result_.top_point_found = false;
    latest_result_.top_count = 0;

    RCLCPP_WARN(
      node_->get_logger(),
      "TopCanopyPoint returned no valid top point");
  }

  latest_result_.valid = true;

  RCLCPP_INFO(node_->get_logger(),
              "Perception ready. Stored %zu closest points and %d/%zu valid row normals.",
              latest_result_.x.size(),
              valid_normals,
              latest_result_.normals.size());

  return true;
}

PerceptionResult PerceptionClient::getResult() const
{
  return latest_result_;
}

void PerceptionClient::clearResult()
{
  latest_result_.x.clear();
  latest_result_.y.clear();
  latest_result_.z.clear();
  latest_result_.found.clear();
  latest_result_.normals.clear();
  latest_result_.normal_valid.clear();
  latest_result_.valid = false;
  latest_result_.top_point_found = false;
  latest_result_.top_x = 0.0;
  latest_result_.top_y = 0.0;
  latest_result_.top_z = 0.0;
  latest_result_.top_detected_max_z = 0.0;
  latest_result_.top_count = 0;
}

}  // namespace futuraps_task_planner