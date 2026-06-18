#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "futuraps_task_planner/types.hpp"

namespace futuraps_task_planner
{

class PlatformMoving
{
public:
  explicit PlatformMoving(rclcpp::Node * node);

  void configure(const PlatformMovingConfig& config);

  bool startMovePlatformDistance(double dist_in_m);
  bool startMovingForward();
  bool stopMovingForward();

  bool motionFinished() const;

private:
  enum class PidRequestPurpose
  {
    NONE,
    START_MOVE,
    STOP_MOVE,
  };

  PlatformMovingConfig config_;

  bool requestPidEnabled(bool enabled, PidRequestPurpose purpose);
  void pollPidEnableRequest();
  void handlePidEnableSuccess();
  void handlePidEnableFailure(const std::string& reason);
  void startBrakeHold();
  void publishZeroCommand();
  void publishBrakeHoldIfActive();
  bool setPlatformStartPosition();
  bool robotHasMovedDist(double target_dist);
  void motionTimerCallback();

  rclcpp::Node * node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_pid_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture pid_enable_future_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_unstamped_pub_;
  rclcpp::TimerBase::SharedPtr motion_timer_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  geometry_msgs::msg::Vector3 start_position_;
  bool is_robot_moving_{false};
  bool move_start_pending_{false};
  double target_distance_{0.0};
  bool motion_finished_{false};
  bool pid_request_pending_{false};
  bool pid_request_value_{false};
  PidRequestPurpose pid_request_purpose_{PidRequestPurpose::NONE};
  rclcpp::Time pid_request_sent_time_{0, 0, RCL_ROS_TIME};
  double pid_request_timeout_s_{1.0};
  rclcpp::Time brake_hold_until_{0, 0, RCL_ROS_TIME};
  double brake_hold_duration_s_{1.0};
  bool brake_hold_active_{false};
};

}  // namespace futuraps_task_planner
