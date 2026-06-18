#include "rclcpp/rclcpp.hpp"
#include "futuraps_perception/local_map_filter.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<futuraps::LocalMapFilterNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
