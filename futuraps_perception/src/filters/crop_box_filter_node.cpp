#include "rclcpp/rclcpp.hpp"
#include "futuraps_perception/crop_box_filter.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<futuraps::CropBoxFilterNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
