#include "pcd_map_server/pcd_map_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pcd_map_server::PcdMapServer>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
