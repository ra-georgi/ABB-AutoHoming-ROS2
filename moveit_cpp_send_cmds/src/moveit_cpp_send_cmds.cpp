#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "orion_cmd",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("orion_cmd");

  rclcpp::shutdown();
  return 0;
}
