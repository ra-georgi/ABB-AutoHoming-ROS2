#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>

int main(int argc, char ** argv)
{
  // Creating node and logger
  rclcpp::init(argc,argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "yumi_cmd",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("yumi_cmd_right");
  auto trajectory_publisher = node->create_publisher<moveit_msgs::msg::RobotTrajectory>("robot_trajectory", 10);

  // Interface and target setup
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node,"yumi_armR");

  auto const joint_targets = []{
    std::vector<double> joint_values = {-0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    return joint_values;
  }();
  move_group_interface.setJointValueTarget(joint_targets);

  // Create Plan
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok,msg);
  }();

  if (success) 
  {
    // move_group_interface.execute(plan);
    // Publish the trajectory
    trajectory_publisher->publish(plan.trajectory);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning Failed!");
  }

  rclcpp::shutdown();
  return 0;
}
