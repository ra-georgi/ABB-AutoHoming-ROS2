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
    "orion_cmd",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("orion_cmd");

  // Interface and target setup
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node,"orion_planning_group");

  auto const joint_targets = []{
    std::vector<double> joint_values = {0.0, 0.0};
    return joint_values;
  }();
  move_group_interface.setJointValueTarget(joint_targets);

  // Create Collision Object

  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()]{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "cyl";

    shapes::Mesh* m = shapes::createMeshFromResource("package://moveit_cpp_send_cmds/meshes/cylinder.stl");
    shape_msgs::msg::Mesh cyl_mesh;
    shapes::ShapeMsg cyl_mesh_msg;
    shapes::constructMsgFromShape(m,cyl_mesh_msg);
    cyl_mesh = boost::get<shape_msgs::msg::Mesh>(cyl_mesh_msg);
    
    geometry_msgs::msg::Pose cyl_pose;
    cyl_pose.orientation.w = 1;
    cyl_pose.orientation.x = 0;
    cyl_pose.orientation.y = 0;
    cyl_pose.orientation.z = 0;
    cyl_pose.position.x = 0.4;
    cyl_pose.position.y = -12;
    cyl_pose.position.z = 0.0;

    collision_object.meshes.push_back(cyl_mesh);
    collision_object.mesh_poses.push_back(cyl_pose);
    collision_object.operation = collision_object.ADD;
    
    return collision_object;
  }();

  moveit::planning_interface::PlanningSceneInterface  planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Create Plan
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok,msg);
  }();

  if (success) 
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning Failed!");
  }

  rclcpp::shutdown();
  return 0;
}
