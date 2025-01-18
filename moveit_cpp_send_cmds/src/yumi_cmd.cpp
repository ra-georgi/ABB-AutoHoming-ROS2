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

  auto const logger = rclcpp::get_logger("yumi_cmd");
  auto trajectory_publisher = node->create_publisher<moveit_msgs::msg::RobotTrajectory>("robot_trajectory", 10);

  // Interface and target setup
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node,"yumi_armL");

  auto const joint_targets = []{
    std::vector<double> joint_values = {0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    return joint_values;
  }();
  move_group_interface.setJointValueTarget(joint_targets);

  // Create Collision Object

  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()]{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "cyl";

    shapes::Mesh* m = shapes::createMeshFromResource("package://moveit_cpp_send_cmds/meshes/cylinder_update.stl");

    // Scale the mesh to convert mm to meters
    double scale_factor = 0.001;
    for (unsigned int i = 0; i < m->vertex_count; ++i) {
        m->vertices[3 * i] *= scale_factor;     // Scale x
        m->vertices[3 * i + 1] *= scale_factor; // Scale y
        m->vertices[3 * i + 2] *= scale_factor; // Scale z
    }    
    
    shape_msgs::msg::Mesh cyl_mesh;
    shapes::ShapeMsg cyl_mesh_msg;
    shapes::constructMsgFromShape(m,cyl_mesh_msg);
    cyl_mesh = boost::get<shape_msgs::msg::Mesh>(cyl_mesh_msg);
    
    geometry_msgs::msg::Pose cyl_pose;
    cyl_pose.orientation.w = 1;
    cyl_pose.orientation.x = 0;
    cyl_pose.orientation.y = 0;
    cyl_pose.orientation.z = 0;
    cyl_pose.position.x = 0.0;
    cyl_pose.position.y = 0.0;
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
    // move_group_interface.execute(plan);

    RCLCPP_INFO(logger, "Planning succeeded!");

    // Publish the trajectory
    trajectory_publisher->publish(plan.trajectory);

    // // Extract joint trajectory
    // const auto &trajectory = plan.trajectory.joint_trajectory;
    // const auto &joint_names = trajectory.joint_names;

    // // Iterate through trajectory points to get joint angles
    // for (size_t i = 0; i < trajectory.points.size(); ++i)
    // {
    //   const auto &point = trajectory.points[i];
    //   RCLCPP_INFO(logger, "Waypoint %ld:", i);

    //   for (size_t j = 0; j < joint_names.size(); ++j)
    //   {
    //     RCLCPP_INFO(logger, "  %s: %f", joint_names[j].c_str(), point.positions[j]);
    //   }
    // }  
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning Failed!");
  }

  rclcpp::shutdown();
  return 0;
}
