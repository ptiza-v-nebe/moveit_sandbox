#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Path Constraint", rvt::WHITE, rvt::XLARGE);
  
  
  visual_tools.prompt("press next to plan to home pose");
  move_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.prompt("press next to move to simple pose");
  move_group.move();

  
  visual_tools.prompt("press next to plan to constrainted pose");
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  //q_rot = tf::createQuaternionFromRPY(r, p, y);
  //tf2::Quaternion myQuaternion;
  //myQuaternion.setRPY( 0, 0, 0 );
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.4;
  target_pose1.position.y = -0.4;
  target_pose1.position.z = 0.9;
  move_group.setPoseTarget(target_pose1);
  move_group.setPlanningTime(10.0);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  bool success2 = (move_group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools.prompt("press next to move to constrainted pose");
  move_group.move();

  visual_tools.prompt("press next to end");

  ros::shutdown();
  return 0;
}
