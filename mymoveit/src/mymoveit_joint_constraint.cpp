#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>

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
  visual_tools.publishText(text_pose, "Joint Constraint", rvt::WHITE, rvt::XLARGE);
  
  move_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  bool home_success = (move_group.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  visual_tools.prompt("move to pose that satisfy constraints");
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[2] = 0.57;  // radians
  move_group.setJointValueTarget(joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
  bool joint_success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  visual_tools.prompt("press next to plan and move to constrainted joint");
  moveit_msgs::JointConstraint jc;
  jc.joint_name = "panda_joint3";
  jc.position = 0.57;
  jc.tolerance_above = 0.1;
  jc.tolerance_below = 0.1;
  jc.weight = 1;
  moveit_msgs::Constraints constraints;
  constraints.joint_constraints.push_back(jc);
  move_group.setPathConstraints(constraints);

  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 1;
  target_pose.position.x = 0.4;
  target_pose.position.y = -0.1;
  target_pose.position.z = 0.9;
  move_group.setPoseTarget(target_pose);
  move_group.setPlanningTime(10.0);
  moveit::planning_interface::MoveGroupInterface::Plan target_plan;
  bool target_success = (move_group.plan(target_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  ros::shutdown();
  return 0;
}
