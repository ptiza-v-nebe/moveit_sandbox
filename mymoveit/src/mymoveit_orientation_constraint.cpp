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
  visual_tools.publishText(text_pose, "Path Constraint", rvt::WHITE, rvt::XLARGE);
  
  move_group.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  visual_tools.prompt("move to pose that satisfy constraints");
  


  geometry_msgs::PoseStamped constrainted_pose = move_group.getCurrentPose("panda_link7");
  tf::Quaternion q_rot = tf::createQuaternionFromRPY(3.14159265359, 0, 0);
  tf::Vector3 vec3(1,0,0);
  q_rot.setRotation(vec3,0.0);
  quaternionTFToMsg(q_rot, constrainted_pose.pose.orientation);
  //constrainted_pose.pose.orientation.w = 1.0;
  constrainted_pose.pose.position.x = -0.1;
  constrainted_pose.pose.position.y = 0.4;
  constrainted_pose.pose.position.z = 0.6;
  move_group.setPoseTarget(constrainted_pose);
  moveit::planning_interface::MoveGroupInterface::Plan constrainted_plan;
  bool constrainted_success = (move_group.plan(constrainted_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();

  visual_tools.prompt("press next to plan and move to constrainted pose");
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 1.0;
  ocm.absolute_z_axis_tolerance = 1.0;
  ocm.weight = 1.0;
  moveit_msgs::Constraints constraints;
  constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(constraints);

  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 1;
  target_pose.position.x = 0.4;
  target_pose.position.y = -0.1;
  target_pose.position.z = 0.9;
  move_group.setPoseTarget(target_pose);
  move_group.setPlanningTime(20.0);
  moveit::planning_interface::MoveGroupInterface::Plan target_plan;
  bool target_success = (move_group.plan(target_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.move();
  move_group.clearPathConstraints();
}

  ros::shutdown();
  return 0;
}
