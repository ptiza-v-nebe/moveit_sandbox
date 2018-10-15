#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv){
ros::init(argc, argv, "move_visual_tools_demo");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();
  
  visual_tools.prompt("Write some text in space");
  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 0.9;
  visual_tools.publishText(text_pose, "Some text somewhere in space", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();
  visual_tools.prompt("Mark some pose");
  
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;

  visual_tools.publishAxisLabeled(target_pose1, "Some pose");
  visual_tools.trigger();
  visual_tools.prompt("End ros");

  
  ros::shutdown();
  return 0;
}