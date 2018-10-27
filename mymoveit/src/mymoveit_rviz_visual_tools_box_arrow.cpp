// ROS
#include <ros/ros.h>

// For visualizing things in rviz
//#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

// C++
#include <string>
#include <vector>

namespace rvt = rviz_visual_tools;


int main(int argc, char** argv)
{
  //init ros
  ros::init(argc, argv, "visual_tools_demo");
  ROS_INFO_STREAM("Visual Tools Demo");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //init visual tools
  rvt::RvizVisualToolsPtr visual_tools_;
  visual_tools_.reset(new rvt::RvizVisualTools("world", "/rviz_visual_tools"));
  visual_tools_->loadMarkerPub();  // create publisher before waiting

  ROS_INFO("Drawing shapes and colors");
  //ros::Duration(5.0).sleep();

  // Clear previous messages
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();

  //publish text
  //Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  //visual_tools_->publishText(text_pose, "Test text", rvt::WHITE, rvt::XXLARGE, false);
  //visual_tools_->trigger();

  //publish wireframe cuboid
  Eigen::Affine3d wcuboid_pose = Eigen::Affine3d::Identity();
  wcuboid_pose.translation().x() += 0.0;
  wcuboid_pose.translation().y() += 0.0;
  wcuboid_pose = wcuboid_pose * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  visual_tools_->publishWireframeCuboid(wcuboid_pose, 0.3, 0.3, 0.1);
  visual_tools_->trigger(); 

  //1. publish arbitrary arrow (not working properly, you have to estimate the final pose)
       //another point: axis angle and rotation() for some reason do not work properly in moveit and rviz
  Eigen::Affine3d arrow_pose = Eigen::Affine3d::Identity();
  arrow_pose.translation().x() += 0.15;
  arrow_pose.translation().y() += 0.15;
  arrow_pose.translation().z() -= 0.05;
  arrow_pose.rotate(Eigen::AngleAxisd(M_PI/4.0, Eigen::Vector3d(0,1,1)));
  visual_tools_->publishArrow(arrow_pose, rviz_visual_tools::RED, rviz_visual_tools::MEDIUM);
  visual_tools_->trigger();

  //2. publish arbitrary arrow (best controllable result for rotating is using two vectors and get rotation quat from them)
  arrow_pose = Eigen::Affine3d::Identity();
  arrow_pose.translation().x() += 0.15;
  arrow_pose.translation().y() += 0.15;
  arrow_pose.translation().z() += 0.05;
  //Eigen::Quaterniond rot;
  //rot.setFromTwoVectors(Eigen::Vector3d(1,0,0),Eigen::Vector3d(1,1,1));
  Eigen::Quaterniond rot = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(1,0,0),Eigen::Vector3d(1,1,1));
  arrow_pose.rotate(rot);
  visual_tools_->publishArrow(arrow_pose, rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
  visual_tools_->trigger();

  //3. publish arbitrary arrow (conversion from axis angle two quat and then using it as rotation does the job well)
       //there is still problem that you have to estimate your final pose
  arrow_pose = Eigen::Affine3d::Identity();
  arrow_pose.translation().x() -= 0.15;
  arrow_pose.translation().y() += 0.15;
  arrow_pose.translation().z() -= 0.05;
  Eigen::Quaterniond quat = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/10 * 6,Eigen::Vector3d(0,1,1))); //rotate around zy
  arrow_pose.rotate(quat);
  visual_tools_->publishArrow(arrow_pose, rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
  visual_tools_->trigger();

  //4. publish arbitrary arrow (Compact 1liner for rotation)
  arrow_pose = Eigen::Affine3d::Identity();
  arrow_pose.translation().x() -= 0.15;
  arrow_pose.translation().y() += 0.15;
  arrow_pose.translation().z() += 0.05;
  arrow_pose.rotate(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(1,0,0),Eigen::Vector3d(-1,1,1)));
  visual_tools_->publishArrow(arrow_pose, rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
  visual_tools_->trigger();

  /*
  //for some reason this one gives some weird angle spaces, they are not equal
  Eigen::Quaterniond quat1(Eigen::AngleAxisd(0.0,Eigen::Vector3d(0,1,1)));
  Eigen::Quaterniond quat2(Eigen::AngleAxisd(M_PI,Eigen::Vector3d(0,1,1)));
  int numOfSteps = 10;
  for(int i=0; i < numOfSteps; i++){
    arrow_pose = Eigen::Affine3d::Identity();
    arrow_pose.translation().x() -= 0.15;
    arrow_pose.translation().y() += 0.15;
    arrow_pose.translation().z() -= 0.05;
    arrow_pose.rotate(quat1.slerp((i+1)*0.1,quat2));
    visual_tools_->publishArrow(arrow_pose, rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
  }*/
  
  /*
  //properly rotating, all arrows has defined angle space without any weird things
  int numOfSteps = 10;
  Eigen::Quaterniond quat(Eigen::AngleAxisd(M_PI/numOfSteps,Eigen::Vector3d(0,1,1)));
  for(int i=0; i < 15; i++){
    visual_tools_->publishArrow(arrow_pose, rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
    arrow_pose.rotate(quat);
  }*/

  /*
  //Does weird things, the angle spaces are not equal after some point
  Eigen::Transform <float , 3 , Eigen::Affine > t; //this one is behind Affine3d class
  Eigen::AngleAxisd angleaxis(M_PI/10.0, Eigen::Vector3d(0,1,1));
  for(int i=0; i < 15; i++){
    arrow_pose.rotate(angleaxis);
    visual_tools_->publishArrow(arrow_pose, rviz_visual_tools::GREEN, rviz_visual_tools::MEDIUM);
  }*/
  
  ROS_INFO_STREAM("Shutting down.");
  ros::shutdown();

  return 0;
}
