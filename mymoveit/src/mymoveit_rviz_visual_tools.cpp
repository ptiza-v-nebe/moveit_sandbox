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

  // Clear previos messages
  visual_tools_->deleteAllMarkers();
  visual_tools_->enableBatchPublishing();

  //publish text
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  visual_tools_->publishText(text_pose, "Test text", rvt::WHITE, rvt::XXLARGE, false);
  visual_tools_->trigger();
  
  //publish sphere
  Eigen::Affine3d sphere_pose = Eigen::Affine3d::Identity();
  sphere_pose.translation().x() = 0.4;
  visual_tools_->publishSphere(sphere_pose, rviz_visual_tools::GREEN, rviz_visual_tools::XXXLARGE);
  visual_tools_->trigger();

  //publish bunch of spheres
  EigenSTL::vector_Vector3d sphere_points;
  sphere_pose = Eigen::Affine3d::Identity();
  sphere_pose.translation().y() += 0.2;
  sphere_points.emplace_back(sphere_pose.translation());
  sphere_pose.translation().y() += 0.2;
  sphere_points.emplace_back(sphere_pose.translation());
  visual_tools_->publishSpheres(sphere_points, rviz_visual_tools::BLUE, rviz_visual_tools::XXXLARGE);
  visual_tools_->trigger();

  //publish Z arrow
  Eigen::Affine3d arrowZ_pose = Eigen::Affine3d::Identity();
  arrowZ_pose.translation().y() += 0.6;
  visual_tools_->publishZArrow(arrowZ_pose, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
  visual_tools_->trigger();
  
  //publish arbitrary arrow
  Eigen::Affine3d arrow_pose = Eigen::Affine3d::Identity();
  arrow_pose.translation().x() += 0.6;
  arrow_pose.translation().y() += 0.6;
  visual_tools_->publishArrow(arrow_pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE);
  visual_tools_->trigger();

  //publish single line
  Eigen::Affine3d line_pose1 = Eigen::Affine3d::Identity();
  Eigen::Affine3d line_pose2 = Eigen::Affine3d::Identity();
  line_pose1.translation().x() += 0.2;
  line_pose1.translation().y() += 0.2;
  line_pose2 = line_pose1;
  line_pose2.translation().x() += 0.2;
  line_pose2.translation().y() += 0.2;
  visual_tools_->publishLine(line_pose1, line_pose2, rviz_visual_tools::PURPLE, rviz_visual_tools::LARGE);
  visual_tools_->trigger();

  //publish multiple lines at once
  EigenSTL::vector_Vector3d line_points1;
  EigenSTL::vector_Vector3d line_points2;
  Eigen::Affine3d line_pose = Eigen::Affine3d::Identity();

  line_pose.translation().x() += 0.8;
  line_pose.translation().y() += 0.8;
  line_points1.emplace_back(line_pose.translation());

  line_pose.translation().x() += 0.0;
  line_pose.translation().y() += 0.8;
  line_points1.emplace_back(line_pose.translation());

  line_pose.translation().x() += 0.8;
  line_pose.translation().y() += 0.8;
  line_points2.emplace_back(line_pose.translation());

  line_pose.translation().x() += 0.8;
  line_pose.translation().y() += 0.0;
  line_points2.emplace_back(line_pose.translation());

  std::vector<rviz_visual_tools::colors> lines_colors;
  lines_colors.push_back(rviz_visual_tools::RED);
  lines_colors.push_back(rviz_visual_tools::WHITE);

  visual_tools_->publishLines(line_points1, line_points2, lines_colors, rviz_visual_tools::XXLARGE);
  visual_tools_->trigger();

  //publish axis
  Eigen::Affine3d axis_pose = Eigen::Affine3d::Identity();
  axis_pose.translation().x() += 0.0;
  axis_pose.translation().y() += 1.0;
  visual_tools_->publishAxisLabeled(axis_pose, "Axis", rviz_visual_tools::LARGE);
  visual_tools_->trigger();

  Eigen::Affine3d cylinder_pose1 = Eigen::Affine3d::Identity();
  Eigen::Affine3d cylinder_pose2 = Eigen::Affine3d::Identity();
  
  cylinder_pose1.translation().x() = 0.6;
  cylinder_pose1.translation().y() = 0.9;
  cylinder_pose2 = cylinder_pose1;
  cylinder_pose2.translation().z() = 0.2;

  visual_tools_->publishCylinder(cylinder_pose1.translation(), cylinder_pose2.translation(), rviz_visual_tools::ORANGE, rviz_visual_tools::XXXLARGE);
  visual_tools_->trigger();

  //publish rectangular cuboid
  Eigen::Affine3d cuboid_pose1 = Eigen::Affine3d::Identity();
  Eigen::Affine3d cuboid_pose2 = Eigen::Affine3d::Identity();

  cuboid_pose1.translation().x() += 1.2;
  cuboid_pose1.translation().y() += 0.0;
  cuboid_pose2 = cuboid_pose1;
  cuboid_pose2.translation().x() += 0.3;
  cuboid_pose2.translation().y() += 0.4;
  cuboid_pose2.translation().z() += 0.2;
  visual_tools_->publishCuboid(cuboid_pose1.translation(), cuboid_pose2.translation(), rvt::RAND);
  visual_tools_->trigger(); 

  //publish wireframe cuboid
  Eigen::Affine3d wcuboid_pose = Eigen::Affine3d::Identity();
  wcuboid_pose.translation().x() += 1.9;
  wcuboid_pose.translation().y() += 0.0;
  wcuboid_pose = wcuboid_pose * Eigen::AngleAxisd(M_PI/4.0, Eigen::Vector3d::UnitZ());
  wcuboid_pose = wcuboid_pose * Eigen::AngleAxisd(M_PI/4.0, Eigen::Vector3d::UnitX());
  Eigen::Vector3d min_point, max_point;
  min_point << -0.05, -0.05, -0.05;
  max_point << 0.05, 0.05, 0.05;
  visual_tools_->publishWireframeCuboid(wcuboid_pose, min_point, max_point, rvt::RAND);
  visual_tools_->trigger(); 

  //publish sized cuboid
  Eigen::Affine3d wcuboid_sized_pose = Eigen::Affine3d::Identity();
  wcuboid_sized_pose.translation().x() = 0.9;
  wcuboid_sized_pose.rotate(Eigen::AngleAxisd(M_PI/3.0, Eigen::Vector3d(0,0,1)));
  double depth = 0.05, width = 0.1, height = 0.05;
  visual_tools_->publishWireframeCuboid(wcuboid_sized_pose, depth, width, height, rvt::RAND);

  //publish plane
  Eigen::Affine3d plane_pose = Eigen::Affine3d::Identity();
  plane_pose.translation().x() = 1.7;
  plane_pose.translation().y() = 0.5;
  double max_plane_size = 0.05;
  double min_plane_size = 0.075;
  visual_tools_->publishXYPlane(plane_pose, rvt::RED, max_plane_size + min_plane_size);
  visual_tools_->publishXZPlane(plane_pose, rvt::GREEN, max_plane_size + min_plane_size);
  visual_tools_->publishYZPlane(plane_pose, rvt::BLUE, max_plane_size + min_plane_size);
  visual_tools_->trigger();

  //publish graph
  Eigen::Affine3d graph_pose = Eigen::Affine3d::Identity();  
  graph_msgs::GeometryGraph graph;
  graph_msgs::Edges edge;

  //it1
  graph_pose.translation().y() = 1.6;
  graph.nodes.push_back(visual_tools_->convertPose(graph_pose).position);
  graph.edges.push_back(edge);
  edge.node_ids.clear();

  //it2
  graph_pose.translation().x() += 0.1;
  graph_pose.translation().z() += 0.1;
  graph.nodes.push_back(visual_tools_->convertPose(graph_pose).position);
  edge.node_ids.push_back(0);
  graph.edges.push_back(edge);
  edge.node_ids.clear();

  //it3
  graph_pose.translation().x() += 0.3;
  graph_pose.translation().z() += 0.1;
  graph.nodes.push_back(visual_tools_->convertPose(graph_pose).position);
  edge.node_ids.push_back(1);
  graph.edges.push_back(edge);
  edge.node_ids.clear();

  //it4
  graph_pose.translation().x() += 0.1;
  graph_pose.translation().y() += 0.1;
  graph_pose.translation().z() += 0.2;
  graph.nodes.push_back(visual_tools_->convertPose(graph_pose).position);
  edge.node_ids.push_back(1);
  graph.edges.push_back(edge);
  edge.node_ids.clear();

  //it5
  graph_pose.translation().x() += 0.2;
  graph_pose.translation().y() += 0.1;
  graph.nodes.push_back(visual_tools_->convertPose(graph_pose).position);
  edge.node_ids.push_back(0);
  edge.node_ids.push_back(1);
  edge.node_ids.push_back(2);
  graph.edges.push_back(edge);
  edge.node_ids.clear();
  
  visual_tools_->publishGraph(graph, rvt::ORANGE, 0.005);
  visual_tools_->trigger();

  //publish path
  Eigen::Affine3d path_pose = Eigen::Affine3d::Identity();
  EigenSTL::vector_Vector3d path;
  std::vector<rviz_visual_tools::colors> colors;

  colors.push_back(rviz_visual_tools::RED);
  path.emplace_back(path_pose.translation());

  path_pose.translation().y() += -0.3;
  path_pose.translation().x() += -0.5;
  path_pose.translation().z() += 0.5;
  colors.push_back(rviz_visual_tools::RED);
  path.emplace_back(path_pose.translation());

  path_pose.translation().y() += -0.1;
  path_pose.translation().x() += -0.4;
  path_pose.translation().z() += -0.1;
  colors.push_back(rviz_visual_tools::GREEN);
  path.emplace_back(path_pose.translation());

  path_pose.translation().y() += -0.5;
  path_pose.translation().x() += -0.1;
  path_pose.translation().z() += 0.15;
  colors.push_back(rviz_visual_tools::BLUE);
  path.emplace_back(path_pose.translation());

  visual_tools_->publishPath(path, colors);
  visual_tools_->trigger();
  
  ROS_INFO_STREAM("Shutting down.");
  ros::shutdown();

  return 0;
}
