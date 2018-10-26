cmake_minimum_required(VERSION 2.8.3)
project(mymoveit)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  geometric_shapes
  interactive_markers
  moveit_commander
  moveit_core
  moveit_fake_controller_manager
  moveit_ros_perception
  moveit_ros_planning_interface
  moveit_visual_tools
  pcl_conversions
  pcl_ros
  pluginlib
  rosbag
  roscpp
  rospy
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mymoveit_joint_constraint src/mymoveit_joint_constraint.cpp)
target_link_libraries(mymoveit_joint_constraint ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(mymoveit_path_constraint src/mymoveit_path_constraint.cpp)
target_link_libraries(mymoveit_path_constraint ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(mymoveit_orientation_constraint src/mymoveit_orientation_constraint.cpp)
target_link_libraries(mymoveit_orientation_constraint ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(mymoveit_no_constraint src/mymoveit_no_constraint.cpp)
target_link_libraries(mymoveit_no_constraint ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(mymoveit_visual_tools src/mymoveit_visual_tools.cpp)
target_link_libraries(mymoveit_visual_tools ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(mymoveit_rviz_visual_tools src/mymoveit_rviz_visual_tools.cpp)
target_link_libraries(mymoveit_rviz_visual_tools ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(mymoveit_rviz_visual_tools_box_arrow src/mymoveit_rviz_visual_tools_box_arrow.cpp)
target_link_libraries(mymoveit_rviz_visual_tools_box_arrow ${catkin_LIBRARIES} ${Boost_LIBRARIES})


#install(TARGETS mymoveit1 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS mymoveit2 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS mymoveit3 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
