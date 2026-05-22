cmake_minimum_required(VERSION 3.0.2)
project(auto_nav_goal_5_pkg)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  ar_pose
  move_base_msgs
  relative_move
  roscpp
  tf
)

catkin_package(
  CATKIN_DEPENDS actionlib ar_pose move_base_msgs relative_move roscpp tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(auto_nav_5_node src/auto_nav_goal_5.cpp)
target_link_libraries(auto_nav_5_node ${catkin_LIBRARIES})