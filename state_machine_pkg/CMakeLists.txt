cmake_minimum_required(VERSION 3.0.2)
project(state_machine_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_srvs
  std_msgs
  actionlib
  move_base_msgs
  ar_pose
  arm_controller
  relative_move
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES state_machine_pkg
  CATKIN_DEPENDS roscpp rospy std_srvs std_msgs actionlib move_base_msgs ar_pose arm_controller relative_move
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 状态机库
add_library(${PROJECT_NAME}
  src/state_machine.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# 状态机主节点（实际运行用）
add_executable(state_machine_node src/main.cpp)
add_dependencies(state_machine_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_machine_node ${PROJECT_NAME} ${catkin_LIBRARIES})

# 测试节点（模拟测试用）
add_executable(test_state_machine_node test/test_state_machine.cpp)
add_dependencies(test_state_machine_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_state_machine_node ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} state_machine_node test_state_machine_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
