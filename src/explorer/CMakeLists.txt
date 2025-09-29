cmake_minimum_required(VERSION 3.0.2)
project(explorer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(explorer_node src/explorer.cpp src/explorer_node.cpp)
target_link_libraries(explorer_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(explorer_node ${catkin_EXPORTED_TARGETS})
