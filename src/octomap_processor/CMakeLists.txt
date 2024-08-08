cmake_minimum_required(VERSION 3.0.2)
project(octomap_processor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  octomap_ros
  octomap_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros pcl_conversions octomap_ros octomap_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/octomap_processor_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
