cmake_minimum_required(VERSION 3.0.2)
project(ompl_rrt_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  octomap_msgs
  octomap   # 添加这一行
)

find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)  # 添加这一行

# 设置OMPL的路径
set(OMPL_DIR "/usr/local/include/ompl-1.6")
set(FCL_DIR "/usr/local/include/fcl/cmake")

# 手动包含OMPL头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ompl_rrt_planner
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs tf2_ros
  DEPENDS Eigen3 fcl octomap
)

add_executable(rrt_planner src/rrt_planner.cpp)
target_link_libraries(rrt_planner ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${FCL_LIBRARIES} ompl)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
