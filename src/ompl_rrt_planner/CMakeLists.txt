cmake_minimum_required(VERSION 3.0.2)
project(ompl_rrt_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  octomap_msgs
)

find_package(Eigen3 REQUIRED)
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")  # 手动设置Eigen3的路径

find_package(ccd REQUIRED)
set(CCD_LIBRARIES "/lib/x86_64-linux-gnu/libccd.so.2")

find_package(fcl REQUIRED)
set(FCL_INCLUDE_DIRS "/usr/local/include/fcl")  # 手动设置FCL的路径
set(FCL_LIBRARIES "/usr/local/lib/libfcl.so")

find_package(octomap REQUIRED)
set(OCTOMAP_INCLUDE_DIRS "/usr/local/include")  # 手动设置Octomap的路径
set(OCTOMAP_LIBRARIES "/usr/local/lib/liboctomap.so")

set(OMPL_DIR "/usr/local/include/ompl-1.6")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${FCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR} ${FCL_INCLUDE_DIRS}
  LIBRARIES ompl_rrt_planner
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs tf2_ros
  DEPENDS Eigen3 fcl octomap
)

add_executable(rrt_planner src/rrt_planner.cpp)
target_link_libraries(rrt_planner ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${FCL_LIBRARIES} ${CCD_LIBRARIES} ompl)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
