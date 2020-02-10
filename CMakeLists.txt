cmake_minimum_required(VERSION 2.8.3)
project(uav_dynamics)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  rosflight_sil
  visualization_msgs
  tf2
  tf2_ros
  aerowake_utils
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rospy geometry_msgs visualization_msgs
    tf2 tf2_ros aerowake_utils rosflight_sil
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(uav_dynamics_node
  src/uav_dynamics_node.cpp
  src/uav_dynamics.cpp
  src/uav_dynamics_ros.cpp
)
add_dependencies(uav_dynamics_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(uav_dynamics_node
  aerowake-utils-lib
  ${catkin_LIBRARIES}
)

add_executable(world_to_NED_broadcaster
  src/world_to_NED_broadcaster.cpp
)
add_dependencies(world_to_NED_broadcaster
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(world_to_NED_broadcaster
  aerowake-utils-lib
  ${catkin_LIBRARIES}
)
