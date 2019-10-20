cmake_minimum_required(VERSION 2.8.3)
project(uav_dynamics)

find_package(catkin_simple REQUIRED)

catkin_simple()

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
if (NOT TARGET utils)
  include_directories(lib/utils/include)
  add_subdirectory(lib/utils)
endif()

cs_add_executable(uav_dynamics_node
  src/uav_dynamics_node.cpp
  src/uav_dynamics.cpp
  src/uav_dynamics_ros.cpp
)
target_link_libraries(uav_dynamics_node utils ${catkin_LIBRARIES})

cs_add_executable(world_to_NED_broadcaster
  src/world_to_NED_broadcaster.cpp
)
target_link_libraries(world_to_NED_broadcaster utils ${catkin_LIBRARIES})

cs_install()
cs_export()
