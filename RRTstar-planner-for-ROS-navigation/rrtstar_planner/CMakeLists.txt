cmake_minimum_required(VERSION 2.8.3)
project(rrtstar_planner)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  nav_core
  visualization_msgs
  roscpp
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(rrtstar_planner_lib src/rrtstarplan.cpp )

target_link_libraries(rrtstar_planner_lib ${catkin_LIBRARIES})
