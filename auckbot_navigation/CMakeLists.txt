cmake_minimum_required(VERSION 2.8.3)
project(auckbot_navigation)

SET(GCC_COVERAGE_COMPILE_FLAGS "-frounding-math")
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  move_base_msgs
  tf
  nav_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES auckbot_navigation
   CATKIN_DEPENDS roscpp
   DEPENDS CGAL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(path_optimizer src/path_optimizer.cpp)

target_link_libraries(path_optimizer
  ${catkin_LIBRARIES}
  CGAL
)