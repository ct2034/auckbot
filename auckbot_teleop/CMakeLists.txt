cmake_minimum_required(VERSION 2.8.3)
project(auckbot_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
   CATKIN_DEPENDS roscpp
   DEPENDS ncurses
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(auckbot_teleop_node src/keyboard_teleop.cpp)

target_link_libraries(
  auckbot_teleop_node
  ${catkin_LIBRARIES}
  ncurses
)
