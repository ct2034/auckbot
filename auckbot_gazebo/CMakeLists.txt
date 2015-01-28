cmake_minimum_required(VERSION 2.8.3)
project(auckbot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  gazebo_ros
)

add_message_files(
  FILES
  MotorCurrents.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
#  auckbot_gazebo
#  message_generation
)

## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES auckbot_teleop
   CATKIN_DEPENDS roscpp message_runtime
   DEPENDS eigen
)

add_executable(current_pub src/current_pub.cpp)

target_link_libraries(
  current_pub
  ${catkin_LIBRARIES}
)

add_dependencies(current_pub ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

