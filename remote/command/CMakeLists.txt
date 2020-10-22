cmake_minimum_required(VERSION 2.8.3)
project(command)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


add_message_files(
  FILES
  chasis.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
