cmake_minimum_required(VERSION 2.8.3)
project(spmb)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  roscpp
)

# catkin_python_setup()

add_message_files(
   FILES
   actuated.msg
   request.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs 
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

