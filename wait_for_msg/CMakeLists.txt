cmake_minimum_required(VERSION 2.8.3)
project(wait_for_msg)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package(
  DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

## Declare a C++ executable
add_executable(wait_for_msg_node src/wait_for_msg_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(wait_for_msg_node
  ${catkin_LIBRARIES}
)
