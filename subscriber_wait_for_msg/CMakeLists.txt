cmake_minimum_required(VERSION 2.8.3)
project(subscriber_wait_for_msg)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package(
  DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

## Declare a C++ executable
add_executable(subscriber_wait_for_msg_node src/subscriber_wait_for_msg_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(subscriber_wait_for_msg_node
  ${catkin_LIBRARIES}
)
