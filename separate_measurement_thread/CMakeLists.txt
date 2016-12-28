cmake_minimum_required(VERSION 2.8.3)
project(separate_measurement_thread)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package(
  DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

## Declare a C++ executable
add_executable(separate_measurement_thread_node src/separate_measurement_thread_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(separate_measurement_thread_node
  ${catkin_LIBRARIES}
)
