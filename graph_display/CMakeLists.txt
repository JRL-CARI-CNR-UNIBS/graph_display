cmake_minimum_required(VERSION 3.0.2)
add_compile_options(-std=c++2a)
project(graph_display)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  moveit_visual_tools
  eigen_conversions
)

find_package(graph_core REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES graph_display
 CATKIN_DEPENDS roscpp moveit_visual_tools visualization_msgs eigen_conversions
 DEPENDS graph_core
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/graph_display.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${catkin_LIBRARIES}
  graph_core::graph_core
)
