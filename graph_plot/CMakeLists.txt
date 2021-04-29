cmake_minimum_required(VERSION 3.0.2)
project(graph_plot)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  graph_core
  roscpp
  rosparam_utilities
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES graph_plot
 CATKIN_DEPENDS graph_core roscore rosparam_utilities
 DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(plot_paths src/plot_paths.cpp)
add_dependencies(plot_paths ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(plot_paths
  ${catkin_LIBRARIES}
)

