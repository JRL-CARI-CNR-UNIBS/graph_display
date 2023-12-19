cmake_minimum_required(VERSION 3.0.2)
project(graph_plot)
add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS
  graph_core
  roscpp
  rosparam_utilities
  graph_display
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES graph_plot
 CATKIN_DEPENDS graph_core roscpp rosparam_utilities graph_display
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

add_executable(plot_tree src/plot_tree.cpp)
add_dependencies(plot_tree ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(plot_tree
  ${catkin_LIBRARIES}
)

add_executable(plot_ik src/plot_ik.cpp)
add_dependencies(plot_ik ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(plot_ik
  ${catkin_LIBRARIES}
)
