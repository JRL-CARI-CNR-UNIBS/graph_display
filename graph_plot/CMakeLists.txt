cmake_minimum_required(VERSION 3.8)
project(graph_plot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(graph_ros2 REQUIRED)
find_package(graph_display REQUIRED)
find_package(graph_display REQUIRED)

#add_executable(plot_paths src/plot_paths.cpp)
#ament_target_dependencies(plot_paths
#  PUBLIC
#    rclcpp
#    rclcpp_components
#    graph_ros2
#    graph_display
#)
#target_include_directories(plot_paths
#  PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

#add_executable(plot_tree src/plot_tree.cpp)
#ament_target_dependencies(plot_tree
#  PUBLIC
#    rclcpp
#    rclcpp_components
#    graph_ros2
#    graph_display
#)
#target_include_directories(plot_tree
#  PUBLIC
#    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
#    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

add_executable(plot_ik src/plot_ik.cpp)
ament_target_dependencies(plot_ik
  PUBLIC
    rclcpp
    rclcpp_components
    graph_ros2
    graph_display
)
target_include_directories(plot_ik
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

install(TARGETS
#  plot_paths
#  plot_tree
  plot_ik
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME} )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
