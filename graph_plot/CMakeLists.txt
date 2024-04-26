cmake_minimum_required(VERSION 3.8)
project(graph_plot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(graph_display REQUIRED)
find_package(graph_ros2 REQUIRED)

add_executable(plot_paths src/plot_paths.cpp)
ament_target_dependencies(plot_paths
    rclcpp
    rclcpp_components
    graph_display
    graph_ros2
)

add_executable(plot_tree src/plot_tree.cpp)
ament_target_dependencies(plot_tree
    rclcpp
    rclcpp_components
    graph_display
    graph_ros2
)

add_executable(plot_ik src/plot_ik.cpp)
ament_target_dependencies(plot_ik
    rclcpp
    rclcpp_components
    graph_display
    graph_ros2
)

install(TARGETS
  plot_paths
  plot_tree
  plot_ik
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME} )

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

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
