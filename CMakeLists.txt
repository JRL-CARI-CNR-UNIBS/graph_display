cmake_minimum_required(VERSION 3.16)
project(graph_display)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
  # set(CMAKE_BUILD_TYPE "Debug")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if(CMAKE_BUILD_TYPE MATCHES "Release")
    add_compile_options(-funroll-loops -Wall -Ofast -flto) #-O3
  endif() 
  if(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_options(-Wall -g -Og)
  endif() 
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(graph_core REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  moveit_visual_tools
  eigen_conversions
)

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

#Install

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
