cmake_minimum_required(VERSION 3.0.2)
project(cubic_spline)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11 -O3 -g -Wall)
add_compile_options(-std=c++17 -O3 -g -Wall)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cubic_spline
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME}    src/cubic_spline.cpp)
add_library(debug_path_creator src/debug_path_creator.cpp)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node    src/cubic_spline_node.cpp)
add_executable(debug_path_creator_node src/debug_path_creator_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(debug_path_creator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node    ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(debug_path_creator_node debug_path_creator ${catkin_LIBRARIES})