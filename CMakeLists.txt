cmake_minimum_required(VERSION 2.8.3)
project(egocircle)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ego_circle_lib
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_ros pcl_conversions sensor_msgs tf2_ros tf2_sensor_msgs visualization_msgs
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

# Target names are not consistent with filenames to avoid breaking api changes
add_library(ego_circle_lib src/ego_circle.cpp)
add_dependencies(ego_circle_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(ego_circle_lib PUBLIC -std=c++11)
target_link_libraries(ego_circle_lib 
  ${catkin_LIBRARIES}
)

add_executable(ego_circle src/ego_circle_node.cpp)
add_dependencies(ego_circle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(ego_circle PUBLIC -std=c++11)
target_link_libraries(ego_circle
  ego_circle_lib
  ${catkin_LIBRARIES}
)
