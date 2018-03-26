cmake_minimum_required(VERSION 2.8.3)
project(euclidean_clustering)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_filters
  my_new_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(euclidean_cluster_extraction src/euclidean_cluster_extraction.cpp)
target_link_libraries(euclidean_cluster_extraction ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(colored_clusters src/colored_clusters.cpp)
target_link_libraries(colored_clusters ${catkin_LIBRARIES} ${PCL_LIBRARIES})