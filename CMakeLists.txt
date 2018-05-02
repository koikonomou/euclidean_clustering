cmake_minimum_required(VERSION 2.8.3)
project(euclidean_clustering)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_filters
  my_new_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL 1.8.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost COMPONENTS format)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(euclidean_cluster_extraction src/euclidean_cluster_extraction.cpp)
target_link_libraries(euclidean_cluster_extraction ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(colored_clusters src/colored_clusters.cpp)
target_link_libraries(colored_clusters ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(tracking src/tracking.cpp)
target_link_libraries(tracking ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cluster_id src/cluster_id.cpp)
target_link_libraries(cluster_id ${catkin_LIBRARIES} )


