cmake_minimum_required(VERSION 2.8.3)
project(object_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
pcl_ros
roscpp
sensor_msgs
std_msgs
pcl_conversions
rospy
tf
cv_bridge
image_transport
compressed_image_transport
jsk_recognition_msgs
jsk_rviz_plugins
tf2
tf2_ros
tf2_msgs
argo_track_msgs
track_array_msgs)


catkin_package(
CATKIN_DEPENDS 
	   pcl_ros
	   roscpp
	   sensor_msgs
	   pcl_conversions
   INCLUDE_DIRS include
#  LIBRARIES object_detection
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 src
 ${catkin_INCLUDE_DIRS}
)



add_executable(
   test_sift
   src/test_sift.cpp
 )

add_executable(
   create_range_img
   src/range_img.cpp
)

add_executable(
   voxel
   src/test_voxel_grid_rate.cpp
)


add_executable(
   range_image_visualization
   src/range_image_visualization.cpp
)



add_executable(
   read_pc
   src/read_pc.cpp
)

add_executable(
   model_sift
   src/model_sift.cpp
)

add_executable(
   kd_search
   src/kd_search.cpp
)

add_executable(
   icp
   src/icp.cpp
)

add_executable(
   pass
   src/passthrough.cpp
)

add_executable(
   ply2pcd
   src/ply2pcd.cpp
)

add_executable(
   icp_matching
   src/icp_matching.cpp
)




target_link_libraries(
   test_sift
   ${catkin_LIBRARIES}
)

target_link_libraries(
   create_range_img
   ${catkin_LIBRARIES}
)

target_link_libraries(
   range_image_visualization
   ${catkin_LIBRARIES}
)

target_link_libraries(
   voxel
   ${catkin_LIBRARIES}
)

target_link_libraries(
   read_pc
   ${catkin_LIBRARIES}
)

target_link_libraries(
   model_sift
   ${catkin_LIBRARIES}
)

target_link_libraries(
   kd_search
   ${catkin_LIBRARIES}
)


target_link_libraries(
   icp
   ${catkin_LIBRARIES}
)

target_link_libraries(
   pass
   ${catkin_LIBRARIES}
)

target_link_libraries(
   ply2pcd
   ${catkin_LIBRARIES}
)

target_link_libraries(
   icp_matching
   ${catkin_LIBRARIES}
)



