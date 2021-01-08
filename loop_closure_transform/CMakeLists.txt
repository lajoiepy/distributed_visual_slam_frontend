cmake_minimum_required(VERSION 3.0.2)
project(loop_closure_transform)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs  )

find_package(OpenCV REQUIRED)

### Find GTSAM ###
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})
message("GTSAM Libraries: ${GTSAM_LIBRARIES}")

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  StereoMatch.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy kimera_vio kimera_vio_ros tf tf_conversions tf2_ros
)

include_directories(
  /root/vio-ws/src/Kimera-VIO/include/
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/loop_closure_transform_node.cpp)
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtsam 
  /root/vio-ws/devel/lib/libkimera_vio.so 
  /root/vio-ws/devel/lib/libkimera_vio_ros.so
  )