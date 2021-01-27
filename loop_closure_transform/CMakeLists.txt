cmake_minimum_required(VERSION 2.8.3)
project(loop_closure_transform)
set (CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  rtabmap_ros
  tf 
  tf_conversions 
  tf2_ros 
  eigen_conversions
  cv_bridge
  image_transport
  sensor_msgs
  roslaunch
)

roslaunch_add_file_check(launch)


FIND_PACKAGE(RTABMap REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(GTSAM REQUIRED)
FIND_PACKAGE(PCL REQUIRED)

# Find Qt5 first
FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui Svg QUIET)
IF(NOT Qt5_FOUND)
  FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtSvg)
ENDIF(NOT Qt5_FOUND)

# fix libproj.so not found on Xenial
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
endif()

if(POLICY CMP0020)
  cmake_policy(SET CMP0020 OLD)
endif()


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  KeyPointVec.msg
  KeyPoint3DVec.msg
  Descriptors.msg
  StereoMatch.msg
  EstTransform.msg
  StereoExtractResult.msg
  StereoImagePair.msg
  EstimatedTransform.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  FindMatches.srv
  ReceiveLoopClosures.srv
  PoseEstimates.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  rtabmap_ros
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES loop_closure_transform
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs message_runtime rtabmap_ros tf tf_conversions tf2_ros eigen_conversions cv_bridge image_transport sensor_msgs std_srvs
)

include_directories(
  ${RTABMap_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

link_directories(${GTSAM_LIBRARY_DIRS})

add_library(loop_closure_transform 
  src/Registration.cpp src/RegistrationVis.cpp src/MsgConversion.cpp src/logger.cpp
)
target_link_libraries(loop_closure_transform ${catkin_LIBRARIES} ${RTABMap_LIBRARIES} ${OpenCV_LIBRARIES} ${QT_LIBRARIES} ${PCL_LIBRARIES} gtsam)
add_dependencies(loop_closure_transform loop_closure_transform_gencpp)

# add_executable(stereo_cam_geometric_tools src/stereo_cam_geometric_tools.cpp src/Registration.cpp src/RegistrationVis.cpp src/MsgConversion.cpp)
# target_link_libraries(stereo_cam_geometric_tools ${catkin_LIBRARIES} ${RTABMap_LIBRARIES} ${OpenCV_LIBRARIES} ${QT_LIBRARIES} ${PCL_LIBRARIES} gtsam loop_closure_transform)
# add_dependencies(stereo_cam_geometric_tools ${catkin_EXPORTED_TARGETS} loop_closure_transform_gencpp)

add_executable(${PROJECT_NAME}_node 
  src/loop_closure_transform_node.cpp
  src/stereo_cam_geometric_tools.cpp
  )
target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  loop_closure_transform
  gtsam 
  )