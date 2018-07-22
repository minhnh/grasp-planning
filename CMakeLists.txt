cmake_minimum_required(VERSION 2.8.3)
project(grasp_planning)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gqcnn
  mdr_pickup_action
  mas_perception_libs
  mcr_perception_msgs
  message_filters
  image_transport
  sensor_msgs
  visualization_msgs
  roslint
  roscpp
  rospy
)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    ros/include
  CATKIN_DEPENDS
    ssd_keras_ros
)

###########
## Build ##
###########
include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

# executable image_detection_gqcnn_node
add_executable(image_detection_gqcnn_node
  ros/src/image_detection_gqcnn_node.cpp
)
add_dependencies(image_detection_gqcnn_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(image_detection_gqcnn_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS image_detection_gqcnn_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

#############
## Testing ##
#############
roslint_cpp()
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()
