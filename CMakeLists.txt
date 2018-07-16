cmake_minimum_required(VERSION 2.8.3)
project(grasp_planning)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  rospy
)

# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    ros/include
  CATKIN_DEPENDS
    ssd_keras_ros
    gqcnn
    mas_perception_libs
)

###########
## Build ##
###########
include_directories(
 ros/include
  ${catkin_INCLUDE_DIRS}
)

# executable ssd_keras_gqcnn_node
add_executable(ssd_keras_gqcnn_node
  ros/src/ssd_keras_gqcnn_node.cpp
)
add_dependencies(ssd_keras_gqcnn_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ssd_keras_gqcnn_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS ssd_keras_gqcnn_node
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
