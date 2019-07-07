cmake_minimum_required(VERSION 2.8.3)
project(state_estimate_filter_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    tf
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

add_definitions(${EIGEN3_DEFINITIONS})

add_executable(ar_pose_publisher
  src/ar_pose_publisher.cpp)
target_link_libraries(ar_pose_publisher
  ${catkin_LIBRARIES})

add_executable(phoxi_pose_publisher
  src/phoxi_pose_publisher.cpp)
target_link_libraries(phoxi_pose_publisher
  ${catkin_LIBRARIES})

add_executable(oneshot_calibrator
  src/oneshot_calibrator.cpp)
target_link_libraries(oneshot_calibrator
  ${catkin_LIBRARIES})

add_executable(fixed_phoxi_tf_broadcaster
  src/fixed_phoxi_tf_broadcaster.cpp
  src/fixed_phoxi_tf_broadcaster_node.cpp)
target_link_libraries(fixed_phoxi_tf_broadcaster
  ${catkin_LIBRARIES})

add_library(state_estimate_filter
  src/state_estimate_filter.cpp)
target_link_libraries(state_estimate_filter
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES})

###########
# Install #
###########

set(EXEC_FILES
  ar_pose_publisher
  phoxi_pose_publisher
  oneshot_calibrator
  fixed_phoxi_tf_broadcaster)

set(DIRECTORIES
  launch)

# Install executables
foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(nodes)

# Install include directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install all files
foreach(dirs ${DIRECTORIES})
  install(DIRECTORY ${dirs}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach(dirs)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
