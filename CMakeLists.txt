cmake_minimum_required(VERSION 2.8.3)
project(state_estimate_filter_ros)

# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
  DEPENDS EIGEN3 OpenCV
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS})

add_definitions(${EIGEN3_DEFINITIONS})

add_library(${PROJECT_NAME}_state_estimate_filter
  src/state_estimate_filter.cpp)
target_link_libraries(${PROJECT_NAME}_state_estimate_filter
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES})

add_library(${PROJECT_NAME}_particle_filter
  src/particle_filter.cpp)
target_link_libraries(${PROJECT_NAME}_particle_filter
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES})

# probability distribution
add_library(${PROJECT_NAME}_probability_distribution
  src/probability_distribution/probability_distribution.cpp)
target_link_libraries(${PROJECT_NAME}_probability_distribution
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES})

add_library(${PROJECT_NAME}_normal_distribution
  src/probability_distribution/normal_distribution.cpp)
target_link_libraries(${PROJECT_NAME}_normal_distribution
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES})

add_library(${PROJECT_NAME}_depth_sensor_distribution
  src/probability_distribution/depth_sensor_distribution.cpp)
target_link_libraries(${PROJECT_NAME}_depth_sensor_distribution
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES})

add_executable(${PROJECT_NAME}_image_noise_mixer
  src/image_noise_mixer/image_noise_mixer.cpp
  src/particle_filter.cpp
  src/probability_distribution/normal_distribution.cpp
  src/probability_distribution/probability_distribution.cpp
  src/state_estimate_filter.cpp)
target_link_libraries(${PROJECT_NAME}_image_noise_mixer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_probability_distribution_run
  src/probability_distribution/depth_sensor_distribution.cpp
  src/probability_distribution/normal_distribution.cpp
  src/probability_distribution/probability_distribution.cpp
  src/probability_distribution/probability_distribution_run.cpp)
target_link_libraries(${PROJECT_NAME}_probability_distribution_run
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${PYTHON_LIBRARIES})

add_executable(${PROJECT_NAME}_my_eigen_samples
  src/eigen_samples.cpp)
target_link_libraries(${PROJECT_NAME}_my_eigen_samples
  ${Eigen_LIBRARIES})

add_executable(${PROJECT_NAME}_state_estimate_filter_node
  src/particle_filter.cpp
  src/probability_distribution/normal_distribution.cpp
  src/probability_distribution/probability_distribution.cpp
  src/state_estimate_filter.cpp
  src/state_estimate_filter_node.cpp)
target_link_libraries(${PROJECT_NAME}_state_estimate_filter_node
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES})

###########
# Install #
###########

set(BUILT_FILES
  ${PROJECT_NAME}_state_estimate_filter
  ${PROJECT_NAME}_particle_filter
  ${PROJECT_NAME}_my_eigen_samples
  ${PROJECT_NAME}_image_noise_mixer
  ${PROJECT_NAME}_probability_distribution_run)

set(DIRECTORIES
  launch)

# Install executables
foreach(nodes ${BUILT_FILES})
  install(TARGETS ${nodes}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

# Install include directory
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install all files
foreach(dirs ${DIRECTORIES})
  install(DIRECTORY ${dirs}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslaunch
    rostest
  )

#roslaunch_add_file_check(launch)

  add_rostest_gtest(probability_distribution_test
    test/probability_distribution.test
    test/probability_distribution_test.cpp
    src/probability_distribution/normal_distribution.cpp
    src/probability_distribution/probability_distribution.cpp)
  target_link_libraries(probability_distribution_test
      ${catkin_LIBRARIES})

endif()
