cmake_minimum_required(VERSION 2.8.3)
project(state_estimate_filter_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    DEPENDS Eigen3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

add_definitions(${EIGEN3_DEFINITIONS})

add_library(state_estimate_filter
  src/state_estimate_filter.cpp)
target_link_libraries(state_estimate_filter
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES})

add_library(particle_filter
  src/particle_filter.cpp)
target_link_libraries(particle_filter
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES})

add_executable(my_eigen_samples
  src/eigen_samples.cpp)
target_link_libraries(eigen_samples
  ${Eigen_LIBRARIES})

###########
# Install #
###########

set(BUILT_FILES
  state_estimate_filter
  particle_filter
  my_eigen_samples)

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
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
