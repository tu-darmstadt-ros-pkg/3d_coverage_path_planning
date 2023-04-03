cmake_minimum_required(VERSION 3.1)
project(three_dimensional_coverage_path_planning)


set(CMAKE_CXX_STANDARD 14)

# TEMPORARY
#add_definitions(-Wall -Wextra -Wno-unused-function -Wno-unused-variable)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  pluginlib
  actionlib
  rosbag

  octomap_ros
  pcl_ros
  voxblox_ros

  mesh_map
  mbf_mesh_core

  mesh_conversions
  mesh_to_sampled_point_cloud
  mesh_to_sdf

  filters
  robot_body_filter

  grid_map_planner_lib
  grid_map_proc

  hector_waypoint_order

  three_dimensional_coverage_path_planning_msgs
  )


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# switch off using hector make --this -DWITH_OPENMP=OFF
option(WITH_OPENMP "Use OpenMP for parallelization" OFF)
if (WITH_OPENMP)
  find_package(OpenMP)
  if (OPENMP_FOUND)
    MESSAGE("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  endif ()
endif (WITH_OPENMP)
unset(WITH_OPENMP CACHE) # unset cache so that default value is restored


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS

  roscpp
  roslib
  sensor_msgs
  pluginlib
  actionlib
  rosbag

  octomap_ros
  pcl_ros
  voxblox_ros

  mesh_map
  mbf_mesh_core

  mesh_conversions
  mesh_to_sampled_point_cloud
  mesh_to_sdf

  filters
  robot_body_filter

  grid_map_planner_lib
  grid_map_proc

  three_dimensional_coverage_path_planning_msgs

  #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


set(HEADER_FILES
  include/three_dimensional_coverage_path_planning/three_dimensional_coverage_path_planning.h
  include/three_dimensional_coverage_path_planning/three_dimensional_coverage_path_planning_actions.h

  include/three_dimensional_coverage_path_planning/data_recording/data_recorder_base.h
  include/three_dimensional_coverage_path_planning/data_recording/lidar_data_recorder.h

  include/three_dimensional_coverage_path_planning/model/model_data.h
  include/three_dimensional_coverage_path_planning/model/building_model.h
  include/three_dimensional_coverage_path_planning/model/real_world_model.h
  include/three_dimensional_coverage_path_planning/model/sdf_map.h

  include/three_dimensional_coverage_path_planning/path_execution/path_executor.h

  include/three_dimensional_coverage_path_planning/utils/colors.h
  include/three_dimensional_coverage_path_planning/utils/utils.h
  include/three_dimensional_coverage_path_planning/utils/utils_with_types.h
  include/three_dimensional_coverage_path_planning/utils/file_utils.h
  include/three_dimensional_coverage_path_planning/utils/types.h
  include/three_dimensional_coverage_path_planning/utils/action_types.h
  include/three_dimensional_coverage_path_planning/utils/transformation_helper.h

  include/three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint.h
  include/three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_computer.h
  include/three_dimensional_coverage_path_planning/viewpoint_computation/reward_evaluator.h

  include/three_dimensional_coverage_path_planning/viewpoint_computation/candidate_generator/candidate_generator_base.h
  include/three_dimensional_coverage_path_planning/viewpoint_computation/candidate_generator/mesh_navigation_candidate_generator.h

  include/three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/viewpoint_selector_base.h
  include/three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/greedy_selector.h
  include/three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/greedy_without_redundancies_selector.h
  include/three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/probabilistic_selector.h
  include/three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/probabilistic_exp_dist_selector.h

  include/three_dimensional_coverage_path_planning/viewpoint_computation/visibility_checker/visibility_checker_base.h
  include/three_dimensional_coverage_path_planning/viewpoint_computation/visibility_checker/octomap_checker.h
  include/three_dimensional_coverage_path_planning/viewpoint_computation/visibility_checker/sdf_checker.h
  include/three_dimensional_coverage_path_planning/viewpoint_computation/visibility_checker/sensor_checker.h
  include/three_dimensional_coverage_path_planning/viewpoint_computation/visibility_checker/self_filter_checker.h

  include/three_dimensional_coverage_path_planning/waypoint_order_computation/waypoint_order_computer.h
  include/three_dimensional_coverage_path_planning/waypoint_order_computation/mesh_navigation_path_planner.h

  )

set(SOURCES
  src/three_dimensional_coverage_path_planning.cpp
  src/three_dimensional_coverage_path_planning_actions.cpp

  src/data_recording/data_recorder_base.cpp
  src/data_recording/lidar_data_recorder.cpp

  src/model/model_data.cpp
  src/model/building_model.cpp
  src/model/real_world_model.cpp
  src/model/sdf_map.cpp

  src/path_execution/path_executor.cpp

  src/util/transformation_helper.cpp

  src/viewpoint_computation/viewpoint.cpp
  src/viewpoint_computation/viewpoint_computer.cpp
  src/viewpoint_computation/reward_evaluator.cpp

  src/viewpoint_computation/candidate_generator/candidate_generator_base.cpp
  src/viewpoint_computation/candidate_generator/mesh_navigation_candidate_generator.cpp

  src/viewpoint_computation/viewpoint_selector/viewpoint_selector_base.cpp
  src/viewpoint_computation/viewpoint_selector/greedy_selector.cpp
  src/viewpoint_computation/viewpoint_selector/greedy_without_redundancies_selector.cpp
  src/viewpoint_computation/viewpoint_selector/probabilistic_selector.cpp
  src/viewpoint_computation/viewpoint_selector/probabilistic_exp_dist_selector.cpp

  src/viewpoint_computation/visibility_checker/visibility_checker_base.cpp
  src/viewpoint_computation/visibility_checker/octomap_checker.cpp
  src/viewpoint_computation/visibility_checker/sdf_checker.cpp
  src/viewpoint_computation/visibility_checker/sensor_checker.cpp
  src/viewpoint_computation/visibility_checker/self_filter_checker.cpp

  src/waypoint_order_computation/waypoint_order_computer.cpp
  src/waypoint_order_computation/mesh_navigation_path_planner.cpp

  )


## Declare a C++ library
add_library(${PROJECT_NAME} ${HEADER_FILES} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})


#############
## Install ##
#############

install(DIRECTORY
  launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )


#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  include_directories(${rostest_INCLUDE_DIRS})

  ## Add gtest based cpp test target and link libraries
  add_rostest_gtest(test_model_data test/test_model_data.test test/test_model_data.cpp)
  target_link_libraries(test_model_data ${PROJECT_NAME} ${rostest_LIBRARIES})

  add_rostest_gtest(test_viewpoint_selector test/test_viewpoint_selector.test test/test_viewpoint_selector.cpp)
  target_link_libraries(test_viewpoint_selector ${PROJECT_NAME} ${rostest_LIBRARIES})

endif ()



## Test node (no gtest), especially used to test and evaluate different solvers.

option(TEST_SELECTOR "Build executable for testing the selectors" ON) # TODO set TEST_SELECTOR to OFF

if (TEST_SELECTOR)
  ## Declare a C++ executable
  add_executable(test_selector_node test/test_selector_node/test_selector_node.cpp)
  add_dependencies(test_selector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_selector_node ${PROJECT_NAME})

endif ()