cmake_minimum_required(VERSION 3.0.2)
project(three_dimensional_coverage_path_planning_behaviors)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
    LIBRARIES ${PROJECT_NAME})

install(DIRECTORY manifest DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
