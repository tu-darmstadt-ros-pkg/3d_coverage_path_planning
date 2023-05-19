cmake_minimum_required(VERSION 3.0.2)
project(three_dimensional_coverage_path_planning_states)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME})
