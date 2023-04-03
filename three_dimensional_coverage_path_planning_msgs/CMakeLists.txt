cmake_minimum_required(VERSION 3.5)
project(three_dimensional_coverage_path_planning_msgs)


find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  std_msgs
  nav_msgs
  move_base_lite_msgs
  )


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Viewpoint.msg
  ViewpointArray.msg
  Waypoint.msg
  Path.msg
  PointArray.msg
  MoveResultCode.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  Plan3dCoveragePathOnMesh.action
  InitializeExecutionOf3dCoveragePath.action
  MoveToWaypointOf3dCoveragePath.action
  WaypointOf3dCoveragePathReached.action
  ExecuteRecoveryOn3dCoveragePath.action
  FinishExecutionOf3dCoveragePath.action
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  move_base_lite_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs std_msgs geometry_msgs nav_msgs move_base_lite_msgs
)
