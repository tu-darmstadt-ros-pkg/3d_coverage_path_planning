
#include "three_dimensional_coverage_path_planning/three_dimensional_coverage_path_planning_actions.h"

#include <ros/ros.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  three_dimensional_coverage_path_planning::ThreeDimensionalCoveragePathPlanningActions coverage_path_planning_actions(nh, pnh);

  // TODO all these parameters (load/save model, load/save path, execute path): in FlexBE
  bool load_models = pnh.param("load_models", false);
  bool save_models = pnh.param("save_models", true);
  bool load_path = pnh.param("load_path", false);
  bool save_path = pnh.param("save_path", true);
  bool execute_path = pnh.param("execute_path", true);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}