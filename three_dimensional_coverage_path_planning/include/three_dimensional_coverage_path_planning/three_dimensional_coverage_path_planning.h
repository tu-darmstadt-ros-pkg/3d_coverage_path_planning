
#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_H

#include "three_dimensional_coverage_path_planning/model/building_model.h"
#include "three_dimensional_coverage_path_planning/model/real_world_model.h"

#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_computer.h"
#include "three_dimensional_coverage_path_planning/waypoint_order_computation/waypoint_order_computer.h"
#include "three_dimensional_coverage_path_planning/data_recording/data_recorder_base.h"

#include "three_dimensional_coverage_path_planning/utils/action_types.h"

#include <three_dimensional_coverage_path_planning_msgs/Path.h>

#include <ros/ros.h>


namespace three_dimensional_coverage_path_planning
{

class ThreeDimensionalCoveragePathPlanning
{
public:
  ThreeDimensionalCoveragePathPlanning(ros::NodeHandle& nh, ros::NodeHandle& pnh);


  void plan3dCoveragePathOnMesh(bool load_models, bool save_models, bool load_path,
                                PlanActionServer* const action_server = nullptr);

  /**
   * get path as message
   * @param viewpoint_info if true, the reward and the visible targets lists are also stored in the message. Otherwise only the waypoint and the viewpoint position are stored.
   * @return
   */
  three_dimensional_coverage_path_planning_msgs::Path getPath(bool viewpoint_info = true);


  void initializeExecutionOfPath(const three_dimensional_coverage_path_planning_msgs::Path& path);

  void waypointReached(const three_dimensional_coverage_path_planning_msgs::Waypoint& waypoint);

  void recover(const three_dimensional_coverage_path_planning_msgs::Waypoint& waypoint, bool waypoint_reached);

  std::string finishExecutionOfPath();

private:


  void convertViewpointsToWaypoints();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // unique_ptr to allow the models to be initialized when required and not necessarily in constructor
  std::unique_ptr<BuildingModel> building_model_;
  std::unique_ptr<RealWorldModel> real_world_model_;

  std::unique_ptr<ViewpointComputer> viewpoint_computer_;
  std::unique_ptr<WaypointOrderComputer> waypoint_order_computer_;

  std::unique_ptr<DataRecorderBase> data_recorder_;
  pluginlib::ClassLoader<DataRecorderBase> data_recorder_loader_;

  std::shared_ptr<std::vector<Viewpoint>> selected_viewpoints_;
  std::map<Pose3d, Viewpoint, Pose3dLessComparator> waypoints_to_viewpoints_;
  Pose3dVector waypoints_unordered_;
  Pose3dVector path_scanpoints_;

  hector_waypoint_order::PathMap paths_between_waypoints_;

  // visualization
  ros::Publisher waypoints_pub_;
  ros::Publisher complete_original_path_pub_;
};
} // end namespace three_dimensional_coverage_path_planning

#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_H
