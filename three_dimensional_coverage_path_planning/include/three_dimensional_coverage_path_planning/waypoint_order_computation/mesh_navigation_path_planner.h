#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_MESH_NAVIGATION_PATH_PLANNER_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_MESH_NAVIGATION_PATH_PLANNER_H

#include <hector_waypoint_order/cost_computation/path_planner/path_planner_base.h>

#include <mesh_map/mesh_map.h>
#include <mbf_mesh_core/mesh_planner.h>

namespace three_dimensional_coverage_path_planning
{
class MeshNavigationPathPlanner : public hector_waypoint_order::PathPlannerBase
{
public:

  void initialize(ros::NodeHandle& nh) override;

  bool planPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& target, nav_msgs::Path& path, double& costs) override;

private:

  std::unique_ptr<pluginlib::ClassLoader <mbf_mesh_core::MeshPlanner>> planner_plugin_loader_;

  mesh_map::MeshMap::Ptr mesh_map_ptr_;

  mbf_mesh_core::MeshPlanner::Ptr mesh_planner_ptr;

  float tolerance_;

  ros::Publisher path_pub_;
  ros::Publisher path_nan_pub_;
  ros::Publisher path_failed_pub_;

};
} // end namespace three_dimensional_coverage_path_planning

#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_MESH_NAVIGATION_PATH_PLANNER_H
