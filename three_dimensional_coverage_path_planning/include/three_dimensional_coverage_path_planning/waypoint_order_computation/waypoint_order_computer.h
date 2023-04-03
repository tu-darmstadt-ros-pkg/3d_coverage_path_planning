#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_WAYPOINT_ORDER_COMPUTER_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_WAYPOINT_ORDER_COMPUTER_H


#include "three_dimensional_coverage_path_planning/model/model_data.h"

#include <hector_waypoint_order/cost_computation/cost_computer_base.h>
#include <hector_waypoint_order/waypoint_order_computer_base.h>

#include <pluginlib/class_loader.h>


namespace three_dimensional_coverage_path_planning
{

class WaypointOrderComputer
{
public:

  WaypointOrderComputer(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model);

  /**
   * Compute order for the given waypoints, depending on configurated cost computer and waypoint order computer.
   * @param waypoints_unordered
   * @return Vector containing the waypoints in best order.
   */
  Pose3dVector computeWaypointOrder(const Pose3dVector& waypoints_unordered);

  hector_waypoint_order::PathMap getPathsBetweenWaypoints();

private:

  std::unique_ptr<hector_waypoint_order::CostComputerBase> cost_computer_;
  pluginlib::ClassLoader<hector_waypoint_order::CostComputerBase> cost_computer_loader_;

  std::unique_ptr<hector_waypoint_order::WaypointOrderComputerBase> order_computer_;
  pluginlib::ClassLoader<hector_waypoint_order::WaypointOrderComputerBase> order_computer_loader_;

  ros::NodeHandle pnh_;

  std::shared_ptr<const ModelData> model_;

  ros::Publisher path_pub_;

  hector_waypoint_order::PathMap path_map_;
};
} // end namespace three_dimensional_coverage_path_planning

#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_WAYPOINT_ORDER_COMPUTER_H
