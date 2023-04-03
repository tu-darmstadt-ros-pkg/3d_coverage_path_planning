
#include "three_dimensional_coverage_path_planning/waypoint_order_computation/waypoint_order_computer.h"

#include "three_dimensional_coverage_path_planning/utils/utils_with_types.h"

#include <hector_waypoint_order/utils/file_utils.h>
#include <nav_msgs/Path.h>

namespace three_dimensional_coverage_path_planning
{

WaypointOrderComputer::WaypointOrderComputer(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model)
  : cost_computer_loader_("hector_waypoint_order",
                          "hector_waypoint_order::CostComputerBase"),
    order_computer_loader_("hector_waypoint_order",
                           "hector_waypoint_order::WaypointOrderComputerBase"),
    pnh_(pnh, "waypoint_order_computation"),
    model_(std::move(model))
{

  std::string cost_computer_name = pnh_.param<std::string>("cost_computer_plugin",
                                                           "hector_waypoint_order::PathCostComputer");

  cost_computer_.reset(cost_computer_loader_.createUnmanagedInstance(cost_computer_name));
  cost_computer_->initialize(pnh_);


  std::string order_computer_name = pnh_.param<std::string>("waypoint_order_computer_plugin",
                                                            "hector_waypoint_order::MstTspSolver");

  order_computer_.reset(order_computer_loader_.createUnmanagedInstance(order_computer_name));

  path_pub_ = pnh_.advertise<nav_msgs::Path>("tsp_path", 5, true);
}

Pose3dVector WaypointOrderComputer::computeWaypointOrder(const Pose3dVector& waypoints_unordered)
{
  std::vector<geometry_msgs::PoseStamped> waypoints_msgs;

  // convert Pose3d to geometry_msgs::PoseStamped
  for (auto& waypoint: waypoints_unordered)
  {
    waypoints_msgs.push_back(utils::toMsgPoseStamped(waypoint));
  }

  // compute costs
  hector_waypoint_order::CostMap waypoint_cost_map = cost_computer_->computeCosts(waypoints_msgs);

  path_map_ = cost_computer_->getPaths();

  // TEMPORARY save waypoints and cost map for TSP solver tests
//  std::string file_name = "/home/katrin/hector/src/hector_waypoint_order/test/test_node/data/tsp_waypoints_costmap.bag";
//  hector_waypoint_order::file_utils::writeWaypointsAndCostMapToFile(file_name, waypoints_msgs, waypoint_cost_map);


  // compute order
  order_computer_->initialize(pnh_, waypoints_msgs, waypoint_cost_map);

  std::vector<geometry_msgs::PoseStamped> scan_waypoints = order_computer_->computeWaypointOrder();

  // TEMPORARY print
  ROS_WARN_STREAM("Computed path costs: " << order_computer_->getPathCosts());


  // compute complete path for visualization
  if (scan_waypoints.size() > 1)
  {
    // use paths between the waypoints (computed in cost computer) to make complete path
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = model_->getModelFrame();

    // add first waypoint
    path_msg.poses.push_back(scan_waypoints[0]);

    for (unsigned int i = 1; i < scan_waypoints.size(); ++i)
    {
      auto part_path = path_map_.at({scan_waypoints[i - 1], scan_waypoints[i]});

      // part_path is added starting with the second element to avoid point duplicates as
      // end of last part_path = begin of current part_path
      path_msg.poses.insert(path_msg.poses.end(), part_path.poses.begin() + 1, part_path.poses.end());
    }

    path_pub_.publish(path_msg);
  }


  // convert msgs to Pose3d
  Pose3dVector path;
  for (auto& point: scan_waypoints)
  {
    path.push_back(utils::toPose3d(point));
  }

  return path;
}

hector_waypoint_order::PathMap WaypointOrderComputer::getPathsBetweenWaypoints()
{
  return path_map_;
}
} // end namespace three_dimensional_coverage_path_planning
