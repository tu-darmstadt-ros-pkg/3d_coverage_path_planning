#include "three_dimensional_coverage_path_planning/waypoint_order_computation/mesh_navigation_path_planner.h"
#include "three_dimensional_coverage_path_planning/utils/utils.h"

#include <pluginlib/class_list_macros.h>
#include <tf2_ros/buffer.h>

namespace three_dimensional_coverage_path_planning
{

void MeshNavigationPathPlanner::initialize(ros::NodeHandle& nh)
{
  hector_waypoint_order::PathPlannerBase::initialize(nh);

  // mesh_map and mesh_planner have hard-coded namespaces they use for parameters. So here some parameters need to be changed/copied.
  ros::NodeHandle nh_node_ns("~");

  // set model_frame as global frame for mesh_map
  std::string model_frame = nh_node_ns.param<std::string>("model/model_frame", "building");
  nh_node_ns.setParam("mesh_map/global_frame", model_frame);


  // replace mesh_file with complete path
  std::string data_directory = nh_node_ns.param<std::string>("model/data_directory",
                                                             ros::package::getPath(ROS_PACKAGE_NAME) + "/data/");
  std::string mesh_file = nh_node_ns.param<std::string>("mesh_map/mesh_file", "");

  mesh_file = (mesh_file[0] == '/') ? mesh_file : data_directory + "/" + mesh_file;

  nh_node_ns.setParam("mesh_map/mesh_file", mesh_file);


  // tfBuffer is not used in MeshMap, but is required as argument.
  tf2_ros::Buffer tf_buffer;

  // read map from file defined in "~/mesh_map/mesh_file" with other parameters in namespace "~/mesh_map/"
  mesh_map_ptr_ = boost::make_shared<mesh_map::MeshMap>(tf_buffer);
  mesh_map_ptr_->readMap(false);

  // init planner loader
  planner_plugin_loader_ =
    std::make_unique<pluginlib::ClassLoader<mbf_mesh_core::MeshPlanner>>("mbf_mesh_core", "mbf_mesh_core::MeshPlanner");

  std::string planner_type, planner_name;

  utils::searchAndGetParam<std::string>(pnh_, "planner_type", "dijkstra_mesh_planner/DijkstraMeshPlanner",
                                        planner_type);
  utils::searchAndGetParam<std::string>(pnh_, "planner_name", "dijkstra_mesh_planner", planner_name);

  // load and init planner plugin
  mesh_planner_ptr = planner_plugin_loader_->createInstance(planner_type);

  if (!mesh_planner_ptr->initialize(planner_name, mesh_map_ptr_))
  {
    throw std::runtime_error("Initialization of mesh planner failed!");
  }

  // set logger level for planner
  std::string package_name = planner_type.substr(0, planner_type.find('/'));
  if (ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME"." + package_name, ros::console::levels::Warn))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  // get tolerance
  utils::searchAndGetParam(pnh_, "tolerance", 0.2f, tolerance_);


  path_pub_ = pnh_.advertise<nav_msgs::Path>("paths_between_candidates", 5, false);
  path_nan_pub_ = pnh_.advertise<nav_msgs::Path>("paths_nan", 5, false);
  path_failed_pub_ = pnh_.advertise<nav_msgs::Path>("paths_failed", 5, false);
}


bool
MeshNavigationPathPlanner::planPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& target,
                                    nav_msgs::Path& path, double& costs)
{

  // reset costs
  costs = 0;

  std::vector<geometry_msgs::PoseStamped> plan;
  std::string message;
  uint32_t result_code = mesh_planner_ptr->makePlan(start, target, tolerance_, plan, costs, message);

  // result code description: (taken from mbf_msgs/GetPath.action)
  /**
    # Predefined success codes:
    uint8 SUCCESS         = 0
    # 1..9 are reserved as plugin specific non-error results

    # Possible error codes:
    uint8 CANCELED        = 51
    uint8 INVALID_START   = 52
    uint8 INVALID_GOAL    = 53
    uint8 NO_PATH_FOUND   = 54
    (...) (here only the ones that are used in dijkstra and cvp)

    # 71..99 are reserved as plugin specific errors
  **/

  // success
  if (result_code < 10)
  {
    bool contains_nan = false;
    for (auto& p: plan)
    {
      // Due to a problem in the mesh navigation, sometimes it could happen that the orientation contains NaNs.
      // This problem should be fixed by now, but the check still exists to be sure.
      if (std::isnan(p.pose.orientation.x))
      {
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z = 0;
        p.pose.orientation.w = 0;
        contains_nan = true;
      }
    }

    // pass computed plan to output
    path.header.frame_id = start.header.frame_id;
    path.poses = plan;

    // remove time stamps from poses, as the time stamps are the same for each pose and our vehicle controller cannot handle equal timestamps that are not 0.
    for (auto& pose: path.poses)
    {
      pose.header.stamp = ros::Time();
    }

    if (contains_nan)
    {
      path_nan_pub_.publish(path);
    }
    else
    {
      path_pub_.publish(path);
    }

    return true;
  }

  // error
  if (result_code == 51)
  {
    ROS_ERROR_STREAM("Mesh planner: makePlan was canceled.");
  }
  else if (result_code == 52)
  {
    ROS_ERROR_STREAM(
      "Mesh planner: invalid start: position" << utils::to_string(start.pose.position) << ", orientation: "
                                              << utils::to_string(start.pose.orientation));
  }
  else if (result_code == 53)
  {
    ROS_ERROR_STREAM(
      "Mesh planner: invalid goal: position" << utils::to_string(target.pose.position) << ", orientation: "
                                             << utils::to_string(target.pose.orientation));
  }
  else if (result_code == 54)
  {
    ROS_ERROR_STREAM("Mesh planner could not find a path between " << utils::to_string(start.pose.position) << " and "
                                                                   << utils::to_string(target.pose.position) << ".");
  }
  else
  {
    std::string error_mgs = "Mesh planner resulted with error code " + std::to_string(result_code);
    if (!message.empty())
    {
      error_mgs += " and message \"" + message + "\"";
    }
    ROS_ERROR_STREAM(error_mgs);
  }

  nav_msgs::Path failed_path;
  failed_path.header.frame_id = start.header.frame_id;
  failed_path.poses.push_back(start);
  failed_path.poses.push_back(target);

  path_failed_pub_.publish(failed_path);

  return false;
}
} // end namespace three_dimensional_coverage_path_planning

PLUGINLIB_EXPORT_CLASS(three_dimensional_coverage_path_planning::MeshNavigationPathPlanner,
                       hector_waypoint_order::PathPlannerBase)