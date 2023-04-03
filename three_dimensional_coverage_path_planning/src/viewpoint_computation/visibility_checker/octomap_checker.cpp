#include "three_dimensional_coverage_path_planning/viewpoint_computation/visibility_checker/octomap_checker.h"

#include "three_dimensional_coverage_path_planning/utils/utils_with_types.h"

#include <pluginlib/class_list_macros.h>

namespace three_dimensional_coverage_path_planning
{


OctomapChecker::OctomapChecker()
{
  plugin_name_ = "OctomapChecker";

  // get max index to use as vector sizes
  num_idx_ = 1 + std::max({OCTOMAP_CHECK_PASSED_IDX, OCTOMAP_RAYCAST_BLOCKED_IDX, OCTOMAP_RAYCAST_FAILED_IDX});
}

void OctomapChecker::initialize(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model)
{
  VisibilityCheckerBase::initialize(pnh, model);

  tolerance_ = pnh_.param("distance_tolerance", 0);

  // castRay gives center of cubes.
  // diagonal of a cube is sqrt(3)*edge_length, a point in the cube can be max. half this length away from the center
  threshold_ = sqrt(3) / 2 * model_->getOctomap()->getResolution() + tolerance_;

  if (publish_visibility_checks_)
  {
    // publisher and msg setup
    check_pubs_.resize(num_idx_);
    check_pubs_[OCTOMAP_CHECK_PASSED_IDX] = pnh_.advertise<visualization_msgs::Marker>("octomap_check_passed", 10,
                                                                                       true);
    check_pubs_[OCTOMAP_RAYCAST_BLOCKED_IDX] = pnh_.advertise<visualization_msgs::Marker>("octomap_raycast_blocked", 10,
                                                                                          true);
    check_pubs_[OCTOMAP_RAYCAST_FAILED_IDX] = pnh_.advertise<visualization_msgs::Marker>("octomap_raycast_failed", 10,
                                                                                         true);
  }
}


bool OctomapChecker::check(Pose3d& start, Point3d& end)
{

  if (!model_)
  {
    throw std::runtime_error("No model set in " + getPluginName() + "!");
  }

  auto octomap = model_->getOctomap();
  if (!octomap)
  {
    ROS_WARN_STREAM_THROTTLE(5, "No octomap in model with name " << model_->getModelName()
                                                                 << ", so no octomap check is performed.");
    return false;
  }

  // prepare data
  octomap::point3d origin, target;
  utils::toOctomapPoint3d(start.position, origin);
  utils::toOctomapPoint3d(end, target);

  octomap::point3d direction, ray_end;
  direction = (target - origin).normalize();

  // set max_range to distance between target and origin to abort
  double max_range = origin.distance(target) + 0.1;

  // set ignoreUnknown true, as otherwise, if the start node is unknown, the ray cast is aborted directly
  bool result = octomap->castRay(origin, direction, ray_end, true, max_range);


  if (!result)
  {
    addMarkerToMsg(OCTOMAP_RAYCAST_FAILED_IDX, start.position, end, colors::yellow());
    ROS_WARN_STREAM(
      "Cast ray returned false, which means that either max range or octree bound are reached or an unknown node was hit. Origin: "
        << origin << ", target: " << target);
    incStats(OCTOMAP_RAYCAST_FAILED_IDX);
    return false;
  }


  // check if ray cast end matches target point, i.e. if target point is visible
  double distance = ray_end.distance(target);
  if (distance <= threshold_)
  {
    // mark as visible
    addMarkerToMsg(OCTOMAP_CHECK_PASSED_IDX, start.position, end, colors::green());
    incStats(OCTOMAP_CHECK_PASSED_IDX);
    return true;
  }
  else
  {
    // mark as blocked
    addMarkerToMsg(OCTOMAP_RAYCAST_BLOCKED_IDX, start.position, end, colors::red());
    incStats(OCTOMAP_RAYCAST_BLOCKED_IDX);
    return false;
  }

  return true;
}


void OctomapChecker::printStats()
{
  if (!print_stats_)
  {
    return;
  }

  std::string stats = "Octomap Checker Statistics:\n";

  stats += "Octomap: check passed (true): " + std::to_string(stats_[OCTOMAP_CHECK_PASSED_IDX]) + "\n";
  stats += "\n";
  stats += "Octomap: raycast blocked (false): " + std::to_string(stats_[OCTOMAP_RAYCAST_BLOCKED_IDX]) + "\n";
  stats += "Octomap: raycast failed (false): " + std::to_string(stats_[OCTOMAP_RAYCAST_FAILED_IDX]) + "\n";
  stats += "\n";

  ROS_INFO_STREAM_NAMED(ROS_PACKAGE_NAME, stats);
}
} // end namespace three_dimensional_coverage_path_planning


PLUGINLIB_EXPORT_CLASS(three_dimensional_coverage_path_planning::OctomapChecker,
                       three_dimensional_coverage_path_planning::VisibilityCheckerBase
)