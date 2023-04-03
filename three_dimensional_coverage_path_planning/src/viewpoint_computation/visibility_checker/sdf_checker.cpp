
#include "three_dimensional_coverage_path_planning/viewpoint_computation/visibility_checker/sdf_checker.h"

#include <pluginlib/class_list_macros.h>

namespace three_dimensional_coverage_path_planning
{

SdfChecker::SdfChecker()
{
  plugin_name_ = "SdfChecker";

  // get max index to use as vector size
  num_idx_ = 1 + std::max({SDF_START_DISTANCE_SMALLER, SDF_END_DISTANCE_SMALLER, SDF_START_GRADIENT_IDX,
                           SDF_END_GRADIENT_IDX, SDF_GRADIENT_CHECK_PASSED});
}


void SdfChecker::initialize(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model)
{
  VisibilityCheckerBase::initialize(pnh, model);

  if (publish_visibility_checks_)
  {
    // publisher and msg setup
    check_pubs_.resize(num_idx_);
    check_pubs_[SDF_START_DISTANCE_SMALLER] = pnh_.advertise<visualization_msgs::Marker>("sdf_start_distance_smaller",
                                                                                         10, true);
    check_pubs_[SDF_END_DISTANCE_SMALLER] = pnh_.advertise<visualization_msgs::Marker>("sdf_end_distance_smaller", 10,
                                                                                       true);
    check_pubs_[SDF_START_GRADIENT_IDX] = pnh_.advertise<visualization_msgs::Marker>("sdf_start_gradient", 10, true);
    check_pubs_[SDF_END_GRADIENT_IDX] = pnh_.advertise<visualization_msgs::Marker>("sdf_end_gradient", 10, true);
    check_pubs_[SDF_GRADIENT_CHECK_PASSED] = pnh_.advertise<visualization_msgs::Marker>("sdf_gradient_check_passed", 10,
                                                                                        true);
  }
}


bool SdfChecker::check(Pose3d& start, Point3d& end)
{
  if (!model_)
  {
    throw std::runtime_error("No model set in " + getPluginName() + "!");
  }

  // check if model has an sdf
  std::shared_ptr<const SdfMap> sdf = model_->getSdf();
  if (!sdf)
  {
    ROS_WARN_STREAM_THROTTLE(5, "No sdf in model with name " << model_->getModelName()
                                                             << ", so no sdf check is performed.");
    return false;
  }


  // get distances and gradients at both points
  double start_distance, end_distance;
  Eigen::Vector3d start_gradient, end_gradient;

#ifdef _OPENMP
#pragma omp critical
#endif
  {
    try
    {
      // try to get values from map for start position
      auto res = start_distances_gradients.at(start.position);
      start_distance = res.first;
      start_gradient = res.second;
    }
    catch (std::out_of_range&)
    {
      start_distance = sdf->getDistanceAtPosition(start.position);
      sdf->getGradientAtPosition(start.position, start_gradient);

      start_distances_gradients[start.position] = std::make_pair(start_distance, start_gradient);
    }

    try
    {
      // try to get values from map for end position
      auto res = end_distances_gradients.at(end);
      end_distance = res.first;
      end_gradient = res.second;
    }
    catch (std::out_of_range&)
    {
      end_distance = sdf->getDistanceAtPosition(end);

      sdf->getGradientAtPosition(end, end_gradient, false);

      end_distances_gradients[end] = std::make_pair(end_distance, end_gradient);
    }
  }

  // get directions between the two points for starting from both
  Eigen::Vector3d start_to_end = end - start.position;
  Eigen::Vector3d end_to_start = start.position - end;

  // round in order to get rid of rounding errors in the calculation and to add a few tolerances
  int num_decimal_places = 4;
  utils::round(start_gradient, num_decimal_places);
  utils::round(end_gradient, num_decimal_places);

  // check if distance to next obstacle is greater than largest absolute value of distance between start and end along one axis,
  // as then no obstacle can be met
  double start_end_distance = start_to_end.norm();
  if (start_end_distance <= start_distance)
  {
    addMarkerToMsg(SDF_START_DISTANCE_SMALLER, start.position, end, colors::green(), colors::yellow());
    incStats(SDF_START_DISTANCE_SMALLER);
    return true;
  }
  if (start_end_distance <= end_distance)
  {
    addMarkerToMsg(SDF_END_DISTANCE_SMALLER, start.position, end, colors::green(), colors::yellow());
    incStats(SDF_END_DISTANCE_SMALLER);
    return true;
  }


  for (int i = 0; i < 3; i++)
  {
    if (!((end_to_start[i] >= 0 && end_gradient[i] >= 0) || (end_to_start[i] <= 0 && end_gradient[i] <= 0)))
    {
      addMarkerToMsg(SDF_END_GRADIENT_IDX, start.position, end, colors::red());
      incStats(SDF_END_GRADIENT_IDX);
      return false;
    }

    // Note: i think the check is unnecessary because only a few, if any, candidates are close to walls. It results in more problems if used.
//    // check if at position i in gradient and direction is the same sign (equality to zero is allowed),
//    // otherwise it would reach a wall
//    if (!((start_to_end[i] >= 0 && start_gradient[i] >= 0) || (start_to_end[i] <= 0 && start_gradient[i] <= 0)))
//    {
//      addMarkerToMsg(SDF_START_GRADIENT_IDX, start.position, end, colors::blue());
//      incStats(SDF_START_GRADIENT_IDX);
//      return false;
//    }


  }

  // if every check was passed, return true
  addMarkerToMsg(SDF_GRADIENT_CHECK_PASSED, start.position, end, colors::green());
  incStats(SDF_GRADIENT_CHECK_PASSED);
  return true;
}


void SdfChecker::printStats()
{
  if (!print_stats_)
  {
    return;
  }

  std::string stats = "Sdf Checker Statistics:\n";

  stats += "\n";
  stats += "SDF: gradient check passed (true): " + std::to_string(stats_[SDF_GRADIENT_CHECK_PASSED]) + "\n";
  stats += "SDF: start distance (true): " + std::to_string(stats_[SDF_START_DISTANCE_SMALLER]) + "\n";
  stats += "SDF: end distance (true): " + std::to_string(stats_[SDF_END_DISTANCE_SMALLER]) + "\n";
  stats += "\n";
  stats += "SDF: start gradient (false): " + std::to_string(stats_[SDF_START_GRADIENT_IDX]) + "\n";
  stats += "SDF: end gradient (false): " + std::to_string(stats_[SDF_END_GRADIENT_IDX]) + "\n";
  stats += "\n";

  ROS_INFO_STREAM_NAMED(ROS_PACKAGE_NAME, stats);
}
} // end namespace three_dimensional_coverage_path_planning


PLUGINLIB_EXPORT_CLASS(three_dimensional_coverage_path_planning::SdfChecker,
                       three_dimensional_coverage_path_planning::VisibilityCheckerBase)