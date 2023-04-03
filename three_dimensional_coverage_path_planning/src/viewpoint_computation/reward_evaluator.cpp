#include "three_dimensional_coverage_path_planning/viewpoint_computation/reward_evaluator.h"

#include "three_dimensional_coverage_path_planning/utils/utils_with_types.h"

namespace three_dimensional_coverage_path_planning
{
RewardEvaluator::RewardEvaluator(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> base_model,
                                 const std::shared_ptr<const Point3dSet>& target_set)
  : base_model_(std::move(base_model)),
    target_points_(target_set->begin(), target_set->end()),
    pnh_(pnh),
    visibility_checker_loader_("three_dimensional_coverage_path_planning",
                               "three_dimensional_coverage_path_planning::VisibilityCheckerBase")
{

  // init visibility checker
  std::vector<std::string> plugin_names;
  pnh_.getParam("visibility_checker_plugins", plugin_names);

  for (auto& name: plugin_names)
  {
    try
    {
      std::shared_ptr<VisibilityCheckerBase> tmp_ptr(visibility_checker_loader_.createUnmanagedInstance(name));
      tmp_ptr->initialize(pnh_, base_model_);
      visibility_checker_.push_back(tmp_ptr);
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("The visibility checker plugin \"" << name << "\" failed to load: " << ex.what());
    }
  }

#ifdef _OPENMP
  print_stats_ = false;
#else
  utils::searchAndGetParam(pnh_, "print_stats", false, print_stats_);
#endif

  bool publish_candidates;
  utils::searchAndGetParam(pnh_, "publish_all_candidates", false, publish_candidates);
  if (publish_candidates)
  {
    rated_candidates_pub_ = std::make_shared<ros::Publisher>(
      pnh_.advertise<visualization_msgs::Marker>("rated_candidates", 400, true));
  }
}

void RewardEvaluator::computeRewards(std::vector<Viewpoint>& candidates)
{

  // TODO check times in computeRewards with and without OPENMP
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  for (int i = 0; i < candidates.size(); ++i)
  {
    rateCandidate(candidates[i]);

    candidates[i].publishViewpoint(rated_candidates_pub_, 20 + i);

    // TEMPORARY sleep
//    sleep(1);

  }


  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  ROS_WARN_STREAM("computeRewards (for all candidates): elapsed time: "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]");
}

void RewardEvaluator::rateCandidate(Viewpoint& candidate)
{
  // error if there are different frames as then visibility etc. cannot be checked properly
  if (candidate.getPose().frame_id != base_model_->getModelFrame())
  {
    throw std::logic_error(
      "Candidate has different frame id than model! Model: " + base_model_->getModelFrame() + ", candidate: " +
      candidate.getPose().frame_id + ".");
  }

  candidate.setVisibleTargets(*findVisibleTargetPoints(candidate.getPose()));

  if (print_stats_)
  {
    ROS_INFO_STREAM("Statistics for candidate: " << utils::to_string(candidate.getPose().position));
    for (const auto& checker: visibility_checker_)
    {
      checker->printStats();
    }
  }
}


std::shared_ptr<Point3dSet> RewardEvaluator::findVisibleTargetPoints(Pose3d start_pose)
{

  // copy target set to have a vector that can be modified for each candidate (so that it later only contains the visible targets)
  Point3dVector targets_current_candidate = target_points_;

  // own scope for openmp, only for better readability
  {
#ifdef _OPENMP
#pragma omp parallel for shared (targets_current_candidate, start_pose)
#endif
    for (auto it = targets_current_candidate.begin();
         it != targets_current_candidate.end(); ++it) // NOLINT(modernize-loop-convert)
    {
      // if the target point is not visible from the start point, mark it as unseen by setting all values to float max value
      if (!isVisible(start_pose, *it))
      {
        it->setConstant(DBL_MAX);
      }
    }
  }

  for (const auto& checker: visibility_checker_)
  {
    // publish and reset msgs afterwards, so that for the next start point the old messages are not still in vector
    checker->publishAllMarkers();
    checker->resetAllMsgs();
  }

  // TEMPORARY print
  {
//    ROS_INFO_STREAM("Result size before erase: " << targets_current_candidate.size());
  }

  // remove all invisible points
  targets_current_candidate.erase(std::remove_if(targets_current_candidate.begin(), targets_current_candidate.end(),
                                                 [](Point3d& point)
                                                 {
                                                   return point.isConstant(DBL_MAX);
                                                 }),
                                  targets_current_candidate.end());

  // TEMPORARY print
  {
    ROS_INFO_STREAM("Result size after erase: " << targets_current_candidate.size());
  }

  std::shared_ptr<Point3dSet> result = std::make_shared<Point3dSet>(targets_current_candidate.begin(),
                                                                    targets_current_candidate.end());

  return result;
}


bool RewardEvaluator::isVisible(Pose3d& start_pose, Point3d& target_point)
{
  // check for each defined visibility checker, if target_position might be seen from start_position
  for (auto& checker: visibility_checker_)
  {
    if (!checker->check(start_pose, target_point))
    {
      return false;
    }
  }
  return true;
}
} // end namespace three_dimensional_coverage_path_planning