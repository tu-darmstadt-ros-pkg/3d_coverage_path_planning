#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/probabilistic_exp_dist_selector.h"

#include <pluginlib/class_list_macros.h>


namespace three_dimensional_coverage_path_planning
{


void ProbabilisticExpDistSelector::initialize(ros::NodeHandle& nh, std::vector<Viewpoint>& candidates,
                                              const std::shared_ptr<const Point3dSet>& target_set)
{
  ViewpointSelectorBase::initialize(nh, candidates, target_set);

  num_minimal_selected_viewpoints_ = all_candidates_.size();

  num_minimal_selection_uncovered_targets_ = all_target_points_.size();

  num_repetitions_ = pnh_.param("num_selections_repetitions", 25);

  // initialize random generator
  std::random_device rd;
  probability_gen_ = std::mt19937(rd());

  // Mean of the exponential distribution is 1/lambda.
  // --> With lambda = 0.5 the mean value is 2.
  // --> The values at index 0, 1 and 2 and equally often chosen as all values > 2 (simplified!).
  double lambda = 0.8; // TODO maybe get lambda also from parameter server?

  exp_probability_dist_ = std::exponential_distribution<>(lambda);
}


void ProbabilisticExpDistSelector::selectViewpoints()
{
  for (int i = 0; i < num_repetitions_; i++)
  {
    resetSelector();

    while (!isSelectionSufficient())
    {

      // sort by reward
      std::sort(remaining_candidates_.begin(), remaining_candidates_.end(), ViewpointRewardComparator());

      // remove the candidates with reward < min_reward
      if (remaining_candidates_[0].getReward() < min_reward_)
      {
        // find last element that has reward < min_reward
        auto end_it = remaining_candidates_.begin();
        for (; end_it != remaining_candidates_.end(); ++end_it)
        {
          if (end_it->getReward() >= min_reward_)
          {
            break;
          }
        }
        remaining_candidates_.erase(remaining_candidates_.begin(), end_it);
      }



      // probabilistically select an index in range [0,remaining_candidates_.size-1] based on an exponential distribution
      int random_idx = INT_MAX;
      while (random_idx < 0 || random_idx > (remaining_candidates_.size() - 1))
      {
        random_idx = static_cast<int>(std::round(exp_probability_dist_(probability_gen_)));
      }


      // select the respective element (as the element with the largest reward is last, select the random_idx-th element from the back)
      selectViewpoint(remaining_candidates_[(remaining_candidates_.size() - 1) - random_idx]);
    }

    // TEMPORARY print
    ROS_INFO_STREAM("Selected viewpoints: " << selected_viewpoints_->size() << ", uncovered targets: "
                                            << uncovered_target_points_.size());


    // if less selected viewpoints or same amount of selected viewpoints but with less uncovered target points
    if (selected_viewpoints_->size() < num_minimal_selected_viewpoints_ ||
        (selected_viewpoints_->size() == num_minimal_selected_viewpoints_ &&
         uncovered_target_points_.size() < num_minimal_selection_uncovered_targets_)
      )
    {
      // accept as current best solution
      minimal_selection_selected_viewpoints_ = selected_viewpoints_;
      minimal_selection_uncovered_target_points_ = uncovered_target_points_;

      num_minimal_selected_viewpoints_ = selected_viewpoints_->size();
      num_minimal_selection_uncovered_targets_ = uncovered_target_points_.size();

      // TEMPORARY print
      ROS_WARN_STREAM("Size of current smallest set of selected viewpoints: " << num_minimal_selected_viewpoints_);
      ROS_INFO_STREAM("Size of uncovered target set for smallest set of selected viewpoints: "
                        << num_minimal_selection_uncovered_targets_);
    }
  }

  selected_viewpoints_ = minimal_selection_selected_viewpoints_;
  uncovered_target_points_ = minimal_selection_uncovered_target_points_;
}
} // end namespace three_dimensional_coverage_path_planning


PLUGINLIB_EXPORT_CLASS(three_dimensional_coverage_path_planning::ProbabilisticExpDistSelector,
                       three_dimensional_coverage_path_planning::ViewpointSelectorBase)