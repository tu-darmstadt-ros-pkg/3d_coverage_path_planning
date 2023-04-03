#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/probabilistic_selector.h"


#include <pluginlib/class_list_macros.h>
#include <numeric>

namespace three_dimensional_coverage_path_planning
{


void ProbabilisticSelector::initialize(ros::NodeHandle& nh, std::vector<Viewpoint>& candidates,
                                       const std::shared_ptr<const Point3dSet>& target_set)
{
  ViewpointSelectorBase::initialize(nh, candidates, target_set);

  num_minimal_selected_viewpoints_ = all_candidates_.size();

  num_minimal_selection_uncovered_targets_ = all_target_points_.size();

  num_repetitions_ = pnh_.param("num_selections_repetitions", 25);

  // initialize random generator
  std::random_device rd;
  probability_gen_ = std::mt19937(rd());
  probability_dis_ = std::uniform_real_distribution<>(0.0, 1.0);
}


void ProbabilisticSelector::selectViewpoints()
{
  for (int i = 0; i < num_repetitions_; i++)
  {
    resetSelector();

    while (!isSelectionSufficient())
    {

      std::vector<double> probabilities;

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

      // compute sum_rewards of weights for probability computation
      double sum_rewards = std::accumulate(remaining_candidates_.begin(), remaining_candidates_.end(), 0.0,
                                           [](double sum, const Viewpoint& b)
                                           {
                                             return sum + b.getReward();
                                           });

      // compute probabilities
      for (auto& c: remaining_candidates_)
      {
        if (probabilities.empty())
        {
          probabilities.push_back(c.getReward() / sum_rewards);
        }
        else
        {
          // sum the probabilities as they are not really probabilities but limits that define the probability
          probabilities.push_back(probabilities.back() + c.getReward() / sum_rewards);
        }
      }


      // based on the rewards, probabilistically select an index
      double random_value = probability_dis_(probability_gen_);

      int idx = 0;
      for (; idx < probabilities.size(); ++idx)
      {
        if (random_value < probabilities[idx])
        {
          break;
        }
      }

      // select respective element
      selectViewpoint(remaining_candidates_[idx]);
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


PLUGINLIB_EXPORT_CLASS(three_dimensional_coverage_path_planning::ProbabilisticSelector,
                       three_dimensional_coverage_path_planning::ViewpointSelectorBase)