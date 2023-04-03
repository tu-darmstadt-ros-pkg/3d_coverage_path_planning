#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/greedy_without_redundancies_selector.h"

#include "three_dimensional_coverage_path_planning/utils/utils_with_types.h"

#include <pluginlib/class_list_macros.h>

namespace three_dimensional_coverage_path_planning
{

GreedyWithoutRedundanciesSelector::GreedyWithoutRedundanciesSelector()
{
  // in this selector the mapping covered target point --> viewpoints that cover this point is required, so enable storing of these data
  store_map_target_to_viewpoints_ = true;
}

void GreedyWithoutRedundanciesSelector::initialize(ros::NodeHandle& nh, std::vector<Viewpoint>& candidates,
                                                   const std::shared_ptr<const Point3dSet>& target_set)
{
  ViewpointSelectorBase::initialize(nh, candidates, target_set);

  max_num_lost_covered_targets_ = pnh_.param("max_num_lost_covered_targets", 50);
}


void GreedyWithoutRedundanciesSelector::selectViewpoints()
{
  while (!isSelectionSufficient())
  {
    // sort by reward
    std::sort(remaining_candidates_.begin(), remaining_candidates_.end(), ViewpointRewardComparator());

    // select best = last
    selectViewpoint(remaining_candidates_.back());

    // backtracking step to remove redundancies
    removeRedundancies();
  }
}

void GreedyWithoutRedundanciesSelector::removeRedundancies()
{
  for (auto& viewpoint: *selected_viewpoints_)
  {
    int lost_covered_targets_cntr = 0;

    // check if viewpoint is redundant and can be deleted (only check targets seen from this viewpoint as the others are definitely covered by other viewpoints
    for (auto& target: viewpoint.getTargetsSeenFromThisViewpoint())
    {
      // if the respective target point is only covered by the current viewpoint, the "lost covered target points" counter is incremented
      if (map_covered_targets_to_viewpoints_.at(target).size() == 1)
      {
        ++lost_covered_targets_cntr;

        // if too many targets would be uncovered (="lost") if the current viewpoint would be removed from selection,
        // break and check next viewpoint
        if (lost_covered_targets_cntr > max_num_lost_covered_targets_)
        {
          break;
        }
      }
    }

    // if only few target points would be uncovered (="lost") if the current viewpoint would be removed from selection,
    // it is counted as a redundancy and removed from selection
    if (lost_covered_targets_cntr <= max_num_lost_covered_targets_)
    {

      for (auto& target: viewpoint.getAllVisibleTargets())
      {
        std::vector<Viewpoint>& viewpoints = map_covered_targets_to_viewpoints_[target];


        // remove viewpoint from mapping
        viewpoints.erase(std::remove(viewpoints.begin(), viewpoints.end(), viewpoint), viewpoints.end());

        // if this target was lost:
        if (viewpoints.empty())
        {
          // remove it from mapping
          map_covered_targets_to_viewpoints_.erase(target);

          // add it to uncovered list
          uncovered_target_points_.insert(target);

          // add it to the seen targets for all remaining viewpoints that can see the target
          for (auto& rc: remaining_candidates_)
          {
            if (rc.getAllVisibleTargets().find(target) != rc.getAllVisibleTargets().end())
            {
              rc.markTargetAsSeenByThisViewpoint(target);
            }
          }
        }
          // if target is not lost by removing viewpoint (= there are other viewpoints that see the target)
        else
        {
          // check if it was marked as seen by viewpoint and if so, add it to first of other viewpoints (but in selected_viewpoints list!!
          if (viewpoint.getTargetsSeenFromThisViewpoint().find(target) !=
              viewpoint.getTargetsSeenFromThisViewpoint().end())
          {
            auto first_of_other_viewpoints = std::find(selected_viewpoints_->begin(), selected_viewpoints_->end(),
                                                       *viewpoints.begin());

            if (first_of_other_viewpoints == selected_viewpoints_->end())
            {
              throw std::runtime_error(
                "Mapping (target -> viewpoints from which target can be seen) contains not selected viewpoints!");
            }

            first_of_other_viewpoints->markTargetAsSeenByThisViewpoint(target);
          }
        }
      }

      ROS_INFO_STREAM_NAMED(ROS_PACKAGE_NAME,
                            "Removed redundant viewpoint at position " << utils::to_string(viewpoint.getPose().position)
                                                                       << ".");

      // mark viewpoint to be deleted (cannot be deleted immediately, as the selected_viewpoints list is used for iterator)
      viewpoint = Viewpoint(Pose3d("delete"));

      // Note: The removed is not added to remaining candidates, as if it is removed once as redundant it will not be added again later.
    }
  }


  // remove all viewpoints marked as deleted from selection
  selected_viewpoints_->erase(std::remove_if(selected_viewpoints_->begin(), selected_viewpoints_->end(),
                                             [](Viewpoint& vp) { return (vp.getPose().frame_id == "delete"); }),
                              selected_viewpoints_->end());
}
} // end namespace three_dimensional_coverage_path_planning

PLUGINLIB_EXPORT_CLASS(three_dimensional_coverage_path_planning::GreedyWithoutRedundanciesSelector,
                       three_dimensional_coverage_path_planning::ViewpointSelectorBase)