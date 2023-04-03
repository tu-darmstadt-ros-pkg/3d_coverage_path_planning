#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/greedy_selector.h"

#include <pluginlib/class_list_macros.h>

namespace three_dimensional_coverage_path_planning
{

void GreedySelector::selectViewpoints()
{
  while (!isSelectionSufficient())
  {
    // sort by reward
    std::sort(remaining_candidates_.begin(), remaining_candidates_.end(), ViewpointRewardComparator());

    // select best = last
    selectViewpoint(remaining_candidates_.back());

    // TEMPORARY sleep in selector
//    sleep(1);
  }
}
} // end namespace three_dimensional_coverage_path_planning


PLUGINLIB_EXPORT_CLASS(three_dimensional_coverage_path_planning::GreedySelector,
                       three_dimensional_coverage_path_planning::ViewpointSelectorBase)