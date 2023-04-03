#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_GREEDY_WITHOUT_REDUNDANCIES_SELECTOR_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_GREEDY_WITHOUT_REDUNDANCIES_SELECTOR_H

#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/viewpoint_selector_base.h"


namespace three_dimensional_coverage_path_planning
{


class GreedyWithoutRedundanciesSelector : public ViewpointSelectorBase
{
public:

  GreedyWithoutRedundanciesSelector();

  void initialize(ros::NodeHandle& nh, std::vector<Viewpoint>& candidates,
                  const std::shared_ptr<const Point3dSet>& target_set) override;

  void selectViewpoints() override;

private:
  void removeRedundancies();

  int max_num_lost_covered_targets_;
};
} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_GREEDY_WITHOUT_REDUNDANCIES_SELECTOR_H
