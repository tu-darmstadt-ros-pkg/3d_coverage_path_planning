#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_PROBABILISTIC_EXT_DIST_SELECTOR_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_PROBABILISTIC_EXT_DIST_SELECTOR_H

#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/viewpoint_selector_base.h"

#include <random>

namespace three_dimensional_coverage_path_planning
{

class ProbabilisticExpDistSelector : public ViewpointSelectorBase
{
public:

  void initialize(ros::NodeHandle& nh, std::vector<Viewpoint>& candidates,
                  const std::shared_ptr<const Point3dSet>& target_set) override;

  void selectViewpoints() override;


private:
  int num_repetitions_;

  unsigned long num_minimal_selected_viewpoints_;
  unsigned long num_minimal_selection_uncovered_targets_;

  std::shared_ptr<std::vector<Viewpoint>> minimal_selection_selected_viewpoints_;
  Point3dSet minimal_selection_uncovered_target_points_;

  std::mt19937 probability_gen_;
  std::exponential_distribution<> exp_probability_dist_;
};
} // end namespace three_dimensional_coverage_path_planning

#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_PROBABILISTIC_EXT_DIST_SELECTOR_H
