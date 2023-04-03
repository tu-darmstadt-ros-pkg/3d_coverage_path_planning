#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_GREEDY_SELECTOR_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_GREEDY_SELECTOR_H


#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/viewpoint_selector_base.h"


namespace three_dimensional_coverage_path_planning
{

class GreedySelector : public ViewpointSelectorBase
{
public:
  void selectViewpoints() override;
};
} // end namespace three_dimensional_coverage_path_planning

#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_GREEDY_SELECTOR_H
