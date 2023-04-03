#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_VIEWPOINT_COMPUTER_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_VIEWPOINT_COMPUTER_H

#include "three_dimensional_coverage_path_planning/viewpoint_computation/candidate_generator/candidate_generator_base.h"
#include "three_dimensional_coverage_path_planning/viewpoint_computation/reward_evaluator.h"
#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/viewpoint_selector_base.h"

#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint.h"
#include "three_dimensional_coverage_path_planning/model/model_data.h"

#include "three_dimensional_coverage_path_planning/utils/action_types.h"

#include <pluginlib/class_loader.h>

namespace three_dimensional_coverage_path_planning
{


class ViewpointComputer
{

public:

  ViewpointComputer(std::shared_ptr<const ModelData> base_model, std::shared_ptr<const Point3dSet> target_set,
                    PlanActionServer* const action_server = nullptr);

  std::shared_ptr<std::vector<Viewpoint>> computeViewpoints();


private:

  void generateCandidates();

  void rateCandidates();

  void selectViewpoints();


  pluginlib::ClassLoader<CandidateGeneratorBase> candidate_generator_loader_;
  std::unique_ptr<CandidateGeneratorBase> candidate_generator_;

  std::unique_ptr<RewardEvaluator> reward_evaluator_;

  pluginlib::ClassLoader<ViewpointSelectorBase> viewpoint_selector_loader_;
  std::unique_ptr<ViewpointSelectorBase> viewpoint_selector_;


  // data
  std::shared_ptr<const ModelData> base_model_;

  std::vector<Viewpoint> candidates_;

  std::shared_ptr<const Point3dSet> target_set_;


  // ros
  ros::NodeHandle pnh_;

  PlanActionServer* const action_server_;

  // visualization
  ros::Publisher all_candidates_pub_;
  ros::Publisher uncovered_target_points_pub_;
};
} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_VIEWPOINT_COMPUTER_H
