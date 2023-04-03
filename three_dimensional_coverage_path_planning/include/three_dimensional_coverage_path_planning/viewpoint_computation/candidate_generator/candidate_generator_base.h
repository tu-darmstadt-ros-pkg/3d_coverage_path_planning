#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_CANDIDATE_GENERATOR_BASE_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_CANDIDATE_GENERATOR_BASE_H

#include "three_dimensional_coverage_path_planning/model/model_data.h"
#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint.h"


namespace three_dimensional_coverage_path_planning
{

class CandidateGeneratorBase
{
public:
  CandidateGeneratorBase() = default;

  virtual void initialize(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model);

  std::vector<Viewpoint> generateCandidates();

protected:

  /**
   * Generate candidate positions and store them in candidate_viewpoints_.
   */
  virtual void generateCandidatePositions() = 0;

  /**
   * Generate candidate orientations for the candidate positions stored already in candidate_viewpoints_.
   */
  virtual void generateCandidateOrientations();


  ros::NodeHandle pnh_;

  std::shared_ptr<const ModelData> model_;

  std::vector<Viewpoint> candidate_viewpoints_;
};
} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_CANDIDATE_GENERATOR_BASE_H
