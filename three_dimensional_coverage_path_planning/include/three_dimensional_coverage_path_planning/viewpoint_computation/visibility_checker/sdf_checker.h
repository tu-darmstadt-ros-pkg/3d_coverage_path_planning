#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_SDF_CHECKER_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_SDF_CHECKER_H

#include "three_dimensional_coverage_path_planning/viewpoint_computation/visibility_checker/visibility_checker_base.h"


namespace three_dimensional_coverage_path_planning
{
class SdfChecker : public VisibilityCheckerBase
{

public:
  SdfChecker();

  void initialize(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model) override;

  /**
   * Check if a point might be visible using the sdf from the model.
   * @param start viewpoint
   * @param end point, that should be seen
   * @return true if it might be visible according to sdf, false otherwise
   */
  bool check(Pose3d& start, Point3d& end) override;

  // stats
  void printStats() override;

private:

  // each candidate will be checked with (nearly) each target point, so create a map in order to only compute gradients once
  std::map<Point3d, std::pair<double, Eigen::Vector3d>, Point3dLessComparator> end_distances_gradients; /// map to save the computed/retrieved distances and gradients of target points
  std::map<Point3d, std::pair<double, Eigen::Vector3d>, Point3dLessComparator> start_distances_gradients; /// map to save the computed/retrieved distances and gradients of candidate points


  const int SDF_GRADIENT_CHECK_PASSED = 0;
  const int SDF_START_GRADIENT_IDX = 1;
  const int SDF_END_GRADIENT_IDX = 2;
  const int SDF_START_DISTANCE_SMALLER = 3;
  const int SDF_END_DISTANCE_SMALLER = 4;
};
} // end namespace three_dimensional_coverage_path_planning

#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_SDF_CHECKER_H
