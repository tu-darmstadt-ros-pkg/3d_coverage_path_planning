#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_OCTOMAP_CHECKER_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_OCTOMAP_CHECKER_H


#include "three_dimensional_coverage_path_planning/viewpoint_computation/visibility_checker/visibility_checker_base.h"

namespace three_dimensional_coverage_path_planning
{

class OctomapChecker : public VisibilityCheckerBase
{

public:
  OctomapChecker();

  void initialize(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model) override;

  /**
 * Check if a point might be visible using the octomap from the model.
 * @param start viewpoint
 * @param end point, that should be seen
 * @return true if it is visible according to octomap (last check: raycasting), false otherwise
 */
  bool check(Pose3d& start, Point3d& end) override;


  // stats
  void printStats() override;


private:
  const int OCTOMAP_CHECK_PASSED_IDX = 0;
  const int OCTOMAP_RAYCAST_BLOCKED_IDX = 1;
  const int OCTOMAP_RAYCAST_FAILED_IDX = 2;

  double tolerance_;
  double threshold_; /// threshold for visibility check, if distance between ray cast end and target is <= threshold, target is visible
};
} // end namespace three_dimensional_coverage_path_planning

#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_OCTOMAP_CHECKER_H
