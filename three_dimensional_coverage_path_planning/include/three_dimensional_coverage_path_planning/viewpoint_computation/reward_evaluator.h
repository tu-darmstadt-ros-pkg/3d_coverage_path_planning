#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_REWARD_EVALUATOR_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_REWARD_EVALUATOR_H

#include "three_dimensional_coverage_path_planning/model/model_data.h"
#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint.h"

#include "three_dimensional_coverage_path_planning/viewpoint_computation/visibility_checker/visibility_checker_base.h"

#include <pluginlib/class_loader.h>

namespace three_dimensional_coverage_path_planning
{


class RewardEvaluator
{
public:
  /**
   *
   * @param base_model Base model on which the candidates will be rated.
   * @param target_set Target that should be covered.
   */
  RewardEvaluator(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> base_model,
                  const std::shared_ptr<const Point3dSet>& target_set);

  /**
   * Compute reward for each candidate in list.
   * @param [in, out] candidates
   */
  void computeRewards(std::vector<Viewpoint>& candidates);


private:
  /**
   * Compute reward of a single candidate using base_model
   * @param [in, out] candidate
   */
  void rateCandidate(Viewpoint& candidate);


  /**
   * Find all visible target points for the given start pose.
   * @param start_pose
   * @return
   */
  std::shared_ptr<Point3dSet> findVisibleTargetPoints(Pose3d start_pose);

  /**
   * Check if target_point is visible from start_pose.
   * @param start_pose
   * @param target_point
   * @return true if visible, false otherwise
   */
  bool isVisible(Pose3d& start_pose, Point3d& target_point);


  std::shared_ptr<const ModelData> base_model_;

  // vector here instead of a set in order to allow the parallel execution later
  // to modify the elements and erase them afterwards instead of immediately erasing them
  const Point3dVector target_points_; ///vector containing all target points

  ros::NodeHandle pnh_;

  // visibility checker
  pluginlib::ClassLoader<VisibilityCheckerBase> visibility_checker_loader_;
  std::vector<std::shared_ptr<VisibilityCheckerBase>> visibility_checker_;

  bool print_stats_ = false;

  std::shared_ptr<ros::Publisher> rated_candidates_pub_;
};
} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_REWARD_EVALUATOR_H
