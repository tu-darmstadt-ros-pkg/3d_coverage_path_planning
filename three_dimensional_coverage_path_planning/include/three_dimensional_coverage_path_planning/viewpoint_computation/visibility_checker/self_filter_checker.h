#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_SELF_FILTER_CHECKER_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_SELF_FILTER_CHECKER_H


#include "three_dimensional_coverage_path_planning/viewpoint_computation/visibility_checker/visibility_checker_base.h"


namespace three_dimensional_coverage_path_planning
{

class SelfFilterChecker : public VisibilityCheckerBase
{

public:
  SelfFilterChecker();

  void initialize(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model) override;

  /**
   * Check if a point might be visible using the robot body filter.
   * @param start viewpoint
   * @param end point, that should be seen
   * @return true if it might be visible according to robot body filter, false otherwise
   */
  bool check(Pose3d& start, Point3d& end) override;


  // stats
  void printStats() override;


private:

  /**
   * Generate points on a unit sphere. Use spherical fibonacci point sets / grids to generate the points evenly distributed.
   * (See "Spherical Fibonacci Mapping" by B. Keinert et al. (https://dl.acm.org/doi/abs/10.1145/2816795.2818131))
   * @param num_points number of points to generate
   */
  void generatePointsOnSphericalMask(int num_points);

  /**
   * Apply the robot body filter on mask with sensor placed at origin.
   * Assign each point in mask intensity 0 or 1 with 0 = blocked by self filter, 1 = visible.
   */
  void applySelfFilterOnMask();

  /**
   * Check if a point in a given direction (= normalized point in sensor frame) is visible according to self filter.
   * (See "Spherical Fibonacci Mapping" by B. Keinert et al. (https://dl.acm.org/doi/abs/10.1145/2816795.2818131))
   * @param direction_to_check
   * @return true if point in this direction can be seen according to mask
   */
  bool checkDirectionOnMask(Point3d& direction_to_check);

  /**
   * Compute the point at index i of the fibonacci point set with num_points points.
   * (See "Spherical Fibonacci Mapping" by B. Keinert et al. (https://dl.acm.org/doi/abs/10.1145/2816795.2818131))
   * @param i index of point
   * @param num_points
   * @return
   */
  Eigen::Vector3d computeFibPoint(int i, int num_points) const;


  pcl::PointCloud<pcl::PointXYZI> self_filter_mask_; /// mask containing points on a sphere with intensity = 0 if points are filtered by self filter (= are blocked) and 1 if they are not blocked (= can be seen).

  std::string sensor_frame_;

  /**
   * Golden ratio = PHI, a here often used property of the golden ratio is:
   * PHI^(-1) = PHI - 1
   * hence also the following is equal:
   * x / PHI = x * PHI^(-1) = x * (PHI-1)
   */
  const double GOLDEN_RATIO = (1.0 + std::sqrt(5.0)) / 2.0;


  const int SELF_FILTER_CHECK_PASSED_IDX = 0;
  const int SELF_FILTER_BLOCKED_IDX = 1;


  ros::Publisher mask_pub_;
};
} // end namespace three_dimensional_coverage_path_planning



#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_SELF_FILTER_CHECKER_H
