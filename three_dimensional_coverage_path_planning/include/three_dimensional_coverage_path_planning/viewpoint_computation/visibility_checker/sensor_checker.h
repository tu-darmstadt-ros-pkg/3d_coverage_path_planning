#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_OCTOMAP_CHECKER_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_OCTOMAP_CHECKER_H


#include "three_dimensional_coverage_path_planning/viewpoint_computation/visibility_checker/visibility_checker_base.h"

#include <geometry_msgs/TransformStamped.h>


namespace three_dimensional_coverage_path_planning
{

class SensorChecker : public VisibilityCheckerBase
{

public:
  SensorChecker();

  void initialize(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model) override;

  /**
   * Check if a point might be visible using the sensor specifications.
   * @param start viewpoint
   * @param end point, that should be seen
   * @return true if it might be visible according to sensor specifications, false otherwise
   */
  bool check(Pose3d& start, Point3d& end) override;


  // stats
  void printStats() override;


private:
  const int SENSOR_CHECK_PASSED_IDX = 0;
  const int SENSOR_OUT_OF_MAX_RANGE_IDX = 1;
  const int SENSOR_OUT_OF_MIN_RANGE_IDX = 2;
  const int SENSOR_OUT_OF_HORIZ_FOV_IDX = 3;
  const int SENSOR_OUT_OF_VERT_FOV_IDX = 4;

  double max_range_;
  double min_range_;


  std::string sensor_frame_;

  double fov_horizontal_min_;
  double fov_horizontal_max_;
  double fov_vertical_min_;
  double fov_vertical_max_;

};
} // end namespace three_dimensional_coverage_path_planning

#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_OCTOMAP_CHECKER_H
