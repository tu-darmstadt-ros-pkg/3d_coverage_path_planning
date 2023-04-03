#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_TRANSFORMATION_HELPER_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_TRANSFORMATION_HELPER_H


#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint.h"

#include <geometry_msgs/TransformStamped.h>


namespace three_dimensional_coverage_path_planning
{

class TransformationHelper
{
public:

  explicit TransformationHelper(ros::NodeHandle& nh);

  Pose3d transformViewpointToWaypoint(const Viewpoint& viewpoint);

  Viewpoint transformWaypointToViewpoint(const Pose3d& waypoint);


  // ===== static methods ===== //

  /**
   * Transform a pose from model frame in sensor frame, given by a sensor origin also in model frame.
   * @param sensor_origin Sensor origin in model frame
   * @param pose_in_model_frame pose in model frame that should be transformed
   * @param sensor_frame_id Sensor frame id
   * @return Pose in sensor frame
   */
  static Pose3d transformFromModelToSensorFrame(const Pose3d& sensor_origin, const Pose3d& pose_in_model_frame,
                                         std::string& sensor_frame_id);

  static geometry_msgs::TransformStamped getTransformation(const std::string& source_frame, const std::string& target_frame);

private:

  ros::NodeHandle nh_;

  std::string base_frame_;
  std::string sensor_frame_;

  geometry_msgs::TransformStamped transform_base_to_sensor_;
  geometry_msgs::TransformStamped transform_sensor_to_base_;
};
} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_TRANSFORMATION_HELPER_H
