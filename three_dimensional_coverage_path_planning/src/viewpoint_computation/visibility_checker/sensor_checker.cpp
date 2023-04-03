#include "three_dimensional_coverage_path_planning/viewpoint_computation/visibility_checker/sensor_checker.h"

#include "three_dimensional_coverage_path_planning/utils/transformation_helper.h"

#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace three_dimensional_coverage_path_planning
{


SensorChecker::SensorChecker()
{
  plugin_name_ = "SensorChecker";

  // get max index to use as vector size
  num_idx_ = 1 + std::max({SENSOR_CHECK_PASSED_IDX,
                           SENSOR_OUT_OF_MAX_RANGE_IDX, SENSOR_OUT_OF_MIN_RANGE_IDX,
                           SENSOR_OUT_OF_HORIZ_FOV_IDX, SENSOR_OUT_OF_VERT_FOV_IDX});
}


void SensorChecker::initialize(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model)
{
  VisibilityCheckerBase::initialize(pnh, model);

  if (publish_visibility_checks_)
  {
    // publisher and msg setup
    check_pubs_.resize(num_idx_);
    check_pubs_[SENSOR_CHECK_PASSED_IDX] = pnh_.advertise<visualization_msgs::Marker>("sensor_check_passed", 10, true);
    check_pubs_[SENSOR_OUT_OF_MAX_RANGE_IDX] = pnh_.advertise<visualization_msgs::Marker>("sensor_out_of_max_range", 10,
                                                                                          true);
    check_pubs_[SENSOR_OUT_OF_MIN_RANGE_IDX] = pnh_.advertise<visualization_msgs::Marker>("sensor_out_of_min_range", 10,
                                                                                          true);
    check_pubs_[SENSOR_OUT_OF_HORIZ_FOV_IDX] = pnh_.advertise<visualization_msgs::Marker>("sensor_out_of_horiz_fov", 10,
                                                                                          true);
    check_pubs_[SENSOR_OUT_OF_VERT_FOV_IDX] = pnh_.advertise<visualization_msgs::Marker>("sensor_out_of_vert_fov", 10,
                                                                                         true);
  }

  // get parameters
  max_range_ = pnh_.param<double>("max_range", 100.0);
  min_range_ = pnh_.param<double>("min_range", 0.0);

  utils::searchAndGetParam(pnh_, "sensor_frame", sensor_frame_);

  std::map<std::string, double> fov;
  utils::searchAndGetParam(pnh_, "field_of_view", fov);

  // get fov angles and convert to radian
  fov_horizontal_min_ = fov.at("horizontal_min") * M_PI / 180;
  fov_horizontal_max_ = fov.at("horizontal_max") * M_PI / 180;
  fov_vertical_min_ = fov.at("vertical_min") * M_PI / 180;
  fov_vertical_max_ = fov.at("vertical_max") * M_PI / 180;
}


bool SensorChecker::check(Pose3d& start, Point3d& end)
{
  // Note: sensor checker: probably have two ranges
  // 1. "good", exact range and
  // 2. "can see it but not in a great resolution length and only check this if no other view point was seen"?

  double distance = (start.position - end).norm();
  if (distance < min_range_)
  {
    addMarkerToMsg(SENSOR_OUT_OF_MIN_RANGE_IDX, start.position, end, colors::cyan());
    incStats(SENSOR_OUT_OF_MIN_RANGE_IDX);
    return false;
  }

  if (distance > max_range_)
  {
    addMarkerToMsg(SENSOR_OUT_OF_MAX_RANGE_IDX, start.position, end, colors::cyan());
    incStats(SENSOR_OUT_OF_MAX_RANGE_IDX);
    return false;
  }



  // check field_of_view

  Pose3d end_pose(end, start.frame_id);

  // compute end in sensor frame with sensor located at start position (= viewing direction from start to end)
  Pose3d viewing_direction_pose = TransformationHelper::transformFromModelToSensorFrame(start, end_pose, sensor_frame_);

  Point3d viewing_direction = viewing_direction_pose.position;
  

  // compute angles of direction with x-z plane (for horizontal fov) or x-y plane (for vertical fov)
  Point3d normal_x_z_plane(0, 1, 0);
  double angle_horizontal = utils::anglePlaneVector(normal_x_z_plane, viewing_direction);


  Point3d normal_x_y_plane(0, 0, 1);
  double angle_vertical = utils::anglePlaneVector(normal_x_y_plane, viewing_direction);


  // Angle is always the smaller angle to the plane, no matter if x is positive or negative.
  // This is only ok for negative x, if for the field of view of the other orientation (horizontal/vertical),
  // the minimum and maximum are outside the range [-90,90]. In this case also the mirrored field of view is allowed.
  // But otherwise, the supplementary angle to 180/-180 needs to be used
  // in order to get again the angle to the plane along the positive x-axis.
  // In this way, the angles are correctly outside the allowed area and the target is marked as not visible.
  if (viewing_direction.x() < 0)
  {
    if (fov_vertical_max_ < 0.5 * M_PI || fov_vertical_min_ > -0.5 * M_PI)
    {
      if (angle_horizontal > 0)
      {
        angle_horizontal = M_PI - angle_horizontal;
      }
      else
      {
        angle_horizontal = (-M_PI) - angle_horizontal;
      }
    }

    if (fov_horizontal_max_ < 0.5 * M_PI || fov_horizontal_min_ > -0.5 * M_PI)
    {
      if (angle_vertical > 0)
      {
        angle_vertical = M_PI - angle_vertical;
      }
      else
      {
        angle_vertical = (-M_PI) - angle_vertical;
      }
    }
  }

  // check angles
  if (!(fov_horizontal_min_ <= angle_horizontal && angle_horizontal <= fov_horizontal_max_))
  {
    addMarkerToMsg(SENSOR_OUT_OF_HORIZ_FOV_IDX, start.position, end, colors::violet());
    incStats(SENSOR_OUT_OF_HORIZ_FOV_IDX);
    return false;
  }

  if (!(fov_vertical_min_ <= angle_vertical && angle_vertical <= fov_vertical_max_))
  {
    addMarkerToMsg(SENSOR_OUT_OF_VERT_FOV_IDX, start.position, end, colors::orange());
    incStats(SENSOR_OUT_OF_VERT_FOV_IDX);
    return false;
  }




  // Note: maybe add other distance criteria: sensor range might not be enough
  // e.g. no ray in a room is greater than 20m, eventually check if distance is smaller than 20m as other
  // things cannot exist except in floors and here is it not wanted, etc..


  addMarkerToMsg(SENSOR_CHECK_PASSED_IDX, start.position, end, colors::green());
  incStats(SENSOR_CHECK_PASSED_IDX);
  return true;
}


void SensorChecker::printStats()
{
  if (!print_stats_)
  {
    return;
  }

  std::string stats = "Sensor Checker Statistics:\n";

  stats += "Sensor checker passed (true): " + std::to_string(stats_[SENSOR_CHECK_PASSED_IDX]) + "\n";
  stats += "\n";
  stats += "Sensor: out of max range (false): " + std::to_string(stats_[SENSOR_OUT_OF_MAX_RANGE_IDX]) + "\n";
  stats += "Sensor: out of min range (false): " + std::to_string(stats_[SENSOR_OUT_OF_MIN_RANGE_IDX]) + "\n";
  stats += "\n";
  stats +=
    "Sensor: out of horizontal field of view (false): " + std::to_string(stats_[SENSOR_OUT_OF_HORIZ_FOV_IDX]) + "\n";
  stats +=
    "Sensor: out of vertical field of view (false): " + std::to_string(stats_[SENSOR_OUT_OF_VERT_FOV_IDX]) + "\n";
  stats += "\n";

  ROS_INFO_STREAM_NAMED(ROS_PACKAGE_NAME, stats);
}
} // end namespace three_dimensional_coverage_path_planning


PLUGINLIB_EXPORT_CLASS(three_dimensional_coverage_path_planning::SensorChecker,
                       three_dimensional_coverage_path_planning::VisibilityCheckerBase)