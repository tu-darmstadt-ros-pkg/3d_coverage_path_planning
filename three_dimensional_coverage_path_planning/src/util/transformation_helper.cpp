
#include "three_dimensional_coverage_path_planning/utils/transformation_helper.h"
#include "three_dimensional_coverage_path_planning/utils/utils_with_types.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace three_dimensional_coverage_path_planning
{

TransformationHelper::TransformationHelper(ros::NodeHandle& nh) : nh_(nh)
{
  utils::searchAndGetParam(nh_, "base_frame", base_frame_);
  utils::searchAndGetParam(nh_, "sensor_frame", sensor_frame_);
}

Pose3d TransformationHelper::transformViewpointToWaypoint(const Viewpoint& viewpoint)
{
  // if transform has not been stored yet, look it up
  if (transform_base_to_sensor_.header.frame_id.empty() || transform_base_to_sensor_.child_frame_id.empty())
  {
    // get transform from base frame to sensor: ^sensor T _base
    transform_base_to_sensor_ = getTransformation(base_frame_, sensor_frame_);
  }


  // get transform from sensor to model (assume viewpoint is at sensor position):
  // (^model T _sensor) * (^sensor Origin) = ^model (sensor_origin)
  // --> solve for (^model T _sensor) with (^sensor Origin) = Identity. Result is (^model (sensor_origin)) = Viewpoint

  // ^model T _sensor
  geometry_msgs::TransformStamped transform_sensor_to_model = utils::toMsgTransformStamped(viewpoint.getPose(),
                                                                                           sensor_frame_);


  // make base frame origin pose
  Pose3d base_frame_origin(base_frame_);
  geometry_msgs::PoseStamped base_in_base_frame = utils::toMsgPoseStamped(base_frame_origin);

  // convert: Base frame origin --> base in sensor frame --> base in model frame = waypoint
  geometry_msgs::PoseStamped base_in_sensor_frame, base_in_model_frame;
  tf2::doTransform(base_in_base_frame, base_in_sensor_frame, transform_base_to_sensor_);
  tf2::doTransform(base_in_sensor_frame, base_in_model_frame, transform_sensor_to_model);

  // base in model frame is waypoint in model frame
  Pose3d waypoint = utils::toPose3d(base_in_model_frame);

  return waypoint;
}

Viewpoint TransformationHelper::transformWaypointToViewpoint(const Pose3d& waypoint)
{
  // if transform has not been stored yet, look it up
  if (transform_sensor_to_base_.header.frame_id.empty() || transform_sensor_to_base_.child_frame_id.empty())
  {
    // get transform from sensor frame to base: ^base T _sensor
    transform_sensor_to_base_ = getTransformation(sensor_frame_, base_frame_);
  }


  // get transform from base to model (assume waypoint is at base link position):
  // (^model T _base) * (^base Origin) = ^model (base_origin)
  // --> solve for (^model T _base) with (^base Origin) = Identity. Result is (^model (base_origin)) = waypoint

  // ^model T _base
  geometry_msgs::TransformStamped transform_base_to_model = utils::toMsgTransformStamped(waypoint,
                                                                                         base_frame_);

  // make sensor frame origin pose
  Pose3d sensor_frame_origin(sensor_frame_);
  geometry_msgs::PoseStamped sensor_in_sensor_frame = utils::toMsgPoseStamped(sensor_frame_origin);

  // convert: sensor frame origin --> sensor in base frame --> sensor in model frame = viewpoint
  geometry_msgs::PoseStamped sensor_in_base_frame, sensor_in_model_frame;

  tf2::doTransform(sensor_in_sensor_frame, sensor_in_base_frame, transform_sensor_to_base_);
  tf2::doTransform(sensor_in_base_frame, sensor_in_model_frame, transform_base_to_model);


  // sensor in model frame is viewpoint in model frame
  Viewpoint viewpoint(utils::toPose3d(sensor_in_model_frame));

  return viewpoint;
}


Pose3d
TransformationHelper::transformFromModelToSensorFrame(const Pose3d& sensor_origin, const Pose3d& pose_in_model_frame,
                                                      std::string& sensor_frame_id)
{

  geometry_msgs::PoseStamped pose_in_model_frame_msg = utils::toMsgPoseStamped(pose_in_model_frame);

  // get transformation from model sensor frame: ^sensor T _model * ^model(sensor_origin) = ^sensor(sensor_origin) = Identity
  // --> ^sensor T _model = inverse( ^model(sensor_origin) )

  // convert sensor_origin to tf2::Transform
  tf2::Quaternion quaternion(sensor_origin.orientation.x(), sensor_origin.orientation.y(),
                             sensor_origin.orientation.z(),
                             sensor_origin.orientation.w());

  tf2::Vector3 position(sensor_origin.position.x(), sensor_origin.position.y(), sensor_origin.position.z());

  tf2::Transform sensor_origin_in_model_frame(quaternion, position);


  // invert ^model(sensor_origin)
  tf2::Stamped<tf2::Transform> transform_model_to_sensor = tf2::Stamped<tf2::Transform>(
    sensor_origin_in_model_frame.inverse(), ros::Time(), sensor_frame_id);

  // transform given pose
  geometry_msgs::PoseStamped pose_in_sensor_frame_msg;
  tf2::doTransform(pose_in_model_frame_msg, pose_in_sensor_frame_msg, toMsg(transform_model_to_sensor));


  return utils::toPose3d(pose_in_sensor_frame_msg);
}

geometry_msgs::TransformStamped
TransformationHelper::getTransformation(const std::string& source_frame, const std::string& target_frame)
{
  geometry_msgs::TransformStamped result;
  try
  {
    // prepare tf listener
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    auto transform_timeout = ros::Duration(3.0);

    // get transform from sensor frame to fov frame: ^fov T _sensor
    result = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0), transform_timeout);
  } catch (tf2::TransformException& ex)
  {
    throw std::runtime_error(
      "Could not get transform from " + source_frame + " to " + target_frame + ": " + ex.what());
  }

  return result;
}
} // end namespace three_dimensional_coverage_path_planning