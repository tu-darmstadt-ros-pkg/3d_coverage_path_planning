#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_UTILS_WITH_TYPES_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_UTILS_WITH_TYPES_H

#include "three_dimensional_coverage_path_planning/utils/types.h"
#include "three_dimensional_coverage_path_planning/utils/utils.h"

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


namespace three_dimensional_coverage_path_planning
{

namespace utils
{


// === pose conversions ===
static geometry_msgs::PoseStamped toMsgPoseStamped(const Pose3d& input)
{
  geometry_msgs::PoseStamped output;
  output.header.frame_id = input.frame_id;
  output.pose.position = toMsgPoint(input.position);
  output.pose.orientation = toMsgQuaternion(input.orientation);

  return output;
}


static geometry_msgs::TransformStamped toMsgTransformStamped(const Pose3d& input, std::string child_frame_id)
{
  geometry_msgs::TransformStamped output;

  output.header.frame_id = input.frame_id;
  output.child_frame_id = child_frame_id;

  output.transform.translation.x = input.position.x();
  output.transform.translation.y = input.position.y();
  output.transform.translation.z = input.position.z();

  output.transform.rotation = toMsgQuaternion(input.orientation);

  return output;
}


static void toPose3d(const geometry_msgs::PoseStamped& input, Pose3d& output)
{
  output.frame_id = input.header.frame_id;
  toEigenVector3d(input.pose.position, output.position);
  toEigenQuaternion(input.pose.orientation, output.orientation);
}


static Pose3d toPose3d(const geometry_msgs::PoseStamped& input)
{
  Pose3d output(input.header.frame_id);
  toEigenVector3d(input.pose.position, output.position);
  toEigenQuaternion(input.pose.orientation, output.orientation);
  return output;
}


// === container conversion ===
static std::vector<geometry_msgs::PointStamped> toMsgPointStampedArray(const Point3dSet& input, const std::string& frame_id)
{
  std::vector<geometry_msgs::PointStamped> output;

  for (auto& point: input)
  {
    geometry_msgs::PointStamped p;

    p.header.frame_id = frame_id;
    p.point = toMsgPoint(point);

    output.push_back(p);
  }
  return output;
}

static std::vector<geometry_msgs::PointStamped> toMsgPointStampedArray(const Point3dVector& input, const std::string& frame_id)
{
  std::vector<geometry_msgs::PointStamped> output;

  for (auto& point: input)
  {
    geometry_msgs::PointStamped p;

    p.header.frame_id = frame_id;
    p.point = toMsgPoint(point);

    output.push_back(p);
  }
  return output;
}


static Point3dSet toPoint3dSet(const std::vector<geometry_msgs::PointStamped>& input)
{
  Point3dSet output;

  for (auto& point_msg: input)
  {
    Point3d p;
    toEigenVector3d(point_msg.point, p);
    output.insert(p);
  }
  return output;
}

static Point3dVector toPoint3dVector(const std::vector<geometry_msgs::PointStamped>& input)
{
  Point3dVector output;

  for (auto& point_msg: input)
  {
    Point3d p;
    toEigenVector3d(point_msg.point, p);
    output.push_back(p);
  }
  return output;
}


// === to string methods ===

static std::string to_string(const Point3d& point)
{
  return "(" + std::to_string(point.x()) + ", " + std::to_string(point.y()) + ", " + std::to_string(point.z()) + ")";
}
} // end namespace utils


} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_UTILS_WITH_TYPES_H
