#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_TYPES_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_TYPES_H

#include "three_dimensional_coverage_path_planning/utils/utils.h"

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <set>
#include <vector>
#include <utility>

namespace three_dimensional_coverage_path_planning
{

using Point3d = Eigen::Vector3d;
using Quaterniond = Eigen::Quaterniond;


struct Pose3d
{
  Pose3d(Point3d position, Quaterniond orientation, const std::string& frame_id)
  {
    this->position = std::move(position);
    this->orientation = std::move(orientation);
    this->frame_id = frame_id;
  }

  Pose3d(Point3d position, const std::string& frame_id)
  {
    this->position = std::move(position);
    this->orientation.setIdentity();
    this->frame_id = frame_id;
  }

  explicit Pose3d(const std::string& frame_id)
  {
    this->position.setZero();
    this->orientation.setIdentity();
    this->frame_id = frame_id;
  }

  Point3d position;
  Quaterniond orientation;

  std::string frame_id;
};


struct Point3dLessComparator
{
  bool operator()(const Point3d& lhs, const Point3d& rhs) const
  {
    return utils::compare(lhs, rhs) < 0;
  }
};

struct Pose3dLessComparator
{
  bool operator()(const Pose3d& lhs, const Pose3d& rhs) const
  {
    // Logic error if the poses are if different frames.
    // This should not happen, as the comparator is mainly used in sort algorithms and containers as e.g. set or map.
    if (lhs.frame_id != rhs.frame_id)
    {
      throw std::logic_error(
        "Tried to compare two poses in different frames! lhs: " + lhs.frame_id + ", rhs: " + rhs.frame_id + ".");
    }

    // check position first
    auto position_result = utils::compare(lhs.position, rhs.position);
    if (position_result != 0)
    {
      return position_result < 0;
    }
    else
    {
      // if position is equal, compare orientation
      return utils::compare(lhs.orientation, rhs.orientation) < 0;
    }
  }
};


using Point3dSet = std::set<Point3d, Point3dLessComparator>;
using Point3dVector = std::vector<Point3d>;

using Pose3dVector = std::vector<Pose3d>;
} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_TYPES_H
