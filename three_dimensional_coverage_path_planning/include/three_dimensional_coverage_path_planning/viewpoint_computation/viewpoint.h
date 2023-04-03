#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_VIEWPOINT_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_VIEWPOINT_H

#include "three_dimensional_coverage_path_planning/utils/types.h"
#include "three_dimensional_coverage_path_planning/utils/colors.h"

#include "three_dimensional_coverage_path_planning_msgs/Viewpoint.h"

namespace three_dimensional_coverage_path_planning
{

class Viewpoint
{
public:
  Viewpoint(Pose3d pose, const Point3dSet& visible_targets);

  explicit Viewpoint(Pose3d pose);

  explicit Viewpoint(const three_dimensional_coverage_path_planning_msgs::Viewpoint& viewpoint_msg);

  /**
   * set the visible targets for a viewpoint and computes initial reward
   * @throw logic_error if called on a viewpoint for which the targets have already been set
   * @param visible_targets
   */
  void setVisibleTargets(const Point3dSet& visible_targets);

  void markTargetAsSeenByThisViewpoint(const Point3d& target, bool publish = true);

  void updateReward(const Point3dSet& seen_targets, bool publish = true);

  bool operator==(const Viewpoint& rhs) const
  {
    return ((this->pose_.position == rhs.pose_.position) &&
            this->pose_.orientation.isApprox(rhs.pose_.orientation));
  }


  Pose3d getPose() const;

  double getReward() const;

  const Point3dSet& getAllVisibleTargets() const;

  const Point3dVector& getTargetsSeenFromOtherViewpoint() const;

  const Point3dSet& getTargetsSeenFromThisViewpoint() const;

  three_dimensional_coverage_path_planning_msgs::Viewpoint toMsg() const;

  void publishViewpoint(std::shared_ptr<ros::Publisher> pub = nullptr, int id = -1);

private:

  Pose3d pose_;
  double reward_;

  Point3dSet all_visible_targets_; /// all targets that are visible from this viewpoint

  Point3dVector targets_seen_from_other_viewpoint_; /// targets that have been marked as seen from other viewpoints (copied here while calling updateReward)

  Point3dSet targets_seen_from_this_viewpoint_; /// targets that have counted into the reward_ of this viewpoint and hence should be seen from this viewpoint later (copied here when viewpoint is selected)

  // visualization
  std::shared_ptr<ros::Publisher> marker_publisher_;
  int id_ = -1;
  colors::ColorGenerator color_generator_;
};


struct ViewpointRewardComparator
{
  bool operator()(const Viewpoint& lhs, const Viewpoint& rhs)
  {
    return lhs.getReward() < rhs.getReward();
  }
};
} // end namespace three_dimensional_coverage_path_planning

#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_VIEWPOINT_H
