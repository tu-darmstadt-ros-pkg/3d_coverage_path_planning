
#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint.h"
#include "three_dimensional_coverage_path_planning/utils/utils.h"
#include "three_dimensional_coverage_path_planning/utils/utils_with_types.h"

#include <algorithm>
#include <utility>
#include <visualization_msgs/Marker.h>

namespace three_dimensional_coverage_path_planning
{
Viewpoint::Viewpoint(Pose3d pose, const Point3dSet& visible_targets) : pose_(std::move(pose)),
                                                                       all_visible_targets_(visible_targets),
                                                                       color_generator_(60)
{
  // set all visible targets as seen from this viewpoint first
  targets_seen_from_this_viewpoint_ = visible_targets;

  reward_ = static_cast<double>(visible_targets.size());
}


Viewpoint::Viewpoint(Pose3d pose) : pose_(std::move(pose)), color_generator_(60)
{
  reward_ = 0;
}


Viewpoint::Viewpoint(const three_dimensional_coverage_path_planning_msgs::Viewpoint& viewpoint_msg)
  : pose_(utils::toPose3d(viewpoint_msg.pose)), color_generator_(60)
{
  reward_ = viewpoint_msg.reward;

  all_visible_targets_ = utils::toPoint3dSet(viewpoint_msg.all_visible_targets);

  targets_seen_from_this_viewpoint_ = utils::toPoint3dSet(viewpoint_msg.targets_seen_from_this_viewpoint);
  targets_seen_from_other_viewpoint_ = utils::toPoint3dVector(viewpoint_msg.targets_seen_from_other_viewpoints);
}


void Viewpoint::setVisibleTargets(const Point3dSet& visible_targets)
{
  if (!all_visible_targets_.empty())
  {
    throw std::logic_error(
      "Method setVisibleTargets called for a viewpoint for which the targets have already been set!");
  }

  all_visible_targets_ = visible_targets;

  // set all visible targets as seen from this viewpoint first
  targets_seen_from_this_viewpoint_ = visible_targets;
  reward_ = static_cast<double>(visible_targets.size());
}

void Viewpoint::markTargetAsSeenByThisViewpoint(const Point3d& target, bool publish)
{
  auto it = std::find(targets_seen_from_other_viewpoint_.begin(), targets_seen_from_other_viewpoint_.end(), target);
  if (it == targets_seen_from_other_viewpoint_.end())
  {
    throw std::runtime_error("Try to mark target as seen by current viewpoint that is not visible by viewpoint!");
  }

  targets_seen_from_other_viewpoint_.erase(it);
  targets_seen_from_this_viewpoint_.insert(target);

  ++reward_;

  if (publish)
  {
    publishViewpoint();
  }
}

void Viewpoint::updateReward(const Point3dSet& seen_targets, bool publish)
{
  auto old_size = targets_seen_from_other_viewpoint_.size();

  // find seen targets, that also could be seen from this viewpoint in order to remove them from the reward_
  std::set_intersection(seen_targets.begin(), seen_targets.end(),
                        targets_seen_from_this_viewpoint_.begin(), targets_seen_from_this_viewpoint_.end(),
                        std::back_inserter(targets_seen_from_other_viewpoint_), Point3dLessComparator());

  // update reward
  auto num_new_from_other_seen_targets = targets_seen_from_other_viewpoint_.size() - old_size;
  reward_ -= static_cast<double>(num_new_from_other_seen_targets);

  // update seen from this viewpoint list
  for (auto it = targets_seen_from_other_viewpoint_.end() - static_cast<long>(num_new_from_other_seen_targets);
       it != targets_seen_from_other_viewpoint_.end(); ++it)
  {
    targets_seen_from_this_viewpoint_.erase(*it);
  }

  // Note: viewpoint updateReward: if required, here a vector/set with the seen targets can be returned (create here, return after sort)
  //Point3dVector redundant_points(targets_seen_from_other_viewpoint_.end() - static_cast<long>(old_size), targets_seen_from_other_viewpoint_.end());

  // sort newly added targets
  std::sort(targets_seen_from_other_viewpoint_.begin(), targets_seen_from_other_viewpoint_.end(),
            Point3dLessComparator());

  // publish updated marker (only if pub and id have been set earlier, so pass no argument here)
  if (publish)
  {
    publishViewpoint();
  }
}

Pose3d Viewpoint::getPose() const
{
  return pose_;
}

double Viewpoint::getReward() const
{
  return reward_;
}

const Point3dSet& Viewpoint::getAllVisibleTargets() const
{
  return all_visible_targets_;
}

const Point3dVector& Viewpoint::getTargetsSeenFromOtherViewpoint() const
{
  return targets_seen_from_other_viewpoint_;
}

const Point3dSet& Viewpoint::getTargetsSeenFromThisViewpoint() const
{
  return targets_seen_from_this_viewpoint_;
}


three_dimensional_coverage_path_planning_msgs::Viewpoint Viewpoint::toMsg() const
{
  three_dimensional_coverage_path_planning_msgs::Viewpoint vp_msg;

  vp_msg.pose = utils::toMsgPoseStamped(pose_);

  vp_msg.reward = reward_;

  vp_msg.all_visible_targets = utils::toMsgPointStampedArray(all_visible_targets_, pose_.frame_id);

  vp_msg.targets_seen_from_this_viewpoint = utils::toMsgPointStampedArray(targets_seen_from_this_viewpoint_,
                                                                          pose_.frame_id);

  vp_msg.targets_seen_from_other_viewpoints = utils::toMsgPointStampedArray(targets_seen_from_other_viewpoint_,
                                                                            pose_.frame_id);

  return vp_msg;
}


void Viewpoint::publishViewpoint(std::shared_ptr<ros::Publisher> pub, int id)
{
  if (pub)
  {
    marker_publisher_ = std::move(pub);
  }
  if (id != -1)
  {
    id_ = id;
  }

  if (!marker_publisher_ || id_ == -1)
  {
    // cannot publish as publisher or id is invalid
    return;
  }

  visualization_msgs::Marker msg;


  msg.header.frame_id = pose_.frame_id;

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::ARROW;
  msg.ns = "rated_candidates";
  msg.id = id_;

  msg.color = color_generator_.nextColor();

  // set arrow length according to reward
  msg.scale.x = reward_ / 1000.0;
  msg.scale.y = 0.025;
  msg.scale.z = 0.025;

  msg.pose = utils::toMsgPoseStamped(pose_).pose;

  marker_publisher_->publish(msg);
}
} // end namespace three_dimensional_coverage_path_planning