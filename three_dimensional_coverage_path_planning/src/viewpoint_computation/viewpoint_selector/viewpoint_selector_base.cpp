
#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint_selector/viewpoint_selector_base.h"

#include "three_dimensional_coverage_path_planning/utils/utils_with_types.h"


namespace three_dimensional_coverage_path_planning
{

ViewpointSelectorBase::ViewpointSelectorBase()
{
  store_map_target_to_viewpoints_ = false;
}

void ViewpointSelectorBase::initialize(ros::NodeHandle& nh, std::vector<Viewpoint>& candidates,
                                       const std::shared_ptr<const Point3dSet>& target_set)
{
  pnh_ = nh;

  min_reward_ = pnh_.param<double>("min_reward", 100);

  if (pnh_.param<bool>("publish_selected_candidates", false))
  {
    selected_candidates_pub_ = std::make_shared<ros::Publisher>(
      pnh_.advertise<visualization_msgs::Marker>("selected_candidates", 100, true));

    selected_cand_msg_.action = visualization_msgs::Marker::ADD;
    selected_cand_msg_.type = visualization_msgs::Marker::SPHERE;
    selected_cand_msg_.pose.orientation.w = 1;
    selected_cand_msg_.color = colors::cyan();
    selected_cand_msg_.scale.x = 0.125;
    selected_cand_msg_.scale.y = 0.125;
    selected_cand_msg_.scale.z = 0.125;
  }

  all_candidates_ = candidates;
  remaining_candidates_ = candidates;

  all_target_points_ = *target_set;
  uncovered_target_points_ = *target_set;

  selected_viewpoints_ = std::make_shared<std::vector<Viewpoint>>();
}


std::shared_ptr<std::vector<Viewpoint>> ViewpointSelectorBase::getSelectedViewpoints()
{
  return selected_viewpoints_;
}

const Point3dSet& ViewpointSelectorBase::getUncoveredTargetPoints()
{
  return uncovered_target_points_;
}


void ViewpointSelectorBase::selectViewpoint(Viewpoint viewpoint)
{
  // Note: Use copy here for Viewpoint as this often comes from remaining_candidates_ which is modified in this method.

  // TEMPORARY print + sleep
//  ROS_INFO_STREAM("Select viewpoint " << viewpoint.getPose().position.x() << ", " << viewpoint.getPose().position.y()
//                                      << " with reward: " << viewpoint.getReward());
//
//  ros::Duration d(0.5);
//  d.sleep();

  selected_viewpoints_->push_back(viewpoint);

  remaining_candidates_.erase(std::remove(remaining_candidates_.begin(), remaining_candidates_.end(), viewpoint),
                              remaining_candidates_.end());

  // update all remaining candidates
  for (auto& candidate: remaining_candidates_)
  {
    // Error if frame ids are different, as all candidates should be in the same frame, so something must have gone wrong.
    // Also, if they would be in different frames their target points could not be compared.
    if (candidate.getPose().frame_id != viewpoint.getPose().frame_id)
    {
      throw std::logic_error(
        "Selected viewpoint has a different frame id than at least one other candidate! Selected viewpoint: " +
        viewpoint.getPose().frame_id + ", candidate: " + candidate.getPose().frame_id);
    }

    candidate.updateReward(viewpoint.getTargetsSeenFromThisViewpoint());
  }


  // remove seen_targets targets from uncovered targets list
  Point3dSet swap_tmp;
  std::set_difference(std::make_move_iterator(uncovered_target_points_.begin()),
                      std::make_move_iterator(uncovered_target_points_.end()),
                      viewpoint.getTargetsSeenFromThisViewpoint().begin(),
                      viewpoint.getTargetsSeenFromThisViewpoint().end(),
                      std::inserter(swap_tmp, swap_tmp.begin()),
                      Point3dLessComparator()
  );
  uncovered_target_points_.swap(swap_tmp);


  // if requested, add viewpoint to mapping for all visible target points
  if(store_map_target_to_viewpoints_)
  {
    for(auto& target: viewpoint.getAllVisibleTargets())
    {
      map_covered_targets_to_viewpoints_[target].push_back(viewpoint);
    }
  }


  // visualization: publish selected candidates
  if (selected_candidates_pub_)
  {
    selected_cand_msg_.header.frame_id = selected_viewpoints_->back().getPose().frame_id;
    selected_cand_msg_.id = 1000 + selected_viewpoints_->size();

    selected_cand_msg_.pose = utils::toMsgPoseStamped(selected_viewpoints_->back().getPose()).pose;

    selected_candidates_pub_->publish(selected_cand_msg_);
  }
}


bool ViewpointSelectorBase::isSelectionSufficient()
{
  // no candidates left
  if (remaining_candidates_.empty())
  {
    return true;
  }

  // all target points covered
  if (uncovered_target_points_.empty())
  {
    return true;
  }

  // no remaining candidate has a reward_ greater than min_reward, so no candidate will improve the selection
  bool no_candidate_with_reward = true;
  for (auto it = remaining_candidates_.rbegin(); it != remaining_candidates_.rend(); it++)
  {
    // use min_reward instead of 0 as otherwise many points with only few new covered target points are selected,
    // so this parameter allows to trade off between the number of covered targets and the number of sights.
    if (it->getReward() >= min_reward_)
    {
      no_candidate_with_reward = false;
      break;
    }
  }
  if (no_candidate_with_reward)
  {
    return true;
  }

  // if none of the above applies, selection is not sufficient
  return false;
}

void ViewpointSelectorBase::resetSelector()
{
  remaining_candidates_ = all_candidates_;

  uncovered_target_points_ = all_target_points_;

  selected_viewpoints_ = std::make_shared<std::vector<Viewpoint>>();

  if (selected_candidates_pub_)
  {
    visualization_msgs::Marker delete_msg;
    delete_msg.action = visualization_msgs::Marker::DELETEALL;
    delete_msg.ns = selected_cand_msg_.ns;

    selected_candidates_pub_->publish(delete_msg);
  }
}
} // end namespace three_dimensional_coverage_path_planning