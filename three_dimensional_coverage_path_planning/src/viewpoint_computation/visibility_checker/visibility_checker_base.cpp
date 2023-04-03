
#include "three_dimensional_coverage_path_planning/viewpoint_computation/visibility_checker/visibility_checker_base.h"

#include "three_dimensional_coverage_path_planning/utils/utils.h"

namespace three_dimensional_coverage_path_planning
{


void VisibilityCheckerBase::initialize(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model)
{
  pnh_ = ros::NodeHandle(pnh, "visibility_checker");

  // if parallelization with openmp is used, do not publish any messages, otherwise check parameter
#ifdef _OPENMP
  publish_visibility_checks_ = false;
  print_stats_ = false;
#else
  utils::searchAndGetParam(pnh_, "publish_visibility_checks", false, publish_visibility_checks_);
  utils::searchAndGetParam(pnh_, "print_stats", false, print_stats_);
#endif


  // set model
  model_ = std::move(model);

  if (print_stats_)
  {
    stats_.resize(num_idx_);
  }

  if (publish_visibility_checks_)
  {
    //msg setup (publisher are set up in the plugin constructors)
    check_msgs_.resize(num_idx_);
    points_.resize(num_idx_);
    colors_.resize(num_idx_);

    for (auto& check_msg: check_msgs_)
    {
      check_msg = visualization_msgs::Marker();
      check_msg.ns = "visibility_check";
    }
  }
}


void VisibilityCheckerBase::printStats()
{
  if (!print_stats_)
  {
    return;
  }

  std::string stats = "Visibility Checker Statistics: \n\n";

  for (auto& data: stats_)
  {
    stats += std::to_string(data) + "\n";
  }

  ROS_INFO_STREAM_NAMED(ROS_PACKAGE_NAME, stats);
}


void VisibilityCheckerBase::publishAllMarkers()
{
  for (unsigned int i = 0; i < check_msgs_.size(); i++)
  {
    publishMarkerMsg(i);
  }
};

void VisibilityCheckerBase::resetAllMsgs()
{
  for (unsigned int i = 0; i < check_msgs_.size(); i++)
  {
    points_[i].clear();
    colors_[i].clear();
  }
}

std::string VisibilityCheckerBase::getPluginName()
{
  return plugin_name_;
}


// =========================================== //
// ============= private methods ============= //
// =========================================== //


void VisibilityCheckerBase::addMarkerToMsg(const int& idx, Point3d& start, Point3d& end,
                                           std_msgs::ColorRGBA color_start, std_msgs::ColorRGBA color_end)
{
  if (!publish_visibility_checks_)
  {
    return;
  }

  points_[idx].push_back(utils::toMsgPoint(start));
  points_[idx].push_back(utils::toMsgPoint(end));
  colors_[idx].push_back(color_start);
  colors_[idx].push_back(color_end);
}

void VisibilityCheckerBase::addMarkerToMsg(const int& idx, Point3d& start, Point3d& end,
                                           std_msgs::ColorRGBA color)
{
  addMarkerToMsg(idx, start, end, color, color);
}


void VisibilityCheckerBase::publishMarkerMsg(const int& idx)
{
  // only publish if required and if there are any points
  if (!publish_visibility_checks_ || points_[idx].empty())
  {
    return;
  }

//  publishDeleteMarker(check_pubs_[idx]);

  check_msgs_[idx].header.stamp = ros::Time::now();
  check_msgs_[idx].header.frame_id = model_->getModelFrame();

  bool use_lines;
  utils::searchAndGetParam(pnh_, "visibility_lines", false, use_lines);

  if (use_lines)
  {
    check_msgs_[idx].type = visualization_msgs::Marker::LINE_LIST;
  }
  else
  {
    check_msgs_[idx].type = visualization_msgs::Marker::POINTS;
  }

  check_msgs_[idx].action = visualization_msgs::Marker::ADD;
  check_msgs_[idx].pose.orientation.w = 1;
  check_msgs_[idx].scale.x = 0.025;
  check_msgs_[idx].scale.y = 0.025;
  check_msgs_[idx].scale.z = 0.025;
  check_msgs_[idx].id = idx;

  check_msgs_[idx].points = points_[idx];
  check_msgs_[idx].colors = colors_[idx];

  check_pubs_[idx].publish(check_msgs_[idx]);
}


void VisibilityCheckerBase::incStats(const int& idx)
{
  // only increase stats if required
  if (print_stats_)
  {
    stats_[idx]++;
  }
}
} // end namespace three_dimensional_coverage_path_planning
