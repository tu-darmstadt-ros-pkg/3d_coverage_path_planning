
#include "three_dimensional_coverage_path_planning/data_recording/data_recorder_base.h"
#include "three_dimensional_coverage_path_planning/utils/utils.h"
#include <ros/package.h>

namespace three_dimensional_coverage_path_planning
{

void DataRecorderBase::initialize(ros::NodeHandle& nh)
{
  pnh_ = ros::NodeHandle(nh, "data_recording");

  // get data topics with names
  XmlRpc::XmlRpcValue data_topics_list;
  pnh_.getParam("data_topics", data_topics_list);

  if (data_topics_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    throw std::invalid_argument(
      "Parameter data_topics is not a list! Type is " + std::to_string(data_topics_list.getType()));
  }

  for (int i = 0; i < data_topics_list.size(); i++) // NOLINT(modernize-loop-convert)
  {
    std::string name = data_topics_list[i]["name"];
    std::string topic = data_topics_list[i]["topic"];

    data_topics_[name] = topic;
  }


  // get other parameters
  recording_time_ = ros::Duration(pnh_.param<double>("recording_time_s", 0.0));

  recorded_data_directory_ = pnh_.param<std::string>("recorded_data_directory",
                                                     ros::package::getPath(ROS_PACKAGE_NAME) + "/recorded_data/");

  fixed_frame_ = pnh_.param<std::string>("fixed_frame", "world");
}

void DataRecorderBase::recordData(const geometry_msgs::PoseStamped& robot_pose)
{
  ROS_INFO_STREAM("Record data at position: " << utils::to_string(robot_pose.pose.position) << " and orientation "
                                              << utils::to_string(robot_pose.pose.orientation));

  current_robot_pose_ = robot_pose;

  recording_started_ = true;

  // wait for given waiting time (while e.g. subscriber callbacks are executed)
  // another possibility for subclasses would be to use ros::topic::waitForMessage instead of subscribers and callbacks
  recording_time_.sleep();

  recording_started_ = false;
}

void DataRecorderBase::finish()
{
}

std::string DataRecorderBase::getDirectory()
{
  return recorded_data_directory_;
}
} // end namespace three_dimensional_coverage_path_planning
