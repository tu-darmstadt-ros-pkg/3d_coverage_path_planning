#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_FILE_UTILS_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_FILE_UTILS_H


#include "three_dimensional_coverage_path_planning/viewpoint_computation/viewpoint.h"
#include "three_dimensional_coverage_path_planning/utils/utils_with_types.h"
#include "three_dimensional_coverage_path_planning_msgs/ViewpointArray.h"
#include "three_dimensional_coverage_path_planning_msgs/PointArray.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ros/package.h>


namespace three_dimensional_coverage_path_planning
{

namespace utils
{


static void
writeRankedCandidatesAndTargetSetToFile(const std::string& file_name, const std::vector<Viewpoint>& candidates,
                                        const std::shared_ptr<const Point3dSet>& target_set, std::string target_set_frame_id)
{
  rosbag::Bag bag;
  bag.open(file_name, rosbag::bagmode::Write);

  // convert candidates to message
  three_dimensional_coverage_path_planning_msgs::ViewpointArray candidates_msg;
  candidates_msg.viewpoints.reserve(candidates.size());
  for(auto vp : candidates)
  {
    candidates_msg.viewpoints.push_back(vp.toMsg());
  }

  // convert target_set to message
  three_dimensional_coverage_path_planning_msgs::PointArray target_set_msg;
  target_set_msg.points = utils::toMsgPointStampedArray(*target_set, target_set_frame_id);

  // write messages to file
  bag.write("candidates", ros::Time::now(), candidates_msg);
  bag.write("target_set", ros::Time::now(), target_set_msg);

  bag.close();
}





static void
readRankedCandidatesAndTargetSetFromFile(const std::string& file_name, std::vector<Viewpoint>& candidates,
                                         std::shared_ptr<const Point3dSet>& target_set)
{

  rosbag::Bag bag;
  try
  {
    bag.open(file_name);
  }
  catch (rosbag::BagException& e)
  {
    ROS_ERROR_STREAM("Could not open bag file with name: " << file_name);
    throw e;
  }


  // define topics for view
  std::vector<std::string> topics = {"candidates", "target_set"};

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ROS_INFO_STREAM("Reading from bag file " << file_name << ".");

  for (rosbag::MessageInstance const msg: view)
  {
    ROS_INFO_STREAM("Read message on topic: " << msg.getTopic() << "with data type: " << msg.getDataType());

    // candidates topic
    if (msg.getTopic() == topics[0])
    {
      three_dimensional_coverage_path_planning_msgs::ViewpointArrayPtr candidates_msg = msg.instantiate<three_dimensional_coverage_path_planning_msgs::ViewpointArray>();

      // check if it could be converted to requested msg type
      if (candidates_msg != nullptr)
      {
        for(auto& candidate: candidates_msg->viewpoints)
        {
          Viewpoint candidate_vp(candidate);
          candidates.push_back(candidate_vp);
        }
      }
    }
      // target_set topic
    else if (msg.getTopic() == topics[1])
    {
      three_dimensional_coverage_path_planning_msgs::PointArrayPtr target_set_msg = msg.instantiate<three_dimensional_coverage_path_planning_msgs::PointArray>();

      // check if it could be converted to Costmap msg type
      if (target_set_msg != nullptr)
      {
        target_set = std::make_shared<const Point3dSet>(utils::toPoint3dSet(target_set_msg->points));
      }
    }
  }

  bag.close();
}



template<typename T>
static void writeVectorInFile(std::string file_name, std::vector<T> data)
{
  // if file name is not absolute, complete it
  file_name = (file_name[0] == '/') ? file_name : ros::package::getPath(ROS_PACKAGE_NAME) + "/data/" + file_name;

  ROS_INFO_STREAM("Try to write data in file " << file_name);

  std::ofstream output_file(file_name);

  // write each entry in a new line
  std::string delimiter = "\n";

  std::ostream_iterator<T> output_iterator(output_file, delimiter.c_str());

  std::copy(data.begin(), data.end(), output_iterator);
}



template<typename T1, typename T2>
static void writeVectorInFile(std::string file_name, std::vector<std::pair<T1, T2>> data)
{
  // if file name is not absolute, complete it
  file_name = (file_name[0] == '/') ? file_name : ros::package::getPath(ROS_PACKAGE_NAME) + "/data/" + file_name;

  ROS_INFO_STREAM("Try to write data in file " << file_name);

  std::ofstream output_file(file_name);

  for(auto& element: data)
  {
    output_file << element.first << " " << element.second << std::endl;
  }
}




} // end namespace utils





} // end namespace three_dimensional_coverage_path_planning

#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_FILE_UTILS_H
