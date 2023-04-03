#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_DATA_RECORDER_BASE_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_DATA_RECORDER_BASE_H


#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>


namespace three_dimensional_coverage_path_planning
{

class DataRecorderBase
{
public:
  /**
   * This method is called before the first waypoint is approached.
   * In the base implementation gets the data topics and names from parameter server as well as
   * the recorded data directory, the fixed frame and the recording time.
   * @param nh
   */
  virtual void initialize(ros::NodeHandle& nh);

  /**
   * Record data on the given waypoint/robot pose.
   * The base implementation sets recording_started_, sleeps for the given recording time
   * and changes recording_started_ back to false afterwards.
   * @param robot_pose
   */
  virtual void recordData(const geometry_msgs::PoseStamped& robot_pose);

  /**
   * Method that is called when path execution is finished (after last waypoint).
   * Override this method e.g. if the data are stored but not saved while executing a single waypoint (e.g. when aggregating point clouds).
   * Base implementation is empty.
   */
  virtual void finish();


  /**
   * Get the directory, that contains the recorded data.
   * @return recorded data directory
   */
  std::string getDirectory();

protected:

  ros::NodeHandle pnh_;

  std::map<std::string, std::string> data_topics_;

  std::vector<ros::Subscriber> data_subscriber_;

  geometry_msgs::PoseStamped current_robot_pose_;

  ros::Duration recording_time_;

  std::string recorded_data_directory_;

  std::string fixed_frame_;


  bool recording_started_ = false;
};
} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_DATA_RECORDER_BASE_H
