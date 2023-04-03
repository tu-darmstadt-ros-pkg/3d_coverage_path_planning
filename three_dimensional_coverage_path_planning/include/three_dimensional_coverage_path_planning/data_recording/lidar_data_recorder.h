#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_LIDAR_DATA_RECORDER_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_LIDAR_DATA_RECORDER_H


#include "three_dimensional_coverage_path_planning/data_recording/data_recorder_base.h"
#include "three_dimensional_coverage_path_planning/utils/colors.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace three_dimensional_coverage_path_planning
{


class LidarDataRecorder : public DataRecorderBase
{
public:

  LidarDataRecorder();

  void initialize(ros::NodeHandle& nh) override;

  void recordData(const geometry_msgs::PoseStamped& robot_pose) override;

  void finish() override;


private:

  void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, const std::string& name);

  void save_point_cloud();


  std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>> accumulated_point_clouds_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool keep_original_color_;

  std::uint8_t rgb_[3];

  colors::ColorGenerator color_generator_;

  std::map<std::string, ros::Publisher> acc_point_cloud_pub_;

  bool publish_intermediate_results_;

  bool save_intermediate_results_;



};
} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_LIDAR_DATA_RECORDER_H
