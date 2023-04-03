
#include "three_dimensional_coverage_path_planning/data_recording/lidar_data_recorder.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/ply_io.h>

#include <pluginlib/class_list_macros.h>


namespace three_dimensional_coverage_path_planning
{


LidarDataRecorder::LidarDataRecorder() : tf_listener_(tf_buffer_), color_generator_(10)
{
}


void LidarDataRecorder::initialize(ros::NodeHandle& nh)
{
  DataRecorderBase::initialize(nh);

  // get parameter
  keep_original_color_ = pnh_.param<bool>("keep_original_color", false);

  publish_intermediate_results_ = pnh_.param<bool>("publish_intermediate_results", false);
  save_intermediate_results_ = pnh_.param<bool>("save_intermediate_results", false);


  // init subscriber
  for (auto& name_topic: data_topics_)
  {
    // bind callback function with usage of name
    boost::function<void(const sensor_msgs::PointCloud2::ConstPtr&)> cb =
      boost::bind(&LidarDataRecorder::point_cloud_callback, this, _1, name_topic.first);

    // add subscriber
    ros::Subscriber sub = pnh_.subscribe(name_topic.second, 10, cb);
    data_subscriber_.push_back(sub);

    // add publisher
    if (publish_intermediate_results_)
    {
      ros::Publisher pub = pnh_.advertise<sensor_msgs::PointCloud2>("recorded_data_" + name_topic.first, 1, true);
      acc_point_cloud_pub_[name_topic.first] = pub;
    }
  }


  // init point clouds
  for (auto& name: data_topics_)
  {
    accumulated_point_clouds_[name.first] = pcl::PointCloud<pcl::PointXYZRGB>();
    accumulated_point_clouds_[name.first].header.frame_id = fixed_frame_;
  }
}


void LidarDataRecorder::recordData(const geometry_msgs::PoseStamped& robot_pose)
{
  auto current_color = color_generator_.nextColor();
  rgb_[0] = current_color.r * 255;
  rgb_[1] = current_color.g * 255;
  rgb_[2] = current_color.b * 255;

  DataRecorderBase::recordData(robot_pose);

  ROS_INFO_STREAM("Finished recording data.");

  for (auto& entry: accumulated_point_clouds_)
  {
    ROS_INFO_STREAM("Acc. point cloud: name \"" << entry.first << "\", size: " << entry.second.size());
  }


  if (save_intermediate_results_)
  {
    save_point_cloud();
  }
}


void
LidarDataRecorder::point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, const std::string& name)
{
  // only execute callback if a waypoint was reached/scan was requested
  if (!recording_started_)
  {
    return;
  }

  // transform point cloud to fixed frame (world)
  sensor_msgs::PointCloud2 transformed_cloud_msg;
  bool transform_result = pcl_ros::transformPointCloud(fixed_frame_, *cloud_msg, transformed_cloud_msg, tf_buffer_);

  if (!transform_result)
  {
    ROS_WARN_STREAM("Cloud could not be transformed. Use latest transform.");
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tf_buffer_.lookupTransform(fixed_frame_, cloud_msg->header.frame_id, ros::Time(0));

      tf2::doTransform(*cloud_msg, transformed_cloud_msg, transformStamped);


      geometry_msgs::TransformStamped transform_msg = tf_buffer_.lookupTransform(fixed_frame_,
                                                                                 cloud_msg->header.frame_id,
                                                                                 ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_STREAM("Cloud could not be transformed with lastest transform: " << ex.what());
    }
  }

  if (transformed_cloud_msg.data.empty())
  {
    ROS_ERROR_STREAM("Transformed point cloud was empty, see console for pcl error!");
    return;
  }

  // convert to pcl point cloud

  pcl::PointCloud<pcl::PointXYZRGB> point_cloud;

  // if incoming point cloud has no colors, i.e. has type PointXYZ, convert to PointXYZRGB
  if (std::find_if(transformed_cloud_msg.fields.begin(), transformed_cloud_msg.fields.end(),
                   [](sensor_msgs::PointField& field) { return field.name == "rgb"; }) ==
      transformed_cloud_msg.fields.end())
  {
    pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
    pcl::fromROSMsg(transformed_cloud_msg, tmp_cloud);

    pcl::copyPointCloud(tmp_cloud, point_cloud);
  }
  else
  {
    pcl::fromROSMsg(transformed_cloud_msg, point_cloud);
  }


  // set point cloud color if requested
  if (!keep_original_color_)
  {
    for (auto& point: point_cloud)
    {
      point.r = rgb_[0];
      point.g = rgb_[1];
      point.b = rgb_[2];
    }
  }

  // add point cloud to accumulated
  accumulated_point_clouds_[name] += point_cloud;

  // publish accumulated point cloud
  if (publish_intermediate_results_)
  {
    sensor_msgs::PointCloud2 acc_pc_msg;
    pcl::toROSMsg(accumulated_point_clouds_[name], acc_pc_msg);
    acc_point_cloud_pub_[name].publish(acc_pc_msg);
  }
}

void LidarDataRecorder::finish()
{
  save_point_cloud();
}

void LidarDataRecorder::save_point_cloud()
{
  for (auto& name_point_cloud: accumulated_point_clouds_)
  {
    // get timestamp
    char buffer[256];
    auto time = static_cast<std::time_t>(ros::WallTime::now().toSec());
    auto local_time = std::localtime(&time);
    strftime(buffer, sizeof(buffer), "%y-%m-%d_%H-%M-%S", local_time);

    std::string file_name =
      recorded_data_directory_ + "/" + name_point_cloud.first + "_" + std::string(buffer) + ".ply";

    // save file
    try
    {
      int result = pcl::io::savePLYFile(file_name, name_point_cloud.second);

      if (result == 0)
      {
        ROS_INFO_STREAM("Saved point cloud to file " << file_name);
      }
      else
      {
        ROS_ERROR_STREAM("No ply file saved for name " << name_point_cloud.first
                                                       << " as an pcl writer error occurred. See pcl error on console for details.");
      }
    }
    catch (std::exception& e)
    {
      ROS_ERROR_STREAM(
        "No ply file saved for name " << name_point_cloud.first << " as an pcl writer error occurred: " << e.what());
    }
  }
}
} // end namespace three_dimensional_coverage_path_planning

PLUGINLIB_EXPORT_CLASS(three_dimensional_coverage_path_planning::LidarDataRecorder,
                       three_dimensional_coverage_path_planning::DataRecorderBase)