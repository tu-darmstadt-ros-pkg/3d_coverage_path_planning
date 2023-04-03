#include "three_dimensional_coverage_path_planning/viewpoint_computation/visibility_checker/self_filter_checker.h"

#include "three_dimensional_coverage_path_planning/utils/transformation_helper.h"
#include <pcl_conversions/pcl_conversions.h>

#include <filters/filter_chain.hpp>

#include <pluginlib/class_list_macros.h>

namespace three_dimensional_coverage_path_planning
{


SelfFilterChecker::SelfFilterChecker()
{
  plugin_name_ = "SelfFilterChecker";

  // get max index to use as vector size
  num_idx_ = 1 + std::max({SELF_FILTER_CHECK_PASSED_IDX, SELF_FILTER_BLOCKED_IDX});
}


void SelfFilterChecker::initialize(ros::NodeHandle& pnh, std::shared_ptr<const ModelData> model)
{
  VisibilityCheckerBase::initialize(pnh, model);

  if (publish_visibility_checks_)
  {
    // publisher and msg setup
    check_pubs_.resize(num_idx_);
    check_pubs_[SELF_FILTER_CHECK_PASSED_IDX] = pnh_.advertise<visualization_msgs::Marker>("self_filter_check_passed",
                                                                                           10, true);
    check_pubs_[SELF_FILTER_BLOCKED_IDX] = pnh_.advertise<visualization_msgs::Marker>("self_filter_blocked", 10,
                                                                                      true);
  }

  utils::searchAndGetParam(pnh_, "sensor_frame", sensor_frame_);


  // precompute a spherical mask and apply self filter on it in order to easily check if a point is filtered by self filter or not during execution.
  generatePointsOnSphericalMask(10000);
  applySelfFilterOnMask();


  if (publish_visibility_checks_)
  {
    mask_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("self_filter_mask", 1, true);

    sensor_msgs::PointCloud2 mask_msg;
    pcl::toROSMsg(self_filter_mask_, mask_msg);

    mask_msg.header.frame_id = model_->getModelFrame();

    mask_pub_.publish(mask_msg);
  }
}


bool SelfFilterChecker::check(Pose3d& start, Point3d& end)
{
  // convert point to pose
  Pose3d end_in_model_frame(end, start.frame_id);

  // compute end in sensor frame when sensor is located at start
  Pose3d end_in_start_frame = TransformationHelper::transformFromModelToSensorFrame(start, end_in_model_frame,
                                                                                    sensor_frame_);

  // normalize direction to be checked on unit spherical mask
  end_in_start_frame.position.normalize();

  // check if point is visible
  if (checkDirectionOnMask(end_in_start_frame.position))
  {
    addMarkerToMsg(SELF_FILTER_CHECK_PASSED_IDX, start.position, end, colors::green());
    incStats(SELF_FILTER_CHECK_PASSED_IDX);
    return true;
  }
  else
  {
    addMarkerToMsg(SELF_FILTER_BLOCKED_IDX, start.position, end, colors::violet());
    incStats(SELF_FILTER_BLOCKED_IDX);
    return false;
  }
}


void SelfFilterChecker::printStats()
{
  if (!print_stats_)
  {
    return;
  }

  std::string stats = "Self-filter Checker Statistics:\n";

  stats += "Self-filter check passed (true): " + std::to_string(stats_[SELF_FILTER_CHECK_PASSED_IDX]) + "\n";
  stats += "\n";
  stats += "Self-filter: blocked (false): " + std::to_string(stats_[SELF_FILTER_BLOCKED_IDX]) + "\n";
  stats += "\n";

  ROS_INFO_STREAM_NAMED(ROS_PACKAGE_NAME, stats);
}


void SelfFilterChecker::generatePointsOnSphericalMask(int num_points)
{
  self_filter_mask_.clear();
  self_filter_mask_.resize(num_points);


  for (int i = 0; i < num_points; ++i)
  {
    // compute next point
    auto fib_point = computeFibPoint(i, num_points);

    // add to mask
    utils::toPclPoint(fib_point, self_filter_mask_[i]);

    // set default intensity to 1 = visible, if self filter (e.g. configuration etc) does not work, this check just returns true for all points
    self_filter_mask_[i].intensity = 1;
  }
}


void SelfFilterChecker::applySelfFilterOnMask()
{
  /**
   * see:
   * self filter: http://wiki.ros.org/robot_body_filter
   * hector fork: https://github.com/tu-darmstadt-ros-pkg/robot_body_filter
   *
   * how it is used as nodelet in our workspace:
   *    https://github.com/tu-darmstadt-ros-pkg/hector_filter_nodelets/blob/master/src/cloud_filter_chain.cpp
   *
   * configuration of cloud_filter_chain for asterix (taken from there and slightly adapted in own config file):
   *    https://git.sim.informatik.tu-darmstadt.de/hector/asterix_launch/-/blob/master/asterix_onboard_launch/config/lidar_proc/vlp16_robot_body_filter_config.yaml
   */

  // setup filter chain for robot self filter
  filters::FilterChain<sensor_msgs::PointCloud2> chain("sensor_msgs::PointCloud2");

  bool chain_config_result = chain.configure("cloud_filter_chain", pnh_);

  if (!chain_config_result)
  {
    ROS_ERROR_STREAM("Configuration of self filter chain failed!");
    return;
  }



  // convert input cloud = self_filter_mask_ to PointCloud2
  sensor_msgs::PointCloud2 cloud_msg_in;
  pcl::toROSMsg(self_filter_mask_, cloud_msg_in);

  // Note: This sleep is required for robot body filter.
  //  - Without any sleep, the configuration time is same as input cloud time, which leads to rejection of the cloud.
  //  - With a shorter sleep, the filtered point cloud is empty.
  //  - Without the sleep and instead the cloud time stamp now+ros::Duration(2.0) the cloud is not rejected (see first point) but filtered cloud is also empty.
  //  Maybe in the future, there might be another solution for this problem, but up to now, a short sleep is required.
  // (if this should be tested without or a shorter sleep:
  // in rqt in plugin "Logger level", set for node "three_dimensional_coverage_path_planning" Logger "ros" to Level "Debug",
  // then some error messages from the robot body filter are printed.
  auto d = ros::Duration(2.0);
  d.sleep();

  // TODO sleep required for robot body filter: maybe if filtered cloud is still empty after sleep,
  //  try again x times until a timeout was exceeded?

  // correct sensor frame and current time stamp required for robot self filter
  // (time stamp compared with configuration time, "older" point clouds are rejected)
  cloud_msg_in.header.frame_id = sensor_frame_;
  cloud_msg_in.header.stamp = ros::Time::now();


  // apply chain on self filter mask
  sensor_msgs::PointCloud2 cloud_msg_out;
  bool chain_update_result = chain.update(cloud_msg_in, cloud_msg_out);

  if (!chain_update_result)
  {
    ROS_ERROR_STREAM("Error when applying self filter!");
    return;
  }

  if (cloud_msg_out.data.empty())
  {
    ROS_ERROR_STREAM("Error when applying self filter: filtered cloud is empty!");
    return;
  }


  // convert back to pcl point cloud in order to compare with self_filter_mask_
  pcl::PointCloud<pcl::PointXYZI> filtered_cloud;
  pcl::fromROSMsg(cloud_msg_out, filtered_cloud);


  // check which points of cloud_msg_out have not been filtered out and set these to intensity 1, all other points to intensity 0
  for (auto& point: self_filter_mask_)
  {
    // comparator
    auto is_equal_to_point = [point](pcl::PointXYZI& input)
    {
      return
        input.x == point.x &&
        input.y == point.y &&
        input.z == point.z;
    };

    // try to find point in filtered cloud
    auto it = std::find_if(filtered_cloud.begin(), filtered_cloud.end(), is_equal_to_point);


    if (it == filtered_cloud.end())
    {
      // not found = filtered = blocked
      point.intensity = 0;
    }
    else
    {
      // found = not filtered = visible
      point.intensity = 1;

      // remove found point from filtered cloud in order to speed up later find operations
      filtered_cloud.erase(it);
    }
  }
}

bool SelfFilterChecker::checkDirectionOnMask(Point3d& direction_to_check)
{
  if (direction_to_check.norm() != 1)
  {
    direction_to_check.normalize();
  }


  // compute the nearest point on mask, according to "Spherical Fibonacci Mapping" by B. Keinert et al.
  // (https://dl.acm.org/doi/abs/10.1145/2816795.2818131), Listing 1

  // p in paper: direction_to_check

  // n in paper
  int num_points = static_cast<int>(self_filter_mask_.size());

  // PHI in paper is called GOLDEN_RATIO here


  // compute spherical coordinates of point
  double cosTheta = direction_to_check.z();
  double phi;
  try
  {
    phi = std::atan2(direction_to_check.y(), direction_to_check.x());
  }
  catch (std::domain_error&)
  {
    phi = M_PI;
  }

  // compute zone number k (Eq. 5 in paper) (log with base golden_ration^2 required, hence use base change)
  double k = std::max(2.0,
                      std::floor(std::log(num_points * M_PI * std::sqrt(5.0) * (1 - cosTheta * cosTheta)) /
                                 std::log(GOLDEN_RATIO * GOLDEN_RATIO)));

  // compute approximated Fibonacci numbers F_k, F0 = rounded F_k, and F1 = rounded F_(k+1)
  double Fk = std::pow(GOLDEN_RATIO, k) / std::sqrt(5);
  double F0 = std::round(Fk);
  double F1 = std::round(Fk * GOLDEN_RATIO);



  // modf_tmp variable required for integral part of modf
  double modf_tmp;

  // compute B_k as (b_k, b_{k+1}) (Eq. 8) with Eq. 13 for b_k and b_{k + 1}
  Eigen::Matrix2d B;
  B(0, 0) = 2 * M_PI * std::modf((F0 + 1) * (GOLDEN_RATIO - 1), &modf_tmp) - 2 * M_PI * (GOLDEN_RATIO - 1);
  B(1, 0) = -2 * F0 / num_points;

  B(0, 1) = 2 * M_PI * std::modf((F1 + 1) * (GOLDEN_RATIO - 1), &modf_tmp) - 2 * M_PI * (GOLDEN_RATIO - 1);
  B(1, 1) = -2 * F1 / num_points;


  // compute inverse from B_k
  Eigen::Matrix2d B_inv = B.inverse();

  // offset (defined between Eq. 8 and 9 in paper)
  double z0 = (1 - 1.0 / num_points);

  // compute c according to Eq. 11 and 10,
  // c is the lowest coordinate of a corresponding cell on the local grid (one corner of the cell) for zone number k
  Eigen::Vector2d c = B_inv * Eigen::Vector2d(phi, cosTheta - z0);
  utils::floor(c);


  // d and j in paper
  auto min_dist = DBL_MAX;
  int min_idx = 0;

  // check for each of the 4 corners of the cell, which is closest to the desired point
  for (unsigned int s = 0; s < 4; ++s)
  {
    // cell corners are: (c, c + (0,1)^T, c + (1,0)^T, c + (1,1)^T
    auto corner = Eigen::Vector2d(s % 2, s / 2) + c;

    // compute z = cosTheta of current corner
    double cosTheta_local = B.row(1).dot(corner) + z0;

    double clamp_val = cosTheta_local;
    if (clamp_val < -1)
      clamp_val = -1;
    else if (clamp_val > 1)
      clamp_val = 1;

    cosTheta_local = clamp_val * 2 - cosTheta_local;


    // compute index i according to Eq. 12 (computation of z = cosTheta in Eq. 2 reformulated for i and rounded to the nearest integer)
    int i = std::floor(num_points * (1 - cosTheta_local) / 2.0);


    // compute spherical coordinates of point with index i
    auto q_i = computeFibPoint(i, num_points);

    // compute error of point with index i to get the closet one of the potential nearest points
    double squaredDistance = (q_i - direction_to_check).dot(q_i - direction_to_check);

    // check if point q_i is new nearest point
    if (squaredDistance < min_dist)
    {
      min_dist = squaredDistance;
      min_idx = i;
    }
  }

  // result: Point with index min_idx is the nearest to the requested one
  pcl::PointXYZI nearest_point_on_mask = self_filter_mask_[min_idx];

  // if intensity is greater 0, the point is not blocked and hence can be seen
  return (nearest_point_on_mask.intensity > 0);
}


Eigen::Vector3d SelfFilterChecker::computeFibPoint(int i, int num_points) const
{
  // compute spherical coords according to Eq. 1,2 in paper
  double modf_tmp;
  double phi = 2 * M_PI * std::modf(i * (GOLDEN_RATIO - 1), &modf_tmp);

  // compute cos(theta) instead of theta to avoid multiple sin/cos/acos operations
  double cosTheta = 1 - (2.0 * i + 1.0) / num_points;

  // compute sin(theta) = sin(arccos(cosTheta)) = sqrt(1 - cosTheta^2)
  double sinTheta = std::sqrt(1 - cosTheta * cosTheta);

  // convert to cartesian
  Eigen::Vector3d cartesian(std::cos(phi) * sinTheta,
                            std::sin(phi) * sinTheta,
                            cosTheta);

  return cartesian;
}
} // end namespace three_dimensional_coverage_path_planning


PLUGINLIB_EXPORT_CLASS(three_dimensional_coverage_path_planning::SelfFilterChecker,
                       three_dimensional_coverage_path_planning::VisibilityCheckerBase)