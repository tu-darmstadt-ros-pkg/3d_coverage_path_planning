#include "three_dimensional_coverage_path_planning/path_execution/path_executor.h"

#include "three_dimensional_coverage_path_planning/utils/utils.h"
#include "three_dimensional_coverage_path_planning/utils/transformation_helper.h"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_proc/grid_map_polygon_tools.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace three_dimensional_coverage_path_planning
{

PathExecutor::PathExecutor(const nav_msgs::Path& path,
                           const move_base_lite_msgs::FollowPathOptions& follow_path_options,
                           MoveToWaypointActionServer* const action_server)
  : follow_path_action_client_("/controller/follow_path", false)
{

  // resend a not updated path x seconds after last change
  // It should be updated in earlier when robot is moving, at least be truncated.
  // If this does not happen, the robot does not move, e.g. because vehicle controller had a problem. Then resend path.
  resend_same_path_ = ros::Duration(8);

  MoveToWaypointActionFeedback feedback;

  if (action_server)
  {
    feedback.progress = "Start following path.";
    action_server->publishFeedback(feedback);
  }

  // TEMPORARY
  ROS_INFO_STREAM(
    "Start following path from: " << utils::to_string(path.poses[0].pose.position) << " to "
                                  << utils::to_string(path.poses.back().pose.position) << ".");

  pnh_ = ros::NodeHandle("~");

  original_path_pub_ = pnh_.advertise<nav_msgs::Path>("original_path", 100, true);
  updated_path_pub_ = pnh_.advertise<nav_msgs::Path>("updated_path", 100, true);


  // set options and path
  follow_path_goal_.follow_path_options = follow_path_options;

  current_path_ = path;

  original_path_pub_.publish(current_path_);


  // get parameter
  robot_length_ = pnh_.param("robot_length", 0.85);
  robot_width_ = pnh_.param("robot_width", 0.65);

  utils::searchAndGetParam<std::string>(pnh_, "base_frame", "base_link", base_frame_);


  // TODO maybe set thresholds for grid map planner (lethal dist, penalty dist, penalty weight) --> defaults: 4 [cells?], 12[cells?], 1
//  grid_map_planner_.setDistanceThresholds(6, 10, 1);


  action_finished_ = false;

  // subscribe to map
  first_map_cb_call_ = true;

  map_subscriber_ = nh_.subscribe("/map", 1, &PathExecutor::mapCallback, this);

  robot_pose_subscriber_ = nh_.subscribe("/robot_pose", 1, &PathExecutor::robotPoseCallback, this);


  // wait for map message, with first message also the callback will be called and the path updated and published
  ROS_INFO_STREAM("Waiting for first map message");
  ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
  ROS_INFO_STREAM("Received first map message");


  ROS_INFO_STREAM("Wait for followPathAction to finish");
  // wait until either action was finished
  while (!action_finished_)
  {
    // if action was preempted, cancel everything
    if (!action_utils::actionOk(action_server))
    {
      follow_path_action_client_.cancelAllGoals();
      break;
    }
  }
}


PathExecutor::~PathExecutor()
{
  // wait until all callback are finished
  callback_mutex.lock();

  callback_mutex.unlock();
}


int PathExecutor::getFollowPathResult() const
{
  return result_;
}


void PathExecutor::mapCallback(const nav_msgs::OccupancyGrid& map)
{

  callback_mutex.lock();

  if (action_finished_)
  {
    ROS_WARN_STREAM("New map received, but action was already finished. Return.");
    callback_mutex.unlock();
    return;
  }

  nav_msgs::Path updated_path = updatePathOnNewMap(map, current_path_);

  // check if path is empty
  if (!first_map_cb_call_ && updated_path.poses.empty())
  {
    ROS_INFO_STREAM("Updated path was empty.");
    callback_mutex.unlock();
    return;
  }

  // check if updated path is same as current path. Only resend same path if last goal is too long ago
  if (!first_map_cb_call_ && updated_path.poses == current_path_.poses &&
      (ros::Time::now() - last_goal_stamp_) < resend_same_path_)
  {
    callback_mutex.unlock();
    return;
  }

  // TEMPORARY
  ROS_INFO_STREAM_NAMED(ROS_PACKAGE_NAME,
                        "Path was updated. New path contains " << updated_path.poses.size() << " poses.");

  for (auto& pose: updated_path.poses)
  {
    if (std::isnan(pose.pose.position.x) || std::isnan(pose.pose.position.y) || std::isnan(pose.pose.position.z) ||
        std::isnan(pose.pose.orientation.x) || std::isnan(pose.pose.orientation.y) ||
        std::isnan(pose.pose.orientation.z) || std::isnan(pose.pose.orientation.w))
    {
      std::string err_msg = "Updated path contains NaNs in position: " + utils::to_string(pose.pose.position)
                            + " or orientation: " + utils::to_string(pose.pose.orientation);
      ROS_ERROR_STREAM(err_msg);
      callback_mutex.unlock();
      throw std::runtime_error(err_msg);
    }
  }

  current_path_ = updated_path;

  follow_path_goal_.target_path = current_path_;
  follow_path_action_client_.sendGoal(follow_path_goal_,
                                      boost::bind(&PathExecutor::followPathDoneCallback, this, _1, _2));
  last_goal_stamp_ = ros::Time::now();

  updated_path_pub_.publish(current_path_);

  if (first_map_cb_call_)
  {
    first_map_cb_call_ = false;
  }

  callback_mutex.unlock();
}


nav_msgs::Path
PathExecutor::updatePathOnNewMap(const nav_msgs::OccupancyGrid& occupancy_grid, const nav_msgs::Path& path)
{
  if (path.poses.empty())
  {
    ROS_ERROR_STREAM("Cannot update empty path on new map!");
    return path;
  }


  // truncate path, so that only the not yet visited waypoints are considered

  // first nearest position in path
  double min_dist = DBL_MAX;
  int min_idx = 0;
  for (int i = 0; i < path.poses.size(); i++)
  {
    double dist = utils::distanceBetweenPositions(robot_pose_.pose.position, path.poses[i].pose.position);
    if (dist <= min_dist)
    {
      min_dist = dist;
      min_idx = i;
    }
  }


  // add all starting from min_idx to truncated path
  nav_msgs::Path truncated_path;
  truncated_path.header = path.header;
  truncated_path.poses.insert(truncated_path.poses.end(), path.poses.begin() + min_idx, path.poses.end());


  nav_msgs::Path updated_path;
  updated_path.header = truncated_path.header;

  // store map
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, std::string("occupancy"),
                                                   grid_map_);

  grid_map_planner_.setMap(grid_map_);


  // if the closest pose is too far away from the robot, the vehicle controller cannot execute it. So plan a path to first pose and add it
  if (min_dist > 0.3)
  {
    std::vector<geometry_msgs::PoseStamped> plan;
    grid_map_planner_.makePlan(robot_pose_.pose, truncated_path.poses[0].pose, plan);

    ROS_INFO_STREAM(
      "Distance to first pose was too large! Planned path to first pose with " << plan.size() << " poses.");

    // add old path to plan and replace in truncated path
    plan.insert(plan.end(), truncated_path.poses.begin(), truncated_path.poses.end());

    truncated_path.poses = plan;
  }


  // check if path is in collision
  grid_map::Polygon footprint;
  grid_map_polygon_tools::setFootprintPoly(robot_length_, robot_width_, footprint, base_frame_);

  // true = collision, false = no collision
  std::vector<bool> collision = grid_map_polygon_tools::isPathInCollisionPerPose(footprint, grid_map_, truncated_path);

  if (collision.size() != truncated_path.poses.size())
  {
    throw std::runtime_error(
      "Error when checking collision of path. Collision vector and poses vector had different sizes!");
  }

  // handle first and last waypoint extra (path.poses not empty checked earlier and same size checked above)
  if (collision[0])
  {
    // TODO handle first point in collision
    ROS_ERROR_STREAM(
      "First pose of path in collision! Pose: " << utils::to_string(truncated_path.poses[0].pose.position));
    return truncated_path;
  }
  if (collision.back())
  {
    // TODO handle last point in collision
    ROS_ERROR_STREAM(
      "Last pose of path in collision! Pose: " << utils::to_string(truncated_path.poses.back().pose.position));
    return truncated_path;
  }


  std::vector<std::pair<int, int>> blocked_path_parts;
  int max_gap_size = 5;

  for (int i = 0; i < truncated_path.poses.size(); ++i)
  {
    // in collision
    if (collision[i])
    {
      // if there is already a blocked part and it is less than max_gap_size poses ago, assume this blocked pose at part of last blocked part path
      if (!blocked_path_parts.empty() && (i - blocked_path_parts.back().second) < max_gap_size)
      {
        blocked_path_parts.back().second = i;
      }
      else
      {
        // list empty or last blocked path part is too long ago, add a new blocked part
        blocked_path_parts.emplace_back(i, i);
      }
    }
  }


  for (auto& b: blocked_path_parts)
  {
    ROS_INFO_STREAM("Blocked part found: First: " << b.first << ", second: " << b.second);
  }


  // if nothing is blocked, send truncated path as updated
  if (blocked_path_parts.empty())
  {
    updated_path.poses = truncated_path.poses;
  }
  else
  {
    // else add free parts and replan blocked ones

    int first_free = 0;
    for (auto& blocked: blocked_path_parts)
    {
      // insert all free poses (range [first free, blocked.first) )
      updated_path.poses.insert(updated_path.poses.end(),
                                truncated_path.poses.begin() + first_free,
                                truncated_path.poses.begin() + blocked.first - 1);

      // replan blocked part
      ROS_INFO_STREAM("Replanning partial path between "
                        << utils::to_string(truncated_path.poses[blocked.first - 1].pose.position)
                        << " and " << utils::to_string(truncated_path.poses[blocked.second + 1].pose.position) << " ("
                        << (blocked.second + 1 - blocked.first - 1 + 1)
                        << " poses).");

      std::vector<geometry_msgs::PoseStamped> plan;
      grid_map_planner_.makePlan(truncated_path.poses[blocked.first - 1].pose,
                                 truncated_path.poses[blocked.second + 1].pose, plan);

      ROS_INFO_STREAM("Replanned path contains " << plan.size() << " poses.");

      // add replanned path to updated path
      updated_path.poses.insert(updated_path.poses.end(), plan.begin(), plan.end());


      // update first free
      first_free = blocked.second + 2;
    }

    // add last path part (after last blocked until end)
    if (first_free < truncated_path.poses.size())
    {
      updated_path.poses.insert(updated_path.poses.end(),
                                truncated_path.poses.begin() + first_free,
                                truncated_path.poses.end());
    }
  }


  return updated_path;
}

void PathExecutor::followPathDoneCallback(const actionlib::SimpleClientGoalState& state,
                                          const move_base_lite_msgs::FollowPathResultConstPtr& result)
{

  using namespace std::chrono_literals;
  bool mutex_lock_result = callback_mutex.try_lock_for(500ms);

  // if mutex could not be locked in given time (i.e. map callback is currently running!), return so that map callback can send its new goal
  if (!mutex_lock_result)
  {
    ROS_WARN_STREAM("FollowPathDoneCallback: lock timed out.");
    return;
  }

  if (action_finished_)
  {
    ROS_WARN_STREAM("followPathDone received, but action was already finished. Return.");
    callback_mutex.unlock();
    return;
  }

  // TEMPORARY
  ROS_INFO_STREAM("follow path done: State: " << state.toString());
  ROS_INFO_STREAM("follow path done: State Text: " << state.getText());

  if (follow_path_action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {

    result_ = result->result.val;

    // TEMPORARY
    ROS_WARN_STREAM("Follow Path Result: " << result->result.val);
  }
  else if (follow_path_action_client_.getState() == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    if (num_recovery_tries_ < max_recovery_tries_)
    {
      // blocked: move backwards for short time and then try again
      stuckRecovery();
      num_recovery_tries_++;
      callback_mutex.unlock();
      return;
    }
    else
    {
      ROS_ERROR_STREAM("Waypoint could not be reached. Tried " << max_recovery_tries_
                                                               << " recoveries but is still blocked. Skip waypoint.");
    }
  }

  action_finished_ = true;

  callback_mutex.unlock();
}


void PathExecutor::robotPoseCallback(const geometry_msgs::PoseStamped& pose_msg)
{
  robot_pose_ = pose_msg;

  if (old_robot_poses_queue_.empty())
  {
    robot_pose_old_ = robot_pose_;
    old_robot_poses_queue_.push_back(robot_pose_);
  }
  else if (utils::distanceBetweenPositions(old_robot_poses_queue_.back().pose.position, robot_pose_.pose.position) >
           0.03)
  {
    old_robot_poses_queue_.push_back(robot_pose_);
  }

  if (utils::distanceBetweenPositions(robot_pose_old_.pose.position, robot_pose_.pose.position) > 0.1)
  {
    robot_pose_old_ = old_robot_poses_queue_.front();
    old_robot_poses_queue_.pop_front();
  }
}

void PathExecutor::stuckRecovery()
{
  ros::Duration recovery_time(0.5);
  double recovery_speed = 0.4;


  ros::Publisher cmd_vel_pub = pnh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, false);
  geometry_msgs::Twist msg;

  // transform last robot pose to base frame
  auto transform = TransformationHelper::getTransformation("world", base_frame_);
  geometry_msgs::Pose transformed_robot_pose_old;
  tf2::doTransform(robot_pose_old_.pose, transformed_robot_pose_old, transform);

  // drive towards old position
  if (transformed_robot_pose_old.position.x < 0)
  {
    msg.linear.x = -recovery_speed;
  }
  else
  {
    msg.linear.x = recovery_speed;
  }

  ros::Time start_time = ros::Time::now();

  ROS_WARN_STREAM_NAMED(ROS_PACKAGE_NAME, "Stuck recovery (" << num_recovery_tries_ << ". try)! Drive backwards for "
                                                             << recovery_time << "s with linear x speed "
                                                             << msg.linear.x << ". Old pose transformed: "
                                                             << utils::to_string(transformed_robot_pose_old.position));
  ROS_INFO_STREAM("Current robot pose: " << utils::to_string(robot_pose_.pose.position) << ", old robot pose: "
                                         << utils::to_string(robot_pose_old_.pose.position));

  while ((ros::Time::now() - start_time) < recovery_time)
  {
    cmd_vel_pub.publish(msg);
  }

  // send 0 to stop robot
  msg.linear.x = 0;
  cmd_vel_pub.publish(msg);

  // resend updated path
  follow_path_action_client_.sendGoal(follow_path_goal_,
                                      boost::bind(&PathExecutor::followPathDoneCallback, this, _1, _2));
  last_goal_stamp_ = ros::Time::now();
}
} // end namespace three_dimensional_coverage_path_planning