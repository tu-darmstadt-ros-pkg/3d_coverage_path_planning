#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_PATH_EXECUTOR_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_PATH_EXECUTOR_H

#include "three_dimensional_coverage_path_planning/utils/action_types.h"

#include <nav_msgs/OccupancyGrid.h>

#include <move_base_lite_msgs/FollowPathAction.h>
#include <actionlib/client/simple_action_client.h>

#include <grid_map_planner_lib/grid_map_planner.h>

#include <ros/ros.h>


namespace three_dimensional_coverage_path_planning
{

class PathExecutor
{
public:

  PathExecutor(const nav_msgs::Path& path, const move_base_lite_msgs::FollowPathOptions& follow_path_options,
               MoveToWaypointActionServer* const action_server = nullptr);

  ~PathExecutor();

  int getFollowPathResult() const;

private:

  void mapCallback(const nav_msgs::OccupancyGrid& map);

  nav_msgs::Path updatePathOnNewMap(const nav_msgs::OccupancyGrid& occupancy_grid, const nav_msgs::Path& path);

  void followPathDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const move_base_lite_msgs::FollowPathResultConstPtr& result);

  void robotPoseCallback(const geometry_msgs::PoseStamped& pose_msg);


  void stuckRecovery();


  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;


  bool first_map_cb_call_;

  bool action_finished_;

  std::timed_mutex callback_mutex;

  int result_{};

  ros::Subscriber map_subscriber_;

  ros::Subscriber robot_pose_subscriber_;
  geometry_msgs::PoseStamped robot_pose_;
  geometry_msgs::PoseStamped robot_pose_old_;
  std::deque<geometry_msgs::PoseStamped> old_robot_poses_queue_;

  int max_recovery_tries_ = 5;
  int num_recovery_tries_ = 0;


  ros::Publisher original_path_pub_;
  ros::Publisher updated_path_pub_;

  grid_map::GridMap grid_map_;
  grid_map_planner::GridMapPlanner grid_map_planner_;


  actionlib::SimpleActionClient<move_base_lite_msgs::FollowPathAction> follow_path_action_client_;
  move_base_lite_msgs::FollowPathGoal follow_path_goal_;

  ros::Time last_goal_stamp_;
  ros::Duration resend_same_path_;

  nav_msgs::Path current_path_;

  std::string base_frame_;
  double robot_length_;
  double robot_width_;
};
} // end namespace three_dimensional_coverage_path_planning

#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_PATH_EXECUTOR_H
