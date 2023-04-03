

#include "three_dimensional_coverage_path_planning/three_dimensional_coverage_path_planning.h"

#include "three_dimensional_coverage_path_planning/utils/utils_with_types.h"
#include "three_dimensional_coverage_path_planning/utils/transformation_helper.h"

#include <dynamic_reconfigure/Reconfigure.h>

namespace three_dimensional_coverage_path_planning
{

ThreeDimensionalCoveragePathPlanning::ThreeDimensionalCoveragePathPlanning(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), data_recorder_loader_("three_dimensional_coverage_path_planning",
                                              "three_dimensional_coverage_path_planning::DataRecorderBase")
{
}

void
ThreeDimensionalCoveragePathPlanning::plan3dCoveragePathOnMesh(bool load_models, bool save_models, bool load_path,
                                                               PlanActionServer* const action_server)
{

  PlanActionFeedback feedback;

  if (action_server)
  {
    feedback.progress = load_models ? "Loading models" : "Computing models";
    action_server->publishFeedback(feedback);
  }


  // init building model
  building_model_ = std::make_unique<BuildingModel>(nh_, pnh_, load_models, save_models);

  if (!action_utils::actionOk(action_server))
  {
    return;
  }

  if (load_path)
  {
    if (action_server)
    {
      feedback.progress = "Loading path";
      action_server->publishFeedback(feedback);
    }

    // TODO load path
  }
  else
  {
    if (action_server)
    {
      feedback.progress = "Computing viewpoints";
      action_server->publishFeedback(feedback);
    }

    viewpoint_computer_ = std::make_unique<ViewpointComputer>(building_model_->getCompleteModel(),
                                                              building_model_->getTargetSet(),
                                                              action_server);
    selected_viewpoints_ = viewpoint_computer_->computeViewpoints();

    if (!action_utils::actionOk(action_server))
    {
      return;
    }

    if (action_server)
    {
      feedback.progress = "Converting viewpoints to waypoints";
      action_server->publishFeedback(feedback);
    }

    convertViewpointsToWaypoints();

    //TEMPORARY print waypoints_to_viewpoints
//    ROS_WARN_STREAM("waypoints to viewpoints:\n");
//    for (auto& entry: waypoints_to_viewpoints_)
//    {
//      ROS_INFO_STREAM("waypoint: " << utils::to_string(entry.first.position) <<
//                                   ", viewpoint: " << utils::to_string(entry.second.getPose().position)
//                                   << ", in frame: "
//                                   << entry.second.getPose().frame_id);
//    }


    if (!action_utils::actionOk(action_server))
    {
      return;
    }


    if (action_server)
    {
      feedback.progress = "Computing waypoint order";
      action_server->publishFeedback(feedback);
    }

    waypoint_order_computer_ = std::make_unique<WaypointOrderComputer>(pnh_, building_model_->getCompleteModel());
    path_scanpoints_ = waypoint_order_computer_->computeWaypointOrder(waypoints_unordered_);

    paths_between_waypoints_ = waypoint_order_computer_->getPathsBetweenWaypoints();

    if (!action_utils::actionOk(action_server))
    {
      return;
    }
  }

  if (action_server)
  {
    feedback.progress = "Path planning finished";
    action_server->publishFeedback(feedback);
  }
}

three_dimensional_coverage_path_planning_msgs::Path ThreeDimensionalCoveragePathPlanning::getPath(bool viewpoint_info)
{
  three_dimensional_coverage_path_planning_msgs::Path path;

  // get model frame id
  std::string model_frame = building_model_->getModelFrame();
  path.header.frame_id = model_frame;

  for (int i = 0; i < path_scanpoints_.size(); ++i)
  {

    // viewpoint message
    three_dimensional_coverage_path_planning_msgs::Viewpoint vp_msg;

    auto viewpoint = waypoints_to_viewpoints_.at(path_scanpoints_[i]);
    vp_msg.pose = utils::toMsgPoseStamped(viewpoint.getPose());

    // get additional viewpoint info if requested (e.g. for recovery)
    if (viewpoint_info)
    {
      vp_msg.reward = viewpoint.getReward();

      vp_msg.all_visible_targets = utils::toMsgPointStampedArray(viewpoint.getAllVisibleTargets(), model_frame);

      vp_msg.targets_seen_from_this_viewpoint = utils::toMsgPointStampedArray(
        viewpoint.getTargetsSeenFromThisViewpoint(), model_frame);

      vp_msg.targets_seen_from_other_viewpoints = utils::toMsgPointStampedArray(
        viewpoint.getTargetsSeenFromOtherViewpoint(), model_frame);
    }



    // waypoint message
    three_dimensional_coverage_path_planning_msgs::Waypoint wp_msg;

    wp_msg.waypoint = utils::toMsgPoseStamped(path_scanpoints_[i]);
    wp_msg.viewpoint = vp_msg;

    // get path (only if not first waypoint)
    if (i > 0)
    {
      wp_msg.path_to_waypoint = paths_between_waypoints_.at({
                                                              utils::toMsgPoseStamped(path_scanpoints_[i - 1]),
                                                              utils::toMsgPoseStamped(path_scanpoints_[i])
                                                            });
    }

    path.waypoints.push_back(wp_msg);
  }

  return path;
}


void ThreeDimensionalCoveragePathPlanning::initializeExecutionOfPath(
  const three_dimensional_coverage_path_planning_msgs::Path& path)
{

  // Set unknown space to free in grid map, as grid map planner otherwise has problems if a goal is outside of the known space.
  // Additionally, if there is no obstacle behind the free space, e.g. in the entrance, it is not marked as free otherwise.
  ros::ServiceClient map_params_service_client = pnh_.serviceClient<dynamic_reconfigure::Reconfigure>(
    "/ethz_grid_map_proc/set_parameters");

  dynamic_reconfigure::Reconfigure map_param_srv;
  dynamic_reconfigure::BoolParameter map_param_unknown_to_free;
  map_param_unknown_to_free.name = "unknown_space_to_free";
  map_param_unknown_to_free.value = true;
  map_param_srv.request.config.bools.push_back(map_param_unknown_to_free);

  map_params_service_client.call(map_param_srv);


  if (!path.waypoints.empty())
  {
    // build complete path and publish it
    complete_original_path_pub_ = pnh_.advertise<nav_msgs::Path>("complete_original_path", 100, true);

    nav_msgs::Path vis_path_msg;
    vis_path_msg.header.frame_id = path.waypoints.back().path_to_waypoint.header.frame_id;

    vis_path_msg.poses.push_back(path.waypoints[0].waypoint);
    for (unsigned int i = 1; i < path.waypoints.size(); ++i)
    {
      vis_path_msg.poses.insert(vis_path_msg.poses.end(), path.waypoints[i].path_to_waypoint.poses.begin(),
                                path.waypoints[i].path_to_waypoint.poses.end());
    }

    complete_original_path_pub_.publish(vis_path_msg);



    // publish waypoints
    waypoints_pub_ = pnh_.advertise<visualization_msgs::Marker>("waypoints", 100, true);

    visualization_msgs::Marker msg;
    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::SPHERE_LIST;
    msg.pose.orientation.w = 1;
    msg.scale.x = 0.125;
    msg.scale.y = 0.125;
    msg.scale.z = 0.125;

    msg.header.frame_id = path.waypoints[0].waypoint.header.frame_id;
    msg.id = 15;

    colors::ColorGenerator color_gen(static_cast<int>(path.waypoints.size()));

    for (auto& wp: path.waypoints)
    {
      msg.points.push_back(wp.waypoint.pose.position);
      msg.colors.push_back(color_gen.nextColor());
    }

    waypoints_pub_.publish(msg);
  }


  // init real world model
  real_world_model_ = std::make_unique<RealWorldModel>(nh_, pnh_);

  ROS_INFO_STREAM("Real world model initialized.");

  // load data recorder

  // BUG somewhere in the next three lines sometimes a segmentation fault occurs (message "Data recorder initialized" does not appear) when started a second time (behavior started, error when moving to waypoint, behavior started again).
  std::string recorder_name = pnh_.param<std::string>("data_recording/data_recorder_plugin",
                                                      "three_dimensional_coverage_path_planning::LidarDataRecorder");
  data_recorder_.reset(data_recorder_loader_.createUnmanagedInstance(recorder_name));

  data_recorder_->initialize(pnh_);

  ROS_INFO_STREAM("Data recorder initialized.");
}


void ThreeDimensionalCoveragePathPlanning::waypointReached(
  const three_dimensional_coverage_path_planning_msgs::Waypoint& waypoint)
{
  ROS_INFO_STREAM("ThreeDimensionalCoveragePathPlanning::waypointReached");

  data_recorder_->recordData(waypoint.waypoint);

  // TODO recovery: check if all expected targets have been covered (with tolerance)
}

void
ThreeDimensionalCoveragePathPlanning::recover(const three_dimensional_coverage_path_planning_msgs::Waypoint& waypoint,
                                              bool waypoint_reached)
{
  ROS_INFO_STREAM("ThreeDimensionalCoveragePathPlanning::recover");

  // Note: make robust in checks and feedbackloop/decisions

  // TODO implement ThreeDimensionalCoveragePathPlanning::recover
}

std::string ThreeDimensionalCoveragePathPlanning::finishExecutionOfPath()
{
  ROS_WARN_STREAM("ThreeDimensionalCoveragePathPlanning::finishExecutionOfPath");

  data_recorder_->finish();

  return data_recorder_->getDirectory();
}


void ThreeDimensionalCoveragePathPlanning::convertViewpointsToWaypoints()
{

  TransformationHelper transformation_helper(pnh_);

  waypoints_to_viewpoints_.clear();
  waypoints_unordered_.clear();

  // convert viewpoints using transformation from above
  for (auto& viewpoint: *selected_viewpoints_)
  {
    auto waypoint = transformation_helper.transformViewpointToWaypoint(viewpoint);

    waypoints_to_viewpoints_.emplace(waypoint, viewpoint);
    waypoints_unordered_.push_back(waypoint);
  }
}
}// end namespace three_dimensional_coverage_path_planning