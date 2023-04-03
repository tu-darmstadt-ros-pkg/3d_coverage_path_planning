#include "three_dimensional_coverage_path_planning/three_dimensional_coverage_path_planning_actions.h"
#include "three_dimensional_coverage_path_planning/path_execution/path_executor.h"

namespace three_dimensional_coverage_path_planning
{

ThreeDimensionalCoveragePathPlanningActions::ThreeDimensionalCoveragePathPlanningActions(ros::NodeHandle& nh,
                                                                                         ros::NodeHandle& pnh)
  : coverage_path_planning_(nh, pnh), nh_(nh), pnh_(pnh),

    plan_action_server_(pnh, "plan_path", boost::bind(
      &ThreeDimensionalCoveragePathPlanningActions::plan3dCoveragePathOnMeshCallback, this, _1), false),

    initialize_execution_action_server_(pnh, "initialize_execution_of_path", boost::bind(
      &ThreeDimensionalCoveragePathPlanningActions::initializeExecutionOfPathCallback, this, _1), false),
    move_to_waypoint_action_server_(pnh, "move_to_waypoint", boost::bind(
      &ThreeDimensionalCoveragePathPlanningActions::moveToWaypointCallback, this, _1), false),
    waypoint_reached_action_server_(pnh, "waypoint_reached", boost::bind(
      &ThreeDimensionalCoveragePathPlanningActions::waypointReachedCallback, this, _1), false),
    recovery_action_server_(pnh, "execute_recovery", boost::bind(
      &ThreeDimensionalCoveragePathPlanningActions::executeRecoveryCallback, this, _1), false),
    finish_execution_action_server_(pnh, "finish_execution_of_path", boost::bind(
      &ThreeDimensionalCoveragePathPlanningActions::finishExecutionOfPathCallback, this, _1), false)
{
  // Start all action servers
  plan_action_server_.start();
  initialize_execution_action_server_.start();
  move_to_waypoint_action_server_.start();
  waypoint_reached_action_server_.start();
  recovery_action_server_.start();
  finish_execution_action_server_.start();

  state_ = IDLE;

  reset_state_service_ = pnh_.advertiseService("reset_state", &ThreeDimensionalCoveragePathPlanningActions::resetState, this);
}


void ThreeDimensionalCoveragePathPlanningActions::plan3dCoveragePathOnMeshCallback(const PlanActionGoal& goal)
{
  // check if action is allowed
  if (state_ != IDLE)
  {
    // Note: since FlexBE does not seem to use the passed text, it is additionally sent as a feedback.
    PlanActionFeedback feedback;
    feedback.progress = "Planning cannot be started when not in state idle! State was: " + std::to_string(state_);
    plan_action_server_.publishFeedback(feedback);
    plan_action_server_.setAborted(PlanActionResult(), feedback.progress);
    ROS_WARN_STREAM_NAMED(ROS_PACKAGE_NAME, feedback.progress);
    return;
  }

  state_ = PLANNING;

  ROS_INFO_STREAM_NAMED(ROS_PACKAGE_NAME, "Plan 3D coverage path on mesh started.");


  try
  {

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


    // compute/load models and plan, do all precomputations
    coverage_path_planning_.plan3dCoveragePathOnMesh(goal->load_models, true, false, &plan_action_server_);


    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    ROS_WARN_STREAM("Plan 3D coverage path: elapsed time: "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]");
  }
  catch (std::exception& e)
  {
    PlanActionFeedback feedback;
    feedback.progress = std::string("Exception thrown while planning path on mesh: ") + e.what();
    plan_action_server_.publishFeedback(feedback);
    plan_action_server_.setAborted(PlanActionResult(), feedback.progress);
    ROS_ERROR_STREAM_NAMED(ROS_PACKAGE_NAME, feedback.progress);

    state_ = IDLE;
    return;
  }

  if (!action_utils::actionOk(&plan_action_server_))
  {
    ROS_WARN_STREAM_NAMED(ROS_PACKAGE_NAME, "Action plan3dCoveragePathOnMesh was preempted.");
    plan_action_server_.setPreempted();
    state_ = IDLE;
    return;
  }

  PlanActionResult result;

  result.path = coverage_path_planning_.getPath();
  result.path_without_viewpoint_information = coverage_path_planning_.getPath(false);

  ROS_INFO_STREAM_NAMED(ROS_PACKAGE_NAME, "Path planning finished.");

  ROS_INFO_STREAM_NAMED(ROS_PACKAGE_NAME,
                        "Plan 3D coverage path on mesh finished. Path has " << result.path.waypoints.size()
                                                                            << " waypoints.");

  plan_action_server_.setSucceeded(result);

  state_ = IDLE;
}


void ThreeDimensionalCoveragePathPlanningActions::initializeExecutionOfPathCallback(
  const InitializeExecutionActionGoal& goal)
{
  // check if action is allowed
  if (state_ != IDLE)
  {
    InitializeExecutionActionFeedback feedback;
    feedback.progress =
      "Path execution cannot be initialized when not in state idle! State was: " + std::to_string(state_);
    initialize_execution_action_server_.publishFeedback(feedback);
    initialize_execution_action_server_.setAborted(InitializeExecutionActionResult(), feedback.progress);
    ROS_WARN_STREAM_NAMED(ROS_PACKAGE_NAME, feedback.progress);
    return;
  }

  state_ = INITIALIZE;

  // TODO pass action server and check if action was preempted
  coverage_path_planning_.initializeExecutionOfPath(goal->path);

  initialize_execution_action_server_.setSucceeded(InitializeExecutionActionResult());

  state_ = PATH_EXECUTION;
  
}


void ThreeDimensionalCoveragePathPlanningActions::moveToWaypointCallback(const MoveToWaypointActionGoal& goal)
{
  // check if action is allowed
  if (state_ != PATH_EXECUTION)
  {
    MoveToWaypointActionFeedback feedback;
    feedback.progress =
      "Move to waypoint action cannot be started when not in state path_execution! State was: " +
      std::to_string(state_);
    move_to_waypoint_action_server_.publishFeedback(feedback);
    move_to_waypoint_action_server_.setAborted(MoveToWaypointActionResult(), feedback.progress);
    ROS_WARN_STREAM_NAMED(ROS_PACKAGE_NAME, feedback.progress);
    return;
  }

  state_ = MOVE_TO_WAYPOINT;

  PathExecutor path_executor(goal->path_to_waypoint, goal->follow_path_options, &move_to_waypoint_action_server_);

  auto path_executor_result = path_executor.getFollowPathResult();

  ROS_INFO_STREAM("Result of path executor was: " << path_executor_result);

  auto action_result = MoveToWaypointActionResult();

  if(path_executor_result == move_base_lite_msgs::ErrorCodes::SUCCESS)
  {
    action_result.result.value = three_dimensional_coverage_path_planning_msgs::MoveResultCode::SUCCESS;
    move_to_waypoint_action_server_.setSucceeded(action_result);
  }
  else
  {
    ROS_ERROR_STREAM("Move base lite: follow path action resulted with error code: " << action_result);
    action_result.result.value = three_dimensional_coverage_path_planning_msgs::MoveResultCode::GOAL_BLOCKED;
    move_to_waypoint_action_server_.setAborted(action_result);
    state_ = PATH_EXECUTION;
    return;
  }

  state_ = PATH_EXECUTION;
}


void ThreeDimensionalCoveragePathPlanningActions::waypointReachedCallback(
  const WaypointReachedActionGoal& goal)
{
  // check if action is allowed
  if (state_ != PATH_EXECUTION)
  {
    WaypointReachedActionFeedback feedback;
    feedback.progress =
      "Waypoint reached action cannot be started when not in state path_execution! State was: " +
      std::to_string(state_);
    waypoint_reached_action_server_.publishFeedback(feedback);
    waypoint_reached_action_server_.setAborted(WaypointReachedActionResult(), feedback.progress);
    ROS_WARN_STREAM_NAMED(ROS_PACKAGE_NAME, feedback.progress);
    return;
  }

  state_ = WAYPOINT_REACHED;

  // TODO pass action server and check if action was preempted
  coverage_path_planning_.waypointReached(goal->waypoint);

  waypoint_reached_action_server_.setSucceeded(WaypointReachedActionResult());

  state_ = PATH_EXECUTION;
}

void ThreeDimensionalCoveragePathPlanningActions::executeRecoveryCallback(const RecoveryActionGoal& goal)
{
  // check if action is allowed
  if (state_ != PATH_EXECUTION)
  {
    RecoveryActionFeedback feedback;
    feedback.progress =
      "Recovery action cannot be started when not in state path_execution! State was: " + std::to_string(state_);
    recovery_action_server_.publishFeedback(feedback);
    recovery_action_server_.setAborted(RecoveryActionResult(), feedback.progress);
    ROS_WARN_STREAM_NAMED(ROS_PACKAGE_NAME, feedback.progress);
    return;
  }

  state_ = RECOVERY;

  // TODO pass action server and check if action was preempted
  coverage_path_planning_.recover(goal->waypoint, goal->waypoint_reached);

  recovery_action_server_.setSucceeded(RecoveryActionResult());

  state_ = PATH_EXECUTION;
}

void ThreeDimensionalCoveragePathPlanningActions::finishExecutionOfPathCallback(
  const FinishExecutionActionGoal& goal)
{
  // check if action is allowed --> make this action always allowed so that it can be called manually in case of an error e.g. in pcl writer
//  if (state_ != PATH_EXECUTION && state_ != IDLE)
//  {
//    FinishExecutionActionFeedback feedback;
//    feedback.progress =
//      "Path execution cannot be finished when not in state path_execution! State was: " + std::to_string(state_);
//    finish_execution_action_server_.publishFeedback(feedback);
//    finish_execution_action_server_.setAborted(FinishExecutionActionResult(), feedback.progress);
//    ROS_WARN_STREAM_NAMED(ROS_PACKAGE_NAME, feedback.progress);
//    return;
//  }

  state_ = FINISH;

  FinishExecutionActionResult result;

  // TODO pass action server and check if action was preempted
  result.recorded_data_directory = coverage_path_planning_.finishExecutionOfPath();

  finish_execution_action_server_.setSucceeded(result);

  state_ = IDLE;
}


bool ThreeDimensionalCoveragePathPlanningActions::resetState(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  state_ = IDLE;
  ROS_INFO_STREAM("State reset.");
  return true;
}

} // end namespace three_dimensional_coverage_path_planning