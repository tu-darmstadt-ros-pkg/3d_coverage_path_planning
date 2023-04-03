#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_ACTION_TYPES_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_ACTION_TYPES_H

#include <three_dimensional_coverage_path_planning_msgs/Plan3dCoveragePathOnMeshAction.h>

#include <three_dimensional_coverage_path_planning_msgs/InitializeExecutionOf3dCoveragePathAction.h>
#include <three_dimensional_coverage_path_planning_msgs/MoveToWaypointOf3dCoveragePathAction.h>
#include <three_dimensional_coverage_path_planning_msgs/WaypointOf3dCoveragePathReachedAction.h>
#include <three_dimensional_coverage_path_planning_msgs/ExecuteRecoveryOn3dCoveragePathAction.h>
#include <three_dimensional_coverage_path_planning_msgs/FinishExecutionOf3dCoveragePathAction.h>

#include <actionlib/server/simple_action_server.h>

namespace three_dimensional_coverage_path_planning
{

// plan 3d coverage path on mesh
using PlanActionServer = actionlib::SimpleActionServer<three_dimensional_coverage_path_planning_msgs::Plan3dCoveragePathOnMeshAction>;

using PlanActionGoal = three_dimensional_coverage_path_planning_msgs::Plan3dCoveragePathOnMeshGoalConstPtr;
using PlanActionResult = three_dimensional_coverage_path_planning_msgs::Plan3dCoveragePathOnMeshResult;
using PlanActionFeedback = three_dimensional_coverage_path_planning_msgs::Plan3dCoveragePathOnMeshFeedback;


// init execution of 3d coverage path
using InitializeExecutionActionServer = actionlib::SimpleActionServer<three_dimensional_coverage_path_planning_msgs::InitializeExecutionOf3dCoveragePathAction>;
using InitializeExecutionActionGoal = three_dimensional_coverage_path_planning_msgs::InitializeExecutionOf3dCoveragePathGoalConstPtr;
using InitializeExecutionActionResult = three_dimensional_coverage_path_planning_msgs::InitializeExecutionOf3dCoveragePathResult;
using InitializeExecutionActionFeedback = three_dimensional_coverage_path_planning_msgs::InitializeExecutionOf3dCoveragePathFeedback;


// move to waypoint
using MoveToWaypointActionServer = actionlib::SimpleActionServer<three_dimensional_coverage_path_planning_msgs::MoveToWaypointOf3dCoveragePathAction>;
using MoveToWaypointActionGoal = three_dimensional_coverage_path_planning_msgs::MoveToWaypointOf3dCoveragePathGoalConstPtr;
using MoveToWaypointActionResult = three_dimensional_coverage_path_planning_msgs::MoveToWaypointOf3dCoveragePathResult;
using MoveToWaypointActionFeedback = three_dimensional_coverage_path_planning_msgs::MoveToWaypointOf3dCoveragePathFeedback;


// waypoint reached
using WaypointReachedActionServer = actionlib::SimpleActionServer<three_dimensional_coverage_path_planning_msgs::WaypointOf3dCoveragePathReachedAction>;
using WaypointReachedActionGoal = three_dimensional_coverage_path_planning_msgs::WaypointOf3dCoveragePathReachedGoalConstPtr;
using WaypointReachedActionResult = three_dimensional_coverage_path_planning_msgs::WaypointOf3dCoveragePathReachedResult;
using WaypointReachedActionFeedback = three_dimensional_coverage_path_planning_msgs::WaypointOf3dCoveragePathReachedFeedback;


// execute recovery
using RecoveryActionServer = actionlib::SimpleActionServer<three_dimensional_coverage_path_planning_msgs::ExecuteRecoveryOn3dCoveragePathAction>;
using RecoveryActionGoal = three_dimensional_coverage_path_planning_msgs::ExecuteRecoveryOn3dCoveragePathGoalConstPtr;
using RecoveryActionResult = three_dimensional_coverage_path_planning_msgs::ExecuteRecoveryOn3dCoveragePathResult;
using RecoveryActionFeedback = three_dimensional_coverage_path_planning_msgs::ExecuteRecoveryOn3dCoveragePathFeedback;




// finish execution of 3d coverage path
using FinishExecutionActionServer = actionlib::SimpleActionServer<three_dimensional_coverage_path_planning_msgs::FinishExecutionOf3dCoveragePathAction>;
using FinishExecutionActionGoal = three_dimensional_coverage_path_planning_msgs::FinishExecutionOf3dCoveragePathGoalConstPtr;
using FinishExecutionActionResult = three_dimensional_coverage_path_planning_msgs::FinishExecutionOf3dCoveragePathResult;
using FinishExecutionActionFeedback = three_dimensional_coverage_path_planning_msgs::FinishExecutionOf3dCoveragePathFeedback;


namespace action_utils
{

/**
 * Checks if action server is still ok.
 * @param action_server
 * @return true if action_server is null or it exists and it is active and not preempted or aborted.
 */
template<typename T>
static bool actionOk(actionlib::SimpleActionServer<T>* const action_server)
{
  if (!action_server || (action_server->isActive() && !action_server->isPreemptRequested()))
  {
    return true;
  }

  // TEMPORARY print
  {
    ROS_WARN_STREAM("Action not active or preempted.");
  }

  return false;
}

}
} // end namespace three_dimensional_coverage_path_planning

#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_ACTION_TYPES_H
