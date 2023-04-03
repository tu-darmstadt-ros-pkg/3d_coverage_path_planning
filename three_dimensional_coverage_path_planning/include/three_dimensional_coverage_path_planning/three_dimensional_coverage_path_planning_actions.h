#ifndef THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_ACTIONS_H
#define THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_ACTIONS_H


#include "three_dimensional_coverage_path_planning/utils/action_types.h"

#include "three_dimensional_coverage_path_planning/three_dimensional_coverage_path_planning.h"

#include <std_srvs/Empty.h>

namespace three_dimensional_coverage_path_planning
{

/**
 * Class that contains and handles all the action server and callbacks for 3D coverage path planning
 */
class ThreeDimensionalCoveragePathPlanningActions
{

public:
  ThreeDimensionalCoveragePathPlanningActions(ros::NodeHandle& nh, ros::NodeHandle& pnh);


private:

  void plan3dCoveragePathOnMeshCallback(const PlanActionGoal& goal);

  void initializeExecutionOfPathCallback(const InitializeExecutionActionGoal& goal);

  void moveToWaypointCallback(const MoveToWaypointActionGoal& goal);

  void waypointReachedCallback(const WaypointReachedActionGoal& goal);

  void executeRecoveryCallback(const RecoveryActionGoal& goal);

  void finishExecutionOfPathCallback(const FinishExecutionActionGoal& goal);

  bool resetState(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);


  ThreeDimensionalCoveragePathPlanning coverage_path_planning_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;


  // action servers
  PlanActionServer plan_action_server_;

  InitializeExecutionActionServer initialize_execution_action_server_;
  MoveToWaypointActionServer move_to_waypoint_action_server_;
  WaypointReachedActionServer waypoint_reached_action_server_;
  RecoveryActionServer recovery_action_server_;
  FinishExecutionActionServer finish_execution_action_server_;


  ros::ServiceServer reset_state_service_;


  /**
   * State idle and path_execution: no action callback is currently running.
   * Transitions: see behind the state constants.
   * Only the given transitions are allowed, otherwise the respective action is rejected/aborted
   */
  int state_; /// Current state

  const int IDLE = 0; /// idle --> planning; idle --> initialize
  const int PATH_EXECUTION = 1; /// path_execution --> move_to_waypoint; path_execution --> waypoint_reached; path_execution --> recovery; path_execution --> finish

  const int PLANNING = 2; /// planning --> idle

  const int INITIALIZE = 3; /// initialize --> path_execution
  const int MOVE_TO_WAYPOINT = 4; /// move_to_waypoint --> path_execution
  const int WAYPOINT_REACHED = 5; /// waypoint_reached --> path_execution
  const int RECOVERY = 6; /// recovery --> path_execution
  const int FINISH = 7; /// finish --> idle


};
} // end namespace three_dimensional_coverage_path_planning


#endif //THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_THREE_DIMENSIONAL_COVERAGE_PATH_PLANNING_ACTIONS_H
