#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from three_dimensional_coverage_path_planning_msgs.msg import MoveToWaypointOf3dCoveragePathAction, MoveToWaypointOf3dCoveragePathGoal, MoveToWaypointOf3dCoveragePathFeedback

from actionlib_msgs.msg import GoalStatus
from rospy import Time

'''
Created on 24.03.2022

@author: Katrin Becker
'''


class ThreeDimensionalCoveragePathPlanningMoveToWaypointState(EventState):
    '''
    Move to next waypoint by following the given path. Uses the MoveToWaypointOf3dCoveragePath action.
    
    ># path             nav_msgs/Path   Path to follow to reach the next waypoint.
    
    -- speed            float           Desired speed for move base lite. If left at 0.0, defaults if the controller are used.

    <= success                          Everything worked correctly.
    <= failed                           An error occurred, e.g. goal was blocked or move to waypoint action was not allowed.
    '''

    def __init__(self, speed = 0.0):
        '''
        Constructor
        '''
        super(ThreeDimensionalCoveragePathPlanningMoveToWaypointState, self).__init__(outcomes=['success', 'failed'],
                                                                                       input_keys=['path'])

        self._action_topic = '/three_dimensional_coverage_path_planning/move_to_waypoint'
        self._action_client = ProxyActionClient({self._action_topic: MoveToWaypointOf3dCoveragePathAction})

        self._speed = speed        
        
        self._failed = False
        self._success = False

    def execute(self, userdata):
        '''
        Execute this state
        '''

        if self._failed:
            return 'failed'
        if self._success:
            return 'success'

        if self._action_client.has_feedback(self._action_topic):
            # get and remove feedback (remove it, so that it is only printed if there is new feedback)
            feedback = self._action_client.get_feedback(self._action_topic)
            self._action_client.remove_feedback(self._action_topic)

            Logger.loginfo('[ThreeDimensionalCoveragePathPlanningMoveToWaypointState] Progress of MoveToWaypointOf3dCoveragePath action: %s' % str(feedback.progress))

        if self._action_client.has_result(self._action_topic):

            status = self._action_client.get_state(self._action_topic)

            if status == GoalStatus.SUCCEEDED:

                result = self._action_client.get_result(self._action_topic)
                
                
                Logger.loginfo('[ThreeDimensionalCoveragePathPlanningMoveToWaypointState] Result of move to waypoint action: %s' % str(result.result.value))
                
                
                self._success = True


            else:
                Logger.logwarn('Action failed: %s' % str(status))
                self._failed = True

    def on_enter(self, userdata):

        self._failed = False
        self._success = False
        
        Logger.loginfo('[ThreeDimensionalCoveragePathPlanningMoveToWaypointState] Try to move to next waypoint by following the given path.')

        action_goal = MoveToWaypointOf3dCoveragePathGoal()
        action_goal.path_to_waypoint = userdata.path
        
        
        # See move_base_lite_msgs/FollowPathOptions.msg for documentation of parameters.
        action_goal.follow_path_options.desired_speed = self._speed
        action_goal.follow_path_options.is_fixed = False
        action_goal.follow_path_options.goal_pose_position_tolerance = 0.1  # If left at 0.0, defaults of the controller are used.
        action_goal.follow_path_options.goal_pose_angle_tolerance = 0.1
        action_goal.follow_path_options.rotate_front_to_goal_pose_orientation = True
        action_goal.follow_path_options.reverse_allowed = True
        action_goal.follow_path_options.reverse_forced = False
        action_goal.follow_path_options.reset_stuck_history = False
        

        try:
            self._action_client.send_goal(self._action_topic, action_goal)

        except Exception as e:
            Logger.logwarn('[ThreeDimensionalCoveragePathPlanningMoveToWaypointState] Failed when sending goal. Error :\n%s' % str(e))
            self._failed = True

    def on_stop(self):
        try:
            if self._action_client.is_available(self._action_topic) \
                    and not self._action_client.has_result(self._action_topic):
                self._action_client.cancel(self._action_topic)
        except:
            # client already closed
            pass

    def on_exit(self, userdata):
        try:
            if self._action_client.is_available(self._action_topic) and not self._action_client.has_result(self._action_topic):
                self._action_client.cancel(self._action_topic)
        except:
            # client already closed
            pass

    def on_pause(self):
        self._action_client.cancel(self._action_topic)

    def on_resume(self, userdata):
        self.on_enter(userdata)
