#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from three_dimensional_coverage_path_planning_msgs.msg import ExecuteRecoveryOn3dCoveragePathAction, ExecuteRecoveryOn3dCoveragePathGoal, ExecuteRecoveryOn3dCoveragePathFeedback
from three_dimensional_coverage_path_planning_msgs.msg import Waypoint

from actionlib_msgs.msg import GoalStatus
from rospy import Time

'''
Created on 09.03.2022

@author: Katrin Becker
'''


class ThreeDimensionalCoveragePathPlanningRecoveryState(EventState):
    '''
    Call recovery, e.g. when a waypoint could not be reached or not all expected target points have been covered.
    Uses the FinishExecutionOf3dCoveragePath action.

    <= success                          Everything worked correctly.
    <= failed                           An error occurred or recovery action was not allowed.
    '''

    def __init__(self):
        '''
        Constructor
        '''
        super(ThreeDimensionalCoveragePathPlanningRecoveryState, self).__init__(outcomes=['success', 'failed'],
                                                                                input_keys=['waypoint',
                                                                                            'waypoint_reached'])

        self._action_topic = '/three_dimensional_coverage_path_planning/execute_recovery'
        self._action_client = ProxyActionClient({self._action_topic: ExecuteRecoveryOn3dCoveragePathAction})

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

            Logger.loginfo('Progress of ExecuteRecoveryOn3dCoveragePath action: %s' % str(feedback.progress))

        if self._action_client.has_result(self._action_topic):

            status = self._action_client.get_state(self._action_topic)

            if status == GoalStatus.SUCCEEDED:

                # result = self._action_client.get_result(self._action_topic)
                self._success = True


            else:
                Logger.logwarn('Action failed: %s' % str(status))
                self._failed = True

    def on_enter(self, userdata):

        self._failed = False
        self._success = False

        action_goal = ExecuteRecoveryOn3dCoveragePathGoal()
        action_goal.waypoint = userdata.waypoint
        action_goal.waypoint_reached = userdata.waypoint_reached

        try:
            self._action_client.send_goal(self._action_topic, action_goal)

        except Exception as e:
            Logger.logwarn('Failed when sending goal. Error :\n%s' % str(e))
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
            if self._action_client.is_available(self._action_topic) and not self._action_client.has_result(
                    self._action_topic):
                self._action_client.cancel(self._action_topic)
        except:
            # client already closed
            pass

    def on_pause(self):
        self._action_client.cancel(self._action_topic)

    def on_resume(self, userdata):
        self.on_enter(userdata)
