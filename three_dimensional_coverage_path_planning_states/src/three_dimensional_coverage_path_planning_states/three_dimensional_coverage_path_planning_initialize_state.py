#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from three_dimensional_coverage_path_planning_msgs.msg import InitializeExecutionOf3dCoveragePathAction, InitializeExecutionOf3dCoveragePathGoal, InitializeExecutionOf3dCoveragePathFeedback
from three_dimensional_coverage_path_planning_msgs.msg import Path


from actionlib_msgs.msg import GoalStatus
from rospy import Time

'''
Created on 09.03.2022

@author: Katrin Becker
'''


class ThreeDimensionalCoveragePathPlanningInitializeState(EventState):
    '''
    Initialize the execute of the given 3D coverage path. Uses the InitializeExecutionOf3dCoveragePath action.

    ># path              Path           Path that should be executed. If path.waypoints is empty, no path will be loaded (not for execution but for e.g. recovery)

    <= success                          Everything loaded correctly.
    <= failed                           An error occurred or initialize action was not allowed.
    '''

    def __init__(self):
        '''
        Constructor
        '''
        super(ThreeDimensionalCoveragePathPlanningInitializeState, self).__init__(outcomes=['success','failed'],
                                                  input_keys=['path'])

        self._action_topic = '/three_dimensional_coverage_path_planning/initialize_execution_of_path'
        self._action_client = ProxyActionClient({self._action_topic: InitializeExecutionOf3dCoveragePathAction})

        self._failed  = False
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

            Logger.loginfo('Progress of InitializeExecutionOf3dCoveragePath action: %s' % str(feedback.progress))



        if self._action_client.has_result(self._action_topic):

            status = self._action_client.get_state(self._action_topic)

            if status == GoalStatus.SUCCEEDED:

                # result = self._action_client.get_result(self._action_topic)
                self._success = True

            else:
                Logger.logwarn('Action failed: %s' % str(status))
                self._failed = True



    def on_enter(self, userdata):

        self._failed    = False
        self._success = False

        action_goal = InitializeExecutionOf3dCoveragePathGoal()
        action_goal.path = userdata.path

        Logger.logwarn('Loaded path has %i waypoints.' % len(userdata.path.waypoints))

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
            if self._action_client.is_available(self._action_topic) \
                    and not self._action_client.has_result(self._action_topic):
                self._action_client.cancel(self._action_topic)
        except:
            # client already closed
            pass



    def on_pause(self):
        self._action_client.cancel(self._action_topic)



    def on_resume(self, userdata):
        self.on_enter(userdata)
