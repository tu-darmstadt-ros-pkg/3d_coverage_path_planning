#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from three_dimensional_coverage_path_planning_msgs.msg import Plan3dCoveragePathOnMeshAction, Plan3dCoveragePathOnMeshGoal, Plan3dCoveragePathOnMeshFeedback
from three_dimensional_coverage_path_planning_msgs.msg import Path, Waypoint


from actionlib_msgs.msg import GoalStatus
from rospy import Time

'''
Created on 15.02.2022

@author: Katrin Becker
'''


class ThreeDimensionalCoveragePathPlanningPrecomputationState(EventState):
    '''
    Execute the precomputations for the 3D coverage path planning. Uses the Plan3dCoveragePathOnMesh action.

    ># load_models                      bool           If the models should be loaded from files or computed from a mesh (given in config files of package).
    
    #> path                             Path           Computed path.
    #> path_without_viewpoint_info      Path           Computed path without viewpoint information (reward, visible targets lists).

    <= success                          A path could be computed and it contains at least one waypoint.
    <= failed                           An error occurred or computed path had no waypoints.
    '''

    def __init__(self):
        '''
        Constructor
        '''
        super(ThreeDimensionalCoveragePathPlanningPrecomputationState, self).__init__(outcomes=['success','failed'],
                                                  input_keys=['load_models'], output_keys=['path', 'path_without_viewpoint_info'])


        self._action_topic = '/three_dimensional_coverage_path_planning/plan_path'
        self._action_client = ProxyActionClient({self._action_topic: Plan3dCoveragePathOnMeshAction})

        self._failed    = False
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
            
            # get and remove feedback (remove it, so that the progress is only printed if there is new feedback)
            feedback = self._action_client.get_feedback(self._action_topic)
            self._action_client.remove_feedback(self._action_topic)
            
            Logger.loginfo('Progress of Plan3dCoveragePathOnMesh action: %s' % str(feedback.progress))
            
        
        
        if self._action_client.has_result(self._action_topic):
        
            status = self._action_client.get_state(self._action_topic)
            
            if status == GoalStatus.SUCCEEDED:
                
                result = self._action_client.get_result(self._action_topic)
                
                if result.path.waypoints and result.path_without_viewpoint_information.waypoints:
                    userdata.path = result.path
                    userdata.path_without_viewpoint_info = result.path_without_viewpoint_information
                    self._success = True
                    
                else:
                    Logger.logwarn('Path has no waypoints!')
                    self._failed = True
                
            else:
                Logger.logwarn('Action failed: %s' % str(status))
                self._failed    = True



    def on_enter(self, userdata):
        
        self._failed    = False
        self._success = False

        action_goal = Plan3dCoveragePathOnMeshGoal()
        action_goal.load_models = userdata.load_models

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
