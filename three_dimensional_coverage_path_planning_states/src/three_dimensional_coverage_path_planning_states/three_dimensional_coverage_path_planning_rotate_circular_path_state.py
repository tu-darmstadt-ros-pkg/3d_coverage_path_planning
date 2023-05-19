#!/usr/bin/env python

from sys import float_info

from math import sqrt

import copy

import rospy
from flexbe_core import EventState, Logger

from three_dimensional_coverage_path_planning_msgs.msg import Path, Waypoint


from actionlib_msgs.msg import GoalStatus
from rospy import Time

'''
Created on 18.03.2022

@author: Katrin Becker
'''


class ThreeDimensionalCoveragePathPlanningRotateCircularPathState(EventState):
    '''
    Rotate a given circular path, so that the waypoint closest to current position is the first (and last) waypoint in path. The distance of current position to waypoints is computed using euclidean distance.

    ># path              Path                           Path to be rotated. As a circular path is required, the first and last waypoint need to be the same.
    ># current_pose      geometry_msgs/PoseStamped      Current pose of the robot.
    
    #> path              Path                           Rotated path. Still a circular path with first and last waypoint equal, "old" first/last waypoint duplicate removed

    <= success                                          Path could be rotated.
    <= failed                                           An error occurred or input path had no waypoints or was not circular.
    '''

    def __init__(self):
        '''
        Constructor
        '''
        super(ThreeDimensionalCoveragePathPlanningRotateCircularPathState, self).__init__(outcomes=['success','failed'],
                                                  input_keys=['path', 'current_pose'], output_keys=['path'])


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
        



    def on_enter(self, userdata):
        
        self._failed  = False
        self._success = False


        if not userdata.path.waypoints or userdata.path.waypoints[0].waypoint != userdata.path.waypoints[-1].waypoint:
            self._failed = True
            return


        start = userdata.current_pose.pose.position
        
        min_distance = float_info.max
        min_idx = -1
        
        # find desired start point
        for idx in range(0, len(userdata.path.waypoints)):
            waypoint = userdata.path.waypoints[idx].waypoint.pose.position
                       
            distance = sqrt( (waypoint.x - start.x)**2 + (waypoint.y - start.y)**2 + (waypoint.z - start.z)**2  )
            
            if distance < min_distance:
                
                min_distance = distance
                min_idx = idx
        
        
        Logger.loginfo('Closest pose: %i' % min_idx)
        
        
        if min_idx == 0 or min_idx == len(userdata.path.waypoints):
            Logger.loginfo('First pose is closest, no need to rotate path.')
            self.success = True
        
        else:

            # split old array by min_idx
            tmp = userdata.path.waypoints[:min_idx]
            del userdata.path.waypoints[:min_idx]

            # append tmp to back of path (start with element 1 instead of 0 in order to remove duplicate old first/last element, also remove old first instead of old last, as last has the path to it, first not)
            userdata.path.waypoints.extend(tmp[1:])

            # duplicate new first element to end of path in order to create a circular one
            userdata.path.waypoints.append(copy.deepcopy(userdata.path.waypoints[0]))
            
            
            # clear path of first waypoint
            userdata.path.waypoints[0].path_to_waypoint.poses = []
            
            self._success = True


    def on_stop(self):
        pass



    def on_exit(self, userdata):
        pass



    def on_pause(self):
        pass



    def on_resume(self, userdata):
        pass
