#!/usr/bin/env python

import rospy
import rospkg
import json
from flexbe_core import EventState, Logger
from rospy_message_converter import message_converter

import tf2_ros
import tf2_geometry_msgs

from rospy import Time

'''
Created on 15.02.2022

@author: Katrin Becker
'''


class ThreeDimensionalCoveragePathPlanningLoadPathState(EventState):
    '''
    Saves the given data.

    #> data              path           Loaded path.
    
    -- world_frame       string         World frame, in which the path should be transformed.

    <= success                          Data are loaded.
    <= failed                           An error occurred while loading.
    '''

    def __init__(self, world_frame='world', use_recovery='True'):
        '''
        Constructor
        '''
        super(ThreeDimensionalCoveragePathPlanningLoadPathState, self).__init__(outcomes=['success','failed'],
                                                  output_keys=['data'])
                                                  

        self._world_frame = world_frame
        self._use_recovery = use_recovery

        self._failed    = False
        self._success = False
        
        # init tf buffer and listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)



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
        
        
        try:
            data_directory = rospy.get_param("/three_dimensional_coverage_path_planning/model/data_directory")
        except:
            rospack = rospkg.RosPack()
            data_directory = rospack.get_path('three_dimensional_coverage_path_planning')
            data_directory = data_directory + '/data'
        
        file_name = data_directory + '/'
        
        if self._use_recovery:
            file_name = file_name + 'path.json'
        else:
            file_name = file_name + 'path_without_viewpoint_information.json'
            
        
        Logger.loginfo('Try to load path from file: %s.' % file_name)

        # load data from file 
        with open(file_name, 'r', encoding='utf-8') as input_file:
            
            json_dictionary = json.load(input_file)
            
        # convert to ros message
        userdata.data = message_converter.convert_dictionary_to_ros_message('three_dimensional_coverage_path_planning_msgs/Path', json_dictionary)
            
        
        # transform waypoints (and paths between waypoints) to world_frame
        
        target_frame = self._world_frame
        model_frame = userdata.data.header.frame_id
        
        try:
            transform = self._tf_buffer.lookup_transform(target_frame, model_frame, rospy.Time(), rospy.Duration(3.0))
            
            for waypoint in userdata.data.waypoints:
            
                # transform waypoint
                waypoint.waypoint = tf2_geometry_msgs.do_transform_pose(waypoint.waypoint, transform)
                
                # transform path to waypoint
                waypoint.path_to_waypoint.header.frame_id = target_frame
                for i in range(0, len(waypoint.path_to_waypoint.poses)):
                  waypoint.path_to_waypoint.poses[i] = tf2_geometry_msgs.do_transform_pose(waypoint.path_to_waypoint.poses[i], transform)
            
            userdata.data.header.frame_id = target_frame
            
            self._success = True
            
        except:
            Logger.logerr('Transformation from %s to %s not found!' % model_frame, target_frame)
            
            self._failed = true
        



    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        pass


    def on_resume(self, userdata):
        self.on_enter(userdata)
