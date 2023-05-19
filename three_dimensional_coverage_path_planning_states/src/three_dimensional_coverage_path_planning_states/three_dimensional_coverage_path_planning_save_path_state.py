#!/usr/bin/env python

import rospy
import rospkg
import json
from flexbe_core import EventState, Logger

from rospy import Time

'''
Created on 15.02.2022

@author: Katrin Becker
'''


class ThreeDimensionalCoveragePathPlanningSavePathState(EventState):
    '''
    Saves the given paths.

    ># path_json                        dictionary         Path to save (as json dictionary).
    ># path_wo_viewpoint_info_json      dictionary         Path without viewpoint info to save (as json dictionary).

    <= success                                             Data are saved.
    <= failed                                              An error occurred while saving.
    '''

    def __init__(self):
        '''
        Constructor
        '''
        super(ThreeDimensionalCoveragePathPlanningSavePathState, self).__init__(outcomes=['success','failed'],
                                                  input_keys=['path_json', 'path_wo_viewpoint_info_json'])
                                                  

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
        
        # convert json string to dictionary (if passed as string this would be required)
        #json_dictionary = json.loads(userdata.path_json)
        
        json_dictionaries = [userdata.path_json, userdata.path_wo_viewpoint_info_json]
        
        
        # set file names
        file_names = ['path', 'path_without_viewpoint_information']
        
        
        # get file path
        try:
            data_directory = rospy.get_param("/three_dimensional_coverage_path_planning/model/data_directory")
        except:
            rospack = rospkg.RosPack()
            data_directory = rospack.get_path('three_dimensional_coverage_path_planning')
            data_directory = data_directory + '/data'
        
        
        # save files
        for i in range(0, len(json_dictionaries)):
            
            #time_stamp = time.time()
            #time_string = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            #file_name = data_directory + '/' + file_names[i] + time_string + '.json'
            
            
            file_name = data_directory + '/' + file_names[i] + '.json'
            
            Logger.loginfo('Try to save path to file: %s.' % file_name)

            # save data to file 
            with open(file_name, 'w', encoding='utf-8') as output_file:
                
                # no indent: single line but smaller file size
                json.dump(json_dictionaries[i], output_file, ensure_ascii=False, indent=None)
            
        
        self._success = True



    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        pass


    def on_resume(self, userdata):
        self.on_enter(userdata)
