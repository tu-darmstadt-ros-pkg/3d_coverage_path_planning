#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from three_dimensional_coverage_path_planning_behaviors.three_dimesional_coverage_path_planning_waypoint_sm import three_dimesional_coverage_path_planning_waypointSM
from flexbe_common_states.get_attribute_state import GetAttributeState
from flexbe_common_states.get_first_from_array_state import GetFirstFromArrayState
from flexbe_common_states.get_robot_pose import GetRobotPose
from three_dimensional_coverage_path_planning_states.three_dimensional_coverage_path_planning_finish_state import ThreeDimensionalCoveragePathPlanningFinishState
from three_dimensional_coverage_path_planning_states.three_dimensional_coverage_path_planning_initialize_state import ThreeDimensionalCoveragePathPlanningInitializeState
from three_dimensional_coverage_path_planning_states.three_dimensional_coverage_path_planning_load_path_state import ThreeDimensionalCoveragePathPlanningLoadPathState
from three_dimensional_coverage_path_planning_states.three_dimensional_coverage_path_planning_rotate_circular_path_state import ThreeDimensionalCoveragePathPlanningRotateCircularPathState
from flexbe_common_states.write_time_to_file import WriteTimeToFile
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Mar 07 2022
@author: Katrin Becker
'''
class three_dimesional_coverage_path_planning_execute_pathSM(Behavior):
	'''
	This behavior handles the execution of a precomputed path.
	'''


	def __init__(self):
		super(three_dimesional_coverage_path_planning_execute_pathSM, self).__init__()
		self.name = 'three_dimesional_coverage_path_planning_execute_path'

		# parameters of this behavior
		self.add_parameter('use_recovery', False)
		self.add_parameter('speed', 0.0)
		self.add_parameter('use_planning', False)

		# references to used behaviors
		self.add_behavior(three_dimesional_coverage_path_planning_waypointSM, 'three_dimesional_coverage_path_planning_waypoint')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1256 y:532, x:936 y:49
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.use_recovery = self.use_recovery

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:26 y:50
			OperatableStateMachine.add('write_start_time',
										WriteTimeToFile(file_name='start_time.txt'),
										transitions={'success': 'load_path', 'failed': 'load_path'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:543 y:537
			OperatableStateMachine.add('get_next_waypoint',
										GetFirstFromArrayState(position=0),
										transitions={'succeeded': 'three_dimesional_coverage_path_planning_waypoint', 'empty': 'finish_execution'},
										autonomy={'succeeded': Autonomy.Off, 'empty': Autonomy.Off},
										remapping={'array': 'waypoints', 'element': 'waypoint'})

			# x:135 y:245
			OperatableStateMachine.add('get_robot_pose',
										GetRobotPose(),
										transitions={'succeeded': 'rotate_path'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'robot_pose'})

			# x:155 y:588
			OperatableStateMachine.add('get_waypoints_from_path',
										GetAttributeState(attribute_name='waypoints'),
										transitions={'succeeded': 'get_next_waypoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object': 'path', 'attribute': 'waypoints'})

			# x:119 y:464
			OperatableStateMachine.add('initialize_execution',
										ThreeDimensionalCoveragePathPlanningInitializeState(),
										transitions={'success': 'get_waypoints_from_path', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'path': 'path'})

			# x:117 y:149
			OperatableStateMachine.add('load_path',
										ThreeDimensionalCoveragePathPlanningLoadPathState(world_frame='world', use_recovery=self.use_recovery),
										transitions={'success': 'get_robot_pose', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'data': 'path'})

			# x:79 y:351
			OperatableStateMachine.add('rotate_path',
										ThreeDimensionalCoveragePathPlanningRotateCircularPathState(),
										transitions={'success': 'initialize_execution', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'path': 'path', 'current_pose': 'robot_pose'})

			# x:474 y:690
			OperatableStateMachine.add('three_dimesional_coverage_path_planning_waypoint',
										self.use_behavior(three_dimesional_coverage_path_planning_waypointSM, 'three_dimesional_coverage_path_planning_waypoint',
											parameters={'speed': self.speed, 'use_planning': self.use_planning}),
										transitions={'finished': 'get_next_waypoint', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'use_recovery': 'use_recovery', 'waypoint': 'waypoint'})

			# x:1128 y:393
			OperatableStateMachine.add('write_end_time',
										WriteTimeToFile(file_name='end_time.txt'),
										transitions={'success': 'finished', 'failed': 'finished'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:866 y:522
			OperatableStateMachine.add('finish_execution',
										ThreeDimensionalCoveragePathPlanningFinishState(),
										transitions={'success': 'write_end_time', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
