#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_common_states.convert_to_json_state import ConvertToJsonState
from three_dimensional_coverage_path_planning_states.three_dimensional_coverage_path_planning_precomputation_state import ThreeDimensionalCoveragePathPlanningPrecomputationState
from three_dimensional_coverage_path_planning_states.three_dimensional_coverage_path_planning_save_path_state import ThreeDimensionalCoveragePathPlanningSavePathState
from flexbe_states.decision_state import DecisionState
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

from three_dimensional_coverage_path_planning_msgs.msg import Path, Waypoint

# [/MANUAL_IMPORT]


'''
Created on Tue Feb 15 2022
@author: Katrin Becker
'''
class three_dimesional_coverage_path_planning_precomputationsSM(Behavior):
	'''
	This behavior handles the precomputations for the 3D coverage path planning.
	'''


	def __init__(self):
		super(three_dimesional_coverage_path_planning_precomputationsSM, self).__init__()
		self.name = 'three_dimesional_coverage_path_planning_precomputations'

		# parameters of this behavior
		self.add_parameter('for_mission_execution_manager', False)
		self.add_parameter('load_models', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1022 y:419, x:1525 y:405
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['for_mission_execution_manager'])
		_state_machine.userdata.for_mission_execution_manager = self.for_mission_execution_manager
		_state_machine.userdata.load_models = self.load_models

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:177 y:68
			OperatableStateMachine.add('execute_precomputations',
										ThreeDimensionalCoveragePathPlanningPrecomputationState(),
										transitions={'success': 'convert_path_to_json', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'load_models': 'load_models', 'path': 'path', 'path_without_viewpoint_info': 'path_wo_viewpoint_info'})

			# x:1082 y:60
			OperatableStateMachine.add('convert_path_wo_viewpoint_info_to_json',
										ConvertToJsonState(),
										transitions={'success': 'for_mission_execution_manager_usage?', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'message': 'path_wo_viewpoint_info', 'json_string': 'path_wo_vp_info_json_string', 'json_dictionary': 'path_wo_vp_info_json_dictionary'})

			# x:324 y:365
			OperatableStateMachine.add('for_mission_execution_manager_usage?',
										DecisionState(outcomes=['mission_execution_manager_usage', 'general_usage'], conditions=lambda x: 'mission_execution_manager_usage' if x else 'general_usage'),
										transitions={'mission_execution_manager_usage': 'tmp_save_for_mission_execution_manager', 'general_usage': 'save_for_general_usage'},
										autonomy={'mission_execution_manager_usage': Autonomy.Off, 'general_usage': Autonomy.Off},
										remapping={'input_value': 'for_mission_execution_manager'})

			# x:838 y:521
			OperatableStateMachine.add('save_for_general_usage',
										ThreeDimensionalCoveragePathPlanningSavePathState(),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'path_json': 'path_json_dictionary', 'path_wo_viewpoint_info_json': 'path_wo_vp_info_json_dictionary'})

			# x:848 y:318
			OperatableStateMachine.add('tmp_save_for_mission_execution_manager',
										LogState(text='Save for mission execution manager', severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:739 y:60
			OperatableStateMachine.add('convert_path_to_json',
										ConvertToJsonState(),
										transitions={'success': 'convert_path_wo_viewpoint_info_to_json', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'message': 'path', 'json_string': 'path_json_string', 'json_dictionary': 'path_json_dictionary'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
