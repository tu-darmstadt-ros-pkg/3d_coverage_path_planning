#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_common_states.get_attribute_state import GetAttributeState
from flexbe_common_states.move_to_waypoint_state import MoveToWaypointState
from three_dimensional_coverage_path_planning_states.three_dimensional_coverage_path_planning_move_to_waypoint_state import ThreeDimensionalCoveragePathPlanningMoveToWaypointState
from three_dimensional_coverage_path_planning_states.three_dimensional_coverage_path_planning_recovery_state import ThreeDimensionalCoveragePathPlanningRecoveryState
from three_dimensional_coverage_path_planning_states.three_dimensional_coverage_path_planning_waypoint_reached_state import ThreeDimensionalCoveragePathPlanningWaypointReachedState
from flexbe_states.decision_state import DecisionState
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Mar 09 2022
@author: Katrin Becker
'''
class three_dimesional_coverage_path_planning_waypointSM(Behavior):
	'''
	This behavior handles the execution of one waypoint.
	'''


	def __init__(self):
		super(three_dimesional_coverage_path_planning_waypointSM, self).__init__()
		self.name = 'three_dimesional_coverage_path_planning_waypoint'

		# parameters of this behavior
		self.add_parameter('speed', 0.5)
		self.add_parameter('use_planning', True)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1301 y:424, x:131 y:585
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['use_recovery', 'waypoint'])
		_state_machine.userdata.waypoint = None
		_state_machine.userdata.speed = self.speed
		_state_machine.userdata.use_recovery = True
		_state_machine.userdata.waypoint_reached = False
		_state_machine.userdata.first_call = False

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:93 y:45
			OperatableStateMachine.add('get_path_from_waypoint',
										GetAttributeState(attribute_name='path_to_waypoint'),
										transitions={'succeeded': 'has_path?', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object': 'waypoint', 'attribute': 'path_to_waypoint'})

			# x:343 y:165
			OperatableStateMachine.add('get_waypoint_pose_from_waypoint',
										GetAttributeState(attribute_name='waypoint'),
										transitions={'succeeded': 'move_to_next_waypoint_without_path', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'object': 'waypoint', 'attribute': 'waypoint_pose'})

			# x:388 y:42
			OperatableStateMachine.add('has_path?',
										DecisionState(outcomes=['path','no_path'], conditions=lambda x: 'no_path' if not x.poses else 'path'),
										transitions={'path': 'move_to_next_waypoint', 'no_path': 'get_waypoint_pose_from_waypoint'},
										autonomy={'path': Autonomy.Off, 'no_path': Autonomy.Off},
										remapping={'input_value': 'path_to_waypoint'})

			# x:638 y:43
			OperatableStateMachine.add('move_to_next_waypoint',
										ThreeDimensionalCoveragePathPlanningMoveToWaypointState(speed=self.speed),
										transitions={'success': 'waypoint_reached', 'failed': 'use_recovery?'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'path': 'path_to_waypoint'})

			# x:576 y:143
			OperatableStateMachine.add('move_to_next_waypoint_without_path',
										MoveToWaypointState(position_tolerance=0.1, angle_tolerance=0.1, rotate_to_goal=True, reexplore_time=5, reverse_allowed=True, reverse_forced=False, use_planning=self.use_planning),
										transitions={'reached': 'waypoint_reached', 'failed': 'use_recovery?', 'stuck': 'use_recovery?', 'cancel': 'use_recovery?'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off, 'stuck': Autonomy.Off, 'cancel': Autonomy.Off},
										remapping={'waypoint': 'waypoint_pose', 'speed': 'speed', 'first_call': 'first_call'})

			# x:1092 y:261
			OperatableStateMachine.add('recovery',
										ThreeDimensionalCoveragePathPlanningRecoveryState(),
										transitions={'success': 'finished', 'failed': 'skip_current_waypoint'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint', 'waypoint_reached': 'waypoint_reached'})

			# x:1500 y:258
			OperatableStateMachine.add('skip_current_waypoint',
										LogState(text='Skip current waypoint as it is not reachable!', severity=Logger.REPORT_ERROR),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:1192 y:66
			OperatableStateMachine.add('use_recovery?',
										DecisionState(outcomes=['recovery', 'no_recovery'], conditions=lambda use_recovery: 'recovery' if use_recovery else 'no_recovery'),
										transitions={'recovery': 'recovery', 'no_recovery': 'skip_current_waypoint'},
										autonomy={'recovery': Autonomy.Off, 'no_recovery': Autonomy.Off},
										remapping={'input_value': 'use_recovery'})

			# x:828 y:420
			OperatableStateMachine.add('waypoint_reached',
										ThreeDimensionalCoveragePathPlanningWaypointReachedState(),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
