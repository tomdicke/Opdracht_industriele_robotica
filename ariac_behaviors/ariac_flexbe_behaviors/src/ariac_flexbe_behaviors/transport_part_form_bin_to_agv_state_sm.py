#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.start_assignment_state import StartAssignment
from ariac_flexbe_states.end_assignment_state import EndAssignment
from ariac_flexbe_states.srdf_state_to_moveit_ariac_state import SrdfStateToMoveitAriac
from flexbe_states.wait_state import WaitState
from ariac_flexbe_states.detect_part_camera_ariac_state import DetectPartCameraAriacState
from ariac_flexbe_states.compute_grasp_ariac_state import ComputeGraspAriacState
from ariac_flexbe_states.moveit_to_joints_dyn_ariac_state import MoveitToJointsDynAriacState
from ariac_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
from ariac_flexbe_behaviors.transport_part_from_bin_to_agv_1_sm import transport_part_from_bin_to_agv_1SM
from ariac_flexbe_states.set_conveyorbelt_power_state import SetConveyorbeltPowerState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Apr 22 2020
@author: Gerard Harkema
'''
class transport_part_form_bin_to_agv_stateSM(Behavior):
	'''
	transports part from it's bin to the selected agv
	'''


	def __init__(self):
		super(transport_part_form_bin_to_agv_stateSM, self).__init__()
		self.name = 'transport_part_form_bin_to_agv_state'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(transport_part_from_bin_to_agv_1SM, 'transport_part_from_bin_to_agv_1')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:52 y:611, x:343 y:359
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.agv_id = 'agv1'
		_state_machine.userdata.part_type = 'gear_part'
		_state_machine.userdata.pose_on_agv = []
		_state_machine.userdata.pose = []
		_state_machine.userdata.part_pose = []
		_state_machine.userdata.joint_values = []
		_state_machine.userdata.joint_names = []
		_state_machine.userdata.offset = 0.1
		_state_machine.userdata.move_group = 'manipulator'
		_state_machine.userdata.move_group_prefix = '/ariac/arm1'
		_state_machine.userdata.config_name_home = 'home'
		_state_machine.userdata.action_topic = '/move_group'
		_state_machine.userdata.robot_name = ''
		_state_machine.userdata.config_name_bin3PreGrasp = 'bin3PreGrasp'
		_state_machine.userdata.config_name_tray1PreDrop = 'tray1PreDrop'
		_state_machine.userdata.camera_ref_frame = 'arm1_linear_arm_actuator'
		_state_machine.userdata.camera_topic = '/ariac/logical_camera_5'
		_state_machine.userdata.camera_frame = 'logical_camera_5_frame'
		_state_machine.userdata.tool_link = 'ee_link'
		_state_machine.userdata.agv_pose = []
		_state_machine.userdata.part_offset = 0.035
		_state_machine.userdata.part_rotation = 0
		_state_machine.userdata.conveyor_belt_power = 100.0
		_state_machine.userdata.config_name_bin1PreGrasp = 'bin1PreGrasp'
		_state_machine.userdata.arm_id = "arm1"
		_state_machine.userdata.part_drop_offset = 0.1
		_state_machine.userdata.StartText = 'Opdracht gestart'
		_state_machine.userdata.StopText = 'Opdracht gestopt'
		_state_machine.userdata.Shipments = []
		_state_machine.userdata.material_locations = []
		_state_machine.userdata.NumberOfShipments = 0
		_state_machine.userdata.OrderId = ''
		_state_machine.userdata.Products = []
		_state_machine.userdata.NumberOfProducts = 0
		_state_machine.userdata.MaterialsLocationList = []
		_state_machine.userdata.config_name_bin3PreDrop = 'bin3PreDrop'
		_state_machine.userdata.part = 'gasket_part'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:54 y:102
			OperatableStateMachine.add('StartAssignment',
										StartAssignment(),
										transitions={'continue': 'ConveyorBelt'},
										autonomy={'continue': Autonomy.Off})

			# x:197 y:603
			OperatableStateMachine.add('EndAssignment',
										EndAssignment(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:654 y:113
			OperatableStateMachine.add('MoveR1Home',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'DetectCameraPart', 'planning_failed': 'WaitRetry1', 'control_failed': 'WaitRetry1', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_home', 'move_group': 'move_group', 'move_group_prefix': 'move_group_prefix', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:666 y:12
			OperatableStateMachine.add('WaitRetry1',
										WaitState(wait_time=5),
										transitions={'done': 'MoveR1Home'},
										autonomy={'done': Autonomy.Off})

			# x:836 y:111
			OperatableStateMachine.add('DetectCameraPart',
										DetectPartCameraAriacState(time_out=5.0),
										transitions={'continue': 'MoveR1PreGrasp1', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'camera_ref_frame', 'camera_topic': 'camera_topic', 'camera_frame': 'camera_frame', 'part': 'part', 'pose': 'pose'})

			# x:1039 y:116
			OperatableStateMachine.add('MoveR1PreGrasp1',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'ComputePick', 'planning_failed': 'WaitRetry2', 'control_failed': 'WaitRetry2', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_bin1PreGrasp', 'move_group': 'move_group', 'move_group_prefix': 'move_group_prefix', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1048 y:21
			OperatableStateMachine.add('WaitRetry2',
										WaitState(wait_time=5),
										transitions={'done': 'MoveR1PreGrasp1'},
										autonomy={'done': Autonomy.Off})

			# x:1038 y:200
			OperatableStateMachine.add('ComputePick',
										ComputeGraspAriacState(joint_names=['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']),
										transitions={'continue': 'MoveR1ToPick1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'move_group_prefix': 'move_group_prefix', 'tool_link': 'tool_link', 'pose': 'pose', 'offset': 'part_offset', 'rotation': 'part_rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1069 y:379
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=1),
										transitions={'done': 'transport_part_from_bin_to_agv_1'},
										autonomy={'done': Autonomy.Off})

			# x:1303 y:280
			OperatableStateMachine.add('WaitRetry3',
										WaitState(wait_time=5),
										transitions={'done': 'MoveR1ToPick1'},
										autonomy={'done': Autonomy.Off})

			# x:1035 y:284
			OperatableStateMachine.add('MoveR1ToPick1',
										MoveitToJointsDynAriacState(),
										transitions={'reached': 'EnableGripper', 'planning_failed': 'WaitRetry3', 'control_failed': 'EnableGripper'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'move_group_prefix': 'move_group_prefix', 'move_group': 'move_group', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:818 y:282
			OperatableStateMachine.add('EnableGripper',
										VacuumGripperControlState(enable=True),
										transitions={'continue': 'Wait', 'failed': 'failed', 'invalid_arm_id': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'invalid_arm_id': Autonomy.Off},
										remapping={'arm_id': 'arm_id'})

			# x:294 y:484
			OperatableStateMachine.add('DisableGripper',
										VacuumGripperControlState(enable=False),
										transitions={'continue': 'EndAssignment', 'failed': 'failed', 'invalid_arm_id': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'invalid_arm_id': Autonomy.Off},
										remapping={'arm_id': 'arm_id'})

			# x:975 y:541
			OperatableStateMachine.add('transport_part_from_bin_to_agv_1',
										self.use_behavior(transport_part_from_bin_to_agv_1SM, 'transport_part_from_bin_to_agv_1'),
										transitions={'finished': 'DisableGripper', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:222 y:106
			OperatableStateMachine.add('ConveyorBelt',
										SetConveyorbeltPowerState(),
										transitions={'continue': 'MoveR1Home', 'fail': 'failed'},
										autonomy={'continue': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'power': 'conveyor_belt_power'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
