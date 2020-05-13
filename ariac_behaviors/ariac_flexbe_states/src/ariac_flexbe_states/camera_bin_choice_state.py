#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger


class CameraBinChoiceState(EventState):
	'''
	State for deciding witch bin and camera will be used
	
	<# bin_loc		string		Bin of part	
	#> camera_topic		StorageUnit[]	Camera topic
	#> camera_frame		StorageUnit[]	Camera frame
	#> camera_ref_frame	StorageUnit[]	Camera ref_frame
	<= continue 			Given time has passed.
	<= invalid_bin 			Invalid bin number

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(CameraBinChoiceState, self).__init__(input_keys = ['bin_loc'], outcomes = ['continue', 'invalid_bin'], output_keys = ['camera_ref_frame','camera_frame','camera_topic'])

		
		pass # Nothing to do in this example.


	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		
		if userdata.bin_loc == 'bin1':
			camera_topic = '/ariac/logical_camera_1'
			camera_frame = 'logical_camera_1_frame'
			camera_ref_frame = 'arm1_linear_arm_actuator'
		elif userdata.bin_loc == 'bin2':
			camera_topic = '/ariac/logical_camera_2'
			camera_frame = 'logical_camera_2_frame'
			camera_ref_frame = 'arm1_linear_arm_actuator'
		elif userdata.bin_loc == 'bin3':
			camera_topic = '/ariac/logical_camera_3'
			camera_frame = 'logical_camera_3_frame'
			camera_ref_frame = ''
		elif userdata.bin_loc == 'bin4':
			camera_topic = '/ariac/logical_camera_4'
			camera_frame = 'logical_camera_4_frame'
			camera_ref_frame = ''
		elif userdata.bin_loc == 'bin5':
			camera_ref_frame = 'arm2_linear_arm_actuator'
			camera_topic = '/ariac/logical_camera_5'
			camera_frame = 'logical_camera_5_frame'
		elif userdata.bin_loc == 'bin6':
			camera_ref_frame = 'arm2_linear_arm_actuator'
			camera_topic = '/ariac/logical_camera_6'
			camera_frame = 'logical_camera_6_frame'
		else:
			return 'invalid_bin'
		
		pass # Nothing to do in this example.

	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		# The following code is just for illustrating how the behavior logger works.
		# Text logged by the behavior logger is sent to the operator and displayed in the GUI.

		pass # Nothing to do in this example.


	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass # Nothing to do in this example.


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.

		# In this example, we use this event to set the correct start time.
		pass # Nothing to do in this example.


	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do in this example.
		
