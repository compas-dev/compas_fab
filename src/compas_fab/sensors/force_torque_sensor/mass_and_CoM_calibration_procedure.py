from __future__ import print_function

import sys
import os
from os.path import isdir
import time
from datetime import datetime
import thread
import json

import abb_communication
from abb_communication.clients.rfl_robot.communication.communication import ABBCommunication

import rospy
import geometry_msgs.msg


# NOTE: this scrips uses ABB communication library and rospy for collecting poses and wrenches to feed into the mass_and_CoM_estimation scrips.
#		It should use proper compas_fab methods instead. To be revised!



### global variables
ft_samples = 5000
ft_data = []
pose_samples = 5
poses = []
_current_config = None
_current_config_index = None
_previous_pose = None
data_folder = r'/home/administrator/workspace/fts_calibration/calibration_records/'
gantry = None


# n = 30, seed = 12
calibration_configurations = [
	[0.0, 0.0, 0.0, 0.0, 83.509516689032623, 80.726161859546323],
	[0.0, 0.0, 0.0, 0.0, 67.937284150473175, 93.685533444788803],
	[0.0, 0.0, 0.0, 0.0, 63.747612792896099, 107.41825363333243],
	[0.0, 0.0, 0.0, 0.0, 62.390513042695261, 62.9816804271357],
	[0.0, 0.0, 0.0, 0.0, 57.328032047012996, 15.470124061048011],
	[0.0, 0.0, 0.0, 0.0, 49.130625274291759, 100.76952296506238],
	[0.0, 0.0, 0.0, 0.0, 46.498734178115946, 97.157886122852574],
	[0.0, 0.0, 0.0, 0.0, 43.227750201846732, 23.220768767121655],
	[0.0, 0.0, 0.0, 0.0, 33.953944790258255, 99.216886377273113],
	[0.0, 0.0, 0.0, 0.0, 31.840442117388907, 101.2062328610748],
	[0.0, 0.0, 0.0, 0.0, 29.651644924414935, 117.25529024294269],
	[0.0, 0.0, 0.0, 0.0, 28.241105931704279, 140.22743294205247],
	[0.0, 0.0, 0.0, 0.0, 18.390054846692454, 152.06367715573128],
	[0.0, 0.0, 0.0, 0.0, 17.372042990060603, 125.09320635065507],
	[0.0, 0.0, 0.0, 0.0, 12.062103527920529, 111.99436841877866],
	[0.0, 0.0, 0.0, 0.0, 11.570184973299794, 116.16854499418378],
	[0.0, 0.0, 0.0, 0.0, 9.0379564033195425, 112.56916030873242],
	[0.0, 0.0, 0.0, 0.0, 6.6011478185745505, 44.709673993653638],
	[0.0, 0.0, 0.0, 0.0, 1.3817870096352902, 142.36272588134494],
	[0.0, 0.0, 0.0, 0.0, -2.9854312693861971, 13.337568032963873],
	[0.0, 0.0, 0.0, 0.0, -18.878206083827536, 109.6161663252995],
	[0.0, 0.0, 0.0, 0.0, -20.891174811720418, 8.2831118543020921],
	[0.0, 0.0, 0.0, 0.0, -29.862220170584237, 129.57036408032286],
	[0.0, 0.0, 0.0, 0.0, -32.53808952183357, 143.17379958216242],
	[0.0, 0.0, 0.0, 0.0, -37.169697655509424, 156.66232689116231],
	[0.0, 0.0, 0.0, 0.0, -48.075436983144016, 15.607108687166772],
	[0.0, 0.0, 0.0, 0.0, -62.438006171136919, 162.0235287070783],
	[0.0, 0.0, 0.0, 0.0, -63.948713454094388, 32.811875233920318],
	[0.0, 0.0, 0.0, 0.0, -67.68183480374023, 53.772178620505287],
	[0.0, 0.0, 0.0, 0.0, -83.81893003053068, 68.637581488021681]
]


def ft_callback(data):
	if (len(ft_data) < ft_samples):
		ft_data.append( [
			data.wrench.force.x,
			data.wrench.force.y,
			data.wrench.force.z,
			data.wrench.torque.x,
			data.wrench.torque.y,
			data.wrench.torque.z] )


def record_calibration_data(_config, _config_idx):
	# TODO: implement timeout for this function

	### ros connection
	rospy.init_node('fts_calibration', anonymous=True)
	rospy.Subscriber("/netft_data", geometry_msgs.msg.WrenchStamped, ft_callback)
	

	while (not rospy.is_shutdown()):

		# once enough data has been collected, dump them in a json file
		if (len(ft_data) >= ft_samples) and (len(poses) >= pose_samples):

			print("collected", len(poses), "poses and", len(ft_data), "FT readings.")
			
			data_entry = {}
			data_entry['timestamp'] = datetime.now().strftime("%Y%m%d-%H%M%S")
			data_entry['configuration'] = _config
			data_entry['poses'] = poses
			data_entry['wrench'] = ft_data

			if (not isdir(data_folder)):
				print("ERROR:", data_folder, "does NOT exist!")
				return
			data_file = data_folder + r"calibration_record_" + str(_config_idx) + r"__" + datetime.now().strftime("%Y%m%d-%H%M%S")

			with open(data_file, 'wb') as json_file:
				json.dump(data_entry, json_file, sort_keys=False, indent=2, separators=(',', ': '))
			
			time.sleep(0.25)
			return

		# record new input data
		else:
			print("collecting input data...")

			new_input_pose = robot.get_current_pose_cartesian()[:7]
			if (len(poses) < pose_samples):
				poses.append( [
					new_input_pose[0],      #pose.position.x
					new_input_pose[1],      #pose.position.y
					new_input_pose[2],      #pose.position.z
					new_input_pose[3],      #pose.orientation.x
					new_input_pose[4],      #pose.orientation.y
					new_input_pose[5],      #pose.orientation.z
					new_input_pose[6]       #pose.orientation.w
				] )

			time.sleep(0.1)


### connect to ABB robot
ip_abb = "192.168.0.11"		#RFL robot
#ip_abb = "127.0.0.1"       #RobotStudio simulation
robot = ABBCommunication("ABB", ip_abb, port_snd=30003, port_rcv=30004)
robot.start()
time.sleep(2)

#robot setup
robot.float_arbitrary = 1   #robot 11
robot.set_speed_mid()
robot.set_tool0()


### main loop
config_counter = 24		#in case you want to resume a calibration routine that failed, write here the config index you stopped at
while(True):

	### get keyboard input
	inpt = raw_input("ENTER 'N' TO SEND NEXT CONFIGURATION, 'R' TO RECORD DATA, 'X' TO EXIT: ")
	print('\x1b[1;32;40m' + inpt + '\x1b[0m')
	

	### send config to robot
	if (inpt == "n") or (inpt == "N"):

		if gantry is None:
			gantry = list(robot.get_current_pose_cartesian()[7:10])

		config_counter += 1
		_current_config_index = config_counter

		if (config_counter < len(calibration_configurations)):
			_current_config = calibration_configurations[config_counter] + gantry
			print("config =", _current_config)
		else:
			print("Procedure completed!")
			exit(0)

		#send absolute configuration
		robot.send_axes_absolute(_current_config)
	

	### record pose and forces
	elif (inpt == "r") or (inpt == "R"):

		time.sleep(5)

		current_pose = robot.get_current_pose_cartesian()[:7]

		if (current_pose != _previous_pose) and (_current_config is not None):
			#record data
			ft_data = []
			poses = []
			record_calibration_data(_current_config, _current_config_index)
			_previous_pose = current_pose
		elif (_current_config is None):
			print("Send configuration before recording data!")
		else:
			print("\033[91m {}\033[00m" .format("ERROR: robot.get_current_pose_cartesian() did not update!"))

	### exit
	elif (inpt == "x") or (inpt == "x"):
		exit(0)

