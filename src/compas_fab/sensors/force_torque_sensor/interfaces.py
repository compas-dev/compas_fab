import time
import roslibpy
import numpy as np
import copy
from threading import Lock, Event

from compensation import get_bias_compensated_wrench, get_gravity_and_bias_compensated_wrench
from set_bias import reset_bias

import compas.geometry as cg


# print in color for better readability 
def prGreen(skk): print("\033[92m {}\033[00m" .format(skk))
def prRed(skk): print("\033[91m {}\033[00m" .format(skk))
def prBlue(skk): print("\033[94m {}\033[00m" .format(skk))



class FTSInterfaces(object):
	def __init__(self, robot_id, ros_connection, sync_event, FTS_behaviour=1.0):
		self.robot_id = robot_id
		self._initReady_event = Event()
		self.sync_event = sync_event
		self.ros_connection = ros_connection
		self.ros_connection.on_ready(self.ros_connection_ready)
		self.robot_is_connected = True
		self.FTS_device = 'ATI'

		# Create mutex for protecting data shared in-between the callbacks
		# FT data variables
		self.FT_data_lock = Lock()
		self._latest_ft_data = None
		self.fts_failure_status = False
		
		# FT DATA FILTERING LISTS
		self._filter_ft_data = []

		# FT sensor behaviour:
		# 1.0 for behaviour standard fts behaviour
		# -1.0 for negated behaviour (all axis are inversed)
		self.FTS_behaviour = FTS_behaviour
		prBlue("FT sensor behaviour = " + str(self.FTS_behaviour))

  
	def wait_ready(self, timeout=None):
		prBlue("Waiting for connection to be ready...")
		self._initReady_event.wait(timeout)
		prBlue("Connection is now ready!")
		
		# reset bias at the beginning of every assembly task
		# NOTE: if there are any contact forces while resetting the bias, the result will NOT be correct!!!
		reset_bias(ros_connection = self.ros_connection, target_robot = self.robot_id, FTS_device = self.FTS_device)


	def ros_connection_ready(self):
		prBlue("Executing ros_connection_ready() function")

		if self.FTS_device == 'ATI':
			# Create ROS subscriber for receiving ati f/t data
			self.ati_ft_sub = roslibpy.Topic(self.ros_connection, "/netft_data", "geometry_msgs/WrenchStamped")
			self.ati_ft_sub.subscribe(self.fts_callback)
			# Create ROS subscriber for receiving ati f/t sensor diagnostics
			self.ati_ft_diagnostics_sub = roslibpy.Topic(self.ros_connection, "/diagnostics", "diagnostic_msgs/DiagnosticArray")
			self.ati_ft_diagnostics_sub.subscribe(self.fts_diagnostics_callback)
			#self.FTS_device_range = [330.0, 330.0, 990.0, 30.0, 30.0, 30.0]	#ATI mid range
			self.FTS_device_range = [667.233, 667.233, 2001.69, 67.7908, 67.7908, 67.7908]	#ATI high range
		elif self.FTS_device == 'BOTA':
			# Create ROS subscriber for receiving bota f/t data
			self.ati_ft_sub = roslibpy.Topic(self.ros_connection, "/ft_driver/force", "geometry_msgs/WrenchStamped")
			self.ati_ft_sub.subscribe(self.fts_callback)
			self.FTS_device_range = [3000.0, 3000.0, 6000.0, 100.0, 100.0, 77.0]

		# Create ROS publisher for publishing unbiased and gravity compensated FT data
		self.correct_FT_frame = True
		self.pub_ft_unbiased_and_compensated_toggle = True
		if self.pub_ft_unbiased_and_compensated_toggle:
			self.ft_unbiased_and_compensated_topic = roslibpy.Topic(self.ros_connection, '/ft_data__unbiased_and_compensated', 'geometry_msgs/Wrench')
			self.ft_unbiased_and_compensated_topic.advertise()

		#bias parameters from ROS parameter server
		self.bias_parameter_instance = roslibpy.Param(self.ros_connection, "FT_bias")
		self.bias_parameter = None
		self.current_bias = None

		#wait
		time.sleep(2)

		# wait_ready function set
		self._initReady_event.set()


	# Callback for f/t sensor diagnostics
	def fts_diagnostics_callback(self, data):
		self._latest_ft_diagnostic_data = data

		# check f/t sensor status system
		try:
			list_of_values = list(data["status"]["values"])
			for item_in_list in list_of_values:
				for key in item_in_list:
					if key == "System status":
						fts_sys_status = item_in_list[key]
						break
			self.fts_failure_status = any(fts_sys_status)
		except:
			pass


	# Callback for receiving f/t sensor data
	def fts_callback(self, data):
		with self.FT_data_lock:
			self._latest_ft_data = data

			# E-STOP if (FT data > readable range)
			# set self.FTS_device_range for your FTS device range
			if (
				   abs(self._latest_ft_data["wrench"]["force"]["x"])  > self.FTS_device_range[0]
				or abs(self._latest_ft_data["wrench"]["force"]["y"])  > self.FTS_device_range[1]
				or abs(self._latest_ft_data["wrench"]["force"]["z"])  > self.FTS_device_range[2]
				or abs(self._latest_ft_data["wrench"]["torque"]["x"]) > self.FTS_device_range[3]
				or abs(self._latest_ft_data["wrench"]["torque"]["y"]) > self.FTS_device_range[4]
				or abs(self._latest_ft_data["wrench"]["torque"]["z"]) > self.FTS_device_range[5]
			):
				prRed("Emergency stop triggered because FT data exceed readable range!")
				prRed("values = " + str(self._latest_ft_data["wrench"]))
				
				self.trigger_hard_stop()
				self.robot_is_connected = False


	@property
	def ft_data__raw(self):
		with self.FT_data_lock:
			data = self._latest_ft_data

		# get [Fx, Fy, Fz, Tx, Ty, Tz] from the FT sensor 
		if data:
			return [data["wrench"]["force"]["x"] * self.FTS_behaviour,
					data["wrench"]["force"]["y"] * self.FTS_behaviour,
					data["wrench"]["force"]["z"] * self.FTS_behaviour,
					data["wrench"]["torque"]["x"] * self.FTS_behaviour,
					data["wrench"]["torque"]["y"] * self.FTS_behaviour,
					data["wrench"]["torque"]["z"] * self.FTS_behaviour]
		else:
			return None


	@property
	def ft_data__unbiased(self):
		wrench_data = self.ft_data__raw
		
		while self.current_bias is None:
			self.get_current_bias_from_ros_parameter_server(self.robot_id)
		
		if wrench_data and self.current_bias:
			ft_unbiased = get_bias_compensated_wrench(wrench_data, self.current_bias)
			ft_unbiased = [x * self.FTS_behaviour for x in ft_unbiased]
			return ft_unbiased
		else:
			prRed("ERROR: No wrench or bias data!")
			return None


	def bias_param_callback(self, data):
		self.bias_parameter = data
	

	def get_current_bias_from_ros_parameter_server(self, robot_id):
		#gets the bias from the ROS parameter server
		self.bias_parameter_instance.get(self.bias_param_callback)
		
		prBlue("Retrieving current bias from ROS parameter server...")
		#time.sleep(0.1)
		
		if self.bias_parameter:
			self.bias_parameter = self.bias_parameter[robot_id]
			self.current_bias = [
				self.bias_parameter['F_bias_X'],
				self.bias_parameter['F_bias_Y'],
				self.bias_parameter['F_bias_Z'],
				self.bias_parameter['T_bias_X'],
				self.bias_parameter['T_bias_Y'],
				self.bias_parameter['T_bias_Z']]
			prBlue("Current bias for " + str(robot_id) + " = " + str(self.current_bias))
		else:
			prRed("ERROR: get_bias_parameter returned NONE!")
		#time.sleep(0.1)


	def rotate_wrench(self, wrench, rotation):
		"""
		Rotate a wrench or force-torque vector given a rotation matrix.
		:param wrench: list, wrench or force-torque vector [fx, fy, fz, tx, ty, tz]
		:param rotation: matrix, 3x3 rotation matrix [[xx, yx, zx], [xy, yy, zy], [xz, yz, zz]]
		:return: list, wrench or force-torque vector [fx, fy, fz, tx, ty, tz]
		"""
		rotated_wrench = []
		for i in range(2):
			vector = wrench[i * 3: i * 3 + 3]
			# Transpose to [[x], [y], [z]]
			vector = np.matrix(vector).T
			# Apply rotation
			vector = np.matmul(rotation, vector).T
			# Convert back to vector
			vector = vector.tolist()[0]
			# Include in rotated_wrench
			rotated_wrench.extend(vector)
		return rotated_wrench


	@property
	def ft_data__unbiased_and_compensated(self):

		wrench_data = self._latest_ft_data
		current_pose = self.robot_pose

		#if self.current_bias is None:
		while self.current_bias is None:
			self.get_current_bias_from_ros_parameter_server(self.robot_id)

		if (wrench_data and current_pose and self.current_bias):
			#get unbiased and compensated wrench
			ft_unbiased_and_compensated = get_gravity_and_bias_compensated_wrench(current_pose, wrench_data['wrench'], self.current_bias)
			ft_unbiased_and_compensated = [x * self.FTS_behaviour for x in ft_unbiased_and_compensated]
			#print("ft_unbiased_and_compensated BEFORE correction", ft_unbiased_and_compensated)
			
			#correct wrench
			if self.correct_FT_frame:
				rotation = np.matrix([
					[1, 0, 0],
					[0, -1, 0],
					[0, 0, -1]]).T
				ft_unbiased_and_compensated = self.rotate_wrench(ft_unbiased_and_compensated, rotation)
			#print("ft_unbiased_and_compensated AFTER correction", ft_unbiased_and_compensated)

			#publish to topic
			if self.pub_ft_unbiased_and_compensated_toggle and ft_unbiased_and_compensated:
				self.ft_unbiased_and_compensated_topic.publish( roslibpy.Message({
					'force' : {
						'x' : ft_unbiased_and_compensated[0],
						'y' : ft_unbiased_and_compensated[1],
						'z' : ft_unbiased_and_compensated[2]
					},
					'torque' : {
						'x' : ft_unbiased_and_compensated[3],
						'y' : ft_unbiased_and_compensated[4],
						'z' : ft_unbiased_and_compensated[5]
					}
				}))
			
			#prBlue("ft_unbiased_and_compensated = " + str(ft_unbiased_and_compensated))
			return ft_unbiased_and_compensated
		else:
			prRed("ERROR: No wrench, or pose, or bias data!")
			return None

	@property
	def ft_data__unbiased_and_compensated__binary(self):
		return binaryFT(self.ft_data__unbiased_and_compensated)


	def filter_ft_data(self, data, window=250):

		# filtering list
		self._filter_ft_data.append(data)
		if (len(self._filter_ft_data) > window):
			del self._filter_ft_data[0]
		
		return np.mean( self._filter_ft_data, axis=0).tolist()


	def filtered_ft_data(self, type="raw", window=10):
		if type == "raw":
			return self.filter_ft_data( self.ft_data__raw, window )
		elif type == "unbiased":
			return self.filter_ft_data( self.ft_data__unbiased, window )
		elif type == "unbiased_and_compensated":
			return self.filter_ft_data( self.ft_data__unbiased_and_compensated, window )
		else:
			raise ValueError("ERROR: type argument not valid!")


	@static
	def binaryFT(wrench):
		# BINARY FT DATA THRESHOLDS
		f_threshold = 10.0
		t_threshold = 5.0
	
		binary_ft = [0,0,0,0,0,0]

		#Fx
		if (wrench[0] > f_threshold): binary_ft[0] = 1
		elif (wrench[0] < -f_threshold): binary_ft[0] = -1
		else: binary_ft[0] = 0
		#Fy
		if (wrench[1] > f_threshold): binary_ft[1] = 1
		elif (wrench[1] < -f_threshold): binary_ft[1] = -1
		else: binary_ft[1] = 0
		#Fz
		if (wrench[2] > f_threshold): binary_ft[2] = 1
		elif (wrench[2] < -f_threshold): binary_ft[2] = -1
		else: binary_ft[2] = 0
		#Tx
		if (wrench[3] > t_threshold): binary_ft[3] = 1
		elif (wrench[3] < -t_threshold): binary_ft[3] = -1
		else: binary_ft[3] = 0
		#Ty
		if (wrench[4] > t_threshold): binary_ft[4] = 1
		elif (wrench[4] < -t_threshold): binary_ft[4] = -1
		else: binary_ft[4] = 0
		#Tz
		if (wrench[5] > t_threshold): binary_ft[5] = 1
		elif (wrench[5] < -t_threshold): binary_ft[5] = -1
		else: binary_ft[5] = 0
		
		return binary_ft






if __name__ == '__main__':

	import threading
	from twisted.internet import reactor
	import roslibpy
	
	#ROS connection init
	ros_bridge_ip = 'localhost'     #localhost = 127.0.0.1
	ros_connection = roslibpy.Ros(host=ros_bridge_ip, port=9090)
	# Start the roslibpy reactor in a separate thread to avoid blocking main thread
	ros_thread = threading.Thread(target=reactor.run, args=(False,))
	ros_thread.daemon = True
	ros_thread.start()

	# Create deep timber controller instance
	sync_event = threading.Event()
	deep_interfaces = FTSInterfaces("robot11", ros_connection, sync_event)
	deep_interfaces.wait_ready()
	print("FTSInterfaces running")

	while True:
		print("ft_data__unbiased_and_compensated:", deep_interfaces.ft_data__unbiased)
		time.sleep(0.1)
