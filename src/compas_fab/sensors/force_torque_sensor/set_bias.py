import roslibpy
import threading
import sys
import time
import copy
import warnings
from twisted.internet import reactor
from scipy import stats

from compensation import get_wrench_list, get_gravity_compensated_wrench, lists_subtraction
from pose import Pose



"""
### SCRIPT SKETCH:
0. print short guide on how and when to execute this script (how and in which condition to set the bias) as documentation and reminder
1. connects to /egm_feedback topic to get the current pose
2. from /netft_data topic gets N readings and avarages them to filter them (trim mean?)
3. if these topics or data are None, then stop the script and print error
4. from compensation.py, use the get_gravity_compensated_wrench(pose, ft_data) function to calculate the grrvity compensated FT data 
5. since there's no contact force, these result will be the bias!
6. set this to be the bias_parameter_value
7. print results
"""



#global variables
ft_sample_length = 500
latest_ft_data = []
latest_egm_positon = []



def AtiFtCallback(data):
	latest_ft_data.append(data)
def EgmCallback(data):
	latest_egm_positon.append(data)


def reset_bias(ros_connection = None, target_robot = 'robot11', FTS_device = 'ATI'):

	print("Resetting bias...")

	if (ros_connection is None):
		#ROS connection init
		ros_bridge_ip = 'localhost'     #localhost = 127.0.0.1
		ros_connection = roslibpy.Ros(host=ros_bridge_ip, port=9090)

		# Start the roslibpy reactor in a separate thread to avoid blocking main thread
		ros_thread = threading.Thread(target=reactor.run, args=(False,))
		ros_thread.daemon = True
		ros_thread.start()

		#wait for ROS connection to be ready
		ready = False
		while not ready:
			if ros_connection.is_connected:
				print('ROS is connected')
				ready = True
			else:
				print('ROS is connecting...')
				time.sleep(0.2)

   
	if ros_connection.is_connected:
		# Create ROS subscriber for receiving robot position through EGM
		robot_feedback_sub = roslibpy.Topic(ros_connection, "/egm_feedback", "egm_interface/egm_data")
		robot_feedback_sub.subscribe(EgmCallback)

		if FTS_device == 'ATI':
			# Create ROS subscriber for receiving ati f/t data
			ati_ft_sub = roslibpy.Topic(ros_connection, "/netft_data", "geometry_msgs/WrenchStamped")
			ati_ft_sub.subscribe(AtiFtCallback)
		elif FTS_device == 'BOTA':
			ati_ft_sub = roslibpy.Topic(ros_connection, "/ft_driver/force", "geometry_msgs/WrenchStamped")
			ati_ft_sub.subscribe(AtiFtCallback)

		#record FT data sample
		while_time = time.time()
		ft_data_sample = []
		while len(latest_ft_data) <= ft_sample_length:
			#print("latest_ft_data list len =", len(latest_ft_data))
			time.sleep(0.001)
			if len(latest_ft_data) <= ft_sample_length:
				ft_data_sample = copy.deepcopy(latest_ft_data)
			if (time.time() - while_time) > 5:   #seconds
				raise Exception("FT data recording while loop timeout!")
		else:
			print(ft_sample_length, "FT data readings have been recorded!")

		#record pose
		if len(latest_egm_positon) == 0:
			raise ValueError("ERROR: no pose data available!")
		
		latest_egm_positon_list = copy.deepcopy(latest_egm_positon)
		egm_poses_list = []
		for p in latest_egm_positon_list:
			egm_poses_list.append([p['x'], p['y'], p['z'], p['q0'], p['q1'], p['q2'], p['q3']])
		egm_pose = stats.trim_mean(egm_poses_list, 0.15).tolist()

		robot_pose = Pose.from_list(egm_pose, convention='wxyz')
		print("reset bias pose =", robot_pose)

		#filter data
		ft_data_sample__wrench = []
		for d in ft_data_sample:
			ft_data_sample__wrench.append( tuple(get_wrench_list(d['wrench'])) )
		
		warnings.filterwarnings("ignore")   #FutureWarning: Using a non-tuple sequence for multidimensional indexing is deprecated...
		ft_data_mean = (stats.trim_mean(ft_data_sample__wrench, 0.15)).tolist()
		#print("FT raw data trimmed mean =", ft_data_mean)

		#gravity compensated wrench
		ft_bias = get_gravity_compensated_wrench(robot_pose, ft_data_mean)
		print("FT bias =", ft_bias)

		
		#ROS parameter bias values
		bias_parameter_value = {
			target_robot : {
				'robotID':   target_robot,
				'timestamp': time.time(),
				'F_bias_X':  ft_bias[0],
				'F_bias_Y':  ft_bias[1],
				'F_bias_Z':  ft_bias[2],
				'T_bias_X':  ft_bias[3],
				'T_bias_Y':  ft_bias[4],
				'T_bias_Z':  ft_bias[5]
			}
		}

		#set bias parameter
		time.sleep(0.2)

		bias_parameter_name = "FT_bias"
		bias_parameter = roslibpy.Param(ros_connection, bias_parameter_name)
		bias_parameter.set(bias_parameter_value)
		print("ROS parameter", bias_parameter_name, "set to:")
		print(bias_parameter_value)
		time.sleep(0.2)


	else:
		print("ERROR: ROS could not connect!")
	



if __name__ == "__main__":

	# when executing this script, add argument to command line to specify which robot you are setting the bias for
	command_line_args = sys.argv
	try:
		target_robot = command_line_args[1]
		print("target_robot =", target_robot)
		FTS_device = command_line_args[2]
		print("FTS device =", FTS_device)
	except:
		print("ERROR: specify which robot you are setting the bias for as an argument in the command line!")
		sys.exit()

	#start bias reset function
	reset_bias(target_robot = target_robot, FTS_device = FTS_device)

	
	

