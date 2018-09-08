#!/usr/bin/env python
# license removed for brevity

# ---> example file for testing publishing and subscribing to ROS Topics from within Grasshopper
# ---> run e02publisher_topic_poses_ros.py file in ROS
# ---> subscribe to ROS Topic \poses in Grasshopper

import rospy
from geometry_msgs.msg import PoseArray, Pose
import math

def posetalker():
    pub = rospy.Publisher('poses', PoseArray, queue_size=2)
    rospy.init_node('posetalker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    counter = 0
    while not rospy.is_shutdown():
    	counter += 0.1
    	msg = PoseArray()

    	msg.header.stamp = rospy.Time.now()
    	num = 30
    	for j in range(num/2):
    		for i in range(num):
	    		pose = Pose()
	    		pose.position.x = j * 0.26
	    		pose.position.y = i * 0.13 
	    		pose.position.z = math.sin(i+j+counter) + math.cos(j+i+counter)
	    		pose.orientation.w = 1.0

	    		msg.poses.append(pose)

    	#hello_str = "hello world %s" % rospy.get_time()
    	rospy.loginfo(msg)
    	pub.publish(msg)
    	rate.sleep()

if __name__ == '__main__':
    try:
        posetalker()
    except rospy.ROSInterruptException:
        pass