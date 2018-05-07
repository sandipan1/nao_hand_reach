#!/usr/bin/python
# RShoulderPitch:-2.0857 to 2.0857    # -1(highest) to 2.0(lowest)
# RShoulderRoll:-1.3265 to 0.3142     #  -1.3 to 0
# RElbowYaw:-2.0857 to 2.0857     # -2 to 2
# RElbowRoll:0.0349 to 1.5446   # 0 to 1.3
# RWristYaw:-1.8238 to 1.8238
# RHand:Open and close

import rospy

from std_msgs.msg import String

import naoqi
import time
import numpy as np
from naoqi import ALProxy,motion 

PORT =9559
robotIP="10.42.0.1"
fractionMaxSpeed=0.5

try:
	postureProxy=ALProxy("ALRobotPosture",robotIP,PORT)
except(Exception, e):
	print ("Could not create proxy to ALRobotPosture")
	print ("Error was: ", e)

postureProxy.goToPosture("Stand", 0.7)

try:
	motionProxy = ALProxy("ALMotion", robotIP, PORT)
except (Exception,e):
	print ("Could not create proxy to ALMotion")
	print ("Error was: ",e)
names=["RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll"]
angles=[0,-0.5,2,0]
angles1=[1,-0.5,1,1]

from rospy_tutorials.msg import Floats
def callback(data):
	print ("start")
	#rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
	print rospy.get_name(), "I heard %s "%str(data.data)
	print (type(data.data)) 
	dat=list(data.data)
	print (type(dat)) 

	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	motionProxy.setAngles(names,dat,fractionMaxSpeed)
	time.sleep(1.0)

	print("hello")
	motionProxy.setAngles(names,dat,fractionMaxSpeed)

	#time.sleep(2.0)
	print("bye")


def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('listener', anonymous=True)  # to avoid problem with names


	rospy.Subscriber('floats', Floats	, callback)


	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()
