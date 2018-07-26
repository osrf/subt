#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import (Twist, Pose, Point, Quaternion, PoseStamped)
import tf.transformations as tf
import numpy

def callbackCmdVel(data):
	vel = data
	newPose = currentPose
	quaternion = (newPose.orientation.x, newPose.orientation.y, newPose.orientation.z, newPose.orientation.w)
	vec = (vel.linear.x, vel.linear.y, vel.linear.z, 1)
	Rot = tf.quaternion_matrix(quaternion).tolist()
	buff = numpy.matmul(Rot, vec)
	newPose.position.x += buff[0]
	newPose.position.y += buff[1]
	newPose.position.z += buff[2]
	quaternion0 = (newPose.orientation.x, newPose.orientation.y, newPose.orientation.z, newPose.orientation.w)
	#print(quaternion0)
	quaternion1 = tf.quaternion_from_euler(vel.angular.x, vel.angular.y, vel.angular.z)
	#print(quaternion1)
	quaternion = tf.quaternion_multiply(quaternion1, quaternion0)
	#print(quaternion)
	newPose.orientation.x = quaternion[0]
	newPose.orientation.y = quaternion[1]
	newPose.orientation.z = quaternion[2]
	newPose.orientation.w = quaternion[3]
	poseStamped = PoseStamped()
	poseStamped.pose = newPose
	pub.publish(poseStamped)

if __name__=="__main__" and len(sys.argv) == 4:

	rospy.init_node('cmd_vel_converter', anonymous=True)

	rospy.Subscriber(sys.argv[1], Twist, callbackCmdVel)
	pub = rospy.Publisher(sys.argv[2], PoseStamped, queue_size=1)
	buff = eval(sys.argv[3])
	currentPose = Pose()
	currentPose.position.x = buff[0]
	currentPose.position.y = buff[1]
	currentPose.position.z = buff[2]
	currentPose.orientation.x = buff[3]
	currentPose.orientation.y = buff[4]
	currentPose.orientation.z = buff[5]
	currentPose.orientation.w = buff[6]

	rospy.spin()
