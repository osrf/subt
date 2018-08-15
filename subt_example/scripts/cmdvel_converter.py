#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import (Twist, Pose, Point, Quaternion, PoseStamped)
import tf.transformations as tf
import numpy
from threading import Lock

class Converter:
	def __init__(self):
		self.currentPose = None
		self.pub = None
		self.mutex = Lock()

	def callbackOdom(self, data):
		self.mutex.acquire()
		self.currentPose = data
		self.mutex.release()

	def callbackCmdVel(self, data):
		self.mutex.acquire()
		if self.currentPose is None or self.pub is None:
			self.mutex.release()
			return
		newPose = self.currentPose
		self.mutex.release()
		vel = data

		quaternion = (newPose.orientation.x, newPose.orientation.y, newPose.orientation.z, newPose.orientation.w)
		vec = (vel.linear.x, vel.linear.y, vel.linear.z, 1)
		Rot = tf.quaternion_matrix(quaternion).tolist()
		buff = numpy.matmul(Rot, vec)
		newPose.position.x += buff[0]
		newPose.position.y += buff[1]
		newPose.position.z += buff[2]

		quaternion0 = (newPose.orientation.x, newPose.orientation.y, newPose.orientation.z, newPose.orientation.w)
		quaternion1 = tf.quaternion_from_euler(vel.angular.x, vel.angular.y, vel.angular.z)
		quaternion = tf.quaternion_multiply(quaternion1, quaternion0)
		newPose.orientation.x = quaternion[0]
		newPose.orientation.y = quaternion[1]
		newPose.orientation.z = quaternion[2]
		newPose.orientation.w = quaternion[3]

		poseStamped = PoseStamped()
		poseStamped.pose = newPose
		self.pub.publish(poseStamped)

if __name__=="__main__":
	filtered_argv = rospy.myargv(sys.argv)

	if len(filtered_argv) != 4:
		exit()

	rospy.init_node('cmd_vel_converter', anonymous=True)

	converter = Converter()
	rospy.Subscriber(filtered_argv[1], Pose, converter.callbackOdom)
	rospy.Subscriber(filtered_argv[2], Twist, converter.callbackCmdVel)
	converter.pub = rospy.Publisher(filtered_argv[3], PoseStamped, queue_size=1)

	rospy.spin()
