#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import (Twist, Pose, Point, Quaternion, PoseStamped)
import tf.transformations as tf

currentPose=Pose(Point(0,0,0),Quaternion(0,0,0,0))
pub=""

def callbackPose(data):
	currentPose = data

def callbackCmdVel(data):
	vel = data
	newPose = currentPose
	newPose.position.x += vel.linear.x
	newPose.position.y += vel.linear.y
	newPose.position.z += vel.linear.z
	quaternion = (newPose.orientation.x, newPose.orientation.y, newPose.orientation.z, newPose.orientation.w)
	quaternion = tf.quaternion_multiply(quaternion, tf.quaternion_from_euler(vel.angular.x, vel.angular.y, vel.angular.z))
	newPose.orientation.x = quaternion[0]
	newPose.orientation.y = quaternion[1]
	newPose.orientation.z = quaternion[2]
	newPose.orientation.w = quaternion[3]
	poseStamped = PoseStamped()
	poseStamped.pose = newPose
	pub.publish(poseStamped)

if __name__=="__main__" and len(sys.argv) == 4:

	rospy.init_node('cmd_vel_converter', anonymous=True)

	rospy.Subscriber(sys.argv[1] + "/pose", Pose, callbackPose)
	rospy.Subscriber(sys.argv[2] + "/cmd_vel", Twist, callbackCmdVel)
	pub = rospy.Publisher(sys.argv[3] + '/pose', PoseStamped)

	rospy.spin()
