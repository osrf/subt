#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import (Twist, Pose, Point, Quaternion, PoseStamped)
import tf

currentPose=Pose(Point(0,0,0),Quaternion(0,0,0,0))
pub=""

def callbackPose(data):
	currentPose = data

def callbackCmdVel(data):
	vel = data
	newPose = currentPose
	newPose.position += vel.linear
	q = Quaternion(vel.angular.x, vel.angular.y, vel.angular.z)
	newPose.orientation *= q
	poseStamped = PoseStamped()
	poseStamped.pose = newPose
	pub.publish(poseStamped)

if __name__=="__main__" and len(sys.argv) == 4:

	rospy.init_node('cmd_vel_converter', anonymous=True)

	rospy.Subscriber(sys.argv[1] + "/pose", Pose, callbackPose)
	rospy.Subscriber(sys.argv[2] + "/cmd_vel", Twist, callbackCmdVel)
	pub = rospy.Publisher(sys.argv[3] + '/pose', PoseStamped, queue_size = 1)

	rospy.spin()
