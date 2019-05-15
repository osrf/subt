#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import (Twist, Pose, Point, Quaternion, PoseStamped)
from mav_msgs.msg import RollPitchYawrateThrust
import tf.transformations as tf
import numpy
from threading import Lock

class Converter:
    def __init__(self):
        self.pub = None

    def callbackCmdVel(self, data):
        if self.pub is None:
            return

        vel = data

        cmd = RollPitchYawrateThrust()
        cmd.header.stamp = rospy.Time.now()
        #  cmd.header.frame_id = "X3/base_link"
        cmd.roll = -vel.linear.y
        cmd.pitch = vel.linear.x
        cmd.yaw_rate = vel.angular.z
        cmd.thrust.z = vel.linear.z
        self.pub.publish(cmd)

if __name__=="__main__":
    filtered_argv = rospy.myargv(sys.argv)

    if len(filtered_argv) != 3:
        print("Not enough args")
        exit()

    rospy.init_node('cmd_vel_converter', anonymous=True)

    converter = Converter()
    rospy.Subscriber(filtered_argv[1], Twist, converter.callbackCmdVel)
    converter.pub = rospy.Publisher(filtered_argv[2], RollPitchYawrateThrust, queue_size=1)

    print("Spinning")
    rospy.spin()
