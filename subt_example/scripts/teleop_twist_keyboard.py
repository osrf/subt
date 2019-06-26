#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy
import re

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

import sys, select, termios, tty, yaml, rospkg



msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u	i	o
   j	k	l
   m	,	.

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U	I	O
   J	K	L
   M	<	>

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

0-9 : select robots by index

a/s : turn lights on/off

CTRL-C to quit
"""

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
		   }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
		  }

addressBindings={
		'!':'1',
		'@':'2',
		'#':'3',
		'$':'4',
		'%':'5',
		'^':'6',
		'&':'7',
		'*':'8',
		'(':'9',
		')':'0',
}

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('teleop_twist_keyboard')

        robotNames = []
        robotAddressMap = {}
        all_topics = rospy.get_published_topics()
        for (topic, topictype) in all_topics:
                if re.match('.*/cmd_vel$', topic):
                        (beg, name, rem) = topic.split('/')
                        rospy.loginfo('Discovered {}'.format(name))
                        robotNames.append(name)
                        robotAddressMap[name] = name

	robotKeyNameMap = {}
	velPubs = {}
	commPubs = {}
	selPubs = {}
	lightPubs = {}
	addressMap = {}
	for robot in robotNames:
		key = str((len(robotKeyNameMap)+1)%10)
		robotKeyNameMap[key] = robot
		velPubs[key] = rospy.Publisher(robot + '/cmd_vel_relay', Twist, queue_size = 1)
		commPubs[key] = rospy.Publisher(robot + '/comm', String, queue_size = 1)
		selPubs[key] = rospy.Publisher(robot + '/select', Bool, queue_size = 1)
		lightPubs[key] = rospy.Publisher(robot + '/light', Bool, queue_size = 1)
		addressMap[key] = robotAddressMap[robot]
		if len(robotKeyNameMap) == 10:
			break
	currentRobotKey = '1'
	flag = Bool()
	flag.data = True
	selPubs[currentRobotKey].publish(flag)

	speed = rospy.get_param("~speed", 0.5)
	turn = rospy.get_param("~turn", 1.0)
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		print(msg)
		print('--------------------------')
		print('Robot List:')
		for i in range(0,len(robotKeyNameMap)):
			key = str((i+1)%10)
			print(key + ': ' + robotKeyNameMap[key])
		print('To send a packet: Shift+<number key>')
		print('--------------------------')
		print(vels(speed,turn))

		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				th = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print(vels(speed,turn))
				if (status == 14):
					print(msg)
				status = (status + 1) % 15
			elif key in robotKeyNameMap.keys():
				flag = Bool()
				flag.data = False
				selPubs[currentRobotKey].publish(flag)
				currentRobotKey = key
				flag.data = True
				selPubs[currentRobotKey].publish(flag)
				print("You selected " + robotKeyNameMap[key] + " to control")
				continue
			elif key in addressBindings.keys():
				commPubs[currentRobotKey].publish(addressMap[addressBindings[key]])
				continue
			elif key == 'a':
				flag = Bool()
				flag.data = True
				lightPubs[currentRobotKey].publish(flag)
				continue
			elif key == 's':
				flag = Bool()
				flag.data = False
				lightPubs[currentRobotKey].publish(flag)
				continue
			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			velPubs[currentRobotKey].publish(twist)

	except Exception as e:
		print(e)

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		for key in velPubs.keys():
			velPubs[key].publish(twist)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
