#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import rospy

import sys
from select import select
import termios
import tty

def getKey(settings):
	tty.setraw(sys.stdin.fileno())
	# sys.stdin.read() returns a string on Linux
	rlist, _, _ = select([sys.stdin], [], [], 0.5)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def saveTerminalSettings():
	return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def publisher():
	settings = saveTerminalSettings()
	rospy.init_node('key_publisher', anonymous=True)
	pub = rospy.Publisher('get_key', String, queue_size=100)
	rate = rospy.Rate(10) # 10hz
	try:
		while True:
			key = getKey(settings)
			if key != '':
				rospy.loginfo('Key:' + key)
				pub.publish(key)
			if (key == '\x03'): break
			rate.sleep()
	except Exception as e: print(e)
	finally: restoreTerminalSettings(settings)

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass