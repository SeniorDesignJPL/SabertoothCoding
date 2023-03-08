#!/usr/bin/env python3
from ast import List
import rospy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from motor_controller.msg import Motor
from enum import Enum

class Wheels():
	FRONT_LEFT = 0
	FRONT_RIGHT = 1
	CENTER_LEFT = 2
	CENTER_RIGHT = 3
	BACK_LEFT = 4
	BACK_RIGHT = 5

motor_addresses = [0, 1, 2, 3, 4, 5]
motor_speeds = [0] * 6

def setSpeeds(msg: Twist):
	motor_speeds[Wheels.FRONT_LEFT] = 2000
	motor_speeds[Wheels.FRONT_RIGHT] = 2000
	motor_speeds[Wheels.CENTER_LEFT] = 2000
	motor_speeds[Wheels.CENTER_RIGHT] = 2000
	motor_speeds[Wheels.BACK_LEFT] = 2000
	motor_speeds[Wheels.BACK_RIGHT] = 2000

def callback(msg: Twist, publisher: rospy.Publisher):
	motor = Motor()
	setSpeeds(msg)

	motor = Motor()
	for i, addr in enumerate(motor_addresses):
		motor.address = addr
		motor.speed = motor_speeds[i]
		# rospy.logdebug('Speed: ' + str(motor.speed))
		# rospy.logdebug('Addr: ' + str(motor.address))
		publisher.publish(motor)

def publisher():
	rospy.init_node('publisher', anonymous=True, log_level=rospy.DEBUG)
	pub = rospy.Publisher('arduino/motors', Motor, queue_size=100)
	rospy.Subscriber("cmd_vel", Twist, callback, callback_args=pub)
	rospy.spin()

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass