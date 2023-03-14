#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from motor_controller.msg import Motor

motor_addresses = [0, 1, 2, 3, 4, 5]
motor_speeds = [0] * 6

def callback(msg: Twist, publisher: rospy.Publisher):
	motor = Motor()
	for i, addr in enumerate(motor_addresses):
		motor.address = addr
		motor.speed = int(msg.linear.x * 1000)
		publisher.publish(motor)

def publisher():
	rospy.init_node('publisher', anonymous=True, log_level=rospy.DEBUG)

	pub = rospy.Publisher('arduino/motors', Motor, queue_size=100)
	rospy.Subscriber('cmd_vel', Twist, callback, callback_args=pub)

	rospy.spin()

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass