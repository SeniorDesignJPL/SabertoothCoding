#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

motor_addresses = [0, 1, 2, 3, 4, 5]
motor_speeds = [0] * 6

def callback(msg: Twist, publisher: rospy.Publisher):
	speeds = Int32MultiArray()
	for i, _ in enumerate(motor_addresses):
		motor_speeds[i] = int(msg.linear.x * 1000)
	speeds.data = motor_speeds
	publisher.publish(speeds)

def publisher():
	rospy.init_node('publisher', anonymous=True, log_level=rospy.DEBUG)

	pub = rospy.Publisher('arduino/motors', Int32MultiArray, queue_size=100)
	rospy.Subscriber('cmd_vel', Twist, callback, callback_args=pub)

	rospy.spin()

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass