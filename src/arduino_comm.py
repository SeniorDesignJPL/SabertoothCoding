#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

wheel_addresses = [0, 1, 2, 3, 4, 5]
corner_addresses = [0, 1, 2, 3]
wheel_speeds = [0] * 6
corner_positions = [0] * 4

def callback(msg: Twist, publishers: rospy.Publisher):
	speeds = Int32MultiArray()
	turning_positions = Int32MultiArray()
	for i, _ in enumerate(wheel_addresses):
		wheel_speeds[i] = int(msg.linear.x * 1000)
	for i, _ in enumerate(corner_addresses):
		corner_positions[i] = int(msg.angular.x * 1000)

	turning_positions.data = corner_positions
	speeds.data = wheel_speeds
	publishers[0].publish(speeds)
	publishers[1].publish(corner_positions)

def turning():
	pass

def publisher():
	rospy.init_node('publisher', anonymous=True, log_level=rospy.DEBUG)

	wheel_pub = rospy.Publisher('arduino/wheels', Int32MultiArray, queue_size=100)
	corner_pub = rospy.Publisher('arduino/corners', Int32MultiArray, queue_size=100)
	rospy.Subscriber('cmd_vel', Twist, callback, callback_args=(wheel_pub, corner_pub))

	rospy.spin()

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass