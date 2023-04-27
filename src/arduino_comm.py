#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray, String

FR = 0
MR = 1
BR = 2
FL = 3
ML = 4
BL = 5

speed = 800
turn = 400

class Robot:
	def __init__(self) -> None:
		self.publisher = rospy.Publisher('arduino/wheels', Int32MultiArray, queue_size=100)
		self.key_sub = rospy.Subscriber('get_key', String, self.callback)
		self.speeds = [0] * 6
		self.prev_speeds = [0] * 6
		self.speed_msg = Int32MultiArray()

	def callback(self, msg: String):
		rospy.loginfo('Key: ' + str(msg.data))
		key = msg.data
		left_speed = 0
		right_speed = 0
		if key == 'w':
			left_speed = speed
			right_speed = speed
		elif key == 's':
			left_speed = -speed
			right_speed = -speed
		elif key == 'd':
			left_speed = speed + turn
			right_speed = speed
		elif key == 'a':
			left_speed = speed
			right_speed = speed + turn
		else:
			left_speed = 0
			right_speed = 0
		
		print('Speeds... Left: ' + str(left_speed) + ' Right: ' + str(right_speed))

		self.speeds[FL] = int(left_speed)
		self.speeds[ML] = int(left_speed)
		self.speeds[BL] = int(left_speed)
		self.speeds[FR] = int(right_speed)
		self.speeds[MR] = int(right_speed)
		self.speeds[BR] = int(right_speed)

	def start(self):
		rospy.init_node('publisher', anonymous=True)
		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			self.speed_msg.data = self.speeds
			if self.speeds != self.prev_speeds:
				rospy.loginfo('Publish')
				self.publisher.publish(self.speed_msg)
				self.prev_speeds = self.speeds.copy()
			rate.sleep()

if __name__ == '__main__':
	robot = Robot()
	try:
		robot.start()
	except rospy.ROSInterruptException:
		pass