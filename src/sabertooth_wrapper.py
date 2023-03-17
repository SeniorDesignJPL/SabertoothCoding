import math
import rospy
from msg import CommandDrive, CommandCorner, Motor

class SabertoothWrapper(object):
	"""Interface between the Sabertooth motor drivers and the higher level rover code"""

	def __init__(self):
		rospy.loginfo("Initializing motor controllers")

		self.motor_addresses = [0, 1, 2, 3, 4, 5]
		self.sabertooth_drive_indices = ["drive_left_front", "drive_left_middle", "drive_left_back", "drive_right_front", "drive_right_middle", "drive_right_back"]
		self.sabertooth_corner_indices = ["corner_left_front", "corner_left_back", "corner_right_front", "corner_right_back"]

		self.corner_cmd_buffer = None
		self.drive_cmd_buffer = None
		self.sabertooth_mapping = rospy.get_param('~sabertooth_mapping')

		self.velocity_timeout = rospy.Duration(rospy.get_param('/velocity_timeout', 2.0))

		self.stop_motors()

		# set up publishers and subscribers
		self.motor_pub = rospy.Publisher('arduino/motors', Motor, queue_size=10)
		self.corner_cmd_sub = rospy.Subscriber("/cmd_corner", CommandCorner, self.corner_cmd_cb, queue_size=1) # TODO: May need to increase queue size
		self.drive_cmd_sub = rospy.Subscriber("/cmd_drive", CommandDrive, self.drive_cmd_cb, queue_size=1)

	def run(self):
		"""Blocking loop which runs after initialization has completed"""
		rate = rospy.Rate(8)

		# time last command was executed in the run loop
		time_last_cmd = rospy.Time.now()

		while not rospy.is_shutdown():
			now = rospy.Time.now()

			# Check to see if there are commands in the buffer to send to the motor controller
			if self.drive_cmd_buffer:
				drive_fcn = self.send_drive_buffer_velocity
				drive_fcn(self.drive_cmd_buffer)
				self.drive_cmd_buffer = None
				time_last_cmd = now
				idle_ramp = False
				idle = False
				
			if self.corner_cmd_buffer:
				self.send_corner_buffer(self.corner_cmd_buffer)
				self.corner_cmd_buffer = None
				time_last_cmd = now
				idle_ramp = False
				idle = False

			# TODO: Subscribe to encoder values
			# read from sabertooths and publish
			# try:
			# 	self.read_encoder_values()
			# 	self.enc_pub.publish(self.current_enc_vals)
			# except AssertionError as read_exc:
			# 	rospy.logwarn( "Failed to read encoder values")

			# stop the motors if we haven't received a command in a while
			if not idle and (now - time_last_cmd > self.velocity_timeout):
				# rather than a hard stop, send a ramped velocity command to 0
				if not idle_ramp:
					rospy.loginfo( "Idling: ramping down velocity to zero")
					idle_ramp = True
					drive_cmd_buffer = CommandDrive()
					self.send_drive_buffer_velocity(drive_cmd_buffer)
				# if we've already ramped down, send a full stop to minimize
				# idle power consumption
				else:
					rospy.loginfo( "Idling: full stopping motors")
					self.stop_motors()
					idle = True
				
				# so that there's a delay between ramping and full stop
				time_last_cmd = now

			rate.sleep()

	def corner_cmd_cb(self, cmd):
		"""
		Takes the corner command and stores it in the buffer to be sent
		on the next iteration of the run() loop.
		"""
		rospy.logdebug("Corner command callback received: {}".format(cmd))
		self.corner_cmd_buffer = [cmd.left_front_pos, cmd.left_back_pos, cmd.right_front_pos, cmd.right_back_pos]

	def drive_cmd_cb(self, cmd):
		"""
		Takes the drive command and stores it in the buffer to be sent
		on the next iteration of the run() loop.
		"""
		rospy.logdebug("Drive command callback received: {}".format(cmd))
		self.drive_cmd_buffer = [cmd.left_front_vel, cmd.left_front_vel, cmd.left_middle_vel, cmd.right_front_vel, cmd.right_middle_vel, cmd.right_back_vel]

	def send_corner_buffer(self, cmd):
		"""
		Sends the corner command to the motor controller.
		"""
		for i, motor_cmd in enumerate(cmd):
			# Convert position to tick
			encmin, encmax = self.encoder_limits[self.sabertooth_corner_indices[i]]
			motor_tick = self.position2tick(motor_cmd, encmin, encmax,
											self.sabertooth_mapping[self.sabertooth_corner_indices[i]]["ticks_per_rev"],
											self.sabertooth_mapping[self.sabertooth_corner_indices[i]]["gear_ratio"])
			props = self.sabertooth_mapping[i]
			vel_cmd = self.velocity2qpps(motor_cmd, props["ticks_per_rev"], props["gear_ratio"])
			self.send_velocity_cmd(props["address"], props["channel"], vel_cmd)

			# Send position
			self.send_position_cmd(self.sabertooth_mapping[self.sabertooth_corner_indices[i]]["address"],
							   	self.sabertooth_mapping[self.sabertooth_corner_indices[i]]["channel"],
							   	motor_tick)

	def send_drive_buffer_velocity(self, cmd):
		"""
		Sends the drive command to the motor controller, closed loop velocity commands
		"""
		for i, motor_cmd in enumerate(cmd):
			props = self.sabertooth_mapping[self.sabertooth_drive_indices[i]]
			vel_cmd = self.velocity2qpps(motor_cmd, props["ticks_per_rev"], props["gear_ratio"])
			self.send_velocity_cmd(props["address"], props["channel"], vel_cmd)

	def position2tick(self, position, enc_min, enc_max, ticks_per_rev, gear_ratio):
		"""
		Convert the absolute position from radian relative to the middle position to ticks
				Clip values that are outside the range [enc_min, enc_max]
		:param position:
		:param enc_min:
		:param enc_max:
		:param ticks_per_rev:
		:return:
		"""
		ticks_per_rad = ticks_per_rev / (2 * math.pi)
		if enc_min is None or enc_max is None:
			return position * ticks_per_rad
		mid = enc_min + (enc_max - enc_min) / 2
		tick = int(mid + position * ticks_per_rad / gear_ratio)

		return max(enc_min, min(enc_max, tick))

	def velocity2qpps(self, velocity, ticks_per_rev, gear_ratio):
		"""
		Convert the given velocity to quadrature pulses per second
		:param velocity: rad/s
		:param ticks_per_rev:
		:param gear_ratio:
		:return: int
		"""
		return int(velocity * gear_ratio * ticks_per_rev / (2 * math.pi))
	
	#TODO: Fix this for Saberteeth motor controllers
	def send_position_cmd(self, address, channel, target_tick):
		"""
		Wrapper around one of the send position commands
		:param address:
		:param channel:
		:param target_tick: int
		"""
		cmd_args = [self.corner_accel, self.corner_max_vel, self.corner_accel, target_tick, 1]
		if channel == "M1":
			return self.rc.SpeedAccelDeccelPositionM1(address, *cmd_args)
		elif channel == "M2":
			return self.rc.SpeedAccelDeccelPositionM2(address, *cmd_args)
		else:
			raise AttributeError("Received unknown channel '{}'. Expected M1 or M2".format(channel))
	
	#TODO: Fix this for Saberteeth motor controllers
	def send_velocity_cmd(self, address, channel, target_qpps):
		"""
		Wrapper around one of the send velocity commands (QPPS: quadrature pulses per second)
		:param address:
		:param channel:
		:param target_qpps: int
		"""
		# clip values
		target_qpps = max(-self.roboclaw_overflow, min(self.roboclaw_overflow, target_qpps))
		if channel == "M1":
			return self.rc.SpeedAccelM1(address, self.drive_accel, target_qpps)
		elif channel == "M2":
			return self.rc.SpeedAccelM2(address, self.drive_accel, target_qpps)
		else:
			raise AttributeError("Received unknown channel '{}'. Expected M1 or M2".format(channel))

	def stop_motors(self):
		"""Stops all motors on Rover"""
		motor = Motor()

		for addr in self.motor_addresses:
			motor.address = addr
			motor.speed = 0
			self.motor_pub.publish(motor)


if __name__ == "__main__":
	rospy.init_node("Sabertooth Wrapper", log_level=rospy.INFO)
	rospy.loginfo("Starting the Sabertooth wrapper node")

	wrapper = SabertoothWrapper()
	rospy.on_shutdown(wrapper.stop_motors)
	wrapper.run()

