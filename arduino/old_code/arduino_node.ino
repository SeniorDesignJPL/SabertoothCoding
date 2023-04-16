/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <motor_controller/Motor.h>
#include <USBSabertooth.h>
#include <SoftwareSerial.h>

SoftwareSerial SWSerial(NOT_A_PIN, 18); // RX on no pin (unused), TX on pin 11 (to S1).
USBSabertoothSerial C(SWSerial); // Use SWSerial as the serial port.
USBSabertooth ST[3] = {USBSabertooth(C, 128), USBSabertooth(C, 129), USBSabertooth(C, 130)};
int motor_speeds[] = {0, 0, 0, 0, 0, 0};
int prev_speeds[] = {0, 0, 0, 0, 0, 0};

ros::NodeHandle nh;

void messageCb (const motor_controller::Motor& msg) {
	nh.loginfo(
		("Controller: " + String(msg.address / 2) +
		", Motor: " + String((msg.address % 2) + 1) +
		", Speed: " + String(msg.speed)).c_str()
	);
	motor_speeds[msg.address] = msg.speed;
	// ST[msg.address / 2].motor((msg.address % 2) + 1, msg.speed);
}

ros::Subscriber<motor_controller::Motor> sub("arduino/motors", &messageCb);

void setup() {
	nh.initNode();
	nh.subscribe(sub);
	SWSerial.begin(9600);
}

void loop() {
	if (prev_speeds[0] != motor_speeds[0]) {
		ST[0].motor(1, motor_speeds[0]);
		prev_speeds[0] = motor_speeds[0];
	}
	if (prev_speeds[2] != motor_speeds[2]) {
		ST[1].motor(1, motor_speeds[2]);
		prev_speeds[2] = motor_speeds[2];
	}
	nh.spinOnce();
	delay(1);
}