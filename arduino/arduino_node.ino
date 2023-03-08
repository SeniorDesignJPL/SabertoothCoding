/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <USBSabertooth.h>
#include <SoftwareSerial.h>

SoftwareSerial SWSerial(NOT_A_PIN, 18); // RX on no pin (unused), TX on pin 11 (to S1).
USBSabertoothSerial C(SWSerial);             // Use SWSerial as the serial port.
USBSabertooth ST1(C, 128); // Address 128, and use SWSerial as the serial port.
USBSabertooth ST2(C, 129); // Address 129, and use SWSerial as the serial port.

ros::NodeHandle nh;

void messageCb( const std_msgs::Int32MultiArray& msg) {
	if (msg.data[0] == 1) {
		nh.loginfo("Forward");
		ST1.motor(1, msg.data[1]);
	}
	else if (msg.data[0] == 2) {
		nh.loginfo("Backward");
		ST2.motor(1, msg.data[1]);
	}
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("motors", &messageCb);

void setup() {
	nh.initNode();
	nh.subscribe(sub);
	SWSerial.begin(9600);
}

void loop() {
	nh.spinOnce();
	delay(1);
}