#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <USBSabertooth.h>
#include <SoftwareSerial.h>

SoftwareSerial SWSerial(NOT_A_PIN, 18); // RX on no pin (unused), TX on pin 11 (to S1).
USBSabertoothSerial C(SWSerial); // Use SWSerial as the serial port.
USBSabertooth ST1[2] = {USBSabertooth(C, 128), USBSabertooth(C, 130)};

ros::NodeHandle nh;

void messageCb( const std_msgs::Int32MultiArray& msg) {
	nh.loginfo(String(msg.data[0]).c_str());
	nh.loginfo(String(msg.data[1]).c_str());
	if (msg.data[0] == 1) {
		nh.loginfo("Forward");
		ST1[0].motor(1, msg.data[1]);
	}
	else if (msg.data[0] == 2) {
		nh.loginfo("Backward");
		ST1[1].motor(1, msg.data[1]);
	}
}

ros::Subscriber<std_msgs::Int32MultiArray> sub("arduino/motors", &messageCb);

void setup() {
	nh.initNode();
	nh.subscribe(sub);
	SWSerial.begin(9600);
}

void loop() {
	nh.spinOnce();
	delay(1);
}