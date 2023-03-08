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
USBSabertooth ST1[6] = {USBSabertooth(C, 128), USBSabertooth(C, 129), USBSabertooth(C, 130), USBSabertooth(C, 131), USBSabertooth(C, 132), USBSabertooth(C, 133)};

ros::NodeHandle nh;

void messageCb (const motor_controller::Motor& msg) {
	nh.loginfo(("Addr: " + String(msg.address) + ", Speed: " + String(msg.speed)).c_str());
	ST1[msg.address].motor(1, msg.speed);
}

ros::Subscriber<motor_controller::Motor> sub("arduino/motors", &messageCb);

void setup() {
	nh.initNode();
	nh.subscribe(sub);
	SWSerial.begin(9600);
}

void loop() {
	nh.spinOnce();
	delay(1);
}