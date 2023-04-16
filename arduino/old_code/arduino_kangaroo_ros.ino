// Speed Control Sample for Kangaroo
// Copyright (c) 2013 Dimension Engineering LLC
// See license.txt for license details.

#include <Kangaroo.h>
#include "Sabertooth.h"
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <motor_controller/Motor.h>

// Arduino TX (pin 11) goes to Kangaroo S1
// Arduino RX (pin 10) goes to Kangaroo S2
// Arduino GND         goes to Kangaroo 0V
// Arduino 5V          goes to Kangaroo 5V (OPTIONAL, if you want Kangaroo to power the Arduino)

// Independent mode channels on Kangaroo are, by default, '1' and '2'.
KangarooSerial  K(Serial1);
KangarooChannel K1(K, '1', 128);
KangarooChannel K2(K, '2', 128);
// KangarooChannel K_MOTORS[2] = { KangarooChannel(K, '1', 128), KangarooChannel(K, '2', 128) }

int motor_speeds[] = {0, 0, 0, 0, 0, 0};
int prev_speeds[] = {0, 0, 0, 0, 0, 0};

ros::NodeHandle nh;

void messageCb (const motor_controller::Motor& msg) {
	motor_speeds[msg.address] = msg.speed;
}

ros::Subscriber<motor_controller::Motor> sub("arduino/motors", &messageCb);

void setup() {
	nh.initNode();
	nh.subscribe(sub);
	Serial1.begin(9600);
	
	K1.start();
	K2.start();
	K1.home().wait();
	K2.home().wait();
}

// .wait() waits until the command is 'finished'. For speed, this means it reached near the
// requested speed. You can also call K1.s(speed); without .wait() if you want to command it
// but not wait to get up to speed. If you do this, you may want to use K1.getS().value()
// to check progress.
void loop() {
	if (prev_speeds[0] != motor_speeds[0]) {
		K1.s(motor_speeds[0]);
		prev_speeds[0] = motor_speeds[0];
	}
	if (prev_speeds[1] != motor_speeds[1]) {
		K2.s(motor_speeds[1]);
		prev_speeds[1] = motor_speeds[1];
	}
	nh.spinOnce();
	delay(1);
}