#include <ros.h>
#include <USBSabertooth.h>
#include <Sabertooth.h>
#include <std_msgs/Int32MultiArray.h>

#define SabertoothSerial2x5 Serial2
#define SabertoothSerial2x32 Serial3

void wheelCb(const std_msgs::Int32MultiArray& msg);

USBSabertoothSerial C(SabertoothSerial2x32); // Use SWSerial as the serial port.
USBSabertooth USB_ST = USBSabertooth(C, 128); // Use SWSerial as the serial port.
Sabertooth ST_1 = Sabertooth(128);
Sabertooth ST_2 = Sabertooth(129);

long wheel_speeds[] = {0, 0, 0, 0, 0, 0};
long prev_speeds[] = {0, 0, 0, 0, 0, 0};

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Int32MultiArray> wheel_subscriber("arduino/wheels", &wheelCb);

void setup() {
	nh.initNode();
	nh.subscribe(wheel_subscriber);

	// Setup all the serial ports needed
	SabertoothSerial2x5.begin(38400);
	Sabertooth::autobaud(SabertoothSerial2x5);
	SabertoothSerial2x32.begin(9600);
}

void loop() {
	// Set the speed for the motors which need new speeds
	for (int i = 0; i < 6; i++) {
		if (prev_speeds[i] != wheel_speeds[i]) {
			// nh.loginfo(("Wheel: " + String(wheel_speeds[i]*127/2047)).c_str());
			switch (i) {
				case 0: USB_ST.motor(1, wheel_speeds[i]);
				case 1: USB_ST.motor(2, wheel_speeds[i]);
				case 2: ST_1.motor(1, wheel_speeds[i]*127/2047);
				case 3: ST_1.motor(2, wheel_speeds[i]*127/2047);
				case 4: ST_2.motor(1, wheel_speeds[i]*127/2047);
				case 5: ST_2.motor(2, wheel_speeds[i]*127/2047);
			}
			prev_speeds[i] = wheel_speeds[i];
		}
	}

	nh.spinOnce();
	delay(1);
}

void wheelCb(const std_msgs::Int32MultiArray& msg) {
	for (int i = 0; i < 6; i++) {
		wheel_speeds[i] = msg.data[i];
	}
}
