#include <ros.h>
#include <Kangaroo.h>
#include <USBSabertooth.h>
#include <Sabertooth.h>
#include <std_msgs/Int32MultiArray.h>

#define KangarooSerialPort Serial1
#define SabertoothSerial2x5 Serial2
#define SabertoothSerial2x32 Serial3

void wheelCb(const std_msgs::Int32MultiArray& msg);
void cornerCb(const std_msgs::Int32MultiArray& msg);

KangarooSerial K(KangarooSerialPort);

KangarooChannel K1(K, '1', 131);
KangarooChannel K2(K, '2', 131);
KangarooChannel K3(K, '1', 132);
KangarooChannel K4(K, '2', 132);

USBSabertoothSerial C(SabertoothSerial2x32); // Use SWSerial as the serial port.
USBSabertooth USB_ST = USBSabertooth(C, 128); // Use SWSerial as the serial port.
Sabertooth ST_1 = Sabertooth(128);
Sabertooth ST_2 = Sabertooth(129);

long wheel_speeds[] = {0, 0, 0, 0, 0, 0};
long prev_speeds[] = {0, 0, 0, 0, 0, 0};
int corner_positions[] = {0, 0, 0, 0};
int prev_positions[] = {0, 0, 0, 0};

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Int32MultiArray> wheel_subscriber("arduino/wheels", &wheelCb);
ros::Subscriber<std_msgs::Int32MultiArray> corner_subscriber("arduino/corners", &cornerCb);

void setup() {
	nh.initNode();
	nh.subscribe(wheel_subscriber);
	nh.subscribe(corner_subscriber);

	// Setup all the serial ports needed
	KangarooSerialPort.begin(9600);
	SabertoothSerial2x5.begin(38400);
	Sabertooth::autobaud(SabertoothSerial2x5);
	SabertoothSerial2x32.begin(9600);

	// Setup and home all the kangaroos
	KangarooMonitor  K1Monitor, K2Monitor, K3Monitor, K4Monitor;
	KangarooMonitor* monitorList[4] = { &K1Monitor, &K2Monitor, &K3Monitor, &K4Monitor };

	K1.start();
	K2.start();
	K3.start();
	K4.start();
	K1Monitor = K1.home();
	K2Monitor = K2.home();
	K3Monitor = K3.home();
	K4Monitor = K4.home();

	waitAll(4, monitorList);
}

void loop() {
	// Set the speed for the motors which need new speeds
	for (int i = 0; i < 6; i++) {
		if (prev_speeds[i] != wheel_speeds[i]) {
			switch (i) {
				case 0: USB_ST.motor(1, wheel_speeds[i]);
				case 1: USB_ST.motor(2, wheel_speeds[i]);
				case 2: ST_1.motor(1, wheel_speeds[i]*127/2047); nh.loginfo(("Wheel: " + String(wheel_speeds[i]*127/2047)).c_str());
				case 3: ST_1.motor(2, wheel_speeds[i]*127/2047);
				case 4: ST_2.motor(1, wheel_speeds[i]*127/2047);
				case 5: ST_2.motor(2, wheel_speeds[i]*127/2047);
			}
			prev_speeds[i] = wheel_speeds[i];
		}
	}
	// Set the corner motor positions using kangaroos
	for (int i = 0; i < 4; i++) {
		if (prev_positions[i] != corner_positions[i]) {
			switch (i) {
				case 1: K1.p(corner_positions[i]);
				case 2: K2.p(corner_positions[i]);
				case 3: K3.p(corner_positions[i]);
				case 4: K4.p(corner_positions[i]);
			}
			prev_positions[i] = corner_positions[i];
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

void cornerCb(const std_msgs::Int32MultiArray& msg) {
	for (int i = 0; i < 4; i++) {
		corner_positions[i] = msg.data[i];
	}
}