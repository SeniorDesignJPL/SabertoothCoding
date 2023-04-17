// FR: (128, 1), MR: (128, 2), FL: (129, 1), ML: (129, 2), BR: (130, 1), BL: (130, 2)
// Wheel msg order: FR, MR, FL, ML,  BR, BL
// Corner msg order: FR, BR, FL, BL
#include <ros.h>
#include <USBSabertooth.h>
#include <SoftwareSerial.h>
#include <motor_controller/Drivebase.h>
#include <std_msgs/Int32MultiArray.h>

void wheelsCb(const std_msgs::Int32MultiArray& msg);
void cornersCb(const std_msgs::Int32MultiArray& msg);
float map(float min, float max, float val);

struct Motor { int motorControllerAddr; int motorNum; };
struct MotorLayout { Motor motorList[6]; };

SoftwareSerial SWSerial(NOT_A_PIN, 16); // RX on no pin (unused), TX on pin 16 (to S1).
USBSabertoothSerial C(SWSerial); // Use SWSerial as the serial port.
USBSabertooth wheels_ST[3] = {USBSabertooth(C, 128), USBSabertooth(C, 129), USBSabertooth(C, 130)};
USBSabertooth corners_ST[2] = {USBSabertooth(C, 131), USBSabertooth(C, 132)};
const int ENCODER_LIMITS[4][2] = {{1, 3}, {1, 3}, {1, 3}, {1, 3}}; // TODO: Measure these values

const float POT_TOLERANCE = 0.1; // TODO: Change to accurate tolerance
const MotorLayout WHEELS = {{{128, 1}, {128, 2}, {129, 1}, {129, 2}, {130, 1}, {130, 2}}};
const MotorLayout CORNERS = {{{131, 1}, {131, 2}, {132, 1}, {132, 2}}}; // TODO: Confirm accuracy
int wheel_index = 0;
int corner_index = 0;
int difference = 0;
int speed = 0;
int wheel_speeds[] = {0, 0, 0, 0, 0, 0};
int prev_speeds[] = {0, 0, 0, 0, 0, 0};
int corner_positions[] = {0, 0, 0, 0}; // TODO: Might need decimals
int prev_positions[] = {0, 0, 0, 0};
int encoder_pins[] = {A0, A1, A2, A3};

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Int32MultiArray> motor_subscriber("arduino/wheels", &wheelsCb);
ros::Subscriber<std_msgs::Int32MultiArray> motor_subscriber("arduino/corners", &cornersCb);

void setup() {
	nh.initNode();
	nh.subscribe(motor_subscriber);
	SWSerial.begin(9600);
}

void loop() {
	// Set the speed for the motors which need new speeds
	for (int i = 0; i < 6; i++) {
		if (prev_speeds[i] != wheel_speeds[i]) {
			wheel_index = WHEELS.motorList[i].motorControllerAddr - 128;
			wheels_ST[wheel_index].motor(WHEELS.motorList[i].motorNum, wheel_speeds[i]);
			prev_speeds[i] = wheel_speeds[i];
		}
	}
	for (int i = 0; i < 4; i++) {
		corner_index = CORNERS.motorList[i].motorControllerAddr - 128;
		difference = analogRead(encoder_pins[i]) - map(ENCODER_LIMITS[i][1], ENCODER_LIMITS[i][2], corner_positions[i]);
		if (difference > POT_TOLERANCE) speed = 100;
		else if (difference < -POT_TOLERANCE) speed = -100;
		else speed = 0;
		corners_ST[corner_index].motor(CORNERS.motorList[i].motorNum, speed);
	}
	nh.spinOnce();
	delay(1);
}

void wheelsCb(const std_msgs::Int32MultiArray& msg) {
	for (int i = 0; i < 6; i++) {
		wheel_speeds[i] = msg.data[i];
	}
}

void cornersCb(const std_msgs::Int32MultiArray& msg) {
	for (int i = 0; i < 4; i++) {
		corner_positions[i] = msg.data[i]
	}
}

float map(float min, float max, float val) {
	const float MAX = 100.0;
	if (val > MAX) return 100.0;
	else if (val < 0) return 0;
	return val / MAX * (max - min) + min;
}