// FR: (128, 1), MR: (128, 2), FL: (129, 1), ML: (129, 2), BR: (130, 1), BL: (130, 2)
#include <ros.h>
#include <USBSabertooth.h>
#include <SoftwareSerial.h>
#include <motor_controller/Drivebase.h>
#include <std_msgs/Int32MultiArray.h>

void messageCb(const std_msgs::Int32MultiArray& msg);

struct Motor { int motorControllerAddr; int motorNum; };
struct MotorLayout { Motor motorList[6]; };

SoftwareSerial SWSerial(NOT_A_PIN, 16); // RX on no pin (unused), TX on pin 16 (to S1).
USBSabertoothSerial C(SWSerial); // Use SWSerial as the serial port.
USBSabertooth ST[3] = {USBSabertooth(C, 128), USBSabertooth(C, 129), USBSabertooth(C, 130)};

int motor_index = 0;
int motor_speeds[] = {0, 0, 0, 0, 0, 0};
int prev_speeds[] = {0, 0, 0, 0, 0, 0};
const MotorLayout MOTORS = {{{128, 1}, {128, 2}, {129, 1}, {129, 2}, {130, 1}, {130, 2}}};

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Int32MultiArray> motor_subscriber("arduino/motors", &messageCb);

void setup() {
	nh.initNode();
	nh.subscribe(motor_subscriber);
	SWSerial.begin(9600);
}

void loop() {
	// Set the speed for the motors which need new speeds
	for (int i = 0; i < 6; i++) {
		if (prev_speeds[i] != motor_speeds[i]) {
			motor_index = MOTORS.motorList[i].motorControllerAddr - 128;
			ST[motor_index].motor(MOTORS.motorList[i].motorNum, motor_speeds[i]);
			prev_speeds[i] = motor_speeds[i];
		}
	}
	nh.spinOnce();
	delay(1);
}

void messageCb(const std_msgs::Int32MultiArray& msg) {
	for (int i = 0; i < 6; i++) {
		motor_speeds[i] = msg.data[i];
	}
}