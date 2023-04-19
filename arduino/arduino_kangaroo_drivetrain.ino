#include <ros.h>
#include <Kangaroo.h>
#include <USBSabertooth.h>
#include <SoftwareSerial.h>
#include <motor_controller/Drivebase.h>
#include <std_msgs/Int32MultiArray.h>

void messageCb(const std_msgs::Int32MultiArray& msg);

struct Motor { int motorControllerAddr; int motorNum; };
struct MotorLayout { Motor motorList[6]; };

KangarooSerial K(Serial1);
KangarooChannel K_arr[4] = {
    KangarooChannel(K, '1', 131),
    KangarooChannel(K, '2', 131),
    KangarooChannel(K, '1', 132),
    KangarooChannel(K, '2', 132)
};

USBSabertoothSerial C(Serial2); // Use SWSerial as the serial port.
USBSabertooth ST[3] = {USBSabertooth(C, 128), USBSabertooth(C, 129), USBSabertooth(C, 130)};

int motor_index = 0;
int wheel_speeds[] = {0, 0, 0, 0, 0, 0};
int prev_speeds[] = {0, 0, 0, 0, 0, 0};
int corner_positions[] = {0, 0, 0, 0};
int prev_positions[] = {0, 0, 0, 0};
// FR: (128, 1), MR: (128, 2), FL: (129, 1), ML: (129, 2), BR: (130, 1), BL: (130, 2)
const MotorLayout WHEELS = {{{128, 1}, {128, 2}, {129, 1}, {129, 2}, {130, 1}, {130, 2}}};
const MotorLayout CORNERS = {{{131, 1}, {131, 2}, {132, 1}, {132, 2}}};

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Int32MultiArray> wheel_subscriber("arduino/wheels", &wheelCb);
ros::Subscriber<std_msgs::Int32MultiArray> corner_subscriber("arduino/corners", &cornerCb);

void setup() {
	nh.initNode();
	nh.subscribe(wheel_subscriber);
	nh.subscribe(corner_subscriber);
	Serial1.begin(9600);
	Serial2.begin(9600);

    for (int i = 0; i < 4; i++) {
        K_arr[i].start();
        K_arr[i].wait();
    }
}

void loop() {
	// Set the speed for the motors which need new speeds
	for (int i = 0; i < 6; i++) {
		if (prev_speeds[i] != wheel_speeds[i]) {
			motor_index = WHEELS.motorList[i].motorControllerAddr - 128;
			ST[motor_index].motor(WHEELS.motorList[i].motorNum, wheel_speeds[i]);
			prev_speeds[i] = wheel_speeds[i];
		}
	}
    for (int i = 0; i < 4; i++) {
        if (prev_positions[i] != corner_positions[i]) {
            K1.p(corner_positions[i]);
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