#include <USBSabertooth.h>
#include <SoftwareSerial.h>

SoftwareSerial SWSerial(NOT_A_PIN, 16); // RX on no pin (unused), TX on pin 16 (to S1).
USBSabertoothSerial C(SWSerial); // Use SWSerial as the serial port.
USBSabertooth ST1[3] = {USBSabertooth(C, 128), USBSabertooth(C, 129), USBSabertooth(C, 130)};

void setup() {
	SWSerial.begin(9600);
}

void loop() {
	ST1[0].motor(1, 500);
	ST1[0].motor(2, 20);
	ST1[1].motor(1, 20);
	ST1[1].motor(2, 20);
	ST1[2].motor(1, 20);
	ST1[2].motor(2, 20);
	delay(1);
}