#include <USBSabertooth.h>
#include <Sabertooth.h>

#define SabertoothSerial2x5 Serial2
#define SabertoothSerial2x32 Serial3

USBSabertoothSerial C(SabertoothSerial2x32); // Use SWSerial as the serial port.
USBSabertooth USB_ST = USBSabertooth(C, 128); // Use SWSerial as the serial port.
Sabertooth ST_1 = Sabertooth(128);
Sabertooth ST_2 = Sabertooth(129);

void setup() {
	SabertoothSerial2x32.begin(9600);
	SabertoothSerial2x5.begin(38400);
	Sabertooth::autobaud(SabertoothSerial2x5);
}

void loop() {
	USB_ST.motor(1, 0);
	USB_ST.motor(2, 0);
	ST_1.motor(1, 0);
	ST_1.motor(2, 0);
	ST_2.motor(1, 0);
	ST_2.motor(2, 0);
	delay(1);
}