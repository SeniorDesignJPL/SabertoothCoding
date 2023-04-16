// Speed Control Sample for Kangaroo
// Copyright (c) 2013 Dimension Engineering LLC
// See license.txt for license details.

#include <Kangaroo.h>
#include "Sabertooth.h"

// Arduino TX (pin 11) goes to Kangaroo S1
// Arduino RX (pin 10) goes to Kangaroo S2
// Arduino GND         goes to Kangaroo 0V
// Arduino 5V          goes to Kangaroo 5V (OPTIONAL, if you want Kangaroo to power the Arduino)

// Independent mode channels on Kangaroo are, by default, '1' and '2'.
#define TX_PIN 18
#define RX_PIN 19

SoftwareSerial  SerialPort(RX_PIN, TX_PIN);
KangarooSerial  K(SerialPort);
KangarooChannel K1(K, '1', 128);
KangarooChannel K2(K, '2', 129);

void setup() {
	SerialPort.begin(9600);
	
	K1.start();
	K1.home().wait();
}

// .wait() waits until the command is 'finished'. For speed, this means it reached near the
// requested speed. You can also call K1.s(speed); without .wait() if you want to command it
// but not wait to get up to speed. If you do this, you may want to use K1.getS().value()
// to check progress.
void loop() {
	int speed = 200; 
	K1.s(speed);
	delay(5000);
	
	K1.s(-speed);
	delay(5000);
}