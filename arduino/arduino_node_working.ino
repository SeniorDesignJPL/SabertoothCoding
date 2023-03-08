/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <USBSabertooth.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>

SoftwareSerial SWSerial(NOT_A_PIN, 18); // RX on no pin (unused), TX on pin 11 (to S1).
USBSabertoothSerial C(SWSerial);             // Use SWSerial as the serial port.
USBSabertooth ST1(C, 128); // Address 128, and use SWSerial as the serial port.
USBSabertooth ST2(C, 129); // Address 129, and use SWSerial as the serial port.

ros::NodeHandle nh;

int a = 0;

void messageCb( const std_msgs::UInt16& msg){
  a = msg.data;
  if (msg.data == 1) {
    nh.loginfo("If");
    ST1.motor(1, 2000);
    ST2.motor(1, 2000);
  }
}

ros::Subscriber<std_msgs::UInt16> sub("toggle_led", &messageCb );

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  SWSerial.begin(9600);
}

void loop() {
  nh.spinOnce();
  delay(1);
}