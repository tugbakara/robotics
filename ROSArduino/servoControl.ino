// Hobby servo R/C controlled by sending a squarewave
// pulse of 1-2 milliseconds in width every 20 milliseconds. 
// This typically moves the servo arm from 0-180 degrees.
// In that scrpits one servo was controlled, but the same library can be used to control up
// to 12 servos on most Arduino boards and 48 on the Arduino Mega.

// #include WProgram.h is required for Arduino versions < 1.0. Since we cannot re-release all past version,
//However, since 1.0 has been released for quite some time now, you could consider to stop supporting Arduino versions older than 1.0 and only include Arduino.h.
#if defined(ARDUINO) && ARDUINO  >= 100
    #include "Arduino.h"
#else
    #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h> 

ros::NodeHandle nodeHandle;
Servo servoMotor;

void servoCallback(const std_msgs::UInt16 &cmd_msg){
    servoMotor.write(cmd_msg.data); // setting servo motor angle bwetween 0 and 180.
}
ros::Subscriber<std_msgs::UInt16> servoSubscriber("servo",servoCallback); // on every servo topic call back,we write the servos new angle to the servo object.
void setup(){
    nodeHandle.initNode();
    nodeHandle.subscrie(servorSubscriber);
    servoMotor.attach(9);
}
void loop(){
    nodeHandle.spinOnce();
    delay(1);
}

