#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h> // used for messages coming and sending mesages from and to Arduino.
ros::NodeHandle nodeHandle;
sensor_msgs::Range rangeMessage;
//creating publisher object with topic /ultrasound_range
ros::Publisher rangePublisher("/ultrasound_range",&rangeMessage);
char frameID[] = "/ultrasound"; // needed for the header part of the message
// pin number of the sensor's output
const int sensorPin = 7;
const boolean CENTIMETERS = true;
const boolean INCHES = false;

void setup() {
    nodeHandle.initNode();
    nodeHandle.advertise(rangePublisher);
    // Static parameters of the range messages:
    rangeMessage.radiation_type = sensor_msgs::Range::ULTRASOUND; // if it was IR , this value can be changed 1 to 0 (enum 1 = ULTRASOUND)
    rangeMessage.header.frame_id = frameID;
    rangeMessage.field_of_view = 0.1;
    rangeMessage.min_range = 0.002; // 2 cm
    rangeMessage.max_range = 0.150; // 150 cm
    Serial.begin(9600); // frequency range 
}
long getRange(int pinNumber,boolean inCM)
{
    // Establishing variables for duration of the sensorPin, and the distance result in inches and centimeters
    long duration, distance,inches,cm;
    // sensorPin is triggered by a high pulse of 2 or more microseconds.
    // Giving a short low pulse beforehand to ensure a clean high pulse
    pinMode(sensorPin,OUTPUT);
    digitalWrite(sensorPin,LOW);
    delayMicroseconds(2);
    digitalWrite(sensorPin,HIGH);
    delayMicroseconds(5);
    digitalWrite(sensorPin,LOW);
    // Same pin is used to read the signal from the sensorPin
    // A high pulse whose duration is the time (in ms) from the sending of the sensorPin
    // to the reception of its echo of an object.
    pinMode(sensorPin,INPUT);
    duration = pulseIn(sensorPin,HIGH);
    // convertion time into distance
    cm = microsecondsToCentimeters(duration);
    inches = microsecondsToInches(duration);    
}
void loop(){
    rangeMessage.range = getRange(sensorPin,CENTIMETERS); // updating the value of the range messages
    rangeMessage.header.stamp = nodeHandle.now(); //  updating the time stamp for the current time.
    rangePublisher.publish(&rangeMessage);
    nodeHandle.spinOnce();
    delay(500);
}
long microsecondsToInches(long microseconds){
    // According to sensor that was used in that experiment, in data sheet there
    //are 73.746 microseconds per inch.
    return ((microseconds/74)/2);
}
long microsecondsToCentimeters(long microseconds){
    // Speed of the sound is 340 m/s or 29 microseonds per centimeter.
    return ((microseconds/29)/2);
}