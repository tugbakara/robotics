## Description
In this folder, using rosserial_arduino which is rosserial_client library and rosserai_python which is ROS side interface were used to communicate between microcontroller and ROS ecosysstem and related to that topic, some scripts have been tried on Arduino Uno.

# NOTE:
If you use different sensors, you need to use different sensor ROS messages!

### Executing:
- for ultrasonic sensor
In the terminal you can use these commands with given sequence or for more rapid executing you can create .launch file which includes all the commands written below.
I. roscore
II. rosrun rosserial_python serial_node.py portNumberPath/ofTheArduinotoROSEcosystem (starting the server and ublishing message)
III. rostopic info /ultrasound_range (if you wonder you can see with that command which topics are using after calling the rosserial_python)
IV. rostopic echo /ultrasound_range ( now you can see the outcomes from sensor)
