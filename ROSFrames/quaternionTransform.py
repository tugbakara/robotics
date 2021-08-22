#!/usr/bin/env python
# In this example, convertions occured between rpy
# (roll-pitch-yaw) and quternion values by giving arbitrary values.
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf

roll = math.radians(30)
pitch = math.radians(42)
yaw = math.radians(58)
quaternionValues = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
print("Using quaternion_from_euler function, the result is:")
for i in range(4):
    print(quaternionValues[i])
# opposite conversion to see the same values which is given above 10th to 12th statements

rpy = tf.transformations.euler_from_quaternion(quaternionValues)
rollFromQ = rpy[0]
pitchFromQ = rpy[1]
yawFromQ = rpy[2]
print("Opposite conversion using euler_from_quaternion: ")
print " roll = ", math.degrees(rollFromQ), " pitch = ", math.degrees(pitchFromQ), " yaw = ", math.degrees(yawFromQ)

print("Defining new quaternion values by giving arbitrary values in a list : ")
nQuaternionValues = (-3.88256895463e-06,0.0015896463485,0.001397167245,0.0)
rpy = tf.transformations.euler_from_quaternion(nQuaternionValues)
rollFromQ = rpy[0]
pitchFromQ = rpy[1]
yawFromQ = rpy[2]
print " roll = ",math.degrees(rollFromQ), " pitch = ",math.degrees(pitchFromQ), " yaw = ",math.degrees(yawFromQ)