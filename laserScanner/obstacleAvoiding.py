#!/usr/bin/env python
import rospy
from std_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

minDistance = 9999
def scanCallback(scanData):
    global minDistance
    minValue,minIndex = minRangeIndex(scanData.ranges)
    minDistance = minValue
    maxValue,maxIndex = maxRangeIndex(scanData.ranges)
    averageValue = averageRange(scanData.ranges)
    average2 = avrgBetween2Indices(scanData.ranges,2,7)
    print("The filed of view: ",fieldOfView(scanData))
def fieldOfView(scanData,i,j):
    return ((scanData.angle_max - scanData.angle_min)*180.0/math.pi)
def minRangeIndex(ranges):
    ranges = [x for x in ranges if not math.math.isnan(x)]
    if(len(ranges) != 0):
        return (min(ranges),ranges.index(min(ranges)))
    else:
        return 0.1
def maxRangeIndex(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    if(len(ranges) != 0):
        return (max(ranges),ranges.index(max(ranges)))
    else:
        return 4.0
def averageRange(ranges):
    ranges = [x for x in ranges if not math.isnan(x)]
    if(len(ranges) != 0):
        return (sum(ranges) / float(len(ranges)))
    else:
        return 0.0
def avrgBetween2Indices(ranges,i,j):
    ranges = [x for x in ranges if not math.isnan(x)]
    if(len(ranges) != 0):
        sliceOfArray = ranges[i:j+1]
        return (sum(sliceOfArray) / float(len(sliceOfArray)))
    else:
        return 0.0
def rotateRobot(avoidbObstacle):
    global minDistance
    velocityPublisher = rospy.Publisher("/cmd_vel_mux/input/teleop",Twist,queue_size = 10)
    velocityMessage = Twist()
    velocityMessage.linear.x = 0.0
    velocityMessage.angular.z = 0.6
    loopRate = rospy.Rate(10)
    if(avoidbObstacle):
        while(minDistance < 1.5):
            velocityPublisher.publish(velocityMessage)
            loopRate.sleep()    
        velocityMessage.angular.z = 0.0
        velocityPublisher.publish(velocityMessage)
    else:
        velocityPublisher.publish(velocityMessage)
        loopRate.sleep()
def moveRobot(avoidObstacle):
    global minDistance
    velocityPublisher = rospy.Publisher("/cmd_vel_mux/input/teleop",Twist,queue_size = 10)
    velocityMessage = Twist()
    loopRate = rospy.Rate(10)
    while(True):
        if(avoidObstacle):
            velocityMessage.linear.x = 0.3
            velocityMessage.angular.z = 0.0
            while(minDistance > 0.7):
                velocityPublisher.publish(velocityMessage)
                loopRate.sleep()
            velocityMessage.linear.x = 0.0
            velocityMessage.angular.z = 1.5
            while(minDistance < 1.5):
                velocityPublisher.publish(velocityMessage)
                loopRate.sleep()
        else:
            velocityPublisher.publish(velocityMessage)
            loopRate.sleep()
if __name__ == '__main__':
    rospy.init_node("avoidObstacleScan",anonymous = True)
    topicName = 'scan'
    rospy.Subscriber(topicName,LaserScan,scanCallback)
    time.sleep()
    moveRobot(True)
    rotateRobot(True)
    rospy.spin()


