#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist # to take linear and angular velocities
from turtlesim.msg import Pose # to take current position of robot in every sequence that we decided before.
import math # to calculate distance moved by the robot
import time # to make PID control 

# 4th step: functions that is needed for the movements have been written and coordinates for the Pose messages have been assigned.
x, y = 0
yaw = 0 

def poseCallback (poseMessage): # This function is called every change in position happens.
    global x,y, yaw # to assert every changes in position we make these values up to date and readable in every parsion of the script
    x = poseMessage.x
    y = poseMessage.y
    yaw = poseMessage.theta

def moveRobot(speed,limitDistance,forward_backward):
    # 5th step: Twist object which carries velocity messages is called.
    velocityMessage = Twist()
    # 6th step: Current position must be called as global values
    global x,y
    x0 = x # initial x coordinate
    y0 = y # initial y coordinate
    # 7th step: Speed condition is decided according to moving forward or backward.
    if forward_backward == 1:
        velocityMessage.linear.x = abs(speed)
    else:
        velocityMessage.linear.x = -abs(speed)
    
    distanceMoved = 0.0 # before any movement distance value
    loopRate = rospy.Rate(10) # The messages are published 10 times a second.
    topicAdress = '/turtle1/cmd_vel'
    velocityPublisher = rospy.Publisher(topicAdress,Twist,queue_size = 10) # Publisher is created with topic which the publisher will use ,message type that is Twist in this condition and queue size for the storing up to chosen (10) range into the buffer.
    while True: # Entering the loop that makes the actual moving 
        rospy.loginfo("Robot moves.") # The information is given to the terminal
        velocityPublisher.publish(velocityMessage) # Velocity messages are published
        loopRate.sleep() # This script is made for executing with proper loopRate()
        distanceMoved = abs(math.sqrt(pow((x-x0),2)+pow((-y0),2)))
        print(distanceMoved)
        if not (distanceMoved < limitDistance): # Condition that controls the exact final point is set.
            rospy.loginfo("Robot has reached the destination.")
            break
    
    # 9th step: Robot is stopped when the robot reached by giving values of velocity as 0 and finally this values must be published.
    velocityMessage.linear.x = 0.0
    velocityPublisher.publish(velocityMessage)

def rotateRobot(desiredAngularSpeedInDegree,limitAngleInDegree,clckwise_cclckwsie):
    # Twist object is created and the values of these messages are set up to zero due to make initial values always in the same orientation.
    velocityMessage = Twist()
    velocityMessage.linear.x = 0.0
    velocityMessage.linear.y = 0.0
    velocityMessage.linear.z = 0.0
    velocityMessage.angular.x = 0.0
    velocityMessage.angular.y = 0.0
    velocityMessage.angular.z = 0.0

    angularSpeed = math.radians(desiredAngularSpeedInDegree) # this conversion is made due to difficulties of transforming degree to radian as a humanbeing. This is an obligation because ROS uses radians.
    # 10th step: Angular speed is decided according to clockwise or counter-clockwise
    if clckwise_cclckwsie:
        velocityMessage.angular.z = -abs(angularSpeed)
    else:
        velocityMessage.angular.z = abs(angularSpeed)
    
    t0 = rospy.Time.now().to_sec() # to make time substraction first time is taken before the loop
    currentAngleInDegree = 0.0
    loopRate = rospy.Rate(10)
    topicAdress = '/turtle1/cmd_vel'
    velocityPublisher = rospy.Publisher(topicAdress,Twist,queue_size = 10)
    while True:
        rospy.loginfo("Robot is rotating.")
        velocityPublisher.publish(velocityMessage)
        t1 = rospy.Time.now().to_sec()
        currentAngleInDegree = (t1 - t0)*desiredAngularSpeedInDegree
        print(currentAngleInDegree)
        loopRate.sleep()
        if (currentAngleInDegree > limitAngleInDegree):
            rospy.loginfo("Robot has reached desired angle.")
            break
    velocityMessage.angular.z = 0.0
    velocityPublisher.publish(velocityMessage)

if __name__  == "__main__":
    try:
        # 1st step: Create a node
        nodeName = "cleanerRobotNode"
        rospy.init_node(nodeName,anonymous = True) # A node should have a unique name due to that reason anonymous = True
        # 2nd step: Subscriber and publisher are created
        velocityTopicAdress = '/turtle1/cmd_vel'
        velocityPublisher = rospy.Publisher(topicAdress,Twist,queue_size = 10)
        poseTopicAdress = '/turtle1/pose'
        poseSubscriber = rospy.Subscriber(poseTopicAdress,poseCallback)
        time.sleep(2)
        # 3rd step: Methods which make movement are constructed
        moveRobot(2.0,2.0,True)
        rotateRobot(2.0,45,True)
    except rospy.ROSInterruptException:
        rospy.loginfo("Node has been terminated!")





