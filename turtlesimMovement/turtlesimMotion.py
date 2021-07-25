#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist # to take linear and angular velocities
from turtlesim.msg import Pose # to take current position of robot in every sequence that we decided before.
import math # to calculate distance moved by the robot
import time # to make PID control 

# 4th step: functions that is needed for the movements have been written and coordinates for the Pose messages have been assigned.
x = 0
y = 0
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
    velocityPublisher = rospy.Publisher(topicAdress,Twist,queue_size = 10) # Publisher is created with topic which the publisher will use ,message type that is Twist
    # in this condition and queue size for the storing up to chosen (10) range into the buffer.
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

def rotateRobot(desiredAngularSpeedInDegree,desiredAngleInDegree,clckwise_cclckwsie):
    global yaw
    # Twist object is created and the values of these messages are set up to zero due to make initial values always in the same orientation.
    velocityMessage = Twist()
    velocityMessage.linear.x = 0.0
    velocityMessage.linear.y = 0.0
    velocityMessage.linear.z = 0.0
    velocityMessage.angular.x = 0.0
    velocityMessage.angular.y = 0.0
    velocityMessage.angular.z = 0.0

    angularSpeed = math.radians(desiredAngularSpeedInDegree) # this conversion is made due to difficulties of transforming degree to radian as a humanbeing.
    # This is an obligation because ROS uses radians.
    # 10th step: Angular speed is decided according to clockwise or counter-clockwise
    if clckwise_cclckwsie:
        velocityMessage.angular.z = -abs(angularSpeed)
    else:
        velocityMessage.angular.z = abs(angularSpeed)
    
    t0 = rospy.Time.now().to_sec() # to make time substraction first time is taken before the loop
    currentAngleInDegree = 0.0
    loopRate = rospy.Rate(15)
    topicAdress = '/turtle1/cmd_vel'
    velocityPublisher = rospy.Publisher(topicAdress,Twist,queue_size = 10)
    while True:
        rospy.loginfo("Robot is rotating.")
        velocityPublisher.publish(velocityMessage)
        t1 = rospy.Time.now().to_sec()
        currentAngleInDegree = (t1 - t0)*desiredAngularSpeedInDegree
        print(currentAngleInDegree)
        loopRate.sleep()
        if (currentAngleInDegree > desiredAngleInDegree):
            rospy.loginfo("Robot has reached desired angle.")
            break
    velocityMessage.angular.z = 0.0
    velocityPublisher.publish(velocityMessage)

def goToGoal(goalX,goalY):
    global x,y,yaw
    velocityMessage = Twist()   
    # PID controller was used in this method to achive less error when goal keeps closer.
    while True:
        kLinear = 0.5 # this coefficient can be changed according to system response
        distance = abs(math.sqrt(pow((goalX-x),2)+pow((goalY-y),2))) # In tis formula x and y are coming from Pose messages which are about coordinates 
        # of turtlesim
        linearSpeed = kLinear*distance # Proportional gain is used when the distance to the gal location decreaes ,speed also decreases.
        kAngular = 4.0
        angleOrientationToGoal = math.atan2(goalY-y,goalX-x) # Most proper angle is made for the goal 
        angularSpeed = (angleOrientationToGoal - yaw)*kAngular # When the angle exist with respcet to goal, angle decreases as the robot has a 
        # linear orientation o goal
        velocityMessage.linear.x = linearSpeed
        velocityMessage.angular.z = angularSpeed
        velocityPublisher.publish(velocityMessage)
        if distance < 0.01:
            break
                      
def setDesiredOrientation(desiredAngleInDegree):
    desiredAngleInRadians = math.radians(desiredAngleInDegree)
    exactAngleInRadians = desiredAngleInRadians - yaw
    if exactAngleInRadians < 0:
        clckwise_cclckwsie = 1
    else:
        clckwise_cclckwsie = 0
    print(exactAngleInRadians)
    print(desiredAngleInRadians)
    rotateRobot(20,math.degrees(exactAngleInRadians),clckwise_cclckwsie)

def gridMovement(): # in this movement almost every motion is tried to make similar real-world app.
    desiredPosition = Pose() # position info is taken to know coordinates that robot is standing and make the proper go to goal action
    #In position message , x,y and orientation which is theta are the desired infos for that method
    desiredPosition.x = 1
    desiredPosition.y = 1
    desiredPosition.theta = 0
    # All the values were set up by me, in these methods' values written in below are changable to test how outcomes affect
    goToGoal(8,8)
    setDesiredOrientation(math.degrees(desiredPosition.theta))
    moveRobot(2.0,3.0,True)
    rotateRobot(30,180,False)
    pass

def spiralMovement(): # In this method PID control was used.
    velocityMessage = Twist()
    currentPose = Pose()
    loopRate = rospy.Rate(10)
    topicAdress = '/turtle1/cmd_vel'
    velocityPublisher  rospy.Publisher(topicAdress,Twist,queue_size = 10)
    kAngularSpeed = 4
    kRadius = 0
    while((currentPose.x < 10.5) and (currentPose.y < 10.5)):
        # Current positions are limited for the turtlesim UI , It has almost x = 11 and y = 11 ranges in the coordinate system
        kRadius = kRadius + 1
        velocityMessage.linear.x = kRadius
        velocityMessage.linear.y = 0
        velocityMessage.linear.z = 0
        velocityMessage.angular.x = 0
        velocityMessage.angular.y = 0
        velocityMessage.angular.z = kAngularSpeed
        velocityPublisher.publish(velocityMessage)
        loopRate.sleep()
    velocityMessage.linear.x = 0
    velocityMessage.angular.x = 0
    velocityPublisher.publish(velocityMessage)

if __name__  == "__main__":
    try:
        # 1st step: Create a node
        nodeName = "cleanerRobotNode"
        rospy.init_node(nodeName,anonymous = True) # A node should have a unique name due to that reason anonymous = True
        # 2nd step: Subscriber and publisher are created
        velocityTopicAdress = '/turtle1/cmd_vel'
        velocityPublisher = rospy.Publisher(velocityTopicAdress,Twist,queue_size = 10)
        poseTopicAdress = '/turtle1/Pose'
        poseSubscriber = rospy.Subscriber(poseTopicAdress,Pose,poseCallback)
        time.sleep(2)
        # 3rd step: Methods which make movement are constructed
        # moveRobot(2.0,2.0,True)
        rotateRobot(25,45,True)
        # goToGoal(9,9)
        # setDesiredOrientation(180)
        # gridMovement()
        # spiralMovement()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node has been terminated!")