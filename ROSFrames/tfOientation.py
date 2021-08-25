#!/usr/bin/env python
# In this code subscription was made to odom topic to get the position and orientation of the robot.
import rospy
import time
import math
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
# odom pose = orientation + position 
def odomCallBack(odomMessage):
    print("odometry position callback function")
    print("x = ",odomMessage.pose.pose.position.x)
    print("y = ",odomMessage.pose.pose.position.y)
    print("linear speed in x = ",odomMessage.twist.twist.linear.x)
    print("angular speed in z = ",odomMessage.twist.twist.angular.z)
    print("position in x with quaternion format = ",odomMessage.pose.pose.orientation.x)
    print("position in y with quaternion format = ",odomMessage.pose.pose.orientation.y)
    print("position in z with quaternion format = ",odomMessage.pose.pose.orientation.z)
    print(" position magnitude with quaternion format = ",odomMessage.pose.pose.orientation.w)
    quaternionTuple = (
                        odomMessage.pose.pose.orientation.x,
                        odomMessage.pose.pose.orientation.y,
                        odomMessage.pose.pose.orientation.z,
                        odomMessage.pose.pose.orientation.w
                        )
    rpy = tf.transformations.euler_from_quaternion(quaternionTuple)
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]
    print(math.degrees(roll)," ", math.degrees(pitch), " ", math.degrees(yaw))
    print("Orientation of the robot in z is : ",math.degrees(yaw))
if __name__ == "__main__":
    try:
        rospy.init_node("odomNode",anonymous = True)
        topicName = "/odom"
        poseSubscriber = rospy.Subscriber(topicName,Odometry,odomCallBack)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated!")