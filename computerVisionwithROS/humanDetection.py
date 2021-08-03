#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import sys #It lets us access system-specific parameters and functions.
class FaceRecognation():   
    def __init__(self,nodeName,topicAdress):
        rospy.init_node(nodeName,anonymous = True)
        # bridge object is created to make transition of image formats between openCV and ROS.
        self.bridge = CvBridge()
        rospy.Subscriber(topicAdress,Image,self.imageCallback,queue_size = 1)
        rospy.loginfo("Image has been captured.")
    def imageCallback(self,rosImage):
        try:
            #ROS Image has been translated into CV image format.
            cvImage = self.bridge.imgmsg_to_cv2(rosImage,'bgr8')
        except CvBridgeError as e:
            print(e)
        processedImage = self.determination(cvImage)
        cv2.imshow("Recognized Image",processedImage)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("ROS is shutting down...")
    def determination(self,cvImage):
        grayImage = cv2.cvtColor(cvImage,cv2.COLOR_BGR2GRAY)
        trainedFaceCascade = cv2.CascadeClassifier("usr/openCV/trainedClassfiers/haarcascade_frontalface_default.xml")
        recognizedFaces = trainedFaceCascade.detectMultiScale(grayImage,
                                                                scaleFactor = 1.1, # proportion for lessening image size
                                                                minNeighbors = 5, # number of neighbor rectangle 
                                                                minSize = (30,30)) # in object size
        for(x,y,w,h) in recognizedFaces:
            cv2.rectangle(cvImage,(x,y),(x+w,y+h),(0,255,255),2)
        return cvImage      
        
def main(args):
    try:
        # Subscriber is created for the images that are coming from turtlebot3 itself or usb camera and the images 
        # coming from different environment are also specified with different topics.
        # e.g turtlebot3 waffle : "/camera/rgbimage_raw/compressed",# usb camera : "/usb_cam/image_raw"
        nodeName = input("Enter the node name :")
        print("Your noed name is : {}".format(nodeName))
        topicAdress = input('Enter the adress of the topic to use for Image :')
        print("Your topic adress is : {} ".format(topicAdress))
        #For the process created class is called.
        FaceRecognation(nodeName,topicAdress)
        rospy.spin()
    except KeyboardInterrupt:
        print("Closing...")
        cv2.destroyAllWindows()
if __name__ == '__main__':
    main(sys.argv)