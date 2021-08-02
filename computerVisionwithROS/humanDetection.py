#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import sys #It lets us access system-specific parameters and functions.

class FaceRecognation():
    def __init__(self):
        # ROS node is created.
        self.nodeName = 'imageConverter'
        rospy.init_node(self.nodeName,anonymous = True)
        # Subscriber is created for the images that are coming from turtlebot3 itself or usb camera and the images 
        # coming from different environment are also specified with different topics. e.g turtlebot3 waffle : "/camera/rgbimage_raw/compressed" , usb camera : "/usb_cam/
        # image_raw"
        topicAdress = '/usb_cam/image_raw'
        imageSubscriber = rospy.Subscriber(topicAdress,Image,imageCallback)
            

































# bridge object is created to make transition of image formats between openCV and ROS.
bridge = CvBridge()

def imageCallback(ROSimage):
    print("Image is captured.")
    global bridge
    # Bridge object is used to make converition ROS image into openCV image.
    try:
        cvImage = bridge.imgmsg_to_cv2(ROSimage,"bgr8")
        faceDetection(cvImage)
    except CvBridgeError as e:
        print(e)
    
def faceDetection(cvImage):
    trainedClassifierFace = cv2.CascadeClassifier(r'/home/tugbakara/desktop/haarcascade_frontalface_default.xml')
    while cvImage.isOpened():
        cvImageG = cv2.cvtColor(cvImage,cv2.COLOR_BGR2GRAY)
        detectedHumans = trainedClassifierFace.detectmultiScale(cvImageG,2,2)
        for (x,y,w,h) in detectedHumans:
            roicvImage = cvImage[y:y+h,x:x+h]
            roicvImageG = cvImageG[y:y+w,x:x+w]
        cv2.imshow(-)



def main(args):
    try:
        FaceRecognation()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
