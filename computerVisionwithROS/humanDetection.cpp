#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>

using namespace std;
using namespace cv;
static const std::string openCVWindow = "Image Window";

CascadeClassifier faceCascade;

class ImageConverter
{   
    // Nodeandle,Subscriber,Publisher and Image Transport object are created.
    ros::NodeHandle nodeHandle;
    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber imageSubscriber;
    image_transport::Publisher imagePublisher;
    public:
    ImageConverter():imageTransport(nodeHandle) // constructor of the class is created.
    {
        // Subscription to input video and publishing output video.
        // image_transport publishers advertise individual ROS Topics
        // for each available transport - unlike ROS Publishers, 
        //which advertise a single topic.
        // image_transport publishers are used much like ROS Publishers, but may offer a variety of specialized transport 
        // options (JPEG compression, streaming video, etc.). Different subscribers may request images from the same
        // publisher using different transports.
        imageSubscriber = imageTransport.subscribe("image",1,&ImageConverter::imageCallback,this);
        imagePublisher = imageTransport.advertise("/image_converter/output_video", 1);
                
    }
    ~ImageConverter()
    {
        cv::destroyWindow(openCVWindow);
    }
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cvPointer;
        try
        {
            //converting the ROS imag to openCV BGR8 image format
            cvPointer = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("CvBridge exception: %s",e.what());
            return;
        }
        
        this->humanDetection(cvPointer -> image);
        }
   
    void humanDetection(cv::Mat image)
    {
        Mat matImageGray;
        cvtColor(image,matImageGray,COLOR_BGR2GRAY);
        // Vector rectangle contains reactangles and these rectangles will be at the faces that are detected.
        std::vector<Rect> faceRectangles;
        faceCascade.detectMultiScale(matImageGray,faceRectangles);
        for (size_t i = 0; i < faceRectangles.size();i++)
        {
            cv::Point center(faceRectangles[i].x + faceRectangles[i].width/2,faceRectangles[i].y + faceRectangles[i].height/2);
            cv::rectangle(image,center,Size(faceRectangles[i].width/2,faceRectangles[i].height/2),(0,255,255),8);
            Mat faceROI = matImageGray(faceRectangles[i]);
        }
        
        cv::imshow(openCVWindow,image);
}};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"image_converter");
    string faceClassifier = "/home/tugbakara/catkin_ws/src/ros_essentials_cpp/src/topic03_perception/cpp/haarcascade_frontalface_default.xml";
    if(!faceCascade.load(faceClassifier))
    {
        cout<<"Couldn't load the classifier.";
        return -1;
    }; 
    cout<<"Classifier loaded."<<endl;
    ImageConverter();
    ros::spin();
    return 0;
};
