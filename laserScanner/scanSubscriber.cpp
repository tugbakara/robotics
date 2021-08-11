#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "laserScan/LaserScanner.h"

using namespace std;

sensor_msgs::LaserScan scanMessage;
ros::Subscriber scanSubscriber;

void scanCallback(sensor_msgs::LaserScan scanMessage);

int main(int argc, char **argv)
{
    // Initializing ROS node
    ros::init(argc,argv,"scanSubscriberCPP");
    ros::NodeHandle nodeHandle;
    // Subscriber is created.
    scanSubscriber = nodeHandle.subscribe("/scan",10,scanCallback);
    ros::spin();
}
void scanCallback(sensor_msgs::LaserScan scanMessage)
{
    cout<< "minumum range: "<<LaserScanner::minRange(scanMessage)<<endl;
    cout<< "maximum range: "<<LaserScanner::maxRange(scanMessage)<<endl;
    cout<<"average range: "<<LaserScanner::averageRange(scanMessage,0,600)<<endl;
    if(LaserScanner::obstacleCloseness(scanMessage,0,600,0.69) == true)
    {
        cout<<"Obstacle is too close."<<endl;
    }
    cout<<endl;
}