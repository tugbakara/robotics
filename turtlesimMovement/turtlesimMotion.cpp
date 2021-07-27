#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "sstream"

using namespace std;
ros::Publisher velocityPublisher;
ros::Subscriber poseSubscriber;
turtlesim::Pose robotPosition;

const double xMin = 0.0;
const double yMin = 0.0;
const double xMax = 11.0;
const double yMax = 11.0;

const double PI = 3.14159265359;

void moveRobot(double speed, double distance, double forward_backward);
void rotateRobot(double angularSpeedInRadians, double angleInDegreesInRadians, bool clckwise_cclckwise);
double degreesToRadians(double angleInDegrees);
void setDesiredOrientation(double angleInRadians);
void goToGoal(turtlesim::Pose goalPosition, double limitDistance);
void gridMovement();
void spiralMovement();
void poseCallback(const turtlesim::Pose::ConstPtr &poseMessage);

int main(int argc, char **argv)
{
    // ROS Node is created with giving a name, here is turtlesimMovement
    ros::init(argc, argv, "turtlesimMovement");
    ros::NodeHandle rosNodeHandle; // Noedhandle object is created to handle ros nodes
    double speed,angularSpeed;
    double distance,angleInDegreesInDegrees;
    bool forward_backward,clckwise_cclckwise;
    
    // Publisher and subscriber are created thanks to nodehandle
    velocityPublisher = rosNodeHandle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
    poseSubscriber = rosNodeHandle.subscribe("/turtle1/pose",10,poseCallback);

    ros::Rate loopRate(10);

    cout<<"Enter speed : ";
    cin>>speed;
    cout<<"Enter distance: ";
    cin>>distance;
    cout<<"Robot is moving forward/backward?";
    cin>>forward_backward;
    moveRobot(speed,distance,forward_backward);

    cout<<"Enter angular speed in degree/sec: ";
    cin>>angularSpeed;
    cout<<"Enter desired angleInDegreesInDegrees in degrees: ";
    cin>>angleInDegrees;
    cout<<"Robot is turning clockeise/counter-clockwise: ";
    cin>>clckwise_cclckwise;
    rotateRobot(degreesToRadians(angularSpeed),degreesToRadians(angleInDegrees),clckwise_cclckwise);

    setDesiredOrientation(degreesToRadians(180));

    // Pose object is created to adjust position of the robot fot the method goToGoal 
    turtlesim::Pose goalPosition;
    poseMesaage.x = 1;
    poseMessage.y = 1;
    poseMessag.theta = 0;
    goToGoal(goalPosition,0.01);
    

    gridMovement();

    spiralMovement();

    loopRate.sleep();

    ros::spin();
    
    return 0;

    }
