#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "sstream"

using namespace std;

global ros::Publisher velocityPublisher;
global ros::Subscriber poseSubscriber;
global turtlesim::Pose robotPosition;

const double xMin = 0.0;
const double yMin = 0.0;
const double xMax = 11.0;
const double yMax = 11.0;

const double PI = 3.14159265359;

void moveRobot(double speed, double distance, bool forward_backward);
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

void moveRobot(double speed, double distance,bool forward_backward)
{
    geometry_msgs::Twist velocityMessage;
    if(forward_backward)
        velocityMessage.linear.x = abs(speed);
    else
        velocityMessage.linear.x = -abs(speed);

    velocityMessage.linear.y = 0;
    velocityMessage.linear.z = 0;
    velocityMessage.angular.x = 0;
    velocityMessage.angular.y = 0;
    velocityMesaage.angular.z = 0;
    double t0 = ros::Time::now().toSec();
    double distanceMoved = 0.0;
    ros::Rate loopRate(100);

    do{
        velocityPublisher.publish(velocityMessage);
        double t1 = ros::Time::now().toSec();
        distanceMoved = speed*(t1 - t10)
        ros::spinOnce();
        loopRate.sleep();
        cout<<(t1 - t0)<<","<<distanceMoved<<endl;      
    }while(distanceMoved < distance);
    velocityMessage.linear.x = 0;
    velocityPulisher.publish(velocityMessage);
}

void rotateRobot(double angularSpeedInRadians, double angleInDegreesInRadians, bool clckwise_cclckwise)
{
    geometry_msgs::Twist velocityMessage;
    velocityMessage.linear.x = 0;
    velocityMessage.linear.y = 0;
    velocityMessage.linear.z = 0;
    velocityMessage.angular.x = 0;
    velocityMessage.angular.y = 0;
    if(clckwise_cclckwise)
        velocityMessage.angular.z = -abs(angularSpeedInRadians);
    else
        velocityMessage.angular.z = abs(angularSpeedInRadians);

    double currentAngle = 0.0;
    double t0 = ros::Time::now().toSec();
    ros::Rate loopRate(10);
    do{
        velocityublisher.publish(velocityMessage);
        double t1 = ros::Time::now().toSec();
        currentAngle = angularSpeedInRadians*(t1 - t0);
        ros::spinOnce();
        loopRate.sleep();
    }while(currentAngle < angleInDegreesInRadians);
    velocityMessage.angular.z = 0;
    velocityPublisher.publish(velocityMessage);
}

double degreesToRadians(double angleInDegrees)
{
    return angleInDegrees*PI/180.0;
}

void setDesiredOrientation(double desiredAngleInRadians)
{
    double limitAngleInRadians = desiredAngleInRadians - robotPosition.theta;
    bool clckwise_cclckwise = ((limitAngleInRadians < 0)? true:false);
    cout<<desiredAngleInRadians<<","<<robotPosition.theta<<","<<limitAngleInRadians<<","<<clckwise_cclckwise<<endl;
    rotateRobot(degreesToRadians(10),abs(limitAngleInRadians),clckwise_cclckwise);
}

double getDistance(double x1, double y1, double x2,double y2)
{
    return sqrt(pow(x1 - x2),2) +pow((y1 -y2),2));
}

void goToGoal(turtlesim::Pose goalPosition, double limitDistance)
{
    geometry_msgs::Twist velocityMessage;
    ros::Rate loopRate(20);
    double distanceMoved = 0.0;
    do
    {
        double kLinear = 1.0;
        double kAngular = 4.0;
        double distance = getDistance(robotPosition.x,robotPosition.y,goalPosition.x,goalPosition.y);
        double distanceMoved = distanceMoved + distance;
        velocityMessage.linear.x = (kLinear*distance);
        velocityMessage.linear.y = 0;
        velocityMessage.linear.z = 0;
        velocityMessage.angular.x = 0;
        velocityMessage.angular.y = 0;
        desiredAngleToGoal = atan2(goalPosition.y - robotPosition.y,goalPosition.x - robotPosition.x)
        velocityMessage.angular.z = kAngular*(desiredAngleToGoal - robotPosition.theta);
        velocityPublisher.publish(velocityMessage);
        ros::spinOnce();
        loopRate.sleep();
    } while (getDistance(robotPosition.x,robotPosition.y,goalPosition.x,goalPosition.y) > limitDistance);
    cout<<"Robot has reached to the goal."<<endl;
    velocityMessage.linear.x = 0;
    velocityMessage.angular.z = 0;
    velocityPublisher.publish(velocityMessage); 
}

void gridMovement()
{
    ros::Rate loopRate(0.5);
    turtlesim::Pose goalPosition;
    goalPosition.x = 1;
    goalPosition.y = 1;
    goalPosition.theta = 0;
    goToGoal(goalPosition,0.01);
    loopRate.sleep();
    moveRobot(3.0,8.0,true);
    loopRate.sleep();
    rotateRobot(degreesToRadians(10),degreesToRadians(90),false);
    loopRate.sleep();
}

void spiralMovement()
{
    geometry_msgs::Twist velocityMessage;
    double spinCount = 0;
    double constantAngularSpeed = 4;
    double constantLinearSpeed = 0.5;
    ros::Rate loopRate(10);

    do
    { 
        constantLinearSpeed + = 1.0;
        velocityMessage.linear.x = constantLinearSpeed;
        velocityMessage.linear.y = 0;
        velocityMessage.linear.z = 0;
        velocityMessage.angular.x = 0;
        velocityMessage.angular.y = 0;
        velocityMessage.angular.z = constantAngularSpeed;
        cout<<"Linear velocity in x direction: "<<velocityMessage.linear.x<<endl;
        cout<<"Angular speed in z direction: "<<velocityMessage.angular.z<<endl;
        velocityPublisher.publish(velocityMessage);
        ros::spinOnce();
        loopRate.sleep();
    } while ((robotPosition.x < 10.0) && (rootPosition.y < 10.0));
    velocityMessage.linear.x = 0;
    velocityMessage.angular.z = 0;
    velocityPublisher.publish(velocityMessage);    
}

void poseCallback(const turtlesim::Pose::ConstPtr &poseMessage)
{
    robotPosition.x = poseMessage -> x;
    robotPosition.y = poseMessage -> y;
    robotPosition.theta = poseMessage -> theta;
}
