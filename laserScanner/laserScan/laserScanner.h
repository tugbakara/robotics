#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "utilityLibrary.h"

using std::string;
#ifndef laserScan_h
#define laserScan_h
class LaserScanner
{
    public:
    static int indexOfMaxRange(sensor_msgs::LaserScan & laserScanMessage);
    static int indexOfMinRange(sensor_msgs::LaserScan & laserScanMessage);
    
    static double relativeAngleOfMaxRange(sensor_msgs::LaserScan & laserScanMessage);
    static double relativeAngleOfMinRange(sensor_msgs::LaserScan & laserScanMessage);

    static void printingLaserScanRanges(sensor_msgs::LaserScan laserScanMessage);

    static double averageRange(sensor_msgs::LaserScan & laserScanMessage, int startIndex, int endIndex);

    static double maxRange(sensor_msgs::LaserScan & laserScanMessage);
    static double maxRange(sensor_msgs::LaserScan & laserScanMessage, int startIndex, int endIndex);
    static double minRange(sensor_msgs::LaserScan & laserScanMessage);
    static double minRange(sensor_msgs::LaserScan & laserScanMessage, int startIndex, int endIndex);
    static bool obstacleClosness(sensor_msgs::LaserScan &laserScanMessage, int startIndex, int endIndex, double distanceThreshold);
};
#endif
