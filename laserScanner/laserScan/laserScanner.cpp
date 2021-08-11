#include "laserScanner.h"
using namespace std;

double LaserScanner::averageRange(sensor_msgs::LaserScan &laserScanMessage, int startIndex, int endIndex)
{
    double average = 0;
    for(int i = startIndex;i < endIndex;i++)
    {
        if(!std::isnan(laserScanMessage.ranges[i]))
            average = average + laserScanMessage.ranges[i];
    }
    average = average/(endIndex-startIndex + 1);
    return average;
}
double LaserScanner::minRange(sensor_msgs::LaserScan & laserScanMessage, int startIndex,int endIndex)
{
    int minIndex = 0;
    for(int i = startIndex;i < endIndex;i++)
    {
        if(!std::isnan(laserScanMessage.ranges[i]))
        {
            if((laserScanMessage.ranges[i] >= laserScanMessage.range_min) && (laserScanMessage.ranges[i] <= laserScanMessage.range_max))
            {
                minIndex = i;
                break;
            }
        }
    }
    for(int i = minIndex+1;i < endIndex;i++)
    {
        if(!std::isnan(laserScanMessage.ranges[i]))
        {
            if((laserScanMessage.ranges[i]>= laserScanMessage.range_min) && (laserScanMessage.ranges[i] <= laserScanMessage.range_max))
                {
                    if(laserScanMessage.ranges[minIndex] > laserScanMessage.ranges[i])
                        {
                        minIndex = i;
                        }
                }
        }
    }
    return laserScanMessage.ranges[minIndex] ;
}
double LaserScanner::maxRange(sensor_msgs::LaserScan & laserScanMessage, int startIndex, int endIndex)
{
    int maxIndex = 0;
    for(int i = startIndex;i < endIndex;i++)
    {       
        if(laserScanMessage.ranges[maxIndex] < laserScanMessage.ranges[i])
            {
                maxIndex = i;
                break;
            }
    }
    for(int i = maxIndex + 1;i < endIndex;i++)
    {
        if((laserScanMessage.ranges[i] >= laserScanMessage.range_min) && (laserScanMessage.ranges[i] <= laserScanMessage.range_max))
        {
            if(laserScanMessage.ranges[maxIndex] < laserScanMessage.ranges[i])
            {
                maxIndex = i;
            }
        }
    }
    return laserScanMessage.ranges[maxIndex];
}
int LaserScanner::indexOfMaxRange(sensor_msgs::LaserScan & laserScanMessage)
{
    int maxIndex = 0;
    for(int i = 0;i < laserScanMessage.ranges.size();i++)
    {
        if((laserScanMessage.ranges[i] >= laserScanMessage.range_min) && (laserScanMessage.ranges[i] <= laserScanMessage.range_max))
        {
            maxIndex = i;
            break;
        }
    }
    for(int i = maxIndex+1;i < laserScanMessage.ranges.size();i++)
    {
        if((laserScanMessage.ranges[i] >= laserScanMessage.range_min) && (laserScanMessage.ranges[i] <= laserScanMessage.range_max))
        {
            if(laserScanMessage.ranges[maxIndex] < laserScanMessage.ranges[i])
            {
                maxIndex = i;
            }
        }
    }
    return maxIndex;
}
int LaserScanner::indexOfMinRange(sensor_msgs::LaserScan & laserScanMessage)
{
    int minIndex = 0;
    for(int i = 0; i < laserScanMessage.ranges.size();i++)
    {
        if((laserScanMessage.ranges[i] >= laserScanMessage.range_min) && (laserScanMessage.ranges[i] <= laserScanMessage.range_max))
        {
            minIndex = i;
            break;
        }
    }
    for(int i = minIndex+1; i < laserScanMessage.ranges.size();i++)
    {
        if((laserScanMessage.ranges[i] >= laserScanMessage.range_min) && (laserScanMessage.ranges[i] <= laserScanMessage.range_max))
        {
            if(laserScanMessage.ranges[minIndex] > laserScanMessage.ranges[i])
            {
                minIndex = i;
            }
        }
    }
    return minIndex;
}
double LaserScanner::maxRange(sensor_msgs::LaserScan & laserScanMsg)
{
    return laserScanMsg.ranges[indexOfMaxRange(laserScanMsg)];
}
double LaserScanner::minRange(sensor_msgs::LaserScan & laserScanMsg)
{
    return laserScanMsg.ranges[indexOfMinRange(laserScanMsg)];
}
double LaserScanner::relativeAngleOfMaxRange(sensor_msgs::LaserScan & laserScanMessage)
{
    return -(laserScanMessage.angle_min + (laserScanMessage.angle_increment*indexOfMaxRange(laserScanMessage)));
}
double LaserScanner::relativeAngleOfMinRage(sensor_msgs::LaserScan & laserScanMessage)
{
    return -(laserScanMessage.angle_min + (laserScanMessage.angle_increment*indexOfMinRange(laserScanMessage)));
}
bool LaserScanner::obstacleClosness(sensor_msgs::LaserScan & laserScanMessage, int startIndex, int endIndex, double distanceThreshold)
{
    bool result = false;
    if(minRange(laserScanMessage,startIndex,endIndex) < distanceThreshold) 
        result = true;
    
    return result;
}
void LaserScanner::printingLaserScanRanges(sensor_msgs::LaserScan laserScanMessage)
{
    cout<<endl;
    cout<<"--- LaserScan Info (sensor_msgs/LaserScan.msg) ---"<<endl;
    cout<<setw(20)<<"Size of the range"<<laserScanMessage.ranges.size()<<endl;
    cout<<setw(20)<<"Minumum range: "<<laserScanMessage.range_min<<"m"<<endl;
    cout<<"Maximum range:"<<laserScanMessage.range_max<<"m"<<endl;
    cout<<"Scan time: "<<laserScanMessage.scan_time<<"seconds"<<endl;

    cout<<setw(20)<<"Current LaserScan Ranges"<<endl;
    cout<<setw(20)<<"Minumum range [range : "<<minRange(laserScanMessage)<<", angle: "<<radianToDegree(relativeAngleOfMinRage
    (laserScanMessage))<<", index: "<<indexOfMinRange(laserScanMessage)<<"]"<<endl;
    cout<<setw(20)<<"Maximum range [range : "<<maxRange(laserScanMessage)<<", angle"<<radianToDegree(relativeAngleOfMaxRange
    (laserScanMessage))<<", index: "<<indexOfMaxRange(laserScanMessage)<<"]"<<endl;
    cout<<"---"<<endl<<endl;
}