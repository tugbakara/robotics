#include "utilityLibrary.h"
using namespace std;

// In the getDistance function inputs are coordinates of two points which are x and y.
DISTANCE getDistance(POSE x1,POSE y1, POSE x2, POSE y2)
{
    return sqrt(((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1)));
}
// In the yawCalculation function degree netween two points is calculated
POSE yawCalculation(POSE x1, POSE y1, POSE x2,POSE y2)
{
    POSE degree = atan2((y2 - y1),(x2 - x1));
    if(degree < 0) degree += 2*PI;
    degree *= 180.0/PI;
    return degree;
}
// Conversion from radianToDegree and degreeToRadian are created:
double radianToDegree(double angleInRadian)
{
    return (angleInRadian*57.2957795);
}
double degreeoRadian(double anglenDegree)
{ 
    return (anglenDegree/57.2957795);
}