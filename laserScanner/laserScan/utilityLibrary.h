/* this file is created to list the publicly accessible instance variables
and  methods in the class decleration.*/
#include <math.h>
/* #ifndef and #define are known as header guards. Their primary purpose
is to prevent cpp header files from being included multiple times. */
#ifndef navigationLibraryH
#define navigationLibraryH
#define PI 3.14159265
/*typedef is a reserved keyword in the programming languages C and C++. It 
is used to create an additional name (alias) for another data type, but 
does not create a new type. */
typedef float DISTANCE;
typedef long ODOMETRY;
typedef short ANGLE;
typedef unsigned short SPEED;
typedef double VELOCITY;
typedef double POSE;
typedef unsigned char BUFFER;
// Conversion is made from degree to radian
double degreeToRadian(double angleInDegree);
double radianToDegree(double degreeInRadian);
// Robot pose is defined
typedef struct {
    POSE x;
    POSE y;
    POSE yaw; // orientation of the robot
}POSE2D; //POSE2D is a type specifier not a variable.
// Distance and Bearing calculation funcstions:
DISTANCE getDistance(POSE x1, POSE y1, POSE x2, POSE y2);
POSE yawCalculation(POSE x1, POSE y1, POSE x2, POSE y2);
#endif
