#include "poseHelper.hpp"
/*
   Return the angle between two vectors on a plane
   The angle is from vector 1 to vector 2, positive anticlockwise
   The result is between -pi -> pi
*/
//TODO (Vitaliy Koshura): Need unit test
double PoseHelper::angle2D(double x1, double y1, double x2, double y2)
{
    double dtheta, theta1, theta2;
    theta1 = atan2(y1, x1);
    theta2 = atan2(y2, x2);
    dtheta = theta2 - theta1;
    while (dtheta > M_PI)
        dtheta -= (M_PI * 2.0);
    while (dtheta < -M_PI)
        dtheta += (M_PI * 2.0);
    return(dtheta);
}

