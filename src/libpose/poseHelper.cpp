#include "poseHelper.hpp"

double PoseHelper::angle2D(double x1, double y1, double x2, double y2)
{
    //input has a zero vector
    if( (x1==0.0 && y1==0.0) || (x2==0.0 && y2==0.0) ){
        //zero vector is both parallel and perpendicular to every vector
        return 0.0;
    }

    double dtheta, theta1, theta2;
    //angle between Ox and first vector
    theta1 = atan2(y1, x1);
    //angle between Ox and second vector
    theta2 = atan2(y2, x2);
    //angle between first and second vector
    dtheta = theta2 - theta1;
    //normalize angle to range [-PI;PI]
    while (dtheta > M_PI)
        dtheta -= (M_PI * 2.0);
    while (dtheta < -M_PI)
        dtheta += (M_PI * 2.0);
    return(dtheta);
}
