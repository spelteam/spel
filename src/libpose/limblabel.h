#ifndef LIMBLABEL_H
#define LIMBLABEL_H

#include <opencv2/opencv.hpp>
#include "score.h"
#include <string.h>

using namespace cv;
using namespace std;

class LimbLabel
{

public:

    LimbLabel();
    LimbLabel(const LimbLabel& ll);
    LimbLabel(int id, Point2f centre, float angle, vector<Point2f> polygon, vector<Score> scores);

    string toString(); //output labels as printable string
    LimbLabel & operator=(const LimbLabel &ll);
    bool operator<(const LimbLabel &ll) const;
    bool operator>(const LimbLabel &ll) const;
    bool operator==(const LimbLabel &ll) const;
    bool operator!=(const LimbLabel &ll) const;

private:

    int limbId; //identifier representing what bone this label belongs to
    Point2f center; //Label center
    float angle; //degrees rotation from y=0 (horizontal line)
    vector<Score> scores; //detector scores
    vector<Point2f> polygon; //polygon for the label (already rotated, scaled etc. - from which scores are generated)

    bool isOccluded; //signify label is for an occluded limb
    bool isWeak; //signify lable is from a badly localised part

    void getEndpoints(Point2f &p0, Point2f &p1); //compute the endpoints of the limb that this label would produce
   
}
