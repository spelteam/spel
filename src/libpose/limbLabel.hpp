#ifndef _LIBPOSE_LIMBLABEL_HPP_
#define _LIBPOSE_LIMBLABEL_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "score.hpp"

using namespace std;
using namespace cv;

class LimbLabel
{
  public:
    LimbLabel();
    LimbLabel(const LimbLabel& ll);
    LimbLabel(int id, Point2f centre, float angle, vector<Point2f> polygon, vector<Score> scores);

// output labels as printable string
    string toString();
    LimbLabel & operator=(const LimbLabel &ll);
    bool operator==(const LimbLabel &ll) const;
    bool operator!=(const LimbLabel &ll) const;
// compute the endpoints of the limb that this label would produce
    void getEndpoints(Point2f &p0, Point2f &p1); 
    void addScore(Score detectionScore);

    vector<Score> getScores(); //get the label scores
  private:
// identifier representing what bone this label belongs to
    int limbID;
// Label center 
    Point2f center;
// degrees rotation from y=0 (horizontal line)
    float angle;
// detector scores
    vector<Score> scores;
// polygon for the label (already rotated, scaled etc. - from which scores are generated)
    vector<Point2f> polygon;
// signify label is for an occluded limb
    bool isOccluded;
// signify lable is from a badly localised part
    bool isWeak; 
};

#endif  // _LIBPOSE_LIMBLABEL_HPP_

