#ifndef _LIBPOSE_DETECTOR_HPP_
#define _LIBPOSE_DETECTOR_HPP_

#include <vector>
#include "frame.hpp"
#include "limbLabel.hpp"

template <class T>
struct TPOSERECT
{
  T p1;
  T p2;
  T p3;
  T p4;
};

template <class T>
struct POSERECT
{
  POSERECT() : point1(), point2(), point3(), point4() {}
  POSERECT(T _point1, T _point2, T _point3, T _point4) : point1(_point1), point2(_point2), point3(_point3), point4(_point4) {}
  T point1;
  T point2;
  T point3;
  T point4;
//TODO (Vitaliy Koshura): Need unit test
  uint8_t containsPoint(T point)
  {
    vector <T> contour;
    contour.push_back(point1);
    contour.push_back(point2);
    contour.push_back(point3);
    contour.push_back(point4);
    return pointPolygonTest(contour, point, false);
  }
  vector <T> asVector()
  {
    vector <T> contour;
    contour.push_back(point1);
    contour.push_back(point2);
    contour.push_back(point3);
    contour.push_back(point4);
    return contour;
  }
};

class Detector
{
  public:
    virtual int getID(void) = 0;
    virtual void setID(int _id) = 0;
    virtual void train(vector <Frame*> frames) = 0;
    virtual vector <vector <LimbLabel> > detect(Frame *frame, vector <float> params) = 0;
  private:
    // T model;
};

#endif  // _LIBPOSE_DETECTOR_HPP_

