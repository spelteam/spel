#include "frame.hpp"

Frame::Frame(void)
{
}

Frame::~Frame(void)
{
}

int Frame::getID(void)
{
  return id;
}

void Frame::setID(int _id)
{
  id = _id;
}

Mat Frame::getImage(void)
{
  return image;
}

void Frame::setImage(Mat _image)
{
  image = _image;
}

Mat Frame::getMask(void)
{
  return mask;
}

void Frame::setMask(Mat _mask)
{
  mask = _mask; 
}

Skeleton Frame::getSkeleton(void)
{
  return skeleton;
}

void Frame::setSkeleton(Skeleton _skeleton)
{
  skeleton = _skeleton;
}

Point2f Frame::getGroundPoint(void)
{
  return groundPoint;
}

void Frame::setGroundPoint(Point2f _groundPoint)
{
  groundPoint = _groundPoint;
}

//TODO(Vitaliy Koshura): write real implementation
/*vector <Point2f> Frame::getPartPolygon(int partID)
{
  vector <Point2f> result;

  return result;
}*/

