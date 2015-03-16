#include "frame.hpp"

Frame::Frame(void)
{
}

Frame::~Frame(void)
{
  image.release();
  mask.release();
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
  image.release();
  image = _image;
}

Mat Frame::getMask(void)
{
  return mask;
}

void Frame::setMask(Mat _mask)
{
  mask.release();
  mask = _mask; 
}

Skeleton Frame::getSkeleton(void)
{
  return skeleton;
}

Skeleton* Frame::getSkeletonPtr(){
    return &skeleton;
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

vector <Point2f> Frame::getPartPolygon(int partID)
{
  tree <BodyPart> partTree = getSkeleton().getPartTree();
  tree <BodyPart>::iterator i;
  for (i = partTree.begin(); i != partTree.end(); i++)
  {
    if (i->getPartID() == partID)
    {
      return i->getPartPolygon().asVector();
    }
  }
  return vector <Point2f> ();
}

