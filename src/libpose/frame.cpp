#include "frame.hpp"

Frame::Frame(void)
{
    setParentFrameID(-1);
    setID(-1);
    setGroundPoint(Point2f(0,0));
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

void Frame::shiftSkeleton2D(Point2f point) //shift in 2D and recompute 3D?
{
    tree <BodyJoint> jointTree = skeleton.getJointTree();
    for(tree <BodyJoint>::iterator i = jointTree.begin(); i != jointTree.end(); ++i)
    {
        //add point to every joint
        Point2f prevLoc = i->getImageLocation();
        Point2f nextLoc = prevLoc+point;
        i->setImageLocation(nextLoc);
    }
    skeleton.setJointTree(jointTree);
    skeleton.infer3D();
}

int Frame::getParentFrameID(void)
{
    return parentFrameID;
}

void Frame::setParentFrameID(int _parentFrameID)
{
    parentFrameID = _parentFrameID;
}
