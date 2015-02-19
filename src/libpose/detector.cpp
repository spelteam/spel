#include "detector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "

void Detector::getNeighborFrame(Frame *frame, Frame **prevFrame, Frame **nextFrame, uint32_t &step, uint32_t &stepCount)
{
  if (frame == 0)
  {
    stringstream ss;
    ss << "Invalid frame pointer";
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }
  if (prevFrame == 0)
  {
    stringstream ss;
    ss << "Invalid prevFrame pointer";
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }
  if (nextFrame == 0)
  {
    stringstream ss;
    ss << "Invalid nextFrame pointer";
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }

  for (vector <Frame*>::iterator i = frames.begin(); i != frames.end(); ++i)
  {
    Frame *f = *i;
    if (f->getID() < frame->getID())
    {
      if (f->getFrametype() == KEYFRAME || f->getFrametype() == LOCKFRAME)
      {
        (*prevFrame) = f;
        stepCount = 0;
      }
      else
      {
        stepCount++;
      }
    }
    else if (f->getID() > frame->getID())
    {
      stepCount++;
      if (f->getFrametype() == KEYFRAME || f->getFrametype() == LOCKFRAME)
      {
        (*nextFrame) = f;
        break;
      }
    }
    else // equal
    {
      stepCount++;
      if ((*prevFrame) == 0)
      {
        stringstream ss;
        ss << "Couldn't find previous keyframe to the frame " << frame->getID();
        throw logic_error(ss.str());
      }
      else
      {
        if (stepCount == 0)
        {
          stringstream ss;
          ss << "Invalid stepCount";
          throw logic_error(ss.str());
        }
        else
        {
          step = stepCount;
        }
      }
    }
  }
}

void Detector::getRawBodyPartPosition(Frame *frame, Frame *prevFrame, Frame *nextFrame, int parentJointID, int childJointID, uint32_t &step, uint32_t &stepCount, Point2f &j0, Point2f &j1)
{
  if (frame == 0)
  {
    stringstream ss;
    ss << "Invalid frame pointer";
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }
  if (prevFrame == 0)
  {
    stringstream ss;
    ss << "Invalid prevFrame pointer";
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }
  if (nextFrame == 0)
  {
    stringstream ss;
    ss << "Invalid nextFrame pointer";
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }
  if (stepCount == 0)
  {
    stringstream ss;
    ss << "Stepcount can't be 0";
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }
  Skeleton skeleton = frame->getSkeleton();
  BodyJoint *parentJoint = skeleton.getBodyJoint(parentJointID);
  BodyJoint *childJoint = skeleton.getBodyJoint(childJointID);
  Skeleton prevSkeleton = prevFrame->getSkeleton();
  tree <BodyJoint> prevBodyJoints = prevSkeleton.getJointTree();
  Skeleton nextSkeleton = nextFrame->getSkeleton();
  tree <BodyJoint> nextBodyJoints = nextSkeleton.getJointTree();
  Point2f pj0, nj0, pj1, nj1;
  for (tree <BodyJoint>::iterator i = prevBodyJoints.begin(); i != prevBodyJoints.end(); ++i)
  {
    if (i->getLimbID() == parentJoint->getLimbID())
    {
      pj0 = i->getImageLocation();
    }
    if (i->getLimbID() == childJoint->getLimbID())
    {
      pj1 = i->getImageLocation();
    }
  }
  for (tree <BodyJoint>::iterator i = nextBodyJoints.begin(); i != nextBodyJoints.end(); ++i)
  {
    if (i->getLimbID() == parentJoint->getLimbID())
    {
      nj0 = i->getImageLocation();
    }
    if (i->getLimbID() == childJoint->getLimbID())
    {
      nj1 = i->getImageLocation();
    }
  }
  float interpolateStep = (float)step / (float)stepCount;  // Interpolate2Ddisplacement
  j0 = pj0 * (1 - interpolateStep) + nj0 * interpolateStep;
  j1 = pj1 * (1 - interpolateStep) + nj1 * interpolateStep;
}

float Detector::getBoneLength(Point2f begin, Point2f end)
{
  return (begin == end) ? 1.0f : (float)sqrt(PoseHelper::distSquared(begin, end));
}

float Detector::getBoneWidth(float length, BodyPart bodyPart)
{
  float ratio = bodyPart.getLWRatio();
  if (ratio == 0)
  {
    stringstream ss;
    ss << "Ratio can't be 0";
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }
  return length / ratio;
}

POSERECT <Point2f> Detector::getBodyPartRect(BodyPart bodyPart, Point2f j0, Point2f j1, Size ethalon)
{
  Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
  float x = boxCenter.x;
  float y = boxCenter.y;
  float boneLength = ethalon == Size(0, 0) ? getBoneLength(j0, j1) : ethalon.width - 0.5;
  float boxWidth = ethalon == Size(0, 0) ? getBoneWidth(boneLength, bodyPart) : ethalon.height - 0.5;
  float angle = float(PoseHelper::angle2D(1, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI));
  Point2f c1, c2, c3, c4, polyCenter;
  c1 = Point2f(0.f, 0.5f * boxWidth);
  c2 = Point2f(boneLength, 0.5f * boxWidth);
  c3 = Point2f(boneLength, -0.5f * boxWidth);
  c4 = Point2f(0.f, -0.5f * boxWidth);
  polyCenter = Point2f(boneLength * 0.5f, 0.f);
  c1 = PoseHelper::rotatePoint2D(c1, polyCenter, angle) + boxCenter - polyCenter;
  c2 = PoseHelper::rotatePoint2D(c2, polyCenter, angle) + boxCenter - polyCenter;
  c3 = PoseHelper::rotatePoint2D(c3, polyCenter, angle) + boxCenter - polyCenter;
  c4 = PoseHelper::rotatePoint2D(c4, polyCenter, angle) + boxCenter - polyCenter;
  return POSERECT <Point2f>(c1, c2, c3, c4);
}

Mat Detector::rotateImageToDefault(Mat imgSource, POSERECT <Point2f> &initialRect, float angle, Size size)
{
  POSERECT <Point2f> tempRect = initialRect;
  Mat partImage = Mat(size.height, size.width, CV_8UC3, Scalar(255, 255, 255));
  float xmax, ymax, xmin, ymin;
  initialRect.GetMinMaxXY <float>(xmin, ymin, xmax, ymax);
  Point2f center = initialRect.GetCenter<Point2f>();
  Point2f newCenter = Point2f(0.5 * size.width, 0.5 * size.height);
  initialRect.point1 = PoseHelper::rotatePoint2D(initialRect.point1, center, angle * -1) - center + newCenter;
  initialRect.point2 = PoseHelper::rotatePoint2D(initialRect.point2, center, angle * -1) - center + newCenter;
  initialRect.point3 = PoseHelper::rotatePoint2D(initialRect.point3, center, angle * -1) - center + newCenter;
  initialRect.point4 = PoseHelper::rotatePoint2D(initialRect.point4, center, angle * -1) - center + newCenter;
  for (float x = xmin; x <= xmax; x++)
  {
    for (float y = ymin; y <= ymax; y++)
    {
      PHPoint<int32_t> phpoint;
      Point2f p = Point2f(x, y);
      if (tempRect.containsPoint(p) >= 0)
      {
        p = PoseHelper::rotatePoint2D(p, center, angle * -1) - center + newCenter;
        phpoint.x = (int32_t)p.x;
        phpoint.y = (int32_t)p.y;
        try
        {
          Vec3b color = imgSource.at<Vec3b>((int32_t)y, (int32_t)x);
          partImage.at<Vec3b>(phpoint.y, phpoint.x) = color;
        }
        catch (...)
        {
          stringstream ss;
          ss << "Couldn't put value of indeces " << "[" << (int32_t)x << "][" << (int32_t)y << "] into indeces [" << phpoint.x << "][" << phpoint.y << "]";
#ifdef DEBUG
          cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
          throw logic_error(ss.str());
        }
      }
    }
  }
  return partImage;
}
