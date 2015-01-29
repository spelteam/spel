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
