#include "detector.hpp"

void Detector::getNeighborFrame(Frame **curFrame, Frame **prevFrame, Frame **nextFrame, uint32_t &step, uint32_t &stepCount)
{
  for (vector <Frame*>::iterator i = frames.begin(); i != frames.end(); ++i)
  {
    Frame *f = *i;
    if (f->getID() < (*curFrame)->getID())
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
    else if (f->getID() > (*curFrame)->getID())
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
        ss << "Couldn't find previous keyframe to the frame " << (*curFrame)->getID();
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
