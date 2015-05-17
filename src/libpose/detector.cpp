#include "detector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "

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

POSERECT <Point2f> Detector::getBodyPartRect(BodyPart bodyPart, Point2f j0, Point2f j1, Size blockSize)
{
  Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
  //float x = boxCenter.x;
  //float y = boxCenter.y;
  float boneLength = getBoneLength(j0, j1);
  if (blockSize.width > 0)
  {
    if (boneLength < blockSize.width)
    {
      boneLength = static_cast <float> (blockSize.width - 1);
    }
    else
    {
      boneLength = boneLength + blockSize.width - ((int)boneLength % blockSize.width) - 1;
    }
  }
  float boxWidth = getBoneWidth(boneLength, bodyPart);
  if (blockSize.height > 0)
  {
    if (boxWidth < blockSize.height)
    {
      boxWidth = static_cast <float> (blockSize.height - 1);
    }
    else
    {
      boxWidth = boxWidth + blockSize.width - ((int)boxWidth % blockSize.height) - 1;
    }
  }
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
  Mat partImage = Mat(size, CV_8UC3, Scalar(0, 0, 0));
  Point2f center = initialRect.GetCenter<Point2f>();
  Point2f newCenter = Point2f(0.5f * size.width, 0.5f * size.height);
  for (int32_t x = 0; x < size.width; x++)
  {
    for (int32_t y = 0; y < size.height; y++)
    {
      Point2f p = Point2f((float)x, (float)y);
      try
      {
        p = PoseHelper::rotatePoint2D(p, newCenter, angle) + center - newCenter;
        Vec3b color = imgSource.at<Vec3b>((int)round(p.y), (int)round(p.x));
        partImage.at<Vec3b>(y, x) = color;
      }
      catch (...)
      {
        stringstream ss;
        ss << "Couldn't get value of indeces " << "[" << x << "][" << y << "] from indeces [" << p.x << "][" << p.y << "]";
#ifdef DEBUG
        cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
        throw logic_error(ss.str());
      }
    }
  }
  return partImage;
}

vector <vector <LimbLabel>> Detector::merge(vector <vector <LimbLabel>> first, vector <vector <LimbLabel>> second)
{
  if (first.size() != second.size() && first.size() > 0 && second.size() > 0)
  {
    stringstream ss;
    ss << "Can't merge vectors with different sizes (different count of BodyPart): First: " << first.size() << " Second: " << second.size();
#ifdef DEBUG
    cerr << ERROR_HEADER << ss.str() << endl;
#endif  // DEBUG
    throw logic_error(ss.str());
  }
  if (first.size() == 0)
  {
    return second;
  }
  if (second.size() == 0)
  {
    return first;
  }
  bool bFound = false;
  for (vector <vector <LimbLabel>>::iterator f = first.begin(); f != first.end(); ++f)
  {
    for (vector <vector <LimbLabel>>::iterator s = second.begin(); s != second.end(); ++s)
    {
      if (f->front().getLimbID() == s->front().getLimbID())
      {
        for (vector <LimbLabel>::iterator fl = f->begin(); fl != f->end(); ++fl)
        {
          for (vector <LimbLabel>::iterator sl = s->begin(); sl != s->end(); ++sl)
          {
            if (fl->getCenter() == sl->getCenter() && fl->getAngle() == sl->getAngle())
            {
              bFound = true;
              vector <Score> sls = sl->getScores();
              for (vector <Score>::iterator ss = sls.begin(); ss != sls.end(); ++ss)
              {
                fl->addScore(*ss);
              }
            }
          }
        }

        for (vector <LimbLabel>::iterator sl = s->begin(); sl != s->end(); ++sl)
        {
          bFound = false;
          for (vector <LimbLabel>::iterator fl = f->begin(); fl != f->end(); ++fl)
          {
            if (fl->getCenter() == sl->getCenter() && fl->getAngle() == sl->getAngle())
            {
              bFound = true;
              break;
            }
          }
          if (!bFound)
          {
            f->push_back(*sl);
          }
        }
        break;
      }
    }
  }

  vector <vector <LimbLabel>> result;
  for (vector <vector <LimbLabel>>::iterator l = first.begin(); l != first.end(); ++l)
  {
    vector <LimbLabel> v = *l;
    sort(v.begin(), v.end());
    result.push_back(v);
  }

  return result;
}
