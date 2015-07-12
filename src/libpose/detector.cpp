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
  float angle = float(PoseHelper::angle2D(1.0, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI));
  Point2f c1, c2, c3, c4, polyCenter;
  c1 = Point2f(0.f, 0.5f * boxWidth);
  c2 = Point2f(boneLength, 0.5f * boxWidth);
  c3 = Point2f(boneLength, -0.5f * boxWidth);
  c4 = Point2f(0.f, -0.5f * boxWidth);

  //  polyCenter = Point2f(boneLength * 0.5f, 0.f);
  //  c1 = PoseHelper::rotatePoint2D(c1, polyCenter, angle) + boxCenter - polyCenter;
  //  c2 = PoseHelper::rotatePoint2D(c2, polyCenter, angle) + boxCenter - polyCenter;
  //  c3 = PoseHelper::rotatePoint2D(c3, polyCenter, angle) + boxCenter - polyCenter;
  //  c4 = PoseHelper::rotatePoint2D(c4, polyCenter, angle) + boxCenter - polyCenter;


  c1 = PoseHelper::rotatePoint2D(c1, Point2f(0, 0), angle);
  c2 = PoseHelper::rotatePoint2D(c2, Point2f(0, 0), angle);
  c3 = PoseHelper::rotatePoint2D(c3, Point2f(0, 0), angle);
  c4 = PoseHelper::rotatePoint2D(c4, Point2f(0, 0), angle);

  polyCenter = 0.25*c1 + 0.25*c2 + 0.25*c3 + 0.25*c4;

  c1 = c1 - polyCenter + boxCenter;
  c2 = c2 - polyCenter + boxCenter;
  c3 = c3 - polyCenter + boxCenter;
  c4 = c4 - polyCenter + boxCenter;

  return POSERECT <Point2f>(c1, c2, c3, c4);
}

Mat Detector::rotateImageToDefault(Mat imgSource, POSERECT <Point2f> &initialRect, float angle, Size size)
{
  Mat partImage = Mat(size, CV_8UC3, Scalar(0, 0, 0));
  Point2f center = initialRect.GetCenter<Point2f>();
  Point2f newCenter = Point2f(0.5f * size.width, 0.5f * size.height);
  int width = imgSource.size().width; // !!! For testing
  int height = imgSource.size().height; // !!! For testing
  for (int32_t x = 0; x < size.width; x++)
  {
    for (int32_t y = 0; y < size.height; y++)
    {
      Point2f p = Point2f((float)x, (float)y);
      try
      {
        p = PoseHelper::rotatePoint2D(p, newCenter, angle) + center - newCenter;
        if (0 <= p.x && 0 <= p.y && p.x < width - 1 && p.y < height - 1) // !!! For testing
          if (0 <= x && x < size.width - 1 && 0 <= y && y < size.height - 1) // !!! For testing
          {
            Vec3b color = imgSource.at<Vec3b>((int)round(p.y), (int)round(p.x));
            partImage.at<Vec3b>(y, x) = color;
          }
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
  //  if(first==second) //if two identical vectors are thrown in
  //      return first;

  map<string, float> detectorNames;

  vector<vector<LimbLabel>> result;

  for (vector <vector <LimbLabel>>::iterator part = first.begin(); part != first.end(); ++part) //for each part
  {
    vector<LimbLabel> partResult;

    //iterate through first list, compare to second list, any labels that are matched are combined and pushed
    //any labels that are not found, are added
    for (vector<LimbLabel>::iterator firstIter = part->begin(); firstIter != part->end(); ++firstIter) //for each label in first
    {
      bool isFound = false;

      LimbLabel foundLabel;
      for (vector <vector <LimbLabel>>::iterator s = second.begin(); s != second.end(); ++s)
      {
        if (firstIter->getLimbID() == s->front().getLimbID())
        {
          for (vector<LimbLabel>::iterator secondIter = s->begin(); secondIter != s->end(); ++secondIter) //for each label second
          {
            if (firstIter->getPolygon() == secondIter->getPolygon()) //if they have the same polygon
            {
              isFound = true;
              foundLabel = *secondIter;
              break;
            }
          }
        }
      }

      if (isFound) //if label was found, create a copy, and add a score from other label to it
      {
        LimbLabel newLabel(*firstIter);
        LimbLabel sl(foundLabel);
        //check any score differences, and push them
        vector<Score> secondScores = sl.getScores();
        vector<Score> firstScores = newLabel.getScores();
        for (vector<Score>::iterator i = secondScores.begin(); i != secondScores.end(); ++i)
        {
          bool scoreFound = false;

          for (vector<Score>::iterator j = firstScores.begin(); j != firstScores.end(); ++j)
          {
            if ((*i) == (*j))
            {
              scoreFound = true;
              break;
            }
          }

          if (!scoreFound) //add if not found
            newLabel.addScore(*i);
        }
        //emplace scores
        vector <Score> newLabelScores = newLabel.getScores();
        for (vector <Score>::iterator i = newLabelScores.begin(); i != newLabelScores.end(); ++i)
          detectorNames.emplace(i->getDetName(), i->getCoeff());
        newLabel.setScores(newLabelScores);
        partResult.push_back(newLabel);

      }
      else //if label wasn't found, create and push to vector
      {
        LimbLabel newLabel(*firstIter);

        //emplace scores
        vector <Score> newLabelScores = newLabel.getScores();
        for (vector <Score>::iterator i = newLabelScores.begin(); i != newLabelScores.end(); ++i)
          detectorNames.emplace(i->getDetName(), i->getCoeff());
        newLabel.setScores(newLabelScores);
        partResult.push_back(newLabel);
      }
    }

    //now iterate through the second list, and push back any labels that are not found in result vector
    for (vector <vector <LimbLabel>>::iterator s = second.begin(); s != second.end(); ++s)
    {
      if ((part->front().getLimbID() == s->front().getLimbID()))
      {
        for (vector<LimbLabel>::iterator secondIter = s->begin(); secondIter != s->end(); ++secondIter) //for each label in second
        {
          bool isFound = false;

          for (vector<LimbLabel>::iterator resIter = partResult.begin(); resIter != partResult.end(); ++resIter)
          {
            if (resIter->getPolygon() == secondIter->getPolygon())
            {
              isFound = true;
              break;
            }
          }

          if (!isFound) //if label not found, push it to result vector
          {
            LimbLabel newLabel(*secondIter);
            vector <Score> newLabelScores = newLabel.getScores();
            //emplace scores
            for (vector <Score>::iterator i = newLabelScores.begin(); i != newLabelScores.end(); ++i)
              detectorNames.emplace(i->getDetName(), i->getCoeff());
            newLabel.setScores(newLabelScores);
            partResult.push_back(newLabel);
          }
        }
      }
    }
    result.push_back(partResult);
  }

  //now the vectors are merged, but there may be score mismatches
  //unique names of detectors along with their params that are present are stored in detectorNames (duplicates for same detector are ignored)

  //look at each label, and add a score of 1.0 for each missing
  for (vector <vector <LimbLabel>>::iterator part = first.begin(); part != first.end(); ++part) //for each part
  {
    for (vector <vector <LimbLabel>>::iterator r = result.begin(); r != result.end(); ++r)
    {
      if (part->front().getLimbID() == r->front().getLimbID())
      {
        for (vector<LimbLabel>::iterator l = r->begin(); l != r->end(); ++l) //for each label
        {
          vector<Score> scores = l->getScores();

          for (map<string, float>::iterator m = detectorNames.begin(); m != detectorNames.end(); ++m) //for each detector, check whether label has a score for it
          {
            bool detFound = false;

            for (vector <Score>::iterator i = scores.begin(); i != scores.end(); ++i)
            {
              if (m->first == i->getDetName()) //name
              {
                if (i->getScore() == -1) //change all -1 to 1
                  i->setScore(1.0);
                detFound = true;
                break;
              }
            }
            l->setScores(scores);
            if (!detFound) //this detector score is missing
            {
              Score newScore(1.0, m->first, m->second);
              l->addScore(newScore);
            }
          }
        }
      }
    }
  }

  //finally, sort the labels
  for (vector <vector <LimbLabel>>::iterator l = result.begin(); l != result.end(); ++l)
  {
    sort(l->begin(), l->end());
  }

  return result;
}

LimbLabel Detector::generateLabel(BodyPart bodyPart, Point2f j0, Point2f j1, string detectorName, float _usedet)
{
  vector <Score> s;
  Point2f boxCenter = j0 * 0.5 + j1 * 0.5;
  float rot = float(PoseHelper::angle2D(1, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI));
  POSERECT <Point2f> rect = getBodyPartRect(bodyPart, j0, j1);
  
  float score = compare();
  Score sc(score, detectorName, _usedet);
  s.push_back(sc);
  return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s, score == -1.0f);
}

Frame *Detector::getFrame(uint32_t frameId)
{
  for (auto f : frames)
  {
    if (f->getID() == frameId)
      return f;
  }
  return 0;
}

