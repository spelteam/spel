#include "detector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "
namespace SPEL
{
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
              if (!detFound) //this detector score is missing
              {
                Score newScore(1.0, m->first, m->second);
                scores.push_back(newScore);
              }
            }
            l->setScores(scores);
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

  vector <vector <LimbLabel> > Detector::detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels)
  {
    float searchDistCoeff = 0.5;
    const string sSearchDistCoeff = "searchDistCoeff";

    float minTheta = 90; // border for search
    const string sMinTheta = "minTheta";

    float maxTheta = 100; // border for search
    const string sMaxTheta = "maxTheta";

    float stepTheta = 10; // angular step of search
    const string sStepTheta = "stepTheta";

    uint32_t uniqueLocationCandidates = 4; // limiting the choice of the solutions number for each bodypart
    const string sUniqueLocationCandidates = "uniqueLocationCandidates";

    float scaleParam = 1; // scaling coefficient
    const string sScaleParam = "scaleParam";

    float searchDistCoeffMult = 1.25;
    const string sSearchDistCoeffMult = "searchDistCoeffMult";

#ifdef DEBUG
    uint8_t debugLevel = 5;
#else
    uint8_t debugLevel = 1;
#endif // DEBUG
    string sDebugLevel = "debugLevel";

    float rotationThreshold = 0.025f;
    const string sRotationThreshold = "rotationThreshold";

    float isWeakThreshold = 0.1f;
    const string sisWeakThreshold = "isWeakThreshold";

    float searchStepCoeff = 0.2f;
    const string sSearchStepCoeff = "searchStepCoeff";

    // first we need to check all used params
    params.emplace(sSearchDistCoeff, searchDistCoeff);
    params.emplace(sMinTheta, minTheta);
    params.emplace(sMaxTheta, maxTheta);
    params.emplace(sStepTheta, stepTheta);
    params.emplace(sUniqueLocationCandidates, uniqueLocationCandidates);
    params.emplace(sScaleParam, scaleParam);
    params.emplace(sSearchDistCoeffMult, searchDistCoeffMult);

    params.emplace(sDebugLevel, debugLevel);
    params.emplace(sRotationThreshold, rotationThreshold);
    params.emplace(sisWeakThreshold, isWeakThreshold);
    params.emplace(sSearchStepCoeff, searchStepCoeff);

    //now set actual param values
    searchDistCoeff = params.at(sSearchDistCoeff);
    minTheta = params.at(sMinTheta);
    maxTheta = params.at(sMaxTheta);
    stepTheta = params.at(sStepTheta);
    uniqueLocationCandidates = params.at(sUniqueLocationCandidates);
    scaleParam = params.at(sScaleParam);
    searchDistCoeffMult = params.at(sSearchDistCoeffMult);
    debugLevel = params.at(sDebugLevel);
    rotationThreshold = params.at(sRotationThreshold);
    isWeakThreshold = params.at(sisWeakThreshold);
    searchStepCoeff = params.at(sSearchStepCoeff);
    debugLevelParam = static_cast <uint8_t> (params.at(sDebugLevel));

    int originalSize = frame->getFrameSize().height;

    Frame *workFrame = 0;
    if (frame->getFrametype() == KEYFRAME)
      workFrame = new Keyframe();
    else if (frame->getFrametype() == LOCKFRAME)
      workFrame = new Lockframe();
    else if (frame->getFrametype() == INTERPOLATIONFRAME)
      workFrame = new Interpolation();

    workFrame = frame->clone(workFrame);

    float resizeFactor = workFrame->Resize(maxFrameHeight);

    vector <vector <LimbLabel> > t;
    Skeleton skeleton = workFrame->getSkeleton(); // copy skeleton from the frame
    tree <BodyPart> partTree = skeleton.getPartTree(); // copy tree of bodypart from the skeleton

    Mat maskMat = workFrame->getMask(); // copy mask from the frame

    stringstream detectorName;
    detectorName << getID();

    // For all body parts
    for (tree <BodyPart>::iterator iteratorBodyPart = partTree.begin(); iteratorBodyPart != partTree.end(); ++iteratorBodyPart)
    { //Temporary variables
      vector <LimbLabel> labels;
      vector <LimbLabel> sortedLabels;
      Point2f j0, j1;

      try
      {
        j0 = skeleton.getBodyJoint(iteratorBodyPart->getParentJoint())->getImageLocation(); // copy current bodypart parent joint
        j1 = skeleton.getBodyJoint(iteratorBodyPart->getChildJoint())->getImageLocation(); // copy current bodypart child joint
      }
      catch (...)
      {
        stringstream ss;
        ss << "Can't get joints";
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }

      float boneLength = getBoneLength(j0, j1); // distance between nodes
      float boxWidth = getBoneWidth(boneLength, *iteratorBodyPart); // current body part polygon width
      Point2f direction = j1 - j0; // direction of bodypart vector
      float theta = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI));  // bodypart tilt angle 
      float minDist = boxWidth * params.at(sSearchStepCoeff); // linear step of searching
      if (minDist < 2) minDist = 2; // the minimal linear step
      float searchDistance = iteratorBodyPart->getSearchRadius();
      try
      {
        if (searchDistance <= 0)
          searchDistance = boneLength * params.at(sSearchDistCoeff); // the limiting of search area
      }
      catch (...)
      {
        stringstream ss;
        ss << "Maybe there is no '" << sSearchDistCoeff << "' param";
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
      if (searchDistance <= 0)
        searchDistance = minDist + 1;
      Point2f suggestStart = 0.5 * j1 + 0.5 * j0; // reference point - the bodypart center
      // Scan the area around the reference point
      for (float x = suggestStart.x - searchDistance * 0.5f; x < suggestStart.x + searchDistance * 0.5f; x += minDist)
      {
        for (float y = suggestStart.y - searchDistance * 0.5f; y < suggestStart.y + searchDistance * 0.5f; y += minDist)
        {
          if (x < maskMat.cols && y < maskMat.rows)
          {
            uint8_t mintensity = 0;
            try
            {
              mintensity = maskMat.at<uint8_t>((int)y, (int)x); // copy mask at current pixel
            }
            catch (...)
            {
              stringstream ss;
              ss << "Can't get value in maskMat at " << "[" << (int)y << "][" << (int)x << "]";
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
            bool blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold
            if (!blackPixel)
            { // Scan the possible rotation zone
              float deltaTheta = abs(iteratorBodyPart->getRotationSearchRange());// + abs(rotationThreshold);
              float maxLocalTheta = iteratorBodyPart->getRotationSearchRange() == 0 ? maxTheta : deltaTheta;
              float minLocalTheta = iteratorBodyPart->getRotationSearchRange() == 0 ? minTheta : deltaTheta;
              for (float rot = theta - minLocalTheta; rot < theta + maxLocalTheta; rot += stepTheta)
              {
                // build  the vector label
                sortedLabels.push_back(generateLabel(boneLength, rot, x, y, *iteratorBodyPart, workFrame)); // add label to current bodypart labels
              }
            }
          }
        }
      }
      if (sortedLabels.size() == 0) // if labels for current body part is not builded
      {
        for (float rot = theta - minTheta; (rot < theta + maxTheta || (rot == theta - minTheta && rot >= theta + maxTheta)); rot += stepTheta)
        {
          // build  the vector label
          sortedLabels.push_back(generateLabel(boneLength, rot, suggestStart.x, suggestStart.y, *iteratorBodyPart, workFrame)); // add label to current bodypart labels
        }
      }
      float uniqueLocationCandidates = 0;
      try
      {
        uniqueLocationCandidates = params.at(sUniqueLocationCandidates); // copy the value from input parameters
      }
      catch (...)
      {
        stringstream ss;
        ss << "Maybe there is no '" << sUniqueLocationCandidates << "' param";
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }
      if (sortedLabels.size() > 0) // if labels vector is not empty
      {
        sort(sortedLabels.begin(), sortedLabels.end()); // sort labels by "SumScore" ?
        Mat locations(workFrame->getFrameSize().height, workFrame->getFrameSize().width, DataType<uint32_t>::type); // create the temporary matrix
        for (int32_t i = 0; i < workFrame->getFrameSize().width; i++)
        {
          for (int32_t j = 0; j < workFrame->getFrameSize().height; j++)
          {
            try
            {
              locations.at<uint32_t>(j, i) = 0; // init elements of the "location" matrix
            }
            catch (...)
            {
              stringstream ss;
              ss << "There is no value of locations at " << "[" << j << "][" << i << "]";
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
          }
        }

        vector <LimbLabel> orphanedLabels;
        for (vector <vector <LimbLabel>>::iterator i = limbLabels.begin(); i != limbLabels.end(); ++i)
        {
          if (i->size() > 0 && iteratorBodyPart->getPartID() == i->at(0).getLimbID())
          {
            copy(i->begin(), i->end(), orphanedLabels.begin());
            break;
          }
        }

        // For all "sortedLabels"
        for (uint32_t i = 0; i < sortedLabels.size(); i++)
        {
          uint32_t x = (uint32_t)sortedLabels.at(i).getCenter().x; // copy center coordinates of current label
          uint32_t y = (uint32_t)sortedLabels.at(i).getCenter().y; // copy center coordinates of current label
          try
          {
            bool bFound = false;

            for (vector <LimbLabel>::iterator l = orphanedLabels.begin(); l != orphanedLabels.end(); ++i)
            {
              vector <Point2f> first = l->getPolygon();
              vector <Point2f> second = sortedLabels.at(i).getPolygon();
              if (equal(first.begin(), first.end(), second.begin()))
              {
                orphanedLabels.erase(l);
                bFound = true;
                break;
              }
            }

            if (bFound || locations.at<uint32_t>(y, x) < uniqueLocationCandidates) // current point is occupied by less then "uniqueLocationCandidates" of labels with a greater score
            {
              try
              {
                labels.push_back(sortedLabels.at(i)); // add the current label in the resulting set of labels
              }
              catch (...)
              {
                stringstream ss;
                ss << "Maybe there is no value of sortedLabels at " << "[" << i << "]";
                if (debugLevelParam >= 1)
                  cerr << ERROR_HEADER << ss.str() << endl;
                throw logic_error(ss.str());
              }
              if (!bFound)
                locations.at<uint32_t>(y, x) += 1; // increase the counter of labels number at given point
            }
          }
          catch (...)
          {
            stringstream ss;
            ss << "Maybe there is no value of locations at " << "[" << y << "][" << x << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
        locations.release();

        // Generate LimbLabels for left orphaned labels
        for (vector <LimbLabel>::iterator l = orphanedLabels.begin(); l != orphanedLabels.end(); ++l)
        {
          labels.push_back(generateLabel(boneLength, l->getAngle(), l->getCenter().x, l->getCenter().y, *iteratorBodyPart, workFrame)); // add label to current bodypart labels
        }
        // Sort labels again
        sort(labels.begin(), labels.end());

      }
      PoseHelper::RecalculateScoreIsWeak(labels, detectorName.str(), isWeakThreshold);
      if (labels.size() > 0)
        t.push_back(labels); // add current point labels
    }
    maskMat.release();

    delete workFrame;

    for (auto i = 0; i < t.size(); ++i)
    {
      for (auto j = 0; j < t.at(i).size(); ++j)
      {
        LimbLabel label = t.at(i).at(j);
        label.Resize(pow(resizeFactor, -1));
        t[i][j] = label;
      }
    }

    return merge(limbLabels, t);
  }

  LimbLabel Detector::generateLabel(float boneLength, float rotationAngle, float x, float y, BodyPart bodyPart, Frame *workFrame)
  {
    // Create a new label vector and build it label
    Point2f p0 = Point2f(0, 0); // the point of unit vector
    Point2f p1 = Point2f(1.0, 0); // the point of unit vector
    p1 *= boneLength; // change the vector length 
    p1 = PoseHelper::rotatePoint2D(p1, p0, rotationAngle); // rotate the vector
    Point2f mid = 0.5 * p1; // center of the vector
    p1 = p1 + Point2f(x, y) - mid; // shift the vector to current point
    p0 = Point2f(x, y) - mid; // shift the vector to current point

    return generateLabel(bodyPart, workFrame, p0, p1); // build  the vector label
  }

}
