#include "detector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "
namespace SPEL
{
  Detector::Detector(void)
  {
  }

  Detector::~Detector(void)
  {
  }

  float Detector::getBoneLength(Point2f begin, Point2f end)
  {
    return (begin == end) ? 1.0f : (float)sqrt(PoseHelper::distSquared(begin, end));
  }

  float Detector::getBoneWidth(float length, BodyPart bodyPart)
  {
    auto ratio = bodyPart.getLWRatio();
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
    auto boneLength = getBoneLength(j0, j1);
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
    auto boxWidth = getBoneWidth(boneLength, bodyPart);
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
    auto angle = static_cast <float> (PoseHelper::angle2D(1.0, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI));
    Point2f c1, c2, c3, c4, polyCenter;
    c1 = Point2f(0.f, 0.5f * boxWidth);
    c2 = Point2f(boneLength, 0.5f * boxWidth);
    c3 = Point2f(boneLength, -0.5f * boxWidth);
    c4 = Point2f(0.f, -0.5f * boxWidth);

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
    auto partImage = Mat(size, CV_8UC3, Scalar(0, 0, 0));
    auto center = initialRect.GetCenter<Point2f>();
    auto newCenter = Point2f(0.5f * size.width, 0.5f * size.height);
    auto width = imgSource.size().width; // !!! For testing
    auto height = imgSource.size().height; // !!! For testing
    for (auto x = 0; x < size.width; x++)
    {
      for (auto y = 0; y < size.height; y++)
      {
        auto p = Point2f((float)x, (float)y);
        try
        {
          p = PoseHelper::rotatePoint2D(p, newCenter, angle) + center - newCenter;
          if (0 <= p.x && 0 <= p.y && p.x < width - 1 && p.y < height - 1) // !!! For testing
            if (0 <= x && x < size.width - 1 && 0 <= y && y < size.height - 1) // !!! For testing
            {
              auto color = imgSource.at<Vec3b>((int)round(p.y), (int)round(p.x));
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

  map <uint32_t, vector <LimbLabel>> Detector::merge(map <uint32_t, vector <LimbLabel>> first, map <uint32_t, vector <LimbLabel>> second)
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

    map <string, float> detectorNames;

    map <uint32_t, vector <LimbLabel>> result;

    for (auto part : first) //for each part
    {
      vector<LimbLabel> partResult;

      //iterate through first list, compare to second list, any labels that are matched are combined and pushed
      //any labels that are not found, are added
      for (auto firstIter : part.second) //for each label in first
      {
        auto isFound = false;

        LimbLabel foundLabel;
        try
        {
          auto s = second.at(part.first);
          for (auto secondIter : s)
          {
            if (firstIter.getLimbID() == secondIter.getLimbID() && firstIter.getPolygon() == secondIter.getPolygon())
            {
              isFound = true;
              foundLabel = secondIter;
              break;
            }
          }
        }
        catch (...)
        {
          isFound = false;
        }

        if (isFound) //if label was found, create a copy, and add a score from other label to it
        {
          auto newLabel(firstIter);
          auto sl(foundLabel);
          //check any score differences, and push them
          auto secondScores = sl.getScores();
          auto firstScores = newLabel.getScores();

          for (auto i : secondScores)
          {
            auto scoreFound = false;
            for (auto j : firstScores)
            {
              if (i == j)
              {
                scoreFound = true;
                break;
              }
            }

            if (!scoreFound) //add if not found
              newLabel.addScore(i);
          }
          //emplace scores
          auto newLabelScores = newLabel.getScores();
          for (auto i : newLabelScores)
            detectorNames.emplace(i.getDetName(), i.getCoeff());
          newLabel.setScores(newLabelScores);
          partResult.push_back(newLabel);

        }
        else //if label wasn't found, create and push to vector
        {
          auto newLabel(firstIter);
          //emplace scores
          auto newLabelScores = newLabel.getScores();
          for (auto i : newLabelScores)
            detectorNames.emplace(i.getDetName(), i.getCoeff());
          newLabel.setScores(newLabelScores);
          partResult.push_back(newLabel);
        }
      }

      //now iterate through the second list, and push back any labels that are not found in result vector
      try
      {
        auto s = second.at(part.first);

        for (auto secondIter : s)
        {
          auto isFound = false;
          for (auto resIter : partResult)
          {
            if (secondIter.getLimbID() == resIter.getLimbID() && secondIter.getPolygon() == resIter.getPolygon())
            {
              isFound = true;
              break;
            }
          }
          if (!isFound) //if label not found, push it to result vector
          {
            auto newLabel(secondIter);
            auto newLabelScores = newLabel.getScores();
            //emplace scores
            for (auto i : newLabelScores)
              detectorNames.emplace(i.getDetName(), i.getCoeff());
            newLabel.setScores(newLabelScores);
            partResult.push_back(newLabel);
          }
        }
      }
      catch (...)
      {
      }

      result.insert(pair < uint32_t, vector <LimbLabel>>(part.first, partResult));
    }

    //now the vectors are merged, but there may be score mismatches
    //unique names of detectors along with their params that are present are stored in detectorNames (duplicates for same detector are ignored)

    //look at each label, and add a score of 1.0 for each missing

    for (auto &&r : result)
    {
      for (auto &&l : r.second) //for each label
      {
        auto scores = l.getScores();

        for (auto m : detectorNames) //for each detector, check whether label has a score for it
        {
          auto detFound = false;

          for (auto &&i : scores)
          {
            if (m.first == i.getDetName()) //name
            {
              if (i.getScore() == -1) //change all -1 to 1
                i.setScore(1.0f);
              detFound = true;
              break;
            }
          }
          if (!detFound) //this detector score is missing
            scores.push_back(Score(1.0f, m.first, m.second));
        }
        l.setScores(scores);
      }
    }
    //finally, sort the labels
    for (auto &&l : result)
    {
      sort(l.second.begin(), l.second.end());
    }

    return result;
  }

  LimbLabel Detector::generateLabel(BodyPart bodyPart, Point2f j0, Point2f j1, string detectorName, float _usedet)
  {
    auto boxCenter = j0 * 0.5 + j1 * 0.5;
    auto rot = float(PoseHelper::angle2D(1, 0, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI));
    auto rect = getBodyPartRect(bodyPart, j0, j1);

    auto score = compare();
    vector <Score> s;
    s.push_back(Score(score, detectorName, _usedet));
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

  map <uint32_t, vector <LimbLabel>> Detector::detect(Frame *frame, map <string, float> params, map <uint32_t, vector <LimbLabel>> limbLabels)
  {
    auto searchDistCoeff = 0.5f;
    const string sSearchDistCoeff = "searchDistCoeff";

    auto minTheta = 90.0f; // border for search
    const string sMinTheta = "minTheta";

    auto maxTheta = 100.0f; // border for search
    const string sMaxTheta = "maxTheta";

    auto stepTheta = 10.0f; // angular step of search
    const string sStepTheta = "stepTheta";

    auto uniqueLocationCandidates = 4.0f; // limiting the choice of the solutions number for each bodypart
    const string sUniqueLocationCandidates = "uniqueLocationCandidates";

    auto scaleParam = 1.0f; // scaling coefficient
    const string sScaleParam = "scaleParam";

    auto searchDistCoeffMult = 1.25f;
    const string sSearchDistCoeffMult = "searchDistCoeffMult";

#ifdef DEBUG
    uint8_t debugLevel = 5;
#else
    uint8_t debugLevel = 1;
#endif // DEBUG
    const string sDebugLevel = "debugLevel";

    auto rotationThreshold = 0.025f;
    const string sRotationThreshold = "rotationThreshold";

    auto isWeakThreshold = 0.1f;
    const string sisWeakThreshold = "isWeakThreshold";

    auto searchStepCoeff = 0.2f;
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

    auto originalSize = frame->getFrameSize().height;

    Frame *workFrame = 0;
    if (frame->getFrametype() == KEYFRAME)
      workFrame = new Keyframe();
    else if (frame->getFrametype() == LOCKFRAME)
      workFrame = new Lockframe();
    else if (frame->getFrametype() == INTERPOLATIONFRAME)
      workFrame = new Interpolation();

    workFrame = frame->clone(workFrame);

    auto resizeFactor = workFrame->Resize(maxFrameHeight);

    map <uint32_t, vector <LimbLabel> > tempLabelVector;
    auto skeleton = workFrame->getSkeleton(); // copy skeleton from the frame
    auto partTree = skeleton.getPartTree(); // copy tree of bodypart from the skeleton

    auto maskMat = workFrame->getMask(); // copy mask from the frame

    stringstream detectorName;
    detectorName << getID();

    // For all body parts
    for (auto iteratorBodyPart : partTree)
    { //Temporary variables
      vector <LimbLabel> labels;
      vector <LimbLabel> sortedLabels;
      Point2f j0, j1;

      try
      {
        j0 = skeleton.getBodyJoint(iteratorBodyPart.getParentJoint())->getImageLocation(); // copy current bodypart parent joint
        j1 = skeleton.getBodyJoint(iteratorBodyPart.getChildJoint())->getImageLocation(); // copy current bodypart child joint
      }
      catch (...)
      {
        stringstream ss;
        ss << "Can't get joints";
        if (debugLevelParam >= 1)
          cerr << ERROR_HEADER << ss.str() << endl;
        throw logic_error(ss.str());
      }

      auto boneLength = getBoneLength(j0, j1); // distance between nodes
      auto boxWidth = getBoneWidth(boneLength, iteratorBodyPart); // current body part polygon width
      auto direction = j1 - j0; // direction of bodypart vector
      auto theta = float(PoseHelper::angle2D(1.0, 0, direction.x, direction.y) * (180.0 / M_PI));  // bodypart tilt angle 
      auto minDist = boxWidth * params.at(sSearchStepCoeff); // linear step of searching
      if (minDist < 2) minDist = 2; // the minimal linear step
      auto searchDistance = iteratorBodyPart.getSearchRadius();
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
      auto suggestStart = 0.5 * j1 + 0.5 * j0; // reference point - the bodypart center
      auto searchXMin = suggestStart.x - searchDistance * 0.5f;
      auto searchXMax = suggestStart.x + searchDistance * 0.5f;
      auto searchYMin = suggestStart.y - searchDistance * 0.5f;
      auto searchYMax = suggestStart.y + searchDistance * 0.5f;
      // Scan the area around the reference point
      for (auto x = searchXMin; x < searchXMax; x += minDist)
      {
        for (auto y = searchYMin; y < searchYMax; y += minDist)
        {
          if (x < maskMat.cols && y < maskMat.rows && x >= 0 && y >= 0)
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
            auto blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold
            if (!blackPixel)
            { // Scan the possible rotation zone
              auto deltaTheta = abs(iteratorBodyPart.getRotationSearchRange());// + abs(rotationThreshold);
              auto maxLocalTheta = iteratorBodyPart.getRotationSearchRange() == 0 ? maxTheta : deltaTheta;
              auto minLocalTheta = iteratorBodyPart.getRotationSearchRange() == 0 ? minTheta : deltaTheta;
              for (auto rot = theta - minLocalTheta; rot < theta + maxLocalTheta; rot += stepTheta)
              {
                // build  the vector label
                sortedLabels.push_back(generateLabel(boneLength, rot, x, y, iteratorBodyPart, workFrame)); // add label to current bodypart labels
              }
            }
          }
        }
      }
      if (sortedLabels.size() == 0) // if labels for current body part is not builded
      {
        for (auto rot = theta - minTheta; (rot < theta + maxTheta || (rot == theta - minTheta && rot >= theta + maxTheta)); rot += stepTheta)
        {
          // build  the vector label
          sortedLabels.push_back(generateLabel(boneLength, rot, suggestStart.x, suggestStart.y, iteratorBodyPart, workFrame)); // add label to current bodypart labels
        }
      }
      auto uniqueLocationCandidates = .0f;
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
        for (auto i = 0; i < workFrame->getFrameSize().width; i++)
        {
          for (auto j = 0; j < workFrame->getFrameSize().height; j++)
          {
            try
            {
              locations.at<uint32_t>(j, i) = 0; // init elements of the "location" matrix
            }
            catch (...)
            {
              locations.release();
              stringstream ss;
              ss << "There is no value of locations at " << "[" << j << "][" << i << "]";
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
              throw logic_error(ss.str());
            }
          }
        }

        vector <LimbLabel> generatedPartLabels;

        if (limbLabels.size() > 0)
        {
          if (sortedLabels.size() > 0)
          {
            try
            {
              auto partLabel = limbLabels.at(sortedLabels.front().getLimbID());
              for (auto generated : partLabel)
                generatedPartLabels.push_back(generated);
            }
            catch (...)
            {
              stringstream ss;
              ss << "Can't find generated limb label " << sortedLabels.front().getLimbID();
              if (debugLevelParam >= 1)
                cerr << ERROR_HEADER << ss.str() << endl;
            }
          }
        }

        // For all "sortedLabels"
        for (auto i : sortedLabels)
        {
          auto bFound = false;
          try
          {
            for (auto generatedLabels : generatedPartLabels)
            {
              auto first = i.getPolygon();
              auto second = generatedLabels.getPolygon();
              if (first.size() == second.size())
                for (auto polygonSize = 0; polygonSize < first.size(); polygonSize++)
                  bFound = bFound && first.at(polygonSize) == second.at(polygonSize);
            }
          }
          catch (...)
          {
            stringstream ss;
            ss << "Can't find generated limb label";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
          }
          
          auto x = (uint32_t)i.getCenter().x; // copy center coordinates of current label
          auto y = (uint32_t)i.getCenter().y; // copy center coordinates of current label
          try
          {
            if (bFound || locations.at<uint32_t>(y, x) < uniqueLocationCandidates) // current point is occupied by less then "uniqueLocationCandidates" of labels with a greater score
            {
              try
              {
                labels.push_back(i); // add the current label in the resulting set of labels
              }
              catch (...)
              {
                locations.release();
                stringstream ss;
                ss << "Maybe there is no value of sortedLabels";
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
            locations.release();
            stringstream ss;
            ss << "Maybe there is no value of locations at " << "[" << y << "][" << x << "]";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
            throw logic_error(ss.str());
          }
        }
        locations.release();

        // Generate LimbLabels for left orphaned labels

        for (auto partLabel : limbLabels)
        {
          try
          {
            if (labels.size() == 0 || partLabel.first == labels.front().getLimbID())
            {
              for (auto potentiallyOrphanedLabels : partLabel.second)
              {
                auto bFound = false;
                try
                {
                  for (auto generatedLabels : labels)
                  {
                    auto potentiallyOrphanedLabelsPolygon = potentiallyOrphanedLabels.getPolygon();
                    auto generatedLabelsPolygon = generatedLabels.getPolygon();
                    if (potentiallyOrphanedLabelsPolygon.size() == generatedLabelsPolygon.size())
                    {
                      for (auto polygonSize = 0; polygonSize < potentiallyOrphanedLabelsPolygon.size(); polygonSize++)
                      {
                        if(potentiallyOrphanedLabelsPolygon.at(polygonSize) == generatedLabelsPolygon.at(polygonSize))
                            bFound=true;
                      }
                    }
                  }
                }
                catch (...)
                {
                  stringstream ss;
                  ss << "Can't find generated limb label";
                  if (debugLevelParam >= 1)
                    cerr << ERROR_HEADER << ss.str() << endl;
                }
                if (!bFound)
                  labels.push_back(generateLabel(boneLength, potentiallyOrphanedLabels.getAngle(), potentiallyOrphanedLabels.getCenter().x, potentiallyOrphanedLabels.getCenter().y, iteratorBodyPart, workFrame));
              }
              break;
            }
          }
          catch (...)
          {
            stringstream ss;
            ss << "Something went wrong. Just keep going";
            if (debugLevelParam >= 1)
              cerr << ERROR_HEADER << ss.str() << endl;
          }
        }

        //// Sort labels again
        sort(labels.begin(), labels.end());

      }
      PoseHelper::RecalculateScoreIsWeak(labels, detectorName.str(), isWeakThreshold);
      if (labels.size() > 0)
        tempLabelVector.insert(pair<uint32_t, vector <LimbLabel>>(iteratorBodyPart.getPartID(), labels)); // add current point labels
    }

    delete workFrame;

    for (auto i = 0; i < tempLabelVector.size(); ++i)
    {
      for (auto j = 0; j < tempLabelVector.at(i).size(); ++j)
      {
        LimbLabel label = tempLabelVector.at(i).at(j);
        label.Resize(pow(resizeFactor, -1));
        tempLabelVector[i][j] = label;
      }
    }

    return merge(limbLabels, tempLabelVector);
  }

  LimbLabel Detector::generateLabel(float boneLength, float rotationAngle, float x, float y, BodyPart bodyPart, Frame *workFrame)
  {
    // Create a new label vector and build it label
    auto p0 = Point2f(0, 0); // the point of unit vector
    auto p1 = Point2f(1.0, 0); // the point of unit vector
    p1 *= boneLength; // change the vector length 
    p1 = PoseHelper::rotatePoint2D(p1, p0, rotationAngle); // rotate the vector
    auto mid = 0.5 * p1; // center of the vector
    p1 = p1 + Point2f(x, y) - mid; // shift the vector to current point
    p0 = Point2f(x, y) - mid; // shift the vector to current point

    return generateLabel(bodyPart, workFrame, p0, p1); // build  the vector label
  }

}
