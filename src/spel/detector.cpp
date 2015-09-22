#include "detector.hpp"

#define ERROR_HEADER __FILE__ << ":" << __LINE__ << ": "
namespace SPEL
{
  Detector::Detector(void) noexcept
  {
  }

  Detector::~Detector(void) noexcept
  {
  }

  float Detector::getBoneLength(const cv::Point2f &begin, const cv::Point2f &end) noexcept
  {
    return (begin == end) ? 1.0f : static_cast<float>(sqrt(spelHelper::distSquared(begin, end)));
  }

  float Detector::getBoneWidth(const float &length, const BodyPart &bodyPart) noexcept
  {
    auto ratio = bodyPart.getLWRatio();
    if (ratio == 0.0f)
    {
      std::cerr << ERROR_HEADER << "Ratio can't be 0" << std::endl;
      return 0.0f;
    }
    return length / ratio;
  }

  POSERECT <cv::Point2f> Detector::getBodyPartRect(BodyPart bodyPart, cv::Point2f j0, cv::Point2f j1, cv::Size blockSize) noexcept
  {
    auto boxCenter = j0 * 0.5 + j1 * 0.5;
    auto boneLength = getBoneLength(j0, j1);
    if (blockSize.width > 0)
    {
      if (boneLength < blockSize.width)
        boneLength = static_cast <float> (blockSize.width - 1);
      else
        boneLength = boneLength + blockSize.width - (static_cast<int>(boneLength) % blockSize.width) - 1;
    }
    auto boxWidth = getBoneWidth(boneLength, bodyPart);
    if (blockSize.height > 0)
    {
      if (boxWidth < blockSize.height)
        boxWidth = static_cast <float> (blockSize.height - 1);
      else
        boxWidth = boxWidth + blockSize.width - (static_cast<int>(boxWidth) % blockSize.height) - 1;
    }
    auto angle = spelHelper::angle2D(1.0f, 0.0f, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI);
    auto c1 = cv::Point2f(0.f, 0.5f * boxWidth);
    auto c2 = cv::Point2f(boneLength, 0.5f * boxWidth);
    auto c3 = cv::Point2f(boneLength, -0.5f * boxWidth);
    auto c4 = cv::Point2f(0.f, -0.5f * boxWidth);

    c1 = spelHelper::rotatePoint2D(c1, cv::Point2f(0, 0), angle);
    c2 = spelHelper::rotatePoint2D(c2, cv::Point2f(0, 0), angle);
    c3 = spelHelper::rotatePoint2D(c3, cv::Point2f(0, 0), angle);
    c4 = spelHelper::rotatePoint2D(c4, cv::Point2f(0, 0), angle);

    auto polyCenter = 0.25*c1 + 0.25*c2 + 0.25*c3 + 0.25*c4;

    c1 = c1 - polyCenter + boxCenter;
    c2 = c2 - polyCenter + boxCenter;
    c3 = c3 - polyCenter + boxCenter;
    c4 = c4 - polyCenter + boxCenter;

    return POSERECT <cv::Point2f>(c1, c2, c3, c4);
  }

  cv::Mat Detector::rotateImageToDefault(cv::Mat imgSource, POSERECT <cv::Point2f> &initialRect, float angle, cv::Size size)
  {
    auto partImage = cv::Mat(size, CV_8UC3, cv::Scalar(0, 0, 0));
    auto center = initialRect.GetCenter<cv::Point2f>();
    auto newCenter = cv::Point2f(0.5f * size.width, 0.5f * size.height);
    auto width = imgSource.size().width; // !!! For testing
    auto height = imgSource.size().height; // !!! For testing
    for (auto x = 0; x < size.width; x++)
    {
      for (auto y = 0; y < size.height; y++)
      {
        auto p = cv::Point2f(static_cast<float>(x), static_cast<float>(y));
        p = spelHelper::rotatePoint2D(p, newCenter, angle) + center - newCenter;
        try
        {
          if (0 <= p.x && 0 <= p.y && p.x < width - 1 && p.y < height - 1) // !!! For testing
            if (0 <= x && x < size.width - 1 && 0 <= y && y < size.height - 1) // !!! For testing
              partImage.at<cv::Vec3b>(y, x) = imgSource.at<cv::Vec3b>(static_cast<int>(round(p.y)), static_cast<int>(round(p.x)));
        }
        catch (...)
        {
          std::stringstream ss;
          ss << "Couldn't get value of indeces " << "[" << x << "][" << y << "] from indeces [" << p.x << "][" << p.y << "]";
#ifdef DEBUG
          std::cerr << ERROR_HEADER << ss.str() << std::endl;
#endif  // DEBUG
          throw std::logic_error(ss.str());
        }
      }
    }
    return partImage;
  }

  std::map <uint32_t, std::vector <LimbLabel>> Detector::merge(const std::map <uint32_t, std::vector <LimbLabel>> &first, const std::map <uint32_t, std::vector <LimbLabel>> &second, const std::map <uint32_t, std::vector <LimbLabel>> &secondUnfiltered)
  {
    if (first.size() != second.size() && first.size() > 0 && second.size() > 0)
    {
      std::stringstream ss;
      ss << "Can't merge vectors with different sizes (different count of BodyPart): First: " << first.size() << " Second: " << second.size();
#ifdef DEBUG
      std::cerr << ERROR_HEADER << ss.str() << std::endl;
#endif  // DEBUG
      throw std::logic_error(ss.str());
    }
    if (first.size() == 0)
      return second;
    if (second.size() == 0)
      return first;

    std::map <std::string, float> detectorNames;

    std::map <uint32_t, std::vector <LimbLabel>> result;

    for (auto part : first) //for each part
    {
      std::vector<LimbLabel> partResult;

      //iterate through first list, compare to second list, any labels that are matched are combined and pushed
      //any labels that are not found, are added
      for (const auto &firstIter : part.second) //for each label in first
      {
        auto isFound = false;

        LimbLabel foundLabel;
        try
        {
          auto s = second.at(part.first);
          for (const auto &secondIter : s)
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

          for (const auto &i : secondScores)
          {
            auto scoreFound = false;
            for (const auto &j : firstScores)
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
          for (const auto &i : newLabelScores)
            detectorNames.emplace(i.getDetName(), i.getCoeff());
          newLabel.setScores(newLabelScores);
          partResult.push_back(newLabel);

        }
        else //if label wasn't found, create and push to vector
        {
          auto newLabel(firstIter);
          //emplace scores
          auto newLabelScores = newLabel.getScores();
          for (const auto &i : newLabelScores)
            detectorNames.emplace(i.getDetName(), i.getCoeff());

          auto foundUnfiltered = secondUnfiltered.find(firstIter.getLimbID());
          if (foundUnfiltered != secondUnfiltered.end())
          {
            auto bFound = false;
            LimbLabel foundLabel;
            for (const auto &l : foundUnfiltered->second)
            {
              if (l.getLimbID() == firstIter.getLimbID() && l.getPolygon() == firstIter.getPolygon())
              {
                bFound = true;
                foundLabel = l;
                break;
              }
            }
            if (bFound)
            {
              for (const auto &i : foundLabel.getScores())
              {
                newLabelScores.push_back(i);
                detectorNames.emplace(i.getDetName(), i.getCoeff());
              }
            }
          }
          newLabel.setScores(newLabelScores);
          partResult.push_back(newLabel);
        }
      }

      //now iterate through the second list, and push back any labels that are not found in result vector
      try
      {
        auto s = second.at(part.first);

        for (const auto &secondIter : s)
        {
          auto isFound = false;
          for (const auto &resIter : partResult)
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
            for (const auto &i : newLabelScores)
              detectorNames.emplace(i.getDetName(), i.getCoeff());
            newLabel.setScores(newLabelScores);
            partResult.push_back(newLabel);
          }
        }
      }
      catch (...)
      {
      }

      result.insert(std::pair <uint32_t, std::vector <LimbLabel>>(part.first, partResult));
    }

    //now the vectors are merged, but there may be score mismatches
    //unique names of detectors along with their params that are present are stored in detectorNames (duplicates for same detector are ignored)

    //look at each label, and add a score of 1.0 for each missing

    for (auto &r : result)
    {
      for (auto &l : r.second) //for each label
      {
        auto scores = l.getScores();

        for (const auto &m : detectorNames) //for each detector, check whether label has a score for it
        {
          auto detFound = false;

          for (auto &i : scores)
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
    for (auto &l : result)
    {
      sort(l.second.begin(), l.second.end());
    }

    return result;
  }

  LimbLabel Detector::generateLabel(const BodyPart &bodyPart, const cv::Point2f &j0, const cv::Point2f &j1, const std::string &detectorName, float _usedet)
  {
    auto boxCenter = j0 * 0.5 + j1 * 0.5;
    auto rot = static_cast<float>(spelHelper::angle2D(1.0f, 0.0f, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI));
    auto rect = getBodyPartRect(bodyPart, j0, j1);

    auto score = compare();
    std::vector <Score> s;
    s.push_back(Score(score, detectorName, _usedet));
    return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s, score == -1.0f);
  }

  Frame *Detector::getFrame(uint32_t frameId) noexcept
  {
    for (auto f : frames)
      if (f->getID() == frameId)
        return f;

    return nullptr;
  }

  std::map <uint32_t, std::vector <LimbLabel>> Detector::detect(const Frame *frame, std::map <std::string, float> params, const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels)
  {
    // first we need to check all used params
    params.emplace(DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_COEFFICIENT());
    params.emplace(DETECTOR_DETECT_PARAMETERS::MIN_THETA());
    params.emplace(DETECTOR_DETECT_PARAMETERS::MAX_THETA());
    params.emplace(DETECTOR_DETECT_PARAMETERS::STEP_THETA());
    params.emplace(DETECTOR_DETECT_PARAMETERS::UNIQUE_LOCATION_CANDIDATES_COEFFICIENT());
    params.emplace(DETECTOR_DETECT_PARAMETERS::UNIQUE_ANGLE_CANDIDATES_COEFFICIENT());
    params.emplace(DETECTOR_DETECT_PARAMETERS::SCALE_COEFFICIENT());
    params.emplace(DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_MULT_COEFFICIENT());

    params.emplace(COMMON_SPEL_PARAMETERS::DEBUG_LEVEL());
    params.emplace(DETECTOR_DETECT_PARAMETERS::ROTATION_THRESHOLD());
    params.emplace(DETECTOR_DETECT_PARAMETERS::IS_WEAK_THRESHOLD());
    params.emplace(DETECTOR_DETECT_PARAMETERS::SEARCH_STEP_COEFFICIENT());

    //now set actual param values
    auto searchDistCoeff = params.at(DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_COEFFICIENT().first);
    auto minTheta = params.at(DETECTOR_DETECT_PARAMETERS::MIN_THETA().first);
    auto maxTheta = params.at(DETECTOR_DETECT_PARAMETERS::MAX_THETA().first);
    auto stepTheta = params.at(DETECTOR_DETECT_PARAMETERS::STEP_THETA().first);
    auto uniqueLocationCandidates = params.at(DETECTOR_DETECT_PARAMETERS::UNIQUE_LOCATION_CANDIDATES_COEFFICIENT().first);
    auto uniqueAngleCandidates = params.at(DETECTOR_DETECT_PARAMETERS::UNIQUE_ANGLE_CANDIDATES_COEFFICIENT().first);
    auto scaleParam = params.at(DETECTOR_DETECT_PARAMETERS::SCALE_COEFFICIENT().first);
    auto searchDistCoeffMult = params.at(DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_MULT_COEFFICIENT().first);
    auto rotationThreshold = params.at(DETECTOR_DETECT_PARAMETERS::ROTATION_THRESHOLD().first);
    auto isWeakThreshold = params.at(DETECTOR_DETECT_PARAMETERS::IS_WEAK_THRESHOLD().first);
    auto searchStepCoeff = params.at(DETECTOR_DETECT_PARAMETERS::SEARCH_STEP_COEFFICIENT().first);
    debugLevelParam = static_cast <uint8_t> (params.at(COMMON_SPEL_PARAMETERS::DEBUG_LEVEL().first));

    auto originalSize = frame->getFrameSize().height;

    Frame *workFrame = nullptr;
    if (frame->getFrametype() == KEYFRAME)
      workFrame = new Keyframe();
    else if (frame->getFrametype() == LOCKFRAME)
      workFrame = new Lockframe();
    else if (frame->getFrametype() == INTERPOLATIONFRAME)
      workFrame = new Interpolation();

    if (workFrame == nullptr)
    {
      std::stringstream ss;
      ss << "Unknown frame found";
      if (debugLevelParam >= 1)
        std::cerr << ERROR_HEADER << ss.str() << std::endl;
      throw std::logic_error(ss.str());
    }

    workFrame = frame->clone(workFrame);

    auto resizeFactor = workFrame->Resize(maxFrameHeight);

    std::map <uint32_t, std::vector <LimbLabel> > tempLabelVector;
    auto skeleton = workFrame->getSkeleton(); // copy skeleton from the frame
    auto partTree = skeleton.getPartTree(); // copy tree of bodypart from the skeleton

    auto maskMat = workFrame->getMask(); // copy mask from the frame

    std::stringstream detectorName;
    detectorName << getID();

    std::map <uint32_t, std::vector <LimbLabel>> sortedLabelsMap;

    // For all body parts
    for (const auto &iteratorBodyPart : partTree)
    { //Temporary variables
      std::vector <LimbLabel> labels;
      std::vector <LimbLabel> sortedLabels;
      cv::Point2f j0, j1;

      try
      {
        j0 = skeleton.getBodyJoint(iteratorBodyPart.getParentJoint())->getImageLocation(); // copy current bodypart parent joint
        j1 = skeleton.getBodyJoint(iteratorBodyPart.getChildJoint())->getImageLocation(); // copy current bodypart child joint
      }
      catch (...)
      {
        std::stringstream ss;
        ss << "Can't get joints";
        if (debugLevelParam >= 1)
          std::cerr << ERROR_HEADER << ss.str() << std::endl;
        throw std::logic_error(ss.str());
      }

      auto boneLength = getBoneLength(j0, j1); // distance between nodes
      auto boxWidth = getBoneWidth(boneLength, iteratorBodyPart); // current body part polygon width
      auto direction = j1 - j0; // direction of bodypart vector
      auto theta = static_cast<float>(spelHelper::angle2D(1.0f, 0.0f, direction.x, direction.y) * (180.0 / M_PI));  // bodypart tilt angle 
      auto minDist = boxWidth * searchStepCoeff; // linear step of searching
      if (minDist < 2) 
        minDist = 2; // the minimal linear step

      auto searchDistance = iteratorBodyPart.getSearchRadius();
      if (searchDistance <= 0)
        searchDistance = boneLength * searchDistCoeff; // the limiting of search area

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
            auto mintensity = 0;
            try
            {
              mintensity = maskMat.at<uint8_t>(static_cast<int>(y), static_cast<int>(x)); // copy mask at current pixel
            }
            catch (...)
            {
              std::stringstream ss;
              ss << "Can't get value in maskMat at " << "[" << (int)y << "][" << (int)x << "]";
              if (debugLevelParam >= 1)
                std::cerr << ERROR_HEADER << ss.str() << std::endl;
              throw std::logic_error(ss.str());
            }
            auto blackPixel = mintensity < 10; // pixel is not significant if the mask value is less than this threshold
            if (!blackPixel)
            { // Scan the possible rotation zone
              auto deltaTheta = abs(iteratorBodyPart.getRotationSearchRange());
              auto maxLocalTheta = iteratorBodyPart.getRotationSearchRange() == 0 ? maxTheta : deltaTheta;
              auto minLocalTheta = iteratorBodyPart.getRotationSearchRange() == 0 ? minTheta : deltaTheta;
              // build  the vector label
              for (auto rot = theta - minLocalTheta; rot < theta + maxLocalTheta; rot += stepTheta)
                sortedLabels.push_back(generateLabel(boneLength, rot, x, y, iteratorBodyPart, workFrame)); // add label to current bodypart labels
            }
          }
        }
      }

      // build  the vector label
      if (sortedLabels.size() == 0) // if labels for current body part is not builded
        for (auto rot = theta - minTheta; (rot < theta + maxTheta || (rot == theta - minTheta && rot >= theta + maxTheta)); rot += stepTheta)
          sortedLabels.push_back(generateLabel(boneLength, rot, suggestStart.x, suggestStart.y, iteratorBodyPart, workFrame)); // add label to current bodypart labels

      if (sortedLabels.size() > 0) // if labels vector is not empty
        labels = filterLimbLabels(sortedLabels, uniqueLocationCandidates, uniqueAngleCandidates);

      spelHelper::RecalculateScoreIsWeak(labels, detectorName.str(), isWeakThreshold);
      if (labels.size() > 0)
        tempLabelVector.insert(std::pair<uint32_t, std::vector <LimbLabel>>(iteratorBodyPart.getPartID(), labels)); // add current point labels
      sortedLabelsMap.insert(std::pair <uint32_t, std::vector <LimbLabel>>(iteratorBodyPart.getPartID(), sortedLabels));
    }

    delete workFrame;

    for (auto &i : tempLabelVector)
      for (auto &label : i.second)
        label.Resize(pow(resizeFactor, -1));

    return merge(limbLabels, tempLabelVector, sortedLabelsMap);
  }

  LimbLabel Detector::generateLabel(float boneLength, float rotationAngle, float x, float y, BodyPart bodyPart, Frame *workFrame)
  {
    // Create a new label vector and build it label
    auto p0 = cv::Point2f(0.0f, 0.0f); // the point of unit vector
    auto p1 = cv::Point2f(1.0f, 0.0f); // the point of unit vector
    p1 *= boneLength; // change the vector length 
    p1 = spelHelper::rotatePoint2D(p1, p0, rotationAngle); // rotate the vector
    auto mid = 0.5f * p1; // center of the vector
    p1 = p1 + cv::Point2f(x, y) - mid; // shift the vector to current point
    p0 = cv::Point2f(x, y) - mid; // shift the vector to current point

    return generateLabel(bodyPart, workFrame, p0, p1); // build  the vector label
  }

  std::vector<LimbLabel> Detector::filterLimbLabels(std::vector <LimbLabel> &sortedLabels, float uniqueLocationCandidates, float uniqueAngleCandidates)
  {
    if (uniqueLocationCandidates < 0.0f || uniqueLocationCandidates > 1.0f || uniqueAngleCandidates < 0.0f || uniqueAngleCandidates > 1.0f)
      return sortedLabels;

    std::map<float, std::map<float, std::vector<uint32_t> > > locationMap;
    std::map<float, std::vector<uint32_t> > angleMap;

    sort(sortedLabels.begin(), sortedLabels.end()); // sort labels by "SumScore" ?

    for (auto index = 0; index < sortedLabels.size(); ++index)
    {
      auto location = sortedLabels[index].getCenter();
      auto angle = sortedLabels[index].getAngle();
      if (locationMap.find(location.x) == locationMap.end()) //not found
      {
        std::map<float, std::vector<uint32_t> > yMap; //create the new map
        std::vector<uint32_t> indices; //create the index vector
        indices.push_back(index); //push the index
        yMap.emplace(location.y, indices); //put in yMap
        locationMap.emplace(location.x, yMap); //push the  yMap
      }
      else //found x coordinate
      {
        auto yMap = locationMap.at(location.x);
        if (yMap.find(location.y) == yMap.end()) //not found
        {
          std::vector<uint32_t> indices; //read the existing vector
          indices.push_back(index); //push back another index
          yMap.emplace(location.y, indices); //set in map
        }
        else
        {
          std::vector<uint32_t> indices;
          indices.push_back(index);
          yMap.at(location.y) = indices;
        }
        //update the location map
        locationMap.at(location.x) = yMap;
      }
      if (angleMap.find(angle) == angleMap.end()) //not found
      {
        std::vector<uint32_t> indices;
        indices.push_back(index);
        angleMap.emplace(angle, indices);
      }
      else //found
      {
        auto indices = angleMap.at(angle); //read the existing vector
        indices.push_back(index); //push back another index
        angleMap.at(angle) = indices; //set in map
      }
    }

    std::vector<uint32_t> bestByLocation;
    std::vector<uint32_t> bestByAngle;

    for (const auto &yMap : locationMap) //take the top from every location
    {
      for (const auto &yiter : yMap.second) //take the top from every location
      {
        auto indices = yiter.second;
        uint32_t numToPush = indices.size() * uniqueLocationCandidates;
        if (numToPush < 1) 
          numToPush = 1;
        for (auto i = 0; i < numToPush; ++i)
          bestByLocation.push_back(indices[i]);
      }
    }

    for (const auto &iter : angleMap) //take the top from every angle
    {
      auto indices = iter.second;
      auto numToPush = indices.size() * uniqueAngleCandidates;
      if (numToPush < 1) 
        numToPush = 1;
      for (auto i = 0; i < numToPush; ++i)
        bestByAngle.push_back(indices[i]);
    }

    //now intersect
    sort(bestByLocation.begin(), bestByLocation.end());
    sort(bestByAngle.begin(), bestByAngle.end());

    std::vector<uint32_t> bestIntersect;
    set_union(bestByLocation.begin(), bestByLocation.end(), bestByAngle.begin(), bestByAngle.end(), back_inserter(bestIntersect));

    std::vector<LimbLabel> labels;
    for (auto index : bestIntersect)
      labels.push_back(sortedLabels.at(index));

    return labels;
  }

}
