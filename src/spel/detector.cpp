// STL
#ifdef WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif  // WINDOWS
#include <exception>

#include "detector.hpp"
#include "spelHelper.hpp"
#include "sequence.hpp"
#include "keyframe.hpp"
#include "lockframe.hpp"
#include "interpolation.hpp"
#include "spelParameters.hpp"
#include "spelObject.hpp"

namespace SPEL
{
  Detector::Detector(void) noexcept
  {
    m_id = 0x00000000;
  }

  Detector::~Detector(void) noexcept
  {
  }

  int Detector::getID(void) const noexcept
  {
    return m_id;
  }

  void Detector::setID(const int id)
  {
    m_id = id;
  }

  std::map <uint32_t, std::vector <LimbLabel>> Detector::merge(
    const std::map <uint32_t, std::vector <LimbLabel>> &first, 
    const std::map <uint32_t, std::vector <LimbLabel>> &second, 
    const std::map <uint32_t, std::vector <LimbLabel>> &secondUnfiltered) 
    const
  {
    if (first.size() != second.size() && first.size() > 0 && second.size() > 0)
    {
      std::stringstream ss;
      ss << "Can't merge vectors with different sizes " << 
        "(different count of BodyPart): First: " << first.size() << 
        " Second: " << second.size();
      DebugMessage(ss.str(), 1);
      throw std::logic_error(ss.str());
    }
    if (first.size() == 0)
      return second;
    if (second.size() == 0)
      return first;

    std::map <std::string, float> detectorNames;

    std::map <uint32_t, std::vector <LimbLabel>> result;

    for (const auto &part : first) //for each part
    {
      std::vector<LimbLabel> partResult;

      // iterate through first list, compare to second list, 
      // any labels that are matched are combined and pushed
      // any labels that are not found, are added
      for (const auto &firstIter : part.second) //for each label in first
      {
        auto isFound = false;

        LimbLabel foundLabel;
        try
        {
          const auto &s = second.at(part.first);
          for (const auto &secondIter : s)
          {
            if (firstIter.getLimbID() == secondIter.getLimbID() && 
              firstIter.getPolygon() == secondIter.getPolygon())
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
        // if label was found, create a copy, 
        // and add a score from other label to it
        if (isFound)
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
            for (const auto &l : foundUnfiltered->second)
            {
              if (l.getLimbID() == firstIter.getLimbID() && 
                l.getPolygon() == firstIter.getPolygon())
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

      // now iterate through the second list, 
      // and push back any labels that are not found in result vector
      try
      {
        const auto &s = second.at(part.first);

        for (const auto &secondIter : s)
        {
          auto isFound = false;
          for (const auto &resIter : partResult)
          {
            if (secondIter.getLimbID() == resIter.getLimbID() && 
              secondIter.getPolygon() == resIter.getPolygon())
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

      result.insert(std::pair <uint32_t, std::vector <LimbLabel>>(part.first, 
        partResult));
    }

    // now the vectors are merged, but there may be score mismatches
    // unique names of detectors along with their params that are present are 
    // stored in detectorNames (duplicates for same detector are ignored)

    // look at each label, and add a score of 1.0 for each missing

    for (auto &r : result)
    {
      for (auto &l : r.second) //for each label
      {
        auto scores = l.getScores();
        //for each detector, check whether label has a score for it
        for (const auto &m : detectorNames) 
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

  LimbLabel Detector::generateLabel(const BodyPart &bodyPart, 
    const cv::Point2f &parent, const cv::Point2f &child, const 
    std::string &detectorName, const float coeff, std::function<float()> compare) 
    const
  {
    auto boxCenter = parent * 0.5 + child * 0.5;
    auto rot = static_cast<float>(spelHelper::angle2D(1.0f, 0.0f, 
      child.x - parent.x, child.y - parent.y) * (180.0 / M_PI));
    auto rect = bodyPart.getBodyPartRect(parent, child);

    auto score = compare();
    std::vector <Score> s;
    s.push_back(Score(score, detectorName, coeff));
    return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s, 
      score == -1.0f);
  }

  Frame *Detector::getFrame(const int32_t frameId) const noexcept
  {
    for (auto f : frames)
      if (f->getID() == frameId)
        return f;

    return nullptr;
  }

  std::map <uint32_t, std::vector <LimbLabel>> Detector::detect(
    Frame *frame, std::map <std::string, float> params, 
    const std::map <uint32_t, std::vector <LimbLabel>> &limbLabels, 
    DetectorHelper *detectorHelper) const
  {
    // first we need to check all used params
    params.emplace(DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_COEFFICIENT());
    params.emplace(DETECTOR_DETECT_PARAMETERS::MIN_THETA());
    params.emplace(DETECTOR_DETECT_PARAMETERS::MAX_THETA());
    params.emplace(DETECTOR_DETECT_PARAMETERS::STEP_THETA());
    params.emplace(
      DETECTOR_DETECT_PARAMETERS::UNIQUE_LOCATION_CANDIDATES_COEFFICIENT());
    params.emplace(
      DETECTOR_DETECT_PARAMETERS::UNIQUE_ANGLE_CANDIDATES_COEFFICIENT());
    params.emplace(DETECTOR_DETECT_PARAMETERS::SCALE_COEFFICIENT());
    params.emplace(
      DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_MULT_COEFFICIENT());

    params.emplace(DETECTOR_DETECT_PARAMETERS::ROTATION_THRESHOLD());
    params.emplace(DETECTOR_DETECT_PARAMETERS::IS_WEAK_THRESHOLD());
    params.emplace(DETECTOR_DETECT_PARAMETERS::SEARCH_STEP_COEFFICIENT());

    //now set actual param values
    auto searchDistCoeff = params.at(
      DETECTOR_DETECT_PARAMETERS::SEARCH_DISTANCE_COEFFICIENT().first);
    auto minTheta = params.at(DETECTOR_DETECT_PARAMETERS::MIN_THETA().first);
    auto maxTheta = params.at(DETECTOR_DETECT_PARAMETERS::MAX_THETA().first);
    auto stepTheta = params.at(DETECTOR_DETECT_PARAMETERS::STEP_THETA().first);
    auto uniqueLocationCandidates = params.at(
      DETECTOR_DETECT_PARAMETERS::UNIQUE_LOCATION_CANDIDATES_COEFFICIENT().first);
    auto uniqueAngleCandidates = params.at(
      DETECTOR_DETECT_PARAMETERS::UNIQUE_ANGLE_CANDIDATES_COEFFICIENT().first);
    auto isWeakThreshold = params.at(
      DETECTOR_DETECT_PARAMETERS::IS_WEAK_THRESHOLD().first);
    auto searchStepCoeff = params.at(
      DETECTOR_DETECT_PARAMETERS::SEARCH_STEP_COEFFICIENT().first);

    Frame *workFrame = nullptr;
    if (frame->getFrametype() == KEYFRAME)
      workFrame = new Keyframe();
    else if (frame->getFrametype() == LOCKFRAME)
      workFrame = new Lockframe();
    else if (frame->getFrametype() == INTERPOLATIONFRAME)
      workFrame = new Interpolation();

    if (workFrame == nullptr)
    {
      const std::string str = "Unknown frame found";
      DebugMessage(str, 1);
      throw std::logic_error(str);
    }

    workFrame = frame->clone(workFrame);

    auto resizeFactor = workFrame->Resize(maxFrameHeight);

    std::map <uint32_t, std::vector <LimbLabel> > tempLabelVector;
    const auto &skeleton = workFrame->getSkeleton(); // copy skeleton from the frame
    // copy tree of bodypart from the skeleton
    const auto &partTree = skeleton.getPartTree(); 

    const auto &maskMat = workFrame->getMask(); // copy mask from the frame

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
        // copy current bodypart parent joint
        j0 = skeleton.getBodyJoint(
          iteratorBodyPart.getParentJoint())->getImageLocation();
        // copy current bodypart child joint
        j1 = skeleton.getBodyJoint(
          iteratorBodyPart.getChildJoint())->getImageLocation();
      }
      catch (...)
      {
        const std::string str = "Can't get joints";
        DebugMessage(str, 1);
        throw std::out_of_range(str);
      }
      // distance between nodes
      auto boneLength = BodyPart::getBoneLength(j0, j1);
      // current body part polygon width
      auto boxWidth = iteratorBodyPart.getBoneWidth(boneLength);
      auto direction = j1 - j0; // direction of bodypart vector
      auto theta = static_cast<float>(spelHelper::angle2D(1.0f, 0.0f, 
        direction.x, direction.y) * (180.0 / M_PI));  // bodypart tilt angle 
      auto minDist = boxWidth * searchStepCoeff; // linear step of searching
      if (minDist < 2) 
        minDist = 2; // the minimal linear step

      auto searchDistance = iteratorBodyPart.getSearchRadius();
      // the limiting of search area
      if (searchDistance <= 0)
        searchDistance = boneLength * searchDistCoeff; 

      if (searchDistance <= 0)
        searchDistance = minDist + 1;
      // reference point - the bodypart center
      auto suggestStart = 0.5 * j1 + 0.5 * j0; 
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
              mintensity = maskMat.at<uint8_t>(static_cast<int>(y), 
                static_cast<int>(x)); // copy mask at current pixel
            }
            catch (...)
            {
              std::stringstream ss;
              ss << "Can't get value in maskMat at " << "[" << (int)y << 
                "][" << (int)x << "]";
              DebugMessage(ss.str(), 1);
              throw std::out_of_range(ss.str());
            }
            // pixel is not significant if the mask value is 
            // less than this threshold
            auto blackPixel = mintensity < 10; 
            if (!blackPixel)
            { // Scan the possible rotation zone
              auto deltaTheta = std::abs(
                iteratorBodyPart.getRotationSearchRange());
              auto maxLocalTheta = 
                iteratorBodyPart.getRotationSearchRange() == 0 ? 
                maxTheta : deltaTheta;
              auto minLocalTheta = 
                iteratorBodyPart.getRotationSearchRange() == 0 ? 
                minTheta : deltaTheta;
              // build  the vector label
              for (auto rot = theta - minLocalTheta; 
              rot < theta + maxLocalTheta; rot += stepTheta)
                // add label to current bodypart labels
                sortedLabels.push_back(generateLabel(boneLength, rot, x, y, 
                  iteratorBodyPart, workFrame, detectorHelper, params));
            }
          }
        }
      }

      // build  the vector label
      // if labels for current body part is not builded
      if (sortedLabels.size() == 0) 
        for (auto rot = theta - minTheta; (rot < theta + maxTheta || 
          (rot == theta - minTheta && rot >= theta + maxTheta)); 
          rot += stepTheta)
          sortedLabels.push_back(generateLabel(boneLength, rot, 
            suggestStart.x, suggestStart.y, iteratorBodyPart, workFrame, 
            detectorHelper, params)); // add label to current bodypart labels

      if (sortedLabels.size() > 0) // if labels vector is not empty
        labels = filterLimbLabels(sortedLabels, uniqueLocationCandidates, 
          uniqueAngleCandidates);

      spelHelper::RecalculateScoreIsWeak(labels, detectorName.str(), 
        isWeakThreshold);
      if (labels.size() > 0)
        // add current point labels
        tempLabelVector.insert(std::pair<uint32_t, 
          std::vector <LimbLabel>>(iteratorBodyPart.getPartID(), labels));
      sortedLabelsMap.insert(std::pair <uint32_t, 
        std::vector <LimbLabel>>(iteratorBodyPart.getPartID(), sortedLabels));
    }

    delete workFrame;

    if (resizeFactor)
      for (auto &i : tempLabelVector)
        for (auto &label : i.second)
          label.Resize(pow(resizeFactor, -1));

    return merge(limbLabels, tempLabelVector, sortedLabelsMap);
  }

  LimbLabel Detector::generateLabel(const float boneLength, 
    const float rotationAngle, const float x, const float y, 
    const BodyPart &bodyPart, Frame *workFrame, 
    DetectorHelper *detectorHelper, std::map <std::string, float> params) 
    const
  {
    // Create a new label vector and build it label
    auto p0 = cv::Point2f(0.0f, 0.0f); // the point of unit vector
    auto p1 = cv::Point2f(1.0f, 0.0f); // the point of unit vector
    p1 *= boneLength; // change the vector length 
    p1 = spelHelper::rotatePoint2D(p1, p0, rotationAngle); // rotate the vector
    auto mid = 0.5f * p1; // center of the vector
    p1 = p1 + cv::Point2f(x, y) - mid; // shift the vector to current point
    p0 = cv::Point2f(x, y) - mid; // shift the vector to current point
    // build  the vector label
    return generateLabel(bodyPart, workFrame, p0, p1, detectorHelper, params); 
  }

  std::vector<LimbLabel> Detector::filterLimbLabels(
    const std::vector <LimbLabel> &sortedLabels, 
    const float uniqueLocationCandidates, const float uniqueAngleCandidates)
  {
    if (uniqueLocationCandidates < 0.0f || uniqueLocationCandidates > 1.0f || 
      uniqueAngleCandidates < 0.0f || uniqueAngleCandidates > 1.0f)
      return sortedLabels;

    std::map<float, std::map<float, std::vector<uint32_t> > > locationMap;
    std::map<float, std::vector<uint32_t> > angleMap;

    std::vector <LimbLabel> copyLabels;
    copyLabels.reserve(sortedLabels.size());
    copy(sortedLabels.begin(), sortedLabels.end(), back_inserter(copyLabels));

    sort(copyLabels.begin(), copyLabels.end()); // sort labels by "SumScore" ?

    for (auto index = 0; index < copyLabels.size(); ++index)
    {
      auto location = copyLabels.at(index).getCenter();
      auto angle = copyLabels.at(index).getAngle();
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
        auto &yMap = locationMap.at(location.x);
        if (yMap.find(location.y) == yMap.end()) //not found
        {
          std::vector<uint32_t> indices; //read the existing vector
          indices.push_back(index); //push back another index
          yMap.emplace(location.y, indices); //set in map
        }
        else
        {
          auto &indices = yMap.at(location.y);
          indices.push_back(index);
        }
      }
      if (angleMap.find(angle) == angleMap.end()) //not found
      {
        std::vector<uint32_t> indices;
        indices.push_back(index);
        angleMap.emplace(angle, indices);
      }
      else //found
      {
        auto &indices = angleMap.at(angle); //read the existing vector
        indices.push_back(index); //push back another index
      }
    }

    std::vector<uint32_t> bestByLocation;
    std::vector<uint32_t> bestByAngle;

    for (const auto &yMap : locationMap) //take the top from every location
    {
      for (const auto &yiter : yMap.second) //take the top from every location
      {
        auto indices = yiter.second;
        auto numToPush = static_cast<uint32_t>(indices.size() * 
          uniqueLocationCandidates);
        if (numToPush < 1) 
          numToPush = 1;
        for (auto i = 0U; i < numToPush; ++i)
          bestByLocation.push_back(indices.at(i));
      }
    }

    for (const auto &iter : angleMap) //take the top from every angle
    {
      auto indices = iter.second;
      auto numToPush = indices.size() * uniqueAngleCandidates;
      if (numToPush < 1) 
        numToPush = 1;
      for (auto i = 0; i < numToPush; ++i)
        bestByAngle.push_back(indices.at(i));
    }

    //now intersect
    sort(bestByLocation.begin(), bestByLocation.end());
    sort(bestByAngle.begin(), bestByAngle.end());

    std::vector<uint32_t> bestIntersect;
    set_union(bestByLocation.begin(), bestByLocation.end(), 
      bestByAngle.begin(), bestByAngle.end(), back_inserter(bestIntersect));

    std::vector<LimbLabel> labels;
    for (auto index : bestIntersect)
      labels.push_back(copyLabels.at(index));

    return labels;
  }

  DetectorHelper::DetectorHelper(void) noexcept
  {
  }

  DetectorHelper::~DetectorHelper(void) noexcept
  {
  }

}
