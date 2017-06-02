// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com
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
#include "spelGeometry.hpp"

namespace SPEL
{
  Detector::Detector(void) : m_id(0U), maxFrameHeight(0U)
  {
  }

  Detector::~Detector(void)
  {
  }

  int Detector::getID(void) const
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

      const auto &s = second.find(part.first);
      if (s != second.end())
      {
        for (const auto &secondIter : (*s).second)
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
    std::string &detectorName, const float coeff,
    const std::function<float()> &compare) const
  {
    const auto &boxCenter = parent * 0.5 + child * 0.5;
    const auto rot = spelHelper::getAngle(parent, child);
    const auto &rect = bodyPart.getBodyPartRect(parent, child);

    const auto score = compare();
    std::vector <Score> s;
    s.push_back(Score(score, detectorName, coeff));
    bool isOccluded = spelHelper::compareFloat(score, -1.0f) == 0;
    return LimbLabel(bodyPart.getPartID(), boxCenter, rot, rect.asVector(), s,
      isOccluded);
  }

  Frame *Detector::getFrame(const int32_t frameId) const
  {
    for (auto f : m_frames)
      if (f->getID() == frameId)
        return f;

    return nullptr;
  }

  std::map <uint32_t, std::vector <LimbLabel>> Detector::detect(
    Frame *frame, std::map <std::string, float> params,
    std::map <uint32_t, std::vector <LimbLabel>> &limbLabels,
    DetectorHelper *detectorHelper) const
  {
    // first we need to check all used params
    emplaceDefaultParameters(params);

    auto workFrame = frame->clone(new Interpolation());

    auto resizeFactor = workFrame->Resize(maxFrameHeight);

    if (limbLabels.empty())
      limbLabels = generateLimbLabels(workFrame, params);

    for (auto &labels : limbLabels)
    {
      for (auto &label : labels.second)
      {
        // we need to adjust LimbLabel size so we need to resize it twice:
        // at first before calculating score, then after
        if(resizeFactor > 0)
          label.Resize(resizeFactor);
        calculateLabelScore(workFrame, detectorHelper, label, params);
        if (resizeFactor > 0)
          label.Resize(pow(resizeFactor, -1));
      }
    }

    delete workFrame;

    return limbLabels;
  }

  LimbLabel Detector::generateLabel(const float boneLength,
    const float rotationAngle, const float x, const float y,
    const BodyPart &bodyPart, Frame *workFrame,
    DetectorHelper *detectorHelper, std::map <std::string, float> params)
    const
  {
    emplaceDefaultParameters(params);
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

  void Detector::addLabelScore(LimbLabel & label,
    const std::string & detectorName, const float coeff,
    const std::function<float()>& compare) const
  {
    label.addScore(Score(compare(), detectorName, coeff));
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

  void Detector::train(const std::vector<Frame*>& frames,
    std::map<std::string, float> params,
    const std::function<void(Frame*, const float)>& handler)
  {
    emplaceDefaultParameters(params);
    if (frames.size() == 0)
    {
      const std::string str = "No input frames";
      DebugMessage(str, 1);
      throw std::logic_error(str); // the sequence of frames is empty
    }

    // vector of pointers - presents a sequence of frames
    m_frames = frames;
    // sorting frames by id
    sort(m_frames.begin(), m_frames.end(), Frame::FramePointerComparer);

    params.at(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().name()) =
      std::min(params.at(
        COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().name()),
        static_cast<float>(m_frames.front()->getFrameSize().height));

    maxFrameHeight = static_cast<uint32_t>(
      params.at(COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().name()));

    // Handling all frames
    for (auto &frame : m_frames)
    {
      if (frame->getFrametype() != KEYFRAME &&
        frame->getFrametype() != LOCKFRAME)
        continue;

      const auto scale = frame->Resize(maxFrameHeight);

      if (SpelObject::getDebugLevel() >= 2)
        std::cout << "Training on frame " << frame->getID() << std::endl;

      handler(frame, scale);

      try
      {
        frame->UnloadAll();
      }
      catch (const std::exception &ex)
      {
        DebugMessage(ex.what(), 1);
      }
    }
  }

  void Detector::emplaceDefaultParameters(
    std::map<std::string, float>& params) const
  {
    spelHelper::mergeParameters(params,
      COMMON_SPEL_PARAMETERS::getParameters());
    spelHelper::mergeParameters(params,
      COMMON_DETECTOR_PARAMETERS::getParameters());
    spelHelper::mergeParameters(params,
      DETECTOR_DETECT_PARAMETERS::getParameters());
  }

  DetectorHelper::DetectorHelper(void)
  {
  }

  DetectorHelper::~DetectorHelper(void)
  {
  }

}
