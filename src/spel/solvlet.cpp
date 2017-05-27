// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com
#include "solvlet.hpp"
#include "solver.hpp"
#include "spelParameters.hpp"
namespace SPEL
{

  Solvlet::Solvlet(void)
    : Solvlet(-1, std::vector<LimbLabel>())
  {
  }

  Solvlet::Solvlet(const int id, const std::vector<LimbLabel> &labels)
    : m_frameId(id), m_labels(labels)
  {
  }

  Solvlet::Solvlet(const Solvlet & s)
    : m_frameId(s.m_frameId), m_labels(s.m_labels)
  {
  }

  Solvlet::Solvlet(Solvlet && s)
    : m_frameId(std::move(s.m_frameId)),
    m_labels(std::move(s.m_labels))
  {
  }

  Solvlet::~Solvlet(void)
  {
  }

  Solvlet &Solvlet::operator=(const Solvlet &s)
  {
    if (this == &s)
      return *this;
    m_labels = s.m_labels;
    m_frameId = s.m_frameId;
    return *this;
  }

  Solvlet & Solvlet::operator=(Solvlet && s)
  {
    m_frameId = std::move(s.m_frameId);
    std::swap(m_labels, s.m_labels);

    return *this;
  }

  bool Solvlet::operator<(const Solvlet &s) const
  {
    return m_frameId < s.m_frameId;
  }

  bool Solvlet::operator>(const Solvlet &s) const
  {
    return m_frameId > s.m_frameId;
  }

  int Solvlet::getFrameID(void) const
  {
    return m_frameId;
  }

  void Solvlet::setFrameID(int _id)
  {
    m_frameId = _id;
  }

  std::vector<LimbLabel> Solvlet::getLabels(void) const
  {
    return m_labels;
  }

  const std::vector<LimbLabel>* Solvlet::getLabelsPtr() const {
    return &m_labels;
  }

  void Solvlet::setLabels(const std::vector<LimbLabel> &labels)
  {
    m_labels = labels;
  }

  Skeleton Solvlet::toSkeleton(const Skeleton &example) const
  {
    auto retSkel = example;

    const auto &partTree = retSkel.getPartTree();
    const auto &jointTree = retSkel.getJointTree();

    if (partTree.size() != m_labels.size())
    {
      const std::string str = "Incorrect skeleton size.";
      DebugMessage(str, 1);
      throw std::logic_error(str);
    }
    for (const auto &part : partTree)
    {
      for (const auto &label : m_labels)
      {
        if (part.getPartID() == label.getLimbID()) //if you find the right label
        {
          //get joint IDs
          const auto cJointID = part.getChildJoint(); //child joint
          const auto pJointID = part.getParentJoint(); //parent joint

          auto cjfound = false, pjfound = false;
          BodyJoint pj, cj;
          //identify these nodes in the joint tree
          for (const auto &joint : jointTree)
          {
            if (joint.getLimbID() == cJointID)
            {
              cj = joint;
              cjfound = true;
            }
            else if (joint.getLimbID() == pJointID)
            {
              pj = joint;
              pjfound = true;
            }
            if (cjfound && pjfound)
              break;
          }
          cv::Point2f pjep, cjep; //parent and child joints from label
          label.getEndpoints(pjep, cjep); //set them from label
          //set parent joint only for root node
          //set child joint for all other nodes
          part.getPartID() == 0 ? pj.setImageLocation(pjep) : 
            cj.setImageLocation(cjep);
        } //TODO: introduce a more complex scheme of doing this, such as finding midpoints, or points
      }
    }

    retSkel.setJointTree(jointTree);
    retSkel.infer3D();

    return retSkel;
  }

  float Solvlet::evaluateSolution(Frame* frame, const std::map<std::string, float> &params)
  {
    // /*
    //   There should clearly be several factors that affect the outcome of an evaluation:
    //   1) Mask coverage
    //   2) Parts falling outside mask range
    //   3) ?

    //   */

    //engaged pixles - the total number of pixels that are either within a limb label of within the mask
    //correct pixels - those pixles that are black and outside a label, and white and inside a label
    //incorrect pixels - those pixels that are black and inside a label, and white and outside a label
    //score = correct/(correct+incorrect)

    auto workFrame = frame->clone(new Frame());
    const auto factor = workFrame->Resize(static_cast<uint32_t>(params.at(
      COMMON_SPEL_PARAMETERS::MAX_FRAME_HEIGHT().name())));

    for (auto &label : m_labels)
      label.Resize(factor);

    auto correctPixels = 0U, incorrectPixels = 0U;

    const auto mask = workFrame->getMask();

    for (auto i = 0; i < mask.cols; ++i) //at every col - x
    {
      for (auto j = 0; j < mask.rows; ++j) //and every row - y
      {
        //check whether pixel hit a label from solution
        auto labelHit = false;
        for (const auto &label : m_labels)
        {
          if (label.containsPoint(cv::Point2f(static_cast<float>(i), static_cast<float>(j)))) //this is done in x,y coords
          {
            labelHit = true;
            break;
          }
        }

        //check pixel colour
        const auto blackPixel = (mask.at<uchar>(j, i) < 10);

        if (blackPixel && labelHit) //if black in label, incorrect
          ++incorrectPixels;
        else if (!blackPixel && !labelHit) //if white not in label, incorret
          ++incorrectPixels;
        else if (!blackPixel && labelHit)//otherwise correct
          ++correctPixels;
      }
    }
    
    auto solutionEval = static_cast<float>(correctPixels) / (correctPixels + incorrectPixels);

    //now check for critical part failures - label mostly outside of mask
    std::vector<std::pair<int, float>> badLabelScores;
    const auto badLabelThresh = params.at(COMMON_SOLVER_PARAMETERS::BAD_LABEL_THRESH().name());

    for (const auto &label : m_labels)
    {
      const auto &poly = label.getPolygon(); //get the label polygon
      //compute min and max x and y
      std::vector<float> xS = { poly[0].x, poly[1].x, poly[2].x, poly[3].x };
      std::vector<float> yS = { poly[0].y, poly[1].y, poly[2].y, poly[3].y };

      const auto xMin = *(min_element(xS.begin(), xS.end()));
      const auto xMax = *(max_element(xS.begin(), xS.end()));

      const auto yMin = *(min_element(yS.begin(), yS.end()));
      const auto yMax = *(max_element(yS.begin(), yS.end()));

      auto labelPixels = 0U;
      auto badLabelPixels = 0U;

      for (auto x = xMin; x < xMax; ++x)
      {
        for (auto y = yMin; y < yMax; ++y)
        {
          if (label.containsPoint(cv::Point2f(x, y)))
          {            
            ++labelPixels;
            //this is done with reverse y,x
            if (mask.at<uchar>(static_cast<int>(y), static_cast<int>(x)) < 10)
              ++badLabelPixels;
          }
        }
      }

      const auto labelRatio = 1.0f - static_cast<float>(badLabelPixels) / labelPixels; //high is good

      if (labelRatio < badLabelThresh && !label.getIsOccluded()) //not weak, not occluded, badly localised
        badLabelScores.push_back(std::make_pair(label.getLimbID(), labelRatio));
    }

    if (SpelObject::getDebugLevel() >= 1)
    {
      for (const auto &badL : badLabelScores)
      {
        std::stringstream ss;
        ss << "Part " << badL.first << " is badly localised, with score " << badL.second;
        DebugMessage(ss.str(), 1);
      }
    }

    if (badLabelScores.size() != 0) //make the solution eval fail if a part is badly localised
      solutionEval -= 1.0f;

    if (SpelObject::getDebugLevel() >= 1)
    {
      std::stringstream ss;
      ss << "Solution evaluation score - " << solutionEval << " for frame " << frame->getID() << " solve from " << frame->getParentFrameID();
      DebugMessage(ss.str(), 1);
    }

    delete workFrame;

    return solutionEval;
  }  
}
