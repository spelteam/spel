// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com
#include "solvlet.hpp"

namespace SPEL
{

  Solvlet::Solvlet(void)
  {
    frameId = -1;
  }
  Solvlet::Solvlet(int id, std::vector<LimbLabel> _labels)
  {
    frameId = id;
    labels = _labels;
  }

  Solvlet::~Solvlet(void)
  {
  }

  Solvlet &Solvlet:: operator=(const Solvlet &s)
  {
    if (this == &s)
    {
      return *this;
    }
    this->setLabels(s.getLabels());
    this->setFrameID(s.getFrameID());
    return *this;
  }

  bool Solvlet::operator<(const Solvlet &s) const
  {
    return this->getFrameID() < s.getFrameID();
  }

  bool Solvlet::operator>(const Solvlet &s) const
  {
    return this->getFrameID()>s.getFrameID();
  }

  int Solvlet::getFrameID(void) const
  {
    return frameId;
  }

  void Solvlet::setFrameID(int _id)
  {
    frameId = _id;
  }

  std::vector<LimbLabel> Solvlet::getLabels(void) const
  {
    return labels;
  }

  const std::vector<LimbLabel>* Solvlet::getLabelsPtr() const {
    return &labels;
  }

  void Solvlet::setLabels(std::vector<LimbLabel> _labels)
  {
    labels = _labels;
  }

  Skeleton Solvlet::toSkeleton(const Skeleton &example) const
  {
    Skeleton retSkel = example;

    tree<BodyPart> partTree = retSkel.getPartTree();
    tree<BodyJoint> jointTree = retSkel.getJointTree();
    tree<BodyJoint>::iterator cjIter, pjIter, jointIter;

    assert(partTree.size() == labels.size()); //there should be the same number of body parts

    for (tree<BodyPart>::iterator partIter = partTree.begin(); partIter != partTree.end(); ++partIter)
    {
      for (auto i = 0; i < labels.size(); ++i)
      {
        if (partIter->getPartID() == labels[i].getLimbID()) //if you find the right label
        {
          //get joint IDs
          int partID = partIter->getPartID(); //part
          int cJointID = partIter->getChildJoint(); //child joint
          int pJointID = partIter->getParentJoint(); //parent joint

          cv::Point2f pj, cj; //parent and child joints from label
          labels[i].getEndpoints(pj, cj); //set them from label


          cjIter = jointTree.end();
          pjIter = jointTree.end();

          //identify these nodes in the joint tree
          for (jointIter = jointTree.begin(); jointIter != jointTree.end(); ++jointIter)
          {
            if (jointIter->getLimbID() == cJointID)
              cjIter = jointIter;
            if (jointIter->getLimbID() == pJointID)
              pjIter = jointIter;
            if (cjIter != jointTree.end() && pjIter != jointTree.end())
              break;
          }

          if (partID == 0) //root
          {
            //set 2D joint locations
            pjIter->setImageLocation(pj); //set parent joint only for root node
          }
          cjIter->setImageLocation(cj); //set child joint for all other nodes
        } //TODO: introduce a more complex scheme of doing this, such as finding midpoints, or points
      }
    }

    retSkel.setJointTree(jointTree);
    retSkel.infer3D();

    return retSkel;
  }

  float Solvlet::evaluateSolution(Frame* frame, std::map<std::string, float> params)
  {
    std::vector<LimbLabel> labels = this->getLabels();
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

    //emplace defaults
    params.emplace("badLabelThresh", 0.52); //if less than 52% of the pixels are in the mask, label this label bad
    params.emplace("debugLevel", 1);
    params.emplace("maxFrameHeight", 288);  //emplace if not defined

    int maxFrameHeight = params.at("maxFrameHeight");
    int debugLevel = params.at("debugLevel");

    cv::Mat mask = frame->getMask().clone();

    float factor = 1;
    //compute the scaling factor
    if (maxFrameHeight != 0)
    {
      factor = (float)maxFrameHeight / (float)mask.rows;

      resize(mask, mask, cvSize(mask.cols * factor, mask.rows * factor));
    }
    for (std::vector<LimbLabel>::iterator label = labels.begin(); label != labels.end(); ++label)
    {
      label->Resize(factor);
    }

    int correctPixels = 0, incorrectPixels = 0;
    int pixelsInMask = 0;
    int coveredPixelsInMask = 0;
    int incorrectlyCoveredPixels = 0;
    int missedPixels = 0;

    for (int i = 0; i < mask.cols; ++i) //at every col - x
    {
      for (int j = 0; j < mask.rows; ++j) //and every row - y
      {
        //int test = labels[0].containsPoint(Point2f(480,100));
        //check whether pixel hit a label from solution
        bool labelHit = false;
        for (std::vector<LimbLabel>::iterator label = labels.begin(); label != labels.end(); ++label)
        {
          if (label->containsPoint(cv::Point2f(i, j))) //this is done in x,y coords
          {
            labelHit = true;
            //break;
          }
        }

        //check pixel colour
        int intensity = mask.at<uchar>(j, i); //this is done with reve
        bool blackPixel = (intensity < 10);

        if (!blackPixel)
          pixelsInMask++;

        if (blackPixel && labelHit) //if black in label, incorrect
        {
          incorrectPixels++;
          incorrectlyCoveredPixels++;
        }
        else if (!blackPixel && !labelHit) //if white not in label, incorret
        {
          incorrectPixels++;
          missedPixels++;
        }
        else if (!blackPixel && labelHit)//otherwise correct
        {
          correctPixels++;
          coveredPixelsInMask++;
        }
        //            else //black pixel and not label hit
        //                correctPixels++; //don't count these at all?
      }
    }


    double solutionEval = (float)correctPixels / ((float)correctPixels + (float)incorrectPixels);

    //now check for critical part failures - label mostly outside of mask

    std::vector<cv::Point2f> badLabelScores;
    float badLabelThresh = params.at("badLabelThresh");

    for (std::vector<LimbLabel>::iterator label = labels.begin(); label != labels.end(); ++label)
    {
      std::vector<cv::Point2f> poly = label->getPolygon(); //get the label polygon
        //compute min and max x and y
        //float xMin, xMax, yMin, yMax;
      std::vector<float> xS = { poly[0].x, poly[1].x, poly[2].x, poly[3].x };
      std::vector<float> yS = { poly[0].y, poly[1].y, poly[2].y, poly[3].y };
      auto xMin = min_element(xS.begin(), xS.end());
      auto xMax = max_element(xS.begin(), xS.end());

      auto yMin = min_element(yS.begin(), yS.end());
      auto yMax = max_element(yS.begin(), yS.end());

      int labelPixels = 0;
      int badLabelPixels = 0;

      for (int x = *xMin; x < *xMax; ++x)
      {
        for (int y = *yMin; y < *yMax; ++y)
        {
          if (label->containsPoint(cv::Point2f(x, y)))
          {
            int intensity = mask.at<uchar>(y, x); //this is done with reverse y,x
            bool blackPixel = (intensity < 10);
            labelPixels++;
            if (blackPixel)
              ++badLabelPixels;
          }
        }
      }

      float labelRatio = 1.0 - (float)badLabelPixels / (float)labelPixels; //high is good

      if (labelRatio < badLabelThresh /*&& !label->getIsWeak()*/ && !label->getIsOccluded()) //not weak, not occluded, badly localised
        badLabelScores.push_back(cv::Point2f(label->getLimbID(), labelRatio));
    }

    if (debugLevel >= 1)
    {
      for (std::vector<cv::Point2f>::iterator badL = badLabelScores.begin(); badL != badLabelScores.end(); ++badL)
      {
        std::cerr << "Part " << badL->x << " is badly localised, with score " << badL->y << std::endl;
      }
    }

    if (badLabelScores.size() != 0) //make the solution eval fail if a part is badly localised
      solutionEval = solutionEval - 1.0;

    if (debugLevel >= 1)
      std::cerr << "Solution evaluation score - " << solutionEval << " for frame " << frame->getID() << " solve from " << frame->getParentFrameID() << std::endl;

    frame->UnloadAll();

    return solutionEval;
  }
  
}
