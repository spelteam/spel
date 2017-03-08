#ifndef __SOLVER_HPP_
#define __SOLVER_HPP_

// SPEL definitions
#include "predef.hpp"

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

#include "solver.hpp"
#include "frame.hpp"

#include "imagemasksimilaritymatrix.hpp"
#include "imagepixelsimilaritymatrix.hpp"

namespace SPEL
{
  class frameSolver
  {
  public:
    struct JointLink {
      JointLink(int partID_, int isChildJoint);
      int partID;
      bool num; // 0 ==  parrent joint, 1 == child joint
    };
    struct partAdjacentsJoints {
      std::vector<JointLink> ChildJointConnections;
      std::vector<JointLink> ParentJointConnections;
      void clear(void)
      {
        ChildJointConnections.clear();
        ParentJointConnections.clear();
      }
    };
    struct Label {
      std::vector<cv::Point2f> joints;
      float score;
    };

  public:
    frameSolver(Skeleton pattern);
    ~frameSolver(void);
    void clear(void);
    void refresh(void);
    float labelScore(int partID, int labelIndex);
    Solvlet solveFrame(std::map<uint32_t, std::vector<LimbLabel>> limbLabels, int frameID = -1);
    std::map<uint32_t, partAdjacentsJoints> toJointMap(Skeleton skeleton);
    Skeleton getAverageJointsSkeleton(Skeleton pattern) ;
    Skeleton getShiftedLabelsSkeleton(Skeleton pattern);

    int skeletonSize();
    bool IsSolved();
    long int iterations;

  private:
    std::vector<cv::Point2f> getLimbLabelJoints(LimbLabel limbLabel);
    std::map<int, std::vector<Label>> prepareLimbLabels(std::map<uint32_t, std::vector<LimbLabel>> limbLabels);
    
    std::map<uint32_t, partAdjacentsJoints> parts; // partJoints, indexed by part ID
    std::map<int, std::vector<Label>> labels; // adapted for this solver labels form
    std::map<uint32_t, uint32_t> SkeletonLabelsIndexes; // partID ~ labelIndex
    std::map<uint32_t, double> SkeletonLabelsScores;
    std::map<uint32_t, bool> ignored;

    bool solved;
  }; 
  
  class _Solver : public Solver
  {
  public:
    _Solver(void);
    ~_Solver(void);
    
    std::vector<Solvlet> solve(Sequence& seq);
    std::vector<Solvlet> solve(Sequence& seq, std::map<std::string, float> params);

    void emplaceDefaultParameters(std::map<std::string, float> &params) const;

  };
}

#endif; //__SOLVER_HPP_
