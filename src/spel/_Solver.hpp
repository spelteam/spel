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
  class IndexedSkeletonModel
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
    ~IndexedSkeletonModel();
    IndexedSkeletonModel(Skeleton skeleton, std::string name = "");
    partAdjacentsJoints getPartConnections(int partID);

    void fromSkeleton(Skeleton skeleton, std::string name = "");
    std::string getName();
    uint32_t size();
    void clear();

  private:
    friend class frameSolver;
    std::map<uint32_t, partAdjacentsJoints> parts;
    std::string m_name;
  };

  class frameSolver
  {
  public:
    struct Label {
      std::vector<cv::Point2f> joints;
      float score;
    };
  public:
    frameSolver(IndexedSkeletonModel *pattern);
    ~frameSolver(void);
    void clear(void);
    void refresh(void);
    float labelScore(int partID, int labelIndex);
    Solvlet solveFrame(std::map<uint32_t, std::vector<LimbLabel>> limbLabels, int frameID = -1);   
    Skeleton getAverageJointsSkeleton(Skeleton pattern) ;
    Skeleton getShiftedLabelsSkeleton(Skeleton pattern);

    bool IsSolved();
    long int iterations;

  private:
    std::vector<cv::Point2f> getLimbLabelJoints(LimbLabel limbLabel);
    std::map<int, std::vector<Label>> prepareLimbLabels(std::map<uint32_t, std::vector<LimbLabel>> limbLabels);
     
    IndexedSkeletonModel *m_pattern; // partJoints, indexed by part ID
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
    void train(std::vector<Frame*> &slice, std::map<std::string, float> &params);
    std::map<uint32_t, std::vector<LimbLabel>>detect(Frame* &frame, std::map<std::string, float> &params, Skeleton prevSkeleton);
    static std::vector<std::vector<Frame*>> createSlices(std::vector<Frame*> frames);

    void emplaceDefaultParameters(std::map<std::string, float> &params) const;

  private:
    std::vector<Detector*> detectors;
    std::vector<std::string> detectorsNames;
  };
}

#endif; //__SOLVER_HPP_
