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
    void createPartsConnections(Skeleton &skeleton);
    void createIndexedJoints(Skeleton &skeleton);

    std::map<uint32_t, partAdjacentsJoints> parts;
    std::map<uint32_t, std::vector<int>> joints;
    std::string m_name; // Sequence or project name
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
    void setInterframeDistances(int PartID);
    double getInterframeDistance(int PartID, int isChildJoint);
    double getSkeletonScore();
    float getBadPartScore();
    int findBadPart();
    void recalculateAdjacentsLabels(int partID);
    std::vector<cv::Point2f> getAvgJointsLocation(int partID);   
    bool initialize(std::map<uint32_t, std::vector<LimbLabel>> limbLabels);
    void singleIteration();
    Solvlet solveFrame(std::map<uint32_t, std::vector<LimbLabel>> limbLabels, int frameID = -1);   
    Skeleton getAverageJointsSkeleton(Skeleton pattern) ;
    Skeleton getShiftedLabelsSkeleton(Skeleton pattern);
    void setParrentFrameSolver(frameSolver * fsolver);
    void setChildFrameSolver(frameSolver * fsolver);
    Solvlet getSolve(std::map<uint32_t, std::vector<LimbLabel>> &limbLabels);

    long int getIterationNumber();
    bool isSolved();
    void setSolved(bool isSolved);
    void setFrameID(int frameID);
    void setLogStream(std::ostream * logStream);

  private:   
    std::vector<cv::Point2f> getLimbLabelJoints(LimbLabel limbLabel);
    /*std::map<int, std::vector<Label>>*/void prepareLimbLabels(std::map<uint32_t, std::vector<LimbLabel>> &limbLabels);

    std::vector<frameSolver*> neighborFrameSolvers;
    IndexedSkeletonModel *m_pattern; // partJoints, indexed by part ID
    int m_frameID;
    std::map<int, std::vector<Label>> labels; // adapted for this solver labels form
    std::map<uint32_t, uint32_t> SkeletonLabelsIndexes; // partID ~ labelIndex
    std::map<uint32_t, double> SkeletonLabelsScores;
    std::map<uint32_t, double> prevFrameDistances;
    std::map<uint32_t, bool> ignored;

    long int iterations;
    int idleIterations = 0;
    const int iterationsLimit = 30000;
    float skeletonScore;
    float badPartID;
    float badPartScore;
    bool solved;

    std::ostream * LogStream;
  }; 
  
  class _Solver : public Solver
  {
  public:
    _Solver(void);
    ~_Solver(void);
    
    std::vector<Solvlet> solve(Sequence& seq);
    std::vector<Solvlet> solve(Sequence& seq, std::map<std::string, float> params);
    std::vector<Solvlet> solveGlobal(Sequence& seq, std::map<std::string, float> params);
    void train(std::vector<Frame*> &slice, std::map<std::string, float> &params);
    std::map<uint32_t, std::vector<LimbLabel>>detect(std::map<std::string, float> &params, Frame* &frame, Frame* previousFrame = 0);
    Solvlet solveFrame(std::map<std::string, float> &params, frameSolver &fSolver, Frame* frame, Frame* prevFrame = 0);
    void emplaceDefaultParameters(std::map<std::string, float> &params) const;

    static std::vector<std::vector<Frame*>> createSlices(std::vector<Frame*> &frames);   
    void setLogStream(std::ostream * logStream);
    long int getTrainTime();
    long int getDetectTime();
    
  private:
    std::vector<Detector*> detectors;
    std::vector<std::string> detectorsNames;
    long int trainTime;
    long int detectTime;
    std::ostream * LogStream;
  };
}

#endif; //__SOLVER_HPP_
