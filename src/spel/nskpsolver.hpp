#ifndef _NSKPSOLVER_HPP_
#define _NSKPSOLVER_HPP_

// SPEL definitions
#include "predef.hpp"

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

#include "solver.hpp"
#include "frame.hpp"
#include "imagepixelsimilaritymatrix.hpp"
#include "minspanningtree.hpp"

namespace SPEL
{
  ///define the space and the model

  class NSKPSolver : public Solver
  {
  private:
    struct SolvletScore
    {
      Solvlet solvlet;
      float score;
      int parentFrame;
    };

  public:
    NSKPSolver(void);
    ///inherited virtual
    ~NSKPSolver(void);
    ///inherited virtual
    std::vector<Solvlet> solve(Sequence& frames);
    ///inherited virtual
    std::vector<Solvlet> solve(Sequence &frames, std::map<std::string, float>  params);
    ///inherited virtual
    std::vector<Solvlet> solve(Sequence& frames, std::map<std::string, float>  params, const ImageSimilarityMatrix& ISM);

    std::vector<cv::Point2i> suggestKeyframes(const ImageSimilarityMatrix& ism, std::map<std::string, float> params);
  private:
#ifdef DEBUG
    FRIEND_TEST(nskpsolverTests, findFrameIndexById);
    FRIEND_TEST(nskpsolverTests, ScoreCostAndJointCost);
    FRIEND_TEST(nskpsolverTests, evaluateSolution);
    FRIEND_TEST(nskpsolverTests, findFrameIndexById);
    FRIEND_TEST(nskpsolverTests, propagateFrame);
    FRIEND_TEST(nskpsolverTests, propagateKeyframes);
    FRIEND_TEST(nskpsolverTests, buildMSTs);
    FRIEND_TEST(nskpsolverTests, suggestKeyframes_A);
    FRIEND_TEST(nskpsolverTests, suggestKeyframes_B);
#endif  // DEBUG
    std::vector<Solvlet> propagateKeyframes(std::vector<Frame*>& frames, std::map<std::string, float>  params, const ImageSimilarityMatrix& ism, const std::vector<MinSpanningTree> &trees, std::vector<int> &ignore);
    std::vector<MinSpanningTree > buildFrameMSTs(const ImageSimilarityMatrix &ism, std::map<std::string, float> params);

    float evaluateSolution(Frame* frame, std::vector<LimbLabel> labels, std::map<std::string, float> params);

    uint32_t findFrameIndexById(int id, std::vector<Frame*> frames);
    float computeScoreCost(const LimbLabel& label, std::map<std::string, float> params);

    float computeJointCost(const LimbLabel& child, const LimbLabel& parent, std::map<std::string, float> params, bool toChild);
    float computeNormJointCost(const LimbLabel& child, const LimbLabel& parent, std::map<std::string, float> params, float max, bool toChild);

    float computePriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, std::map<std::string, float> params);
    float computeNormPriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, std::map<std::string, float> params, float min, float max);

    std::vector<NSKPSolver::SolvletScore> propagateFrame(int frameId, const std::vector<Frame *> &frames, std::map<std::string, float> params, const ImageSimilarityMatrix& ism, const std::vector<MinSpanningTree> &trees, std::vector<int> &ignore);
    int test(int frameId, const std::vector<Frame*>& frames, std::map<std::string, float> params, const ImageSimilarityMatrix &ism, const std::vector<MinSpanningTree> &trees, std::vector<int>& ignore);
  };

}

#endif  // _NSKPSOLVER_HPP_
