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
    NSKPSolver(void) noexcept;
    ///inherited virtual
    ~NSKPSolver(void) noexcept;
    ///inherited virtual
    std::vector<Solvlet> solve(Sequence& frames) const noexcept;
    ///inherited virtual
    std::vector<Solvlet> solve(Sequence &frames, 
      std::map<std::string, float>  params) const noexcept;
    ///inherited virtual
    std::vector<Solvlet> solve(Sequence& frames, 
      std::map<std::string, float>  params, 
      const ImageSimilarityMatrix& ISM) const noexcept;

    std::vector<std::pair<int, int>> suggestKeyframes(
      const ImageSimilarityMatrix& ism, 
      std::map<std::string, float> params) const;
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
    std::vector<Solvlet> propagateKeyframes(std::vector<Frame*>& frames, 
      std::map<std::string, float>  params, const ImageSimilarityMatrix& ism, 
      const std::vector<MinSpanningTree> &trees, 
      std::vector<int> &ignore) const;
    std::vector<MinSpanningTree > buildFrameMSTs(
      const ImageSimilarityMatrix &ism, 
      std::map<std::string, float> params) const;

    float evaluateSolution(Frame* frame, std::vector<LimbLabel> labels, 
      std::map<std::string, float> params) const;

    float computeScoreCost(const LimbLabel& label, 
      std::map<std::string, float> params) const;

    float computeJointCost(const LimbLabel& child, const LimbLabel& parent, 
      bool toChild) const;
    float computeNormJointCost(const LimbLabel& child, const LimbLabel& parent,
      std::map<std::string, float> params, float max, bool toChild) const;

    float computePriorCost(const LimbLabel& label, const BodyPart& prior, 
      const Skeleton& skeleton) const;
    float computeNormPriorCost(const LimbLabel& label, const BodyPart& prior, 
      const Skeleton& skeleton, std::map<std::string, float> params, 
      float max) const;

    std::vector<NSKPSolver::SolvletScore> propagateFrame(const int frameId, 
      const std::vector<Frame *> &frames, std::map<std::string, float> params, 
      const ImageSimilarityMatrix& ism, 
      const std::vector<MinSpanningTree> &trees, 
      std::vector<int> &ignore) const;
    /// <summary>Emplaces the default parameters.</summary>
    /// <param name="params">The parameters.</param>
    void emplaceDefaultParameters(
      std::map <std::string, float> &params) const noexcept;
  };

}

#endif  // _NSKPSOLVER_HPP_
