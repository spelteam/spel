#ifndef _TLPSSOLVER_HPP_
#define _TLPSSOLVER_HPP_

// SPEL definitions
#include "predef.hpp"

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

// STL
#include <vector>

#include "solver.hpp"
#include "frame.hpp"

namespace SPEL
{
  ///define the space and the model
  class TLPSSolver : public Solver
  {
  public:
    TLPSSolver(void);
    ///inherited virtual
    ~TLPSSolver(void);
    ///inherited virtual
    std::vector<Solvlet> solve(Sequence& frames) ;
    ///inherited virtual
    std::vector<Solvlet> solve(Sequence& frames, std::map<std::string, float> params) ;
    std::vector<Solvlet> solve(Sequence& frames, std::map<std::string, float> params, std::vector<Solvlet> solvlets) ;
  private:
#ifdef DEBUG
    FRIEND_TEST(tlpssolverTests, solveGlobal);
    FRIEND_TEST(tlpssolverTests, evaluateSolution);
    FRIEND_TEST(tlpssolverTests, findFrameIndexById);
    FRIEND_TEST(tlpssolverTests, ScoreCostAndJointCost);
    FRIEND_TEST(tlpssolver_Tests, slice_);
#endif  // DEBUG
      
    std::vector<Solvlet> solveWindowed(Sequence &sequence, std::map<std::string, float> params); //inherited virtual
    std::vector<Solvlet> solveGlobal(Sequence &sequence, std::map<std::string, float> params); //inherited virtual

    float evaluateSolution(Frame* frame, std::vector<LimbLabel> labels, std::map<std::string, float> params);

    float computeScoreCost(const LimbLabel& label, std::map<std::string, float> params);
    float computeJointCost(const LimbLabel& child, const LimbLabel& parent, std::map<std::string, float> params, bool toChild);
    float computeNormJointCost(const LimbLabel& child, const LimbLabel& parent, std::map<std::string, float> params, float jointMax, bool toChild);
    float computePriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, std::map<std::string, float> params);
    float computeNormPriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, std::map<std::string, float> params, float max);

    float computePastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, std::map<std::string, float> params);
    float computeNormPastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, std::map<std::string, float> params, float jointMax);
    float computeFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, std::map<std::string, float> params);
    float computeNormFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, std::map<std::string, float> params, float max);
    float computeAnchorCost(const LimbLabel& thisLabel, Frame* anchor, std::map<std::string, float> params);
    float computeNormAnchorCost(const LimbLabel& thisLabel, Frame* anchor, std::map<std::string, float> params, float jointMax);

    ///separate the sequence into slices, for temporal solve
    std::vector<std::vector<Frame*> > slice(const std::vector<Frame*>& frames);
    /// <summary>Emplaces the default parameters.</summary>
    /// <param name="params">The parameters.</param>
    void emplaceDefaultParameters(
      std::map <std::string, float> &params) const ;
  };

}

#endif  // _TLPSSOLVER_HPP_
