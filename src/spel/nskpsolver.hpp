#ifndef _NSKPSOLVER_HPP_
#define _NSKPSOLVER_HPP_

// SPEL definitions
#include "predef.hpp"

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

// STL
#include <vector>
#include <limits>
#include <opencv2/opencv.hpp>
#include <tree.hh>
#include <algorithm>
#include <chrono>
#include <future>

// OpenGM
#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/graphicalmodel/space/discretespace.hxx>
#include <opengm/operations/adder.hxx>
#include <opengm/functions/explicit_function.hxx>
#include <opengm/inference/messagepassing/messagepassing.hxx>
#include <opengm/operations/minimizer.hxx>

#include "lockframe.hpp"
#include "colorHistDetector.hpp"
#include "hogDetector.hpp"
#include "surfDetector.hpp"
#include "tlpssolver.hpp"
#include "solver.hpp"
#include "frame.hpp"
#include "imagepixelsimilaritymatrix.hpp"
#include "minspanningtree.hpp"

namespace SPEL
{
  ///define the space and the model

  class NSKPSolver : public Solver
  {
  protected:
    typedef struct SolvletScore
    {
      Solvlet solvlet;
      float score;
      int parentFrame;
    } SolvletScore;

  public:
    NSKPSolver(void);
    ///inherited virtual
    virtual ~NSKPSolver(void);
    ///inherited virtual
    virtual std::vector<Solvlet> solve(Sequence& frames);
    ///inherited virtual
    virtual std::vector<Solvlet> solve(Sequence &frames, std::map<std::string, float>  params);
    ///inherited virtual
    virtual std::vector<Solvlet> solve(Sequence& frames, std::map<std::string, float>  params, const ImageSimilarityMatrix& ISM);

    virtual std::vector<cv::Point2i> suggestKeyframes(const ImageSimilarityMatrix& ism, std::map<std::string, float> params);
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
  protected:
    virtual std::vector<Solvlet> propagateKeyframes(std::vector<Frame*>& frames, std::map<std::string, float>  params, const ImageSimilarityMatrix& ism, const std::vector<MinSpanningTree> &trees, std::vector<int> &ignore);
    virtual std::vector<MinSpanningTree > buildFrameMSTs(const ImageSimilarityMatrix &ism, std::map<std::string, float> params);

    virtual float evaluateSolution(Frame* frame, std::vector<LimbLabel> labels, std::map<std::string, float> params);

    virtual uint32_t findFrameIndexById(int id, std::vector<Frame*> frames);
    virtual float computeScoreCost(const LimbLabel& label, std::map<std::string, float> params);

    virtual float computeJointCost(const LimbLabel& child, const LimbLabel& parent, std::map<std::string, float> params, bool toChild);
    virtual float computeNormJointCost(const LimbLabel& child, const LimbLabel& parent, std::map<std::string, float> params, float max, bool toChild);

    virtual float computePriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, std::map<std::string, float> params);
    virtual float computeNormPriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, std::map<std::string, float> params, float min, float max);

    virtual std::vector<NSKPSolver::SolvletScore> propagateFrame(int frameId, const std::vector<Frame *> frames, std::map<std::string, float> params, const ImageSimilarityMatrix& ism, const std::vector<MinSpanningTree> &trees, std::vector<int> &ignore);
    virtual int test(int frameId, const std::vector<Frame*>& frames, std::map<std::string, float> params, const ImageSimilarityMatrix &ism, const std::vector<MinSpanningTree> &trees, std::vector<int>& ignore);
  };

}

#endif  // _NSKPSOLVER_HPP_
