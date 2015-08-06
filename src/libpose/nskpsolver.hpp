#ifndef _NSKPSOLVER_HPP_
#define _NSKPSOLVER_HPP_

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
#include "solution.hpp"
#include "frame.hpp"
#include "imagesimilaritymatrix.hpp"
#include "minspanningtree.hpp"

namespace SPEL
{
  using namespace std;
  using namespace std::chrono;
  using namespace opengm;
  using namespace cv;

  ///define the space and the model

  class NSKPSolver : public Solver
  {
    typedef struct SolvletScore
    {
      Solvlet solvlet;
      float score;
      int parentFrame;
    } SolvletScore;

  public:
    NSKPSolver();
    ///inherited virtual
    ~NSKPSolver();
    ///inherited virtual
    vector<Solvlet> solve(Sequence& frames);
    ///inherited virtual
    vector<Solvlet> solve(Sequence &frames, map<string, float>  params);
    ///inherited virtual
    vector<Solvlet> solve(Sequence& frames, map<string, float>  params, const ImageSimilarityMatrix& ISM);

    vector<Point2i> suggestKeyframes(const ImageSimilarityMatrix& ism, map<string, float> params);

    //INHERITED
    //public:
    // string getName(); //get the solver name. Every class inheriting solver has its own Name
    // string getId(); //get the solver Id. Every class inheriting solver has is own ID
  private:
#ifdef DEBUG
    FRIEND_TEST(nskpsolverTests, findFrameIndexById);
    FRIEND_TEST(nskpsolverTests, ScoreCostAndJointCost);
    FRIEND_TEST(nskpsolverTests, evaluateSolution);
#endif  // DEBUG
    vector<Solvlet> propagateKeyframes(vector<Frame*>& frames, map<string, float>  params, const ImageSimilarityMatrix& ism, vector<int> &ignore);
    vector<MinSpanningTree > buildFrameMSTs(const ImageSimilarityMatrix &ism, map<string, float> params); //int treeSize, float threshold)

    float evaluateSolution(Frame* frame, vector<LimbLabel> labels, map<string, float> params);

    uint32_t findFrameIndexById(int id, vector<Frame*> frames);
    float computeScoreCost(const LimbLabel& label, map<string, float> params);

    float computeJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params, bool toChild);
    float computeNormJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params, float max, bool toChild);

    float computePriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, map<string, float> params);
    float computeNormPriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, map<string, float> params, float min, float max);

    vector<NSKPSolver::SolvletScore> propagateFrame(int frameId, const vector<Frame *> frames, map<string, float> params, const ImageSimilarityMatrix& ism, const vector<MinSpanningTree> &trees, vector<int> &ignore);
    int test(int frameId, const vector<Frame*>& frames, map<string, float> params, const ImageSimilarityMatrix &ism, const vector<MinSpanningTree> &trees, vector<int>& ignore); //test function

    vector<vector<Frame*> > slice(const vector<Frame*>& frames);

    //INHERITED
    //int id;
    //string name;
  };

}

#endif  // _NSKPSOLVER_HPP_

