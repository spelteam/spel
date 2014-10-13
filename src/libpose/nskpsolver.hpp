#ifndef _NSKPSOLVER_HPP_
#define _NSKPSOLVER_HPP_

//OpenGM
#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/graphicalmodel/space/discretespace.hxx>
#include <opengm/operations/adder.hxx>
#include <opengm/functions/explicit_function.hxx>
#include <opengm/inference/messagepassing/messagepassing.hxx>
#include <opengm/operations/minimizer.hxx>

#include <vector>
#include "solver.hpp"
#include "solution.hpp"
#include "frame.hpp"
#include "imagesimilaritymatrix.hpp"
#include "minspanningtree.hpp"
#include <opencv2/opencv.hpp>


using namespace std;
using namespace opengm;

//define the space and the model

class NSKPSolver: Solver
{
  //define the space
  typedef opengm::DiscreteSpace<> Space;
  //define the model
  typedef opengm::GraphicalModel<float, opengm::Adder, opengm::ExplicitFunction<float>, Space> Model;

  //define the update rules
  typedef BeliefPropagationUpdateRules<Model, opengm::Minimizer> UpdateRules;
  //define the inference algorithm
  typedef MessagePassing<Model, opengm::Minimizer, UpdateRules, opengm::MaxDistance> BeliefPropagation;

  public:
    NSKPSolver();
    ~NSKPSolver(); //inherited virtual
    Solution solve(const vector<Frame*>& frames); //inherited virtual
    Solution solve(const vector<Frame*>& frames, const vector<float>& params); //inherited virtual
    Solution solve(const vector<Frame*>& frames, const vector<float>& params, const ImageSimilarityMatrix& ISM); //inherited virtual
  //INHERITED
    //public:
    // string getName(); //get the solver name. Every class inheriting solver has its own Name
    // string getId(); //get the solver Id. Every class inheriting solver has is own ID
  private:
    vector<Frame*> propagateKeyframes(const vector<Frame*>& frames, const vector<float>& params, const ImageSimilarityMatrix& ism);
    vector<MinSpanningTree > buildFrameMSTs(ImageSimilarityMatrix ism, int treeSize, float threshold);
    
    vector<Point2i> suggestKeyframes(vector<MinSpanningTree>& mstVec);
    float evaluateSolution(Frame* frame, vector<LimbLabel> labels, bool debug);

    int findFrameIndexById(int id, vector<Frame*> frames);
    float computeScoreCost(const LimbLabel& label);
    float computeJointCost(const LimbLabel& child, const LimbLabel& parent);
    float computePriorCost(const LimbLabel& label, const BodyPart& prior);

    //INHERITED
    //int id;
    //string name;
};

#endif  // _NSKPSOLVER_HPP_

