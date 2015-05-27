#ifndef _TLPSSOLVER_HPP_
#define _TLPSSOLVER_HPP_

//OpenGM
#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/graphicalmodel/graphicalmodel_hdf5.hxx>
#include <opengm/graphicalmodel/space/discretespace.hxx>
#include <opengm/operations/adder.hxx>
#include <opengm/functions/explicit_function.hxx>
#include <opengm/inference/messagepassing/messagepassing.hxx>
#include <opengm/operations/minimizer.hxx>

#include <vector>
#include "solver.hpp"
#include "solution.hpp"
#include "frame.hpp"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace opengm;

#define PI 3.14159265

///define the space and the model
class TLPSSolver: Solver
{

    ///define the space
    typedef opengm::DiscreteSpace<> Space;
    ///define the model
    typedef opengm::GraphicalModel<float, opengm::Adder, opengm::ExplicitFunction<float>, Space> Model;

    ///define the update rules
    typedef BeliefPropagationUpdateRules<Model, opengm::Minimizer> UpdateRules;
    ///define the inference algorithm
    typedef MessagePassing<Model, opengm::Minimizer, UpdateRules, opengm::MaxDistance> BeliefPropagation;

public:
    TLPSSolver(void);
    ///inherited virtual
    ~TLPSSolver(void);
    ///inherited virtual
    vector<Solvlet> solve(Sequence& frames);
    ///inherited virtual
    vector<Solvlet> solve(Sequence& frames, map<string, float> params);
    //INHERITED
    //public:
    // string getName(); //get the solver name. Every class inheriting solver has its own Name
    // string getId(); //get the solver Id. Every class inheriting solver has is own ID
private:

    float evaluateSolution(Frame* frame, vector<LimbLabel> labels, map<string, float> params);

    int findFrameIndexById(int id, vector<Frame*> frames);
    float computeScoreCost(const LimbLabel& label, map<string, float> params);
    float computeJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params, bool toChild);
    float computeNormJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params, float jointMax, bool toChild);
    float computePriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, map<string, float> params);
    float computeNormPriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, map<string, float> params, float max);

    float computePastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, map<string, float> params);
    float computeNormPastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, map<string, float> params, float jointMax);
    float computeFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, map<string, float> params);
    float computeNormFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, map<string, float> params, float max);
    float computeAnchorCost(const LimbLabel& thisLabel, Frame* anchor, map<string, float> params);
    float computeNormAnchorCost(const LimbLabel& thisLabel, Frame* anchor, map<string, float> params, float jointMax);

    ///separate the sequence into slices, for temporal solve
    vector<vector<Frame*> > slice(const vector<Frame*>& frames);

    //INHERITED
    //private:
    //int id;
    //string name;
};

#endif  // _TLPSSOLVER_HPP_

