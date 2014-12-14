#ifndef _TLPSSOLVER_HPP_
#define _TLPSSOLVER_HPP_

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
#include <opencv2/opencv.hpp>

using namespace std;
using namespace opengm;

//define the space and the model

#define PI 3.14159265

class TLPSSolver: Solver
{

  // struct AxisAngle
  // {
  //   Point3f axis=Point3f();
  //   float angle=-1;
  // };

  // struct Quaternion
  // {
  //   float w;
  //   float x;
  //   float y;
  //   float z;
  // };

  //define the space
  typedef opengm::DiscreteSpace<> Space;
  //define the model
  typedef opengm::GraphicalModel<float, opengm::Adder, opengm::ExplicitFunction<float>, Space> Model;

  //define the update rules
  typedef BeliefPropagationUpdateRules<Model, opengm::Minimizer> UpdateRules;
  //define the inference algorithm
  typedef MessagePassing<Model, opengm::Minimizer, UpdateRules, opengm::MaxDistance> BeliefPropagation;

  public:
    TLPSSolver();
    ~TLPSSolver(); //inherited virtual
    Solution solve(const vector<Frame*>& frames); //inherited virtual
    Solution solve(const vector<Frame*>& frames, map<string, float> params); //inherited virtual
  //INHERITED
    //public:
    // string getName(); //get the solver name. Every class inheriting solver has its own Name
    // string getId(); //get the solver Id. Every class inheriting solver has is own ID
  private:

    float evaluateSolution(Frame* frame, vector<LimbLabel> labels, map<string, float> params);

    int findFrameIndexById(int id, vector<Frame*> frames);
    float computeScoreCost(const LimbLabel& label, map<string, float> params);
    float computeJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params);
    float computePriorCost(const LimbLabel& label, const BodyPart& prior, Skeleton& skeleton, map<string, float> params);
    float computePastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, map<string, float> params);
    float computeFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, map<string, float> params);
    float computeAnchorCost(const LimbLabel& thisLabel, Frame* anchor, map<string, float> params);

    vector<vector<Frame*> > slice(const vector<Frame*>& frames, map<string, float> params); //separate the sequence into slices, for temporal solve
    vector<Frame*> interpolateSlice(vector<Frame*> slice);//, map<string, float> params)
    float interpolateFloat(float prevAngle, float nextAngle, int step, int numSteps);

    // Point3f rotate(Point3f point, Mat rotationMatrix);

    // TLPSSolver::AxisAngle computeAxisAngle(Point3f vec1, Point3f vec2, float epsilon);
    // Mat axisAngleToRotMatrix(TLPSSolver::AxisAngle input);
    // Mat mapVectorRotation(Point3f vec1, Point3f vec2, float epsilon);

    //INHERITED
    //int id;
    //string name;
};

#endif  // _TLPSSOLVER_HPP_

