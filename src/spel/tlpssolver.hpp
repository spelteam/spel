#ifndef _TLPSSOLVER_HPP_
#define _TLPSSOLVER_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <vector>
#include <opencv2/opencv.hpp>
#include <math.h>

//OpenGM
#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/graphicalmodel/graphicalmodel_hdf5.hxx>
#include <opengm/graphicalmodel/space/discretespace.hxx>
#include <opengm/operations/adder.hxx>
#include <opengm/functions/explicit_function.hxx>
#include <opengm/inference/messagepassing/messagepassing.hxx>
#include <opengm/operations/minimizer.hxx>

// Eigen3
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "solver.hpp"
#include "frame.hpp"
#include "lockframe.hpp"
#include "colorHistDetector.hpp"
#include "hogDetector.hpp"
#include "surfDetector.hpp"
#include "interpolation.hpp"

namespace SPEL
{
  ///define the space and the model
  class TLPSSolver : public Solver
  {
  public:
    TLPSSolver(void);
    ///inherited virtual
    virtual ~TLPSSolver(void);
    ///inherited virtual
    virtual std::vector<Solvlet> solve(Sequence& frames);
    ///inherited virtual
    virtual std::vector<Solvlet> solve(Sequence& frames, std::map<std::string, float> params);
    virtual std::vector<Solvlet> solve(Sequence& frames, std::map<std::string, float> params, std::vector<Solvlet> solvlets);
  protected:

    virtual std::vector<Solvlet> solveWindowed(Sequence &sequence, std::map<std::string, float> params); //inherited virtual
    virtual std::vector<Solvlet> solveGlobal(Sequence &sequence, std::map<std::string, float> params); //inherited virtual

    virtual float evaluateSolution(Frame* frame, std::vector<LimbLabel> labels, std::map<std::string, float> params);

    virtual int findFrameIndexById(int id, std::vector<Frame*> frames);
    virtual float computeScoreCost(const LimbLabel& label, std::map<std::string, float> params);
    virtual float computeJointCost(const LimbLabel& child, const LimbLabel& parent, std::map<std::string, float> params, bool toChild);
    virtual float computeNormJointCost(const LimbLabel& child, const LimbLabel& parent, std::map<std::string, float> params, float jointMax, bool toChild);
    virtual float computePriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, std::map<std::string, float> params);
    virtual float computeNormPriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, std::map<std::string, float> params, float max);

    virtual float computePastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, std::map<std::string, float> params);
    virtual float computeNormPastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, std::map<std::string, float> params, float jointMax);
    virtual float computeFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, std::map<std::string, float> params);
    virtual float computeNormFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, std::map<std::string, float> params, float max);
    virtual float computeAnchorCost(const LimbLabel& thisLabel, Frame* anchor, std::map<std::string, float> params);
    virtual float computeNormAnchorCost(const LimbLabel& thisLabel, Frame* anchor, std::map<std::string, float> params, float jointMax);

    ///separate the sequence into slices, for temporal solve
    virtual std::vector<std::vector<Frame*> > slice(const std::vector<Frame*>& frames);
  };

}

#endif  // _TLPSSOLVER_HPP_
