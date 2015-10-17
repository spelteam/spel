#ifndef _SEQUENCE_HPP_
#define _SEQUENCE_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>
#include <math.h>

// Eigen3
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "spelHelper.hpp"
#include "interpolation.hpp"
#include "frame.hpp"

namespace SPEL
{
  /// Used to evaluate accuracy of a detection
  class Sequence
  {
  public:
    Sequence(void);
    Sequence(const Sequence& seq);
    ///constructor
    Sequence(int idx, std::string seqName, std::vector<Frame*> seq);
    virtual ~Sequence(void);
    virtual std::string getName(void) const;
    virtual void setName(const std::string& _name);
    virtual int getID(void) const;
    virtual void setID(const int& _id);
    virtual std::vector<Frame*> getFrames(void) const;
    virtual void setFrames(const std::vector<Frame*> _frames);
    ///compute (or re-compute) interpolation for all frames which are not a keyframe or a lockframe
    virtual void computeInterpolation(std::map<std::string, float> &params);
    virtual void estimateUniformScale(std::map<std::string, float> &params);

  protected:
    virtual std::vector<Frame*> interpolateSlice(std::vector<Frame*> slice, std::map<std::string, float> params);
    virtual std::vector<Frame*> interpolateSlice2D(std::vector<Frame*> slice, std::map<std::string, float> params);
  private:
    /// detection score
    std::vector<Frame*> frames;
    ///sequence name
    std::string name;
    int id;
  };

}

#endif  // _SEQUENCE_HPP_
