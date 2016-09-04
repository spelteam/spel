#ifndef _SEQUENCE_HPP_
#define _SEQUENCE_HPP_

#ifdef DEBUG
#include <gtest/gtest_prod.h>
#endif  // DEBUG

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
    std::string getName(void) const;
    void setName(const std::string& _name);
    int getID(void) const;
    void setID(const int& _id);
    std::vector<Frame*> getFrames(void) const;
    Frame* getFrame(uint32_t id) const;
    void setFrames(const std::vector<Frame*> &_frames);
    ///compute (or re-compute) interpolation for all frames which are not a keyframe or a lockframe
    void computeInterpolation(std::map<std::string, float> &params);
    void estimateUniformScale(std::map<std::string, float> &params);

  private:
    std::vector<Frame*> interpolateSlice(std::vector<Frame*> slice, std::map<std::string, float> params);
    std::vector<Frame*> interpolateSlice2D(std::vector<Frame*> slice, std::map<std::string, float> params);
#ifdef DEBUG
    FRIEND_TEST(SequenceTests, CopyConstructor);
    FRIEND_TEST(SequenceTests, interpolateSlice2D);
#endif  // DEBUG
  
    /// detection score
    std::vector<Frame*> frames;
    ///sequence name
    std::string name;
    int id;
  };

}

#endif  // _SEQUENCE_HPP_
