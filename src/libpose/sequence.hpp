#ifndef _SEQUENCE_HPP_
#define _SEQUENCE_HPP_

#include <string>
#include "frame.hpp"

using namespace std;

// Used to evaluate accuracy of a detection
class Sequence
{
  public:
    Sequence(void);
    Sequence(const Sequence& seq);
    Sequence(string seqName, vector<Frame*> seq); //constructor
    void computeInterpolation(map<string, float> params); //compute (or re-compute) interpolation for all frames which are not a keyframe or a lockframe
    
  private:
    vector<Frame*> interpolateSlice(vector<Frame*> slice, map<string, float> params);
    vector<Frame*> frames; // detection score
    string name; // detector name, name of the algorithm that generate the evaluation
};

#endif  // _SEQUENCE_HPP_
 