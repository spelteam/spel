#ifndef _SEQUENCE_HPP_
#define _SEQUENCE_HPP_

#include <string>
#include "frame.hpp"

using namespace std;

/// Used to evaluate accuracy of a detection
class Sequence
{
public:
    Sequence(void);
    Sequence(const Sequence& seq);
    ///constructor
    Sequence(int idx, string seqName, vector<Frame*> seq);
    string getName() const;
    void setName(const string& _name);
    int getID() const;
    void setID(const int& _id);
    vector<Frame*> getFrames() const;
    void setFrames(const vector<Frame*> _frames);
    ///compute (or re-compute) interpolation for all frames which are not a keyframe or a lockframe
    void computeInterpolation(map<string, float> &params);
    void estimateUniformScale(map<string, float> &params);
    
private:
    vector<Frame*> interpolateSlice(vector<Frame*> slice, map<string, float> params);
    /// detection score
    vector<Frame*> frames;
    ///sequence name
    string name;
    int id;
};

#endif  // _SEQUENCE_HPP_

