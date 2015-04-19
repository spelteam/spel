#ifndef _SOLVLET_HPP_
#define _SOLVLET_HPP_

#include <string>
#include <vector>
#include "limbLabel.hpp"
#include "skeleton.hpp"

using namespace std;

class Solvlet
{
  public:
    Solvlet(void);
    Solvlet(int id, vector<LimbLabel> labels);
    
    Solvlet &operator=(const Solvlet &s);
    // bool operator==(const Solvlet &s) const;
    // bool operator!=(const Solvlet &s) const;

    int getFrameID(void) const;
    void setFrameID(int _id);

    vector<LimbLabel> getLabels(void) const;
    void setLabels(vector<LimbLabel> _labels);

    Skeleton toSkeleton(void);
  
  private:
    int frameId;
    vector<LimbLabel> labels;
};

#endif  // _SOLVLET_HPP_
