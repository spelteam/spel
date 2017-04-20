#ifndef _SOLVLET_HPP_
#define _SOLVLET_HPP_

// SPEL definitions
#include "predef.hpp"

// STL
#include <string>
#include <vector>

#include "limbLabel.hpp"
#include "skeleton.hpp"
#include "frame.hpp"

namespace SPEL
{
  class Solvlet
  {
  public:
    Solvlet(void);
    Solvlet(int id, std::vector<LimbLabel> labels);
    ~Solvlet(void);

    Solvlet &operator=(const Solvlet &s);
    bool operator<(const Solvlet &s) const;
    bool operator>(const Solvlet &s) const;

    int getFrameID(void) const;
    void setFrameID(int _id);

    std::vector<LimbLabel> getLabels(void) const;
    const std::vector<LimbLabel>* getLabelsPtr(void) const;
    void setLabels(std::vector<LimbLabel> _labels);

    Skeleton toSkeleton(const Skeleton &example) const;
    float evaluateSolution(Frame* frame, std::map<std::string, float> params);

  private:
    int frameId;
    std::vector<LimbLabel> labels;
  };

}

#endif  // _SOLVLET_HPP_
