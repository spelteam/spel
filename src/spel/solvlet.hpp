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
    virtual ~Solvlet(void);

    virtual Solvlet &operator=(const Solvlet &s);
    virtual bool operator<(const Solvlet &s) const;
    virtual bool operator>(const Solvlet &s) const;

    virtual int getFrameID(void) const;
    virtual void setFrameID(int _id);

    virtual std::vector<LimbLabel> getLabels(void) const;
    virtual const std::vector<LimbLabel>* getLabelsPtr(void) const;
    virtual void setLabels(std::vector<LimbLabel> _labels);

    virtual Skeleton toSkeleton(const Skeleton &example);
    virtual float evaluateSolution(Frame* frame, std::map<std::string, float> params);

  private:
    int frameId;
    std::vector<LimbLabel> labels;
  };

}

#endif  // _SOLVLET_HPP_
