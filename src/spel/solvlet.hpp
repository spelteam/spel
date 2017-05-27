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
    Solvlet(const int id, const std::vector<LimbLabel> &labels);
    Solvlet(const Solvlet &s);
    Solvlet(Solvlet &&s);
    ~Solvlet(void);

    Solvlet &operator=(const Solvlet &s);
    Solvlet &operator=(Solvlet &&s);
    bool operator<(const Solvlet &s) const;
    bool operator>(const Solvlet &s) const;

    int getFrameID(void) const;
    void setFrameID(int id);

    std::vector<LimbLabel> getLabels(void) const;
    const std::vector<LimbLabel>* getLabelsPtr(void) const;
    void setLabels(const std::vector<LimbLabel> &labels);

    Skeleton toSkeleton(const Skeleton &example) const;
    float evaluateSolution(Frame* frame, const std::map<std::string, float> &params);

  private:
    int m_frameId;
    std::vector<LimbLabel> m_labels;
  };

}

#endif  // _SOLVLET_HPP_
