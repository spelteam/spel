#ifndef _SOLUTION_HPP_
#define _SOLUTION_HPP_

#include <string>
#include <vector>
#include "limbLabel.hpp"
#include "skeleton.hpp"

using namespace std;

class Solution
{
  public:
    Solution(void);
    Solution(int id, string sName, vector<float> params, vector<LimbLabel> sol);
    
    Solution &operator=(const Solution &s);
    bool operator==(const Solution &s) const;
    bool operator!=(const Solution &s) const;

    int getId(void);
    void setId(int _id);

    vector <LimbLabel> getLabels(void);
    void setLabels(vector <LimbLabel> _labels);

    Skeleton toSkeleton(void);
  
  private:
    int solverId; //id of the solver solution was obtained from
    string solverName; //name of the solver
    vector <float> solverParams;
    vector <LimbLabel> labels;
};

#endif  // _SOLUTION_HPP_
 
