#ifndef _PROJECTRUNNER_H_
#define _PROJECTRUNNER_H_

#include <iostream>
#include "projectLoader.hpp"

using namespace std;

class ProjectRunner
{
public:
  ProjectRunner(string _testName);
  int Run(int argc, char **argv);
  virtual void train(vector <Frame*> _frames, map <string, float> params) = 0;
  virtual vector <vector <LimbLabel> > detect(Frame *frame, map <string, float> params, vector <vector <LimbLabel>> limbLabels) = 0;
  virtual void DrawSpecific(string outFolder) { };
private:
  string testName;
};

#endif // _PROJECTRUNNER_H_
