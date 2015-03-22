#include <iostream>
#include <colorHistDetector.hpp>
#include "projectLoader.hpp"

using namespace std;


int main (int argc, char **argv)
{
  if (argc != 3) 
  {
    cout << "Usage colorHistDetectorTest [project.xml] [out directory]" << endl;
    return -1;
  }
  string curFolder = argv[1];
  curFolder = curFolder.substr(0, curFolder.find_last_of("/"));
  if (curFolder.back() != '/')
  {
    curFolder += '/';
  }

  ProjectLoader projectLoader(curFolder);

  cout << "Loading project..." << endl;
 
  if (projectLoader.Load(argv[1]) == true)
  {  
    cout << "Project was successfully loaded" << endl;
  }
  else
  {
    cout << "Project was not loaded" << endl;
    return -1;
  }
  ColorHistDetector detector;
  map <string, float> params;
  cout << "Training..." << endl;
  try
  {
    detector.train(projectLoader.getFrames(), params);
    Sequence *seq = new Sequence(0, "colorHistDetector", projectLoader.getFrames());
    if (seq != 0)
    {
      seq->estimateUniformScale(params);
      seq->computeInterpolation(params);
      delete seq;
    }
  }
  catch(exception &e)
  {
    cerr << e.what() << endl;
    return -1;
  }

  cout << "Training complete" << endl;
    vector <Frame*> vFrames = projectLoader.getFrames();
  vector <Frame*>::iterator i;
  map <string, float> detectParams;
  cout << "Detecting..." << endl;
  for (i = vFrames.begin(); i != vFrames.end(); ++i)
  {
    //check if frame has a keyframe before AND after
    bool hasPrevAnchor=false, hasFutureAnchor=false;
    //before
    for (vector<Frame*>::reverse_iterator prevAnchor(i); prevAnchor != vFrames.rend(); ++prevAnchor)
    {
      if ((*prevAnchor)->getFrametype() == KEYFRAME || (*prevAnchor)->getFrametype() == LOCKFRAME)
      {
        hasPrevAnchor = true;
        break;
      }
    }
    //after
    for (vector<Frame*>::iterator futureAnchor = i; futureAnchor != vFrames.end(); ++futureAnchor)
    {
      if ((*futureAnchor)->getFrametype() == KEYFRAME || (*futureAnchor)->getFrametype() == LOCKFRAME)
      {
        hasFutureAnchor = true;
        break;
      }
    }
    if (!hasPrevAnchor || !hasFutureAnchor)
      continue;

    Frame *f = *i;
#ifndef DEBUG
    if (f->getFrametype() == INTERPOLATIONFRAME)
#endif  // DEBUG
    {
      vector <vector <LimbLabel> > labels;
      try
      {
        labels = detector.detect(f, detectParams, labels);
        projectLoader.Save(labels, argv[2], f->getID());
        projectLoader.Draw(labels, f, argv[2], f->getID(), Scalar(0, 0, 0), Scalar(0, 0, 255), 2);
      }
      catch (exception &e)
      {
        cerr << e.what() << endl;
        continue;
      }
    }
  }
  cout << "Detecting complete" << endl;
  for (i = vFrames.begin(); i != vFrames.end(); ++i)
  {
    Frame *f = *i;
    if (f != 0)
      delete f;
  }
  return 0;
}

