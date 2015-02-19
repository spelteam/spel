#include <iostream>
#include <hogDetector.hpp>
#include "projectLoader.hpp"

using namespace std;


int main (int argc, char **argv)
{
  if (argc != 3) 
  {
    cout << "Usage hogDetectorTest [project.xml] [out directory]" << endl;
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
  HogDetector detector;
  map <string, float> params;
  cout << "Training..." << endl;
  try
  {
    detector.train(projectLoader.getFrames(), params);
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
    Frame *f = *i;
#ifndef DEBUG
    if (f->getFrametype() == INTERPOLATIONFRAME)
#endif  // DEBUG
    {
      vector <vector <LimbLabel> > labels;
      try
      {
        labels = detector.detect(f, detectParams);
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
