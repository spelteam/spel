#include <iostream>
#include <tlpssolver.hpp>
#include "projectLoader.hpp"

using namespace std;

int main (int argc, char **argv)
{
  if (argc != 3) 
  {
    cout << "Usage tlpsSolverTest [project.xml] [out directory]" << endl;
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
  
  map <string, float> params; //use the default params
  params.emplace("debugLevel", 3); //set the debug setting to highest (0,1,2,3)

  vector<Solvlet> solve;
  
  vector <Frame*> vFrames = projectLoader.getFrames();
  Sequence seq(0, "test", vFrames);

  //now test inrepolation for this sequence
  seq.estimateUniformScale(params);
  seq.computeInterpolation(params);

  //first, test the 3D locations
  for(uint32_t i=0; i<vFrames.size(); ++i)
  {
      if(vFrames[i]->getFrametype()==KEYFRAME)
        cout << vFrames[i]->getID() << " KEYFRAME " << endl;
      else
          cout << vFrames[i]->getID() << endl;

      tree <BodyJoint> jointTree = vFrames[i]->getSkeleton().getJointTree();
      tree <BodyJoint>::iterator iter;
      for(iter=jointTree.begin(); iter!=jointTree.end(); ++iter)
      {
          Point3f spaceLoc = iter->getSpaceLocation();
          cout << "\t id: " << iter->getLimbID() << " x: " << spaceLoc.x
               << " y: " << spaceLoc.y << " z: " << spaceLoc.z << endl;
      }
  }

  exit(0);

  vector <Frame*>::iterator i;

  TLPSSolver tSolver;

  cout << "Solving using TLPSSolver..." << endl;
  //solve with some default params
  solve = tSolver.solve(seq, params);

  //draw the solution
  for(i=vFrames.begin(); i!=vFrames.end(); ++i)
  {
    Frame *f = *i;

    projectLoader.drawFrameSolvlets(solve[f->getID()], f, argv[2], Scalar(0,0,255), 2);
  }

  return 0;
}

