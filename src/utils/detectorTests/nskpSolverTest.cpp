#include <iostream>
#include <nskpsolver.hpp>
#include "projectLoader.hpp"

using namespace std;

int main (int argc, char **argv)
{
    if (argc != 3)
    {
        cout << "Usage nskpSolverTest [project.xml] [out directory]" << endl;
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
    params.emplace("debugLevel", 1); //set the debug setting to highest (0,1,2,3)

    vector<Solvlet> solve;

    vector <Frame*> vFrames = projectLoader.getFrames();
    Sequence seq(0, "test", vFrames);

    //now test inrepolation for this sequence
    seq.estimateUniformScale(params);
    seq.computeInterpolation(params);

    NSKPSolver tSolver;
    cout << "Solving using NSKPSolver..." << endl;
    //solve with some default params
    ImageSimilarityMatrix ism(vFrames);
    solve = tSolver.solve(seq, params, ism);

    //draw the solution
    for(uint32_t i=0; i<solve.size();++i)
    {
        Frame* frame = vFrames[solve[i].getFrameID()];
        Frame* parent = vFrames[frame->getParentFrameID()];

        projectLoader.drawLockframeSolvlets(ism, solve[i], frame, parent, seq, argv[2], Scalar(0,0,255), 1);
    }

    return 0;
}

