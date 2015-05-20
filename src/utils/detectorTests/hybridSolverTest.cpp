#include <iostream>
#include <nskpsolver.hpp>
#include <tlpssolver.hpp>
#include "projectLoader.hpp"

using namespace std;

int main (int argc, char **argv)
{
    if (argc != 3)
    {
        cout << "Usage hybridSolverTest [project.xml] [out directory]" << endl;
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

    vector <Frame*> vFrames = projectLoader.getFrames();
    Sequence seq(0, "test", vFrames);

    //now test inrepolation for this sequence
    seq.estimateUniformScale(params);
    seq.computeInterpolation(params);

    NSKPSolver nSolver;
    TLPSSolver tSolver;
    cout << "Solving using NSKPSolver..." << endl;
    //solve with some default params
    //ImageSimilarityMatrix ism(vFrames);
    ImageSimilarityMatrix ism;
    if(!ism.read("testISM.ism"))
    {
        ism.buildImageSimilarityMatrix(vFrames);
        ism.write(("testISM.ism"));
    }
    params.emplace("nskpIters", 0); //do as many NSKP iterations as is useful at each run
    params.emplace("acceptLockframeThreshold", 0.52); //set the threshold for NSKP and TLPSSolvers, forcing TLPS to reject some solutions

    vector<Solvlet> finalSolve;
    int prevSolveSize=0;
    do
    {
        prevSolveSize=finalSolve.size(); //set size of the final solve

        vector<Solvlet> nskpSolve, tlpsSolve;
        //do an iterative NSKP solve
        nskpSolve = nSolver.solve(seq, params, ism);

        for(vector<Solvlet>::iterator s=nskpSolve.begin(); s!=nskpSolve.end();++s)
            finalSolve.push_back(*s);

        //then, do a temporal solve
        seq.computeInterpolation(params); //recompute interpolation (does this improve results?)

        tlpsSolve = tSolver.solve(seq, params);

        for(vector<Solvlet>::iterator s=tlpsSolve.begin(); s!=nskpSolve.end();++s)
            finalSolve.push_back(*s);

    } while(finalSolve.size()>prevSolveSize);


    //draw the solution
    for(uint32_t i=0; i<finalSolve.size();++i)
    {
        Frame* frame = seq.getFrames()[finalSolve[i].getFrameID()];
        Frame* parent = seq.getFrames()[frame->getParentFrameID()];

        projectLoader.drawLockframeSolvlets(ism, finalSolve[i], frame, parent, argv[2], Scalar(0,0,255), 1);
    }


//    for(uint32_t i=0; i<solve.size();++i)
//    {
//        Frame* frame = vFrames[solve[i].getFrameID()];

//        projectLoader.drawFrameSolvlets(solve[i], frame, argv[2], Scalar(0,0,255), 1);
//    }

    return 0;
}

