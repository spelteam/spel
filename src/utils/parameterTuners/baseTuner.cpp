#include <iostream>
#include <nskpsolver.hpp>
#include <tlpssolver.hpp>
#include "projectLoader.hpp"
#include <string>

using namespace std;

int main (int argc, char **argv)
{
    if (argc != 3)
    {
        cout << "Usage baseTuner [project.xml] [out directory]" << endl;
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

    float jointCoeff_min=0.0, jointCoeff_max=1.0, jointCoeff_step=0.1,
            baseRotationRange_min=10, baseRotationRange_max=180, baseRotationRange_step=10,
            partDepthRotationCoeff_min=1.0, partDepthRotationCoeff_max=2.0, partDepthRotationCoeff_step=0.1;

    for(float baseRotationRange = baseRotationRange_min; baseRotationRange<=baseRotationRange_max; baseRotationRange+=baseRotationRange_step)
    {
        for(float jointCoeff=jointCoeff_min; jointCoeff<=jointCoeff_max; jointCoeff+=jointCoeff_step)
        {
            for(float partDepthRotationCoeff=partDepthRotationCoeff_min; partDepthRotationCoeff<=partDepthRotationCoeff_max; partDepthRotationCoeff+=partDepthRotationCoeff_step)
            {
                vector <Frame*> vFrames = projectLoader.getFrames();
                Sequence seq(0, "test", vFrames);

                NSKPSolver nSolver;
                cout << "Solving using NSKPSolver..." << endl;
                //solve with some default params
                //ImageSimilarityMatrix ism(vFrames);
                ImageSimilarityMatrix ism;
                if(!ism.read("testISM.ism"))
                {
                    ism.buildImageSimilarityMatrix(vFrames);
                    ism.write(("testISM.ism"));
                }

                map <string, float> params; //use the default params
                params.emplace("debugLevel", 1); //set the debug setting to highest (0,1,2,3)

                params.emplace("imageCoeff", 1.0); //set solver detector infromation sensitivity
                params.emplace("jointCoeff", 1.0); //set solver body part connectivity sensitivity
                params.emplace("priorCoeff", 0.0); //set solver distance to prior sensitivity

                params.emplace("useCSdet", 0.5); //determine if ColHist detector is used and with what coefficient
                params.emplace("useHoGdet", 1.0); //determine if HoG descriptor is used and with what coefficient
                params.emplace("useSURFdet", 0.0); //determine whether SURF detector is used and with what coefficient

                params.emplace("nskpIters", 0); //do as many NSKP iterations as is useful at each run
                params.emplace("acceptLockframeThreshold", 0.52); //set the threshold for NSKP and TLPSSolvers, forcing TLPS to reject some solutions
                params.emplace("badLabelThresh", 0.45); //set bad label threshold, which will force solution discard at 0.45
                params.emplace("partDepthRotationCoeff", 1.25); //search radius increase for each depth level in the part tree

                params.emplace("baseRotationRange", 50); //search angle range of +/- 50 degrees

                vector<Solvlet> nskpSolve;
                //do an iterative NSKP solve
                nskpSolve = nSolver.solve(seq, params, ism);

                string baseOutFolder(argv[2]);
                baseOutFolder = baseOutFolder + "/" + to_string(baseRotationRange)+"_"+to_string(jointCoeff)+"_"+to_string(partDepthRotationCoeff);
                //draw the solution
                for(uint32_t i=0; i<nskpSolve.size();++i)
                {
                    Frame* frame = seq.getFrames()[nskpSolve[i].getFrameID()];
                    Frame* parent = seq.getFrames()[frame->getParentFrameID()];

                    projectLoader.drawLockframeSolvlets(ism, nskpSolve[i], frame, parent, baseOutFolder.c_str(), Scalar(0,0,255), 1);
                }
                //export the resulting skeletons from these labels
           }
        }
    }


    return 0;
}

