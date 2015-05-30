#include <iostream>
#include <nskpsolver.hpp>
#include <tlpssolver.hpp>
#include "projectLoader.hpp"
#include <string>

using namespace std;

Mat computeErrorToGT(vector<Solvlet> solves, vector<Frame*> keyframes) //return squared error from solve to GT
{
    Mat errorMatrix;
    if(!solves.size()==keyframes.size())
    {
        cerr<<"Number of solves and number of keyframes do not match!" << endl;
        return errorMatrix;
    }

    errorMatrix.create(solves.size(), solves[0].getLabels().size(), DataType<float>::type);

    vector<vector<float> > globalPartErrors;

    for(uint32_t i=0; i<keyframes.size();++i) //for every frame
    {
        //if there is a solvlet for it
        bool isSolved = false;
        int solveIndex=-1;
        for(uint32_t j=0; j<solves.size();++j)
        {
            if(solves[j].getFrameID()==i)
            {
                isSolved=true;
                solveIndex=j;
                break;
            }
        }

        if(isSolved)
        {
            Skeleton kSkel = keyframes[i]->getSkeleton();
            tree<BodyPart> partTree = kSkel.getPartTree();

            vector<float> partErrors;
            vector<LimbLabel> labels = solves[solveIndex].getLabels();
            for(tree<BodyPart>::iterator partIter=partTree.begin(); partIter!=partTree.end(); ++partIter) //for every bodypart
            {

                LimbLabel label=labels[partIter->getPartID()];
                Point2f p0,p1;
                label.getEndpoints(p0,p1);
                Point2f t0(kSkel.getBodyJoint(partIter->getParentJoint())->getImageLocation());
                Point2f t1(kSkel.getBodyJoint(partIter->getChildJoint())->getImageLocation());

                //dist(p0gt, p0sim)*0.5+dist(p1gt, p1sim)*0.5;
                float d0 = PoseHelper::distSquared(p0,t0); //dist between parent joints
                float d1 = PoseHelper::distSquared(p1,t1); //dist between child joints

                float error = 0.5*d0+0.5*d1; //average error for the two joints
                errorMatrix.at<float>(solveIndex, partIter->getPartID()) = error;

            }
            globalPartErrors.push_back(partErrors);
        }
    }
    return errorMatrix;
}

int main (int argc, char **argv)
{
    if (argc != 2)
    {
        cout << "Usage hybridSolverTest [settings file]" << endl;
        return -1;
    }

    //do file parsing
    ifstream in(argv[1]);

    string inProj, gtProj, outFold, paramName;
    float param_min, param_max, param_step;

    in >> inProj >> gtProj >> outFold >> paramName >> param_min >> param_max >> param_step;

    string curFolder = inProj;
    curFolder = curFolder.substr(0, curFolder.find_last_of("/"));
    if (curFolder.back() != '/')
    {
        curFolder += '/';
    }

    ProjectLoader projectLoader(curFolder);
    ProjectLoader gtLoader(curFolder);

    cout << "Loading projects..." << endl;

    if (projectLoader.Load(inProj) == true)
    {
        cout << "Test project was successfully loaded" << endl;
    }
    else
    {
        cout << "Test project was not loaded" << endl;
        return -1;
    }

    if (gtLoader.Load(gtProj) == true)
    {
        cout << "GT project was successfully loaded" << endl;
    }
    else
    {
        cout << "GT project was not loaded" << endl;
        return -1;
    }

    vector <Frame*> gtFrames = gtLoader.getFrames(); //the ground truth frames to compare against
    Sequence gtSeq(0,"gt", gtFrames);

    ImageSimilarityMatrix ism;
    string ismFile(projectLoader.getProjectTitle()+".ism");
    if(!ism.read(ismFile))
    {
        ism.buildImageSimilarityMatrix(projectLoader.getFrames());
        ism.write(ismFile);
    }

    string baseOutFolder(outFold);
    baseOutFolder += "/" + paramName + "/";
    projectLoader.CreateDirectorySystemIndependent(baseOutFolder);
    baseOutFolder+= projectLoader.getProjectTitle() +"/";
    projectLoader.CreateDirectorySystemIndependent(baseOutFolder);

    ofstream out(baseOutFolder+"/nskpSolver_"+paramName+".err");

    out << 1 << " " <<paramName << endl;

    //now print this matrix to file
    for(float param = param_min; param<=param_max; param+=param_step)
    {
        vector <Frame*> vFrames = projectLoader.getFrames();
        Sequence seq(0, "test", vFrames);

        NSKPSolver nSolver;
        cout << "Solving with " << paramName << " at " << param << endl;

        map <string, float> params; //use the default params
        params.emplace(paramName, param); //emplace the param we're testing

        //global settings
        params.emplace("imageCoeff", 1.0); //set solver detector infromation sensitivity
        params.emplace("jointCoeff", 0.6); //set solver body part connectivity sensitivity
        params.emplace("priorCoeff", 0.0); //set solver distance to prior sensitivity

        //detector settings
        params.emplace("useCSdet", 1.0); //determine if ColHist detector is used and with what coefficient
        params.emplace("useHoGdet", 0.1); //determine if HoG descriptor is used and with what coefficient
        params.emplace("useSURFdet", 0.0); //determine whether SURF detector is used and with what coefficient

        params.emplace("grayImages", 1); // use grayscale images for HoG?

        //solver settings
        params.emplace("nskpIters", 1); //do as many NSKP iterations as is useful at each run
        params.emplace("acceptLockframeThreshold", 0.52); // 0.52 set the threshold for NSKP and TLPSSolvers, forcing TLPS to reject some solutions
        params.emplace("badLabelThresh", 0.45); //set bad label threshold, which will force solution discard at 0.45
        params.emplace("partDepthRotationCoeff", 1.25); //search radius increase for each depth level in the part tree

        params.emplace("anchorBindDistance", 0); //restrict search regions if within bind distance of existing keyframe or lockframe (like a temporal link
        params.emplace("anchorBindCoeff", 0.3); //multiplier for narrowing the search range if close to an anchor (lockframe/keyframe)
        params.emplace("bindToLockframes", 0); //should binds be also used on lockframes?
        //params.emplace("maxFrameHeight", 288); //scale to 288p - same size as trijump video seq, for detection

        seq.estimateUniformScale(params);
        seq.computeInterpolation(params);

        vector<Solvlet> nskpSolve;
        //do an iterative NSKP solve
        nskpSolve = nSolver.solve(seq, params, ism);

        //now do the error analysis
        Mat errors;
        if(nskpSolve.size()>0)
            errors = computeErrorToGT(nskpSolve, gtFrames);

        for(uint32_t i=0; i<nskpSolve.size();++i)
        {
            for(uint32_t row=0; row<errors.rows; ++row)
            {
                //this is a row in the output file
                //frameID p1Value p2Value p3Value .. pNValue limb1RMS limb2RMS limb3RMS limb4RMS ... limbKRMS meanRMS evalScore
                out << nskpSolve[i].getFrameID() << " " << param << " ";
                for(uint32_t col=0; col<errors.cols; ++col)
                {
                    //this is an item of the row
                    out << errors.at<float>(row,col) << " ";
                }
                out << endl; //newline at the end of the block
            }

            Frame* frame = seq.getFrames()[nskpSolve[i].getFrameID()];
            Frame* parent = seq.getFrames()[frame->getParentFrameID()];

            projectLoader.drawLockframeSolvlets(ism, nskpSolve[i], frame, parent, (baseOutFolder+"/"+to_string(param)+"/").c_str(), Scalar(0,0,255), 1);
        }

        errors.release();

    }
    out.close();

    return 0;
}

