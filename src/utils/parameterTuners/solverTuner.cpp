#include <iostream>
#include <nskpsolver.hpp>
#include <tlpssolver.hpp>
#include "projectLoader.hpp"
#include <string>
#include <cstdlib>
#include <ctime>
#ifdef WINDOWS
#include <random>
#endif

#define NUM_THREADS 1

using namespace std;

Mat computeErrorToGT(vector<Solvlet> solves, vector<Frame*> keyframes) //return squared error from solve to GT
{
    Mat errorMatrix;
    if(solves.size()>keyframes.size())
    {
        cerr<<"Number of solves exceeds number of keyframes!" << endl;
        return errorMatrix;
    }

    errorMatrix.create(solves.size(), //number of frames
                       solves[0].getLabels().size(),//number of limbs
                       DataType<float>::type); //data type


    for(uint32_t i=0; i<solves.size();++i)
    {
        int frameID=solves[i].getFrameID();
        Skeleton kSkel = keyframes[frameID]->getSkeleton();
        tree<BodyPart> partTree = kSkel.getPartTree();

        vector<LimbLabel> labels = solves[i].getLabels();
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
            errorMatrix.at<float>(i, partIter->getPartID()) = error;
        }
    }

    return errorMatrix;
}

void addKeyframeNoise(vector<Frame*>& frames, float mean, float sd, float max)
{
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(mean,sd);

    for(uint32_t i=0; i<frames.size(); ++i)
    {
        if(frames[i]->getFrametype()==KEYFRAME) //if it's a keyframe only
        {
            Skeleton skel=frames[i]->getSkeleton();
            tree<BodyJoint> jointTree = skel.getJointTree();

            for(tree<BodyJoint>::iterator joint=jointTree.begin(); joint!=jointTree.end(); ++joint)
            {
                float x;
                while(true)
                {
                    x = distribution(generator);
                    if(abs(x)<=max)
                        break;
                }

                float y;
                while(true)
                {
                    y = distribution(generator);
                    if(abs(y)<=max)
                        break;
                }

                joint->setImageLocation(joint->getImageLocation()+Point2f(x,y));
            }
            skel.setJointTree(jointTree); //set perturbed joint tree to skel
            frames[i]->setSkeleton(skel); //set perturbed skel to frame
        }
    }
}

vector<Solvlet> getSolve(const vector<Frame*>& gtFrames, vector<Frame*>& vFrames, vector<int> requestedKeyframes, vector<Point2i> suggestedKeyframes,
                         string paramName, float param, string balanceParamName, string solverName, float mean, float sd,
                         float max, ImageSimilarityMatrix ism, map<string,float> defaultParams)
{
    vector<int> actualKeyframes;
    //build the sequence based on
    vFrames.clear();


    for(uint32_t i=0; i<gtFrames.size(); ++i) //init
    {
        Frame * fp = new Interpolation();
        vFrames.push_back(fp);
    }

    //insert any specified keyframes
    for(uint32_t i=0; i<requestedKeyframes.size();++i)
    {
        if(requestedKeyframes[i]!=-1) //if it's not an automatic one
        {
            int frameID=requestedKeyframes[i];
            //delete vFrames[requestedKeyframes[i]]; //free memory
            vFrames[frameID] = new Keyframe(); //assign new keyframe
            //copy all the data
\
            vFrames[frameID]->setSkeleton(gtFrames[frameID]->getSkeleton());
            vFrames[frameID]->setID(gtFrames[frameID]->getID());
            vFrames[frameID]->setImage(gtFrames[frameID]->getImage());
            vFrames[frameID]->setMask(gtFrames[frameID]->getMask());

            actualKeyframes.push_back(frameID);
        }
    }

    //insert the automatically suggested keyframes, if any
    for(uint32_t i=0; i<requestedKeyframes.size();++i)
    {
        if(requestedKeyframes[i]==-1) //if it's not an automatic one
        {
            //for each suggested keyframe, find the first one that isn't yet in the list of keyframes
            for(vector<Point2i>::iterator fi=suggestedKeyframes.begin(); fi!=suggestedKeyframes.end(); ++fi)
            {
                int frameID=fi->x;
                bool alreadyPresent=false;
                for(vector<int>::iterator ak=actualKeyframes.begin(); ak!=actualKeyframes.end(); ++ak)
                {
                    if(*ak==fi->x)
                    {
                        alreadyPresent=true;
                        break;
                    }
                }
                if(alreadyPresent==false) //not present yet
                {
                    //add it

                    //delete vFrames[requestedKeyframes[i]]; //free memory
                    vFrames[frameID] = new Keyframe(); //assign new keyframe
                    //copy all the data
    \
                    vFrames[frameID]->setSkeleton(gtFrames[frameID]->getSkeleton());
                    vFrames[frameID]->setID(gtFrames[frameID]->getID());
                    vFrames[frameID]->setImage(gtFrames[frameID]->getImage());
                    vFrames[frameID]->setMask(gtFrames[frameID]->getMask());

                    actualKeyframes.push_back(frameID);

                    //break
                    break;
                }
            }
        }
    }

    //fill in the rest with interpolation frames
    for(uint32_t i=0; i<vFrames.size(); ++i)
    {
        if(vFrames[i]->getFrametype()!=KEYFRAME) //if not keyframes
        {
            int frameID=i;
            //delete vFrames[requestedKeyframes[i]]; //free memory
            vFrames[frameID] = new Interpolation(); //assign new keyframe
            //copy all the data
\
            vFrames[frameID]->setID(gtFrames[frameID]->getID());
            vFrames[frameID]->setImage(gtFrames[frameID]->getImage().clone()); //deep copy image
            vFrames[frameID]->setMask(gtFrames[frameID]->getMask().clone()); //deep copy mask

            actualKeyframes.push_back(frameID);
        }
    }

    //the new frame set has been generated, and can be used for solving

    cout << "Solving with " << paramName << " at " << param << endl;

    if(paramName=="gaussianNoise") //if we're testing gaussian noise
    {
        addKeyframeNoise(vFrames, mean, sd, max); //mean = min, sd = step, max = max
    }

    map <string, float> params=defaultParams; //transfer the default params

    //emplace general defaults
    params.emplace(paramName, param); //emplace the param we're testing
    if(balanceParamName!="none") //if there is a balance param, emplace it too
        params.emplace(balanceParamName, 1.0-param); //it will be 1.0-paramValue

    //global settings
    params.emplace("imageCoeff", 1.0); //set solver detector infromation sensitivity
    params.emplace("jointCoeff", 0.0); //set solver body part connectivity sensitivity
    params.emplace("priorCoeff", 0.0); //set solver distance to prior sensitivity

    //detector settings
    params.emplace("useCSdet", 0.0); //determine if ColHist detector is used and with what coefficient
    params.emplace("useHoGdet", 1.0); //determine if HoG descriptor is used and with what coefficient
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
    params.emplace("maxFrameHeight", 280); //scale to 288p - same size as trijump video seq, for detection

    Sequence seq(0, "test", vFrames);

    seq.estimateUniformScale(params);
    seq.computeInterpolation(params);

    NSKPSolver nSolver;
    TLPSSolver tSolver;

    vector<Solvlet> fSolve;
    //do an iterative NSKP solve
    if(solverName=="TLPSSolver")
        fSolve = tSolver.solve(seq, params);
    else if(solverName=="NSKPSolver")
        fSolve = nSolver.solve(seq, params, ism);
    else if(solverName=="hybridSolver")
    {
        vector<Solvlet> finalSolve;
        int prevSolveSize=0;
        do
        {
            prevSolveSize=finalSolve.size(); //set size of the final solve

            vector<Solvlet> nskpSolve, tlpsSolve;
            //do an iterative NSKP solve
            nskpSolve = nSolver.solve(seq, params, ism);

            for(vector<Solvlet>::iterator s=nskpSolve.begin(); s!=nskpSolve.end(); ++s)
                finalSolve.push_back(*s);

            //then, do a temporal solve
            seq.computeInterpolation(params); //recompute interpolation (does this improve results?)

            tlpsSolve = tSolver.solve(seq, params);

            for(vector<Solvlet>::iterator s=tlpsSolve.begin(); s!=tlpsSolve.end(); ++s)
                finalSolve.push_back(*s);

        } while(finalSolve.size()>prevSolveSize);
        fSolve = finalSolve;
    }

    return fSolve;
}

int main (int argc, char **argv)
{
    if (argc != 2)
    {
        cout << "Usage solverTuner [settings file]" << endl;
        return -1;
    }

    /* Generate a new random seed from system time - do this once in your constructor */
    srand(time(0));

    //do file parsing
    ifstream in(argv[1]);

    string gtProj, outFold, paramName, balanceParamName, solverName;
    float param_min, param_max, param_step;
    int numParams, numKeyframes; //number of extra params to include
    map<string,float> defaultParams;
    vector<int> requestedKeyframes;

    try
    {
        //File format:
        //Project: ../../../src/utils/general/testdata1/trijumpSDGT.xml

        //OutDir: solverTuner_out

        //SolverName: NSKPSolver

        //NumKeyframes: 3
        //KeyframeNumbers: -1 -1 -1

        //TestParameter: jointCoeff
        //ParameterMin: 0.0
        //ParameterMax: 1.0
        //ParameterStep: 0.1

        //BalanceParameter: imageCoeff

        //NumOtherParams: 2
        //useCSdet 0.1
        //useHoGdet 1.0

        string temp;
        in >> temp >> gtProj >> temp >> outFold >> temp >> solverName >> temp >> numKeyframes >> temp;

        //read the keyframe ID's, if -1 then the are autogenerated
        for(uint32_t i=0; i<numKeyframes; ++i)
        {
            int k;
            in>>k;
            requestedKeyframes.push_back(k);
        }

        in >> temp >> paramName >> temp >> param_min >> temp >> param_max >> temp >> param_step;

        in >> temp >> balanceParamName;
        //now read in any static parameters
        in >> temp >> numParams;
        for(uint32_t i=0; i<numParams; ++i)
        {
            string p;
            float val;
            in >> p >> val;
            defaultParams.emplace(p, val);
        }
    }
    catch (int e)
    {
        cerr << "Something wrong in the input file " << argv[1] << " please check format. Exception " << e << endl;
        return 0; //return
    }
    //set up the params

    string curFolder = gtProj;
    curFolder = curFolder.substr(0, curFolder.find_last_of("/"));
    if (curFolder.back() != '/')
    {
        curFolder += '/';
    }

    ProjectLoader gtLoader(curFolder);

    cout << "Loading projects..." << endl;

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

    ImageSimilarityMatrix ism;
    string ismFile(gtLoader.getProjectTitle()+".ism");
    if(!ism.read(ismFile))
    {
        ism.buildImageSimilarityMatrix(gtLoader.getFrames());
        ism.write(ismFile);
    }

    string baseOutFolder(outFold);
    gtLoader.CreateDirectorySystemIndependent(baseOutFolder);
    baseOutFolder += "/" + paramName + "/";
    gtLoader.CreateDirectorySystemIndependent(baseOutFolder);
    baseOutFolder+="/" + solverName + "/";
    gtLoader.CreateDirectorySystemIndependent(baseOutFolder);
    baseOutFolder+= gtLoader.getProjectTitle() +"/";
    gtLoader.CreateDirectorySystemIndependent(baseOutFolder);

    ofstream out(baseOutFolder+"/"+paramName+"_"+solverName+"_"+gtLoader.getProjectTitle()+".err");

//    Project: ../../../src/utils/general/testdata1/trijumpSDGT.xml

//    OutDir: solverTuner_out

//    SolverName: NSKPSolver

//    NumKeyframes: 3
//    KeyframeNumbers: -1 -1 -1

//    TestParameter: jointCoeff
//    ParameterMin: 0.0
//    ParameterMax: 1.0
//    ParameterStep: 0.1

//    BalanceParameter: imageCoeff

//    NumOtherParams: 2
//    useCSdet 0.1
//    useHoGdet 1.0

    //write file header, containing config file data (could be read for another experiment if needed
    out << "Project: " << argv[1] << endl; //settings file
    out << "OutDir: " << outFold << endl; //project name
    out << "SolverName: " << solverName << endl; //solver name
    out << "NumKeyframes: " << numKeyframes << " "; //number of keyframes
    out << "KeyframeNumbers: ";
    for(uint32_t i=0; i<numKeyframes; ++i) // and what they are
        out << requestedKeyframes[i] << " ";
    out << endl;
    out << "TestParameter: " << paramName << endl;
    out << "ParameterMin: " << param_min << endl;
    out << "ParameterMax: " << param_max << endl;
    out << "ParameterStep: " << param_step << endl; // the param we're testing

    out << "BalanceParameter: " << balanceParamName << endl;
    out << "NumOtherParams: " << numParams << endl;
    for(map<string,float>::iterator p=defaultParams.begin(); p!=defaultParams.end(); ++p) // and what they are
        out << p->first << " "  << p->second << endl;

    //get the keyframe suggestions from NSKPSolver
    //NSKPSolver s;
    vector<Point2i> suggestedKeyframes = NSKPSolver().suggestKeyframes(ism, defaultParams);

    float mean=param_min, max=param_max, sd=param_step;

    if(paramName=="gaussianNoise") //will always do 100 trials
    {
        param_min=0;
        param_max=100;
        param_step=1;
    }

    //now print this matrix to file
    for(float param = param_min; param<=param_max; param+=param_step) //do 100 trials for gaussian noise
    {
        vector<Frame*> vFrames;
        vector<Solvlet> fSolve = getSolve(gtFrames, vFrames, requestedKeyframes, suggestedKeyframes,
                                          paramName, param, balanceParamName, solverName, mean, sd,
                                          max, ism, defaultParams);
        //now do the error analysis
        Mat errors;
        if(fSolve.size()>0)
            errors = computeErrorToGT(fSolve, gtFrames);

        out << param << endl;
        out << "{" << endl;

        for(uint32_t row=0; row<errors.rows; ++row)
        {
            //this is a row in the output file
            //frameID p1Value p2Value p3Value .. pNValue limb1RMS limb2RMS limb3RMS limb4RMS ... limbKRMS meanRMS evalScore
            out << fSolve[row].getFrameID() << " ";
            for(uint32_t col=0; col<errors.cols; ++col)
            {
                //this is an item of the row
                out << errors.at<float>(row,col) << " ";
            }
            out << endl; //newline at the end of the block

            Frame* frame = vFrames[fSolve[row].getFrameID()];
            Frame* parent = vFrames[frame->getParentFrameID()];

            gtLoader.drawLockframeSolvlets(ism, fSolve[row], frame, parent, (baseOutFolder+"/"+to_string(param)+"/").c_str(), Scalar(0,0,255), 1);
        }

        out << "}" << endl;

        //release errors
        errors.release();
        //delete sequence
        for(uint32_t i=0; i<vFrames.size(); ++i)
        {
            delete vFrames[i];
        }
        vFrames.clear();

    }
    out.close();

    return 0;
}

