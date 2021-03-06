// SPEL definitions
#include "predef.hpp"

#include <future>
#include <iostream>
#include <nskpsolver.hpp>
#include <tlpssolver.hpp>
#include "projectLoader.hpp"
#include "colorHistDetector.hpp"
#include "surfDetector.hpp"
#include "hogDetector.hpp"
#include <string>
#include <cstdlib>
#include <ctime>
#ifdef WINDOWS
#include <random>
#endif

using namespace std;
using namespace SPEL;

vector<vector<float> > computeLabelErrorStatToGT(map<uint32_t, vector<LimbLabel> > labels, Frame* keyframe) //return squared error from solve to GT for one frame
{
    vector<vector<float> > errors;

    for(uint32_t i=0; i<labels.size();++i)
    {
        errors.push_back(vector<float>());
    }

    //float frameRMS=0;
    Skeleton kSkel = keyframe->getSkeleton();
    tree<BodyPart> partTree = kSkel.getPartTree();

    for(tree<BodyPart>::iterator partIter=partTree.begin(); partIter!=partTree.end(); ++partIter) //for every bodypart
    {
        //for each body part
        vector<float> partErrors;

        //get the GT points
        Point2f t0(kSkel.getBodyJoint(partIter->getParentJoint())->getImageLocation());
        Point2f t1(kSkel.getBodyJoint(partIter->getChildJoint())->getImageLocation());

        for(vector<LimbLabel>::iterator label=labels[partIter->getPartID()].begin(); label!=labels[partIter->getPartID()].end(); ++label)
        {
            //for each label
            Point2f p0,p1;
            label->getEndpoints(p0,p1);

            float d0 = spelHelper::distSquared(p0,t0); //dist between parent joints
            float d1 = spelHelper::distSquared(p1,t1); //dist between child joints

            partErrors.push_back(sqrt(0.5*d0+0.5*d1)); //average error for the two joints
        }
        errors[partIter->getPartID()]=partErrors;
    }

    return errors;
}

vector<map<uint32_t, vector<LimbLabel> > > doInterpolationDetect(vector<Detector*> detectors, vector<Frame*> vFrames, map<string,float> params)
{
    vector<map<uint32_t, vector<LimbLabel> > > allLabels;
    //vector<Solvlet> solvlets;
    Sequence seq(0, "test", vFrames);
    seq.estimateUniformScale(params);
    seq.computeInterpolation(params);

    vector<Frame*> frames = seq.getFrames();
    for(uint32_t frame=0; frame<frames.size(); ++frame)
    {
        map<uint32_t, vector<LimbLabel> > labels;
        for(uint32_t i=0; i<detectors.size();++i)
            labels = detectors[i]->detect(frames[frame], params, labels); //detect labels based on keyframe training
        
        //detector settings
        params.emplace("useCSdet", 0.0); //determine if ColHist detector is used and with what coefficient
        params.emplace("useHoGdet", 0.0); //determine if HoG descriptor is used and with what coefficient
        params.emplace("useSURFdet", 0.0); //determine whether SURF detector is used and with what coefficient
        

        //make sure all labels contain the same number of detection scores, pad with 1's if they don't

        string hogName = "18500";
        string csName = "4409412";
        string surfName = "21316";

        float useHoG = params.at("useHoGdet");
        float useCS = params.at("useCSdet");
        float useSURF = params.at("useSURFdet");

        for(uint32_t i=0; i<labels.size();++i)
        {
            for(uint32_t j=0; j<labels[i].size(); ++j)
            {
                vector<Score> scores=labels[i][j].getScores();

                bool hogFound=false;
                bool csFound=false;
                bool surfFound=false;

                for(uint32_t k=0; k<scores.size();++k)
                {
                    if(scores[k].getDetName()==hogName)
                        hogFound=true;
                    else if(scores[k].getDetName()==csName)
                        csFound=true;
                    else if(scores[k].getDetName()==surfName)
                        surfFound=true;
                }

                if(!surfFound && useSURF)
                {
                    Score sc(1.0, surfName, useSURF);
                    labels[i][j].addScore(sc);
                }
                if(!csFound && useCS)
                {
                    Score sc(1.0, csName, useCS);
                    labels[i][j].addScore(sc);
                }

                if(!hogFound && useHoG)
                {
                    Score sc(1.0, hogName, useHoG);
                    labels[i][j].addScore(sc);
                }

            }
            sort(labels[i].begin(), labels[i].end());
        }

        allLabels.push_back(labels);
    }
    return allLabels;
}

vector<map<uint32_t, vector<LimbLabel> > > doPropagationDetect(vector<Detector*> detectors, vector<Frame*> vFrames, const ImageSimilarityMatrix &ism, map<string,float> params)
{
    vector<map<uint32_t, vector<LimbLabel> > > allLabels;
    //vector<Solvlet> solvlets;
    Sequence seq(0, "test", vFrames);
    seq.estimateUniformScale(params);
    seq.computeInterpolation(params);

    int kfr=-1;
    vector<Frame*> frames = seq.getFrames();
    for(uint32_t i=0; i<frames.size(); ++i)
    {
        if(frames[i]->getFrametype()==KEYFRAME)
        {
            kfr=i;
            break;
        }
    }
    if(kfr==-1)
    {
        cerr<< "No keyframes found!" << endl;
        return allLabels;
    }

    //propagate the zero keyframe


    for(uint32_t frame=0; frame<frames.size(); ++frame)
    {
        Frame * lockframe = new Interpolation();
        //Skeleton shiftedPrior = frames[frameId]->getSkeleton();
        lockframe->setSkeleton(frames[kfr]->getSkeleton());
        lockframe->setID(frames[frame]->getID());
        lockframe->SetImageFromPath(frames[frame]->GetImagePath());
        lockframe->SetMaskFromPath(frames[frame]->GetMaskPath());

        //compute the shift between the frame we are propagating from and the current frame
        Point2f shift = ism.getShift(frames[kfr]->getID(),frames[frame]->getID());
        lockframe->shiftSkeleton2D(shift);

        map<uint32_t, vector<LimbLabel> > labels;
        for(uint32_t i=0; i<detectors.size();++i)
            labels = detectors[i]->detect(lockframe, params, labels); //detect labels based on keyframe training

        //make sure all labels contain the same number of detection scores, pad with 1's if they don't

        string hogName = "18500";
        string csName = "4409412";
        string surfName = "21316";

        float useHoG = params.at("useHoGdet");
        float useCS = params.at("useCSdet");
        float useSURF = params.at("useSURFdet");

        for(uint32_t i=0; i<labels.size();++i)
        {
            for(uint32_t j=0; j<labels[i].size(); ++j)
            {
                vector<Score> scores=labels[i][j].getScores();

                bool hogFound=false;
                bool csFound=false;
                bool surfFound=false;

                for(uint32_t k=0; k<scores.size();++k)
                {
                    if(scores[k].getDetName()==hogName)
                        hogFound=true;
                    else if(scores[k].getDetName()==csName)
                        csFound=true;
                    else if(scores[k].getDetName()==surfName)
                        surfFound=true;
                }

                if(!surfFound && useSURF)
                {
                    Score sc(1.0, surfName, useSURF);
                    labels[i][j].addScore(sc);
                }
                if(!csFound && useCS)
                {
                    Score sc(1.0, csName, useCS);
                    labels[i][j].addScore(sc);
                }

                if(!hogFound && useHoG)
                {
                    Score sc(1.0, hogName, useHoG);
                    labels[i][j].addScore(sc);
                }

            }
            sort(labels[i].begin(), labels[i].end());
        }

        allLabels.push_back(labels);
    }
    return allLabels;
}

/* Add noise to joint labelling for keyframes with mean and sd and max.
 *
 */
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

std::string base_name(std::string const & path)
{
    return path.substr(path.find_last_of("/\\") + 1);
}

/*Add noise to background masks used for detection to a single frame, and replace that frame's mask loader to the distorted mask*/
void addGaussianMaskNoise(Frame* frame, double mean=0, double standard_deviation=10.0, string tempPath="_tempMasks/")
{

    //Needed paramaters:
    //Percent of mask to add noise to
    //random generator of coordinates to add noise to?
    Mat image, distorted_image;
    image = imread(frame->GetMaskPath());

    string imgName = base_name(frame->GetMaskPath()); //extract image name here

    // We need to work with signed images (as noise can be
    // negative as well as positive). We use 16 bit signed
    // images as otherwise we would lose precision.

    if(image.empty())
    {
        cout<<"[Error]! Input Image Empty!";
        return;
    }

    Mat mSrc_16SC;
    Mat mGaussian_noise = Mat(image.size(),CV_16SC3);
    randn(mGaussian_noise,Scalar::all(mean), Scalar::all(standard_deviation));

    image.convertTo(mSrc_16SC,CV_16SC3);
    addWeighted(mSrc_16SC, 1.0, mGaussian_noise, 1.0, 0.0, mSrc_16SC); //add noise
    mSrc_16SC.convertTo(distorted_image, image.type()); //convert back

    threshold(distorted_image, distorted_image, 125, 255, 0); //now threshold the image - all values about 125 should go to 255, all below should go to zero
    //    image.convertTo(distorted_image,image.type());

    //    addWeighted(distorted_image, 1.0, noise_image, 1.0, 0.0, distorted_image);
    //distorted_image.convertTo(image,image.type());

    //save the distorted image
    string writePath = tempPath+"/"+imgName;
    imwrite(tempPath, distorted_image);
    //now update the frame to point to this image
    frame->SetMaskFromPath(writePath);
}


/* Construct a deep copy test sequence on the basis of a ground-truth sequence
 *
 * Input: Accepts GT sequnce (gtFrames), container for the newly generated sequence (vFrames), and
 * suggestions for what keyframes should be picked (requestedKeyframes). The size of the vector determines the number of keyframes
 * the values of the elements dicated which frames should be selected as keyframes (if possible), and -1 means
 * that selection of keyframe should be automatic.
 *
 * Output: Fills vFrames with the constructed sequence and returns by reference
 */
void createTestSequence(const vector<Frame*>& gtFrames, vector<Frame*>& vFrames, ImageSimilarityMatrix& ism, vector<int> requestedKeyframes, map<string,float>&  defaultParams)
{
    cout << "Creating test sequence..." << endl;

    bool requireSuggestions=false;
    for(auto i=0; i<requestedKeyframes.size(); ++i)
        if(requestedKeyframes[i]==-1)
            requireSuggestions=true;

    vector<std::pair<int, int>> suggestedKeyframes;

    if(requireSuggestions)
        suggestedKeyframes = NSKPSolver().suggestKeyframes(ism, defaultParams);

    vector<int> actualKeyframes;
    //build the sequence based on
    //vector <Frame*> vFrames;

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
            vFrames[frameID]->SetImageFromPath(gtFrames[frameID]->GetImagePath());
            vFrames[frameID]->SetMaskFromPath(gtFrames[frameID]->GetMaskPath());

            actualKeyframes.push_back(frameID);
        }
    }

    //insert the automatically suggested keyframes, if any
    for(uint32_t i=0; i<requestedKeyframes.size();++i)
    {
        if(requestedKeyframes[i]==-1) //if it's not an automatic one
        {
            //for each suggested keyframe, find the first one that isn't yet in the list of keyframes
            for(auto fi=suggestedKeyframes.begin(); fi!=suggestedKeyframes.end(); ++fi)
            {
                int frameID=fi->first;
                bool alreadyPresent=false;
                for(vector<int>::iterator ak=actualKeyframes.begin(); ak!=actualKeyframes.end(); ++ak)
                {
                    if(*ak==fi->first)
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
                    vFrames[frameID]->SetImageFromPath(gtFrames[frameID]->GetImagePath());
                    vFrames[frameID]->SetMaskFromPath(gtFrames[frameID]->GetMaskPath());

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
            vFrames[frameID]->SetImageFromPath(gtFrames[frameID]->GetImagePath()); //deep copy image
            vFrames[frameID]->SetMaskFromPath(gtFrames[frameID]->GetMaskPath()); //deep copy mask

            //actualKeyframes.push_back(frameID);
        }
    }

    //the new frame set has been generated, and can be used for solving

    cout << "Keyframes: " << " ";

    for(uint32_t i=0; i<actualKeyframes.size();++i)
    {
        cout << actualKeyframes[i] << " ";
    }
    cout << endl;

    return;
}
/* Loads configurations from a file for running the experiment. The file format is fixed to:
 *
 * ----------------------------------------------------------------
 * Project: ../../../src/utils/general/testdata1/trijumpSDGT.xml
 *
 * OutDir: solverTuner_out
 *
 * SolverName: NSKPSolver
 *
 * NumKeyframes: 3
 * KeyframeNumbers: -1 -1 -1
 *
 * TestParameter: jointCoeff
 * ParameterMin: 0.0
 * ParameterMax: 1.0
 * ParameterStep: 0.1
 *
 * BalanceParameter: imageCoeff
 *
 * NumOtherParams: 3
 * useCSdet 0.1
 * useHoGdet 1.0
 * num_trials 100
 *
 * ----------------------------------------------------------------
 *
 * Project: - path to the project to load (relative or absolute)
 * OutDir: - path to output director
 * SolverName: - name of the solver to run experiments on
 * NumKeyframes: - number of keyframes to use
 * KeyframeNumbers: - which frames should be keyframes, -1 for automatic selection based on MST
 * TestParameter: - the parameter to test ()
 * ParameterMin: - starting parameter value
 * ParameterMax: - end parameter value
 * ParameterStep: - parameter step size
 * BalanceParameter: - parameter to balance the test parameter with, such that if testing between 0 and 1 with step 0.1
 * then the balance parameter coefficient will be 1.0-param value at every step.
 * NumOtherParams: - the number of other parameters/settings to pass to the tuner. The parameters themselves are
 * found below, one per line, in the format:
 * [PARAM NAME] [PARAM VALUE]
 * Acceptable parameters to pass include any SPEL detector/solver parameters, as well as num_trials
 *
 * num_trials is a special parameter that indicates the number of trials that the results should be averaged over. This is
 * most useful when running tests that have an element of randomness (e.g. random input/mask noise).
 *
 * Input: path to configuration file (configFilePath)
 *
 * Output: default configs (map<string,float>)
 */


int main (int argc, char **argv)
{
#if defined(MEMORY_DEBUG) && defined(UNIX)
    Debug(libcw_do.on());
    Debug(dc::malloc.on());
#endif  // MEMORY_DEBUG && UNIX
    if (argc != 2)
    {
        cout << "Usage solverTuner [settings file]" << endl;
        return -1;
    }

    // ///////////////////////////////////////////SET-UP BLOCK//////////////////////////////////////////////////////////////////////////////////////
    // //////////////////////////////////////////////START//////////////////////////////////////////////////////////////////////////////////////////

    SpelObject::setDebugLevel(0);

    map<string,float> defaultParams;
    //readFile
    auto startSetup = chrono::steady_clock::now();

    string configFilePath = argv[1];
    //do file parsing
    ifstream in(configFilePath);

    string gtProj, outFold, paramName, balanceParamName, detectorName;
    float param_min, param_max, param_step;
    int numParams, numKeyframes; //number of extra params to include
    vector<int> requestedKeyframes;

    try
    {
        string temp;
        in >> temp >> gtProj >> temp >> outFold >> temp >> detectorName >> temp >> numKeyframes >> temp;

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
        cerr << "Something is wrong in the input file " << configFilePath << " please check format. Exception " << e << endl;
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

    ImagePixelSimilarityMatrix ism;
    string ismFile("ism/"+gtLoader.getProjectTitle()+".ism");
    if(!ism.read(ismFile))
    {
        ism.buildImageSimilarityMatrix(gtLoader.getFrames());
        ism.write(ismFile);
        exit(0); //terminate
    }

    string baseOutFolder(outFold);
    gtLoader.CreateDirectorySystemIndependent(baseOutFolder);
    baseOutFolder += "/" + paramName + "/";
    gtLoader.CreateDirectorySystemIndependent(baseOutFolder);
    baseOutFolder+="/" + detectorName + "/";
    gtLoader.CreateDirectorySystemIndependent(baseOutFolder);
    baseOutFolder+= gtLoader.getProjectTitle() +"/";
    gtLoader.CreateDirectorySystemIndependent(baseOutFolder);

    ofstream out(baseOutFolder+"/"+paramName+"_"+detectorName+"_"+gtLoader.getProjectTitle()+".err");

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
    out << "DetectorName: " << detectorName << endl; //solver name
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

    float mean=param_min, max=param_max, sd=param_step;

    //now print this matrix to file
    //paralellize this

    float ismMean = ism.mean();
    float ismSd = ism.stddev();
    float ismMin = ism.min();

    cout << "ISM Mean " << ismMean << " sd " << ismSd << " min " << ismMin << endl;

    cout << "The min is " << (ismMean-ismMin)/ismSd << " deviations away from mean. " << endl;
    cout << "One sd is " << ismSd/ismMin << " of min." << endl;

    auto endSetup = chrono::steady_clock::now();

    // Store the time difference between start and end
    auto diffSetup = endSetup - startSetup;

    cout << "Set-up finished in " << chrono::duration <double, milli> (diffSetup).count()  << " ms" << endl;

    defaultParams.emplace("useCSdetMult", 1.0);
    defaultParams.emplace("useHoGdetMult", 1.0);
    defaultParams.emplace("useSURFdetMult", 1.0);
    defaultParams.emplace("num_trials", 1); //set the default number of trials to run to 1

    //    float param_min, param_max, param_step;

    //    param_min = defaultParams.at("param_min");
    //    param_max = defaultParams.at("param_max");
    //    param_step = defaultParams.at("param_step");

    int num_trials = defaultParams.at("num_trials"); //num_trials defines how many independent trials should be done and results averaged over.

    // ///////////////////////////////////////////SET-UP BLOCK//////////////////////////////////////////////////////////////////////////////////////
    // ///////////////////////////////////////////////END///////////////////////////////////////////////////////////////////////////////////////////

    for(float param = param_min; param<param_max+param_step; param+=param_step) //do 100 trials for gaussian noise
    {
        cout << paramName << " at " << param << " started..." << endl;
        auto start = chrono::steady_clock::now();

        //vector<vector<float> > frameErrorsTrial; //frame errors across the whole trial
        for(int trial=0; trial<num_trials; ++trial)
        {

            vector<Frame*> vFrames;

            //create the test sequence for this set
            createTestSequence(gtFrames, vFrames, ism, requestedKeyframes, defaultParams);

            map <string, float> params=defaultParams; //transfer the default params

            float simThreshD = 1.0+3.0*ismSd/ismMin;

            params.emplace("mstThresh", simThreshD); //set similarity as multiple of minimum, MUST be >=1

            //test sequence creation block start

            //test sequence creation block end
            if(paramName=="keyframeNoise") //if we're testing gaussian noise
            {
                addKeyframeNoise(vFrames, 0, param, 1000); //mean = min, sd = step, max = max
            }

            //emplace general defaults
            params.emplace(paramName, param); //emplace the param we're testing
            if(balanceParamName!="none") //if there is a balance param, emplace it too
                params.emplace(balanceParamName, 1.0-param); //it will be 1.0-paramValue

            params.emplace("grayImages", 1); // use grayscale images for HoG?
            params.emplace("searchDistCoeff", 1.5); //use a larger default search radius
            params.emplace("searchStepCoeff", 0.5); //use a smaller search step
            params.emplace("baseRotationStep", 10); //use base 10 degrees rotation step
            params.emplace("baseRotationRange", 40); //use base 90 degrees rotation range

            params.emplace("maxTheta", params.at("baseRotationRange"));
            params.emplace("minTheta", params.at("baseRotationRange"));
            params.emplace("stepTheta", params.at("baseRotationStep"));

            params.emplace("useSURFdet", 0.0);
            params.emplace("useCSdet", 0.0);
            params.emplace("useHoGdet", 0.0);

            //now apply the multipliers, they are 1.0 by default
            params.at("useSURFdet") = params.at("useSURFdet")*params.at("useSURFdetMult");
            params.at("useCSdet") = params.at("useCSdet")* params.at("useCSdetMult");
            params.at("useHoGdet") = params.at("useHoGdet")*params.at("useHoGdetMult");

            params.emplace("maxFrameHeight", 288); //scale to 288p - same size as trijump video seq, for detection
            params.emplace("uniqueLocationCandidates", 360); //no max on location candidates

            //params.emplace("searchDistCoeff", 3); //set search region to huge

            //params.emplace("mstThresh", 3.5); //set the max number of part candidates to allow into the solver

            Sequence seq(0, "test", vFrames);

            seq.estimateUniformScale(params);
            seq.computeInterpolation(params);

            for (auto f : vFrames)
                delete f;
            vFrames.clear();
            vFrames = seq.getFrames();

            //        float lineWidth = (float)vFrames[0]->getImage().rows/210.0;

            //        if(param==param_min || paramName=="numKeyframes") //only draw once, unless we're testing number of keyframes
            //        {
            //            for(uint32_t i=0; i<vFrames.size(); ++i)
            //                gtLoader.drawSkeleton(vFrames[i], baseOutFolder+"/", Scalar(0,0,255), lineWidth);
            //        }

            vector<map<uint32_t, vector<LimbLabel> > > labels; //used for detetor tuning
            vector<Frame*> trimmed; //trim the sequence by removing starting and trailing non-keyframes


            vector<Detector*> detectors;
            if(params.at("useCSdet")!=0)
            {
                Detector * d = new ColorHistDetector();
                detectors.push_back(d);
            }
            if(params.at("useHoGdet")!=0)
            {
                Detector * d = new HogDetector();
                detectors.push_back(d);
            }
            if(params.at("useSURFdet")!=0)
            {
                Detector * d = new SurfDetector();
                detectors.push_back(d);
            }

            bool firstKeyframeSeen = false;

            vector<Frame*> temp;
            for(uint32_t i=0; i<vFrames.size(); ++i)
            {
                if(vFrames[i]->getFrametype()==KEYFRAME)
                {
                    //set the flag to true
                    firstKeyframeSeen=true;

                    //push back this keyframe at the end
                    temp.push_back(vFrames[i]);

                    //go through all temp and push them back
                    for(uint32_t j=0; j<temp.size(); ++j)
                        trimmed.push_back(temp[j]);
                    //clear out temp
                    temp.clear();
                }
                else if(firstKeyframeSeen) //if not keyframe, just push to temp vector
                {
                    temp.push_back(vFrames[i]);
                }
            }
            //trimmed.push_back(temp[0]);

            //finally, generate new masks and replace path to them in trimmed sequence if needed
            if(paramName=="maskNoise")
            {
                for(auto i=0; i<trimmed.size(); ++i)
                {
                    //for each frame in the trimmed sequence
                    addGaussianMaskNoise(trimmed.at(i));
                }
            }

            auto endSeqBuild = chrono::steady_clock::now();

            // Store the time difference between start and end
            auto diffSeqBuild = endSeqBuild - start;

            cout << "\tSequence built in " << chrono::duration <double, milli> (diffSeqBuild).count()  << " ms" << endl;

            cout << "\tTraining detector..." <<endl;

            if(detectorName=="interpolationDetect") //then we are just doing a detector test!
            {
                vector<Frame*> trainingFrames;
                trainingFrames.push_back(trimmed[0]);
                trainingFrames.push_back(trimmed[trimmed.size()-1]);
                //train detectors
                for(uint32_t i=0; i<detectors.size(); ++i)
                    detectors[i]->train(trainingFrames, params);

                auto endTrain = chrono::steady_clock::now();

                // Store the time difference between start and end
                auto diffTrain = endTrain - endSeqBuild;

                cout << "\tDetectors trained in " << chrono::duration <double, milli> (diffTrain).count()  << " ms" << endl;

                cout << "\tDetecting..." << endl;
                labels = doInterpolationDetect(detectors, trimmed, params);

                auto endDetect = chrono::steady_clock::now();

                // Store the time difference between start and end
                auto diffDetect = endDetect - endTrain;

                cout << "\tDetections completed in " << chrono::duration <double, milli> (diffDetect).count()  << " ms" << endl;
            }

            else if(detectorName=="propagationDetect")
            {
                //train detectors
                vector<Frame*> trainingFrames;
                trainingFrames.push_back(trimmed[0]);
                trainingFrames.push_back(trimmed[trimmed.size()-1]);
                //train detectors
                for(uint32_t i=0; i<detectors.size(); ++i)
                    detectors[i]->train(trainingFrames, params);

                auto endTrain = chrono::steady_clock::now();

                // Store the time difference between start and end
                auto diffTrain = endTrain - endSeqBuild;

                cout << "\tDetectors trained in " << chrono::duration <double, milli> (diffTrain).count()  << " ms" << endl;

                cout << "\tDetecting..." << endl;
                labels = doPropagationDetect(detectors,trimmed, ism, params);

                auto endDetect = chrono::steady_clock::now();

                // Store the time difference between start and end
                auto diffDetect = endDetect - endTrain;

                cout << "\tDetections completed in " << chrono::duration <double, milli> (diffDetect).count()  << " ms" << endl;
            }

            //begin writing to file block - DO THIS ONLY IF WE ARE DOING A SINGLE CYCLE
            if(num_trials==1)
            {
                out << param << endl;
                out << "{" << endl;
                for(uint32_t l=0; l< labels.size(); ++l)
                {
                    out << "\t" << trimmed[l]->getID() << endl;
                    out << "\t[" << endl;
                    //for every frame
                    auto frameLabels = labels[l]; //frame labels
                    vector<vector<float> > frameErrors; //frame errors (at this particular frame)
                    frameErrors = computeLabelErrorStatToGT(frameLabels, gtFrames[l]);

                    for(uint32_t row=0; row<frameErrors.size(); ++row)
                    {
                        vector<LimbLabel> partLabels = frameLabels[row];
                        if(partLabels.size()>0)
                        {
                            out << "\t\t" << partLabels[0].getLimbID() << endl;
                            out << "\t\t(" << endl;
                            vector<float> partErrors = frameErrors[row]; //errors for a part

                            BodyPart * bp = vFrames[l]->getSkeleton().getBodyPart(partLabels[0].getLimbID());
                            float partWidth = bp->getRelativeLength()/bp->getLWRatio()*vFrames[l]->getSkeleton().getScale();
                            float acceptThresh=025;

                            for(uint32_t e=0; e<partErrors.size(); ++e)
                            {
                                //this is a row in the output file
                                //rank labelScore labelError

                                string hogName = "18500";
                                string csName = "4409412";
                                string surfName = "21316";

                                vector<Score> scores = partLabels[e].getScores();

                                float useHoG = params.at("useHoGdet");
                                float useCS = params.at("useCSdet");
                                float useSURF = params.at("useSURFdet");

                                //compute the weighted sum of scores
                                float finalScore=0;
                                bool hogFound=false;
                                bool csFound=false;
                                bool surfFound=false;
                                float csScore, hogScore, surfScore;
                                for(uint32_t i=0; i<scores.size(); ++i)
                                {
                                    float score = scores[i].getScore();
                                    if(scores[i].getScore()==-1)//if score is -1, set it to 1
                                    {
                                        score=1.0; //set a high cost for invalid scores
                                    }
                                    if(scores[i].getDetName()==hogName)
                                    {
                                        hogScore = score;
                                        hogFound=true;
                                    }
                                    else if(scores[i].getDetName()==csName)
                                    {
                                        csScore = score;
                                        csFound=true;
                                    }
                                    else if(scores[i].getDetName()==surfName)
                                    {
                                        surfScore=score;
                                        surfFound=true;
                                    }
                                }
                                if(!csFound && useCS)
                                    csScore=1.0;
                                if(!hogFound && useHoG)
                                    hogScore=1.0;
                                if(!surfFound && useSURF)
                                    surfScore=1.0;

                                finalScore=csScore*useCS+hogScore*useHoG+surfScore*useSURF;
                                //float avgScore = partLabels[e].getAvgScore();

                                //                        if(row==15)
                                //                            partLabels[0] > partLabels[1];


                                //now add 1.0*coeff for each not found score in this label, that should have been there (i.e., assume the worst)
                                //finalScore+=1.0*useHoG*(!hogFound)+1.0*useCS*(!csFound)+1.0*useSURF*(!surfFound);

                                vector<Point2f> poly = partLabels[e].getPolygon();
                                bool isAccepted = false;
                                if(e <= partWidth*acceptThresh)
                                    isAccepted = true;

                                out << "\t\t\t" << e << " " << finalScore << " " << partErrors[e] << " " << isAccepted << " " << csScore*useCS << " " <<  hogScore*useHoG << " " << surfScore*useSURF << " ";

                                for(auto i=0; i<poly.size();++i)
                                    out << poly[i].x << " " << poly[i].y << " ";
                                out << endl;
                            }

                            out << "\t\t)" << endl;
                        }

                    }

                    out << "\t]" << endl;
                }
                out << "}" << endl;
            }
             //end of a trial solve
        }
        //end of a param solve
        //draw interpolation

        auto end = chrono::steady_clock::now();

        // Store the time difference between start and end
        auto diff = end - start;

        cout << paramName << " at " << param << " finished in " << chrono::duration <double, milli> (diff).count()  << " ms" << endl;
    }
    out.close();

//        for(auto i=0; i<gtFrames.size(); ++i)
//            delete gtFrames[i];
        gtFrames.clear();

        cout << "Tuning finished, terminating. " << endl;
#if defined(MEMORY_DEBUG) && defined(UNIX)
        Debug(list_allocations_on(libcw_do));
#endif  // MEMORY_DEBUG && UNIX
        return 0;
    }
