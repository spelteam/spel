/*
 * Parameter names that can be tested:
 *
 * maskNoise - test how the solvers cope with noisy masks
 * keyframeNoise - test how solvers cope with noisy user input (keyframe labels)
 * numKeyframes - test how solvers cope with different absolute numbers of keyframes [USE PERCENT KEYFRAMES FOR BETTER COMPARISON METRICS]
 * Solver - test how different solvers perform in different situations (solver types include: NSKPSolver, TLPSSolvers, hybridSolver, Interpolation)
 * percentKeyframes - test how different solvers perform with a different percentage of the image sequence being keyframed
 *
 */

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
#include <chrono>
#ifdef WINDOWS
#include <random>
#endif

using namespace std;
using namespace SPEL;

Mat computeErrorToGT(const vector<Solvlet>& solves, vector<Frame*> keyframes) //return squared error from solve to GT
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
            float d0 = spelHelper::distSquared(p0,t0); //dist between parent joints
            float d1 = spelHelper::distSquared(p1,t1); //dist between child joints

            float error = 0.5*d0+0.5*d1; //average error for the two joints
            errorMatrix.at<float>(i, partIter->getPartID()) = error;
        }
    }

    return errorMatrix;
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

vector<Frame*> generateTestFramesPrecise(vector<Frame*> gtFrames, map<string,float> params, vector<std::pair<int, int>> suggestedKeyframes, vector<int> requestedKeyframes, int maxKeyframes)
{

    vector<int> actualKeyframes;
    vector<Frame*> vFrames;
    //init
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
            if(frameID<gtFrames.size())
            {
                delete vFrames[frameID];
                vFrames[frameID] = new Keyframe(); //assign new keyframe
                //copy all the data

                vFrames[frameID]->setSkeleton(gtFrames[frameID]->getSkeleton());
                vFrames[frameID]->setID(gtFrames[frameID]->getID());
                vFrames[frameID]->SetImageFromPath(gtFrames[frameID]->GetImagePath());
                vFrames[frameID]->SetMaskFromPath(gtFrames[frameID]->GetMaskPath());

                actualKeyframes.push_back(frameID);
            }
        }
        if(actualKeyframes.size()>=maxKeyframes)
            break; //sot adding keyuframes if we're at max, during the keyframe numbers test
    }

    //insert the automatically suggested keyframes, if any
    //        int numKeyframesToTake = requestedKeyframes.size();
    //        if(solverName=="TLPSSolver")
    //            numKeyframesToTake = 4; //take the int that is 1/4th of the num of keyframes
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

                    delete vFrames[frameID]; //free memory
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
        if(actualKeyframes.size()>=maxKeyframes)
            break; //stop adding keyframes if we're at max, during the keyframe numbers test
    }

    //fill in the rest with interpolation frames
    for(uint32_t i=0; i<vFrames.size(); ++i)
    {
        if(vFrames[i]->getFrametype()!=KEYFRAME) //if not keyframes
        {
            int frameID=i;
            delete vFrames[frameID]; //free memory
            vFrames[frameID] = new Interpolation(); //assign new interpolation frame, without a skeleton
            //copy all the data
            \
            vFrames[frameID]->setID(gtFrames[frameID]->getID());
            vFrames[frameID]->SetImageFromPath(gtFrames[frameID]->GetImagePath()); //deep copy image
            vFrames[frameID]->SetMaskFromPath(gtFrames[frameID]->GetMaskPath()); //deep copy mask

            //actualKeyframes.push_back(frameID);
        }
    }
    return vFrames;
}
vector<Frame*> generateTestFramesPercent(vector<Frame*> gtFrames, map<string,float> params, vector<std::pair<int, int>> suggestedKeyframes)
{

    vector<int> actualKeyframes;
    vector<Frame*> vFrames;
    //init
    for(uint32_t i=0; i<gtFrames.size(); ++i) //init
    {
        Frame * fp = new Interpolation();
        vFrames.push_back(fp);
    }

    float percentKeyframes=params.at("percentKeyframes");
    uint32_t maxNumKeyframes=(float)gtFrames.size()/100.0*percentKeyframes;

    if(params.at("useOptimalKeyframes")) //use the predictive keyframes
    {
        //for each suggested keyframe, find the first one that isn't yet in the list of keyframes
        for(auto fi=suggestedKeyframes.begin(); fi!=suggestedKeyframes.end(); ++fi)
        {
            int frameID=fi->first;

            if(frameID<vFrames.size())
            {
                delete vFrames[frameID]; //free memory
                vFrames[frameID] = new Keyframe(); //assign new keyframe
                //copy all the data
                vFrames[frameID]->setSkeleton(gtFrames[frameID]->getSkeleton());
                vFrames[frameID]->setID(gtFrames[frameID]->getID());
                vFrames[frameID]->SetImageFromPath(gtFrames[frameID]->GetImagePath());
                vFrames[frameID]->SetMaskFromPath(gtFrames[frameID]->GetMaskPath());

                actualKeyframes.push_back(frameID);
            }
        }
        //always add one in the end and the start
        if(vFrames[0]->getFrametype()!=KEYFRAME) //if not a keyframe yet, make it one
        {
            delete vFrames[0];
            vFrames[0] = new Keyframe(); //assign new keyframe
            //copy all the data

            vFrames[0]->setSkeleton(gtFrames[0]->getSkeleton());
            vFrames[0]->setID(gtFrames[0]->getID());
            vFrames[0]->SetImageFromPath(gtFrames[0]->GetImagePath());
            vFrames[0]->SetMaskFromPath(gtFrames[0]->GetMaskPath());

            actualKeyframes.push_back(0);
        }
        //always add one at the end (extra)
        if(vFrames[vFrames.size()-1]->getFrametype()!=KEYFRAME) //if not a keyframe yet, make it one
        {
            delete vFrames[vFrames.size()-1];
            vFrames[vFrames.size()-1] = new Keyframe(); //assign new keyframe
            //copy all the data

            vFrames[vFrames.size()-1]->setSkeleton(gtFrames[vFrames.size()-1]->getSkeleton());
            vFrames[vFrames.size()-1]->setID(gtFrames[vFrames.size()-1]->getID());
            vFrames[vFrames.size()-1]->SetImageFromPath(gtFrames[vFrames.size()-1]->GetImagePath());
            vFrames[vFrames.size()-1]->SetMaskFromPath(gtFrames[vFrames.size()-1]->GetMaskPath());

            actualKeyframes.push_back(vFrames.size()-1);
        }
        //what if we didn't get enough keyframes to cover that percentage?

        while(actualKeyframes.size()<maxNumKeyframes) //now fill in the rest by subdivision
        {
            sort(actualKeyframes.begin(), actualKeyframes.end()); //sort the keyframes

            //now select spot for new keyframe
            uint32_t maxGapSize=0;
            uint32_t maxGapIndex=-1;
            for(auto f=1; f<actualKeyframes.size();++f)
            {
                uint32_t gapSize=abs(actualKeyframes[f]-actualKeyframes[f-1]);

                if(gapSize>maxGapSize)
                {
                    maxGapSize=gapSize;
                    maxGapIndex=actualKeyframes[f-1]+gapSize/2; //previous keyframe plus hafl the gap size
                }
            }
            if(maxGapIndex!=-1)
            {
                delete vFrames[maxGapIndex];
                vFrames[maxGapIndex] = new Keyframe(); //assign new keyframe
                //copy all the data

                vFrames[maxGapIndex]->setSkeleton(gtFrames[maxGapIndex]->getSkeleton());
                vFrames[maxGapIndex]->setID(gtFrames[maxGapIndex]->getID());
                vFrames[maxGapIndex]->SetImageFromPath(gtFrames[maxGapIndex]->GetImagePath());
                vFrames[maxGapIndex]->SetMaskFromPath(gtFrames[maxGapIndex]->GetMaskPath());

                actualKeyframes.push_back(maxGapIndex);
            }
        }
    }
    else //otherwise, systematically assign keyframes to sequence, such that they are evenly distributed
    {
        int keyframeStep=float(gtFrames.size())/maxNumKeyframes; //frames/numKeyframes = step size?so if it's 2, then every 2nd frame is a keyframe

        for(auto frameID=0; frameID<vFrames.size();frameID+=keyframeStep)
        {
            delete vFrames[frameID];
            vFrames[frameID] = new Keyframe(); //assign new keyframe
            //copy all the data

            vFrames[frameID]->setSkeleton(gtFrames[frameID]->getSkeleton());
            vFrames[frameID]->setID(gtFrames[frameID]->getID());
            vFrames[frameID]->SetImageFromPath(gtFrames[frameID]->GetImagePath());
            vFrames[frameID]->SetMaskFromPath(gtFrames[frameID]->GetMaskPath());

            actualKeyframes.push_back(frameID);
        }
        //also, the last frame is always a keyframe (it's an extra keyframe)
        if(vFrames[vFrames.size()-1]->getFrametype()!=KEYFRAME) //if not a keyframe yet, make it one
        {
            delete vFrames[vFrames.size()-1];
            vFrames[vFrames.size()-1] = new Keyframe(); //assign new keyframe
            //copy all the data

            vFrames[vFrames.size()-1]->setSkeleton(gtFrames[vFrames.size()-1]->getSkeleton());
            vFrames[vFrames.size()-1]->setID(gtFrames[vFrames.size()-1]->getID());
            vFrames[vFrames.size()-1]->SetImageFromPath(gtFrames[vFrames.size()-1]->GetImagePath());
            vFrames[vFrames.size()-1]->SetMaskFromPath(gtFrames[vFrames.size()-1]->GetMaskPath());

            actualKeyframes.push_back(vFrames.size()-1);
        }
    }
    //create keyframes as a percentage of the sequence total number of frames, based on param

    //now fill the rest with interpolation frames that don't contain the ground truth skeletons
    for(uint32_t i=0; i<vFrames.size(); ++i)
    {
        if(vFrames[i]->getFrametype()!=KEYFRAME) //if not keyframes
        {
            int frameID=i;
            //delete vFrames[requestedKeyframes[i]]; //free memory
            delete vFrames[frameID];
            vFrames[frameID] = new Interpolation(); //assign new interpolation frame, without a skeleton
            //copy all the data
            \
            vFrames[frameID]->setID(gtFrames[frameID]->getID());
            vFrames[frameID]->SetImageFromPath(gtFrames[frameID]->GetImagePath()); //deep copy image
            vFrames[frameID]->SetMaskFromPath(gtFrames[frameID]->GetMaskPath()); //deep copy mask

            //actualKeyframes.push_back(frameID);
        }
    }
    return vFrames;
}

vector<Solvlet> solvletsFromSkeleton(vector<Frame*> vFrames)
{
    vector<Solvlet> solvlets;

    for(auto&& frame: vFrames)
    {
        Solvlet solvlet;
        vector<LimbLabel> solvletLabels;
        Skeleton skel=frame->getSkeleton();
        tree<BodyPart> partTree=skel.getPartTree();
        //generate a label for each part
        for(tree<BodyPart>::iterator part=partTree.begin(); part!=partTree.end(); ++part)
        {
            Point2f j0, j1;
            try
            {
                j0 = skel.getBodyJoint(part->getParentJoint())->getImageLocation(); // copy current bodypart parent joint
                j1 = skel.getBodyJoint(part->getChildJoint())->getImageLocation(); // copy current bodypart child joint
            }
            catch (...)
            {
                stringstream ss;
                ss << "Can't get joints";
                throw logic_error(ss.str());
            }

            auto ratio = part->getLWRatio();
            auto boneLength = sqrt(spelHelper::distSquared(j0, j1)); // distance between nodes
            float boxWidth= boneLength / ratio;
            auto boxCenter=0.5*j0+0.5*j1; //the midpoint

            auto angle = static_cast <float> (spelHelper::angle2D(1.0f, 0.0f, j1.x - j0.x, j1.y - j0.y) * (180.0 / M_PI));
            Point2f c1, c2, c3, c4, polyCenter;
            c1 = Point2f(0.f, 0.5f * boxWidth);
            c2 = Point2f(boneLength, 0.5f * boxWidth);
            c3 = Point2f(boneLength, -0.5f * boxWidth);
            c4 = Point2f(0.f, -0.5f * boxWidth);

            c1 = spelHelper::rotatePoint2D(c1, Point2f(0, 0), angle);
            c2 = spelHelper::rotatePoint2D(c2, Point2f(0, 0), angle);
            c3 = spelHelper::rotatePoint2D(c3, Point2f(0, 0), angle);
            c4 = spelHelper::rotatePoint2D(c4, Point2f(0, 0), angle);

            polyCenter = 0.25*c1 + 0.25*c2 + 0.25*c3 + 0.25*c4;

            c1 = c1 - polyCenter + boxCenter;
            c2 = c2 - polyCenter + boxCenter;
            c3 = c3 - polyCenter + boxCenter;
            c4 = c4 - polyCenter + boxCenter;

            vector<Point2f> poly={c1, c2, c3, c4};

            solvletLabels.push_back(LimbLabel(part->getPartID(), boxCenter, angle, poly, vector<Score>(), false));
        }
        solvlet.setLabels(solvletLabels);
    }

    return solvlets;
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

int main (int argc, char **argv)
{

    SpelObject::setDebugLevel(0);
#if defined(MEMORY_DEBUG) && defined(UNIX)
    Debug(libcw_do.on());
    Debug(dc::malloc.on());
#endif  // MEMORY_DEBUG && UNIX
    if (argc != 2)
    {
        cout << "Usage solverTuner [settings file]" << endl;
        return -1;
    }

    auto startSetup = chrono::steady_clock::now();

    //do file parsing
    ifstream in(argv[1]);

    string gtProj, outFold, paramName, balanceParamName, solverName;
    float param_min, param_max, param_step;
    uint32_t numParams, numKeyframes; //number of extra params to include
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
        for(auto i=0; i<numKeyframes; ++i)
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

    ImagePixelSimilarityMatrix ism;
    string ismFile("ism/"+gtLoader.getProjectTitle()+".ism");
    if(!ism.read(ismFile))
    {
        ism.buildImageSimilarityMatrix(gtLoader.getFrames());
        ism.write(ismFile);
        exit(0); //terminate
    }

    //    float ismMean = ism.mean();
    //    float ismSd = ism.stddev();
    //    float ismMin = ism.min();

    //    cout << "ISM Mean " << ismMean << " sd " << ismSd << " min " << ismMin << endl;

    //    cout << "The min is " << (ismMean-ismMin)/ismSd << " deviations away from mean. " << endl;
    //    cout << "One sd is " << ismSd/ismMin << " of min." << endl;

    //    float numSdMinToMean=(ismMean-ismMin)/ismSd;
    //    float sdPartMin = ismSd/ismMin;
    //    float sdFactor=0.47;//go up half-way to mean from min
    //    float simThreshD = 1.0+sdFactor*numSdMinToMean*sdPartMin;

    //    cout << "Setting simThresh to " << simThreshD << endl;

    //    defaultParams.emplace("mstThresh", simThreshD); //set similarity as multiple of minimum, MUST be >=1

    string baseOutFolder(outFold);
    gtLoader.CreateDirectorySystemIndependent(baseOutFolder);
    baseOutFolder += "/" + paramName + "/";
    //gtLoader.CreateDirectorySystemIndependent(baseOutFolder);
    //baseOutFolder+="/" + solverName + "/";
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


    float mean=param_min, max=param_max, sd=param_step;

    //    if(paramName=="keyframeNoise") //will always do 100 trials
    //    {
    //        param_min=0;
    //        param_max=100;
    //        param_step=1;
    //    }

    //now print this matrix to file
    //paralellize this


    auto endSetup = chrono::steady_clock::now();

    // Store the time difference between start and end
    auto diffSetup = endSetup - startSetup;

    cout << " Set-up finished in " << chrono::duration <double, milli> (diffSetup).count()  << " ms" << endl;

    //'2Dint', '3Dint', 'tlpsOnly', 'tlpsNSKP', 'fullHybrid'
    vector<string> solverNames;

    solverNames.push_back("NSKPSolver");
    solverNames.push_back("TLPSSolver");
    solverNames.push_back("hybridSolver");
    //solverNames.push_back("3Dint");
    //solverNames.push_back("2Dint");

    defaultParams.emplace("num_trials", 1); //set the default number of trials to run to 1

    int num_trials = defaultParams.at("num_trials"); //num_trials defines how many independent trials should be done and results averaged over.

    for(float param = param_min; param<param_max+param_step; param+=param_step) //do 100 trials for gaussian noise
    {
        vector<Mat> crossTrialErrors;
        //Sequence lastSequence; //store the last used sequence here

        uint32_t maxKeyframes=UINT_MAX;
        if(paramName=="numKeyframes")
        {
            maxKeyframes = param;
        }
        if(paramName=="Solver")
        {
            solverName=solverNames.at(param);
            if(param>=solverNames.size())
                break;//end the loop if we're over the total number of solvers
        }

        map <string, float> params=defaultParams; //transfer the default params

        //emplace general defaults
        params.emplace(paramName, param); //emplace the param we're testing
        if(balanceParamName!="none") //if there is a balance param, emplace it too
            params.emplace(balanceParamName, 1.0-param); //it will be 1.0-paramValue

        //global settings
        params.emplace("imageCoeff", 0.2); //set solver detector infromation sensitivity
        params.emplace("jointCoeff", 0.8); //set solver body part connectivity sensitivity
        params.emplace("priorCoeff", 0.0); //set solver distance to prior sensitivity
        params.emplace("tempCoeff", 0.05); //set the temporal connection sensitivity for TLPS

        //detector settings
        params.emplace("useCSdet", 0.1); //determine if ColHist detector is used and with what coefficient
        params.emplace("useHoGdet", 9.0); //determine if HoG descriptor is used and with what coefficient
        params.emplace("useSURFdet", 0.0); //determine whether SURF detector is used and with what coefficient

        params.emplace("grayImages", 1); // use grayscale images for HoG?

        //solver settings
        params.emplace("nskpIters", 0); //do as many NSKP iterations as is useful at each run
        params.emplace("nskpLockframeThreshold", 0.52); // 0.52 set the threshold for NSKP and TLPSSolvers, forcing TLPS to reject some solutions

        if(solverName=="hybridSolver")
            params.emplace("tlpsLockframeThreshold", 0.49); // 0.52 set the threshold for NSKP and TLPSSolvers, forcing TLPS to reject some solutions
        else
            params.emplace("tlpsLockframeThreshold", 0.0); // 0.52 set the threshold for NSKP and TLPSSolvers, forcing TLPS to reject some solutions

        params.emplace("badLabelThresh", 0.40); //set bad label threshold, which will force solution discard at 0.45
        params.emplace("partDepthRotationCoeff", 1.25); //search radius increase for each depth level in the part tree

        params.emplace("anchorBindDistance", 0); //restrict search regions if within bind distance of existing keyframe or lockframe (like a temporal link
        params.emplace("anchorBindCoeff", 0.0); //multiplier for narrowing the search range if close to an anchor (lockframe/keyframe)
        params.emplace("bindToLockframes", 0); //should binds be also used on lockframes?
        params.emplace("maxFrameHeight", 288); //scale to 288p - same size as trijump video seq, for detection

        params.emplace("grayImages", 1); // use grayscale images for HoG?
        params.emplace("searchDistCoeff", 5.0); //use a larger default search radius of 3.5 widths
        params.emplace("searchStepCoeff", 0.2); //use a smaller search step
        params.emplace("baseRotationStep", 10); //use base 10 degrees rotation step
        params.emplace("baseRotationRange", 40); //use base 80 degrees rotation range

        params.emplace("minTheta", params.at("baseRotationRange"));
        params.emplace("stepTheta", params.at("baseRotationStep"));

        params.emplace("maxPartCandidates", 0.1); //Take the top 5% of candidates
        params.emplace("isWeakThreshold", 0.0); //if SD is less than 0.3 of mean-min
        params.emplace("uniqueLocationCandidates", 0.3);
        params.emplace("uniqueAngleCandidates", 0.3);
        params.emplace("debugLevel", 0);

        params.emplace("mstThresh", 3.0); //use outliners that are at least 2.5 stddev from the mean
        params.emplace("withTLPS", 0); //don't use TLPS after NSKP by default
        params.emplace("hybridIters", 1);

        params.emplace("useOptimalKeyframes", 1); //use optimal keyframes by default
        params.emplace("percentKeyframes", 30); //do a 10% keyframe solve by default

        //generate keyframe suggestions
        auto startSuggestions = chrono::steady_clock::now();
        cout << "Generating keyframe suggestions... "  << endl;
        auto suggestedKeyframes = NSKPSolver().suggestKeyframes(ism, params);
        auto start = chrono::steady_clock::now();

        auto diffSuggestions = start-startSuggestions;

        cout << "Suggestions generated in " << chrono::duration <double, milli> (diffSuggestions).count()  << " ms" << endl;

        //build the sequence based on
        vector <Frame*> vFrames;

        vector<Solvlet> fSolve_last;

        for(auto trial=0; trial<num_trials; ++trial)
        {
            cout << "Solving with " << paramName << " at " << param << endl;
            cout << "Building test sequence... " << paramName << " at " << param << endl;

            if(paramName=="percentKeyframes" || paramName=="Solver")
            {
                vFrames = generateTestFramesPercent(gtFrames, params, suggestedKeyframes);
            }
            else
            {
                vFrames = generateTestFramesPrecise(gtFrames, params, suggestedKeyframes, requestedKeyframes, maxKeyframes);
            }


            //the new frame set has been generated, and can be used for solving


            //        cout << "Keyframes: " << " ";
            //        for(uint32_t i=0; i<actualKeyframes.size();++i)
            //        {
            //            cout << actualKeyframes[i] << " ";
            //        }
            //        cout << endl;

            if(paramName=="keyframeNoise") //if we're testing gaussian noise
            {
                addKeyframeNoise(vFrames, mean, sd, max); //mean = min, sd = step, max = max
            }

            if(paramName=="maskNoise")
            {
                for(auto i=0; i<vFrames.size(); ++i)
                {
                    //for each frame in the trimmed sequence
                    addGaussianMaskNoise(vFrames.at(i));
                }
            }

            Sequence seq(0, "test", vFrames);
            //lastSequnece = seq;

            //delete vFrames - we only need the sequence now
            //        for(auto f=0; f<vFrames.size(); ++f)
            //            delete vFrames[f];
            //        vFrames.clear();

            seq.estimateUniformScale(params);
            seq.computeInterpolation(params);

            for (auto f : vFrames)
                delete f;
            vFrames.clear();
            vFrames = seq.getFrames();

            NSKPSolver nSolver;
            TLPSSolver tSolver;

            auto endSeqBuild = chrono::steady_clock::now();

            // Store the time difference between start and end
            auto diffSeqBuild = endSeqBuild - start;

            cout << "Sequence built in " << chrono::duration <double, milli> (diffSeqBuild).count()  << " ms" << endl;

            vector<Solvlet> fSolve; //used for solver tuning
            //do an iterative NSKP solve
            if(solverName=="TLPSSolver")
                fSolve = tSolver.solve(seq, params);
            else if(solverName=="NSKPSolver")
                fSolve = nSolver.solve(seq, params, ism);
            else if(solverName=="hybridSolver")
            {
                vector<Solvlet> finalSolve;
                uint32_t prevSolveSize=0;

                int numIters=0;
                do
                {
                    prevSolveSize=finalSolve.size(); //set size of the final solve

                    vector<Solvlet> nskpSolve, tlpsSolve;
                    if(params.at("withNSKP")) //if it's with NSKP, first the NSKP stuff starts, then TLPS gets called inside it
                    {
                        //do an iterative NSKP solve
                        nskpSolve = nSolver.solve(seq, params, ism);

                        for(const auto &s: nskpSolve)
                            finalSolve.push_back(s);

                    }
                    else if(params.at("withTLPS")) //otherwise, only call TLPS
                    {
                        //then, do a temporal solve
                        seq.estimateUniformScale(params);
                        seq.computeInterpolation(params); //recompute interpolation (does this improve results?)

                        tlpsSolve = tSolver.solve(seq, params);

                        for(const auto &s: tlpsSolve)
                            finalSolve.push_back(s);
                    }
                    numIters++;

                } while(finalSolve.size()>prevSolveSize && numIters<params.at("hybridIters")); //don't do more iters than necessary

                //now do the final TLPSSolve
                //then, do a temporal solve
                seq.estimateUniformScale(params);
                seq.computeInterpolation(params); //recompute interpolation (does this improve results?)
                params.at("tlpsLockframeThreshold") = 0.0; //set the threshold to zero, to get solves for the currently unsolved frames

                vector<Solvlet> tlpsSolve = tSolver.solve(seq, params);

                for(const auto &s: tlpsSolve)
                    finalSolve.push_back(s);


                fSolve = finalSolve;
            }
            else if(solverName=="3Dint")
            {
                //generate labels for each frame for each part where there is an interpolation
                params.emplace("interpolate2d", 0); //do 3D interpolation
                seq.computeInterpolation(params);

                vFrames.clear();
                vFrames = seq.getFrames();

                fSolve=solvletsFromSkeleton(vFrames);
            }
            else if(solverName=="2Dint")
            {
                //generate labels for each frame for each part where there is an interpolation
                params.emplace("interpolate2d", 1); //force 2D interpolation
                seq.computeInterpolation(params);

                vFrames.clear();
                vFrames = seq.getFrames();

                fSolve=solvletsFromSkeleton(vFrames);
            }

            //now sort fSolve solvlets
            sort(fSolve.begin(), fSolve.end());
            fSolve_last = fSolve;

            auto endSolve = chrono::steady_clock::now();

            // Store the time difference between start and end
            auto diffSolve = endSolve - endSeqBuild;

            cout << "Sequence solved in " << chrono::duration <double, milli> (diffSolve).count()  << " ms" << endl;

            //now do the error solve and file output logic

            Mat errors; //used for storing errors
            cerr << "Computing Errors to GT..." << endl;
            if(fSolve.size()>0)
                errors = computeErrorToGT(fSolve, gtFrames);
            cerr << "done!" << endl;

            if(paramName=="Solver")
                out << solverNames.at(param) << "  " << chrono::duration <double, milli> (diffSolve).count() << endl;
            else
                out << param << "  " << chrono::duration <double, milli> (diffSolve).count() << endl;
            out << "{" << endl;

            if(num_trials==1) //IF NUMBER OF TRIALS IS 1, PROCEED AS NORMAL
            {
                cout << "Saving errors..." << endl;
                for(auto row=0; row<errors.rows; ++row)
                {
                    cerr << "Processing error row " << row << endl;
                    Frame* frame = seq.getFrames()[fSolve[row].getFrameID()];

                    //this is a row in the output file
                    //frameID evalScore limb1RMS limb2RMS limb3RMS limb4RMS ... limbKRMS
                    out << fSolve[row].getFrameID() << " " << fSolve[row].evaluateSolution(frame, params) << " "; //output frameID and the solution evaluation score
                    vector<LimbLabel> solLabels = fSolve[row].getLabels();
                    for(auto col=0; col<errors.cols; ++col)
                    {
                        //this is an item of the row
                        out << errors.at<float>(row,col) << " ";

                        //output polygon locations after the error, so each part has a 9-tuple - (error, x0, y0, x1, y1, x2, y2, x3, y3)
                        vector<Point2f> poly = solLabels[col].getPolygon();
                        for(auto point=poly.begin(); point!=poly.end(); ++point)
                            out << point->x << " " << point->y << " ";
                    }
                    out << endl; //newline at the end of the block

                    cout << "Drawing solutions..." << endl;
                    if(paramName=="Solver") //draw solutions if we are doing optimal solves
                    {
                        if(frame->getParentFrameID()!=-1)
                        {
                            Frame* parent = seq.getFrames()[frame->getParentFrameID()];
                            gtLoader.drawLockframeSolvlets(ism, fSolve[row], frame, parent, (baseOutFolder+"/"+solverNames[param]+"/").c_str(), Scalar(0,0,255), 2);
                        }
                        else
                            gtLoader.drawFrameSolvlets(fSolve[row], frame, (baseOutFolder+"/"+solverNames[param]+"/").c_str(), Scalar(0,0,255), 2);
                    }
                    cout << "done!" << endl;
                    delete frame;
                }
                cerr << "Error saved!" << endl;

                out << "}" << endl;

                //release errors if they are no longer necessary
                errors.release();
            }
            else //IF NUM TRIALS IS BIGGER THAN 1
            {
                crossTrialErrors.push_back(errors);
            }

            vFrames.clear();
        }
        //now what do we do with all the errors? since they are all one error per-part, we can just average them all, and output in the exact same way as before (only once)
        if(num_trials>1)
        {
            if(paramName=="percentKeyframes" || paramName=="Solver")
            {
                vFrames = generateTestFramesPercent(gtFrames, params, suggestedKeyframes);
            }
            else
            {
                vFrames = generateTestFramesPrecise(gtFrames, params, suggestedKeyframes, requestedKeyframes, maxKeyframes);
            }

            Sequence seq(0, "test", vFrames);

            Mat errors=crossTrialErrors.at(0); //the errors matrix will be the average of all error matrices across this parameter

            for(auto i=1; i<crossTrialErrors.size();++i)
            {
                errors+=crossTrialErrors.at(i);
            }
            //now errors is the sum of all error matrices
            errors = errors/(float)crossTrialErrors.size();

            cout << "Saving errors..." << endl;
            for(auto row=0; row<errors.rows; ++row)
            {
                cerr << "Processing error row " << row << endl;
                Frame* frame = seq.getFrames()[fSolve_last[row].getFrameID()];

                //this is a row in the output file
                //frameID evalScore limb1RMS limb2RMS limb3RMS limb4RMS ... limbKRMS
                out << fSolve_last[row].getFrameID() << " " << fSolve_last[row].evaluateSolution(frame, params) << " "; //output frameID and the solution evaluation score
                vector<LimbLabel> solLabels = fSolve_last[row].getLabels();
                for(auto col=0; col<errors.cols; ++col)
                {
                    //this is an item of the row
                    out << errors.at<float>(row,col) << " ";

                    //output polygon locations after the error, so each part has a 9-tuple - (error, x0, y0, x1, y1, x2, y2, x3, y3)
                    vector<Point2f> poly = solLabels[col].getPolygon();
                    for(auto point=poly.begin(); point!=poly.end(); ++point)
                        out << point->x << " " << point->y << " ";
                }
                out << endl; //newline at the end of the block

                //DON'T DRAW IF SOLVING AVERAGES - IT'S MEANINGLESS
//                cout << "Drawing solutions..." << endl;
//                if(paramName=="Solver") //draw solutions if we are doing optimal solves
//                {
//                    if(frame->getParentFrameID()!=-1)
//                    {
//                        Frame* parent = seq.getFrames()[frame->getParentFrameID()];
//                        gtLoader.drawLockframeSolvlets(ism, fSolve[row], frame, parent, (baseOutFolder+"/"+solverNames[param]+"/").c_str(), Scalar(0,0,255), 2);
//                    }
//                    else
//                        gtLoader.drawFrameSolvlets(fSolve[row], frame, (baseOutFolder+"/"+solverNames[param]+"/").c_str(), Scalar(0,0,255), 2);
//                }
//                cout << "done!" << endl;
                delete frame;
            }
            cerr << "Error saved!" << endl;

            out << "}" << endl;

            //release errors if they are no longer necessary
            errors.release();

            for(auto i=0; i<crossTrialErrors.size();++i)
            {
                crossTrialErrors.at(i).release();
            }
        }

        auto end = chrono::steady_clock::now();

        // Store the time difference between start and end
        auto diff = end - start;

        cout << paramName << " at " << param << " finished in " << chrono::duration <double, milli> (diff).count()  << " ms after " << num_trials << " trials." << endl;
    }

    out.close();

    cout << "Tuning finished, terminating. " << endl;
#if defined(MEMORY_DEBUG) && defined(UNIX)
    Debug(list_allocations_on(libcw_do));
#endif  // MEMORY_DEBUG && UNIX
    return 0;
}
