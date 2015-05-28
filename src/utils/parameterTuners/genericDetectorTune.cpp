#include <iostream>
#include <nskpsolver.hpp>
#include <tlpssolver.hpp>
#include "projectLoader.hpp"
#include <string>
#include "colorHistDetector.hpp"
#include "surfDetector.hpp"
#include "hogDetector.hpp"

using namespace std;

Mat computeErrorToGT(vector<Solvlet> solves, vector<Frame*> keyframes) //return squared error from solve to GT
{
    Mat errorMatrix;
    if(!solves.size()==keyframes.size())
    {
        cerr<<"Number of solves and number of keyframes do not match!" << endl;
        return errorMatrix;
    }

    errorMatrix.create(keyframes.size(), solves[0].getLabels().size(), DataType<float>::type);

    vector<vector<float> > globalPartErrors;

    for(uint32_t i=0; i<keyframes.size();++i) //for every frame
    {
        float frameRMS=0;
        Skeleton kSkel = keyframes[i]->getSkeleton();
        tree<BodyPart> partTree = kSkel.getPartTree();

        vector<float> partErrors;
        vector<LimbLabel> labels = solves[i].getLabels();
        for(tree<BodyPart>::iterator partIter=partTree.begin(); partIter!=partTree.end(); ++partIter) //for every bodypart
        {
            vector<LimbLabel>::iterator label;
            for(label=labels.begin(); label!=labels.end(); ++label)
            {
                if(label->getLimbID()==partIter->getPartID())
                    break;
            }

            Point2f p0,p1;
            label->getEndpoints(p0,p1);
            Point2f t0(kSkel.getBodyJoint(partIter->getParentJoint())->getImageLocation());
            Point2f t1(kSkel.getBodyJoint(partIter->getChildJoint())->getImageLocation());

            //dist(p0gt, p0sim)*0.5+dist(p1gt, p1sim)*0.5;
            float d0 = PoseHelper::distSquared(p0,t0); //dist between parent joints
            float d1 = PoseHelper::distSquared(p1,t1); //dist between child joints

            float error = 0.5*d0+0.5*d1; //average error for the two joints
            errorMatrix.at<float>(i,partIter->getPartID()) = error;

        }
        globalPartErrors.push_back(partErrors);
        //frameRMS=sqrt(frameRMS/(float)solves.size()); //the RMS error for this frame

        //write this frame to file
    }
    return errorMatrix;
}

vector<Solvlet> doInterpolationDetect(Detector* detector, vector<Frame*> vFrames, map<string,float> params)
{
    vector<Solvlet> solvlets;
    Sequence seq(0, "test", vFrames);
    seq.estimateUniformScale(params);
    seq.computeInterpolation(params);

    vector<Frame*> frames = seq.getFrames();
    for(uint32_t frame=0; frame<frames.size(); ++frame)
    {
        vector<vector<LimbLabel> > labels;
        labels = detector->detect(frames[frame], params, labels); //detect labels based on keyframe training
        //now take the best from each frame into a Solvlet
        Solvlet solvlet;
        vector<LimbLabel> bestLabels;
        for(vector<vector<LimbLabel> >::iterator pl=labels.begin(); pl!=labels.end(); ++pl)
        {
            bestLabels.push_back(pl->at(0)); //push back the top scoring label for this part
        }

        //now set solvlet
        solvlet.setLabels(bestLabels);
        solvlet.setFrameID(frame);
        //and push to solvlets
        solvlets.push_back(solvlet);
    }
    return solvlets;
}

vector<Solvlet> doPropagationDetect(Detector* detector, vector<Frame*> vFrames, ImageSimilarityMatrix ism, map<string,float> params)
{
    vector<Solvlet> solvlets;
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
        return solvlets;
    }

    //propagate the zero keyframe


    for(uint32_t frame=0; frame<frames.size(); ++frame)
    {
        Frame * lockframe = new Interpolation();
        //Skeleton shiftedPrior = frames[frameId]->getSkeleton();
        lockframe->setSkeleton(frames[kfr]->getSkeleton());
        lockframe->setID(frames[frame]->getID());
        lockframe->setImage(frames[frame]->getImage());
        lockframe->setMask(frames[frame]->getMask());

        //compute the shift between the frame we are propagating from and the current frame
        Point2f shift = ism.getShift(frames[kfr]->getID(),frames[frame]->getID());
        lockframe->shiftSkeleton2D(shift);

        vector<vector<LimbLabel> > labels;
        labels = detector->detect(lockframe, params, labels); //detect labels based on keyframe training
        //now take the best from each frame into a Solvlet
        Solvlet solvlet;
        vector<LimbLabel> bestLabels;
        for(vector<vector<LimbLabel> >::iterator pl=labels.begin(); pl!=labels.end(); ++pl)
        {
            bestLabels.push_back(pl->at(0)); //push back the top scoring label for this part
        }

        //now set solvlet
        solvlet.setLabels(bestLabels);
        solvlet.setFrameID(frame);
        //and push to solvlets
        solvlets.push_back(solvlet);
    }
    return solvlets;
}

int main (int argc, char **argv)
{
    if (argc != 4)
    {
        cout << "Usage hybridSolverTest [project.xml] [ground_truth.xml] [out directory]" << endl;
        return -1;
    }
    string curFolder = argv[1];
    curFolder = curFolder.substr(0, curFolder.find_last_of("/"));
    if (curFolder.back() != '/')
    {
        curFolder += '/';
    }

    ProjectLoader projectLoader(curFolder);
    ProjectLoader gtLoader(curFolder);

    cout << "Loading projects..." << endl;

    if (projectLoader.Load(argv[1]) == true)
    {
        cout << "Test project was successfully loaded" << endl;
    }
    else
    {
        cout << "Test project was not loaded" << endl;
        return -1;
    }

    if (gtLoader.Load(argv[2]) == true)
    {
        cout << "GT project was successfully loaded" << endl;
    }
    else
    {
        cout << "GT project was not loaded" << endl;
        return -1;
    }

    //set up tune parameters to test
    //    searchDistCoeff	0.5
    //    minTheta	90
    //    maxTheta	100
    //    stepTheta	10
    //    uniqueLocationCandidates	4
    //    searchDistCoeffMult	1.25
    //    maxFrameHeight 420 //

    vector <Frame*> gtFrames = gtLoader.getFrames(); //the ground truth frames to compare against

    float searchDistCoeff_min=0.25, searchDistCoeff_max=2.0, searchDistCoeff_step=0.25; //pixel search radius
    float dTheta_min=20, dTheta_max=180, dTheta_step=10;
    float thetaStep_min=5, thetaStep_max=30, thetaStep_step=5;
    float searchStepCoeff_min=0.05, searchStepCoeff_max=0.5, searchStepCoeff_step=0.05;
    int uniqueLocationCandidates_min=1,uniqueLocationCandidates_max=128, uniqueLocationCandidates_step=1, uniqueLocationCandidates_step_mul=2;
    int frameHeight_max=gtFrames[0]->getImage().rows, frameHeight_step=frameHeight_max/5, frameHeight_min=frameHeight_step; //never upscale, it's silly 5 steps to reach 1080p
    //build ISM, for shifts
    ImageSimilarityMatrix ism;
    ism.buildImageSimilarityMatrix(gtFrames); //only 15 or so images should be quick anyway

    Sequence gtSeq(0,"gt", gtFrames);
   // Sequence seq(1,"test", projectLoader.getFrames());
    map<string,float> pr0;
    gtSeq.computeInterpolation(pr0);
   // seq.computeInterpolation(pr1);
    gtSeq.estimateUniformScale(pr0);
   // seq.estimateUniformScale(pr1);

    //CASE_1: Interpolation
    //do train on the two end frames
    vector<string> tuneTypes;
    tuneTypes.push_back("interpolation");
    tuneTypes.push_back("propagation");

    //search distance tuning
    for(vector<string>::iterator tuneType=tuneTypes.begin(); tuneType!=tuneTypes.end(); ++tuneType) //do the two methods of propagation
    {
        vector<string> detNames; //just for saving purposes
        vector<Detector*> detectors;

        detectors.push_back(new HogDetector());
        detNames.push_back("HoG");

        detectors.push_back(new ColorHistDetector());
        detNames.push_back("ColorHist");

//        detectors.push_back(new SurfDetector()); //don't use SURF for now
//        detNames.push_back("SURF");

        vector<Frame*> trainingFrames;

        trainingFrames.push_back(gtFrames.front()); //set training frame for propagation
        if((*tuneType)=="interpolation")
            trainingFrames.push_back(gtFrames.back()); //set the second training frame for interpolation

        for(uint32_t i=0; i<detectors.size(); ++i)
        {
            detectors[i]->train(trainingFrames, map<string,float>()); //don't pass any training params (there aren't any anyway)
        }

        for(uint32_t d=0; d<detectors.size(); ++d) //go through every detector
        {
            string outFile=string(argv[3])+"/"+projectLoader.getProjectTitle()+"/";
            string tuneName="searchDist";

            outFile+=tuneName;
            outFile+="_";
            outFile+=detNames[d];

            //open file for writing
            ofstream out;

            try
            {
                cout << "On " << outFile << endl;
                out.open(outFile);
                //write file header first
                out << 2 << " " << "searchDistCoeff" << " " << "searchStepCoeff" << endl;

                for(float searchDistCoeff=searchDistCoeff_min;searchDistCoeff<=searchDistCoeff_max;searchDistCoeff+=searchDistCoeff_step) //8 steps
                {
                    for(float searchStepCoeff=searchStepCoeff_min;searchStepCoeff<=searchStepCoeff_max;searchStepCoeff+=searchStepCoeff_step) //5 steps
                    {
                        map<string, float> params;
                        params.emplace("searchDistCoeff", searchDistCoeff); //orientation step
                        params.emplace("searchStepCoeff", searchStepCoeff); //orientation step
                        params.emplace("debugLevel", 0); //orientation step
                        params.emplace("minTheta", 180); //- orientation
                        params.emplace("maxTheta", 190); //+orientation
                        params.emplace("stepTheta", 10); //orientation step
                        params.emplace("uniqueLocationCandidates", 10);


                        //there is now a solvlet at each frame for this detector

                        Sequence seq(1,"test", projectLoader.getFrames());

                        seq.computeInterpolation(params);
                        seq.estimateUniformScale(params);

                        Mat errors;

                        if((*tuneType)=="interpolation")
                            errors = computeErrorToGT(doInterpolationDetect(detectors[d], seq.getFrames(), params), gtFrames);
                        else
                            errors = computeErrorToGT(doPropagationDetect(detectors[d], seq.getFrames(), ism, params), gtFrames);

                        //now write all this error data, along with the params that spawned them
                        for(uint32_t row=0; row<errors.rows; ++row)
                        {
                            //this is a row in the output file
                            //frameID p1Value p2Value p3Value .. pNValue limb1RMS limb2RMS limb3RMS limb4RMS ... limbKRMS meanRMS evalScore
                            out << row << " " << searchDistCoeff << " " << searchStepCoeff << " ";
                            for(uint32_t col=0; col<errors.cols; ++col)
                            {
                                //this is an item of the row
                                out << errors.at<float>(row,col) << " ";
                            }
                            out << endl; //newline at the end of the block
                        }
                    }
                }

                out.close();
            }
            catch (int e)
            {
                cerr << "Unable to open " << outFile << " for writing. Exception " << e << endl;
                //return 0;
            }

            outFile=string(argv[3])+"/"+projectLoader.getProjectTitle()+"/";
            tuneName="rotRange";

            outFile+=tuneName;
            outFile+="_";
            outFile+=detNames[d];

            //open file for writing
            ofstream outRot;

            try
            {
                cout << "On " << outFile << endl;
                outRot.open(outFile);
                //write file header first
                outRot << 2 << " " << "dTheta" << " " << "thetaStep" << endl;

                for(float dTheta=dTheta_min;dTheta<=dTheta_max;dTheta+=dTheta_step) //9 steps
                {
                    for(float thetaStep = thetaStep_min; thetaStep<=thetaStep_max; thetaStep+=thetaStep_step) //6 steps
                    {
                        map <string, float> params; //use the default params

                        params.emplace("minTheta", dTheta); //- orientation
                        params.emplace("maxTheta", dTheta+dTheta_step); //+orientation
                        params.emplace("stepTheta", thetaStep); //orientation step

                        Sequence seq(1,"test", projectLoader.getFrames());

                        seq.computeInterpolation(params);
                        seq.estimateUniformScale(params);

                        Mat errors;

                        if((*tuneType)=="interpolation")
                            errors = computeErrorToGT(doInterpolationDetect(detectors[d], seq.getFrames(), params), gtFrames);
                        else
                            errors = computeErrorToGT(doPropagationDetect(detectors[d], seq.getFrames(), ism, params), gtFrames);

                        //now write all this error data, along with the params that spawned them
                        for(uint32_t row=0; row<errors.rows; ++row)
                        {
                            //this is a row in the output file
                            //frameID p1Value p2Value p3Value .. pNValue limb1RMS limb2RMS limb3RMS limb4RMS ... limbKRMS meanRMS evalScore
                            outRot << row << " " << dTheta << " " << thetaStep << " ";
                            for(uint32_t col=0; col<errors.cols; ++col)
                            {
                                //this is an item of the row
                                outRot << errors.at<float>(row,col) << " ";
                            }
                            outRot << endl;
                        }
                    }
                }
                outRot.close();
            }
            catch (int e)
            {
                cerr << "Unable to open " << outFile << " for writing. Exception " << e << endl;
                //return 0;
            }

            outFile=curFolder+"/"+argv[3]+"/"+projectLoader.getProjectTitle()+"/";
            tuneName="localCandidates";
            outFile+=tuneName;
            outFile+="_";
            outFile+=detNames[d];

            //open file for writing
            ofstream outLoc;

            try
            {
                cout << "On " << outFile << endl;
                outLoc.open(outFile);
                //write file header first
                outLoc << 1 << " " << "uniqueLocationCandidates" << endl;

                for(int uniqueLocationCandidates=uniqueLocationCandidates_min;uniqueLocationCandidates<=uniqueLocationCandidates_max;uniqueLocationCandidates+=uniqueLocationCandidates_step) //7 steps
                {
                    map <string, float> params; //use the default params

                    params.emplace("uniqueLocationCandidates", uniqueLocationCandidates);

                    Sequence seq(1,"test", projectLoader.getFrames());

                    seq.computeInterpolation(params);
                    seq.estimateUniformScale(params);

                    Mat errors;

                    if((*tuneType)=="interpolation")
                        errors = computeErrorToGT(doInterpolationDetect(detectors[d], seq.getFrames(), params), gtFrames);
                    else
                        errors = computeErrorToGT(doPropagationDetect(detectors[d], seq.getFrames(), ism, params), gtFrames);

                    //now write all this error data, along with the params that spawned them
                    for(uint32_t row=0; row<errors.rows; ++row)
                    {
                        //this is a row in the output file
                        //frameID p1Value p2Value p3Value .. pNValue limb1RMS limb2RMS limb3RMS limb4RMS ... limbKRMS meanRMS evalScore
                        outLoc << row << " " << uniqueLocationCandidates <<  " ";
                        for(uint32_t col=0; col<errors.cols; ++col)
                        {
                            //this is an item of the row
                            outLoc << errors.at<float>(row,col) << " ";
                        }
                        outLoc << endl;
                    }

                    uniqueLocationCandidates_step=uniqueLocationCandidates_step*uniqueLocationCandidates_step_mul;
                }
                outLoc.close();
            }
            catch (int e)
            {
                cerr << "Unable to open " << outFile << " for writing. Exception " << e << endl;
                return 0;
            }

            //image resizing tuning
            outFile=curFolder+"/"+argv[3]+"/"+projectLoader.getProjectTitle()+"/";
            tuneName="imgResize";
            outFile+=tuneName;
            outFile+="_";
            outFile+=detNames[d];

            //open file for writing
            ofstream outRes;
            try
            {
                cout << "On " << outFile << endl;
                outRes.open(outFile);
                outRes << 1 << " " << "maxFrameHeight" << endl;

                for(int frameHeight=frameHeight_min; frameHeight<=frameHeight_max; frameHeight+=frameHeight_step) //last compression level is native, 5 steps
                {
                    map <string, float> params; //use the default params

                    params.emplace("maxFrameHeight", frameHeight);

                    Sequence seq(1,"test", projectLoader.getFrames());

                    seq.computeInterpolation(params);
                    seq.estimateUniformScale(params);

                    Mat errors;

                    if((*tuneType)=="interpolation")
                        errors = computeErrorToGT(doInterpolationDetect(detectors[d], seq.getFrames(), params), gtFrames);
                    else
                        errors = computeErrorToGT(doPropagationDetect(detectors[d], seq.getFrames(), ism, params), gtFrames);

                    //now write all this error data, along with the params that spawned them
                    for(uint32_t row=0; row<errors.rows; ++row)
                    {
                        //this is a row in the output file
                        //frameID p1Value p2Value p3Value .. pNValue limb1RMS limb2RMS limb3RMS limb4RMS ... limbKRMS meanRMS evalScore
                        outRes << row << " " << frameHeight << " ";
                        for(uint32_t col=0; col<errors.cols; ++col)
                        {
                            //this is an item of the row
                            outRes << errors.at<float>(row,col) << " ";
                        }
                        outRes << endl;
                    }
                }
                outRes.close();
            }
            catch (int e)
            {
                cerr << "Unable to open " << outFile << " for writing. Exception " << e << endl;
                return 0;
            }
        }
    }

    return 0;
}

