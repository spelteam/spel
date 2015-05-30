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

vector<Solvlet> doInterpolationDetectMix(vector<Detector*> detectors, vector<Frame*> vFrames, map<string,float> params)
{
    //emplace zeros by default
    params.emplace("useHoGdet", 0);
    params.emplace("useCSet", 0);
    params.emplace("useSURFdet", 0);

    int numDetectors=1*(params.at("useHoGdet")!=0) + 1.0*(params.at("useSURFdet")!=0) + 1.0*(params.at("useCSdet")!=0);

    vector<Solvlet> solvlets;
    Sequence seq(0, "test", vFrames);
    seq.estimateUniformScale(params);
    seq.computeInterpolation(params);

    vector<Frame*> frames = seq.getFrames();
    for(uint32_t frame=0; frame<frames.size(); ++frame)
    {
        vector<vector<LimbLabel> > labels;
        for(uint32_t d=0; d<detectors.size(); ++d)
            labels = detectors[d]->detect(frames[frame], params, labels); //detect labels based on keyframe training
        //now take the best from each frame into a Solvlet
        Solvlet solvlet;
        vector<LimbLabel> bestLabels;
        for(vector<vector<LimbLabel> >::iterator pl=labels.begin(); pl!=labels.end(); ++pl)
        {
            for(vector<LimbLabel>::iterator l=pl->begin(); l!=pl->end(); ++l)
            {
                if(l->getScores().size()==numDetectors) //if we have the right number of scores in the score vector
                    bestLabels.push_back(*l); //push back the top scoring label for this part
            }
        }

        //now set solvlet
        solvlet.setLabels(bestLabels);
        solvlet.setFrameID(frame);
        //and push to solvlets
        solvlets.push_back(solvlet);
    }
    return solvlets;
}

vector<Solvlet> doPropagationDetectMix(vector<Detector*> detectors, vector<Frame*> vFrames, ImageSimilarityMatrix ism, map<string,float> params)
{
    //emplace zeros by default
    params.emplace("useHoGdet", 0);
    params.emplace("useCSet", 0);
    params.emplace("useSURFdet", 0);

    int numDetectors=1*(params.at("useHoGdet")!=0) + 1.0*(params.at("useSURFdet")!=0) + 1.0*(params.at("useCSdet")!=0);

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
        for(uint32_t d=0; d<detectors.size(); ++d)
            labels = detectors[d]->detect(lockframe, params, labels); //detect labels based on keyframe training
        //now take the best from each frame into a Solvlet
        Solvlet solvlet;
        vector<LimbLabel> bestLabels;
        for(vector<vector<LimbLabel> >::iterator pl=labels.begin(); pl!=labels.end(); ++pl)
        {
            for(vector<LimbLabel>::iterator l=pl->begin(); l!=pl->end(); ++l)
            {
                if(l->getScores().size()==numDetectors) //if we have the right number of scores in the score vector
                    bestLabels.push_back(*l); //push back the top scoring label for this part
            }
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

    float csCoeff_min=0.0, csCoeff_max=1.0, csCoeff_step=0.05;

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
            map<string,float> dp;
            if(detNames[i]=="HoG") //use grayscale images for HoG
                dp.emplace("grayImages", 1);
            detectors[i]->train(trainingFrames, dp); //don't pass any training params (there aren't any anyway)
        }

            string outFile=string(argv[3])+"/"+projectLoader.getProjectTitle()+"/";
            string tuneName="detectorMix";

            outFile+=tuneName;
            outFile+="_";
            outFile+=*tuneType;
            outFile+=".err";

            //open file for writing
            ofstream out;

            try
            {
                cout << "On " << outFile << endl;
                out.open(outFile);
                //write file header first
                out << 1 << " " << "csCoeff" <<  endl;

                for(float csCoeff=csCoeff_min;csCoeff<=csCoeff_max;csCoeff+=csCoeff_step) //20 steps
                {
                        map<string, float> params;
                        //detector settings
                        params.emplace("useCSdet", csCoeff); //determine if ColHist detector is used and with what coefficient
                        params.emplace("useHoGdet", (1.0-csCoeff)); //determine if HoG descriptor is used and with what coefficient
                        params.emplace("useSURFdet", 0.0); //determine whether SURF detector is used and with what coefficient

                        params.emplace("debugLevel", 0); //orientation step

                        //there is now a solvlet at each frame for this detector

                        Sequence seq(1,"test", projectLoader.getFrames());

                        seq.computeInterpolation(params);
                        seq.estimateUniformScale(params);

                        Mat errors;

                        if((*tuneType)=="interpolation")
                            errors = computeErrorToGT(doInterpolationDetectMix(detectors, seq.getFrames(), params), gtFrames);
                        else
                            errors = computeErrorToGT(doPropagationDetectMix(detectors, seq.getFrames(), ism, params), gtFrames);

                        //now write all this error data, along with the params that spawned them
                        for(uint32_t row=0; row<errors.rows; ++row)
                        {
                            //this is a row in the output file
                            //frameID p1Value p2Value p3Value .. pNValue limb1RMS limb2RMS limb3RMS limb4RMS ... limbKRMS meanRMS evalScore
                            out << row << " " << csCoeff << " ";
                            for(uint32_t col=0; col<errors.cols; ++col)
                            {
                                //this is an item of the row
                                out << errors.at<float>(row,col) << " ";
                            }
                            out << endl; //newline at the end of the block
                        }
                        errors.release();
                }

                out.close();
            }
            catch (int e)
            {
                cerr << "Unable to open " << outFile << " for writing. Exception " << e << endl;
                //return 0;
            }
//        }
    }

    return 0;
}

