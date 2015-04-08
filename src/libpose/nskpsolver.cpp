#include <limits>
#include "nskpsolver.hpp"
#include <tree.hh>
#include "lockframe.hpp"
#include "colorHistDetector.hpp"
#include "hogDetector.hpp"
#include "tlpssolver.hpp"

//using namespace opengm;

NSKPSolver::NSKPSolver()
{
    id = 1; //this should be unique
    name = "NSKP"; //this should be unique
}

NSKPSolver::~NSKPSolver() //inherited virtual
{

}

vector<Solvlet> NSKPSolver::solve(Sequence& sequence) //inherited virtual
{
    map<string, float> params; //set the default parameters vector

    //set some parameter defaults

    //pass to next level solve function, for ISM computing
    return this->solve(sequence, params);
}

vector<Solvlet> NSKPSolver::solve(Sequence& sequence, map<string, float> params) //inherited virtual
{
    //compute the ISM here
    ImageSimilarityMatrix ISM(sequence.getFrames());

    //pass to solve function
    return this->solve(sequence, params, ISM);
}

vector<Solvlet> NSKPSolver::solve(Sequence& sequence, map<string, float>  params, const ImageSimilarityMatrix& ism) //inherited virtual
{
    //parametrise the number of times frames get propagated
    params.emplace("nskpIters", 1); //set number of iterations, 0 to iterate until no new lockframes are introduced

    uint32_t nskpIters=params.at("nskpIters");
    if(nskpIters==0)
        nskpIters=INT32_MAX;

    vector<Solvlet> solvlets;
    sequence.computeInterpolation(params); //interpolate the sequence first
    //propagate keyframes
    vector<Frame*> propagatedFrames = sequence.getFrames();
    uint32_t lockframesLastIter=0;
    for(uint32_t i=0; i<nskpIters; ++i)
    {
        if(i>=nskpIters)
            break;

        vector<Solvlet> sol = propagateKeyframes(propagatedFrames, params, ism);

        for(vector<Solvlet>::iterator s=sol.begin(); s!=sol.end();++s)
            solvlets.push_back(*s);

        //calculate number of lockframes in the sequence
        uint32_t numLockframes=0;
        for(vector<Frame*>::iterator frameIter=propagatedFrames.begin(); frameIter!=propagatedFrames.end(); ++frameIter)
        {
            if((*frameIter)->getFrametype()==LOCKFRAME)
                numLockframes++;
        }

        if(numLockframes==lockframesLastIter) //terminate loop if no more lockframes are generated
        {
            cerr << "Terminating keyframe propagation after " << i << " iterations." << endl;
            break;
        }
        lockframesLastIter=numLockframes;
    }

    //create tlps solver
    TLPSSolver tlps;
    //sequence.setFrames(propagatedFrames);

    //the params map should countain all necessary parameters for solving, if they don't exist, default values should be used

    //return the TLPS solve
    return solvlets;
    //return tlps.solve(sequence, params);
}

vector<Solvlet> NSKPSolver::propagateKeyframes(vector<Frame*>& frames, map<string, float>  params, const ImageSimilarityMatrix& ism)
{
    // //@Q should frame ordering matter? in this function it should not matter, so no checks are necessary
    // float mst_thresm_multiplier=params.at("mst_thresh_multiplier"); //@FIXME PARAM this is a param, not static
    // int mst_max_size=params.at("mst_max_size"); //@FIXME PARAM this is a param, not static

    //the params vector should contain all necessary parameters, if a parameter is not present, default values should be used

    vector<Solvlet> solvlets;
    int numLockframesGenerated=0;
    params.emplace("useHoGdet", 0); //determine if HoG descriptor is used and with what coefficient
    params.emplace("useCSdet", 1); //determine if colorhist detector is used and with what coefficient
    params.emplace("useSURFdet", 0); //determine whether SURF detector is used and with what coefficient
    params.emplace("maxPartCandidates", 500); //set the max number of part candidates to allow into the solver
    params.emplace("acceptLockframeThreshold", 0.5); //set up the lockframe accept threshold by mask coverage
    params.emplace("debugLevel", 1); //set up the lockframe accept threshold by mask coverage
    params.emplace("propagateToLockframes", 0);


    float useHoG = params.at("useHoGdet");
    float useCS = params.at("useCSdet");
    //float useSURF = params.at("useSURFdet");
    uint32_t debugLevel = params.at("debugLevel");
    bool propagateToLockframes=params.at("propagateToLockframes");

    vector<Frame*> lockframes;

    //build frame MSTs by ID's as in ISM
    vector<MinSpanningTree> trees = buildFrameMSTs(ism, params);
    //now add variables to the space, with number of detections
    for(uint32_t frameId=0; frameId<frames.size(); ++frameId)
    {
        if(frames[frameId]->getFrametype()!=0x02) //as long as it's not an interpolated frame, try to propagate from it
        {
            tree<int> mst = trees[frames[frameId]->getID()].getMST(); //get the MST, by ID, as in ISM
            tree<int>::iterator mstIter;
            //do OpenGM solve for single factor graph

            vector<Detector*> detectors;
            if(useHoG)
                detectors.push_back(new HogDetector());
            if(useCS)
                detectors.push_back(new ColorHistDetector());

            vector<Frame*> trainingFrames;
            trainingFrames.push_back(frames[frameId]); //set training frame by index

            for(uint32_t i=0; i<detectors.size(); ++i)
            {
                detectors[i]->train(trainingFrames, params);
            }

            for(mstIter=mst.begin(); mstIter!=mst.end(); ++mstIter) //for each frame in the MST
            {
                if(frames[*mstIter]->getFrametype()==KEYFRAME) //skip keyframes
                    continue;
                else if(frames[*mstIter]->getFrametype()==LOCKFRAME && !propagateToLockframes) //skip lockframes, unless explicitly requested
                    continue;
                vector<vector<LimbLabel> > labels, tempLabels;
                vector<vector<LimbLabel> >::iterator labelPartsIter;

                Frame * lockframe = new Lockframe();
                //Skeleton shiftedPrior = frames[frameId]->getSkeleton();
                lockframe->setSkeleton(frames[frameId]->getSkeleton());
                lockframe->setID(frames[*mstIter]->getID());
                lockframe->setImage(frames[*mstIter]->getImage());
                lockframe->setMask(frames[*mstIter]->getMask());

                //compute the shift between the frame we are propagating from and the current frame
                Point2f shift = ism.getShift(frames[frameId]->getID(),frames[*mstIter]->getID());

                lockframe->shiftSkeleton2D(shift); //shift the skeleton by the correct amount
                Skeleton skeleton = lockframe->getSkeleton();

                for(uint32_t i=0; i<detectors.size(); ++i) //for every detector
                {
                    labels = detectors[i]->detect(lockframe, params, labels); //detect labels based on keyframe training
                }

                for(labelPartsIter=labels.begin();labelPartsIter!=labels.end();++labelPartsIter) //now take the top labels
                {
                    uint32_t maxPartCandidates = params.at("maxPartCandidates");
                    vector<LimbLabel> temp;

                    if((labelPartsIter->at(0)).getIsWeak() || (labelPartsIter->at(0)).getIsOccluded()) //if weak or occluded, add all labels
                        maxPartCandidates = labelPartsIter->size();
                    for(uint32_t currentSize=0; currentSize<maxPartCandidates && currentSize<labelPartsIter->size(); ++currentSize)
                    {
                        temp.push_back(labelPartsIter->at(currentSize));
                    }
                    tempLabels.push_back(temp);
                }

                labels = tempLabels;

                vector<size_t> numbersOfLabels; //numbers of labels per part

                for(uint32_t i=0; i<labels.size(); ++i)
                {
                    numbersOfLabels.push_back(labels[i].size());
                } //numbers of labels now contains the numbers

                Space space(numbersOfLabels.begin(), numbersOfLabels.end());
                Model gm(space);

                //Skeleton skeleton = frames[*mstIter]->getSkeleton();
                tree<BodyPart> partTree = skeleton.getPartTree();
                tree<BodyPart>::iterator partIter, parentPartIter;

                uint32_t jointFactors=0, suppFactors=0, priorFactors=0;
                //label score cost
                for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter) //for each of the detected parts
                {
                    vector<int> varIndices; //create vector of indices of variables
                    varIndices.push_back(partIter->getPartID()); //push first value in

                    size_t scoreCostShape[]={numbersOfLabels[partIter->getPartID()]}; //number of labels
                    ExplicitFunction<float> scoreCostFunc(scoreCostShape, scoreCostShape+1); //explicit function declare

                    for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                    {
                        scoreCostFunc(i) = computeScoreCost(labels[partIter->getPartID()].at(i), params); //compute the label score cost
                    }

                    Model::FunctionIdentifier scoreFid = gm.addFunction(scoreCostFunc); //explicit function add to graphical model
                    gm.addFactor(scoreFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                    suppFactors++;

                    ExplicitFunction<float> priorCostFunc(scoreCostShape, scoreCostShape+1); //explicit function declare

                    //precompute the maxium and minimum for normalisation
                    float priorMin=FLT_MAX, priorMax=0;
                    vector<LimbLabel>::iterator lbl;
                    //vector<float> priorCostFuncValues;
                    for(lbl=labels[partIter->getPartID()].begin(); lbl!=labels[partIter->getPartID()].end(); ++lbl) //for each label in for this part
                    {
                        float val = computePriorCost(*lbl, *partIter, skeleton, params);

                        if(val<priorMin)
                            priorMin = val;
                        if(val>priorMax)
                            priorMax = val;

                        //priorCostFuncValues.push_back(val);
                    }

                    //now set up the solutions
                    for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                    {
                        priorCostFunc(i) = computeNormPriorCost(labels[partIter->getPartID()].at(i), *partIter, skeleton, params, priorMin, priorMax);
                    }

                    Model::FunctionIdentifier priorFid = gm.addFunction(priorCostFunc); //explicit function add to graphical model
                    gm.addFactor(priorFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                    priorFactors++;

                    if(partIter!=partTree.begin()) //if iterator is not on root node, there is always a parent body part
                    {
                        varIndices.clear();
                        parentPartIter=partTree.parent(partIter); //find the parent of this part
                        varIndices.push_back(parentPartIter->getPartID()); //push back parent partID as the second variable index
                        varIndices.push_back(partIter->getPartID()); //push first value in (parent, this)

                        size_t jointCostShape[]={numbersOfLabels[parentPartIter->getPartID()], numbersOfLabels[partIter->getPartID()]}; //number of labels
                        ExplicitFunction<float> jointCostFunc(jointCostShape, jointCostShape+2); //explicit function declare

                        //first figure out which of the current body part's joints should be in common with the parent body part

                        bool toChild=false;
                        if(parentPartIter->getChildJoint() == partIter->getParentJoint())
                        {
                            //then it's connected to child
                            toChild=true;
                        }

                        float jointMin=FLT_MAX, jointMax=0;
                        for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                        {
                            for(uint32_t j=0; j<labels[parentPartIter->getPartID()].size(); ++j)
                            {
                                float val = computeJointCost(labels[partIter->getPartID()].at(i), labels[parentPartIter->getPartID()].at(j), params, toChild);

                                if(val<jointMin)
                                    jointMin = val;
                                if(val>jointMax)
                                    jointMax = val;
                            }
                        }

                        for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
                        {
                            for(uint32_t j=0; j<labels[parentPartIter->getPartID()].size(); ++j)
                            {
                                //for every child/parent pair, compute score
                                jointCostFunc(j, i) = computeNormJointCost(labels[partIter->getPartID()].at(i), labels[parentPartIter->getPartID()].at(j), params, jointMax, toChild);
                            }
                        }

                        Model::FunctionIdentifier jointFid = gm.addFunction(jointCostFunc); //explicit function add to graphical model
                        gm.addFactor(jointFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
                        jointFactors++;
                    }
                }

                if(debugLevel>=1)
                {
                    float k;
                    k=skeleton.getPartTree().size(); //number of bones
                    float expectedSuppFactors=k; //n*k
                    float expectedPriorFactors=k; //n*k
                    float expectedJointFactors=(k-1); //n*(k-1)

                    assert(expectedSuppFactors==suppFactors);
                    assert(expectedJointFactors==jointFactors);
                    assert(priorFactors==expectedPriorFactors);
                }

                // set up the optimizer (loopy belief propagation)

                const size_t maxNumberOfIterations = 40;
                const double convergenceBound = 1e-7;
                const double damping = 0.5;
                BeliefPropagation::Parameter parameter(maxNumberOfIterations, convergenceBound, damping);
                BeliefPropagation bp(gm, parameter);

                // optimize (approximately)
                BeliefPropagation::VerboseVisitorType visitor;
                bp.infer(visitor);

                // obtain the (approximate) argmin
                vector<size_t> labeling(labels.size());
                bp.arg(labeling);

                vector<LimbLabel> solutionLabels;
                for(uint32_t i=0; i<labels.size();++i)
                {
                    solutionLabels.push_back(labels[i][labeling[i]]); //pupulate solution vector
                }
                //labeling now contains the approximately optimal labels for this problem


                if(debugLevel>=1)
                    cerr << "Evaluating solve for frame " << frames[*mstIter]->getID() << " based on prior from frame " << frames[frameId]->getID() << "..."<< endl;

                float solutionScore = evaluateSolution(frames[*mstIter], solutionLabels, params);

                if(debugLevel>=1)
                    cerr << "\tScore: " << solutionScore << endl;

                //evaluate the solution, and decide whether it should be added to keyframes


                float acceptLockframeThreshold = params.at("acceptLockframeThreshold"); //@PARAM this is a parameter, not a static value
                if(solutionScore>=acceptLockframeThreshold)
                {
                    //set up the lockframe
                    Solvlet solvlet(*mstIter, solutionLabels);
                    solvlets.push_back(solvlet);
                    Skeleton skel(solvlet.toSkeleton());
                    skel.setScale(frames[*mstIter]->getSkeleton().getScale()); //set skeleton scale
                    lockframe->setSkeleton(skel);
                    lockframe->setParentFrameID(frames[frameId]->getID());

                    lockframes.push_back(lockframe);
                    numLockframesGenerated++;
                }
            }
        }
    }

    for(uint32_t i=0; i<lockframes.size();++i)
    {
        *(frames[lockframes[i]->getID()]) = *(lockframes.at(i)); //make pointers point to the correct objects
    }

    cerr << "Generated " << numLockframesGenerated << " lockframes!" << endl;

    return solvlets;
}

//return the index of the first instance of frame with matching id 
//if no matching id in vector, return -1
uint32_t NSKPSolver::findFrameIndexById(int id, vector<Frame*> frames)
{
    for(uint32_t i=0; i<frames.size(); ++i)
    {
        if(frames[i]->getID()==id)
            return i;
    }
    return -1;
}

//compute label score
float NSKPSolver::computeScoreCost(const LimbLabel& label, map<string, float> params)
{
    params.emplace("imageCoeff", 0.0);
    params.emplace("useHoGdet", 0);
    params.emplace("useCSdet", 1.0);

    float lambda = params.at("imageCoeff");

    //@FIX
    float useHoG = params.at("useHoGdet");
    float useCS = params.at("useCSdet");

    //for now, just return the first available score
    vector<Score> scores = label.getScores();

    if(label.getIsOccluded()||label.getIsWeak()) //if it's occluded, return zero
        return 0;
    if(scores.size()>0)
        return lambda*scores[0].getScore();
    else //if no scores present return -1
        return 0;
}

//compute distance to parent limb label
float NSKPSolver::computeJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params, bool toChild)
{
    //emplace default
    //params.emplace("jointCoeff", 0.5);
    //float lambda = params.at("jointCoeff");
    Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    child.getEndpoints(c0,c1);
    parent.getEndpoints(p0,p1);

    //normalise this?

    //return the squared distance from the lower parent joint p1, to the upper child joint c0
    if(toChild)
        return pow((c0.x-p1.x), 2)+pow((c0.y-p1.y), 2);
    else //otherwise we're connected to the parent's parent body part
        return pow((c0.x-p0.x), 2)+pow((c0.y-p0.y), 2);
}

//compute distance to parent limb label
float NSKPSolver::computeNormJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params, float max, bool toChild)
{
    //emplace default
    params.emplace("jointCoeff", 0.5);
    params.emplace("jointLeeway", 0.05);
    float lambda = params.at("jointCoeff");
    float leeway = params.at("jointLeeway");
    Point2f p0, p1, c0, c1;

    //@FIX this is really too simplistic, connecting these points
    child.getEndpoints(c0,c1);
    parent.getEndpoints(p0,p1);

    //child length
    float clen = sqrt(pow(c0.x-c1.x, 2)+pow(c0.y-c1.y, 2));

    //normalise this?
    //give some leeway
    float score = 0;
    if(toChild) //connected to parent's child body part
        score = (pow(c0.x-p1.x, 2)+pow(c0.y-p1.y, 2))/max;
    else
        score = (pow(c0.x-p0.x, 2)+pow(c0.y-p0.y, 2))/max;
    if(score<pow(clen*leeway, 2)) //any distnace below leeway is zero
        score=0;
    //return the squared distance from the lower parent joint p1, to the upper child joint c0
    return lambda*score;
}

//compute distance to the body part prior
float NSKPSolver::computePriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, map<string, float> params)
{
    //  params.emplace("priorCoeff", 0.0);
    //	float lambda = params.at("priorCoeff");
    Point2f p0,p1, pp0, pp1;
    label.getEndpoints(p0,p1);
    pp0 = skeleton.getBodyJoint(prior.getParentJoint())->getImageLocation();
    pp1 = skeleton.getBodyJoint(prior.getChildJoint())->getImageLocation();

    //normalise this cost?

    //return the sum of squared distances between the corresponding joints, prior to label
    return pow((p0.x-pp0.x), 2)+pow((p0.y-pp0.y), 2)+pow((p1.x-pp1.x), 2)+pow((p1.y-pp1.y), 2);
}

float NSKPSolver::computeNormPriorCost(const LimbLabel& label, const BodyPart& prior, const Skeleton& skeleton, map<string, float> params, float min, float max)
{
    params.emplace("priorCoeff", 0.0);
    float lambda = params.at("priorCoeff");
    Point2f p0,p1, pp0, pp1;
    label.getEndpoints(p0,p1);
    pp0 = skeleton.getBodyJoint(prior.getParentJoint())->getImageLocation();
    pp1 = skeleton.getBodyJoint(prior.getChildJoint())->getImageLocation();

    //normalise this cost?

    //return the sum of squared distances between the corresponding joints, prior to label
    return lambda*((pow((p0.x-pp0.x), 2)+pow((p0.y-pp0.y), 2)+pow((p1.x-pp1.x), 2)+pow((p1.y-pp1.y), 2))/max);
}

//build an MST for every frame and return the vector
vector<MinSpanningTree > NSKPSolver::buildFrameMSTs(ImageSimilarityMatrix ism, map<string, float> params) //int treeSize, float threshold)
{
    //emplace defaults
    params.emplace("treeSize", ism.size()); //no size limit
    params.emplace("simThresh", 2.5); //set similarity as multiple of minimum, MUST be >=1
    int treeSize = params.at("treeSize");
    float simThresh = params.at("simThresh");
    vector<MinSpanningTree> frameMST;
    for(uint32_t i=0; i<ism.size(); ++i)
    {
        frameMST.push_back(MinSpanningTree());
    }

    //vector<vector<int> > frameMSTvec;

    for(uint32_t i=0; i<ism.size(); ++i)
    {
        //for each frame, build an MST
        MinSpanningTree frameTree(ism, i, treeSize, simThresh);
        //and add to vector
        frameMST[i] = frameTree;
    }

    return frameMST;
}

//suggest maximum number of keyframes
//function should return vector with suggested keyframe numbers
vector<Point2i> NSKPSolver::suggestKeyframes(vector<MinSpanningTree>& mstVec, map<string, float> params)
{
    vector<vector<uint32_t> > orderedList;
    for(uint32_t i=0; i<mstVec.size(); ++i)
    {
        tree<int>::iterator iter;
        tree<int> MST = mstVec[i].getMST();
        
        vector<uint32_t> frames;

        for(iter=MST.begin(); iter!=MST.end(); ++iter)
        {
            frames.push_back(*iter); //create a vector of vectors
        }
        orderedList.push_back(frames);
    }
    //this is the simple way of counting the number of frames that made it in
    //alternatively we could come up with a more complex scheme for determining keyframe optimality
    vector<Point2i> frameOrder;
    while(orderedList.size()!=0)
    {
        //find the largest frame MST:
        uint32_t maxSize=0;
        uint32_t idx=-1;
        for(uint32_t i=0; i<orderedList.size(); ++i)
        {
            if(orderedList[i].size()> maxSize)
            {
                idx = i;
                maxSize = orderedList[i].size();
            }
        }
        //if there largest vector remaining is of legnth zero, stop
        if(maxSize==1)
            break;


        //store it as the best candidate
        vector<uint32_t> erasedVector = orderedList[idx];

        frameOrder.push_back(Point2i(erasedVector[0], maxSize));

        orderedList.erase(orderedList.begin() + idx);

        //remove all values in that vector from all others
        for(uint32_t i=0; i<orderedList.size(); ++i)
        {
            //for each element in erasedVector
            for(uint32_t j=0; j<erasedVector.size(); ++j)
            {
                orderedList[i].erase(std::remove(orderedList[i].begin(), orderedList[i].end(), erasedVector[j]), orderedList[i].end());
            }
        }
    }
    //return the resulting vector
    return frameOrder;
}

float NSKPSolver::evaluateSolution(Frame* frame, vector<LimbLabel> labels, map<string, float> params)
{
    // /*
    //   There should clearly be several factors that affect the outcome of an evaluation:
    //   1) Mask coverage
    //   2) Parts falling outside mask range
    //   3) ?

    //   */

    //engaged pixles - the total number of pixels that are either within a limb label of within the mask
    //correct pixels - those pixles that are black and outside a label, and white and inside a label
    //incorrect pixels - those pixels that are black and inside a label, and white and outside a label
    //score = correct/(correct+incorrect)

    //emplace defaults
    params.emplace("badLabelThresh", 0.4); //if less than 40% of the pixels are in the mask, label this label bad
    params.emplace("debugLevel", 1);

    int debugLevel = params.at("debugLevel");

    Mat mask = frame->getMask();
    int correctPixels=0, incorrectPixels=0;
    int pixelsInMask=0;
    int coveredPixelsInMask=0;
    int incorrectlyCoveredPixels=0;
    int missedPixels=0;

    for(int i=0; i<mask.cols; ++i) //at every col - x
    {
        for(int j=0; j<mask.rows; ++j) //and every row - y
        {
            //int test = labels[0].containsPoint(Point2f(480,100));
            //check whether pixel hit a label from solution
            bool labelHit=false;
            for(vector<LimbLabel>::iterator label=labels.begin(); label!=labels.end(); ++label)
            {
                if(label->containsPoint(Point2f(i,j))) //this is done in x,y coords
                {
                    labelHit=true;
                    //break;
                }
            }

            //check pixel colour
            int intensity = mask.at<uchar>(j, i); //this is done with reve
            bool blackPixel=(intensity<10);

            if(!blackPixel)
                pixelsInMask++;

            if(blackPixel && labelHit) //if black in label, incorrect
            {
                incorrectPixels++;
                incorrectlyCoveredPixels++;
            }
            else if(!blackPixel && !labelHit) //if white not in label, incorret
            {
                incorrectPixels++;
                missedPixels++;
            }
            else if(!blackPixel && labelHit)//otherwise correct
            {
                correctPixels++;
                coveredPixelsInMask++;
            }
            //            else //black pixel and not label hit
            //                correctPixels++; //don't count these at all?
        }
    }


    double solutionEval = (float)correctPixels/((float)correctPixels+(float)incorrectPixels);

    if(debugLevel>=1)
        cerr << "Solution evaluation score - " << solutionEval << endl;

    //now check for critical part failures - label mostly outside of mask

    vector<Point2f> badLabelScores;
    float badLabelThresh = params.at("badLabelThresh");

    for(vector<LimbLabel>::iterator label = labels.begin(); label!=labels.end(); ++label)
    {
        vector<Point2f> poly = label->getPolygon(); //get the label polygon
        //compute min and max x and y
        //float xMin, xMax, yMin, yMax;
        vector<float> xS = {poly[0].x, poly[1].x, poly[2].x, poly[3].x};
        vector<float> yS = {poly[0].y, poly[1].y, poly[2].y, poly[3].y};
        auto xMin = min_element(xS.begin(), xS.end());
        auto xMax = max_element(xS.begin(), xS.end());

        auto yMin = min_element(yS.begin(), yS.end());
        auto yMax = max_element(yS.begin(), yS.end());

        int labelPixels=0;
        int badLabelPixels=0;

        for(int x=*xMin; x<*xMax; ++x)
        {
            for(int y=*yMin; y<*yMax; ++y)
            {
                if(label->containsPoint(Point2f(x,y)))
                {
                    int intensity = mask.at<uchar>(y, x); //this is done with reverse y,x
                    bool blackPixel=(intensity<10);
                    labelPixels++;
                    if(blackPixel)
                        ++badLabelPixels;
                }
            }
        }

        float labelRatio = (float)badLabelPixels/(float)labelPixels; //high is bad

        if(labelRatio>badLabelThresh)
            badLabelScores.push_back(Point2f(label->getLimbID(), labelRatio));
    }

    if(debugLevel>=1)
    {
        for(vector<Point2f>::iterator badL=badLabelScores.begin(); badL!=badLabelScores.end(); ++badL)
        {
            cerr << "Part " << badL->x << " is badly localised, with score " << 1.0-badL->y << endl;
        }
    }

    if(badLabelScores.size()!=0) //make the solution eval fail if a part is badly localised
        solutionEval=solutionEval-1.0;

    return solutionEval;
}
