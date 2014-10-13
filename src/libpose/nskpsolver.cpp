#include "nskpsolver.hpp"
#include <tree.hh>
#include "lockframe.hpp"
#include "colorHistDetector.hpp"

#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/graphicalmodel/space/simplediscretespace.hxx>
#include <opengm/functions/potts.hxx>
#include <opengm/operations/adder.hxx>
#include <opengm/inference/messagepassing/messagepassing.hxx>

using namespace opengm;

NSKPSolver::NSKPSolver()
{
	id = 1; //this should be unique
	name = "NSKP"; //this should be unique
}

NSKPSolver::~NSKPSolver() //inherited virtual
{

}

Solution NSKPSolver::solve(const vector<Frame*>& frames) //inherited virtual
{
	vector<float> params; //set the default parameters vector

	//set some parameter deefaults
	
	//pass to next level solve function, for ISM computing
	return this->solve(frames, params);

}

Solution NSKPSolver::solve(const vector<Frame*>& frames, const vector<float>& params) //inherited virtual
{
	//compute the ISM here
	ImageSimilarityMatrix ISM(frames);

	//pass to solve function
	return this->solve(frames, params, ISM);
}

Solution NSKPSolver::solve(const vector<Frame*>& frames, const vector<float>& params, const ImageSimilarityMatrix& ism) //inherited virtual
{
	Solution finalSolution;
	
	vector<float> propagationParams;

	vector<Frame*> newFrames = propagateKeyframes(frames, propagationParams, ism);

    //generate new set of frames, with lockframes for solving by TLPS

	//finally, solve the resulting set of frames using the TLPS solver
	// TLPSSolver tlps;
	// Solution finalSolution = tlps.solve();

	//fill it with the soltuion stuff

	//return it
	return finalSolution;
}

vector<Frame*> NSKPSolver::propagateKeyframes(const vector<Frame*>& frames, const vector<float>& params, const ImageSimilarityMatrix& ism)
{
	// //@Q should frame ordering matter? in this function it should not matter, so no checks are necessary
	// float mst_thresm_multiplier=2.0; //@PARAM this is a param, not static
	// int mst_max_size=100; //@PARAM this is a param, not static

	// vector<Frame*> lockframes;

	// //build frame MSTs by ID's as in ISM
	// vector<MinSpanningTree> trees = buildFrameMSTs(ism, mst_max_size, mst_thresm_multiplier);
	// //now add variables to the space, with number of detections
	// for(int frameId=0; frameId<frames.size(); ++frameId)
	// {
	// 	if(frames[frameId]->getFrametype()!=0x02) //as long as it's not an interpolated frame, try to propagate from it
	// 	{
	// 		tree<int> mst = trees[frames[frameId]->getID()].getMST(); //get the MST, by ID, as in ISM
	// 		tree<int>::iterator mstIter;
	// 		//do OpenGM solve for single factor graph
	// 		ColorHistDetector chDetector;
	// 		vector<Frame*> trainingFrames;
	// 		trainingFrames.push_back(frames[frameId]); //set training frame by index
			
	// 		vector<float> trainingParams; //@PARAM this is a param, not static
	// 		chDetector.train(trainingFrames, 0);
	// 		vector<float> detectionParams; //@PARAM this is a param, not static


	// 		for(mstIter=mst.begin(); mstIter!=mst.end(); ++mstIter) //for each frame in the MST
	// 		{
	// 			vector<vector<LimbLabel> > labels = chDetector.detect(frames[*mstIter], detectionParams); //detect labels based on keyframe training

	// 			vector<size_t> numbersOfLabels; //numbers of labels per part

	// 			for(int i=0; i<labels.size(); ++i)
	// 			{
	// 				numbersOfLabels.push_back(labels[i].size());
	// 			} //numbers of labels now contains the numbers

	// 			Space space(numbersOfLabels.begin(), numbersOfLabels.end());
	// 			Model gm(space);

	// 			tree<BodyPart> partTree = frames[*mstIter]->getSkeleton().getPartTree();
	// 			tree<BodyPart>::iterator partIter, parentPartIter;
	// 			//label score cost
	// 			for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter) //for each of the detected parts
	// 			{
	// 				vector<int> varIndices; //create vector of indices of variables
	// 				varIndices.push_back(partIter->getPartID()); //push first value in

	// 				size_t scoreCostShape[]={numbersOfLabels[partIter->getPartID()]}; //number of labels
	// 				ExplicitFunction<float> scoreCostFunc(scoreCostShape, scoreCostShape+1); //explicit function declare

	// 				for(int i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
	// 				{
	// 					scoreCostFunc(i) = computeScoreCost(labels[partIter->getPartID()].at(i)); //compute the label score cost
	// 				}

	// 				Model::FunctionIdentifier scoreFid = gm.addFunction(scoreCostFunc); //explicit function add to graphical model
	// 				gm.addFactor(scoreFid, varIndices.begin(), varIndices.end()); //bind to factor and variables

	// 				ExplicitFunction<float> priorCostFunc(scoreCostShape, scoreCostShape+1); //explicit function declare

	// 				for(int i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
	// 				{
	// 					priorCostFunc(i) = computePriorCost(labels[partIter->getPartID()].at(i), *partIter);
	// 				}

	// 				Model::FunctionIdentifier priorFid = gm.addFunction(priorCostFunc); //explicit function add to graphical model
	// 				gm.addFactor(priorFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
					
	// 				if(partIter!=partTree.begin()) //if iterator is not on root node, there is always a parent body part
	// 				{
	// 					parentPartIter=partTree.parent(partIter); //find the parent of this part
	// 					varIndices.push_back(partIter->getPartID()); //push back parent partID as the second variable index

	// 					size_t jointCostShape[]={numbersOfLabels[partIter->getPartID()], numbersOfLabels[parentPartIter->getPartID()]}; //number of labels
	// 					ExplicitFunction<float> jointCostFunc(jointCostShape, jointCostShape+2); //explicit function declare

	// 					for(int i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
	// 					{
	// 						for(int j=0; j<labels[parentPartIter->getPartID()].size(); ++j)
	// 						{
	// 							//for every child/parent pair, compute score
	// 							jointCostFunc(i, j) = computeJointCost(labels[partIter->getPartID()].at(i), labels[parentPartIter->getPartID()].at(j));
	// 						}
	// 					}

	// 					Model::FunctionIdentifier jointFid = gm.addFunction(jointCostFunc); //explicit function add to graphical model
	// 					gm.addFactor(jointFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
	// 				}
	// 			}

	// 			// set up the optimizer (loopy belief propagation)

	// 		   	const size_t maxNumberOfIterations = 40;
	// 		   	const double convergenceBound = 1e-7;
	// 		   	const double damping = 0.5;
	// 		   	BeliefPropagation::Parameter parameter(maxNumberOfIterations, convergenceBound, damping);
	// 		   	BeliefPropagation bp(gm, parameter);
				
	// 		   // optimize (approximately)
	// 			BeliefPropagation::VerboseVisitorType visitor;
	// 			bp.infer(visitor);

	// 			// obtain the (approximate) argmin
	// 			vector<size_t> labeling(labels.size());
	// 			bp.arg(labeling);

	// 			vector<LimbLabel> solutionLabels;
	// 			for(int i=0; i<labels.size();++i)
	// 			{
	// 				solutionLabels.push_back(labels[i][labeling[i]]); //pupulate solution vector
	// 			}
	// 			//labeling now contains the approximately optimal labels for this problem
	// 			float solutionScore = evaluateSolution(frames[frameId], solutionLabels, true);
	// 			//evaluate the solution, and decide whether it should be added to keyframes

	// 			float acceptLockframeThreshold; //@PARAM this is a parameter, not a static value
	// 			if(solutionScore<=acceptLockframeThreshold)
	// 			{
	// 				//set up the lockframe
	// 				Solvlet solvlet(*mstIter, solutionLabels);
	// 				Skeleton skel(solvlet.toSkeleton());
	// 				Lockframe lockframe;

	// 				lockframe.setImage(frames[frameId]->getImage());
	// 				lockframe.setMask(frames[frameId]->getMask());
	// 				lockframe.setSkeleton(skel);
	// 				lockframe.setID(frames[frameId]->getID());

	// 				//create a frame pointer and push to the return vector
	// 				Frame * ptr = &lockframe;
	// 				lockframes.push_back(ptr);
	// 			}
	// 		}
	// 	}
	// }

	// vector<Frame*> returnFrames;
	// //look at lockframes and frames, and put together the return frames
	// for(int i=0; i<frames.size(); ++i)
	// {
	// 	int lockframeIndex = findFrameIndexById(frames[i]->getID(), frames);
	// 	if(lockframeIndex>=0) //if a new lockframe exists at this id
	// 	{
	// 		//push it into the sequen
	// 		returnFrames.push_back(lockframes[lockframeIndex]);
	// 	}
	// 	else //otherwise push back the old frame
	// 	{
	// 		returnFrames.push_back(frames[i]);
	// 	}
	// }
	// return returnFrames;

	 // construct a label space with numberOfVariables many variables,
   // each having numberOfLabels many labels
   const size_t numberOfVariables = 40; 
   const size_t numberOfLabels = 5;
   typedef SimpleDiscreteSpace<size_t, size_t> Space;
   Space space(numberOfVariables, numberOfLabels);

   // construct a graphical model with 
   // - addition as the operation (template parameter Adder)
   // - support for Potts functions (template parameter PottsFunction<double>)
   typedef OPENGM_TYPELIST_2(ExplicitFunction<double>, PottsFunction<double>) FunctionTypelist;
   typedef GraphicalModel<double, Adder, FunctionTypelist, Space> Model;
   Model gm(space);
   
   // for each variable, add one 1st order functions and one 1st order factor
   for(size_t v = 0; v < numberOfVariables; ++v) {
      const size_t shape[] = {numberOfLabels};
      ExplicitFunction<double> f(shape, shape + 1);
      for(size_t s = 0; s < numberOfLabels; ++s) {
         f(s) = static_cast<double>(rand()) / RAND_MAX;
      }
      Model::FunctionIdentifier fid = gm.addFunction(f);

      size_t variableIndices[] = {v};
      gm.addFactor(fid, variableIndices, variableIndices + 1);
   }

   // add one (!) 2nd order Potts function
   PottsFunction<double> f(numberOfLabels, numberOfLabels, 0.0, 0.3);
   Model::FunctionIdentifier fid = gm.addFunction(f);

   // for each pair of consecutive variables,
   // add one factor that refers to the Potts function 
   for(size_t v = 0; v < numberOfVariables - 1; ++v) {
      size_t variableIndices[] = {v, v + 1};
      gm.addFactor(fid, variableIndices, variableIndices + 2);
   }    

   // set up the optimizer (loopy belief propagation)
   typedef BeliefPropagationUpdateRules<Model, Minimizer> UpdateRules;
   typedef MessagePassing<Model, Minimizer, UpdateRules, MaxDistance> BeliefPropagation;
   const size_t maxNumberOfIterations = numberOfVariables * 2;
   const double convergenceBound = 1e-7;
   const double damping = 0.0;
   BeliefPropagation::Parameter parameter(maxNumberOfIterations, convergenceBound, damping);
   BeliefPropagation bp(gm, parameter);

   // optimize (approximately)
   BeliefPropagation::VerboseVisitorType visitor;
   bp.infer(visitor);

   // obtain the (approximate) argmin
   vector<size_t> labeling(numberOfVariables);
   bp.arg(labeling);
}

//return the index of the first instance of frame with matching id 
//if no matching id in vector, return -1
int NSKPSolver::findFrameIndexById(int id, vector<Frame*> frames)
{
	for(int i=0; i<frames.size(); ++i)
	{
		if(frames[i]->getID()==id)
			return i;
	}
	return -1;
}

//compute label score
float NSKPSolver::computeScoreCost(const LimbLabel& label)
{
	//@FIX
	//for now, just return the first available score
	vector<Score> scores = label.getScores();
	if(scores.size()>0)
		return scores[0].getScore();
	else //if no scores present return -1
		return -1;
}

//compute distance to parent limb label
float NSKPSolver::computeJointCost(const LimbLabel& child, const LimbLabel& parent)
{
	Point2f p0, p1, c0, c1;

	//@FIX this is really too simplistic, connecting these points
	child.getEndpoints(c0,c1);
	parent.getEndpoints(p0,p1);

	//return the squared distance from the lower parent joint p1, to the upper child joint c0
	return pow((c0.x-p1.x), 2)+pow((c0.y-p1.y), 2);
}

//compute distance to the body part prior
float NSKPSolver::computePriorCost(const LimbLabel& label, const BodyPart& prior)
{
	Point2f p0,p1, pp0, pp1;
	label.getEndpoints(p0,p1);
	pp0 = prior.getParentJoint()->getImageLocation();
	pp0 = prior.getChildJoint()->getImageLocation();

	//return the sum of squared distances between the corresponding joints, prior to label
	return pow((p0.x-pp0.x), 2)+pow((p0.y-pp0.y), 2)+pow((p1.x-pp1.x), 2)+pow((p1.y-pp1.y), 2);
}



//build an MST for every frame and return the vector
vector<MinSpanningTree > NSKPSolver::buildFrameMSTs(ImageSimilarityMatrix ism, int treeSize, float threshold)
{
    vector<MinSpanningTree> frameMST;

    vector<vector<int> > frameMSTvec;

    for(int i=0; i<ism.size(); ++i)
    {
    	//for each frame, build an MST
    	MinSpanningTree frameTree(ism, i, treeSize, threshold);
    	//and add to vector
        frameMST[i] = frameTree;
    }

    return frameMST;
}

//suggest maximum number of keyframes
//function should return vector with suggested keyframe numbers
vector<Point2i> NSKPSolver::suggestKeyframes(vector<MinSpanningTree>& mstVec)
{
	vector<vector<int> > orderedList;
	for(int i=0; i<mstVec.size(); ++i)
	{
	    tree<int>::iterator iter;
        tree<int> MST = mstVec[i].getMST();	
        
        vector<int> frames;

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
        int maxSize=0;
        int idx=-1;
        for(int i=0; i<orderedList.size(); ++i)
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
        vector<int> erasedVector = orderedList[idx];

        frameOrder.push_back(Point2i(erasedVector[0], maxSize));

        orderedList.erase(orderedList.begin() + idx);

        //remove all values in that vector from all others
        for(int i=0; i<orderedList.size(); ++i)
        {
            //for each element in erasedVector
            for(int j=0; j<erasedVector.size(); ++j)
            {
                orderedList[i].erase(std::remove(orderedList[i].begin(), orderedList[i].end(), erasedVector[j]), orderedList[i].end());
            }
        }
    }
    //return the resulting vector
    return frameOrder;
}

float NSKPSolver::evaluateSolution(Frame* frame, vector<LimbLabel> labels, bool debug)
{
//     /*
//       There should clearly be several factors that affect the outcome of an evaluation:
//       1) Mask coverage
//       2) Parts falling outside mask range
//       3) ?

//       */
//     float numPixels=0;
//     float numHitPixels=0; //number of pixels in polygon that are inside the mask
//     float numMissPixels=0; //number of pixels in polygons that are outside of the mask
//     Mat maskMat = frame->getMask();
//     //we are at some frame with the image already loaded (currentFrameNumber)    

//     //count the total number of mask pixels
//     for(int x=0; x<maskMat.rows(); ++x)
//     {
//         for(int y=0; y<maskMat.cols(); ++y)
//         {
//             int intensity = maskMat.at<uchar>(y, x);
//             bool blackPixel=intensity<10; //if all intensities are zero

//             //QColor maskCol = mask.pixel(x,y);
//             if(!blackPixel) //only take in pixels that are in the mask
//             {
//                 numPixels++; //count the total number of pixels
//             }
//         }
//     }

//     vector<float> limbScores;
//     for(int numLabel=0; numLabel<labels.size(); ++numLabel)
//     {
//         float pixHit=0;
//         float pixTotal=0;

//         LimbLabel label = labels[numLabel];
//         QPolygon poly = label.p;


//         for(int x=0; x<maskMat.rows(); ++x)
//         {
//             for(int y=0; y<maskMat.cols(); ++y)
//             {
//             	//@NEEDS FIXING
//                 if(poly.containsPoint(QPoint(x,y), Qt::OddEvenFill)) //polygon from label) //only take in pixels that are in the mask
//                 {
//                     pixTotal++;

//                     int intensity = maskMat.at<uchar>(y, x);

//                     bool blackPixel=intensity<10; //if all intensities are zero

//                     //QColor maskCol = mask.pixel(x,y);
//                     if(!blackPixel)
//                     {
//                         pixHit++;
//                     }
//                 }
//             }
//         }

//         limbScores.push_back(pixHit/pixTotal);
//     }


//     for(int x=0; x<mask.width(); ++x)
//     {
//         for(int y=0; y<mask.height(); ++y)
//         {

//             int intensity = maskMat.at<uchar>(y, x);

//             bool blackPixel=intensity<10; //if all intensities are zero
//             //QColor maskCol = mask.pixel(x,y);

//             if(!blackPixel) //only take in pixels that are in the mask
//             {
//                 numPixels++; //count the total numbe rf pixels
//                 bool hit=false;
//                 for(int i=0; i<labels.size(); ++i)
//                 {

//                     LimbLabel label = labels[i];
//                     QPolygon poly = label.p;
//                     if(poly.containsPoint(QPoint(x,y), Qt::OddEvenFill)) //polygon from label
//                     {
//                         hit = true;
//                         if(debug)
//                             this->ui->imageViewer->drawDot(Point2i(x, y), 1, QColor(0,255,0)); //draw a green point wherever it hits
//                         break;
//                     }
//                 }
//                 if(hit)
//                 {
//                     //colour this pixel in as hit
//                     numHitPixels++;
//                 }
//                 else
//                 {
//                     if(debug)
//                         this->ui->imageViewer->drawDot(Point2i(x, y), 1, QColor(255,0,0)); //draw a green point wherever it hits
//                 }
//             }
//             else
//             {
//                 bool hit=false;
//                 for(int i=0; i<labels.size(); ++i)
//                 {
//                     LimbLabel label = labels[i];
//                     QPolygon poly = label.p;
//                     if(poly.containsPoint(QPoint(x,y), Qt::OddEvenFill)) //polygon from label
//                     {
//                         hit = true;
//                         if(debug)
//                             this->ui->imageViewer->drawDot(Point2i(x, y), 1, QColor(0,0,0)); //draw a green point wherever it hits
//                         break;
//                     }
//                 }
//                 if(hit)
//                 {
//                     //colour this pixel in as hit
//                     numMissPixels++;
//                 }
//             }
//         }
//     }

//     float avgSuppScore=0;

//     for(int i=0; i<labels.size();++i)
//     {
//         if(i==0 || i>5) //count only the real labels
//             avgSuppScore+=labels[i].supportScore;
//     }
//     avgSuppScore=avgSuppScore/(labels.size()-5);

//     //show the worst half of labels
//     //    cerr << "Poorly localised labels: ";
//     //    for(int i=0; i<labels.size(); ++i)
//     //    {
//     //        if((i==0 || i>5) && labels[i].supportScore>avgSuppScore) //high is bad
//     //            cerr << limbName(i).toStdString() << " (" <<labels[i].supportScore<<") ";
//     //    }
//     //    cerr << endl;

//     cerr << "Poorly localised labels (by hit/total pixels ratio): ";
//     for(int i=0; i<labels.size(); ++i)
//     {
//         if((i==0 || i>5) && limbScores[i]<0.4) //high is bad
//             cerr << limbName(i).toStdString() << " (" <<limbScores[i]<<") ";
//     }
//     cerr << endl;

//     //also, if there are any completely broken limbs, this should lower the score

//     return (numHitPixels-numMissPixels)/numPixels;
	return 1.0;
}
