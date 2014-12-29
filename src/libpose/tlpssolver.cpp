#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "tlpssolver.hpp"
#include "lockframe.hpp"
#include "colorHistDetector.hpp"
#include "math.h"
#include "interpolation.hpp"

using namespace Eigen;

TLPSSolver::TLPSSolver()
{
	id=0;
	name="TLPS";
}

TLPSSolver::~TLPSSolver()
{

}

vector<vector<Solvlet> > TLPSSolver::solve(const vector<Frame*>& frames) //inherited virtual
{
	map<string, float> params; //set the default parameters vector

	//set some parameter deefaults
	params.at("scoreIndex")=0;
	
	//pass to next level solve function, for ISM computing
	return this->solve(frames, params);
}

vector<vector<Solvlet> > TLPSSolver::solve(const vector<Frame*>& frames, map<string, float> params) //inherited virtual
{
	//first slice up the sequences
	vector<vector<Frame*> > slices = slice(frames);

	vector<vector<Solvlet> > sequenceSolvlets; //one vector of solvlets per slice
	for(uint32_t sliceNumber=0; sliceNumber<slices.size(); ++sliceNumber)
	{
		//for every slice, build a factor graph
		vector<Frame*> seqSlice = slices[sliceNumber]; //the slice we are working with 

		//train detectors
		ColorHistDetector chDetector;
		vector<Frame*> trainingFrames;
		trainingFrames.push_back(seqSlice.front()); //set training frame by index
		trainingFrames.push_back(seqSlice.back());
		
		chDetector.train(trainingFrames, params);

		vector<vector<vector<LimbLabel> > > detections; //numbers of labels per part, per frame, for this slice

		//initialise first level
		for (uint32_t i=0; i<seqSlice.size(); ++i) // access by reference, the type of i is int&
        {
        	detections.push_back(vector<vector<LimbLabel> >());
        	// for(auto&& j:i)
        	// {
        	// 	j.push_back(LimbLabel());
        	// }
        }

		//first do the detections, and store them

		for(uint32_t currentFrame=1; currentFrame<seqSlice.size()-1; ++currentFrame) //for every frame but first and last
		{
			//current frame represents the current frame, previous frame is currentFrame-1, next frame is currentFrame+1

			vector<vector<LimbLabel> > labels = chDetector.detect(seqSlice[currentFrame], params); //detect labels based on keyframe training

			detections[currentFrame] = labels; //store all detections into detections
		}

		vector<size_t> numbersOfLabels; //numbers of labels per part

		for(uint32_t i=0; i<detections.size(); ++i)
		{
			for(uint32_t j=0; j<detections[i].size(); ++j)
			{
				numbersOfLabels.push_back(detections[i][j].size());
			}
		} //numbers of labels now contains the numbers of labels, and their respective variations

		//use this to shape the space, this will be rather large
		Space space(numbersOfLabels.begin(), numbersOfLabels.end());
		Model gm(space);

		//now do the factors
		for(uint32_t currentFrame=1; currentFrame<seqSlice.size()-1; ++currentFrame) //for every frame but first and last
		{
			tree<BodyPart> partTree = seqSlice[currentFrame]->getSkeleton().getPartTree();
			tree<BodyPart>::iterator partIter, parentPartIter;

			//construct the image score cost factors
			//label score cost
			vector<vector<LimbLabel> > labels = detections[currentFrame];
			for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter) //for each of the detected parts
			{
				vector<int> varIndices; //create vector of indices of variables
				varIndices.push_back((currentFrame-1)*partTree.size()+partIter->getPartID()); //push first value in

				size_t scoreCostShape[]={labels[partIter->getPartID()].size()}; //number of labels
				
				ExplicitFunction<float> scoreCostFunc(scoreCostShape, scoreCostShape+1); //explicit function declare

				for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
				{
					scoreCostFunc(i) = computeScoreCost(labels[partIter->getPartID()].at(i), params); //compute the label score cost
				}

				Model::FunctionIdentifier scoreFid = gm.addFunction(scoreCostFunc); //explicit function add to graphical model
				gm.addFactor(scoreFid, varIndices.begin(), varIndices.end()); //bind to factor and variables

				if(currentFrame == 1) //anchor to first skeleton in sequence
				{
					//we already know the shape from the previous functions
					ExplicitFunction<float> anchorCostFunc(scoreCostShape, scoreCostShape+1); //explicit function declare

					for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
					{
						anchorCostFunc(i) = computeAnchorCost(labels[partIter->getPartID()].at(i), seqSlice[0], params);
					}

					Model::FunctionIdentifier anchorFid = gm.addFunction(anchorCostFunc); //explicit function add to graphical model
					gm.addFactor(anchorFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
				}
				else if(currentFrame==(seqSlice.size()-1)) //anchor to 
				{
					//we already know the shape from the previous functions
					ExplicitFunction<float> anchorCostFunc(scoreCostShape, scoreCostShape+1); //explicit function declare

					for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
					{
						anchorCostFunc(i) = computeAnchorCost(labels[partIter->getPartID()].at(i), seqSlice[seqSlice.size()-1], params);
					}

					Model::FunctionIdentifier anchorFid = gm.addFunction(anchorCostFunc); //explicit function add to graphical model
					gm.addFactor(anchorFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
				}

				if(partIter!=partTree.begin()) //if iterator is not on root node, there is always a parent body part
				{
					parentPartIter=partTree.parent(partIter); //find the parent of this part
					varIndices.push_back((currentFrame-1)*partTree.size()+parentPartIter->getPartID()); //push back parent partID as the second variable index

					size_t jointCostShape[]={labels[partIter->getPartID()].size(), labels[parentPartIter->getPartID()].size()}; //number of labels
					ExplicitFunction<float> jointCostFunc(jointCostShape, jointCostShape+2); //explicit function declare

					for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
					{
						for(uint32_t j=0; j<labels[parentPartIter->getPartID()].size(); ++j)
						{
							//for every child/parent pair, compute score
							jointCostFunc(i, j) = computeJointCost(labels[partIter->getPartID()].at(i), labels[parentPartIter->getPartID()].at(j), params);
						}
					}

					Model::FunctionIdentifier jointFid = gm.addFunction(jointCostFunc); //explicit function add to graphical model
					gm.addFactor(jointFid, varIndices.begin(), varIndices.end()); //bind to factor and variables

				}
				
				//futre temporal link (if not anchored)
				if(currentFrame!=seqSlice.size()-1)
				{
					varIndices.clear();
					varIndices.push_back((currentFrame-1)*partTree.size()+partIter->getPartID()); //push back parent partID as the second variable index
					varIndices.push_back((currentFrame)*partTree.size()+partIter->getPartID());

					size_t futureCostShape[]={labels[partIter->getPartID()].size(), detections[currentFrame+1][parentPartIter->getPartID()].size()}; //number of labels
					ExplicitFunction<float> futureTempCostFunc(futureCostShape, futureCostShape+1); //explicit function declare

					for(uint32_t i=0; i<labels[partIter->getPartID()].size(); ++i) //for each label in for this part
					{
						for(uint32_t j=0; j<detections[currentFrame+1][partIter->getPartID()].size(); ++j)
						{
							futureTempCostFunc(i,j) = computeFutureTempCost(labels[partIter->getPartID()].at(i), detections[currentFrame+1][partIter->getPartID()].at(j), params);
						}
					}
					Model::FunctionIdentifier futureTempCostFid = gm.addFunction(futureTempCostFunc); //explicit function add to graphical model
					gm.addFactor(futureTempCostFid, varIndices.begin(), varIndices.end()); //bind to factor and variables
				}
			}
		}

		//now solve the slice

		const size_t maxNumberOfIterations = 100;
	   	const double convergenceBound = 1e-7;
	   	const double damping = 0.5;
	   	BeliefPropagation::Parameter parameter(maxNumberOfIterations, convergenceBound, damping);
	   	BeliefPropagation bp(gm, parameter);
		
	   	// optimize (approximately)
		BeliefPropagation::VerboseVisitorType visitor;
		bp.infer(visitor);
		//pass to solve function

		// obtain the (approximate) argmin
		vector<size_t> labeling((detections.size()-2)*detections[1].size()); //number of non-anchor frames * number of bodyparts
		bp.arg(labeling);

		vector<vector<LimbLabel> > solutionLabels;
		for(uint32_t i=0; i<seqSlice.size(); ++i)
		{
			solutionLabels.push_back(vector<LimbLabel>());
			// for(uint32_t j=0; j<detections[i].size(); ++j)
			// {
			// 	solutionLabels[i].push_back(
			// }
		}

		for(uint32_t i=1; i<seqSlice.size()-1;++i) //frames
		{
			for(uint32_t j=0; j<detections[i].size(); ++j) //parts
			{
				solutionLabels[i].push_back(detections[i][j][(i-1)*detections[i].size()+j]); //pupulate solution vector
			}
		}

		vector<Solvlet> solvlets;
		//now set up a solvlet for every frame
		for(uint32_t i=1; i<seqSlice.size();++i)
		{
			solvlets.push_back(Solvlet(seqSlice[i]->getID(), solutionLabels[i]));
		}

		sequenceSolvlets.push_back(solvlets);
	}

	//sequence solvlets now contain all the solvlets for the entire sequence

	//@TODO change to solution?
	//convert the solvlet vector into a solution
	//Solution(int id, int solverId, int seqId, vector<float> params, vector<Solvlet> solvlets);

	//take information from solver parameters to do this


	return sequenceSolvlets;
}

float TLPSSolver::evaluateSolution(Frame* frame, vector<LimbLabel> labels, map<string, float> params)
{
	//Solution evaluator - how should this work? What should it return? What should it do?
	return 0;
}

int TLPSSolver::findFrameIndexById(int id, vector<Frame*> frames)
{
	for(uint32_t i=0; i<frames.size(); ++i)
	{
		if(frames[i]->getID()==id)
			return i;
	}
	return -1;
}

float TLPSSolver::computeScoreCost(const LimbLabel& label, map<string, float> params)
{
	//@FIX
	float lambda = params.at("imageCoeff");
	float scoreIndex = params.at("scoreIndex");
	//for now, just return the first available score
	vector<Score> scores = label.getScores();
	if(scores.size()>0)
	{
		return lambda*scores[scoreIndex].getScore();
	}
	else //if no scores present return -1
		return -1;
}
    
float TLPSSolver::computeJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params)
{
	//emplace default
	float lambda = params.at("jointCoeff");
	//@PARAM there should be a parameter that sets the joint connectivity penalty constant (alpha)
	//compute the joint joining cost for the skeleton
	Point2f p0, p1, c0, c1;

	//@FIX this is really too simplistic, connecting these points
	child.getEndpoints(c0,c1);
	parent.getEndpoints(p0,p1);

	//return the squared distance from the lower parent joint p1, to the upper child joint c0
	return lambda*(pow((c0.x-p1.x), 2)+pow((c0.y-p1.y), 2));
}

float TLPSSolver::computePriorCost(const LimbLabel& label, const BodyPart& prior, Skeleton& skeleton, map<string, float> params)
{
	//emplace default
	float lambda = params.at("priorCoeff");
	//@PARAM there should be a parameter that sets the strength of the penalty when deviating from the prior
	//compute the cost to interpolation prior
	Point2f p0, p1;
	label.getEndpoints(p0,p1);
	Point2f pp0 = skeleton.getBodyJoint(prior.getParentJoint())->getImageLocation();
	Point2f pp1 = skeleton.getBodyJoint(prior.getChildJoint())->getImageLocation();

	//return the sum of squared distances between the corresponding joints, prior to label
	return lambda*(pow((p0.x-pp0.x), 2)+pow((p0.y-pp0.y), 2)+pow((p1.x-pp1.x), 2)+pow((p1.y-pp1.y), 2));
}
    
float TLPSSolver::computePastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, map<string, float> params)
{
	//emplace default
	float lambda = params.at("tempCoeff");
	//compute the temporal connection cost to label in the past

	//@PARAM there should be a parameter that sets the temporal cost constant (beta)
	//compute the joint joining cost for the skeleton
	Point2f p0, p1, c0, c1;

	//@FIX this is really too simplistic, connecting these points
	thisLabel.getEndpoints(c0,c1);
	pastLabel.getEndpoints(p0,p1);

	//return the squared distance from the lower parent joint p1, to the upper child joint c0
	return lambda*(pow((c0.x-p0.x), 2)+pow((c0.y-p0.y), 2)+pow((c1.x-p1.x), 2)+pow((c1.y-p1.y), 2));
}

float TLPSSolver::computeFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, map<string, float> params)
{
	//emplace default
	float lambda = params.at("tempCoeff");
	//compute temporal connection cost to label in the future
	//@PARAM there needs to be a beta param here as well
	Point2f p0, p1, c0, c1;

	//@FIX this is really too simplistic, connecting these points
	thisLabel.getEndpoints(c0,c1);
	futureLabel.getEndpoints(p0,p1);

	//return the squared distance from the lower parent joint p1, to the upper child joint c0
	return lambda*(pow((c0.x-p0.x), 2)+pow((c0.y-p0.y), 2)+pow((c1.x-p1.x), 2)+pow((c1.y-p1.y), 2));
}

float TLPSSolver::computeAnchorCost(const LimbLabel& thisLabel, Frame* anchor, map<string, float> params)
{
	//emploace default
	float lambda = params.at("anchorCoeff");
	
	int limbId = thisLabel.getLimbID();

	vector<Point2f> partPolygon = anchor->getPartPolygon(limbId);
	vector<Point2f> labelPolygon = thisLabel.getPolygon();

	assert(partPolygon.size()==labelPolygon.size());

	//return the error between these'
	float score=0;
    for(size_t i=0; i<partPolygon.size(); ++i)
		score+= pow(partPolygon[i].x-labelPolygon[i].x, 2)+pow(partPolygon[i].y-labelPolygon[i].y,2);
	
	return lambda*score;
	//compute the cost of anchoring this label 
}

vector<vector<Frame*> > TLPSSolver::slice(const vector<Frame*>& frames) //separate the sequence into slices, for temporal solve
{
	//frames should be sliced into frame sets, where every non Keyframe non Lockframe frame should belong to a BOUNDED set
	//unbounded sets are not included in the solve
	vector<vector<Frame*> > aux;
	vector<vector<Frame*> > slices;

	vector<Frame*> currentSet;
	//bool isOpen;
	for(uint32_t i=0; i<frames.size(); ++i)
	{
		currentSet.push_back(frames[i]); //push the frame to current set
		if(frames[i]->getFrametype()==KEYFRAME || frames[i]->getFrametype()==LOCKFRAME)
		{
			aux.push_back(currentSet);
			currentSet.clear();
			currentSet.push_back(frames[i]);
		}
	}
	//now go through every set, and eliminate it if:
	//1) it contains 2 or less elements
	//2) it doesn't end with a LOCKFRAME or a KEYFRAME
	//3) it doesn't begin with a LOCKFRAME or a KEYFRAME
	for(uint32_t i=0;i<aux.size(); ++i)
	{
		if(aux[i].at(0)->getFrametype()==LOCKFRAME || aux[i].at(0)->getFrametype()==KEYFRAME) //if the set STARTS with a keyframe or a lockframe
		{
			if(aux[i].back()->getFrametype()==LOCKFRAME || aux[i].back()->getFrametype()==KEYFRAME) //if the set ENDS with a keyframe or a lockframe
			{
				if(aux[i].size()>2) //if size is greater than two elements
					slices.push_back(aux[i]); //push back slice
			}
		}
	}

	return slices; 
}

vector<Frame*> TLPSSolver::interpolateSlice(vector<Frame*> slice)//, map<string, float> params)
{
	//first make sure that the front and back frames HAVE 3D space locations computed
	//if not, compute them

	//check that the slice contains a keyframe/lockframe at each end
	assert(slice.front()->getFrametype()!=LOCKFRAME && slice.front()->getFrametype()!=KEYFRAME);
	assert(slice.back()->getFrametype()!=LOCKFRAME && slice.back()->getFrametype()!=KEYFRAME);
	
	vector<Frame*> result;

	for(uint32_t i=0; i<slice.size();++i)
	{
		Frame* ptr=NULL;
		result.push_back(ptr);
	}
	//attach the start and end, and generate everything in between in this function
	result[0] = slice[0];
	result[result.size()-1] = slice[slice.size()-1];

	//front and back are parent skeletons, their information 

	//matrix.create(3, 3, DataType<float>::type);
	//now we need to build a tree of AxisAngles that reflects the rotations
	Skeleton prevSkel = slice.front()->getSkeleton();
	Skeleton futureSkel = slice.back()->getSkeleton();

	//set up the two part trees, the part tree is used for structure
	tree<BodyPart> partTree = slice.front()->getSkeleton().getPartTree();
	tree<BodyPart>::iterator partIter, parentIter; //the body part iterator

	vector<Eigen::Quaternionf> rotations;
	vector<Eigen::Vector3f> unrotatedPast;
	vector<Eigen::Vector3f> unrotatedFuture;

	for(uint32_t i=0; i<partTree.size(); ++i)
	{
		rotations.push_back(Eigen::Quaternionf());
		//unrotatedNodes.push_back(Eigen::Vector3f());
	}

	for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter)
	{	
		//get partent and child joints of previous part
		BodyJoint* childJointP = prevSkel.getBodyJoint(partIter->getChildJoint());
		BodyJoint* parentJointP = prevSkel.getBodyJoint(partIter->getParentJoint());

		//get parent and child joints of future part
		BodyJoint* childJointF = futureSkel.getBodyJoint(partIter->getChildJoint());
		BodyJoint* parentJointF = futureSkel.getBodyJoint(partIter->getParentJoint());

		//convert Point3f to Vector3f
		Point3f childLocP = childJointP->getSpaceLocation();
		Point3f parentLocP = parentJointP->getSpaceLocation();

		Point3f childLocF = childJointF->getSpaceLocation();
		Point3f parentLocF = parentJointF->getSpaceLocation();

		Eigen::Vector3f prevVec(childLocP.x-parentLocP.x, childLocP.y-parentLocP.y, childLocP.z-parentLocP.z);
		Eigen::Vector3f futureVec(childLocF.x-parentLocF.x, childLocF.y-parentLocF.y, childLocF.z-parentLocF.z);


		//get all rotations that need to happen
		vector<Eigen::Quaternionf> previousNodes;
 		do
		{
			parentIter = partTree.parent(partIter);
			if(parentIter!=NULL)
			{
	        	previousNodes.push_back(rotations[parentIter->getPartID()]);
	        }
        } while(parentIter!=NULL);
        
        //now unrotate starting from root down
        for(uint32_t i=previousNodes.size(); i>=0; --i)
        {
        	prevVec = previousNodes[i].conjugate()._transformVector(prevVec);
        	futureVec = previousNodes[i].conjugate()._transformVector(prevVec);
        }
		//Eigen::Vector3f xaxis(1,0,0);

		//compute quaterion between two positions
		Quaternionf rotation;
		rotation.setFromTwoVectors(prevVec, futureVec); //quaternion set
		rotations[partIter->getPartID()] = rotation.normalized(); //push to vector of rotations
		unrotatedPast[partIter->getPartID()] = prevVec;
		unrotatedFuture[partIter->getPartID()] = futureVec;
	}

	//now that quaterion rotations are computed, we can compute the new skeleton
	for(uint32_t i=1; i<slice.size()-1; ++i)
	{
		//only the t value depends on frame number
		Skeleton interpolatedSkeleton = prevSkel;
		vector<Vector3f> currentPartState;
		partTree = 	interpolatedSkeleton.getPartTree();	

		for(uint32_t j=0; j<partTree.size(); ++j)
		{
			currentPartState.push_back(Eigen::Vector3f());
		}

		for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter)
		{

			Eigen::Vector3f prevVec=unrotatedPast[partIter->getPartID()];
			//rotate by this quaternion, and every quaternion up in the tree
			Quaternionf thisRot = rotations[partIter->getPartID()];

			float angle = 2*acos(thisRot.w());

			float xa = thisRot.x()/sqrt(1.0-thisRot.w()*thisRot.w());
			float ya = thisRot.y()/sqrt(1.0-thisRot.w()*thisRot.w());
			float za = thisRot.z()/sqrt(1.0-thisRot.w()*thisRot.w());

			Vector3f thisAxis(xa,ya,za);

			angle = interpolateFloat(0, angle, i, slice.size()); //interpolate the angle

			//re-generate AngleAxis object and create quaternion

			//Quaternion to axis angle
			// 			angle = 2 * acos(qw)
			// x = qx / sqrt(1-qw*qw)
			// y = qy / sqrt(1-qw*qw)
			// z = qz / sqrt(1-qw*qw)
			//Quaternion<float> q;  q = AngleAxis<float>(angle_in_radian, axis);

			thisRot = AngleAxisf(angle,thisAxis);

			prevVec = thisRot._transformVector(prevVec); //rotate the vector by 

			//convert rotation to axis angle, interpolate angle, convert back to quaternion
						
	       	//rotate hierarchy
	        do
	        {
				parentIter = partTree.parent(partIter);
				if(parentIter!=NULL)
				{
		        	//if there is a parent rotate by its quaternion
		        	Eigen::Quaternionf prevQuat = rotations[parentIter->getPartID()];

		        	float angleP = 2*acos(prevQuat.w());

					float xp = prevQuat.x()/sqrt(1.0-prevQuat.w()*prevQuat.w());
					float yp = prevQuat.y()/sqrt(1.0-prevQuat.w()*prevQuat.w());
					float zp = prevQuat.z()/sqrt(1.0-prevQuat.w()*prevQuat.w());

					Vector3f prevAxis(xp,yp,zp);

					angleP = interpolateFloat(0, angleP, i, slice.size()); //interpolate the angle

		        	prevQuat = AngleAxisf(angleP, prevAxis);

		        	prevVec = prevQuat._transformVector(prevVec);
		        }
	        } while(parentIter!=NULL);

	        currentPartState[partIter->getPartID()] = prevVec;
		}
		//now update skeleton accordingly - locations just need to be added together (i.e. add child joint to vector) 

		for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter)
		{
			Vector3f childJoint = currentPartState[partIter->getPartID()];
			Vector3f parentJoint(0,0,0);

	        do
	        {
				parentIter = partTree.parent(partIter);
				if(parentIter!=NULL)
				{
		        	//if there is a parent, first unrotate by its quaternion
		        	childJoint=childJoint+currentPartState[parentIter->getPartID()];
		        	parentJoint=parentJoint+currentPartState[parentIter->getPartID()];
		        }
	        } while(parentIter!=NULL);

	        //parent and child joints now contain the 
    		BodyJoint* childJointT = interpolatedSkeleton.getBodyJoint(partIter->getChildJoint());
			BodyJoint* parentJointT = interpolatedSkeleton.getBodyJoint(partIter->getParentJoint());
			childJointT->setSpaceLocation(Point3f(childJoint.x(), childJoint.y(), childJoint.z()));
			parentJointT->setSpaceLocation(Point3f(parentJoint.x(), parentJoint.y(), parentJoint.z()));
		}

		//now skeleton should be set for this frame
		//slice[i].setSkeleton(interpolatedSkeleton);

		//frame type should be updated to interpolaion
		Interpolation interpolatedFrame;
		interpolatedFrame.setSkeleton(interpolatedSkeleton);
		interpolatedFrame.setID(slice[i]->getID());
		interpolatedFrame.setMask(slice[i]->getMask());
		interpolatedFrame.setGroundPoint(slice[i]->getGroundPoint());
		interpolatedFrame.setImage(slice[i]->getImage());

		//delete slice[i];
		result[i] = &interpolatedFrame;
	}

	return result; //result contains Frame*'s that have been modified according to 

}

inline float TLPSSolver::interpolateFloat(float prevAngle, float nextAngle, int step, int numSteps)
{
    float t;
    if(numSteps!=0)
        t = (float)step/(float)numSteps;
    else
        t=0;
    return prevAngle*(1-t)+nextAngle*t;
}

