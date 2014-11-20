#include "tlpssolver.hpp"
#include "lockframe.hpp"
#include "colorHistDetector.hpp"
#include "math.h"

TLPSSolver::TLPSSolver()
{
	id=0;
	name="TLPS";
}

TLPSSolver::~TLPSSolver()
{

}

Solution TLPSSolver::solve(const vector<Frame*>& frames) //inherited virtual
{
	map<string, float> params; //set the default parameters vector

	//set some parameter deefaults
	params.at("scoreIndex")=0;
	
	//pass to next level solve function, for ISM computing
	return this->solve(frames, params);
}

Solution TLPSSolver::solve(const vector<Frame*>& frames, map<string, float> params) //inherited virtual
{
	Solution solution;

	//first slice up the sequences
	vector<vector<Frame*> > slices = slice(frames, params);

	for(uint sliceNumber=0; sliceNumber<slices.size(); ++sliceNumber)
	{
		//for every slice, build a factor graph
		vector<Frame*> seqSlice = slices[sliceNumber]; //the slice we are working with 

		//train detectors
		ColorHistDetector chDetector;
		vector<Frame*> trainingFrames;
		trainingFrames.push_back(seqSlice.front()); //set training frame by index
		trainingFrames.push_back(seqSlice.back());
		
		chDetector.train(trainingFrames, params);

		//no need to detect on keyframes and lockframes, use their joints to link to rather than labels
		vector<vector<LimbLabel> > pastLabels;
		for(uint currentFrame=1; currentFrame<seqSlice.size()-1; ++currentFrame) //for every frame but first and last
		{
			//current frame represents the current frame, previous frame is currentFrame-1, next frame is currentFrame+1

			vector<vector<LimbLabel> > labels = chDetector.detect(frames[currentFrame], params); //detect labels based on keyframe training

			vector<size_t> numbersOfLabels; //numbers of labels per part

			for(uint i=0; i<labels.size(); ++i)
			{
				numbersOfLabels.push_back(labels[i].size());
			} //numbers of labels now contains the numbers

			Space space(numbersOfLabels.begin(), numbersOfLabels.end());
			Model gm(space);

			tree<BodyPart> partTree = frames[currentFrame]->getSkeleton().getPartTree();
			tree<BodyPart>::iterator partIter, parentPartIter;

			//construct the image score cost factors

			//compute the prior cost (if any)

			//construct the future temporal link

			//construct the past temporal link
		}
	}

	//pass to solve function
	return solution;
}

float TLPSSolver::evaluateSolution(Frame* frame, vector<LimbLabel> labels, map<string, float> params)
{
	//Solution evaluator
	return 0;
}

int TLPSSolver::findFrameIndexById(int id, vector<Frame*> frames)
{
	for(uint i=0; i<frames.size(); ++i)
	{
		if(frames[i]->getID()==id)
			return i;
	}
	return -1;
}

float TLPSSolver::computeScoreCost(const LimbLabel& label, map<string, float> params)
{
	//@FIX
	//for now, just return the first available score
	vector<Score> scores = label.getScores();
	if(scores.size()>0)
	{
		int scoreIndexString = params.at("scoreIndex");
		return scores[scoreIndexString].getScore();
	}
	else //if no scores present return -1
		return -1;
}
    
float TLPSSolver::computeJointCost(const LimbLabel& child, const LimbLabel& parent, map<string, float> params)
{
	//@PARAM there should be a parameter that sets the joint connectivity penalty constant (alpha)
	//compute the joint joining cost for the skeleton
	Point2f p0, p1, c0, c1;

	//@FIX this is really too simplistic, connecting these points
	child.getEndpoints(c0,c1);
	parent.getEndpoints(p0,p1);

	//return the squared distance from the lower parent joint p1, to the upper child joint c0
	return pow((c0.x-p1.x), 2)+pow((c0.y-p1.y), 2);
}

float TLPSSolver::computePriorCost(const LimbLabel& label, const BodyPart& prior, Skeleton& skeleton, map<string, float> params)
{
	//@PARAM there should be a parameter that sets the strength of the penalty when deviating from the prior
	//compute the cost to interpolation prior
	Point2f p0, p1;
	label.getEndpoints(p0,p1);
	Point2f pp0 = skeleton.getBodyJoint(prior.getParentJoint())->getImageLocation();
	Point2f pp1 = skeleton.getBodyJoint(prior.getChildJoint())->getImageLocation();

	//return the sum of squared distances between the corresponding joints, prior to label
	return pow((p0.x-pp0.x), 2)+pow((p0.y-pp0.y), 2)+pow((p1.x-pp1.x), 2)+pow((p1.y-pp1.y), 2);
}
    
float TLPSSolver::computePastTempCost(const LimbLabel& thisLabel, const LimbLabel& pastLabel, map<string, float> params)
{
	//compute the temporal connection cost to label in the past

	//@PARAM there should be a parameter that sets the temporal cost constant (beta)
	//compute the joint joining cost for the skeleton
	Point2f p0, p1, c0, c1;

	//@FIX this is really too simplistic, connecting these points
	thisLabel.getEndpoints(c0,c1);
	pastLabel.getEndpoints(p0,p1);

	//return the squared distance from the lower parent joint p1, to the upper child joint c0
	return pow((c0.x-p0.x), 2)+pow((c0.y-p0.y), 2)+pow((c1.x-p1.x), 2)+pow((c1.y-p1.y), 2);
}

float TLPSSolver::computeFutureTempCost(const LimbLabel& thisLabel, const LimbLabel& futureLabel, map<string, float> params)
{
	//compute temporal connection cost to label in the future
	//@PARAM there needs to be a beta param here as well
	Point2f p0, p1, c0, c1;

	//@FIX this is really too simplistic, connecting these points
	thisLabel.getEndpoints(c0,c1);
	futureLabel.getEndpoints(p0,p1);

	//return the squared distance from the lower parent joint p1, to the upper child joint c0
	return pow((c0.x-p0.x), 2)+pow((c0.y-p0.y), 2)+pow((c1.x-p1.x), 2)+pow((c1.y-p1.y), 2);
}

float TLPSSolver::computeAnchorCost(const LimbLabel& thisLabel, Frame* anchor, map<string, float> params)
{
	return 0;
	//compute the cost of anchoring this label 
}

vector<vector<Frame*> > TLPSSolver::slice(const vector<Frame*>& frames, map<string, float> params) //separate the sequence into slices, for temporal solve
{
	//frames should be sliced into frame sets, where every non Keyframe non Lockframe frame should belong to a BOUNDED set
	//unbounded sets are not included in the solve
	vector<vector<Frame*> > aux;
	vector<vector<Frame*> > slices;

	vector<Frame*> currentSet;
	bool isOpen;
	for(uint i=0; i<frames.size(); ++i)
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
	for(uint i=0;i<aux.size(); ++i)
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

//epsilon is a very small positive value
//angle is in radians
TLPSSolver::AxisAngle TLPSSolver::computeAxisAngle(Point3f vec1, Point3f vec2, float epsilon)
{
	//two cases need to be handled
	//1. angle approaches 0 - zero rotation angle
	//2. angle approaches 180 - any perpendicular axis is the axis of rotation

	//normalise the vectors
	float nVec1 = 1.0/sqrt(vec1.x*vec1.x+vec1.y*vec1.y+vec1.z*vec1.z);
	float nVec2 = 1.0/sqrt(vec2.x*vec2.x+vec2.y*vec2.y+vec2.z*vec2.z);

	// vec1 = vec1*nVec1;
	// vec2 = vec2*nVec2;

	//angle is the arccos dot product of the two normalised vectors
	float angle = acos(vec1.dot(vec2)*nVec2*nVec2);

	Point3f axis;
	//depending on what the angle is, decide how the axis should be computed
	if(angle<=epsilon) //nearly zero angle
	{
		axis = Point3f(0,0,0); //zero axis 
	}
	else if(angle<=PI+epsilon && angle>=PI-epsilon) //if angle is within epsilon of PI (almost 180 degrees)
	{
		axis = Point3f(-vec1.y, vec1.x, vec1.z);//then axis = vec1 rotated by 90 degrees
	}
	else //if we are not in any of the exceptional cases
	{
		//axis is the cross product of the two vectors
		axis = vec1.cross(vec2);
		float aNorm = sqrt(axis.x*axis.x+axis.y*axis.y+axis.z*axis.z);
		axis = axis*(1.0/aNorm); //normalise the axis
	}
	//set the rotation
	TLPSSolver::AxisAngle answer;
	answer.axis = axis;
	answer.angle = angle;

	return answer; //return the axis and angle
}

//convert AxisAngles to a 3x3 rotation matrix
Mat TLPSSolver::axisAngleToRotMatrix(TLPSSolver::AxisAngle input)
{
	//http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/

	float c,s,t, x, y, z;
	c = cos(input.angle);
	s = sin(input.angle);
	t = 1-c;

	//for x,y,z normalise the axis vector first
	Point3f axis = input.axis;
	float axisNorm = sqrt(axis.x*axis.x+axis.y*axis.y+axis.z*axis.z);
	axis = axis*(1.0/axisNorm);

	x = axis.x;
	y = axis.y;
	z = axis.z;

	Mat matrix;
	matrix.create(3, 3, DataType<float>::type);
	matrix.at<float>(0,0) = t*x*x+c;
	matrix.at<float>(0,1) = t*x*y-z*s;
	matrix.at<float>(0,2) = t*x*z+y*s;

	matrix.at<float>(1,0) = t*x*y+z*s;
	matrix.at<float>(1,1) = t*y*y+c;
	matrix.at<float>(1,2) = t*y*z-x*s;

	matrix.at<float>(2,0) = t*x*z-y*s;
	matrix.at<float>(2,1) = t*y*z+x*s;
	matrix.at<float>(2,2) = t*z*z+c;

	return matrix;
}

//get rotation matrix that maps one vector to another in 3D space
Mat TLPSSolver::mapVectorRotation(Point3f vec1, Point3f vec2, float epsilon)
{
	return axisAngleToRotMatrix(computeAxisAngle(vec1,vec2,epsilon));
}

vector<Frame*> TLPSSolver::interpolateSlice(vector<Frame*> slice)//, map<string, float> params)
{
	//first make sure that the front and back frames HAVE 3D space locations computed
	//if not, compute them

	//check that the slice contains a keyframe/lockframe at each end
	assert(slice.front()->getFrametype()!=LOCKFRAME && slice.front()->getFrametype()!=KEYFRAME);
	assert(slice.back()->getFrametype()!=LOCKFRAME && slice.back()->getFrametype()!=KEYFRAME);
	
	//front and back are parent skeletons, their information 

	//matrix.create(3, 3, DataType<float>::type);
	//now we need to build a tree of AxisAngles that reflects the rotations
	Skeleton prevSkel = slice.front()->getSkeleton();
	Skeleton futureSkel = slice.back()->getSkeleton();

	//set up the two part trees, the part tree is used for structure
	tree<BodyPart> partTree = slice.front()->getSkeleton().getPartTree();
	tree<BodyPart>::iterator partIter, parentIter; //the body part iterator

	vector<TLPSSolver::AxisAngle> prevRots; //previous keyframe rotation matrix
	vector<TLPSSolver::AxisAngle> futureRots; //nextKeyframe rotation matrix

	//free matrix memory with release() function
	//http://answers.opencv.org/question/14285/how-to-free-memory-through-cvmat/

	//convert the two into rotations for front tree
	Point3f xAxis(1.0,0,0);

	BodyPart rootPart = *partTree.begin();
	BodyJoint* childJointP = prevSkel.getBodyJoint(rootPart.getChildJoint());
	BodyJoint* parentJointP = prevSkel.getBodyJoint(rootPart.getParentJoint());

	Point3f childLocP = childJointP->getSpaceLocation();
	Point3f parentLocP = parentJointP->getSpaceLocation();

	TLPSSolver::AxisAngle initialRotP = computeAxisAngle(xAxis, childLocP-parentLocP, 0.0005); //initial previous rotation

	BodyJoint* childJointF = futureSkel.getBodyJoint(rootPart.getChildJoint());
	BodyJoint* parentJointF = futureSkel.getBodyJoint(rootPart.getParentJoint());

	Point3f childLocF = childJointF->getSpaceLocation();
	Point3f parentLocF = parentJointF->getSpaceLocation();

	TLPSSolver::AxisAngle initialRotF = computeAxisAngle(xAxis, childLocF-parentLocF, 0.0005); //initial future rotation

	// // //insert tip
	// // prevRotIter = prevRots.insert(prevRots.begin(), initialRotP);
	// // futureRotIter = futureRots.insert(futureRots.begin(), initialRotF);
	
	// //initialise rotation vectors by pushing 3x3 matrices into them
	// for(uint i=0; i<partTree.size(); ++i)
	// {
	// 	prevRots.push_back(TLPSSolver::AxisAngle());
	// 	futureRots.push_back(TLPSSolver::AxisAngle());
	// }

	// prevRots[0] = initialRotP;
	// futureRots[0] = initialRotF;

	// //at this point, the root has already been set to zero
	// for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter)
	// {
	// 	//for prevSkel and futureSkel, at the same time
	// 	//recover joints
	// 	childJointP = prevSkel.getBodyJoint(partIter->getChildJoint());
	// 	parentJointP = prevSkel.getBodyJoint(partIter->getParentJoint());

	// 	childJointF = futureSkel.getBodyJoint(partIter->getChildJoint());
	// 	parentJointF = futureSkel.getBodyJoint(partIter->getParentJoint());
		
	// 	//at every part, for both, past and future skel, translate parent joint to origin
	// 	childLocF = childJointF->getSpaceLocation();
	// 	parentLocF = parentJointF->getSpaceLocation();

	// 	childLocP = childJointP->getSpaceLocation();
	// 	parentLocP = parentJointP->getSpaceLocation();

	// 	Point3f partVecP = childLocP-parentLocP;
	// 	Point3f partVecF = childLocF-parentLocF;
		
	// 	//the vectors are the child-parent, these need to be unrotated
	// 	//unrotate xAxis and child joint by multiplying by all rotation matrix inverses that are up in the hierarchy
	// 	//find path to parent node
		
	// 	while(partTree.parent(partIter)!=partTree.begin())
	// 	{
	// 		//multiply by parent matrix
	// 		partVecP = rotate(partVecP, prevRots[partIter->getPartID()]);
	// 		partVecF = rotate(partVecF, futureRots[partIter->getPartID()]);

	// 		parentIter = partTree.parent(partIter);
	// 	}


	// 	//then compute the rotation between xAxis and child joint vectorhttp://stackoverflow.com/questions/4099369/interpolate-between-rotation-matrices
	// 	TLPSSolver::AxisAngle initialRotP = computeAxisAngle(xAxis, childLocP-parentLocP); //initial previous rotation
	// 	TLPSSolver::AxisAngle initialRotF = computeAxisAngle(xAxis, childLocF-parentLocF); //initial future rotation

	// 	//finally, store the rotation matrix
	// 	prevRots[partIter->getPartID()] = initialRotP;
	// 	futureRots[partIter->getPartID()] = initialRotF;
			
	// }
	// //at the end of this step we should have all rotation stored as axis-angles
	// //these shoulod be accessed in order of the tree

	// //now for every frame, for every part, compute the interpolated rotation, create rotated skeleton (3D), and set the frame accordingly
	// for(uint frame=1; frame<slice.size()-1; ++i) //frame is the frame ID within the slice
	// {
	// 	//create a new skeleton of the same type as previous
	// 	Skeleton newSkeleton; //this is the new skeleton for this frame
		
	// 	Interpolation newFrame; //this is the new frame => copy everything from existing frame
	// 	newFrame.setImage(slice[frame]->getImage()); //set the image
	// 	newFrame.setMask(slice[frame]->getMask()); //set the mask
	// 	newFrame.setGroundPoint(slice[frame]->getGroundPoint()); //set the ground point
	// 	newFrame.setId(slice[frame]->getId());

	// 	newSkeleton = prevSkel; //newSkeleton is a copy of prevSkel

	// 	//now for every body part
	// 	for(partIter=partTree.begin(); partIter!=partTree.end(); ++partIter)
	// 	{
	// 		//compute the interpolated quaternion rotaiton
	// 		Quaternion pastQuat, futureQuat;


	// 	}
	// }

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

Point3f TLPSSolver::rotate(Point3f point, Mat rotationMatrix)
{
	//first make sure rotation matrix is of the right size
	assert(rotationMatrix.rows == 3);
	assert(rotationMatrix.cols ==3);

	//if it is, continue
	Point3f result;
	result.x = rotationMatrix.at<float>(0,0)*point.x+rotationMatrix.at<float>(0,1)*point.y+rotationMatrix.at<float>(0,2)*point.z;
	result.y = rotationMatrix.at<float>(1,0)*point.x+rotationMatrix.at<float>(1,1)*point.y+rotationMatrix.at<float>(1,2)*point.z;
	result.z = rotationMatrix.at<float>(2,0)*point.x+rotationMatrix.at<float>(2,1)*point.y+rotationMatrix.at<float>(2,2)*point.z;

	return result;
}

//Quaternions 
//http://3dgep.com/understanding-quaternions/#Quaternion_Exponentiation
//http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/code/index.htm#scale
//http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/arithmetic/index.htm


// Quaternion axisAngleToQuaternion(AxisAngle axisangle)
// {
// 	Quaternion quat;
// 	Point3f axis = axisangle.axis;
// 	float angle = axisange.angle;
// 	quat.x = axis.x*sin(angle/2.0);
// 	quat.y = axis.y*sin(angle/2.0);
// 	quat.z = axis.z*sin(angle/2.0);
// 	quat.w = cos(angle/2.0);

// 	return quat;
// }

// Quaternion quaternionSLERP(Quaterion q1, Quaternion q2, float t)
// {
// 	//make sure t is between 0 and 1
// 	assert(t<1.0 && t>0);

// 	return q1*pow((q1.conjugate()*q2), t);
// }

// Quaternion pow(Quaternion quat, float power)
// {
// 	return exp(t*log(quat));
// }