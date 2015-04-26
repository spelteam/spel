#include "solvlet.hpp"

Solvlet::Solvlet(void)
{
	frameId=-1;
}
Solvlet::Solvlet(int id, vector<LimbLabel> _labels)
{
	frameId=id;
	labels = _labels;
}
    
Solvlet &Solvlet::operator=(const Solvlet &s)
{
  if (this == &s)
  {
    return *this;
  }
  this->setLabels(s.getLabels());
  this->setFrameID(s.getFrameID());
  return *this;
}

// bool operator==(const Solvlet &s) const
// {
// 	if(s.getFrameId==frameId && s.)
// }

// bool operator!=(const Solvlet &s) const
// {

// }

int Solvlet::getFrameID(void) const
{
	return frameId;
}

void Solvlet::setFrameID(int _id)
{
	frameId=_id;
}

vector<LimbLabel> Solvlet::getLabels(void) const
{
	return labels;
}
void Solvlet::setLabels(vector<LimbLabel> _labels)
{
	labels = _labels;
}

Skeleton Solvlet::toSkeleton(const Skeleton &example)
{
    Skeleton retSkel = example;

    tree<BodyPart> partTree = retSkel.getPartTree();
    tree<BodyJoint> jointTree = retSkel.getJointTree();
    tree<BodyJoint>::iterator cjIter, pjIter, jointIter;

    assert(partTree.size()==labels.size()); //there should be the same number of body parts

    for(tree<BodyPart>::iterator partIter=partTree.begin(); partIter!=partTree.end(); ++partIter)
    {   
        for(int i=0; i<labels.size(); ++i)
        {
            if(partIter->getPartID()==labels[i].getLimbID()) //if you find the right label
            {
                //get joint IDs
                int partID = partIter->getPartID(); //part
                int cJointID = partIter->getChildJoint(); //child joint
                int pJointID = partIter->getParentJoint(); //parent joint

                Point2f pj,cj; //parent and child joints from label
                labels[i].getEndpoints(pj,cj); //set them from label


                cjIter=jointTree.end();
                pjIter=jointTree.end();

                //identify these nodes in the joint tree
                for(jointIter=jointTree.begin(); jointIter!=jointTree.end(); ++jointIter)
                {
                    if(jointIter->getLimbID()==cJointID)
                        cjIter=jointIter;
                    if(jointIter->getLimbID()==pJointID)
                        pjIter=jointIter;
                    if(cjIter!=jointTree.end() && pjIter!=jointTree.end())
                        break;
                }

                if(partID==0) //root
                {
                    //set 2D joint locations
                    pjIter->setImageLocation(pj); //set porent joint only for root node
                }
                cjIter->setImageLocation(cj); //set child joint for all other nodes
            } //TODO: introduce a more complex scheme of doing this, such as finding midpoints, or points
        }
    }

    retSkel.setJointTree(jointTree);
    retSkel.infer3D();

    return retSkel;
}


//Skeleton MainWindow::skeletonFromLabels(vector<LimbLabel> labels)
//{
//    QLOG_TRACE() << "skeletonFromLabels()" << endl;
//    Skeleton skel;
//    tree<Point3f> * partTree = skel.getPartTree();
//    tree<Point3f>::iterator iter, parent, child;

//    if(labels.size()!=skel.getNumBones())
//    {
//        cerr << "Incorrect number of labels. " << endl;
//        return skel;
//    }
//    else
//    {
//        //build a skeleton from the labels
//        for(iter=partTree->begin(); iter!=partTree->end(); ++iter)
//        {
//            //compute intersection from child to parent
//            //making all the fake parts (1-5) simply connect the non-fake parts
//            //this leaves the torso unmodified
//            if(iter->x==0) //torso
//            {
//                Point2f x,y;
//                labels[0].getEndpoints(x,y);
//                skel.setImageJoint(0, x);
//                skel.setImageJoint(1, y);
//            }
//            else if(iter->x<=10) //"fake" parts, just stretch from label to label, so these remain unchanged
//            {
//                vector<Point2i> bones(skel.getBones());
//                LimbLabel currentLabel = labels[iter->x];
//                Point2f cx, cy;
//                currentLabel.getEndpoints(cx,cy);

//                //1 and 2 connect to bottom (px)
//                //3 4 and 5 to top (py)

//                int pJoint = bones[iter->x].x;
//                int cJoint = bones[iter->x].y;
//                //we know parent is the root bone
//                //we are always setting the y bone, the x bone is set by torso already

//                skel.setImageJoint(pJoint,cx);
//                skel.setImageJoint(cJoint,cy);
//            }
//            else
//            {
//                vector<Point2i> bones(skel.getBones());

//                LimbLabel currentLabel = labels[iter->x];
//                parent = partTree->parent(iter);
//                LimbLabel parentLabel = labels[parent->x];

//                Point2f px, py, cx, cy;
//                currentLabel.getEndpoints(cx,cy);
//                parentLabel.getEndpoints(px,py);

//                //compute midpoint between the bottom of parent (py) and top of current bone (cx), and set it as the joint

//                //py = skel.get2Djoints()[bones[iter->x].x];

//                Point2f mid = 0.5*cx+0.5*py;

//                int index = bones[iter->x].x; //first joint of child

//                skel.setImageJoint(index, mid); //set to midpoint between the two
//                if(iter->x>=13 || iter->x==9)
//                {
//                    //these are also tips, set them to label's cy
//                    skel.setImageJoint(bones[iter->x].y, cy);
//                }
//            //altering orientation slightly
//            }
//        }
//        return skel;
//    }
//    return skel;
//}
