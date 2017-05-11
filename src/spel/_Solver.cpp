// This is an open source non-commercial project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "lockframe.hpp"

#include "colorHistDetector.hpp"
#include "hogDetector.hpp"
#include "surfDetector.hpp"
#include "SURFDetector2.hpp"

#include "_Solver.hpp"
#include "spelGeometry.hpp"

#include "spelParameters.hpp"

namespace SPEL
{
  std::vector<std::vector<Frame*>> _Solver::createSlices(std::vector<Frame*> &frames)
  {
    long t0 = clock();
    std::vector<std::vector<Frame*>> Slices;

    std::vector<Frame*> temp;
    int keyframesCount = 0;
    Frame* frame = 0;
    for (int i = 0; i < frames.size(); i++)
    {
      frame = 0;
      while(i < frames.size() && frames[i]->getFrametype() == KEYFRAME)
      {
        frame = frames[i];
        i++;
      }
      if (frame != 0)
      {
        keyframesCount++;
        temp.push_back(frame);
      }
      while(i < frames.size() && frames[i]->getFrametype() != KEYFRAME)
      {
        temp.push_back(frames[i]);
        i++;
      }
      if (i < frames.size())
      {
        temp.push_back(frames[i]);
        keyframesCount++;
        i--;
      }

      if(keyframesCount >= 1 && keyframesCount < temp.size())
        Slices.push_back(temp);
      temp.clear();     
    }
    long t1 = clock();

    return Slices;
  }

//=========================================================================
//"IndexedSkeletonModel" class

  IndexedSkeletonModel::JointLink::JointLink(int partID_, int isChildJoint)
  {
    partID = partID_;
    num = isChildJoint;
  }
  
  IndexedSkeletonModel::IndexedSkeletonModel(Skeleton skeleton, std::string name)
  {
    fromSkeleton(skeleton, name);
  }

  IndexedSkeletonModel::~IndexedSkeletonModel()
  {
    clear();
  }

  IndexedSkeletonModel::partAdjacentsJoints IndexedSkeletonModel::getPartConnections(int partID)
  {
    return parts.at(partID);
  }

  void IndexedSkeletonModel::fromSkeleton(Skeleton skeleton, std::string name)
  {
    m_name = name;
    createPartsConnections(skeleton);
    createIndexedJoints(skeleton);
  }

  void IndexedSkeletonModel::createPartsConnections(Skeleton &skeleton)
  {
    tree<BodyPart> partTree = skeleton.getPartTree();
    tree<BodyJoint> jointTree = skeleton.getJointTree();
    int partsCount = partTree.size();
    int jointsCount = jointTree.size();

    std::map<int, std::vector<int>> PartsByParentJoint;
    std::map<int, std::vector<int>> PartsByChildJoint;
    for (int i = 0; i < jointsCount; i++)
    {
      PartsByParentJoint.emplace(std::pair<int, std::vector<int>>(i, std::vector<int>()));
      PartsByChildJoint.emplace(std::pair<int, std::vector<int>>(i, std::vector<int>()));
    }

    // Parts ID indexed by joints ID
    tree<BodyPart>:: pre_order_iterator p = partTree.begin();
    for (tree<BodyPart>::iterator p = partTree.begin(); p != partTree.end(); p++)
    {
      int j0_id = p->getParentJoint();
      int j1_id = p->getChildJoint();
      int partID = p->getPartID();
      PartsByParentJoint.at(j0_id).push_back(partID);
      PartsByChildJoint.at(j1_id).push_back(partID);
    }

    std::map<uint32_t, partAdjacentsJoints> bodyBarts;
    for(int i = 0; i < partsCount; i++)
      bodyBarts.emplace(std::pair<uint32_t, partAdjacentsJoints>(i, partAdjacentsJoints()) );

    // Joints ID indexed by Part ID
    for (tree<BodyPart>::iterator p = partTree.begin(); p != partTree.end(); p++)
    {
      int j0_id = p->getParentJoint();
      int j1_id = p->getChildJoint();
      int partID = p->getPartID();

      std::vector<JointLink> ChildJointConnections;
      std::vector<JointLink> ParentJointConnections;

      // Create parent joint connections list
      for (int i = 0; i < PartsByParentJoint[j0_id].size(); i++)
        if (PartsByParentJoint[j0_id][i] != partID)
          ParentJointConnections.push_back(JointLink(PartsByParentJoint[j0_id][i], 0)); // Parent joints of another parts
      for (int i = 0; i < PartsByChildJoint[j0_id].size(); i++)
        if (PartsByChildJoint[j0_id][i] != partID)
          ParentJointConnections.push_back(JointLink(PartsByChildJoint[j0_id][i], 1)); // Child joints of another parts

      // Create child joint connections list
      for (int i = 0; i < PartsByParentJoint[j1_id].size(); i++)
        if(PartsByParentJoint[j1_id][i] != partID)
         ChildJointConnections.push_back(JointLink(PartsByParentJoint[j1_id][i], 0)); // Can be connected only to a parent joint of anoter parts

      bodyBarts[partID].ChildJointConnections = ChildJointConnections;
      bodyBarts[partID].ParentJointConnections = ParentJointConnections;
    }

    PartsByParentJoint.clear();
    PartsByChildJoint.clear();
  
    parts = bodyBarts;
  }

  void IndexedSkeletonModel::createIndexedJoints(Skeleton &skeleton)
  {
    for(auto i = joints.begin(); i != joints.end(); i++)
      i->second.clear();
    joints.clear();
    tree<BodyPart>* partTree = skeleton.getPartTreePtr();
    for(tree<BodyPart>::iterator p = partTree->begin(); p!=partTree->end(); p++)
    {
      int partID = p->getPartID();
      std::vector<int> jointsID = { p->getParentJoint(), p->getChildJoint() };
      joints.emplace(std::pair<uint32_t, std::vector<int>>(partID, jointsID));
    }
  }

  std::string IndexedSkeletonModel::getName()
  {
    return m_name;
  }

  uint32_t IndexedSkeletonModel::size()
  {
    return parts.size();
  }

  void IndexedSkeletonModel::clear()
  {
    for(int i = 0;  i < parts.size(); i++)
      parts[i].clear();
    parts.clear();
    for (int i = 0; i < joints.size(); i++)
      joints[i].clear();
    joints.clear();
  }
//=========================================================================
//"frameSolver" class

  frameSolver::frameSolver(IndexedSkeletonModel *pattern)
  {
    m_pattern = pattern;
    neighborFrameSolvers = { 0, 0 };
    for(uint32_t i = 0; i < pattern->size(); i++)
    {
      SkeletonLabelsScores.emplace(std::pair<uint32_t, double>(i, 0));
      ignored.emplace(std::pair<uint32_t, bool>(i, false));
      prevFrameDistances.emplace(std::pair<uint32_t, double>(i, 0));
      SkeletonLabelsIndexes.emplace(std::pair<uint32_t, uint32_t>(i, 0));
    }
    refresh();

    LogStream = &std::cout;
  }

  frameSolver::~frameSolver(void)
  {
    clear();
  }

  void frameSolver::clear()
  {
    for (int i = 0; i < labels.size(); i++)
      labels[i].clear();
    labels.clear();
    SkeletonLabelsIndexes.clear();
    SkeletonLabelsScores.clear();
    prevFrameDistances.clear();

    ignored.clear();
    solved = false;
    iterations = 0;
  }

  void frameSolver::refresh(void) //?
  {
    solved = false;
    iterations = 0;
    idleIterations = 0;
    skeletonScore = 0.0f;
    badPartID = 0;
    badPartScore = 0.0f;
    for (uint32_t i = 0; i < m_pattern->size(); i++)
    {
      SkeletonLabelsIndexes[i] = 0;
      SkeletonLabelsScores[i] = 0;
      prevFrameDistances[i] = 0;
      ignored[i] = false;
      labels[i].clear();
    }
    labels.clear();
  }

  bool frameSolver::isSolved()
  {
    return solved;
  }

  void frameSolver::setSolved(bool isSolved)
  {
    solved = isSolved;
  }

  void frameSolver::setFrameID(int frameID)
  {
    m_frameID = frameID;
  }

  float frameSolver::labelScore(int partID, int labelIndex)
  {
    double score = labels[partID][labelIndex].score;
    double jointsDistancesSum1 = 0.0f;
    double jointsDistancesSum2 = 0.0f;
    int n = m_pattern->parts[partID].ParentJointConnections.size();
    for (int k = 0; k < n; k++)
    {
      int p = m_pattern->parts[partID].ParentJointConnections[k].partID; // adjacents Part ID
      int j = m_pattern->parts[partID].ParentJointConnections[k].num; // adjacents Joint Type
      int t = SkeletonLabelsIndexes[p]; // adjacents Part label index
      cv::Point2f d = labels[p][t].joints[j] - labels[partID][labelIndex].joints[0];
      double distance = /*sqrt*/(d.x*d.x + d.y*d.y);
      jointsDistancesSum1 = jointsDistancesSum1 + distance;
    }
    int m = m_pattern->parts[partID].ChildJointConnections.size();
    for (int k = 0; k < m; k++)
    {
      int p = m_pattern->parts[partID].ChildJointConnections[k].partID; // adjacents Part ID
      int j = m_pattern->parts[partID].ChildJointConnections[k].num; // adjacents Joint Type
      int t = SkeletonLabelsIndexes[p]; // adjacents Part label index
      cv::Point2f d = labels[p][t].joints[j] - labels[partID][labelIndex].joints[1];
      double distance = /*sqrt*/(d.x*d.x + d.y*d.y);
      jointsDistancesSum2 = jointsDistancesSum2 + distance;
    }

    if (n > 0) jointsDistancesSum1 /= static_cast<double>(n);
    else jointsDistancesSum1 = score*score*100.0f;
    if (m > 0) jointsDistancesSum2 /= static_cast<double>(m);
    else jointsDistancesSum2 = score*score*100.0f;

    if (neighborFrameSolvers[0] != 0)
    {
      jointsDistancesSum1 += getInterframeDistance(partID, 0);
      jointsDistancesSum2 += getInterframeDistance(partID, 1);
    }
    if (neighborFrameSolvers[1] != 0)
    {
      jointsDistancesSum1 += neighborFrameSolvers[1]->getInterframeDistance(partID, 0);
      jointsDistancesSum2 += neighborFrameSolvers[1]->getInterframeDistance(partID, 1);
    }

    score = static_cast<float>(score*score*(jointsDistancesSum1 + jointsDistancesSum2));

    return score;
  }

  std::vector<cv::Point2f> frameSolver::getAvgJointsLocation(int partID)
  {
    std::vector<cv::Point2f> avgLocations = {cv::Point2f(0.0f, 0.0f), cv::Point2f(0.0f, 0.0f)};
    float N = 0.0f;
    int n = m_pattern->parts[partID].ParentJointConnections.size();
    for (int k = 0; k < n; k++)
    {
      int p = m_pattern->parts[partID].ParentJointConnections[k].partID; // adjacents Part ID
      int j = m_pattern->parts[partID].ParentJointConnections[k].num; // adjacents Joint Type
      int t = SkeletonLabelsIndexes[p]; // adjacents Part label index
      avgLocations[0] += labels[p][t].joints[j];
      N++;
    }
    if (N > 1) avgLocations[0] /= N;
    N = 0.0f;
    int m = m_pattern->parts[partID].ChildJointConnections.size();
    for (int k = 0; k < m; k++)
    {
      int p = m_pattern->parts[partID].ChildJointConnections[k].partID; // adjacents Part ID
      int j = m_pattern->parts[partID].ChildJointConnections[k].num; // adjacents Joint Type
      int t = SkeletonLabelsIndexes[p]; // adjacents Part label index
      avgLocations[1] += labels[p][t].joints[j];
      N++;
    } 
    if (N > 1) avgLocations[1] /= N;

    return avgLocations;
  }

  void frameSolver::setInterframeDistances(int PartID)
  { 
    std::vector<cv::Point2f> avgLocations = getAvgJointsLocation(PartID);
    if(neighborFrameSolvers[0] != 0)
    {
      std::vector<double> temp(2);
      std::vector<cv::Point2f> D = neighborFrameSolvers[0]->getAvgJointsLocation(PartID);
      for (int joint = 0; joint < D.size(); joint++)
      {
        D[joint] = avgLocations[joint] - D[joint];
        temp[joint] = sqrt(D[joint].x*D[joint].x + D[joint].y*D[joint].y);
        prevFrameDistances[m_pattern->joints[PartID][joint]] = temp[joint];
      }	       
      temp.clear();
    }
    avgLocations.clear();
  }

  double frameSolver::getInterframeDistance(int PartID, int isChildJoint)
  {
    int jointID = m_pattern->joints[PartID][isChildJoint];
    return prevFrameDistances[jointID];
  }

  double frameSolver::getSkeletonScore()
  {
    return skeletonScore;
  }

  float frameSolver::getBadPartScore()
  {
    return badPartScore;
  }

  int  frameSolver::findBadPart()
  {
    skeletonScore = 0.0f;
    badPartID = 0;
    badPartScore = 0.0f;
    for (int i = 0; i < SkeletonLabelsIndexes.size(); i++)       
    {
      if (!ignored[i])
        if (SkeletonLabelsScores[i] > badPartScore)
        {
          badPartScore = SkeletonLabelsScores[i];
          badPartID = i;    
        }		
      skeletonScore = skeletonScore + SkeletonLabelsScores[i];
    }

    return badPartID;
  }

  // Initialization (create first approximation)
  bool frameSolver::initialize(std::map<uint32_t, std::vector<LimbLabel>> limbLabels)
  {
    if (limbLabels.size() != m_pattern->size())
      return false;

    refresh();
    /*labels =*/ prepareLimbLabels(limbLabels);
    for (int i = 0; i < labels.size(); i++)
    {
      int optLabelIndex = 0;
      for(int k = 0; k < labels[i].size(); k++)
        if(labels[i][k].score < labels[i][optLabelIndex].score)
          optLabelIndex = k;
      SkeletonLabelsIndexes[i] = optLabelIndex;
    }
    if (neighborFrameSolvers[0] != 0)
      for (int i = 0; i < labels.size(); i++)
        setInterframeDistances(i);
    for (int i = 0; i < SkeletonLabelsIndexes.size(); i++)  
    {
      int currentLabelIndex = SkeletonLabelsIndexes[i];
      SkeletonLabelsScores[i] = labelScore(i, currentLabelIndex);
      skeletonScore = skeletonScore + SkeletonLabelsScores[i];
    }
    findBadPart();

    return true;
  }

  void frameSolver::singleIteration()
  {
    iterations++;
    //*LogStream << std::endl << "Iteration " << iterations << ":" << std::endl;
    //oldSkeletonScore = skeletonScore;
    //*LogStream << "  bad partID = " << badPartID << std::endl;

    // Searching new label for the bad part
    int newLabelIndex = SkeletonLabelsIndexes[badPartID];
      
    float tempLabelScore = 0.0f;
    int actualLabels;/* = int(d*labels[badPartID].size());
    /if(labels[badPartID].size() < actualLabels)*/
    actualLabels = labels[badPartID].size();
    for (int l = 0; l < actualLabels; l++)
    {
      tempLabelScore = labelScore(badPartID, l);
      if (tempLabelScore < SkeletonLabelsScores[badPartID])
        newLabelIndex = l;	  
    }
    //*LogStream << " newLabelIndex =" << newLabelIndex << std::endl;

    // Modify skeleton
    if (newLabelIndex == SkeletonLabelsIndexes[badPartID])
    {
      ignored[badPartID] = true;
      idleIterations++;
      //*LogStream << "  idle iteration on part " << badPartID << std::endl;
    }
    else
    {
      // Replacing the bad part
      for(int i = 0; i < ignored.size(); i++)
        ignored[i] = false;
      idleIterations = 0;
      SkeletonLabelsIndexes[badPartID] = newLabelIndex;
      //*LogStream << "  replaced part " << badPartID << std::endl;

      setInterframeDistances(badPartID);
      recalculateAdjacentsLabels(badPartID);
      if (neighborFrameSolvers[0] != 0)
      {
        neighborFrameSolvers[0]->recalculateAdjacentsLabels(badPartID);
        if(neighborFrameSolvers[0]->isSolved() != true)
          neighborFrameSolvers[0]->findBadPart();
      }
      if (neighborFrameSolvers[1] != 0)
      {
        neighborFrameSolvers[1]->recalculateAdjacentsLabels(badPartID);
        if (neighborFrameSolvers[1]->isSolved() != true)
          neighborFrameSolvers[1]->findBadPart();
      }
    }   
    findBadPart(); 
  }

  // Recalculation new and adjusted labels scores
  void frameSolver::recalculateAdjacentsLabels(int badPartID)
  {
    int newLabelIndex = SkeletonLabelsIndexes[badPartID];
    SkeletonLabelsScores[badPartID] = labelScore(badPartID, newLabelIndex);
    for (int k = 0; k < m_pattern->parts[badPartID].ParentJointConnections.size(); k++)
    {
      int p = m_pattern->parts[badPartID].ParentJointConnections[k].partID;
      int l = SkeletonLabelsIndexes[p];
      SkeletonLabelsScores[p] = labelScore(p, l);
    }
    for (int i = 0; i < m_pattern->parts[badPartID].ChildJointConnections.size(); i++)
    {
      int p = m_pattern->parts[badPartID].ChildJointConnections[i].partID;
      int l = SkeletonLabelsIndexes[p];
      SkeletonLabelsScores[p] = labelScore(p, l);
    }
  }

  Solvlet frameSolver::solveFrame(std::map<uint32_t, std::vector<LimbLabel>> limbLabels, int frameID)
  {  
    initialize(limbLabels);
    
    // Starting solve
    /*float searchDepth = 0.1f;
    for (float d = 0.0f; d <1.0f; d += searchDepth)
    {*/
    while(idleIterations <= m_pattern->size() && iterations < iterationsLimit)
      singleIteration();
    //}
    //*LogStream << "Iterations count = " << iterations << std::endl;

    // Create solve for current frame
    Solvlet solve;
    std::vector<LimbLabel> temp;
    for (int p = 0; p < labels.size(); p++)
      temp.push_back(limbLabels[p][SkeletonLabelsIndexes[p]]);
    solve.setLabels(temp);
    solve.setFrameID(frameID);

    return solve;
  }

  Skeleton frameSolver::getAverageJointsSkeleton(Skeleton pattern)
  {
    long t0 = clock();
    Skeleton temp = pattern;
    tree<BodyPart> partTree = pattern.getPartTree();
    tree<BodyJoint> jointTree = pattern.getJointTree();
    
    std::map<int, cv::Point2f> joints;
    std::map<int, int> count;
    for (int i = 0; i < jointTree.size(); i++)
    {
      joints.emplace(std::pair<int, cv::Point2f>(i, cv::Point2f(0,0)));
      count.emplace(std::pair<int, int>(i, 0));
    }

    for(tree<BodyPart>::iterator p = partTree.begin(); p !=partTree.end(); p++)
    {
      int joint0ID = p->getParentJoint();
      int joint1ID = p->getChildJoint();
      int partID = p->getPartID();
      
      int labelID = SkeletonLabelsIndexes[partID];
      std::vector<cv::Point2f> labelJoints = labels[partID][labelID].joints;

      joints[joint0ID] += labelJoints[0];
      count[joint0ID]++;
      joints[joint1ID] += labelJoints[1];
      count[joint1ID]++;
    }
    for(int i = 0; i < joints.size(); i++)
      if(count[i] > 0)
        joints[i] = joints[i]/float(count[i]);

    for (tree<BodyJoint>::iterator p = jointTree.begin(); p != jointTree.end(); p++)
    {
      cv::Point2f P = joints[p->getLimbID()];
      p->setImageLocation(P);
      p->setSpaceLocation(cv::Point3f(P.x, P.y, 0.0f));
    }
    joints.clear();
    count.clear();

    temp.setJointTree(jointTree);
    long t1 = clock();

    //*LogStream << "  Skeleton creating time = " << clock_to_ms(t1 - t0) << " ms\n";

    return temp;
  }

  void frameSolver::setParrentFrameSolver(frameSolver * fsolver)
  {
    neighborFrameSolvers[0] = fsolver;
  }

  void frameSolver::setChildFrameSolver(frameSolver * fsolver)
  {
    neighborFrameSolvers[1] = fsolver;
  }

  long int frameSolver::getIterationNumber()
  {
    return iterations;
  }

  Solvlet frameSolver::getSolve(std::map<uint32_t, std::vector<LimbLabel>> &limbLabels)
  {
    Solvlet solve;
    std::vector<LimbLabel> temp;
    for (int p = 0; p < labels.size(); p++)
      temp.push_back(limbLabels[p][SkeletonLabelsIndexes[p]]);
    solve.setLabels(temp);
    solve.setFrameID(m_frameID);
    /*std::vector<LimbLabel> limbLabels;
    for (int p = 0; p < m_pattern->size(); p++)
    {
      std::vector<cv::Point2f>  joints = labels[p][SkeletonLabelsIndexes[p]].joints;
      std::vector<cv::Point2f> polygon = BuildPartPolygon()
    }
    LimbLabel();*/

    return solve;
  }

  Skeleton frameSolver::getShiftedLabelsSkeleton(Skeleton pattern)
  {
    long t0 = clock();
    Skeleton temp = pattern;
    tree<BodyPart> partTree = pattern.getPartTree();
    tree<BodyJoint> jointTree = pattern.getJointTree();
    
    std::map<int, cv::Point2f> joints;
    std::map<int, int> count;
    for (int i = 0; i < jointTree.size(); i++)
    {
      joints.emplace(std::pair<int, cv::Point2f>(i, cv::Point2f(0,0)));
      count.emplace(std::pair<int, int>(i, 0));
    }

    cv::Point2f center;
    for (int k = 0; k < SkeletonLabelsIndexes.size(); k++)
    {
      int labelID = SkeletonLabelsIndexes[k];
      std::vector<cv::Point2f> labelJoints = labels[k][labelID].joints;
      center = center + labelJoints[0];
      center = center + labelJoints[1];
    }
    center = center / static_cast<float>(SkeletonLabelsIndexes.size());

    tree<BodyPart>::iterator p = partTree.begin();
    int joint0ID = p->getParentJoint();
    int joint1ID = p->getChildJoint();
    int partID = p->getPartID();
    int labelID = SkeletonLabelsIndexes[partID];
    std::vector<cv::Point2f> labelJoints = labels[partID][labelID].joints;
    joints[joint0ID] = labelJoints[0];
    joints[joint1ID] = labelJoints[1];

    for(tree<BodyPart>::iterator p = partTree.begin(); p !=partTree.end(); p++)
    {
      int p1 = p->getChildJoint();
      int p0 = p->getParentJoint();
      for (tree<BodyPart>::sibling_iterator c = partTree.begin(p); c != partTree.end(p); c++)
      {
        int j0 = c->getParentJoint();
        int j1 = c->getChildJoint();
        int partID = c->getPartID();
        int labelID = SkeletonLabelsIndexes[partID];
        std::vector<cv::Point2f> labelJoints = labels[partID][labelID].joints;
        int parentJointID = p1;
        if (j0 == p0) parentJointID = p0;
        cv::Point2f shift = labelJoints[0] - joints[parentJointID];
        //joints[joint0ID] = labelJoints[0] - shift;
        joints[j1] = labelJoints[1] - shift;
      }
    }

    cv::Point2f center2(0, 0);
    /*for(int i = 0; i < joints.size(); i++)
      center2 = center2 + joints[i];
    center = center / static_cast<float>(joints.size());
    center = (center2 - center);
    */
    for (tree<BodyJoint>::iterator p = jointTree.begin(); p != jointTree.end(); p++)
    {
      cv::Point2f P = joints[p->getLimbID()];// -center;
      p->setImageLocation(P);
      p->setSpaceLocation(cv::Point3f(P.x, P.y, 0.0f));
    }
    joints.clear();
    count.clear();

    temp.setJointTree(jointTree);
    long t1 = clock();

    //*LogStream << "  Skeleton creating time = " << clock_to_ms(t1 - t0) << " ms\n";

    return temp;
  }

  void frameSolver::setLogStream(std::ostream * logStream)
  {
    LogStream = logStream;
  }

  std::vector<cv::Point2f> frameSolver::getLimbLabelJoints(LimbLabel limbLabel)
  {
    std::vector<cv::Point2f> polygon = limbLabel.getPolygon();
    cv::Point2f p0 = 0.5f*(polygon[0] + polygon[3]);
    cv::Point2f p1 = 0.5f*(polygon[1] + polygon[2]);

    return std::vector<cv::Point2f> {p0, p1};
  }

  /*std::map<int, std::vector<frameSolver::Label>>*/void frameSolver::prepareLimbLabels(std::map<uint32_t, std::vector<LimbLabel>> &limbLabels)
  {
    //int labelsPerPart = 17;
    for (auto l = labels.begin(); l != labels.end(); l++)
      l->second.clear();
    labels.clear();
    for (int i = 0; i < limbLabels.size(); i++)
    {
      std::vector<Label> temp;
      /*int N = 0;
      while(limbLabels[i][N].getAvgScore() < 1.0f)
        N++;
      if (N < 1) N++; 
      // N = labelsPerPart;
      if (limbLabels[i].size() < N) N = limbLabels[i].size();*/	  
      for (int k = 0; k < limbLabels[i].size(); k++) // k < N
      {
        Label label;
        label.score = 0.0f;
        label.joints = getLimbLabelJoints(limbLabels[i][k]);
        std::vector<Score> scores = limbLabels[i][k].getScores();
        float n = 0.0f;
        //label.score = limbLabels[i][k].getAvgScore(); // score variant A
        // score variant B:
        for (int t = 0; t < scores.size(); t++)
        {
          float s = scores[t].getScore();
          if (s > 0.0f && s <= 1.0f)
          {
            label.score += s;
            n++;
          }
        }
        if (n > 0)
          label.score = label.score / n;
        else
          label.score = 1.0f;
        // end of score variant B
        temp.push_back(label);
      }
      labels.emplace(std::pair<int, std::vector<Label>>(i, temp));
      temp.clear();
    }
    //return labels;
  }
//==================================================================
//"_Solver" class

  _Solver::_Solver(void)
  {
    m_id = 2;
    m_name = "_";
    LogStream = &std::cout;
  }

  _Solver::~_Solver(void)
  {
    for(int i = 0; i < detectors.size(); i++)
      delete detectors[i];
    detectors.clear();
    detectorsNames.clear();
  } 

  std::vector<Solvlet> _Solver::solve(Sequence& seq)
  {
    std::vector<Solvlet> temp;
    return temp;
  }

  std::vector<Solvlet> _Solver::solve(Sequence& seq, std::map<std::string,float> params)
  {
    std::vector<Solvlet> solves;
    std::vector<Frame*> frames = seq.getFrames();
    std::vector<std::vector<Frame*>> slices = createSlices(frames);
    /*ImagePixelSimilarityMatrix* ISM = new ImagePixelSimilarityMatrix();
    if (!M->read("ISM.txt") || M->size() != frames.size())
    {*/
      //DebugMessage("ISM not found ", 2);
      /*DebugMessage("Building Image similarity matrix ", 2);
      ISM->buildImageSimilarityMatrix(frames, 0, 0, false, false);
      DebugMessage("ISM created ", 2);*/
    /*}*/

    emplaceDefaultParameters(params);

    // Create interpolation
    /*clearSkeletons(frames);
    interpolate3(frames, ISM);*/

    //Create detectors
    for(int i = 0; i < detectors.size(); i++)
      delete detectors[i];
    detectors.clear();
    bool useHist = params.at("useCSdet") > 0.01f;
    bool useHog = params.at("useHoGdet") > 0.01f;
    bool useSurf2 = params.at("useSURFdet") > 0.01f;
    if (useHog)
    {
      detectors.push_back(new HogDetector());
      detectorsNames.push_back("HOGDetector");
    }
    if (useHist) 
    {
      detectors.push_back(new ColorHistDetector());
      detectorsNames.push_back("ColorHistDetector");
    }
    if (useSurf2)
    {
      detectors.push_back(new SURFDetector2());
      detectorsNames.push_back("SURFDetector2");
    }

    // Search a keyframe
    int i = 0;
    while (i < frames.size() && frames[i]->getFrametype() != KEYFRAME)
      i++;

    // Create skeleton pattern
    Skeleton pattern;
    if (frames[i]->getFrametype() == KEYFRAME)
      pattern = frames[i]->getSkeleton();
    IndexedSkeletonModel *indexedSkeleton = new IndexedSkeletonModel(pattern);

    // Solving
    trainTime = 0;
    detectTime = 0;
    frameSolver fsolver(indexedSkeleton);
    fsolver.setLogStream(LogStream);
    if (indexedSkeleton->size() > 0)
    for (int q = 0; q < slices.size(); q++)
    { 
      *LogStream << "Solving of the Slices[" << q << "] started\n";
      long int t0 = 0, t1 = 0, T0 = 0, T1 = 0;

      DebugMessage("Training on slice " + std::to_string(q), 1);
      train(slices[q], params);
      *LogStream << "\n";

      bool b = false;
      int i = 1, n = 1, m = slices[q].size();
      Frame* prevFrame = slices[q][0];
      if (slices[q][0]->getFrametype() != KEYFRAME)
      {
        i = slices[q].size() - 1;
        n = -1;
        m = -1;
        prevFrame = slices[q][i];
      }
      else
      {
        b = (slices[q][m - 1]->getFrametype() == KEYFRAME);
        m = m/2;
      }

      DebugMessage("Solving the slice " + std::to_string(q), 1);
      /*if (slices[q].size() < 13)  //?   
        interpolate2(slices[q]); //?
      *LogStream << "interpolate2 - Ok" << std::endl;*/
      for (i; i != m; i = i + n)
      if(i < slices[q].size())
        if (slices[q][i]->getFrametype() != KEYFRAME)
        {        
          *LogStream << "  frame[" << slices[q][i]->getID() << "].skeleton = '" << slices[q][i]->getSkeletonPtr()->getName() << "'"
            //<< " size = " << slices[q][i]->getSkeletonPtr()->getPartTreeCount()
            << std::endl;
          Solvlet solve = solveFrame(params, fsolver, slices[q][i], prevFrame);
          solves.push_back(solve);
          prevFrame = slices[q][i];      
        }
      if (b)
      {
        n = slices[q].size()-1;
        prevFrame = slices[q][n];
        for (int k = n; k >= i; k--)
        if(k < slices[q].size())
          if (slices[q][k]->getFrametype() != KEYFRAME)
          {       
            *LogStream << "  frame[" << slices[q][k]->getID() << "].skeleton = '" << slices[q][k]->getSkeletonPtr()->getName() << "'"
              //<< " size = "  << slices[q][i]->getSkeletonPtr()->getPartTreeCount() 
              << std::endl;
            Solvlet solve = solveFrame(params, fsolver, slices[q][k], prevFrame);
            solves.push_back(solve);
            prevFrame = slices[q][k];      
          }
      }
      seq.setFrames(frames);

      *LogStream << "Slices[" << q << "] solved\n";
    }
    delete indexedSkeleton;

    return solves;
  }

  // Solve all frames from current slice as a single tree
  std::vector<Solvlet> _Solver::solveGlobal(Sequence& seq, std::map<std::string, float> params)
  {
    //Prepare slices
    seq.estimateUniformScale(params);
    seq.computeInterpolation(params);
    std::vector<Frame*> frames = seq.getFrames();
    std::vector<std::vector<Frame*>> temp = createSlices(frames);
    std::vector<std::vector<Frame*>> slices;
    for(int i = 0; i < temp.size(); i++)
      if (temp[i][0]->getFrametype() == KEYFRAME && temp[i][temp[i].size() - 1]->getFrametype() == KEYFRAME)
        slices.push_back(temp[i]);
    temp.clear();

    /*// Search a keyframe
    int i = 0;
    while (i < frames.size() && frames[i]->getFrametype() != KEYFRAME)
      i++;*/

    Skeleton skeleton = slices[0][0]->getSkeleton();
    IndexedSkeletonModel *pattern = new IndexedSkeletonModel(skeleton);

    //Create detectors
    for(int i = 0; i < detectors.size(); i++)
      delete detectors[i];
    detectors.clear();
    bool useHist = params.at("useCSdet") > 0.01f;
    bool useHog = params.at("useHoGdet") > 0.01f;
    bool useSurf2 = params.at("useSURFdet") > 0.01f;
    if (useHog)
    {
      detectors.push_back(new HogDetector());
      detectorsNames.push_back("HOGDetector");
    }
    if (useHist) 
    {
      detectors.push_back(new ColorHistDetector());
      detectorsNames.push_back("ColorHistDetector");
    }
    if (useSurf2)
    {
      detectors.push_back(new SURFDetector2());
      detectorsNames.push_back("SURFDetector2");
    }

    //Solving
    trainTime = 0;
    detectTime = 0;
    std::vector<Solvlet> Solves;
    DebugMessage("Slices count: " + std::to_string(slices.size()),1);
    for (int q = 0; q < slices.size(); q++)
    {
      DebugMessage("Solving slice " + std::to_string(q), 1);
      *LogStream << "\nSolving slice " << q << std::endl;
      int m = slices[q].size();
      std::vector<std::map<uint32_t, std::vector<LimbLabel>>> sliceLabels(m);

      //Create "frameSolvers"
      std::vector<frameSolver*> fsolvers;
      for (int f = 0; f < m; f++)
        fsolvers.push_back(new frameSolver(pattern));
      for (int i = 1; i < m; i++)
        fsolvers[i]->setParrentFrameSolver(fsolvers[i-1]);
      for (int i = 0; i < m-1; i++)
        fsolvers[i]->setChildFrameSolver(fsolvers[i+1]);

      train(slices[q], params);
      
      //Init frameSolvers  
      for (int f = 0; f < m; f++)
      {
        fsolvers[f]->setFrameID(slices[q][f]->getID());
        if (slices[q][f]->getFrametype() != KEYFRAME)
          sliceLabels[f] = detect(params, slices[q][f]);
        else
          sliceLabels[f] = createLabels(slices[q][f]->getSkeleton());// load from fromKeyframe		
        fsolvers[f]->initialize(sliceLabels[f]);
      }

      for (int i = 0; i < m - 1; i++)
        fsolvers[i]->singleIteration();

      // Iterations
      bool sliceSolved = false;
      int badSkeleton = 1;
      long int iterations = 0;
      long int IdleIterations = 0;
      std::vector<bool> ignored(m, false);
      for (int i = 0; i < m; i++)
        if (slices[q][i]->getFrametype() == KEYFRAME)
        {
          ignored[i] = true;
          fsolvers[i]->setSolved(true);
        }
      while (!sliceSolved && iterations < 30000)//IdleIterations < (2*m)
      {
        iterations++;
        //Search a bad frame
        double maxScore = 0.0;
        int temp = 0;
        for(int i = 1; i < m - 1; i++)
          if(!ignored[i] && fsolvers[i]->getBadPartScore() > maxScore)//getSkeletonScore()
          {
            temp = i;
            maxScore = fsolvers[i]->getBadPartScore();//getSkeletonScore() 	
          }
        if (temp != 0)
        {
          if (temp == badSkeleton || fsolvers[temp]->isSolved())
          {
            ignored[temp] = true;
            DebugMessage("Idle iteration on frame " + std::to_string(slices[q][temp]->getID()), 1);
            *LogStream << iterations << ". Idle iteration on frame " << slices[q][temp]->getID() << " BadPartScore = "
              << fsolvers[temp]->getBadPartScore() << " SkeletonSkore = " << fsolvers[temp]->getSkeletonScore() << std::endl;
          }
          else
          {
            badSkeleton = temp;
            DebugMessage("Iteration on frame " + std::to_string(slices[q][badSkeleton]->getID()), 1);
            fsolvers[badSkeleton]->singleIteration();
            *LogStream << iterations << ". Iteration on frame " << slices[q][badSkeleton]->getID() << " BadPartScore = "
              << fsolvers[badSkeleton]->getBadPartScore() << " SkeletonScore = " << fsolvers[badSkeleton]->getSkeletonScore() << std::endl;
          }      
        }
        if(temp == 0)
        {
          IdleIterations++;
          if(IdleIterations >= (m-2) )
          {
            sliceSolved = true;
            *LogStream << "Slice " << q << " Solved, maxScore = " << maxScore << std::endl;
          }          
          for (int i = 0; i < m; i++)
            if (slices[q][i]->getFrametype() != KEYFRAME)
            {
              ignored[i] = false;
              fsolvers[i]->setSolved(false);
            }       
        }
      }
      DebugMessage("Iterations count: " + std::to_string(iterations), 1);    

      for (int i = 0; i < m; i++)
      {
        if (slices[q][i]->getFrametype() != KEYFRAME)
        {
          Skeleton temp = fsolvers[i]->getShiftedLabelsSkeleton(skeleton);
          temp.setName("Solve.toSkeleton");
          slices[q][i]->setSkeleton(temp);
          *LogStream << "Solve" << slices[q][i]->getID() << ".toSkeleton" << std::endl;
          Solves.push_back(fsolvers[i]->getSolve(sliceLabels[i]));        
        }
        delete fsolvers[i];
      }
      fsolvers.clear();         
    }
    seq.setFrames(frames);
    delete pattern;

    return Solves;
  }

  void _Solver::train(std::vector<Frame*> &slice, std::map<std::string, float> &params)
  {
    long int t0 = 0, t1 = 0;

    for (int i = 0; i < detectors.size(); i++)
      if (detectors[i] != 0)
      {
        *LogStream << detectorsNames[i] + " train: ";
        t0 = clock();
        detectors[i]->train(slice, params);
        t1 = clock();
        trainTime += t1 - t0;
        *LogStream << " time = " << spelHelper::clock_to_ms(t1 - t0) << "ms - Ok\n";
      }
  }

  std::map<uint32_t, std::vector<LimbLabel>> _Solver::detect(std::map<std::string, float> &params, Frame* &frame, Frame* previousFrame)
  {
    long int T0 = clock();

    bool haveSkeleton = (frame->getSkeletonPtr()->getPartTreeCount() > 0);
    //bool isInterpolation = (frame->getSkeletonPtr()->getName() == "interpolate2");
    if(!haveSkeleton && previousFrame != 0 )
      setSkeleton(frame, previousFrame);

    std::map<uint32_t, std::vector<LimbLabel>> LimbLabels;
    long int t0, t1;
    for (int d = 0; d < detectors.size(); d++)
    {
      *LogStream << " " + detectorsNames[d] + ": ";
      t0 = clock();
      LimbLabels = detectors[d]->detect(frame, params, LimbLabels);
      t1 = clock();
      detectTime += t1 - t0;
      int labelsCount = 0;
      for (int l = 0; l < LimbLabels.size(); l++)
        labelsCount += LimbLabels[l].size();
      *LogStream << "Limb labels count = " << labelsCount << ", time = " << spelHelper::clock_to_ms(t1 - t0) << "ms - Ok\n";
    }

    return LimbLabels;
  }

  Solvlet _Solver::solveFrame(std::map<std::string, float> &params, frameSolver &fSolver, Frame* frame, Frame* prevFrame)
  {
    long int T0 = clock();
    *LogStream << " Detect on frame [" << frame->getID() << "] started " << std::endl;
    std::map<uint32_t, std::vector<LimbLabel>> LimbLabels = detect(params, frame, prevFrame);

    *LogStream << " Solving of the frame [" << frame->getID() << "]: ";
    long int t0 = 0, t1 = 0;
    t0 = clock();
    Solvlet solve = fSolver.solveFrame(LimbLabels, frame->getID());
    t1 = clock();
    *LogStream << "iterations count = " << fSolver.getIterationNumber() << ", time = " << spelHelper::clock_to_ms(t1 - t0) << "ms - Ok\n";
    *LogStream << std::endl;	

    Skeleton pattern = frame->getSkeleton();
    Skeleton temp = (0.5f*fSolver.getAverageJointsSkeleton(pattern) + 0.5f*fSolver.getShiftedLabelsSkeleton(pattern));
    Skeleton skeleton = temp;// for supporting tree.hh 3.1
    skeleton.setName("Solve.toSkeleton");
    frame->setSkeleton(skeleton);
    long int T1 = clock();
    DebugMessage("Frame " + std::to_string(frame->getID()) + " solved - " + std::to_string(spelHelper::clock_to_ms(T1 - T0)) + " ms", 1);

    return solve;
  }
  
  void _Solver::setLogStream(std::ostream * logStream)
  {
    LogStream = logStream;
  }

  long int _Solver::getTrainTime()
  {
    return trainTime;
  }

  long int _Solver::getDetectTime()
  {
    return detectTime;
  }

  void _Solver::emplaceDefaultParameters(std::map<std::string, float> &params) const
  {
    params.emplace("useCSdet", 0.0f);
    params.emplace("useHoGdet", 1.0f); 
    params.emplace("useSURFdet", 0.0f);
  }
  //==================================================================
}
