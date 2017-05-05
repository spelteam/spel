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
  std::vector<std::vector<Frame*>> _Solver::createSlices(std::vector<Frame*> frames)
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
    m_name = name;
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
  }
//=========================================================================
//"frameSolver" class

  frameSolver::frameSolver(IndexedSkeletonModel *pattern)
  {
    m_pattern = pattern;
    iterations = 0;
    bool solved = false;
    for (int i = 0; i < pattern->size(); i++)
    {
      SkeletonLabelsScores.emplace(std::pair<uint32_t, float>(i, 0));
      ignored.emplace(std::pair<uint32_t, bool>(i, false));
    }
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
    ignored.clear();
    solved = false;
    iterations = 0;
  }

  void frameSolver::refresh(void) //?
  {
    for (int i = 0; i < ignored.size(); i++)
      ignored[i] = false;
    for (int i = 0; i < labels.size(); i++)
      labels[i].clear();
    labels.clear();

    solved = false;
    iterations = 0;
  }

  bool frameSolver::IsSolved()
  {
    return solved;
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
    score = score*score*(jointsDistancesSum1 + jointsDistancesSum2);

    return score;
  }

  Solvlet frameSolver::solveFrame(std::map<uint32_t, std::vector<LimbLabel>> limbLabels, int frameID)
  {
    iterations = 0;
    int iterationsLimit = 30000;

    // Prepare LimbLabels
    labels.clear();	
    labels = prepareLimbLabels(limbLabels);

    // Initialization (create first approximation)
    for (int i = 0; i < ignored.size(); i++)
      ignored[i] = false;
    SkeletonLabelsIndexes.clear();
    for (int i = 0; i < labels.size(); i++)
    {
      int optLabelIndex = 0;
      for(int k = 0; k < labels[i].size(); k++)
        if(labels[i][k].score < labels[i][optLabelIndex].score)
          optLabelIndex = k;
      SkeletonLabelsIndexes.emplace(std::pair<uint32_t, uint32_t>(i, optLabelIndex));
    }
    float skeletonScore = 0.0f;
    for (int i = 0; i < SkeletonLabelsIndexes.size(); i++)  
    {
      int currentLabelIndex = SkeletonLabelsIndexes[i];
      SkeletonLabelsScores[i] = labelScore(i, currentLabelIndex);
      skeletonScore = skeletonScore + SkeletonLabelsScores[i];
    }
    

    // Starting solve
    /*float searchDepth = 0.1f;
    for (float d = 0.0f; d <1.0f; d += searchDepth)
    {*/
    int idleIterations = 0;
    int badPartID = 0;
    float badPartScore = 0.0f;
    float oldSkeletonScore = 0.0f;

    while(idleIterations <= m_pattern->size() && iterations < iterationsLimit)
    {
      iterations++;
      //std::cout << std::endl << "Iteration " << iterations << ":" << std::endl;

      oldSkeletonScore = skeletonScore;
      skeletonScore = 0.0f;
      badPartID = 0;
      badPartScore = 0.0f;

      // Searth a bad label
      for (int i = 0; i < SkeletonLabelsIndexes.size(); i++)
        if(!ignored[i])    
        {
          int currentLabelIndex = SkeletonLabelsIndexes[i];
          if (SkeletonLabelsScores[i] > badPartScore)
          {
            badPartScore = SkeletonLabelsScores[i];
            badPartID = i;    
          }		
          skeletonScore = skeletonScore + SkeletonLabelsScores[i];
        }
      //std::cout << "  bad partID = " << badPartID << std::endl;

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
      //std::cout << " newLabelIndex =" << newLabelIndex << std::endl;

      // Modify skeleton
      if (newLabelIndex == SkeletonLabelsIndexes[badPartID])
      {
        ignored[badPartID] = true;
        idleIterations++;
        //std::cout << "  idle iteration on part " << badPartID << std::endl;
      }
      else
      {
        // Replacing the bad part
        for(int i = 0; i < ignored.size(); i++)
          ignored[i] = false;
        idleIterations = 0;
        SkeletonLabelsIndexes[badPartID] = newLabelIndex;
        //std::cout << "  replaced part " << badPartID << std::endl;

        // Recalculation new and adjusted labels scores
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
    }
    //}
    //std::cout << "Iteration count = " << iterations << std::endl;

    // Create solvet skeleton for current frame
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

    //std::cout << "  Skeleton creating time = " << clock_to_ms(t1 - t0) << " ms\n";

    return temp;
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

    //std::cout << "  Skeleton creating time = " << clock_to_ms(t1 - t0) << " ms\n";

    return temp;
  }

  

  std::vector<cv::Point2f> frameSolver::getLimbLabelJoints(LimbLabel limbLabel)
  {
    std::vector<cv::Point2f> polygon = limbLabel.getPolygon();
    cv::Point2f p0 = 0.5f*(polygon[0] + polygon[3]);
    cv::Point2f p1 = 0.5f*(polygon[1] + polygon[2]);

    return std::vector<cv::Point2f> {p0, p1};
  }

  std::map<int, std::vector<frameSolver::Label>> frameSolver::prepareLimbLabels(std::map<uint32_t, std::vector<LimbLabel>> limbLabels)
  {
    //int labelsPerPart = 17;
    std::map<int, std::vector<Label>> labels;
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
      labels.emplace(std::pair<uint32_t, std::vector<Label>>(i, temp));
    }

    return labels;
  }
//==================================================================
//"_Solver" class

  _Solver::_Solver(void)
  {
    m_id = 2;
    m_name = "_";
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
    ImagePixelSimilarityMatrix* ISM = new ImagePixelSimilarityMatrix();
    /*if (!M->read("ISM.txt") || M->size() != frames.size())
    {*/
      //DebugMessage("ISM not found ", 2);
      /*DebugMessage("Building Image similarity matrix ", 2);
      ISM->buildImageSimilarityMatrix(frames, 0, 0, false, false);
      DebugMessage("ISM created ", 2);*/
    /*}*/

    emplaceDefaultParameters(params);

    // Create interpolation
    clearSkeletons(frames);
    interpolate3(frames, ISM);

    //Create detectors
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
    frameSolver fsolver(indexedSkeleton);
    if (indexedSkeleton->size() > 0)
    for (int q = 0; q < slices.size(); q++)
    { 
      std::cout << "Solving of the Slices[" << q << "] started\n";
      long int t0 = 0, t1 = 0, T0 = 0, T1 = 0;

      DebugMessage("Traning on slice " + std::to_string(q), 2);
      train(slices[q], params);
      std::cout << "\n";

      int i = 1, n = 1, m = slices[q].size();
      Skeleton skeleton = slices[q][0]->getSkeleton();
      if(slices[q][0]->getFrametype() != KEYFRAME)
      {
        i = slices[q].size() - 1;
        skeleton = slices[q][i]->getSkeleton();
        n = -1;
        m = -1;
      }

      DebugMessage("Solving the slice " + std::to_string(q), 2);

      if (slices[q].size() < 11)  //?   
        interpolate2(slices[i]); //?
      for (i; i != m; i = i + n)
      if(i < slices[q].size())
        if (slices[q][i]->getFrametype() != KEYFRAME)
        {
          std::cout << " Detect on frame [" << slices[q][i]->getID() << "] started " << std::endl;
          T0 = clock();
          std::map<uint32_t, std::vector<LimbLabel>> LimbLabels = detect(slices[q][i], params, skeleton);

          std::cout << " Solving of the frame [" << slices[q][i]->getID() << "]: ";
          //fsolver.refresh();         
          t0 = clock();
          Solvlet solve = fsolver.solveFrame(LimbLabels, slices[q][i]->getID());
          solves.push_back(solve);
          t1 = clock();
          std::cout << "iterations count = " << fsolver.iterations << ", time = " << spelHelper::clock_to_ms(t1 - t0) << "ms - Ok\n";
          Skeleton temp = (0.5f*fsolver.getAverageJointsSkeleton(pattern) + 0.5f*fsolver.getShiftedLabelsSkeleton(pattern));
          skeleton = temp;// for supporting tree.hh 3.1
          skeleton.setName("solved");
          slices[q][i]->setSkeleton(skeleton);
          T1 = clock();
          DebugMessage("Frame " + std::to_string(slices[q][i]->getID()) + " solved - " + std::to_string(spelHelper::clock_to_ms(T1 - T0)) + " ms" , 2);
          std::cout << std::endl;
        }

      seq.setFrames(frames);

      std::cout << "Slices[" << q << "] solved\n";
    }

    delete indexedSkeleton;

    return solves;
  }

  void _Solver::train(std::vector<Frame*> &slice, std::map<std::string, float> &params)
  {
    long int t0 = 0, t1 = 0;

    for (int i = 0; i < detectors.size(); i++)
      if (detectors[i] != 0)
      {
        std::cout << detectorsNames[i] + " train: ";
        t0 = clock();
        detectors[i]->train(slice, params);
        t1 = clock();
        std::cout << " time = " << spelHelper::clock_to_ms(t1 - t0) << "ms - Ok\n";
      }
  }

  std::map<uint32_t, std::vector<LimbLabel>> _Solver::detect(Frame* &frame, std::map<std::string, float> &params, Skeleton prevSkeleton)
  {
    long int T0 = clock();

    bool haveSkeleton = (frame->getSkeleton().getPartTreePtr()->size() > 0);
    if (!haveSkeleton)
    {
      cv::Point2f shift(0, 0);

      // Copiyng skeleton from neighbor frame, variant A
      /*
      int p = i - n;
      if(p != m) shift = ISM->getShift(i, p);
      slices[q][i]->setSkeleton((skeleton + shift));// +slices[q][i]->getSkeleton())*0.5f);

      // Debug
      //cv::Mat tempMask = slices[q][i]->getMask().clone();
      //putSkeletonMask(tempMask, skeleton + shift, cv::Size(0, 0), 128);
      //imwrite(std::to_string(slices[q][i]->getID()) + ".png", tempMask);
      //tempMask.release();*/

      // Copiyng skeleton from neighbor frame, variant B
      /**/
      // Create mask for the previous skeleton
      cv::Size size = frame->getMask().size();

      // Select mask ROI
      std::vector<cv::Point2i> endpoints1 = SearchROI(frame->getMask());
      cv::Rect ROI1 = toROIRect(endpoints1);
      correctROI(ROI1, size);

      // Selecting skeleton ROI
      std::vector<cv::Point2f> endpoints2 = getEndpoints(prevSkeleton);
      cv::Rect ROI2 = toROIRect(endpoints2);
      correctROI(ROI2, size);

      // Calculate distance
      cv::Point2i c1 = MaskCenter(frame->getMask(), ROI1);
      cv::Point2f c2 = SkeletonCenter(prevSkeleton);
      shift = cv::Point2i(static_cast<int>(c2.x), static_cast<int>(c2.y)) - c1;

      // Copy skeleton
      frame->setSkeleton(prevSkeleton - shift);

      // Debug
      //cv::Mat tempMask;// = cv::Mat::zeros(size, CV_8UC1);;
      //putSkeletonMask(tempMask, skeleton, size, 255);
      //cv::Point2i c = MaskCenter(tempMask, ROI2);
      //tempMask.release();

      // Debug messages
      //std::cout << " Frame " << std::to_string(slices[q][i]->getID()) << " endpoints: " << endpoints1 << std::endl;
      //std::cout << " Skeleton endpoints: " << endpoints2 << std::endl;
      //std::cout << " Mask ROI: " << ROI1 << std::endl;
      //std::cout << " Skeleton ROI: " << ROI2 << std::endl;
      //std::cout << " Mask center: " << c1 << std::endl;
      //std::cout << " Skeleton mask center: " << c << std::endl;
      //std::cout << " Skeleton center: " << c2 << std::endl;
      //tempMask = slices[q][i]->getMask().clone();  
      //putSkeletonMask(tempMask, skeleton - shift, cv::Size(0,0), 128);
      //imwrite(to_string(slices[q][i]->getID(), 3) + ".png", tempMask);
      //tempMask.release();
    }

    std::map<uint32_t, std::vector<LimbLabel>> LimbLabels;

    long int t0, t1;
    for (int d = 0; d < detectors.size(); d++)
    {
      std::cout << " " + detectorsNames[d] + ": ";
      t0 = clock();
      LimbLabels = detectors[d]->detect(frame, params, LimbLabels);
      t1 = clock();
      int labelsCount = 0;
      for (int l = 0; l < LimbLabels.size(); l++)
        labelsCount += LimbLabels[l].size();
      std::cout << "Limb labels count = " << labelsCount << ", time = " << spelHelper::clock_to_ms(t1 - t0) << "ms - Ok\n";
    }

    return LimbLabels;
  }


  void _Solver::emplaceDefaultParameters(std::map<std::string, float> &params) const
  {
    params.emplace("useCSdet", 0.0f);
    params.emplace("useHoGdet", 1.0f); 
    params.emplace("useSURFdet", 0.0f);
  }
  //==================================================================
}
