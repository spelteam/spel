#include <QTime>
#include <QDir>
#include <nskpsolver.hpp>
#include <tlpssolver.hpp>
#include <keyframe.hpp>
#include <interpolation.hpp>
#include <lockframe.hpp>
#include <sequence.hpp>

#include "model_handlers/projecthandler.h"
#include "model_handlers/bodyjointsheaderhandler.h"
#include "model_handlers/bodypartsheaderhandler.h"
#include "model_handlers/frameshandler.h"

#include "projectattr.h"
#include "projectdocument.h"
#include "utility.h"
#include "exceptions.h"
#include "project.h"

#include <QDebug>


namespace posegui {

  //PUBLIC

  Project& Project::getInstance(){
    static Project instance;
    return instance;
  }

  void Project::open(const QString &filename, QFutureWatcher<void> *watcher){
    if (currState == ProjectState::LOADED){
      throw InvalidProjectState("Project is openned!");
    }
    setProjectFilename(filename);
    setProjectFolder(filename);
    //read document
    ProjectDocument document;
    document.read(filename);
    const QDomDocument& doc =
      document.getDocument();
    const QDomElement& projectElem =
      doc.documentElement();
    const QDomElement& jointHeaderElem =
      projectElem.elementsByTagName(BodyJointHeaderAttrs::JOINTS_TAG)
      .at(0).firstChildElement();
    const QDomElement& partHeaderElem =
      projectElem.elementsByTagName(BodyPartHeaderAttrs::PARTS_TAG)
      .at(0).firstChildElement();
    const QDomElement& frameHeaderElem =
      projectElem.elementsByTagName(FrameAttrs::FRAMES_TAG)
      .at(0).firstChildElement();
    //read project settings
    ProjectHandler projectHandler;
    settings = projectHandler.read(projectElem);
    projectPaths.imgFolderPath = settings.imgFolderPath;
    projectPaths.maskFolderPath = settings.maskFolderPath;
    if (watcher){
      watcher->progressTextChanged("Project settings was loaded");
      watcher->progressValueChanged(10);
    }
    //read body joints
    BodyJointsHeaderHandler jointsHeaderHandler;
    tree<BodyJoint> joints = jointsHeaderHandler
      .read(jointHeaderElem);
    //read body parts
    BodyPartsHeaderHandler partsHeaderHandler;
    std::vector<BodyPart> partsList = partsHeaderHandler
      .read(partHeaderElem, joints);
    tree<BodyPart> parts;
    Utility::buildBodyPartTree(partsList, parts);
    //create skeleton
    skeleton->setJointTree(joints);
    skeleton->setPartTree(parts);
    if (watcher){
      watcher->progressTextChanged("Skeleton was loaded");
      watcher->progressValueChanged(30);
    }
    //read frames
    FramesHandler framesHandler;
    frames = framesHandler.read(frameHeaderElem, projectPaths, skeleton);
    if (watcher){
      watcher->progressTextChanged("Frames was loaded");
      watcher->progressValueChanged(100);
    }
    //mark as opened
    currState = ProjectState::OPENED;
  }

  void Project::save(const QString &filename, QFutureWatcher<void> *watcher){
    if (currState != ProjectState::LOADED){
      throw InvalidProjectState("Project is not openned!");
    }
    QDomDocument doc;
    //read project settings
    ProjectHandler projectHandler;
    QDomElement projectElem =
      projectHandler.write(settings, doc);
    //read skeleton
    BodyJointsHeaderHandler jointsHeaderHandler;
    QDomElement jointsHeaderElem =
      jointsHeaderHandler.write(*skeleton->getJointTreePtr(), doc);
    BodyPartsHeaderHandler partsHeaderHandler;
    QDomElement partsHeaderElem =
      partsHeaderHandler.write(*skeleton->getPartTreePtr(), doc);
    //read frames
    FramesHandler framesHandler;
    QDomElement framesHeaderElem =
      framesHandler.write(frames, projectPaths, doc);
    //set document structure
    doc.appendChild(projectElem);
    projectElem.appendChild(jointsHeaderElem);
    projectElem.appendChild(partsHeaderElem);
    projectElem.appendChild(framesHeaderElem);
    //write to file
    ProjectDocument document;
    document.setDocument(doc);
    document.write(projectPaths.projectFolder + filename + ".xml");
  }

  void Project::save(QFutureWatcher<void> *watcher){
    save(projectPaths.projectFilename, watcher);
  }

  const QString& Project::getProjectFolder() const{
    return projectPaths.projectFolder;
  }

  std::vector<Frame*> Project::getFrames() const{
    std::vector<Frame*> framesPtr;

    for (const FramePtr& currFrame : frames){
      framesPtr.push_back(currFrame.get());
    }

    return framesPtr;
  }

  Frame* Project::getFrame(int num) const{
    return frames.at(num).get();
  }

  const std::vector<LimbLabel> *Project::getLabels(int num) const{
    const std::vector<LimbLabel>* result = nullptr;
    int frameId = getFrame(num)->getID();
    qDebug() << "Solvlets was generating for next frames:" << endl;
    for (const Solvlet& solvlet : solve){
      if (solvlet.getFrameID() == frameId){
        result = solvlet.getLabelsPtr();
        qDebug() << "frameId: " << frameId << endl;
        break;
      }
    }
    return result;
  }

  const FilePathStorage &Project::getPaths() const{
    return projectPaths;
  }

  Project::ProjectState Project::getState() const{
    return currState;
  }

  void Project::exchangeAtKeyframe(int num){
    FramePtr keyFrame = FramePtr(new Keyframe());
    //TODO: [!] get nearest keyframe
    //get first keyframe
    for (unsigned int i = 0; i < frames.size(); ++i){
      if (frames.at(i)->getFrametype() == KEYFRAME){
        keyFrame->setSkeleton(frames.at(i)->getSkeleton());
        break;
      }
    }
    frames.at(num) = std::move(keyFrame);
    //update state of solve box
    keyframeUpdated();
  }

  void Project::exchangeAtInterpolation(int num){
    FramePtr interFrame = FramePtr(new Interpolation());
    interFrame->setSkeleton(*skeleton.get());
    frames.at(num) = std::move(interFrame);
    //update state of solve box
    keyframeUpdated();
  }

  void Project::interpolateFrames(){
    Sequence seq(0, "test", Project::getInstance().getFrames());
    map<string, float> params;
    seq.estimateUniformScale(params);
    seq.computeInterpolation(params);
  }

  void Project::solveFrames(){
    //test interpolation
    interpolateFrames();
    QTime timer;
    timer.start();
    //run solver
    std::vector<Solvlet> solve;
    NSKPSolver solver;
    map <string, float> params; //use the default params

    params.emplace("debugLevel", 3); //set the debug setting to highest (0,1,2,3)
    Sequence seq(0, "test", getFrames());
    seq.estimateUniformScale(params);
    seq.computeInterpolation(params);
    //global settings
    params.emplace("imageCoeff", 1.0); //set solver detector infromation sensitivity
    params.emplace("jointCoeff", 1.0); //set solver body part connectivity sensitivity
    params.emplace("priorCoeff", 0.0); //set solver distance to prior sensitivity

    //detector settings
    params.emplace("useCSdet", 0.0); //determine if ColHist detector is used and with what coefficient
    params.emplace("useHoGdet", 1.0); //determine if HoG descriptor is used and with what coefficient
    params.emplace("useSURFdet", 0.0); //determine whether SURF detector is used and with what coefficient

    //solver settings
    params.emplace("nskpIters", 0); //do as many NSKP iterations as is useful at each run
    params.emplace("acceptLockframeThreshold", 0.52); //set the threshold for NSKP and TLPSSolvers, forcing TLPS to reject some solutions
    params.emplace("badLabelThresh", 0.45); //set bad label threshold, which will force solution discard at 0.45
    params.emplace("partDepthRotationCoeff", 1.25); //search radius increase for each depth level in the part tree
    //nskp solver solve
    ImageSimilarityMatrix ism;
    if (!ism.read("testISM.ism"))
    {
      ism.buildImageSimilarityMatrix(getFrames());
      ism.write(("testISM.ism"));
    }
    solve = solver.solve(seq, params, ism);
    //draw solution
    QFile file("solve.txt");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)){
      qDebug() << "Cannot open file" << endl;
      return;
    }
    QTextStream out(&file);
    for (const Solvlet& solvlet : solve){
      out << "Solvlet|Frame Id: " << solvlet.getFrameID() << endl;
      vector<LimbLabel> labels = solvlet.getLabels();
      for (const LimbLabel& label : labels){
        out << "Limb|Id: " << label.getLimbID() << endl;
        out << "Limb|Is occluded: " << label.getIsOccluded() << endl;
        //out << "Limb|Is weak: " << label.getIsWeak() << endl;
        out << "Limb|Angle: " << label.getAngle() << endl;
        out << "Limb|Center: " << label.getCenter().x << " " << label.getCenter().y << endl;
        out << "Limb|Polygon: " << endl;
        for (auto vertex : label.getPolygon()){
          out << "Vertex: " << vertex.x << " " << vertex.y << endl;
        }
        out << "Limb|Scores: " << endl;
        for (auto score : label.getScores()){
          out << "Score: " << score.getScore() << endl;
        }
        out << "\n";
      }
      out << "\n\n\n";
    }
    file.close();
    /*for( const Solvlet& solvlet : solve ){
        qDebug() << "Solvlet Frame id: " << solvlet.getFrameID() << endl;
        auto frameIterator = std::find_if(frames.begin(), frames.end(),
        [&solvlet]( FramePtr& frame ){
        return frame->getID() == solvlet.getFrameID();
        });
        if( frameIterator != frames.end() && (*frameIterator)->getFrametype() == LOCKFRAME ){
        qDebug() << "Lockframe!" <<  endl;
        } else{
        qDebug() << "NOT Lockframe!" <<  endl;
        }

        }*/
    this->solve = solve;
    int elapsed = timer.elapsed();
    qDebug() << "Solve is finished: " + QString::number(elapsed) << endl;
  }



  //PRIVATE

  Project::Project(QObject *parent)
    :QObject(parent),
    futureWatcher(),
    frames(),
    solve(),
    skeleton(new Skeleton()),
    projectPaths()
  {
    //connect
    QObject::connect(this, &Project::load, this, &Project::loadProjectEvent);
    QObject::connect(this, &Project::close, this, &Project::closeProjectEvent);
  }

  void Project::closeProjectEvent(){
    //clear resources
    frames.clear();
    projectPaths.paths.clear();
    projectPaths.projectFolder.clear();
    projectPaths.projectFilename.clear();
    projectPaths.imgFolderPath.clear();
    projectPaths.maskFolderPath.clear();
    //stop processes
    //TODO: FIX STOP THREAD CRASH PROGRAM
    futureWatcher.cancel();
    futureWatcher.waitForFinished();
    //mark as closed
    currState = ProjectState::CLOSED;
  }

  //TODO: [!] This event is called last.
  void Project::loadProjectEvent(){
    //interpolate frames
    interpolateFrames();
    //mark as loaded
    qDebug() << "Project loading" << endl;
    currState = ProjectState::LOADED;

  }

  void Project::setProjectFolder(const QString &filename){
    QDir d = QFileInfo(filename).absoluteDir();
    projectPaths.projectFolder = d.absolutePath() + "/";
  }

  void Project::setProjectFilename(const QString &filename){
    QString name = QFileInfo(filename).baseName();
    projectPaths.projectFilename = name;
  }

}
//TODO: [i] Name of skeleton - forget
//TODO: [i] Scaling of skeleton - ignoring value. Try scale img later.

