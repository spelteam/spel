#include "frameshandler.h"

#include <QDomElement>
#include <QDomDocument>

#include "frameattrstorage.h"
#include "filepathstorage.h"
#include <interpolation.hpp>

#include "projectattr.h"
#include "utility.h"
#include "exceptions.h"

//TODO:[!] Refactor this module

namespace posegui {

  Frames FramesHandler::read(const QDomElement &data, FilePathStorage &filepaths, const SkeletonPtr &skeleton){
    Frames frames;
    //cols and rows of image in first occured frame
    int32_t firstFrameCol = -1;
    int32_t firstFrameRow = -1;
    //number of current frame
    int num = 0;
    while (!data.isNull()){
      FrameAttrStorage frameAttrStorage = frameHandler.read(data);
      //add new paths
      filepaths.paths[num] = {
        frameAttrStorage.imgPath,
        frameAttrStorage.maskPath,
        frameAttrStorage.camPath
      };
      //load image
      QString fullPath = (
        filepaths.projectFolder +
        filepaths.imgFolderPath +
        frameAttrStorage.imgPath
        );
      cv::Mat image = cv::imread(
        fullPath.toStdString(),
        CV_LOAD_IMAGE_COLOR
        );
      if (image.data == nullptr){
        throw FileNotRead("Can't read image file: " + fullPath);
      }
      //save original size of image
      int32_t imageOrigCols = image.cols;
      int32_t imageOrigRows = image.rows;
      Utility::resizeImage(image, firstFrameCol, firstFrameRow);
      //load mask
      fullPath = (
        filepaths.projectFolder +
        filepaths.maskFolderPath +
        frameAttrStorage.maskPath
        );
      cv::Mat mask = cv::imread(
        fullPath.toStdString(),
        CV_LOAD_IMAGE_GRAYSCALE
        );
      if (mask.data == nullptr){
        throw FileNotRead("Can't read mask file: " + fullPath);
      }
      Utility::resizeImage(mask, firstFrameCol, firstFrameRow);
      //create frame
      FramePtr newFrame;
      if (frameAttrStorage.isKeyframe){
        newFrame = FramePtr(new Keyframe());
        tree<BodyJoint> bodyJoints = skeleton->getJointTree();
        tree<BodyPart> bodyParts = skeleton->getPartTree();
        QDomElement jointsFrame = data
          .elementsByTagName(BodyJointAttrs::JOINTS_TAG)
          .at(0)
          .firstChildElement();
        float colsFactor = static_cast<float>(image.cols) / imageOrigCols;
        float rowsFactor = static_cast<float>(image.rows) / imageOrigRows;
        jointsHandler.read(jointsFrame, bodyJoints, colsFactor, rowsFactor);
        QDomElement partsFrame = data
          .elementsByTagName(BodyPartAttrs::PARTS_TAG)
          .at(0)
          .firstChildElement();
        partsHandler.read(partsFrame, bodyParts);
        Skeleton newSkeleton;
        newSkeleton.setJointTree(bodyJoints);
        newSkeleton.setPartTree(bodyParts);
        newSkeleton.setScale(100.f);
        newFrame->setSkeleton(newSkeleton);
      }
      else{
        newFrame = FramePtr(new Interpolation());
        newFrame->setSkeleton(*skeleton.get());
      }
      newFrame->setID(frameAttrStorage.id);
      Point2f gp;
      gp.x = frameAttrStorage.gpX;
      gp.y = frameAttrStorage.gpY;
      newFrame->setGroundPoint(gp);
      newFrame->setImage(image);
      newFrame->setMask(mask);
      //add frame to list
      frames.push_back(std::move(newFrame));
      //go to next frame
      const_cast<QDomElement&>(data) = data.nextSiblingElement();
      ++num;
    }
    return frames;
  }

  QDomElement FramesHandler::write(const Frames &model, const FilePathStorage &filepaths, QDomDocument &controller){
    QDomElement elem = controller
      .createElement(FrameAttrs::FRAMES_TAG);
    //number of current frame
    int num = 0;
    for (const FramePtr& frame : model){
      FrameAttrStorage frameAttrStorage;
      frameAttrStorage.id = frame->getID();
      frameAttrStorage.imgPath = filepaths.paths[num].imgPath;
      frameAttrStorage.maskPath = filepaths.paths[num].maskPath;
      frameAttrStorage.camPath = filepaths.paths[num].camPath;
      frameAttrStorage.gpX = frame->getGroundPoint().x;
      frameAttrStorage.gpY = frame->getGroundPoint().y;
      frameAttrStorage.isKeyframe = (frame->getFrametype() == KEYFRAME);
      QDomElement frameElem = frameHandler.write(frameAttrStorage, controller);
      if (frameAttrStorage.isKeyframe){
        QDomElement jointsElem = jointsHandler
          .write(frame->getSkeletonPtr()->getJointTree(), controller);
        QDomElement partsElem = partsHandler
          .write(frame->getSkeletonPtr()->getPartTree(), controller);
        frameElem.appendChild(jointsElem);
        frameElem.appendChild(partsElem);
      }
      elem.appendChild(frameElem);
      //go to next frame
      ++num;
    }
    return elem;
  }

}

