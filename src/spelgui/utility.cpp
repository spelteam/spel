// SPEL definitions
#include "predef.hpp"

#include "utility.h"

#include <QFile>
#include "projectattr.h"
#include "exceptions.h"

#include <limblabelitem.h>

namespace posegui {

  Utility::Utility(QObject *parent) : QObject(parent)
  {

  }

  Utility::~Utility()
  {}

  QString Utility::fileToString(const QString &filename){
    QFile file(filename);
    file.open(QFile::ReadOnly);
    QString result(file.readAll());
    file.close();

    return result;
  }

  /*void Utility::loadXmlPart(const QDomElement &element, const QString& partTagName,
                  std::function<void(const QDomElement&, int)> loadAttrs)
                  {
                  const QDomNodeList& partNodes = element
                  .elementsByTagName(partTagName).at(0)
                  .childNodes();
                  for(int i = 0; i < partNodes.size(); i++ ){
                  const QDomElement& childNode = partNodes.at(i).toElement();

                  loadAttrs( childNode, i );
                  }
                  }*/

  BodyJoint* Utility::getJointById(const tree<BodyJoint> &bodyJoints, int id){
    tree<BodyJoint>::iterator it = bodyJoints.begin();
    while (it != bodyJoints.end()){
      if (it->getLimbID() == id) return &(*it);
      it++;
    }

    return nullptr;
  }

  BodyPart* Utility::getBodyPartById(const tree<BodyPart> &bodyParts, int id){
    tree<BodyPart>::iterator it = bodyParts.begin();
    while (it != bodyParts.end()){
      if (it->getPartID() == id) return &(*it);
      it++;
    }

    return nullptr;
  }

  BodyJointItem* Utility::getJointItemById(const QList<QGraphicsItem *> &items, int id){
    for (auto it = items.begin(); it != items.end(); ++it){
      if (dynamic_cast<BodyJointItem*>(*it) &&
        static_cast<BodyJointItem*>(*it)->getId() == id)
      {
        return static_cast<BodyJointItem*>(*it);
      }
    }
    return nullptr;
  }

  void Utility::resizeImage(Mat &image, int32_t &cols, int32_t &rows){
    if (cols <= 0 || rows <= 0){
      cols = image.cols;
      rows = image.rows;
    }
    if (cols != image.cols || rows != image.rows){
      cv::resize(image, image, cv::Size(cols, rows));
    }
  }

  bool Utility::isJointItem(const QList<QGraphicsItem*>::iterator &it){
    return dynamic_cast<BodyJointItem*>(*it);
  }

  bool Utility::isSkeletonItem(const QList<QGraphicsItem*>::iterator &it){
    return dynamic_cast<BodyJointItem*>(*it) ||
      dynamic_cast<BodyPartItem*>(*it) ||
      dynamic_cast<LimbLabelItem*>(*it);
  }

  QColor Utility::blendColors(const QColor &first, const QColor &second){
    QColor mix;
    mix.setRed((first.red() + second.red()) / 2);
    mix.setGreen((first.green() + second.green()) / 2);
    mix.setBlue((first.blue() + second.blue()) / 2);
    mix.setAlpha((first.alpha() + second.alpha()) / 2);

    return mix;
  }

  void Utility::buildBodyPartTree(std::vector<BodyPart> &bodyList,
    tree<BodyPart> &bodyParts)
  {
    tree<BodyPart>::iterator root = bodyParts.begin();
    tree<BodyPart>::iterator currNode;
    bool isRootExist = false;
    //find root element
    for (auto it = bodyList.begin(); it != bodyList.end(); ++it){
      if (it->getPartID() == BodyPartHeaderAttrs::ROOT_ID){
        currNode = bodyParts.insert(root, *it);
        bodyList.erase(it);
        isRootExist = true;
        break;
      }
    }
    if (!isRootExist){
      throw InvalidProjectStructure("Root element of skeleton doesn't exist");
    }
    std::queue<tree<BodyPart>::iterator> availableJoints;
    //add body parts to root element
    bodyList.erase(std::remove_if(bodyList.begin(), bodyList.end(),
      [&](const BodyPart &bodyPart){
      //check whether current body part is a child of root element
      if (currNode->getParentJoint() == bodyPart.getParentJoint() ||
        currNode->getChildJoint() == bodyPart.getParentJoint())
      {
        //add body part to tree
        availableJoints.push(bodyParts.append_child(currNode, bodyPart));
        //remove current body part from list
        //and go next body part
        return true;
      }
      else{
        //go next body part
        return false;
      }
    }), bodyList.end());
    //add body parts to rest part of tree
    while (!availableJoints.empty()){
      //get current body part
      currNode = availableJoints.front();
      availableJoints.pop();
      //add body parts to tree
      //and remove from list
      bodyList.erase(std::remove_if(bodyList.begin(), bodyList.end(),
        [&](const BodyPart &bodyPart){
        if (currNode->getChildJoint() == bodyPart.getParentJoint()){
          availableJoints.push(bodyParts.append_child(currNode, bodyPart));
          return true;
        }
        else{
          return false;
        }
      }), bodyList.end());
    }
    if (!bodyList.empty()){
      throw InvalidProjectStructure("Some of body parts are invalid. Can't add to tree");
    }
  }

  QPixmap Utility::loadMask(QImage mask, int opacity){
    //set white pixels as transparent
    QBitmap bitmap = QBitmap::fromImage(mask)
      .createMaskFromColor(qRgb(255, 255, 255));
    //set alpha channel to mask
    QImage alphaValues(mask.size(), QImage::Format_ARGB32_Premultiplied);
    alphaValues.fill(qRgb(opacity, opacity, opacity));
    mask.setAlphaChannel(alphaValues);
    QPixmap maskPixmap = QPixmap::fromImage(mask);
    maskPixmap.setMask(bitmap);

    return maskPixmap;
  }

}

