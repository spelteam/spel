// SPEL definitions
#include "predef.hpp"

#include "frameview2d.h"
#include "bodyjointitem.h"
#include "bodypartitem.h"
#include "limblabelitem.h"
#include "project.h"
#include "utility.h"
#include <QBitmap>
#include <QWheelEvent>

using posegui::Project;
//PUBLIC
#include <QDebug>

FrameView2D::FrameView2D(QWidget *parent) :
QWidget(parent),
frameImage(nullptr),
frameMask(nullptr),
opacityValue(0)
{
  scene = new QGraphicsScene;
  scene->setItemIndexMethod(QGraphicsScene::NoIndex);
  view = new FrameGraphicsView(scene, parent);
}

FrameView2D::~FrameView2D(){
  delete view;
  delete scene;
}

void FrameView2D::loadProjectEvent(){
  qDebug() << "Frame view loading" << endl;
  //EMPTY
}

#include <QDebug>
void FrameView2D::closeProjectEvent(){
  scene->clear();
  view->update();
  //no image exist on view
  frameImage = nullptr;
}

void FrameView2D::scaleItemsEvent(int value){
  //TEMP
  auto itemList = scene->items();
  for (auto it = itemList.begin(); it != itemList.end(); it++){
    if (dynamic_cast<BodyJointItem*>(*it)){
      (*it)->setScale(value / 10.f);
    }
  }
  scene->update();
}

void FrameView2D::changeMaskOpacityEvent(int value){
  opacityValue = value;
  if (frameImage){
    using posegui::Utility;
    frameMask->setPixmap(Utility::loadMask(frameMaskOrig, opacityValue));
  }
}

void FrameView2D::pickFrameEvent(int num){
  loadFrameImage(num);
  loadFrameSkeleton(num);
}

//PRIVATE

void FrameView2D::loadFrameImage(int num){
  auto projectPaths = Project::getInstance().getPaths();
  auto projectFolder = Project::getInstance().getProjectFolder();
  QString imgPath = projectFolder + projectPaths.imgFolderPath + projectPaths.paths[num].imgPath;
  QString maskPath = projectFolder + projectPaths.maskFolderPath + projectPaths.paths[num].maskPath;

  QImage img, mask;
  if (img.load(imgPath) && mask.load(maskPath)){
    //save orig mask
    frameMaskOrig = mask;
    using posegui::Utility;
    if (frameImage){
      //frameImage exist on view
      frameImage->setPixmap(QPixmap::fromImage(img));
      frameMask->setPixmap(Utility::loadMask(mask, opacityValue));
    }
    else{
      //frameImage not exist on view
      QPixmap imgPixmap = QPixmap::fromImage(img);
      frameImage = scene->addPixmap(imgPixmap);
      frameMask = scene->addPixmap(Utility::loadMask(mask, opacityValue));
      frameImage->setZValue(-1.0);
      frameMask->setZValue(-0.9);
      //set scene bounded rect
      scene->setSceneRect(imgPixmap.rect());
    }
    view->update();
  }
  else{
    qDebug() << "FrameView2D: Error! Image not found!" << endl;
  }
}

void FrameView2D::loadFrameSkeleton(int num){
  //clear frame view
  auto itemList = scene->items();
  for (auto it = itemList.begin(); it != itemList.end(); ++it){
    if (posegui::Utility::isSkeletonItem(it)){
      scene->removeItem(*it);
    }
  }
  //get frame
  auto frame = Project::getInstance().getFrame(num);
  //load skeleton
  BodyJointItem::Frametype = frame->getFrametype();
  auto bodyJoints = frame->getSkeletonPtr()
    ->getJointTreePtr();
  auto it = bodyJoints->begin();
  while (it != bodyJoints->end()){
    BodyJoint *joint = &(*it);
    BodyJointItem* newItem = new BodyJointItem(joint);
    scene->addItem(newItem);

    ++it;
  }
  //load body parts
  BodyPartItem::Frametype = frame->getFrametype();
  auto bodyParts = frame->getSkeletonPtr()
    ->getPartTreePtr();
  auto pit = bodyParts->begin();
  while (pit != bodyParts->end()){
    BodyPart *part = &(*pit);
    BodyJointItem* child =
      posegui::Utility::getJointItemById(scene->items(), part->getChildJoint());
    BodyJointItem* parent =
      posegui::Utility::getJointItemById(scene->items(), part->getParentJoint());
    if (!child || !parent){
      qDebug() << "Error! Joints not exist!" << endl;
      return;
    }
    BodyPartItem* newItem = new BodyPartItem(part, child, parent);
    scene->addItem(newItem);

    ++pit;
  }
  //load limp labels
  LimbLabelItem::Frametype = LOCKFRAME;
  auto labels = Project::getInstance().getLabels(num);
  if (labels != nullptr){
    qDebug() << "Limb label NOT null" << endl;
    for (LimbLabel label : *labels){
      LimbLabelItem* limbLabelItem = new LimbLabelItem(label);
      scene->addItem(limbLabelItem);
    }
  }
}

//TODO: [L] Fix zoom in / zoom out
void FrameView2D::FrameGraphicsView::wheelEvent(QWheelEvent *event){
  // Scale the view / do the zoom
  double scaleFactor = 1.15;
  if (event->delta() > 0) {
    qreal factor = transform().scale(scaleFactor, scaleFactor)
      .mapRect(QRectF(0, 0, 1, 1))
      .width();
    if (factor < 0.07 || factor > 100) return;
    // Zoom in
    scale(scaleFactor, scaleFactor);
  }
  else {
    qreal factor = transform().scale(1.0 / scaleFactor, 1.0 / scaleFactor)
      .mapRect(QRectF(0, 0, 1, 1))
      .width();
    if (factor < 0.07 || factor > 100) return;
    // Zooming out
    scale(1.0 / scaleFactor, 1.0 / scaleFactor);
  }
  update();
}

void FrameView2D::FrameGraphicsView::mousePressEvent(QMouseEvent *event){
  if (event->button() == Qt::RightButton){
    if (dragMode() == DragMode::RubberBandDrag){
      setDragMode(DragMode::ScrollHandDrag);
      setCursor(Qt::OpenHandCursor);
    }
    else{
      setDragMode(DragMode::RubberBandDrag);
      setCursor(Qt::ArrowCursor);
    }
  }
  update();
  QGraphicsView::mousePressEvent(event);
}


