// SPEL definitions
#include "predef.hpp"

#include "bodypartitem.h"

#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include "bodyjointitem.h"
#include "utility.h"

//PUBLIC

FRAMETYPE BodyPartItem::Frametype = INTERPOLATIONFRAME;

BodyPartItem::BodyPartItem(BodyPart *bodyPart, BodyJointItem *parentJoint,
  BodyJointItem *childJoint)
  : source(parentJoint),
  dest(childJoint),
  bodyPart(bodyPart)
{
  setAcceptedMouseButtons(Qt::NoButton);
  //set tool tip
  updateToolTip();
  //set params
  if (Frametype == KEYFRAME){
    setAcceptedMouseButtons(Qt::AllButtons);
  }
  setAcceptHoverEvents(true);
  source->addBodyPart(this);
  dest->addBodyPart(this);
  adjust();
  setZValue(0.5);
}

BodyPartItem::~BodyPartItem()
{
}

void BodyPartItem::adjust(){
  prepareGeometryChange();
}

QRectF BodyPartItem::boundingRect() const{
  if (!source || !dest) return QRectF();

  qreal offset = 0.3;
  return QRectF(source->pos(), dest->pos())
    .normalized()
    .adjusted(-offset, -offset, offset, offset);
}

void BodyPartItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
  QWidget *widget)
{
  if (!source || !dest) return;

  QColor color;// = bodyPart->getIsOccluded() ?
  //Palette::occluded : Palette::notOccluded;
  switch (Frametype){
  case INTERPOLATIONFRAME:
    color = Palette::interpolation;
    break;
  case KEYFRAME:
    color = Palette::keyframe;
    break;
  case LOCKFRAME:
    color = Palette::lockframe;
    break;
  }
  if (!bodyPart->getIsOccluded()){
    color = posegui::Utility::blendColors(color, Palette::notOccluded);
  }
  qreal width = 0.5;

  if (option->state & QStyle::State_MouseOver){
    color = posegui::Utility::blendColors(color, Palette::selected);
    width = 0.7;
  }
  painter->setPen(QPen(color, width));
  painter->drawLine(source->pos(), dest->pos());
  //for test purposes
  //painter->drawRect(boundingRect());

}

//PROTECTED

void BodyPartItem::mousePressEvent(QGraphicsSceneMouseEvent *event){
  if (event->button() == Qt::RightButton){
    //update body part occluded option
    bodyPart->setIsOccluded(!bodyPart->getIsOccluded());
    //update tool tip
    updateToolTip();
  }
  update();
  QGraphicsItem::mousePressEvent(event);
}

void BodyPartItem::hoverEnterEvent(QGraphicsSceneHoverEvent *event){
  update();
  setCursor(Qt::ArrowCursor);
}

void BodyPartItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *event){
  update();
}

//PRIVATE

QColor BodyPartItem::Palette::occluded = Qt::yellow;
QColor BodyPartItem::Palette::notOccluded = Qt::white;
QColor BodyPartItem::Palette::interpolation = Qt::blue;
QColor BodyPartItem::Palette::lockframe = Qt::green;
QColor BodyPartItem::Palette::keyframe = Qt::red;
QColor BodyPartItem::Palette::selected = Qt::white;

void BodyPartItem::updateToolTip(){
  QString toolTip;
  QString name = QString::fromStdString(bodyPart->getPartName());
  QString occluded = bodyPart->getIsOccluded() ? "true" : "false";
  toolTip = name + "\nIs occluded: " + occluded;

  setToolTip(toolTip);
}

