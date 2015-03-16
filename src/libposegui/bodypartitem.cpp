#include "bodypartitem.h"

#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include "bodyjointitem.h"
#include "utility.h"
#include <bodyPart.hpp>

//PUBLIC

FRAMETYPE BodyPartItem::Frametype = INTERPOLATIONFRAME;

BodyPartItem::BodyPartItem(BodyPart *bodyPart, BodyJointItem *parentJoint,
                           BodyJointItem *childJoint)
    : source(parentJoint),
      dest(childJoint),
      bodyPart(bodyPart)
{
    //set tool tip
    updateToolTip();
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
    if( !source || !dest ) return QRectF();

    qreal offset = 0.5;
    return QRectF(source->pos(), dest->pos())
            .normalized()
            .adjusted(-offset, -offset, offset, offset);
}

void BodyPartItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                          QWidget *widget)
{
    if( !source || !dest ) return;

    QColor color = bodyPart->getIsOccluded() ?
                Palette::occluded : Palette::notOccluded;
    qreal width = 0;
    switch(Frametype){
    case INTERPOLATIONFRAME:
        color = Utility::blendColors(color,Palette::interpolation);
        break;
    case KEYFRAME:
        color = Utility::blendColors(color,Palette::keyframe);
        break;
    case LOCKFRAME:
        color = Utility::blendColors(color,Palette::lockframe);
        break;
    }
    if( option->state & QStyle::State_MouseOver ){
        color = Utility::blendColors(color,Palette::selected);
        width = 0.7;
    }
    painter->setPen(QPen(color,width));
    painter->drawLine( source->pos(), dest->pos() );
    //qreal offset = 0.5;
    /*painter->drawRect(QRectF(source->pos(), dest->pos())
                      .normalized()
                      .adjusted(-offset, -offset, offset, offset));*/

}

//PROTECTED

void BodyPartItem::mousePressEvent(QGraphicsSceneMouseEvent *event){
    if( event->button() == Qt::RightButton ){
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
QColor BodyPartItem::Palette::notOccluded = Qt::cyan;
QColor BodyPartItem::Palette::interpolation = Qt::blue;
QColor BodyPartItem::Palette::lockframe = Qt::green;
QColor BodyPartItem::Palette::keyframe = Qt::red;
QColor BodyPartItem::Palette::selected = Qt::white;

void BodyPartItem::updateToolTip(){
    QString toolTip;
    QString name = QString::fromStdString(bodyPart->getPartName());
    QString occluded = bodyPart->getIsOccluded()?"true":"false";
    toolTip = name + "\nIs occluded: " + occluded;

    setToolTip(toolTip);
}

