#include "bodypartitem.h"

#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include <bodyjointitem.h>

#include "project.h"

//PUBLIC

BodyPartItem::BodyPartItem(BodyJointItem *parentJoint,
                           BodyJointItem *childJoint)
    : source(parentJoint),
      dest(childJoint)
{
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

void BodyPartItem::setId(int id){
    this->id = id;
    //set tool tip
    const int FRAME = 0;
    BodyPart* bodyPart = Project::getInstance()
            .getFrame(FRAME)->getSkeleton().getBodyPart(id);
    QString toolTip = QString::fromStdString(bodyPart->getPartName());
    setToolTip(toolTip);
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

    QColor color;
    if( option->state & QStyle::State_MouseOver ){
        color = Qt::blue;
    } else{
        color = Qt::red;
    }
    painter->setPen(QPen(color,0));
    painter->drawLine( source->pos(), dest->pos() );
    qreal offset = 0.5;
    /*painter->drawRect(QRectF(source->pos(), dest->pos())
                      .normalized()
                      .adjusted(-offset, -offset, offset, offset));*/

}

//PROTECTED

void BodyPartItem::hoverEnterEvent(QGraphicsSceneHoverEvent *event){
    update();
    setCursor(Qt::ArrowCursor);
}

void BodyPartItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *event){
    update();
}
