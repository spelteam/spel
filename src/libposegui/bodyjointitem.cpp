#include "bodyjointitem.h"

#include "bodypartitem.h"
#include "project.h"
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QGraphicsSceneMouseEvent>
//PUBLIC

//TODO: [L] Rich tool tip
//TODO: [!] Context menu
BodyJointItem::BodyJointItem(QGraphicsItem *parent)
    : QGraphicsItem(parent),
      id(-1)
{
    setFlag(ItemIsSelectable);
    setFlag(ItemIsMovable);
    setFlag(ItemSendsGeometryChanges);
    setCacheMode(DeviceCoordinateCache);
    setZValue(1.0);
    setAcceptHoverEvents(true);
}

BodyJointItem::~BodyJointItem()
{
}

void BodyJointItem::addBodyPart(BodyPartItem *bodyPart){
    partList << bodyPart;
}

void BodyJointItem::setId(int id){
    this->id = id;
    //set tool tip
    const int FRAME = 0;
    BodyJoint* joint = Project::getInstance()
            .getFrame(FRAME)->getSkeleton().getBodyJoint(id);
    QString toolTip = QString::fromStdString(joint->getJointName());
    setToolTip(toolTip);
}

int BodyJointItem::getId() const{
    return id;
}

QRectF BodyJointItem::boundingRect() const{
    qreal offset = 1;
    qreal boundRadius = radius+offset;
    return QRectF(-boundRadius, -boundRadius, 2*boundRadius, 2*boundRadius);
}

#include <QDebug>
void BodyJointItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                          QWidget *widget)
{
    QColor color;
    if( option->state & QStyle::State_MouseOver ){
        color = Qt::blue;
    } else{
        color = Qt::red;
    }
    QBrush brush = QBrush(color,Qt::SolidPattern);
    QPen pen = QPen(brush,0,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin);
    //QRadialGradient grad = QRadialGradient({0,0},radius);
    //grad.setColorAt(0.0,Qt::red);
    //grad.setColorAt(1.0, Qt::blue);
    painter->setPen(pen);
    //painter->setBrush(grad);
    //painter->setBackground(grad);
    //painter->setBackgroundMode(Qt::OpaqueMode);
    //painter->setBrush(grad);
    painter->drawEllipse( {0,0}, radius, radius );
    painter->drawRect(-radius-1, -radius-1, 2*(radius+1), 2*(radius+1));
}

//PROTECTED

QVariant BodyJointItem::itemChange(GraphicsItemChange change, const QVariant &value){
    switch(change){
    case ItemPositionHasChanged:
        foreach (auto *part, partList) {
            part->adjust();
        }
        break;
    default:
        break;
    }

    return QGraphicsItem::itemChange(change,value);
}

void BodyJointItem::mousePressEvent(QGraphicsSceneMouseEvent *event){
    if( event->button() == Qt::RightButton ){
        //TODO: [L] Update depth
    }
    update();
    QGraphicsItem::mousePressEvent(event);
}

void BodyJointItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    //TODO: [L] Update joint position
    update();
    QGraphicsItem::mouseReleaseEvent(event);
}

void BodyJointItem::hoverEnterEvent(QGraphicsSceneHoverEvent *event){
    //repaint
    update();
    setCursor(Qt::ArrowCursor);
}

void BodyJointItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *event){
    //repaint
    update();
}
