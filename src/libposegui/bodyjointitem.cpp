#include "bodyjointitem.h"

#include "bodypartitem.h"
#include "utility.h"
#include <bodyJoint.hpp>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QGraphicsSceneMouseEvent>
//PUBLIC

//TODO: [L] Rich tool tip
//TODO: [!] Context menu
BodyJointItem::BodyJointItem(BodyJoint *joint, QGraphicsItem *parent)
    : QGraphicsItem(parent),
      joint(joint)
{
    //set position
    setPos(joint->getImageLocation().x,joint->getImageLocation().y);
    //set params
    setFlag(ItemIsSelectable);
    setFlag(ItemIsMovable);
    setFlag(ItemSendsGeometryChanges);
    setCacheMode(DeviceCoordinateCache);
    setZValue(1.0);
    setAcceptHoverEvents(true);
    //set tool tip
    updateToolTip();
}

BodyJointItem::~BodyJointItem()
{
}

void BodyJointItem::addBodyPart(BodyPartItem *bodyPart){
    partList << bodyPart;
}

int BodyJointItem::getId() const{
    return joint->getLimbID();
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
    QColor color = joint->getDepthSign() ? Palette::inDepth : Palette::outDepth;
    qreal width = 0;
    if( option->state & (QStyle::State_MouseOver | QStyle::State_Selected) ){
        color = Utility::blendColors(color,Palette::selected);
        width = 0.7;
    }
    QBrush brush = QBrush(color,Qt::SolidPattern);
    QPen pen = QPen(brush,width,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin);
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
        //update joint position
        joint->setImageLocation({pos().x(),pos().y()});
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
        //update joint depth sign
        joint->setDepthSign(!joint->getDepthSign());
        //update tool tip
        updateToolTip();
    }
    update();
    QGraphicsItem::mousePressEvent(event);
}

void BodyJointItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    update();
    QGraphicsItem::mouseReleaseEvent(event);
    //update joint position
    joint->setImageLocation({pos().x(),pos().y()});
    //update tool tip
    updateToolTip();
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

//PRIVATE

QColor BodyJointItem::Palette::inDepth = Qt::yellow;
QColor BodyJointItem::Palette::outDepth = Qt::cyan;
QColor BodyJointItem::Palette::selected = Qt::white;

void BodyJointItem::updateToolTip(){
    QString toolTip;
    QString name = QString::fromStdString(joint->getJointName());
    QString depthSign = joint->getDepthSign() ? "true" : "false";
    QString pos = "x: " + QString::number(joint->getImageLocation().x) +
                  " y: " + QString::number(joint->getImageLocation().y);
    toolTip = name + "\nDepth sign: " + depthSign + "\n" + pos;
    setToolTip(toolTip);
}
