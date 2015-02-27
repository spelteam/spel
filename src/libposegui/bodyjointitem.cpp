#include "bodyjointitem.h"

#include "bodypartitem.h"
#include <QPainter>
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
}

BodyJointItem::~BodyJointItem()
{
}

void BodyJointItem::addBodyPart(BodyPartItem *bodyPart){
    partList << bodyPart;
}

void BodyJointItem::setId(int id){
    this->id = id;
}

int BodyJointItem::getId() const{
    return id;
}

QRectF BodyJointItem::boundingRect() const{
    qreal offset = 1;
    qreal boundRadius = radius+offset;
    return QRectF(-boundRadius, -boundRadius, 2*boundRadius, 2*boundRadius);
}

void BodyJointItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                          QWidget *widget)
{
    QBrush brush = QBrush(Qt::red,Qt::SolidPattern);
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
    update();
    QGraphicsItem::mousePressEvent(event);
}

void BodyJointItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event){
    update();
    QGraphicsItem::mouseReleaseEvent(event);
}
