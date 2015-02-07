#include "bodyjointitem.h"

#include <QPainter>
//PUBLIC

BodyJointItem::BodyJointItem()
{

}

BodyJointItem::~BodyJointItem()
{

}

QRectF BodyJointItem::boundingRect() const{
    return QRectF(-radius, -radius, 2*radius, 2*radius);
}

void BodyJointItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                          QWidget *widget)
{
    painter->setPen(QPen({{Qt::GlobalColor::red}},4));
    painter->drawEllipse( {0,0}, radius, radius );
}
